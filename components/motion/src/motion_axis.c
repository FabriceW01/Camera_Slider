#include "motion_axis.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    MOTION_MOVE_IDLE = 0,
    MOTION_MOVE_POSITION,
    MOTION_MOVE_CONTINUOUS,
} motion_move_kind_t;

typedef struct {
    gptimer_handle_t timer;
    QueueHandle_t event_queue;
    portMUX_TYPE spinlock;
    motion_axis_pins_t pins;
    app_runtime_config_t config;
    motion_move_kind_t move_kind;
    motion_direction_t direction;
    int32_t current_position_steps;
    int32_t target_position_steps;
    int32_t remaining_steps;
    float current_speed_steps_per_sec;
    float cruise_speed_steps_per_sec;
    float acceleration_steps_per_sec2;
    float first_step_speed_steps_per_sec;
    bool running;
    bool step_high;
    bool stop_requested;
    bool initialized;
    app_limit_status_t limits;
    bool left_last_sample;
    bool right_last_sample;
    uint64_t left_last_change_ms;
    uint64_t right_last_change_ms;
} motion_axis_ctx_t;

static const char *TAG = "motion_axis";
static const uint32_t STEP_PULSE_HIGH_US = 4;
static const uint32_t DIR_SETTLE_US = 10;
static const uint32_t TIMER_RESOLUTION_HZ = 1000000;
static const size_t MOTION_EVENT_QUEUE_LENGTH = 16;

static motion_axis_ctx_t s_motion = {
    .spinlock = portMUX_INITIALIZER_UNLOCKED,
};

static inline uint32_t motion_speed_to_interval_us(float speed_steps_per_sec)
{
    if (speed_steps_per_sec <= 0.0f) {
        return 1000000U;
    }

    uint32_t interval_us = (uint32_t)lroundf(1000000.0f / speed_steps_per_sec);
    uint32_t minimum_interval_us = STEP_PULSE_HIGH_US + 1;
    return interval_us < minimum_interval_us ? minimum_interval_us : interval_us;
}

static inline float motion_compute_stop_distance_steps(float current_speed_steps_per_sec, float acceleration_steps_per_sec2)
{
    if (acceleration_steps_per_sec2 <= 0.0f) {
        return INFINITY;
    }

    return (current_speed_steps_per_sec * current_speed_steps_per_sec) / (2.0f * acceleration_steps_per_sec2);
}

static inline float motion_next_speed_after_step(float current_speed_steps_per_sec, float acceleration_steps_per_sec2, bool accelerate)
{
    if (acceleration_steps_per_sec2 <= 0.0f) {
        return current_speed_steps_per_sec;
    }

    float delta = 2.0f * acceleration_steps_per_sec2;
    float next_squared = accelerate
        ? (current_speed_steps_per_sec * current_speed_steps_per_sec) + delta
        : (current_speed_steps_per_sec * current_speed_steps_per_sec) - delta;
    if (next_squared < 0.0f) {
        next_squared = 0.0f;
    }

    return sqrtf(next_squared);
}

static void motion_fill_event_limits_from_ctx(motion_event_t *event)
{
    event->limits = s_motion.limits;
}

static bool motion_push_event_from_isr(motion_event_type_t type, BaseType_t *high_task_woken)
{
    motion_event_t event = {
        .type = type,
        .position_steps = s_motion.current_position_steps,
    };
    motion_fill_event_limits_from_ctx(&event);
    return xQueueSendFromISR(s_motion.event_queue, &event, high_task_woken) == pdTRUE;
}

static void motion_push_event_from_task(motion_event_type_t type)
{
    motion_event_t event = {
        .type = type,
        .position_steps = s_motion.current_position_steps,
    };
    motion_fill_event_limits_from_ctx(&event);
    if (xQueueSend(s_motion.event_queue, &event, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Dropping motion event %d because the queue is full.", type);
    }
}

static void motion_apply_direction_locked(motion_direction_t direction)
{
    int level = direction == MOTION_DIRECTION_POSITIVE ? 1 : 0;
    if (s_motion.config.axis_behavior.invert_direction_output) {
        level = !level;
    }

    gpio_set_level(s_motion.pins.dir_pin, level);
    s_motion.direction = direction;
}

static bool motion_limit_pressed_raw(gpio_num_t pin)
{
    return gpio_get_level(pin) == 0;
}

static motion_event_type_t motion_limit_event_for_pin(gpio_num_t pin)
{
    return pin == s_motion.pins.limit_left_pin ? MOTION_EVENT_LIMIT_LEFT : MOTION_EVENT_LIMIT_RIGHT;
}

static void IRAM_ATTR motion_force_stop_from_isr(motion_event_type_t event_type, BaseType_t *high_task_woken)
{
    if (!s_motion.running) {
        return;
    }

    s_motion.running = false;
    s_motion.stop_requested = false;
    s_motion.move_kind = MOTION_MOVE_IDLE;
    s_motion.current_speed_steps_per_sec = 0.0f;
    s_motion.cruise_speed_steps_per_sec = 0.0f;
    s_motion.acceleration_steps_per_sec2 = 0.0f;
    s_motion.step_high = false;

    gpio_set_level(s_motion.pins.step_pin, 0);
    gptimer_stop(s_motion.timer);
    motion_push_event_from_isr(event_type, high_task_woken);
}

static void IRAM_ATTR motion_limit_isr_handler(void *arg)
{
    gpio_num_t pin = (gpio_num_t)(uintptr_t)arg;
    BaseType_t high_task_woken = pdFALSE;

    portENTER_CRITICAL_ISR(&s_motion.spinlock);
    if (pin == s_motion.pins.limit_left_pin) {
        s_motion.limits.left_raw_pressed = true;
    } else if (pin == s_motion.pins.limit_right_pin) {
        s_motion.limits.right_raw_pressed = true;
    }

    if (s_motion.running) {
        motion_force_stop_from_isr(motion_limit_event_for_pin(pin), &high_task_woken);
    }
    portEXIT_CRITICAL_ISR(&s_motion.spinlock);

    if (high_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static bool IRAM_ATTR motion_timer_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    (void)user_ctx;
    BaseType_t high_task_woken = pdFALSE;
    static gptimer_alarm_config_t next_alarm = {};

    portENTER_CRITICAL_ISR(&s_motion.spinlock);

    if (!s_motion.running) {
        gptimer_stop(timer);
        portEXIT_CRITICAL_ISR(&s_motion.spinlock);
        return false;
    }

    if (!s_motion.step_high) {
        gpio_set_level(s_motion.pins.step_pin, 1);
        s_motion.step_high = true;
        next_alarm.alarm_count = edata->count_value + STEP_PULSE_HIGH_US;
        gptimer_set_alarm_action(timer, &next_alarm);
        portEXIT_CRITICAL_ISR(&s_motion.spinlock);
        return false;
    }

    gpio_set_level(s_motion.pins.step_pin, 0);
    s_motion.step_high = false;
    s_motion.current_position_steps += (int32_t)s_motion.direction;

    if (s_motion.current_speed_steps_per_sec <= 0.0f) {
        s_motion.current_speed_steps_per_sec = s_motion.first_step_speed_steps_per_sec;
    }

    if (s_motion.move_kind == MOTION_MOVE_POSITION && s_motion.remaining_steps > 0) {
        s_motion.remaining_steps--;
    }

    if (s_motion.move_kind == MOTION_MOVE_POSITION && s_motion.remaining_steps <= 0) {
        s_motion.running = false;
        s_motion.current_speed_steps_per_sec = 0.0f;
        s_motion.cruise_speed_steps_per_sec = 0.0f;
        s_motion.acceleration_steps_per_sec2 = 0.0f;
        s_motion.move_kind = MOTION_MOVE_IDLE;
        gptimer_stop(timer);
        motion_push_event_from_isr(MOTION_EVENT_MOVE_COMPLETE, &high_task_woken);
        portEXIT_CRITICAL_ISR(&s_motion.spinlock);
        return high_task_woken == pdTRUE;
    }

    if (s_motion.move_kind == MOTION_MOVE_CONTINUOUS && s_motion.stop_requested &&
        s_motion.current_speed_steps_per_sec <= (s_motion.first_step_speed_steps_per_sec + 1.0f)) {
        s_motion.running = false;
        s_motion.stop_requested = false;
        s_motion.current_speed_steps_per_sec = 0.0f;
        s_motion.cruise_speed_steps_per_sec = 0.0f;
        s_motion.acceleration_steps_per_sec2 = 0.0f;
        s_motion.move_kind = MOTION_MOVE_IDLE;
        gptimer_stop(timer);
        motion_push_event_from_isr(MOTION_EVENT_STOPPED, &high_task_woken);
        portEXIT_CRITICAL_ISR(&s_motion.spinlock);
        return high_task_woken == pdTRUE;
    }

    if (s_motion.move_kind == MOTION_MOVE_POSITION && s_motion.stop_requested &&
        s_motion.current_speed_steps_per_sec <= (s_motion.first_step_speed_steps_per_sec + 1.0f)) {
        s_motion.running = false;
        s_motion.stop_requested = false;
        s_motion.current_speed_steps_per_sec = 0.0f;
        s_motion.cruise_speed_steps_per_sec = 0.0f;
        s_motion.acceleration_steps_per_sec2 = 0.0f;
        s_motion.move_kind = MOTION_MOVE_IDLE;
        gptimer_stop(timer);
        motion_push_event_from_isr(MOTION_EVENT_STOPPED, &high_task_woken);
        portEXIT_CRITICAL_ISR(&s_motion.spinlock);
        return high_task_woken == pdTRUE;
    }

    float next_speed_steps_per_sec = s_motion.current_speed_steps_per_sec;
    bool accelerate = false;
    bool decelerate = false;

    if (s_motion.move_kind == MOTION_MOVE_CONTINUOUS) {
        if (s_motion.stop_requested) {
            decelerate = true;
        } else if (s_motion.current_speed_steps_per_sec < s_motion.cruise_speed_steps_per_sec) {
            accelerate = true;
        }
    } else if (s_motion.move_kind == MOTION_MOVE_POSITION) {
        if (s_motion.stop_requested) {
            decelerate = true;
        } else {
            float stop_distance_steps = motion_compute_stop_distance_steps(
                s_motion.current_speed_steps_per_sec,
                s_motion.acceleration_steps_per_sec2);
            if ((float)s_motion.remaining_steps <= stop_distance_steps) {
                decelerate = true;
            } else if (s_motion.current_speed_steps_per_sec < s_motion.cruise_speed_steps_per_sec) {
                accelerate = true;
            }
        }
    }

    if (accelerate) {
        next_speed_steps_per_sec = motion_next_speed_after_step(
            s_motion.current_speed_steps_per_sec,
            s_motion.acceleration_steps_per_sec2,
            true);
        if (next_speed_steps_per_sec > s_motion.cruise_speed_steps_per_sec) {
            next_speed_steps_per_sec = s_motion.cruise_speed_steps_per_sec;
        }
    } else if (decelerate) {
        next_speed_steps_per_sec = motion_next_speed_after_step(
            s_motion.current_speed_steps_per_sec,
            s_motion.acceleration_steps_per_sec2,
            false);
        if (next_speed_steps_per_sec < s_motion.first_step_speed_steps_per_sec) {
            next_speed_steps_per_sec = s_motion.first_step_speed_steps_per_sec;
        }
    } else {
        next_speed_steps_per_sec = s_motion.cruise_speed_steps_per_sec;
    }

    s_motion.current_speed_steps_per_sec = next_speed_steps_per_sec;
    uint32_t interval_us = motion_speed_to_interval_us(next_speed_steps_per_sec);
    uint32_t low_interval_us = interval_us > STEP_PULSE_HIGH_US ? (interval_us - STEP_PULSE_HIGH_US) : 1;
    next_alarm.alarm_count = edata->count_value + low_interval_us;
    gptimer_set_alarm_action(timer, &next_alarm);

    portEXIT_CRITICAL_ISR(&s_motion.spinlock);
    return high_task_woken == pdTRUE;
}

static esp_err_t motion_start_locked(motion_move_kind_t move_kind, motion_direction_t direction, int32_t target_position_steps, int32_t remaining_steps, float cruise_speed_steps_per_sec, float acceleration_steps_per_sec2)
{
    if (remaining_steps < 0 || cruise_speed_steps_per_sec <= 0.0f || acceleration_steps_per_sec2 <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_motion.running) {
        return ESP_ERR_INVALID_STATE;
    }

    bool left_pressed = motion_limit_pressed_raw(s_motion.pins.limit_left_pin);
    bool right_pressed = motion_limit_pressed_raw(s_motion.pins.limit_right_pin);
    s_motion.limits.left_raw_pressed = left_pressed;
    s_motion.limits.right_raw_pressed = right_pressed;

    if ((direction == MOTION_DIRECTION_NEGATIVE && left_pressed) ||
        (direction == MOTION_DIRECTION_POSITIVE && right_pressed)) {
        return ESP_ERR_INVALID_STATE;
    }

    motion_apply_direction_locked(direction);
    s_motion.move_kind = move_kind;
    s_motion.target_position_steps = target_position_steps;
    s_motion.remaining_steps = remaining_steps;
    s_motion.current_speed_steps_per_sec = 0.0f;
    s_motion.cruise_speed_steps_per_sec = cruise_speed_steps_per_sec;
    s_motion.acceleration_steps_per_sec2 = acceleration_steps_per_sec2;
    s_motion.first_step_speed_steps_per_sec = sqrtf(2.0f * acceleration_steps_per_sec2);
    if (s_motion.first_step_speed_steps_per_sec > cruise_speed_steps_per_sec) {
        s_motion.first_step_speed_steps_per_sec = cruise_speed_steps_per_sec;
    }
    if (s_motion.first_step_speed_steps_per_sec <= 0.0f) {
        s_motion.first_step_speed_steps_per_sec = cruise_speed_steps_per_sec;
    }
    s_motion.stop_requested = false;
    s_motion.step_high = false;
    s_motion.running = true;

    static gptimer_alarm_config_t alarm_config = {};
    uint32_t first_interval_us = motion_speed_to_interval_us(s_motion.first_step_speed_steps_per_sec);
    if (first_interval_us < DIR_SETTLE_US) {
        first_interval_us = DIR_SETTLE_US;
    }

    ESP_RETURN_ON_ERROR(gptimer_set_raw_count(s_motion.timer, 0), TAG, "Failed to reset timer count");
    alarm_config.alarm_count = first_interval_us;
    ESP_RETURN_ON_ERROR(gptimer_set_alarm_action(s_motion.timer, &alarm_config), TAG, "Failed to schedule motion alarm");
    return gptimer_start(s_motion.timer);
}

esp_err_t motion_axis_init(const motion_axis_pins_t *pins, const app_runtime_config_t *config)
{
    if (pins == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_motion.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_motion, 0, sizeof(s_motion));
    s_motion.spinlock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    s_motion.pins = *pins;
    s_motion.config = *config;
    s_motion.event_queue = xQueueCreate(MOTION_EVENT_QUEUE_LENGTH, sizeof(motion_event_t));
    if (s_motion.event_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << pins->step_pin) | (1ULL << pins->dir_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&output_config), TAG, "Failed to configure step and direction pins");
    gpio_set_level(pins->step_pin, 0);
    gpio_set_level(pins->dir_pin, 0);

    gpio_config_t input_config = {
        .pin_bit_mask = (1ULL << pins->limit_left_pin) | (1ULL << pins->limit_right_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&input_config), TAG, "Failed to configure limit switch pins");

    esp_err_t isr_service_result = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (isr_service_result != ESP_OK && isr_service_result != ESP_ERR_INVALID_STATE) {
        return isr_service_result;
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(pins->limit_left_pin, motion_limit_isr_handler, (void *)(uintptr_t)pins->limit_left_pin), TAG, "Failed to install left limit ISR");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(pins->limit_right_pin, motion_limit_isr_handler, (void *)(uintptr_t)pins->limit_right_pin), TAG, "Failed to install right limit ISR");

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    ESP_RETURN_ON_ERROR(gptimer_new_timer(&timer_config, &s_motion.timer), TAG, "Failed to create GPTimer");

    gptimer_event_callbacks_t callbacks = {
        .on_alarm = motion_timer_on_alarm,
    };
    ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(s_motion.timer, &callbacks, NULL), TAG, "Failed to register GPTimer callback");
    ESP_RETURN_ON_ERROR(gptimer_enable(s_motion.timer), TAG, "Failed to enable GPTimer");

    s_motion.limits.left_raw_pressed = motion_limit_pressed_raw(pins->limit_left_pin);
    s_motion.limits.right_raw_pressed = motion_limit_pressed_raw(pins->limit_right_pin);
    s_motion.limits.left_pressed = s_motion.limits.left_raw_pressed;
    s_motion.limits.right_pressed = s_motion.limits.right_raw_pressed;
    s_motion.left_last_sample = s_motion.limits.left_raw_pressed;
    s_motion.right_last_sample = s_motion.limits.right_raw_pressed;
    s_motion.initialized = true;

    ESP_LOGI(TAG, "Motion axis initialized on STEP=%d DIR=%d LIMIT_LEFT=%d LIMIT_RIGHT=%d",
             pins->step_pin, pins->dir_pin, pins->limit_left_pin, pins->limit_right_pin);
    return ESP_OK;
}

esp_err_t motion_axis_apply_runtime_config(const app_runtime_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_motion.spinlock);
    if (s_motion.running) {
        portEXIT_CRITICAL(&s_motion.spinlock);
        return ESP_ERR_INVALID_STATE;
    }
    s_motion.config = *config;
    portEXIT_CRITICAL(&s_motion.spinlock);
    return ESP_OK;
}

esp_err_t motion_axis_start_profiled_move(int32_t target_position_steps, float cruise_speed_steps_per_sec, float acceleration_steps_per_sec2)
{
    int32_t current_position_steps = motion_axis_get_current_position_steps();
    int32_t delta_steps = target_position_steps - current_position_steps;
    if (delta_steps == 0) {
        return ESP_OK;
    }

    motion_direction_t direction = delta_steps > 0 ? MOTION_DIRECTION_POSITIVE : MOTION_DIRECTION_NEGATIVE;
    int32_t remaining_steps = delta_steps > 0 ? delta_steps : -delta_steps;

    portENTER_CRITICAL(&s_motion.spinlock);
    esp_err_t result = motion_start_locked(
        MOTION_MOVE_POSITION,
        direction,
        target_position_steps,
        remaining_steps,
        cruise_speed_steps_per_sec,
        acceleration_steps_per_sec2);
    portEXIT_CRITICAL(&s_motion.spinlock);
    return result;
}

esp_err_t motion_axis_start_timed_move(int32_t target_position_steps, float duration_seconds, float max_speed_steps_per_sec, float max_acceleration_steps_per_sec2)
{
    int32_t current_position_steps = motion_axis_get_current_position_steps();
    int32_t distance_steps = target_position_steps - current_position_steps;
    float planned_speed_steps_per_sec = 0.0f;
    float planned_acceleration_steps_per_sec2 = 0.0f;
    if (!motion_axis_plan_exact_move(distance_steps, duration_seconds, max_speed_steps_per_sec, max_acceleration_steps_per_sec2, &planned_speed_steps_per_sec, &planned_acceleration_steps_per_sec2)) {
        return ESP_ERR_INVALID_ARG;
    }

    return motion_axis_start_profiled_move(target_position_steps, planned_speed_steps_per_sec, planned_acceleration_steps_per_sec2);
}

esp_err_t motion_axis_start_continuous_move(motion_direction_t direction, float cruise_speed_steps_per_sec, float acceleration_steps_per_sec2)
{
    portENTER_CRITICAL(&s_motion.spinlock);
    esp_err_t result = motion_start_locked(
        MOTION_MOVE_CONTINUOUS,
        direction,
        s_motion.current_position_steps,
        INT32_MAX,
        cruise_speed_steps_per_sec,
        acceleration_steps_per_sec2);
    portEXIT_CRITICAL(&s_motion.spinlock);
    return result;
}

esp_err_t motion_axis_request_stop(void)
{
    portENTER_CRITICAL(&s_motion.spinlock);
    if (!s_motion.running) {
        portEXIT_CRITICAL(&s_motion.spinlock);
        return ESP_OK;
    }

    s_motion.stop_requested = true;
    portEXIT_CRITICAL(&s_motion.spinlock);
    return ESP_OK;
}

esp_err_t motion_axis_emergency_stop(void)
{
    portENTER_CRITICAL(&s_motion.spinlock);
    if (!s_motion.running) {
        portEXIT_CRITICAL(&s_motion.spinlock);
        return ESP_OK;
    }

    s_motion.running = false;
    s_motion.step_high = false;
    s_motion.stop_requested = false;
    s_motion.current_speed_steps_per_sec = 0.0f;
    s_motion.cruise_speed_steps_per_sec = 0.0f;
    s_motion.acceleration_steps_per_sec2 = 0.0f;
    s_motion.move_kind = MOTION_MOVE_IDLE;
    gptimer_stop(s_motion.timer);
    gpio_set_level(s_motion.pins.step_pin, 0);
    portEXIT_CRITICAL(&s_motion.spinlock);

    motion_push_event_from_task(MOTION_EVENT_STOPPED);
    return ESP_OK;
}

esp_err_t motion_axis_set_current_position(int32_t position_steps)
{
    portENTER_CRITICAL(&s_motion.spinlock);
    if (s_motion.running) {
        portEXIT_CRITICAL(&s_motion.spinlock);
        return ESP_ERR_INVALID_STATE;
    }
    s_motion.current_position_steps = position_steps;
    portEXIT_CRITICAL(&s_motion.spinlock);
    return ESP_OK;
}

int32_t motion_axis_get_current_position_steps(void)
{
    int32_t position_steps = 0;
    portENTER_CRITICAL(&s_motion.spinlock);
    position_steps = s_motion.current_position_steps;
    portEXIT_CRITICAL(&s_motion.spinlock);
    return position_steps;
}

bool motion_axis_is_busy(void)
{
    bool running = false;
    portENTER_CRITICAL(&s_motion.spinlock);
    running = s_motion.running;
    portEXIT_CRITICAL(&s_motion.spinlock);
    return running;
}

void motion_axis_service(void)
{
    uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
    bool left_raw_pressed = motion_limit_pressed_raw(s_motion.pins.limit_left_pin);
    bool right_raw_pressed = motion_limit_pressed_raw(s_motion.pins.limit_right_pin);

    portENTER_CRITICAL(&s_motion.spinlock);

    s_motion.limits.left_raw_pressed = left_raw_pressed;
    s_motion.limits.right_raw_pressed = right_raw_pressed;

    if (left_raw_pressed != s_motion.left_last_sample) {
        s_motion.left_last_sample = left_raw_pressed;
        s_motion.left_last_change_ms = now_ms;
    } else if (s_motion.limits.left_pressed != left_raw_pressed &&
               (now_ms - s_motion.left_last_change_ms) >= s_motion.config.homing.limit_debounce_ms) {
        s_motion.limits.left_pressed = left_raw_pressed;
    }

    if (right_raw_pressed != s_motion.right_last_sample) {
        s_motion.right_last_sample = right_raw_pressed;
        s_motion.right_last_change_ms = now_ms;
    } else if (s_motion.limits.right_pressed != right_raw_pressed &&
               (now_ms - s_motion.right_last_change_ms) >= s_motion.config.homing.limit_debounce_ms) {
        s_motion.limits.right_pressed = right_raw_pressed;
    }

    portEXIT_CRITICAL(&s_motion.spinlock);
}

bool motion_axis_poll_event(motion_event_t *event_out)
{
    if (event_out == NULL || s_motion.event_queue == NULL) {
        return false;
    }

    return xQueueReceive(s_motion.event_queue, event_out, 0) == pdTRUE;
}

void motion_axis_get_status(motion_axis_status_t *status_out)
{
    if (status_out == NULL) {
        return;
    }

    memset(status_out, 0, sizeof(*status_out));

    portENTER_CRITICAL(&s_motion.spinlock);
    status_out->current_position_steps = s_motion.current_position_steps;
    status_out->target_position_steps = s_motion.target_position_steps;
    status_out->current_speed_steps_per_sec = s_motion.current_speed_steps_per_sec;
    status_out->cruise_speed_steps_per_sec = s_motion.cruise_speed_steps_per_sec;
    status_out->acceleration_steps_per_sec2 = s_motion.acceleration_steps_per_sec2;
    status_out->moving = s_motion.running;
    status_out->continuous = s_motion.move_kind == MOTION_MOVE_CONTINUOUS;
    status_out->stop_requested = s_motion.stop_requested;
    status_out->direction = s_motion.direction;
    status_out->limits = s_motion.limits;
    portEXIT_CRITICAL(&s_motion.spinlock);
}

float motion_axis_compute_min_time_s(int32_t distance_steps, float max_speed_steps_per_sec, float max_acceleration_steps_per_sec2)
{
    float distance = fabsf((float)distance_steps);
    if (distance <= 0.0f || max_speed_steps_per_sec <= 0.0f || max_acceleration_steps_per_sec2 <= 0.0f) {
        return 0.0f;
    }

    float ramp_distance = (max_speed_steps_per_sec * max_speed_steps_per_sec) / max_acceleration_steps_per_sec2;
    if (distance <= ramp_distance) {
        return 2.0f * sqrtf(distance / max_acceleration_steps_per_sec2);
    }

    float cruise_distance = distance - ramp_distance;
    return (2.0f * max_speed_steps_per_sec / max_acceleration_steps_per_sec2) + (cruise_distance / max_speed_steps_per_sec);
}

bool motion_axis_plan_exact_move(int32_t distance_steps, float duration_seconds, float max_speed_steps_per_sec, float max_acceleration_steps_per_sec2, float *planned_cruise_speed_steps_per_sec, float *planned_acceleration_steps_per_sec2)
{
    float distance = fabsf((float)distance_steps);
    if (planned_cruise_speed_steps_per_sec == NULL || planned_acceleration_steps_per_sec2 == NULL) {
        return false;
    }

    if (distance <= 0.0f || duration_seconds <= 0.0f || max_speed_steps_per_sec <= 0.0f || max_acceleration_steps_per_sec2 <= 0.0f) {
        return false;
    }

    float minimum_time_seconds = motion_axis_compute_min_time_s(distance_steps, max_speed_steps_per_sec, max_acceleration_steps_per_sec2);
    if (duration_seconds + 0.0001f < minimum_time_seconds) {
        return false;
    }

    float triangular_peak_speed = (2.0f * distance) / duration_seconds;
    float triangular_acceleration = (4.0f * distance) / (duration_seconds * duration_seconds);

    if (triangular_peak_speed <= max_speed_steps_per_sec + 0.0001f &&
        triangular_acceleration <= max_acceleration_steps_per_sec2 + 0.0001f) {
        *planned_cruise_speed_steps_per_sec = triangular_peak_speed;
        *planned_acceleration_steps_per_sec2 = triangular_acceleration;
        return true;
    }

    float denominator = (max_speed_steps_per_sec * duration_seconds) - distance;
    if (denominator <= 0.0f) {
        return false;
    }

    float trapezoidal_acceleration = (max_speed_steps_per_sec * max_speed_steps_per_sec) / denominator;
    if (trapezoidal_acceleration > max_acceleration_steps_per_sec2 + 0.0001f) {
        return false;
    }

    *planned_cruise_speed_steps_per_sec = max_speed_steps_per_sec;
    *planned_acceleration_steps_per_sec2 = trapezoidal_acceleration;
    return true;
}
