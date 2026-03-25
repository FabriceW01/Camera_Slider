#include "app_controller.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "motion_axis.h"

typedef enum {
    APP_HOME_PHASE_IDLE = 0,
    APP_HOME_PHASE_CLEAR_SWITCH,
    APP_HOME_PHASE_FAST_APPROACH,
    APP_HOME_PHASE_BACKOFF,
    APP_HOME_PHASE_SLOW_APPROACH,
} app_home_phase_t;

typedef struct {
    bool active;
    app_home_phase_t phase;
} app_home_context_t;

typedef struct {
    bool active;
    bool prepositioning_to_a;
    bool moving_to_b;
    app_repeat_mode_t repeat_mode;
} app_ab_context_t;

typedef struct {
    bool loaded;
    bool active;
    bool paused;
    bool prepositioning;
    bool loop_repositioning;
    bool reverse_direction;
    bool pause_requested;
    int32_t paused_target_position_steps;
    uint32_t paused_remaining_duration_ms;
    int current_from_index;
    int current_to_index;
    uint32_t current_segment_duration_ms;
    uint64_t current_segment_started_ms;
    app_repeat_mode_t repeat_mode;
    app_keyframe_program_t program;
} app_program_context_t;

typedef struct {
    QueueHandle_t command_queue;
    SemaphoreHandle_t lock;
    TaskHandle_t task_handle;
    app_runtime_config_t config;
    app_status_snapshot_t snapshot;
    app_home_context_t homing;
    app_ab_context_t ab;
    app_program_context_t program;
    bool single_move_active;
    bool stop_requested;
    bool pending_jog_valid;
    int32_t pending_jog_speed_steps_per_sec;
    app_recent_error_t recent_errors[APP_MAX_RECENT_ERRORS];
    size_t recent_error_count;
    size_t recent_error_head;
    bool started;
} app_controller_ctx_t;

static const char *TAG = "app_controller";
static const size_t APP_COMMAND_QUEUE_LENGTH = 16;
static const uint32_t APP_CONTROL_TICK_MS = 20;
static const motion_axis_pins_t APP_AXIS_PINS = {
    .step_pin = GPIO_NUM_2,
    .dir_pin = GPIO_NUM_1,
    .limit_left_pin = GPIO_NUM_3,
    .limit_right_pin = GPIO_NUM_4,
};

static app_controller_ctx_t s_app = {};

static uint64_t app_controller_now_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000);
}

static bool app_controller_is_homed(void)
{
    return s_app.snapshot.homed;
}

static int32_t app_controller_max_position_steps(const app_runtime_config_t *config)
{
    return app_config_mm_to_steps(config, config->mechanics.slider_travel_mm);
}

static motion_direction_t app_controller_home_direction(const app_runtime_config_t *config)
{
    return config->homing.home_reference_side == APP_HOME_REFERENCE_LEFT
        ? MOTION_DIRECTION_NEGATIVE
        : MOTION_DIRECTION_POSITIVE;
}

static motion_direction_t app_controller_away_from_home_direction(const app_runtime_config_t *config)
{
    return config->homing.home_reference_side == APP_HOME_REFERENCE_LEFT
        ? MOTION_DIRECTION_POSITIVE
        : MOTION_DIRECTION_NEGATIVE;
}

static motion_event_type_t app_controller_expected_home_limit_event(const app_runtime_config_t *config)
{
    return config->homing.home_reference_side == APP_HOME_REFERENCE_LEFT
        ? MOTION_EVENT_LIMIT_LEFT
        : MOTION_EVENT_LIMIT_RIGHT;
}

static bool app_controller_limit_event_matches_home(const app_runtime_config_t *config, motion_event_type_t event_type)
{
    return event_type == app_controller_expected_home_limit_event(config);
}

static int32_t app_controller_home_reference_position_steps(const app_runtime_config_t *config)
{
    if (config->homing.home_reference_side == APP_HOME_REFERENCE_LEFT) {
        return 0;
    }

    return app_controller_max_position_steps(config);
}

static void app_controller_reset_transient_actions(void)
{
    s_app.homing.active = false;
    s_app.homing.phase = APP_HOME_PHASE_IDLE;
    s_app.ab.active = false;
    s_app.ab.prepositioning_to_a = false;
    s_app.ab.moving_to_b = false;
    s_app.program.active = false;
    s_app.program.paused = false;
    s_app.program.prepositioning = false;
    s_app.program.loop_repositioning = false;
    s_app.program.reverse_direction = false;
    s_app.program.pause_requested = false;
    s_app.program.current_from_index = 0;
    s_app.program.current_to_index = 0;
    s_app.program.current_segment_duration_ms = 0;
    s_app.program.current_segment_started_ms = 0;
    s_app.single_move_active = false;
    s_app.stop_requested = false;
    s_app.pending_jog_valid = false;
    s_app.pending_jog_speed_steps_per_sec = 0;
}

static void app_controller_record_error(const char *format, ...)
{
    char message[APP_ERROR_MESSAGE_MAX];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    size_t slot = s_app.recent_error_head % APP_MAX_RECENT_ERRORS;
    s_app.recent_errors[slot].uptime_ms = app_controller_now_ms();
    snprintf(s_app.recent_errors[slot].message, sizeof(s_app.recent_errors[slot].message), "%s", message);
    s_app.recent_error_head = (slot + 1) % APP_MAX_RECENT_ERRORS;
    if (s_app.recent_error_count < APP_MAX_RECENT_ERRORS) {
        s_app.recent_error_count++;
    }

    snprintf(s_app.snapshot.last_error, sizeof(s_app.snapshot.last_error), "%s", message);
    ESP_LOGE(TAG, "%s", message);
}

static void app_controller_sync_recent_errors_locked(void)
{
    s_app.snapshot.recent_error_count = s_app.recent_error_count;
    for (size_t i = 0; i < s_app.recent_error_count; ++i) {
        size_t ring_index = (s_app.recent_error_head + APP_MAX_RECENT_ERRORS - 1 - i) % APP_MAX_RECENT_ERRORS;
        s_app.snapshot.recent_errors[i] = s_app.recent_errors[ring_index];
    }
}

static void app_controller_set_state(app_state_t state, app_active_mode_t active_mode)
{
    s_app.snapshot.state = state;
    s_app.snapshot.active_mode = active_mode;
}

static void app_controller_set_idle_or_ready_state(void)
{
    if (s_app.snapshot.homed) {
        app_controller_set_state(APP_STATE_READY, APP_ACTIVE_MODE_NONE);
    } else {
        app_controller_set_state(APP_STATE_IDLE, APP_ACTIVE_MODE_NONE);
    }
}

static bool app_controller_target_within_bounds(int32_t target_position_steps)
{
    if (!s_app.snapshot.homed) {
        return true;
    }

    return target_position_steps >= s_app.snapshot.min_position_steps &&
           target_position_steps <= s_app.snapshot.max_position_steps;
}

static void app_controller_refresh_snapshot(void)
{
    motion_axis_status_t motion_status = {};
    motion_axis_get_status(&motion_status);

    xSemaphoreTake(s_app.lock, portMAX_DELAY);
    s_app.snapshot.moving = motion_status.moving;
    s_app.snapshot.position_steps = motion_status.current_position_steps;
    s_app.snapshot.position_mm = app_config_steps_to_mm(&s_app.config, motion_status.current_position_steps);
    s_app.snapshot.limits = motion_status.limits;
    s_app.snapshot.min_position_steps = 0;
    s_app.snapshot.max_position_steps = app_controller_max_position_steps(&s_app.config);
    s_app.snapshot.repeat_mode = s_app.config.axis_behavior.repeat_mode;
    s_app.snapshot.loaded_keyframe_count = s_app.program.loaded ? s_app.program.program.count : 0;
    s_app.snapshot.free_heap_bytes = esp_get_free_heap_size();
    s_app.snapshot.uptime_ms = app_controller_now_ms();
    app_controller_sync_recent_errors_locked();
    xSemaphoreGive(s_app.lock);
}

static esp_err_t app_controller_start_position_move(int32_t target_position_steps, int32_t custom_speed_steps_per_sec, bool use_custom_speed, app_active_mode_t active_mode)
{
    if (!app_controller_target_within_bounds(target_position_steps)) {
        return ESP_ERR_INVALID_ARG;
    }

    float speed_steps_per_sec = use_custom_speed
        ? (float)custom_speed_steps_per_sec
        : s_app.config.motion.max_speed_steps_per_sec;
    esp_err_t result = motion_axis_start_profiled_move(
        target_position_steps,
        speed_steps_per_sec,
        s_app.config.motion.acceleration_steps_per_sec2);
    if (result == ESP_OK) {
        s_app.single_move_active = true;
        app_controller_set_state(APP_STATE_RUNNING_PROGRAM, active_mode);
    }
    return result;
}

static void app_controller_finish_program(void)
{
    s_app.program.active = false;
    s_app.program.paused = false;
    s_app.program.pause_requested = false;
    s_app.program.prepositioning = false;
    s_app.program.loop_repositioning = false;
    s_app.program.current_segment_duration_ms = 0;
    app_controller_set_idle_or_ready_state();
}

static esp_err_t app_controller_start_program_segment(int from_index, int to_index, uint32_t duration_ms)
{
    if (!s_app.program.active || duration_ms == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t target_position_steps = s_app.program.program.keyframes[to_index].position_steps;
    esp_err_t result = motion_axis_start_timed_move(
        target_position_steps,
        (float)duration_ms / 1000.0f,
        s_app.config.motion.max_speed_steps_per_sec,
        s_app.config.motion.acceleration_steps_per_sec2);
    if (result != ESP_OK) {
        return result;
    }

    s_app.program.current_from_index = from_index;
    s_app.program.current_to_index = to_index;
    s_app.program.current_segment_duration_ms = duration_ms;
    s_app.program.current_segment_started_ms = app_controller_now_ms();
    s_app.program.prepositioning = false;
    s_app.program.loop_repositioning = false;
    app_controller_set_state(APP_STATE_RUNNING_PROGRAM, APP_ACTIVE_MODE_KEYFRAME);
    return ESP_OK;
}

static uint32_t app_controller_segment_duration_ms(const app_keyframe_program_t *program, int from_index, int to_index)
{
    uint32_t from_time = program->keyframes[from_index].time_ms;
    uint32_t to_time = program->keyframes[to_index].time_ms;
    return to_time > from_time ? (to_time - from_time) : 0;
}

static void app_controller_start_program_sequence(void)
{
    if (s_app.program.program.count < 2) {
        app_controller_finish_program();
        return;
    }

    uint32_t first_duration_ms = app_controller_segment_duration_ms(&s_app.program.program, 0, 1);
    if (app_controller_start_program_segment(0, 1, first_duration_ms) != ESP_OK) {
        app_controller_record_error("Unable to start the first keyframe segment.");
        app_controller_finish_program();
    }
}

static esp_err_t app_controller_start_homing_phase(app_home_phase_t phase)
{
    esp_err_t result = ESP_OK;
    motion_direction_t direction = MOTION_DIRECTION_NEGATIVE;

    switch (phase) {
        case APP_HOME_PHASE_CLEAR_SWITCH:
            direction = app_controller_away_from_home_direction(&s_app.config);
            result = motion_axis_start_profiled_move(
                motion_axis_get_current_position_steps() + ((direction == MOTION_DIRECTION_POSITIVE ? 1 : -1) * s_app.config.homing.homing_backoff_steps),
                s_app.config.homing.homing_speed_steps_per_sec,
                s_app.config.motion.acceleration_steps_per_sec2);
            break;
        case APP_HOME_PHASE_FAST_APPROACH:
            direction = app_controller_home_direction(&s_app.config);
            result = motion_axis_start_continuous_move(
                direction,
                s_app.config.homing.homing_speed_steps_per_sec,
                s_app.config.motion.acceleration_steps_per_sec2);
            break;
        case APP_HOME_PHASE_BACKOFF:
            direction = app_controller_away_from_home_direction(&s_app.config);
            result = motion_axis_start_profiled_move(
                motion_axis_get_current_position_steps() + ((direction == MOTION_DIRECTION_POSITIVE ? 1 : -1) * s_app.config.homing.homing_backoff_steps),
                s_app.config.homing.homing_speed_steps_per_sec,
                s_app.config.motion.acceleration_steps_per_sec2);
            break;
        case APP_HOME_PHASE_SLOW_APPROACH:
            direction = app_controller_home_direction(&s_app.config);
            result = motion_axis_start_continuous_move(
                direction,
                s_app.config.homing.homing_slow_speed_steps_per_sec,
                s_app.config.motion.acceleration_steps_per_sec2);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    if (result == ESP_OK) {
        s_app.homing.active = true;
        s_app.homing.phase = phase;
        app_controller_set_state(APP_STATE_HOMING, APP_ACTIVE_MODE_HOMING);
        s_app.snapshot.homed = false;
    }

    return result;
}

static esp_err_t app_controller_begin_homing(void)
{
    motion_axis_status_t motion_status = {};
    motion_axis_get_status(&motion_status);
    if (motion_status.moving) {
        return ESP_ERR_INVALID_STATE;
    }

    app_controller_reset_transient_actions();
    bool home_limit_pressed = s_app.config.homing.home_reference_side == APP_HOME_REFERENCE_LEFT
        ? motion_status.limits.left_raw_pressed
        : motion_status.limits.right_raw_pressed;
    if (home_limit_pressed) {
        return app_controller_start_homing_phase(APP_HOME_PHASE_CLEAR_SWITCH);
    }

    return app_controller_start_homing_phase(APP_HOME_PHASE_FAST_APPROACH);
}

static void app_controller_stop_and_fail_limit(const char *message)
{
    s_app.snapshot.homed = false;
    app_controller_reset_transient_actions();
    app_controller_set_state(APP_STATE_LIMIT_TRIGGERED, APP_ACTIVE_MODE_NONE);
    app_controller_record_error("%s", message);
}

static void app_controller_finalize_homing_success(void)
{
    int32_t home_position_steps = app_controller_home_reference_position_steps(&s_app.config);
    motion_axis_set_current_position(home_position_steps);
    s_app.snapshot.homed = true;
    s_app.homing.active = false;
    s_app.homing.phase = APP_HOME_PHASE_IDLE;
    app_controller_set_state(APP_STATE_READY, APP_ACTIVE_MODE_NONE);
}

static void app_controller_handle_limit_event(const motion_event_t *event)
{
    if (s_app.homing.active) {
        bool expected_home_limit = app_controller_limit_event_matches_home(&s_app.config, event->type);
        switch (s_app.homing.phase) {
            case APP_HOME_PHASE_FAST_APPROACH:
                if (expected_home_limit) {
                    if (app_controller_start_homing_phase(APP_HOME_PHASE_BACKOFF) != ESP_OK) {
                        app_controller_stop_and_fail_limit("Failed to start homing backoff after the first switch hit.");
                    }
                } else {
                    app_controller_stop_and_fail_limit("The opposite-end switch triggered during the fast homing approach.");
                }
                return;
            case APP_HOME_PHASE_SLOW_APPROACH:
                if (expected_home_limit) {
                    app_controller_finalize_homing_success();
                } else {
                    app_controller_stop_and_fail_limit("The opposite-end switch triggered during the slow homing approach.");
                }
                return;
            default:
                app_controller_stop_and_fail_limit("A limit switch triggered during an unexpected homing phase.");
                return;
        }
    }

    app_controller_stop_and_fail_limit(
        event->type == MOTION_EVENT_LIMIT_LEFT
            ? "Left limit switch triggered during motion."
            : "Right limit switch triggered during motion.");
}

static void app_controller_start_ab_leg(bool moving_to_b)
{
    int32_t target_steps = moving_to_b
        ? s_app.snapshot.ab_points.b_position_steps
        : s_app.snapshot.ab_points.a_position_steps;
    if (motion_axis_start_profiled_move(
            target_steps,
            s_app.config.motion.max_speed_steps_per_sec,
            s_app.config.motion.acceleration_steps_per_sec2) == ESP_OK) {
        s_app.ab.active = true;
        s_app.ab.prepositioning_to_a = false;
        s_app.ab.moving_to_b = moving_to_b;
        app_controller_set_state(APP_STATE_RUNNING_PROGRAM, APP_ACTIVE_MODE_AB);
    } else {
        app_controller_record_error("Unable to start the next A/B segment.");
        s_app.ab.active = false;
        app_controller_set_idle_or_ready_state();
    }
}

static void app_controller_handle_move_complete(void)
{
    if (s_app.homing.active) {
        if (s_app.homing.phase == APP_HOME_PHASE_CLEAR_SWITCH) {
            if (app_controller_start_homing_phase(APP_HOME_PHASE_FAST_APPROACH) != ESP_OK) {
                app_controller_record_error("Unable to start the fast homing approach.");
                app_controller_set_state(APP_STATE_ERROR, APP_ACTIVE_MODE_HOMING);
            }
            return;
        }

        if (s_app.homing.phase == APP_HOME_PHASE_BACKOFF) {
            if (app_controller_start_homing_phase(APP_HOME_PHASE_SLOW_APPROACH) != ESP_OK) {
                app_controller_record_error("Unable to start the slow homing approach.");
                app_controller_set_state(APP_STATE_ERROR, APP_ACTIVE_MODE_HOMING);
            }
        }
        return;
    }

    if (s_app.program.active) {
        if (s_app.program.prepositioning) {
            app_controller_start_program_sequence();
            return;
        }

        if (s_app.program.loop_repositioning) {
            s_app.program.loop_repositioning = false;
            s_app.program.reverse_direction = false;
            app_controller_start_program_sequence();
            return;
        }

        if (!s_app.program.reverse_direction) {
            if (s_app.program.current_to_index < (int)s_app.program.program.count - 1) {
                int from_index = s_app.program.current_to_index;
                int to_index = from_index + 1;
                app_controller_start_program_segment(
                    from_index,
                    to_index,
                    app_controller_segment_duration_ms(&s_app.program.program, from_index, to_index));
                return;
            }

            if (s_app.program.repeat_mode == APP_REPEAT_MODE_PING_PONG && s_app.program.program.count > 1) {
                s_app.program.reverse_direction = true;
                int from_index = (int)s_app.program.program.count - 1;
                int to_index = from_index - 1;
                app_controller_start_program_segment(
                    from_index,
                    to_index,
                    app_controller_segment_duration_ms(&s_app.program.program, to_index, from_index));
                return;
            }

            if (s_app.program.repeat_mode == APP_REPEAT_MODE_LOOP) {
                int32_t first_position_steps = s_app.program.program.keyframes[0].position_steps;
                if (motion_axis_start_profiled_move(
                        first_position_steps,
                        s_app.config.motion.max_speed_steps_per_sec,
                        s_app.config.motion.acceleration_steps_per_sec2) == ESP_OK) {
                    s_app.program.loop_repositioning = true;
                    app_controller_set_state(APP_STATE_RUNNING_PROGRAM, APP_ACTIVE_MODE_KEYFRAME);
                    return;
                }

                app_controller_record_error("Unable to loop keyframes back to the first waypoint.");
                app_controller_finish_program();
                return;
            }

            app_controller_finish_program();
            return;
        }

        if (s_app.program.current_to_index > 0) {
            int from_index = s_app.program.current_to_index;
            int to_index = from_index - 1;
            app_controller_start_program_segment(
                from_index,
                to_index,
                app_controller_segment_duration_ms(&s_app.program.program, to_index, from_index));
            return;
        }

        s_app.program.reverse_direction = false;
        if (s_app.program.repeat_mode == APP_REPEAT_MODE_PING_PONG) {
            app_controller_start_program_sequence();
            return;
        }

        app_controller_finish_program();
        return;
    }

    if (s_app.ab.active) {
        if (s_app.ab.prepositioning_to_a) {
            app_controller_start_ab_leg(true);
            return;
        }

        if (s_app.ab.moving_to_b) {
            if (s_app.ab.repeat_mode == APP_REPEAT_MODE_NONE) {
                s_app.ab.active = false;
                app_controller_set_idle_or_ready_state();
                return;
            }

            app_controller_start_ab_leg(false);
            return;
        }

        app_controller_start_ab_leg(true);
        return;
    }

    if (s_app.single_move_active) {
        s_app.single_move_active = false;
        app_controller_set_idle_or_ready_state();
        return;
    }
}

static void app_controller_handle_stopped(void)
{
    if (s_app.program.active && s_app.program.pause_requested) {
        s_app.program.pause_requested = false;
        s_app.program.paused = true;
        app_controller_set_state(APP_STATE_PAUSED, APP_ACTIVE_MODE_KEYFRAME);
        return;
    }

    if (s_app.pending_jog_valid) {
        int32_t pending_speed = s_app.pending_jog_speed_steps_per_sec;
        s_app.pending_jog_valid = false;
        s_app.pending_jog_speed_steps_per_sec = 0;
        if (pending_speed != 0) {
            motion_direction_t direction = pending_speed > 0 ? MOTION_DIRECTION_POSITIVE : MOTION_DIRECTION_NEGATIVE;
            if (motion_axis_start_continuous_move(
                    direction,
                    (float)(pending_speed > 0 ? pending_speed : -pending_speed),
                    s_app.config.motion.acceleration_steps_per_sec2) == ESP_OK) {
                app_controller_set_state(APP_STATE_MANUAL_JOG, APP_ACTIVE_MODE_MANUAL);
                return;
            }
        }
    }

    if (s_app.stop_requested) {
        s_app.stop_requested = false;
        app_controller_reset_transient_actions();
    }

    app_controller_set_idle_or_ready_state();
}

static void app_controller_apply_config_patch(const app_config_patch_t *patch)
{
    app_runtime_config_t next_config = s_app.config;

    if (patch->mask & APP_CONFIG_PATCH_MAX_SPEED) {
        next_config.motion.max_speed_steps_per_sec = patch->values.motion.max_speed_steps_per_sec;
    }
    if (patch->mask & APP_CONFIG_PATCH_ACCELERATION) {
        next_config.motion.acceleration_steps_per_sec2 = patch->values.motion.acceleration_steps_per_sec2;
    }
    if (patch->mask & APP_CONFIG_PATCH_JOG_SPEED) {
        next_config.motion.jog_speed_steps_per_sec = patch->values.motion.jog_speed_steps_per_sec;
    }
    if (patch->mask & APP_CONFIG_PATCH_HOMING_SPEED) {
        next_config.homing.homing_speed_steps_per_sec = patch->values.homing.homing_speed_steps_per_sec;
    }
    if (patch->mask & APP_CONFIG_PATCH_HOMING_SLOW_SPEED) {
        next_config.homing.homing_slow_speed_steps_per_sec = patch->values.homing.homing_slow_speed_steps_per_sec;
    }
    if (patch->mask & APP_CONFIG_PATCH_HOMING_BACKOFF) {
        next_config.homing.homing_backoff_steps = patch->values.homing.homing_backoff_steps;
    }
    if (patch->mask & APP_CONFIG_PATCH_LIMIT_DEBOUNCE) {
        next_config.homing.limit_debounce_ms = patch->values.homing.limit_debounce_ms;
    }
    if (patch->mask & APP_CONFIG_PATCH_STEPS_PER_REV) {
        next_config.mechanics.steps_per_revolution = patch->values.mechanics.steps_per_revolution;
    }
    if (patch->mask & APP_CONFIG_PATCH_MICROSTEPS) {
        next_config.mechanics.microsteps = patch->values.mechanics.microsteps;
    }
    if (patch->mask & APP_CONFIG_PATCH_BELT_PITCH) {
        next_config.mechanics.belt_pitch_mm = patch->values.mechanics.belt_pitch_mm;
    }
    if (patch->mask & APP_CONFIG_PATCH_PULLEY_TEETH) {
        next_config.mechanics.pulley_teeth = patch->values.mechanics.pulley_teeth;
    }
    if (patch->mask & APP_CONFIG_PATCH_STEPS_PER_MM) {
        next_config.mechanics.steps_per_mm = patch->values.mechanics.steps_per_mm;
    }
    if (patch->mask & APP_CONFIG_PATCH_SLIDER_TRAVEL) {
        next_config.mechanics.slider_travel_mm = patch->values.mechanics.slider_travel_mm;
    }
    if (patch->mask & APP_CONFIG_PATCH_HOME_SIDE) {
        next_config.homing.home_reference_side = patch->values.homing.home_reference_side;
    }
    if (patch->mask & APP_CONFIG_PATCH_INVERT_DIRECTION) {
        next_config.axis_behavior.invert_direction_output = patch->values.axis_behavior.invert_direction_output;
    }
    if (patch->mask & APP_CONFIG_PATCH_REPEAT_MODE) {
        next_config.axis_behavior.repeat_mode = patch->values.axis_behavior.repeat_mode;
    }
    if (patch->mask & APP_CONFIG_PATCH_MOTION_PROFILE) {
        next_config.motion.motion_profile = patch->values.motion.motion_profile;
    }

    next_config.mechanics.effective_steps_per_revolution =
        next_config.mechanics.steps_per_revolution * next_config.mechanics.microsteps;
    next_config.mechanics.travel_mm_per_revolution =
        next_config.mechanics.belt_pitch_mm * (float)next_config.mechanics.pulley_teeth;
    if (!(patch->mask & APP_CONFIG_PATCH_STEPS_PER_MM) &&
        next_config.mechanics.travel_mm_per_revolution > 0.0f) {
        next_config.mechanics.steps_per_mm =
            (float)next_config.mechanics.effective_steps_per_revolution / next_config.mechanics.travel_mm_per_revolution;
    }

    char validation_error[APP_ERROR_MESSAGE_MAX];
    if (!app_config_validate(&next_config, validation_error, sizeof(validation_error))) {
        app_controller_record_error("%s", validation_error);
        return;
    }

    if (motion_axis_apply_runtime_config(&next_config) != ESP_OK) {
        app_controller_record_error("Runtime configuration can only be changed while the axis is idle.");
        return;
    }

    bool requires_rehome = (patch->mask & (APP_CONFIG_PATCH_STEPS_PER_REV |
                                           APP_CONFIG_PATCH_MICROSTEPS |
                                           APP_CONFIG_PATCH_BELT_PITCH |
                                           APP_CONFIG_PATCH_PULLEY_TEETH |
                                           APP_CONFIG_PATCH_STEPS_PER_MM |
                                           APP_CONFIG_PATCH_SLIDER_TRAVEL |
                                           APP_CONFIG_PATCH_HOME_SIDE |
                                           APP_CONFIG_PATCH_INVERT_DIRECTION)) != 0;

    s_app.config = next_config;
    if (requires_rehome) {
        s_app.snapshot.homed = false;
        app_controller_set_state(APP_STATE_IDLE, APP_ACTIVE_MODE_NONE);
    }
}

static void app_controller_handle_command(const app_command_t *command)
{
    if (command == NULL) {
        return;
    }

    switch (command->type) {
        case APP_COMMAND_HOME:
            if (app_controller_begin_homing() != ESP_OK) {
                app_controller_record_error("Unable to start homing from the current state.");
            }
            break;
        case APP_COMMAND_STOP:
        case APP_COMMAND_STOP_PROGRAM:
            s_app.stop_requested = true;
            motion_axis_request_stop();
            break;
        case APP_COMMAND_JOG: {
            int32_t requested_speed = command->data.jog.speed_steps_per_sec;
            if (requested_speed == 0) {
                s_app.pending_jog_valid = false;
                motion_axis_request_stop();
                break;
            }

            motion_direction_t direction = requested_speed > 0 ? MOTION_DIRECTION_POSITIVE : MOTION_DIRECTION_NEGATIVE;
            float speed_steps_per_sec = (float)(requested_speed > 0 ? requested_speed : -requested_speed);

            if (s_app.snapshot.state == APP_STATE_MANUAL_JOG && motion_axis_is_busy()) {
                s_app.pending_jog_valid = true;
                s_app.pending_jog_speed_steps_per_sec = requested_speed;
                motion_axis_request_stop();
                break;
            }

            if (motion_axis_start_continuous_move(direction, speed_steps_per_sec, s_app.config.motion.acceleration_steps_per_sec2) == ESP_OK) {
                app_controller_set_state(APP_STATE_MANUAL_JOG, APP_ACTIVE_MODE_MANUAL);
            } else {
                app_controller_record_error("Unable to start manual jog motion.");
            }
            break;
        }
        case APP_COMMAND_MOVE: {
            int32_t current_position_steps = motion_axis_get_current_position_steps();
            int32_t target_position_steps = command->data.move.use_steps
                ? command->data.move.distance_or_target_steps
                : app_config_mm_to_steps(&s_app.config, command->data.move.distance_or_target_mm);

            if (command->data.move.mode == APP_MOVE_MODE_RELATIVE) {
                target_position_steps += current_position_steps;
            } else if (!app_controller_is_homed()) {
                app_controller_record_error("Absolute moves require a homed slider.");
                break;
            }

            if (app_controller_start_position_move(
                    target_position_steps,
                    command->data.move.custom_speed_steps_per_sec,
                    command->data.move.has_custom_speed,
                    APP_ACTIVE_MODE_NONE) != ESP_OK) {
                app_controller_record_error("Unable to start the requested move.");
            }
            break;
        }
        case APP_COMMAND_SET_A:
            if (!app_controller_is_homed()) {
                app_controller_record_error("Point A can only be captured after homing.");
                break;
            }
            s_app.snapshot.ab_points.has_a = true;
            s_app.snapshot.ab_points.a_position_steps = motion_axis_get_current_position_steps();
            break;
        case APP_COMMAND_SET_B:
            if (!app_controller_is_homed()) {
                app_controller_record_error("Point B can only be captured after homing.");
                break;
            }
            s_app.snapshot.ab_points.has_b = true;
            s_app.snapshot.ab_points.b_position_steps = motion_axis_get_current_position_steps();
            break;
        case APP_COMMAND_RUN_AB: {
            if (!app_controller_is_homed() ||
                !s_app.snapshot.ab_points.has_a ||
                !s_app.snapshot.ab_points.has_b) {
                app_controller_record_error("A/B runs require a homed system and both A and B points.");
                break;
            }

            s_app.ab.active = true;
            s_app.ab.repeat_mode = command->data.run_ab.repeat_mode;
            s_app.config.axis_behavior.repeat_mode = command->data.run_ab.repeat_mode;

            int32_t current_position_steps = motion_axis_get_current_position_steps();
            if (current_position_steps != s_app.snapshot.ab_points.a_position_steps) {
                s_app.ab.prepositioning_to_a = true;
                if (motion_axis_start_profiled_move(
                        s_app.snapshot.ab_points.a_position_steps,
                        s_app.config.motion.max_speed_steps_per_sec,
                        s_app.config.motion.acceleration_steps_per_sec2) == ESP_OK) {
                    app_controller_set_state(APP_STATE_RUNNING_PROGRAM, APP_ACTIVE_MODE_AB);
                } else {
                    s_app.ab.active = false;
                    app_controller_record_error("Unable to pre-position to point A.");
                }
            } else {
                app_controller_start_ab_leg(true);
            }
            break;
        }
        case APP_COMMAND_SET_PROGRAM:
            if (command->data.program.count < 2 || command->data.program.count > APP_MAX_KEYFRAMES) {
                app_controller_record_error("Keyframe programs must contain between 2 and 16 waypoints.");
                break;
            }

            if (command->data.program.keyframes[0].time_ms != 0) {
                app_controller_record_error("The first keyframe time must be 0 ms.");
                break;
            }

            for (size_t i = 1; i < command->data.program.count; ++i) {
                if (command->data.program.keyframes[i].time_ms <= command->data.program.keyframes[i - 1].time_ms) {
                    app_controller_record_error("Keyframe times must be strictly increasing.");
                    return;
                }

                int32_t segment_distance_steps = command->data.program.keyframes[i].position_steps - command->data.program.keyframes[i - 1].position_steps;
                uint32_t segment_duration_ms = command->data.program.keyframes[i].time_ms - command->data.program.keyframes[i - 1].time_ms;
                if (!motion_axis_plan_exact_move(
                        segment_distance_steps,
                        (float)segment_duration_ms / 1000.0f,
                        s_app.config.motion.max_speed_steps_per_sec,
                        s_app.config.motion.acceleration_steps_per_sec2,
                        &(float){0},
                        &(float){0})) {
                    app_controller_record_error("At least one keyframe segment is infeasible with the current motion limits.");
                    return;
                }
            }

            s_app.program.program = command->data.program;
            s_app.program.loaded = true;
            break;
        case APP_COMMAND_RUN_PROGRAM:
            if (!app_controller_is_homed()) {
                app_controller_record_error("Keyframe runs require a homed slider.");
                break;
            }
            if (!s_app.program.loaded) {
                app_controller_record_error("No keyframe program has been uploaded.");
                break;
            }

            s_app.program.active = true;
            s_app.program.paused = false;
            s_app.program.prepositioning = false;
            s_app.program.loop_repositioning = false;
            s_app.program.pause_requested = false;
            s_app.program.repeat_mode = command->data.run_program.repeat_mode;
            s_app.program.reverse_direction = false;
            s_app.config.axis_behavior.repeat_mode = command->data.run_program.repeat_mode;

            if (motion_axis_get_current_position_steps() != s_app.program.program.keyframes[0].position_steps) {
                if (motion_axis_start_profiled_move(
                        s_app.program.program.keyframes[0].position_steps,
                        s_app.config.motion.max_speed_steps_per_sec,
                        s_app.config.motion.acceleration_steps_per_sec2) == ESP_OK) {
                    s_app.program.prepositioning = true;
                    app_controller_set_state(APP_STATE_RUNNING_PROGRAM, APP_ACTIVE_MODE_KEYFRAME);
                } else {
                    s_app.program.active = false;
                    app_controller_record_error("Unable to pre-position to the first keyframe.");
                }
            } else {
                app_controller_start_program_sequence();
            }
            break;
        case APP_COMMAND_PAUSE_PROGRAM:
            if (!s_app.program.active || s_app.program.paused || s_app.program.prepositioning || s_app.program.loop_repositioning) {
                app_controller_record_error("Pause is only available during an active timed keyframe segment.");
                break;
            }
            s_app.program.pause_requested = true;
            s_app.program.paused_target_position_steps = s_app.program.program.keyframes[s_app.program.current_to_index].position_steps;
            s_app.program.paused_remaining_duration_ms = s_app.program.current_segment_duration_ms;
            {
                uint64_t elapsed_ms = app_controller_now_ms() - s_app.program.current_segment_started_ms;
                if (elapsed_ms < s_app.program.current_segment_duration_ms) {
                    s_app.program.paused_remaining_duration_ms = (uint32_t)(s_app.program.current_segment_duration_ms - elapsed_ms);
                } else {
                    s_app.program.paused_remaining_duration_ms = 100;
                }
            }
            motion_axis_request_stop();
            break;
        case APP_COMMAND_RESUME_PROGRAM:
            if (!s_app.program.active || !s_app.program.paused) {
                app_controller_record_error("There is no paused keyframe program to resume.");
                break;
            }
            s_app.program.paused = false;
            if (motion_axis_start_timed_move(
                    s_app.program.paused_target_position_steps,
                    (float)s_app.program.paused_remaining_duration_ms / 1000.0f,
                    s_app.config.motion.max_speed_steps_per_sec,
                    s_app.config.motion.acceleration_steps_per_sec2) == ESP_OK) {
                s_app.program.current_segment_duration_ms = s_app.program.paused_remaining_duration_ms;
                s_app.program.current_segment_started_ms = app_controller_now_ms();
                app_controller_set_state(APP_STATE_RUNNING_PROGRAM, APP_ACTIVE_MODE_KEYFRAME);
            } else {
                s_app.program.paused = true;
                app_controller_record_error("Unable to resume the paused keyframe segment.");
            }
            break;
        case APP_COMMAND_UPDATE_CONFIG:
            app_controller_apply_config_patch(&command->data.config_patch);
            break;
        default:
            break;
    }
}

static void app_controller_handle_motion_event(const motion_event_t *event)
{
    if (event == NULL) {
        return;
    }

    switch (event->type) {
        case MOTION_EVENT_MOVE_COMPLETE:
            app_controller_handle_move_complete();
            break;
        case MOTION_EVENT_STOPPED:
            app_controller_handle_stopped();
            break;
        case MOTION_EVENT_LIMIT_LEFT:
        case MOTION_EVENT_LIMIT_RIGHT:
            app_controller_handle_limit_event(event);
            break;
        case MOTION_EVENT_FAULT:
            app_controller_record_error("Motion subsystem reported a fault.");
            app_controller_set_state(APP_STATE_ERROR, APP_ACTIVE_MODE_NONE);
            break;
        default:
            break;
    }
}

static void app_controller_task(void *arg)
{
    (void)arg;

    while (true) {
        app_command_t command = {};
        if (xQueueReceive(s_app.command_queue, &command, pdMS_TO_TICKS(APP_CONTROL_TICK_MS)) == pdTRUE) {
            app_controller_handle_command(&command);
        }

        motion_axis_service();

        motion_event_t event = {};
        while (motion_axis_poll_event(&event)) {
            app_controller_handle_motion_event(&event);
        }

        app_controller_refresh_snapshot();
    }
}

esp_err_t app_controller_start(void)
{
    if (s_app.started) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_app, 0, sizeof(s_app));

    s_app.command_queue = xQueueCreate(APP_COMMAND_QUEUE_LENGTH, sizeof(app_command_t));
    if (s_app.command_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_app.lock = xSemaphoreCreateMutex();
    if (s_app.lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    app_config_load_defaults(&s_app.config);
    char validation_error[APP_ERROR_MESSAGE_MAX];
    if (!app_config_validate(&s_app.config, validation_error, sizeof(validation_error))) {
        ESP_LOGE(TAG, "%s", validation_error);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(motion_axis_init(&APP_AXIS_PINS, &s_app.config), TAG, "Failed to initialize the motion axis");

    memset(&s_app.snapshot, 0, sizeof(s_app.snapshot));
    snprintf(s_app.snapshot.firmware_version, sizeof(s_app.snapshot.firmware_version), "%s", APP_FIRMWARE_VERSION);
    s_app.snapshot.homed = false;
    app_controller_set_state(APP_STATE_IDLE, APP_ACTIVE_MODE_NONE);
    s_app.snapshot.max_position_steps = app_controller_max_position_steps(&s_app.config);

    if (xTaskCreate(app_controller_task, "app_ctrl", 8192, NULL, 5, &s_app.task_handle) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_app.started = true;
    return ESP_OK;
}

esp_err_t app_controller_submit_command(const app_command_t *command, uint32_t timeout_ms)
{
    if (!s_app.started || command == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if (xQueueSend(s_app.command_queue, command, timeout_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

void app_controller_get_status_snapshot(app_status_snapshot_t *snapshot_out)
{
    if (snapshot_out == NULL) {
        return;
    }

    xSemaphoreTake(s_app.lock, portMAX_DELAY);
    *snapshot_out = s_app.snapshot;
    xSemaphoreGive(s_app.lock);
}

void app_controller_get_runtime_config(app_runtime_config_t *config_out)
{
    if (config_out == NULL) {
        return;
    }

    xSemaphoreTake(s_app.lock, portMAX_DELAY);
    *config_out = s_app.config;
    xSemaphoreGive(s_app.lock);
}
