#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#include "app_models.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTION_DIRECTION_NEGATIVE = -1,
    MOTION_DIRECTION_POSITIVE = 1,
} motion_direction_t;

typedef enum {
    MOTION_EVENT_NONE = 0,
    MOTION_EVENT_MOVE_COMPLETE,
    MOTION_EVENT_STOPPED,
    MOTION_EVENT_LIMIT_LEFT,
    MOTION_EVENT_LIMIT_RIGHT,
    MOTION_EVENT_FAULT,
} motion_event_type_t;

typedef struct {
    motion_event_type_t type;
    int32_t position_steps;
    app_limit_status_t limits;
} motion_event_t;

typedef struct {
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
    gpio_num_t limit_left_pin;
    gpio_num_t limit_right_pin;
} motion_axis_pins_t;

typedef struct {
    int32_t current_position_steps;
    int32_t target_position_steps;
    float current_speed_steps_per_sec;
    float cruise_speed_steps_per_sec;
    float acceleration_steps_per_sec2;
    bool moving;
    bool continuous;
    bool stop_requested;
    motion_direction_t direction;
    app_limit_status_t limits;
} motion_axis_status_t;

esp_err_t motion_axis_init(const motion_axis_pins_t *pins, const app_runtime_config_t *config);
esp_err_t motion_axis_apply_runtime_config(const app_runtime_config_t *config);
esp_err_t motion_axis_start_profiled_move(int32_t target_position_steps, float cruise_speed_steps_per_sec, float acceleration_steps_per_sec2);
esp_err_t motion_axis_start_timed_move(int32_t target_position_steps, float duration_seconds, float max_speed_steps_per_sec, float max_acceleration_steps_per_sec2);
esp_err_t motion_axis_start_continuous_move(motion_direction_t direction, float cruise_speed_steps_per_sec, float acceleration_steps_per_sec2);
esp_err_t motion_axis_request_stop(void);
esp_err_t motion_axis_emergency_stop(void);
esp_err_t motion_axis_set_current_position(int32_t position_steps);
int32_t motion_axis_get_current_position_steps(void);
bool motion_axis_is_busy(void);
void motion_axis_service(void);
bool motion_axis_poll_event(motion_event_t *event_out);
void motion_axis_get_status(motion_axis_status_t *status_out);
float motion_axis_compute_min_time_s(int32_t distance_steps, float max_speed_steps_per_sec, float max_acceleration_steps_per_sec2);
bool motion_axis_plan_exact_move(int32_t distance_steps, float duration_seconds, float max_speed_steps_per_sec, float max_acceleration_steps_per_sec2, float *planned_cruise_speed_steps_per_sec, float *planned_acceleration_steps_per_sec2);

#ifdef __cplusplus
}
#endif
