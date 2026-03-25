#include "app_config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static bool app_strings_equal(const char *lhs, const char *rhs)
{
    return lhs != NULL && rhs != NULL && strcmp(lhs, rhs) == 0;
}

void app_config_load_defaults(app_runtime_config_t *config)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));

    snprintf(config->wifi_ap.ssid, sizeof(config->wifi_ap.ssid), "KameraSlider");
    snprintf(config->wifi_ap.password, sizeof(config->wifi_ap.password), "slider-setup");
    snprintf(config->wifi_ap.ip_address, sizeof(config->wifi_ap.ip_address), "192.168.4.1");
    config->wifi_ap.channel = 6;
    config->wifi_ap.max_connections = 4;

    config->mechanics.steps_per_revolution = 200;
    config->mechanics.microsteps = 32;
    config->mechanics.effective_steps_per_revolution = 6400;
    config->mechanics.belt_pitch_mm = 2.0f;
    config->mechanics.pulley_teeth = 20;
    config->mechanics.travel_mm_per_revolution = 40.0f;
    config->mechanics.steps_per_mm = 160.0f;
    config->mechanics.slider_travel_mm = 1000.0f;

    config->motion.max_speed_steps_per_sec = 4000.0f;
    config->motion.acceleration_steps_per_sec2 = 1000.0f;
    config->motion.jog_speed_steps_per_sec = 2400.0f;
    config->motion.motion_profile = APP_MOTION_PROFILE_TRAPEZOIDAL;

    config->homing.homing_speed_steps_per_sec = 1200.0f;
    config->homing.homing_slow_speed_steps_per_sec = 400.0f;
    config->homing.homing_backoff_steps = 640;
    config->homing.limit_debounce_ms = 15;
    config->homing.home_reference_side = APP_HOME_REFERENCE_LEFT;

    config->axis_behavior.axis_id = APP_AXIS_SLIDER;
    config->axis_behavior.invert_direction_output = false;
    config->axis_behavior.repeat_mode = APP_REPEAT_MODE_NONE;
}

bool app_config_validate(const app_runtime_config_t *config, char *error_buffer, size_t error_buffer_size)
{
    if (config == NULL) {
        snprintf(error_buffer, error_buffer_size, "Configuration is missing.");
        return false;
    }

    if (config->wifi_ap.ssid[0] == '\0') {
        snprintf(error_buffer, error_buffer_size, "AP SSID must not be empty.");
        return false;
    }

    if (strlen(config->wifi_ap.password) < 8) {
        snprintf(error_buffer, error_buffer_size, "AP password must be at least 8 characters.");
        return false;
    }

    if (config->mechanics.steps_per_revolution <= 0 ||
        config->mechanics.microsteps <= 0 ||
        config->mechanics.belt_pitch_mm <= 0.0f ||
        config->mechanics.pulley_teeth <= 0 ||
        config->mechanics.steps_per_mm <= 0.0f ||
        config->mechanics.slider_travel_mm <= 0.0f) {
        snprintf(error_buffer, error_buffer_size, "Mechanics values must be positive.");
        return false;
    }

    if (config->motion.max_speed_steps_per_sec <= 0.0f ||
        config->motion.acceleration_steps_per_sec2 <= 0.0f ||
        config->motion.jog_speed_steps_per_sec <= 0.0f) {
        snprintf(error_buffer, error_buffer_size, "Motion speeds and acceleration must be positive.");
        return false;
    }

    if (config->homing.homing_speed_steps_per_sec <= 0.0f ||
        config->homing.homing_slow_speed_steps_per_sec <= 0.0f ||
        config->homing.homing_backoff_steps <= 0) {
        snprintf(error_buffer, error_buffer_size, "Homing parameters must be positive.");
        return false;
    }

    if (config->homing.limit_debounce_ms == 0 || config->homing.limit_debounce_ms > 250) {
        snprintf(error_buffer, error_buffer_size, "Limit debounce must be between 1 and 250 ms.");
        return false;
    }

    return true;
}

float app_config_steps_to_mm(const app_runtime_config_t *config, int32_t steps)
{
    if (config == NULL || config->mechanics.steps_per_mm <= 0.0f) {
        return 0.0f;
    }

    return (float)steps / config->mechanics.steps_per_mm;
}

int32_t app_config_mm_to_steps(const app_runtime_config_t *config, float millimeters)
{
    if (config == NULL || config->mechanics.steps_per_mm <= 0.0f) {
        return 0;
    }

    return (int32_t)lroundf(millimeters * config->mechanics.steps_per_mm);
}

const char *app_state_to_string(app_state_t state)
{
    switch (state) {
        case APP_STATE_BOOT:
            return "BOOT";
        case APP_STATE_IDLE:
            return "IDLE";
        case APP_STATE_HOMING:
            return "HOMING";
        case APP_STATE_READY:
            return "READY";
        case APP_STATE_MANUAL_JOG:
            return "MANUAL_JOG";
        case APP_STATE_RUNNING_PROGRAM:
            return "RUNNING_PROGRAM";
        case APP_STATE_PAUSED:
            return "PAUSED";
        case APP_STATE_ERROR:
            return "ERROR";
        case APP_STATE_LIMIT_TRIGGERED:
            return "LIMIT_TRIGGERED";
        default:
            return "UNKNOWN";
    }
}

const char *app_active_mode_to_string(app_active_mode_t mode)
{
    switch (mode) {
        case APP_ACTIVE_MODE_NONE:
            return "NONE";
        case APP_ACTIVE_MODE_MANUAL:
            return "MANUAL";
        case APP_ACTIVE_MODE_AB:
            return "AB";
        case APP_ACTIVE_MODE_KEYFRAME:
            return "KEYFRAME";
        case APP_ACTIVE_MODE_HOMING:
            return "HOMING";
        default:
            return "UNKNOWN";
    }
}

const char *app_repeat_mode_to_string(app_repeat_mode_t mode)
{
    switch (mode) {
        case APP_REPEAT_MODE_NONE:
            return "none";
        case APP_REPEAT_MODE_LOOP:
            return "loop";
        case APP_REPEAT_MODE_PING_PONG:
            return "ping_pong";
        default:
            return "unknown";
    }
}

const char *app_home_reference_to_string(app_home_reference_side_t side)
{
    switch (side) {
        case APP_HOME_REFERENCE_LEFT:
            return "left";
        case APP_HOME_REFERENCE_RIGHT:
            return "right";
        default:
            return "unknown";
    }
}

const char *app_motion_profile_to_string(app_motion_profile_t profile)
{
    switch (profile) {
        case APP_MOTION_PROFILE_TRAPEZOIDAL:
            return "trapezoidal";
        default:
            return "unknown";
    }
}

bool app_parse_repeat_mode(const char *value, app_repeat_mode_t *out_mode)
{
    if (value == NULL || out_mode == NULL) {
        return false;
    }

    if (app_strings_equal(value, "none")) {
        *out_mode = APP_REPEAT_MODE_NONE;
        return true;
    }

    if (app_strings_equal(value, "loop")) {
        *out_mode = APP_REPEAT_MODE_LOOP;
        return true;
    }

    if (app_strings_equal(value, "ping_pong")) {
        *out_mode = APP_REPEAT_MODE_PING_PONG;
        return true;
    }

    return false;
}

bool app_parse_home_reference_side(const char *value, app_home_reference_side_t *out_side)
{
    if (value == NULL || out_side == NULL) {
        return false;
    }

    if (app_strings_equal(value, "left")) {
        *out_side = APP_HOME_REFERENCE_LEFT;
        return true;
    }

    if (app_strings_equal(value, "right")) {
        *out_side = APP_HOME_REFERENCE_RIGHT;
        return true;
    }

    return false;
}

bool app_parse_motion_profile(const char *value, app_motion_profile_t *out_profile)
{
    if (value == NULL || out_profile == NULL) {
        return false;
    }

    if (app_strings_equal(value, "trapezoidal")) {
        *out_profile = APP_MOTION_PROFILE_TRAPEZOIDAL;
        return true;
    }

    return false;
}
