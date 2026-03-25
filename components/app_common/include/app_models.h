#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APP_MAX_KEYFRAMES 16
#define APP_MAX_RECENT_ERRORS 8
#define APP_ERROR_MESSAGE_MAX 96
#define APP_FIRMWARE_VERSION "0.1.0"

typedef enum {
    APP_STATE_BOOT = 0,
    APP_STATE_IDLE,
    APP_STATE_HOMING,
    APP_STATE_READY,
    APP_STATE_MANUAL_JOG,
    APP_STATE_RUNNING_PROGRAM,
    APP_STATE_PAUSED,
    APP_STATE_ERROR,
    APP_STATE_LIMIT_TRIGGERED,
} app_state_t;

typedef enum {
    APP_ACTIVE_MODE_NONE = 0,
    APP_ACTIVE_MODE_MANUAL,
    APP_ACTIVE_MODE_AB,
    APP_ACTIVE_MODE_KEYFRAME,
    APP_ACTIVE_MODE_HOMING,
} app_active_mode_t;

typedef enum {
    APP_AXIS_SLIDER = 0,
} app_axis_id_t;

typedef enum {
    APP_REPEAT_MODE_NONE = 0,
    APP_REPEAT_MODE_LOOP,
    APP_REPEAT_MODE_PING_PONG,
} app_repeat_mode_t;

typedef enum {
    APP_HOME_REFERENCE_LEFT = 0,
    APP_HOME_REFERENCE_RIGHT,
} app_home_reference_side_t;

typedef enum {
    APP_MOTION_PROFILE_TRAPEZOIDAL = 0,
} app_motion_profile_t;

typedef enum {
    APP_MOVE_MODE_ABSOLUTE = 0,
    APP_MOVE_MODE_RELATIVE,
} app_move_mode_t;

typedef enum {
    APP_COMMAND_HOME = 0,
    APP_COMMAND_STOP,
    APP_COMMAND_JOG,
    APP_COMMAND_MOVE,
    APP_COMMAND_SET_A,
    APP_COMMAND_SET_B,
    APP_COMMAND_RUN_AB,
    APP_COMMAND_SET_PROGRAM,
    APP_COMMAND_RUN_PROGRAM,
    APP_COMMAND_PAUSE_PROGRAM,
    APP_COMMAND_RESUME_PROGRAM,
    APP_COMMAND_STOP_PROGRAM,
    APP_COMMAND_UPDATE_CONFIG,
} app_command_type_t;

typedef struct {
    int32_t position_steps;
    uint32_t time_ms;
} app_keyframe_t;

typedef struct {
    app_axis_id_t axis_id;
    size_t count;
    app_keyframe_t keyframes[APP_MAX_KEYFRAMES];
} app_keyframe_program_t;

typedef struct {
    bool has_a;
    bool has_b;
    int32_t a_position_steps;
    int32_t b_position_steps;
} app_ab_points_t;

typedef struct {
    uint64_t uptime_ms;
    char message[APP_ERROR_MESSAGE_MAX];
} app_recent_error_t;

typedef struct {
    char ssid[32];
    char password[64];
    uint8_t channel;
    uint8_t max_connections;
    char ip_address[16];
} app_wifi_ap_config_t;

typedef struct {
    int32_t steps_per_revolution;
    int32_t microsteps;
    int32_t effective_steps_per_revolution;
    float belt_pitch_mm;
    int32_t pulley_teeth;
    float travel_mm_per_revolution;
    float steps_per_mm;
    float slider_travel_mm;
} app_mechanics_config_t;

typedef struct {
    float max_speed_steps_per_sec;
    float acceleration_steps_per_sec2;
    float jog_speed_steps_per_sec;
    app_motion_profile_t motion_profile;
} app_motion_config_t;

typedef struct {
    float homing_speed_steps_per_sec;
    float homing_slow_speed_steps_per_sec;
    int32_t homing_backoff_steps;
    uint32_t limit_debounce_ms;
    app_home_reference_side_t home_reference_side;
} app_homing_config_t;

typedef struct {
    app_axis_id_t axis_id;
    bool invert_direction_output;
    app_repeat_mode_t repeat_mode;
} app_axis_behavior_config_t;

typedef struct {
    app_wifi_ap_config_t wifi_ap;
    app_mechanics_config_t mechanics;
    app_motion_config_t motion;
    app_homing_config_t homing;
    app_axis_behavior_config_t axis_behavior;
} app_runtime_config_t;

typedef struct {
    bool left_raw_pressed;
    bool right_raw_pressed;
    bool left_pressed;
    bool right_pressed;
} app_limit_status_t;

typedef struct {
    app_state_t state;
    app_active_mode_t active_mode;
    app_repeat_mode_t repeat_mode;
    bool homed;
    bool moving;
    bool pause_requested;
    int32_t position_steps;
    float position_mm;
    int32_t min_position_steps;
    int32_t max_position_steps;
    app_limit_status_t limits;
    app_ab_points_t ab_points;
    size_t loaded_keyframe_count;
    char firmware_version[16];
    char last_error[APP_ERROR_MESSAGE_MAX];
    size_t recent_error_count;
    app_recent_error_t recent_errors[APP_MAX_RECENT_ERRORS];
    uint32_t free_heap_bytes;
    uint64_t uptime_ms;
} app_status_snapshot_t;

typedef struct {
    int32_t speed_steps_per_sec;
} app_jog_command_t;

typedef struct {
    app_move_mode_t mode;
    bool use_steps;
    int32_t distance_or_target_steps;
    float distance_or_target_mm;
    bool has_custom_speed;
    int32_t custom_speed_steps_per_sec;
} app_move_command_t;

typedef struct {
    app_repeat_mode_t repeat_mode;
} app_run_ab_command_t;

typedef struct {
    app_repeat_mode_t repeat_mode;
} app_run_program_command_t;

typedef enum {
    APP_CONFIG_PATCH_MAX_SPEED = UINT64_C(1) << 0,
    APP_CONFIG_PATCH_ACCELERATION = UINT64_C(1) << 1,
    APP_CONFIG_PATCH_JOG_SPEED = UINT64_C(1) << 2,
    APP_CONFIG_PATCH_HOMING_SPEED = UINT64_C(1) << 3,
    APP_CONFIG_PATCH_HOMING_SLOW_SPEED = UINT64_C(1) << 4,
    APP_CONFIG_PATCH_HOMING_BACKOFF = UINT64_C(1) << 5,
    APP_CONFIG_PATCH_LIMIT_DEBOUNCE = UINT64_C(1) << 6,
    APP_CONFIG_PATCH_STEPS_PER_REV = UINT64_C(1) << 7,
    APP_CONFIG_PATCH_MICROSTEPS = UINT64_C(1) << 8,
    APP_CONFIG_PATCH_BELT_PITCH = UINT64_C(1) << 9,
    APP_CONFIG_PATCH_PULLEY_TEETH = UINT64_C(1) << 10,
    APP_CONFIG_PATCH_STEPS_PER_MM = UINT64_C(1) << 11,
    APP_CONFIG_PATCH_SLIDER_TRAVEL = UINT64_C(1) << 12,
    APP_CONFIG_PATCH_HOME_SIDE = UINT64_C(1) << 13,
    APP_CONFIG_PATCH_INVERT_DIRECTION = UINT64_C(1) << 14,
    APP_CONFIG_PATCH_REPEAT_MODE = UINT64_C(1) << 15,
    APP_CONFIG_PATCH_MOTION_PROFILE = UINT64_C(1) << 16,
} app_config_patch_mask_t;

typedef struct {
    uint64_t mask;
    app_runtime_config_t values;
} app_config_patch_t;

typedef struct {
    app_command_type_t type;
    union {
        app_jog_command_t jog;
        app_move_command_t move;
        app_run_ab_command_t run_ab;
        app_keyframe_program_t program;
        app_run_program_command_t run_program;
        app_config_patch_t config_patch;
    } data;
} app_command_t;
