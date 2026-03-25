#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_models.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_config_load_defaults(app_runtime_config_t *config);
bool app_config_validate(const app_runtime_config_t *config, char *error_buffer, size_t error_buffer_size);
float app_config_steps_to_mm(const app_runtime_config_t *config, int32_t steps);
int32_t app_config_mm_to_steps(const app_runtime_config_t *config, float millimeters);
const char *app_state_to_string(app_state_t state);
const char *app_active_mode_to_string(app_active_mode_t mode);
const char *app_repeat_mode_to_string(app_repeat_mode_t mode);
const char *app_home_reference_to_string(app_home_reference_side_t side);
const char *app_motion_profile_to_string(app_motion_profile_t profile);
bool app_parse_repeat_mode(const char *value, app_repeat_mode_t *out_mode);
bool app_parse_home_reference_side(const char *value, app_home_reference_side_t *out_side);
bool app_parse_motion_profile(const char *value, app_motion_profile_t *out_profile);

#ifdef __cplusplus
}
#endif
