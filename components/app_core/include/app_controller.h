#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "app_models.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t app_controller_start(void);
esp_err_t app_controller_submit_command(const app_command_t *command, uint32_t timeout_ms);
void app_controller_get_status_snapshot(app_status_snapshot_t *snapshot_out);
void app_controller_get_runtime_config(app_runtime_config_t *config_out);

#ifdef __cplusplus
}
#endif
