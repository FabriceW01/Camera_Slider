#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "app_models.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*web_submit_command_fn)(const app_command_t *command, uint32_t timeout_ms);
typedef void (*web_get_status_fn)(app_status_snapshot_t *snapshot_out);
typedef void (*web_get_config_fn)(app_runtime_config_t *config_out);

typedef struct {
    web_submit_command_fn submit_command;
    web_get_status_fn get_status;
    web_get_config_fn get_config;
} web_service_bindings_t;

esp_err_t web_service_start(const web_service_bindings_t *bindings);

#ifdef __cplusplus
}
#endif
