#include "web_service.h"

#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/ip4_addr.h"

#include "app_config.h"
#include "web_assets.h"

typedef struct {
    bool started;
    httpd_handle_t server;
    esp_netif_t *ap_netif;
    web_service_bindings_t bindings;
} web_service_ctx_t;

static const char *TAG = "web_service";
static const uint32_t WEB_COMMAND_TIMEOUT_MS = 100;
static web_service_ctx_t s_web = {};

static esp_err_t web_send_json_response(httpd_req_t *req, cJSON *payload)
{
    char *body = cJSON_PrintUnformatted(payload);
    if (body == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to render JSON.");
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t result = httpd_resp_sendstr(req, body);
    cJSON_free(body);
    return result;
}

static esp_err_t web_send_ok(httpd_req_t *req, cJSON *data)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    if (data == NULL) {
        data = cJSON_CreateObject();
    }
    cJSON_AddItemToObject(root, "data", data);
    esp_err_t result = web_send_json_response(req, root);
    cJSON_Delete(root);
    return result;
}

static esp_err_t web_send_error(httpd_req_t *req, const char *error_code, const char *message, const char *status_text)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *error = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", false);
    cJSON_AddStringToObject(error, "code", error_code);
    cJSON_AddStringToObject(error, "message", message);
    cJSON_AddItemToObject(root, "error", error);
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (body == NULL) {
        httpd_resp_set_status(req, status_text);
        return httpd_resp_sendstr(req, message);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, status_text);
    esp_err_t result = httpd_resp_sendstr(req, body);
    cJSON_free(body);
    return result;
}

static esp_err_t web_send_embedded_text(httpd_req_t *req, const char *content_type, const uint8_t *data, size_t length)
{
    httpd_resp_set_type(req, content_type);
    return httpd_resp_send(req, (const char *)data, (ssize_t)length);
}

static esp_err_t web_read_request_body(httpd_req_t *req, char **buffer_out)
{
    if (buffer_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *buffer_out = NULL;
    if (req->content_len == 0) {
        return ESP_OK;
    }

    if (req->content_len > 8192) {
        return ESP_ERR_INVALID_SIZE;
    }

    char *buffer = calloc(1, (size_t)req->content_len + 1);
    if (buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    int received_total = 0;
    while (received_total < req->content_len) {
        int received = httpd_req_recv(req, buffer + received_total, req->content_len - received_total);
        if (received <= 0) {
            free(buffer);
            return ESP_FAIL;
        }
        received_total += received;
    }

    *buffer_out = buffer;
    return ESP_OK;
}

static cJSON *web_parse_json_body(httpd_req_t *req)
{
    char *buffer = NULL;
    esp_err_t result = web_read_request_body(req, &buffer);
    if (result != ESP_OK) {
        return NULL;
    }

    if (buffer == NULL) {
        return cJSON_CreateObject();
    }

    cJSON *root = cJSON_Parse(buffer);
    free(buffer);
    return root;
}

static esp_err_t web_submit_command_and_ack(httpd_req_t *req, const app_command_t *command)
{
    esp_err_t result = s_web.bindings.submit_command(command, WEB_COMMAND_TIMEOUT_MS);
    if (result == ESP_OK) {
        return web_send_ok(req, NULL);
    }

    return web_send_error(
        req,
        "command_rejected",
        result == ESP_ERR_TIMEOUT ? "Controller queue is busy." : "Controller rejected the command.",
        result == ESP_ERR_TIMEOUT ? "503 Service Unavailable" : "409 Conflict");
}

static cJSON *web_status_to_json(const app_status_snapshot_t *snapshot)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *limits = cJSON_CreateObject();
    cJSON *ab = cJSON_CreateObject();
    cJSON *bounds = cJSON_CreateObject();
    cJSON *recent_errors = cJSON_CreateArray();

    cJSON_AddStringToObject(root, "state", app_state_to_string(snapshot->state));
    cJSON_AddStringToObject(root, "active_mode", app_active_mode_to_string(snapshot->active_mode));
    cJSON_AddStringToObject(root, "repeat_mode", app_repeat_mode_to_string(snapshot->repeat_mode));
    cJSON_AddBoolToObject(root, "homed", snapshot->homed);
    cJSON_AddBoolToObject(root, "moving", snapshot->moving);
    cJSON_AddNumberToObject(root, "position_steps", snapshot->position_steps);
    cJSON_AddNumberToObject(root, "position_mm", snapshot->position_mm);
    cJSON_AddStringToObject(root, "firmware_version", snapshot->firmware_version);
    cJSON_AddStringToObject(root, "last_error", snapshot->last_error);
    cJSON_AddNumberToObject(root, "free_heap_bytes", snapshot->free_heap_bytes);
    cJSON_AddNumberToObject(root, "uptime_ms", (double)snapshot->uptime_ms);
    cJSON_AddNumberToObject(root, "loaded_keyframe_count", (double)snapshot->loaded_keyframe_count);

    cJSON_AddNumberToObject(bounds, "min_steps", snapshot->min_position_steps);
    cJSON_AddNumberToObject(bounds, "max_steps", snapshot->max_position_steps);
    cJSON_AddItemToObject(root, "soft_bounds", bounds);

    cJSON_AddBoolToObject(limits, "left_raw_pressed", snapshot->limits.left_raw_pressed);
    cJSON_AddBoolToObject(limits, "right_raw_pressed", snapshot->limits.right_raw_pressed);
    cJSON_AddBoolToObject(limits, "left_pressed", snapshot->limits.left_pressed);
    cJSON_AddBoolToObject(limits, "right_pressed", snapshot->limits.right_pressed);
    cJSON_AddItemToObject(root, "limits", limits);

    cJSON_AddBoolToObject(ab, "has_a", snapshot->ab_points.has_a);
    cJSON_AddBoolToObject(ab, "has_b", snapshot->ab_points.has_b);
    cJSON_AddNumberToObject(ab, "a_position_steps", snapshot->ab_points.a_position_steps);
    cJSON_AddNumberToObject(ab, "b_position_steps", snapshot->ab_points.b_position_steps);
    cJSON_AddItemToObject(root, "ab", ab);

    for (size_t i = 0; i < snapshot->recent_error_count; ++i) {
        cJSON *entry = cJSON_CreateObject();
        cJSON_AddNumberToObject(entry, "uptime_ms", (double)snapshot->recent_errors[i].uptime_ms);
        cJSON_AddStringToObject(entry, "message", snapshot->recent_errors[i].message);
        cJSON_AddItemToArray(recent_errors, entry);
    }
    cJSON_AddItemToObject(root, "recent_errors", recent_errors);

    return root;
}

static cJSON *web_config_to_json(const app_runtime_config_t *config)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *wifi_ap = cJSON_CreateObject();
    cJSON *mechanics = cJSON_CreateObject();
    cJSON *motion = cJSON_CreateObject();
    cJSON *homing = cJSON_CreateObject();
    cJSON *axis_behavior = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "axis_id", "slider");

    cJSON_AddStringToObject(wifi_ap, "ssid", config->wifi_ap.ssid);
    cJSON_AddStringToObject(wifi_ap, "password", config->wifi_ap.password);
    cJSON_AddNumberToObject(wifi_ap, "channel", config->wifi_ap.channel);
    cJSON_AddNumberToObject(wifi_ap, "max_connections", config->wifi_ap.max_connections);
    cJSON_AddStringToObject(wifi_ap, "ip_address", config->wifi_ap.ip_address);
    cJSON_AddBoolToObject(wifi_ap, "read_only", true);
    cJSON_AddItemToObject(root, "wifi_ap", wifi_ap);

    cJSON_AddNumberToObject(mechanics, "steps_per_revolution", config->mechanics.steps_per_revolution);
    cJSON_AddNumberToObject(mechanics, "microsteps", config->mechanics.microsteps);
    cJSON_AddNumberToObject(mechanics, "effective_steps_per_revolution", config->mechanics.effective_steps_per_revolution);
    cJSON_AddNumberToObject(mechanics, "belt_pitch_mm", config->mechanics.belt_pitch_mm);
    cJSON_AddNumberToObject(mechanics, "pulley_teeth", config->mechanics.pulley_teeth);
    cJSON_AddNumberToObject(mechanics, "travel_mm_per_revolution", config->mechanics.travel_mm_per_revolution);
    cJSON_AddNumberToObject(mechanics, "steps_per_mm", config->mechanics.steps_per_mm);
    cJSON_AddNumberToObject(mechanics, "slider_travel_mm", config->mechanics.slider_travel_mm);
    cJSON_AddItemToObject(root, "mechanics", mechanics);

    cJSON_AddNumberToObject(motion, "max_speed_steps_per_sec", config->motion.max_speed_steps_per_sec);
    cJSON_AddNumberToObject(motion, "acceleration_steps_per_sec2", config->motion.acceleration_steps_per_sec2);
    cJSON_AddNumberToObject(motion, "jog_speed_steps_per_sec", config->motion.jog_speed_steps_per_sec);
    cJSON_AddStringToObject(motion, "motion_profile", app_motion_profile_to_string(config->motion.motion_profile));
    cJSON_AddItemToObject(root, "motion", motion);

    cJSON_AddNumberToObject(homing, "homing_speed_steps_per_sec", config->homing.homing_speed_steps_per_sec);
    cJSON_AddNumberToObject(homing, "homing_slow_speed_steps_per_sec", config->homing.homing_slow_speed_steps_per_sec);
    cJSON_AddNumberToObject(homing, "homing_backoff_steps", config->homing.homing_backoff_steps);
    cJSON_AddNumberToObject(homing, "limit_debounce_ms", config->homing.limit_debounce_ms);
    cJSON_AddStringToObject(homing, "home_reference_side", app_home_reference_to_string(config->homing.home_reference_side));
    cJSON_AddItemToObject(root, "homing", homing);

    cJSON_AddBoolToObject(axis_behavior, "invert_direction_output", config->axis_behavior.invert_direction_output);
    cJSON_AddStringToObject(axis_behavior, "repeat_mode", app_repeat_mode_to_string(config->axis_behavior.repeat_mode));
    cJSON_AddItemToObject(root, "axis_behavior", axis_behavior);

    return root;
}

static esp_err_t web_root_handler(httpd_req_t *req)
{
    return web_send_embedded_text(req, "text/html; charset=utf-8", web_asset_index_html, web_asset_index_html_len);
}

static esp_err_t web_js_handler(httpd_req_t *req)
{
    return web_send_embedded_text(req, "application/javascript; charset=utf-8", web_asset_app_js, web_asset_app_js_len);
}

static esp_err_t web_css_handler(httpd_req_t *req)
{
    return web_send_embedded_text(req, "text/css; charset=utf-8", web_asset_styles_css, web_asset_styles_css_len);
}

static esp_err_t web_status_handler(httpd_req_t *req)
{
    app_status_snapshot_t snapshot = {};
    s_web.bindings.get_status(&snapshot);
    return web_send_ok(req, web_status_to_json(&snapshot));
}

static esp_err_t web_config_handler(httpd_req_t *req)
{
    app_runtime_config_t config = {};
    s_web.bindings.get_config(&config);
    return web_send_ok(req, web_config_to_json(&config));
}

static esp_err_t web_home_handler(httpd_req_t *req)
{
    app_command_t command = {.type = APP_COMMAND_HOME};
    return web_submit_command_and_ack(req, &command);
}

static esp_err_t web_stop_handler(httpd_req_t *req)
{
    app_command_t command = {.type = APP_COMMAND_STOP};
    return web_submit_command_and_ack(req, &command);
}

static esp_err_t web_set_a_handler(httpd_req_t *req)
{
    app_command_t command = {.type = APP_COMMAND_SET_A};
    return web_submit_command_and_ack(req, &command);
}

static esp_err_t web_set_b_handler(httpd_req_t *req)
{
    app_command_t command = {.type = APP_COMMAND_SET_B};
    return web_submit_command_and_ack(req, &command);
}

static esp_err_t web_jog_handler(httpd_req_t *req)
{
    cJSON *body = web_parse_json_body(req);
    if (body == NULL) {
        return web_send_error(req, "invalid_json", "Unable to parse jog request JSON.", "400 Bad Request");
    }

    app_command_t command = {.type = APP_COMMAND_JOG};
    cJSON *action = cJSON_GetObjectItemCaseSensitive(body, "action");
    cJSON *speed_steps = cJSON_GetObjectItemCaseSensitive(body, "speed_steps_per_sec");
    cJSON *speed_mm = cJSON_GetObjectItemCaseSensitive(body, "speed_mm_per_sec");

    if (cJSON_IsString(action) && action->valuestring != NULL && strcmp(action->valuestring, "stop") == 0) {
        command.data.jog.speed_steps_per_sec = 0;
    } else if (cJSON_IsNumber(speed_steps)) {
        command.data.jog.speed_steps_per_sec = speed_steps->valueint;
    } else if (cJSON_IsNumber(speed_mm)) {
        app_runtime_config_t config = {};
        s_web.bindings.get_config(&config);
        command.data.jog.speed_steps_per_sec = app_config_mm_to_steps(&config, (float)speed_mm->valuedouble);
    } else {
        cJSON_Delete(body);
        return web_send_error(req, "invalid_jog_request", "Provide action=stop or a signed speed value.", "400 Bad Request");
    }

    cJSON_Delete(body);
    return web_submit_command_and_ack(req, &command);
}

static esp_err_t web_move_handler(httpd_req_t *req)
{
    cJSON *body = web_parse_json_body(req);
    if (body == NULL) {
        return web_send_error(req, "invalid_json", "Unable to parse move request JSON.", "400 Bad Request");
    }

    app_command_t command = {.type = APP_COMMAND_MOVE};
    cJSON *mode = cJSON_GetObjectItemCaseSensitive(body, "mode");
    if (!cJSON_IsString(mode) || mode->valuestring == NULL) {
        cJSON_Delete(body);
        return web_send_error(req, "invalid_move_mode", "Move mode must be absolute or relative.", "400 Bad Request");
    }

    command.data.move.mode = strcmp(mode->valuestring, "relative") == 0 ? APP_MOVE_MODE_RELATIVE : APP_MOVE_MODE_ABSOLUTE;
    cJSON *steps = cJSON_GetObjectItemCaseSensitive(body, command.data.move.mode == APP_MOVE_MODE_ABSOLUTE ? "position_steps" : "distance_steps");
    cJSON *millimeters = cJSON_GetObjectItemCaseSensitive(body, command.data.move.mode == APP_MOVE_MODE_ABSOLUTE ? "position_mm" : "distance_mm");
    cJSON *custom_speed = cJSON_GetObjectItemCaseSensitive(body, "speed_steps_per_sec");

    if (cJSON_IsNumber(steps)) {
        command.data.move.use_steps = true;
        command.data.move.distance_or_target_steps = steps->valueint;
    } else if (cJSON_IsNumber(millimeters)) {
        command.data.move.use_steps = false;
        command.data.move.distance_or_target_mm = (float)millimeters->valuedouble;
    } else {
        cJSON_Delete(body);
        return web_send_error(req, "invalid_move_target", "Provide a move target in steps or millimeters.", "400 Bad Request");
    }

    if (cJSON_IsNumber(custom_speed)) {
        command.data.move.has_custom_speed = true;
        command.data.move.custom_speed_steps_per_sec = custom_speed->valueint;
    }

    cJSON_Delete(body);
    return web_submit_command_and_ack(req, &command);
}

static esp_err_t web_ab_run_handler(httpd_req_t *req)
{
    cJSON *body = web_parse_json_body(req);
    if (body == NULL) {
        return web_send_error(req, "invalid_json", "Unable to parse A/B run JSON.", "400 Bad Request");
    }

    app_command_t command = {
        .type = APP_COMMAND_RUN_AB,
        .data.run_ab.repeat_mode = APP_REPEAT_MODE_NONE,
    };
    cJSON *repeat_mode = cJSON_GetObjectItemCaseSensitive(body, "repeat_mode");
    if (cJSON_IsString(repeat_mode) && repeat_mode->valuestring != NULL) {
        if (!app_parse_repeat_mode(repeat_mode->valuestring, &command.data.run_ab.repeat_mode)) {
            cJSON_Delete(body);
            return web_send_error(req, "invalid_repeat_mode", "Repeat mode must be none, loop, or ping_pong.", "400 Bad Request");
        }
    }

    cJSON_Delete(body);
    return web_submit_command_and_ack(req, &command);
}

static esp_err_t web_program_handler(httpd_req_t *req)
{
    cJSON *body = web_parse_json_body(req);
    if (body == NULL) {
        return web_send_error(req, "invalid_json", "Unable to parse program JSON.", "400 Bad Request");
    }

    cJSON *keyframes = cJSON_GetObjectItemCaseSensitive(body, "keyframes");
    if (!cJSON_IsArray(keyframes)) {
        cJSON_Delete(body);
        return web_send_error(req, "invalid_keyframes", "Program payload must contain a keyframes array.", "400 Bad Request");
    }

    app_runtime_config_t config = {};
    s_web.bindings.get_config(&config);

    app_command_t command = {
        .type = APP_COMMAND_SET_PROGRAM,
        .data.program.axis_id = APP_AXIS_SLIDER,
        .data.program.count = (size_t)cJSON_GetArraySize(keyframes),
    };

    if (command.data.program.count > APP_MAX_KEYFRAMES) {
        cJSON_Delete(body);
        return web_send_error(req, "too_many_keyframes", "A maximum of 16 keyframes is supported.", "400 Bad Request");
    }

    for (size_t i = 0; i < command.data.program.count; ++i) {
        cJSON *entry = cJSON_GetArrayItem(keyframes, (int)i);
        cJSON *position_steps = cJSON_GetObjectItemCaseSensitive(entry, "position_steps");
        cJSON *position_mm = cJSON_GetObjectItemCaseSensitive(entry, "position_mm");
        cJSON *time_ms = cJSON_GetObjectItemCaseSensitive(entry, "time_ms");

        if (!cJSON_IsNumber(time_ms)) {
            cJSON_Delete(body);
            return web_send_error(req, "invalid_keyframe_time", "Each keyframe must include time_ms.", "400 Bad Request");
        }

        command.data.program.keyframes[i].time_ms = (uint32_t)time_ms->valuedouble;
        if (cJSON_IsNumber(position_steps)) {
            command.data.program.keyframes[i].position_steps = position_steps->valueint;
        } else if (cJSON_IsNumber(position_mm)) {
            command.data.program.keyframes[i].position_steps = app_config_mm_to_steps(&config, (float)position_mm->valuedouble);
        } else {
            cJSON_Delete(body);
            return web_send_error(req, "invalid_keyframe_position", "Each keyframe must include position_steps or position_mm.", "400 Bad Request");
        }
    }

    cJSON_Delete(body);
    return web_submit_command_and_ack(req, &command);
}

static esp_err_t web_program_run_handler(httpd_req_t *req)
{
    cJSON *body = web_parse_json_body(req);
    if (body == NULL) {
        return web_send_error(req, "invalid_json", "Unable to parse program run JSON.", "400 Bad Request");
    }

    app_command_t command = {
        .type = APP_COMMAND_RUN_PROGRAM,
        .data.run_program.repeat_mode = APP_REPEAT_MODE_NONE,
    };
    cJSON *repeat_mode = cJSON_GetObjectItemCaseSensitive(body, "repeat_mode");
    if (cJSON_IsString(repeat_mode) && repeat_mode->valuestring != NULL) {
        if (!app_parse_repeat_mode(repeat_mode->valuestring, &command.data.run_program.repeat_mode)) {
            cJSON_Delete(body);
            return web_send_error(req, "invalid_repeat_mode", "Repeat mode must be none, loop, or ping_pong.", "400 Bad Request");
        }
    }

    cJSON_Delete(body);
    return web_submit_command_and_ack(req, &command);
}

static esp_err_t web_program_pause_handler(httpd_req_t *req)
{
    app_command_t command = {.type = APP_COMMAND_PAUSE_PROGRAM};
    return web_submit_command_and_ack(req, &command);
}

static esp_err_t web_program_resume_handler(httpd_req_t *req)
{
    app_command_t command = {.type = APP_COMMAND_RESUME_PROGRAM};
    return web_submit_command_and_ack(req, &command);
}

static esp_err_t web_program_stop_handler(httpd_req_t *req)
{
    app_command_t command = {.type = APP_COMMAND_STOP_PROGRAM};
    return web_submit_command_and_ack(req, &command);
}

static void web_apply_optional_float_field(cJSON *object, const char *name, uint64_t mask, app_config_patch_t *patch, float *destination)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (cJSON_IsNumber(item)) {
        patch->mask |= mask;
        *destination = (float)item->valuedouble;
    }
}

static void web_apply_optional_i32_field(cJSON *object, const char *name, uint64_t mask, app_config_patch_t *patch, int32_t *destination)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (cJSON_IsNumber(item)) {
        patch->mask |= mask;
        *destination = item->valueint;
    }
}

static void web_apply_optional_u32_field(cJSON *object, const char *name, uint64_t mask, app_config_patch_t *patch, uint32_t *destination)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (cJSON_IsNumber(item)) {
        patch->mask |= mask;
        *destination = (uint32_t)item->valuedouble;
    }
}

static esp_err_t web_config_update_handler(httpd_req_t *req)
{
    cJSON *body = web_parse_json_body(req);
    if (body == NULL) {
        return web_send_error(req, "invalid_json", "Unable to parse configuration JSON.", "400 Bad Request");
    }

    app_runtime_config_t current_config = {};
    s_web.bindings.get_config(&current_config);

    app_command_t command = {
        .type = APP_COMMAND_UPDATE_CONFIG,
    };
    command.data.config_patch.values = current_config;

    cJSON *motion = cJSON_GetObjectItemCaseSensitive(body, "motion");
    if (cJSON_IsObject(motion)) {
        web_apply_optional_float_field(motion, "max_speed_steps_per_sec", APP_CONFIG_PATCH_MAX_SPEED, &command.data.config_patch, &command.data.config_patch.values.motion.max_speed_steps_per_sec);
        web_apply_optional_float_field(motion, "acceleration_steps_per_sec2", APP_CONFIG_PATCH_ACCELERATION, &command.data.config_patch, &command.data.config_patch.values.motion.acceleration_steps_per_sec2);
        web_apply_optional_float_field(motion, "jog_speed_steps_per_sec", APP_CONFIG_PATCH_JOG_SPEED, &command.data.config_patch, &command.data.config_patch.values.motion.jog_speed_steps_per_sec);
        cJSON *motion_profile = cJSON_GetObjectItemCaseSensitive(motion, "motion_profile");
        if (cJSON_IsString(motion_profile) && motion_profile->valuestring != NULL) {
            if (!app_parse_motion_profile(motion_profile->valuestring, &command.data.config_patch.values.motion.motion_profile)) {
                cJSON_Delete(body);
                return web_send_error(req, "invalid_motion_profile", "Only trapezoidal motion_profile is supported.", "400 Bad Request");
            }
            command.data.config_patch.mask |= APP_CONFIG_PATCH_MOTION_PROFILE;
        }
    }

    cJSON *homing = cJSON_GetObjectItemCaseSensitive(body, "homing");
    if (cJSON_IsObject(homing)) {
        web_apply_optional_float_field(homing, "homing_speed_steps_per_sec", APP_CONFIG_PATCH_HOMING_SPEED, &command.data.config_patch, &command.data.config_patch.values.homing.homing_speed_steps_per_sec);
        web_apply_optional_float_field(homing, "homing_slow_speed_steps_per_sec", APP_CONFIG_PATCH_HOMING_SLOW_SPEED, &command.data.config_patch, &command.data.config_patch.values.homing.homing_slow_speed_steps_per_sec);
        web_apply_optional_i32_field(homing, "homing_backoff_steps", APP_CONFIG_PATCH_HOMING_BACKOFF, &command.data.config_patch, &command.data.config_patch.values.homing.homing_backoff_steps);
        web_apply_optional_u32_field(homing, "limit_debounce_ms", APP_CONFIG_PATCH_LIMIT_DEBOUNCE, &command.data.config_patch, &command.data.config_patch.values.homing.limit_debounce_ms);
        cJSON *home_side = cJSON_GetObjectItemCaseSensitive(homing, "home_reference_side");
        if (cJSON_IsString(home_side) && home_side->valuestring != NULL) {
            if (!app_parse_home_reference_side(home_side->valuestring, &command.data.config_patch.values.homing.home_reference_side)) {
                cJSON_Delete(body);
                return web_send_error(req, "invalid_home_side", "home_reference_side must be left or right.", "400 Bad Request");
            }
            command.data.config_patch.mask |= APP_CONFIG_PATCH_HOME_SIDE;
        }
    }

    cJSON *mechanics = cJSON_GetObjectItemCaseSensitive(body, "mechanics");
    if (cJSON_IsObject(mechanics)) {
        web_apply_optional_i32_field(mechanics, "steps_per_revolution", APP_CONFIG_PATCH_STEPS_PER_REV, &command.data.config_patch, &command.data.config_patch.values.mechanics.steps_per_revolution);
        web_apply_optional_i32_field(mechanics, "microsteps", APP_CONFIG_PATCH_MICROSTEPS, &command.data.config_patch, &command.data.config_patch.values.mechanics.microsteps);
        web_apply_optional_float_field(mechanics, "belt_pitch_mm", APP_CONFIG_PATCH_BELT_PITCH, &command.data.config_patch, &command.data.config_patch.values.mechanics.belt_pitch_mm);
        web_apply_optional_i32_field(mechanics, "pulley_teeth", APP_CONFIG_PATCH_PULLEY_TEETH, &command.data.config_patch, &command.data.config_patch.values.mechanics.pulley_teeth);
        web_apply_optional_float_field(mechanics, "steps_per_mm", APP_CONFIG_PATCH_STEPS_PER_MM, &command.data.config_patch, &command.data.config_patch.values.mechanics.steps_per_mm);
        web_apply_optional_float_field(mechanics, "slider_travel_mm", APP_CONFIG_PATCH_SLIDER_TRAVEL, &command.data.config_patch, &command.data.config_patch.values.mechanics.slider_travel_mm);
    }

    cJSON *axis_behavior = cJSON_GetObjectItemCaseSensitive(body, "axis_behavior");
    if (cJSON_IsObject(axis_behavior)) {
        cJSON *invert_direction = cJSON_GetObjectItemCaseSensitive(axis_behavior, "invert_direction_output");
        if (cJSON_IsBool(invert_direction)) {
            command.data.config_patch.values.axis_behavior.invert_direction_output = cJSON_IsTrue(invert_direction);
            command.data.config_patch.mask |= APP_CONFIG_PATCH_INVERT_DIRECTION;
        }
        cJSON *repeat_mode = cJSON_GetObjectItemCaseSensitive(axis_behavior, "repeat_mode");
        if (cJSON_IsString(repeat_mode) && repeat_mode->valuestring != NULL) {
            if (!app_parse_repeat_mode(repeat_mode->valuestring, &command.data.config_patch.values.axis_behavior.repeat_mode)) {
                cJSON_Delete(body);
                return web_send_error(req, "invalid_repeat_mode", "repeat_mode must be none, loop, or ping_pong.", "400 Bad Request");
            }
            command.data.config_patch.mask |= APP_CONFIG_PATCH_REPEAT_MODE;
        }
    }

    cJSON_Delete(body);
    if (command.data.config_patch.mask == 0) {
        return web_send_error(req, "no_changes", "No writable configuration values were provided.", "400 Bad Request");
    }

    return web_submit_command_and_ack(req, &command);
}

static void web_wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base != WIFI_EVENT) {
        return;
    }

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "Station connected, AID=%d", event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "Station disconnected, AID=%d", event->aid);
    }
}

static esp_err_t web_start_wifi_ap(const app_runtime_config_t *config)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Failed to initialize esp_netif");
    esp_err_t event_loop_result = esp_event_loop_create_default();
    if (event_loop_result != ESP_OK && event_loop_result != ESP_ERR_INVALID_STATE) {
        return event_loop_result;
    }

    s_web.ap_netif = esp_netif_create_default_wifi_ap();
    if (s_web.ap_netif == NULL) {
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &web_wifi_event_handler, NULL), TAG, "Failed to register Wi-Fi event handler");

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_init_config), TAG, "Failed to initialize Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "Failed to select Wi-Fi RAM storage");

    esp_netif_ip_info_t ip_info = {};
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    ESP_RETURN_ON_ERROR(esp_netif_dhcps_stop(s_web.ap_netif), TAG, "Failed to stop DHCP server");
    ESP_RETURN_ON_ERROR(esp_netif_set_ip_info(s_web.ap_netif, &ip_info), TAG, "Failed to configure AP IP");
    ESP_RETURN_ON_ERROR(esp_netif_dhcps_start(s_web.ap_netif), TAG, "Failed to restart DHCP server");

    wifi_config_t wifi_config = {
        .ap = {
            .channel = config->wifi_ap.channel,
            .max_connection = config->wifi_ap.max_connections,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .ssid_hidden = 0,
        },
    };
    snprintf((char *)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", config->wifi_ap.ssid);
    snprintf((char *)wifi_config.ap.password, sizeof(wifi_config.ap.password), "%s", config->wifi_ap.password);
    wifi_config.ap.ssid_len = (uint8_t)strlen(config->wifi_ap.ssid);
    if (wifi_config.ap.password[0] == '\0') {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "Failed to set AP mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config), TAG, "Failed to apply AP configuration");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Failed to start Wi-Fi AP");

    ESP_LOGI(TAG, "AP ready: SSID=%s IP=%s", config->wifi_ap.ssid, config->wifi_ap.ip_address);
    return ESP_OK;
}

static esp_err_t web_register_uri(httpd_handle_t server, const char *uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t *req))
{
    httpd_uri_t config = {
        .uri = uri,
        .method = method,
        .handler = handler,
        .user_ctx = NULL,
    };
    return httpd_register_uri_handler(server, &config);
}

esp_err_t web_service_start(const web_service_bindings_t *bindings)
{
    if (bindings == NULL ||
        bindings->submit_command == NULL ||
        bindings->get_status == NULL ||
        bindings->get_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_web.started) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_web, 0, sizeof(s_web));
    s_web.bindings = *bindings;

    app_runtime_config_t config = {};
    s_web.bindings.get_config(&config);
    ESP_RETURN_ON_ERROR(web_start_wifi_ap(&config), TAG, "Failed to start Wi-Fi AP");

    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.max_uri_handlers = 18;
    server_config.uri_match_fn = httpd_uri_match_wildcard;
    ESP_RETURN_ON_ERROR(httpd_start(&s_web.server, &server_config), TAG, "Failed to start HTTP server");

    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/", HTTP_GET, web_root_handler), TAG, "Failed to register /");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/app.js", HTTP_GET, web_js_handler), TAG, "Failed to register /app.js");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/styles.css", HTTP_GET, web_css_handler), TAG, "Failed to register /styles.css");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/status", HTTP_GET, web_status_handler), TAG, "Failed to register /api/status");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/config", HTTP_GET, web_config_handler), TAG, "Failed to register GET /api/config");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/config", HTTP_POST, web_config_update_handler), TAG, "Failed to register POST /api/config");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/home", HTTP_POST, web_home_handler), TAG, "Failed to register /api/home");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/stop", HTTP_POST, web_stop_handler), TAG, "Failed to register /api/stop");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/jog", HTTP_POST, web_jog_handler), TAG, "Failed to register /api/jog");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/move", HTTP_POST, web_move_handler), TAG, "Failed to register /api/move");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/ab/set_a", HTTP_POST, web_set_a_handler), TAG, "Failed to register /api/ab/set_a");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/ab/set_b", HTTP_POST, web_set_b_handler), TAG, "Failed to register /api/ab/set_b");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/ab/run", HTTP_POST, web_ab_run_handler), TAG, "Failed to register /api/ab/run");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/program", HTTP_POST, web_program_handler), TAG, "Failed to register /api/program");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/program/run", HTTP_POST, web_program_run_handler), TAG, "Failed to register /api/program/run");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/program/pause", HTTP_POST, web_program_pause_handler), TAG, "Failed to register /api/program/pause");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/program/resume", HTTP_POST, web_program_resume_handler), TAG, "Failed to register /api/program/resume");
    ESP_RETURN_ON_ERROR(web_register_uri(s_web.server, "/api/program/stop", HTTP_POST, web_program_stop_handler), TAG, "Failed to register /api/program/stop");

    s_web.started = true;
    return ESP_OK;
}
