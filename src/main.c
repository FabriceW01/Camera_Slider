#include "esp_log.h"
#include "nvs_flash.h"

#include "app_controller.h"
#include "web_service.h"

static const char *TAG = "main";

void app_main(void)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(result);

    ESP_ERROR_CHECK(app_controller_start());

    web_service_bindings_t bindings = {
        .submit_command = app_controller_submit_command,
        .get_status = app_controller_get_status_snapshot,
        .get_config = app_controller_get_runtime_config,
    };
    ESP_ERROR_CHECK(web_service_start(&bindings));

    ESP_LOGI(TAG, "Kamera Slider firmware is running.");
}
