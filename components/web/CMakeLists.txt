idf_component_register(
    SRCS
        "src/web_service.c"
        "generated/web_assets.c"
    INCLUDE_DIRS
        "include"
        "generated"
    REQUIRES app_common esp_http_server esp_wifi esp_event esp_netif json
)
