idf_component_register(
    SRCS "src/app_controller.c"
    INCLUDE_DIRS "include"
    REQUIRES app_common motion esp_timer freertos esp_system
)
