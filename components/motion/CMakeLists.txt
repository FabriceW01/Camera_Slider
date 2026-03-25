idf_component_register(
    SRCS "src/motion_axis.c"
    INCLUDE_DIRS "include"
    REQUIRES app_common esp_driver_gptimer esp_driver_gpio esp_timer freertos
)
