idf_component_register(
    SRCS "main.c" "state_estimator.c" "flight_control.c"
    INCLUDE_DIRS "."
    REQUIRES 
        drivers 
        mpu6500 
        driver 
        esp_timer
        mathtools
        altitude_filter
        flow_filter
        esp_wifi nvs_flash esp_event esp_netif esp_adc
)