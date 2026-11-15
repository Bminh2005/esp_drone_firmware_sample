idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        drivers 
        mpu6500 
        driver 
        esp_timer
)