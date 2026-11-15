idf_component_register(
    SRCS
        # Driver PMW3901
        "spi_devices/pmw3901/pmw3901.c"
        "spi_devices/pmw3901/flowdeck_v1v2.c"

        # Driver VL53L0X / VL53L1X (chọn phiên bản bạn dùng)
        "i2c_devices/vl53l0/vl53l0x.c"
        "i2c_devices/vl53l0/zranger.c"
        # "i2c_devices/vl53l1/vl53l1x.c"
        # "i2c_devices/vl53l1/zranger2.c"

    INCLUDE_DIRS
        "spi_devices/pmw3901/include"
        "i2c_devices/vl53l0/include"
        "i2c_devices/vl53l1/include"
    REQUIRES
        driver # Các thư viện SPI/I2C mặc định của hệ thống
)