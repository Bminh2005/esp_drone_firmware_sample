/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      driver_mpu6500_interface.c
 * @brief     driver mpu6500 interface implementation file for ESP-IDF v5.3+
 * @version   1.0.0
 * @author    Shifeng Li / ESP-IDF Integration
 * @date      2024-07-30
 */

#include "driver_mpu6500_interface.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* =========================================================================
 * HARDWARE CONFIGURATION (Adjust GPIO pins based on your board layout)
 * ========================================================================= */

/* Configuration for I2C */
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ 400000 /* 400 kHz Fast Mode */

/* Configuration for SPI */
#define SPI_MASTER_HOST SPI2_HOST
#define SPI_MASTER_SCLK_IO GPIO_NUM_18
#define SPI_MASTER_MOSI_IO GPIO_NUM_23
#define SPI_MASTER_MISO_IO GPIO_NUM_19
#define SPI_MASTER_CS_IO GPIO_NUM_5
#define SPI_MASTER_FREQ_HZ (10 * 1000 * 1000) /* 10 MHz */

/* ESP Logging Tag */
static const char *TAG = "mpu6500_interface";

/* Internal handles for I2C */
static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t s_i2c_dev_handle = NULL;
static uint8_t s_cur_i2c_addr = 0xFF;

/* Internal handles for SPI */
static spi_device_handle_t s_spi_dev_handle = NULL;

/* =========================================================================
 * I2C INTERFACE IMPLEMENTATION (ESP-IDF v5.3+)
 * ========================================================================= */

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t mpu6500_interface_iic_init(void)
{
    if (s_i2c_bus_handle != NULL)
    {
        return 0; /* Already initialized */
    }

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    if (i2c_new_master_bus(&bus_config, &s_i2c_bus_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus!");
        return 1;
    }

    return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t mpu6500_interface_iic_deinit(void)
{
    if (s_i2c_dev_handle != NULL)
    {
        if (i2c_master_bus_rm_device(s_i2c_dev_handle) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to remove I2C device!");
            return 1;
        }
        s_i2c_dev_handle = NULL;
        s_cur_i2c_addr = 0xFF;
    }

    if (s_i2c_bus_handle != NULL)
    {
        if (i2c_del_master_bus(s_i2c_bus_handle) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to delete I2C master bus!");
            return 1;
        }
        s_i2c_bus_handle = NULL;
    }

    return 0;
}

/**
 * @brief Helper function to register/ensure I2C device instance by 7-bit address
 */
static esp_err_t mpu6500_i2c_ensure_device(uint8_t addr)
{
    /* Convert 8-bit write address (0xD0/0xD2) to 7-bit (0x68/0x69) if necessary */
    uint8_t dev_7bit_addr = (addr >= 0x80) ? (addr >> 1) : addr;

    if (s_i2c_bus_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_i2c_dev_handle == NULL || s_cur_i2c_addr != dev_7bit_addr)
    {
        if (s_i2c_dev_handle != NULL)
        {
            i2c_master_bus_rm_device(s_i2c_dev_handle);
            s_i2c_dev_handle = NULL;
        }

        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = dev_7bit_addr,
            .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        };

        esp_err_t err = i2c_master_bus_add_device(s_i2c_bus_handle, &dev_cfg, &s_i2c_dev_handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to add I2C device at address 0x%02X!", dev_7bit_addr);
            return err;
        }
        s_cur_i2c_addr = dev_7bit_addr;
    }

    return ESP_OK;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mpu6500_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (mpu6500_i2c_ensure_device(addr) != ESP_OK)
    {
        return 1;
    }

    if (i2c_master_transmit_receive(s_i2c_dev_handle, &reg, 1, buf, len, -1) != ESP_OK)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mpu6500_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (mpu6500_i2c_ensure_device(addr) != ESP_OK)
    {
        return 1;
    }

    uint8_t *tx_buf = (uint8_t *)malloc(len + 1);
    if (tx_buf == NULL)
    {
        return 1;
    }

    tx_buf[0] = reg;
    if (len > 0 && buf != NULL)
    {
        memcpy(&tx_buf[1], buf, len);
    }

    esp_err_t ret = i2c_master_transmit(s_i2c_dev_handle, tx_buf, len + 1, -1);
    free(tx_buf);

    return (ret == ESP_OK) ? 0 : 1;
}

/* =========================================================================
 * SPI INTERFACE IMPLEMENTATION
 * ========================================================================= */

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t mpu6500_interface_spi_init(void)
{
    if (s_spi_dev_handle != NULL)
    {
        return 0;
    }

    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MASTER_MISO_IO,
        .mosi_io_num = SPI_MASTER_MOSI_IO,
        .sclk_io_num = SPI_MASTER_SCLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_HZ,
        .mode = 0, /* MPU6500 supports SPI Mode 0 (CPOL=0, CPHA=0) and Mode 3 */
        .spics_io_num = SPI_MASTER_CS_IO,
        .queue_size = 7,
    };

    if (spi_bus_initialize(SPI_MASTER_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus!");
        return 1;
    }

    if (spi_bus_add_device(SPI_MASTER_HOST, &devcfg, &s_spi_dev_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add SPI device!");
        spi_bus_free(SPI_MASTER_HOST);
        return 1;
    }

    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t mpu6500_interface_spi_deinit(void)
{
    if (s_spi_dev_handle != NULL)
    {
        if (spi_bus_remove_device(s_spi_dev_handle) != ESP_OK)
        {
            return 1;
        }
        s_spi_dev_handle = NULL;
    }

    if (spi_bus_free(SPI_MASTER_HOST) != ESP_OK)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mpu6500_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (s_spi_dev_handle == NULL || buf == NULL || len == 0)
    {
        return 1;
    }

    uint8_t *tx_buf = (uint8_t *)malloc(len + 1);
    uint8_t *rx_buf = (uint8_t *)malloc(len + 1);
    if (!tx_buf || !rx_buf)
    {
        if (tx_buf)
            free(tx_buf);
        if (rx_buf)
            free(rx_buf);
        return 1;
    }

    memset(tx_buf, 0, len + 1);
    tx_buf[0] = reg | 0x80; /* Bit 7 = 1 specifies a Read operation for MPU6500 */

    spi_transaction_t t = {
        .length = 8 * (len + 1),
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    esp_err_t ret = spi_device_transmit(s_spi_dev_handle, &t);
    if (ret == ESP_OK)
    {
        memcpy(buf, &rx_buf[1], len);
    }

    free(tx_buf);
    free(rx_buf);

    return (ret == ESP_OK) ? 0 : 1;
}

/**
 * @brief     interface spi bus write
 * @param[in] reg register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mpu6500_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (s_spi_dev_handle == NULL)
    {
        return 1;
    }

    uint8_t *tx_buf = (uint8_t *)malloc(len + 1);
    if (tx_buf == NULL)
    {
        return 1;
    }

    tx_buf[0] = reg & 0x7F; /* Bit 7 = 0 specifies a Write operation for MPU6500 */
    if (len > 0 && buf != NULL)
    {
        memcpy(&tx_buf[1], buf, len);
    }

    spi_transaction_t t = {
        .length = 8 * (len + 1),
        .tx_buffer = tx_buf,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_transmit(s_spi_dev_handle, &t);
    free(tx_buf);

    return (ret == ESP_OK) ? 0 : 1;
}

/* =========================================================================
 * UTILITY FUNCTIONS (DELAY & DEBUG PRINT)
 * ========================================================================= */

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void mpu6500_interface_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void mpu6500_interface_debug_print(const char *const fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

/* =========================================================================
 * CALLBACK FUNCTIONS (INTERRUPT & DMP)
 * ========================================================================= */

/**
 * @brief     interface receive callback
 * @param[in] type irq type
 * @note      none
 */
void mpu6500_interface_receive_callback(uint8_t type)
{
    switch (type)
    {
    case MPU6500_INTERRUPT_MOTION:
    {
        mpu6500_interface_debug_print("mpu6500: irq motion.\n");
        break;
    }
    case MPU6500_INTERRUPT_FIFO_OVERFLOW:
    {
        mpu6500_interface_debug_print("mpu6500: irq fifo overflow.\n");
        break;
    }
    case MPU6500_INTERRUPT_FSYNC_INT:
    {
        mpu6500_interface_debug_print("mpu6500: irq fsync int.\n");
        break;
    }
    case MPU6500_INTERRUPT_DMP:
    {
        mpu6500_interface_debug_print("mpu6500: irq dmp.\n");
        break;
    }
    case MPU6500_INTERRUPT_DATA_READY:
    {
        mpu6500_interface_debug_print("mpu6500: irq data ready.\n");
        break;
    }
    default:
    {
        mpu6500_interface_debug_print("mpu6500: irq unknown code.\n");
        break;
    }
    }
}

/**
 * @brief     interface dmp tap callback
 * @param[in] count tap count
 * @param[in] direction tap direction
 * @note      none
 */
void mpu6500_interface_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction)
    {
    case MPU6500_DMP_TAP_X_UP:
    {
        mpu6500_interface_debug_print("mpu6500: tap irq x up with %d.\n", count);
        break;
    }
    case MPU6500_DMP_TAP_X_DOWN:
    {
        mpu6500_interface_debug_print("mpu6500: tap irq x down with %d.\n", count);
        break;
    }
    case MPU6500_DMP_TAP_Y_UP:
    {
        mpu6500_interface_debug_print("mpu6500: tap irq y up with %d.\n", count);
        break;
    }
    case MPU6500_DMP_TAP_Y_DOWN:
    {
        mpu6500_interface_debug_print("mpu6500: tap irq y down with %d.\n", count);
        break;
    }
    case MPU6500_DMP_TAP_Z_UP:
    {
        mpu6500_interface_debug_print("mpu6500: tap irq z up with %d.\n", count);
        break;
    }
    case MPU6500_DMP_TAP_Z_DOWN:
    {
        mpu6500_interface_debug_print("mpu6500: tap irq z down with %d.\n", count);
        break;
    }
    default:
    {
        mpu6500_interface_debug_print("mpu6500: tap irq unknown code.\n");
        break;
    }
    }
}

/**
 * @brief     interface dmp orient callback
 * @param[in] orientation dmp orientation
 * @note      none
 */
void mpu6500_interface_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation)
    {
    case MPU6500_DMP_ORIENT_PORTRAIT:
    {
        mpu6500_interface_debug_print("mpu6500: orient irq portrait.\n");
        break;
    }
    case MPU6500_DMP_ORIENT_LANDSCAPE:
    {
        mpu6500_interface_debug_print("mpu6500: orient irq landscape.\n");
        break;
    }
    case MPU6500_DMP_ORIENT_REVERSE_PORTRAIT:
    {
        mpu6500_interface_debug_print("mpu6500: orient irq reverse portrait.\n");
        break;
    }
    case MPU6500_DMP_ORIENT_REVERSE_LANDSCAPE:
    {
        mpu6500_interface_debug_print("mpu6500: orient irq reverse landscape.\n");
        break;
    }
    default:
    {
        mpu6500_interface_debug_print("mpu6500: orient irq unknown code.\n");
        break;
    }
    }
}