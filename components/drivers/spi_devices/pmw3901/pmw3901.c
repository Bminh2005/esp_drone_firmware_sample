/*
 *
 *
 * LPS node firmware.
 *
 * Copyright 2017, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* pwm3901.c: PMW3901 driver */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// ESP-IDF Drivers & Utilities
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h" // Hoặc "driver/i2c.h" nếu dùng IDF v4.x
#include "esp_rom_delay.h"     // Cung cấp hàm delay microsecond
#include "esp_log.h"
#include "driver/gpio.h"
static const char *TAG = "PMW3901";

// --- ÁNH XÁ HÀM NỘI BỘ ---
#define sleepus(us) esp_rom_delay_us(us)
#define sleepms(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#define DEBUG_PRINT(...) ESP_LOGI(TAG, __VA_ARGS__)
#define DEBUG_MODULE "PMW"

extern spi_device_handle_t pmw3901_spi_handle;

static bool isInit = false;

static void registerWrite(uint32_t csPin, uint8_t reg, uint8_t value)
{
    // Set MSB to 1 for write
    reg |= 0x80u;

    spi_transaction_t t_reg = {
        .length = 8, // 8 bits (1 byte)
        .tx_buffer = &reg,
    };
    spi_transaction_t t_val = {
        .length = 8, // 8 bits (1 byte)
        .tx_buffer = &value,
    };

    // Kéo CS xuống LOW
    gpio_set_level((gpio_num_t)csPin, 0);
    esp_rom_delay_us(50);

    // Gửi byte địa chỉ
    spi_device_polling_transmit(pmw3901_spi_handle, &t_reg);
    esp_rom_delay_us(50);

    // Gửi byte dữ liệu
    spi_device_polling_transmit(pmw3901_spi_handle, &t_val);
    esp_rom_delay_us(50);

    // Kéo CS lên HIGH
    gpio_set_level((gpio_num_t)csPin, 1);
    esp_rom_delay_us(200);
}

static uint8_t registerRead(uint32_t csPin, uint8_t reg)
{
    uint8_t data = 0;

    // Set MSB to 0 for read
    reg &= ~0x80u;

    spi_transaction_t t_reg = {
        .length = 8,
        .tx_buffer = &reg,
    };
    spi_transaction_t t_data = {
        .length = 8,
        .rx_buffer = &data,
    };

    // Kéo CS xuống LOW
    gpio_set_level((gpio_num_t)csPin, 0);
    esp_rom_delay_us(50);

    // Gửi byte địa chỉ
    spi_device_polling_transmit(pmw3901_spi_handle, &t_reg);

    // Khoảng chờ bắt buộc của PMW3901 trước khi nhận byte dữ liệu
    esp_rom_delay_us(500);

    // Đọc byte dữ liệu trả về
    spi_device_polling_transmit(pmw3901_spi_handle, &t_data);
    esp_rom_delay_us(50);

    // Kéo CS lên HIGH
    gpio_set_level((gpio_num_t)csPin, 1);
    esp_rom_delay_us(200);

    return data;
}

static void InitRegisters(uint32_t csPin)
{
    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x61, 0xAD);
    registerWrite(csPin, 0x7F, 0x03);
    registerWrite(csPin, 0x40, 0x00);
    registerWrite(csPin, 0x7F, 0x05);
    registerWrite(csPin, 0x41, 0xB3);
    registerWrite(csPin, 0x43, 0xF1);
    registerWrite(csPin, 0x45, 0x14);
    registerWrite(csPin, 0x5B, 0x32);
    registerWrite(csPin, 0x5F, 0x34);
    registerWrite(csPin, 0x7B, 0x08);
    registerWrite(csPin, 0x7F, 0x06);
    registerWrite(csPin, 0x44, 0x1B);
    registerWrite(csPin, 0x40, 0xBF);
    registerWrite(csPin, 0x4E, 0x3F);
    registerWrite(csPin, 0x7F, 0x08);
    registerWrite(csPin, 0x65, 0x20);
    registerWrite(csPin, 0x6A, 0x18);
    registerWrite(csPin, 0x7F, 0x09);
    registerWrite(csPin, 0x4F, 0xAF);
    registerWrite(csPin, 0x5F, 0x40);
    registerWrite(csPin, 0x48, 0x80);
    registerWrite(csPin, 0x49, 0x80);
    registerWrite(csPin, 0x57, 0x77);
    registerWrite(csPin, 0x60, 0x78);
    registerWrite(csPin, 0x61, 0x78);
    registerWrite(csPin, 0x62, 0x08);
    registerWrite(csPin, 0x63, 0x50);
    registerWrite(csPin, 0x7F, 0x0A);
    registerWrite(csPin, 0x45, 0x60);
    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x4D, 0x11);
    registerWrite(csPin, 0x55, 0x80);
    registerWrite(csPin, 0x74, 0x1F);
    registerWrite(csPin, 0x75, 0x1F);
    registerWrite(csPin, 0x4A, 0x78);
    registerWrite(csPin, 0x4B, 0x78);
    registerWrite(csPin, 0x44, 0x08);
    registerWrite(csPin, 0x45, 0x50);
    registerWrite(csPin, 0x64, 0xFF);
    registerWrite(csPin, 0x65, 0x1F);
    registerWrite(csPin, 0x7F, 0x14);
    registerWrite(csPin, 0x65, 0x67);
    registerWrite(csPin, 0x66, 0x08);
    registerWrite(csPin, 0x63, 0x70);
    registerWrite(csPin, 0x7F, 0x15);
    registerWrite(csPin, 0x48, 0x48);
    registerWrite(csPin, 0x7F, 0x07);
    registerWrite(csPin, 0x41, 0x0D);
    registerWrite(csPin, 0x43, 0x14);
    registerWrite(csPin, 0x4B, 0x0E);
    registerWrite(csPin, 0x45, 0x0F);
    registerWrite(csPin, 0x44, 0x42);
    registerWrite(csPin, 0x4C, 0x80);
    registerWrite(csPin, 0x7F, 0x10);
    registerWrite(csPin, 0x5B, 0x02);
    registerWrite(csPin, 0x7F, 0x07);
    registerWrite(csPin, 0x40, 0x41);
    registerWrite(csPin, 0x70, 0x00);

    vTaskDelay(M2T(10)); // delay 10ms

    registerWrite(csPin, 0x32, 0x44);
    registerWrite(csPin, 0x7F, 0x07);
    registerWrite(csPin, 0x40, 0x40);
    registerWrite(csPin, 0x7F, 0x06);
    registerWrite(csPin, 0x62, 0xF0);
    registerWrite(csPin, 0x63, 0x00);
    registerWrite(csPin, 0x7F, 0x0D);
    registerWrite(csPin, 0x48, 0xC0);
    registerWrite(csPin, 0x6F, 0xD5);
    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x5B, 0xA0);
    registerWrite(csPin, 0x4E, 0xA8);
    registerWrite(csPin, 0x5A, 0x50);
    registerWrite(csPin, 0x40, 0x80);

    registerWrite(csPin, 0x7F, 0x00);
    registerWrite(csPin, 0x5A, 0x10);
    registerWrite(csPin, 0x54, 0x00);
}

bool pmw3901Init(uint32_t csPin)
{
    if (isInit)
    {
        return true;
    }

    // 1. Khởi tạo chân CS GPIO trên ESP-IDF (thay thế pinMode)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << csPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)csPin, 1);

    // Thay cho spiBegin() & vTaskDelay(M2T(40))
    vTaskDelay(pdMS_TO_TICKS(40));

    // Reset sequence toggle CS Pin
    gpio_set_level((gpio_num_t)csPin, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level((gpio_num_t)csPin, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level((gpio_num_t)csPin, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    // Đọc Chip ID kiểm tra kết nối
    uint8_t chipId = registerRead(csPin, 0x00);
    uint8_t invChipId = registerRead(csPin, 0x5f);

    ESP_LOGI(TAG, "Motion chip id: 0x%02x:0x%02x", chipId, invChipId);

    if (chipId == 0x49 || invChipId == 0xB6)
    {
        // Power on reset
        registerWrite(csPin, 0x3a, 0x5a);
        vTaskDelay(pdMS_TO_TICKS(5));

        // Đọc xóa dữ liệu rác trên các thanh ghi motion
        registerRead(csPin, 0x02);
        registerRead(csPin, 0x03);
        registerRead(csPin, 0x04);
        registerRead(csPin, 0x05);
        registerRead(csPin, 0x06);
        vTaskDelay(pdMS_TO_TICKS(1));

        // Cấu hình bảng thanh ghi khởi tạo
        InitRegisters(csPin);

        isInit = true;
    }

    return isInit;
}

void pmw3901ReadMotion(uint32_t csPin, motionBurst_t *motion)
{
    uint8_t address = 0x16; // Thanh ghi Burst read

    // Kéo CS LOW
    gpio_set_level((gpio_num_t)csPin, 0);
    esp_rom_delay_us(50);

    // 1. Gửi byte địa chỉ 0x16
    spi_transaction_t t_addr = {
        .length = 8,
        .tx_buffer = &address,
    };
    spi_device_polling_transmit(pmw3901_spi_handle, &t_addr);

    esp_rom_delay_us(50);

    // 2. Đọc Burst toàn bộ struct motionBurst_t (thay thế spiExchange)
    spi_transaction_t t_burst = {
        .length = sizeof(motionBurst_t) * 8, // Chuyển byte thành bit
        .rx_buffer = motion,
    };
    spi_device_polling_transmit(pmw3901_spi_handle, &t_burst);

    esp_rom_delay_us(50);

    // Kéo CS HIGH
    gpio_set_level((gpio_num_t)csPin, 1);
    esp_rom_delay_us(50);

    // Dao ngược Endianness cho giá trị Shutter
    uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
    realShutter |= (motion->shutter & 0x0ff) << 8;
    motion->shutter = realShutter;
}
