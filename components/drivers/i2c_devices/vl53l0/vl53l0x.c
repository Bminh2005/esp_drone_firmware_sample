/**
 * VL53L0X Time-of-flight distance sensor driver for ESP-IDF
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "vl53l0x.h"

static const char *TAG = "VL53L0X";
#include "driver/i2c_master.h"

// --- BỘ ĐỆM Macro & Log cho ESP-IDF ---
#define DEBUG_PRINTD(fmt, ...) ESP_LOGE(TAG, fmt, ##__VA_ARGS__)
#define M2T(ms) pdMS_TO_TICKS(ms)

// Record the current time to check an upcoming timeout against
#define startTimeout() (dev->timeout_start_ms = xTaskGetTickCount())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (dev->io_timeout > 0 && ((uint16_t)xTaskGetTickCount() - dev->timeout_start_ms) > dev->io_timeout)

// Decode VCSEL pulse period in PCLKs from register value
#define decodeVcselPeriod(reg_val) (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in nanoseconds from VCSEL period in PCLKs
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// --- WRAPPER HÀM GIAO TIẾP I2C CHO ESP-IDF v5.x (driver/i2c_master.h) ---

static bool i2cdevReadByte(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t *data)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, 1, -1) == ESP_OK;
}

static bool i2cdevWriteByte(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};
    return i2c_master_transmit(dev_handle, buffer, 2, -1) == ESP_OK;
}

static bool i2cdevReadReg8(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint16_t len, uint8_t *data)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, -1) == ESP_OK;
}

static bool i2cdevWriteReg8(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint16_t len, const uint8_t *data)
{
    uint8_t buffer[len + 1];
    buffer[0] = reg;
    memcpy(&buffer[1], data, len);
    return i2c_master_transmit(dev_handle, buffer, len + 1, -1) == ESP_OK;
}

static bool i2cdevWriteBit(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b = 0;
    if (!i2cdevReadByte(dev_handle, reg, &b))
        return false;
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return i2cdevWriteByte(dev_handle, reg, b);
}
// --- KHAI BÁO PROTOTYPE NỘI BỘ ---
static bool vl53l0xGetSpadInfo(VL53L0xDev *dev, uint8_t *count, bool *type_is_aperture);
static void vl53l0xGetSequenceStepEnables(VL53L0xDev *dev, SequenceStepEnables *enables);
static void vl53l0xGetSequenceStepTimeouts(VL53L0xDev *dev, SequenceStepEnables const *enables, SequenceStepTimeouts *timeouts);
static bool vl53l0xPerformSingleRefCalibration(VL53L0xDev *dev, uint8_t vhv_init_byte);
static uint16_t vl53l0xDecodeTimeout(uint16_t reg_val);
static uint16_t vl53l0xEncodeTimeout(uint16_t timeout_mclks);
static uint32_t vl53l0xTimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t vl53l0xTimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

uint16_t vl53l0xReadReg16Bit(VL53L0xDev *dev, uint8_t reg);
static bool vl53l0xWriteReg16Bit(VL53L0xDev *dev, uint8_t reg, uint16_t val);
static bool vl53l0xWriteReg32Bit(VL53L0xDev *dev, uint8_t reg, uint32_t val);

bool vl53l0xInit(VL53L0xDev *dev, i2c_master_dev_handle_t i2c_dev, bool io_2V8)
{
    dev->i2c_dev = i2c_dev;
    dev->devAddr = VL53L0X_DEFAULT_ADDRESS;
    dev->io_timeout = 500; // ms
    dev->did_timeout = false;

    if (!vl53l0xInitSensor(dev, io_2V8))
    {
        return false;
    }
    return true;
}

bool vl53l0xTestConnection(VL53L0xDev *dev)
{
    bool ret = true;
    ret &= vl53l0xGetModelID(dev) == VL53L0X_IDENTIFICATION_MODEL_ID;
    ret &= vl53l0xGetRevisionID(dev) == VL53L0X_IDENTIFICATION_REVISION_ID;
    return ret;
}

uint16_t vl53l0xGetModelID(VL53L0xDev *dev)
{
    return vl53l0xReadReg16Bit(dev, VL53L0X_RA_IDENTIFICATION_MODEL_ID);
}

uint8_t vl53l0xGetRevisionID(VL53L0xDev *dev)
{
    uint8_t output = 0;
    i2cdevReadByte(dev->i2c_dev, VL53L0X_RA_IDENTIFICATION_REVISION_ID, &output);
    return output;
}

bool vl53l0xSetI2CAddress(VL53L0xDev *dev, uint8_t address)
{
    bool pass = i2cdevWriteByte(dev->i2c_dev, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, address);
    dev->devAddr = address;
    return pass;
}

bool vl53l0xInitSensor(VL53L0xDev *dev, bool io_2v8)
{
    uint8_t temp;

    if (io_2v8)
    {
        i2cdevWriteBit(dev->i2c_dev, VL53L0X_RA_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 0, 0x01);
    }

    if (!i2cdevWriteByte(dev->i2c_dev, 0x88, 0x00))
    {
        return false;
    }

    i2cdevWriteByte(dev->i2c_dev, 0x80, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x00, 0x00);
    i2cdevReadByte(dev->i2c_dev, 0x91, &dev->stop_variable);
    i2cdevWriteByte(dev->i2c_dev, 0x00, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x80, 0x00);

    i2cdevReadByte(dev->i2c_dev, VL53L0X_RA_MSRC_CONFIG_CONTROL, &temp);
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_MSRC_CONFIG_CONTROL, temp | 0x12);

    vl53l0xSetSignalRateLimit(dev, 0.25f);
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    uint8_t spad_count;
    bool spad_type_is_aperture;

    if (!vl53l0xGetSpadInfo(dev, &spad_count, &spad_type_is_aperture))
    {
        return false;
    }

    uint8_t ref_spad_map[6];
    i2cdevReadReg8(dev->i2c_dev, VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map);

    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++)
    {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
        {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }

    i2cdevWriteReg8(dev->i2c_dev, VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map);

    // Tuning Settings
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x00, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x09, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x10, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x11, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x24, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x25, 0xFF);
    i2cdevWriteByte(dev->i2c_dev, 0x75, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x4E, 0x2C);
    i2cdevWriteByte(dev->i2c_dev, 0x48, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x30, 0x20);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x30, 0x09);
    i2cdevWriteByte(dev->i2c_dev, 0x54, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x31, 0x04);
    i2cdevWriteByte(dev->i2c_dev, 0x32, 0x03);
    i2cdevWriteByte(dev->i2c_dev, 0x40, 0x83);
    i2cdevWriteByte(dev->i2c_dev, 0x46, 0x25);
    i2cdevWriteByte(dev->i2c_dev, 0x60, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x27, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x50, 0x06);
    i2cdevWriteByte(dev->i2c_dev, 0x51, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x52, 0x96);
    i2cdevWriteByte(dev->i2c_dev, 0x56, 0x08);
    i2cdevWriteByte(dev->i2c_dev, 0x57, 0x30);
    i2cdevWriteByte(dev->i2c_dev, 0x61, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x62, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x64, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x65, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x66, 0xA0);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x22, 0x32);
    i2cdevWriteByte(dev->i2c_dev, 0x47, 0x14);
    i2cdevWriteByte(dev->i2c_dev, 0x49, 0xFF);
    i2cdevWriteByte(dev->i2c_dev, 0x4A, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x7A, 0x0A);
    i2cdevWriteByte(dev->i2c_dev, 0x7B, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x78, 0x21);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x23, 0x34);
    i2cdevWriteByte(dev->i2c_dev, 0x42, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x44, 0xFF);
    i2cdevWriteByte(dev->i2c_dev, 0x45, 0x26);
    i2cdevWriteByte(dev->i2c_dev, 0x46, 0x05);
    i2cdevWriteByte(dev->i2c_dev, 0x40, 0x40);
    i2cdevWriteByte(dev->i2c_dev, 0x0E, 0x06);
    i2cdevWriteByte(dev->i2c_dev, 0x20, 0x1A);
    i2cdevWriteByte(dev->i2c_dev, 0x43, 0x40);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x34, 0x03);
    i2cdevWriteByte(dev->i2c_dev, 0x35, 0x44);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x31, 0x04);
    i2cdevWriteByte(dev->i2c_dev, 0x4B, 0x09);
    i2cdevWriteByte(dev->i2c_dev, 0x4C, 0x05);
    i2cdevWriteByte(dev->i2c_dev, 0x4D, 0x04);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x44, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x45, 0x20);
    i2cdevWriteByte(dev->i2c_dev, 0x47, 0x08);
    i2cdevWriteByte(dev->i2c_dev, 0x48, 0x28);
    i2cdevWriteByte(dev->i2c_dev, 0x67, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x70, 0x04);
    i2cdevWriteByte(dev->i2c_dev, 0x71, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x72, 0xFE);
    i2cdevWriteByte(dev->i2c_dev, 0x76, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x77, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x0D, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x80, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x01, 0xF8);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x8E, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x00, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x80, 0x00);

    // Interrupt config
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    i2cdevWriteBit(dev->i2c_dev, VL53L0X_RA_GPIO_HV_MUX_ACTIVE_HIGH, 4, 0);
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR, 0x01);

    dev->measurement_timing_budget_us = vl53l0xGetMeasurementTimingBudget(dev);
    dev->measurement_timing_budget_ms = (uint16_t)(dev->measurement_timing_budget_us / 1000.0f);

    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0xE8);
    vl53l0xSetMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    // Perform Calibration
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!vl53l0xPerformSingleRefCalibration(dev, 0x40))
    {
        DEBUG_PRINTD("Failed VHV calibration");
        return false;
    }

    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!vl53l0xPerformSingleRefCalibration(dev, 0x00))
    {
        DEBUG_PRINTD("Failed phase calibration");
        return false;
    }

    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0xE8);
    return true;
}

bool vl53l0xSetSignalRateLimit(VL53L0xDev *dev, float limit_Mcps)
{
    if (limit_Mcps < 0 || limit_Mcps > 511.99f)
    {
        return false;
    }
    uint16_t fixed_pt = (uint16_t)(limit_Mcps * (1 << 7));
    return vl53l0xWriteReg16Bit(dev, VL53L0X_RA_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, fixed_pt);
}

bool vl53l0xSetMeasurementTimingBudget(VL53L0xDev *dev, uint32_t budget_us)
{
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead = 1320;
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;
    uint32_t const MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget)
    {
        return false;
    }

    uint32_t used_budget_us = StartOverhead + EndOverhead;
    vl53l0xGetSequenceStepEnables(dev, &enables);
    vl53l0xGetSequenceStepTimeouts(dev, &enables, &timeouts);

    if (enables.tcc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }
    if (enables.dss)
    {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }
    if (enables.pre_range)
    {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range)
    {
        used_budget_us += FinalRangeOverhead;

        if (used_budget_us > budget_us)
        {
            return false;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;
        uint16_t final_range_timeout_mclks =
            vl53l0xTimeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
        {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        uint16_t temp = vl53l0xEncodeTimeout(final_range_timeout_mclks);
        vl53l0xWriteReg16Bit(dev, VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, temp);

        dev->measurement_timing_budget_us = budget_us;
        dev->measurement_timing_budget_ms = (uint16_t)(dev->measurement_timing_budget_us / 1000.0f);
    }
    return true;
}

uint32_t vl53l0xGetMeasurementTimingBudget(VL53L0xDev *dev)
{
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead = 1910;
    uint16_t const EndOverhead = 960;
    uint16_t const MsrcOverhead = 660;
    uint16_t const TccOverhead = 590;
    uint16_t const DssOverhead = 690;
    uint16_t const PreRangeOverhead = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t budget_us = StartOverhead + EndOverhead;

    vl53l0xGetSequenceStepEnables(dev, &enables);
    vl53l0xGetSequenceStepTimeouts(dev, &enables, &timeouts);

    if (enables.tcc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }
    if (enables.dss)
    {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    }
    else if (enables.msrc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }
    if (enables.pre_range)
    {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }
    if (enables.final_range)
    {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    dev->measurement_timing_budget_us = budget_us;
    dev->measurement_timing_budget_ms = (uint16_t)(dev->measurement_timing_budget_us / 1000.0f);
    return budget_us;
}

bool vl53l0xSetVcselPulsePeriod(VL53L0xDev *dev, vcselPeriodType type, uint8_t period_pclks)
{
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    vl53l0xGetSequenceStepEnables(dev, &enables);
    vl53l0xGetSequenceStepTimeouts(dev, &enables, &timeouts);

    if (type == VcselPeriodPreRange)
    {
        switch (period_pclks)
        {
        case 12:
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
            break;
        case 14:
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
            break;
        case 16:
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
            break;
        case 18:
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
            break;
        default:
            return false;
        }

        i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        uint16_t new_pre_range_timeout_mclks = vl53l0xTimeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
        uint16_t new_pre_range_timeout_encoded = vl53l0xEncodeTimeout(new_pre_range_timeout_mclks);
        vl53l0xWriteReg16Bit(dev, VL53L0X_RA_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_pre_range_timeout_encoded);

        uint16_t new_msrc_timeout_mclks = vl53l0xTimeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
        i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_MSRC_CONFIG_TIMEOUT_MACROP,
                        (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
    }
    else if (type == VcselPeriodFinalRange)
    {
        switch (period_pclks)
        {
        case 8:
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
            i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_ALGO_PHASECAL_LIM, 0x30);
            i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
            break;
        case 10:
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
            i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_ALGO_PHASECAL_LIM, 0x20);
            i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
            break;
        case 12:
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
            i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_ALGO_PHASECAL_LIM, 0x20);
            i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
            break;
        case 14:
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
            i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
            i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_ALGO_PHASECAL_LIM, 0x20);
            i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
            break;
        default:
            return false;
        }

        i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        uint16_t new_final_range_timeout_mclks = vl53l0xTimeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
        if (enables.pre_range)
        {
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }
        uint16_t new_final_range_timeout_encoded = vl53l0xEncodeTimeout(new_final_range_timeout_mclks);
        vl53l0xWriteReg16Bit(dev, VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, new_final_range_timeout_encoded);
    }
    else
    {
        return false;
    }

    vl53l0xSetMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    uint8_t sequence_config = 0;
    i2cdevReadByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, &sequence_config);
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, 0x02);
    bool ret = vl53l0xPerformSingleRefCalibration(dev, 0x0);
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, sequence_config);

    return ret;
}

uint8_t vl53l0xGetVcselPulsePeriod(VL53L0xDev *dev, vcselPeriodType type)
{
    uint8_t temp = 0;
    if (type == VcselPeriodPreRange)
    {
        i2cdevReadByte(dev->i2c_dev, VL53L0X_RA_PRE_RANGE_CONFIG_VCSEL_PERIOD, &temp);
        return decodeVcselPeriod(temp);
    }
    else if (type == VcselPeriodFinalRange)
    {
        i2cdevReadByte(dev->i2c_dev, VL53L0X_RA_FINAL_RANGE_CONFIG_VCSEL_PERIOD, &temp);
        return decodeVcselPeriod(temp);
    }
    return 255;
}

void vl53l0xStartContinuous(VL53L0xDev *dev, uint32_t period_ms)
{
    i2cdevWriteByte(dev->i2c_dev, 0x80, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x00, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x91, dev->stop_variable);
    i2cdevWriteByte(dev->i2c_dev, 0x00, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x80, 0x00);

    if (period_ms != 0)
    {
        uint16_t osc_calibrate_val = vl53l0xReadReg16Bit(dev, VL53L0X_RA_OSC_CALIBRATE_VAL);
        if (osc_calibrate_val != 0)
        {
            period_ms *= osc_calibrate_val;
        }
        vl53l0xWriteReg32Bit(dev, VL53L0X_RA_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
        i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSRANGE_START, 0x04);
    }
    else
    {
        i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSRANGE_START, 0x02);
    }
}

void vl53l0xStopContinuous(VL53L0xDev *dev)
{
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSRANGE_START, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x00, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x91, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x00, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
}

uint16_t vl53l0xReadRangeContinuousMillimeters(VL53L0xDev *dev)
{
    startTimeout();
    uint8_t val = 0;

    while ((val & 0x07) == 0)
    {
        i2cdevReadByte(dev->i2c_dev, VL53L0X_RA_RESULT_INTERRUPT_STATUS, &val);

        if ((val & 0x07) == 0)
        {
            vTaskDelay(M2T(1));
        }

        if (checkTimeoutExpired())
        {
            dev->did_timeout = true;
            return 65535;
        }
    }

    uint16_t range = vl53l0xReadReg16Bit(dev, VL53L0X_RA_RESULT_RANGE_STATUS + 10);
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR, 0x01);
    return range;
}

uint16_t vl53l0xReadRangeSingleMillimeters(VL53L0xDev *dev)
{
    i2cdevWriteByte(dev->i2c_dev, 0x80, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x00, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x91, dev->stop_variable);
    i2cdevWriteByte(dev->i2c_dev, 0x00, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x80, 0x00);

    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSRANGE_START, 0x01);

    startTimeout();
    uint8_t val = 0x01;

    while (val & 0x01)
    {
        i2cdevReadByte(dev->i2c_dev, VL53L0X_RA_SYSRANGE_START, &val);
        if (checkTimeoutExpired())
        {
            dev->did_timeout = true;
            return 65535;
        }
    }

    return vl53l0xReadRangeContinuousMillimeters(dev);
}

static bool vl53l0xGetSpadInfo(VL53L0xDev *dev, uint8_t *count, bool *type_is_aperture)
{
    uint8_t tmp;

    i2cdevWriteByte(dev->i2c_dev, 0x80, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x00, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x06);
    i2cdevWriteBit(dev->i2c_dev, 0x83, 2, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x07);
    i2cdevWriteByte(dev->i2c_dev, 0x81, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x80, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x94, 0x6b);
    i2cdevWriteByte(dev->i2c_dev, 0x83, 0x00);

    startTimeout();
    uint8_t val = 0x00;

    while (val == 0x00)
    {
        i2cdevReadByte(dev->i2c_dev, 0x83, &val);
        if (checkTimeoutExpired())
        {
            return false;
        }
    }

    i2cdevWriteByte(dev->i2c_dev, 0x83, 0x01);
    i2cdevReadByte(dev->i2c_dev, 0x92, &tmp);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    i2cdevWriteByte(dev->i2c_dev, 0x81, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x06);
    i2cdevWriteBit(dev->i2c_dev, 0x83, 2, 0);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0x00, 0x01);
    i2cdevWriteByte(dev->i2c_dev, 0xFF, 0x00);
    i2cdevWriteByte(dev->i2c_dev, 0x80, 0x00);

    return true;
}

static void vl53l0xGetSequenceStepEnables(VL53L0xDev *dev, SequenceStepEnables *enables)
{
    uint8_t sequence_config = 0;
    i2cdevReadByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG, &sequence_config);

    enables->tcc = (sequence_config >> 4) & 0x1;
    enables->dss = (sequence_config >> 3) & 0x1;
    enables->msrc = (sequence_config >> 2) & 0x1;
    enables->pre_range = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;
}

static void vl53l0xGetSequenceStepTimeouts(VL53L0xDev *dev, SequenceStepEnables const *enables, SequenceStepTimeouts *timeouts)
{
    timeouts->pre_range_vcsel_period_pclks = vl53l0xGetVcselPulsePeriod(dev, VcselPeriodPreRange);

    uint8_t temp = 0;
    i2cdevReadByte(dev->i2c_dev, VL53L0X_RA_MSRC_CONFIG_TIMEOUT_MACROP, &temp);
    timeouts->msrc_dss_tcc_mclks = temp + 1;
    timeouts->msrc_dss_tcc_us = vl53l0xTimeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

    uint16_t pre_range_encoded = vl53l0xReadReg16Bit(dev, VL53L0X_RA_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI);
    timeouts->pre_range_mclks = vl53l0xDecodeTimeout(pre_range_encoded);
    timeouts->pre_range_us = vl53l0xTimeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = vl53l0xGetVcselPulsePeriod(dev, VcselPeriodFinalRange);

    uint16_t final_range_encoded = vl53l0xReadReg16Bit(dev, VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI);
    timeouts->final_range_mclks = vl53l0xDecodeTimeout(final_range_encoded);

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = vl53l0xTimeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

static uint16_t vl53l0xDecodeTimeout(uint16_t reg_val)
{
    return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t vl53l0xEncodeTimeout(uint16_t timeout_mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00LU) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else
    {
        return 0;
    }
}

static uint32_t vl53l0xTimeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t vl53l0xTimeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

static bool vl53l0xPerformSingleRefCalibration(VL53L0xDev *dev, uint8_t vhv_init_byte)
{
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSRANGE_START, 0x01 | vhv_init_byte);

    startTimeout();
    uint8_t temp = 0x00;

    while ((temp & 0x07) == 0)
    {
        i2cdevReadByte(dev->i2c_dev, VL53L0X_RA_RESULT_INTERRUPT_STATUS, &temp);
        if (checkTimeoutExpired())
        {
            return false;
        }
    }

    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR, 0x01);
    i2cdevWriteByte(dev->i2c_dev, VL53L0X_RA_SYSRANGE_START, 0x00);
    return true;
}

uint16_t vl53l0xReadReg16Bit(VL53L0xDev *dev, uint8_t reg)
{
    uint8_t buffer[2] = {0};
    i2cdevReadReg8(dev->i2c_dev, reg, 2, buffer);
    return ((uint16_t)(buffer[0]) << 8) | buffer[1];
}

static bool vl53l0xWriteReg16Bit(VL53L0xDev *dev, uint8_t reg, uint16_t val)
{
    uint8_t buffer[2] = {((val >> 8) & 0xFF), (val & 0xFF)};
    return i2cdevWriteReg8(dev->i2c_dev, reg, 2, buffer);
}

static bool vl53l0xWriteReg32Bit(VL53L0xDev *dev, uint8_t reg, uint32_t val)
{
    uint8_t buffer[4] = {((val >> 24) & 0xFF), ((val >> 16) & 0xFF), ((val >> 8) & 0xFF), (val & 0xFF)};
    return i2cdevWriteReg8(dev->i2c_dev, reg, 4, buffer);
}