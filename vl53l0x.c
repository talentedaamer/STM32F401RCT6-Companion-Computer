#include "vl53l0x.h"
#include <string.h>

/* ST's standard VL53L0X register map - same names/addresses used by every
 * open-source minimal port of this sensor's init sequence. */
#define REG_SYSRANGE_START                              0x00
#define REG_SYSTEM_SEQUENCE_CONFIG                       0x01
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO                 0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR                       0x0B
#define REG_RESULT_INTERRUPT_STATUS                      0x13
#define REG_RESULT_RANGE_STATUS                          0x14
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0             0xB0
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT            0xB6
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD          0x4E
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET             0x4F
#define REG_MSRC_CONFIG_CONTROL                          0x60
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  0x44
#define REG_GPIO_HV_MUX_ACTIVE_HIGH                      0x84
#define REG_IDENTIFICATION_MODEL_ID                      0xC0

#define VL53L0X_EXPECTED_MODEL_ID                        0xEE

static I2CDriver *dev;
static uint8_t stop_variable;

/* ---- low-level register access ---- */

static bool write_reg8(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2cMasterTransmitTimeout(dev, VL53L0X_I2C_ADDR, buf, 2,
                                     NULL, 0, TIME_MS2I(50)) == MSG_OK;
}

static bool write_reg16(uint8_t reg, uint16_t val) {
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2cMasterTransmitTimeout(dev, VL53L0X_I2C_ADDR, buf, 3,
                                     NULL, 0, TIME_MS2I(50)) == MSG_OK;
}

/* Only ever called with the 6-byte SPAD enable map below. */
static bool write_multi6(uint8_t reg, const uint8_t *src) {
    uint8_t buf[7];
    buf[0] = reg;
    memcpy(&buf[1], src, 6);
    return i2cMasterTransmitTimeout(dev, VL53L0X_I2C_ADDR, buf, 7,
                                     NULL, 0, TIME_MS2I(50)) == MSG_OK;
}

static uint8_t read_reg8(uint8_t reg) {
    uint8_t val = 0;
    i2cMasterTransmitTimeout(dev, VL53L0X_I2C_ADDR, &reg, 1,
                              &val, 1, TIME_MS2I(50));
    return val;
}

static uint16_t read_reg16(uint8_t reg) {
    uint8_t buf[2] = { 0, 0 };
    i2cMasterTransmitTimeout(dev, VL53L0X_I2C_ADDR, &reg, 1,
                              buf, 2, TIME_MS2I(50));
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static bool read_multi6(uint8_t reg, uint8_t *dst) {
    return i2cMasterTransmitTimeout(dev, VL53L0X_I2C_ADDR, &reg, 1,
                                     dst, 6, TIME_MS2I(50)) == MSG_OK;
}

/* Waits for the "new sample ready" interrupt bit. Bounded by a timeout so
 * a flaky sensor/bus can't hang the caller forever. */
static bool wait_for_range_ready(void) {
    systime_t start = chVTGetSystemTimeX();
    while ((read_reg8(REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (chVTTimeElapsedSinceX(start) > TIME_MS2I(500)) {
            return false;
        }
        chThdSleepMilliseconds(1);
    }
    return true;
}

/* Retrieves the factory-calibrated reference SPAD count/type from NVM -
 * needed to build the SPAD enable map used during static init. */
static bool get_spad_info(uint8_t *count, bool *type_is_aperture) {
    write_reg8(0x80, 0x01);
    write_reg8(0xFF, 0x01);
    write_reg8(0x00, 0x00);

    write_reg8(0xFF, 0x06);
    write_reg8(0x83, read_reg8(0x83) | 0x04);
    write_reg8(0xFF, 0x07);
    write_reg8(0x81, 0x01);

    write_reg8(0x80, 0x01);
    write_reg8(0x94, 0x6B);
    write_reg8(0x83, 0x00);

    systime_t start = chVTGetSystemTimeX();
    while (read_reg8(0x83) == 0x00) {
        if (chVTTimeElapsedSinceX(start) > TIME_MS2I(500)) {
            return false;
        }
        chThdSleepMilliseconds(1);
    }
    write_reg8(0x83, 0x01);

    uint8_t tmp = read_reg8(0x92);
    *count = tmp & 0x7F;
    *type_is_aperture = (bool)((tmp >> 7) & 0x01);

    write_reg8(0x81, 0x00);
    write_reg8(0xFF, 0x06);
    write_reg8(0x83, read_reg8(0x83) & (uint8_t)~0x04);
    write_reg8(0xFF, 0x01);
    write_reg8(0x00, 0x01);

    write_reg8(0xFF, 0x00);
    write_reg8(0x80, 0x00);

    return true;
}

static bool perform_single_ref_calibration(uint8_t vhv_init_byte) {
    write_reg8(REG_SYSRANGE_START, 0x01 | vhv_init_byte);
    if (!wait_for_range_ready()) {
        return false;
    }
    write_reg8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    write_reg8(REG_SYSRANGE_START, 0x00);
    return true;
}

/* ST's published "default tuning settings" block - a fixed sequence of
 * register writes with no per-field meaning worth naming individually.
 * Every minimal port of this sensor's driver applies this same block. */
static void load_default_tuning_settings(void) {
    write_reg8(0xFF, 0x01); write_reg8(0x00, 0x00);
    write_reg8(0xFF, 0x00); write_reg8(0x09, 0x00);
    write_reg8(0x10, 0x00); write_reg8(0x11, 0x00);
    write_reg8(0x24, 0x01); write_reg8(0x25, 0xFF);
    write_reg8(0x75, 0x00);
    write_reg8(0xFF, 0x01); write_reg8(0x4E, 0x2C);
    write_reg8(0x48, 0x00); write_reg8(0x30, 0x20);
    write_reg8(0xFF, 0x00); write_reg8(0x30, 0x09);
    write_reg8(0x54, 0x00); write_reg8(0x31, 0x04);
    write_reg8(0x32, 0x03); write_reg8(0x40, 0x83);
    write_reg8(0x46, 0x25); write_reg8(0x60, 0x00);
    write_reg8(0x27, 0x00); write_reg8(0x50, 0x06);
    write_reg8(0x51, 0x00); write_reg8(0x52, 0x96);
    write_reg8(0x56, 0x08); write_reg8(0x57, 0x30);
    write_reg8(0x61, 0x00); write_reg8(0x62, 0x00);
    write_reg8(0x64, 0x00); write_reg8(0x65, 0x00);
    write_reg8(0x66, 0xA0);
    write_reg8(0xFF, 0x01); write_reg8(0x22, 0x32);
    write_reg8(0x47, 0x14); write_reg8(0x49, 0xFF);
    write_reg8(0x4A, 0x00);
    write_reg8(0xFF, 0x00); write_reg8(0x7A, 0x0A);
    write_reg8(0x7B, 0x00); write_reg8(0x78, 0x21);
    write_reg8(0xFF, 0x01); write_reg8(0x23, 0x34);
    write_reg8(0x42, 0x00); write_reg8(0x44, 0xFF);
    write_reg8(0x45, 0x26); write_reg8(0x46, 0x05);
    write_reg8(0x40, 0x40); write_reg8(0x0E, 0x06);
    write_reg8(0x20, 0x1A); write_reg8(0x43, 0x40);
    write_reg8(0xFF, 0x00); write_reg8(0x34, 0x03);
    write_reg8(0x35, 0x44);
    write_reg8(0xFF, 0x01); write_reg8(0x31, 0x04);
    write_reg8(0x4B, 0x09); write_reg8(0x4C, 0x05);
    write_reg8(0x4D, 0x04);
    write_reg8(0xFF, 0x00); write_reg8(0x44, 0x00);
    write_reg8(0x45, 0x20); write_reg8(0x47, 0x08);
    write_reg8(0x48, 0x28); write_reg8(0x67, 0x00);
    write_reg8(0x70, 0x04); write_reg8(0x71, 0x01);
    write_reg8(0x72, 0xFE); write_reg8(0x76, 0x00);
    write_reg8(0x77, 0x00);
    write_reg8(0xFF, 0x01); write_reg8(0x0D, 0x01);
    write_reg8(0xFF, 0x00); write_reg8(0x80, 0x01);
    write_reg8(0x01, 0xF8);
    write_reg8(0xFF, 0x01); write_reg8(0x8E, 0x01);
    write_reg8(0x00, 0x01);
    write_reg8(0xFF, 0x00); write_reg8(0x80, 0x00);
}

bool vl53l0x_init(I2CDriver *i2cp) {
    dev = i2cp;

    if (read_reg8(REG_IDENTIFICATION_MODEL_ID) != VL53L0X_EXPECTED_MODEL_ID) {
        return false;   /* nothing answered at 0x29, or it's not a VL53L0X */
    }

    /* ---- data init ---- */
    write_reg8(0x88, 0x00);   /* 2.8V I/O level, standard I2C mode */
    write_reg8(0x80, 0x01);
    write_reg8(0xFF, 0x01);
    write_reg8(0x00, 0x00);
    stop_variable = read_reg8(0x91);
    write_reg8(0x00, 0x01);
    write_reg8(0xFF, 0x00);
    write_reg8(0x80, 0x00);

    /* disable SIGNAL_RATE_MSRC / SIGNAL_RATE_PRE_RANGE limit checks */
    write_reg8(REG_MSRC_CONFIG_CONTROL, read_reg8(REG_MSRC_CONFIG_CONTROL) | 0x12);

    /* final range signal rate limit = 0.25 MCPS in the sensor's 9.7 fixed-point format */
    write_reg16(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(0.25f * (1 << 7)));

    write_reg8(REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* ---- static init: build the reference SPAD enable map ---- */
    uint8_t spad_count = 0;
    bool spad_type_is_aperture = false;
    if (!get_spad_info(&spad_count, &spad_type_is_aperture)) {
        return false;
    }

    uint8_t ref_spad_map[6];
    read_multi6(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map);

    write_reg8(0xFF, 0x01);
    write_reg8(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    write_reg8(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    write_reg8(0xFF, 0x00);
    write_reg8(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        uint8_t idx = i / 8, bit = i % 8;
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            ref_spad_map[idx] &= (uint8_t)~(1U << bit);
        } else if ((ref_spad_map[idx] >> bit) & 0x01) {
            spads_enabled++;
        }
    }
    write_multi6(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map);

    load_default_tuning_settings();

    /* interrupt on new sample ready, active low */
    write_reg8(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    write_reg8(REG_GPIO_HV_MUX_ACTIVE_HIGH, read_reg8(REG_GPIO_HV_MUX_ACTIVE_HIGH) & (uint8_t)~0x10);
    write_reg8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    write_reg8(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* ---- reference calibration (VHV, then phase) ---- */
    write_reg8(REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!perform_single_ref_calibration(0x40)) {
        return false;
    }
    write_reg8(REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!perform_single_ref_calibration(0x00)) {
        return false;
    }
    write_reg8(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    return true;
}

void vl53l0x_start_continuous(void) {
    write_reg8(0x80, 0x01);
    write_reg8(0xFF, 0x01);
    write_reg8(0x00, 0x00);
    write_reg8(0x91, stop_variable);
    write_reg8(0x00, 0x01);
    write_reg8(0xFF, 0x00);
    write_reg8(0x80, 0x00);

    write_reg8(REG_SYSRANGE_START, 0x02);   /* back-to-back continuous mode */
}

uint16_t vl53l0x_read_range_mm(void) {
    if (!wait_for_range_ready()) {
        return 8190;   /* timed out - report "out of range", not a stale/bogus value */
    }
    uint16_t range_mm = read_reg16(REG_RESULT_RANGE_STATUS + 10);
    write_reg8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    return range_mm;
}
