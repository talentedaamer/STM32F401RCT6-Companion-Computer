/*
 * vl53l0x.h - minimal driver for the ST VL53L0X ToF distance sensor
 * (GY-530 breakout), continuous-ranging mode only, over ChibiOS I2C.
 *
 * This reproduces the well-known minimal init sequence used by most
 * open-source VL53L0X drivers (data init -> SPAD map from NVM -> default
 * tuning register block -> VHV/phase reference calibration), since the
 * sensor won't produce valid ranges without it - there's no shortcut.
 *
 * Wiring (this project): SCL -> PB8, SDA -> PB9, VCC -> 3V3, GND -> GND.
 */
#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"

#define VL53L0X_I2C_ADDR   0x29   /* 7-bit factory default address */

/* Call once after i2cStart() on the bus the sensor is wired to. Runs the
 * full data/static init and reference calibration sequence. Returns false
 * if the sensor never answered (wrong wiring/address) or a calibration
 * step timed out - check this before trusting any reading. */
bool vl53l0x_init(I2CDriver *i2cp);

/* Starts continuous back-to-back ranging. Call once, after a successful
 * vl53l0x_init(). */
void vl53l0x_start_continuous(void);

/* Blocks (with an internal timeout) until the next continuous-mode sample
 * is ready and returns the distance in millimeters. Returns 8190 if the
 * wait timed out or the sensor reports no target in range - treat that the
 * same as "nothing detected". */
uint16_t vl53l0x_read_range_mm(void);

#endif /* VL53L0X_H */
