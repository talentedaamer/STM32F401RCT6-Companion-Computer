/*
 * main.c - VL53L0X (GY-530) ToF distance sensor over I2C1, forwarded to the
 * flight controller as a MAVLink DISTANCE_SENSOR (front-facing) message,
 * plus an LED blink-rate indicator on PC13.
 *
 * Wiring:
 *   VL53L0X VCC -> 3V3, GND -> GND, SCL -> PB8 (I2C1_SCL, AF4), SDA -> PB9 (I2C1_SDA, AF4)
 *   FC TELEM TX -> PA10 (RX1), FC TELEM RX -> PA9 (TX1), FC GND -> GND
 *   Match sd1cfg baud to the FC telemetry port's SERIALx_BAUD.
 *
 * LED behavior (FAR_THRESHOLD_MM / NEAR_THRESHOLD_MM below - edit to retune):
 *   distance  > FAR_THRESHOLD_MM         -> off    (nothing in range)
 *   NEAR_THRESHOLD_MM < distance <= FAR  -> slow blink
 *   distance <= NEAR_THRESHOLD_MM        -> fast blink
 *   sensor never answered at startup     -> fast triple-blink, repeating
 *
 * The previous main.c (MAVLink RTL-redirect + dummy obstacle avoidance) is
 * preserved in _main_mavlink_obstacle.c, not compiled - see CLAUDE.md.
 */
#include "ch.h"
#include "hal.h"
#include "vl53l0x.h"
#include "sensor_distance.h"

#define LED_LINE   PAL_LINE(GPIOC, 13U)
#define LED_ON()   palClearLine(LED_LINE)   /* active-low */
#define LED_OFF()  palSetLine(LED_LINE)

#define FAR_THRESHOLD_MM     1000   /* beyond this: LED off */
#define NEAR_THRESHOLD_MM     300   /* at/below this: fast blink */

static const SerialConfig sd1cfg = {
    921600,  /* baud rate - match your FC telemetry port's SERIALx_BAUD */
    0, 0, 0
};

static const I2CConfig i2cfg = {
    OPMODE_I2C,
    100000,          /* 100kHz standard mode */
    STD_DUTY_CYCLE
};

static bool sensor_ok = false;
/* Written by SensorThread, read by LedThread - a lone aligned 16-bit
 * read/write is atomic on Cortex-M4, so no lock needed for this. */
static volatile uint16_t last_distance_mm = 8190;

/* --- Sensor thread: reads the VL53L0X as fast as it produces continuous-
 *     mode samples and forwards every reading to the FC immediately, so
 *     the MAVLink update rate is never throttled by the LED's blink
 *     timing --- */
static THD_WORKING_AREA(waSensor, 1024);
static THD_FUNCTION(SensorThread, arg) {
    (void)arg;
    while (true) {
        uint16_t range_mm = vl53l0x_read_range_mm();
        last_distance_mm = range_mm;
        sensor_distance_send(range_mm);
    }
}

/* --- LED status thread: runs independently of sensor/MAVLink timing --- */
static THD_WORKING_AREA(waLed, 256);
static THD_FUNCTION(LedThread, arg) {
    (void)arg;
    while (true) {
        if (!sensor_ok) {
            for (int i = 0; i < 3; i++) {
                LED_ON();  chThdSleepMilliseconds(60);
                LED_OFF(); chThdSleepMilliseconds(60);
            }
            chThdSleepMilliseconds(600);
            continue;
        }

        uint16_t distance_mm = last_distance_mm;
        if (distance_mm > FAR_THRESHOLD_MM) {
            LED_OFF();
            chThdSleepMilliseconds(100);
        } else if (distance_mm > NEAR_THRESHOLD_MM) {
            LED_ON();  chThdSleepMilliseconds(300);
            LED_OFF(); chThdSleepMilliseconds(300);
        } else {
            LED_ON();  chThdSleepMilliseconds(80);
            LED_OFF(); chThdSleepMilliseconds(80);
        }
    }
}

int main(void) {
    halInit();
    chSysInit();

    palSetLineMode(LED_LINE, PAL_MODE_OUTPUT_PUSHPULL);
    LED_OFF();

    /* Power-on confirmation: blink 3 times, independent of sensor state */
    for (int i = 0; i < 3; i++) {
        LED_ON();  chThdSleepMilliseconds(150);
        LED_OFF(); chThdSleepMilliseconds(150);
    }

    /* USART1: PA9 = TX1, PA10 = RX1, alternate function 7 - to the FC */
    palSetPadMode(GPIOA, 9,  PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
    sdStart(&SD1, &sd1cfg);
    sensor_distance_init(&SD1);

    /* I2C1: PB8 = SCL, PB9 = SDA, alternate function 4, open-drain + pull-up */
    palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUPDR_PULLUP);
    palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUPDR_PULLUP);
    i2cStart(&I2CD1, &i2cfg);

    sensor_ok = vl53l0x_init(&I2CD1);
    if (sensor_ok) {
        vl53l0x_start_continuous();
    }

    chThdCreateStatic(waLed, sizeof(waLed), NORMALPRIO, LedThread, NULL);
    if (sensor_ok) {
        chThdCreateStatic(waSensor, sizeof(waSensor), NORMALPRIO, SensorThread, NULL);
    }

    systime_t last_hb = chVTGetSystemTimeX();
    while (true) {
        if (chVTTimeElapsedSinceX(last_hb) >= TIME_MS2I(1000)) {
            sensor_distance_send_heartbeat();
            last_hb = chVTGetSystemTimeX();
        }
        chThdSleepMilliseconds(50);
    }
}
