/*
 * STEP 1: Simple ChibiOS LED blink
 * Board: STM32F401RCT6 minimum system board, PC13 onboard LED (active-low)
 * Crystal: 25 MHz HSE
 *
 * This is the minimal, known-good blink you already validated.
 * Use this file as main.c to confirm toolchain + clock + pin before
 * moving to the MAVLink bridge in step 2.
 */

#include "ch.h"
#include "hal.h"

#define LED_LINE   PAL_LINE(GPIOC, 13U)

int main(void) {
    halInit();
    chSysInit();

    palSetLineMode(LED_LINE, PAL_MODE_OUTPUT_PUSHPULL);

    while (true) {
        palClearLine(LED_LINE);   /* LED ON  (active-low) */
        chThdSleepMilliseconds(500);

        palSetLine(LED_LINE);     /* LED OFF */
        chThdSleepMilliseconds(500);
    }
}
