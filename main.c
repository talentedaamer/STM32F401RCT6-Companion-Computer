/*
 * STEP 2: main.c - USART1 (PA9/PA10) to flight controller, MAVLink parsing,
 * LED status indicator (PC13).
 *
 * Wiring:
 *   FC TELEM TX -> PA10 (RX1)
 *   FC TELEM RX -> PA9  (TX1)
 *   FC GND      -> GND
 * Match sd1cfg baud below to your FC's telemetry port setting
 * (57600 is common ArduPilot default; some run 115200/921600).
 */

#include "ch.h"
#include "hal.h"
#include "mavlink_bridge.h"
#include "config.h"

#define LED_ON()   palClearLine(LED_LINE)   /* active-low */
#define LED_OFF()  palSetLine(LED_LINE)

static const SerialConfig sd1cfg = {
    FC_TELEM_BAUD,   /* baud rate - match your FC telemetry port, see config.h */
    0,       /* CR1 */
    0,       /* CR2 */
    0        /* CR3 */
};

/* --- MAVLink RX/logic thread --- */
/* static THD_WORKING_AREA(waMavRx, 512);*/
static THD_WORKING_AREA(waMavRx, 1024);
static THD_FUNCTION(MavRxThread, arg) {
    (void)arg;
    mavlink_bridge_rx_thread_func();   /* never returns */
}

/* --- LED status thread: runs independently so blink timing never
 *     stutters if the MAVLink thread is busy parsing a burst --- */
/* static THD_WORKING_AREA(waLed, 128); */
static THD_WORKING_AREA(waLed, 256); 
static THD_FUNCTION(LedThread, arg) {
    (void)arg;
    while (true) {
        switch (mavlink_bridge_get_led_state()) {
        case LED_STATE_DISCONNECTED:
            LED_OFF(); chThdSleepMilliseconds(200); break;
        case LED_STATE_CONNECTED_BLINK:
            LED_ON();  chThdSleepMilliseconds(100);
            LED_OFF(); chThdSleepMilliseconds(900); break;
        case LED_STATE_REDIRECT_ACTIVE:
            LED_ON();  chThdSleepMilliseconds(80);
            LED_OFF(); chThdSleepMilliseconds(80); break;
        }
    }
}

int main(void) {
    halInit();
    chSysInit();

    /* LED */
    palSetLineMode(LED_LINE, PAL_MODE_OUTPUT_PUSHPULL);
    LED_OFF();   /* start dark until first heartbeat arrives */

    /* Power-on confirmation: blink 3 times, then go dark until
     * the normal MAVLink-driven LED pattern takes over */
    for (int i = 0; i < 3; i++) {
        LED_ON();
        chThdSleepMilliseconds(150);
        LED_OFF();
        chThdSleepMilliseconds(150);
    }

    /* USART1: PA9 = TX1, PA10 = RX1, alternate function 7 */
    palSetPadMode(GPIOA, 9,  PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));

    sdStart(&SD1, &sd1cfg);

    mavlink_bridge_init(&SD1);

    chThdCreateStatic(waMavRx, sizeof(waMavRx), NORMALPRIO, MavRxThread, NULL);
    chThdCreateStatic(waLed,   sizeof(waLed),   NORMALPRIO, LedThread,   NULL);

    systime_t last_hb = chVTGetSystemTimeX();
    while (true) {
        if (chVTTimeElapsedSinceX(last_hb) >= TIME_MS2I(1000)) {
            mavlink_bridge_heartbeat_tick();
            last_hb = chVTGetSystemTimeX();
        }
        chThdSleepMilliseconds(50);
    }
}
