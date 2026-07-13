#include "ch.h"
#include "hal.h"

#define MAVLINK_COMM_NUM_BUFFERS 1
#include "common/mavlink.h"

#define OWN_SYSID   100
#define OWN_COMPID  MAV_COMP_ID_ONBOARD_COMPUTER
#define FC_SYSID    1
#define FC_COMPID   1

#define LED_LINE   PAL_LINE(GPIOC, 13U)
#define LED_ON()   palClearLine(LED_LINE)
#define LED_OFF()  palSetLine(LED_LINE)

static const SerialConfig sd1cfg = { 57600, 0, 0, 0 };

static void mav_send(const mavlink_message_t *msg) {
    static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    sdWrite(&SD1, buf, len);
}

static void send_arm_disarm(bool arm) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(OWN_SYSID, OWN_COMPID, &msg,
        FC_SYSID, FC_COMPID,
        MAV_CMD_COMPONENT_ARM_DISARM, 0,
        arm ? 1.0f : 0.0f,   /* param1: 1=arm, 0=disarm */
        0, 0, 0, 0, 0, 0);
    mav_send(&msg);
}

int main(void) {
    halInit();
    chSysInit();

    palSetLineMode(LED_LINE, PAL_MODE_OUTPUT_PUSHPULL);
    LED_OFF();

    palSetPadMode(GPIOA, 9,  PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
    sdStart(&SD1, &sd1cfg);

    /* 5 blinks */
    for (int i = 0; i < 5; i++) {
        LED_ON();  chThdSleepMilliseconds(200);
        LED_OFF(); chThdSleepMilliseconds(200);
    }

    /* Arm */
    send_arm_disarm(true);
    LED_ON();                          /* solid = "armed" for the test */
    chThdSleepMilliseconds(5000);      /* stay armed 5s so you can see it in MP */

    /* Disarm */
    send_arm_disarm(false);
    LED_OFF();

    while (true) {
        chThdSleepMilliseconds(1000);   /* idle forever */
    }
}
