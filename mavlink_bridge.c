#include "mavlink_bridge.h"
#include <string.h>

static SerialDriver *fc_sd;

typedef struct {
    bool     have_heartbeat;
    uint8_t  fc_sysid;
    uint8_t  fc_compid;
    uint32_t custom_mode;
    landing_state_t land_state;
    systime_t last_heartbeat_rx;
} fc_telemetry_t;

static fc_telemetry_t fc;

#define HEARTBEAT_TIMEOUT_MS 3000

static void mav_send(const mavlink_message_t *msg) {
    static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    sdWrite(fc_sd, buf, len);
}

static void send_own_heartbeat(void) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(OWN_SYSID, OWN_COMPID, &msg,
        MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE);
    mav_send(&msg);
}

static void set_guided_mode(void) {
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(OWN_SYSID, OWN_COMPID, &msg,
        fc.fc_sysid,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        AP_COPTER_MODE_GUIDED);
    mav_send(&msg);
}

static void send_redirect_land(void) {
    mavlink_message_t msg;
    int32_t lat_e7 = (int32_t)(NEW_LANDING_LAT_DEG * 1e7);
    int32_t lon_e7 = (int32_t)(NEW_LANDING_LON_DEG * 1e7);

    mavlink_msg_command_int_pack(OWN_SYSID, OWN_COMPID, &msg,
        fc.fc_sysid, fc.fc_compid,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MAV_CMD_NAV_LAND,
        0, 0,
        0, 0, 0, 0,
        lat_e7, lon_e7, NEW_LANDING_ALT_M);
    mav_send(&msg);
}

static void handle_heartbeat(const mavlink_message_t *m) {
    fc.last_heartbeat_rx = chVTGetSystemTimeX();

    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(m, &hb);

    if (!fc.have_heartbeat) {
        fc.fc_sysid  = m->sysid;
        fc.fc_compid = m->compid;
        fc.have_heartbeat = true;
    }
    fc.custom_mode = hb.custom_mode;

    switch (fc.land_state) {
    case LAND_STATE_IDLE:
        if (fc.custom_mode == AP_COPTER_MODE_RTL) {
            /* Immediate redirect - no altitude wait, no matter
             * what height RTL was engaged at. */
            set_guided_mode();
            chThdSleepMilliseconds(50);
            send_redirect_land();
            fc.land_state = LAND_STATE_REDIRECTED;
        }
        break;

    case LAND_STATE_REDIRECTED:
        /* We commanded GUIDED ourselves, so seeing GUIDED here is
         * expected - not a cancel. Anything else (e.g. pilot flips
         * to LOITER on the transmitter) means the pilot took back
         * control - stand down immediately and stop touching the FC. */
        if (fc.custom_mode != AP_COPTER_MODE_GUIDED) {
            fc.land_state = LAND_STATE_IDLE;
        }
        break;
    }
}

static void dispatch(const mavlink_message_t *m) {
    if (m->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        handle_heartbeat(m);
    }
}

void mavlink_bridge_init(SerialDriver *sd) {
    fc_sd = sd;
    memset(&fc, 0, sizeof(fc));
    fc.last_heartbeat_rx = chVTGetSystemTimeX();
}

void mavlink_bridge_heartbeat_tick(void) {
    send_own_heartbeat();
}

led_status_t mavlink_bridge_get_led_state(void) {
    if (chVTTimeElapsedSinceX(fc.last_heartbeat_rx) > TIME_MS2I(HEARTBEAT_TIMEOUT_MS)) {
        return LED_STATE_DISCONNECTED;
    }
    if (fc.land_state == LAND_STATE_REDIRECTED) {
        return LED_STATE_REDIRECT_ACTIVE;
    }
    return LED_STATE_CONNECTED_BLINK;
}

void mavlink_bridge_rx_thread_func(void) {
    static mavlink_message_t msg;
    static mavlink_status_t status;

    while (true) {
        msg_t c = sdGetTimeout(fc_sd, TIME_MS2I(100));
        if (c >= MSG_OK) {
            if (mavlink_parse_char(MAVLINK_COMM_0, (uint8_t)c, &msg, &status)) {
                dispatch(&msg);
            }
        }
    }
}