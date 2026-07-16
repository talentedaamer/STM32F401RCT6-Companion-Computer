#include "mavlink_bridge.h"
#include <string.h>

static SerialDriver *fc_sd;

typedef struct {
    bool        have_heartbeat;
    uint8_t     fc_sysid;
    uint8_t     fc_compid;
    uint32_t    custom_mode;
    int32_t     relative_alt_mm;
    bool        have_position;
    landing_state_t land_state;
    systime_t   last_heartbeat_rx;
} fc_telemetry_t;

static fc_telemetry_t fc;

#define HEARTBEAT_TIMEOUT_MS 3000

/* ---- outgoing helpers ---- */
static void mav_send(const mavlink_message_t *msg) {
    static uint8_t buf[MAVLINK_MAX_PACKET_LEN];   /* was a local stack array */
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

static void request_position_stream(void) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(OWN_SYSID, OWN_COMPID, &msg,
        fc.fc_sysid, fc.fc_compid,
        MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 250000, /* 250ms = 4Hz */
        0, 0, 0, 0, 0);
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

/* ---- incoming message handling ---- */
static void handle_heartbeat(const mavlink_message_t *m) {
    fc.last_heartbeat_rx = chVTGetSystemTimeX();

    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(m, &hb);

    if (!fc.have_heartbeat) {
        fc.fc_sysid  = m->sysid;
        fc.fc_compid = m->compid;
        fc.have_heartbeat = true;
        request_position_stream();
    }
    fc.custom_mode = hb.custom_mode;

    if (fc.custom_mode == AP_COPTER_MODE_RTL &&
        fc.land_state == LAND_STATE_IDLE) {
        fc.land_state = LAND_STATE_RTL_ACTIVE;
    } else if (fc.custom_mode != AP_COPTER_MODE_RTL &&
               fc.land_state == LAND_STATE_RTL_ACTIVE) {
        fc.land_state = LAND_STATE_IDLE;   /* left RTL before trigger altitude */
    }
}

static void handle_global_position_int(const mavlink_message_t *m) {
    mavlink_global_position_int_t pos;
    mavlink_msg_global_position_int_decode(m, &pos);
    fc.relative_alt_mm = pos.relative_alt;
    fc.have_position = true;
}

static void dispatch(const mavlink_message_t *m) {
    switch (m->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:           handle_heartbeat(m); break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  handle_global_position_int(m); break;
        default: break;
    }
}

static void evaluate_redirect(void) {
    if (fc.land_state == LAND_STATE_RTL_ACTIVE && fc.have_position) {
        float alt_m = fc.relative_alt_mm / 1000.0f;
        if (alt_m <= ALT_TRIGGER_M) {
            set_guided_mode();
            chThdSleepMilliseconds(50);   /* let mode switch land before command */
            send_redirect_land();
            fc.land_state = LAND_STATE_REDIRECTED;
        }
    }
}

/* ---- public API ---- */
void mavlink_bridge_init(SerialDriver *sd) {
    fc_sd = sd;
    memset(&fc, 0, sizeof(fc));
    fc.last_heartbeat_rx = chVTGetSystemTimeX();  /* starts "disconnected", not stale-true */
}

void mavlink_bridge_heartbeat_tick(void) {
    send_own_heartbeat();
}

led_status_t mavlink_bridge_get_led_state(void) {
    if (chVTTimeElapsedSinceX(fc.last_heartbeat_rx) > TIME_MS2I(HEARTBEAT_TIMEOUT_MS)) {
        return LED_STATE_DISCONNECTED;
    }
    if (fc.land_state == LAND_STATE_REDIRECTED) {
        return LED_STATE_REDIRECTED_FASTBLINK;
    }
    if (fc.land_state == LAND_STATE_RTL_ACTIVE) {
        return LED_STATE_RTL_SOLID;
    }
    return LED_STATE_CONNECTED_BLINK;
}

bool mavlink_bridge_is_connected(void) {
    return fc.have_heartbeat;
}

/* Blocking RX loop - runs forever inside its own thread.
 * sdGetTimeout() pulls one byte at a time from ChibiOS's serial input
 * queue (which is already interrupt-fed internally by the driver) - no
 * manual ring buffer or RX ISR needed, unlike the HAL version. */
void mavlink_bridge_rx_thread_func(void) {
    static mavlink_message_t msg;      /* was local - now static */
    static mavlink_status_t status;    /* was local - now static */

    while (true) {
        msg_t c = sdGetTimeout(fc_sd, TIME_MS2I(100));
        if (c >= MSG_OK) {
            if (mavlink_parse_char(MAVLINK_COMM_0, (uint8_t)c, &msg, &status)) {
                dispatch(&msg);
            }
        }
        evaluate_redirect();
    }
}
