#include "mavlink_bridge.h"
#include <string.h>
#include <math.h>

#define EARTH_RADIUS_M   6371000.0
#define DEG_TO_RAD       (3.14159265358979323846 / 180.0)

static SerialDriver *fc_sd;

typedef struct {
    bool     have_heartbeat;
    uint8_t  fc_sysid;
    uint8_t  fc_compid;
    uint32_t custom_mode;
    bool     have_position;
    double   cur_lat;
    double   cur_lon;
    int32_t  relative_alt_mm;
    landing_state_t land_state;
    systime_t last_heartbeat_rx;
} fc_telemetry_t;

static fc_telemetry_t fc;

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

static void request_position_stream(void) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(OWN_SYSID, OWN_COMPID, &msg,
        fc.fc_sysid, fc.fc_compid,
        MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000, /* 200ms = 5Hz */
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

/* Step 1: actually fly to the new coordinates, holding current altitude */
static void send_reposition(void) {
    mavlink_message_t msg;
    int32_t lat_e7 = (int32_t)(NEW_LANDING_LAT_DEG * 1e7);
    int32_t lon_e7 = (int32_t)(NEW_LANDING_LON_DEG * 1e7);
    float alt_m = fc.relative_alt_mm / 1000.0f;   /* keep current altitude */

    uint16_t type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE |
                          POSITION_TARGET_TYPEMASK_VY_IGNORE |
                          POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                          POSITION_TARGET_TYPEMASK_AX_IGNORE |
                          POSITION_TARGET_TYPEMASK_AY_IGNORE |
                          POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                          POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                          POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    mavlink_msg_set_position_target_global_int_pack(OWN_SYSID, OWN_COMPID, &msg,
        0, fc.fc_sysid, fc.fc_compid,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, type_mask,
        lat_e7, lon_e7, alt_m,
        0, 0, 0,   /* vx, vy, vz - ignored */
        0, 0, 0,   /* afx, afy, afz - ignored */
        0, 0);     /* yaw, yaw_rate - ignored */
    mav_send(&msg);
}

/* Step 2: land in place (current=1) - only called once we've arrived */
static void send_land_here(void) {
    mavlink_message_t msg;
    mavlink_msg_command_int_pack(OWN_SYSID, OWN_COMPID, &msg,
        fc.fc_sysid, fc.fc_compid,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MAV_CMD_NAV_LAND,
        1, 0,          /* current=1: land at wherever we are RIGHT NOW */
        0, 0, 0, 0,
        0, 0, 0);      /* lat/lon/alt ignored since current=1 */
    mav_send(&msg);
}

static float horizontal_distance_m(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * DEG_TO_RAD;
    double dlon = (lon2 - lon1) * DEG_TO_RAD;
    double mean_lat = ((lat1 + lat2) / 2.0) * DEG_TO_RAD;
    double x = dlon * cos(mean_lat);
    double y = dlat;
    return (float)(sqrt(x * x + y * y) * EARTH_RADIUS_M);
}

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

    switch (fc.land_state) {
    case LAND_STATE_IDLE:
        if (fc.custom_mode == AP_COPTER_MODE_RTL) {
            set_guided_mode();
            chThdSleepMilliseconds(50);
            send_reposition();
            fc.land_state = LAND_STATE_ENROUTE;
        }
        break;

    case LAND_STATE_ENROUTE:
    case LAND_STATE_LANDING:
        /* Pilot took back control if mode is anything other than GUIDED
         * (we commanded GUIDED ourselves, so seeing GUIDED is expected) */
        if (fc.custom_mode != AP_COPTER_MODE_GUIDED) {
            fc.land_state = LAND_STATE_IDLE;
        }
        break;
    }
}

static void handle_global_position_int(const mavlink_message_t *m) {
    mavlink_global_position_int_t pos;
    mavlink_msg_global_position_int_decode(m, &pos);

    fc.cur_lat = pos.lat / 1e7;
    fc.cur_lon = pos.lon / 1e7;
    fc.relative_alt_mm = pos.relative_alt;
    fc.have_position = true;

    if (fc.land_state == LAND_STATE_ENROUTE) {
        float dist = horizontal_distance_m(fc.cur_lat, fc.cur_lon,
                                            NEW_LANDING_LAT_DEG, NEW_LANDING_LON_DEG);
        if (dist <= ARRIVAL_RADIUS_M) {
            send_land_here();
            fc.land_state = LAND_STATE_LANDING;
        }
    }
}

static void dispatch(const mavlink_message_t *m) {
    switch (m->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:           handle_heartbeat(m); break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  handle_global_position_int(m); break;
        default: break;
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
    if (fc.land_state != LAND_STATE_IDLE) {
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
