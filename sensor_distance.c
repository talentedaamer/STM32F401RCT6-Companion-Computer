#include "sensor_distance.h"
#include "mavlink_bridge.h"   /* OWN_SYSID/OWN_COMPID + vendored common/mavlink.h only -
                                 see the note in sensor_distance.h about why the rest of
                                 mavlink_bridge.c is deliberately NOT used here. */

#define FRONT_SENSOR_ID            0
#define FRONT_SENSOR_ORIENTATION   MAV_SENSOR_ROTATION_NONE

/* VL53L0X's own rated range (datasheet: ~30mm to 1200mm). Readings are
 * clamped into this before sending so an out-of-range/timeout result
 * (vl53l0x_read_range_mm() returns 8190) shows up as "at max range" rather
 * than a nonsensical huge distance. */
#define SENSOR_MIN_RANGE_CM        3
#define SENSOR_MAX_RANGE_CM        120

static SerialDriver *fc_sd;
static const float zero_quaternion[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

static void mav_send(const mavlink_message_t *msg) {
    static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    sdWrite(fc_sd, buf, len);
}

void sensor_distance_init(SerialDriver *sd) {
    fc_sd = sd;
}

void sensor_distance_send_heartbeat(void) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(OWN_SYSID, OWN_COMPID, &msg,
        MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE);
    mav_send(&msg);
}

void sensor_distance_send(uint16_t range_mm) {
    uint16_t range_cm = range_mm / 10;
    if (range_cm > SENSOR_MAX_RANGE_CM) {
        range_cm = SENSOR_MAX_RANGE_CM;
    }

    mavlink_message_t msg;
    mavlink_msg_distance_sensor_pack(OWN_SYSID, OWN_COMPID, &msg,
        (uint32_t)TIME_I2MS(chVTGetSystemTimeX()),
        SENSOR_MIN_RANGE_CM, SENSOR_MAX_RANGE_CM, range_cm,
        MAV_DISTANCE_SENSOR_LASER, FRONT_SENSOR_ID, FRONT_SENSOR_ORIENTATION,
        255 /* covariance unknown */,
        0.0f, 0.0f, zero_quaternion, 0 /* signal_quality unknown */);

    mav_send(&msg);
}
