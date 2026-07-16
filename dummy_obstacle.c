#include "dummy_obstacle.h"
#include <string.h>

typedef enum {
    SENSOR_STATE_IDLE = 0,     /* no event scheduled - reports "clear" */
    SENSOR_STATE_WAITING,      /* counting down start_delay_s */
    SENSOR_STATE_APPROACHING,  /* ramping far_cm -> near_cm */
    SENSOR_STATE_HOLDING       /* sitting at near_cm */
} obstacle_state_t;

typedef struct {
    obstacle_state_t state;
    uint32_t delay_ticks_left;
    uint32_t hold_ticks_left;
    uint16_t near_cm;
    uint16_t current_cm;
    uint8_t  mav_orientation;
} obstacle_sim_t;

static obstacle_sim_t sim[OBSTACLE_SENSOR_COUNT];
static SerialDriver *obs_sd;
static const float zero_quaternion[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static uint32_t seconds_to_ticks(float seconds) {
    if (seconds <= 0.0f) {
        return 0;
    }
    return (uint32_t)((seconds * 1000.0f / OBSTACLE_TICK_MS) + 0.5f);
}

static uint8_t orientation_for_sensor(obstacle_sensor_t sensor) {
    switch (sensor) {
    case OBSTACLE_SENSOR_FRONT: return MAV_SENSOR_ROTATION_NONE;
    case OBSTACLE_SENSOR_RIGHT: return MAV_SENSOR_ROTATION_YAW_90;
    case OBSTACLE_SENSOR_LEFT:  return MAV_SENSOR_ROTATION_YAW_270;
    default:                    return MAV_SENSOR_ROTATION_NONE;
    }
}

static void send_distance_sensor(obstacle_sensor_t sensor) {
    obstacle_sim_t *s = &sim[sensor];
    mavlink_message_t msg;
    static uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_distance_sensor_pack(OWN_SYSID, OWN_COMPID, &msg,
        (uint32_t)TIME_I2MS(chVTGetSystemTimeX()),
        OBSTACLE_MIN_RANGE_CM, OBSTACLE_MAX_RANGE_CM, s->current_cm,
        MAV_DISTANCE_SENSOR_LASER, (uint8_t)sensor, s->mav_orientation,
        255 /* covariance unknown */,
        0.0f, 0.0f, zero_quaternion, 0 /* signal_quality unknown */);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sdWrite(obs_sd, buf, len);
}

/* Steps one sensor's state machine by one OBSTACLE_TICK_MS tick. */
static void step_sensor(obstacle_sensor_t sensor) {
    obstacle_sim_t *s = &sim[sensor];

    switch (s->state) {
    case SENSOR_STATE_IDLE:
        s->current_cm = OBSTACLE_MAX_RANGE_CM;
        break;

    case SENSOR_STATE_WAITING:
        if (s->delay_ticks_left > 0) {
            s->delay_ticks_left--;
        }
        if (s->delay_ticks_left == 0) {
            s->state = SENSOR_STATE_APPROACHING;
        }
        break;

    case SENSOR_STATE_APPROACHING:
        if (s->current_cm > s->near_cm + OBSTACLE_STEP_CM) {
            s->current_cm -= OBSTACLE_STEP_CM;
        } else {
            s->current_cm = s->near_cm;
            s->state = SENSOR_STATE_HOLDING;
        }
        break;

    case SENSOR_STATE_HOLDING:
        s->current_cm = s->near_cm;
        if (s->hold_ticks_left > 0) {
            s->hold_ticks_left--;
        }
        if (s->hold_ticks_left == 0) {
            s->state = SENSOR_STATE_IDLE;
        }
        break;
    }
}

void dummy_obstacle_init(SerialDriver *sd) {
    obs_sd = sd;
    memset(sim, 0, sizeof(sim));
    for (int i = 0; i < OBSTACLE_SENSOR_COUNT; i++) {
        sim[i].state = SENSOR_STATE_IDLE;
        sim[i].current_cm = OBSTACLE_MAX_RANGE_CM;
        sim[i].mav_orientation = orientation_for_sensor((obstacle_sensor_t)i);
    }
}

void dummy_obstacle_trigger(obstacle_sensor_t sensor,
                             float start_delay_s,
                             float far_distance_m,
                             float near_distance_m,
                             float hold_time_s) {
    if (sensor >= OBSTACLE_SENSOR_COUNT) {
        return;
    }
    obstacle_sim_t *s = &sim[sensor];

    uint16_t far_cm  = (uint16_t)(far_distance_m  * 100.0f);
    uint16_t near_cm = (uint16_t)(near_distance_m * 100.0f);

    s->current_cm       = far_cm;
    s->near_cm           = near_cm;
    s->hold_ticks_left   = seconds_to_ticks(hold_time_s);
    s->delay_ticks_left  = seconds_to_ticks(start_delay_s);
    s->state = (s->delay_ticks_left > 0) ? SENSOR_STATE_WAITING
                                          : SENSOR_STATE_APPROACHING;
}

void dummy_obstacle_tick(void) {
    for (int i = 0; i < OBSTACLE_SENSOR_COUNT; i++) {
        step_sensor((obstacle_sensor_t)i);
        send_distance_sensor((obstacle_sensor_t)i);
    }
}

bool dummy_obstacle_is_detected(void) {
    for (int i = 0; i < OBSTACLE_SENSOR_COUNT; i++) {
        if (sim[i].state == SENSOR_STATE_APPROACHING ||
            sim[i].state == SENSOR_STATE_HOLDING) {
            return true;
        }
    }
    return false;
}
