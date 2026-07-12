#ifndef MAVLINK_BRIDGE_H
#define MAVLINK_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"

#define MAVLINK_COMM_NUM_BUFFERS 1
#include "common/mavlink.h"

#define OWN_SYSID   100
#define OWN_COMPID  MAV_COMP_ID_ONBOARD_COMPUTER

/* ArduCopter custom_mode numbers */
#define AP_COPTER_MODE_GUIDED  4
#define AP_COPTER_MODE_LOITER  5
#define AP_COPTER_MODE_RTL     6

/* Hardcoded redirect target */
#define NEW_LANDING_LAT_DEG   33.541991
#define NEW_LANDING_LON_DEG   73.113652
#define NEW_LANDING_ALT_M     0.0f   /* 0 = ground, relative frame */

typedef enum {
    LAND_STATE_IDLE = 0,
    LAND_STATE_REDIRECTED
} landing_state_t;

typedef enum {
    LED_STATE_DISCONNECTED = 0,
    LED_STATE_CONNECTED_BLINK,
    LED_STATE_REDIRECT_ACTIVE
} led_status_t;

void mavlink_bridge_init(SerialDriver *sd);
void mavlink_bridge_rx_thread_func(void);
void mavlink_bridge_heartbeat_tick(void);
led_status_t mavlink_bridge_get_led_state(void);

#endif