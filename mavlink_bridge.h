#ifndef MAVLINK_BRIDGE_H
#define MAVLINK_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"

#define MAVLINK_COMM_NUM_BUFFERS 1
#include "common/mavlink.h"

#include "config.h"

typedef enum {
    LAND_STATE_IDLE = 0,
    LAND_STATE_ENROUTE,     /* flying to the new coordinates */
    LAND_STATE_LANDING      /* arrived, NAV_LAND sent */
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
