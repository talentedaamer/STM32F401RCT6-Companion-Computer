#ifndef MAVLINK_BRIDGE_H
#define MAVLINK_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"

/* Only one UART channel talking MAVLink -> smaller static footprint */
#define MAVLINK_COMM_NUM_BUFFERS 1
#include "common/mavlink.h"

/* Our identity as a companion / redirection node */
#define OWN_SYSID   100
#define OWN_COMPID  MAV_COMP_ID_ONBOARD_COMPUTER

/* ArduPilot Copter custom_mode numbers (differs for PX4 - verify against
 * a live HEARTBEAT dump before trusting this on your specific FC/firmware) */
#define AP_COPTER_MODE_GUIDED  4
#define AP_COPTER_MODE_RTL     6

/* --- Hardcoded redirect target (replace with telemetry-fed value later) --- */
#define NEW_LANDING_LAT_DEG   12.9716000
#define NEW_LANDING_LON_DEG   77.5946000
#define NEW_LANDING_ALT_M     0.0f          /* 0 = ground, relative frame */
#define ALT_TRIGGER_M         30.0f

typedef enum {
    LAND_STATE_IDLE = 0,
    LAND_STATE_RTL_ACTIVE,
    LAND_STATE_REDIRECTED
} landing_state_t;

typedef enum {
    LED_STATE_DISCONNECTED = 0,
    LED_STATE_CONNECTED_BLINK,
    LED_STATE_RTL_SOLID,
    LED_STATE_REDIRECTED_FASTBLINK
} led_status_t;

/* Call once at startup, after sdStart() on the UART you're using */
void mavlink_bridge_init(SerialDriver *sd);

/* Blocking loop - run this as the body of a dedicated thread, never returns */
void mavlink_bridge_rx_thread_func(void);

/* Call once per second (e.g. from your main loop) to emit our own heartbeat */
void mavlink_bridge_heartbeat_tick(void);

/* Poll from your LED thread to decide blink pattern */
led_status_t mavlink_bridge_get_led_state(void);

#endif
