/*
 * sensor_distance.h - forwards real VL53L0X readings to the flight
 * controller as MAVLink DISTANCE_SENSOR messages (front-facing only).
 *
 * This does NOT reuse mavlink_bridge.c's RX thread/state machine - that
 * module also contains the RTL-redirect-to-hardcoded-GPS logic from the
 * other experiment in this repo, and pulling it in here would make the FC
 * land at a hardcoded coordinate the next time it enters RTL, which has
 * nothing to do with obstacle avoidance and would be a nasty surprise on a
 * real flight. This module only reuses mavlink_bridge.h's OWN_SYSID/
 * OWN_COMPID constants and the vendored common/mavlink.h include - it
 * never calls mavlink_bridge.c, which isn't even compiled (see Makefile).
 */
#ifndef SENSOR_DISTANCE_H
#define SENSOR_DISTANCE_H

#include <stdint.h>
#include "ch.h"
#include "hal.h"

/* Call once at startup, after sdStart() on the UART wired to the FC's
 * telemetry port. */
void sensor_distance_init(SerialDriver *sd);

/* Call once per second from your main loop - keeps this companion computer
 * visible to the FC/GCS with a normal HEARTBEAT stream (sysid 100). */
void sensor_distance_send_heartbeat(void);

/* Call with a fresh reading in millimeters (as returned by
 * vl53l0x_read_range_mm()) whenever one is available. Converts to
 * centimeters - MAVLink's native unit for this message - and sends a
 * front-facing DISTANCE_SENSOR message the FC's AP_Proximity MAVLink
 * backend consumes directly. */
void sensor_distance_send(uint16_t range_mm);

#endif /* SENSOR_DISTANCE_H */
