/*
 * dummy_obstacle.h - synthetic single-beam LIDAR generator for bench-testing
 * ArduPilot's built-in obstacle avoidance (AP_Proximity / AC_Avoid, incl.
 * BendyRuler/Dijkstra's object avoidance) without real rangefinder hardware.
 *
 * Streams MAVLINK_MSG_ID_DISTANCE_SENSOR messages - the same message
 * AP_Proximity's MAV backend consumes - one per simulated sensor
 * (front/left/right), tagged with the matching MAV_SENSOR_ROTATION_* so
 * ArduPilot places each reading in the correct proximity sector.
 *
 * Usage:
 *   dummy_obstacle_init(&SD1);
 *   dummy_obstacle_trigger(OBSTACLE_SENSOR_FRONT, 10.0f, 5.0f, 2.0f, 2.0f);
 *   dummy_obstacle_trigger(OBSTACLE_SENSOR_LEFT,  15.0f, 4.0f, 1.5f, 3.0f);
 *   // then, from a thread looping every 50ms:
 *   while (true) {
 *       dummy_obstacle_tick();
 *       chThdSleepMilliseconds(OBSTACLE_TICK_MS);
 *   }
 */
#ifndef DUMMY_OBSTACLE_H
#define DUMMY_OBSTACLE_H

#include "mavlink_bridge.h"   /* pulls in ch.h, hal.h, common/mavlink.h, OWN_SYSID/OWN_COMPID */

/* Sensor generator must be driven at this cadence - dummy_obstacle_tick()
 * assumes exactly this much time passed since the previous call and steps
 * every schedule's countdown/ramp accordingly. */
#define OBSTACLE_TICK_MS        50

/* Approach rate: how far the reported distance closes per tick while a
 * sensor is in its APPROACHING phase. 5cm every 50ms = 1 m/s closing rate. */
#define OBSTACLE_STEP_CM        5

/* "Clear" reading reported whenever a sensor has no obstacle scheduled -
 * matches typical single-point LIDAR (e.g. Benewake/LightWare) max range. */
#define OBSTACLE_MAX_RANGE_CM   4000
#define OBSTACLE_MIN_RANGE_CM   10

typedef enum {
    OBSTACLE_SENSOR_FRONT = 0,
    OBSTACLE_SENSOR_LEFT,
    OBSTACLE_SENSOR_RIGHT,
    OBSTACLE_SENSOR_COUNT
} obstacle_sensor_t;

/* Call once at startup, after sdStart() on the UART the FC is on (same
 * SerialDriver as mavlink_bridge_init(), since it's the one physical link). */
void dummy_obstacle_init(SerialDriver *sd);

/* Schedule one approach-and-hold obstacle event on a sensor:
 *   sensor          - which simulated beam (front/left/right)
 *   start_delay_s   - seconds from *now* (this call) before the object
 *                      starts being reported
 *   far_distance_m  - distance the object is first detected at
 *   near_distance_m - closest distance it approaches to before holding
 *   hold_time_s     - how long (seconds) to hold at near_distance_m before
 *                      the sensor reports "clear" (OBSTACLE_MAX_RANGE_CM) again
 *
 * The transition from far_distance_m down to near_distance_m ramps at
 * OBSTACLE_STEP_CM per OBSTACLE_TICK_MS (matches a real sensor's native
 * update rate rather than jumping instantly), then holds, then clears.
 * Calling this again for the same sensor before a prior event finishes
 * replaces it.
 */
void dummy_obstacle_trigger(obstacle_sensor_t sensor,
                             float start_delay_s,
                             float far_distance_m,
                             float near_distance_m,
                             float hold_time_s);

/* Advance every sensor's state machine by one OBSTACLE_TICK_MS step and
 * stream its current DISTANCE_SENSOR reading. Call this from a dedicated
 * thread sleeping OBSTACLE_TICK_MS between calls - never from ISR/timer
 * callback context, since it blocks on sdWrite(). */
void dummy_obstacle_tick(void);

/* True while any simulated sensor is actively reporting an obstacle
 * (APPROACHING or HOLDING) rather than "clear" or still WAITING to start.
 * Poll this from the LED thread (or anywhere else) to reflect live
 * obstacle-avoidance status. */
bool dummy_obstacle_is_detected(void);

#endif /* DUMMY_OBSTACLE_H */
