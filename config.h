/*
 * config.h - all tunable constants for the MAVLink RTL-redirect bridge.
 * Edit this file to change landing coordinates, telemetry baud, IDs, etc.
 * without touching the logic in main.c / mavlink_bridge.c.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* --- UART / telemetry link --- */
/* Must match the FC's telemetry port SERIALx_BAUD param. 57600 is the
 * common ArduPilot default; some setups run 115200/921600. */
#define FC_TELEM_BAUD          57600

/* --- Onboard status LED --- */
#define LED_LINE               PAL_LINE(GPIOC, 13U)   /* active-low */

/* --- MAVLink identity of this companion computer --- */
#define OWN_SYSID              100
#define OWN_COMPID             MAV_COMP_ID_ONBOARD_COMPUTER

/* --- ArduCopter custom_mode numbers (verify against your firmware/version;
 * PX4 encodes flight mode differently and these do not apply there) --- */
#define AP_COPTER_MODE_GUIDED  4
#define AP_COPTER_MODE_LOITER  5
#define AP_COPTER_MODE_RTL     6

/* --- Redirect-on-RTL landing target --- */
#define NEW_LANDING_LAT_DEG    33.541991
#define NEW_LANDING_LON_DEG    73.113652

/* How close (meters) counts as "arrived" at the redirect target before
 * NAV_LAND is sent. */
#define ARRIVAL_RADIUS_M       3.0f

/* --- Heartbeat / link-health --- */
#define HEARTBEAT_TIMEOUT_MS   3000   /* no RX heartbeat within this -> LED_STATE_DISCONNECTED */

#endif /* CONFIG_H */
