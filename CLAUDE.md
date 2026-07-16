# CLAUDE.md

Guidance for Claude Code (or any agent) working in this repo.

## Working agreement — read first

- **Never run `make` / build the firmware unless the user explicitly asks
  for a build in that turn.** The user builds and flashes on their own
  machine/workflow; don't do it "just to verify" a change compiles.
- This repo has multiple branches at different stages of the same idea
  (currently `main` and `no-alt-landing`). Don't assume a file described in
  an earlier conversation still exists — check the current branch's working
  tree before relying on it (e.g. `config.h` exists on `no-alt-landing` but
  not on `main` as of this writing).

## What this project is

A bare-metal ChibiOS/RT firmware project for an STM32F401RCT6 "minimum
system" board that turns it into a MAVLink companion computer sitting on a
flight controller's telemetry UART. Two things live here:

1. **RTL redirect** (`main.c` + `mavlink_bridge.c/.h`) — watches the FC's
   flight mode and, on RTL, takes over via MAVLink GUIDED mode to land at a
   different point than the FC's own RTL location.
2. **Dummy obstacle generator** (`dummy_obstacle.c/.h`) — synthetic
   single-beam LIDAR data (front/left/right) for bench-testing ArduPilot's
   built-in obstacle avoidance without real rangefinder hardware.

Full hardware/wiring/toolchain/debugging history is in [README.MD](README.MD).

## Build

Only when explicitly asked to build:
```bash
export PATH="$HOME/stm32-sandbox/toolchain/bin:$PATH"
cd ~/stm32-sandbox/ChibiOS/demos/STM32/mavlink-uart
make clean && make
# flash:
STM32_Programmer_CLI -c port=usb1 -w build/ch.bin 0x08000000 -v -rst
```

## Active vs. reference files

The [Makefile](Makefile) hardcodes exactly which `.c` files are compiled:
```makefile
CSRC = $(ALLCSRC) $(TESTSRC) main.c mavlink_bridge.c dummy_obstacle.c
```

Only those are built. Underscore-prefixed files at the top level
(`_main_blink.c`, `_main_arm_disarm_test.c`, `_mavlink_bridge.c/.h`) are
reference/test variants kept for history — not compiled unless you edit the
Makefile or swap one in for `main.c`/`mavlink_bridge.c` manually.

| File | Status | Purpose |
|---|---|---|
| `main.c` | active | Entry point: USART1 init, MAVLink RX thread, LED status thread, heartbeat tick |
| `mavlink_bridge.c` / `.h` | active | RTL detection → GUIDED → `NAV_LAND` at a hardcoded lat/lon once altitude drops to `ALT_TRIGGER_M` |
| `dummy_obstacle.c` / `.h` | active, wired into `main.c` | Synthetic `DISTANCE_SENSOR` generator — see below |
| `_main_blink.c` | reference only | STEP 1: minimal LED blink to validate toolchain/clock/pin |
| `_main_arm_disarm_test.c` | reference only | Standalone arm/disarm command test |
| `_mavlink_bridge.c` / `.h` | reference only | Earlier bridge variant |

## dummy_obstacle.c / .h — synthetic LIDAR for obstacle-avoidance testing

Purpose: feed ArduPilot's `AP_Proximity` MAVLink backend (which drives
`AC_Avoid` / BendyRuler / Dijkstra's avoidance) fake but realistic-looking
single-beam rangefinder data, so avoidance behavior can be exercised without
real sensors — script "an object appears at 5m, closes to 2m, holds 2s,
then clears" and see the FC react.

- Sends `MAVLINK_MSG_ID_DISTANCE_SENSOR`, one message per simulated sensor,
  tagged with `MAV_SENSOR_ROTATION_NONE` (front) / `_YAW_270` (left) /
  `_YAW_90` (right) so ArduPilot buckets each reading into the correct
  proximity sector.
- `dummy_obstacle_init(sd)` — call once with the same `SerialDriver*` as
  `mavlink_bridge_init()` (one physical UART to the FC).
- `dummy_obstacle_trigger(sensor, start_delay_s, far_m, near_m, hold_s)` —
  schedules one event per sensor: after `start_delay_s`, distance ramps from
  `far_m` down to `near_m` at `-5cm`/`50ms` (1 m/s closing rate), holds at
  `near_m` for `hold_s`, then reports "clear" (max range) again. Each
  sensor's timer is independent and counted in `50ms` ticks from the moment
  `trigger()` is called — not from a shared global clock.
- `dummy_obstacle_tick()` — must be called every `OBSTACLE_TICK_MS` (50ms)
  from a dedicated thread (blocks on `sdWrite()`, so never from an
  ISR/timer callback). Advances all 3 sensors' state machines and streams
  their current reading, every tick, including when idle.
- `dummy_obstacle_is_detected()` — true while any sensor is `APPROACHING` or
  `HOLDING` (i.e. actively reporting an obstacle, not idle/clear or still
  counting down a delay). Used by the LED thread to fast-blink on detection.
- Wired into `main.c`: `dummy_obstacle_init(&SD1)` runs once at startup
  (same UART as `mavlink_bridge_init`), and a dedicated `ObstacleThread`
  calls `dummy_obstacle_tick()` every `OBSTACLE_TICK_MS`.
- **Demo scenario, one-shot**: `main.c`'s main loop watches
  `mavlink_bridge_is_connected()` (true from the first decoded HEARTBEAT
  onward) and, the first time it goes true, calls
  `dummy_obstacle_trigger(OBSTACLE_SENSOR_FRONT, 10.0f, 5.0f, 2.0f, 2.0f)` —
  i.e. 10s after first MAVLink connect, front sensor sees an object at 5m
  closing to 2m, holds 2s, then clears. Guarded by
  `obstacle_scenario_armed` so it fires exactly once per boot, not on every
  reconnect. Edit the call in `main()` to retune/relocate the demo, or to
  add more `dummy_obstacle_trigger()` calls for left/right.

## LED status (PC13, active-low)

Driven by `LedThread` in `main.c`, combining `mavlink_bridge`'s connection
state with `dummy_obstacle_is_detected()`. Priority, highest first:

1. Power-on: 3 blinks (150ms on/off), then dark — always runs once at boot,
   independent of everything else below.
2. No heartbeat from FC within `HEARTBEAT_TIMEOUT_MS` → **off**.
3. Obstacle currently detected (any sensor `APPROACHING`/`HOLDING`) →
   **fast blink** (80ms on/off) — overrides RTL/connected states, but not
   "disconnected".
4. Otherwise, `mavlink_bridge`'s own state: `LED_STATE_RTL_SOLID` → solid
   on; `LED_STATE_REDIRECTED_FASTBLINK` → fast blink; `LED_STATE_CONNECTED_BLINK`
   → slow heartbeat blink (100ms on / 900ms off).

FC-side setup needed to actually see ArduPilot react to this data:
`PRX1_TYPE = 2` (MAVLink proximity backend), plus `AVOID_ENABLE` and
`OA_TYPE` (BendyRuler or Dijkstra's) set appropriately for the vehicle/ArduPilot
version in use.
