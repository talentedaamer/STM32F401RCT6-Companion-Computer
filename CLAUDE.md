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
- `main.c` gets swapped out entirely between experiments (MAVLink bridge vs.
  sensor bench tests) rather than merged together. Previous versions are
  kept as underscore-prefixed backups (see table below) — check which one
  is actually active in the Makefile before assuming a function/thread from
  an earlier conversation still runs.

## What this project is

A bare-metal ChibiOS/RT firmware project for an STM32F401RCT6 "minimum
system" board. Two separate lines of work live here, only one of which is
wired into `main.c`/the Makefile at any given time:

1. **MAVLink RTL-redirect bridge** (`mavlink_bridge.c/.h`) — sits on a
   flight controller's telemetry UART; watches for RTL and redirects the
   landing to a hardcoded GPS coordinate. **Not currently compiled** (its
   RX thread/state machine is deliberately not reused elsewhere — see the
   safety note under `sensor_distance.h` below).
2. **Dummy obstacle generator** (`dummy_obstacle.c/.h`) — streams synthetic
   3-sensor LIDAR data to bench-test ArduPilot's obstacle avoidance without
   real hardware. **Not currently compiled.**
3. **Real VL53L0X obstacle sensor** (`vl53l0x.c/.h` + `sensor_distance.c/.h`
   + `main.c`) — reads a GY-530 (VL53L0X) ToF sensor over I2C1 and forwards
   every reading to the FC as a front-facing MAVLink `DISTANCE_SENSOR`
   message, so ArduPilot's real obstacle avoidance can act on it. Also
   drives the onboard LED at a rate proportional to distance, as a bench
   indicator. **Currently active.**

Full hardware/wiring/toolchain/debugging history for the MAVLink side is in
[README.MD](README.MD) — the VL53L0X work postdates that document.

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
CSRC = $(ALLCSRC) $(TESTSRC) main.c vl53l0x.c sensor_distance.c
```

Only those are built. Everything else at the top level is kept on disk for
reference but not compiled unless you edit the Makefile:

| File | Status | Purpose |
|---|---|---|
| `main.c` | **active** | VL53L0X read loop, MAVLink forwarding, LED blink-rate indicator (see below) |
| `vl53l0x.c` / `.h` | **active** | Minimal VL53L0X (GY-530) I2C driver, continuous-ranging mode |
| `sensor_distance.c` / `.h` | **active** | Packs/sends the real sensor reading as MAVLink `DISTANCE_SENSOR` + HEARTBEAT — see below |
| `_main_mavlink_obstacle.c` | reference only | Previous `main.c`: MAVLink RX thread, LED status thread, obstacle-avoidance thread — the whole MAVLink companion-computer firmware, before it was swapped out for the VL53L0X test |
| `mavlink_bridge.c` / `.h` | present, not compiled | RTL detection → GUIDED → `NAV_LAND` at a hardcoded lat/lon once altitude drops to `ALT_TRIGGER_M`. Was active until the VL53L0X swap. Its header (constants + vendored `mavlink.h` include only) is reused by `sensor_distance.c` — its `.c` is not. |
| `dummy_obstacle.c` / `.h` | present, not compiled | Synthetic `DISTANCE_SENSOR` generator for bench-testing ArduPilot proximity/avoidance — see below. Was active until the VL53L0X swap. |
| `_main_blink.c` | reference only | STEP 1: minimal LED blink to validate toolchain/clock/pin |
| `_main_arm_disarm_test.c` | reference only | Standalone arm/disarm command test |
| `_mavlink_bridge.c` / `.h` | reference only | Earlier bridge variant (altitude-triggered immediate land, no reposition step) |

To restore the MAVLink RTL-redirect + dummy obstacle companion computer:
copy `_main_mavlink_obstacle.c` back over `main.c`, add `mavlink_bridge.c
dummy_obstacle.c` back to the Makefile's `CSRC` (dropping `sensor_distance.c`,
which would otherwise duplicate the heartbeat), and revert `HAL_USE_I2C`/
`STM32_I2C_USE_I2C1` if you no longer need I2C (they don't conflict with
the MAVLink UART, so leaving them on is harmless either way).

## Real VL53L0X obstacle sensor -> MAVLink DISTANCE_SENSOR (currently active)

`main.c` reads a **GY-530 breakout (VL53L0X ToF sensor)** over I2C1,
forwards every reading to the FC over USART1 as a MAVLink `DISTANCE_SENSOR`
message (front-facing, sysid 100 "Vehicle 100" like the old bridge), and
blinks the onboard LED (PC13, active-low) at a rate proportional to
distance as a bench indicator.

**Units**: `vl53l0x_read_range_mm()` returns **millimeters** (the sensor's
native register format) as a `uint16_t` — not a string, not meters.
MAVLink's `DISTANCE_SENSOR` message fields are natively **centimeters**
(also `uint16_t`), so `sensor_distance.c` divides by 10 and clamps into
`[SENSOR_MIN_RANGE_CM, SENSOR_MAX_RANGE_CM]` (3–120cm, matching the
VL53L0X's rated ~30mm–1200mm range) before packing the message.

**`sensor_distance.c`/`.h`** — deliberately does NOT call into
`mavlink_bridge.c`'s RX thread/state machine, even though it reuses
`mavlink_bridge.h`'s `OWN_SYSID`/`OWN_COMPID` constants and vendored
`common/mavlink.h` include. That state machine also contains the
RTL-redirect-to-hardcoded-GPS logic from the other experiment in this repo
— pulling it in here would make the FC land at a hardcoded coordinate the
next time it enters RTL, unrelated to obstacle avoidance and a dangerous
surprise on a real flight. `mavlink_bridge.c` itself isn't even in the
Makefile's `CSRC`, so none of that logic is linked in.
- `sensor_distance_init(sd)` — call once with the `SerialDriver*` for the
  UART wired to the FC.
- `sensor_distance_send_heartbeat()` — call once per second; keeps this
  companion computer visible in Mission Planner as "Vehicle 100".
- `sensor_distance_send(range_mm)` — call with each fresh reading; handles
  the mm→cm conversion and clamping described above, sends orientation
  `MAV_SENSOR_ROTATION_NONE` (front), sensor id `0`.

`main.c` runs this on two independent threads so LED blink timing never
throttles how often the FC gets fresh data: `SensorThread` reads the
VL53L0X back-to-back and calls `sensor_distance_send()` immediately on
every sample; `LedThread` just polls the latest reading (a shared
`volatile uint16_t`, safe without a lock since a single aligned 16-bit
read/write is atomic on Cortex-M4) on its own blink schedule.

**Wiring:**
| Signal | STM32 pin |
|---|---|
| VL53L0X VCC | 3V3 |
| VL53L0X GND | GND |
| VL53L0X SCL | PB8 (I2C1_SCL, AF4) |
| VL53L0X SDA | PB9 (I2C1_SDA, AF4) |
| FC TELEM TX | PA10 (USART1 RX1) |
| FC TELEM RX | PA9 (USART1 TX1) |
| FC GND | GND |

**Config enabled for this:** `HAL_USE_I2C` in `cfg/halconf.h` and
`STM32_I2C_USE_I2C1` in `cfg/mcuconf.h`, both `TRUE` (were `FALSE` before
the VL53L0X work started). `HAL_USE_SERIAL`/`STM32_SERIAL_USE_USART1` were
already `TRUE` from the original MAVLink bridge.

**`vl53l0x.c`/`.h`** — a minimal from-scratch driver (not a port of ST's
official API), since the sensor requires a specific data-init → SPAD
map-from-NVM → default tuning-register block → VHV/phase reference
calibration sequence before it produces valid ranges — there's no shortcut
around this part:
- `vl53l0x_init(i2cp)` — runs that full sequence once at startup. Returns
  `false` if the sensor never answered the model-ID check (wiring/address
  problem) or a calibration step timed out.
- `vl53l0x_start_continuous()` — call once after a successful init; puts
  the sensor into back-to-back continuous ranging.
- `vl53l0x_read_range_mm()` — blocks (bounded by an internal timeout) for
  the next sample and returns distance in mm. Returns `8190` on timeout or
  "no target", matching the sensor's own out-of-range convention.

**`main.c` LED behavior** (`FAR_THRESHOLD_MM` / `NEAR_THRESHOLD_MM`, both
near the top of the file — edit to retune):
| Distance | LED |
|---|---|
| > 1000mm (1m) | off — nothing in range |
| 300mm–1000mm | slow blink (300ms on/off) |
| ≤ 300mm | fast blink (80ms on/off) |
| sensor didn't answer at startup | fast triple-blink, repeating — distinct error pattern so a wiring problem doesn't look like "very close obstacle" |

This hasn't been build/hardware-tested yet (builds are only run when
explicitly requested — see Working agreement above); if ranging looks wrong
once flashed, the most likely culprits are the SPAD/tuning-register
sequence in `vl53l0x.c` (transcribed from the well-known open-source
minimal-driver algorithm for this sensor, not verified against this exact
chip) or the I2C1 pin/AF setup in `main.c`.

### FC/Mission Planner parameters for this to actually do anything

The `SERIAL2_PROTOCOL`/`SERIAL2_BAUD`-style pair on whichever FC UART this
board is wired to must already be set to MAVLink + `921600` (matching
`sd1cfg` in `main.c`, changed from `57600` to line up with another PoC's
baud setting so Mission Planner/FC config doesn't need to change per
experiment) — same prerequisite as the old bridge, see README. On top of
that, for ArduPilot to actually route the incoming `DISTANCE_SENSOR` into
obstacle avoidance:
- `PRX1_TYPE = 2` (MAVLink proximity backend) — requires a reboot.
- `AVOID_ENABLE` — bit 2 (value `2`) enables proximity-based avoidance;
  combine with fence (bit 1, value `1`) as needed, e.g. `3` for both.
- `OA_TYPE` — `1` (BendyRuler) or `2` (Dijkstra's); BendyRuler is the
  simpler one to start with.
- Avoidance only engages in position-controlled modes (Loiter, Guided,
  Auto, RTL, etc.) — not Stabilize/Acro.
- Verify with Mission Planner: **Ctrl+F → MAVLink Inspector** should show
  `Vehicle 100 / Comp 191 MAV_COMP_ID_ONBOARD_COMPUTER` sending `HEARTBEAT`
  (1Hz) and `DISTANCE_SENSOR` (as fast as the sensor samples). Once
  `PRX1_TYPE` is set, the Flight Data screen's **Proximity** view should
  show the front sector live instead of just raw Inspector fields.

## MAVLink bridge (currently inactive, kept for reference)

The rest of this section describes `_main_mavlink_obstacle.c` +
`mavlink_bridge.c/.h` + `dummy_obstacle.c/.h` as they behaved when they were
still wired into `main.c`/the Makefile. Restore them (see above) before any
of this applies again.

### dummy_obstacle.c / .h — synthetic LIDAR for obstacle-avoidance testing

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
- Was wired into `main.c`: `dummy_obstacle_init(&SD1)` ran once at startup
  (same UART as `mavlink_bridge_init`), and a dedicated `ObstacleThread`
  called `dummy_obstacle_tick()` every `OBSTACLE_TICK_MS`.
- **Demo scenario, one-shot**: the main loop watched
  `mavlink_bridge_is_connected()` (true from the first decoded HEARTBEAT
  onward) and, the first time it went true, called
  `dummy_obstacle_trigger(OBSTACLE_SENSOR_FRONT, 10.0f, 5.0f, 2.0f, 2.0f)` —
  i.e. 10s after first MAVLink connect, front sensor sees an object at 5m
  closing to 2m, holds 2s, then clears. Guarded by
  `obstacle_scenario_armed` so it fired exactly once per boot, not on every
  reconnect.

### LED status (PC13, active-low) — MAVLink version

Was driven by `LedThread` in `main.c`, combining `mavlink_bridge`'s
connection state with `dummy_obstacle_is_detected()`. Priority, highest
first:

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
