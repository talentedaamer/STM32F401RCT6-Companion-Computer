# CLAUDE.md

Guidance for Claude Code (or any agent) working in this repo.

## What this project is

A bare-metal ChibiOS/RT firmware project for an STM32F401RCT6 "minimum
system" board that turns it into a MAVLink companion computer. It sits on
a flight controller's telemetry UART, watches the flight mode, and — when
the FC enters **RTL (Return-to-Launch)** — takes over via MAVLink GUIDED
mode and redirects the landing to a configured GPS coordinate instead of
letting the FC land at its own RTL point.

Full hardware/wiring/toolchain/debugging history is in [README.MD](README.MD)
— read that for bring-up details. This file is about which source files are
live vs. reference, and how to change behavior safely.

## Build

```bash
export PATH="$HOME/stm32-sandbox/toolchain/bin:$PATH"
cd ~/stm32-sandbox/ChibiOS/demos/STM32/mavlink-uart
make clean && make
# flash:
STM32_Programmer_CLI -c port=usb1 -w build/ch.bin 0x08000000 -v -rst
```

## Active vs. reference files — read this before editing

The [Makefile](Makefile) hardcodes exactly which `.c` files are compiled:

```makefile
CSRC = $(ALLCSRC) $(TESTSRC) main.c mavlink_bridge.c
```

**Only [main.c](main.c) and [mavlink_bridge.c](mavlink_bridge.c) /
[mavlink_bridge.h](mavlink_bridge.h) are built.** Everything else at the
top level with a leading underscore is a *reference/test variant*, kept
for history or for swapping in manually — none of it is compiled unless
you edit the Makefile:

| File | Status | Purpose |
|---|---|---|
| `main.c` | **active** | Real entry point: USART1 init, MAVLink RX thread, LED status thread, heartbeat tick |
| `mavlink_bridge.c` / `.h` | **active** | The real logic: RTL detection → GUIDED → fly to configured lat/lon at current altitude → land once within arrival radius (`LAND_STATE_IDLE → ENROUTE → LANDING`) |
| `config.h` | **active** | All tunable constants (see below) |
| `_main_blink.c` | reference only, not compiled | STEP 1: minimal LED blink used to validate toolchain/clock/pin before adding MAVLink |
| `_main_arm_disarm_test.c` | reference only, not compiled | Standalone test: blinks, sends `MAV_CMD_COMPONENT_ARM_DISARM` (arm), waits 5s, disarms. Used to validate arm/disarm command plumbing independent of the RTL-redirect logic |
| `_mavlink_bridge.c` / `.h` | reference only, not compiled | Older/simpler bridge: altitude-triggered immediate `NAV_LAND` at a fixed point (no reposition/fly-to step, no arrival-radius check). Superseded by the current `mavlink_bridge.c`'s reposition-then-land approach (see commits `no alt limit`, `landing on custom lat long on RTL`) |

If you want to run one of the `_*` test files instead of the real bridge,
either temporarily edit the Makefile's `CSRC` line to point at it, or copy
it over `main.c` — do not just add it alongside `main.c`/`mavlink_bridge.c`,
you'll get duplicate `main()` / duplicate symbol link errors.

## Configuration

All tunable values (landing coordinates, arrival radius, telemetry baud,
sysid/compid, ArduCopter mode numbers, heartbeat timeout, LED pin) now live
in **[config.h](config.h)** — edit that file, not `main.c` or
`mavlink_bridge.h`, to change behavior. It's included by both.

Key values to check per deployment:
- `NEW_LANDING_LAT_DEG` / `NEW_LANDING_LON_DEG` — the redirect landing target. Currently a fixed placeholder; feeding this from a live source instead of a hardcoded constant is a known open item (see README §9.3).
- `FC_TELEM_BAUD` — must match the FC's `SERIALx_BAUD` telemetry param.
- `AP_COPTER_MODE_RTL` — ArduCopter-specific `custom_mode` value; unverified against live hardware (README §9.2), and does not apply to PX4 at all.
- `ARRIVAL_RADIUS_M` — how close (meters) to the target before `NAV_LAND` fires.

## Current firmware behavior (mavlink_bridge.c)

1. On first HEARTBEAT from the FC: record its sysid/compid, request
   `GLOBAL_POSITION_INT` streaming at 5 Hz.
2. While idle, watch `custom_mode`. When it equals `AP_COPTER_MODE_RTL`:
   switch FC to `GUIDED`, then send `SET_POSITION_TARGET_GLOBAL_INT` to fly
   to `NEW_LANDING_LAT_DEG/LON_DEG` at the **current altitude** (no altitude
   trigger/wait — this was removed, see git history `no alt limit`).
3. On each `GLOBAL_POSITION_INT` update while en route: compute horizontal
   distance to target; once within `ARRIVAL_RADIUS_M`, send
   `COMMAND_INT(MAV_CMD_NAV_LAND, current=1)` to land in place.
4. If the pilot takes back control (mode changes away from `GUIDED` while
   we're mid-redirect), stand down back to idle immediately.
5. Status LED (PC13): off = no heartbeat in 3s, slow blink = connected/idle,
   fast blink = redirect in progress.

## Known unresolved issues (see README §9 for full detail)

- HSE (25 MHz crystal) hangs at boot — firmware currently runs on HSI
  (16 MHz internal) as a working substitute. Do not "fix" this by just
  flipping `STM32_HSE_ENABLED` back to `TRUE` without re-reading README §9.1.
- `AP_COPTER_MODE_RTL = 6` is unverified against the actual FC firmware in
  use — confirm via a live HEARTBEAT `custom_mode` dump before trusting it.
- This is a bench-tested proof-of-concept, **not flight-ready**.
