# STM32F401RCT6 + ChibiOS + MAVLink Companion Bridge

A bare-metal ChibiOS project that turns a cheap STM32F401RCT6 "minimum
system" board into a MAVLink companion computer. It listens to a flight
controller's telemetry UART, watches for RTL (Return-to-Launch), and once
altitude drops to a trigger threshold, redirects the landing to a hardcoded
GPS coordinate. A single onboard LED (PC13) reports connection/flight state
at a glance. This is a bench-tested proof-of-concept — **not flight-ready**
until the open items below are resolved.

---

## 1. Hardware

| Item | Detail |
|---|---|
| MCU board | STM32F401RCT6 "minimum system" dev board, 40-pin, USB-C |
| Crystal | 25 MHz HSE (**currently unused — see Known Issues**) |
| Onboard LED | PC13, active-low |
| Buttons | NRST, BOOT, KEY |
| FC link | USART1 — PA9 (TX1) / PA10 (RX1) |
| Power | USB-C (5V) or external 5V pin; do **not** feed VBAT as main power |

**Wiring to flight controller (e.g. FC UART2/telemetry port):**

| FC wire | STM32 pin |
|---|---|
| FC TX (data out of FC) | PA10 (RX1) |
| FC RX (data into FC) | PA9 (TX1) |
| FC GND | GND |

TX always connects to RX on the other device — never TX-to-TX. This was a
real point of confusion during bring-up (see Known Issues), so double-check
before powering up.

---

## 2. Toolchain / environment

Built on a Linux server, isolated from system state (Option B — no Docker,
no `sudo apt install`):

```bash
mkdir -p ~/stm32-sandbox/toolchain && cd ~/stm32-sandbox
wget https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz
tar xf arm-gnu-toolchain-*.tar.xz -C toolchain --strip-components=1
export PATH="$HOME/stm32-sandbox/toolchain/bin:$PATH"

git clone https://github.com/ChibiOS/ChibiOS.git
```

Project lives at:
```
~/stm32-sandbox/ChibiOS/demos/STM32/mavlink-uart/
```
Originally copied from the `RT-STM32F401RE-NUCLEO64` demo template as a
starting point, then customized (see Project structure below).

Flashing is via **STM32CubeProgrammer** (GUI or CLI) or **dfu-util**, using
the board's built-in USB DFU bootloader (hold BOOT, tap NRST, release BOOT):

```bash
STM32_Programmer_CLI -c port=usb1 -w build/ch.bin 0x08000000 -v -rst
# or
dfu-util -a 0 -s 0x08000000:leave -D build/ch.bin
```

---

## 3. Project structure

```
mavlink-uart/
├── board/                  # LOCAL copy of board files (see below — important)
│   ├── board.h
│   ├── board.c
│   └── board.mk
├── cfg/
│   ├── halconf.h           # HAL_USE_SERIAL = TRUE (default in this template)
│   └── mcuconf.h           # clock tree + peripheral enables (see Section 5)
├── external/
│   └── mavlink/            # mavlink/c_library_v2, cloned directly (header-only)
│       └── common/
│           └── mavlink.h
├── main.c                  # entry point, threads, LED status thread
├── mavlink_bridge.c/.h     # MAVLink parsing, state machine, redirect logic
├── Makefile
└── build/                  # make output: ch.elf, ch.bin, ch.hex
```

### Why `board/` is local and not shared

The Nucleo demo template normally references ChibiOS's shared board file:
```
include $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_F401RE/board.mk
```
This board isn't a Nucleo, so `board.h`'s clock definitions needed editing.
Editing the shared file directly required `sudo` and affected every other
ChibiOS demo referencing it. It was copied locally instead:

```bash
mkdir -p board
cp ~/stm32-sandbox/ChibiOS/os/hal/boards/ST_NUCLEO64_F401RE/board.{h,c,mk} board/
```
`board/board.mk` was edited so `BOARDSRC`/`BOARDINC` point to `./board/...`
instead of the shared ChibiOS path, and the main `Makefile`'s `include` line
was updated to `include ./board/board.mk`. This means clock/board edits are
now fully local, version-controllable, and don't need `sudo`.

---

## 4. Getting the MAVLink library

Header-only, no build step:
```bash
cd ~/stm32-sandbox/ChibiOS/demos/STM32/mavlink-uart
git clone https://github.com/mavlink/c_library_v2.git external/mavlink
```

Wired into the Makefile:
```makefile
UINCDIR += external/mavlink external/mavlink/common
CSRC = $(ALLCSRC) \
       $(TESTSRC) \
       main.c \
       mavlink_bridge.c
```

---

## 5. Clock configuration — current state (IMPORTANT)

**This project currently runs on HSI (internal 16 MHz oscillator), NOT the
25 MHz HSE crystal.** This is a known, deliberate rollback made during
debugging — see Known Issues below.

Current working `cfg/mcuconf.h` values:
```c
#define STM32_HSE_ENABLED                   FALSE
#define STM32_PLLSRC                        STM32_PLLSRC_HSI
#define STM32_PLLM_VALUE                    16
#define STM32_PLLN_VALUE                    336
#define STM32_PLLP_VALUE                    4
#define STM32_PLLQ_VALUE                    7
```
Math: `16MHz / 16 = 1MHz` PLL input → `× 336 = 336MHz` VCO → `/ 4 = 84MHz`
sysclock (F401's max), `/ 7 = 48MHz` USB clock.

**The HSE-based config that should eventually replace this** (25MHz crystal):
```c
#define STM32_HSE_ENABLED                   TRUE
#define STM32_PLLSRC                        STM32_PLLSRC_HSE
#define STM32_PLLM_VALUE                    25   /* 25MHz/25 = 1MHz input */
#define STM32_PLLN_VALUE                    336  /* unchanged */
#define STM32_PLLP_VALUE                    4    /* unchanged */
#define STM32_PLLQ_VALUE                    7    /* unchanged */
```
Also requires, in `board/board.h`:
```c
#define STM32_HSECLK                25000000U
```

This HSE config compiles and passes all ChibiOS PLL range checks
(`STM32_PLLIN_MIN/MAX`, `STM32_PLLVCO_MIN/MAX` for `STM32F401xx`), but
**hangs at boot with zero output** — not even the power-on LED blink runs.
This points to the crystal oscillator itself never reporting HSE-ready to
the hardware, not a math/config error. See Known Issues.

---

## 6. Peripheral configuration

**`cfg/halconf.h`** — `HAL_USE_SERIAL` is `TRUE` by default in this
template; no edit was needed.

**`cfg/mcuconf.h`** — USART1 enabled (USART2/6 left off, not used):
```c
#define STM32_SERIAL_USE_USART1             TRUE
#define STM32_SERIAL_USE_USART2             FALSE
#define STM32_SERIAL_USE_USART6             FALSE
```

**`cfg/chconf.h`** — debug checks enabled during the stack-overflow
investigation (see Known Issues/history) and left on:
```c
#define CH_DBG_ENABLE_ASSERTS               TRUE
#define CH_DBG_ENABLE_STACK_CHECK           TRUE
```
Recommended to keep these on for a project this young — they cost a little
flash/cycles but catch corruption early instead of silent freezes.

**USART1 pin setup** (done directly in `main.c`, not via `board.h` tables):
```c
palSetPadMode(GPIOA, 9,  PAL_MODE_ALTERNATE(7));   // PA9  = TX1
palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));   // PA10 = RX1
```

**Baud rate** — set in `main.c`:
```c
static const SerialConfig sd1cfg = {
    57600,   /* must match FC's SERIALx_BAUD param */
    0, 0, 0
};
```
Check/set on the FC side in Mission Planner: `Config → Full Parameter List
→ SERIAL2_PROTOCOL` (must be `1` or `2` = MAVLink) and `SERIAL2_BAUD` (`57`
= 57600).

---

## 7. What the firmware does

**Boot sequence** (`main.c`):
1. `halInit()` / `chSysInit()`
2. LED blinks **3 times** — power-on/firmware-alive confirmation, independent
   of any MAVLink state
3. USART1 pins configured, `sdStart(&SD1, &sd1cfg)`
4. `mavlink_bridge_init(&SD1)`
5. Two threads created: `MavRxThread` (MAVLink RX + logic) and `LedThread`
   (status LED)
6. Main loop sends our own HEARTBEAT once per second

**`MavRxThread`** (`mavlink_bridge_rx_thread_func`):
- Blocks on `sdGetTimeout()`, feeds bytes into `mavlink_parse_char()`
- Dispatches `HEARTBEAT` and `GLOBAL_POSITION_INT`
- On first heartbeat: records FC's sysid/compid, requests
  `GLOBAL_POSITION_INT` streaming via `MAV_CMD_SET_MESSAGE_INTERVAL`
- Tracks `custom_mode`: if it equals `AP_COPTER_MODE_RTL` (6 — **ArduCopter
  only, verify for your firmware**), arms the redirect watch
- Once armed and `relative_alt` drops to ≤ `ALT_TRIGGER_M` (30m): sends
  `SET_MODE(GUIDED)`, waits 50ms, sends `COMMAND_INT(MAV_CMD_NAV_LAND)` at
  the hardcoded `NEW_LANDING_LAT_DEG`/`NEW_LANDING_LON_DEG`. Fires once per
  RTL event.

**`LedThread`** — polls `mavlink_bridge_get_led_state()`:

| State | Pattern | Meaning |
|---|---|---|
| `LED_STATE_DISCONNECTED` | Off | No heartbeat received in last 3s |
| `LED_STATE_CONNECTED_BLINK` | Slow blink (~1Hz) | Heartbeat OK, normal flight |
| `LED_STATE_RTL_SOLID` | Solid on | FC in RTL mode |
| `LED_STATE_REDIRECTED_FASTBLINK` | Fast blink | Redirect command sent |

Note: there is no separate "trying to connect" pattern — disconnected and
"never yet connected" both just show as off.

---

## 8. Verification — confirmed working (as of last test)

**Mission Planner → MAVLink Inspector (`Ctrl+F`)** is the definitive check.
A successful bring-up shows:
```
Vehicle 1    → Comp 1 MAV_COMP_ID_AUTOPILOT1   (the flight controller)
Vehicle 100  → Comp 191 MAV_COMP_ID_ONBOARD_COMPUTER
                 HEARTBEAT (1.0 Hz, #0) 21Bps
                   autopilot = 8   (MAV_AUTOPILOT_INVALID)
                   type      = 18  (MAV_TYPE_ONBOARD_CONTROLLER)
```
This was confirmed working — sysid 100, correct rate, correct type/autopilot
fields — proving UART wiring, baud, MAVLink encoding, and the fixed stack
sizes are all stable. **This confirmation happened on HSI**, not HSE.

**Still to verify:**
- `COMMAND_ACK` after the redirect fires (confirms FC *accepted*, not just
  received, the mode switch and landing command)
- Full RTL → altitude-trigger → redirect sequence, ideally in SITL or
  bench/props-off, not a live flight
- Confirm actual RTL `custom_mode` value for your specific
  firmware/version via a live HEARTBEAT dump before trusting `6`
  (ArduCopter) — PX4 encodes mode differently

---

## 9. Known issues / open items

### 9.1 HSE (25MHz crystal) hangs at boot — currently unresolved
Switching `STM32_HSE_ENABLED` to `TRUE` and `STM32_PLLSRC` to
`STM32_PLLSRC_HSE` (with correct, range-valid PLL values for 25MHz) results
in **zero firmware execution** — not even the very first power-on LED blink,
which normally runs milliseconds after `chSysInit()`. This is consistent
with ChibiOS's `halInit()` blocking indefinitely waiting for the
`RCC_CR_HSERDY` hardware flag, which never sets if the crystal never
actually starts oscillating.

Ruled out so far:
- PLL math itself (confirmed valid against `STM32F401xx`'s actual
  `PLLIN_MIN/MAX`, `PLLVCO_MIN/MAX` bounds in ChibiOS's `hal_lld_type1.h`)
- Firmware logic in general (proven fine — same code runs perfectly on HSI)

Not yet checked:
- Whether `board/board.h`'s HSE bypass mode setting matches this board's
  actual crystal circuit (passive crystal + onboard load caps vs. an active
  oscillator module expect different bypass config — wrong setting alone
  can prevent HSE from ever locking)
- Physical continuity/health of the crystal itself
- Whether the crystal is genuinely 25MHz as silkscreened (visually
  near-identical to 8MHz crystals)

**Until this is resolved, the project runs on HSI as a working
substitute** — fine for bench validation, but baud-rate accuracy and
long-term clock stability should be revisited before this goes near a real
airframe.

### 9.2 RTL custom_mode not yet verified live
`AP_COPTER_MODE_RTL = 6` is correct for stock ArduCopter but unverified
against this specific FC/firmware/version. Confirm via a live HEARTBEAT
`custom_mode` field dump before relying on it. PX4 users: this constant
doesn't apply at all — PX4 encodes mode via `base_mode`/`custom_mode`
sub-fields differently.

### 9.3 Hardcoded landing coordinates
`NEW_LANDING_LAT_DEG`/`NEW_LANDING_LON_DEG` in `mavlink_bridge.h` are
placeholder values. The stated end goal is feeding these from a live
telemetry/GPS source instead — not yet implemented.

---

## 10. Debugging history (for future reference)

A condensed timeline of real issues hit and fixed during bring-up, since
several of these are easy to reintroduce accidentally:

1. **Copied Nucleo demo files verbatim** — inherited wrong LED pin (PA5, not
   PC13) and wrong HSE value (8MHz, not 25MHz) without touching them.
   Lesson: always diff copied template files against the target board
   before building.

2. **`STM32CubeProgrammer` "select start address" error on `.bin`** — normal
   behavior; `.bin` has no embedded address, needs `0x08000000` typed in
   manually, or use `.hex` instead which has it baked in.

3. **PLL errors on 3 different sub-PLLs simultaneously** (main, PLLI2S,
   PLLSAI) after changing `PLLM` for the new crystal — all three share the
   same input divider on STM32F4; only the main PLL's N/P/Q were
   recalculated initially.

4. **Root cause of the PLL errors was actually `STM32_PLLSRC =
   STM32_PLLSRC_HSI`** (PLL was silently sourcing off the 16MHz internal
   oscillator this whole time, not the crystal) — `16MHz / 25 = 640kHz`,
   below the minimum PLL input, which is what actually threw the errors.
   This also revealed the earlier "working" blink test was never actually
   running off the 25MHz crystal at all.

5. **`external/mavlink/common/mavlink.h` not found`** — turned out to be a
   stale build cache issue, not a missing file or wrong path; `make clean`
   resolved it.

6. **Linker errors: undefined reference to `mavlink_bridge_*`** —
   `mavlink_bridge.c` was never added to the Makefile's `CSRC` list
   alongside `main.c`.

7. **LED freezing partway through the power-on blink sequence** — root
   cause was stack overflow from two sources: `waLed`'s working area (128
   bytes) was too small, and later, large `mavlink_message_t` /
   `MAVLINK_MAX_PACKET_LEN` buffers being allocated **on the stack** inside
   `mav_send()` and the RX thread — moved to `static` storage instead of
   local stack variables, plus increased `waMavRx`/`waLed` working-area
   sizes. `CH_DBG_ENABLE_ASSERTS`/`CH_DBG_ENABLE_STACK_CHECK` were also
   turned on (both were `FALSE` by default in this template, meaning the
   original overflow was corrupting memory silently rather than trapping
   cleanly).

8. **FC TX/RX wires physically swapped** — resolved by explicitly mapping
   FC TX → STM32 RX (PA10) and FC RX → STM32 TX (PA9), not connecting
   TX-to-TX.

---

## 11. Rebuilding from scratch

```bash
cd ~/stm32-sandbox/ChibiOS/demos/STM32/mavlink-uart
make clean
make
# flash:
STM32_Programmer_CLI -c port=usb1 -w build/ch.bin 0x08000000 -v -rst
```

Expected behavior on power-up: LED blinks 3 times, then goes dark. Once the
FC is connected and sending valid heartbeats over USART1, LED should start
a steady slow blink within ~1-3 seconds. Confirm via Mission Planner's
MAVLink Inspector (`Ctrl+F`) — look for `Vehicle 100 / Comp 191
MAV_COMP_ID_ONBOARD_COMPUTER` sending `HEARTBEAT` at 1.0Hz.
