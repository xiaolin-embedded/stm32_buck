# DPOS G4 BUCK — STM32G474 digital power supply

Firmware for the **DPOS G4 BUCK V2** board: a 200 kHz synchronous buck with a
100 kHz digital CC/CV loop, a serial shell and a 1 Hz telemetry monitor.
No OLED, USB or SPI2 code is included.

## Specifications

| Item | Value |
|---|---|
| Input voltage | **12–48 V DC** nominal; 50 V input capacitors, 60 V MOSFETs |
| Minimum practical input | ~12 V (9 V gate-drive rail / LM5109 UVLO) |
| Output voltage | **1–40 V** (buck only, Vout < Vin) |
| Maximum output current | **5 A** (22 µH / 5 A inductor) |
| Current limit | `iset` 0–5000 mA; fast trip > 6 A, sustained trip > ilim+0.5 A for 2 ms |
| Switching frequency | 200 kHz HRTIM, 118 ns dead time |
| Control loop | 100 kHz average-current / voltage double loop |
| MCU clock | 170 MHz from the 8 MHz HSE, automatic HSI fallback |
| Shell | USART2, 115200 8N1, PA2/PA3 |
| RTOS | FreeRTOS; control loop in the HRTIM interrupt, one shell task |

## Features

- HRTIM synchronous buck with dead time and ADC triggering at mid on-time
- Incremental PI(D) voltage and current loops; the lower duty wins (CV/CC)
- Soft start and controlled ramp-down at 100 V/s
- Input UV/OV, output OV, average current limit, fast short-circuit trip
- Latched faults with reset command and RGB indication
- Serial shell: version, status, setpoints, power on/off, 1 Hz monitor
- Compile-time calibration gains for Vin/Vout/Iout (measured board values)

## Hardware

| Signal | Pin | Notes |
|---|---|---|
| High-side gate | PA8 | HRTIM_CHA1, AF13 |
| Low-side gate | PA9 | HRTIM_CHA2, AF13 |
| Iout sense | PA1 | ADC2_IN2, 0.03 Ω shunt, ≈525 mV/A |
| Vout sense | PA6 | ADC2_IN3, 33k/2k divider (17.5:1 nominal) |
| Vin sense | PA7 | ADC2_IN4, 33k/2k divider (17.5:1 nominal) |
| Shell TX/RX | PA2/PA3 | USART2, AF7, 115200 8N1 |
| RGB LED | PB0/PB1/PB2 | red / green / blue, active low |

LED behaviour:

| LED | Meaning |
|---|---|
| Blue | 2 Hz heartbeat in every state (firmware alive) |
| Green | steady while the output is enabled (`SOFT_START` or `RUN`) |
| Red | steady while a fault is latched |

## Quick start

Toolchain: `arm-none-eabi-gcc`, CMake ≥ 3.22 and Ninja.

```sh
cmake --preset Debug
cmake --build --preset Debug
```

The ELF is `build/Debug/stm32_buck.elf`.  Full build, flash and serial
instructions are in [docs/build-and-flash.md](docs/build-and-flash.md).

```sh
# flash with STM32CubeProgrammer
STM32_Programmer_CLI -c port=SWD -w build/Debug/stm32_buck.elf -v -rst
```

Serial terminal (115200 8N1):

```sh
picocom -b 115200 /dev/ttyUSB0
```

First run:

```
buck> version
buck> status
buck> vset 5000
buck> iset 3000
buck> poweron
buck> monitor
```

## Shell commands

| Command | Description |
|---|---|
| `version` | firmware name, version, board, build date, loop rates |
| `status` | one-shot Vin/Vout/Iout/Pout, setpoints, state, duty, gains |
| `vset <mV>` | set output voltage, 1000–40000 mV |
| `iset <mA>` | set current limit, 0–5000 mA |
| `poweron` | enable the output (refused while a fault is latched) |
| `poweroff` | disable the output |
| `monitor` | 1 Hz telemetry stream; `q` exits silently |
| `reset` | stop the output and clear a latched fault |
| `help` | command list |

Line editing: **Up/Down** recall the last 10 commands, **Tab** completes
command names, **Backspace** edits.  Commands end with CR or LF.

Monitor line example:

```
Vin=15.050 V Vout=5.002 V Iout=1.002 A Pout=5.012 W RUN/CV D=33.4%
```

`RUN/CV` means the voltage loop is in control; `RUN/CC` means the current
limit is active.

## Control, CV/CC and protection

Both loops run continuously; the applied duty is `min(duty_voltage,
duty_current)`.  A load that demands more than `iset` automatically moves the
supply into CC (voltage falls, current holds at the limit); when the load
drops, the voltage loop takes over again.  There is no mode command.  See
[docs/digital_power.md](docs/digital_power.md) for the equations and tuning
procedure.

- Soft start: `vref` ramps at 1 mV / 10 µs (100 V/s), both up and down.
- VOUT_OV: `max(vref, vset) + 1.5 V`, capped at 40 V, so a normal downward
  setpoint change cannot false-trip.
- IOUT_OC: `ilim + 0.5 A` must persist for 2 ms; above 6 A trips immediately
  (hard short / current-loop failure).
- Faults latch; `reset` (or a power cycle) clears them.

## Calibration

The sense scaling is fixed at build time in
[Core/Inc/app_config.h](Core/Inc/app_config.h):

```c
#define CAL_VIN_GAIN   0.985f
#define CAL_VOUT_GAIN  0.94f
#define CAL_IOUT_GAIN  1.0f
#define IOUT_ZERO_DEADBAND_MA 10u
```

The gains were measured on this board against a DMM (Vin 15.269 → 15.05 V,
Vout 5.00 → 4.70 V).  Adjust and rebuild if the divider or board revision
changes.  Currents below 10 mA display as zero.

## Limits and notes

- 5 A is the current limit of the 22 µH / 5 A inductor.  Continuous power is
  thermal-limited: the vendor runs 5 A at 10 V (50 W) and 3 A at 24–28 V
  (72–84 W).  Do not assume 40 V × 5 A.
- The current limit is a 100 kHz average-current loop, not a cycle-by-cycle
  hardware peak-current comparator.  The COMP3/DAC peak circuit (PA0/PA4) is
  unused.
- OLED, USB and SPI2 functions are not part of this firmware; those pins are
  not configured.
- At no load and output off, a small residual voltage (~0.3 V) can appear
  from high-side MOSFET leakage.  A 1 kΩ–10 kΩ bleeder on the output
  discharges it; it is harmless with any normal load.
- `stm32_buck.ioc` is **stale** and must not be regenerated: the peripheral
  initialisation is hand-written in `Core/Src/app_*.c` and
  `Core/Src/stm32g4xx_hal_msp.c`.  See the repository notes in
  [docs/build-and-flash.md](docs/build-and-flash.md).
- Reverse polarity is not protected; do not reverse the input.

## Documentation

- [docs/digital_power.md](docs/digital_power.md) — control law, CV/CC,
  protection and tuning procedure
- [docs/build-and-flash.md](docs/build-and-flash.md) — toolchain, build,
  flash, serial and troubleshooting
- [docs/design.md](docs/design.md) — architecture, timing, ADC, RTOS and
  verification notes

## Repository layout

```
Core/Inc/app_config.h          board/loop/protection constants
Core/Inc/app_power.h           power API and telemetry types
Core/Src/app_power.c           HRTIM + ADC + control ISR + protections
Core/Src/app_shell.c           USART2 shell, history, Tab completion
Core/Src/app_freertos.c        shell task and kernel hooks
Core/Src/main.c                clock, GPIO, init and kernel start
Core/Src/stm32g4xx_hal_msp.c   pin/DMA/NVIC setup
Core/Src/stm32g4xx_it.c        exception vectors and HAL timebase IRQ
cmake/                         CMake toolchain and generated source lists
docs/                          design, control and build documentation
Drivers/ Middlewares/          STM32G4 HAL/CMSIS and FreeRTOS
```
