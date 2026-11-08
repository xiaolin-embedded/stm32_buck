# Build, flash and serial guide

This is the detailed companion to the quick start in the project
[README](../README.md).  The project builds with CMake presets and Ninja; no
IDE is required.

## 1. Prerequisites

Validated on this machine:

| Tool | Version |
|---|---|
| `arm-none-eabi-gcc` | 13.2.1 (Arm GNU Toolchain 13.2.rel1) |
| CMake | 3.28.3 |
| Ninja | 1.11.1 |

Ubuntu/Debian install:

```sh
sudo apt update
sudo apt install cmake ninja-build gcc-arm-none-eabi
```

Check:

```sh
arm-none-eabi-gcc --version
cmake --version
ninja --version
```

CMake must be at least 3.22 (the presets and the generated toolchain file use
`CMAKE_TRY_COMPILE_TARGET_TYPE` and object libraries).

## 2. Configure and build

From the project root:

```sh
cd /path/to/stm32_buck
```

### Debug

```sh
cmake --preset Debug
cmake --build --preset Debug
```

Output: `build/Debug/stm32_buck.elf`

### Release

```sh
cmake --preset Release
cmake --build --preset Release
```

Output: `build/Release/stm32_buck.elf`

Approximate sizes (current code):

| Preset | Flash | RAM |
|---|---|---|
| Debug | ≈ 58 KB | ≈ 18 KB |
| Release | ≈ 37 KB | ≈ 18 KB |

`app_power.c` is compiled with `-O2` in both presets so the 100 kHz control
ISR meets its 10 µs deadline even in a Debug build.  The rest of the Debug
build uses `-O0 -g3`.

### Incremental build

```sh
cmake --build --preset Debug
```

`ninja: no work to do` simply means no source is newer than the ELF.  Force a
rebuild with:

```sh
cmake --build --preset Debug --clean-first
# or
rm -rf build/Debug
cmake --preset Debug
cmake --build --preset Debug
```

The presets live in `CMakePresets.json`; the compiler flags and linker script
are in `cmake/gcc-arm-none-eabi.cmake`.  The HAL/CMSIS/FreeRTOS source list is
in `cmake/stm32cubemx/CMakeLists.txt`.

## 3. Flash

SWD connection: SWDIO, SWCLK, GND, 3V3 (do not power the board from the
programmer and the bench supply at the same time unless that is intended).
The vendor recommends CMSIS-DAP; ST-Link also works.

### STM32CubeProgrammer CLI

```sh
STM32_Programmer_CLI -c port=SWD \
    -w build/Debug/stm32_buck.elf -v -rst
```

### OpenOCD (CMSIS-DAP)

```sh
openocd -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg \
    -c "program build/Debug/stm32_buck.elf verify reset exit"
```

### OpenOCD (ST-Link)

```sh
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
    -c "program build/Debug/stm32_buck.elf verify reset exit"
```

Use the Release ELF for normal operation, the Debug ELF when debugging.

## 4. Serial console

USART2 is on the P2 header:

| Signal | Pin |
|---|---|
| TX | PA2 |
| RX | PA3 |
| GND | P2 pin 7/8 |

115200 8N1, no flow control.

```sh
picocom -b 115200 /dev/ttyUSB0
# or
minicom -D /dev/ttyUSB0 -b 115200
```

Find the port with `dmesg | tail` or `ls /dev/ttyACM* /dev/ttyUSB*`.
If permission is denied:

```sh
sudo usermod -aG dialout "$USER"
# log out and back in
```

The shell uses ANSI escape sequences for history and redraw; a plain serial
terminal (picocom/minicom) is fine.  If local echo is enabled in the terminal,
you may see your own characters twice; turn local echo off.

## 5. First boot

```
version
status
vset 5000
iset 2000
poweron
monitor
```

Expected: blue heartbeat, then green while the output is enabled; `monitor`
prints one line per second.  Type `q` to leave the monitor silently.

Do not connect a sensitive load before verifying Vout with a DMM.  The
output is disabled after reset.

## 6. Troubleshooting

| Symptom | Check |
|---|---|
| `ninja: no work to do` | Nothing changed; ELF is current. Use `--clean-first` to force. |
| `arm-none-eabi-gcc: not found` | Install the toolchain or fix `PATH`; delete `build/` and reconfigure. |
| CMake error about version/toolchain | CMake must be ≥ 3.22; delete `build/` after changing toolchain. |
| No shell output | USART2 PA2/PA3, 115200, GND, TX/RX crossed; `dialout` permission. |
| No PWM/output | Do not regenerate from the stale `.ioc`; verify `HAL_HRTIM_MspPostInit()` and the DLL calibration are in `MX_HRTIM1_Init()`. |
| Vout reads wrong | The calibration gains in `app_config.h` are board-specific; verify with a DMM. |
| Fault after a setpoint change | Downward changes ramp; if VOUT_OV still trips, check the `max(vref,vset)` OV limit. |
| Load step trips OC | The 2 ms debounce and 6 A fast trip are in `app_config.h`; a step producing > 6 A is intentionally protected. |

## 7. Repository and version control

The working tree is currently **not** a git repository.  The project includes
a `.gitignore` that excludes `build/`, object/ELF/map/`.su` files, editor
directories and `CMakeUserPresets.json`.

Suggested first commit:

```sh
cd /path/to/stm32_buck
git init
git add .
git commit -m "Initial DPOS G4 BUCK firmware: 200 kHz CC/CV buck, serial shell"
git tag v1.0.0
```

Before publishing:

- Choose a license for your application code and add a `LICENSE`.
- Keep the third-party notices: STM32G4 HAL (BSD-3-Clause), CMSIS
  (Apache-2.0), FreeRTOS (MIT).  A `NOTICE` or `LICENSES/` directory is a
  good place.
- Do not commit `build/`; it is ignored.
- Record the board revision and toolchain versions in the README or release
  notes (`arm-none-eabi-gcc 13.2.1`, CMake 3.28, Ninja 1.11).
- `stm32_buck.ioc` is stale: it does not match the hand-written peripheral
  initialisation.  Commit it only with a clear warning, or remove it from the
  repository so nobody runs CubeMX "Generate Code" on it.  The firmware uses
  `Core/Src/app_*.c` and `Core/Src/stm32g4xx_hal_msp.c` as the source of
  truth.
- The firmware version string is `FW_VERSION` in `Core/Inc/app_config.h`;
  keep it in sync with the git tag.
