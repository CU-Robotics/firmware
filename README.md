# CU Robotics Firmware

Firmware for the CU Boulder Robotics Teams' robots built around a PJRC Teensy 4.1. The codebase implements the real-time control loop, sensor interfaces, configuration system, and communications pipeline that link the robot to the Hive competition stack.

## Table of Contents
- [Overview](#overview)
- [Repository Layout](#repository-layout)
- [Quick Start](#quick-start)
  - [Prerequisites](#prerequisites)
  - [Clone and Configure Git Hooks](#clone-and-configure-git-hooks)
  - [Install Tooling](#install-tooling)
  - [Build the Firmware](#build-the-firmware)
  - [Flash a Teensy](#flash-a-teensy)
- [Development Workflow](#development-workflow)
- [Configuration and Runtime Architecture](#configuration-and-runtime-architecture)
- [Documentation](#documentation)
- [Advanced Tools](#advanced-tools)
- [Contributing](#contributing)
- [License](#license)

## Overview
- Real-time firmware written in modern C++23 with Teensy-specific optimizations and a 1 kHz control loop defined in `src/main.cpp`.
- Extensible communications stack in `src/comms/` for high-speed HID packets, Ethernet telemetry, configuration handshakes, and SD-card persistence.
- Modular sensor, estimator, and controller management layers that coordinate Buff encoders, IMUs, LiDAR, current sensing, limit switches, and commanded outputs.
- Cross-platform build system driven by `Makefile`, targeting PJRC Teensy 4.1 with an embedded GNU Arm toolchain and auto-generated `git_info.h` build metadata.
- Integrated tooling for flashing (`tycmd`/TyTools), serial monitoring, GDB debug, DOxygen documentation generation, and memory-usage inspection.

## Repository Layout
- `src/` – Firmware source broken into feature areas (`comms`, `controls`, `sensors`, `filters`, `utils`, and the top-level `main.cpp` entry point).
- `libraries/` – Vendored third-party libraries (Adafruit sensor drivers, FlexCAN_T4, QNEthernet, SdFat, TeensyDebug, unity, etc.). Treat these as read-only unless you are upgrading a dependency.
- `teensy4/` – Core Teensy 4.1 support files and linker scripts provided by PJRC/TeensyDuino.
- `tools/` – Helper scripts and utilities for installing toolchains, flashing, serial monitoring, GDB prep, Waggle integration, memory analysis, and build metadata.
- `docs/` + `Doxyfile` – Doxygen configuration and documentation guidelines that feed the public site at [https://cu-robotics.github.io/firmware](https://cu-robotics.github.io/firmware/).
- `.githooks/` – Local branch-protection hook that blocks direct commits to `main`.
- `Makefile` – Primary build, flash, and helper targets.

## Quick Start

### Prerequisites
- Linux or macOS host with `bash`, `git`, `make`, `gcc`, and `python3` available.
- USB access to a PJRC Teensy 4.1 (Rev D/e) and a reliable power source.
- Network access for first-time tool installation (`make install` downloads packages and may call `sudo apt ...` on Linux).
- Optional but recommended: [Doxygen](https://www.doxygen.nl/manual/install.html) and [Graphviz](https://graphviz.org/) for local documentation builds.

### Clone and Configure Git Hooks
```bash
git clone git@github.com:CU-Robotics/firmware.git
cd firmware
# Enable branch protection locally so commits to main are prevented
git config --local core.hooksPath .githooks
```

### Install Tooling
Run once per machine (re-run when toolchain versions change):
```bash
make install
```
This script:
- Installs TyTools/`tycmd` (uses `apt` on Linux, clones the CU-Robotics fork on macOS).
- Downloads the Arm GNU toolchain into `tools/compiler/`.
- Sets up helper binaries used by the Makefile.

If you also need the Arduino CLI and Teensy cores for development utilities, run `tools/install_arduino.sh`. This script installs `arduino-cli`, adds the Teensy board packages, and chains into the compiler installation.

### Build the Firmware
```bash
make
```
Outputs are written to `build/` plus root-level copies of:
- `firmware.elf` – Linked ELF with debug symbols.
- `firmware.hex` – Intel HEX image used for flashing.
- `firmware.map`/`firmware.dump` – Useful for memory inspection and disassembly.

### Flash a Teensy
```bash
make upload
```
`tycmd` handles the transfer; press the physical program button if the Teensy does not auto-reboot. After flashing, the script waits briefly and launches the serial monitor (see `make monitor` below). If you are routing telemetry to the Waggle dashboard, use `tools/waggle-interceptor/make-upload.sh` instead to intercept the serial stream.

## Development Workflow
- `make help` – Lists the most common targets and their purpose.
- `make clean` / `make clean_src` / `make clean_libs` / `make clean_teensy4` – Remove build artifacts selectively.
- `make monitor` – Builds a lightweight C monitor and falls back to `tycmd monitor` to follow serial output from the Teensy.
- `make gdb` – Prepares a `gdb_commands.txt` with the correct `ttyACM` port and launches `arm-none-eabi-gdb` against the running firmware.
- `make kill` / `make restart` – Use TyTools to force bootloader mode or reboot the board.
- `make cdb` – Regenerates `compile_commands.json` through Bear so IDEs (clangd, VS Code, Zed, etc.) understand the cross-compilation arguments. Follow the prompts in the Makefile comments to point clangd at the bundled toolchain and include directories.
- `python tools/tcm_blame.py build/firmware.map 20` – Print the top memory users in ITCM/DTCM from the generated map file.
- `tools/get_tty_path.sh` – Discover the active Teensy serial interface when scripting.

During every build the Makefile compiles `tools/git_scraper.cpp`, which writes `src/git_info.h` (branch name, commit hash, and message) consumed by `print_logo()` at boot time. Avoid editing that header manually.

## Configuration and Runtime Architecture
- **Main Loop (`src/main.cpp`)** – Initializes Serial, debug channel (`TeensyDebug`), `CommsLayer`, the configuration layer, sensor/estimator/controller managers, the reference governor, and the watchdog. The control loop runs at 1 kHz (`LOOP_FREQ`) with a lower-rate heartbeat for status reporting.
- **Configuration (`src/comms/config_layer.*`)** – Waits for configuration packets from Hive or loads cached settings from `/config.pack` on the SD card. Successful configs are echoed back, persisted, and validated against the referee system ID to ensure robot-matching safety.
- **Communications (`src/comms/`)** – Abstractions for HID packets, Ethernet transport, and `Sendable<T>` wrappers that serialize structured data (robot state, configuration, telemetry, test payloads). `ConfigLayer` and runtime managers share a `CommsLayer` instance to push and receive Hive data.
- **Sensors (`src/sensors/`)** – Drivers and wrappers for Buff encoders, ICM20649 IMUs, REV through-bore encoders, VL53 LiDAR (`d200`), ACS712 current sensing, limit switches, and transmitter inputs (DR16, ET16S). `SensorManager` owns the lifetimes, dispatch, and comms bridging of each sensor type.
- **Controls and Estimation (`src/controls/`, `src/filters/`)** – Base classes and managers for estimators, controllers, and PID/low-pass filters. The estimator uses data from sensors and CAN to build `STATE_LEN` states, while the controller produces motor commands subject to the `Governor`.
- **Utilities (`src/utils/`)** – Profiling, timing helpers, math utilities, wrapping helpers, and the `Watchdog` implementation that supervises loop execution.
- **External Libraries (`libraries/` and `teensy4/`)** – Provide drivers, math helpers, networking stacks, and the Teensy core runtime. Keep their versions in sync with upstream whenever you upgrade hardware support.

Runtime logs, including the ASCII firmware banner, embed timestamps, Git metadata, and cycle counts to simplify debugging after deployment.

## Documentation
- Online docs are published from the `main` branch at [https://cu-robotics.github.io/firmware](https://cu-robotics.github.io/firmware/).
- Local builds require Doxygen and Graphviz:
  ```bash
  doxygen Doxyfile
  ```
  The output lives under `docs/html/index.html`. Watch `docs/doxygen_warnings.txt` for missing or malformed comments.
- Follow the standards described in `docs/README.md`: every class, function, struct, macro, and configuration entity should have clear `@brief`, `@param`, `@return`, and `@note` tags as appropriate.

## Advanced Tools
- **Waggle Interceptor** – `tools/waggle-interceptor/` contains a Rust-based uploader that captures `waggle graph ...` lines from the Teensy serial stream and forwards them to a Waggle dashboard instance.
- **Arduino CLI Utilities** – `tools/install_arduino.sh` installs `arduino-cli` and the Teensy boards package if you need Arduino tooling alongside the GNU Arm flow.
- **Custom Serial Monitor** – `tools/monitor.c` provides a fast TTY reader that pipelines into `monitor.sh`. It compiles locally on demand so `gcc` must be available.
- **Crash Analysis** – `tools/tcm_blame.py` and the generated `.dump` files make it easier to chase down ITCM/DTCM exhaustion or unexpected binary growth.

## Contributing
- `main` is production. Only authorized maintainers merge there, and direct pushes are blocked by the local `.githooks/pre-commit` hook.
- Create a feature branch per change set, keep pull requests focused and reviewable, and include context for reviewers. Several small PRs beat one monolithic update.
- Ensure every submission:
  - Builds cleanly with `make` (warnings are treated as errors).
  - Passes available unit tests once they are added (the `unity` framework is vendored for this purpose).
  - Updates or adds Doxygen comments so documentation stays complete.
  - Preserves formatting/clangd friendliness (regenerate `compile_commands.json` if the include graph changes).
- Document new sensors, estimators, or controllers alongside their configuration knobs to keep the public docs synchronized.

## License
This repository is released under the MIT License (`LICENSE`). Vendored libraries under `libraries/` and `teensy4/` retain their respective upstream licenses; consult those directories before redistributing bundled code.
