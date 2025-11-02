# CU Robotics Firmware

Firmware for CU Robotics robots built around the PJRC Teensy 4.1. The code implements the real-time control loop, sensor interfaces, and communications layers that tie into the Hive control stack and Waggle telemetry. This document focuses on the commands and workflow you should follow before sending a change for review.

## Before You Start
- Linux or macOS host with `bash`, `git`, `make`, and `python3`.
- USB access to a Teensy 4.1 and permissions to use `tycmd` (the upload utility).
- Optional: [Doxygen](https://www.doxygen.nl/index.html) and [Graphviz](https://graphviz.org/) if you plan to rebuild docs locally.

## One-Time Setup
```bash
git clone git@github.com:CU-Robotics/firmware.git
cd firmware
git config --local core.hooksPath .githooks  # enables the branch-protection hook
make install                                 # installs TyTools/tycmd and the Arm GNU toolchain
```

Need Arduino CLI support? Run `./tools/install_arduino.sh` (idempotent, installs the same compiler toolchain).

## Daily Workflow
- `make` — builds the firmware with `-Werror` and writes artifacts to `build/` plus top-level `.elf`, `.hex`, and `.map` files.
- `make upload` — flashes the connected Teensy via `tycmd` (press the physical button if the bootloader does not auto-enter).
- `make monitor` — tails serial output using `tools/monitor.sh`.
- `make gdb` — launches `arm-none-eabi-gdb` after preparing the correct device path.
- `make help` — lists all supported targets.
- `make cdb` — regenerates `compile_commands.json` via Bear for IDE/clangd support.
- `make clean`, `make clean_src`, `make clean_libs`, `make clean_teensy4` — remove build artifacts selectively.
- `python tools/tcm_blame.py build/firmware.map 20` — inspect ITCM/DTCM usage when chasing memory pressure.

## Directory Layout
- `src/` — firmware sources grouped by concern (`comms`, `controls`, `sensors`, `filters`, `utils`, `main.cpp`).
- `libraries/` — vendored dependencies (treat as read-only unless you are updating a library).
- `teensy4/` — PJRC board files and linker scripts.
- `tools/` — install scripts, flashing/monitor utilities, GDB helpers, Waggle interceptor.
- `docs/` — Doxygen configuration and documentation guidelines.

## Documentation
- Run `doxygen Doxyfile` to regenerate HTML docs (output in `docs/html/index.html`).
- Follow the standards in `docs/README.md` when adding docblocks; `doxygen_warnings.txt` lists any missing coverage.
- The main site publishes from `main` at <https://cu-robotics.github.io/firmware/>.

## Contribution Checklist
- Format touched C/C++ files with `clang-format -i`.
- Run `make` (and `make upload` if the change affects hardware behaviour) before requesting review.
- Capture serial logs or bench notes when relevant and attach them to the PR.
- Keep documentation changes factual and scoped to what you validated.
- Coordinate structural or hardware-impacting changes with the firmware leads before merging.
