# firmware
This repo contains the source code for the CU Boulder Robotics Teams' microcontroller firmware. It runs on a PJRC Teensy 4.1 and interfaces with any device running `buff-core`.

# Documentation
You can find up-to-date documentation at [cu-robotics.github.io](cu-robotics.github.io), compiled on merges to `main`.

You can find our documentation guidelines at [docs/README](docs/README.md), and how to access your documentation before being merged to main.

## Installation & Usage
To begin, download this repo: \
```bash
git clone https://gitlab.com/cu-robotics/firmware.git
```

Then, install dependencies: \
`TODO`

`buff-core` is required to build firmware. Clone the repo:
```bash
git clone https://gitlab.com/cu-robotics/buff-core.git
```

Once installed, source `buff-tools`:
```bash
cd buff-core
source buff-tools.bash
```

Then, build firmware:
```bash
buff -b fw
```

Finally, to flash firmware to a Teensy, use `-f` (also builds):
```bash
buff -f fw
```

## Contributing
This repo follows the CU Robotics code standard:
- Branches are categorized into three groups: `production`, `feature`, and `patch`.
- Only authorized members can merge to `production` branches, such as `main`. All merges must go through a review and merge request. Direct pushes to `production` branches are strictly forbidden.
- `feature` branches are named `feature-[featurename]` and contain major new features that require unit and integration testing.
- `patch` branches are named `patch-[patchname]` and contain bugfixes and/or minor adjustments. Unit and integration testing is a soft requirement.
- All software is required to pass formatting and build tests before being merged into a `production` branch.