# Firmware 
This repo contains the source code for the CU Boulder Robotics Teams' microcontroller firmware. It runs on a PJRC Teensy 4.1 and interfaces with any device running `hive`.

## Documentation
You can find up-to-date documentation at [cu-robotics.github.io/firmware](cu-robotics.github.io/firmware/), compiled on merges to `main`.

You can find our documentation guidelines at [docs/README](docs/README.md), and how to access your documentation before being merged to main.

## Installation
To begin, clone the repository:
```bash
git clone git@github.com:CU-Robotics/firmware.git
```

Then, install dependencies:

`githooks` provide branch protection locally. Set the .githook/ directory with:
```bash
git config --local core.hooksPath .githooks
```

Install the required tools
```bash
make install
```

## Usage

Now, navigate to the main directory, and run:

```bash
make
```

This will build the current firmware. To upload, run:

```bash
make upload
```

There are a few other nice helper functions within the makefile. This will list them:

```bash
make help
```


## Contributing
This repo follows the CU Robotics code standard:
- Branches are categorized into three groups: `production`, `feature`, and `patch`.
- Only authorized members can merge to `production` branches, such as `main`. All merges must go through a review and merge request. Direct pushes to `production` branches are strictly forbidden.
- `feature` branches are named `feature-[featurename]` and contain major new features that require unit and integration testing.
- `patch` branches are named `patch-[patchname]` and contain bugfixes and/or minor adjustments. Unit and integration testing is a soft requirement.
- All software is required to pass formatting and build tests before being merged into a `production` branch.
- Other branch-based style questions are answered in the [style guide](docs/README.md).

## Licensing
This repository uses the MIT License, which covers the code and tools written for our robot firmware. See separate copyright/licensing information for the external libraries used in the repo (in the libaries/ and teensy4/ directories).
