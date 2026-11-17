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

To run tests:
```bash
make test
```

### Linting

`make tidy` runs clang-tidy on all of `src/`, and `make tidy-diff` runs it only on the files your branch changed compared to `origin/main`. The checks are in `.clang-tidy`, which clangd also reads. Both targets only print warnings and never fail. CI runs `make tidy-diff` on every pull request and shows the findings on the PR.

Install clang-tidy 18, the version CI uses:

```bash
# Ubuntu / Debian
sudo apt install clang-tidy-18
# macOS (then run with CLANG_TIDY=$(brew --prefix llvm@18)/bin/clang-tidy)
brew install llvm@18
```

The targets use `clang-tidy-18` if it is on your PATH, otherwise `clang-tidy`. Set `CLANG_TIDY=/path/to/clang-tidy` to use another one.


## Contributing
`main` is the production branch, which is required to be in an always working state.
- Only authorized members can merge to `main`. All merges must go through a review and pull request. Direct pushes to `main` branches are strictly forbidden.
- To contribute, create a new branch with a descriptive name for your change. Create a pull request when done, and provide a short explanation for the change.
- Pull Requests (PRs) should be as small as possible, dedicated to a specific purpose. A feature does not have to be complete for a pull request to be made, but what *is* added must be functional and tested. Multiple small PRs is better than one large PR.
    - Smaller PRs are significantly faster to review and test, so expect a slower review process for massive PRs.
- All software is required to pass [documentation](docs/README.md), build, and unit tests, and have all warnings resolved, before being merged into the `main` branch.

## Licensing
This repository uses the MIT License, which covers the code and tools written for our robot firmware. See separate copyright/licensing information for the external libraries used in the repo (in the libaries/ and teensy4/ directories).
