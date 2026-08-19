# syntax=docker/dockerfile:1
#
# Jetson build sandbox for the CU Robotics Teensy 4.1 firmware.
#
# Purpose: verify that tools/install_compiler.sh and a full firmware build work
# on an NVIDIA Jetson (Ubuntu 24.04 / L4T, arm64) without needing a Jetson in
# front of you. It does NOT flash hardware -- no tytools, no USB.
#
# The image deliberately ships only what a freshly flashed JetPack rootfs
# already has, so the install script has to do its own job. Do not pre-install
# the arm-none-eabi toolchain here: installing it is what we are testing.
ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# The Arm toolchain only ships an aarch64 Linux build, so a non-arm64 image
# would fail confusingly much later. Catch it at build time instead.
# docker-compose.yml already pins platform: linux/arm64, so this only trips for
# a hand-rolled `docker build` on an x86 host.
RUN if [ "$(uname -m)" != "aarch64" ]; then \
        echo "ERROR: this image must be built for arm64/aarch64 (got $(uname -m))." >&2; \
        echo "       Use docker compose, or pass --platform linux/arm64 to docker build." >&2; \
        exit 1; \
    fi

# Everything here is Priority: standard or already on a JetPack rootfs -- it is
# stripped from the minimal ubuntu:24.04 image, so adding it back keeps the
# baseline honest rather than failing on things a real Jetson would have.
#   build-essential: host gcc/g++ for tools/git_scraper.cpp (JetPack ships it)
#   wget, xz-utils:  used by tools/install_compiler.sh to fetch and unpack
#   rsync:           copies the read-only repo mount into the work volume
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        curl \
        git \
        make \
        rsync \
        sudo \
        wget \
        xz-utils \
    && rm -rf /var/lib/apt/lists/*

# --- user -------------------------------------------------------------------
# A passwordless sudoer, like the default user on a real Jetson. Running as a
# non-root uid keeps the test honest: a script that writes somewhere privileged
# without sudo fails here, exactly as it would on the Jetson.
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN if getent group "${USER_GID}" >/dev/null; then \
        groupmod -n "${USERNAME}" "$(getent group "${USER_GID}" | cut -d: -f1)"; \
    else \
        groupadd -g "${USER_GID}" "${USERNAME}"; \
    fi \
    && if getent passwd "${USER_UID}" >/dev/null; then \
        usermod -l "${USERNAME}" -d "/home/${USERNAME}" -m -g "${USER_GID}" \
                "$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
    else \
        useradd -u "${USER_UID}" -g "${USER_GID}" -m -s /bin/bash "${USERNAME}"; \
    fi \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/"${USERNAME}" \
    && chmod 0440 /etc/sudoers.d/"${USERNAME}"

# sync-repo copies the read-only /repo mount into the work volume. tools/compiler
# and build/ are excluded so the installed toolchain and object files survive
# between runs; *.dump keeps the multi-hundred-MB disassembly out of the copy.
RUN printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -euo pipefail' \
    'rsync -a --delete \' \
    "  --exclude '/build/' --exclude '/tools/compiler/' --exclude '*.dump' \\" \
    '  /repo/ /workspace/' \
    > /usr/local/bin/sync-repo \
    && chmod +x /usr/local/bin/sync-repo

# /workspace is the mount point for the work volume. A fresh named volume
# inherits this ownership from the image, so the unprivileged user can write to
# it without any chown at runtime.
RUN mkdir -p /workspace && chown "${USER_UID}:${USER_GID}" /workspace

USER ${USERNAME}
WORKDIR /workspace

# The repo is copied in from a mount, so its .git is owned by a uid git may not
# recognize; without this every build fails in the git_scraper step.
RUN git config --global --add safe.directory '*'

CMD ["bash"]
