# syntax=docker/dockerfile:1
#
# Jetson (Ubuntu 24.04, arm64) build sandbox: tests tools/install_compiler.sh
# and a firmware build without a Jetson.
ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN if [ "$(uname -m)" != "aarch64" ]; then \
        echo "ERROR: this image must be built for arm64/aarch64 (got $(uname -m))." >&2; \
        echo "       Use docker compose, or pass --platform linux/arm64 to docker build." >&2; \
        exit 1; \
    fi

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

# Passwordless sudoer, like the default Jetson user. Non-root uid means a
# script that writes somewhere privileged without sudo fails here too.
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

# sync-repo copies /repo into the work volume. tools/compiler and build/ are
# excluded so the toolchain and objects survive between runs; *.dump is
# hundreds of MB.
RUN printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -euo pipefail' \
    'rsync -a --delete \' \
    "  --exclude '/build/' --exclude '/tools/compiler/' --exclude '*.dump' \\" \
    '  /repo/ /workspace/' \
    > /usr/local/bin/sync-repo \
    && chmod +x /usr/local/bin/sync-repo

# Mount point for the work volume. A fresh named volume inherits this
# ownership, so no chown is needed at runtime.
RUN mkdir -p /workspace && chown "${USER_UID}:${USER_GID}" /workspace

USER ${USERNAME}
WORKDIR /workspace

# .git comes from a mount owned by an unrecognized uid; without this the
# git_scraper step fails.
RUN git config --global --add safe.directory '*'

CMD ["bash"]
