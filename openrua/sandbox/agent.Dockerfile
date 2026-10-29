# A CLI environment for metadata and sign-in, without robot dependencies.
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends python3 \
    && rm -rf /var/lib/apt/lists/*

ARG ROBOT_UID=1000
RUN userdel -r ubuntu 2>/dev/null; \
    useradd -m -s /bin/bash -u ${ROBOT_UID} robot \
    && mkdir -p /workspace && chown robot /workspace

# Callers supply the same runtime and installation steps as their task image.
ARG PREPARE=""
RUN if [ -n "${PREPARE}" ]; then bash -c "${PREPARE}"; fi
ARG PREINSTALL=""
ARG PREINSTALL_STAMP=""
RUN if [ -n "${PREINSTALL}" ]; then bash -c "${PREINSTALL}"; fi

USER robot
WORKDIR /workspace
ENV HOME=/workspace \
    XDG_CACHE_HOME=/home/robot/.cache \
    npm_config_cache=/home/robot/.npm
