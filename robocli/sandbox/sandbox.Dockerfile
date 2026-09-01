# Seat image: the robot's native surface and nothing else.
#
# Software list = the explicit assembly list (notes/04 fill-in 2): what a
# real robot's dev machine ships with. Docs are factory-pure (locked
# 2026-08-07): only what the installation naturally carries (--help, man,
# `ros2 interface show` definitions, rclpy sources); no curated snapshot,
# no self-written README. No internet access; enforced at the network
# layer (internal docker network), not here.
# ROS_DISTRO is a build arg: the sandbox runs the SAME distro as the
# machine's sim container (a robot's onboard computer runs the robot's
# own distro); cross-distro moveit_msgs/srv definitions differ and
# break service-response deserialization (2026-08-11 canary: Fast CDR
# exception calling Humble move_group from a Jazzy sandbox).
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}
ARG ROS_DISTRO

RUN apt-get update && apt-get install -y --no-install-recommends \
    # perception toolchain (standard image_pipeline members + CV libs)
    ros-${ROS_DISTRO}-image-view ros-${ROS_DISTRO}-cv-bridge \
    python3-numpy python3-scipy python3-opencv python3-pil \
    # tf2 CLI utilities (tf2_echo etc.)
    ros-${ROS_DISTRO}-tf2-tools ros-${ROS_DISTRO}-tf2-ros \
    # kinematics FROM the machine's own URDF: the graph publishes
    # /robot_description, and a real arm's dev machine can read it. PyKDL
    # was already here transitively (pinned explicitly now so it cannot
    # vanish on a base-image change); what was missing is the parser that
    # turns the published URDF into joint origins and axes. Without it the
    # only way to answer "where is the hand at joint angles q" is to
    # hand-transcribe DH parameters, which 34% of scanned trials did
    # (added 2026-08-20 as a factory-list gap, same class as the pilot's
    # missing numpy/PIL). ROS 2 ships no python URDF->KDL bridge (no
    # kdl_parser_py); urdfdom-py IS the released python URDF reader.
    python3-pykdl ros-${ROS_DISTRO}-urdfdom-py \
    # interface contracts the graph speaks (introspectable via ros2 interface;
    # a service/action is callable only with its type definition installed)
    ros-${ROS_DISTRO}-control-msgs ros-${ROS_DISTRO}-moveit-msgs \
    # ordinary dev-machine editors/pagers (base image has coreutils/grep/sed)
    vim-tiny nano less \
    && rm -rf /var/lib/apt/lists/*

# Non-root robot account; /workspace is the per-container mounted volume
# (the occupant's $HOME and cwd; its only persistent surface). ROBOT_UID
# should match the host user (volume file ownership / caller cleanup);
# the base image's stock `ubuntu` user is removed to free the slot.
# !!! ALWAYS build with --build-arg ROBOT_UID=$(id -u): a mismatched uid
# cannot read 0600 mounted files and every occupant session starts
# broken (2026-08-12 incident: default 1000 vs host 1003).
ARG ROBOT_UID=1000
RUN userdel -r ubuntu 2>/dev/null; \
    useradd -m -s /bin/bash -u ${ROBOT_UID} robot \
    && mkdir -p /workspace && chown robot /workspace
# The machine's shell comes provisioned (a real dev machine sources ROS
# in its shell profile): interactive shells via the SYSTEM bashrc,
# written while still root (review 2026-08-15 M3: the old
# /workspace/.bashrc was dead code, always shadowed by the workspace
# volume mount); non-interactive `bash -c` via BASH_ENV.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc
USER robot
WORKDIR /workspace
ENV HOME=/workspace
ENV BASH_ENV=/opt/ros/${ROS_DISTRO}/setup.bash
# ... and the core ROS env is ALSO baked as image ENV so non-bash paths
# (sh/dash via subprocess(shell=True), direct exec) are provisioned too;
# dash ignores BASH_ENV, and bash-invoked-as-sh enters POSIX mode and
# reads nothing (2026-08-12: occupants' sh subprocesses found no ros2 and
# concluded the machine manual lied). Both distros' python layouts are
# listed; nonexistent entries are harmless.
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:${PATH} \
    AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO} \
    ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3 \
    LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib:/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu \
    PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:/opt/ros/${ROS_DISTRO}/local/lib/python3.10/dist-packages:/opt/ros/${ROS_DISTRO}/lib/python3.12/site-packages

# Generic preinstall slot: an optional ONE-LINE shell command chain the
# caller wants baked into the seat (extra tools, an occupant's CLI, ...).
# Empty (the default) = the pure cockpit. The recipe knows nothing about
# what the chain installs; build-time only; the running sandbox has no
# internet access by design. Baked into a system path (NOT /workspace;
# that is hidden by the volume mount at `up`).
USER root
ARG PREINSTALL=""
RUN if [ -n "${PREINSTALL}" ]; then bash -c "${PREINSTALL}"; fi
USER robot
