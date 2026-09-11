# The sandbox image: the robot's native surface and nothing else.
#
# The software list is what a real robot's development machine ships
# with. Documentation is what the installation naturally carries
# (--help, man, `ros2 interface show` definitions, rclpy sources); no
# curated snapshot, no self-written README. No internet access; that is
# enforced at the network layer (internal docker network), not here.
# ROS_DISTRO is a build arg: the sandbox runs the same distro as the
# robot (a robot's onboard computer runs the robot's own distro);
# cross-distro moveit_msgs definitions differ and break service-response
# deserialization.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}
ARG ROS_DISTRO

# --- openrua-local: packages.ros.org 不在本 pod 的出网白名单里,换成 mkrosrepo.sh
# 从公开镜像 dpkg-repack 出来的本地源。包列表未改动。见 RUNBOOK §0.0b-4。
RUN rm -f /etc/apt/sources.list.d/ros2*.list /etc/apt/sources.list.d/ros2*.sources \
    && echo 'deb [trusted=yes] http://127.0.0.1:8899/ ./' > /etc/apt/sources.list.d/local-ros.list
RUN apt-get update && apt-get install -y --no-install-recommends \
    # perception toolchain (standard image_pipeline members + CV libs)
    ros-${ROS_DISTRO}-image-view ros-${ROS_DISTRO}-cv-bridge \
    python3-numpy python3-scipy python3-opencv python3-pil \
    # tf2 CLI utilities (tf2_echo etc.)
    ros-${ROS_DISTRO}-tf2-tools ros-${ROS_DISTRO}-tf2-ros \
    # kinematics from the robot's own URDF: the graph publishes
    # /robot_description, and a real arm's development machine can read
    # it. PyKDL is pinned explicitly so it cannot vanish on a base-image
    # change; urdfdom-py is the released python URDF reader that turns
    # the published URDF into joint origins and axes (ROS 2 ships no
    # python URDF->KDL bridge). Without it the only way to answer "where
    # is the hand at joint angles q" is to hand-transcribe DH parameters.
    python3-pykdl ros-${ROS_DISTRO}-urdfdom-py \
    # interface contracts the graph speaks (introspectable via ros2 interface;
    # a service/action is callable only with its type definition installed)
    ros-${ROS_DISTRO}-control-msgs ros-${ROS_DISTRO}-moveit-msgs \
    # ordinary dev-machine editors/pagers (base image has coreutils/grep/sed)
    vim-tiny nano less \
    && rm -rf /var/lib/apt/lists/*

# Non-root robot account; /workspace is the per-container mounted volume
# (the user's $HOME and cwd; its only persistent surface). ROBOT_UID
# must match the host user (volume file ownership, caller cleanup; a
# mismatched uid cannot read 0600 mounted files); openrua build sandbox
# passes the current uid. The base image's stock `ubuntu` user is
# removed to free the slot.
ARG ROBOT_UID=1000
RUN userdel -r ubuntu 2>/dev/null; \
    useradd -m -s /bin/bash -u ${ROBOT_UID} robot \
    && mkdir -p /workspace && chown robot /workspace
# The shell comes provisioned (a real development machine sources ROS
# in its shell profile): interactive shells via the system bashrc,
# written while still root (a /workspace/.bashrc would be shadowed by
# the volume mount); non-interactive `bash -c` via BASH_ENV.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc
USER robot
WORKDIR /workspace
ENV HOME=/workspace
ENV BASH_ENV=/opt/ros/${ROS_DISTRO}/setup.bash
# The core ROS environment is also set as image ENV so non-bash paths
# (sh/dash via subprocess(shell=True), direct exec) are provisioned
# too: dash ignores BASH_ENV, and bash invoked as sh enters POSIX mode
# and reads nothing. Both distros' python layouts are listed;
# nonexistent entries are harmless.
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:${PATH} \
    AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO} \
    ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3 \
    LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib:/opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu \
    PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:/opt/ros/${ROS_DISTRO}/local/lib/python3.10/dist-packages:/opt/ros/${ROS_DISTRO}/lib/python3.12/site-packages

# Generic preinstall slot: an optional one-line shell command chain the
# caller wants in the image (extra tools, an agent's CLI). Empty (the
# default) = the bare terminal. The Dockerfile knows nothing about what
# the chain installs; build time only, since the running sandbox has no
# internet access. Installed into a system path (not /workspace, which
# the volume mount hides at `up`).
USER root
ARG PREINSTALL=""
RUN if [ -n "${PREINSTALL}" ]; then bash -c "${PREINSTALL}"; fi
USER robot
