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
# (the user's $HOME and cwd; its only persistent surface, so whatever the
# agent writes under ~ is in the session's record). Logs and caches the
# tools write on their own go to /home/robot instead, through each tool's
# own variable (ROS_HOME, XDG_CACHE_HOME, npm_config_cache), and stay out
# of the record.
# ROBOT_UID must match the host user (volume file ownership, caller
# cleanup; a mismatched uid cannot read 0600 mounted files); openrua
# build sandbox passes the current uid. The base image's stock `ubuntu`
# user is removed to free the slot.
ARG ROBOT_UID=1000
RUN userdel -r ubuntu 2>/dev/null; \
    useradd -m -s /bin/bash -u ${ROBOT_UID} robot \
    && mkdir -p /workspace && chown robot /workspace
# The shell comes provisioned (a real development machine sources ROS
# in its shell profile): /etc/openrua-ros.sh sources ROS, then every
# ROS workspace a robot's own install built under /opt/openrua/overlays
# (its vendor message definitions, say); interactive shells read it via
# the system bashrc, non-interactive `bash -c` via BASH_ENV.
RUN mkdir -p /opt/openrua/overlays \
    && printf '%s\n' \
       "source /opt/ros/${ROS_DISTRO}/setup.bash" \
       'for overlay in /opt/openrua/overlays/*/local_setup.bash; do if [ -f "$overlay" ]; then source "$overlay"; fi; done' \
       > /etc/openrua-ros.sh \
    && echo "source /etc/openrua-ros.sh" >> /etc/bash.bashrc
USER robot
WORKDIR /workspace
ENV HOME=/workspace \
    ROS_HOME=/home/robot/.ros \
    XDG_CACHE_HOME=/home/robot/.cache \
    npm_config_cache=/home/robot/.npm
ENV BASH_ENV=/etc/openrua-ros.sh
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
# caller wants in the image (the agents' CLIs, a robot's own interface
# packages). Empty (the default) = the bare terminal. The Dockerfile
# knows nothing about what the chain installs; build time only, since
# the running sandbox has no internet access. Installed into a system
# path (not /workspace, which the volume mount hides at `up`).
# PREINSTALL_STAMP changes when the chain installs whatever is current
# (an unpinned CLI): a new value misses the layer cache, so an old
# build's layer is never reused for "the current release".
USER root
ARG PREPARE=""
RUN if [ -n "${PREPARE}" ]; then bash -c "${PREPARE}"; fi
ARG PREINSTALL=""
ARG PREINSTALL_STAMP=""
RUN if [ -n "${PREINSTALL}" ]; then bash -c "${PREINSTALL}"; fi
# `python3 -I` (isolated mode; a coding agent may run a file it did not
# write that way) ignores PYTHONPATH, through which ROS 2 and every overlay
# hand over their Python packages: rclpy would be missing. A .pth file in
# the system site-packages is read in isolated mode too. This one lists
# the directories PYTHONPATH holds once ROS and the overlays installed
# above are sourced, one per line: computed from that environment here,
# after the install, and plain paths, the form Python keeps (PEP 829
# deprecates .pth lines that run code). Missing directories are skipped.
RUN bash -c 'source /etc/openrua-ros.sh && tr ":" "\n" <<< "$PYTHONPATH"' \
       > /usr/lib/python3/dist-packages/openrua-ros-path.pth
USER robot
