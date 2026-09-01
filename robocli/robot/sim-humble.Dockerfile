# Sim container for the CaP-Bench leg (py3.10 == Humble native; the cap-x
# robosuite stack for this leg lives in its OWN py3.10 venv, .venv-capbench,
# volume-mounted like the substrate; NEVER the pilot's .venv-libero).
# Hosts: bridge process (robosuite env + rclpy node), MoveIt (graph-side),
# original predicates (in-process; ground truth never reaches the graph).
FROM ros:humble

# Mirror of sim-jazzy.Dockerfile (P0/P1-verified package set), Humble names.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.10-venv \
    libegl1 libgl1 libgles2 libglvnd0 libglx0 \
    libgl1-mesa-dri libglx-mesa0 libegl-mesa0 \
    ros-humble-control-msgs ros-humble-tf2-ros-py \
    ros-humble-moveit ros-humble-moveit-resources-panda-moveit-config \
    ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
