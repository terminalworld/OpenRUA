"""MoveIt graph-side stack for the bridge (P2; locked: MoveIt = mainline
IK home, franka-shaped config copied from a stock package).

Launches, all with use_sim_time (the bridge's /clock rules the graph):

- robot_state_publisher: owns the arm-chain TF from /joint_states + the
  stock panda URDF (moveit_resources; joint names match the bridge's
  published panda_* names exactly). The bridge keeps world->panda_link0
  and camera frames; its own hand TF is switched off when this stack runs
  (TF single-parent rule).
- move_group: planning + /compute_ik; executes through the bridge's
  FollowJointTrajectory and GripperCommand servers (controller config
  below points at those action names; which are the stock names anyway).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    import pathlib

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        # Stock SRDF + two disabled collision pairs for LIBERO's folded home
        # config (see comment inside panda.srdf); the only local deviation.
        .robot_description_semantic(
            file_path=str(pathlib.Path(__file__).resolve().parent / "panda.srdf")
        )
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Point execution at the bridge's action servers (stock names).
    controllers = {
        "moveit_simple_controller_manager": {
            "controller_names": ["panda_arm_controller", "panda_hand_controller"],
            "panda_arm_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": [f"panda_joint{i}" for i in range(1, 8)],
            },
            "panda_hand_controller": {
                "type": "GripperCommand",
                "action_ns": "gripper_action",
                "default": True,
                "joints": ["panda_finger_joint1"],
                "parallel": True,
            },
        },
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
        "trajectory_execution.allowed_execution_duration_scaling": 5.0,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }

    sim_time = {"use_sim_time": True}

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="log",
                parameters=[
                    moveit_config.robot_description,
                    sim_time,
                    # Paused world: the bridge re-publishes /joint_states
                    # with an UNCHANGED sim stamp between commands; rsp
                    # dedups on timestamp and would go silent; late TF
                    # listeners then never see the arm chain. Publish on
                    # every message instead.
                    {"ignore_timestamp": True},
                ],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="log",
                # A machine's driver daemon restarts when it crashes
                # (systemd semantics). 2026-08-12 GatherVegetables: a
                # silent singleton move_group death at minute 7 cost the
                # agent 127min of hand-rolled kinematics and got booked
                # as an agent failure.
                respawn=True,
                respawn_delay=2.0,
                parameters=[moveit_config.to_dict(), controllers, sim_time],
                # remap so the hand controller's action ns resolves to the
                # bridge's /franka_gripper/gripper_action
                remappings=[
                    (
                        "/panda_hand_controller/gripper_action",
                        "/franka_gripper/gripper_action",
                    )
                ],
            ),
        ]
    )
