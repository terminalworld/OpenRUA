"""Engines: everything the bridge asks a physics engine for.

The ROS side and the control line know joints, bodies, cameras and
control ticks; how a particular engine addresses a joint, renders a
camera or assembles an action lives here, one module per engine. A
simulator's yaml names its engine under ``entry_point`` and the
resolved config carries the result as ``machine.backend.simulator.
engine`` (a module path or an absolute file path), which ``load()``
imports.

Adding an engine = one module exposing ``ENGINE``, a callable
``ENGINE(env, cfg)`` whose result implements every name in
``ENGINE_INTERFACE`` (test-enforced). The env is whatever the loader
built (the benchmark's own class, unwrapped by the engine as it sees
fit); the cfg is the resolved config. Conventions the interface fixes
so the ROS side stays engine-free: positions in metres and radians,
poses in the world frame, camera poses in the ROS optical convention
(+Z forward, +X right, +Y down), rendered rows top-down, depth metric.

Zero ROS imports; engine imports live inside the module that needs
them. Zero knowledge of the sibling environments, ros and rpc
packages: the env arrives as a parameter and the bound engine travels
onward the same way (main.py passes it).
"""

from __future__ import annotations

from .. import plug

# The plug shape every engine implements (a duck-typed contract, like LOADER_INTERFACE).
ENGINE_INTERFACE = (
    # the world
    "joints",        # -> [(joint name, qpos index, qvel index)] for every joint
    "time",          # -> simulated seconds
    "qpos",          # -> the position array joints() indexes (a view or a copy: read anew before use)
    "qvel",          # -> the velocity array joints() indexes
    "effort",        # -> actuator effort per velocity index
    "body_pose",     # name -> (pos[3], quat[w x y z], mat[3x3]); KeyError when absent
    "site_pos",      # name -> pos[3]; KeyError when absent
    "objects",       # -> {name: [floats]} privileged object poses (debug truth, never published)
    # cameras
    "camera_names",  # -> [name] the scene defines
    "camera_pose",   # name -> (pos[3], mat[3x3]) in the ROS optical convention; KeyError when absent
    "camera_size",   # name, width, height -> (width, height) it renders at when asked for that size
    "render",        # name, width, height, depth=False -> rgb[h,w,3] uint8 (and depth[h,w] float32 metric)
    "intrinsics",    # name, width, height -> K[3x3] for that size
    # actuation
    "control_dt",    # -> seconds one step advances the world
    "mobile",        # -> bool: the robot has a driven base
    "bind_arms",     # [(qpos indices, hand body name)] per arm -> None; before any of the below
    "has_gripper",   # arm index -> bool
    "ee_wrench",     # arm index -> (force[3], torque[3]) estimated at the end effector
    "apply_tuning",  # -> None: re-apply controller gains (a reset may rebuild controllers)
    "action",        # arm index, dq, grips, base_vel -> the env action (other arms hold)
    "step",          # action -> None (advance one control tick through env.step)
    "fk",            # arm index, q or None -> (hand pos[3], hand mat[3x3]) without moving the world
)


def load(spec: str):
    """The ``ENGINE`` callable of a resolved engine spec: a module path
    (``openrua.robot.sim.bridge.engines.robosuite``) or an absolute
    ``.py`` file (an engine of your own, copied next to the config)."""
    engine = getattr(plug.module(spec), "ENGINE", None)
    if engine is None:
        raise ValueError(f"{spec} exposes no ENGINE")
    return engine


def bind(spec: str, env, cfg: dict):
    """``ENGINE(env, cfg)`` of the spec, checked against the interface."""
    engine = load(spec)(env, cfg)
    plug.check(engine, ENGINE_INTERFACE, spec, "ENGINE")
    return engine
