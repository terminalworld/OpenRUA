"""Boot the simulated robot: environment + surface, one process.

``python -m robocli.robot.onboard.boot --config <resolved yaml>
--task-suite <name> --task-id <n>`` is the body's entry command,
started by the ground-side up verb (subprocess boundary; no imports).
Pure wiring:
every piece of logic lives in the sibling packages; this file only
creates them and passes them to each other.

The config arrives ALREADY RESOLVED to this trial's suite view (the
conductor computes it once and distributes the artifact; ruling
2026-08-16) -- nothing here imports any other part of robocli, so this
package bakes into the body image self-contained.

Runs INSIDE the body (imports rclpy); host-side code must never import
robocli.robot.onboard (layering contract).
"""

from __future__ import annotations

import argparse
import sys
import threading
import time


def main() -> None:
    # Reserve the real stdout for the control channel, then route fd 1 to
    # stderr; simulator libraries print to stdout (LIBERO does) and would
    # otherwise pollute the JSON channel; dup2 catches C-level prints too.
    import os

    ctl_out = os.fdopen(os.dup(1), "w")
    os.dup2(2, 1)
    sys.stdout = sys.stderr

    import rclpy
    import yaml
    from rclpy.executors import MultiThreadedExecutor

    from . import environment
    from .environment.simthread import SimJobRunner
    from .monitor.channel import ControlChannel
    from .monitor.monitor import Monitor
    from .ros_graph.node import GraphNode

    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--task-suite", required=True)
    ap.add_argument("--task-id", type=int, required=True)
    ap.add_argument("--moveit-log", default="/tmp/moveit.log",
                    help="host-visible path for the planning stack's log "
                    "(a mid-trial move_group death must be diagnosable "
                    "post-mortem; 2026-08-12 incident)")
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    if cfg.get("protocol", {}).get("clock", {}).get("mode", "paused") != "paused":
        raise NotImplementedError("free-running clock is the sub-experiment switch")

    # The main thread creates the env (and its EGL context) and becomes the
    # single sim-owner thread; everything else submits jobs to it.
    sim = SimJobRunner()
    loader = environment.get(cfg["task"]["benchmark"])
    env, task_ctx = loader.create(cfg, args.task_suite, args.task_id)
    env.reset()
    sim.bind_current_thread()

    rclpy.init()
    node = GraphNode(env, cfg, sim)
    # The graph re-aligns itself whenever the world is restored under it;
    # simulation only fires the slot, this wiring is the whole coupling.
    monitor = Monitor(env, task_ctx, loader, sim, on_reset=node.refresh)

    stop = {"flag": False}

    def _shutdown(_req: dict) -> dict:
        stop["flag"] = True
        return {}

    node.sensors.publish()

    # ROS spins on background threads; the main thread stays the sim owner
    # and executes the job queue.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # The robot boots COMPLETE: the planning stack is onboard software,
    # brought up by the robot itself (ruling 2026-08-16; nobody reaches
    # into the body to assemble it). The readiness wait services the sim
    # job queue so sensor timers keep flowing meanwhile; the examiner's
    # first question simply waits until the whole robot answers.
    moveit_proc = None
    if cfg.get("machine", {}).get("planning", {}).get("moveit"):
        import subprocess
        from pathlib import Path

        launch = (Path(__file__).resolve().parent / "ros_graph" / "launch"
                  / "moveit.launch.py")
        moveit_proc = subprocess.Popen(
            ["bash", "-c",
             f"exec ros2 launch {launch} > {args.moveit_log} 2>&1"])
        # Deterministic readiness: move_group logs this exact line once
        # its capabilities (compute_ik included) are up. A `ros2 service
        # list` probe answers from the ros2 daemon's cached graph and
        # misses freshly-launched nodes (2026-08-11).
        deadline = time.time() + 120.0
        while True:
            sim.run_pending(timeout=0.1)
            try:
                if "initialization complete" in open(args.moveit_log).read():
                    break
            except OSError:
                pass
            if time.time() > deadline:
                raise TimeoutError("move_group not ready (see moveit log)")

    # The line opens ONLY when the whole robot stands (env + graph +
    # planning stack): the caller's first answered question is the
    # completeness signal, and the precheck can never race a
    # still-warming MoveIt.
    ControlChannel(
        {**monitor.handlers(), "shutdown": _shutdown},
        on_eof=lambda: stop.update(flag=True),
        out=ctl_out,
    ).start()
    print("robot ready", file=sys.stderr, flush=True)
    try:
        while rclpy.ok() and not stop["flag"]:
            sim.run_pending(timeout=0.1)
    finally:
        if moveit_proc is not None:
            moveit_proc.terminate()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        env.close()


if __name__ == "__main__":
    main()
