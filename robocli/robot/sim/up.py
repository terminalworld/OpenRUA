"""Start the simulated robot's container. Host side.

The one place that knows how a simulated robot is started: the image,
the mounts, the render-thread cap, and the bridge command inside the
container. Returns a ``BridgeClient`` over the process's stdin/stdout
(stderr streams into ``log_path``): the caller's only ongoing
relationship with the robot is that line. The pipe is held by the
parent alone, so no network endpoint exists for the agent to find, and
when the parent exits the bridge sees EOF and shuts itself down.

Consumes data only: ``config_path`` is the trial's resolved config file
and ``peers_xml`` the rendered DDS peers profile, both computed once by
the runner and handed in.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
from pathlib import Path

from robocli.robot.sim.client import BridgeClient


def up(
    name: str,
    image: str,
    config_path: str,
    task_suite: str,
    task_id: int,
    simulator: str,
    code_root: str,
    venv: str | None = None,
    uv_dir: str | None = None,
    log_path: Path | None = None,
    moveit_log: str | None = None,
    network: str | None = None,
    static_peer: str | None = None,
    ros_domain: int = 0,
    gpus: bool = False,
    extra_env: list[str] | None = None,
    resources: dict | None = None,
    peers_xml: str | None = None,
    record: str | None = None,
    record_cameras: tuple[str, ...] = (),
) -> BridgeClient:
    """docker-run the container with the bridge as its first process.
    Returns the handle (not yet waited for).

    Four host directories are mounted at their own paths: the robocli
    code (``code_root``, so the simulator venv imports the same package),
    uv's interpreter store (the venv python is a symlink into it), the
    simulator checkout (venv + simulator), and the directory holding the
    resolved config file (the robot's config and any file it names by path).
    ``record`` is a host directory for the bridge's camera frames (under
    the config directory, so the same mount carries it); ``record_cameras``
    narrows the cameras to the names given.
    """
    # The simulator venvs' python is a symlink into uv's interpreter
    # store; mount it read-only at the same path. Derived from the
    # current user's home, never hardcoded (runs move across machines
    # and accounts).
    uv_dir = uv_dir or str(Path("~/.local/share/uv").expanduser())
    if static_peer and not peers_xml:
        raise ValueError(
            "static_peer needs the rendered peers profile too; the "
            "runner renders it (bringup.peers_profile) and "
            "passes peers_xml")
    config_dir = Path(config_path).resolve().parent
    # Concurrent trials share one docker network; DDS domains keep
    # their graphs from cross-talking.
    env = ["-e", f"ROS_DOMAIN_ID={ros_domain}"]
    if static_peer:
        env += ["-e", f"ROS_STATIC_PEERS={static_peer}"]
    if peers_xml:
        # ROS_STATIC_PEERS exists only from Iron on; Humble's Fast DDS
        # ignores it; the profile file is the portable form. Written
        # next to the config, which the container sees through the same
        # mount, and named to Fast DDS by its environment variable.
        peers_file = config_dir / "fastdds-peers.xml"
        peers_file.write_text(peers_xml)
        env += ["-e", f"FASTRTPS_DEFAULT_PROFILES_FILE={peers_file}"]
    net = ["--network", network] if network else []
    # GPU rendering (host needs nvidia-container-toolkit): MUJOCO_GL
    # stays "egl" either way; with the driver injected, EGL picks the
    # GPU; without, it falls back to llvmpipe. Physics is CPU in both
    # cases, so results are comparable across render devices.
    gpu_args = (
        ["--gpus", "all", "-e", "NVIDIA_DRIVER_CAPABILITIES=graphics,utility"]
        if gpus else []
    )
    # Software-render thread cap, derived from this host (runs move
    # across machines): one sim's llvmpipe may use at most a quarter of
    # the cores, clamped to [4, 16], so a few concurrent sims do not
    # thrash and the host keeps headroom. No effect under GPU rendering.
    # Override with resources.render_threads: <int> | "off".
    rt = (resources or {}).get("render_threads", "auto")
    if rt == "auto":
        rt = max(4, min(16, (os.cpu_count() or 8) // 4))
    if rt != "off":
        env += ["-e", f"LP_NUM_THREADS={int(rt)}"]
    venv = venv or f"{simulator}/.venv-libero"
    # The image's entrypoint (ros_entrypoint.sh, from the ros base
    # image) sources ROS and execs this command. The bridge writes its
    # own simulator settings; nothing about the simulator's layout
    # lives here.
    bridge = [
        f"{venv}/bin/python", "-m", "robocli.robot.sim.bridge.main",
        "--config", str(config_path),
        "--task-suite", task_suite, "--task-id", str(task_id),
        *(["--moveit-log", moveit_log] if moveit_log else []),
        *(["--record", str(record)] if record else []),
        *(["--record-cameras", ",".join(record_cameras)] if record_cameras else []),
    ]
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    proc = subprocess.Popen(
        [
            "docker", "run", "-i", "--rm", "--name", name,
            *net, *gpu_args, *env, *(extra_env or []),
            *_mounts(code_root, simulator, str(config_dir)),
            "-v", f"{uv_dir}:{uv_dir}:ro",
            image, *bridge,
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        # stderr streams to the log from the first instant, so a boot
        # failure's message is never lost. One file, whole life:
        # bridge.log.
        stderr=(open(log_path, "w") if log_path is not None
                else subprocess.DEVNULL),
        text=True,
    )
    return BridgeClient(proc, name)


def _mounts(*dirs: str) -> list[str]:
    """``-v d:d`` for each distinct directory, outermost first (docker
    accepts nested mounts; the order only keeps the command readable)."""
    seen: list[str] = []
    for d in sorted(str(Path(d).resolve()) for d in dirs):
        if d not in seen:
            seen.append(d)
    out: list[str] = []
    for d in seen:
        out += ["-v", f"{d}:{d}"]
    return out


def main() -> int:
    """Standalone: start one robot and wire this terminal onto its line.

    Type one JSON question per line ({"cmd": "success"}, {"cmd": "reset",
    "init_state_id": 0}, ...); answers come back one per line. Exiting
    (Ctrl-D) is pipe EOF: the bridge shuts itself down. For debugging;
    trials go through the runner."""
    ap = argparse.ArgumentParser(
        description="start a simulated robot; the terminal becomes its "
                    "control line (Ctrl-D shuts it down)")
    ap.add_argument("--name", required=True, help="robot container name")
    ap.add_argument("--image", required=True)
    ap.add_argument("--config", required=True,
                    help="resolved config yaml (the trial's suite view)")
    ap.add_argument("--task-suite", required=True)
    ap.add_argument("--task-id", type=int, required=True)
    ap.add_argument("--simulator", required=True,
                    help="simulator checkout root (the dir holding .venv-*)")
    ap.add_argument("--venv", default=None,
                    help="simulator venv path (default <simulator>/.venv-libero)")
    ap.add_argument("--code-root", required=True,
                    help="directory holding the robocli package (mounted "
                    "so the simulator venv imports the same code)")
    ap.add_argument("--log", default=None,
                    help="bridge.log path (default: stderr discarded)")
    ap.add_argument("--moveit-log", default=None)
    ap.add_argument("--network", default=None)
    ap.add_argument("--ros-domain", type=int, default=0)
    ap.add_argument("--gpus", action="store_true")
    ap.add_argument("--record", default=None, metavar="DIR",
                    help="record every sim step's cameras under DIR")
    ap.add_argument("--record-cameras", default="",
                    help="comma-separated camera names for --record")
    args = ap.parse_args()

    proc = up(
        name=args.name, image=args.image, config_path=args.config,
        task_suite=args.task_suite, task_id=args.task_id,
        simulator=args.simulator, venv=args.venv,
        code_root=args.code_root,
        log_path=Path(args.log) if args.log else None,
        moveit_log=args.moveit_log, network=args.network,
        ros_domain=args.ros_domain, gpus=args.gpus, record=args.record,
        record_cameras=tuple(c for c in args.record_cameras.split(",") if c),
    )

    def pump_answers() -> None:
        for line in proc.proc.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()

    t = threading.Thread(target=pump_answers, daemon=True)
    t.start()
    print(f"robot {args.name} starting; ask it JSON, one per line "
          "(Ctrl-D = power off)", file=sys.stderr)
    try:
        for line in sys.stdin:
            proc.proc.stdin.write(line)
            proc.proc.stdin.flush()
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        try:
            proc.proc.stdin.close()
        except OSError:
            pass
        proc.proc.wait(timeout=30)
    return proc.proc.returncode or 0


if __name__ == "__main__":
    raise SystemExit(main())
