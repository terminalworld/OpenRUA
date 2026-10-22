"""Sandbox package unit tests: contract, workspace, recipes, verb guards.

Docker-touching paths (build/up/down execution) are covered by the
standalone smoke in the phase log; here = everything testable dry.
"""

from __future__ import annotations

from openrua.config import load_config

from pathlib import Path

import pytest
import yaml
from tests.conftest import requires_image

from openrua.sandbox import build as hbuild
from openrua.sandbox import workspace

REPO = Path(__file__).resolve().parents[2]
LIBERO_CFG = REPO / "openrua" / "configs" / "benchmarks" / "libero_pro.yaml"
PKG = Path(workspace.__file__).resolve().parent


def _cfg() -> dict:
    from openrua.config import normalize_arms
    return normalize_arms(load_config(LIBERO_CFG))


# ------------------------------------------------------------- workspace

def test_seed_copies_template_and_generates_manifest(tmp_path):
    cfg = _cfg()
    dest = tmp_path / "ws"
    workspace.seed(cfg, dest)
    assert (dest / "machine.yaml").exists()
    assert any(dest.glob("docs/*.md")) and any(dest.glob("tools/*/*.py"))
    manifest = yaml.safe_load((dest / "machine.yaml").read_text())
    kinds = {a["kind"]: a for a in manifest["actuators"]}
    assert kinds["joint_trajectory"]["port"] == cfg["machine"]["ports"]["trajectory"]
    assert "base_twist" not in kinds  # arm-only machine


def test_manifest_omits_gripper_for_gripperless_machine(tmp_path):
    # The wipe-suite shape: a machine whose config lists no gripper must
    # not promise one in machine.yaml. The
    # deletion happens BEFORE normalize, like a real suite override.
    from openrua.config import normalize_arms
    cfg = load_config(LIBERO_CFG)
    del cfg["machine"]["ports"]["gripper"]
    normalize_arms(cfg)
    out = tmp_path / "machine.yaml"
    workspace.write_machine_manifest(cfg, out)
    kinds = [a["kind"] for a in yaml.safe_load(out.read_text())["actuators"]]
    assert "gripper" not in kinds


def test_seed_is_noop_without_template(tmp_path):
    cfg = _cfg()
    cfg["machine"].pop("workspace_template")
    workspace.seed(cfg, tmp_path / "ws")
    assert not (tmp_path / "ws").exists()
    assert workspace.template_hash(cfg) == ""


def test_template_hash_is_deterministic_and_content_sensitive(tmp_path):
    # A template edit records a new hash (provenance keeps the old and
    # new apart), so no historical hash is pinned here. What must hold:
    # the hash is deterministic, and any byte change moves it.
    h1, h2 = workspace.template_hash(_cfg()), workspace.template_hash(_cfg())
    assert h1 == h2 and len(h1) == 64
    import shutil
    clone_root = tmp_path / "workspace_template"
    shutil.copytree(workspace.template_dir(_cfg()), clone_root)
    (clone_root / "README.md").write_text("mutated")
    orig = workspace._HERE
    try:
        workspace._HERE = tmp_path
        assert workspace.template_hash(_cfg()) != h1
    finally:
        workspace._HERE = orig


# --------------------------------------------------------------- recipes

def test_recipes_have_generic_slots_and_zero_agent_tokens():
    dockerfile = (PKG / "sandbox.Dockerfile").read_text()
    assert 'ARG PREINSTALL=""' in dockerfile  # optional, empty = the bare terminal
    low = dockerfile.lower()
    assert "claude" not in low and "anthropic" not in low \
        and "agent_install" not in low


# ------------------------------------------------------------ verb guards

def test_build_rejects_double_quotes_and_multiline():
    with pytest.raises(Exception):
        hbuild.build(preinstall='echo "hi"')
    with pytest.raises(Exception):
        hbuild.build(preinstall="line1\nline2")


def test_up_refuses_missing_image(tmp_path):
    from openrua.sandbox import up as hup
    with pytest.raises(Exception) as e:
        hup.up(_cfg(), tmp_path / "ws", image="openrua-definitely-missing-image")
    assert "up never builds" in str(e.value)


# --------------------------------------------------------------- package

def test_harness_imports_no_layer():
    # belt to test_layering's braces: the package imports no other unit
    import ast
    for py in PKG.glob("*.py"):
        tree = ast.parse(py.read_text())
        for node in ast.walk(tree):
            mods = [a.name for a in node.names] if isinstance(node, ast.Import) \
                else [node.module] if isinstance(node, ast.ImportFrom) and node.module else []
            for m in mods:
                assert not any(m.startswith(f"openrua.{layer}")
                               for layer in ("robot.sim.bridge", "runner")), \
                    f"{py.name} imports {m}"


def test_up_rejects_bad_internet_value(tmp_path):
    from openrua.sandbox import up as hup
    with pytest.raises(Exception) as e:
        hup.up(_cfg(), tmp_path / "ws", image="openrua-sandbox-jazzy", internet="all-open")
    assert "--internet" in str(e.value)


# ------------------------------------------------- robustness (live docker)

@requires_image("openrua-sandbox-jazzy")
def test_up_instructive_errors(tmp_path):
    from openrua.sandbox import up as hup
    # missing config file (the command line loads it)
    with pytest.raises(Exception) as e:
        hup.load(tmp_path / "nope.yaml")
    assert "config not found" in str(e.value)
    # bad template name in an otherwise good config
    cfg = _cfg()
    cfg["machine"]["workspace_template"] = "no_such_template"
    with pytest.raises(Exception) as e:
        hup.up(cfg, tmp_path / "ws", image="openrua-sandbox-jazzy")
    assert "no_such_template" in str(e.value)
    # proxy posture without a url
    with pytest.raises(Exception) as e:
        hup.up(_cfg(), tmp_path / "ws", image="openrua-sandbox-jazzy", internet="proxy:")
    assert "needs a url" in str(e.value)


@requires_image("openrua-sandbox-jazzy")
def test_up_surfaces_docker_stderr_on_bad_network(tmp_path):
    from openrua.sandbox import up as hup
    with pytest.raises(Exception) as e:
        hup.up(_cfg(), tmp_path / "ws", image="openrua-sandbox-jazzy",
               network="openrua-definitely-missing-net")
    assert "docker run failed" in str(e.value) \
        and "openrua-definitely-missing-net" in str(e.value)


def test_up_refuses_a_corpse(tmp_path):
    # alpine has no bash: docker accepts the run, the container dies at
    # once; up must refuse to hand back the name.
    import subprocess
    from openrua.sandbox import up as hup
    subprocess.run(["docker", "pull", "-q", "alpine:3.20"],
                   capture_output=True)
    with pytest.raises(Exception) as e:
        hup.up(_cfg(), tmp_path / "ws", image="alpine:3.20",
               network="openrua-internal", name="sandbox-corpse-test")
    # Either guard may catch it (docker refuses at exec, or the liveness
    # check finds an exited container); both are instructive errors.
    assert "docker run failed" in str(e.value) or "docker logs" in str(e.value)
    subprocess.run(["docker", "rm", "-f", "sandbox-corpse-test"],
                   capture_output=True)


def test_package_front_door():
    # From outside, the package is ONE unit: name + verb + --help suffice;
    # internal file names are implementation detail.
    import subprocess, sys
    h = subprocess.run([sys.executable, "-m", "openrua.sandbox", "--help"],
                       capture_output=True, text=True)
    assert h.returncode == 0 and "up" in h.stdout and "build" in h.stdout
    bad = subprocess.run([sys.executable, "-m", "openrua.sandbox", "nope"],
                         capture_output=True, text=True)
    assert bad.returncode == 2 and "unknown verb" in bad.stderr


def test_manifest_drive_never_clobbers_the_ros_type():
    # Regression: robocasa's machine.base.type "holonomic"
    # overwrote the ROS message type column; config extras must land in
    # "drive" and the typed column must win.
    import yaml as _yaml

    from openrua.sandbox.workspace import write_machine_manifest
    cfg = {"machine": {"ports": {"base_twist": "/cmd_vel"}, "cameras": {"list": []},
                       "base": {"type": "holonomic", "frame": "base_link"}}}
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "machine.yaml"
        write_machine_manifest(cfg, p)
        m = _yaml.safe_load(p.read_text())
    base = [a for a in m["actuators"] if a["kind"] == "base_twist"][0]
    assert base["type"] == "geometry_msgs/msg/Twist"
    assert base["drive"] == "holonomic"


def test_manifest_two_arms_two_of_everything(tmp_path):
    import yaml as _yaml

    from openrua.config import apply_suite_overrides, normalize_arms
    from openrua.sandbox.workspace import write_machine_manifest
    cfg = load_config(REPO / "openrua" / "configs" / "benchmarks" / "capbench.yaml")
    apply_suite_overrides(cfg, "capbench_twoarm_lift")
    normalize_arms(cfg)
    out = tmp_path / "machine.yaml"
    write_machine_manifest(cfg, out)
    man = _yaml.safe_load(out.read_text())
    traj = [a for a in man["actuators"] if a["kind"] == "joint_trajectory"]
    grip = [a for a in man["actuators"] if a["kind"] == "gripper"]
    twist = [a for a in man["actuators"] if a["kind"] == "cartesian_twist"]
    assert [t["arm"] for t in traj] == ["left", "right"]
    assert len(grip) == 2 and len(twist) == 2
    assert traj[0]["port"] != traj[1]["port"]
    assert traj[0]["joints"][0].startswith("left")
    wrench = [s for s in man["sensors"] if s["kind"] == "wrench"]
    assert len(wrench) == 2
    assert not man.get("planning")   # machine.yaml promises no planner
    # each arm says its own frames: the machine's frames name one arm only
    assert all(t["base_frame"] and "hand_frame" in t for t in traj)
    assert traj[0]["base_frame"] != traj[1]["base_frame"]
    assert "base_frame: " in out.read_text() and "# this arm's root link" in out.read_text()
    assert "model: " in out.read_text() and "the camera as a person" not in out.read_text()


# --------------------------------------------- manifest explains itself
# What a gripper does with a commanded position is a per-machine fact:
# robosuite's bridge reads a goal as open or closed and sets no force
# limit, a Robotiq on a real arm holds any position and limits force. So
# it lives on the manifest, declared where the gripper is driven, and the
# docs and the field notes only define the keys.

def test_gripper_entry_carries_interface_and_behaviour(tmp_path):
    out = tmp_path / "machine.yaml"
    workspace.write_machine_manifest(_cfg(), out)
    grip = next(a for a in yaml.safe_load(out.read_text())["actuators"]
                if a["kind"] == "gripper")
    assert (grip["unit"], grip["open"], grip["closed"]) == ("m", 0.04, 0.0)
    assert grip["holds_partway"] is False and grip["effort_honoured"] is False


def test_manifest_defines_every_fact_key_it_carries(tmp_path):
    # A value without its definition makes the reader guess; every key
    # that carries a machine fact answers for itself, units included.
    out = tmp_path / "machine.yaml"
    workspace.write_machine_manifest(_cfg(), out)
    text = out.read_text()
    for key in ("unit", "open", "closed", "max_effort", "holds_partway",
                "effort_honoured", "limits_rad", "joints"):
        assert f"{key}:" in text, f"{key} missing from the manifest"
        line = next(ln for ln in text.splitlines()
                    if ln.lstrip().lstrip("- ").startswith(f"{key}:"))
        assert "#" in line, f"{key} carries a value but no definition"
    # definitions only: what this machine does is the value, not the note
    gripper = text.split("kind: gripper", 1)[1].split("- kind:", 1)[0]
    assert "NOT honoured" not in gripper and "metres" not in gripper
    assert yaml.safe_load(text)["schema"].startswith("openrua/")


def test_action_doc_does_not_promise_a_force_limited_gripper():
    doc = (PKG / "workspace" / "docs" / "30-action.md").read_text()
    assert "stalls on contact" not in doc
    assert "keeps squeezing" not in doc
    assert "holds_partway" in doc  # points at the manifest instead
    assert "PER-FINGER" not in doc and "open_m" not in doc


# ------------------------------------- simulation facts stay with simulators
# The generic workspace serves every machine. What only a simulator does
# (paused clock, on-demand rendering, queued base ticks, robosuite's
# observations) reaches the agent through machine.yaml: a simulation:
# section on simulated machines, and notes from whoever declared them.

def _real_cfg(tmp_path) -> dict:
    from openrua.config import compose, normalize_arms
    lab = tmp_path / "lab.yaml"
    lab.write_text(
        "type: panda\nmachine:\n"
        "  backend: {kind: real, discovery: {network: host}}\n  cameras: {list: []}\n"
        "  robot: {model: Lab Panda}\n"
        "  gripper: {holds_partway: true, effort_honoured: true}\n"
        "  notes: {gripper: [stalls when it holds an object]}\n")
    return normalize_arms(compose(str(lab), home=tmp_path).cfg)


def test_simulation_section_only_on_a_simulated_machine(tmp_path):
    workspace.write_machine_manifest(_cfg(), tmp_path / "sim.yaml")
    sim = yaml.safe_load((tmp_path / "sim.yaml").read_text())
    assert sim["simulation"]["clock"] == "paused"
    assert "notes" in sim and "planning" in sim["notes"]          # robosuite's observations
    workspace.write_machine_manifest(_real_cfg(tmp_path), tmp_path / "real.yaml")
    real = yaml.safe_load((tmp_path / "real.yaml").read_text())
    assert "simulation" not in real
    assert real["notes"] == {"gripper": ["stalls when it holds an object"]}


def test_notes_are_joined_by_kind_and_kept_to_the_kinds_present(tmp_path):
    from openrua.config.loader import join_notes
    assert join_notes({"gripper": ["a"]}, None, {"gripper": ["b"], "timing": ["t"]}) == \
        {"gripper": ["a", "b"], "timing": ["t"]}
    cfg = _cfg()
    cfg["machine"]["notes"]["base_twist"] = ["no base on this machine"]
    workspace.write_machine_manifest(cfg, tmp_path / "m.yaml")
    assert "base_twist" not in yaml.safe_load((tmp_path / "m.yaml").read_text())["notes"]


def test_workspace_template_says_nothing_only_a_simulator_does():
    import re
    words = re.compile(r"sim-time|sim-second|paused|\btick|agentview|panda|robot0|/clock",
                       re.IGNORECASE)
    for path in (PKG / "workspace").rglob("*"):
        if path.is_file() and "__pycache__" not in path.parts:
            hit = words.search(path.read_text())
            assert not hit, f"{path.relative_to(PKG)}: {hit.group(0)!r}"


def test_a_machine_of_cameras_alone_lists_no_joint_states(tmp_path):
    # Seen with a camera-only session: machine.yaml listed /joint_states,
    # which nobody published.
    from openrua.runner.preflight import build_checks

    class _Agent:
        def credentials_check(self): return None
        def sandbox_cli_check(self): return None
    cfg = {"machine": {"backend": {"kind": "real"}, "arms": [], "cameras": {"list": []}}}
    workspace.write_machine_manifest(cfg, tmp_path / "m.yaml")
    kinds = {s["kind"] for s in yaml.safe_load((tmp_path / "m.yaml").read_text())["sensors"]}
    assert "joint_state" not in kinds
    assert "joint_states_flow" not in dict(build_checks(cfg, _Agent()))
