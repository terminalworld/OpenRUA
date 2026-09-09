"""The demo pipeline on synthetic artifacts: the bridge's Recording
writes frames and an index, the script operator splits a command file
into operations, ops.jsonl pairs with the index by wall time, and the
renderer produces a video from them."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openrua.demo import compose as demo
from openrua.robot.sim.bridge.rpc import Recording
from openrua.runner import record
from openrua.runner.operators import blocks


class _Engine:
    def render(self, name, width, height, depth=False):
        return np.zeros((height, width, 3), dtype=np.uint8)


def test_recording_writes_frames_and_index(tmp_path):
    rec = Recording(str(tmp_path / "frames"), [("cam_a", 32, 24), ("cam_b", 16, 12)])
    rec.frame(0, _Engine())
    rec.frame(1, _Engine())
    files = sorted(p.name for p in (tmp_path / "frames").glob("*.jpg"))
    assert files == ["000000_cam_a.jpg", "000000_cam_b.jpg",
                     "000001_cam_a.jpg", "000001_cam_b.jpg"]
    index = [json.loads(l) for l in (tmp_path / "frames" / "index.jsonl").read_text().splitlines()]
    assert [i["step"] for i in index] == [0, 1]
    assert index[0]["files"] == ["000000_cam_a.jpg", "000000_cam_b.jpg"]
    assert index[0]["t"] <= index[1]["t"]


def test_recording_every_n_keeps_one_step_in_n(tmp_path):
    rec = Recording(str(tmp_path / "frames"), [("cam_a", 8, 6)], every=3)
    for step in range(1, 8):
        rec.frame(step, _Engine())
    index = [json.loads(l) for l in (tmp_path / "frames" / "index.jsonl").read_text().splitlines()]
    assert [i["step"] for i in index] == [3, 6]
    assert sorted(p.name for p in (tmp_path / "frames").glob("*.jpg")) == \
        ["000003_cam_a.jpg", "000006_cam_a.jpg"]


def test_recording_refuses_a_zero_stride(tmp_path):
    with pytest.raises(ValueError):
        Recording(str(tmp_path / "frames"), [("cam_a", 8, 6)], every=0)


def test_frame_stride_counts_sim_steps_not_index_entries():
    dense = [{"step": s, "t": float(s), "files": []} for s in range(1, 10)]
    sparse = [{"step": s, "t": float(s), "files": []} for s in range(4, 40, 4)]
    assert demo.frame_stride(dense, 4) == 4
    assert demo.frame_stride(sparse, 4) == 1      # one entry already spans 4 steps
    assert demo.frame_stride(sparse, 8) == 2
    assert demo.frame_stride(sparse, 1) == 1      # cannot play slower than recorded
    assert demo.frame_stride([], 4) == 4


def test_blocks_split_on_markers_and_keep_heredocs():
    script = ("#!/usr/bin/env bash\n# header\n\n# openrua op 0\nls\n\n"
              "# openrua op 1\ncat > f <<'EOF'\n# openrua op 99 is content, no marker\nEOF\n")
    ops = blocks(script)
    assert len(ops) == 2
    assert ops[0].strip() == "ls"
    assert "cat > f" in ops[1]
    # only a whole marker line splits; a here-doc line that merely
    # begins like one is content
    assert "no marker" in ops[1]
    assert blocks("echo one\necho two\n") == ["echo one\necho two"]


def test_extraction_marks_every_operation(tmp_path):
    ops = [{"kind": "shell", "command": "ls", "output": "a\nb", "t0": 1.0, "t1": 2.0},
           {"kind": "write", "path": "/tmp/x.py", "content": "print(1)\n", "t0": 3.0, "t1": 3.5},
           {"kind": "read", "path": "/tmp/x.py"}]
    path = record.write_ops(tmp_path, ops)
    rows = [json.loads(l) for l in path.read_text().splitlines()]
    assert [r["i"] for r in rows] == [0, 1]  # the read has no world effect
    assert rows[1]["command"].startswith("mkdir -p")
    assert rows[0]["t0"] == 1.0 and rows[0]["t1"] == 2.0


def test_timeline_orders_command_motion_output():
    index = [{"step": 0, "t": 0.0, "files": []}, {"step": 1, "t": 10.5, "files": []},
             {"step": 2, "t": 11.0, "files": []}, {"step": 3, "t": 12.0, "files": []}]
    ops = [{"i": 0, "command": "ls", "t0": 1.0, "t1": 2.0},
           {"i": 1, "command": "python3 move.py", "t0": 10.0, "t1": 12.0}]
    kinds = [(k, p.get("step", p.get("i"))) for _, k, p in demo.timeline(index, ops)]
    assert kinds == [("frame", 0), ("command", 0), ("output", 0), ("command", 1),
                     ("frame", 1), ("frame", 2), ("frame", 3), ("output", 1)]


def _trial(tmp_path: Path) -> Path:
    trial = tmp_path / "runs" / "libero_pro" / "r" / "trials" / "s-0" / "seed0"
    rec = Recording(str(trial / "frames"), [("agentview", 64, 48), ("wrist", 32, 24)])
    ts = []
    import time
    for step in range(4):
        rec.frame(step, _Engine())
        ts.append(time.time())
        time.sleep(0.01)
    t_mid = (ts[0] + ts[1]) / 2
    record.write_ops(trial, [
        {"kind": "shell", "command": "ls", "output": "README.md", "t0": ts[0] - 1, "t1": t_mid},
        {"kind": "shell", "command": "python3 move.py", "output": "", "t0": t_mid, "t1": ts[-1] + 1},
    ])
    (trial / "result.json").write_text(json.dumps(
        {"task_language": "pick the bowl", "success": True, "task_suite": "s",
         "task_id": 0, "init_state_id": 0}))
    return trial


def test_render_writes_a_video(tmp_path):
    pytest.importorskip("imageio_ffmpeg")
    trial = _trial(tmp_path)
    out = demo.render(trial, style=demo.Style(width=320, height=180, fps=5, font_size=8), gif=True)
    assert out == trial / "demo.mp4" and out.stat().st_size > 0
    assert (trial / "demo.gif").stat().st_size > 0


def test_render_refuses_an_unrecorded_trial_with_the_replay_command(tmp_path):
    trial = tmp_path / "runs" / "b" / "r" / "trials" / "s-3" / "seed1"
    trial.mkdir(parents=True)
    (trial / "result.json").write_text(json.dumps(
        {"task_suite": "s", "task_id": 3, "init_state_id": 1}))
    (trial / "provenance.json").write_text(json.dumps({"config_file": "libero_pro"}))
    from openrua.errors import NotFound
    with pytest.raises(NotFound) as e:
        demo.load(trial)
    assert "--operator script" in e.value.hint and "--record" in e.value.hint
    assert "--task-ids 3 --seeds 1" in e.value.hint


def test_a_real_robot_refuses_to_record():
    from openrua import robot
    from openrua.errors import ConfigError
    with pytest.raises(ConfigError, match="simulated"):
        robot.up({"kind": "real"}, name="x", config_path="c", task_suite="s",
                 task_id=0, log_path=Path("/dev/null"), record="/tmp/frames")


def test_unknown_camera_is_named():
    with pytest.raises(Exception, match="not recorded"):
        demo.pick_cameras(("nope",), ["agentview"])
    assert demo.pick_cameras((), ["a", "b", "c"]) == ("a", "b")
    assert demo.pick_cameras(("b",), ["a", "b"]) == ("b", None)


def test_shell_keeps_state_and_gives_each_op_an_empty_stdin(monkeypatch):
    """One persistent shell: cd carries over; a command that reads stdin
    gets nothing and cannot swallow the operations queued behind it."""
    import subprocess
    from openrua.runner import operators

    real_popen = subprocess.Popen
    monkeypatch.setattr(operators.subprocess, "Popen",
                        lambda argv, **kw: real_popen(["bash"], **kw))
    sh = operators.Shell("ignored")
    try:
        assert sh.run("cd /tmp && X=42", 5)[1] is True
        out, ok = sh.run("pwd; echo $X", 5)
        assert ok and out.split() == ["/tmp", "42"]
        out, ok = sh.run("cat", 5)  # reads stdin: must not hang or eat the next op
        assert ok and out == ""
        out, ok = sh.run("echo after", 5)
        assert ok and out.strip() == "after"
        out, ok = sh.run("cat <<'EOF'\nheredoc still works\nEOF", 5)
        assert ok and out.strip() == "heredoc still works"
    finally:
        sh.close()
