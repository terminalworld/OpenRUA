"""What openrua exec shows while the agent works, and the person's verdict
on a robot nothing scores."""

import io
import json
import threading

from openrua import agents
from openrua.cli.commands import execute
from openrua.cli.progress import Progress
from openrua.runner import session
from openrua.runner import trial
from tests.runner.test_trial_real import _harness


def test_each_command_shows_once_done_with_its_outcome_and_time():
    out, now = io.StringIO(), [0.0]
    view = Progress(out, live=False, clock=lambda: now[0])
    view({"kind": "begin", "agent": "claude-code"})
    view({"kind": "text", "text": "Looking around first.\nThen the cube."})
    view({"kind": "start", "id": "1", "label": "ros2 topic list", "shell": True})
    view({"kind": "start", "id": "1", "label": "ros2 topic list", "shell": True})   # again, fuller
    now[0] = 1.25
    view({"kind": "end", "id": "1", "ok": True})
    view({"kind": "start", "id": "2", "label": "Read /workspace/README.md", "shell": False})
    now[0] = 75
    view({"kind": "end", "id": "2", "ok": False})
    lines = out.getvalue().splitlines()
    assert lines[0].startswith("[exec] claude-code is at work; Ctrl-C stops it")
    assert lines[1] == "agent  Looking around first." and lines[2] == "       Then the cube."
    assert lines[3].startswith("$ ros2 topic list") and lines[3].endswith("✓ 1.2s")
    assert lines[4].startswith("· Read /workspace/README.md") and lines[4].endswith("✗ 1m13s")
    view({"kind": "start", "id": "3", "label": "sleep 600", "shell": True})
    now[0] = 80
    view({"kind": "done"})          # the budget ended the agent mid-command
    lines = out.getvalue().splitlines()
    assert lines[5].startswith("$ sleep 600") and lines[5].endswith("unfinished 5.0s")
    assert len(lines) == 6 and view.running == {}


def test_the_follower_hands_over_each_record_as_it_is_written(tmp_path):
    transcript = tmp_path / "transcript.jsonl"
    transcript.write_text("")
    seen, stop = [], threading.Event()
    agent = agents.get("codex")
    viewer = threading.Thread(target=session.follow, args=(transcript, agent, seen.append, stop))
    viewer.start()
    with transcript.open("a") as f:
        f.write(json.dumps({"type": "item.started", "item": {
            "id": "i1", "type": "command_execution", "command": "/bin/bash -lc 'ros2 node list'"}}) + "\n")
        f.write('{"type": "item.completed", "item": {"id": "i2", "type": "agent_mes')   # half a line
    stop.set()
    viewer.join(5)
    assert seen == [{"kind": "start", "id": "i1", "label": "ros2 node list", "shell": True}]


def test_each_agent_names_its_steps():
    claude, codex = agents.get("claude-code"), agents.get("codex")
    assert claude.steps({"type": "assistant", "message": {"content": [
        {"type": "text", "text": "On it."},
        {"type": "tool_use", "id": "t1", "name": "Bash", "input": {"command": "ros2 topic list"}},
        {"type": "tool_use", "id": "t2", "name": "Read", "input": {"file_path": "/workspace/x.md"}}]}}) == [
        {"kind": "text", "text": "On it."},
        {"kind": "start", "id": "t1", "label": "ros2 topic list", "shell": True},
        {"kind": "start", "id": "t2", "label": "Read /workspace/x.md", "shell": False}]
    assert claude.steps({"type": "user", "message": {"content": [
        {"type": "tool_result", "tool_use_id": "t1", "is_error": True}]}}) == [
        {"kind": "end", "id": "t1", "ok": False}]
    assert codex.steps({"type": "item.completed", "item": {
        "id": "i1", "type": "command_execution", "command": "/bin/bash -lc 'ls -l'",
        "exit_code": 2}})[-1] == {"kind": "end", "id": "i1", "ok": False}
    assert codex.steps({"type": "item.completed", "item": {
        "id": "i2", "type": "agent_message", "text": "Done."}}) == [{"kind": "text", "text": "Done."}]
    assert codex.steps({"type": "turn.completed"}) == []


def test_the_person_beside_a_real_robot_says_whether_it_was_done(monkeypatch, tmp_path, proxy_address):
    handle, cfg, token = _harness(monkeypatch, tmp_path)
    asked = []

    def operator(ctx):
        (ctx["trial_dir"] / "transcript.jsonl").write_text(json.dumps({
            "type": "assistant", "message": {"content": [{"type": "text", "text": "The cup is on the plate."}]}}) + "\n")
        return {"operator": "agent", "termination": "self_finished"}

    rec = trial.run_trial(cfg, tmp_path / "bench.yaml", tmp_path / "runs" / "r1", "suite", 0, 0,
                          operator, 1.0, token_file=str(token), task="put the cup on the plate",
                          home=tmp_path / "home", judge=lambda words: asked.append(words) or True)
    assert asked == ["The cup is on the plate."]
    assert rec["success"] is True and rec["verdict"] == "person"
    assert rec["agent_final_message"] == "The cup is on the plate."
    assert handle.calls[-1] == "shutdown"   # asked while the robot was still up
    rec = trial.run_trial(cfg, tmp_path / "bench.yaml", tmp_path / "runs" / "r2", "suite", 0, 0,
                          operator, 1.0, token_file=str(token), task="put the cup on the plate",
                          home=tmp_path / "home", judge=lambda words: None)
    assert rec["success"] is None and rec["verdict"] == "not_applicable"


def test_the_question_takes_yes_no_or_nothing(monkeypatch):
    for typed, meant in (("y", True), ("Yes", True), ("n", False), ("", None), ("maybe", None)):
        monkeypatch.setattr("builtins.input", lambda prompt, typed=typed: typed)
        assert execute.ask_person("done") is meant

    def interrupted(prompt):
        raise KeyboardInterrupt
    monkeypatch.setattr("builtins.input", interrupted)
    assert execute.ask_person("done") is None
