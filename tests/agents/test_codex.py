"""The Codex hooks against the CLI's JSONL shapes (recorded from codex-cli 0.145.0)."""

import json

from openrua import agents
from openrua.testing import check_manifest

EVENTS = [
    {"type": "thread.started", "thread_id": "01a0789b-21d6-70d3-a99a-e32fb3ed541c"},
    {"type": "turn.started"},
    {"type": "item.completed", "item": {"id": "item_0", "type": "agent_message", "text": "I will run it."}},
    {"type": "item.started", "item": {"id": "item_1", "type": "command_execution",
                                      "command": "/bin/bash -lc 'echo hello'", "aggregated_output": "",
                                      "exit_code": None, "status": "in_progress"}},
    {"type": "item.completed", "item": {"id": "item_1", "type": "command_execution",
                                        "command": "/bin/bash -lc 'echo hello'",
                                        "aggregated_output": "hello\n", "exit_code": 0,
                                        "status": "completed"}},
    {"type": "item.completed", "item": {"id": "item_2", "type": "file_change",
                                        "changes": [{"path": "/workspace/note.txt", "kind": "add"}],
                                        "status": "completed"}},
    {"type": "item.completed", "item": {"id": "item_4", "type": "agent_message", "text": "DONE"}},
    {"type": "turn.completed", "usage": {"input_tokens": 43705, "cached_input_tokens": 39296,
                                         "cache_write_input_tokens": 0, "output_tokens": 347,
                                         "reasoning_output_tokens": 0}},
]


def _transcript(tmp_path, events=EVENTS):
    t = tmp_path / "transcript.jsonl"
    t.write_text("".join(json.dumps(e) + "\n" for e in events))
    return t


def test_bundled_codex_manifest_conforms():
    a = check_manifest(agents.manifest("codex").path)
    assert a.name == "codex" and a.version is None and "@openai/codex" in a.install
    assert "codex@0.153.4" in agents.get("codex@0.153.4").install


def test_launch_argv_is_headless_json_with_the_container_as_the_sandbox():
    a = agents.get("codex")
    argv = a.launch_argv("box", "do it", "gpt-5.6-sol", 100, "http://proxy:8888",
                         options={"effort": "medium"})
    assert argv[:2] == ["docker", "exec"] and "box" in argv
    assert "codex" in argv and "exec" in argv and "--json" in argv
    assert "--dangerously-bypass-approvals-and-sandbox" in argv
    assert "CODEX_HOME=/codex-home" in argv and "HTTPS_PROXY=http://proxy:8888" in argv
    assert 'model_reasoning_effort="medium"' in argv and 'web_search="disabled"' in argv
    assert argv[-1] == "do it" and "--ephemeral" not in argv
    resumed = a.launch_argv("box", "Continue.", "m", 10, "http://p", resume=True, session_id="x")
    i = resumed.index("codex")
    assert resumed[i + 1:i + 4] == ["exec", "resume", "--last"]
    with_token = a.launch_argv("box", "t", "m", 1, "http://p", token_file="/f")
    assert "--env-file" in with_token and "/f" in with_token


def test_read_final_counts_items_and_sums_usage(tmp_path):
    a = agents.get("codex")
    fin = a.read_final(_transcript(tmp_path))
    assert fin["num_turns"] == 4 and fin["hit_max_turns"] is False and fin["segments"] == 1
    assert fin["usage"]["input_tokens"] == 43705 and fin["usage"]["output_tokens"] == 347
    two = _transcript(tmp_path, EVENTS + EVENTS)
    assert a.read_final(two)["segments"] == 2 and a.read_final(two)["usage"]["output_tokens"] == 694
    assert a.read_final(tmp_path / "missing.jsonl") == {}


def test_replay_ops_are_shell_commands_only(tmp_path):
    a = agents.get("codex")
    ops = a.replay_ops(_transcript(tmp_path))
    assert [o["kind"] for o in ops] == ["shell"]
    assert ops[0]["command"] == "/bin/bash -lc 'echo hello'" and ops[0]["output"] == "hello\n"


def test_scan_transcript_sees_a_clean_ending_and_a_quota_error(tmp_path):
    a = agents.get("codex")
    ev = a.scan_transcript(_transcript(tmp_path))
    assert ev["has_final_result"] and not ev["final_is_error"] and ev["quota"] is None
    bad = EVENTS[:2] + [{"type": "turn.failed", "error": {"message": "You have hit your usage limit."}}]
    ev = a.scan_transcript(_transcript(tmp_path, bad))
    assert ev["final_is_error"] and ev["quota"] is not None
    assert a.quota_since(_transcript(tmp_path, bad))["resets_at"] is None


def _rollout_lines():
    """Three model responses, each with the account's rate limits."""
    stamps = ["2026-09-10T19:21:12.000Z", "2026-09-10T19:22:12.000Z", "2026-09-10T19:23:12.000Z"]
    used = [2.0, 2.5, 3.0]
    lines = [json.dumps({"timestamp": stamps[0], "type": "session_meta", "payload": {"id": "s1"}})]
    for stamp, u in zip(stamps, used):
        lines.append(json.dumps({"timestamp": stamp, "type": "response_item",
                                 "payload": {"type": "custom_tool_call", "name": "exec", "input": "ls"}}))
        lines.append(json.dumps({"timestamp": stamp, "type": "token_usage_record",
                                 "payload": {"usage": {"input_tokens": 1000, "cached_input_tokens": 800,
                                                       "output_tokens": 10, "reasoning_output_tokens": 2}}}))
        lines.append(json.dumps({"timestamp": stamp, "type": "event_msg", "payload": {
            "type": "token_count",
            "rate_limits": {"primary": {"used_percent": u, "window_minutes": 10080, "resets_at": 1789668541},
                            "secondary": None, "plan_type": "pro", "rate_limit_reached_type": None}}}))
    return lines


def test_collect_keeps_the_session_log_beside_the_transcript(tmp_path):
    profile = tmp_path / "profile"
    (profile / "sessions" / "2026" / "09" / "10").mkdir(parents=True)
    (profile / "sessions" / "2026" / "09" / "10" / "rollout-2026-09-10T19-21-10-x.jsonl").write_text(
        "\n".join(_rollout_lines()))          # no trailing newline
    trial = tmp_path / "trial"; trial.mkdir()
    kept = agents.get("codex").collect(profile, trial)
    assert kept == [trial / "rollout.jsonl"]
    assert kept[0].read_text().endswith("\n")
    assert agents.get("codex").collect(tmp_path / "empty", trial / "other") == []


def test_rollout_gives_model_responses_as_turns_readings_and_dating(tmp_path):
    trial = tmp_path; (trial / "rollout.jsonl").write_text("\n".join(_rollout_lines()) + "\n")
    transcript = trial / "transcript.jsonl"
    transcript.write_text(json.dumps({"type": "item.completed", "item": {"type": "command_execution"}}) + "\n"
                          + json.dumps({"type": "turn.completed", "usage": {"input_tokens": 99}}) + "\n")
    agent = agents.get("codex")
    final = agent.read_final(transcript)
    assert final["num_turns"] == 3 and final["segments"] == 1
    assert final["usage"] == {"input_tokens": 3000, "cached_input_tokens": 2400,
                              "output_tokens": 30, "reasoning_output_tokens": 6}
    readings = agent.read_rate_limits(transcript)
    assert [r["window"] for r in readings] == ["seven_day"] * 3
    assert [r["utilization"] for r in readings] == [0.02, 0.025, 0.03]
    assert readings[0]["status"] == "ok" and readings[0]["resets_at"] == 1789668541
    assert readings[1]["at"] - readings[0]["at"] == 60.0
    assert agent.assistant_turns_before(transcript, readings[1]["at"]) == 2


def test_without_a_rollout_the_stream_still_answers(tmp_path):
    transcript = tmp_path / "transcript.jsonl"
    transcript.write_text(json.dumps({"type": "item.completed", "item": {"type": "command_execution"}}) + "\n"
                          + json.dumps({"type": "turn.completed", "usage": {"input_tokens": 99}}) + "\n")
    agent = agents.get("codex")
    assert agent.read_final(transcript)["num_turns"] == 1
    assert agent.read_rate_limits(transcript) == []
    assert agent.assistant_turns_before(transcript, 0) is None
