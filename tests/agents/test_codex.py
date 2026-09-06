"""The Codex hooks against the CLI's JSONL shapes (recorded from codex-cli 0.145.0)."""

import json

from robocli import agents
from robocli.testing import check_manifest

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
