"""Agents package unit tests: the prompt contract, the manifest and hooks,
the launcher, the transcript hooks."""

from __future__ import annotations

from openrua.config import load_config

from pathlib import Path

import yaml

from openrua import agents

REPO = Path(__file__).resolve().parents[2]
LIBERO_CFG = REPO / "openrua" / "configs" / "benchmarks" / "libero_pro.yaml"


# ---------------------------------------------------------------- prompt

def test_prompt_has_only_task_placeholder():
    # The launcher formats with {task} alone: any other placeholder would
    # KeyError at launch; a literal brace would corrupt the prompt.
    assert "{task}" in agents.PROMPT
    assert agents.PROMPT.format(task="PROBE").count("PROBE") == 1


def test_configs_carry_no_stale_prompt_key():
    # The prompt lives in agents.PROMPT; a config naming a prompt file
    # would be silently ignored, so ban it.
    for cfg_file in (REPO / "openrua" / "configs" / "benchmarks").glob("*.yaml"):
        cfg = yaml.safe_load(cfg_file.read_text())
        assert "prompt" not in cfg.get("agent", {}), cfg_file.name


# -------------------------------------------------------------- adapters

def test_pins_come_from_the_caller_not_the_manifest():
    latest = agents.get("claude-code")
    assert latest.version is None and "{version}" not in latest.install
    assert "claude-code@" not in latest.install          # the current release
    assert latest.sandbox_cli_check() is None            # nothing to check against
    pinned = agents.get("claude-code@2.1.226")
    assert pinned.version == "2.1.226" and "claude-code@2.1.226" in pinned.install
    name, cmd = pinned.sandbox_cli_check()
    assert name == "sandbox_cli_matches_pin" and "2.1.226" in cmd
    assert agents.get("claude-code", version="2.1.100").version == "2.1.100"
    assert agents.split_pin("codex@0.153.4") == ("codex", "0.153.4")


def test_preflight_includes_version_checks():
    from openrua.runner.preflight import build_checks
    cfg = load_config(LIBERO_CFG)
    names = [n for n, _ in build_checks(cfg, agents.get("claude-code@2.1.226"))]
    assert "sandbox_cli_matches_pin" in names
    assert "credentials_readable" in names
    # without a pin there is nothing to check the sandbox CLI against
    assert "sandbox_cli_matches_pin" not in [n for n, _ in build_checks(cfg, agents.get("claude-code"))]


# -------------------------------------------------- credentials pipeline
# The launcher's precondition is "credentials already live in the
# container"; that pipeline is prepare_profile -> sandbox_mounts ->
# sandbox.up --mount (generic slot) -> preflight credentials_readable.
# Each link gets a dry guard here; the live end is the preflight.

def test_prepare_profile_requires_credentials(tmp_path):
    import pytest
    a = agents.get("claude-code")
    with pytest.raises(RuntimeError, match="credentials missing"):
        agents.prepare_profile(tmp_path, a)


def test_prepare_profile_copies_profile_but_never_the_credentials(tmp_path):
    # Rotation safety: the credentials file must be SHARED (returned as a
    # path for bind-mounting), never copied into the per-entry dir.
    import shutil
    a = agents.get("claude-code")
    (tmp_path / a.credentials.filename).write_text("{}")
    (tmp_path / "settings.json").write_text("{}")
    cfg_dir, shared = agents.prepare_profile(tmp_path, a)
    try:
        assert shared == tmp_path / a.credentials.filename
        assert (cfg_dir / "settings.json").exists()
        assert not (cfg_dir / a.credentials.filename).exists()
    finally:
        shutil.rmtree(cfg_dir, ignore_errors=True)


def test_sandbox_mounts_are_bare_specs_landing_in_the_config_dir(tmp_path):
    # The specs feed sandbox.up's generic --mount slot verbatim, so shape
    # is the contract: SRC:DST, credentials bound INSIDE the profile dir.
    a = agents.get("claude-code")
    specs = a.sandbox_mounts(tmp_path / "cfg", tmp_path / "creds.json")
    dsts = []
    for spec in specs:
        src, dst = spec.rsplit(":", 1)
        assert Path(src).is_absolute() and dst.startswith("/")
        dsts.append(dst)
    profile_dst, creds_dst = dsts
    assert creds_dst == f"{profile_dst}/{a.credentials.filename}"


def test_launch_argv_threads_proxy_and_profile_env():
    # agents -> proxy seam: the proxy URL must reach the CLI process env,
    # and the profile env must point at the mounted profile dir.
    a = agents.get("claude-code")
    argv = a.launch_argv(sandbox="box", prompt="p", model="m",
                         max_turns=3, proxy="http://proxy:9999")
    assert "HTTPS_PROXY=http://proxy:9999" in argv
    assert "HTTP_PROXY=http://proxy:9999" in argv
    env = [x for x in argv if x.startswith(a.credentials.config_env + "=")]
    assert len(env) == 1
    _, cfg_dst = a.sandbox_mounts(Path("/x"), Path("/y"))[0].rsplit(":", 1)
    assert env[0] == f"{a.credentials.config_env}={cfg_dst}"


def test_launch_argv_pins_effort_explicitly():
    # Reasoning effort must be an EXPLICIT flag, never the CLI's implicit
    # default (a CLI update could silently shift it mid-campaign). Default =
    # the adapter's default_options; agent.options in a config overrides.
    a = agents.get("claude-code")
    assert a.default_options["effort"] == "high"
    argv = a.launch_argv(sandbox="box", prompt="p", model="m",
                         max_turns=3, proxy="http://w:9")
    assert argv[argv.index("--effort") + 1] == "high"
    argv2 = a.launch_argv(sandbox="box", prompt="p", model="m",
                          max_turns=3, proxy="http://w:9", options={"effort": "max"})
    assert argv2[argv2.index("--effort") + 1] == "max"


def test_launcher_takes_the_proxy_url_from_the_runner():
    # No fallback URL in the launcher: the runner hands the proxy URL in,
    # and the proxy package is the one place its name and port live.
    import argparse
    from openrua.agents import launcher
    from openrua import proxy
    assert "DEFAULT_PROXY" not in vars(launcher)
    assert (proxy.NAME, proxy.PORT) == ("openrua-proxy", 8888)
    ap = argparse.ArgumentParser()
    ap.add_argument("--proxy", required=True)
    with __import__("pytest").raises(SystemExit):
        ap.parse_args([])


# ------------------------------------------------------------ four doors
# The package's outward surface: verbs (launch/preinstall/whitelist),
# three package-level names (PROMPT, prepare_profile, get), the adapter
# plug shape, and a fence against bypassing get().

def test_every_bundled_agent_conforms():
    from openrua.testing import check_agent
    listed = agents.available()
    assert [a.name for a in listed] == ["claude-code", "codex"]
    for a in listed:
        assert a.agent is not None, a.error
        check_agent(a.agent)
        assert agents.get(a.name) is a.agent or agents.get(a.name).name == a.name


def test_capabilities_are_the_overridden_hooks():
    a = agents.get("claude-code")
    assert {"interactive_argv", "read_final", "quota_since", "replay_ops"} <= a.capabilities
    # a bare adapter has none, and every hook keeps its documented default
    class Bare(agents.Agent):
        name, default_model = "bare", "m"

        def launch_argv(self, sandbox, prompt, model, max_turns, proxy, **_):
            return ["docker", "exec", sandbox, "bare", prompt]
    b = Bare()
    assert b.capabilities == frozenset()
    assert b.interactive_argv("s", "m", "p") is None and b.read_final(Path("/x")) == {}
    assert b.sandbox_mounts(Path("/c"), Path("/f")) == () and b.credentials_check() is None
    from openrua.testing import check_agent
    check_agent(b)


def test_agent_requires_name_and_model_and_refuses_unknown_attributes():
    import pytest
    with pytest.raises(TypeError, match="name and default_model"):
        agents.Agent()
    with pytest.raises(TypeError, match="no attribute"):
        agents.Agent(name="x", default_model="m", colour="red")


def test_user_directory_agent_is_found_and_a_broken_one_is_isolated(tmp_path):
    import pytest
    from openrua.errors import NotFound
    manifests = tmp_path / "agents"
    hooks = tmp_path / "plugins" / "agents"
    manifests.mkdir()
    hooks.mkdir(parents=True)
    (manifests / "my-agent.yaml").write_text(
        "name: my-agent\ndefault_model: m1\nhooks: my_hooks\n")
    (hooks / "my_hooks.py").write_text(
        "from openrua.agents import Agent\n"
        "class My(Agent):\n"
        "    def launch_argv(self, sandbox, prompt, model, max_turns, proxy, **_):\n"
        "        return ['docker', 'exec', sandbox, 'my', prompt]\n"
        "HOOKS = My\n")
    (manifests / "broken.yaml").write_text("name: broken\ndefault_model: m\nhooks: broken\n")
    (hooks / "broken.py").write_text("raise RuntimeError('boom')\n")
    (manifests / "nohooks.yaml").write_text("name: nohooks\ndefault_model: m\nhooks: nohooks\n")
    (hooks / "nohooks.py").write_text("x = 1\n")
    got = agents.get("my-agent", tmp_path)
    assert got.name == "my-agent" and got.default_model == "m1"
    listed = {a.name: a for a in agents.available(tmp_path)}
    assert listed["claude-code"].source == "bundled"
    assert listed["my-agent"].source == "user" and listed["my-agent"].agent is not None
    assert listed["broken"].agent is None and "boom" in listed["broken"].error
    assert listed["nohooks"].agent is None and "HOOKS" in listed["nohooks"].error
    with pytest.raises(NotFound, match="my-agent"):
        agents.get("nope", tmp_path)
    # a manifest without hooks composes to the bare contract; check_manifest says so
    from openrua.testing import check_manifest
    (manifests / "facts-only.yaml").write_text("name: facts-only\ndefault_model: m\n")
    with pytest.raises(AssertionError, match="hooks"):
        check_manifest(manifests / "facts-only.yaml", tmp_path)
    check_manifest(manifests / "my-agent.yaml", tmp_path)


def test_no_direct_hooks_imports_outside_the_registry():
    # Consumers go through agents.get(); naming a hooks module elsewhere
    # breaks the agent-agnostic promise.
    src = Path(agents.__file__).resolve().parents[1]
    registry = src / "agents" / "registry.py"
    offenders = []
    for f in src.rglob("*.py"):
        if f == registry or "__pycache__" in f.parts or (src / "plugins") in f.parents:
            continue
        if "plugins.agents" in f.read_text():
            offenders.append(str(f))
    assert not offenders, offenders


def test_build_takes_install_and_whitelist_from_the_manifests(tmp_path):
    from openrua.cli.commands.build import (default_agent, preinstall_for,
                                            whitelist_for)
    a = agents.get("claude-code")
    assert default_agent(tmp_path) == "claude-code"      # the package default
    assert preinstall_for(None, tmp_path) == a.install
    assert whitelist_for(["claude-code"], tmp_path).splitlines() == list(a.whitelist)
    # several --agent: the union, each line once
    assert whitelist_for(["claude-code", "claude-code"], tmp_path).splitlines() == \
        list(a.whitelist)
    # the user's defaults file picks the default agent
    (tmp_path / "config.yaml").write_text("agent: {name: claude-code}\n")
    assert default_agent(tmp_path) == "claude-code"


def test_agents_module_exposes_launch_only():
    import subprocess
    import sys
    r = subprocess.run([sys.executable, "-m", "openrua.agents", "whitelist"],
                       capture_output=True, text=True)
    assert r.returncode == 2 and "launch" in r.stderr


# ------------------------------------------------------------- no aids

def test_no_agent_aids_ship_with_the_package():
    # Everything the agent is given beyond the native surface is the
    # workspace template; no other aid directory exists.
    src = Path(agents.__file__).resolve().parents[1]
    assert not list(src.glob("*/addons"))


def test_read_final_extracts_turns_usage_and_cost(tmp_path):
    import json
    cc = agents.get("claude-code")
    t = tmp_path / "transcript.jsonl"
    t.write_text(json.dumps({"type": "assistant"}) + "\n" + json.dumps({
        "type": "result", "num_turns": 7, "subtype": "success",
        "usage": {"input_tokens": 1200, "output_tokens": 300},
        "total_cost_usd": 0.42, "duration_ms": 61000}) + "\n")
    fin = cc.read_final(t)
    assert fin == {"num_turns": 7, "hit_max_turns": False,
                   "usage": {"input_tokens": 1200, "output_tokens": 300},
                   "cost_usd": 0.42, "duration_ms": 61000,
                   # an uninterrupted trial is one segment; its numbers are
                   # unchanged by the cross-segment summing
                   "segments": 1}
    bad = tmp_path / "empty.jsonl"
    bad.write_text("")
    assert cc.read_final(bad) == {}


def test_read_final_survives_trailing_system_records(tmp_path):
    # A session that spawned background
    # tasks gets system task-notifications appended AFTER the result
    # record; the token/cost account must still be found.
    import json

    read_final = agents.get("claude-code").read_final
    t = tmp_path / "transcript.jsonl"
    lines = [
        {"type": "assistant", "message": {}},
        {"type": "result", "num_turns": 42, "total_cost_usd": 3.21,
         "usage": {"output_tokens": 999}, "duration_ms": 1000},
        {"type": "system", "subtype": "task_notification"},
        {"type": "system", "subtype": "task_notification"},
    ]
    t.write_text("\n".join(json.dumps(d) for d in lines))
    f = read_final(t)
    assert f["cost_usd"] == 3.21 and f["num_turns"] == 42
    assert f["usage"]["output_tokens"] == 999


def test_read_rate_limits_normalizes_and_dates_each_reading(tmp_path):
    # The quota reading rides in the transcript the CLI already writes
    # no extra request, no credentials, no polling.
    import json

    read_rate_limits = agents.get("claude-code").read_rate_limits
    t = tmp_path / "transcript.jsonl"
    t.write_text("\n".join(json.dumps(d) for d in [
        # an event before any clock appears: dated by the first one that does
        {"type": "rate_limit_event",
         "rate_limit_info": {"status": "allowed_warning", "utilization": 0.73,
                             "rateLimitType": "seven_day",
                             "resetsAt": 1787684400}},
        {"type": "assistant", "timestamp": "2026-08-20T07:44:43Z"},
        {"type": "rate_limit_event",
         "rate_limit_info": {"status": "allowed",
                             "rateLimitType": "five_hour",
                             "resetsAt": 1787670000}},
    ]))
    got = read_rate_limits(t)
    assert [r["window"] for r in got] == ["seven_day", "five_hour"]
    assert got[0]["utilization"] == 0.73 and got[0]["status"] == "allowed_warning"
    assert got[0]["resets_at"] == 1787684400
    # both dated by the only clock in the file, including the one before it
    assert got[0]["at"] == got[1]["at"] > 0
    # a plain "allowed" event carries no number; reported as None, never 0
    assert got[1]["utilization"] is None


def test_read_rate_limits_is_empty_not_fake_when_nothing_reported(tmp_path):
    read_rate_limits = agents.get("claude-code").read_rate_limits
    t = tmp_path / "transcript.jsonl"
    t.write_text('{"type": "assistant", "timestamp": "2026-08-20T07:44:43Z"}')
    assert read_rate_limits(t) == []


def _result(**kw):
    import json
    base = {"type": "result", "subtype": "success", "num_turns": 1,
            "usage": {"input_tokens": 0, "output_tokens": 0},
            "total_cost_usd": 0.0, "duration_ms": 0}
    return json.dumps({**base, **kw})


def test_read_final_sums_across_resume_segments(tmp_path):
    # A trial suspended at a quota wall and resumed writes one result record
    # per segment into the same transcript; the totals must span them, or
    # every resumed trial silently under-reports its turns and its spend.
    read_final = agents.get("claude-code").read_final
    t = tmp_path / "transcript.jsonl"
    t.write_text("\n".join([
        _result(num_turns=4, subtype="error_max_turns", duration_ms=1000,
                total_cost_usd=0.25,
                usage={"input_tokens": 10, "output_tokens": 100,
                       "service_tier": "standard"}),
        _result(num_turns=6, subtype="success", duration_ms=2000,
                total_cost_usd=0.75,
                usage={"input_tokens": 5, "output_tokens": 200,
                       "service_tier": "standard"}),
    ]))
    fin = read_final(t)
    assert fin["segments"] == 2
    assert fin["num_turns"] == 10
    assert fin["cost_usd"] == 1.0
    assert fin["duration_ms"] == 3000
    assert fin["usage"]["output_tokens"] == 300
    # non-additive fields are carried, not summed
    assert fin["usage"]["service_tier"] == "standard"
    # whether the trial ended on its turn budget is the LAST segment's
    # verdict, never a sum: this one finished on the resumed segment
    assert fin["hit_max_turns"] is False


def test_read_final_missing_field_stays_missing(tmp_path):
    # A field no segment reported must read as absent, not as a fake 0.
    read_final = agents.get("claude-code").read_final
    t = tmp_path / "transcript.jsonl"
    t.write_text(_result(num_turns=None))
    assert read_final(t)["num_turns"] is None


def test_launch_argv_names_the_session_then_resumes_it():
    # --session-id creates the named session; --resume continues it. They are
    # not interchangeable, and sending both phases the same flag would either
    # start a fresh session (losing the work) or fail to find one.
    launch_argv = agents.get("claude-code").launch_argv
    kw = dict(sandbox="rc-x-sandbox", prompt="P", model="m", max_turns=5,
              proxy="http://p")
    start = launch_argv(**kw, session_id="SID")
    resume = launch_argv(**kw, session_id="SID", resume=True)
    plain = launch_argv(**kw)
    assert start[start.index("claude") + 1:start.index("claude") + 3] == \
        ["--session-id", "SID"]
    assert resume[resume.index("claude") + 1:resume.index("claude") + 3] == \
        ["--resume", "SID"]
    assert "--session-id" not in plain and "--resume" not in plain


def test_launch_argv_pins_autocompact():
    # Left to the CLI default the threshold could drift on an upgrade, and a
    # resumed trial could compact where an uninterrupted one would not have.
    a = agents.get("claude-code")
    argv = a.launch_argv(sandbox="s", prompt="P", model="m", max_turns=1,
                         proxy="http://p")
    assert argv[argv.index("--autocompact") + 1] == a.default_options["autocompact"]


def test_resume_prompt_cannot_carry_the_task():
    # The resumed agent must be told nothing the session does not already
    # hold; a {task} placeholder here would hand it context an uninterrupted
    # trial never received.
    from openrua import agents
    assert "{task}" not in agents.RESUME_PROMPT
    assert "{task}" in agents.PROMPT


def test_read_final_survives_a_damaged_usage_field(tmp_path):
    # read_final promises {} on an unreadable transcript, so a damaged
    # numeric field must not raise on the way to producing totals.
    read_final = agents.get("claude-code").read_final
    t = tmp_path / "transcript.jsonl"
    t.write_text(_result(num_turns="lots",
                         usage={"output_tokens": "many", "input_tokens": 5}))
    fin = read_final(t)
    assert fin["num_turns"] is None          # unusable, not a fake 0
    assert fin["usage"]["input_tokens"] == 5


def test_launch_argv_raises_the_bash_timeouts_above_the_cli_defaults():
    # A robot motion outlives the CLI's 120s Bash default, and a command
    # pushed to the background mid-motion costs the agent a poll loop.
    # Both timeouts must reach the CLI process env, and the default must
    # stay under the ceiling so `timeout=` can still ask for more.
    a = agents.get("claude-code")
    argv = a.launch_argv(sandbox="box", prompt="p", model="m",
                         max_turns=3, proxy="http://w:9")
    o = a.default_options
    assert f"BASH_DEFAULT_TIMEOUT_MS={o['bash_timeout_ms']}" in argv
    assert f"BASH_MAX_TIMEOUT_MS={o['bash_max_timeout_ms']}" in argv
    assert o["bash_timeout_ms"] > 120_000
    assert o["bash_max_timeout_ms"] > o["bash_timeout_ms"]


def test_sandbox_ships_urdf_kinematics_parsers():
    # /robot_description is on the graph; PyKDL without a parser cannot
    # turn it into a chain, and the agent falls back to hand-written DH.
    # The image list and the machine doc must both carry the parsers.
    from pathlib import Path as _P
    root = _P(__file__).resolve().parents[2] / "openrua" / "sandbox"
    dockerfile = (root / "sandbox.Dockerfile").read_text()
    doc = (root / "workspace" / "docs" / "10-machine.md").read_text()
    for pkg in ("python3-pykdl", "urdfdom-py"):
        assert pkg in dockerfile
    for mod in ("PyKDL", "urdf_parser_py"):
        assert mod in doc


# ------------------------------------------------------ minted-token auth
# The sandbox authenticates with its own `claude setup-token`
# token instead of sharing the host's rotating credentials file. Three
# properties are worth holding down by machine, because each of them
# silently un-does the reason for the change.

def test_launch_argv_hands_the_token_over_by_file_never_by_value():
    # By file, so the value reaches the CLI process and nothing else: not
    # an argv other users can read in `ps`, not the container's stored
    # config that `docker inspect` prints.
    a = agents.get("claude-code")
    argv = a.launch_argv(sandbox="box", prompt="p", model="m", max_turns=3,
                         proxy="http://proxy:9999",
                         token_file="/secrets/.env_tw")
    assert "--env-file" in argv
    assert argv[argv.index("--env-file") + 1] == "/secrets/.env_tw"
    assert argv.index("--env-file") < argv.index("box")  # before the container
    assert not any(x.startswith(a.token_env + "=") for x in argv)


def test_launch_argv_without_a_token_asks_docker_for_no_env_file():
    a = agents.get("claude-code")
    argv = a.launch_argv(sandbox="box", prompt="p", model="m", max_turns=3,
                         proxy="http://proxy:9999")
    assert "--env-file" not in argv


def test_sandbox_mounts_carry_no_credentials_when_a_token_authenticates(tmp_path):
    # The point of the token is that nothing is shared: no credentials
    # file on the host to drift, and none inside the sandbox for the agent
    # under test to read.
    a = agents.get("claude-code")
    specs = a.sandbox_mounts(tmp_path / "cfg")
    assert len(specs) == 1
    assert a.credentials.filename not in specs[0]


def test_prepare_profile_needs_no_credentials_when_a_token_authenticates(tmp_path):
    import shutil
    a = agents.get("claude-code")
    (tmp_path / "settings.json").write_text("{}")   # profile, but no creds
    cfg_dir, shared = agents.prepare_profile(tmp_path, a,
                                             require_credentials=False)
    try:
        assert shared is None
        assert (cfg_dir / "settings.json").exists()
    finally:
        shutil.rmtree(cfg_dir, ignore_errors=True)


def test_env_file_values_are_treated_as_secrets(tmp_path):
    # The token must be scrubbed from the record like anything in the
    # credential JSONs: the agent can print its own environment.
    from openrua.runner import record
    a = agents.get("claude-code")
    f = tmp_path / ".env_tw"
    f.write_text(f"# a comment\n{a.token_env}=sk-ant-oat01-{'x' * 90}\n")
    found = record.secret_strings_from_env_file(f)
    assert found == [f"sk-ant-oat01-{'x' * 90}"]
    assert record.secret_strings_from_env_file(tmp_path / "absent") == []


# ------------------------------------------------------- replay operations

def test_replay_ops_are_adapter_neutral_shapes(tmp_path):
    # record.extract_commands speaks shell/write/edit, never Claude's tool
    # names; the adapter does the translation, once, here.
    import json
    a = agents.get("claude-code")
    t = tmp_path / "transcript.jsonl"
    t.write_text("\n".join(json.dumps(d) for d in [
        {"type": "assistant", "message": {"content": [
            {"type": "tool_use", "id": "1", "name": "Write",
             "input": {"file_path": "/tmp/a.py", "content": "print(1)"}}]}},
        {"type": "user", "timestamp": "2026-08-20T07:44:43Z", "message": {"content": [
            {"type": "tool_result", "tool_use_id": "1", "content": "ok"}]}},
        {"type": "assistant", "message": {"content": [
            {"type": "tool_use", "id": "2", "name": "Bash", "input": {"command": "ls"}},
            {"type": "tool_use", "id": "3", "name": "Read", "input": {"file_path": "x"}}]}},
    ]))
    ops = a.replay_ops(t)
    assert [o["kind"] for o in ops] == ["write", "shell"]
    assert ops[0]["path"] == "/tmp/a.py" and ops[0]["output"] == "ok"
    assert ops[1]["command"] == "ls"
    from openrua.runner import record
    out = tmp_path / "commands.sh"
    record.extract_commands(t, out, a)
    body = out.read_text()
    assert "ls" in body and "/tmp/a.py" in body and "Write" not in body


def test_transcript_evidence_names_the_actual_file(tmp_path):
    import json
    a = agents.get("claude-code")
    t = tmp_path / "segment-2.jsonl"
    t.write_text(json.dumps({"type": "rate_limit_event",
                             "rate_limit_info": {"status": "rejected",
                                                 "rateLimitType": "five_hour"}}))
    assert a.quota_since(t)["evidence"].startswith("segment-2.jsonl:1:")
