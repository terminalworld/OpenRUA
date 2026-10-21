"""What the agent may reach: a free session starts from nothing limited,
a trial from its protocol's limits, and a flag changes only what it names."""

import argparse

import pytest

from openrua.config import schema
from openrua.runner.limits import LIMITS, add_limit_options, limit_overrides


def test_a_free_session_is_not_limited_and_a_trial_is():
    assert schema.Limits().model_dump() == {
        "internet": "open", "web_search": True, "account_connectors": True}
    assert schema.Protocol().limits.model_dump() == {
        "internet": "model-only", "web_search": False, "account_connectors": False}
    assert tuple(schema.Limits.model_fields) == LIMITS
    with pytest.raises(Exception):
        schema.Limits(internet="sometimes")


def test_a_benchmark_protocol_may_open_one_limit_and_keep_the_rest():
    protocol = schema.Protocol(limits={"internet": "open"})
    assert protocol.limits.model_dump() == {
        "internet": "open", "web_search": False, "account_connectors": False}


def test_flags_name_only_what_they_change():
    ap = argparse.ArgumentParser()
    add_limit_options(ap, "default: not limited")
    assert limit_overrides(ap.parse_args([])) == {}
    args = ap.parse_args(["--internet", "model-only", "--no-web-search", "--account-connectors"])
    assert limit_overrides(args) == {"internet": "model-only", "web_search": False,
                                     "account_connectors": True}
    with pytest.raises(SystemExit):
        ap.parse_args(["--internet", "sometimes"])


def test_the_terminal_start_forwards_limit_flags_to_the_session_it_launches(tmp_path, monkeypatch):
    from openrua.cli import build_parser
    from openrua.cli.commands import start
    argv = []
    monkeypatch.setattr(start.service, "start", lambda command, *a: argv.extend(command) or "client")
    args = build_parser().parse_args(["--home", str(tmp_path), "--internet", "model-only",
                                      "--no-web-search"])
    values = dict(robot="panda", sim="robosuite", bench="", agent="codex", model="", name="s1")
    assert start.launch(args, values) == "client"
    assert "--internet=model-only" in argv and "--no-web-search" in argv
    assert not any("account-connectors" in a for a in argv)
