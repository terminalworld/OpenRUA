"""The update offer at start: when to offer, what each answer does."""
from datetime import datetime, timedelta, timezone
from types import SimpleNamespace as NS

import pytest

from openrua.cli import updates

NOW = datetime(2026, 10, 21, tzinfo=timezone.utc)


def selection(agent="claude-code", version=None):
    return NS(cfg={"agent": {"name": agent, "version": version},
                   "machine": {"backend": {"kind": "sim", "sandbox_image": "box",
                                           "ros_distro": "jazzy", "image": "sim"}}},
              benchmark=None, simulator="robosuite", robot="panda")


@pytest.fixture
def world(monkeypatch, tmp_path):
    """The sandbox's CLI and the newest release, as a test sets them."""
    state = {"installed": "2.1.300 (Claude Code)", "latest": "2.1.320", "fetches": 0}
    monkeypatch.setattr(updates, "installed_version", lambda agent, image: state["installed"])

    class Answer:
        def __enter__(self):
            state["fetches"] += 1
            return self
        def __exit__(self, *exc):
            return False
        def read(self, *args):
            return ('{"version": "%s"}' % state["latest"]).encode()

    monkeypatch.setattr(updates.urllib.request, "urlopen", lambda url, timeout: Answer())
    return state, tmp_path


def test_a_newer_release_is_offered_with_the_rebuild(world):
    state, home = world
    offer = updates.offer_for(selection(), home, NOW)
    assert offer.latest == "2.1.320" and not offer.required
    assert offer.rebuild[:2] == ["build", "sandbox"]
    assert "2.1.320 is available" in updates.describe(offer)


def test_the_newest_release_is_read_at_most_once_a_day(world):
    state, home = world
    updates.offer_for(selection(), home, NOW)
    updates.offer_for(selection(), home, NOW + timedelta(hours=3))
    assert state["fetches"] == 1
    updates.offer_for(selection(), home, NOW + timedelta(days=1, minutes=1))
    assert state["fetches"] == 2


def test_nothing_is_offered_when_current_pinned_or_skipped(world):
    state, home = world
    state["installed"] = "2.1.320"
    assert updates.offer_for(selection(), home, NOW) is None
    state["installed"] = "2.1.300"
    assert updates.offer_for(selection(version="2.1.300"), home, NOW) is None
    updates.skip_version(home, "claude-code", "2.1.320")
    assert updates.offer_for(selection(), home, NOW) is None
    state["latest"] = "2.1.330"
    assert updates.offer_for(selection(), home, NOW + timedelta(days=2)).latest == "2.1.330"


def test_older_than_the_minimum_is_required(world):
    state, home = world
    state["installed"], state["latest"] = "2.1.200", "2.1.200"
    offer = updates.offer_for(selection(), home, NOW)
    assert offer.required and offer.latest is None


def test_no_network_offers_nothing_but_the_minimum(world, monkeypatch):
    state, home = world
    def offline(url, timeout):
        raise OSError("no route")
    monkeypatch.setattr(updates.urllib.request, "urlopen", offline)
    assert updates.offer_for(selection(), home, NOW) is None


@pytest.mark.parametrize("answer,goes_on,rebuilt,skipped", [
    ("", True, False, None), ("2", True, False, None),
    ("3", True, False, "2.1.320"), ("1", True, True, None),
])
def test_the_three_answers(world, monkeypatch, answer, goes_on, rebuilt, skipped):
    state, home = world
    calls = []
    monkeypatch.setattr(updates.subprocess, "call", lambda argv: calls.append(argv) or 0)
    offer = updates.offer_for(selection(), home, NOW)
    assert updates.ask(offer, home, read=lambda prompt: answer, show=lambda *a: None) is goes_on
    assert bool(calls) is rebuilt
    assert updates._read(home).get("claude-code", {}).get("skipped") == skipped


def test_a_required_update_that_fails_stops_the_start(world, monkeypatch):
    state, home = world
    state["installed"] = "2.1.200"
    monkeypatch.setattr(updates.subprocess, "call", lambda argv: 1)
    offer = updates.offer_for(selection(), home, NOW)
    assert updates.ask(offer, home, read=lambda prompt: "1", show=lambda *a: None) is False
    assert updates.ask(offer, home, read=lambda prompt: "2", show=lambda *a: None) is False
