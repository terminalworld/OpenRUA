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


def test_a_newer_release_is_offered_as_a_pinned_component_update(world):
    state, home = world
    offer = updates.offer_for(selection(), home, NOW)
    assert offer.latest == "2.1.320" and not offer.required and not offer.restart
    at = offer.update.index("update")
    assert offer.update[at:] == ["update", "--agent", "claude-code@2.1.320", "--image", "box"]
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



def test_openrua_offers_its_own_newer_release_by_its_installer(world, monkeypatch):
    state, home = world
    state["latest"] = None
    class Answer:
        def __enter__(self):
            return self
        def __exit__(self, *exc):
            return False
        def read(self, *args):
            return b'{"info": {"version": "0.15.0"}}'
    monkeypatch.setattr(updates.urllib.request, "urlopen", lambda url, timeout: Answer())
    monkeypatch.setattr(updates.metadata, "version", lambda name: "0.14.0")
    monkeypatch.setattr(updates, "upgrade_command", lambda: ["pip", "install", "-U", "openrua"])
    offer = updates.openrua_offer(home, NOW)
    assert (offer.name, offer.installed, offer.latest, offer.restart) == ("openrua", "0.14.0", "0.15.0", True)
    assert "OpenRUA 0.15.0 is available" in updates.describe(offer)
    calls = []
    monkeypatch.setattr(updates.subprocess, "call", lambda argv: calls.append(argv) or 0)
    # Updated: this run stops, so the new version is what runs next.
    assert updates.ask(offer, home, read=lambda prompt: "1", show=lambda *a: None) is False
    assert calls == [["pip", "install", "-U", "openrua"]]
    monkeypatch.setattr(updates.metadata, "version", lambda name: "0.15.0")
    assert updates.openrua_offer(home, NOW) is None


def test_a_source_checkout_is_never_offered_an_upgrade(monkeypatch):
    class Dist:
        def read_text(self, name):
            return '{"dir_info": {"editable": true}, "url": "file:///src/OpenRUA"}'
    monkeypatch.setattr(updates.metadata, "distribution", lambda name: Dist())
    assert updates.upgrade_command() is None
    class Wheel:
        def read_text(self, name):
            return None
    monkeypatch.setattr(updates.metadata, "distribution", lambda name: Wheel())
    monkeypatch.setattr(updates.sys, "prefix", "/home/u/.local/share/uv/tools/openrua")
    assert updates.upgrade_command() == ["uv", "tool", "upgrade", "openrua"]


def test_required_update_uses_the_minimum_when_the_registry_is_unavailable(world):
    state, home = world
    state['installed'], state['latest'] = '2.1.200', None
    offer = updates.offer_for(selection(), home, NOW)
    assert offer.required and offer.target == '2.1.251'
    assert 'claude-code@2.1.251' in offer.update


def test_local_official_wheels_can_update_via_the_installer(monkeypatch):
    class Dist:
        def read_text(self, name):
            return '{"url":"file:///tmp/openrua.whl"}'
    monkeypatch.setattr(updates.metadata, 'distribution', lambda name: Dist())
    assert updates.upgrade_command()[-3:] == ['install', '-U', 'openrua']


def test_fullscreen_update_does_not_run_after_continue_and_retains_skip_policy(world):
    state, home = world
    offer = updates.offer_for(selection(), home, NOW)
    class App:
        def screen(self, spec):
            assert spec['mode'] == 'update' and spec['phase'] == 'offer'
            return {'action': 'skip'}
    assert updates.present(offer, home, App()) is True
    assert updates._read(home)['claude-code']['skipped'] == offer.latest


def test_failed_fullscreen_update_requires_an_explicit_retry_and_keeps_the_log(world, monkeypatch):
    state, home = world
    offer = updates.offer_for(selection(), home, NOW)
    log = home / 'output.log'; log.write_text('installer failure\n')
    actions = iter(['update', 'logs', 'back', 'retry', 'continue'])
    commands = []
    class App:
        def screen(self, spec):
            action = next(actions)
            if action == 'back':
                assert spec['phase'] == 'logs' and spec['text'] == log.read_text()
            return {'action': action}
        def progress(self, *a, **k): pass
    def run(offer, home, progress):
        commands.append(offer.update)
        return (1 if len(commands) == 1 else 0), log
    monkeypatch.setattr(updates, 'run_logged', run)
    assert updates.present(offer, home, App()) is True
    assert len(commands) == 2 and commands[0] == commands[1]


def test_logged_installer_output_is_private_and_streamed_without_a_terminal(world):
    import sys
    from dataclasses import replace
    state, home = world
    offer = replace(updates.offer_for(selection(), home, NOW),
                    update=[sys.executable, '-c', 'print("[update] Installing selected version"); print("raw build detail")'])
    messages = []
    code, log = updates.run_logged(offer, home, lambda line, **details: messages.append((line, details)))
    assert code == 0 and 'raw build detail' in log.read_text()
    assert log.stat().st_mode & 0o777 == 0o600
    assert log.parent.stat().st_mode & 0o777 == 0o700
    assert messages[0][1]['compact'] is True
    assert any(details.get('summary') == 'Installing selected version' for _, details in messages)
