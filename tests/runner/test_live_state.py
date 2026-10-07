"""Retained resource facts stay complete across concurrent reads and failed writes."""

import io

import pytest

from openrua.runner import live_state


def intercept_writes(monkeypatch, before_write):
    original = io.open

    class Writer:
        def __init__(self, stream):
            self.stream = stream

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return self.stream.__exit__(*args)

        def __getattr__(self, name):
            return getattr(self.stream, name)

        def write(self, text):
            before_write()
            return self.stream.write(text)

    def open_stream(file, mode='r', *args, **kwargs):
        stream = original(file, mode, *args, **kwargs)
        return Writer(stream) if 'w' in mode else stream

    monkeypatch.setattr(io, 'open', open_stream)


def test_readers_keep_complete_facts_while_a_new_state_is_written(tmp_path, monkeypatch):
    previous = {'status': 'running', 'workspace': str(tmp_path / 'files'), 'agent': 'fixture'}
    live_state.save('robot', tmp_path, **previous)
    snapshots = []
    intercept_writes(monkeypatch, lambda: snapshots.append(
        live_state.load('robot', tmp_path, require_running=False)))
    live_state.save('robot', tmp_path, **{**previous, 'status': 'stopped'})
    assert snapshots and all(value == previous for value in snapshots)
    assert live_state.load('robot', tmp_path, require_running=False) == {
        **previous, 'status': 'stopped'}


def test_failed_state_write_keeps_previous_facts_and_removes_partial_file(tmp_path, monkeypatch):
    previous = {'status': 'running', 'workspace': str(tmp_path / 'files')}
    live_state.save('robot', tmp_path, **previous)
    target = live_state.path('robot', tmp_path)
    before = target.read_bytes()
    files = set(target.parent.iterdir())

    def fail():
        raise OSError('injected state write failure')

    intercept_writes(monkeypatch, fail)
    with pytest.raises(OSError, match='injected state write failure'):
        live_state.stopped('robot', tmp_path)
    assert target.read_bytes() == before
    assert set(target.parent.iterdir()) == files
