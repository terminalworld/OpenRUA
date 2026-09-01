"""SimJobRunner unit contract: one owner thread touches the sim, ever.

Cross-thread work lands on the owner's loop; same-thread calls nest
directly; a fire-and-forget failure is COUNTED and printed, never
silent (audit F8) and never fatal.
"""

from __future__ import annotations

import threading

from robocli.robot.onboard.environment.simthread import SimJobRunner


def _owner_loop(sim, stop):
    sim.bind_current_thread()
    while not stop.is_set():
        sim.run_pending(timeout=0.02)


def test_cross_thread_jobs_execute_on_the_owner():
    sim, stop, seen = SimJobRunner(), threading.Event(), []
    t = threading.Thread(target=_owner_loop, args=(sim, stop))
    t.start()
    try:
        while sim._owner is None:  # wait for bind
            pass
        r = sim.submit(lambda: seen.append(threading.current_thread()) or 42)
        assert r == 42 and seen[0] is t  # ran on the owner, result returned
    finally:
        stop.set()
        t.join()


def test_same_thread_calls_nest_directly():
    sim = SimJobRunner()
    sim.bind_current_thread()
    # a job submitting a job must not deadlock on its own queue
    assert sim.submit(lambda: sim.submit(lambda: "nested")) == "nested"


def test_waited_failure_reraises_to_the_submitter():
    import pytest
    sim, stop = SimJobRunner(), threading.Event()
    t = threading.Thread(target=_owner_loop, args=(sim, stop))
    t.start()
    try:
        while sim._owner is None:
            pass
        with pytest.raises(RuntimeError, match="wedged"):
            sim.submit(lambda: (_ for _ in ()).throw(RuntimeError("wedged")))
    finally:
        stop.set()
        t.join()


def test_fire_and_forget_failure_is_counted_not_silent(capsys):
    sim = SimJobRunner()

    def boom():
        raise RuntimeError("per-step assert")

    # enqueue from a non-owner thread (owner unbound yet), then drain as owner
    fut = sim.submit(boom, wait=False)
    sim.bind_current_thread()
    sim.run_pending()
    assert fut.done() and sim.dropped_errors == 1
    assert "fire-and-forget job failed" in capsys.readouterr().err
