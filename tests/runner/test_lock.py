"""lock: one writer per trial directory; holders() is what robocli ps prints."""
import json
import os

from robocli.runner import lock


def test_acquire_refuses_a_live_holder_and_takes_over_a_dead_one(tmp_path):
    trial = tmp_path / "seed0"
    trial.mkdir()
    held = lock.acquire(trial, "rc-x-1")
    assert held is not None
    assert lock.acquire(trial, "rc-x-2") is None          # we are alive
    held.release()
    (trial / lock.LOCK_NAME).write_text(json.dumps(
        {"pid": 2 ** 22 + 7, "host": os.uname().nodename, "stem": "rc-nobody-000000"}))
    taken = lock.acquire(trial, "rc-x-3")                  # dead pid, no containers
    assert taken is not None and lock.holder(trial)["stem"] == "rc-x-3"
    taken.release()


def test_holders_lists_live_and_stale_claims_under_a_root(tmp_path):
    a = tmp_path / "b" / "run" / "trials" / "s-0" / "seed0"
    b = tmp_path / "b" / "run" / "trials" / "s-0" / "seed1"
    a.mkdir(parents=True)
    b.mkdir(parents=True)
    held = lock.acquire(a, "rc-live")
    (b / lock.LOCK_NAME).write_text(json.dumps(
        {"pid": 2 ** 22 + 7, "host": os.uname().nodename, "stem": "rc-gone-000000",
         "since": 0}))
    rows = {r["trial"]: r for r in lock.holders(tmp_path)}
    assert rows["b/run/trials/s-0/seed0"]["live"] is True
    assert rows["b/run/trials/s-0/seed0"]["pid"] == os.getpid()
    assert rows["b/run/trials/s-0/seed1"]["live"] is False
    held.release()
    assert lock.holders(tmp_path) == [r for r in lock.holders(tmp_path) if not r["live"]]


def test_a_reused_pid_does_not_keep_a_claim_alive(tmp_path):
    import subprocess, sys, time
    trial = tmp_path / "seed0"
    trial.mkdir()
    other = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    try:
        (trial / lock.LOCK_NAME).write_text(json.dumps(
            {"pid": other.pid, "host": os.uname().nodename, "stem": "rc-reused-000000",
             "since": time.time() - 3600}))                # claimed an hour before it was born
        assert lock.is_live(lock.holder(trial)) is False
        taken = lock.acquire(trial, "rc-x-4")
        assert taken is not None
        taken.release()
    finally:
        other.kill()


def test_holders_asks_docker_once_for_all_claims(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(lock, "running_containers",
                        lambda stem="": calls.append(stem) or ["rc-up-000000-sim"])
    for i in range(3):
        d = tmp_path / f"seed{i}"
        d.mkdir()
        (d / lock.LOCK_NAME).write_text(json.dumps(
            {"pid": 2 ** 22 + 7, "host": os.uname().nodename,
             "stem": "rc-up-000000" if i == 0 else f"rc-down-{i:06d}"}))
    rows = lock.holders(tmp_path)
    assert calls == [""]
    assert [r["live"] for r in rows] == [True, False, False]
