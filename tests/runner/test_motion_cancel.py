"""An interrupt also cancels the robot's motions, whatever started them."""

import asyncio
from types import SimpleNamespace

from openrua.runner import live, motion
from openrua.runner.managed import ManagedSession
from openrua.sessions.core import Session, initial_state
from openrua.sessions.store import SQLiteStore
from tests.sessions.test_execution import setup


def test_the_motion_actions_are_each_arms_trajectory_and_gripper_and_the_planners():
    machine = {"arms": [{"ports": {"trajectory": "/left/fjt", "gripper": "/left/grip", "twist": "/servo"}},
                        {"ports": {"trajectory": "/right/fjt"}}],
               "planning": {"move_action": "/move_action"}}
    assert motion.motion_actions(machine) == ["/left/fjt", "/left/grip", "/right/fjt", "/move_action"]
    assert motion.motion_actions({"arms": []}) == []


def test_every_action_is_asked_to_cancel_all_its_goals_at_once(monkeypatch):
    started = []
    class Call:
        def __init__(self, argv, **_):
            started.append(argv)
            self.code = 1 if "/missing" in argv[-1] else 0
        def wait(self, timeout):
            return self.code
    monkeypatch.setattr(motion.subprocess, "Popen", Call)
    assert motion.cancel_motion("box", ["/arm/fjt", "/missing"]) == {"/arm/fjt": True, "/missing": False}
    assert started[0][:3] == ["docker", "exec", "box"]
    assert "/arm/fjt/_action/cancel_goal action_msgs/srv/CancelGoal '{}'" in started[0][-1]


def test_an_interrupt_cancels_motion_and_says_so(tmp_path):
    async def run():
        store, session, native, execution = await setup(tmp_path)
        asked = []
        robot = SimpleNamespace(stop_motion=lambda: asked.append(True) or {"/arm/fjt": True, "/grip": False})
        managed = ManagedSession(robot, execution, store)
        execution.on_interrupt = managed.cancel_motion
        message = await execution.command("enqueue", client_id="c", request_id="r", text="move")
        await native.frames.put({"kind": "turn_started", "turn_id": message["id"]})
        for _ in range(200):
            if store.snapshot()["state"]["messages"][0]["status"] == "running":
                break
            await asyncio.sleep(0.01)
        await execution.command("interrupt", message_id=message["id"])
        for _ in range(200):
            events = [e for e in store.events(0, 1000) if e["kind"] == "motion_canceled"]
            if events:
                break
            await asyncio.sleep(0.01)
        assert asked == [True]
        assert events[0]["data"] == {"canceled": ["/arm/fjt"], "failed": ["/grip"]}
        await execution.close()
        store.close()
    asyncio.run(run())
