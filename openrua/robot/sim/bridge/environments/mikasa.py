"""MIKASA-Robo loader: memory-intensive manipulation on ManiSkill 3.

MIKASA-Robo (CognitiveAISystems/MIKASA-Robo, arXiv 2502.07007) registers
its tasks with ManiSkill's registry when its VLA package is imported; the
``-VLA-v0`` variants are the benchmark's language-conditioned set, each
with a sentence the task states (``evaluate()["language_instruction"]``,
which fills in per-episode values such as a rotation angle). The table
below is the benchmark's own ``mikasa_robo_vla_envs.csv``, kept here so
``openrua benchmarks mikasa`` answers without a simulator. The scenes and
the predicate are the benchmark's; the Panda and its ``pd_joint_pos``
controller are ManiSkill's.
"""

from __future__ import annotations

from .maniskill import _assets_next_to_the_venv, _success, make_kwargs

TASKS = [
    ("ShellGameTouch-VLA-v0", "Observe which cup hides the ball, wait, then touch that cup."),
    ("ShellGamePush-VLA-v0", "Observe which cup hides the ball, wait, then push that cup forward."),
    ("InterceptSlow-VLA-v0", "Intercept the rolling ball by moving to its path and deflecting it toward the target."),
    ("InterceptMedium-VLA-v0", "Intercept the rolling ball by moving to its path and deflecting it toward the target."),
    ("InterceptFast-VLA-v0", "Intercept the rolling ball by moving to its path and deflecting it toward the target."),
    ("InterceptGrabSlow-VLA-v0", "Intercept the rolling ball and grasp it to stop it."),
    ("InterceptGrabMedium-VLA-v0", "Intercept the rolling ball and grasp it to stop it."),
    ("InterceptGrabFast-VLA-v0", "Intercept the rolling ball and grasp it to stop it."),
    ("RotateLenientPos-VLA-v0", "Rotate the peg by {angle_deg} degrees to match the target angle."),
    ("RotateLenientPosNeg-VLA-v0", "Rotate the peg by {angle_deg} degrees to match the target angle."),
    ("RotateStrictPos-VLA-v0", "Rotate the peg by {angle_deg} degrees to match the target angle while keeping the center of the peg in place."),
    ("RotateStrictPosNeg-VLA-v0", "Rotate the peg by {angle_deg} degrees to match the target angle while keeping the center of the peg in place."),
    ("TakeItBack-VLA-v0", "Push the cube onto the red target, and when the target changes color, return the cube to its original position."),
    ("RememberColor3-VLA-v0", "Observe the cube's color, wait, then touch the cube of the same color."),
    ("RememberColor5-VLA-v0", "Observe the cube's color, wait, then touch the cube of the same color."),
    ("RememberColor9-VLA-v0", "Observe the cube's color, wait, then touch the cube of the same color."),
    ("RememberShape3-VLA-v0", "Observe the object's shape, wait, then touch the object of the same shape."),
    ("RememberShape5-VLA-v0", "Observe the object's shape, wait, then touch the object of the same shape."),
    ("RememberShape9-VLA-v0", "Observe the object's shape, wait, then touch the object of the same shape."),
    ("RememberShapeAndColor3x2-VLA-v0", "Observe the object's shape and color, wait, then touch the object of the same shape and color."),
    ("RememberShapeAndColor3x3-VLA-v0", "Observe the object's shape and color, wait, then touch the object of the same shape and color."),
    ("RememberShapeAndColor5x3-VLA-v0", "Observe the object's shape and color, wait, then touch the object of the same shape and color."),
    ("BunchOfColors3-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in any order and press the center button."),
    ("BunchOfColors5-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in any order and press the center button."),
    ("BunchOfColors7-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in any order and press the center button."),
    ("SeqOfColors3-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in any order and press the center button."),
    ("SeqOfColors5-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in any order and press the center button."),
    ("SeqOfColors7-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in any order and press the center button."),
    ("ChainOfColors3-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in the same order as the cubes were shown and press the center button."),
    ("ChainOfColors5-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in the same order as the cubes were shown and press the center button."),
    ("ChainOfColors7-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in the same order as the cubes were shown and press the center button."),
    ("ShellGameShuffleTouch-VLA-v0", "Observe which cup hides the ball, track the cups as they shuffle, then touch the correct cup."),
    ("ShellGameShuffleColorLampTouch-VLA-v0", "Observe which color is under each cup, track the cups as they shuffle, then touch the cup matching the lamp color."),
    ("ShellGameColorLampTouch-VLA-v0", "Observe which color is under each cup, then touch the cup matching the lamp color."),
    ("FindImposterColor3-VLA-v0", "Observe the cubes shown, wait, then touch the cube whose color was not present before."),
    ("FindImposterColor5-VLA-v0", "Observe the cubes shown, wait, then touch the cube whose color was not present before."),
    ("FindImposterColor9-VLA-v0", "Observe the cubes shown, wait, then touch the cube whose color was not present before."),
    ("FindImposterShape3-VLA-v0", "Observe the shapes shown, wait, then touch the object whose shape was not present before."),
    ("FindImposterShape5-VLA-v0", "Observe the shapes shown, wait, then touch the object whose shape was not present before."),
    ("FindImposterShape9-VLA-v0", "Observe the shapes shown, wait, then touch the object whose shape was not present before."),
    ("FindImposterShapeAndColor3x2-VLA-v0", "Observe the objects shown, wait, then touch the object whose shape and color combination was not present before."),
    ("FindImposterShapeAndColor3x3-VLA-v0", "Observe the objects shown, wait, then touch the object whose shape and color combination was not present before."),
    ("FindImposterShapeAndColor5x3-VLA-v0", "Observe the objects shown, wait, then touch the object whose shape and color combination was not present before."),
    ("BatteriesCheckerEasy-3-VLA-v0", "Find all working batteries by inserting each one into the socket, observing the lamp result, and then pressing the button to confirm."),
    ("BatteriesCheckerEasy-6-VLA-v0", "Find all working batteries by inserting each one into the socket, observing the lamp result, and then pressing the button to confirm."),
    ("BatteriesCheckerHard-3-VLA-v0", "Find all working batteries by inserting each one into the socket, observing the lamp result, returning it from the socket to its initial slot, and then pressing the button to confirm."),
    ("BatteriesCheckerHard-6-VLA-v0", "Find all working batteries by inserting each one into the socket, observing the lamp result, returning it from the socket to its initial slot, and then pressing the button to confirm."),
    ("BlinkCountButtonPressEasy-VLA-v0", "Count how many times the blue lamp blinks, press the red button exactly that many times when the red lamp turns green, then press the black button to submit your answer."),
    ("BlinkCountButtonPressMedium-VLA-v0", "Count how many times the blue lamp blinks, press the red button exactly that many times when the red lamp turns green, then press the black button to submit your answer."),
    ("BlinkCountButtonPressHard-VLA-v0", "Count how many times the blue lamp blinks, press the red button exactly that many times when the red lamp turns green, then press the black button to submit your answer."),
    ("RememberColor3-Long-VLA-v0", "Observe the cube's color, wait, then touch the cube of the same color."),
    ("RememberColor5-Long-VLA-v0", "Observe the cube's color, wait, then touch the cube of the same color."),
    ("RememberColor9-Long-VLA-v0", "Observe the cube's color, wait, then touch the cube of the same color."),
    ("RememberShape3-Long-VLA-v0", "Observe the object's shape, wait, then touch the object of the same shape."),
    ("RememberShape5-Long-VLA-v0", "Observe the object's shape, wait, then touch the object of the same shape."),
    ("RememberShape9-Long-VLA-v0", "Observe the object's shape, wait, then touch the object of the same shape."),
    ("RememberShapeAndColor3x2-Long-VLA-v0", "Observe the object's shape and color, wait, then touch the object of the same shape and color."),
    ("RememberShapeAndColor3x3-Long-VLA-v0", "Observe the object's shape and color, wait, then touch the object of the same shape and color."),
    ("RememberShapeAndColor5x3-Long-VLA-v0", "Observe the object's shape and color, wait, then touch the object of the same shape and color."),
    ("BunchOfColors3-Long-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in any order and press the center button."),
    ("BunchOfColors5-Long-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in any order and press the center button."),
    ("BunchOfColors7-Long-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in any order and press the center button."),
    ("SeqOfColors3-Long-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in any order and press the center button."),
    ("SeqOfColors5-Long-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in any order and press the center button."),
    ("SeqOfColors7-Long-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in any order and press the center button."),
    ("ChainOfColors3-Long-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in the same order as the cubes were shown and press the center button."),
    ("ChainOfColors5-Long-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in the same order as the cubes were shown and press the center button."),
    ("ChainOfColors7-Long-VLA-v0", "Observe which colored cubes appear during the cue, wait, then touch all of them in the same order as the cubes were shown and press the center button."),
    ("ShellGameShuffleTouch-Long-VLA-v0", "Observe which cup hides the ball, track the cups as they shuffle, then touch the correct cup."),
    ("ShellGameShuffleColorLampTouch-Long-VLA-v0", "Observe which color is under each cup, track the cups as they shuffle, then touch the cup matching the lamp color."),
    ("BlinkCountButtonPressEasy-Long-VLA-v0", "Count how many times the blue lamp blinks, press the red button exactly that many times when the red lamp turns green, then press the black button to submit your answer."),
    ("BlinkCountButtonPressMedium-Long-VLA-v0", "Count how many times the blue lamp blinks, press the red button exactly that many times when the red lamp turns green, then press the black button to submit your answer."),
    ("BlinkCountButtonPressHard-Long-VLA-v0", "Count how many times the blue lamp blinks, press the red button exactly that many times when the red lamp turns green, then press the black button to submit your answer."),
    ("TraceShapeEasy-VLA-v0", "Watch the red cube trace a shape on the table. When the lamp turns green, pick up the green cube and trace exactly the same shape."),
    ("TraceShapeMedium-VLA-v0", "Watch the red cube trace a shape on the table. When the lamp turns green, pick up the green cube and trace exactly the same shape."),
    ("TraceShapeHard-VLA-v0", "Watch the red cube trace a shape on the table. When the lamp turns green, pick up the green cube and trace exactly the same shape."),
    ("TraceShapeSeqEasy-VLA-v0", "Watch the red cube trace a sequence of shapes. When the lamp turns green, pick up the green cube and trace the same sequence in order. After finishing all shapes, press the button to submit your answer."),
    ("TraceShapeSeqMedium-VLA-v0", "Watch the red cube trace a sequence of shapes. When the lamp turns green, pick up the green cube and trace the same sequence in order. After finishing all shapes, press the button to submit your answer."),
    ("TraceShapeSeqHard-VLA-v0", "Watch the red cube trace a sequence of shapes. When the lamp turns green, pick up the green cube and trace the same sequence in order. After finishing all shapes, press the button to submit your answer."),
    ("TimedTransferEasy-VLA-v0", "When the white lamp turns green, start counting steps from that exact moment. Move the blue cube from the green disc to the red disc exactly on step 100 of that count."),
    ("TimedTransferMedium-VLA-v0", "When the white lamp turns green, start counting steps from that exact moment. Move the blue cube from the green disc to the red disc exactly on step 150 of that count."),
    ("TimedTransferHard-VLA-v0", "When the white lamp turns green, start counting steps from that exact moment. Move the blue cube from the green disc to the red disc exactly on step 200 of that count."),
    ("TimedTransferEasy-Long-VLA-v0", "When the white lamp turns green, start counting steps from that exact moment. Move the blue cube from the green disc to the red disc exactly on step 300 of that count."),
    ("TimedTransferMedium-Long-VLA-v0", "When the white lamp turns green, start counting steps from that exact moment. Move the blue cube from the green disc to the red disc exactly on step 500 of that count."),
    ("TimedTransferHard-Long-VLA-v0", "When the white lamp turns green, start counting steps from that exact moment. Move the blue cube from the green disc to the red disc exactly on step 1000 of that count."),
    ("GatherAndRecall1-VLA-v0", "Move all cubes onto the disc. A lamp will briefly flash while you work. After all cubes are placed, press the button matching the flash color."),
    ("GatherAndRecall3-VLA-v0", "Move all cubes onto the disc. A lamp will briefly flash while you work. After all cubes are placed, press the button matching the flash color."),
    ("GatherAndRecall5-VLA-v0", "Move all cubes onto the disc. A lamp will briefly flash while you work. After all cubes are placed, press the button matching the flash color."),
    ("GatherAndRecall7-VLA-v0", "Move all cubes onto the disc. A lamp will briefly flash while you work. After all cubes are placed, press the button matching the flash color."),
    ("GatherAndRecall9-VLA-v0", "Move all cubes onto the disc. A lamp will briefly flash while you work. After all cubes are placed, press the button matching the flash color."),
]
SUITES = {"vla": TASKS}


class MikasaLoader:
    def tasks(self, cfg: dict, task_suite: str) -> list[dict]:
        return [{"task_id": i, "language": s} for i, (_, s) in enumerate(SUITES[task_suite])]

    def create(self, cfg: dict, task_suite: str, task_id: int):
        _assets_next_to_the_venv()
        import gymnasium as gym
        import mani_skill.envs  # noqa: F401
        import mikasa_robo_suite.vla.memory_envs  # noqa: F401  registers the -VLA-v0 tasks

        env_id, sentence = SUITES[task_suite][task_id]
        env = gym.make(env_id, **make_kwargs(cfg, env_id))
        return env, {"task_id": task_id, "env_id": env_id, "language": sentence}

    def init_state(self, ctx: dict, seed: int):
        return seed

    def reset(self, env, ctx: dict, state) -> None:
        env.reset(seed=int(state))

    def success(self, env) -> bool:
        return _success(env)

    def task_info(self, env, ctx: dict) -> dict:
        stated = env.unwrapped.evaluate().get("language_instruction")
        if isinstance(stated, (list, tuple)):
            stated = stated[0] if stated else None
        language = stated if isinstance(stated, str) else ctx.get("language", "")
        return {"language": language, "name": ctx.get("env_id", "")}


LOADER = MikasaLoader()
