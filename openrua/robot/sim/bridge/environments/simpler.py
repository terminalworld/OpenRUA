"""SimplerEnv loader: the WidowX Bridge tasks, as ported to ManiSkill 3.

SimplerEnv (arXiv 2405.05941) evaluates real-to-sim on SAPIEN 2 through
its ManiSkill2 fork; its authors ported the four WidowX Bridge tasks into
ManiSkill 3 (``mani_skill.envs.tasks.digital_twins.bridge_dataset_eval``),
which is what loads here: the same scenes, object placements (the
``episode_id`` grid) and success predicate, on SAPIEN 3. The Google
Robot tasks exist only in the SAPIEN 2 original and are not here.

The Bridge WidowX ships with one control mode (an end-effector delta
pose). The bridge drives joints, so this loader gives the real2sim
agents a ``pd_joint_pos`` mode next to it, built from their own arm
gains; the scenes and the predicate are untouched.
"""

from __future__ import annotations

from .maniskill import _assets_next_to_the_venv, _success, make_kwargs

# (env id, SimplerEnv's task name, the instruction the env states)
TASKS = [
    ("PutSpoonOnTableClothInScene-v1", "widowx_spoon_on_towel", "put the spoon on the towel"),
    ("PutCarrotOnPlateInScene-v1", "widowx_carrot_on_plate", "put carrot on plate"),
    ("StackGreenCubeOnYellowCubeBakedTexInScene-v1", "widowx_stack_cube",
     "stack the green block on the yellow block"),
    ("PutEggplantInBasketScene-v1", "widowx_put_eggplant_in_basket", "put eggplant into yellow basket"),
]
SUITES = {"bridge": TASKS}


def _add_joint_position_mode() -> None:
    from mani_skill.agents.controllers import PDJointPosControllerConfig
    from mani_skill.agents.registration import register_agent
    from mani_skill.envs.tasks.digital_twins.bridge_dataset_eval import base_env

    def with_joint_pos(cls):
        if "pd_joint_pos" in cls.__dict__.get("_openrua_modes", ()):
            return cls

        class JointPos(cls):
            uid = cls.uid
            _openrua_modes = ("pd_joint_pos",)

            @property
            def _controller_configs(self):
                configs = super()._controller_configs
                ee = next(iter(configs.values()))
                arm = PDJointPosControllerConfig(
                    joint_names=self.arm_joint_names, lower=None, upper=None,
                    stiffness=self.arm_stiffness, damping=self.arm_damping,
                    force_limit=self.arm_force_limit, friction=self.arm_friction,
                    normalize_action=False)
                configs["pd_joint_pos"] = dict(arm=arm, gripper=ee["gripper"])
                return configs

        JointPos.__name__ = cls.__name__
        return register_agent(asset_download_ids=["widowx250s"], override=True)(JointPos)

    for name in ("WidowX250SBridgeDatasetFlatTable", "WidowX250SBridgeDatasetSink"):
        setattr(base_env, name, with_joint_pos(getattr(base_env, name)))


class SimplerLoader:
    def tasks(self, cfg: dict, task_suite: str) -> list[dict]:
        return [{"task_id": i, "language": s} for i, (_, _, s) in enumerate(SUITES[task_suite])]

    def create(self, cfg: dict, task_suite: str, task_id: int):
        _assets_next_to_the_venv()
        import gymnasium as gym
        import mani_skill.envs  # noqa: F401

        _add_joint_position_mode()
        env_id, name, sentence = SUITES[task_suite][task_id]
        env = gym.make(env_id, **make_kwargs(cfg, env_id))
        return env, {"task_id": task_id, "env_id": env_id, "name": name, "language": sentence}

    def init_state(self, ctx: dict, seed: int):
        """The seed indexes the task's placement grid (its episode_id)."""
        return seed

    def reset(self, env, ctx: dict, state) -> None:
        env.reset(seed=int(state), options={"episode_id": int(state)})

    def success(self, env) -> bool:
        return _success(env)

    def task_info(self, env, ctx: dict) -> dict:
        u = env.unwrapped
        language = u.get_language_instruction()[0] if hasattr(u, "get_language_instruction") \
            else ctx.get("language", "")
        return {"language": language, "name": ctx.get("name", "")}


LOADER = SimplerLoader()
