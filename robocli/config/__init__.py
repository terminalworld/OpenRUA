"""Configuration: the schema (``schema``), the loader (``loader``) and
where files live (``paths``). The schema models and loader functions are
re-exported here, so ``config.Benchmark`` and ``config.validate`` read
naturally at call sites."""

from robocli.config.loader import (  # noqa: F401
    ConfigError, dump, layer_agent, load_user_config, load_yaml, validate,
)
from robocli.config.schema import (  # noqa: F401
    AgentConfig, AgentManifest, AgentOverrides, Arm, ArmSpec, Backend, Base,
    Benchmark, Cameras, Clock, Control, Credentials, Discovery, Frames, Gripper,
    GripperControl, Hand, Machine, Planning, Ports, Protocol, RealBackend,
    ResolvedConfig, RobotFacts, RobotProfile, SimBackend, Simulator, Task, Tf,
    TrajectoryControl, UserConfig, World,
)
