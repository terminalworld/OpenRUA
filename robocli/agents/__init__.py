"""Agents: the contract, the registry, the launcher, the prompts.

Every fact about a particular coding agent lives in its manifest
(``configs/agents/<name>.yaml``) and its hooks module
(``plugins/agents/<hooks>.py``); every consumer reaches an agent through
``get(name)``. Configs carry ``agent.name`` (the package default is in
``configs/config.yaml``), recorded per trial in ``operator_meta.agent``
so post-hoc tools resolve the agent a trial actually ran.
"""

from robocli.agents.base import HOOK_NAMES, Agent, Credentials  # noqa: F401
from robocli.agents.credentials import prepare_profile  # noqa: F401
from robocli.agents.prompts import PROMPT, RESUME_PROMPT  # noqa: F401
from robocli.agents.registry import (  # noqa: F401
    Listed, Manifest, available, fact_sha256, get, manifest, manifests, preinstall,
    split_pin, whitelist,
)
