"""The agent-facing workspace template is pinned: package reorganisation
must not change one byte of what the agent reads (its hash is recorded in
every trial and compared across campaigns)."""
from openrua.config import load_config
from openrua.sandbox import workspace

PINNED = "812f5dd51bc6fd49e7ae5012ff02808b7421b7f6186e4eb5d482d58ffcabb277"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
