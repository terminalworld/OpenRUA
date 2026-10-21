"""Pin intentional workspace revisions and detect unrelated template changes.

Each trial records the actual hash; published tags retain historical templates.
Review agent-visible changes before updating this expected artifact hash.
"""
from openrua.config import load_config
from openrua.sandbox import workspace

# Reviewed revision: the agent checks declared facts and records differences
# in machine-discrepancies.yaml; cameras are entries with their own topics;
# simulation-only statements live in machine.yaml; the gripper docs and
# tool are direction-free.
PINNED = "5fe10195886f67786cc23b80d6f7609c5a06a8e1f21caa1378a4e3e7a98d6292"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
