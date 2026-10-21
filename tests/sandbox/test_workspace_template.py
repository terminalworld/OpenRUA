"""Pin intentional workspace revisions and detect unrelated template changes.

Each trial records the actual hash; published tags retain historical templates.
Review agent-visible changes before updating this expected artifact hash.
"""
from openrua.config import load_config
from openrua.sandbox import workspace

# Reviewed revision: trajectory and IK tools check where the arm ended up
# and refuse to guess a TCP offset; the agent records differences from
# declared facts; cameras are entries with their own topics; simulation-only
# statements live in machine.yaml; the gripper docs are direction-free.
PINNED = "646db928445ce2cae670e9400b5fb06488238cbb19c07574dc58b4f9974e4483"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
