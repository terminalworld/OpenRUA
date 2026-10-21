"""Pin intentional workspace revisions and detect unrelated template changes.

Each trial records the actual hash; published tags retain historical templates.
Review agent-visible changes before updating this expected artifact hash.
"""
from openrua.config import load_config
from openrua.sandbox import workspace

# Reviewed revision: the gripper docs and tool are direction-free; the
# manifest entry carries unit, open, closed and what the gripper does.
PINNED = "71205ac3ea1cea40eedfd1bd59d48ac485ed18f1c6fa2f7c0ce8ead52d5c9668"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
