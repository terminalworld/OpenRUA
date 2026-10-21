"""Pin intentional workspace revisions and detect unrelated template changes.

Each trial records the actual hash; published tags retain historical templates.
Review agent-visible changes before updating this expected artifact hash.
"""
from openrua.config import load_config
from openrua.sandbox import workspace

# Reviewed revision: simulation-only statements moved to machine.yaml
# (simulation section, notes); the gripper docs and tool are
# direction-free, the manifest entry carrying unit, open, closed and
# what the gripper does.
PINNED = "1c7ee6fb6b87cd29b4fd11d34211398983040671dd52c61e6cdf182e13e1f83b"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
