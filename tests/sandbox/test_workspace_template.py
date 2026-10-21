"""Pin intentional workspace revisions and detect unrelated template changes.

Each trial records the actual hash; published tags retain historical templates.
Review agent-visible changes before updating this expected artifact hash.
"""
from openrua.config import load_config
from openrua.sandbox import workspace

# Reviewed revision: cameras are entries with their own topics (cam_snap
# and px2world read them from machine.yaml); simulation-only statements
# live in machine.yaml; the gripper docs and tool are direction-free.
PINNED = "c3d1edb3855fe096f9cf1ad2da8ef12fed28e014e82d69b5d16c6dabdfc5bbe2"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
