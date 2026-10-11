"""Pin intentional workspace revisions and detect unrelated template changes.

Each trial records the actual hash; published tags retain historical templates.
Review agent-visible changes before updating this expected artifact hash.
"""
from openrua.config import load_config
from openrua.sandbox import workspace

# Reviewed revision: the planning frame is read from machine.yaml, not assumed.
PINNED = "6014790de766e116768ba74b44292686a0069f0b8b11610c6ceb1c7d7bee8a98"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
