"""Pin intentional workspace revisions and detect unrelated template changes.

Each trial records the actual hash; published tags retain historical templates.
Review agent-visible changes before updating this expected artifact hash.
"""
from openrua.config import load_config
from openrua.sandbox import workspace

# Reviewed revision: discrepancies with the live graph are logged in
# machine-discrepancies.yaml (worded as a robot config that can drift);
# tools check where the arm ended up; cameras are entries with their own
# topics; simulation-only statements live in machine.yaml.
PINNED = "46d8c36100c519b8c587bb6bfc0da42018f1003d38913795c9c7b227453584a2"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
