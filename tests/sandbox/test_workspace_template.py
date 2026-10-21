"""Pin intentional workspace revisions and detect unrelated template changes.

Each trial records the actual hash; published tags retain historical templates.
Review agent-visible changes before updating this expected artifact hash.
"""
from openrua.config import load_config
from openrua.sandbox import workspace

# Reviewed revision: discrepancies with the live graph are logged in
# machine-discrepancies.yaml (worded as a robot config that can drift);
# tools check where the arm ended up; cameras are entries with their own
# topics; simulation-only statements live in machine.yaml. AGENTS.md points
# any agent at README.md and machine.yaml; the motion tools cancel their
# goal when interrupted.
PINNED = "14124e656176a18041a4c8b0e570c731f5129d7c7be59d15c64b8a300ed61bf7"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
