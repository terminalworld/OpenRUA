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
# any agent at README.md and machine.yaml, and says /workspace is the one
# place kept after the session; the motion tools cancel their goal when
# interrupted.
PINNED = "f8f5ac893ca3dbf4df4127bf5d1780664ee37df55dcc4f68ed5ad8dc4ba512ab"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
