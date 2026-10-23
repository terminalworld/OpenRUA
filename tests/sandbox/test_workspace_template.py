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
# interrupted; fjt_send takes a position list that starts with a negative
# number and says "canceled" for a goal canceled elsewhere.
# Camera snapshots create output directories and report image write failures.
PINNED = "0c5f8fb8ee160d0f8988ddf0494816d55d8d6793cf704f36df9c064e43a2da8d"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
