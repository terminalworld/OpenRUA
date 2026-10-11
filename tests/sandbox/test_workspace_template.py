"""Pin intentional workspace revisions and detect unrelated template changes.

Each trial records the actual hash; published tags retain historical templates.
Review agent-visible changes before updating this expected artifact hash.
"""
from openrua.config import load_config
from openrua.sandbox import workspace

# Reviewed revision: the TF arm chain is connected; frame comments follow machine.yaml.
PINNED = "4f9286340e2da1a53eff81e09a1c2be44c8c3413b61b464e5cebea193e55d550"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
