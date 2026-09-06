"""The agent-facing workspace template is pinned: package reorganisation
must not change one byte of what the agent reads (its hash is recorded in
every trial and compared across campaigns)."""
from robocli.bench.run import load_config
from robocli.sandbox import workspace

PINNED = "f5999634776108746bd4b5476c9e5643e49a4fbd3fda7c0a59d0411eb5f60836"


def test_workspace_template_hash_is_unchanged():
    assert workspace.template_hash(load_config("libero_pro")) == PINNED
