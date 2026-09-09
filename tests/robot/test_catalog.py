"""The catalog: a loader's tasks hook, asked without an env."""

from __future__ import annotations

import json
import subprocess
import sys

from openrua.robot.sim.bridge import catalog


def _loader_file(tmp_path):
    body = "class L:\n" + "".join(
        f"    def {n}(self, *a, **k): pass\n"
        for n in ("create", "init_state", "reset", "success", "task_info"))
    body += ("    def tasks(self, cfg, suite):\n"
             "        return [{'task_id': i, 'language': f'{suite} {i}'} for i in range(2)]\n")
    p = tmp_path / "loader.py"
    p.write_text(body + "LOADER = L()\n")
    return p


def test_catalog_asks_the_hook_per_suite(tmp_path):
    cfg = {"task": {"loader": str(_loader_file(tmp_path)), "suites": ["a", "b"]}}
    assert catalog.catalog(cfg, ["a", "b"]) == {
        "a": [{"task_id": 0, "language": "a 0"}, {"task_id": 1, "language": "a 1"}],
        "b": [{"task_id": 0, "language": "b 0"}, {"task_id": 1, "language": "b 1"}]}


def test_the_entry_writes_its_answer_to_a_file(tmp_path):
    import yaml
    cfg = {"task": {"loader": str(_loader_file(tmp_path)), "suites": ["a"]}}
    (tmp_path / "cfg.yaml").write_text(yaml.safe_dump(cfg))
    out = tmp_path / "tasks.json"
    subprocess.run([sys.executable, "-m", "openrua.robot.sim.bridge.catalog",
                    "--config", str(tmp_path / "cfg.yaml"), "--out", str(out)], check=True)
    assert json.loads(out.read_text()) == {"a": [{"task_id": 0, "language": "a 0"},
                                                 {"task_id": 1, "language": "a 1"}]}
