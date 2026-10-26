"""Administrative commands stay usable even when the default TUI installation is broken."""

import subprocess
import sys

import pytest

from openrua.cli import main


@pytest.mark.parametrize('argv', [['--cli'], ['chat'], ['chat', '--tui']])
def test_removed_plain_chat_entry_points_are_rejected(argv):
    with pytest.raises(SystemExit) as exc:
        main(argv)
    assert exc.value.code == 2


def test_missing_tui_dependency_has_an_actionable_repair_error():
    code = '''
import builtins
original = builtins.__import__
def without_runtime(name, *args, **kwargs):
    if name == "nodejs_wheel" or name.startswith("nodejs_wheel."):
        raise ModuleNotFoundError("No module named nodejs_wheel", name="nodejs_wheel")
    return original(name, *args, **kwargs)
builtins.__import__ = without_runtime
from openrua.cli import main
assert main(["--version"]) is None
'''
    # argparse exits successfully for --version without importing the TUI.
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    result = subprocess.run([sys.executable, "-c", code.replace('assert main(["--version"]) is None',
                            'raise SystemExit(main(["--resume", "running"]))')], capture_output=True, text=True)
    assert result.returncode == 69
    assert "pip install -U openrua" in result.stderr
    assert "Traceback" not in result.stderr
