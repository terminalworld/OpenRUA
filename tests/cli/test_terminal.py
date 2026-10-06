"""Administrative commands stay usable even when the default TUI installation is broken."""

import subprocess
import sys

from openrua.cli import main


def test_tui_rejects_plain_chat_flags_before_connecting(capsys):
    assert main(["chat", "--tui", "--follow"]) == 2
    assert "cannot be combined" in capsys.readouterr().err


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
                            'raise SystemExit(main(["chat", "--tui"]))')], capture_output=True, text=True)
    assert result.returncode == 69
    assert "pip install -U openrua" in result.stderr
    assert "Traceback" not in result.stderr
