"""The optional terminal client must not change core CLI dependency requirements."""

import subprocess
import sys

from openrua.cli import main


def test_tui_rejects_plain_chat_flags_before_connecting(capsys):
    assert main(["chat", "--tui", "--follow"]) == 2
    assert "cannot be combined" in capsys.readouterr().err


def test_tui_dependency_is_optional_and_has_an_actionable_install_error():
    code = '''
import builtins
original = builtins.__import__
def without_textual(name, *args, **kwargs):
    if name == "textual" or name.startswith("textual."):
        raise ModuleNotFoundError("No module named textual", name="textual")
    return original(name, *args, **kwargs)
builtins.__import__ = without_textual
from openrua.cli import main
assert main(["--version"]) is None
'''
    # argparse exits successfully for --version without importing the TUI.
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    result = subprocess.run([sys.executable, "-c", code.replace('assert main(["--version"]) is None',
                            'raise SystemExit(main(["chat", "--tui"]))')], capture_output=True, text=True)
    assert result.returncode == 69
    assert "pip install -U 'openrua[tui]>=0.1.0'" in result.stderr
    assert "Traceback" not in result.stderr
