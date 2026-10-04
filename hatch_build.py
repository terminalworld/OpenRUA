"""Distribution builds require prepared and verified terminal resources."""

from pathlib import Path
import subprocess
import sys

from hatchling.builders.hooks.plugin.interface import BuildHookInterface


class CustomBuildHook(BuildHookInterface):
    def initialize(self, version, build_data):
        if version == 'editable':
            return
        subprocess.run([sys.executable, str(Path(self.root) / 'scripts/terminal_assets.py'), 'verify'],
                       cwd=self.root, check=True)
