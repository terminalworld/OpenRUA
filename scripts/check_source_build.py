"""Build the package from exactly the source files staged for simulator images."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

from openrua.robot.sim import install


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path.cwd())
    args = parser.parse_args()
    dockerfile, files = install.render({'python': '3.12'}, 'package-check', args.source.resolve())
    with tempfile.TemporaryDirectory(prefix='openrua-source-check-') as directory:
        root = Path(directory)
        install.context(dockerfile, files, root)
        subprocess.run([sys.executable, '-m', 'build', '--wheel', '--outdir', str(root / 'dist'),
                        str(root / 'src')], check=True)


if __name__ == '__main__':
    main()
