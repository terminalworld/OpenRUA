"""Prepare or verify self-contained Pi terminal assets, without runtime downloads."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil


def hashes(root: Path, exclude=()) -> dict[str, str]:
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(root.rglob('*')) if path.is_file() and str(path.relative_to(root)) not in exclude}


def source_hashes(source: Path) -> dict:
    return {'src/' + key: value for key, value in hashes(source / 'src').items()} | {
        name: hashlib.sha256((source / name).read_bytes()).hexdigest()
        for name in ('package.json', 'package-lock.json')}


def prepare(source: Path, output: Path) -> None:
    lock = json.loads((source / 'package-lock.json').read_text())
    for name, facts in lock['packages'].items():
        if not name:
            continue
        installed = source / name / 'package.json'
        if not installed.is_file() or json.loads(installed.read_text())['version'] != facts['version']:
            raise RuntimeError('Missing or stale dependencies. Run npm ci --prefix ui/terminal --ignore-scripts.')
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)
    shutil.copytree(source / 'src', output / 'src')
    # Preserve package resources and license files; no bundler assumptions about
    # import.meta.url or Pi's optional platform helpers.
    for name in lock['packages']:
        if name:
            shutil.copytree(source / name, output / name)
    manifest = {'format': 1, 'source': source_hashes(source), 'files': hashes(output),
                'dependencies': {name: facts['version'] for name, facts in lock['packages'].items() if name}}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


def verify(source: Path, output: Path) -> None:
    manifest = output / 'manifest.json'
    if not manifest.is_file():
        raise RuntimeError('Pi assets are missing. Run npm ci --prefix ui/terminal --ignore-scripts, '
                           'then python scripts/terminal_assets.py prepare before building distributions.')
    facts = json.loads(manifest.read_text())
    if facts['files'] != hashes(output, ('manifest.json',)):
        raise RuntimeError('Pi assets differ from their manifest. Re-run scripts/terminal_assets.py prepare.')
    if facts['source'] != source_hashes(source):
        raise RuntimeError('Pi source changed after asset preparation. Re-run scripts/terminal_assets.py prepare.')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['prepare', 'verify'])
    parser.add_argument('--source', type=Path, default=Path('ui/terminal'))
    parser.add_argument('--output', type=Path, default=Path('openrua/terminal/assets'))
    args = parser.parse_args()
    try:
        (prepare if args.operation == 'prepare' else verify)(args.source, args.output)
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        parser.exit(1, f'{exc}\n')


if __name__ == '__main__':
    main()
