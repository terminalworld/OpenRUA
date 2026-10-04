"""Startup choices backed by recorded simulation checks, not registry membership."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from openrua.config import paths

FIELDS = ('robot', 'sim', 'bench')


def profile_digest() -> str:
    """Invalidate startup evidence when bundled environment declarations change."""
    digest = hashlib.sha256()
    for kind in ('robots', 'simulators', 'benchmarks'):
        root = paths.bundled(kind)
        for path in sorted(root.rglob('*')):
            if path.is_file():
                digest.update(f'{kind}/{path.relative_to(root)}'.encode())
                digest.update(path.read_bytes())
    return digest.hexdigest()


def verified_environments() -> list[dict]:
    root = paths.package_config_path().parent / 'startup'
    digest = profile_digest()
    rows = []
    for path in sorted(root.glob('*.json')):
        report = json.loads(path.read_text())
        if report.get('profiles_sha256') == digest and report.get('status') == 'passed':
            rows.append(report)
    return rows


def options(rows: list[dict], values: dict, field: str) -> list[str]:
    """A field depends only on preceding selections, allowing upstream changes."""
    before = FIELDS[:FIELDS.index(field)]
    return sorted({row['selection'][field] for row in rows
                   if all(not values.get(key) or values[key] == row['selection'][key]
                          for key in before)})


def select(rows: list[dict], values: dict, field: str, value: str) -> dict:
    """Keep compatible descendants; fill a unique choice, otherwise require a choice."""
    result = {**values, field: value}
    for key in FIELDS[FIELDS.index(field) + 1:]:
        available = options(rows, result, key)
        if result.get(key) not in available:
            result[key] = available[0] if len(available) == 1 else ''
    return result
