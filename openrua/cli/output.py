"""How commands print: one listing shape, a table or JSON."""

from __future__ import annotations

import argparse
import json


def add_json(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--json", action="store_true", help="machine-readable output")


def print_listing(rows: list[dict], as_json: bool) -> None:
    """Rows of {name, description, ...}: a table, or JSON."""
    if as_json:
        print(json.dumps(rows, indent=2))
        return
    for r in rows:
        print(f"{r['name']:<20} {r['description']}")
