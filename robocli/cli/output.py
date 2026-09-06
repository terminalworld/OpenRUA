"""How commands print: one listing shape, a table or JSON."""

from __future__ import annotations

import argparse
import json


def add_json(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--json", action="store_true", help="machine-readable output")


def print_listing(rows: list[dict], as_json: bool) -> None:
    """Rows of {name, source, description, shadowed_by}: a table, or JSON."""
    if as_json:
        print(json.dumps(rows, indent=2))
        return
    for r in rows:
        tag = "" if r["source"] == "bundled" else f"  [{r['source']}]"
        print(f"{r['name']:<20} {r['description']}{tag}")
        if r.get("shadowed_by"):
            print(f"{'':<20} note: {r['shadowed_by']} has the same name and is "
                  "ignored; rename it to use it")
