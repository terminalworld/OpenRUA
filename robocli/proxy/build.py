"""Verb ``build``: whitelist -> wall image.

    python3 -m robocli.proxy.build \
        [--whitelist "<one regex per line>"] [--tag robocli-proxy]

Value output: ``<tag> <digest>`` (one line). Empty whitelist (default)
= deny all internet access.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from robocli.proxy import ProxyError

_HERE = Path(__file__).resolve().parent


def build(whitelist: str = "", tag: str = "robocli-proxy",
          port: int = 8888) -> tuple[str, str]:
    """Build the wall image; returns (tag, digest)."""
    r = subprocess.run(
        ["docker", "build", "-f", str(_HERE / "proxy.Dockerfile"),
         "-t", tag, "--build-arg", f"WHITELIST={whitelist}",
         "--build-arg", f"PORT={port}", str(_HERE)],
        capture_output=True, text=True)
    if r.returncode != 0:
        raise ProxyError(f"docker build {tag} failed:\n{r.stderr[-2000:]}")
    digest = subprocess.run(
        ["docker", "inspect", "--format", "{{.Id}}", tag],
        capture_output=True, text=True, check=True).stdout.strip()
    return tag, digest


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--whitelist", default="",
                    help="one regex per line (empty = deny all internet access)")
    ap.add_argument("--tag", default="robocli-proxy")
    ap.add_argument("--port", type=int, default=8888,
                    help="listen port, baked into the image and stamped "
                    "as a label `up` reads back")
    args = ap.parse_args()
    try:
        tag, digest = build(whitelist=args.whitelist,
                            tag=args.tag, port=args.port)
    except ProxyError as e:
        raise SystemExit(str(e))
    print(f"{tag} {digest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
