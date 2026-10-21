"""Verb ``build``: -> proxy image.

    python3 -m openrua.proxy.build [--tag openrua-proxy] [--port 8888]

Value output: ``<tag> <digest>`` (one line). The image holds no
whitelist: ``up`` gives each proxy its own at start.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from openrua.proxy import ProxyError, IMAGE, PORT

_HERE = Path(__file__).resolve().parent


def build(tag: str = IMAGE, port: int = PORT) -> tuple[str, str]:
    """Build the proxy image; returns (tag, digest)."""
    r = subprocess.run(
        ["docker", "build", "-f", str(_HERE / "proxy.Dockerfile"),
         "-t", tag, "--build-arg", f"PORT={port}", str(_HERE)])
    if r.returncode != 0:
        raise ProxyError(f"docker build {tag} failed (exit {r.returncode}); see build output above")
    digest = subprocess.run(
        ["docker", "inspect", "--format", "{{.Id}}", tag],
        capture_output=True, text=True, check=True).stdout.strip()
    return tag, digest


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--tag", default=IMAGE)
    ap.add_argument("--port", type=int, default=PORT,
                    help="listen port, baked into the image and stamped "
                    "as a label `up` reads back")
    args = ap.parse_args()
    try:
        tag, digest = build(tag=args.tag, port=args.port)
    except ProxyError as e:
        raise SystemExit(str(e))
    print(f"{tag} {digest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
