"""Verb ``build``: whitelist -> proxy image.

    python3 -m openrua.proxy.build \
        [--whitelist "<one regex per line>"] [--tag openrua-proxy]

Value output: ``<tag> <digest>`` (one line). Empty whitelist (default)
= deny all internet access.
"""

from __future__ import annotations

import argparse
import hashlib
import subprocess
import sys
from pathlib import Path

from openrua.proxy import ProxyError, IMAGE, PORT

_HERE = Path(__file__).resolve().parent


def build(whitelist: str = "", tag: str = IMAGE,
          port: int = PORT, labels: dict[str, str] | None = None) -> tuple[str, str]:
    """Build the proxy image; returns (tag, digest). ``labels`` are stamped
    alongside the whitelist hash (one per agent, for doctor)."""
    # The image says which whitelist it enforces: doctor compares this
    # label with what the selected agents would emit today.
    digest_in = hashlib.sha256(whitelist.encode()).hexdigest()
    r = subprocess.run(
        ["docker", "build", "-f", str(_HERE / "proxy.Dockerfile"),
         "-t", tag, "--build-arg", f"WHITELIST={whitelist}",
         "--build-arg", f"PORT={port}",
         "--label", f"openrua.whitelist_sha256={digest_in}",
         *(x for k, v in (labels or {}).items() for x in ("--label", f"{k}={v}")),
         str(_HERE)],
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
    ap.add_argument("--tag", default=IMAGE)
    ap.add_argument("--port", type=int, default=PORT,
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
