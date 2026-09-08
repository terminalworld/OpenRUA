"""Shared test helpers.

Some tests exercise live containers and need an image built on this
machine (``openrua build sandbox`` / ``openrua build proxy``). They skip
where the image is absent, so the suite stays green on a checkout that
has never built anything (CI included) and still runs the live checks
wherever the images exist.
"""

from __future__ import annotations

import functools
import subprocess

import pytest


@functools.lru_cache(maxsize=None)
def image_present(name: str) -> bool:
    try:
        r = subprocess.run(["docker", "image", "inspect", name],
                           capture_output=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return r.returncode == 0


def requires_image(name: str):
    """Skip the test unless the docker image ``name`` is built."""
    return pytest.mark.skipif(not image_present(name),
                              reason=f"docker image {name} not built here")
