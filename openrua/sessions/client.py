"""Local session API client, shared by command-line front ends."""

from __future__ import annotations

import json
import os
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, Request, build_opener


class Client:
    def __init__(self, url: str, token: str, timeout: float = 130):
        target = urlsplit(url)
        if target.scheme != "http" or target.hostname != "127.0.0.1" or not target.port or target.path or target.query or target.fragment or target.username:
            raise ValueError("session endpoint must be a local HTTP origin")
        self.url, self.token, self.timeout = url, token, timeout
        # Local session credentials must never be sent through an HTTP proxy.
        self._opener = build_opener(ProxyHandler({}))

    @classmethod
    def from_file(cls, path: Path) -> Client:
        facts = json.loads(path.read_text())
        return cls(facts["url"], facts["token"])

    def request(self, path: str, body: dict | None = None):
        req = Request(self.url + path, data=json.dumps(body).encode() if body is not None else None,
                      headers={"Authorization": "Bearer " + self.token, "Content-Type": "application/json"})
        try:
            with self._opener.open(req, timeout=self.timeout) as response:
                return json.load(response)
        except HTTPError as exc:
            try:
                message = json.load(exc).get("error", str(exc))
            except (ValueError, AttributeError):
                message = str(exc)
            raise RuntimeError(message) from exc
        except (URLError, TimeoutError) as exc:
            raise RuntimeError("session service unavailable or result unknown; inspect its state before retrying a command") from exc

    def snapshot(self) -> dict:
        return self.request("/api/session")

    def events(self, after: int = 0, limit: int = 1000) -> list[dict]:
        return self.request(f"/api/events?after={after}&limit={limit}")

    def command(self, operation: str, **params):
        return self.request("/api/commands", {"operation": operation, "params": params})["result"]

    def end(self) -> None:
        self.request("/api/end", {})


def write_endpoint(path: Path, url: str, token: str) -> None:
    """Publish connection facts privately; do not print or store tokens in URLs."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as stream:
        json.dump({"url": url, "token": token}, stream)
