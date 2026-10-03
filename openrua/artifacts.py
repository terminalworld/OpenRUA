"""Read-only workspace artifacts, independent of sessions and agent vendors."""

from __future__ import annotations

import base64
from contextlib import contextmanager
from datetime import datetime, timezone
import os
from pathlib import Path
import stat
from typing import Protocol


class ArtifactReader(Protocol):
    """List a relative directory or read a relative file as JSON-compatible data."""

    def list(self, path: str) -> dict: ...
    def read(self, path: str) -> dict: ...


class WorkspaceFiles:
    """Bounded reads under one explicit root; symlinks are never followed.

    Each component is opened relative to a directory descriptor, so replacing a
    checked path with a symlink cannot redirect a later read outside the root.
    Files are live workspace artifacts, not immutable observation snapshots.
    """

    def __init__(self, root: Path, max_bytes: int = 16 * 1024 * 1024,
                 max_entries: int = 1000):
        if max_bytes <= 0 or max_entries <= 0:
            raise ValueError("positive artifact limits are required")
        self.root = root.resolve(strict=True)
        if not self.root.is_dir():
            raise ValueError("workspace root must be a directory")
        self.max_bytes, self.max_entries = max_bytes, max_entries

    @contextmanager
    def _open(self, path: str):
        if not isinstance(path, str) or "\0" in path or path.startswith("/"):
            raise ValueError("use a relative workspace path")
        parts = path.split("/") if path else []
        if any(part in ("", ".", "..") for part in parts):
            raise ValueError("use a relative workspace path without . or .. components")
        # Open canonical root components without following newly installed links.
        parts = list(self.root.parts[1:]) + parts
        fd = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
        try:
            for index, part in enumerate(parts):
                flags = os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC
                if index < len(parts) - 1:
                    flags |= os.O_DIRECTORY
                child = os.open(part, flags, dir_fd=fd)
                os.close(fd)
                fd = child
            yield fd
        finally:
            os.close(fd)

    @staticmethod
    def _metadata(info):
        return {"size": info.st_size,
                "modified": datetime.fromtimestamp(info.st_mtime, timezone.utc).isoformat()}

    def list(self, path: str = "") -> dict:
        entries = []
        truncated = False
        with self._open(path) as fd:
            if not stat.S_ISDIR(os.fstat(fd).st_mode):
                raise ValueError("select a directory to list its contents")
            with os.scandir(fd) as scan:
                for index, entry in enumerate(scan):
                    if index >= self.max_entries:
                        truncated = True
                        break
                    try:
                        entry.name.encode("utf-8")
                        info = entry.stat(follow_symlinks=False)
                    except (FileNotFoundError, UnicodeError):
                        continue
                    kind = ("directory" if stat.S_ISDIR(info.st_mode) else
                            "file" if stat.S_ISREG(info.st_mode) else "unsupported")
                    entries.append({"name": entry.name, "kind": kind, **self._metadata(info)})
        entries.sort(key=lambda item: (item["kind"] != "directory", item["name"]))
        return {"path": path, "entries": entries, "truncated": truncated}

    def read(self, path: str) -> dict:
        with self._open(path) as fd:
            before = os.fstat(fd)
            if not stat.S_ISREG(before.st_mode):
                raise ValueError("only regular workspace files can be read")
            if before.st_size > self.max_bytes:
                raise ValueError(f"file exceeds the {self.max_bytes}-byte preview/download limit; inspect it in the terminal")
            with os.fdopen(os.dup(fd), "rb") as stream:
                raw = stream.read(self.max_bytes + 1)
            after = os.fstat(fd)
            if len(raw) > self.max_bytes:
                raise ValueError("file grew beyond the preview/download limit; inspect it in the terminal")
            if (before.st_size, before.st_mtime_ns, before.st_ctime_ns) != (after.st_size, after.st_mtime_ns, after.st_ctime_ns):
                raise ValueError("file changed while being read; refresh to try again")
        mime = "application/octet-stream"
        if raw.startswith(b"\x89PNG\r\n\x1a\n"):
            mime = "image/png"
        elif raw.startswith(b"\xff\xd8\xff"):
            mime = "image/jpeg"
        elif raw.startswith((b"GIF87a", b"GIF89a")):
            mime = "image/gif"
        elif raw.startswith(b"RIFF") and raw[8:12] == b"WEBP":
            mime = "image/webp"
        result = {"path": path, **self._metadata(after), "mime": mime,
                  "data": base64.b64encode(raw).decode("ascii"), "kind": "binary"}
        if mime.startswith("image/"):
            result["kind"] = "image"
        else:
            try:
                text = raw.decode("utf-8")
            except UnicodeError:
                pass
            else:
                if "\0" not in text:
                    result.update(kind="text", mime="text/plain", text=text)
        return result
