"""Workspace reads remain bounded and cannot follow paths outside their root."""

import base64
import os

import pytest

from openrua.artifacts import WorkspaceFiles


def test_images_text_and_binary_have_explicit_preview_types(tmp_path):
    root = tmp_path / "workspace"
    root.mkdir()
    (root / "scripts").mkdir()
    (root / "scripts" / "check.py").write_text('print("<script>not executed</script>")')
    (root / "camera.png").write_bytes(b"\x89PNG\r\n\x1a\nimage")
    (root / "array.npy").write_bytes(b"\x93NUMPY\x00data")
    (root / "plot.svg").write_text('<svg onload="alert(1)"></svg>')
    reader = WorkspaceFiles(root)
    assert reader.list()["entries"][0]["name"] == "scripts"
    assert reader.read("scripts/check.py")["kind"] == "text"
    image = reader.read("camera.png")
    assert image["mime"] == "image/png" and image["kind"] == "image"
    assert base64.b64decode(image["data"]) == (root / "camera.png").read_bytes()
    assert "+00:00" in image["modified"]
    assert reader.read("array.npy")["kind"] == "binary"
    assert reader.read("plot.svg")["mime"] == "text/plain"


@pytest.mark.parametrize("path", ["../secret", "/etc/passwd", "a/../../secret", "a//b", ".", "a/./b", "a\0b"])
def test_traversal_and_ambiguous_paths_are_rejected(tmp_path, path):
    with pytest.raises(ValueError):
        WorkspaceFiles(tmp_path).read(path)


def test_symlinks_and_special_files_are_not_read(tmp_path):
    root = tmp_path / "workspace"
    root.mkdir()
    outside = tmp_path / "secret"
    outside.write_text("private")
    (root / "link").symlink_to(outside)
    (root / "directory-link").symlink_to(tmp_path, target_is_directory=True)
    os.mkfifo(root / "pipe")
    reader = WorkspaceFiles(root)
    for name in ("link", "directory-link/secret", "pipe"):
        with pytest.raises((OSError, ValueError)):
            reader.read(name)
    assert all(entry["kind"] == "unsupported" for entry in reader.list()["entries"])
    with pytest.raises(ValueError):
        reader.read("")


def test_swapping_directory_for_symlink_during_open_does_not_escape(tmp_path, monkeypatch):
    root = tmp_path / "workspace"
    root.mkdir()
    folder = root / "folder"
    folder.mkdir()
    (folder / "data").write_text("inside")
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "data").write_text("private")
    reader = WorkspaceFiles(root)
    original = os.open
    def swapped(path, flags, **kwargs):
        fd = original(path, flags, **kwargs)
        if path == "folder":
            folder.rename(root / "old")
            folder.symlink_to(outside, target_is_directory=True)
        return fd
    monkeypatch.setattr(os, "open", swapped)
    assert reader.read("folder/data")["text"] == "inside"


def test_limits_and_file_modified_during_read(tmp_path, monkeypatch):
    file = tmp_path / "data"
    file.write_bytes(b"12345")
    reader = WorkspaceFiles(tmp_path, max_bytes=4, max_entries=1)
    with pytest.raises(ValueError, match="limit"):
        reader.read("data")
    (tmp_path / "second").touch()
    assert reader.list()["truncated"]
    assert len(reader.list()["entries"]) == 1
    file.write_bytes(b"123")
    original = os.fstat
    calls = 0
    def changed(fd):
        nonlocal calls
        calls += 1
        if calls == 2:
            file.write_bytes(b"4567")
        return original(fd)
    monkeypatch.setattr(os, "fstat", changed)
    with pytest.raises(ValueError, match="changed"):
        reader.read("data")
