"""A simulator install, rendered into an image.

A simulator file (and a benchmark's ``install:`` over it) declares what
the simulated robot's environment is made of: repositories at pinned
commits, patches, a requirements lock, editable checkouts, and a shell
tail for what those cannot say (asset downloads). ``render`` turns that
dict into a Dockerfile on top of the ROS base image; ``context`` lays
the files it references next to it; ``openrua build`` builds the result
as ``openrua-sim-<name>``. Everything the bridge needs is then inside
the image: the checkouts under ``ROOT``, one venv at ``VENV`` on the
image's own Python, and this package. Nothing is mounted at run time
but the trial directory.

``fingerprint`` hashes the rendered Dockerfile together with the files
it copies in; the image carries it as a label, so doctor can tell an
image built from an older declaration without rebuilding anything.
The image's name is the composition's business (config.schema.sim_image);
here it is only a tag handed in.
"""

from __future__ import annotations

import hashlib
import shlex
import shutil
from pathlib import Path

from openrua import __version__

ROOT = "/opt/openrua/simulators"   # checkouts, inside the image
VENV = f"{ROOT}/.venv"             # the one venv the bridge runs in, next to the checkouts:
                                   # a loader finds its checkout at Path(sys.prefix).parent / <path>
FILES = "/opt/openrua/build"       # where the context's files land
PYTHON = f"{VENV}/bin/python"
BASE_IMAGE = "openrua-sim-base"    # + "-<distro>": openrua build base


def base_image(distro: str) -> str:
    return f"{BASE_IMAGE}-{distro}"


def render(install: dict, name: str, code_root: Path) -> tuple[str, dict[str, Path]]:
    """The Dockerfile for one resolved ``install:`` dict, and the files
    it needs in its build context (``{context path: host path}``).
    ``name`` names the declaration in comments; ``code_root`` is the
    directory holding the installed package (a checkout goes into the
    image as source; otherwise the release at this version comes from
    PyPI)."""
    python = install.get("python")
    if not python:
        raise ValueError(f"{name}: install.python is not declared; the image's Python "
                         "must be named (3.12 for jazzy, 3.10 for humble)")
    distro = install.get("ros_distro", "jazzy")
    files: dict[str, Path] = {}

    def stage(host: str | Path, sub: str) -> str:
        """Put a host file into the context under ``files/<sub>/...`` and
        return its path inside the image."""
        host = Path(host)
        key = f"files/{sub}/{host.name}"
        files[key] = host
        return f"{FILES}/{key}"

    lines = [
        "# syntax=docker/dockerfile:1",
        f"# Rendered by openrua build from the {name} declaration.",
        f"FROM {base_image(distro)}",
        f"WORKDIR {ROOT}",
        # Loud failure when the image's Python is not the declared one:
        # the lock was frozen for that version.
        f'RUN test "$(python3 -c \'import sys; print("%d.%d" % sys.version_info[:2])\')" '
        f'= {shlex.quote(str(python))} || {{ echo "the {distro} image runs $(python3 --version), '
        f'the {name} install declares Python {python}" >&2; exit 1; }}',
    ]
    # Checkouts first: they change least, so a change below (the lock, the
    # package) rebuilds from the venv layer, not from the clones.
    for i, c in enumerate(install.get("checkouts") or []):
        dst = c["path"]
        clone = (f"git clone -q {shlex.quote(c['repo'])} {shlex.quote(dst)} && "
                 f"git -C {shlex.quote(dst)} checkout -q {shlex.quote(c['commit'])}")
        if c.get("submodules"):
            subs = " ".join(shlex.quote(x) for x in c["submodules"])
            clone += f" && git -C {shlex.quote(dst)} submodule update -q --init {subs}"
        lines.append(f"RUN {clone}")
        if c.get("patch"):
            p = stage(c["patch"], f"checkout{i}")
            lines.append(f"COPY {_ctx(p)} {p}")
            lines.append(f"RUN git -C {shlex.quote(dst)} apply {p}")
    lines.append(f"RUN UV_PYTHON_DOWNLOADS=never uv venv -q {VENV} --python /usr/bin/python3")
    cache = "--mount=type=cache,target=/root/.cache/uv"
    if install.get("requirements"):
        p = stage(install["requirements"], "requirements")
        lines.append(f"COPY {_ctx(p)} {p}")
        lines.append(f"RUN {cache} uv pip install -q --python {PYTHON} -r {p}")
    editables = [f"-e {shlex.quote(e)}" for e in install.get("editable") or []]
    if editables:
        # compat mode: a path line in a .pth, not setuptools' import hook,
        # which loses packages whose top-level directory holds a
        # same-named subpackage (the LIBERO forks: libero/libero).
        lines.append(f"RUN {cache} CUDA_HOME=/usr uv pip install -q --python {PYTHON} "
                     "--no-deps --config-setting editable_mode=compat " + " ".join(editables))
    if install.get("shell"):
        tail = install["shell"]
        if "{here}" in tail:
            here = Path(install["here"])
            files["here"] = here
            tail = tail.replace("{here}", f"{FILES}/here")
            lines.append(f"COPY here {FILES}/here")
        tail = tail.replace("{root}", ROOT).replace("{venv}", VENV)
        lines.append("# the declaration's shell tail")
        lines.append(f"RUN bash <<'{_HEREDOC}'\n{_PRELUDE}{tail.strip(chr(10))}\n{_HEREDOC}")
    # This package last: it changes most often, and nothing above needs it.
    if (code_root / "pyproject.toml").is_file():
        for item in ("pyproject.toml", "README.md", "LICENSE", "openrua"):
            if (code_root / item).exists():
                files[f"src/{item}"] = code_root / item
        lines.append(f"COPY src {FILES}/src")
        lines.append(f"RUN {cache} uv pip install -q --python {PYTHON} --no-deps {FILES}/src")
    else:
        lines.append(f"RUN {cache} uv pip install -q --python {PYTHON} --no-deps "
                     f"openrua=={__version__}")
    lines.append(f"ENV OPENRUA_SIMULATORS={ROOT} PATH={VENV}/bin:$PATH")
    return "\n".join(lines) + "\n", files


def _ctx(image_path: str) -> str:
    """The context path of a file staged under FILES."""
    return image_path[len(FILES) + 1:]


# A shell tail runs whole as one layer, fed to bash by a Dockerfile
# heredoc (BuildKit syntax; podman's buildah reads it too), so multi-line
# constructs (if/fi, functions) survive. What it may call: ``note``
# prints a line, ``run`` prints the command then runs it.
_HEREDOC = "OPENRUA_SHELL"
_PRELUDE = ("set -euo pipefail\n"
            "note() { printf '    %s\\n' \"$*\"; }\n"
            "run()  { printf '    $ %s\\n' \"$*\"; \"$@\"; }\n")


def context(dockerfile: str, files: dict[str, Path], dest: Path) -> Path:
    """Lay a build context out under ``dest``: the Dockerfile and every
    referenced file or directory at its context path."""
    dest.mkdir(parents=True, exist_ok=True)
    (dest / "Dockerfile").write_text(dockerfile)
    for key, host in files.items():
        target = dest / key
        target.parent.mkdir(parents=True, exist_ok=True)
        if host.is_dir():
            shutil.copytree(host, target, dirs_exist_ok=True,
                            ignore=shutil.ignore_patterns("__pycache__", ".venv*", "*.pyc"))
        else:
            shutil.copyfile(host, target)
    return dest


def fingerprint(dockerfile: str, files: dict[str, Path]) -> str:
    """sha256 over the Dockerfile and the declaration's own files (locks,
    patches, the ``here`` directory); the package source is left out, so
    the label says whether the declaration changed, not the code."""
    h = hashlib.sha256(dockerfile.encode())
    for key in sorted(files):
        if key.startswith("src/"):
            continue
        host = files[key]
        for f in sorted(host.rglob("*")) if host.is_dir() else [host]:
            if f.is_file():
                h.update(str(f.relative_to(host) if host.is_dir() else f.name).encode())
                h.update(f.read_bytes())
    return h.hexdigest()
