"""The declared install: bundled declarations are complete and point at
files that ship; the renderer turns one into a Dockerfile plus a build
context; the fingerprint pins the declaration; image ownership
deduplicates benchmarks that add nothing to their simulator."""
from pathlib import Path

import pytest

from openrua import config
from openrua.config import paths
from openrua.config.schema import sim_image
from openrua.robot.sim import install as installer


def test_every_bundled_benchmark_declares_a_complete_install():
    for e in paths.available("benchmarks"):
        _, b = config.load_benchmark(e.name)
        inst = config.install_for(b["simulator"], e.name)
        assert inst["python"] and inst["requirements"], e.name  # checkouts may be none: a pip-only engine
        assert Path(inst["requirements"]).is_file(), e.name        # ships in the package
        for c in inst["checkouts"]:
            assert len(c["commit"]) == 40, (e.name, c)
            if c.get("patch"):
                assert Path(c["patch"]).is_file(), (e.name, c["patch"])
        if "{here}" in (inst.get("shell") or ""):
            assert Path(inst["here"]).is_dir() and Path(inst["here"]).name == e.name
        dockerfile, files = installer.render(inst, e.name, paths.code_root())
        assert dockerfile.startswith("# syntax=docker/dockerfile:1\n")
        for key, host in files.items():
            assert host.exists(), (e.name, key)


def test_a_benchmark_writes_over_the_simulator_key_by_key():
    sim = config.install_for("robosuite")
    assert sim["python"] == "3.10" and sim["ros_distro"] == "humble"
    lib = config.install_for("robosuite", "libero_pro")
    assert lib["python"] == "3.12" and lib["ros_distro"] == "jazzy"
    assert lib["checkouts"][0]["path"] == "cap-x"                 # the same checkout, declared again
    cap = config.install_for("robosuite", "capbench")
    assert cap == sim                                             # nothing written: the simulator's


def test_image_ownership_deduplicates_benchmarks_that_add_nothing():
    assert config.image_owner("robosuite") == "robosuite"
    assert config.image_owner("robosuite", "capbench") == "robosuite"
    assert config.image_owner("robosuite", "libero_pro") == "libero_pro"
    assert config.image_owner("maniskill", "maniskill") == "maniskill"
    assert config.image_owner("maniskill", "simpler") == "simpler"   # its own shell tail
    assert sim_image("libero_pro") == "openrua-sim-libero_pro"


def test_relative_files_resolve_next_to_the_yaml_that_names_them(tmp_path):
    (tmp_path / "lock.txt").write_text("numpy\n")
    y = tmp_path / "mine.yaml"
    y.write_text("entry_point: libero\ntask: {benchmark: mine, suites: [s]}\n"
                 "robot: panda\nsimulator: robosuite\n"
                 "install: {python: '3.12', requirements: lock.txt,\n"
                 "  shell: 'cat {here}/lock.txt'}\n")
    inst = config.install_for("robosuite", str(y))
    assert inst["requirements"] == str(tmp_path / "lock.txt")
    assert inst["here"] == str(tmp_path / "mine") and "{here}" in inst["shell"]
    assert inst["checkouts"][0]["patch"].endswith("capx-pyproject.patch")   # the simulator's, absolute
    assert config.image_owner("robosuite", str(y)) == "mine"


def test_render_is_a_dockerfile_over_the_base_with_its_files_in_context(tmp_path):
    (tmp_path / "req.txt").write_text("numpy\n")
    (tmp_path / "p.patch").write_text("--- a\n+++ b\n")
    (tmp_path / "here").mkdir()
    (tmp_path / "here" / "suites.tar.gz").write_bytes(b"x")
    inst = {"python": "3.12", "ros_distro": "jazzy",
            "checkouts": [{"path": "cap-x", "repo": "https://x/y.git", "commit": "a" * 40,
                           "submodules": ["sub/one"], "patch": str(tmp_path / "p.patch")}],
            "requirements": str(tmp_path / "req.txt"), "editable": ["cap-x"],
            "shell": "run tar xzf {here}/suites.tar.gz -C {root}/cap-x && {venv}/bin/python -V",
            "here": str(tmp_path / "here")}
    d, files = installer.render(inst, "mine", tmp_path)   # tmp_path: no pyproject
    assert d.splitlines()[2] == "FROM openrua-sim-base-jazzy"
    assert f"WORKDIR {installer.ROOT}" in d
    assert "3.12" in d and "exit 1" in d                        # the Python check fails loudly
    assert f"uv venv -q {installer.VENV} --python /usr/bin/python3" in d
    assert f"git clone -q https://x/y.git cap-x && git -C cap-x checkout -q {'a' * 40}" in d
    assert "submodule update -q --init sub/one" in d
    assert "COPY files/checkout0/p.patch" in d and "git -C cap-x apply" in d
    assert "COPY files/requirements/req.txt" in d and "-r /opt/openrua/build/files/requirements/req.txt" in d
    assert "editable_mode=compat -e cap-x" in d
    assert f"openrua=={config.__version__}" in d if hasattr(config, "__version__") else "openrua==" in d
    assert "COPY here /opt/openrua/build/here" in d
    assert f"tar xzf /opt/openrua/build/here/suites.tar.gz -C {installer.ROOT}/cap-x" in d
    assert f"{installer.VENV}/bin/python -V" in d and "run()" in d
    assert files == {"files/checkout0/p.patch": tmp_path / "p.patch",
                     "files/requirements/req.txt": tmp_path / "req.txt",
                     "here": tmp_path / "here"}
    ctx = installer.context(d, files, tmp_path / "ctx")
    assert (ctx / "Dockerfile").read_text() == d
    assert (ctx / "files" / "requirements" / "req.txt").read_text() == "numpy\n"
    assert (ctx / "here" / "suites.tar.gz").exists()
    # a checkout of the package goes in as source
    (tmp_path / "pyproject.toml").write_text("[project]\nname='x'\n")
    (tmp_path / "openrua").mkdir()
    d2, files2 = installer.render(inst, "mine", tmp_path)
    assert f"COPY src {installer.FILES}/src" in d2 and files2["src/openrua"] == tmp_path / "openrua"
    with pytest.raises(ValueError, match="python"):
        installer.render({"ros_distro": "jazzy"}, "mine", tmp_path)


def test_fingerprint_follows_the_declaration_and_its_files_not_the_code(tmp_path):
    (tmp_path / "req.txt").write_text("numpy\n")
    inst = {"python": "3.12", "requirements": str(tmp_path / "req.txt")}
    d, files = installer.render(inst, "mine", tmp_path)
    one = installer.fingerprint(d, files)
    assert one == installer.fingerprint(d, files)
    (tmp_path / "req.txt").write_text("numpy==2\n")             # the lock changed
    assert installer.fingerprint(d, files) != one
    files_with_src = {**files, "src/openrua": tmp_path}          # code is not part of it
    assert installer.fingerprint(d, files_with_src) == installer.fingerprint(d, files)
