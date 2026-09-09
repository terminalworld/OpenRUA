"""``openrua install [--sim S] [--bench B] [--print]``: build a simulator
install from what its files declare.

The simulator's ``install:`` (and the benchmark's over it) is rendered
to one bash script, printed, saved as ``<home>/simulators/install-<name>.sh``
and run. Rerunning is cheap: every step checks before it acts. ``--print``
stops after printing. Without flags the user's default benchmark, else
default simulator, is installed.
"""

from __future__ import annotations

from openrua import config
from openrua.config import paths
from openrua.errors import UsageError
from openrua.robot.sim import install as installer


def run(args) -> int:
    user = config.load_user_config(paths.config_path(args.home))
    defaults = config.load_user_config(paths.package_config_path())
    bench = args.bench or (None if args.sim else (user.benchmark or defaults.benchmark))
    sim = args.sim
    if bench:
        _, b = config.load_benchmark(bench)
        sim = sim or b.get("simulator")
    sim = sim or user.simulator or defaults.simulator
    if not sim:
        raise UsageError("name what to install: --sim <engine> or --bench <benchmark>",
                         hint="openrua simulators / openrua benchmarks list them; openrua "
                              "config set --sim <name> makes one the default")
    install = config.install_for(sim, bench)
    what = f"{bench} (on {sim})" if bench else sim
    name = paths.find("benchmarks", bench).stem if bench else paths.find("simulators", sim).stem
    root = paths.simulators_dir(args.home)
    script = installer.render(install, what, root, paths.code_root())
    print(script, end="")
    if args.print:
        return 0
    root.mkdir(parents=True, exist_ok=True)
    path = root / f"install-{name}.sh"
    path.write_text(script)
    print(f"[install] saved as {path}; running", flush=True)
    installer.run(path)
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("install", help="build a simulator install (checkouts and venv)",
                       description="Build what a simulator file, and a benchmark's install: "
                       "over it, declare: repositories at pinned commits, a venv at the "
                       "declared Python with the requirements lock, editable checkouts, "
                       "and the package itself. The script is printed, saved under "
                       "<home>/simulators/ and run; rerunning is cheap. Needs uv and git.")
    p.add_argument("--sim", default=None, help="simulator to install (openrua simulators); "
                   "default: the benchmark's, else the user config's")
    p.add_argument("--bench", default=None, help="benchmark whose install to build (openrua "
                   "benchmarks); default: the user config's default benchmark, if any")
    p.add_argument("--print", action="store_true", help="print the script and stop")
    p.set_defaults(fn=run)
