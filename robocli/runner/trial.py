"""One trial: bring-up, reset, preflight, operator, verdict, record, teardown.

Owns the ``runs/<benchmark>/<run-id>/trials/<suite>-<task>/seed<n>/``
layout; what the files say is ``record``'s knowledge (paths passed in).
Blind single-episode protocol: success predicates stay on the robot's
truth side and never reach the agent; reset and initial placement are
invisible on the agent's surface, which removes the basis for
oracle-retry.
"""

from __future__ import annotations

import traceback
import hashlib
import shutil
import subprocess
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

from robocli import agents
from robocli.config import paths
from robocli.proxy.up import ensure as ensure_proxy
from robocli.runner import lock
from robocli.runner import record
from robocli.runner.bringup import bring_up, ensure_internal_network, start_episode
from robocli.runner.preflight import run_preflight
from robocli.sandbox import workspace
from robocli.sandbox.down import down as sandbox_down


DNS_LABEL_MAX = 63  # RFC 1035, single label


def _sandbox_version(sandbox_name: str, agent) -> str | None:
    """The agent CLI's version as the sandbox reports it, or None."""
    if not agent.version_argv:
        return None
    try:
        r = subprocess.run(["docker", "exec", sandbox_name, *agent.version_argv],
                           capture_output=True, text=True, timeout=60)
        return r.stdout.strip() or None
    except (OSError, subprocess.TimeoutExpired):
        return None


def attempt_names(run_dir: Path, task_id: int, seed: int) -> tuple[str, str, str]:
    """The container names of one attempt: ``(stem, sim, sandbox)``.

    Container names double as DNS labels for DDS peer resolution; a
    label over the DNS limit silently fails to resolve and the bridge
    dies at node creation. The budget is derived from the actual
    suffixes so renaming one cannot reopen the hole. The attempt tag
    keeps reruns of one trial (and leftovers of a crashed attempt) from
    colliding on names: names identify attempts, not trial keys."""
    suffixes = ("-sim", "-sandbox")
    attempt_tag = uuid.uuid4().hex[:6]
    stem = f"rc-{run_dir.name}-{task_id}-{seed}"
    budget = (DNS_LABEL_MAX - len(f"-{attempt_tag}")
              - max(len(s) for s in suffixes))
    if len(stem) > budget:
        digest = hashlib.sha256(stem.encode()).hexdigest()[:8]
        stem = f"{stem[:budget - len(digest) - 1]}-{digest}"
    stem = f"{stem}-{attempt_tag}"
    assert all(len(stem + s) <= DNS_LABEL_MAX for s in suffixes)
    return stem, f"{stem}-sim", f"{stem}-sandbox"


def claim(trial_dir: Path, stem: str):
    """Take the trial directory for this attempt, moving a previous
    attempt's artifacts aside. Claim first: a second writer's opening
    act would otherwise pull a running attempt's transcript out from
    under it. Returns the lock to release when the attempt ends."""
    held = lock.acquire(trial_dir, stem)
    if held is None:
        info = lock.holder(trial_dir) or {}
        raise lock.TrialLocked(
            f"{trial_dir}: a live attempt already owns this trial "
            f"(pid {info.get('pid')} on {info.get('host')}); refusing to "
            "write alongside it")
    record.archive_prior_attempt(trial_dir)
    return held


def episode_task(rec: dict, machine, scored: bool, seed: int, task: str | None) -> str:
    """Start the episode and return its task sentence. A simulated robot
    is reset to ``seed`` and asked; a real robot has no truth side, so
    the caller's sentence is the task and the verdict is not
    applicable. Records what identifies the episode's world (a loader
    may fill ``init_state``; robocasa names the kitchen)."""
    if scored:
        info = start_episode(machine, seed)
        language = info.get("language", "")
        if info.get("init_state"):
            rec["init_state"] = info["init_state"]
    else:
        language = task or ""
        rec["verdict"] = "not_applicable"
    rec["task_language"] = language
    if not language.strip():
        raise RuntimeError("empty task: the bridge returned none"
                           if scored else "empty task: a real robot needs --task")
    return language


def operate(cfg: dict, machine, sandbox_name: str, operator, ctx: dict) -> dict:
    """Run the operator under the sim-stall watchdog. The ``success``
    verb runs through the sim thread; if it stops answering, the physics
    owner is wedged: kill the sandbox (stops the agent's quota burn) and
    raise. Timeout sizing: a single trajectory goal executes as one
    sim-thread job and can legitimately run minutes of wall time under
    software rendering; a short timeout would kill healthy trials, a
    long one merely detects wedges later."""
    stall: dict = {}
    wd_stop = threading.Event()
    probe_timeout = float(cfg.get("protocol", {}).get("stall_probe_timeout_s", 600.0))

    def watchdog():
        while not wd_stop.wait(120.0):
            probe: dict = {}

            def ask():
                try:  # rpc timeout == probe timeout: the thread ends
                    # (and releases the rpc lock) the moment the
                    # verdict is due
                    probe["r"] = machine.rpc({"cmd": "success"}, "watchdog",
                                             timeout_s=probe_timeout)
                except Exception:  # noqa: BLE001; timeout = no answer
                    pass

            t = threading.Thread(target=ask, daemon=True)
            t.start()
            t.join(probe_timeout + 10.0)
            if t.is_alive() or "r" not in probe:
                stall["msg"] = ("sim stall: control line unresponsive "
                                f">{probe_timeout:.0f}s (watchdog)")
                subprocess.run(["docker", "kill", sandbox_name], capture_output=True)
                return

    threading.Thread(target=watchdog, daemon=True).start()
    try:
        op_meta = operator(ctx)
    finally:
        wd_stop.set()
    if stall:
        raise RuntimeError(stall["msg"])
    return op_meta


def score(rec: dict, machine) -> None:
    """Blind single episode: one ask, after the operator is done. The
    bridge latches success per step (official any-step-success
    semantics); the end-state verdict is recorded alongside, and the
    closing step count so step-economy statistics exist for failed
    trials too."""
    verdict = machine.rpc({"cmd": "success"})
    rec["success"] = bool(verdict["success"])
    if "success_end_state" in verdict:
        rec["success_end_state"] = bool(verdict["success_end_state"])
    if "success_at" in verdict:  # time-to-success (post-hoc budgets)
        rec["success_at"] = verdict["success_at"]
    # Guarded on its own: auxiliary telemetry must never retract an
    # already-obtained verdict.
    try:
        rec["steps_total"] = machine.rpc({"cmd": "steps"}, timeout_s=60.0).get("steps")
    except Exception:  # noqa: BLE001
        rec["steps_total"] = None
    rec["steps_semantics"] = "episode"  # steps since reset


def run_trial(cfg, cfg_path, run_dir, task_suite, task_id, seed, operator,
              wall_cap_min, ros_domain=0, credentials_dir=None, home=None,
              account_alias=None, script=None, token_file=None, task=None):
    """One trial: claim the directory, bring the robot and sandbox up,
    start the episode, preflight, operate, score, write the record.
    ``task`` is the task sentence for a robot with no truth side (a real
    robot); a simulated robot's task comes from its bridge."""
    scored = cfg.get("machine", {}).get("backend", {}).get("kind") == "sim"
    trial_dir = run_dir / "trials" / f"{task_suite}-{task_id}" / f"seed{seed}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    stem, sim_name, sandbox_name = attempt_names(run_dir, task_id, seed)
    held = claim(trial_dir, stem)
    network = ensure_internal_network()

    t0 = time.time()
    rec: dict = {
        "task_suite": task_suite,
        "task_id": task_id,
        "init_state_id": seed,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        # Which workspace the agent actually got. The run-level
        # config.json carries this too, but only a trial-level stamp can
        # tell trials apart after a mid-campaign template edit.
        "workspace_template_sha256": workspace.template_hash(cfg),
        # Names identify attempts (uuid tag): the handle that joins this
        # record to docker events and logs when a container dies
        # out-of-band.
        "containers": {"sim": sim_name, "sandbox": sandbox_name},
        "ros_domain": ros_domain,
    }
    proxy_url = ensure_proxy(network)
    if account_alias:
        rec["account_alias"] = account_alias
    agent = agents.get(cfg.get("agent", {}).get("name"), home,
                       version=cfg.get("agent", {}).get("version"))
    creds_home = Path(
        credentials_dir
        or cfg.get("agent", {}).get("credentials_dir")
        or paths.credentials_dir(home) / agent.name
    ).expanduser()
    # A token file is the sandbox's whole auth story, so the login
    # profile need not carry credentials then (see agents.prepare_profile).
    cfg_dir, creds_file = agents.prepare_profile(
        creds_home, agent, require_credentials=token_file is None)
    secrets = record.secret_strings(creds_home)  # pre-trial token values
    if token_file:
        # The token never reaches the record: it is as much a secret as
        # anything in the credentials file, and the agent can print its
        # own environment.
        secrets += record.secret_strings_from_env_file(Path(token_file))
    # Container construction inside the try: a failure after the sandbox
    # is up must still write result.json and tear the sandbox down, not
    # orphan it and kill the rest of the invocation's queue.
    sandbox_live = False
    machine = None
    try:
        # The trial's resolved suite view: computed once (main applied
        # the overrides), written as an artifact by bring_up and read by
        # every party below. bring_up tears its own sandbox down if the
        # robot fails, so sandbox_live flips only on success.
        config_path, machine = bring_up(
            cfg, trial_dir, sim_name, sandbox_name, task_suite, task_id,
            network, proxy_url, agent.sandbox_mounts(cfg_dir, creds_file),
            ros_domain, robot_log=trial_dir / "bridge.log", home=home)
        sandbox_live = True
        # The CLI version that actually ran: read inside the sandbox.
        rec["agent_version"] = _sandbox_version(sandbox_name, agent)
        task_language = episode_task(rec, machine, scored, seed, task)
        # Preflight: every verifiable promise the workspace docs make,
        # asserted from the sandbox before the agent starts. Red refuses
        # the trial as an anomaly; the agent never pays for a wrong
        # description.
        conf = run_preflight(cfg, sandbox_name, agent)
        rec["preflight"] = conf["checks"]
        if not conf["ok"]:
            raise RuntimeError(f"preflight failed: {','.join(conf['failed'])}")
        op_meta = operate(cfg, machine, sandbox_name, operator, {
            "machine": machine,
            "trial_dir": trial_dir,
            "cfg": cfg,
            "sandbox": sandbox_name,
            "sim": sim_name,
            "task_language": task_language,
            "active_wall_clock_min": wall_cap_min,
            "proxy": proxy_url,
            "script": script,
            "token_file": token_file,
            "home": home,
        })
        if scored:
            score(rec, machine)
        else:
            rec["success"] = None
            rec["steps_total"] = None
        rec["operator_meta"] = op_meta
        rec["termination"] = op_meta.get("termination", "operator_done")
        # Validity judgment (quota voiding and the like) belongs to
        # whoever reads the artifacts; the runner records faithfully and
        # flags only its own failures below.
        rec["anomaly"] = None
    except Exception as exc:  # noqa: BLE001; anomaly ledger, not a crash
        rec["success"] = False
        rec["termination"] = "anomaly"
        rec["anomaly"] = f"{type(exc).__name__}: {exc}"
        # The one-liner names the failure; the stack finds it. Host-side
        # errors have no bridge.log to fall back on.
        rec["anomaly_traceback"] = traceback.format_exc()
    finally:
        if machine is not None:
            machine.shutdown()
        if sandbox_live:
            sandbox_down(sandbox_name)
        # Post-trial token values too; a mid-trial rotation would leave
        # both generations potentially visible in the record.
        secrets += record.secret_strings(creds_home)
        shutil.rmtree(cfg_dir, ignore_errors=True)
        record.finalize_trial(trial_dir, agent, secrets)
        held.release()
    # wall_seconds spans the whole harness (boot, preflight, operator,
    # teardown). The wall cap is enforced only on the operator and its
    # verdict already sits in rec["termination"]; it is never rewritten
    # from the harness span.
    rec["wall_seconds"] = round(time.time() - t0, 1)
    # Named for the semantics, so a result file says which budget it ran
    # under.
    rec["active_wall_cap_min"] = wall_cap_min
    record.write_result(trial_dir, rec)
    return rec
