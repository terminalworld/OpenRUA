"""One trial: bring-up, reset, preflight, operator, verdict, record, teardown.

Owns the ``runs/<benchmark>/<run-id>/trials/<suite>-<task>/seed<n>/``
layout; what the files say is ``record``'s knowledge (paths passed in).
Blind single-episode protocol: success predicates stay on the robot's
truth side and never reach the agent; reset and initial placement are
invisible on the agent's surface.
"""

from __future__ import annotations

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
from robocli.runner import lock as triallock
from robocli.runner import record
from robocli.runner.bringup import bring_up, ensure_internal_network
from robocli.runner.preflight import run_preflight
from robocli.sandbox import workspace
from robocli.sandbox.down import down as sandbox_down


def run_trial(cfg, cfg_path, run_dir, task_suite, task_id, seed, operator,
              wall_cap_min, ros_domain=0, credentials_dir=None, home=None,
              account_alias=None, script=None, token_file=None):
    trial_dir = run_dir / "trials" / f"{task_suite}-{task_id}" / f"seed{seed}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    # Container names double as DNS labels for DDS peer resolution; a
    # label over the DNS limit silently fails to resolve and the machine
    # dies at node creation (batch-1 incident: run-id + suite repeated =
    # 70 chars, 60/60 trials down). Budget is DERIVED from the actual
    # suffixes so renaming one can never silently reopen the hole.
    # Attempt-unique suffix: reruns of the same trial (and any stale
    # leftovers from a crashed attempt) must never collide on names;
    # names identify ATTEMPTS, not trial keys (2026-08-10 orphan incident).
    # Computed before the directory is claimed, because the claim records
    # this exact stem: after truncation it is the only string that finds
    # this attempt's containers.
    DNS_LABEL_MAX = 63  # RFC 1035, single label
    suffixes = ("-sim", "-sandbox")
    attempt_tag = uuid.uuid4().hex[:6]
    stem = f"rc-{run_dir.name}-{task_id}-{seed}"
    budget = (DNS_LABEL_MAX - len(f"-{attempt_tag}")
              - max(len(s) for s in suffixes))
    if len(stem) > budget:
        digest = hashlib.sha256(stem.encode()).hexdigest()[:8]
        stem = f"{stem[:budget - len(digest) - 1]}-{digest}"
    stem = f"{stem}-{attempt_tag}"
    # Claim the directory BEFORE touching it: the archive step below moves
    # every existing artifact aside, so a second writer's opening act would
    # pull a running attempt's transcript out from under it.
    lock = triallock.acquire(trial_dir, stem)
    if lock is None:
        info = triallock.holder(trial_dir) or {}
        raise triallock.TrialLocked(
            f"{trial_dir}: a live attempt already owns this trial "
            f"(pid {info.get('pid')} on {info.get('host')}); refusing to "
            "write alongside it")
    record.archive_prior_attempt(trial_dir)
    sim_name, sandbox_name = f"{stem}-sim", f"{stem}-sandbox"
    assert all(len(stem + s) <= DNS_LABEL_MAX for s in suffixes)
    network = ensure_internal_network()

    t0 = time.time()
    rec: dict = {
        "task_suite": task_suite,
        "task_id": task_id,
        "init_state_id": seed,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        # Which workspace the occupant actually got. config.json carries
        # this too, but the runner rewrites config.json every trial, so
        # after a mid-campaign template edit only the trial-level stamp
        # can still tell the two halves apart (2026-08-21: the gripper
        # manifest edit landed mid-run and had nothing to be audited by).
        "workspace_template_sha256": workspace.template_hash(cfg),
        # Names identify attempts (uuid tag): the handle that joins this
        # record to docker events/logs when a body dies out-of-band.
        "containers": {"sim": sim_name, "sandbox": sandbox_name},
        "ros_domain": ros_domain,
    }
    proxy_url = ensure_proxy(network)
    if account_alias:
        rec["account_alias"] = account_alias
    agent = agents.get(cfg.get("agent", {}).get("name"), home)
    creds_home = Path(
        credentials_dir
        or cfg.get("agent", {}).get("credentials_dir")
        or paths.credentials_dir(home) / agent.name
    ).expanduser()
    # A token file is the sandbox's whole auth story, so the login profile
    # is no longer required to carry credentials (see agents.prepare_profile).
    cfg_dir, creds_file = agents.prepare_profile(
        creds_home, agent, require_credentials=token_file is None)
    secrets = record.secret_strings(creds_home)  # pre-trial token values
    if token_file:
        # The minted token never reaches the record: it is as much a secret
        # as anything in the credentials file, and the agent can print its
        # own environment.
        secrets += record.secret_strings_from_env_file(Path(token_file))
    # Container construction INSIDE the try (audit 2026-08-14 F15): a
    # machine-constructor exception after the sandbox is up must still
    # write result.json (anomaly) and tear the sandbox down, not orphan it
    # and kill the rest of the invocation's queue.
    sandbox_live = False
    machine = None
    try:
        # The trial's resolved suite view: computed once (main() already
        # applied the overrides), written as an artifact by bring_up and
        # consumed by every party below. bring_up tears its own sandbox
        # down if the body fails, so sandbox_live flips only on success.
        config_path, machine = bring_up(
            cfg, trial_dir, sim_name, sandbox_name, task_suite, task_id,
            network, proxy_url, agent.sandbox_mounts(cfg_dir, creds_file),
            ros_domain, robot_log=trial_dir / "bridge.log", home=home)
        sandbox_live = True
        r = machine.rpc({"cmd": "reset", "init_state_id": seed})
        if not r.get("ok"):
            raise RuntimeError(f"reset failed: {r}")
        info = machine.rpc({"cmd": "task_info"})
        task_language = info.get("language", "")
        rec["task_language"] = task_language
        # Optional generic slot: whatever the loader considers the facts
        # that identify THIS episode's world (robocasa fills the kitchen
        # it ran in). Recorded verbatim; the conductor does not interpret
        # it, and a loader that fills nothing costs nothing.
        if info.get("init_state"):
            rec["init_state"] = info["init_state"]
        if not task_language.strip():
            # An empty mission is never a valid trial (2026-08-11 canary:
            # a task_info bug fed agents "" and they surveyed for 30 min).
            raise RuntimeError("task_info returned empty language")

        # Machine-manual preflight (A6): every verifiable manual promise
        # asserted from the sandbox's vantage BEFORE the agent boards.
        # Red -> refuse trial -> anomaly -> classify INFRA rerun; the
        # agent never pays for a lying manual. Examiner's knowledge;
        # the adapter is handed over (preflight looks nothing up).
        conf = run_preflight(cfg, sandbox_name, agent)
        rec["preflight"] = conf["checks"]
        if not conf["ok"]:
            raise RuntimeError(
                f"preflight failed: {','.join(conf['failed'])}")

        # Sim-stall watchdog (2026-08-12 composite canaries: two trials
        # spent 20+ min against a wedged physics thread, recorded as agent
        # failures with anomaly=null). The "success" verb runs THROUGH the
        # sim thread; if it stops answering, the physics owner is wedged:
        # kill the sandbox (frees the agent's quota burn), flag the trial as
        # an anomaly so classify sends it back for a rerun.
        #
        # Timeout sizing: a single trajectory goal executes as ONE sim-
        # thread job and can legitimately run minutes of wall time (20s of
        # sim = ~400 ticks x ~0.2s under llvmpipe whole-room rendering);
        # a short timeout would KILL healthy trials, which poisons data;
        # a long one merely detects wedges later. 10min clears any
        # plausible single job; a real wedge still surfaces ~12min in
        # (vs the 20-25min agents burned probing dead sims unaided).
        stall: dict = {}
        wd_stop = threading.Event()
        probe_timeout = float(cfg.get("protocol", {})
                              .get("stall_probe_timeout_s", 600.0))

        def _stall_watchdog():
            while not wd_stop.wait(120.0):
                probe: dict = {}

                def _probe():
                    try:  # rpc timeout == probe timeout: the thread ends
                        # (and releases the rpc lock) the moment the
                        # verdict is due, instead of squatting on the pipe
                        probe["r"] = machine.rpc({"cmd": "success"},
                                                 "watchdog",
                                                 timeout_s=probe_timeout)
                    except Exception:  # noqa: BLE001; timeout = no answer
                        pass

                t = threading.Thread(target=_probe, daemon=True)
                t.start()
                t.join(probe_timeout + 10.0)
                if t.is_alive() or "r" not in probe:
                    stall["msg"] = ("sim stall: control channel "
                                    f"unresponsive >{probe_timeout:.0f}s "
                                    "(watchdog)")
                    subprocess.run(["docker", "kill", sandbox_name],
                                   capture_output=True)
                    return

        wd = threading.Thread(target=_stall_watchdog, daemon=True)
        wd.start()
        try:
            op_meta = operator(
                {
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
                }
            )
        finally:
            wd_stop.set()
        if stall:
            raise RuntimeError(stall["msg"])

        # Blind single episode: the examiner asks once, after the operator
        # is done. The machine latches per step (official any-step-success
        # semantics); the end-state verdict rides along for our own books.
        verdict = machine.rpc({"cmd": "success"})
        rec["success"] = bool(verdict["success"])
        if "success_end_state" in verdict:
            rec["success_end_state"] = bool(verdict["success_end_state"])
        if "success_at" in verdict:  # time-to-success (post-hoc budgets)
            rec["success_at"] = verdict["success_at"]
        # Closing step snapshot (ruling 2026-08-14 Q2): total sim steps so
        # step-economy stats exist for FAILED trials too. Individually
        # guarded (diff-review F-D): auxiliary telemetry must never
        # retract an already-obtained verdict.
        try:
            rec["steps_total"] = machine.rpc(
                {"cmd": "steps"}, timeout_s=60.0).get("steps")
        except Exception:  # noqa: BLE001
            rec["steps_total"] = None
        rec["steps_semantics"] = "episode"  # post-F7 counter scope
        # (diff-review F-E: pre-2026-08-14 records counted process-lifetime
        # steps; trim.py only step-judges records that carry this marker)
        rec["operator_meta"] = op_meta
        rec["termination"] = op_meta.get("termination", "operator_done")
        # Validity judgment (quota voiding etc.) belongs to orchestration
        # (classify.py, from artifacts); the conductor records faithfully
        # and flags only its OWN failures below.
        rec["anomaly"] = None
    except Exception as exc:  # noqa: BLE001; anomaly ledger, not a crash
        rec["success"] = False
        rec["termination"] = "anomaly"
        rec["anomaly"] = f"{type(exc).__name__}: {exc}"
        # The one-liner names the failure; the stack finds it. Host-side
        # errors have no bridge.log to fall back on.
        import traceback
        rec["anomaly_traceback"] = traceback.format_exc()
    finally:
        if machine is not None:
            machine.shutdown()
        if sandbox_live:
            sandbox_down(sandbox_name)
        # Post-trial token values too; a mid-trial rotation would leave
        # BOTH generations potentially visible in the record.
        secrets += record.secret_strings(creds_home)
        shutil.rmtree(cfg_dir, ignore_errors=True)
        record.finalize_trial(trial_dir, agent, secrets)
        lock.release()
    # wall_seconds spans the whole harness (boot + gate + agent +
    # teardown). The wall cap is enforced ONLY on the agent subprocess
    # (agent_operator timeout) and its verdict already sits in
    # rec["termination"]; never overwrite it from the harness span
    # (audit 2026-08-14 F3: the old unconditional rewrite relabeled
    # near-cap self-finishes and anomalies as wall_clock_cap; the
    # PreSoakPan self-contradiction).
    rec["wall_seconds"] = round(time.time() - t0, 1)
    # Named for the semantics, so a result file says which budget it ran
    # under without anyone dating it against a commit (ruling 2026-08-20).
    rec["active_wall_cap_min"] = wall_cap_min
    record.write_result(trial_dir, rec)
    return rec
