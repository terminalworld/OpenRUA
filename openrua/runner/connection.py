"""A robot connection that can change while its agent environment stays alive."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path

from openrua import robot
from openrua.config import apply_suite_overrides, compose, normalize_arms
from openrua.errors import UnavailableError
from openrua.runner import bringup, live_state, record
from openrua.runner.motion import cancel_motion, motion_actions
from openrua.sandbox import project
from openrua.robot.cameras import camera_topics


class EnvironmentChanged(RuntimeError):
    """The requested connection needs another agent environment."""
    def __init__(self, resolved):
        super().__init__('the ROS tool environment changed; an internal handover is needed')
        self.resolved = resolved


def environment(cfg: dict) -> dict:
    """Creation-time facts that cannot be applied to a running sandbox."""
    backend = cfg['machine']['backend']
    return deepcopy({'backend': {key: backend.get(key) for key in
                     ('kind', 'sandbox_image', 'ros_distro', 'discovery', 'sandbox_install')},
                     'sandbox': cfg.get('sandbox', {})})


class Connection:
    def __init__(self, request, resolved, directory: Path, config_path: Path,
                 domain: int, network: str, sandbox: str):
        self.request, self.resolved = request, deepcopy(resolved)
        self.directory, self.config_path = directory, config_path
        self.domain, self.network, self.sandbox = domain, network, sandbox
        self.records = None if request.project else directory / 'connections'
        self.machine = None
        self.generation = 0
        self.environment = environment(resolved.cfg)
        self.robot_log = directory / 'robot.log'

    def read_config(self):
        """Read maintained files, including when a broken prerequisite needs building."""
        request = self.request
        resolved = compose(request.robot, request.simulator, request.benchmark,
                           request.home, agent=request.agent)
        if request.task_suite or resolved.suite:
            apply_suite_overrides(resolved.cfg, request.task_suite or resolved.suite)
        normalize_arms(resolved.cfg)
        return resolved

    def prepare(self):
        """Check maintained configuration before stopping an existing connection."""
        resolved = self.read_config()
        failed = robot.not_ready(resolved.cfg['machine']['backend'])
        if failed:
            raise UnavailableError('robot startup checks failed: ' + '; '.join(
                f"{item['name']}: {item['fix']}" for item in failed))
        if environment(resolved.cfg) != self.environment:
            raise EnvironmentChanged(resolved)
        return resolved

    def shutdown(self):
        """Retain ownership on stop failure; never start a competing driver."""
        if self.machine is not None:
            self.machine.shutdown()
            self.machine = None

    def start(self, resolved=None, prepared=None):
        if self.machine is not None:
            raise ValueError('the robot already has an owned connection; use restart')
        resolved = resolved or self.prepare()
        resolved, directory, config_path = prepared or self.stage(resolved)
        cfg = resolved.cfg
        self.resolved, self.directory = resolved, directory
        self.config_path, self.robot_log = config_path, directory / 'robot.log'
        if self.request.project is not None:
            materials = live_state.path(self.request.name, self.request.home).parent / 'robot-materials'
            project.prepare(cfg, self.request.project, materials)
            from openrua.runner.robot_client import guide
            guide(materials)
        try:
            self.machine = bringup.start_robot(
                cfg, config_path, live_state.container_names(self.request.name)[0], self.sandbox,
                self.request.task_suite or resolved.suite,
                self.request.task_id if self.request.task_id is not None else resolved.task_id,
                self.network, self.domain, self.robot_log, wait_ready=False)
            self.machine.wait_ready()
            if cfg['machine']['backend']['kind'] == 'sim':
                task = bringup.start_episode(self.machine, self.request.init_state).get('language', '')
            else:
                task = self.request.task or ''
            self._retain(task)
            self.generation += 1
        except BaseException:
            self.shutdown()
            raise

    def stage(self, resolved):
        """Validate referenced files before replacing a running driver."""
        resolved = deepcopy(resolved)
        cfg = resolved.cfg
        # Every attempt keeps its own configuration and logs.
        directory = project.run_directory(
            self.request.project or self.directory, self.request.name,
            records=self.records)
        bringup.resolve_robot_files(cfg, directory, self.request.home)
        config_path = record.write_config(directory, cfg)
        return resolved, directory, config_path

    def _retain(self, task):
        facts = live_state.load(self.request.name, self.request.home)
        live_state.save(self.request.name, self.request.home, **{
            **facts, 'run_directory': str(self.directory), 'task': task,
            'robot_model': self.resolved.cfg['machine'].get('robot', {}).get('model')})

    def stopped(self):
        return self.machine.stopped() if self.machine is not None else None

    @property
    def cameras(self):
        return camera_topics(self.resolved.cfg) if self.machine is not None else ({}, None)

    def stop_motion(self):
        if self.machine is None:
            return {}
        return cancel_motion(self.sandbox, motion_actions(self.resolved.cfg['machine']))
