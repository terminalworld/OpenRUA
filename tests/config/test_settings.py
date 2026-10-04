"""All settings entry points preserve the same validated user file."""

from concurrent.futures import ThreadPoolExecutor

import pytest

from openrua.config import load_user_config
from openrua.config.settings import update
from openrua.errors import ConfigError


def test_updates_preserve_other_fields_and_validate_before_replacing(tmp_path):
    path = tmp_path / 'config.yaml'
    path.write_text('sandbox: {run_args: [--userns=keep-id]}\n')
    update(tmp_path, {'agent': 'codex', 'robot': 'panda'}, {'model': 'chosen'})
    update(tmp_path, {'benchmark': 'libero_pro'})
    user = load_user_config(path)
    assert user.sandbox.run_args == ['--userns=keep-id']
    assert user.agents['codex'].model == 'chosen'
    before = path.read_bytes()
    with pytest.raises(ConfigError):
        update(tmp_path, {'unexpected': True})
    assert path.read_bytes() == before
    update(tmp_path, {'benchmark': None})
    assert load_user_config(path).benchmark is None
    assert not list(tmp_path.glob('.config-*'))


def test_two_writers_do_not_erase_each_others_fields(tmp_path):
    with ThreadPoolExecutor() as pool:
        futures = [pool.submit(update, tmp_path, values) for values in
                   [{'robot': 'panda'}, {'simulator': 'robosuite'}, {'benchmark': 'capbench'}]]
        for future in futures:
            future.result()
    user = load_user_config(tmp_path / 'config.yaml')
    assert (user.robot, user.simulator, user.benchmark) == ('panda', 'robosuite', 'capbench')
