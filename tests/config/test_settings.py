"""Personal defaults preserve independent agent preferences."""

from concurrent.futures import ThreadPoolExecutor

import pytest

from openrua.config import load_user_config
from openrua.config.settings import update
from openrua.errors import ConfigError


def test_updates_preserve_other_fields_and_validate_before_replacing(tmp_path):
    path = tmp_path / 'config.yaml'
    path.write_text('sandbox: {run_args: [--userns=keep-id]}\n')
    update(tmp_path, {'agent': 'codex'}, {'model': 'chosen'})
    assert load_user_config(path).sandbox.run_args == ['--userns=keep-id']
    assert load_user_config(path).agents['codex'].model == 'chosen'
    before = path.read_bytes()
    with pytest.raises(ConfigError):
        update(tmp_path, {'robot': 'panda'})
    assert path.read_bytes() == before
    assert not list(tmp_path.glob('.config-*'))


def test_two_writers_do_not_erase_each_others_agent_fields(tmp_path):
    update(tmp_path, {'agent': 'codex'})
    with ThreadPoolExecutor() as pool:
        futures = [pool.submit(update, tmp_path, {}, values) for values in
                   [{'model': 'chosen'}, {'version': '0.163.0'}, {'options': {'effort': 'high'}}]]
        for future in futures:
            future.result()
    agent = load_user_config(tmp_path / 'config.yaml').agents['codex']
    assert agent.model == 'chosen' and agent.version == '0.163.0' and agent.options['effort'] == 'high'
