"""Agent metadata environments are independent of robot projects and accounts."""

from copy import copy
from types import SimpleNamespace

import pytest

from openrua import agents
from openrua.cli import build_parser
from openrua.cli.commands import build
from openrua.errors import UnavailableError


def test_agent_image_identity_follows_installation_and_user(monkeypatch):
    adapter = agents.get('codex@0.164.0')
    baseline = build.agent_image(adapter, robot_uid=1001)
    assert baseline == build.agent_image(agents.get('codex@0.164.0'), robot_uid=1001)
    assert baseline != build.agent_image(agents.get('codex@0.165.0'), robot_uid=1001)
    assert baseline != build.agent_image(adapter, robot_uid=1002)
    for field, value in [('install', 'echo custom CLI'), ('prepare', 'echo custom runtime'),
                         ('prepare_check', 'custom-runtime --version'), ('min_version', '0.165.0')]:
        changed = copy(adapter)
        setattr(changed, field, value)
        assert baseline != build.agent_image(changed, robot_uid=1001)
    monkeypatch.setattr(build, 'agent_recipe_sha256', lambda: 'changed-base-recipe')
    assert baseline != build.agent_image(adapter, robot_uid=1001)


def test_build_agent_ignores_broken_project_and_resolves_user_pin(tmp_path, monkeypatch):
    home = tmp_path / 'home'
    home.mkdir()
    (home / 'config.yaml').write_text('agent: codex\nagents:\n  codex:\n    version: 0.164.0\n')
    project = tmp_path / 'project'
    (project / '.openrua').mkdir(parents=True)
    (project / '.openrua/settings.json').write_text('{')
    captured = []
    monkeypatch.setattr(build.projects, 'selection', lambda *a: pytest.fail('must not read the robot project'))
    monkeypatch.setattr(build.config, 'compose', lambda *a, **k: pytest.fail('must not compose robot config'))
    monkeypatch.setattr(build, '_agent', lambda adapter, **kw: captured.append((adapter, kw)) or ('cli', 'digest'))
    args = build_parser().parse_args(['--home', str(home), '-C', str(project),
                                     'build', 'agent', '--tag', 'cli', '--robot-uid', '1002'])
    assert args.fn(args) == 0
    adapter, parameters = captured[0]
    assert adapter.name == 'codex' and adapter.version == '0.164.0'
    assert parameters == {'tag': 'cli', 'robot_uid': 1002}


@pytest.mark.parametrize('fails', [False, True])
def test_agent_build_uses_manifest_recipe_and_only_publishes_valid_candidate(monkeypatch, fails):
    adapter = agents.get('codex@0.164.0')
    compiled, commands, checked = [], [], []
    def compile(**kwargs):
        compiled.append(kwargs)
        return kwargs['tag'], 'sha256:agent'
    def check(image, chosen):
        checked.append(image)
        assert chosen == [adapter]
        if fails:
            raise UnavailableError('CLI version is incompatible')
    monkeypatch.setattr(build, 'build_agent', compile)
    monkeypatch.setattr(build, 'build_sandbox', lambda **kw: pytest.fail('must not build ROS'))
    monkeypatch.setattr(build, '_report_versions', check)
    monkeypatch.setattr(build.subprocess, 'run',
                        lambda argv, **kw: commands.append(argv) or SimpleNamespace(returncode=0))
    if fails:
        with pytest.raises(UnavailableError, match='incompatible'):
            build._agent(adapter, tag='requested', robot_uid=1001)
    else:
        assert build._agent(adapter, tag='requested', robot_uid=1001) == ('requested', 'sha256:agent')
    recipe = compiled[0]
    assert recipe['preinstall'] == agents.preinstall([adapter])
    assert recipe['prepare'] == agents.preparation([adapter])
    assert recipe['robot_uid'] == 1001 and recipe['stamp'] == ''
    assert recipe['labels']['openrua.agent.codex.version'] == '0.164.0'
    assert checked == [recipe['tag']]
    assert commands[-1] == ['docker', 'rmi', recipe['tag']]
    promoted = [command for command in commands if command[1] == 'tag']
    assert promoted == ([] if fails else [['docker', 'tag', recipe['tag'], 'requested']])


def test_report_versions_checks_resolved_agent_pin(monkeypatch):
    adapter = agents.get('codex@0.164.0')
    calls = []
    def run(image, command):
        calls.append(command)
        assert image == 'candidate'
        return SimpleNamespace(returncode=1 if command != 'codex --version' else 0,
                               stdout='codex-cli 0.163.0\n', stderr='')
    monkeypatch.setattr(build, 'cli_result', run)
    with pytest.raises(UnavailableError, match='not 0.164.0'):
        build._report_versions('candidate', [adapter])
    assert '0.164.0' in calls[-1]
