"""Project skill menus stay scoped and adoption never overwrites existing work."""

import json
from pathlib import Path

import pytest

from openrua.cli.project_skills import ProjectSkills
from openrua.config import projects
from tests.runner.test_skill_review import apparatus


@pytest.fixture
def tools(apparatus, monkeypatch):
    a = apparatus
    projects.trust(a.root, a.home)
    from openrua.cli.commands import skill_review
    monkeypatch.setattr(skill_review, 'selection', lambda *args: ('codex', a.adapter, a.settings, 'fixture-model'))
    return a, ProjectSkills(a.home, lambda: a.root)


def candidate(tools):
    a, api = tools
    result = a.run()
    detail = api.dispatch({'operation': 'report', 'id': result['id']})
    return result, detail['candidates'][0]


def test_view_preview_import_and_reopen_saved_method(tools):
    a, api = tools
    result, selected = candidate(tools)
    assert selected['can_import'] and not (a.root / '.agents').exists()
    detail = api.dispatch({'operation': 'candidate', 'id': result['id'], 'key': selected['key']})
    assert 'name: camera-check' in detail['text']
    imported = api.dispatch({'operation': 'import', 'id': result['id'],
                             'key': selected['key'], 'fingerprint': selected['fingerprint']})
    assert imported['imported'] == '.agents/skills/camera-check'
    reopened = ProjectSkills(a.home, lambda: a.root)
    overview = reopened.dispatch({'operation': 'overview'})
    assert len(overview['entries']) == 1
    saved = reopened.dispatch({'operation': 'skill', 'key': overview['entries'][0]['key']})
    assert saved['text'] == detail['text']
    assert (a.root / '.claude/skills/camera-check/SKILL.md').read_text() == saved['text']


def test_previewed_candidate_changes_are_rejected_without_writing(tools):
    a, api = tools
    result, selected = candidate(tools)
    path = Path(result['output']) / 'proposed' / selected['key']
    path.write_text(path.read_text() + '\nChanged after preview.\n')
    with pytest.raises(ValueError, match='changed after preview'):
        api.dispatch({'operation': 'import', 'id': result['id'], 'key': selected['key'],
                      'fingerprint': selected['fingerprint']})
    assert not (a.root / '.agents').exists()


def test_existing_skill_is_preserved_and_merge_only_returns_a_draft(tools):
    a, api = tools
    result, selected = candidate(tools)
    current = a.root / '.agents/skills/camera-check/SKILL.md'
    current.parent.mkdir(parents=True)
    current.write_text('---\nname: camera-check\ndescription: Existing project method.\n---\nKeep newer edits.\n')
    with pytest.raises(ValueError, match='cannot be imported'):
        api.dispatch({'operation': 'import', 'id': result['id'], 'key': selected['key'],
                      'fingerprint': selected['fingerprint']})
    draft = api.dispatch({'operation': 'merge', 'id': result['id'], 'key': selected['key']})['draft']
    assert '/workspace/.openrua/skill-reviews/' in draft
    assert 'Preserve newer project edits' in draft
    assert 'Keep newer edits' in current.read_text()
    assert len(a.calls) == 1


def test_requests_cannot_choose_another_project_or_read_arbitrary_files(tools, tmp_path):
    a, api = tools
    result, selected = candidate(tools)
    for body in [dict(operation='overview', project=str(tmp_path)),
                 dict(operation='skill', key='../secret'), dict(operation='report', id='../secret'),
                 dict(operation='candidate', id=result['id'], key='../../secret')]:
        with pytest.raises(ValueError): api.dispatch(body)
    projects.trust(tmp_path, a.home)
    other = ProjectSkills(a.home, lambda: tmp_path)
    with pytest.raises(ValueError, match='not found'):
        other.dispatch(dict(operation='report', id=result['id']))


def test_adoption_requires_project_trust_and_policy_keeps_saved_skills(tools):
    a, api = tools
    result, selected = candidate(tools)
    (a.home / 'trusted-workspaces.json').write_text('[]')
    with pytest.raises(ValueError, match='trust'):
        api.dispatch(dict(operation='import', id=result['id'], key=selected['key'], fingerprint=selected['fingerprint']))
    projects.trust(a.root, a.home)
    api.dispatch(dict(operation='policy', mode='manual'))
    assert api.dispatch(dict(operation='overview'))['policy'] == 'manual'
    assert len(api.dispatch(dict(operation='overview'))['reports']) == 1


def test_saved_skill_links_outside_project_are_not_previewed(tools, tmp_path):
    a, api = tools
    outside = tmp_path / 'outside'
    outside.mkdir()
    (outside / 'SKILL.md').write_text('---\nname: private\ndescription: PRIVATE OUTSIDE DATA\n---\n')
    directory = a.root / '.agents/skills'
    directory.mkdir(parents=True)
    (directory / 'outside').symlink_to(outside)
    assert api.dispatch(dict(operation='overview'))['entries'] == []


def test_authenticated_http_action_imports_without_sending_a_robot_message(tools, tmp_path):
    import asyncio
    from openrua.sessions.client import Client
    from tests.sessions.test_http import running_server, stop
    a, api = tools
    result, selected = candidate(tools)
    async def run():
        store, session, native, execution, server = await running_server(
            tmp_path / 'http-owner', actions={'project-skills': api.action})
        try:
            first = Client(server.url, server.token)
            request = dict(operation='overview')
            overview = await asyncio.to_thread(first.request, '/api/actions/project-skills', request)
            assert overview['result']['project'] == str(a.root)
            second = Client(server.url, server.token)
            imported = await asyncio.to_thread(second.request, '/api/actions/project-skills',
                dict(operation='import', id=result['id'], key=selected['key'], fingerprint=selected['fingerprint']))
            assert imported['result']['name'] == selected['name']
            assert store.snapshot()['state']['messages'] == []
            assert not any('submit' in frame for frame in native.writes)
            with pytest.raises(RuntimeError):
                await asyncio.to_thread(second.request, '/api/actions/project-skills',
                                        dict(operation='overview', project='/other'))
        finally: await stop(store, execution, server)
    asyncio.run(run())


def test_saved_relative_agent_manifest_resolves_from_its_project(tmp_path, monkeypatch):
    from openrua import agents
    from openrua.cli.commands import skill_review
    root, home = tmp_path / 'project', tmp_path / 'home'
    root.mkdir()
    manifest = root / 'project-codex.yaml'
    manifest.write_bytes(agents.manifest('codex').path.read_bytes())
    (root / '.openrua').mkdir()
    (root / '.openrua/settings.json').write_text(json.dumps(dict(agent='./project-codex.yaml', model='project-model')))
    monkeypatch.chdir(tmp_path)
    requested, adapter, settings, model = skill_review.selection(root, home)
    assert requested == str(manifest)
    assert adapter.name == 'codex' and model == 'project-model'


def test_import_cannot_redirect_project_writes_to_a_linked_external_directory(tools, tmp_path):
    a, api = tools
    result, selected = candidate(tools)
    outside = tmp_path / 'outside-skills'
    outside.mkdir()
    (a.root / '.agents').symlink_to(outside)
    with pytest.raises(ValueError, match='inside the project'):
        api.dispatch(dict(operation='import', id=result['id'], key=selected['key'], fingerprint=selected['fingerprint']))
    assert list(outside.iterdir()) == []


def test_unchanged_baseline_methods_are_not_shown_as_proposals(tools):
    from tests.test_skills import write_skill
    a, api = tools
    write_skill(a.root / '.agents/skills', 'unchanged-method')
    result, selected = candidate(tools)
    detail = api.dispatch(dict(operation='report', id=result['id']))
    assert [row['name'] for row in detail['candidates']] == ['camera-check']
    assert result['proposed_skills'] == 1


@pytest.mark.parametrize('legacy', [False, True])
def test_conflict_comparison_uses_original_hashes_when_baseline_was_redacted(tools, monkeypatch, legacy):
    from tests.test_skills import write_skill
    from openrua.runner import skill_review
    a, api = tools
    folder = write_skill(a.root / '.agents/skills')
    file = folder / 'SKILL.md'
    file.write_text(file.read_text() + '\nSynthetic token: ' + a.key.read_text() + '\n')
    def edit(adapter, profile, workspace, *args, **kwargs):
        proposal = workspace / 'proposed/.agents/skills/camera-check/SKILL.md'
        proposal.write_text(proposal.read_text() + '\nReviewed instructions.\n')
        return 'completed'
    monkeypatch.setattr(skill_review, '_worker', edit)
    result = a.run()
    if legacy:
        report = Path(result['output']) / 'report.json'
        data = json.loads(report.read_text())
        for row in data['changes']: row.pop('source_sha256', None)
        report.write_text(json.dumps(data))
    detail = api.dispatch(dict(operation='report', id=result['id']))
    assert detail['candidates'][0]['source_changed'] is False
    file.write_text(file.read_text() + '\nNewer edit after review.\n')
    detail = api.dispatch(dict(operation='report', id=result['id']))
    assert detail['candidates'][0]['source_changed'] is True
