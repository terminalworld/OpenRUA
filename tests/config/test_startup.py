"""Guided choices require real startup evidence and cannot form invalid pairs."""
from openrua.config import compose, startup


def test_every_offered_combination_has_current_live_evidence(tmp_path):
    rows = startup.verified_environments()
    assert rows, 'Run scripts/check_environment.py before offering environment choices'
    for row in rows:
        selection = row['selection']
        composed = compose(selection['robot'], selection['sim'], selection['bench'], tmp_path)
        assert composed.suite == row['suite'] and composed.task_id == row['task_id']
        assert row['checks'] == dict(joint_states=True, camera_frame=True, trajectory=True)
        assert all(value['id'].startswith('sha256:') for value in row['images'].values())


def test_choices_follow_tested_combinations_and_repair_descendants():
    rows = [{'selection': dict(robot='panda', sim='robosuite', bench='')},
            {'selection': dict(robot='panda-omron', sim='robosuite', bench='robocasa365')},
            {'selection': dict(robot='widowx', sim='maniskill', bench='simpler')}]
    values = dict(robot='panda', sim='robosuite', bench='')
    changed = startup.select(rows, values, 'robot', 'panda-omron')
    assert changed == rows[1]['selection']
    assert startup.options(rows, changed, 'bench') == ['robocasa365']
    assert startup.select(rows, changed, 'robot', 'widowx') == rows[2]['selection']
    assert '' not in startup.options(rows, changed, 'bench')


def test_missing_or_stale_evidence_is_not_offered(monkeypatch):
    monkeypatch.setattr(startup, 'profile_digest', lambda: 'changed')
    assert startup.verified_environments() == []
