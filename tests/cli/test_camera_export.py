"""Camera video retries use saved evidence and never rerun a robot task."""

import json

from PIL import Image
import pytest

from openrua.cli import build_parser
from openrua.cli.commands import demo
from openrua.errors import UsageError
from openrua.runner import lock


def parse(directory, *flags):
    return build_parser().parse_args(['demo', str(directory), '--camera-only', *flags])


def test_camera_only_retry_preserves_task_and_raw_frames(tmp_path):
    frames = tmp_path / 'frames'
    frames.mkdir()
    original = {'success': True, 'termination': 'self_finished', 'wall_seconds': 12,
                'cleanup': {'status': 'completed', 'errors': []},
                'video_export': {'status': 'failed'}}
    result = tmp_path / 'result.json'
    result.write_text(json.dumps(original))
    for name, color in [('front', 'red'), ('wrist', 'blue')]:
        Image.new('RGB', (24, 16), color).save(frames / f'000000_{name}.png')
    index = frames / 'index.jsonl'
    index.write_text(json.dumps({'step': 0, 't': 2.0,
                                'files': ['000000_front.png', '000000_wrist.png']}) + '\n')
    raw = {p.name: p.read_bytes() for p in frames.iterdir()}
    assert demo.run(parse(tmp_path)) == 0
    saved = json.loads(result.read_text())
    assert saved['video_export']['status'] == 'completed'
    assert {k: v for k, v in saved.items() if k != 'video_export'} == {
        k: v for k, v in original.items() if k != 'video_export'}
    assert {p.name: p.read_bytes() for p in frames.iterdir()} == raw
    assert sorted(p.name for p in (tmp_path / 'videos').iterdir()) == ['front.mp4', 'wrist.mp4']
    assert lock.holder(tmp_path) is None


def test_camera_export_refuses_a_trial_with_an_active_writer(tmp_path):
    claim = lock.acquire(tmp_path, 'fixture')
    try:
        with pytest.raises(UsageError, match='still running'):
            demo.run(parse(tmp_path))
        assert lock.holder(tmp_path) is not None
        assert not (tmp_path / 'videos').exists()
    finally:
        claim.release()


def test_camera_export_reports_missing_frames_without_changing_verdict(tmp_path, capsys):
    result = tmp_path / 'result.json'
    result.write_text(json.dumps({'success': False, 'termination': 'self_finished'}))
    assert demo.run(parse(tmp_path)) == 1
    saved = json.loads(result.read_text())
    assert saved['success'] is False
    assert saved['video_export']['status'] == 'failed'
    assert 'index.jsonl' in capsys.readouterr().out


@pytest.mark.parametrize('flag,value', [('--speed', '4'), ('--cameras', 'front'), ('--fps', '10')])
def test_camera_export_rejects_edited_demo_options(tmp_path, flag, value):
    with pytest.raises(UsageError, match='terminal and camera demo'):
        demo.run(parse(tmp_path, flag, value))
    assert lock.holder(tmp_path) is None
