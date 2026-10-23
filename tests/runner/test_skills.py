"""Native instruction entry points preserve existing project instructions."""

from openrua import agents, skills
from openrua.runner.skills import prepare
from openrua.sandbox import project


def test_native_instruction_file_survives_existing_project_memory_and_new_sessions(tmp_path):
    root, home, materials = tmp_path / 'project', tmp_path / 'home', tmp_path / 'materials'
    project.prepare({}, root, materials)
    original = 'User-native instructions.\n'
    (root / 'CLAUDE.md').write_text(original)
    adapter = agents.get('claude-code')
    native = root / adapter.skills.instructions
    native.parent.mkdir(parents=True)
    native.write_text('Keep this existing rule.\n')
    for _ in range(2): prepare(root, home, adapter, materials)
    assert (root / 'CLAUDE.md').read_text() == original
    assert native.read_text().startswith('Keep this existing rule.\n')
    assert native.read_text().count('## OpenRUA skills') == 1
    assert skills.GUIDANCE in native.read_text()
