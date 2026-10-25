"""Run setup, preparation and conversation through one real terminal host."""

import fcntl
import json
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import sys
import termios
import time

import pytest

pytestmark = pytest.mark.skipif(
    not (Path(__file__).resolve().parents[2] / 'ui/terminal/node_modules').exists(),
    reason='terminal integration job installs the pinned npm dependencies',
)


@pytest.mark.parametrize('columns,rows', [(100, 30), (40, 18)])
def test_session_setup_cancel_new_conversation_and_resume_keep_both_owners(tmp_path, columns, rows):
    master, slave = pty.openpty()
    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', rows, columns, 0, 0))
    result = tmp_path / 'result.json'
    child = subprocess.Popen([sys.executable, '-m', 'ui.terminal.test.session_setup', str(result)],
        cwd=Path(__file__).resolve().parents[2], stdin=slave, stdout=slave, stderr=slave,
        env={**os.environ, 'TERM': 'xterm-256color'})
    os.close(slave)
    output, frame = bytearray(), bytearray()

    def wait(text):
        deadline = time.monotonic() + 12
        while text.encode() not in frame:
            if select.select([master], [], [], .05)[0]:
                try: data = os.read(master, 65536)
                except OSError: break
                output.extend(data); frame.extend(data)
            assert child.poll() is None, output.decode(errors='replace')
            assert time.monotonic() < deadline, output.decode(errors='replace')
        assert text.encode() in frame, output.decode(errors='replace')

    def key(data):
        time.sleep(.2)
        while select.select([master], [], [], 0)[0]:
            try: output.extend(os.read(master, 65536))
            except OSError: break
        frame.clear(); os.write(master, data)

    try:
        wait('Welcome to OpenRUA')
        assert '╭'.encode() in frame and b'OpenRUA' in frame
        key(b'Original instruction\r'); wait('Working')
        assert b'Welcome to OpenRUA' not in frame
        key(b'/setup\r'); wait('Review your project')
        assert b'Keep session running' not in output
        key(b'\x04'); wait('Welcome back to OpenRUA')
        key(b'/setup\r'); wait('Review your project'); key(b'1')
        wait('Welcome to OpenRUA')
        key(b'New instruction\r'); wait('Working')
        assert b'Welcome to OpenRUA' not in frame
        key(b'/resume\r'); wait('Original conversation'); key(b'\r')
        wait('Welcome back to OpenRUA'); key(b'/quit\r')
        wait('Keep session running'); key(b'\r')
        deadline = time.monotonic() + 10
        while child.poll() is None and time.monotonic() < deadline:
            if select.select([master], [], [], .05)[0]:
                try: output.extend(os.read(master, 65536))
                except OSError: break
        assert child.wait(timeout=2) == 0, output.decode(errors='replace')
        while select.select([master], [], [], 0)[0]:
            try: output.extend(os.read(master, 65536))
            except OSError: break
        assert json.loads(result.read_text()) == dict(calls=['original', 'original'], saves=1,
            messages=[['Original instruction'], ['New instruction']],
            closed=[False, False], interrupted=[False, False])
        assert output.count(b'\x1b[?1049h') == output.count(b'\x1b[?1049l') == 1
    finally:
        if child.poll() is None: child.kill()
        child.wait(); os.close(master)


@pytest.mark.parametrize('columns,rows', [(100, 30), (40, 18)])
def test_setup_retry_chat_and_leave_share_one_fullscreen(tmp_path, columns, rows):
    master, slave = pty.openpty()
    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', rows, columns, 0, 0))
    result = tmp_path / 'result.json'
    root = Path(__file__).resolve().parents[2]
    child = subprocess.Popen([sys.executable, '-m', 'ui.terminal.test.application', str(result)],
                             cwd=root, stdin=slave, stdout=slave, stderr=slave,
                             env={**os.environ, 'TERM': 'xterm-256color'})
    os.close(slave)
    output, frame = bytearray(), bytearray()

    def wait(text):
        target = text.encode()
        deadline = time.monotonic() + 12
        while target not in frame:
            if select.select([master], [], [], .05)[0]:
                try:
                    data = os.read(master, 65536)
                except OSError:
                    break
                output.extend(data); frame.extend(data)
            assert child.poll() is None, output.decode(errors='replace')
            assert time.monotonic() < deadline, output.decode(errors='replace')
        assert target in frame, output.decode(errors='replace')

    def key(value):
        time.sleep(.2)
        while select.select([master], [], [], 0)[0]:
            try: output.extend(os.read(master, 65536))
            except OSError: break
        frame.clear()
        os.write(master, value)

    try:
        wait('Choose your environment'); key(b'2')
        wait('Choose a simulation'); key(b'2')
        wait('Choose an environment'); key(b'1')
        wait('Choose your coding agent'); key(b'1')
        wait('Agent settings'); key(b'3')
        wait('Select Model')
        assert b'Native demo-b' in frame
        key(b'1'); wait('Agent settings'); key(b'1')
        wait('Review your project'); key(b'1')
        wait('Fixture build failed'); key(b'1')
        wait('Ready')
        assert b'Welcome to OpenRUA' in frame
        key(b'Inspect\r')
        wait('Working'); key(b'/model\r')
        wait('Select Model'); key(b'2')
        wait('Select Reasoning Level'); key(b'\x1b')
        wait('Select Model'); key(b'\r')
        wait('Select Reasoning Level'); key(b'\r')
        wait('Selected demo-b'); key(b'/files\r')
        wait('measurement.txt'); key(b'2')
        wait('Recorded measurement'); key(b'\x1b')
        wait('Workspace /'); key(b'\x1b')
        time.sleep(.1); key(b'\x04')
        wait('Keep session running'); key(b'\r')
        deadline = time.monotonic() + 10
        while child.poll() is None and time.monotonic() < deadline:
            if select.select([master], [], [], .05)[0]:
                try: output.extend(os.read(master, 65536))
                except OSError: break
        assert child.wait(timeout=2) == 0, output.decode(errors='replace')
        while select.select([master], [], [], 0)[0]:
            try: output.extend(os.read(master, 65536))
            except OSError: break
        facts = json.loads(result.read_text())
        assert facts['builds'] == 2 and facts['saves'] == 2
        assert facts['settings']['model'] == 'demo-b'
        assert facts['settings']['effort'] == 'medium'
        assert facts['active_settings']['model'] == 'demo-a'
        assert not facts['closed']
        assert b'INSTALLATION_DETAIL_FOR_LOGS' not in output
        assert output.count(b'\x1b[?1049h') == output.count(b'\x1b[?1049l') == 1
        assert output.rfind(b'Recorded measurement') < output.rfind(b'\x1b[?1049l')
    finally:
        if child.poll() is None: child.kill()
        child.wait()
        os.close(master)


def test_parent_exit_restores_terminal_and_does_not_reopen_an_exit_menu(tmp_path):
    master, slave = pty.openpty()
    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 24, 80, 0, 0))
    child = subprocess.Popen([sys.executable, '-m', 'ui.terminal.test.application', str(tmp_path / 'result.json')],
        cwd=Path(__file__).resolve().parents[2], stdin=slave, stdout=slave, stderr=slave,
        env={**os.environ, 'TERM': 'xterm-256color'})
    os.close(slave)
    output, frame = bytearray(), bytearray()
    def wait(text):
        deadline = time.monotonic() + 12
        while text.encode() not in frame:
            if select.select([master], [], [], .05)[0]:
                data = os.read(master, 65536); output.extend(data); frame.extend(data)
            assert time.monotonic() < deadline, output.decode(errors='replace')
            assert child.poll() is None, output.decode(errors='replace')
    def key(data):
        time.sleep(.2)
        while select.select([master], [], [], 0)[0]: output.extend(os.read(master, 65536))
        frame.clear(); os.write(master, data)
    try:
        wait('Choose your environment'); key(b'2')
        wait('Choose a simulation'); key(b'2')
        wait('Choose an environment'); key(b'1')
        wait('Choose your coding agent'); key(b'1')
        wait('Agent settings'); key(b'1')
        wait('Review your project'); key(b'1')
        wait('Fixture build failed'); key(b'1')
        wait('Ready')
        child.terminate()
        child.wait(timeout=5)
        deadline = time.monotonic() + 8
        disconnected = False
        while time.monotonic() < deadline:
            if select.select([master], [], [], .05)[0]:
                try: data = os.read(master, 65536)
                except OSError:
                    disconnected = True; break
                if not data:
                    disconnected = True; break
                output.extend(data)
        assert disconnected, 'frontend kept the terminal open after its parent exited'
        assert output.count(b'\x1b[?1049h') == output.count(b'\x1b[?1049l') == 1
        assert b'Keep session running' not in output
    finally:
        if child.poll() is None: child.kill()
        child.wait()
        os.close(master)


def test_real_startup_reports_invalid_preferences_then_opens_after_explicit_repair(tmp_path):
    import yaml
    home, project = tmp_path / 'home', tmp_path / 'project'
    home.mkdir(); project.mkdir()
    config = home / 'config.yaml'
    preferences = {'agent': 'codex', 'agents': {'codex': {'model': 'personal-model'}}}
    config.write_text(yaml.safe_dump({**preferences, 'robot': 'panda',
                                    'simulator': 'robosuite', 'benchmark': 'libero_pro'}))
    original = config.read_bytes()

    def run(actions):
        master, slave = pty.openpty()
        fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 30, 100, 0, 0))
        child = subprocess.Popen([sys.executable, '-m', 'ui.terminal.test.startup',
                                  '--home', str(home), '--workspace', str(project)],
            cwd=Path(__file__).resolve().parents[2], stdin=slave, stdout=slave, stderr=slave,
            env={**os.environ, 'TERM': 'xterm-256color'})
        os.close(slave)
        output, frame = bytearray(), bytearray()
        try:
            for marker, key in actions:
                deadline = time.monotonic() + 15
                while marker.encode() not in frame:
                    if select.select([master], [], [], .05)[0]:
                        try: data = os.read(master, 65536)
                        except OSError as exc:
                            raise AssertionError(output.decode(errors='replace')) from exc
                        output.extend(data); frame.extend(data)
                    assert child.poll() is None, output.decode(errors='replace')
                    assert time.monotonic() < deadline, output.decode(errors='replace')
                time.sleep(.2)
                while select.select([master], [], [], 0)[0]: output.extend(os.read(master, 65536))
                frame.clear(); os.write(master, key)
            deadline = time.monotonic() + 10
            while child.poll() is None and time.monotonic() < deadline:
                if select.select([master], [], [], .05)[0]:
                    try: output.extend(os.read(master, 65536))
                    except OSError: break
            code = child.wait(timeout=2)
            while select.select([master], [], [], 0)[0]:
                try: output.extend(os.read(master, 65536))
                except OSError: break
            assert output.count(b'\x1b[?1049h') == output.count(b'\x1b[?1049l') == 1
            assert not (home / 'sandboxes').exists()
            assert not (project / '.openrua/settings.json').exists()
            return code, bytes(output)
        finally:
            if child.poll() is None: child.kill()
            child.wait(); os.close(master)

    code, output = run([('Use and trust this robot workspace?', b'\r')])
    assert code == 78
    assert b'config schema --user' in output
    assert b'session may still be running' not in output
    assert config.read_bytes() == original

    # The schema stays strict. Repair is explicit and retains personal facts.
    config.write_text(yaml.safe_dump(preferences))
    cleaned = config.read_bytes()
    code, output = run([
        ('Choose your environment', b'2'), ('Choose a simulation', b'1'),
        ('Choose an environment', b'1'), ('Choose your coding agent', b'2'),
        ('Agent settings', b'1'), ('Review your project', b'3'),
        ('personal-model', b'\x04'),
    ])
    assert code == 0
    assert config.read_bytes() == cleaned


@pytest.mark.parametrize('columns,rows', [(100, 30), (40, 18)])
def test_startup_update_retry_log_and_continue_share_one_fullscreen(tmp_path, columns, rows):
    home, project = tmp_path / 'home', tmp_path / 'project'
    home.mkdir(); project.mkdir()
    master, slave = pty.openpty()
    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', rows, columns, 0, 0))
    child = subprocess.Popen([sys.executable, '-m', 'ui.terminal.test.updates',
        '--home', str(home), '--workspace', str(project), '--trust-workspace'],
        cwd=Path(__file__).resolve().parents[2], stdin=slave, stdout=slave, stderr=slave,
        env={**os.environ, 'TERM': 'xterm-256color'})
    os.close(slave)
    output, frame = bytearray(), bytearray()
    def wait(text):
        deadline = time.monotonic() + 15
        while text.encode() not in frame:
            if select.select([master], [], [], .05)[0]:
                try: data = os.read(master, 65536)
                except OSError: break
                output.extend(data); frame.extend(data)
            assert child.poll() is None, output.decode(errors='replace')
            assert time.monotonic() < deadline, output.decode(errors='replace')
        assert text.encode() in frame, output.decode(errors='replace')
    def key(value):
        time.sleep(.2)
        while select.select([master], [], [], 0)[0]:
            try: output.extend(os.read(master, 65536))
            except OSError: break
        frame.clear(); os.write(master, value)
    try:
        wait('Update now'); key(b'1')
        wait('Update failed')
        assert b'RAW INSTALLER OUTPUT' not in output
        key(b'3'); wait('RAW INSTALLER OUTPUT')
        key(b'\x1b'); wait('Update failed')
        key(b'2'); wait('Update verified')
        key(b'1'); wait('Choose your environment'); key(b'\x04')
        child.wait(timeout=5)
        while select.select([master], [], [], 0)[0]:
            try: output.extend(os.read(master, 65536))
            except OSError: break
        assert child.returncode == 0, output.decode(errors='replace')
        assert (home / 'attempts').read_text() == '2'
        logs = list((home / 'updates').glob('*/output.log'))
        assert len(logs) == 2 and all(p.stat().st_mode & 0o777 == 0o600 for p in logs)
        assert not (home / 'sandboxes').exists()
        assert output.count(b'\x1b[?1049h') == output.count(b'\x1b[?1049l') == 1
    finally:
        if child.poll() is None: child.kill()
        child.wait(); os.close(master)


def test_interrupting_an_update_stops_its_installer_and_restores_the_terminal(tmp_path):
    home, project = tmp_path / 'home', tmp_path / 'project'
    home.mkdir(); project.mkdir()
    master, slave = pty.openpty()
    child = subprocess.Popen([sys.executable, '-m', 'ui.terminal.test.updates', '--slow-fixture',
        '--home', str(home), '--workspace', str(project), '--trust-workspace'],
        cwd=Path(__file__).resolve().parents[2], stdin=slave, stdout=slave, stderr=slave,
        env={**os.environ, 'TERM': 'xterm-256color'})
    os.close(slave)
    output = bytearray(); selected = False; interrupted = False
    try:
        deadline = time.monotonic() + 15
        while child.poll() is None and time.monotonic() < deadline:
            if select.select([master], [], [], .05)[0]:
                try: output.extend(os.read(master, 65536))
                except OSError: break
            if not selected and b'Update now' in output:
                os.write(master, b'1'); selected = True
            if selected and not interrupted and (home / 'installer-pid').exists():
                os.write(master, b'\x03'); interrupted = True
        assert child.wait(timeout=5) == 130, output.decode(errors='replace')
        while select.select([master], [], [], 0)[0]:
            try: output.extend(os.read(master, 65536))
            except OSError: break
        assert interrupted
        with pytest.raises(ProcessLookupError):
            os.kill(int((home / 'installer-pid').read_text()), 0)
        assert output.count(b'\x1b[?1049h') == output.count(b'\x1b[?1049l') == 1
        assert b'Traceback' not in output
    finally:
        if child.poll() is None: child.kill()
        child.wait(); os.close(master)
