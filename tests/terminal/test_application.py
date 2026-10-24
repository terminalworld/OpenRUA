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
        wait('Review your project'); key(b'1')
        wait('Fixture build failed'); key(b'1')
        wait('Ready'); key(b'Inspect\r')
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
        assert output.count(b'\x1b[?1049h') == output.count(b'\x1b[?1049l') == 1
        assert output.rfind(b'Recorded measurement') < output.rfind(b'\x1b[?1049l')
    finally:
        if child.poll() is None: child.kill()
        child.wait()
        os.close(master)
