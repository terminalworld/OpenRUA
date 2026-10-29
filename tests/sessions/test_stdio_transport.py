"""Native JSON framing with a real child process, without an agent or robot."""

import asyncio
import json
import sys

import pytest

from openrua.sessions.execution import StdioTransport


def test_large_fragmented_frame_and_following_frame_are_preserved(tmp_path):
    size = 5 * 1024 * 1024
    program = '''
import json, sys
raw = json.dumps({'kind': 'tool', 'output': 'x' * int(sys.argv[1])}).encode() + b'\\n'
for offset in range(0, len(raw), 8191):
    sys.stderr.buffer.write(b'e' * 8191)
    sys.stderr.buffer.flush()
    sys.stdout.buffer.write(raw[offset:offset + 8191])
    sys.stdout.buffer.flush()
print('   ')
print('{"kind": "finished"}', flush=True)
'''

    async def run():
        transport = await StdioTransport.start(
            [sys.executable, '-u', '-c', program, str(size)], tmp_path / 'agent.log')
        try:
            frame = await asyncio.wait_for(transport.read(), 10)
            assert frame == {'kind': 'tool', 'output': 'x' * size}
            assert await transport.read() == {'kind': 'finished'}
            assert await transport.read() is None
            assert not transport.invalid_output.exists()
            assert (tmp_path / 'agent.log').stat().st_size > size
        finally:
            await transport.close()
    asyncio.run(run())


@pytest.mark.parametrize('valid_utf8', [True, False])
def test_split_utf8_is_decoded_or_retained_without_losing_the_next_frame(tmp_path, valid_utf8):
    expected = {'output': 'caf\u00e9 \U0001f916'}
    raw = json.dumps(expected, ensure_ascii=False).encode() if valid_utf8 else b'{"output":"\xff"}'
    program = '''
import sys, time
for byte in bytes.fromhex(sys.argv[1]):
    sys.stdout.buffer.write(bytes([byte]))
    sys.stdout.buffer.flush()
    time.sleep(0.001)
sys.stdout.buffer.write(b'\\n \\r\\n{"kind":"finished"}\\n')
'''

    async def run():
        transport = await StdioTransport.start(
            [sys.executable, '-u', '-c', program, raw.hex()], tmp_path / 'agent.log',
            buffer_bytes=1)
        try:
            if valid_utf8:
                assert await asyncio.wait_for(transport.read(), 10) == expected
                assert not transport.invalid_output.exists()
            else:
                with pytest.raises(ValueError, match='not a JSON object'):
                    await asyncio.wait_for(transport.read(), 10)
                assert transport.invalid_output.read_bytes() == raw + b'\n'
                assert transport.invalid_output.stat().st_mode & 0o777 == 0o600
            assert await transport.read() == {'kind': 'finished'}
            assert await transport.read() is None
        finally:
            await transport.close()
    asyncio.run(run())


@pytest.mark.parametrize('valid', [True, False])
def test_large_last_frame_at_eof_is_decoded_or_retained(tmp_path, valid):
    size = 5 * 1024 * 1024
    program = '''
import json, sys
raw = json.dumps({'output': 'x' * int(sys.argv[1])})
sys.stdout.write(raw if sys.argv[2] == 'valid' else raw[:-2])
'''

    async def run():
        transport = await StdioTransport.start(
            [sys.executable, '-c', program, str(size), 'valid' if valid else 'partial'],
            tmp_path / 'agent.log')
        try:
            if valid:
                assert await transport.read() == {'output': 'x' * size}
            else:
                with pytest.raises(ValueError, match='not a JSON object'):
                    await transport.read()
                assert transport.invalid_output.read_text() == json.dumps({'output': 'x' * size})[:-2]
                assert transport.invalid_output.stat().st_mode & 0o777 == 0o600
            assert await transport.read() is None
        finally:
            await transport.close()
    asyncio.run(run())


def test_incomplete_large_frame_can_be_cancelled_and_process_closed(tmp_path):
    marker = tmp_path / 'written'
    program = '''
import sys
from pathlib import Path
sys.stdout.write('{"output":"' + 'x' * (5 * 1024 * 1024))
sys.stdout.flush()
Path(sys.argv[1]).touch()
sys.stdin.read()
'''

    async def run():
        transport = await StdioTransport.start(
            [sys.executable, '-u', '-c', program, str(marker)], tmp_path / 'agent.log')
        reading = asyncio.create_task(transport.read())
        try:
            async def written():
                while not marker.exists():
                    await asyncio.sleep(0.01)
            await asyncio.wait_for(written(), 10)
            assert not reading.done()
            reading.cancel()
            with pytest.raises(asyncio.CancelledError):
                await reading
        finally:
            reading.cancel()
            await asyncio.gather(reading, return_exceptions=True)
            await asyncio.wait_for(transport.close(), 2)
        assert transport.process.returncode == 0
        assert asyncio.all_tasks() == {asyncio.current_task()}
    asyncio.run(run())
