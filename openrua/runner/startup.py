"""Report startup phases through the service's existing log."""

from contextlib import contextmanager
import time


BEGIN = '[startup] '
TIMING = '[startup timing] '


@contextmanager
def phase(message: str):
    """Show the current work and retain its duration without hiding failures."""
    print(BEGIN + message, flush=True)
    started = time.monotonic()
    outcome = 'completed'
    try:
        yield
    except BaseException:
        outcome = 'failed'
        raise
    finally:
        print(f'{TIMING}{message}: {outcome} in {time.monotonic() - started:.3f}s', flush=True)


def read_progress(stream, report=print) -> None:
    """Forward complete phase records, leaving other service output in its log."""
    while True:
        position = stream.tell()
        line = stream.readline()
        if not line.endswith('\n'):
            stream.seek(position)
            return
        if line.startswith(BEGIN):
            report('[prepare] ' + line[len(BEGIN):].rstrip())
        elif line.startswith(TIMING):
            report(line.rstrip())
