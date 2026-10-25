"""Run the real startup update flow with a bounded local installer fixture."""
import sys
import time
from pathlib import Path

if '--installer' in sys.argv:
    path = Path(sys.argv[-1])
    attempt = int(path.read_text()) + 1 if path.exists() else 1
    path.write_text(str(attempt))
    (path.parent / 'installer-pid').write_text(str(__import__('os').getpid()))
    print('[update] Installing selected fixture version', flush=True)
    print('RAW INSTALLER OUTPUT', flush=True)
    time.sleep(30 if '--hold' in sys.argv else .35)
    raise SystemExit(1 if attempt == 1 else 0)

from openrua.cli import main, updates
from openrua.cli.updates import Offer

home = Path(sys.argv[sys.argv.index('--home') + 1])
slow = '--slow-fixture' in sys.argv
if slow: sys.argv.remove('--slow-fixture')
required = '--required-fixture' in sys.argv
if required: sys.argv.remove('--required-fixture')
updates.offers = lambda composed, folder: [Offer('fixture-agent', '1.0', '2.0', required,
    [sys.executable, str(Path(__file__).resolve()), '--installer', *(['--hold'] if slow else []), str(home / 'attempts')], False,
    'Fixture Agent', '2.0')]
raise SystemExit(main())
