"""``python -m robocli`` is the same command line as ``robocli``."""

import sys

from robocli.cli import main

sys.exit(main())
