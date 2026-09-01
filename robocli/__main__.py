"""``python -m robocli`` is the same front door as the ``robocli`` command."""

import sys

from robocli.cli import main

sys.exit(main())
