"""``python -m openrua`` is the same command line as ``openrua``."""

import sys

from openrua.cli import main

sys.exit(main())
