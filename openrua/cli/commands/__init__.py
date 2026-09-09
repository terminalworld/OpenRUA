"""One module per verb, each with ``add_parser(sub)`` and ``run(args)``.

Registration order is the ``--help`` order.
"""

from openrua.cli.commands import (agent, bench, build, config, demo, doctor, down, install,
                                  list, probe, ps, run, up)

COMMANDS = (list, build, install, run, up, agent, down, bench, ps, demo, probe, config, doctor)
