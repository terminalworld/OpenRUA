"""One module per verb, each with ``add_parser(sub)`` and ``run(args)``.

Registration order is the ``--help`` order.
"""

from robocli.cli.commands import (agent, build, config, demo, doctor, down, list, probe,
                                  run, up)

COMMANDS = (list, build, up, agent, down, run, demo, probe, config, doctor)
