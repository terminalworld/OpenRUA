"""One module per verb, each with ``add_parser(sub)`` and ``run(args)``.

Registration order is the ``--help`` order.
"""

from openrua.cli.commands import (agent, bench, build, clean, config, demo, doctor, down,
                                  list, probe, ps, run, up, serve, session, chat)

COMMANDS = (list, build, run, up, serve, chat, session, agent, down, bench, ps, demo, probe, config, clean, doctor)
