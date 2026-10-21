"""One module per verb, each with ``add_parser(sub)`` and ``run(args)``.

Registration order is the ``--help`` order.
"""

from openrua.cli.commands import (agent, bench, build, clean, config, connect, demo, doctor, down,
                                  execute, list, probe, ps, up, serve, session, chat)

COMMANDS = (list, build, execute, up, connect, serve, chat, session, agent, down, bench, ps, demo, probe, config, clean, doctor)
