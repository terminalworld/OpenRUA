"""One module per verb, each with ``add_parser(sub)`` and ``run(args)``.

Registration order is the ``--help`` order.
"""

from openrua.cli.commands import (agent, bench, build, clean, config, connect, demo, doctor, down,
                                  execute, list, login, probe, ps, up, serve, session, skills, update)

COMMANDS = (list, build, update, execute, up, connect, serve, session, agent, down, bench, ps, demo, probe, config, skills, login, clean, doctor)
