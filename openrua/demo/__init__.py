"""A demo video of a trial: the terminal on the left, the robot's
cameras on the right, from the artifacts a recorded trial leaves
behind (``frames/`` and ``ops.jsonl``).

``render(trial)`` is the whole interface; ``openrua demo <trial>`` is
its command. The unit reads files and imports ``openrua.errors`` only;
its rendering libraries are the ``demo`` extra
(``pip install 'openrua[demo]'``).
"""

from openrua.demo.compose import Style, render  # noqa: F401
