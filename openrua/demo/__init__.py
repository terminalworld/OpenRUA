"""A demo video of a trial: the terminal on the left, the robot's
cameras on the right, from the artifacts a recorded trial leaves
behind (``frames/`` and ``ops.jsonl``).

``render(trial)`` composes the edited demo using the ``demo`` extra.
``cameras.export_camera_videos(frames, output)`` exports each camera
at its recorded times using the base installation. The unit reads
files and has no robot or agent dependencies.
"""

from openrua.demo.compose import Style, render  # noqa: F401
