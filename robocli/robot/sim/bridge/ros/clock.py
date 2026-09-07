"""Clock semantics: paused (mainline) vs free-running (sub-experiment).

Same code, two switches. Mainline = paused:
sim time advances only when a command triggers a step; ``/clock``
publishes sim time and every node on the graph runs with
``use_sim_time:=true``. Observation never advances the world; the
bridge re-publishes current state at a low wall-clock rate without
stepping. The free-running thread (deadman semantics, RTF disclosure)
is added when the sub-experiment activates; the switch already exists in
the config (``clock.mode``).
"""

from __future__ import annotations

from builtin_interfaces.msg import Time as TimeMsg


def sim_time_msg(sim_seconds: float) -> TimeMsg:
    msg = TimeMsg()
    msg.sec = int(sim_seconds)
    msg.nanosec = int((sim_seconds - int(sim_seconds)) * 1e9)
    return msg
