"""Harness: the machine's cockpit factory. One contract, three verbs.

    Input : a reachable ROS 2 machine (network / domain / peer) + its
            assembly config (+ a seat-image choice).
    Output: ONE LIVE SANDBOX CONTAINER; a workspace generated for this
            machine, in which a human or an agent operates the robot
            through its native CLI / rclpy. Nothing else.

Verbs (each independently runnable; parameter in, value/file out):

- ``build`` : recipe + PREINSTALL (optional one-line install chain) +
  distro + uid -> seat image (prints name + digest). Images split by
  distro x preinstall, NEVER by benchmark or task; task facts reach
  the sandbox only through the workspace at ``up``. The internet wall is
  its own package (robocli.proxy: the shared gatehouse).
- ``up``    : machine reachability + config + image + workspace dir +
  generic birth-time slots (--mount / --env) -> one live container
  (prints its name). Never builds: a missing image is an instructive
  error, not an implicit build with guessed parameters.
- ``down``  : container name -> removed.

What this package deliberately does NOT know: agents, tasks, prompts,
credentials semantics, scoring, timing. Occupant luggage passes through
generic slots only (PREINSTALL at build; --mount/--env at up; docker
mounts must exist at birth). The layering contract (pyproject +
tests/test_layering.py) machine-enforces that this package imports no
other layer: the cockpit is blind to measurement.
"""


from robocli.errors import UnavailableError


class SandboxError(UnavailableError):
    """Library-level failure of a sandbox verb. Verbs raise THIS (an
    ordinary Exception a consumer's anomaly handler can catch); only the
    CLI mains convert it to SystemExit. Raising SystemExit from library
    code would bypass `except Exception` guards and kill whole batches
    (review 2026-08-15 H1)."""
