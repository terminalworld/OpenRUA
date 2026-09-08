"""The sandbox: the agent's terminal on the robot's ROS 2 graph.

    Input : a reachable ROS 2 machine (network / domain / peer) + its
            resolved config (+ an image choice).
    Output: one live container with a workspace generated for this
            machine, in which a person or an agent operates the robot
            through its native CLI and rclpy. Nothing else.

Verbs (each independently runnable; parameters in, value or file out):

- ``build`` : Dockerfile + PREINSTALL (an optional one-line install
  chain) + distro + uid -> image (prints name and digest). Images split
  by distro and preinstall, never by benchmark or task; task facts reach
  the sandbox only through the workspace at ``up``. Internet access is
  the proxy package's job.
- ``up``    : machine reachability + config + image + workspace dir +
  generic creation-time slots (--mount / --env) -> one live container
  (prints its name). Never builds: a missing image is an instructive
  error, not an implicit build with guessed parameters.
- ``down``  : container name -> removed.

What this package does not know: agents, tasks, prompts, credentials
semantics, scoring, timing. Agent-specific files pass through generic
slots only (PREINSTALL at build; --mount/--env at up; docker mounts
must exist at creation). The layering contract (pyproject and
tests/test_layering.py) enforces that this package imports no other
unit: what the agent experiences knows nothing about measurement.
"""


from openrua.errors import UnavailableError


class SandboxError(UnavailableError):
    """Failure of a sandbox verb. Verbs raise this (an ordinary Exception
    a consumer's handler can catch); only the command-line entry points
    convert it to an exit status."""
