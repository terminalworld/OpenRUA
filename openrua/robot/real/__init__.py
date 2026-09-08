"""A real robot: its ROS 2 graph is provided by the vendor's own stack.

- ``up.py``    run the profile's launch command if it has one, return a
               handle that waits until the graph answers a probe
- ``down.py``  stop a launch process started by ``up``

There is no bridge: the agent reaches the graph the sandbox was pointed
at (``machine.backend.discovery``), and the handle's rpc answers
``not_applicable`` because no truth side exists on hardware.
"""
