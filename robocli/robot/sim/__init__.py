"""A simulated robot: a container whose first process is the bridge.

- ``build.py``   Dockerfile.<distro> -> image, self-describing labels
- ``up.py``      image + trial parameters -> live container, returns a
                 ``client.BridgeClient`` (the handle)
- ``down.py``    force-remove the container
- ``client.py``  the host end of the bridge's control line
- ``bridge/``    the software running inside the container

Host side throughout except ``bridge/``, which nothing host-side imports.
"""
