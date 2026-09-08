"""The proxy: the one route from the sandboxes to the internet.

Sandboxes live on an internal docker network with no route out; this
package is the single dual-homed relay (one interface on the internal
net, one on the default bridge) that forwards only whitelisted hosts
(tinyproxy, default-deny, 443 CONNECT). One proxy serves every sandbox;
its lifetime spans campaigns, not trials.

Verbs:
- ``build`` : whitelist (generic slot, one regex per line; empty = deny
  all) -> proxy image (prints tag + digest).
- ``up``    : idempotent ensure -> proxy URL (``http://<name>:8888``).
  Running means done; concurrent cold-start losers adopt the winner's
  instance; a live proxy is never ``rm -f``-ed (it would cut every
  in-flight session). Contrast sandbox.up, which always creates a fresh
  container: one is a standing facility, the other a per-use product.
- ``down``  : name -> removed (only for teardown and testing; campaigns
  leave the proxy standing).

Which hosts an agent needs is the agent's knowledge: callers pass the
manifest's regexes as the whitelist value. This package knows no agent
and imports no other unit.
"""


from openrua.errors import UnavailableError

NAME = "openrua-proxy"    # the standing container
IMAGE = "openrua-proxy"   # the image openrua build proxy produces
PORT = 8888               # tinyproxy's listen port, also stamped on the image as a label


class ProxyError(UnavailableError):
    """Failure of a proxy verb (see SandboxError: libraries raise
    exceptions, only the command-line entry points exit)."""
