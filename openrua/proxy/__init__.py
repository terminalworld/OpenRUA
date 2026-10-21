"""The proxy: the one route out for a sandbox limited to its model service.

A run whose agent may reach only its model service puts the sandbox on
an internal docker network with no route out; this package is the
dual-homed relay (one interface on the internal net, one on the default
bridge) that forwards only whitelisted hosts (tinyproxy, default-deny,
443 CONNECT). One proxy serves every sandbox with the same whitelist;
its lifetime spans campaigns, not trials. A run without that limit
starts no proxy.

Verbs:
- ``build`` : -> proxy image (prints tag + digest); the image holds no
  whitelist.
- ``up``    : whitelist (generic slot, one regex per line; empty = deny
  all) -> idempotent ensure -> proxy URL (``http://<name>:8888``).
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


import hashlib

from openrua.errors import UnavailableError

NAME = "openrua-proxy"    # the standing containers' name prefix
IMAGE = "openrua-proxy"   # the image openrua build proxy produces
PORT = 8888               # tinyproxy's listen port, also stamped on the image as a label


class ProxyError(UnavailableError):
    """Failure of a proxy verb (see SandboxError: libraries raise
    exceptions, only the command-line entry points exit)."""


def name_for(whitelist: str) -> str:
    """The standing proxy for this whitelist: the prefix plus the hash of
    its hosts, so a proxy's name says which hosts it lets through."""
    return f"{NAME}-{hashlib.sha256(whitelist.encode()).hexdigest()[:8]}"
