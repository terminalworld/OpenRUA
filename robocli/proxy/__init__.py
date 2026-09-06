"""Proxy: the compound's shared gatehouse; the ONE door to the internet.

Sandboxes live on an internal docker network with no route out; this
package is the single dual-homed relay (one foot on the internal net,
one on the default bridge) that forwards ONLY whitelisted domains
(tinyproxy, default-deny, 443 CONNECT). One wall serves every sandbox;
its lifetime spans campaigns, not trials.

Verbs:
- ``build`` : whitelist (generic slot, one regex per line; empty = deny
  all) -> wall image (prints tag + digest).
- ``up``    : IDEMPOTENT ensure -> proxy URL (``http://<name>:8888``).
  Singleton-infra semantics; running means done; concurrent cold-start
  losers adopt the winner's instance; never ``rm -f`` a live wall (it
  would cut every in-flight session). Contrast sandbox.up, which always
  births a fresh container: one is a standing facility, the other a
  per-use product.
- ``down``  : name -> removed (only for teardown/testing; campaigns
  leave the wall standing).

Which domains an occupant needs is occupant knowledge: callers pass the
adapter's regexes as the whitelist value. This package knows no agent
and imports no layer.
"""


from robocli.errors import UnavailableError


class ProxyError(UnavailableError):
    """Library-level failure of a proxy verb (see SandboxError's
    rationale in robocli.sandbox: libraries raise Exceptions, only CLI
    mains exit)."""
