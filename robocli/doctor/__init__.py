"""``robocli doctor``: is this machine ready to bring a robot up?

Every check is a function returning ``CheckResult`` rows (``id``,
``label``, ``severity``, ``detail``, ``hint``); ``run`` collects them in
order. One report renders two ways with no divergence: lines with a
mark per row and the fix under every error, or JSON (``--json``, or
whenever stdout is not a terminal). The exit status is 1 exactly when an
error row exists; warnings never fail.
"""

from robocli.doctor import checks  # noqa: F401
from robocli.doctor.checks import CHECKS, Context, run  # noqa: F401
from robocli.doctor.report import CheckResult, Report, print_report  # noqa: F401
