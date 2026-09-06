"""The doctor's findings: rows, one report, rendered as lines or JSON."""

from __future__ import annotations

import json
import sys
from dataclasses import asdict, dataclass, field

SEVERITIES = ("ok", "warning", "error")
MARK = {"ok": "ok", "warning": "??", "error": "!!"}


@dataclass
class CheckResult:
    id: str
    label: str
    severity: str = "ok"
    detail: str = ""
    hint: str = ""


@dataclass
class Report:
    checks: list[CheckResult] = field(default_factory=list)

    @property
    def summary(self) -> dict[str, int]:
        return {s: sum(1 for c in self.checks if c.severity == s) for s in SEVERITIES}

    @property
    def ok(self) -> bool:
        return self.summary["error"] == 0

    def to_json(self) -> str:
        return json.dumps({"ok": self.ok, "summary": self.summary,
                           "checks": [asdict(c) for c in self.checks]}, indent=2)

    def render(self) -> str:
        lines = ["robocli doctor"]
        for c in self.checks:
            detail = f"  ({c.detail})" if c.detail else ""
            lines.append(f"  [{MARK[c.severity]}] {c.label}{detail}")
            if c.severity == "error" and c.hint:
                lines.append(f"       fix: {c.hint}")
        s = self.summary
        lines.append(f"{s['ok']} ok, {s['warning']} warnings, {s['error']} errors")
        return "\n".join(lines)


def print_report(report: Report, as_json: bool | None = None) -> int:
    """Print the report the right way for the reader and return the
    exit status: ``--json`` forces JSON, and so does a non-terminal
    stdout; errors, not warnings, make it 1."""
    if as_json is None:
        as_json = not sys.stdout.isatty()
    print(report.to_json() if as_json else report.render())
    return 0 if report.ok else 1
