"""Inspect and manage an existing shared session, without starting an agent."""

from __future__ import annotations

import json
import sqlite3
from uuid import uuid4

from openrua.config import paths
from openrua.errors import UnavailableError
from openrua.runner.live_state import DEFAULT_NAME
from openrua.sessions.client import Client
from openrua.sessions.store import SQLiteStore


def connect(args) -> Client:
    endpoint = paths.sandbox_dir(args.name, args.home) / "endpoint.json"
    try:
        return Client.from_file(endpoint)
    except (OSError, KeyError, ValueError) as exc:
        raise UnavailableError(f"no usable shared service for {args.name!r}: {exc}",
                               hint="start it with openrua serve <robot> --name <name>") from exc


def run(args) -> int:
    operation = args.action
    directory = paths.sandbox_dir(args.name, args.home)
    if operation in {"status", "events"} and not (directory / "endpoint.json").exists():
        try:
            store = SQLiteStore.open_readonly(directory / "conversation.sqlite")
            try:
                result = store.snapshot() if operation == "status" else store.events(args.after, args.limit)
            finally:
                store.close()
        except (sqlite3.Error, ValueError) as exc:
            raise UnavailableError(str(exc), hint="check the retained session name and directory") from exc
        print(json.dumps(result, indent=2, ensure_ascii=False))
        return 0
    client = connect(args)
    try:
        if operation == "status":
            result = client.snapshot()
        elif operation == "events":
            result = client.events(args.after, args.limit)
        elif operation == "end":
            client.end()
            result = {"closed": True, "files_retained": True}
        elif operation == "send":
            request_id = args.request_id or str(uuid4())
            # Expose retry identity before making the request, including on a
            # lost acknowledgement. Reuse both IDs with the same text to retry.
            print(json.dumps({"client_id": args.client_id, "request_id": request_id}), flush=True)
            result = client.command("enqueue", client_id=args.client_id, request_id=request_id, text=args.text)
        elif operation == "respond":
            result = client.command(operation, request_id=args.request_id, answers=json.loads(args.answers))
        else:
            parameters = {"interrupt": ("message_id",), "resume": ("pause_id",),
                          "withdraw": ("message_id",), "edit": ("message_id", "text", "revision"),
                          "resolve_unknown": ("message_id", "note")}
            result = client.command(operation, **{key: getattr(args, key) for key in parameters[operation]})
    except (RuntimeError, ValueError) as exc:
        raise UnavailableError(str(exc), hint="inspect openrua session --name <name> status before retrying") from exc
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("session", help="inspect, queue messages, pause/resume or end a shared session")
    p.add_argument("--name", default=DEFAULT_NAME)
    actions = p.add_subparsers(dest="action", required=True)
    actions.add_parser("status", help="snapshot, pending questions and message IDs")
    events = actions.add_parser("events", help="read retained events after a cursor")
    events.add_argument("--after", type=int, default=0)
    events.add_argument("--limit", type=int, default=1000)
    send = actions.add_parser("send", help="enqueue a message without waiting for its result")
    send.add_argument("text")
    send.add_argument("--client-id", default="cli", help="reuse with request ID for safe retry")
    send.add_argument("--request-id", default=None, help="unique for this message; reuse only for retries")
    for action in ("interrupt", "withdraw"):
        item = actions.add_parser(action)
        item.add_argument("message_id")
    resume = actions.add_parser("resume", help="release the currently confirmed pause")
    resume.add_argument("pause_id", help="copy the current pause_id from status")
    edit = actions.add_parser("edit")
    edit.add_argument("message_id")
    edit.add_argument("text")
    edit.add_argument("--revision", type=int, required=True)
    reconcile = actions.add_parser("resolve_unknown", help="record checking an uncertain execution, without replaying it")
    reconcile.add_argument("message_id")
    reconcile.add_argument("note")
    respond = actions.add_parser("respond", help="answer a pending agent question")
    respond.add_argument("request_id")
    respond.add_argument("answers", help='JSON mapping question IDs to lists of answers')
    actions.add_parser("end", help="stop owned resources and retain messages and workspace")
    p.set_defaults(fn=run)
