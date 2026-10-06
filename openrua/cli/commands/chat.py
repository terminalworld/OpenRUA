"""Chat with a shared native agent; leaving this client does not end its session."""

from __future__ import annotations

import json
import time
from uuid import uuid4

from openrua.cli.commands.session import connect
from openrua.errors import UnavailableError, UsageError
from openrua.runner.live_state import DEFAULT_NAME


class Display:
    def __init__(self):
        self.streamed = set()

    def event(self, record: dict) -> None:
        if record["kind"] != "agent_event":
            return
        event = record["data"]
        data = event.get("data", {})
        key = (event.get("turn_id"), data.get("item_id"))
        if event["kind"] == "text_delta":
            self.streamed.add(key)
            print(data.get("text", ""), end="", flush=True)
        elif event["kind"] == "item":
            if data["kind"] == "message" and data["phase"] == "completed" and key not in self.streamed:
                print(data.get("text", ""), flush=True)
            elif data["kind"] != "message" and data["phase"] == "started":
                print(f"\n[{data['kind']}] {data.get('text', '')}", flush=True)
        elif event["kind"] == "input_required":
            print("\n[agent needs input] " + json.dumps(data, ensure_ascii=False), flush=True)
        elif event["kind"] == "turn_finished":
            print(f"\n[{data['status']}] {event.get('turn_id')}", flush=True)


def watch(client, after: int, message_id: str | None = None) -> None:
    display = Display()
    cursor = after
    while True:
        events = client.events(cursor)
        for event in events:
            display.event(event)
            cursor = event["seq"]
        # Drain all events preceding the snapshot cursor before declaring a
        # turn complete; otherwise its last answer could be missed by a race.
        snapshot = client.snapshot()
        if cursor < snapshot["cursor"]:
            continue
        state = snapshot["state"]
        if state["closed"]:
            return
        if message_id:
            message = next(m for m in state["messages"] if m["id"] == message_id)
            if message["status"] in {"completed", "failed", "interrupted", "unknown", "withdrawn", "reconciled"}:
                print(f"[chat] {message['status']}; event cursor {cursor}", flush=True)
                return
            if state["paused"] and state["active"] != message_id:
                print(f"[chat] message retained while queue is paused; event cursor {cursor}", flush=True)
                return
            if state["requests"]:
                print("[chat] reply using openrua session --name <name> respond; the session continues", flush=True)
                return
        time.sleep(0.25)


def run(args) -> int:
    if args.tui:
        if args.message is not None or args.follow or args.after:
            raise UsageError("--tui cannot be combined with a message, --follow, or --after")
        from openrua.terminal.launcher import check_terminal
        try:
            check_terminal()
        except RuntimeError as exc:
            raise UnavailableError(str(exc)) from exc
        client = connect(args)
        client.timeout = 5
        from openrua.cli.commands.start import open_terminal
        return open_terminal(args.home, client, args.name)
    client = connect(args)
    client_id = str(uuid4())
    try:
        if args.follow:
            if args.message:
                raise ValueError("--follow reads events and does not submit a message")
            watch(client, args.after)
            return 0
        prompt = args.message
        interactive = prompt is None
        while True:
            if prompt is None:
                try:
                    prompt = input("you> ")
                except EOFError:
                    break
            if interactive and prompt.strip() == '/resume':
                from openrua.cli.commands.start import choose_history
                selected = choose_history(args.home)
                if selected:
                    client, name = selected
                    if getattr(client, 'read_only', False):
                        print(f'[chat] read-only history: openrua --resume {name}')
                        client = connect(args)
                    else:
                        args.name = name
                        print(f'[chat] connected to {name}')
                prompt = None
                continue
            if prompt.strip():
                cursor = client.snapshot()["cursor"]
                request_id = str(uuid4())
                print(f"[chat] client {client_id}; request {request_id}", flush=True)
                message = client.command("enqueue", client_id=client_id, request_id=request_id, text=prompt)
                print(f"[chat] accepted {message['id']}", flush=True)
                watch(client, cursor, message["id"])
            if not interactive:
                break
            prompt = None
    except KeyboardInterrupt:
        print("\n[chat] detached; agent execution and queued messages are unchanged")
    except (RuntimeError, ValueError) as exc:
        raise UnavailableError(str(exc), hint="reconnect with chat --follow; inspect session status before resending") from exc
    return 0


def add_parser(sub) -> None:
    p = sub.add_parser("chat", help="chat with an existing shared agent, or reconnect to its events",
                       description=__doc__)
    p.add_argument("message", nargs="?", help="one message; omit for a conversation")
    p.add_argument("--name", default=DEFAULT_NAME)
    p.add_argument("--tui", action="store_true", help="open keyboard terminal chat (included in the default installation)")
    p.add_argument("--follow", action="store_true", help="observe events without submitting anything")
    p.add_argument("--after", type=int, default=0, help="event cursor for --follow (default: from the beginning)")
    p.set_defaults(fn=run)
