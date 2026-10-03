"""Chat-first terminal client. The session service owns all execution decisions."""

from __future__ import annotations

import asyncio
from uuid import uuid4

from rich.text import Text
from textual import work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import Button, Collapsible, Footer, Input, Select, Static, TextArea

from openrua.sessions.client import Client


def literal(value: str) -> Text:
    # Never interpret agent output as terminal escapes or Rich markup.
    return Text("".join(c for c in value if c in "\n\t" or c.isprintable()))


class Edit(ModalScreen[str | None]):
    """Keep the observed message revision outside the editor until confirmation."""

    BINDINGS = [("escape", "cancel", "Cancel")]

    def __init__(self, title: str, text: str = ""):
        super().__init__()
        self.title_text, self.value = title, text

    def compose(self) -> ComposeResult:
        with Vertical(classes="dialog"):
            yield Static(literal(self.title_text))
            yield TextArea(self.value, id="edit-text")
            with Horizontal(classes="buttons"):
                yield Button("Save", id="save", variant="primary")
                yield Button("Cancel", id="cancel")

    def on_mount(self):
        self.query_one(TextArea).focus()

    def action_cancel(self):
        self.dismiss(None)

    def on_button_pressed(self, event: Button.Pressed):
        event.stop()
        if event.button.id == "save":
            value = self.query_one(TextArea).text
            if value.strip():
                self.dismiss(value)
        else:
            self.dismiss(None)


class Confirm(ModalScreen[bool]):
    BINDINGS = [("escape", "cancel", "Cancel")]

    def __init__(self, question: str):
        super().__init__()
        self.question = question

    def compose(self) -> ComposeResult:
        with Vertical(classes="dialog"):
            yield Static(literal(self.question))
            with Horizontal(classes="buttons"):
                yield Button("Confirm", id="confirm", variant="warning")
                yield Button("Cancel", id="cancel")

    def action_cancel(self):
        self.dismiss(False)

    def on_button_pressed(self, event: Button.Pressed):
        event.stop()
        self.dismiss(event.button.id == "confirm")


class Questions(ModalScreen[dict | None]):
    BINDINGS = [("escape", "cancel", "Cancel")]

    def __init__(self, request: dict):
        super().__init__()
        self.request = request

    def compose(self) -> ComposeResult:
        with VerticalScroll(classes="dialog"):
            yield Static("The agent needs your input")
            for i, question in enumerate(self.request["questions"]):
                yield Static(literal(question["text"]))
                if question.get("choices"):
                    yield Select([(literal(c), c) for c in question["choices"]], id=f"answer-{i}")
                else:
                    yield Input(password=question.get("secret", False), id=f"answer-{i}")
            with Horizontal(classes="buttons"):
                yield Button("Send answer", id="answer", variant="primary")
                yield Button("Cancel", id="cancel")

    def action_cancel(self):
        self.dismiss(None)

    def on_button_pressed(self, event: Button.Pressed):
        event.stop()
        if event.button.id != "answer":
            self.dismiss(None)
            return
        answers = {}
        for i, question in enumerate(self.request["questions"]):
            value = self.query_one(f"#answer-{i}").value
            if value is Select.BLANK or not str(value).strip():
                return
            answers[question["id"]] = [str(value)]
        self.dismiss(answers)


class Turn(Vertical):
    def __init__(self, message: dict):
        super().__init__(classes="turn")
        self.message = message
        self.outputs: dict[str, tuple[Static, Collapsible | None, str]] = {}

    def compose(self) -> ComposeResult:
        yield Static(literal(f"You · {self.message['status']}"), classes="turn-status")
        yield Static(literal(self.message["text"]), classes="user-text")
        yield Vertical(classes="outputs")
        yield Static("", classes="result")

    def refresh_message(self, message):
        if message == self.message:
            return
        self.message = message
        self.query_one(".turn-status", Static).update(literal(f"You · {message['status']}"))
        self.query_one(".user-text", Static).update(literal(message["text"]))

    async def event(self, event):
        data = event.get("data", {})
        if event["kind"] == "turn_finished":
            error = data.get("error") or {}
            self.query_one(".result", Static).update(literal(data.get("text") or error.get("message") or data["status"]))
        elif event["kind"] in {"text_delta", "item"}:
            key = data.get("item_id") or "text"
            is_message = event["kind"] == "text_delta" or data.get("kind") == "message"
            if key not in self.outputs:
                output = Static("", classes="agent-output" if is_message else "tool-output")
                fold = None if is_message else Collapsible(output, title="Tool", collapsed=True)
                await self.query_one(".outputs").mount(fold or output)
                self.outputs[key] = output, fold, ""
            output, fold, previous = self.outputs[key]
            if is_message:
                text = previous + data.get("text", "") if event["kind"] == "text_delta" else data.get("text") or previous
            else:
                text = data.get("output") or str(data.get("details") or "")
                fold.title = literal(f"{data.get('phase', '')} · {data.get('text') or data.get('kind', 'Tool')}")
            output.update(literal(text))
            self.outputs[key] = output, fold, text


class QueueEntry(Vertical):
    def __init__(self, message: dict):
        super().__init__(classes="queue-entry")
        self.message = message

    def compose(self) -> ComposeResult:
        yield Static(literal(self.message["text"]))
        with Horizontal(classes="buttons"):
            yield Button("Edit", name="edit", classes="edit-queued")
            yield Button("Withdraw", name="withdraw", classes="withdraw-queued")


class ChatApp(App):
    """A detachable client, with no robot or agent implementation dependencies."""

    TITLE = "OpenRUA"
    CSS_PATH = "chat.tcss"
    ENABLE_COMMAND_PALETTE = False
    BINDINGS = [Binding("ctrl+s", "send", "Send", priority=True),
                Binding("ctrl+p", "panel", "Queue", priority=True),
                Binding("ctrl+q", "quit", "Detach", priority=True)]

    def __init__(self, client: Client, name: str = "robot"):
        super().__init__()
        self.client, self.session_name = client, name
        self.client_id = str(uuid4())
        self.pending: dict | None = None
        self.cursor = 0
        self.state: dict = {}
        self.turns: dict[str, Turn] = {}
        self.queue_state = None
        self.online = False
        self.connection_notice = None
        self.busy = False

    def compose(self) -> ComposeResult:
        yield Static(literal(f"OpenRUA · {self.session_name} · connecting"), id="status")
        with Horizontal(id="body"):
            yield VerticalScroll(id="thread")
            with VerticalScroll(id="panel"):
                yield Static("Queued instructions")
                yield Vertical(id="queue")
        yield Static("", id="notice")
        with Horizontal(classes="buttons", id="actions"):
            yield Button("Queue", id="toggle-panel")
            yield Button("Interrupt", id="interrupt", disabled=True)
            yield Button("Resume", id="resume", disabled=True)
            yield Button("Answer", id="questions", disabled=True)
            yield Button("End session", id="end", disabled=True)
        yield TextArea(id="message")
        with Horizontal(classes="buttons"):
            yield Button("Send", id="send", variant="primary", disabled=True)
            yield Static("Enter: new line · Ctrl+S: send · Ctrl+Q: detach", id="hint")
        yield Footer()

    def on_mount(self):
        self.view = self.screen
        self.view.query_one("#panel").display = False
        self.view.query_one("#message").focus()
        self.poll_worker = self.poll()

    def on_resize(self, event):
        self.set_class(event.size.width < 90, "narrow")

    def notice(self, text: str):
        self.view.query_one("#notice", Static).update(literal(text))

    def action_panel(self):
        panel = self.view.query_one("#panel")
        panel.display = not panel.display

    def controls(self):
        state = self.state
        ready = self.online and not state.get("closed") and not self.busy
        self.view.query_one("#send", Button).disabled = not ready
        self.view.query_one("#send", Button).label = "Retry send" if self.pending else "Send"
        self.view.query_one("#message", TextArea).read_only = bool(self.pending)
        self.view.query_one("#interrupt", Button).disabled = not (ready and state.get("active"))
        self.view.query_one("#resume", Button).disabled = not (ready and state.get("paused") and state.get("connected") and not state.get("active") and not any(m["status"] == "unknown" for m in state.get("messages", [])))
        self.view.query_one("#questions", Button).disabled = not (ready and any(r["status"] == "pending" for r in state.get("requests", {}).values()))
        self.view.query_one("#end", Button).disabled = not ready

    async def sync(self):
        snapshot = await asyncio.to_thread(self.client.snapshot)
        self.state = snapshot["state"]
        self.controls()
        thread = self.view.query_one("#thread", VerticalScroll)
        follow = thread.is_vertical_scroll_end
        for message in self.state["messages"]:
            if message["id"] not in self.turns:
                turn = Turn(message)
                await thread.mount(turn)
                self.turns[message["id"]] = turn
            self.turns[message["id"]].refresh_message(message)
            if self.pending and (message["client_id"], message["request_id"]) == (self.pending["client_id"], self.pending["request_id"]):
                self.pending = None
                self.view.query_one("#message", TextArea).load_text("")
                self.notice("Message accepted")
        # Only replay through this snapshot. The next poll handles newer turns.
        while self.cursor < snapshot["cursor"]:
            records = await asyncio.to_thread(self.client.events, self.cursor)
            if not records:
                break
            for record in records:
                if record["seq"] > snapshot["cursor"]:
                    break
                if record["kind"] == "agent_event":
                    event = record["data"]
                    turn = self.turns.get(event.get("turn_id"))
                    if turn:
                        await turn.event(event)
                self.cursor = record["seq"]
        queue = [m for m in self.state["messages"] if m["status"] == "queued"]
        if queue != self.queue_state:
            box = self.view.query_one("#queue", Vertical)
            await box.remove_children()
            for message in queue:
                await box.mount(QueueEntry(message))
            self.queue_state = queue
        self.view.query_one("#toggle-panel", Button).label = f"Queue ({len(queue)})"
        state = self.state
        status = "Ended" if state["closed"] else "Paused" if state["paused"] else "Working" if state["active"] else "Ready" if state["connected"] else "Agent disconnected"
        if state["requests"]:
            status += " · input requested"
        if any(m["status"] == "unknown" for m in state["messages"]):
            status += " · execution unknown; reconcile in session CLI or web"
        self.view.query_one("#status", Static).update(literal(f"OpenRUA · {self.session_name} · {status}"))
        if self.connection_notice is not None:
            if self.view.query_one("#notice", Static).content == literal(self.connection_notice):
                self.notice("Reconnected")
            self.connection_notice = None
        self.online = True
        self.controls()
        if follow:
            thread.scroll_end(animate=False)

    @work(exclusive=True, group="poll")
    async def poll(self):
        while True:
            try:
                await self.sync()
            except (RuntimeError, OSError, ValueError) as exc:
                self.online = False
                self.view.query_one("#status", Static).update("Disconnected · reconnecting; execution may continue")
                self.connection_notice = str(exc)
                self.notice(self.connection_notice)
                self.controls()
            await asyncio.sleep(0.25)

    @work(group="commands")
    async def action_send(self):
        if len(self.screen_stack) > 1 or self.busy or not self.online or self.state.get("closed"):
            return
        text = self.view.query_one("#message", TextArea).text
        if not text.strip():
            return
        if self.pending is None:
            self.pending = dict(client_id=self.client_id, request_id=str(uuid4()), text=text)
        params = self.pending.copy()
        self.busy = True
        self.controls()
        try:
            await asyncio.to_thread(self.client.command, "enqueue", **params)
            # Poll may already have reconciled this acceptance while we waited.
            if self.pending == params:
                self.pending = None
                self.view.query_one("#message", TextArea).load_text("")
            self.notice("Message accepted")
        except (RuntimeError, OSError, ValueError) as exc:
            self.notice(f"{exc}. Retry send uses the same request: {params['request_id']}")
        finally:
            self.busy = False
            self.controls()

    @work(group="commands")
    async def command(self, operation: str, **params):
        if self.busy or not self.online:
            return
        self.busy = True
        self.controls()
        try:
            if operation == "end":
                snapshot = await asyncio.to_thread(self.client.end)
                self.poll_worker.cancel()
                self.state = snapshot["state"]
                for message in self.state["messages"]:
                    if message["id"] in self.turns:
                        self.turns[message["id"]].refresh_message(message)
                self.view.query_one("#status", Static).update(literal(f"OpenRUA · {self.session_name} · Ended"))
            else:
                await asyncio.to_thread(self.client.command, operation, **params)
            self.notice("Request accepted")
        except (RuntimeError, OSError, ValueError) as exc:
            self.notice(f"{exc}. Check the current state before repeating this operation.")
        finally:
            self.busy = False
            self.controls()

    def on_button_pressed(self, event: Button.Pressed):
        button = event.button
        if button.id == "send":
            self.action_send()
        elif button.id == "toggle-panel":
            self.action_panel()
        elif button.id == "interrupt":
            self.command("interrupt", message_id=self.state["active"])
        elif button.id == "resume":
            pause_id = self.state["pause_id"]
            self.push_screen(Confirm("Resume the retained queue? Review queued instructions first."), lambda yes: self.command("resume", pause_id=pause_id) if yes else None)
        elif button.id == "end":
            self.push_screen(Confirm("End this session and stop its resources? Records and workspace will be retained."), lambda yes: self.command("end") if yes else None)
        elif button.id == "questions":
            request = next(r for r in self.state["requests"].values() if r["status"] == "pending")
            self.push_screen(Questions(request), lambda answers: self.command("respond", request_id=request["request_id"], answers=answers) if answers is not None else None)
        elif button.name in {"edit", "withdraw"}:
            row = button.parent.parent
            message = row.message.copy()
            if button.name == "withdraw":
                self.command("withdraw", message_id=message["id"])
            else:
                self.push_screen(Edit("Edit queued instruction", message["text"]), lambda text: self.command("edit", message_id=message["id"], text=text, revision=message["revision"]) if text is not None else None)
