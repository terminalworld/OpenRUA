"""History selection using injected listing and connection operations."""

from __future__ import annotations

import asyncio

from rich.text import Text
from textual import work
from textual.containers import Vertical
from textual.screen import ModalScreen
from textual.widgets import Input, OptionList, Static
from textual.widgets.option_list import Option


def plain(value: str) -> str:
    return ''.join(c for c in value if c.isprintable())


class History(ModalScreen):
    BINDINGS = [('escape', 'cancel', 'Cancel')]
    CSS = '''
    History { align: center middle; }
    #history-dialog { width: 90%; height: 85%; padding: 1 2; border: round $primary; background: $surface; }
    #history-list { height: 1fr; }
    #history-status { height: auto; }
    '''

    def __init__(self, list_sessions, open_session):
        super().__init__()
        self.list_sessions, self.open_session = list_sessions, open_session
        self.rows = []
        self.opening = False

    def compose(self):
        with Vertical(id='history-dialog'):
            yield Static('Resume a conversation · ended/unavailable sessions open read-only')
            yield Input(placeholder='Search by title or ID', id='history-search')
            yield OptionList(id='history-list')
            yield Static('Loading history...', id='history-status')

    def on_mount(self):
        self.load_history()

    @work(exclusive=True)
    async def load_history(self):
        try:
            self.rows = await asyncio.to_thread(self.list_sessions)
            self.filter_rows()
        except Exception as exc:
            self.query_one('#history-status', Static).update(Text(str(exc)))

    def filter_rows(self):
        query = self.query_one('#history-search', Input).value.casefold()
        matches = [r for r in self.rows if query in (r['title'] + ' ' + r['id']).casefold()]
        box = self.query_one('#history-list', OptionList)
        box.clear_options()
        box.add_options([Option(Text(f"{plain(r['title'])}\n{r['id']} · {plain(r['status'])} · {r['time']}"), id=r['id']) for r in matches])
        box.highlighted = 0 if matches else None
        self.query_one('#history-status', Static).update(f'{len(matches)} conversations · choose one or press Escape')

    def on_input_changed(self, event: Input.Changed):
        if event.input.id == 'history-search':
            self.filter_rows()

    def on_option_list_option_selected(self, event: OptionList.OptionSelected):
        if not self.opening:
            self.open_selected(event.option.id)

    @work(exclusive=True)
    async def open_selected(self, name):
        self.opening = True
        self.query_one('#history-status', Static).update('Checking the selected session...')
        try:
            selected = await asyncio.to_thread(self.open_session, name)
            self.dismiss(selected)
        except Exception as exc:
            self.query_one('#history-status', Static).update(Text(str(exc)))
        finally:
            self.opening = False

    def action_cancel(self):
        if not self.opening:
            self.dismiss(None)
