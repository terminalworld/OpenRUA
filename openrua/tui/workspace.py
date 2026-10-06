"""Read-only keyboard workspace browser over the existing session client."""
from __future__ import annotations

import asyncio

from rich.text import Text
from textual import work
from textual.containers import Vertical
from textual.screen import ModalScreen
from textual.widgets import OptionList, Static, TextArea
from textual.widgets.option_list import Option


def literal(value):
    return Text(''.join(c for c in str(value) if c in '\n\t' or c.isprintable()))


class Workspace(ModalScreen):
    BINDINGS = [('escape', 'back', 'Back'), ('r', 'refresh', 'Refresh')]
    DEFAULT_CSS = '''
    Workspace .workspace-dialog { width: 90%; height: 85%; padding: 1 2;
        border: round $primary; background: $surface; }
    Workspace #workspace-list { height: 1fr; }
    Workspace #workspace-preview { height: 1fr; }
    Workspace #workspace-notice { height: auto; }
    '''

    def __init__(self, client):
        super().__init__()
        self.client = client
        self.path = ''
        self.file = None
        self.entries = []
        self.loading = False

    def compose(self):
        with Vertical(classes='workspace-dialog'):
            yield Static('Workspace /', id='workspace-title')
            yield Static('', id='workspace-notice')
            yield OptionList(id='workspace-list')
            yield TextArea(read_only=True, id='workspace-preview')
            yield Static('Up/Down select · Enter open · r refresh · Esc back')

    def on_mount(self):
        self.query_one('#workspace-preview').display = False
        self.load_directory()

    @work(exclusive=True, group='workspace')
    async def load_directory(self):
        self.loading = True
        try:
            listing = await asyncio.to_thread(self.client.workspace_list, self.path)
            self.file = None
            self.entries = ([{'name': '..', 'kind': 'parent'}] if self.path else []) + listing['entries']
            menu = self.query_one(OptionList)
            menu.clear_options()
            menu.add_options(Option(literal(e['name'] + ('/' if e['kind'] == 'directory' else '')),
                                    disabled=e['kind'] not in {'parent', 'file', 'directory'}) for e in self.entries)
            menu.highlighted = next((i for i, entry in enumerate(self.entries)
                                     if entry['kind'] in {'parent', 'file', 'directory'}), None)
            self.query_one('#workspace-title', Static).update(literal('Workspace /' + self.path))
            self.query_one('#workspace-notice', Static).update('Listing truncated; select a subdirectory.' if listing['truncated'] else '')
            self.query_one(TextArea).display = False
            menu.display = True
            menu.focus()
        except (OSError, RuntimeError, ValueError) as exc:
            self.query_one('#workspace-notice', Static).update(literal(str(exc)))
        finally:
            self.loading = False

    @work(exclusive=True, group='workspace')
    async def load_file(self, path):
        self.loading = True
        try:
            file = await asyncio.to_thread(self.client.workspace_read, path)
            self.file = path
            preview = self.query_one(TextArea)
            text = file.get('text', '')[:131072]
            if file['kind'] != 'text':
                text = f"Saved {file['kind']} ({file['mime']}, {file['size']} bytes).\n" \
                       'Use the session browser to view or download it.\n' \
                       'Pi also previews images when supported by the terminal.'
            elif len(file['text']) > 131072:
                text += '\n[Preview limited to 131,072 characters.]'
            preview.load_text(literal(text).plain)
            self.query_one('#workspace-title', Static).update(literal(path))
            self.query_one('#workspace-notice', Static).update('Saved workspace file, not a live sensor stream.')
            self.query_one(OptionList).display = False
            preview.display = True
            preview.focus()
        except (OSError, RuntimeError, ValueError) as exc:
            self.query_one('#workspace-notice', Static).update(literal(str(exc)))
        finally:
            self.loading = False

    def on_option_list_option_selected(self, event):
        event.stop()
        if self.loading:
            return
        entry = self.entries[event.option_index]
        if entry['kind'] == 'parent':
            self.path = self.path.rpartition('/')[0]
            self.load_directory()
        else:
            target = f"{self.path}/{entry['name']}" if self.path else entry['name']
            if entry['kind'] == 'directory':
                self.path = target
                self.load_directory()
            elif entry['kind'] == 'file':
                self.load_file(target)

    def action_refresh(self):
        self.load_file(self.file) if self.file else self.load_directory()

    def action_back(self):
        if self.file:
            self.load_directory()
        elif self.path:
            self.path = self.path.rpartition('/')[0]
            self.load_directory()
        else:
            self.dismiss()
