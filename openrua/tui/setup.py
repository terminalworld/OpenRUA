"""Configuration form with injected save, readiness, and launch operations."""

from __future__ import annotations

import asyncio
from collections.abc import Callable

from rich.text import Text
from textual import work
from textual.app import App, ComposeResult
from textual.containers import Horizontal, VerticalScroll
from textual.suggester import SuggestFromList
from textual.widgets import Button, Footer, Input, Label, Static


class SetupApp(App):
    """Collect startup settings without importing configuration or resources."""

    TITLE = 'OpenRUA'
    BINDINGS = [('ctrl+q', 'leave', 'Quit')]
    CSS = '''
    Screen { background: $surface; }
    #form { width: 100%; max-width: 100; height: 1fr; padding: 1 2; }
    Label { margin-top: 1; }
    Input { height: 3; }
    #description, #result { height: auto; margin: 1 0; }
    #buttons { height: auto; min-height: 3; }
    Button { margin-right: 1; }
    '''

    def __init__(self, values: dict[str, str], choices: dict[str, list[str]],
                 location: str, save: Callable, check: Callable, launch: Callable,
                 agent_models: dict[str, str] | None = None):
        super().__init__()
        self.values, self.choices, self.location = values, choices, location
        self.save_settings, self.check_settings, self.launch_session = save, check, launch
        self.agent_models = agent_models or {}
        self.busy = False
        self.current_agent = values.get('agent', '')

    def compose(self) -> ComposeResult:
        with VerticalScroll(id='form'):
            yield Static('OpenRUA: start a robot conversation', id='title')
            yield Static(Text(f'Settings: {self.location}\n'
                              'You can also use openrua config set or edit this file later.\n'
                              'Tab completes suggested names. Profile paths are accepted.\n'
                              'Saving does not build images, log in, or start a robot.'), id='description')
            labels = {'robot': 'Robot (name or profile path)', 'sim': 'Simulator (blank for a real robot)',
                      'bench': 'Benchmark (blank for the native scene)', 'agent': 'Coding agent',
                      'model': 'Model (blank for the configured default)', 'name': 'Session name'}
            for key, label in labels.items():
                yield Label(label)
                options = self.choices.get(key, [])
                yield Input(self.values.get(key, ''), id=key,
                            placeholder=', '.join(options[:5]),
                            suggester=SuggestFromList(options, case_sensitive=False) if options else None)
            with Horizontal(id='buttons'):
                yield Button('Save & check', id='check')
                yield Button('Save & start', id='start', variant='primary')
                yield Button('Quit', id='quit')
            yield Static('', id='result')
        yield Footer()

    def on_input_changed(self, event: Input.Changed):
        if event.input.id == 'agent' and event.value != self.current_agent:
            self.current_agent = event.value
            self.query_one('#model', Input).value = self.agent_models.get(event.value, '')

    def action_leave(self):
        if self.busy:
            self.query_one('#result', Static).update('Wait for the current check or startup to finish.')
        else:
            self.exit(None)

    def on_button_pressed(self, event: Button.Pressed):
        if event.button.id == 'quit':
            self.action_leave()
        elif not self.busy:
            values = {key: self.query_one(f'#{key}', Input).value.strip() for key in self.values}
            self.perform(values, event.button.id == 'start')

    @work(exclusive=True)
    async def perform(self, values: dict, start: bool):
        self.busy = True
        for widget in self.query('Input, Button'):
            widget.disabled = True
        result = self.query_one('#result', Static)
        result.update('Saving settings and checking preparation...')
        try:
            await asyncio.to_thread(self.save_settings, values)
            ok, report = await asyncio.to_thread(self.check_settings, values)
            result.update(Text(report))
            self.call_after_refresh(result.scroll_visible, animate=False)
            if start and ok:
                result.update(Text(report + '\nStarting the session; this can take a few minutes...'))
                connected = await asyncio.to_thread(self.launch_session, values)
                self.exit((connected, values['name']))
        except Exception as exc:
            hint = getattr(exc, 'hint', '')
            result.update(Text(f'{exc}\n{hint}'))
            self.call_after_refresh(result.scroll_visible, animate=False)
        finally:
            self.busy = False
            for widget in self.query('Input, Button'):
                widget.disabled = False
