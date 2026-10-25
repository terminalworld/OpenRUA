import {Container, Input, SelectList, Text, matchesKey} from '@earendil-works/pi-tui';
import {plain, selectTheme} from './view.mjs';
import {Client} from './client.mjs';
import {selection} from './selection.mjs';
import {Screen} from './surface.mjs';

export {Screen} from './surface.mjs';
export {Setup} from './setup.mjs';

export class Workspace extends Screen {
  constructor(spec, terminal, host) {
    super(terminal, host);
    this.path = spec.path;
    this.ui.setHeading('OpenRUA · Trust project', spec.path);
    this.ui.addChild(new Text(`Trust this project directory?\n${plain(spec.path)}\n\nYour agent can read, edit and run files here.\nYour project files and run history will be saved here.`, 0, 1));
    this.body = new Container(); this.ui.addChild(this.body);
    this.ui.addChild(new Text(plain(spec.notice), 0, 1));
    const menu = selection([
      {value: 'trust', label: 'Yes, use and trust this directory'},
      {value: 'choose', label: 'No, choose another directory'},
      {value: 'create', label: 'Create a project directory'},
      {value: 'quit', label: 'Quit'},
    ], 6, {explicit: true});
    this.ui.setFooter(new Text('↑/↓ Select · Enter Confirm · Esc Close', 0, 0));
    menu.onCancel = () => this.finish();
    menu.onSelect = item => {
      if (item.value === 'trust') this.finish({action: 'trust', path: spec.path});
      else if (item.value === 'quit') this.finish();
      else {
        this.body.clear();
        this.body.addChild(new Text(item.value === 'create' ? 'New directory path' : 'Directory path', 0, 1));
        const input = new Input({prompt: '> '});
        input.onEscape = () => this.finish({action: 'choose', path: spec.path});
        input.onSubmit = value => this.finish({action: 'choose', path: value.trim()});
        this.body.addChild(input); this.ui.setFocus(input); this.ui.requestRender();
      }
    };
    this.body.addChild(menu); this.ui.setFocus(menu);
  }
}

export class History extends Screen {
  constructor(spec, terminal, host) {
    super(terminal, host);
    this.ui.setHeading('OpenRUA · Resume', spec.scope ?? 'All projects');
    this.ui.addChild(new Text('Choose a conversation to open.'
      + (spec.can_all ? spec.scope === 'All projects' ? '\nTab: Show this project' : '\nTab: Show all projects' : ''), 0, 1));
    if (spec.can_all) this.ui.addInputListener(data => {
      if (matchesKey(data, 'tab')) { this.finish({action: 'scope'}); return {consume: true}; }
    });
    this.input = new Input({prompt: 'Search: '});
    this.rows = spec.rows;
    this.listContainer = new Container();
    this.ui.addChild(this.input); this.ui.addChild(this.listContainer);
    this.filter();
    this.ui.addChild(new Text(plain(spec.notice), 0, 1));
    this.ui.setFooter(new Text('Type to filter · ↑/↓ Select · Enter Open · Esc Back', 0, 0));
    const input = this.input;
    this.ui.setFocus({get focused() { return input.focused; }, set focused(value) { input.focused = value; }, handleInput: data => {
      if (['up', 'down', 'enter', 'escape'].some(key => matchesKey(data, key))) this.list.handleInput(data);
      else { this.input.handleInput(data); this.filter(); }
      this.ui.requestRender();
    }, render: () => []});
  }
  filter() {
    const query = this.input.getValue().toLocaleLowerCase();
    const rows = this.rows.filter(row => `${row.title} ${row.id}`.toLocaleLowerCase().includes(query));
    this.list = new SelectList(rows.map(row => ({value: row.id,
      label: `${plain(row.title)} · ${plain(row.id)}`,
      description: `${plain({'Ended': 'Stopped', 'Not answering': 'Cannot connect',
        'Execution status unknown': 'Cannot confirm if it is running'}[row.status] || row.status)} · ${plain(row.time)}${row.project ? ' · ' + plain(row.project) : ''}`})), Math.max(2, Math.min(10, this.ui.terminal.rows - 12)), selectTheme);
    this.list.onSelect = item => this.finish({action: 'select', id: item.value});
    this.list.onCancel = () => this.finish();
    this.listContainer.clear();
    if (!rows.length) this.listContainer.addChild(new Text(this.input.getValue()
      ? 'No conversations match your search.' : 'No saved conversations here yet.', 0, 1));
    this.listContainer.addChild(this.list);
  }

}

export class ArchiveClient {
  constructor(data) {
    this.data = data; this.readOnly = true; this.reason = data.reason;
    if (data.workspace) {
      this.files = new Client(data.workspace.url, data.workspace.token);
      this.workspaceList = path => this.files.workspaceList(path);
      this.workspaceRead = path => this.files.workspaceRead(path);
    }
  }
  async snapshot() { return this.data.snapshot; }
  async events(after = 0) { return this.data.events.filter(event => event.seq > after).slice(0, 1000); }
  async command() { throw new Error('You can view this conversation, but you cannot send messages here.'); }
  async end() { return this.command(); }
  close() { this.files?.close(); }
}
