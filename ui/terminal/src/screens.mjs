import {Container, Input, ProcessTerminal, SelectList, Text, TuiMainScreen, matchesKey} from '@earendil-works/pi-tui';
import {plain, selectTheme} from './view.mjs';

export class Screen {
  constructor(terminal = new ProcessTerminal()) {
    this.ui = new TuiMainScreen(terminal);
    this.result = {action: 'quit'};
    this.done = false;
    this.ui.addChild(new Text('\x1b[1mOpenRUA\x1b[22m', 0, 1));
    this.ui.addInputListener(data => {
      if (matchesKey(data, 'ctrl+c') || matchesKey(data, 'ctrl+d')) {
        this.finish(); return {consume: true};
      }
    });
  }
  finish(result = {action: 'quit'}) {
    if (this.done) return;
    this.done = true; this.result = result; this.ui.stop(); this.resolve?.(result);
  }
  stop() { this.finish(); }
  async run() {
    if (this.done) return this.result;
    return new Promise(resolve => { this.resolve = resolve; this.ui.start(); });
  }
}

export class Setup extends Screen {
  constructor(spec, terminal) {
    super(terminal);
    this.values = {...spec.values}; this.spec = spec;
    this.ui.addChild(new Text(`Configure a new robot conversation\n${plain(spec.location)}\nSettings also remain editable through openrua config set.`, 0, 1));
    this.body = new Container(); this.ui.addChild(this.body);
    this.ui.addChild(new Text(plain(spec.notice), 0, 1));
    this.ui.addChild(new Text('↑/↓ select · Enter edit or activate · Esc back · Ctrl+D quit', 0, 1));
    this.menu();
  }
  menu() {
    this.body.clear();
    const labels = {robot: 'Robot', sim: 'Simulator (blank for real robot)', bench: 'Benchmark (optional)',
      agent: 'Coding agent', model: 'Model (blank for default)', name: 'Session ID / optional name'};
    const items = Object.entries(labels).map(([key, label]) => ({value: key,
      label: `${label}: ${plain(this.values[key]) || '(none)'}`}));
    items.push({value: 'start', label: 'Save and start'}, {value: 'check', label: 'Save and check preparation'},
      {value: 'history', label: '/resume: find an existing conversation'}, {value: 'quit', label: 'Quit'});
    const menu = new SelectList(items, 10, selectTheme);
    menu.onCancel = () => this.finish();
    menu.onSelect = item => {
      if (Object.hasOwn(labels, item.value)) this.edit(item.value, labels[item.value]);
      else this.finish({action: item.value, values: this.values});
    };
    this.body.addChild(menu); this.ui.setFocus(menu); this.ui.requestRender();
  }
  update(key, value) {
    if (key === 'agent' && value.trim() !== this.values.agent) this.values.model = this.spec.models[value.trim()] ?? '';
    this.values[key] = value.trim(); this.menu();
  }
  edit(key, label, custom = false) {
    this.body.clear();
    const choices = this.spec.choices[key] ?? [];
    this.body.addChild(new Text(label, 0, 1));
    if (choices.length && !custom) {
      const menu = new SelectList([
        ...choices.map(value => ({value, label: plain(value)})),
        {value: '\0custom', label: 'Enter a name or profile path'},
        {value: '', label: '(none / configured default)'},
      ], 10, selectTheme);
      menu.setSelectedIndex(Math.max(0, choices.indexOf(this.values[key])));
      menu.onCancel = () => this.menu();
      menu.onSelect = item => item.value === '\0custom' ? this.edit(key, label, true) : this.update(key, item.value);
      this.body.addChild(menu); this.ui.setFocus(menu);
    } else {
      const input = new Input({prompt: '> '}); input.setValue(this.values[key]);
      input.onEscape = () => this.menu();
      input.onSubmit = value => this.update(key, value);
      this.body.addChild(input); this.ui.setFocus(input);
    }
    this.ui.requestRender();
  }

}

export class History extends Screen {
  constructor(spec, terminal) {
    super(terminal);
    this.ui.addChild(new Text('Resume a conversation\nEnded or unavailable sessions open read-only.', 0, 1));
    this.input = new Input({prompt: 'Search: '});
    this.rows = spec.rows;
    this.listContainer = new Container();
    this.ui.addChild(this.input); this.ui.addChild(this.listContainer);
    this.filter();
    this.ui.addChild(new Text(plain(spec.notice), 0, 1));
    this.ui.addChild(new Text('Type to filter · ↑/↓ select · Enter open · Esc cancel', 0, 1));
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
      label: `${plain(row.title)} · ${plain(row.id)}`, description: `${plain(row.status)} · ${plain(row.time)}`})), 10, selectTheme);
    this.list.onSelect = item => this.finish({action: 'select', id: item.value});
    this.list.onCancel = () => this.finish();
    this.listContainer.clear(); this.listContainer.addChild(this.list);
  }

}

export class ArchiveClient {
  constructor(data) { this.data = data; this.readOnly = true; this.reason = data.reason; }
  async snapshot() { return this.data.snapshot; }
  async events(after = 0) { return this.data.events.filter(event => event.seq > after).slice(0, 1000); }
  async command() { throw new Error('Read-only history; no execution has been restarted.'); }
  async end() { return this.command(); }
  close() {}
}
