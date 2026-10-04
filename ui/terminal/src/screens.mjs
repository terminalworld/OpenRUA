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
    this.ui.addChild(new Text(`Configure a new robot conversation\n${plain(spec.location)}\nTested combinations only; dependent fields update together.\nPrepare and start installs missing images automatically.\nCustom profiles remain available through the CLI.`, 0, 1));
    this.body = new Container(); this.ui.addChild(this.body);
    this.notice = new Text(plain(spec.notice), 0, 1); this.ui.addChild(this.notice);
    this.ui.addChild(new Text('↑/↓ select · Enter edit or activate · Esc back · Ctrl+D quit', 0, 1));
    this.menu();
  }
  menu() {
    this.body.clear();
    const labels = {robot: 'Robot', sim: 'Simulator', bench: 'Benchmark / scene',
      agent: 'Coding agent', model: 'Model (blank for default)', name: 'Session ID / optional name'};
    const items = Object.entries(labels).map(([key, label]) => ({value: key,
      label: `${label}: ${plain(this.values[key]) || '(none)'}`}));
    items.push({value: 'start', label: 'Prepare and start'}, {value: 'check', label: 'Save and check preparation'},
      {value: 'history', label: '/resume: find an existing conversation'}, {value: 'quit', label: 'Quit'});
    const menu = new SelectList(items, 10, selectTheme);
    menu.onCancel = () => this.finish();
    menu.onSelect = item => {
      if (Object.hasOwn(labels, item.value)) this.edit(item.value, labels[item.value]);
      else if (['start', 'check'].includes(item.value) && this.spec.environments != null &&
          !this.spec.environments.some(row => ['robot', 'sim', 'bench'].every(key => row.selection[key] === this.values[key]))) {
        this.notice.setText('Choose a tested robot, simulator and benchmark combination first. Custom profiles remain available through the CLI.');
        this.ui.requestRender();
      } else this.finish({action: item.value, values: this.values});
    };
    this.body.addChild(menu); this.ui.setFocus(menu); this.ui.requestRender();
  }
  update(key, value) {
    if (key === 'agent' && value.trim() !== this.values.agent) this.values.model = this.spec.models[value.trim()] ?? '';
    this.values[key] = value.trim();
    const fields = ['robot', 'sim', 'bench'];
    if (this.spec.environments != null && fields.includes(key)) {
      for (const child of fields.slice(fields.indexOf(key) + 1)) {
        const available = this.options(child);
        if (!available.includes(this.values[child])) this.values[child] = available.length === 1 ? available[0] : '';
      }
    }
    this.menu();
  }
  options(key) {
    const fields = ['robot', 'sim', 'bench'];
    if (this.spec.environments == null || !fields.includes(key)) return this.spec.choices[key] ?? [];
    const before = fields.slice(0, fields.indexOf(key));
    return [...new Set(this.spec.environments.filter(row => before.every(k => !this.values[k] || this.values[k] === row.selection[k]))
      .map(row => row.selection[key]))].sort();
  }
  edit(key, label, custom = false) {
    this.body.clear();
    const choices = this.options(key);
    const guided = this.spec.environments != null && ['robot', 'sim', 'bench'].includes(key);
    this.body.addChild(new Text(label, 0, 1));
    if (guided && !choices.length) {
      this.notice.setText('No tested combination for this selection. Change the robot first; use explicit CLI options for custom profiles.');
      this.menu(); return;
    }
    if (choices.length && !custom) {
      const menu = new SelectList([
        ...choices.map(value => ({value, label: plain(value) || '(native scene)'})),
        ...(!guided ? [{value: '\0custom', label: 'Enter a name or profile path'},
        {value: '', label: '(none / configured default)'}] : []),
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
