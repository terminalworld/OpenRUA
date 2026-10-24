import {Container, Input, Text} from '@earendil-works/pi-tui';
import {Screen} from './surface.mjs';
import {bold, muted, plain, truncateToWidth} from './view.mjs';
import {selection} from './selection.mjs';

export class Setup extends Screen {
  constructor(spec, terminal, host) {
    super(terminal, host);
    this.spec = spec; this.values = {...spec.values};
    this.authentication = {...spec.authentication}; this.positions = new Map();
    this.intent = spec.intent || 'start';
    this.ui.setHeading('OpenRUA · Setup', spec.project || spec.location);
    this.body = new Container(); this.ui.addChild(this.body);
    this.notice = new Text(plain(spec.notice), 0, 0);
    if (spec.notice) this.ui.addChild(this.notice);
    this.hints = new Text('', 0, 0); this.ui.setFooter(this.hints);
    const view = spec.view || ((this.values.robot || this.values.bench) ? 'review' : 'environment');
    this.show(view);
  }
  show(view) {
    if (view === 'agent') this.agentSettings();
    else if (view === 'environment') this.environment();
    else this.menu();
  }
  submit(action) {
    this.finish({action, values: {...this.values}, intent: this.intent, view: 'review'});
  }
  list(path, title, items, select, back, {selected, summary = []} = {}) {
    this.body.clear(); this.view = path;
    this.body.addChild(new Text(muted(path), 0, 0));
    this.body.addChild(new Text(bold(title), 0, 0));
    for (const line of summary) this.body.addChild({render: width => [truncateToWidth(line, width)]});
    this.body.addChild(new Text('', 0, 0));
    const menu = selection(items, Math.max(3, Math.min(8, (this.ui.terminal.rows || 24) - 10)));
    const at = items.findIndex(item => item.value === (selected ?? this.positions.get(path)));
    if (at >= 0) menu.setSelectedIndex(at);
    menu.onCancel = () => { this.positions.set(path, menu.getSelectedItem()?.value); back(); };
    menu.onSelect = item => { this.positions.set(path, item.value); select(item.value); };
    this.body.addChild(menu);
    this.hints.setText(muted(`↑/↓ Select · Enter Confirm · Esc ${path === 'Environment' ? 'Close' : 'Back'} · Ctrl+D Quit`));
    this.ui.setFocus(menu); this.ui.requestRender();
  }
  environment() {
    this.list('Environment', 'Choose your environment', [
      {value: 'real', label: 'Real robot', description: 'Use a project configuration or connect a robot'},
      {value: 'simulation', label: 'Simulation', description: 'Explore a scene or work on a benchmark task'},
      {value: 'history', label: 'Resume a conversation…'},
    ], value => value === 'real' ? this.real() : value === 'simulation' ? this.simulation() : this.submit('history'),
    () => (this.values.robot || this.values.bench) ? this.menu() : this.finish());
  }
  real() {
    this.list('Environment › Real robot', 'Choose a connection', [
      {value: 'project', label: 'Use a project configuration…'},
      {value: 'new', label: 'Connect a new robot…'},
    ], value => {
      this.values.sim = ''; this.values.bench = '';
      if (value === 'new') { this.intent = 'connect'; this.menu(); }
      else this.edit('robot', 'Robot configuration path', true, () => this.real());
    }, () => this.environment());
  }
  simulation() {
    this.list('Environment › Simulation', 'Choose a simulation', [
      {value: 'scene', label: 'Native scene', description: 'Explore a simulated environment'},
      {value: 'benchmark', label: 'Benchmark environment', description: 'Work on a task; this does not start batch evaluation'},
    ], value => this.environments(value === 'benchmark'), () => this.environment());
  }
  environments(benchmark) {
    const rows = (this.spec.environments || []).filter(row => row.selection.sim && Boolean(row.selection.bench) === benchmark);
    const items = rows.map((row, index) => ({value: String(index),
      label: plain(benchmark ? row.selection.bench : `${row.selection.robot} / ${row.selection.sim}`),
      description: plain(row.label || '')}));
    items.push({value: 'custom', label: benchmark ? 'Use a benchmark configuration…' : 'Use a simulator configuration…'});
    this.list(`Environment › Simulation › ${benchmark ? 'Benchmark' : 'Scene'}`, 'Choose an environment', items, value => {
      this.intent = 'start';
      if (value === 'custom') {
        this.values.bench = ''; this.values.sim = '';
        if (benchmark) this.values.robot = '';
        this.edit(benchmark ? 'bench' : 'sim', benchmark ? 'Benchmark configuration path' : 'Simulator configuration path', true,
          () => this.environments(benchmark), !benchmark && !this.values.robot
            ? () => this.edit('robot', 'Robot type or configuration', true, () => this.environments(false)) : undefined);
      } else { Object.assign(this.values, rows[Number(value)].selection); this.menu(); }
    }, () => this.simulation());
  }
  menu() {
    const environment = this.intent === 'connect' ? 'Real robot / New connection'
      : this.values.bench || [this.values.robot, this.values.sim].filter(Boolean).join(' / ') || 'Not selected';
    this.list('Review', 'Review your project', [
      {value: 'start', label: this.intent === 'connect' ? 'Connect robot' : 'Save project and start'},
      {value: 'environment', label: 'Change environment…'},
      {value: 'agent', label: 'Agent settings…'},
      {value: 'advanced', label: 'Advanced options…'},
      {value: 'history', label: 'Resume a conversation…'},
    ], value => {
      if (value === 'environment') this.environment();
      else if (value === 'agent') this.agentSettings();
      else if (value === 'advanced') this.advanced();
      else if (value === 'history') this.submit('history');
      else this.submit(this.intent === 'connect' ? 'connect' : 'start');
    }, () => this.environment(), {summary: [bold('Environment'), `  ${plain(environment)}`, bold('Agent'),
      `  ${plain(this.values.agent || 'Configured default')} · ${plain(this.values.model || 'Default model')}`]});
  }
  agentSettings() {
    const rows = [{value: 'agent', label: `Coding agent: ${plain(this.values.agent || 'Configured default')}`},
      {value: 'model', label: `Model: ${plain(this.values.model || 'Native default')}`}];
    if (Object.hasOwn(this.values, 'auth_mode')) rows.push({value: 'auth_mode', label: `Authentication: ${this.authLabel(this.values.auth_mode)}`});
    if (this.values.auth_mode === 'api') rows.push({value: 'api_key_file', label: `API key file: ${plain(this.values.api_key_file || 'Not selected')}`});
    rows.push({value: 'back', label: 'Back'});
    this.list('Agent', 'Agent settings', rows, value => value === 'back' ? this.menu() : this.edit(value,
      {agent: 'Coding agent', model: 'Model ID', auth_mode: 'Authentication', api_key_file: 'API key file path'}[value], false,
      () => this.agentSettings()), () => this.menu());
  }
  advanced() {
    this.list('Advanced', 'Advanced options', [
      {value: 'name', label: `Session name: ${plain(this.values.name || 'Automatic')}`},
      ...(this.intent !== 'connect' ? [{value: 'check', label: 'Save project and check preparation'}] : []),
      {value: 'back', label: 'Back'},
    ], value => value === 'back' ? this.menu() : value === 'check' ? this.submit('check')
      : this.edit('name', 'Session name', false, () => this.advanced()), () => this.menu());
  }
  update(key, value) {
    if (key === 'agent' && value.trim() !== this.values.agent) {
      this.finish({action: 'agent', agent: value.trim(), values: {...this.values}, view: 'agent', intent: this.intent}); return;
    }
    this.values[key] = value.trim();
    if (key === 'auth_mode' && value === 'native') this.values.api_key_file = '';
    const fields = ['robot', 'sim', 'bench'];
    if (this.spec.environments != null && fields.includes(key) && !this.customEdit) {
      for (const child of fields.slice(fields.indexOf(key) + 1)) {
        const choices = this.options(child);
        if (!choices.includes(this.values[child])) this.values[child] = choices.length === 1 ? choices[0] : '';
      }
    }
    if (fields.includes(key)) this.intent = 'start';
    this.returnTo ? this.returnTo() : this.menu();
  }
  options(key) {
    if (key === 'auth_mode') return this.authentication[this.values.agent]?.api ? ['native', 'api'] : ['native'];
    const fields = ['robot', 'sim', 'bench'];
    if (this.spec.environments == null || !fields.includes(key)) return this.spec.choices[key] || [];
    const before = fields.slice(0, fields.indexOf(key));
    return [...new Set(this.spec.environments.filter(row => before.every(k => !this.values[k] || this.values[k] === row.selection[k]))
      .map(row => row.selection[key]))].sort();
  }
  authLabel(mode) { return mode === 'api' ? 'API key file (API billing)' : 'Native CLI login'; }
  edit(key, label, custom = false, back = () => this.menu(), after) {
    this.returnTo = after || (['agent', 'model', 'auth_mode', 'api_key_file'].includes(key) ? () => this.agentSettings() : () => this.menu());
    this.customEdit = custom;
    const choices = custom ? [] : this.options(key);
    if (choices.length) {
      this.list(label, label, [
        ...choices.map(value => ({value, label: key === 'auth_mode' ? this.authLabel(value) : plain(value) || 'Native scene'})),
        ...(!['auth_mode', 'robot', 'sim', 'bench'].includes(key) ? [{value: '\0custom', label: 'Enter a name or profile path…'}] : []),
      ], value => value === '\0custom' ? this.edit(key, label, true, back) : this.update(key, value), back, {selected: this.values[key]});
      return;
    }
    this.body.clear(); this.body.addChild(new Text(bold(label), 0, 1));
    if (key === 'api_key_file') this.body.addChild(new Text('Enter the path to the key file. Do not paste the key here.', 0, 1));
    const input = new Input({prompt: '› '}); input.setValue(custom ? '' : this.values[key] || '');
    input.onEscape = back; input.onSubmit = value => this.update(key, value);
    this.body.addChild(input); this.hints.setText(muted('Enter Confirm · Esc Back'));
    this.ui.setFocus(input); this.ui.requestRender();
  }
}
