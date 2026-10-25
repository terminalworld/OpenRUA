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
    this.agentStep = spec.agent_step ?? spec.view === 'agent-step';
    this.ui.setHeading('OpenRUA · Setup', spec.project || spec.location);
    this.body = new Container(); this.ui.addChild(this.body);
    this.notice = new Text(plain(spec.notice), 0, 0);
    if (spec.notice) this.ui.addChild(this.notice);
    this.hints = new Text('', 0, 0); this.ui.setFooter(this.hints);
    const view = spec.view || ((this.values.robot || this.values.bench) ? 'review' : 'environment');
    this.show(view);
  }
  show(view) {
    if (view === 'models') this.models();
    else if (view === 'agent' || view === 'agent-step') this.agentSettings();
    else if (view === 'choose-agent') this.chooseAgent();
    else if (view === 'environment') this.environment();
    else this.menu();
  }
  submit(action) {
    this.finish({action, values: {...this.values}, intent: this.intent, view: 'review', agent_step: this.agentStep});
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
      if (value === 'new') { this.intent = 'connect'; this.chooseAgent(); }
      else this.edit('robot', 'Robot configuration path', true, () => this.real(), () => this.chooseAgent());
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
            ? () => this.edit('robot', 'Robot type or configuration', true, () => this.environments(false), () => this.chooseAgent())
            : () => this.chooseAgent());
      } else { Object.assign(this.values, rows[Number(value)].selection); this.chooseAgent(); }
    }, () => this.simulation());
  }
  menu() {
    this.agentStep = false;
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
  chooseAgent(step = true) {
    this.agentStep = step;
    const names = [...(this.spec.choices.agent || [])];
    if (this.values.agent && !names.includes(this.values.agent)) names.unshift(this.values.agent);
    const items = names.map(value => {
      const facts = this.authentication[value] || {};
      return {value, label: `${this.values.agent === value ? '✓ ' : ''}${plain(facts.title || value)}`,
        description: [facts.version, facts.model || this.spec.models?.[value]].filter(Boolean).map(plain).join(' · ')};
    });
    items.push({value: '\0custom', label: 'Other agent or manifest…'});
    const back = () => !step ? this.agentSettings() : this.intent === 'connect' || !(this.values.sim || this.values.bench)
      ? this.real() : this.environments(Boolean(this.values.bench));
    this.list('Agent', 'Choose your coding agent', items, value => {
      this.returnTo = () => this.agentSettings();
      if (value === '\0custom') this.edit('agent', 'Coding agent name or manifest path', true, () => this.chooseAgent(step));
      else this.update('agent', value);
    }, back, {selected: this.values.agent});
  }
  agentSettings() {
    const rows = [...(this.agentStep ? [{value: 'continue', label: 'Continue to review'}] : []),
      {value: 'agent', label: `Coding agent: ${plain(this.authentication[this.values.agent]?.title || this.values.agent || 'Configured default')}`},
      {value: 'model', label: `Model: ${plain(this.values.model || 'Native default')}`}];
    if (Object.hasOwn(this.values, 'auth_mode')) rows.push({value: 'auth_mode', label: `Authentication: ${this.authLabel(this.values.auth_mode)}`});
    if (this.values.auth_mode === 'api') rows.push({value: 'api_key_file', label: `API key file: ${plain(this.values.api_key_file || 'Not selected')}`});
    rows.push({value: 'back', label: 'Back'});
    this.list('Agent', 'Agent settings', rows, value => value === 'continue' ? this.menu()
      : value === 'back' ? this.agentStep ? this.chooseAgent() : this.menu()
      : value === 'agent' ? this.chooseAgent(this.agentStep)
      : value === 'model' ? this.authentication[this.values.agent]?.model_selection === false
        ? this.models() : this.submit('models') : this.edit(value,
      {agent: 'Coding agent', model: 'Model ID', auth_mode: 'Authentication', api_key_file: 'API key file path'}[value], false,
      () => this.agentSettings()), () => this.agentStep ? this.chooseAgent() : this.menu());
  }
  models() {
    const selectionAllowed = this.authentication[this.values.agent]?.model_selection !== false;
    const rows = (this.spec.model_choices || []).map(row => ({value: row.value,
      label: plain(row.label || row.value), description: plain(row.description || '')}));
    this.list('Agent › Model', 'Select Model', [
      ...rows,
      ...(selectionAllowed ? [{value: '\0reload', label: 'Reload native models'},
        {value: '\0custom', label: 'Enter a model ID…'}] : []),
      {value: '\0back', label: 'Back'},
    ], value => {
      if (value === '\0back') this.agentSettings();
      else if (value === '\0reload') this.submit('models');
      else if (value === '\0custom') this.edit('model', 'Model ID', true, () => this.models(), () => this.agentSettings());
      else { this.values.model = value; this.agentSettings(); }
    }, () => this.agentSettings(), {selected: this.values.model,
      summary: [muted(`Current model: ${plain(this.values.model || 'Native default')}`),
        ...(!selectionAllowed ? [muted('This preview plugin uses the model configured in its agent manifest.')] : [])]});
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
      this.finish({action: 'agent', agent: value.trim(), values: {...this.values},
        view: this.agentStep ? 'agent-step' : 'agent', intent: this.intent}); return;
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
  authLabel(mode) {
    return mode === 'api' ? 'API key (billed separately)'
      : `${plain(this.authentication[this.values.agent]?.title || 'Agent')} account`;
  }
  edit(key, label, custom = false, back = () => this.menu(), after) {
    this.returnTo = after || (['agent', 'model', 'auth_mode', 'api_key_file'].includes(key) ? () => this.agentSettings() : () => this.menu());
    this.customEdit = custom;
    const choices = custom ? [] : this.options(key);
    if (choices.length) {
      this.list(label, label, [
        ...choices.map(value => ({value, label: key === 'auth_mode' ? this.authLabel(value) : plain(value) || 'Native scene'})),
        ...(!['auth_mode', 'robot', 'sim', 'bench'].includes(key) ? [{value: '\0custom', label: 'Enter a name or profile path…'}] : []),
      ], value => value === '\0custom' ? this.edit(key, label, true, back) : this.update(key, value), back,
      {selected: this.values[key], summary: key === 'auth_mode' && this.authentication[this.values.agent]?.native_hint
        ? [muted(plain(this.authentication[this.values.agent].native_hint))] : []});
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
