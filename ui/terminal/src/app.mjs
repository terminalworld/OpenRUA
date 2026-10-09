import {CombinedAutocompleteProvider, Container, Editor, ProcessTerminal, SelectList, Spacer, Text,
  TuiMainScreen, matchesKey} from '@earendil-works/pi-tui';
import {Controller} from './controller.mjs';
import {Transcript, accent, bold, editorTheme, green, muted, plain, red, selectTheme, yellow} from './view.mjs';
import {browseWorkspace} from './workspace.mjs';
import {answerQuestion} from './questions.mjs';

const commands = [
  {name: 'help', description: 'Keyboard shortcuts and commands'},
  {name: 'tools', description: 'Expand or collapse a tool result'},
  {name: 'files', description: 'Browse saved workspace files and images'},
  {name: 'queue', description: 'Inspect, edit or withdraw queued messages'},
  {name: 'questions', description: 'Answer pending agent questions'},
  {name: 'resume', description: 'Find an existing conversation'},
  {name: 'end', description: 'End execution while retaining the workspace and history'},
  {name: 'interrupt', description: 'Interrupt the current turn and pause the queue'},
  {name: 'continue', description: 'Confirm continuation of the paused queue'},
  {name: 'retry', description: 'Retry an unconfirmed send with the same request ID'},
  {name: 'quit', description: 'Detach; the session keeps running'},
];

const KEYS = 'Enter send · Ctrl+J newline · / commands · Esc interrupt · Ctrl+D detach';
const SPINNER = ['✻', '✢', '✳', '✶', '✳', '✢'];

const elapsed = since => {
  const seconds = Math.max(0, Math.round((Date.now() - since) / 1000));
  return seconds < 60 ? `${seconds}s` : `${Math.floor(seconds / 60)}m${String(seconds % 60).padStart(2, '0')}s`;
};

// The first lines of the screen: the product, the session, and the robot it is
// on, as recorded when the session started (nothing when nothing was recorded).
export function header(name, facts = {}) {
  const lines = [`${bold('OpenRUA')}${name ? muted(' · ' + plain(name)) : ''}`];
  const where = [facts.robot && (facts.robot_model ? `${facts.robot} (${facts.robot_model})` : facts.robot),
    facts.backend === 'real' ? 'real robot' : facts.simulator && `on ${facts.simulator}`].filter(Boolean).join(' ');
  const scene = [facts.benchmark, facts.suite && (facts.task_id != null ? `${facts.suite} #${facts.task_id}` : facts.suite)].filter(Boolean).join(' / ');
  const agent = facts.agent && (facts.model ? `${facts.agent} (${facts.model})` : facts.agent);
  const parts = [where, scene, agent].filter(Boolean).map(plain);
  if (parts.length) lines.push(muted(parts.join(' · ')));
  if (facts.task) lines.push(muted('task: ') + plain(facts.task));
  return lines.join('\n');
}

// The component composition follows Pi's chat-simple example and interactive UI:
// transcript, status, editor, completion; application actions stay callbacks.
export class Chat {
  constructor(client, {terminal = new ProcessTerminal(), pollMs = 350, history = false, name = '', facts = {}} = {}) {
    this.client = client;
    this.history = history; this.result = {action: 'quit'};
    this.ui = new TuiMainScreen(terminal);
    this.transcript = new Transcript();
    this.controller = new Controller(client, this.transcript);
    this.status = new Text('Connecting…', 0, 0);
    this.notice = new Text('', 0, 0);
    this.editor = new Editor(this.ui, editorTheme);
    // Only slash commands: do not complete unrelated host files outside the workspace.
    const completion = new CombinedAutocompleteProvider(commands, process.cwd());
    this.editor.setAutocompleteProvider({
      getSuggestions: (lines, row, col, options) => lines.length === 1 && /^\/[a-z]*$/.test(lines[0])
        ? completion.getSuggestions(lines, row, col, options) : Promise.resolve(null),
      applyCompletion: (...args) => completion.applyCompletion(...args),
    });
    this.pollMs = pollMs;
    this.done = false;
    this.busy = false;
    this.panels = [];
    this.ui.addChild(new Text(header(name, facts), 0, 0));
    this.ui.addChild(this.transcript);
    this.ui.addChild(new Spacer(1)); this.ui.addChild(this.status);
    this.ui.addChild(this.notice); this.ui.addChild(this.editor);
    this.keys = new Text(muted(KEYS), 0, 0);
    this.ui.addChild(this.keys);
    this.started = null;
    this.frame = 0;
    this.editor.onSubmit = text => { void this.submit(text); };
    this.ui.addInputListener(data => {
      if (this.panel) return;
      if (matchesKey(data, 'ctrl+d') && !this.editor.getText()) { this.stop(); return {consume: true}; }
      if (matchesKey(data, 'ctrl+c')) {
        if (this.editor.getText()) this.editor.setText(''); else this.stop();
        return {consume: true};
      }
      if (matchesKey(data, 'escape')) { void this.submit('/interrupt'); return {consume: true}; }
    });
  }

  say(message) { if (!this.done) { this.notice.setText(plain(message)); this.ui.requestRender(); } }

  // A dialog takes the editor's place below the transcript, the way a coding
  // agent's own menus do: nothing is drawn over the conversation, and while one
  // is open Escape closes it instead of meaning interrupt. Dialogs stack: a
  // menu that opens an editor gets its place back when the editor closes.
  get panel() { return this.panels.at(-1) ?? null; }

  dialog(component) {
    const slot = replacement => {
      const at = this.ui.children.indexOf(this.panel?.component ?? this.editor);
      if (at !== -1) this.ui.children.splice(at, 1, replacement);
    };
    slot(component);
    const handle = {component, hide: () => {
      const index = this.panels.indexOf(handle);
      if (index === -1) return;
      const top = index === this.panels.length - 1;
      this.panels.splice(index, 1);
      if (top) {
        const next = this.panel?.component ?? this.editor;
        const at = this.ui.children.indexOf(component);
        if (at !== -1) this.ui.children.splice(at, 1, next);
        this.ui.setFocus(next);
      }
      this.ui.requestRender();
    }};
    this.panels.push(handle);
    this.ui.setFocus(component); this.ui.requestRender();
    return handle;
  }

  rows() { return this.ui.terminal?.rows ?? 24; }

  picker(title, items, select) {
    if (!items.length) { this.say('Nothing to show.'); return; }
    const menu = new SelectList(items, 8, selectTheme);
    const box = new Container();
    box.addChild(new Text(plain(title), 0, 1)); box.addChild(menu);
    box.addChild(new Text(muted('↑/↓ select · Enter confirm · Esc back'), 0, 1));
    box.handleInput = data => menu.handleInput(data);
    const handle = this.dialog(box);
    menu.onCancel = () => handle.hide();
    menu.onSelect = item => {
      handle.hide();
      Promise.resolve().then(() => select(item.value)).catch(error => this.say(error.message));
    };
  }

  confirm(title, action) {
    this.picker(title, [{value: 'cancel', label: 'Cancel'}, {value: 'confirm', label: 'Confirm'}],
      async value => { if (value === 'confirm') { await action(); if (!this.done) await this.refresh(); } });
  }

  editText(title, initial, apply) {
    const editor = new Editor(this.ui, editorTheme);
    editor.setText(initial);
    const box = new Container(); box.addChild(new Text(plain(title), 0, 1)); box.addChild(editor);
    box.addChild(new Text(muted('Enter submit · Ctrl+J newline · Esc cancel'), 0, 1));
    Object.defineProperty(box, 'focused', {get: () => editor.focused, set: value => { editor.focused = value; }});
    box.handleInput = data => {
      if (matchesKey(data, 'escape')) handle.hide(); else editor.handleInput(data);
    };
    const handle = this.dialog(box);
    editor.onSubmit = text => {
      if (!text.trim()) return;
      handle.hide();
      Promise.resolve().then(() => apply(text)).catch(error => this.say(error.message));
    };
  }

  queue() {
    const messages = this.controller.state?.messages.filter(m => m.status === 'queued') ?? [];
    this.picker('Queued instructions', messages.map(m => ({value: m.id, label: plain(m.text)})), id => {
      const message = messages.find(m => m.id === id);
      if (this.client.readOnly) { this.say(message.text); return; }
      this.picker(plain(message.text), [{value: 'back', label: 'Back'}, {value: 'edit', label: 'Edit'},
        {value: 'withdraw', label: 'Withdraw this queued message'}], async action => {
        if (action === 'edit') this.editText('Edit queued instruction', message.text, async text => {
          await this.client.command('edit', {message_id: id, revision: message.revision, text}); await this.refresh();
        });
        if (action === 'withdraw') this.confirm('Withdraw this queued instruction?', async () => {
          await this.client.command('withdraw', {message_id: id});
        });
      });
    });
  }

  questions() {
    if (this.client.readOnly) throw new Error('This is read-only history.');
    const pending = Object.values(this.controller.state?.requests ?? {}).filter(r => r.status === 'pending');
    this.picker('Pending agent questions', pending.map(r => ({value: r.request_id,
      label: plain(r.questions.map(q => q.text).join(' / '))})), id => {
      const request = pending.find(r => r.request_id === id);
      if (request.questions.some(q => q.secret)) {
        this.say('This request contains secret input. Answer it through the browser or existing CLI.'); return;
      }
      const answers = {};
      const next = index => {
        if (index === request.questions.length) {
          this.confirm('Send these answers to the agent?', () => this.client.command('respond', {request_id: id, answers}));
          return;
        }
        const question = request.questions[index];
        answerQuestion(this, question, values => { answers[question.id] = values; next(index + 1); });
      };
      next(0);
    });
  }

  async refresh() {
    await this.controller.refresh();
    if (this.done) return;
    const state = this.controller.state;
    const queued = state.messages.filter(message => message.status === 'queued').length;
    const requests = Object.values(state.requests).filter(r => r.status === 'pending').length;
    if (state.active && !this.started) this.started = Date.now();
    if (!state.active) this.started = null;
    const extra = (queued ? [`${queued} queued`] : [])
      .concat(state.connected || state.closed || this.client.readOnly ? [] : [yellow('agent disconnected')])
      .concat(requests ? [yellow(`${requests} question(s): /questions`)] : [])
      .concat(this.controller.pending ? [yellow('send unconfirmed: /retry')] : []);
    let head;
    if (this.client.readOnly) head = muted('◇ Read-only history');
    else if (state.closed) head = muted('◇ Closed');
    else if (state.paused) head = yellow('‖ Paused · /queue to review, /continue to go on');
    else if (state.active) head = accent(`${SPINNER[this.frame++ % SPINNER.length]} Working… ${elapsed(this.started)}`);
    else head = green('✓ Ready');
    this.status.setText([head, ...extra].join(muted(' · ')));
    this.keys.setText(muted(KEYS));
    this.ui.requestRender();
  }

  async poll() {
    try { await this.refresh(); } catch (error) {
      if (!this.done) {
        this.status.setText(yellow(`⚠ Disconnected · retrying · ${plain(error.message)}`));
        this.keys.setText(yellow('The service is not answering; your draft and queued instructions are kept.'));
        this.ui.requestRender();
      }
    }
    if (!this.done) this.timer = setTimeout(() => { void this.poll(); }, this.pollMs);
  }

  async submit(text) {
    if (!text.trim() || this.done) return;
    if (this.busy) {
      this.editor.setText(text);
      this.say('A request is still pending; this draft has not been sent.');
      return;
    }
    this.busy = true;
    try {
      if (text.startsWith('/')) {
        const state = this.controller.state;
        switch (text.trim()) {
          case '/help':
            this.say(commands.map(c => `/${c.name}: ${c.description}`).join('\n')); break;
          case '/files': await browseWorkspace(this); break;
          case '/quit': this.stop(); break;
          case '/resume':
            if (!this.history) throw new Error('Use openrua --resume to select history.');
            if (this.controller.pending) throw new Error('Resolve the unconfirmed send before changing conversations.');
            this.result = {action: 'history'}; this.stop(); break;
          case '/end':
            if (this.client.readOnly) throw new Error('This is read-only history.');
            this.confirm('End execution and release resources? The workspace and history will be retained.',
              async () => { await this.client.end(); this.stop(); }); break;
          case '/questions': this.questions(); break;
          case '/retry': await this.controller.retry(); await this.refresh(); break;
          case '/tools':
            this.picker('Tool results', [...this.transcript.tools].map(([key, tool]) => ({value: key,
              label: `${tool.expanded ? '▾' : '▸'} ${tool.label}`, description: `${tool.phase}${tool.details ? ' · ' + plain(String(tool.details.command ?? tool.details.file_path ?? '')).split('\n')[0] : ''}`})),
              key => { this.transcript.toggleTool(key); this.ui.requestRender(); }); break;
          case '/queue':
            this.queue(); break;
          case '/interrupt': {
            const id = state?.active;
            if (!id) throw new Error('No active turn.');
            this.confirm('Interrupt this turn and pause the shared queue?', () => this.client.command('interrupt', {message_id: id})); break;
          }
          case '/continue': {
            const id = state?.pause_id;
            if (!id) throw new Error('The queue is not paused.');
            this.confirm('Review /queue first. Continue this paused queue?', () => this.client.command('resume', {pause_id: id})); break;
          }
          default: throw new Error('Unknown command. Use /help.');
        }
      } else {
        await this.controller.send(text);
        this.editor.addToHistory(text);
        this.say('');
        await this.refresh();
      }
    } catch (error) {
      this.say(error.message);
      if (!text.startsWith('/') && !this.editor.getText() &&
          (!this.controller.pending || this.controller.pending.text !== text)) this.editor.setText(text);
    } finally { this.busy = false; }
  }

  async run() {
    await this.refresh();
    if (this.done) return this.result;
    this.ui.setFocus(this.editor);
    this.ui.start();
    await new Promise(resolve => { this.finished = resolve; void this.poll(); });
    return this.result;
  }

  stop() {
    if (this.done) return;
    this.done = true;
    clearTimeout(this.timer);
    this.ui.stop();
    this.client.close();
    this.finished?.();
  }
}
