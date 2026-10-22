import {CombinedAutocompleteProvider, Container, Editor, ProcessTerminal, SelectList, Spacer, Text,
  TuiMainScreen, matchesKey} from '@earendil-works/pi-tui';
import {Controller} from './controller.mjs';
import {PERMISSIONS, agentCommands, commandsFor, notice, parseCommand, status, typedAnswer} from './session.mjs';
import {Transcript, Welcome, accent, bold, editorTheme, green, muted, plain, red, selectTheme, yellow} from './view.mjs';
import {browseWorkspace} from './workspace.mjs';
import {answerQuestion} from './questions.mjs';

const commands = commandsFor('terminal');

const KEYS = 'Enter send (joins a running turn) · Ctrl+J newline · / commands · Esc interrupt · Ctrl+D leave';
// The agent counts as quiet after this long without a new event: say so, so a
// slow model and a stalled one are not mistaken for each other silently.
const QUIET_MS = 60_000;
const SPINNER = ['✻', '✢', '✳', '✶', '✳', '✢'];

const elapsed = since => {
  const seconds = Math.max(0, Math.round((Date.now() - since) / 1000));
  return seconds < 60 ? `${seconds}s` : `${Math.floor(seconds / 60)}m${String(seconds % 60).padStart(2, '0')}s`;
};

// The component composition follows Pi's chat-simple example and interactive UI:
// transcript, status, editor, completion; application actions stay callbacks.
export class Chat {
  constructor(client, {terminal = new ProcessTerminal(), pollMs = 350, history = false, name = '', facts = {}} = {}) {
    this.client = client;
    this.history = history; this.result = {action: 'quit'};
    this.ui = new TuiMainScreen(terminal);
    this.facts = facts;
    this.transcript = new Transcript({workspace: facts.workspace});
    // Every event the agent produces marks it as not quiet (QUIET_MS).
    const record = this.transcript.event.bind(this.transcript);
    this.transcript.event = event => {
      this.lastEvent = Date.now(); record(event);
      // What an event says to the person (an interrupt's cancelled motions,
      // a setting the agent refused), as every client says it.
      const said = notice(event);
      if (said) this.say(said);
    };
    this.controller = new Controller(client, this.transcript);
    this.status = new Text('Connecting…', 0, 0);
    this.notice = new Text('', 0, 0);
    this.editor = new Editor(this.ui, editorTheme);
    // Only slash commands: do not complete unrelated host files outside the
    // workspace. The agent's own commands join ours once it has said them.
    const completion = () => new CombinedAutocompleteProvider(
      commands.concat(this.agentCommands()), process.cwd());
    this.editor.setAutocompleteProvider({
      getSuggestions: (lines, row, col, options) => lines.length === 1 && /^\/[\w:-]*$/.test(lines[0])
        ? completion().getSuggestions(lines, row, col, options) : Promise.resolve(null),
      applyCompletion: (...args) => completion().applyCompletion(...args),
    });
    this.pollMs = pollMs;
    this.done = false;
    this.busy = false;
    this.panels = [];
    this.welcome = new Welcome(name, facts, client.readOnly);
    this.ui.addChild(this.welcome);
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

  // While the agent waits on a question, what is typed is the answer, as in a
  // coding agent's own terminal; a question with several parts, or a secret,
  // goes through /questions.
  answerTyped(text) {
    const typed = typedAnswer(this.controller.state, text);
    if (!typed) return false;
    if (typed.kind === 'choose') { this.questions(); return true; }
    void this.client.command('respond', {request_id: typed.request_id, answers: typed.answers})
      .then(() => this.refresh()).catch(error => this.say(error.message));
    return true;
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
    if (this.welcome.seen === undefined) {
      // Coming back to a conversation that already has messages.
      this.welcome.seen = true; this.welcome.returning = state.messages.length > 0;
    }
    if (state.active && !this.started) this.started = Date.now();
    if (!state.active) this.started = null;
    const {head: said, notes} = status(state, {readOnly: this.client.readOnly});
    const tone = {plain: text => text, muted, good: green, warn: yellow, bad: red, busy: accent};
    // Connecting a robot: each step's check, as OpenRUA ran it on the host.
    const marks = {passed: green('✓'), failed: yellow('✗'), running: accent('…')};
    const extra = notes.map(note => note.kind === 'checks'
      ? note.checks.map(([step, check]) => `${step} ${marks[check.status] ?? ''}`).join(' ')
      : tone[note.tone](plain(note.text)));
    if (state.active && this.lastEvent && Date.now() - this.lastEvent > QUIET_MS)
      extra.splice(1, 0, yellow(`no output for ${elapsed(this.lastEvent)}`));
    if (this.controller.pending) extra.push(yellow('send unconfirmed: /retry'));
    const head = said.kind === 'working'
      ? accent(`${SPINNER[this.frame++ % SPINNER.length]} Working… ${elapsed(this.started)}`)
      : tone[said.tone]({readonly: '◇ ', closed: '◇ ', paused: '‖ ', ready: '✓ '}[said.kind] + plain(said.text));
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

  // The agent's own commands (its /compact, its skills...), as it reported
  // them; typed, they go to the agent as a message, as in its own terminal.
  agentCommands() { return agentCommands(this.controller.state); }

  async configure(setting, value) {
    await this.client.command('configure', {setting, value});
    await this.refresh();
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
      const command = parseCommand(text, this.controller.state);
      if (command?.kind === 'ours') {
        await this.handlers()[command.name](command.args);
      } else if (command?.kind === 'agent') {
        await this.controller.send(text.trim());
        this.editor.addToHistory(text);
        await this.refresh();
      } else if (command) {
        throw new Error('Unknown command. Use /help.');
      } else if (this.answerTyped(text)) {
        this.editor.addToHistory(text);
        this.say('');
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

  // One handler for each command the terminal offers (commandsFor('terminal')).
  handlers() {
    const state = this.controller.state;
    const writable = () => { if (this.client.readOnly) throw new Error('This is read-only history.'); };
    return {
      help: async () => {
        const agent = this.agentCommands();
        const theirs = agent.length ? ['', 'The agent\'s own:', ...agent.map(c => `/${c.name}: ${c.description}`)] : [];
        this.say(commands.map(c => `/${c.name}: ${c.description}`).concat(theirs).join('\n'));
      },
      permissions: async ([value]) => {
        writable();
        if (value) { await this.configure('permissions', value); return; }
        const now = state?.settings?.permissions;
        this.picker('Permissions', PERMISSIONS.map(p => ({value: p.value, label: `${now === p.value ? '✓ ' : '  '}${p.label}`})),
          choice => this.configure('permissions', choice).catch(error => this.say(error.message)));
      },
      model: async ([value]) => {
        if (value) {
          writable();
          await this.configure('model', value);
          this.say(`The model is ${value} from the next request.`); return;
        }
        this.say(`The model: ${plain(state?.settings?.model ?? 'unknown')}. Change it with /model <name>.`);
      },
      files: async () => { await browseWorkspace(this); },
      quit: async () => { this.stop(); },
      resume: async () => {
        if (!this.history) throw new Error('Use openrua --resume to select history.');
        if (this.controller.pending) throw new Error('Resolve the unconfirmed send before changing conversations.');
        this.result = {action: 'history'}; this.stop();
      },
      end: async () => {
        writable();
        this.confirm('End execution and release resources? The workspace and history will be retained.',
          async () => { await this.client.end(); this.stop(); });
      },
      questions: async () => { this.questions(); },
      retry: async () => { await this.controller.retry(); await this.refresh(); },
      tools: async () => {
        this.picker('Tool results', [...this.transcript.tools].map(([key, tool]) => ({value: key,
          label: `${tool.expanded ? '▾' : '▸'} ${tool.label}`, description: `${tool.phase}${tool.details ? ' · ' + plain(String(tool.details.command ?? tool.details.file_path ?? '')).split('\n')[0] : ''}`})),
          key => { this.transcript.toggleTool(key); this.ui.requestRender(); });
      },
      queue: async () => { this.queue(); },
      interrupt: async () => {
        // As in a coding agent's own terminal: stop now, no dialog; what is
        // queued goes next.
        const id = state?.active;
        if (!id) throw new Error('No active turn.');
        await this.client.command('interrupt', {message_id: id});
        this.say('Interrupted. Queued messages go next.');
        await this.refresh();
      },
      continue: async () => {
        const id = state?.pause_id;
        if (!id) throw new Error('The queue is not paused.');
        this.confirm('Review /queue first. Continue this paused queue?', () => this.client.command('resume', {pause_id: id}));
      },
    };
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
