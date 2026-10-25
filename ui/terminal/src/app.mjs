import {CombinedAutocompleteProvider, Container, Editor, ProcessTerminal, SelectList, Spacer, Text,
  isKeyRepeat, matchesKey} from '@earendil-works/pi-tui';
import {Controller} from './controller.mjs';
import {SettingChanges, agentCommands, commandsFor, notice, parseCommand, permissionChoices, status, typedAnswer} from './session.mjs';
import {DialogHeading, PendingMessages, Transcript, Welcome, accent, bold, editorTheme, green, muted, plain, red, selectTheme, yellow} from './view.mjs';
import {browseWorkspace} from './workspace.mjs';
import {answerQuestion} from './questions.mjs';
import {projectMenu, updateReview} from './project.mjs';
import {robotMenu} from './robot.mjs';
import {modelPicker} from './model.mjs';
import {surface} from './surface.mjs';
import {selection} from './selection.mjs';

const commands = commandsFor('terminal');

// The agent counts as quiet after this long without a new event: say so, so a
// slow model and a stalled one are not mistaken for each other silently.
const QUIET_MS = 60_000;
const SPINNER = ['✻', '✢', '✳', '✶', '✳', '✢'];
// Use Claude Code's documented Ctrl+D window for both exit shortcuts.
const QUIT_CONFIRM_MS = 800;

const elapsed = since => {
  const seconds = Math.max(0, Math.round((Date.now() - since) / 1000));
  return seconds < 60 ? `${seconds}s` : `${Math.floor(seconds / 60)}m${String(seconds % 60).padStart(2, '0')}s`;
};

// The component composition follows Pi's chat-simple example and interactive UI:
// transcript, status, editor, completion; application actions stay callbacks.
export class Chat {
  constructor(client, {terminal = new ProcessTerminal(), pollMs = 350, history = false, name = '', facts = {}, host = null,
    canSetup = false, version = '', initialNotice = ''} = {}) {
    this.client = client;
    this.ending = false; this.endConfirmed = false;
    this.history = history; this.result = {action: 'quit'};
    this.canSetup = canSetup; this.version = version; this.showWelcome = !client.readOnly;
    this.initialNotice = initialNotice;
    this.ui = surface(terminal, host, 'footer');
    this.ui.setHeading('OpenRUA', facts.project || name);
    this.facts = facts;
    this.transcript = new Transcript({workspace: facts.workspace});
    this.settingChanges = new SettingChanges(message => this.say(message));
    // Every event the agent produces marks it as not quiet (QUIET_MS).
    const record = this.transcript.event.bind(this.transcript);
    this.transcript.event = (event, {replay = false} = {}) => {
      this.lastEvent = Date.now(); record(event);
      const handled = this.settingChanges.receive(event, this.controller?.state);
      // What an event says to the person (an interrupt's cancelled motions,
      // a setting the agent refused), as every client says it.
      const said = notice(event);
      const oldSetting = replay && event.kind === 'agent_event' && event.data?.kind === 'setting_rejected';
      if (said && !handled && !oldSetting) this.say(said);
    };
    this.controller = new Controller(client, this.transcript);
    this.status = new Text('Connecting…', 0, 0);
    this.notice = new Text(plain(initialNotice), 0, 0);
    this.projectStatus = new Text('', 0, 0);
    this.pendingMessages = new PendingMessages();
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
    this.ui.setCloseHandler(() => this.stop());
    this.busy = false;
    this.inputRevision = 0;
    this.panels = [];
    this.shownQuestions = new Set();
    this.questionDrafts = new Map();
    this.welcome = new Welcome(name, {...facts, version}, client.readOnly);
    const conversation = new Container();
    if (client.readOnly) {
      conversation.addChild(this.welcome);
      if (client.reason) conversation.addChild(new Text(plain(client.reason), 0, 1));
    }
    conversation.addChild(this.transcript);
    this.ui.setBody(conversation);
    this.updateHeading();
    this.ui.addChild(new Spacer(1)); this.ui.addChild(this.status);
    this.ui.addChild(this.pendingMessages); this.ui.addChild(this.projectStatus); this.ui.addChild(this.notice); this.ui.addChild(this.editor);
    this.keys = new Text('', 0, 0);
    this.ui.addChild(this.keys);
    this.started = null;
    this.frame = 0;
    this.quitConfirmation = null;
    this.quitTimer = null;
    this.editor.onSubmit = text => { void this.submit(text); };
    this.ui.setNavigationHandler(data => {
      if (this.panel && ['pageUp', 'pageDown', 'home', 'end'].some(key => matchesKey(data, key))) {
        this.panel.component.handleInput(data); this.ui.requestRender(); return true;
      }
      return false;
    });
    this.ui.addInputListener(data => {
      if (isKeyRepeat(data) && (matchesKey(data, 'ctrl+c')
          || (matchesKey(data, 'ctrl+d') && !this.editor.getText()))) return {consume: true};
      this.inputRevision++;
      if (this.panel) return;
      if (matchesKey(data, 'ctrl+d') && !this.editor.getText()) {
        this.requestLeave('Ctrl+D'); return {consume: true};
      }
      if (matchesKey(data, 'ctrl+c')) {
        if (!this.client.readOnly && this.controller.state?.active) {
          this.resetLeave(); this.updateKeys();
          void this.interrupt().catch(error => this.say(error.message));
        } else {
          if (this.editor.getText()) { this.editor.setText(''); this.resetLeave(); }
          this.requestLeave('Ctrl+C');
        }
        return {consume: true};
      }
      this.resetLeave(); this.updateKeys(); this.ui.requestRender();
      if (matchesKey(data, 'escape') && !this.editor.isShowingAutocomplete()) {
        if (!this.client.readOnly && this.controller.state?.active)
          void this.interrupt().catch(error => this.say(error.message));
        return {consume: true};
      }
    });
  }

  say(message) { if (!this.done) { this.notice.setText(plain(message)); this.ui.requestRender(); } }

  resetLeave() {
    clearTimeout(this.quitTimer);
    this.quitTimer = null; this.quitConfirmation = null;
  }

  requestLeave(key) {
    if (this.quitConfirmation?.key === key && Date.now() < this.quitConfirmation.until) {
      this.stop(); return;
    }
    this.resetLeave();
    this.quitConfirmation = {key, until: Date.now() + QUIT_CONFIRM_MS};
    this.quitTimer = setTimeout(() => {
      this.resetLeave(); this.updateKeys(); this.ui.requestRender();
    }, QUIT_CONFIRM_MS);
    this.quitTimer.unref();
    this.updateKeys(); this.ui.requestRender();
  }

  updateHeading() {
    const welcome = this.showWelcome && !this.panel;
    if (this.headingWelcome === welcome) return;
    this.headingWelcome = welcome;
    if (welcome) this.ui.setHeader(this.welcome);
    else this.ui.setHeading('OpenRUA', this.facts.project || this.welcome.name);
  }

  // A dialog takes the editor's place below the transcript, the way a coding
  // agent's own menus do: nothing is drawn over the conversation, and while one
  // is open Escape closes it instead of meaning interrupt. Dialogs stack: a
  // menu that opens an editor gets its place back when the editor closes.
  get panel() { return this.panels.at(-1) ?? null; }

  dialog(component, {requestId = null} = {}) {
    this.resetLeave();
    this.inputRevision++;
    const slot = replacement => {
      const at = this.ui.children.indexOf(this.panel?.component ?? this.editor);
      if (at !== -1) this.ui.children.splice(at, 1, replacement);
    };
    slot(component);
    const handle = {component, requestId, hide: () => {
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
      this.updateKeys();
      this.updateHeading();
    }};
    this.panels.push(handle);
    this.updateHeading();
    this.updateKeys();
    this.ui.setFocus(component); this.ui.requestRender();
    return handle;
  }

  updateKeys() {
    if (!this.keys) return;
    const state = this.controller.state;
    if (!this.client.readOnly && state?.active && this.quitConfirmation?.key === 'Ctrl+C') this.resetLeave();
    this.keys.setText(muted(this.panel ? ''
      : this.quitConfirmation ? `Press ${this.quitConfirmation.key} again to leave.`
      : this.client.readOnly ? '/files Browse · /resume History · Ctrl+D Close'
      : state?.active ? 'Enter Send · Ctrl+C/Esc Interrupt'
      : 'Enter Send · Ctrl+J Newline · / Commands · Ctrl+D Leave'));
  }

  rows() { return this.ui.terminal?.rows ?? 24; }

  picker(title, items, select, options = {}) {
    const menu = selection(items, Math.max(2, Math.min(options.requestId ? 6 : 8, this.rows() - 12)),
      {explicit: Boolean(options.requestId || options.explicit)});
    const initial = items.findIndex(item => item.value === options.selected);
    if (initial >= 0) menu.setSelectedIndex(initial);
    const box = new Container();
    const heading = new DialogHeading(title, options.requestId, () => this.rows(), () => this.ui.requestRender());
    box.addChild(heading); box.addChild(items.length ? menu : new Text(options.empty || 'Nothing to show yet.', 0, 1));
    box.addChild(new Text(muted(`↑/↓ Select · Enter Confirm · Esc ${options.onCancel ? 'Back' : 'Close'}`), 0, 1));
    box.handleInput = data => { if (!heading.handleInput(data)) menu.handleInput(data); };
    let handle = this.dialog(box, options);
    menu.onCancel = () => { handle.hide(); options.onCancel?.(); };
    menu.onSelect = item => {
      handle.hide();
      const revision = this.inputRevision;
      Promise.resolve().then(() => select(item.value)).catch(error => {
        if (!this.done && !this.panel && revision === this.inputRevision) handle = this.dialog(box, options);
        this.say(error.message);
      });
    };
  }

  confirm(title, action, options = {}) {
    this.picker(title, [{value: 'cancel', label: 'Cancel'}, {value: 'confirm', label: 'Confirm'}],
      async value => { if (value === 'confirm') { await action(); if (!this.done) await this.refresh(); } }, {...options, explicit: true});
  }

  editText(title, initial, apply, options = {}) {
    const editor = new Editor(this.ui, editorTheme);
    editor.setText(initial);
    const heading = new DialogHeading(title, options.requestId, () => this.rows(), () => this.ui.requestRender());
    const box = new Container(); box.addChild(heading); box.addChild(editor);
    box.addChild(new Text(muted('Enter submit · Ctrl+J newline · Esc cancel'), 0, 1));
    Object.defineProperty(box, 'focused', {get: () => editor.focused, set: value => { editor.focused = value; }});
    box.handleInput = data => {
      if (heading.handleInput(data)) return;
      if (matchesKey(data, 'escape')) { handle.hide(); options.onCancel?.(); }
      else editor.handleInput(data);
    };
    let handle = this.dialog(box, options);
    editor.onSubmit = text => {
      if (!text.trim() && !options.allowEmpty) return;
      handle.hide();
      const revision = this.inputRevision;
      Promise.resolve().then(() => apply(text)).catch(error => {
        if (!this.done && !this.panel && revision === this.inputRevision) handle = this.dialog(box, options);
        this.say(error.message);
      });
    };
  }

  queue() {
    const messages = this.controller.state?.messages.filter(m => m.status === 'queued') ?? [];
    this.picker('Waiting messages', messages.map(m => ({value: m.id, label: plain(m.text),
      description: m.held_reason ? `Not sent yet: ${plain(m.held_reason)}` : 'Waiting to be sent'})), id => {
      const message = messages.find(m => m.id === id);
      if (this.client.readOnly) { this.say(message.text); return; }
      this.picker(plain(message.text), [{value: 'back', label: 'Back'}, {value: 'edit', label: 'Edit'},
        ...(message.held_reason ? [{value: 'release', label: 'Send this instruction now…'}] : []),
        {value: 'withdraw', label: 'Remove this waiting message'}], async action => {
        if (action === 'release') this.confirm('Send this instruction now? Check the robot and the result of earlier actions first.', async () => {
          await this.client.command('release', {message_id: id, revision: message.revision});
          await this.refresh();
        });
        if (action === 'edit') this.editText('Edit waiting message', message.text, async text => {
          await this.client.command('edit', {message_id: id, revision: message.revision, text}); await this.refresh();
        });
        if (action === 'withdraw') this.confirm('Remove this message before it is sent?', async () => {
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
    if (this.questionDrafts.get(typed.request_id) === text.trim()) {
      this.editor.setText(text); this.questions(); return true;
    }
    if (typed.kind === 'choose') { this.questions(); return true; }
    void this.client.command('respond', {request_id: typed.request_id, answers: typed.answers})
      .then(() => this.refresh()).catch(error => this.say(error.message));
    return true;
  }

  questions() {
    if (this.client.readOnly) throw new Error('You can view this conversation, but you cannot send messages here.');
    const visible = this.panels.find(panel => panel.requestId);
    if (visible) { this.ui.setFocus(this.panel.component); this.ui.requestRender(); return; }
    const pending = Object.values(this.controller.state?.requests ?? {}).filter(r => r.status === 'pending');
    this.picker('Questions from the agent', pending.map(r => ({value: r.request_id,
      label: plain(r.questions.map(q => q.text).join(' / '))})), id => {
      this.answerRequest(pending.find(r => r.request_id === id));
    }, {requestId: pending.length === 1 ? pending[0].request_id : null});
  }

  answerRequest(request) {
    const id = request.request_id;
    if (!this.shownQuestions.has(id) && this.editor.getText().trim())
      this.questionDrafts.set(id, this.editor.getText().trim());
    this.shownQuestions.add(id);
    if (request.questions.some(q => q.secret)) {
      this.say('This answer needs a private input field. Open the session in the browser to reply.'); return;
    }
    const options = {requestId: id};
    const answers = {};
    const next = index => {
      if (this.controller.state?.requests?.[id]?.status !== 'pending') return;
      if (index === request.questions.length) {
        const summary = request.questions.map(q => `Your answer: ${answers[q.id].join(', ')}\n${q.text}`).join('\n\n');
        this.confirm(`Review your response\n\n${summary}\n\nSend to the agent?`,
          () => this.client.command('respond', {request_id: id, answers}), options);
        return;
      }
      const question = request.questions[index];
      answerQuestion(this, question, values => { answers[question.id] = values; next(index + 1); }, options);
    };
    next(0);
  }

  showPendingQuestion() {
    const state = this.controller.state;
    const pending = new Map(Object.values(state?.requests ?? {}).filter(r => r.status === 'pending')
      .map(request => [request.request_id, request]));
    for (const panel of [...this.panels].reverse())
      if (panel.requestId && (!pending.has(panel.requestId) || state.closed)) panel.hide();
    for (const id of this.shownQuestions) if (!pending.has(id)) {
      this.shownQuestions.delete(id); this.questionDrafts.delete(id);
    }
    if (this.client.readOnly || state.closed || this.panel) return;
    const request = [...pending.values()].find(request => !this.shownQuestions.has(request.request_id));
    if (request) this.answerRequest(request);
  }

  async refresh() {
    await this.controller.refresh();
    if (this.done || this.ending) return;
    const state = this.controller.state;
    this.welcome.facts = {...this.facts, version: this.version,
      model: state.settings?.model || this.facts.model};
    this.pendingMessages.messages = state.messages;
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
    if (said.kind === 'working' && this.lastEvent && Date.now() - this.lastEvent > QUIET_MS)
      extra.splice(1, 0, yellow(`No new output for ${elapsed(this.lastEvent)}`));
    if (this.controller.pending) extra.push(yellow('Message delivery not confirmed. Use /retry.'));
    const head = said.kind === 'working'
      ? accent(`${SPINNER[this.frame++ % SPINNER.length]} Working… ${elapsed(this.started)}`)
      : tone[said.tone](({readonly: '◇ ', closed: '◇ ', paused: '‖ ', ready: '✓ ', waiting: '? ', interrupting: '… '}[said.kind] || '') + plain(said.text));
    const model = state.settings?.model || this.facts.model;
    const effort = state.settings?.effort;
    const nextTurn = state.setting_effective_from?.model === 'next_turn';
    const running = state.active && state.messages.find(message => message.id === state.active)?.turn_settings;
    const same = running && running.model === model && running.effort === effort;
    if (model) extra.push(muted(`model: ${plain(model)}`
      + (effort === undefined ? '' : `/${plain(effort)}`)));
    if (running?.model && nextTurn && !same)
      extra.push(muted(`This response uses ${plain(running.model)}`
        + (running.effort === undefined ? '' : `/${plain(running.effort)}`)));
    this.status.setText([head, ...extra].join(muted(' · ')));
    this.updateKeys();
    this.showPendingQuestion();
    this.ui.requestRender();
  }

  async poll() {
    try { await this.refresh(); void updateReview(this); } catch (error) {
      if (!this.done && !this.ending) {
        this.status.setText(yellow('⚠ Connection lost. Reconnecting…'));
        this.keys.setText(yellow('Your draft is still here. The agent may still be working.'));
        this.ui.requestRender();
      }
    }
    if (!this.done) this.timer = setTimeout(() => { void this.poll(); }, this.pollMs);
  }

  // The agent's own commands (its /compact, its skills...), as it reported
  // them; typed, they go to the agent as a message, as in its own terminal.
  agentCommands() { return agentCommands(this.controller.state); }

  async configure(setting, value) {
    return this.changeSettings('configure', {setting, value}, {[setting]: value});
  }

  async changeSettings(operation, params, values) {
    if (Object.entries(values).every(([key, value]) => this.controller.state?.settings?.[key] === value)) return true;
    const change = this.settingChanges.begin(values);
    try {
      await this.client.command(operation, params);
      await this.refresh();
      return this.settingChanges.complete(change, this.controller.state);
    } catch (error) { this.settingChanges.cancel(change); throw error; }
  }

  effort() {
    const state = this.controller.state;
    if (state?.settings?.effort === undefined) throw new Error('The reasoning effort setting is not available in this session.');
    const now = state.settings.effort;
    const choices = state.setting_choices?.effort ?? [];
    const title = `Reasoning effort: ${plain(now)}`;
    this.picker(title, [...choices.map(choice => ({value: choice.value,
      label: `${now === choice.value ? '✓ ' : ''}${plain(choice.label || choice.value)}`, description: plain(choice.description || '')})),
      ...(!choices.length ? [{value: null, label: 'Enter an effort level…'}] : [])], async value => {
      const apply = async effort => {
        await this.configure('effort', effort.trim());
      };
      if (value === null) this.editText('Reasoning effort', now, apply);
      else await apply(value);
    }, {selected: now});
  }

  async submit(text) {
    if (!text.trim() || this.done) return;
    if (this.busy) {
      this.editor.setText(text);
      this.say('Please wait for the previous request. Your draft has not been sent.');
      return;
    }
    this.busy = true;
    try {
      const command = parseCommand(text, this.controller.state);
      if (command?.kind === 'ours') {
        await this.handlers()[command.name](command.args);
      } else if (command?.kind === 'agent') {
        this.showWelcome = false; this.updateHeading();
        await this.controller.send(text.trim());
        this.editor.addToHistory(text);
        await this.refresh();
      } else if (command) {
        throw new Error('Unknown command. Use /help.');
      } else if (this.projectDraft === text && Object.values(this.controller.state?.requests ?? {}).some(r => r.status === 'pending')) {
        throw new Error('Answer the agent question first. This request stays in your draft.');
      } else if (this.answerTyped(text)) {
        this.showWelcome = false; this.updateHeading();
        this.editor.addToHistory(text);
        this.say('');
      } else {
        this.showWelcome = false; this.updateHeading();
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

  async interrupt() {
    const id = this.controller.state?.active;
    if (!id) { this.say('The agent is not working on a message right now.'); return; }
    await this.client.command('interrupt', {message_id: id});
    await this.refresh();
  }

  // One handler for each command the terminal offers (commandsFor('terminal')).
  handlers() {
    const state = this.controller.state;
    const writable = () => { if (this.client.readOnly) throw new Error('You can view this conversation, but you cannot send messages here.'); };
    return {
      project: async () => projectMenu(this),
      robot: async args => robotMenu(this, args),
      help: async () => {
        const agent = this.agentCommands();
        const theirs = agent.length ? ['', 'The agent\'s own:', ...agent.map(c => `/${c.name}: ${c.description}`)] : [];
        this.say(commands.map(c => `/${c.name}: ${c.description}`).concat(theirs).join('\n'));
      },
      permissions: async ([value]) => {
        writable();
        if (value) { await this.configure('permissions', value); return; }
        if (!permissionChoices(state).length) {
          this.say('This agent does not offer permission settings here. Its approval requests appear in the conversation.'); return;
        }
        const now = state?.settings?.permissions;
        this.picker('Permissions', permissionChoices(state).map(p => ({value: p.value,
          label: `${now === p.value ? '✓ ' : '  '}${p.label}`, description: p.description})),
          choice => this.configure('permissions', choice), {selected: now});
      },
      model: async ([value, level]) => {
        if (state?.setting_controls && !state.setting_controls.includes('model')) {
          this.say('This agent does not let OpenRUA change its model. Set the model in the agent configuration file.'); return;
        }
        if (value === 'effort') {
          writable();
          if (level) await this.configure('effort', level);
          else this.effort();
          return;
        }
        if (value) {
          writable();
          await this.configure('model', value);
          return;
        }
        const now = state?.settings?.model || this.facts.model || 'not reported';
        const title = `Model setting: ${plain(now)}`
          + (state?.settings?.effort !== undefined ? ` · effort: ${plain(state.settings.effort)}` : '');
        if (this.client.readOnly) { this.say(title); return; }
        modelPicker(this);
      },
      files: async () => { await browseWorkspace(this); },
      watch: async () => {
        // A terminal shows no video: the cameras play in the session's
        // browser page (Cursor and VS Code open the address on Ctrl+click).
        writable();
        if (!this.client.origin) throw new Error('Open the session in a browser: openrua session --name NAME web');
        this.say(`See the cameras live: open ${this.client.origin} in a browser, paste this session key, `
          + `then press Watch under Cameras.\n${this.client.token}\nThe key opens this session: keep it to yourself.`);
      },
      quit: async () => { this.stop(); },
      setup: async args => {
        if (!this.canSetup) throw new Error('Open this project with openrua --setup to configure a new session.');
        if (args.length) throw new Error('Use /setup to configure a new session.');
        if (this.controller.pending) throw new Error('We have not confirmed that your last message arrived. Use /retry before opening setup.');
        this.result = {action: 'setup'}; this.stop();
      },
      resume: async (args) => {
        if (!this.history) throw new Error('Run openrua --resume to open a saved conversation.');
        if (this.controller.pending) throw new Error('We have not confirmed that your last message arrived. Use /retry before switching conversations.');
        if (args.some(arg => arg !== '--all')) throw new Error('Use /resume or /resume --all.');
        this.result = {action: args.includes('--all') ? 'history_all' : 'history'}; this.stop();
      },
      end: async () => {
        writable();
        this.confirm('Stop this session and free its resources? Your files and chat history are saved.',
          async () => {
            this.ending = true;
            this.status.setText(yellow('Stopping the agent and releasing resources…'));
            this.ui.requestRender();
            try {
              await this.client.end();
              // Keep the acknowledged result even if the last poll still
              // described an open session. The owner can now close its HTTP server.
              this.endConfirmed = true;
              this.stop();
            } finally { this.ending = false; }
          });
      },
      questions: async () => { this.questions(); },
      retry: async () => {
        if (!this.controller.pending) { this.say('Nothing to retry.'); return; }
        await this.controller.retry(); await this.refresh();
      },
      tools: async () => {
        this.picker('Tool results', [...this.transcript.tools].map(([key, tool]) => ({value: key,
          label: `${tool.expanded ? '▾' : '▸'} ${tool.label}`, description: `${tool.phase}${tool.details ? ' · ' + plain(String(tool.details.command ?? tool.details.file_path ?? '')).split('\n')[0] : ''}`})),
          key => { this.transcript.toggleTool(key); this.ui.requestRender(); }, {empty: 'No tool calls yet.'});
      },
      queue: async () => { this.queue(); },
      interrupt: () => this.interrupt(),
      continue: async () => {
        const id = state?.pause_id;
        if (!id) { this.say('Nothing to continue.'); return; }
        this.confirm('Send the waiting messages? Check /queue first.', () => this.client.command('resume', {pause_id: id}));
      },
    };
  }

  async run() {
    await this.refresh();
    if (this.initialNotice) this.say(this.initialNotice);
    if (this.done) return this.result;
    this.ui.setFocus(this.panel?.component ?? this.editor);
    this.ui.start();
    await new Promise(resolve => { this.finished = resolve; void this.poll(); });
    return this.result;
  }

  stop() {
    this.resetLeave();
    this.workspaceBrowser?.close();
    if (this.done) return;
    this.done = true;
    clearTimeout(this.timer);
    this.ui.stop();
    this.client.close();
    this.finished?.();
  }
}
