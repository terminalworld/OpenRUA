import {CombinedAutocompleteProvider, Container, Editor, ProcessTerminal, SelectList, Spacer, Text,
  TuiMainScreen, matchesKey} from '@earendil-works/pi-tui';
import {Controller} from './controller.mjs';
import {Transcript, editorTheme, plain, selectTheme} from './view.mjs';

const commands = [
  {name: 'help', description: 'Keyboard shortcuts and commands'},
  {name: 'tools', description: 'Expand or collapse a tool result'},
  {name: 'queue', description: 'Inspect the shared queue'},
  {name: 'interrupt', description: 'Interrupt the current turn and pause the queue'},
  {name: 'continue', description: 'Confirm continuation of the paused queue'},
  {name: 'retry', description: 'Retry an unconfirmed send with the same request ID'},
  {name: 'quit', description: 'Detach; the session keeps running'},
];

// The component composition follows Pi's chat-simple example and interactive UI:
// transcript, status, editor, completion; application actions stay callbacks.
export class Chat {
  constructor(client, {terminal = new ProcessTerminal(), pollMs = 350} = {}) {
    this.client = client;
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
    this.ui.addChild(new Text('\x1b[1mOpenRUA\x1b[22m · terminal prototype', 0, 0));
    this.ui.addChild(this.transcript);
    this.ui.addChild(new Spacer(1)); this.ui.addChild(this.status);
    this.ui.addChild(this.notice); this.ui.addChild(this.editor);
    this.ui.addChild(new Text('Enter send · Ctrl+J newline · / commands · Esc interrupt · Ctrl+D detach', 0, 0));
    this.editor.onSubmit = text => { void this.submit(text); };
    this.ui.addInputListener(data => {
      if (this.ui.hasOverlay()) return;
      if (matchesKey(data, 'ctrl+d') && !this.editor.getText()) { this.stop(); return {consume: true}; }
      if (matchesKey(data, 'ctrl+c')) {
        if (this.editor.getText()) this.editor.setText(''); else this.stop();
        return {consume: true};
      }
      if (matchesKey(data, 'escape')) { void this.submit('/interrupt'); return {consume: true}; }
    });
  }

  say(message) { if (!this.done) { this.notice.setText(plain(message)); this.ui.requestRender(); } }

  picker(title, items, select) {
    if (!items.length) { this.say('Nothing to show.'); return; }
    const menu = new SelectList(items, 8, selectTheme);
    const box = new Container();
    box.addChild(new Text(plain(title), 1, 1)); box.addChild(menu);
    box.addChild(new Text('↑/↓ select · Enter confirm · Esc back', 1, 1));
    box.handleInput = data => menu.handleInput(data);
    const handle = this.ui.showOverlay(box, {width: '85%', maxHeight: '80%'});
    menu.onCancel = () => handle.hide();
    menu.onSelect = item => {
      handle.hide();
      Promise.resolve().then(() => select(item.value)).catch(error => this.say(error.message));
    };
  }

  confirm(title, action) {
    this.picker(title, [{value: 'cancel', label: 'Cancel'}, {value: 'confirm', label: 'Confirm'}],
      async value => { if (value === 'confirm') { await action(); await this.refresh(); } });
  }

  async refresh() {
    await this.controller.refresh();
    if (this.done) return;
    const state = this.controller.state;
    const queued = state.messages.filter(message => message.status === 'queued').length;
    const requests = Object.keys(state.requests).length;
    this.status.setText(`${state.closed ? 'Closed' : state.paused ? 'Paused' : state.active ? 'Working' : 'Ready'} · ${queued} queued` +
      (state.connected ? '' : ' · agent disconnected') +
      (requests ? ` · ${requests} question(s): answer through the existing browser or CLI` : '') +
      (this.controller.pending ? ' · send unconfirmed (/retry)' : ''));
    this.ui.requestRender();
  }

  async poll() {
    try { await this.refresh(); } catch (error) { this.say(error.message); }
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
          case '/quit': this.stop(); break;
          case '/retry': await this.controller.retry(); await this.refresh(); break;
          case '/tools':
            this.picker('Tool results', [...this.transcript.tools].map(([key, tool]) => ({value: key,
              label: `${tool.expanded ? '▾' : '▸'} ${tool.phase} ${tool.label}`})),
              key => { this.transcript.toggleTool(key); this.ui.requestRender(); }); break;
          case '/queue':
            this.say(state ? state.messages.filter(m => m.status === 'queued').map((m, i) => `${i + 1}. ${m.text}`).join('\n') || 'Queue is empty.' : 'Connecting…'); break;
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
    if (this.done) return;
    this.ui.setFocus(this.editor);
    this.ui.start();
    await new Promise(resolve => { this.finished = resolve; void this.poll(); });
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
