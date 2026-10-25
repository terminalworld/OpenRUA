import net from 'node:net';
import {createInterface} from 'node:readline';
import {readFile} from 'node:fs/promises';
import {Text, matchesKey} from '@earendil-works/pi-tui';
import {Screen, TerminalHost} from './surface.mjs';
import {plain, muted} from './view.mjs';
import {selection} from './selection.mjs';

export class Progress extends Screen {
  constructor(host, parentPid) {
    super(undefined, host, {quitKeys: false});
    this.parentPid = parentPid; this.lines = [];
    this.compact = false; this.details = false; this.summary = ''; this.started = Date.now();
    this.output = new Text('', 0, 0); this.ui.addChild(this.output);
    this.ui.setFooter(new Text(muted('Preparing the project · Ctrl+C Interrupt'), 0, 1));
    this.ui.addInputListener(data => {
      if (this.compact && data.toLowerCase() === 'l') {
        this.details = !this.details; this.draw(); return {consume: true};
      }
      if (matchesKey(data, 'ctrl+c') || matchesKey(data, 'ctrl+d')) {
        if (Number.isInteger(parentPid) && parentPid > 0) process.kill(parentPid, 'SIGINT');
        return {consume: true};
      }
    });
  }
  update(message) {
    if (message.title) {
      this.lines = []; this.started = Date.now(); this.compact = Boolean(message.compact);
      this.details = false; this.summary = plain(message.summary || '');
      this.ui.setHeading(`OpenRUA · ${plain(message.title)}`, message.detail || '');
    }
    if (message.summary) this.summary = plain(message.summary);
    if (message.line) this.lines.push(plain(message.line));
    this.lines = this.lines.slice(-100);
    if (this.compact && !this.timer) { this.timer = setInterval(() => this.draw(), 1000); this.timer.unref(); }
    this.draw();
  }
  draw() {
    this.output.setText(this.compact ? `${this.summary} ${muted(`· ${Math.floor((Date.now() - this.started) / 1000)}s`)}`
      + (this.details ? '\n\n' + this.lines.join('\n') : '') : this.lines.join('\n'));
    this.ui.setFooter(new Text(muted(this.compact ? `L ${this.details ? 'Hide' : 'Show'} logs · Ctrl+C Interrupt`
      : 'Preparing the project · Ctrl+C Interrupt'), 0, 1));
    this.ui.requestRender();
  }
  stop() { clearInterval(this.timer); super.stop(); }
}

export class Leave extends Screen {
  constructor(prompt, host, terminal) {
    super(terminal, host);
    this.ui.setHeading('OpenRUA · Leave session');
    this.ui.addChild(new Text('Keep this session running?', 0, 1));
    this.ui.addChild(new Text('Your files and chat history are saved.', 0, 1));
    const menu = selection([
      {value: 'no', label: 'No', description: 'Stop this session'},
      {value: 'yes', label: 'Yes', description: 'Run in the background'},
    ], 4, {explicit: true});
    menu.onCancel = () => this.finish({action: 'answer', value: 'cancel'});
    menu.onSelect = item => this.finish({action: 'answer', value: item.value});
    this.ui.addChild(menu); this.ui.setFocus(menu);
    this.ui.setFooter(new Text(muted('↑/↓ Select · Enter Confirm · Esc Back'), 0, 1));
  }
}

export async function serve(fd, parentPid, createScreen, finishScreen) {
  const control = new net.Socket({fd, readable: true, writable: true});
  const lines = createInterface({input: control});
  const host = new TerminalHost();
  const stop = () => control.destroy();
  process.once('SIGINT', stop); process.once('SIGTERM', stop);
  let current = null, progress = null, pending = null, summary = [], closing = false;
  const send = message => { if (!control.destroyed) control.write(JSON.stringify(message) + '\n'); };
  const showProgress = message => {
    if (!progress) {
      current?.stop();
      progress = new Progress(host, parentPid); current = progress;
      progress.run().catch(() => {});
    }
    progress.update(message);
    return progress;
  };
  send({kind: 'ready', pid: process.pid});
  try {
    for await (const line of lines) {
      const message = JSON.parse(line);
      if (message.kind === 'close') break;
      if (message.kind === 'progress') { showProgress(message); continue; }
      if (message.kind === 'suspend') { host.stop(); send({id: message.id, result: null}); continue; }
      if (message.kind === 'resume') { host.start(); send({id: message.id, result: null}); continue; }
      if (message.kind !== 'screen') throw new Error('Unknown terminal presentation operation.');
      if (pending) throw new Error('Another terminal page is still active.');
      current?.stop(); progress = null;
      const spec = JSON.parse(await readFile(message.input, 'utf8'));
      current = await createScreen(spec, host);
      const selected = current;
      pending = selected.run().then(async result => {
        if (closing) return;
        summary = await finishScreen(selected, spec, result, host, showProgress) || [];
        if (!closing) send({id: message.id, result: result || {action: 'quit'}});
      }).catch(error => send({id: message.id, error: error.message}))
        .finally(() => { pending = null; });
    }
  } finally {
    closing = true;
    current?.stop(); host.close(); lines.close(); control.destroy();
    process.removeListener('SIGINT', stop); process.removeListener('SIGTERM', stop);
    for (const line of summary) console.log(line);
  }
}
