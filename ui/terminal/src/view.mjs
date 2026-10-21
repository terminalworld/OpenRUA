import {existsSync} from 'node:fs';
import {join} from 'node:path';
import {Container, Markdown, Spacer, Text, truncateToWidth, visibleWidth, wrapTextWithAnsi} from '@earendil-works/pi-tui';

// Control bytes must not become terminal instructions, including OSC clipboard writes.
export const plain = value => String(value ?? '').replace(/[\x00-\x08\x0b-\x1f\x7f-\x9f]/g, '');
const style = (open, close) => text => `\x1b[${open}m${text}\x1b[${close}m`;
// The 16-colour palette only, so the terminal's own theme decides how it looks.
export const bold = style(1, 22), muted = style(90, 39), accent = style(36, 39);
export const green = style(32, 39), yellow = style(33, 39), red = style(31, 39);
export const selectTheme = {selectedPrefix: accent, selectedText: bold, description: muted,
  scrollInfo: muted, noMatch: muted};
export const editorTheme = {borderColor: muted, selectList: selectTheme};
const markdownTheme = {heading: bold, link: accent, linkUrl: muted, code: accent,
  codeBlock: text => text, codeBlockBorder: muted, quote: muted, quoteBorder: muted,
  hr: muted, listBullet: accent, bold, italic: style(3, 23), strikethrough: style(9, 29), underline: style(4, 24)};

// The first thing on the screen, as a coding agent's own terminal greets you:
// where you are and what you can say, in OpenRUA's own words (no model has
// looked at anything yet). Facts are the session's, as recorded at start.
export class Welcome {
  constructor(name, facts = {}, readOnly = false) {
    this.name = name; this.facts = facts; this.readOnly = readOnly;
    this.returning = false;   // set once the conversation turns out to have messages
  }

  invalidate() {}

  lines() {
    const f = this.facts;
    const agent = plain(f.agent_title || f.agent || 'your coding agent');
    const model = plain(f.robot_model || f.robot || 'robot');
    const a = /^[aeiou]/i.test(model) ? 'an' : 'a';
    const real = f.backend === 'real';
    let hello;
    if (f.backend === 'connect') hello = `Hi! Let's connect your robot. ${agent} does the work with you, and OpenRUA checks each step.`;
    else if (!f.robot) hello = `Hi! ${agent} is ready.`;
    else if (this.returning) hello = `The ${model} is still up, with ${agent} at the controls.`;
    else if (real) hello = `Hi! You're connected to your ${model}, with ${agent} at the controls.`;
    else hello = `Hi! You're connected to ${a} ${model}, simulated in ${plain(f.simulator)}, with ${agent} at the controls.`;
    const paragraphs = [hello];
    if (real) paragraphs.push('Keep the stop button within reach.');
    if (f.backend !== 'connect') {
      const tries = ['"what do you see?"'].concat(f.task ? [`"${plain(f.task)}"`] : []);
      paragraphs.push(tries.map((t, i) => (i ? '    ' : 'Try ') + t).join('\n'));
    }
    paragraphs.push(muted(`/help for commands · Esc ${f.backend === 'connect' ? 'interrupts' : 'stops the robot'}`));
    return paragraphs;
  }

  render(width) {
    const f = this.facts;
    const scene = [f.benchmark, f.suite && (f.task_id != null ? `${f.suite} #${f.task_id}` : f.suite)]
      .filter(Boolean).map(plain).join(' / ');
    const below = muted('  ' + [this.name && `session ${plain(this.name)}`, f.model && plain(f.model), scene]
      .filter(Boolean).join(' · '));
    if (this.readOnly) return [`${bold('OpenRUA')}${muted(` · ${plain(this.name)} · a past conversation, read-only`)}`];
    const inner = Math.max(20, Math.min(width, 62) - 4);
    const body = [`${accent('✻')} ${bold(this.returning ? 'Welcome back to OpenRUA' : 'Welcome to OpenRUA')}`];
    for (const paragraph of this.lines()) {
      body.push('');
      for (const line of paragraph.split('\n')) body.push(...wrapTextWithAnsi(line, inner - 2).map(l => '  ' + l));
    }
    const edge = muted('│');
    const box = [muted('╭' + '─'.repeat(inner + 2) + '╮'),
      ...body.map(line => `${edge} ${line}${' '.repeat(Math.max(0, inner - visibleWidth(line)))} ${edge}`),
      muted('╰' + '─'.repeat(inner + 2) + '╯')];
    return below.trim() ? box.concat(below) : box;
  }
}

// A file the agent names under /workspace, as the path it has on this
// computer: the sandbox's /workspace is a directory here, and a terminal that
// cannot draw a picture (Cursor, VS Code) opens a path on Ctrl+click.
const WORKSPACE_PATH = /\/workspace\/[^\s)\]'"`<>,;*]+/g;

export function onThisComputer(text, workspace, known = new Set()) {
  if (!workspace) return [];
  const found = [];
  for (const mention of new Set(String(text).match(WORKSPACE_PATH) ?? [])) {
    const host = join(workspace, mention.replace(/[.:]+$/, '').slice('/workspace/'.length));
    if (known.has(host) || existsSync(host)) { known.add(host); found.push(host); }
  }
  return found.slice(0, 5);
}

// Lines shown of a folded tool output, and the marker that opens the rest.
export const FOLDED_LINES = 3;

// A marker in the gutter of a block, the way a coding agent's terminal marks each
// of its own steps: the first line carries it, the rest are indented under it.
export class Bullet extends Container {
  constructor(child, mark = () => muted('⏺')) {
    super();
    this.child = child; this.mark = mark;
    this.addChild(child);
  }

  render(width) {
    const lines = this.child.render(Math.max(1, width - 2));
    return lines.map((line, index) => `${index ? ' ' : this.mark()} ${line}`);
  }
}

// One tool call: a coloured mark for its phase, the name with a one-line summary
// of what it was given, and its output folded to a few lines until /tools opens it.
export class ToolView {
  constructor(tool) { this.tool = tool; }

  invalidate() {}

  static summary(details) {
    if (details === undefined) return '';
    if (details && typeof details === 'object') {
      const value = details.command ?? details.file_path ?? details.path ?? details.pattern ?? details.url;
      if (typeof value === 'string') return value.split('\n')[0];
      const keys = Object.keys(details);
      if (!keys.length) return '';
      return keys.map(key => `${key}: ${typeof details[key] === 'string' ? details[key].split('\n')[0] : JSON.stringify(details[key])}`).join(', ');
    }
    return String(details);
  }

  render(width) {
    const {phase, label, details, output, expanded} = this.tool;
    const mark = phase === 'completed' ? green('⏺') : phase === 'failed' ? red('⏺') : accent('⏺');
    const summary = plain(ToolView.summary(details));
    const head = `${mark} ${bold(plain(label))}${summary ? muted('(') + summary + muted(')') : ''}`;
    const inner = Math.max(1, width - 4);
    const lines = [truncateToWidth(head, width, '…')];
    const body = plain(output ?? '').replace(/\s+$/, '');
    if (expanded) {
      const raw = JSON.stringify(details ?? null, null, 2);
      for (const line of plain(raw).split('\n')) lines.push(...wrapTextWithAnsi(muted(line), inner).map((l, i) => `  ${i === 0 && lines.length === 1 ? '⎿' : ' '} ${l}`));
      if (body) for (const line of body.split('\n')) lines.push(...wrapTextWithAnsi(line, inner).map(l => `    ${l}`));
      lines.push(muted('  ⎿ /tools folds this output'));
      return lines;
    }
    if (!body) return lines;
    const all = body.split('\n');
    all.slice(0, FOLDED_LINES).forEach((line, index) => {
      lines.push(`  ${index ? ' ' : '⎿'} ${truncateToWidth(muted(line), inner, '…')}`);
    });
    if (all.length > FOLDED_LINES) lines.push(muted(`    … +${all.length - FOLDED_LINES} lines (/tools)`));
    return lines;
  }
}

const RESULT = {
  failed: text => red(`✗ failed${text ? ': ' + text : ''}`),
  interrupted: () => yellow('‖ interrupted'),
  unknown: text => yellow(`? unknown outcome${text ? ': ' + text : ''} · inspect the robot, then resolve it with the session CLI`),
  withdrawn: () => muted('withdrawn'),
  reconciled: text => muted(`reconciled${text ? ': ' + text : ''}`),
};

export class Transcript extends Container {
  constructor({workspace = null} = {}) {
    super(); this.turns = new Map(); this.tools = new Map();
    this.workspace = workspace; this.known = new Set();
  }

  turn(id) {
    if (!this.turns.has(id)) {
      const block = new Container();
      const user = new Text('', 0, 0);
      const result = new Text('', 0, 0);
      const output = new Container();
      block.addChild(new Spacer(1)); block.addChild(user);
      block.addChild(output); block.addChild(result); this.addChild(block);
      this.turns.set(id, {user, result, output, items: new Map()});
    }
    return this.turns.get(id);
  }

  message(message) {
    const turn = this.turn(message.id);
    if (message.origin === 'agent') {
      // A turn the agent started by itself, a background task having finished.
      turn.user.setText(muted('⏺ the agent went on by itself'));
    } else {
      const note = ['queued', 'dispatching'].includes(message.status) ? ' · queued'
        : message.status === 'steering' ? ' · joining the running turn'
        : message.status === 'steered' ? ' · joined the running turn' : '';
      turn.user.setText(`${accent('❯')} ${bold(plain(message.text))}${note ? muted(note) : ''}`);
    }
    const result = RESULT[message.status];
    if (result) turn.result.setText(result(plain(message.uncertainty ?? message.resolution ?? '')));
    else if (message.status !== 'completed') turn.result.setText('');
  }

  snapshot(state) { for (const message of state.messages) this.message(message); }

  event(record) {
    if (record.kind === 'message_accepted' || record.kind === 'message_edited') this.message(record.data.message);
    if (record.kind !== 'agent_event' || !record.data.turn_id) return;
    const event = record.data;
    const turn = this.turn(event.turn_id);
    const data = event.data;
    if (event.kind === 'turn_finished') {
      const error = plain(data.error?.message ?? '');
      const alreadyShown = error && [...turn.items.values()].some(item => item.message && item.text.trim() === error.trim());
      if (data.status === 'completed') {
        turn.result.setText(turn.items.size || !data.text ? '' : '');
        if (!turn.items.size && data.text) this.item(turn, event.turn_id, 'text', true).set(plain(data.text));
      } else turn.result.setText(RESULT[data.status]?.(alreadyShown ? '' : error) ?? plain(data.status));
      return;
    }
    if (!['text_delta', 'item'].includes(event.kind)) return;
    const itemId = data.item_id ?? 'text';
    const message = event.kind === 'text_delta' || data.kind === 'message' || data.kind === 'question';
    const item = this.item(turn, event.turn_id, itemId, message, data.kind === 'question');
    if (item.message) {
      item.set(event.kind === 'text_delta' ? item.text + plain(data.text) : plain(data.text ?? item.text));
    } else {
      const key = `${event.turn_id}:${itemId}`;
      const prior = this.tools.get(key);
      const tool = {...prior, view: item.view,
        label: plain(data.text ?? data.kind ?? prior?.label ?? 'Tool'),
        phase: plain(data.phase ?? prior?.phase ?? ''),
        details: data.details === undefined ? prior?.details : data.details,
        output: data.output === undefined ? prior?.output : plain(data.output),
        expanded: prior?.expanded ?? false};
      this.tools.set(key, tool); item.view.tool = tool;
    }
  }

  item(turn, turnId, itemId, message, question = false) {
    if (!turn.items.has(itemId)) {
      let entry;
      if (message) {
        const component = new Markdown('', 0, 0, markdownTheme);
        const files = new Text('', 0, 0);
        const block = new Container();
        block.addChild(component); block.addChild(files);
        entry = {text: '', message: true, set: text => {
          entry.text = text; component.setText(text);
          files.setText(onThisComputer(text, this.workspace, this.known)
            .map(path => muted('↳ ') + plain(path)).join('\n'));
        }};
        // A question the agent asks without stopping: the answer is the next
        // message typed.
        turn.output.addChild(question ? new Bullet(block, () => yellow('?')) : new Bullet(block));
      } else {
        const view = new ToolView({label: 'Tool', phase: '', expanded: false});
        entry = {message: false, view};
        turn.output.addChild(view);
      }
      turn.items.set(itemId, entry);
    }
    return turn.items.get(itemId);
  }

  toggleTool(key) {
    const tool = this.tools.get(key);
    if (tool) { tool.expanded = !tool.expanded; }
  }
}

export {truncateToWidth, visibleWidth};
