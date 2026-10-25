import {existsSync} from 'node:fs';
import {join} from 'node:path';
import {Container, Markdown, Spacer, Text, matchesKey, truncateToWidth, visibleWidth, wrapTextWithAnsi} from '@earendil-works/pi-tui';
import {toolSummary} from './session.mjs';

// Control bytes must not become terminal instructions, including OSC clipboard writes.
export const plain = value => String(value ?? '').replace(/[\x00-\x08\x0b-\x1f\x7f-\x9f]/g, '');
const configName = value => plain(value).split(/[\\/]/).at(-1).replace(/\.ya?ml$/i, '');
const style = (open, close) => text => `\x1b[${open}m${text}\x1b[${close}m`;
// The 16-colour palette only, so the terminal's own theme decides how it looks.
export const bold = style(1, 22), muted = style(90, 39), accent = style(36, 39);
export const green = style(32, 39), yellow = style(33, 39), red = style(31, 39);
export class DialogHeading {
  constructor(title, requestId, rows, redraw) {
    this.body = new Text(plain(title), 0, requestId ? 0 : 1);
    this.requestId = requestId; this.rows = rows; this.redraw = redraw;
    this.offset = 0; this.lines = []; this.height = 4;
  }
  invalidate() { this.body.invalidate(); }
  render(width) {
    this.lines = this.body.render(width);
    if (!this.requestId) return this.lines;
    this.height = Math.max(2, Math.floor(this.rows() / 5));
    this.offset = Math.max(0, Math.min(this.offset, this.lines.length - this.height));
    const shown = this.lines.slice(this.offset, this.offset + this.height);
    return ['', yellow(bold('Your response is needed')), '', ...shown,
      ...(this.lines.length > shown.length ? [muted(`PgUp/PgDn read details · lines ${this.offset + 1}-${this.offset + shown.length} of ${this.lines.length}`)] : []), ''];
  }
  handleInput(data) {
    if (!this.requestId || this.lines.length <= this.height) return false;
    const delta = matchesKey(data, 'pageDown') ? this.height : matchesKey(data, 'pageUp') ? -this.height : 0;
    if (!delta) return false;
    this.offset = Math.max(0, Math.min(this.offset + delta, this.lines.length - this.height));
    this.redraw(); return true;
  }
}
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

  render(width) {
    const f = this.facts;
    if (this.readOnly) return [`${bold('OpenRUA')}${muted(` · ${plain(this.name)} · a past conversation, read-only`)}`];
    if (width < 16) return [truncateToWidth(bold('OpenRUA'), width)];
    const inside = Math.max(1, Math.min(width, 88) - 4);
    const title = ` OpenRUA${f.version ? ' v' + plain(f.version) : ''} `;
    const top = accent('╭') + accent(truncateToWidth(title, inside + 2))
      + accent('─'.repeat(Math.max(0, inside + 2 - visibleWidth(title)))) + accent('╮');
    const agent = [plain(f.agent_title || f.agent || 'Coding agent'), plain(f.model || 'Native model')].join(' · ');
    const runtime = [f.backend === 'real' ? 'Real robot' : f.backend === 'connect' ? 'Robot setup'
      : f.simulator || f.backend === 'sim' ? 'Simulation' : 'Robot project',
      plain(f.robot_model || configName(f.robot))].filter(Boolean).join(' · ');
    const scene = [configName(f.simulator), configName(f.benchmark)].filter(Boolean).join(' · ');
    const environment = [runtime, scene].filter(Boolean).join(' · ');
    const path = plain(f.project || f.workspace || '');
    const parts = path.split(/[\\/]/);
    let project = path;
    while (visibleWidth(project) > inside && parts.length > 1) { parts.shift(); project = '…/' + parts.join('/'); }
    const greeting = this.returning ? 'Welcome back to OpenRUA' : 'Welcome to OpenRUA';
    const commands = '/help Commands · Esc Interrupt';
    const rows = [];
    if (inside >= 56) {
      const mark = ['   ╭─╮  ', ' ╭─┴─┴─╮', ' │ • • │', ' ╰─────╯'];
      const text = [bold(greeting), muted('Turn your robot into a coding project.'), agent, muted(environment)];
      for (let i = 0; i < mark.length; i++) rows.push(accent(mark[i]) + '   ' + truncateToWidth(text[i], inside - 11));
    } else {
      rows.push(accent('[• •] ') + bold(greeting),
        agent, muted(runtime));
      if (scene) rows.push(muted(scene));
    }
    if (project) rows.push(muted(project));
    rows.push(muted(commands));
    const edge = accent('│');
    return [top, ...rows.map(row => {
      const text = truncateToWidth(row, inside);
      return `${edge} ${text}${' '.repeat(Math.max(0, inside - visibleWidth(text)))} ${edge}`;
    }), accent('╰' + '─'.repeat(inside + 2) + '╯'), ''];
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

  static summary(details) { return toolSummary(details); }

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

const PENDING = new Set(['queued', 'dispatching', 'steering']);

export class PendingMessages {
  constructor() { this.messages = []; }
  invalidate() {}
  render(width) {
    const messages = this.messages.filter(message => PENDING.has(message.status));
    const labels = {queued: 'Queued', dispatching: 'Sending to agent', steering: 'Adding to current turn'};
    const lines = messages.slice(-3).map(message => yellow(truncateToWidth(
      `${message.held_reason ? 'Held for review (/queue)' : labels[message.status]}: ${plain(message.text).replace(/\s+/g, ' ')}`, width, '…')));
    if (messages.length > 3) lines.unshift(muted(`${messages.length} pending messages · /queue to inspect`));
    return lines;
  }
}

export class Transcript extends Container {
  constructor({workspace = null} = {}) {
    super(); this.turns = new Map(); this.tools = new Map();
    this.workspace = workspace; this.known = new Set();
    this.messages = new Map();
  }

  turn(id) {
    if (!this.turns.has(id)) {
      const block = new Container();
      const user = new Text('', 0, 0);
      const result = new Text('', 0, 0);
      const output = new Container();
      block.addChild(new Spacer(1)); block.addChild(user);
      block.addChild(output); block.addChild(result); this.addChild(block);
      this.turns.set(id, {block, parent: this, user, result, output, items: new Map()});
    }
    return this.turns.get(id);
  }

  message(message) {
    if (PENDING.has(message.status)) return;
    const turn = this.turn(message.id);
    const parent = message.status === 'steered' && message.joined
      ? this.turn(message.joined).output : this;
    if (turn.parent !== parent) {
      const index = turn.parent.children.indexOf(turn.block);
      if (index !== -1) turn.parent.children.splice(index, 1);
      parent.addChild(turn.block); turn.parent = parent;
    }
    if (message.origin === 'agent') {
      // A turn the agent started by itself, a background task having finished.
      turn.user.setText(muted('⏺ the agent went on by itself'));
    } else {
      const note = message.status === 'steered' ? 'Received by agent · added to the current turn' : '';
      turn.user.setText(`${note ? muted(note) + '\n' : ''}${accent('❯')} ${bold(plain(message.text))}`);
    }
    const result = RESULT[message.status];
    if (result) turn.result.setText(result(plain(message.uncertainty ?? message.resolution ?? '')));
    else if (message.status !== 'completed') turn.result.setText('');
  }

  snapshot(state) {
    this.messages = new Map(state.messages.map(message => [message.id, message]));
    for (const message of state.messages) this.message(message);
  }

  event(record) {
    if (record.kind === 'message_accepted' || record.kind === 'message_edited')
      this.message(this.messages.get(record.data.message.id) ?? record.data.message);
    if (record.kind !== 'agent_event' || !record.data.turn_id) return;
    const event = record.data;
    if (event.kind === 'steer_delivered') {
      const message = this.messages.get(event.data.message_id);
      if (message?.status === 'steered') {
        this.message(message);
        const delivered = this.turn(message.id);
        const index = delivered.parent.children.indexOf(delivered.block);
        if (index !== -1) delivered.parent.children.splice(index, 1);
        delivered.parent.addChild(delivered.block);
      }
      return;
    }
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
