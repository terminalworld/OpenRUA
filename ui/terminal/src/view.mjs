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
  interrupted: () => yellow('‖ interrupted · the queue is paused; /continue when ready'),
  unknown: text => yellow(`? unknown outcome${text ? ': ' + text : ''} · inspect the robot, then resolve it with the session CLI`),
  withdrawn: () => muted('withdrawn'),
  reconciled: text => muted(`reconciled${text ? ': ' + text : ''}`),
};

export class Transcript extends Container {
  constructor() { super(); this.turns = new Map(); this.tools = new Map(); }

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
    const waiting = ['queued', 'dispatching'].includes(message.status);
    turn.user.setText(`${accent('❯')} ${bold(plain(message.text))}${waiting ? muted(' · queued') : ''}`);
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
    const message = event.kind === 'text_delta' || data.kind === 'message';
    const item = this.item(turn, event.turn_id, itemId, message);
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

  item(turn, turnId, itemId, message) {
    if (!turn.items.has(itemId)) {
      let entry;
      if (message) {
        const component = new Markdown('', 0, 0, markdownTheme);
        entry = {text: '', message: true, set: text => { entry.text = text; component.setText(text); }};
        turn.output.addChild(new Bullet(component));
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
