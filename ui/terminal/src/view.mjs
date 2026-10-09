import {Container, Markdown, Spacer, Text} from '@earendil-works/pi-tui';

// Control bytes must not become terminal instructions, including OSC clipboard writes.
export const plain = value => String(value ?? '').replace(/[\x00-\x08\x0b-\x1f\x7f-\x9f]/g, '');
const style = (open, close) => text => `\x1b[${open}m${text}\x1b[${close}m`;
const bold = style(1, 22), accent = style(36, 39);
export const muted = style(90, 39);
export const selectTheme = {selectedPrefix: accent, selectedText: bold, description: muted,
  scrollInfo: muted, noMatch: muted};
export const editorTheme = {borderColor: muted, selectList: selectTheme};
const markdownTheme = {heading: bold, link: accent, linkUrl: muted, code: accent,
  codeBlock: text => text, codeBlockBorder: muted, quote: muted, quoteBorder: muted,
  hr: muted, listBullet: accent, bold, italic: style(3, 23), strikethrough: style(9, 29), underline: style(4, 24)};


export class Transcript extends Container {
  constructor() { super(); this.turns = new Map(); this.tools = new Map(); }

  turn(id) {
    if (!this.turns.has(id)) {
      const block = new Container();
      const heading = new Text('', 0, 0);
      const user = new Text('', 0, 0);
      const result = new Text('', 0, 0);
      const output = new Container();
      block.addChild(new Spacer(1)); block.addChild(heading); block.addChild(user);
      block.addChild(output); block.addChild(result); this.addChild(block);
      this.turns.set(id, {heading, user, result, output, items: new Map()});
    }
    return this.turns.get(id);
  }

  message(message) {
    const turn = this.turn(message.id);
    turn.heading.setText(`You · ${plain(message.status)}`);
    turn.user.setText(plain(message.text));
  }

  snapshot(state) { for (const message of state.messages) this.message(message); }

  event(record) {
    if (record.kind === 'message_accepted' || record.kind === 'message_edited') this.message(record.data.message);
    if (record.kind !== 'agent_event' || !record.data.turn_id) return;
    const event = record.data;
    const turn = this.turn(event.turn_id);
    const data = event.data;
    if (event.kind === 'turn_finished') {
      const error = plain(data.error?.message);
      const alreadyShown = error && [...turn.items.values()].some(item => item.message && item.text.trim() === error.trim());
      turn.result.setText(alreadyShown ? plain(data.status) :
        plain(data.error?.message ?? (turn.items.size ? data.status : data.text ?? data.status)));
      return;
    }
    if (!['text_delta', 'item'].includes(event.kind)) return;
    const itemId = data.item_id ?? 'text';
    const message = event.kind === 'text_delta' || data.kind === 'message';
    if (!turn.items.has(itemId)) {
      const component = message ? new Markdown('', 0, 1, markdownTheme) : new Text('', 0, 0);
      turn.items.set(itemId, {component, text: '', message}); turn.output.addChild(component);
    }
    const item = turn.items.get(itemId);
    if (item.message) {
      item.text = event.kind === 'text_delta' ? item.text + plain(data.text) : plain(data.text ?? item.text);
      item.component.setText(item.text);
    } else {
      const key = `${event.turn_id}:${itemId}`;
      const prior = this.tools.get(key);
      const tool = {...prior, component: item.component,
        label: plain(data.text ?? data.kind ?? prior?.label ?? 'Tool'),
        phase: plain(data.phase ?? prior?.phase ?? ''),
        details: data.details === undefined ? prior?.details : plain(JSON.stringify(data.details, null, 2)),
        output: data.output === undefined ? prior?.output : plain(data.output),
        expanded: prior?.expanded ?? false};
      this.tools.set(key, tool); this.renderTool(tool);
    }
  }

  renderTool(tool) {
    const sections = [];
    if (tool.details !== undefined) sections.push(`Details\n${tool.details}`);
    if (tool.output !== undefined) sections.push(`Output\n${tool.output}`);
    tool.component.setText(`${tool.expanded ? '▾' : '▸'} ${tool.phase} · ${tool.label}${tool.expanded ? '\n' + sections.join('\n\n') : '  (/tools)'}`);
  }

  toggleTool(key) {
    const tool = this.tools.get(key);
    if (tool) { tool.expanded = !tool.expanded; this.renderTool(tool); }
  }
}
