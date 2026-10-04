import {Container, Markdown, Spacer, Text} from '@earendil-works/pi-tui';

// Control bytes must not become terminal instructions, including OSC clipboard writes.
export const plain = value => String(value ?? '').replace(/[\x00-\x08\x0b-\x1f\x7f-\x9f]/g, '');
const identity = text => text;
export const selectTheme = Object.fromEntries(['selectedPrefix', 'selectedText', 'description', 'scrollInfo', 'noMatch'].map(key => [key, identity]));
export const editorTheme = {borderColor: identity, selectList: selectTheme};
const markdownTheme = Object.fromEntries(['heading', 'link', 'linkUrl', 'code', 'codeBlock', 'codeBlockBorder',
  'quote', 'quoteBorder', 'hr', 'listBullet', 'bold', 'italic', 'strikethrough', 'underline'].map(key => [key, identity]));

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
      turn.result.setText(plain(data.error?.message ?? (turn.items.size ? data.status : data.text ?? data.status)));
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
        detail: plain(data.output ?? (data.details === undefined ? prior?.detail ?? '' : JSON.stringify(data.details, null, 2))),
        expanded: prior?.expanded ?? false};
      this.tools.set(key, tool); this.renderTool(tool);
    }
  }

  renderTool(tool) {
    tool.component.setText(`${tool.expanded ? '▾' : '▸'} ${tool.phase} · ${tool.label}${tool.expanded ? '\n' + tool.detail : '  (/tools)'}`);
  }

  toggleTool(key) {
    const tool = this.tools.get(key);
    if (tool) { tool.expanded = !tool.expanded; this.renderTool(tool); }
  }
}
