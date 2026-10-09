import {Container, Image, Text, matchesKey} from '@earendil-works/pi-tui';
import {muted, plain} from './view.mjs';

// Presentation only: the existing authenticated API bounds all workspace reads.
// Text is shown through a window of `rows()` lines that the arrow keys move;
// the whole file is never drawn at once, so a long file cannot flood the screen.
export class FilePreview extends Container {
  constructor(file, close, refresh, redraw, rows = () => 24) {
    super();
    this.rows = rows;
    this.offset = 0;
    this.lines = file.kind === 'text' ? plain(file.text.slice(0, 131072)).split('\n') : [];
    if (file.kind === 'text' && file.text.length > 131072) this.lines.push('[Preview limited to 131,072 characters.]');
    this.addChild(new Text(`${plain(file.path)} · ${file.size} bytes\n${muted('Saved workspace file, not a live camera feed.')}`, 0, 1));
    this.body = file.kind === 'image'
      ? new Image(file.data, file.mime, {fallbackColor: plain},
        {filename: plain(file.path), maxWidthCells: 72, maxHeightCells: 16})
      : new Text(file.kind === 'text' ? '' : `Binary file (${plain(file.mime)}); inspect it with the agent or browser.`, 0, 0);
    this.position = new Text('', 0, 0);
    this.addChild(this.body); this.addChild(this.position);
    this.addChild(new Text(muted((file.kind === 'text' ? '↑/↓, PgUp/PgDn scroll · ' : '') + 'r refresh · Esc back' +
      (file.kind === 'image' ? '\nImage display depends on your terminal. Use the session browser if only metadata appears.' : '')), 0, 1));
    this.window();
    this.handleInput = data => {
      if (matchesKey(data, 'escape')) close();
      else if (data === 'r') refresh();
      else {
        const page = Math.max(1, this.height() - 1);
        if (matchesKey(data, 'up')) this.scroll(-1);
        if (matchesKey(data, 'down')) this.scroll(1);
        if (matchesKey(data, 'pageUp')) this.scroll(-page);
        if (matchesKey(data, 'pageDown')) this.scroll(page);
        if (matchesKey(data, 'home')) this.scroll(-this.lines.length);
        if (matchesKey(data, 'end')) this.scroll(this.lines.length);
        redraw();
      }
    };
  }

  // Lines of text shown at once: half the terminal, so the conversation above stays in view.
  height() { return Math.max(5, Math.floor(this.rows() / 2)); }

  scroll(by) {
    this.offset = Math.max(0, Math.min(this.offset + by, this.lines.length - this.height()));
    this.window();
  }

  window() {
    if (!this.lines.length) return;
    const shown = this.lines.slice(this.offset, this.offset + this.height());
    this.body.setText(shown.join('\n'));
    this.position.setText(this.lines.length > shown.length
      ? muted(`lines ${this.offset + 1}-${this.offset + shown.length} of ${this.lines.length}`) : '');
  }
}

export async function browseWorkspace(chat, path = '') {
  if (!chat.client.workspaceList) throw new Error('Workspace access is unavailable for this conversation.');
  const listing = await chat.client.workspaceList(path);
  if (chat.done) return;
  const parent = path.split('/').slice(0, -1).join('/');
  const items = [{value: {action: 'refresh'}, label: 'Refresh directory'}];
  if (path) items.push({value: {action: 'parent'}, label: '../'});
  for (const entry of listing.entries) {
    items.push({value: entry, label: plain(entry.name) + (entry.kind === 'directory' ? '/' : ''),
      description: `${entry.kind} · ${entry.size} bytes`});
  }
  chat.picker(`Workspace /${plain(path)}${listing.truncated ? ' (listing truncated)' : ''}`, items, async entry => {
    if (entry.action) return browseWorkspace(chat, entry.action === 'parent' ? parent : path);
    const target = path ? `${path}/${entry.name}` : entry.name;
    if (entry.kind === 'directory') return browseWorkspace(chat, target);
    if (entry.kind !== 'file') throw new Error('Symlinks and special files cannot be previewed.');
    const show = async () => {
      const file = await chat.client.workspaceRead(target);
      if (chat.done) return;
      const back = () => { handle.hide(); void browseWorkspace(chat, path).catch(e => chat.say(e.message)); };
      const refresh = () => { handle.hide(); void show().catch(e => chat.say(e.message)); };
      const preview = new FilePreview(file, back, refresh, () => chat.ui.requestRender(), () => chat.rows());
      const handle = chat.dialog(preview);
    };
    await show();
  });
}
