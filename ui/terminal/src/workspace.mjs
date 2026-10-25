import {Container, Image, Text, matchesKey} from '@earendil-works/pi-tui';
import {muted, plain} from './view.mjs';
import {selection} from './selection.mjs';

// Presentation only: the existing authenticated API bounds all workspace reads.
// Text is shown through a window of `rows()` lines that the arrow keys move;
// the whole file is never drawn at once, so a long file cannot flood the screen.
export class FilePreview extends Container {
  // ``where``: the file's path on this computer (the sandbox's /workspace is a
  // directory here), null when unknown.
  constructor(file, close, refresh, redraw, rows = () => 24, where = null) {
    super();
    this.rows = rows;
    this.offset = 0;
    this.lines = file.kind === 'text' ? plain(file.text.slice(0, 131072)).split('\n') : [];
    if (file.kind === 'text' && file.text.length > 131072) this.lines.push('[Preview limited to 131,072 characters.]');
    this.addChild(new Text(`${plain(file.path)} · ${file.size} bytes\n${muted('Saved workspace file, not a live camera feed.')}`
      + (where ? `\n${muted('On this computer:')} ${plain(where)}` : ''), 0, 1));
    this.body = file.kind === 'image'
      ? new Image(file.data, file.mime, {fallbackColor: plain},
        {filename: plain(file.path), maxWidthCells: 72, maxHeightCells: 16})
      : new Text(file.kind === 'text' ? '' : `Binary file (${plain(file.mime)}); inspect it with the agent or browser.`, 0, 0);
    this.position = new Text('', 0, 0);
    this.addChild(this.body); this.addChild(this.position);
    this.addChild(new Text(muted((file.kind === 'text' ? '↑/↓, PgUp/PgDn scroll · ' : '') + 'r refresh · Esc back' +
      (file.kind !== 'image' ? '' : where
        ? '\nNo picture above? Open the path on this computer (Ctrl+click in Cursor or VS Code).'
        : '\nImage display depends on your terminal. Use the session browser if only metadata appears.')), 0, 1));
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

// One browser owns the directory, selection and pending reads until it closes.
export class WorkspaceBrowser extends Container {
  constructor(chat) {
    super();
    this.chat = chat; this.path = ''; this.positions = new Map(); this.revision = 0;
    this.closed = false; this.pending = false; this.preview = null;
    this.heading = new Text('Workspace /', 0, 1);
    this.feedback = new Text('', 0, 0);
    this.progress = new Text('', 0, 0);
    this.footer = new Text(muted('← Parent · → Open · Enter Select · Esc Close'), 0, 1);
    this.closedPromise = new Promise(resolve => { this.resolveClose = resolve; });
    this.listing = {entries: []}; this.list();
    this.handle = chat.dialog(this);
  }

  list(selected = this.positions.get(this.path)) {
    const items = [{value: '\0refresh', label: 'Refresh directory'}];
    if (this.path) items.push({value: '\0parent', label: '../'});
    for (const entry of this.listing.entries) items.push({value: entry.name,
      label: plain(entry.name) + (entry.kind === 'directory' ? '/' : ''),
      description: `${entry.kind} · ${entry.size} bytes`});
    this.menu = selection(items, Math.max(2, Math.min(8, this.chat.rows() - 12)), {selected});
    this.menu.onSelect = item => { void this.open(item.value); };
    this.draw();
  }

  draw() {
    this.clear();
    if (this.preview) this.addChild(this.preview);
    else {
      this.heading.setText(`Workspace /${plain(this.path)}${this.listing.truncated ? ' (listing truncated)' : ''}`);
      this.addChild(this.heading); this.addChild(this.menu); this.addChild(this.footer);
    }
    if (this.progress.text) this.addChild(this.progress);
    if (this.feedback.text) this.addChild(this.feedback);
    this.chat.ui.requestRender();
  }

  remember() { this.positions.set(this.path, this.menu.getSelectedItem()?.value); }
  valid(revision) { return !this.closed && !this.chat.done && revision === this.revision; }

  async read(title, operation, apply) {
    const revision = ++this.revision;
    this.pending = true; this.progress.setText(muted(title)); this.feedback.setText(''); this.draw();
    try {
      const result = await operation();
      if (this.valid(revision)) apply(result);
    } catch (error) {
      if (this.valid(revision)) this.feedback.setText(plain(error.message));
    } finally {
      if (this.valid(revision)) { this.pending = false; this.progress.setText(''); this.draw(); }
    }
  }

  async navigate(path, selected) {
    this.remember();
    await this.read(`Loading /${plain(path)}…`, () => this.chat.client.workspaceList(path), listing => {
      this.path = path; this.listing = listing; this.preview = null; this.list(selected);
    });
  }

  parent() {
    if (!this.path) return;
    const parts = this.path.split('/'), child = parts.pop();
    void this.navigate(parts.join('/'), child);
  }

  async open(value) {
    if (this.pending) return;
    if (value === '\0refresh') return this.navigate(this.path);
    if (value === '\0parent') return this.parent();
    const entry = this.listing.entries.find(item => item.name === value);
    if (!entry) return;
    if (entry.kind !== 'file' && entry.kind !== 'directory') {
      this.feedback.setText('Symlinks and special files cannot be previewed.'); this.draw(); return;
    }
    const target = this.path ? `${this.path}/${entry.name}` : entry.name;
    this.remember();
    if (entry.kind === 'directory') return this.navigate(target);
    return this.showFile(target);
  }

  async showFile(target) {
    await this.read(`Reading ${plain(target)}…`, () => this.chat.client.workspaceRead(target), file => {
      const where = this.chat.facts?.workspace ? `${this.chat.facts.workspace}/${target}` : null;
      this.preview = new FilePreview(file, () => this.back(), () => { void this.showFile(target); },
        () => this.chat.ui.requestRender(), () => this.chat.rows(), where);
    });
  }

  back() {
    this.revision++; this.pending = false; this.preview = null;
    this.progress.setText(''); this.feedback.setText(''); this.draw();
  }

  close() {
    if (this.closed) return;
    this.closed = true; this.revision++; this.handle.hide(); this.resolveClose();
  }

  handleInput(data) {
    if (matchesKey(data, 'escape')) {
      if (this.preview) this.back(); else this.close();
      return;
    }
    if (this.pending) return;
    if (this.preview) { this.preview.handleInput(data); return; }
    if (matchesKey(data, 'left')) this.parent();
    else if (matchesKey(data, 'right')) {
      const value = this.menu.getSelectedItem()?.value;
      if (value !== '\0refresh') void this.open(value);
    } else this.menu.handleInput(data);
    this.chat.ui.requestRender();
  }
}

export async function browseWorkspace(chat, path = '') {
  if (!chat.client.workspaceList) throw new Error('Workspace access is unavailable for this conversation.');
  chat.workspaceBrowser?.close();
  const browser = new WorkspaceBrowser(chat);
  chat.workspaceBrowser = browser;
  await Promise.race([browser.navigate(path), browser.closedPromise]);
}
