import {Image, ScrollView, Text, VStack, matchesKey} from '@earendil-works/pi-tui';
import {plain} from './view.mjs';

// Presentation only: the existing authenticated API bounds all workspace reads.
export class FilePreview extends VStack {
  constructor(file, close, refresh, redraw) {
    const metadata = new Text(`${plain(file.path)} · ${file.size} bytes\nSaved workspace file, not a live camera feed.`, 0, 1);
    const content = file.kind === 'image'
      ? new Image(file.data, file.mime, {fallbackColor: plain},
        {filename: plain(file.path), maxWidthCells: 72, maxHeightCells: 16})
      : new Text(file.kind === 'text' ? plain(file.text.slice(0, 131072)) +
        (file.text.length > 131072 ? '\n[Preview limited to 131,072 characters.]' : '')
        : `Binary file (${plain(file.mime)}); inspect it with the agent or browser.`, 0, 0);
    const scroll = new ScrollView(content, {scrollbar: 'auto'});
    const hint = new Text('↑/↓, PgUp/PgDn scroll · r refresh · Esc back' +
      (file.kind === 'image' ? '\nImage display depends on your terminal. Use the session browser if only metadata appears.' : ''), 0, 1);
    super([metadata, {component: scroll, grow: 1, shrink: 1, minSize: 1}, hint]);
    this.scroll = scroll;
    this.handleInput = data => {
      if (matchesKey(data, 'escape')) close();
      else if (data === 'r') refresh();
      else {
        if (matchesKey(data, 'up')) scroll.scrollBy(-1);
        if (matchesKey(data, 'down')) scroll.scrollBy(1);
        if (matchesKey(data, 'pageUp')) scroll.scrollBy(-Math.max(1, scroll.viewportHeight - 1));
        if (matchesKey(data, 'pageDown')) scroll.scrollBy(Math.max(1, scroll.viewportHeight - 1));
        if (matchesKey(data, 'home')) scroll.scrollToStart();
        if (matchesKey(data, 'end')) scroll.scrollToEnd();
        redraw();
      }
    };
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
      const preview = new FilePreview(file, back, refresh, () => chat.ui.requestRender());
      const handle = chat.dialog(preview, {width: '90%', share: 0.85});
    };
    await show();
  });
}
