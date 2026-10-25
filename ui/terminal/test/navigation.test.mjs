import assert from 'node:assert/strict';
import {test} from 'node:test';
import {Chat} from '../src/app.mjs';
import {browseWorkspace} from '../src/workspace.mjs';

class Terminal {
  columns = 80; rows = 24; kittyProtocolActive = false;
  start(input) { this.input = input; } stop() {} write() {} async drainInput() {}
  moveBy() {} hideCursor() {} showCursor() {} clearLine() {}
  clearFromCursor() {} clearScreen() {} setTitle() {} setProgress() {}
}
const tick = () => new Promise(resolve => setImmediate(resolve));
const directory = name => ({name, kind: 'directory', size: 0});
const file = name => ({name, kind: 'file', size: 20});
const listing = (...entries) => ({entries});
function create(t, options = {}) {
  const calls = [], terminal = new Terminal(); Object.assign(terminal, options);
  const client = {readOnly: false, close() {},
    async command(...args) { calls.push(args); },
    async workspaceList(path) {
      calls.push(['list', path]);
      return path === '' ? listing(directory('alpha'))
        : path === 'alpha' ? listing(directory('beta'), file('notes.txt')) : listing();
    },
    async workspaceRead(path) { return {path, kind: 'text', text: 'Saved measurement', size: 17}; }};
  const chat = new Chat(client, {terminal});
  chat.controller.state = {active: 'running-agent', settings: {}};
  chat.editor.setText('Keep this draft');
  t.after(() => chat.stop());
  return {chat, client, calls, terminal};
}

for (const [columns, rows] of [[100, 30], [40, 18]]) {
  test(`file navigation preserves selections and closes the current view at ${columns}x${rows}`, async t => {
    const {chat, calls} = create(t, {columns, rows});
    await browseWorkspace(chat);
    const browser = chat.workspaceBrowser;
    browser.handleInput('\x1b[D');
    assert.equal(calls.length, 1, 'Left at root does not issue a request');
    browser.handleInput('\x1b[C');
    assert.equal(calls.length, 1, 'Right on Refresh does not refresh');
    browser.menu.setSelectedIndex(1); browser.handleInput('\x1b[C'); await tick();
    assert.equal(browser.path, 'alpha');
    browser.menu.setSelectedIndex(2); browser.handleInput('\r'); await tick();
    assert.equal(browser.path, 'alpha/beta');
    browser.handleInput('\x1b[D'); await tick();
    assert.equal(browser.path, 'alpha');
    assert.equal(browser.menu.getSelectedItem().value, 'beta');
    browser.menu.setSelectedIndex(3); browser.handleInput('\x1b[C'); await tick();
    assert.match(browser.render(columns).join('\n'), /Saved measurement/);
    browser.handleInput('\x1b');
    assert.equal(browser.preview, null);
    assert.equal(browser.path, 'alpha');
    assert.equal(browser.menu.getSelectedItem().value, 'notes.txt');
    browser.handleInput('\x1b');
    assert.equal(chat.panel, null);
    assert.equal(chat.editor.getText(), 'Keep this draft');
    assert.equal(calls.some(([operation]) => operation === 'interrupt'), false);
  });
}

test('loading a directory retains keyboard ownership and a closed read cannot reopen it', async t => {
  const {chat, client, terminal, calls} = create(t);
  await browseWorkspace(chat); chat.ui.start();
  const browser = chat.workspaceBrowser;
  let release;
  client.workspaceList = () => new Promise(resolve => { release = resolve; });
  browser.menu.setSelectedIndex(1); terminal.input('\x1b[C');
  assert.equal(chat.panel.component, browser);
  assert.equal(browser.pending, true);
  terminal.input('\x1b');
  assert.equal(chat.panel, null);
  chat.picker('A newer menu', [{value: 'back', label: 'Back'}], () => {});
  const newer = chat.panel;
  release(listing(file('late.txt'))); await tick();
  assert.equal(chat.panel, newer);
  assert.equal(calls.some(([operation]) => operation === 'interrupt'), false);
});

test('closing the initial file read releases the command before the server responds', async t => {
  const {chat, client} = create(t);
  let release, complete = false;
  client.workspaceList = () => new Promise(resolve => { release = resolve; });
  const opening = browseWorkspace(chat).then(() => { complete = true; });
  chat.workspaceBrowser.handleInput('\x1b'); await tick();
  assert.equal(complete, true);
  release(listing(file('late.txt'))); await opening; await tick();
  assert.equal(chat.panel, null);
});

test('failed directory reads keep the old path and cursor, and preview refresh can be canceled', async t => {
  const {chat, client} = create(t);
  await browseWorkspace(chat);
  const browser = chat.workspaceBrowser;
  browser.menu.setSelectedIndex(1);
  const read = client.workspaceList;
  client.workspaceList = async () => { throw new Error('Directory is unavailable'); };
  browser.handleInput('\x1b[C'); await tick();
  assert.equal(browser.path, '');
  assert.equal(browser.menu.getSelectedItem().value, 'alpha');
  assert.match(browser.render(80).join('\n'), /Directory is unavailable/);
  client.workspaceList = read;
  browser.handleInput('\x1b[C'); await tick();
  browser.menu.setSelectedIndex(3); browser.handleInput('\r'); await tick();
  let release;
  client.workspaceRead = () => new Promise(resolve => { release = resolve; });
  browser.handleInput('r');
  browser.handleInput('\x1b');
  assert.equal(browser.preview, null);
  release({path: 'alpha/notes.txt', kind: 'text', text: 'Late preview', size: 12}); await tick();
  assert.equal(browser.preview, null);
  assert.equal(browser.menu.getSelectedItem().value, 'notes.txt');
});

test('idle Escape leaves the draft and status alone, and empty tools remain a dismissible menu', async t => {
  const {chat, calls, terminal} = create(t);
  chat.controller.state = {active: null, settings: {}};
  chat.say('Existing feedback'); chat.ui.setFocus(chat.editor); chat.ui.start();
  terminal.input('\x1b'); await tick();
  assert.equal(chat.notice.text, 'Existing feedback');
  assert.equal(chat.editor.getText(), 'Keep this draft');
  assert.equal(calls.length, 0);
  await chat.handlers().tools();
  assert.match(chat.panel.component.render(80).join('\n'), /No tool calls yet/);
  terminal.input('\x1b');
  assert.equal(chat.panel, null);
});

test('restoring settings stays quiet while a recorded motion-cancellation failure remains visible', t => {
  const {chat} = create(t);
  const messages = []; chat.say = text => messages.push(text);
  chat.transcript.event({kind: 'agent_event', data: {kind: 'setting_rejected',
    data: {setting: 'model', value: 'old-model', reason: 'Old failed selection'}}}, {replay: true});
  assert.deepEqual(messages, []);
  chat.transcript.event({kind: 'motion_canceled', data: {canceled: [], failed: ['/arm/move']}}, {replay: true});
  assert.equal(messages.length, 1);
  assert.match(messages[0], /No answer from \/arm\/move/);
});
