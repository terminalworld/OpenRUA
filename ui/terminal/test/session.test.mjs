import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import {createInterface} from 'node:readline';
import {fileURLToPath} from 'node:url';
import {test} from 'node:test';
import http from 'node:http';
import {Client} from '../src/client.mjs';
import {Controller} from '../src/controller.mjs';
import {Chat} from '../src/app.mjs';
import {Transcript} from '../src/view.mjs';

async function fixture(t) {
  const child = spawn('python', ['-u', '-m', 'ui.terminal.test.service'], {
    cwd: fileURLToPath(new URL('../../../', import.meta.url)), stdio: ['pipe', 'pipe', 'pipe'],
  });
  let errors = '';
  child.stdin.on('error', () => {});
  child.stderr.on('data', bytes => { errors += bytes; });
  const lines = createInterface({input: child.stdout})[Symbol.asyncIterator]();
  const next = async () => {
    const value = await lines.next();
    if (value.done) throw new Error(`Test service stopped: ${errors}`);
    return JSON.parse(value.value);
  };
  t.after(async () => {
    if (child.exitCode === null) {
      const exited = once(child, 'exit'); child.stdin.end('{"op":"stop"}\n');
      const timeout = setTimeout(() => child.kill('SIGKILL'), 2000);
      await exited; clearTimeout(timeout);
    }
  });
  const {endpoint} = await next();
  return {client: await Client.fromFile(endpoint),
    call: async data => { child.stdin.write(JSON.stringify(data) + '\n'); return next(); }};
}

async function until(client, condition) {
  for (let i = 0; i < 200; i++) {
    const {state} = await client.snapshot();
    if (condition(state)) return state;
    await new Promise(resolve => setTimeout(resolve, 10));
  }
  throw new Error('Session state did not converge.');
}

test('Pi client shares the real owner queue, events, interruption and reconnect', {timeout: 15000}, async t => {
  const {client, call} = await fixture(t);
  const view = new Transcript();
  const controller = new Controller(client, view, 'pi-client');
  await controller.refresh();
  const first = await controller.send('Inspect the block');
  await call({op: 'frame', frame: {kind: 'turn_started', turn_id: first.id, data: {}}});
  const other = new Client(client.origin, client.token);
  t.after(() => other.close());
  const second = await other.command('enqueue', {client_id: 'browser', request_id: 'b', text: 'Put it in the bowl'});
  let state = await until(client, s => s.active === first.id && s.messages.length === 2);
  assert.equal(state.messages[1].status, 'queued');
  await call({op: 'frame', frame: {kind: 'text_delta', turn_id: first.id, data: {item_id: 'answer', text: 'Inspecting **camera**'}}});
  await call({op: 'frame', frame: {kind: 'item', turn_id: first.id, data: {item_id: 'tool', kind: 'tool', text: 'read image', phase: 'completed', output: 'frame saved\nline 2\nline 3\nline 4\nline 5'}}});
  for (let i = 0; i < 100 && view.tools.size === 0; i++) {
    await controller.refresh(); await new Promise(resolve => setTimeout(resolve, 5));
  }
  assert.equal(view.tools.size, 1);
  // Folded: the first three output lines and a count of the rest; /tools opens it all.
  let rendered = view.render(80).join('\n');
  assert.match(rendered, /Inspecting/);
  assert.match(rendered, /⎿ .*frame saved/);
  assert.match(rendered, /line 3/);
  assert.doesNotMatch(rendered, /line 4/);
  assert.match(rendered, /\+2 lines \(\/tools\)/);
  view.toggleTool(`${first.id}:tool`);
  rendered = view.render(80).join('\n');
  assert.match(rendered, /line 5/);
  assert.doesNotMatch(rendered, /\+2 lines/);
  await client.command('interrupt', {message_id: first.id});
  await call({op: 'frame', frame: {kind: 'turn_finished', turn_id: first.id, data: {status: 'interrupted'}}});
  // As in a coding agent's own terminal: the interrupt stops the turn, and
  // the queued message goes next without a resume.
  state = await until(client, s => s.active === second.id);
  assert.equal(state.paused, false);
  client.close();
  const reconnected = new Controller(other, new Transcript(), 'reconnected');
  await reconnected.refresh();
  assert.equal(reconnected.state.messages.length, 2);
  const {writes} = await call({op: 'writes'});
  assert.deepEqual(writes.filter(w => w.submit).map(w => w.submit), [first.id, second.id]);
  assert.equal(writes.filter(w => w.interrupt).length, 1);
});

test('refresh after a send cannot reuse a snapshot taken before acceptance', async () => {
  let release;
  const barrier = new Promise(resolve => { release = resolve; });
  let messages = [], snapshots = 0;
  const client = {
    async snapshot() {
      const state = {messages: [...messages]};
      if (++snapshots === 1) await barrier;
      return {state};
    },
    async events() { return []; },
    async command(op, message) { messages.push(message); return message; },
  };
  const controller = new Controller(client, {snapshot() {}, event() {}});
  const poll = controller.refresh();
  await controller.send('Inspect only');
  const afterSend = controller.refresh();
  release();
  await Promise.all([poll, afterSend]);
  assert.equal(controller.state.messages[0]?.text, 'Inspect only');
  assert.equal(snapshots, 2);
});

test('a failed earlier poll does not prevent the next refresh', async () => {
  let reject;
  const barrier = new Promise((resolve, fail) => { reject = fail; });
  let snapshots = 0;
  const client = {
    async snapshot() {
      if (++snapshots === 1) await barrier;
      return {state: {messages: []}};
    },
    async events() { return []; },
  };
  const controller = new Controller(client, {snapshot() {}, event() {}});
  const failed = assert.rejects(controller.refresh(), /offline/);
  const next = controller.refresh();
  reject(new Error('offline'));
  await Promise.all([failed, next]);
  assert.equal(snapshots, 2);
  assert.deepEqual(controller.state.messages, []);
});

test('events accepted after a snapshot wait for its next consistent state', async () => {
  const view = new Transcript();
  const active = {id: 'active', text: 'Inspect', status: 'running'};
  const extra = {id: 'extra', text: 'New camera configuration', status: 'steering', joined: 'active'};
  let delivered = false;
  const before = {seq: 1, kind: 'agent_event', data: {kind: 'item', turn_id: 'active', data: {
    item_id: 'before', kind: 'message', text: 'Before the update', phase: 'completed'}}};
  const records = [before,
    {seq: 2, kind: 'agent_event', data: {kind: 'steer_delivered', turn_id: 'active', data: {message_id: 'extra'}}},
    {seq: 3, kind: 'agent_event', data: {kind: 'item', turn_id: 'active', data: {
      item_id: 'after', kind: 'message', text: 'After the update', phase: 'completed'}}}];
  const client = {
    async snapshot() { return {cursor: delivered ? 3 : 1, state: {messages: [active,
      {...extra, status: delivered ? 'steered' : 'steering'}]}}; },
    async events(after) { return records.filter(record => record.seq > after); },
  };
  const controller = new Controller(client, view);
  await controller.refresh();
  assert.equal(controller.cursor, 1);
  assert.doesNotMatch(view.render(80).join('\n'), /After the update/);
  delivered = true;
  await controller.refresh();
  const rendered = view.render(80).join('\n');
  assert.ok(rendered.indexOf('Before the update') < rendered.indexOf('New camera configuration'));
  assert.ok(rendered.indexOf('New camera configuration') < rendered.indexOf('After the update'));
  assert.equal(controller.cursor, 3);
});

test('lost send acknowledgement retries the same ID without duplicate execution', {timeout: 15000}, async t => {
  const {client, call} = await fixture(t);
  t.after(() => client.close());
  const realCommand = client.command.bind(client);
  let loseReply = true;
  client.command = async (...args) => {
    const result = await realCommand(...args);
    if (loseReply) { loseReply = false; throw new Error('Lost acknowledgement after server acceptance'); }
    return result;
  };
  const controller = new Controller(client, new Transcript(), 'ack-client');
  await assert.rejects(controller.send('Inspect once'), /Lost acknowledgement/);
  const requestId = controller.pending.request_id;
  await assert.rejects(controller.send('Do not duplicate'), /unconfirmed/);
  const result = await controller.retry();
  assert.equal(result.request_id, requestId);
  const {state} = await client.snapshot();
  assert.equal(state.messages.length, 1);
  const {writes} = await call({op: 'writes'});
  assert.equal(writes.filter(w => w.submit).length, 1);
});

test('client rejects remote endpoints and does not follow redirects', async t => {
  for (const url of ['https://127.0.0.1:1234', 'http://example.com:1234', 'http://127.0.0.1:1234/path', 'http://user@127.0.0.1:1234']) {
    assert.throws(() => new Client(url, 'token'), /local HTTP/);
  }
  let hits = 0;
  const server = http.createServer((req, res) => {
    hits++; res.writeHead(302, {'Location': 'http://example.com', 'Content-Type': 'application/json'}); res.end('{"error":"redirect"}');
  }).listen(0, '127.0.0.1');
  await once(server, 'listening'); t.after(() => server.close());
  const client = new Client(`http://127.0.0.1:${server.address().port}`, 'test');
  t.after(() => client.close());
  await assert.rejects(client.snapshot(), /redirect/);
  assert.equal(hits, 1);
});

class Terminal {
  columns = 80;
  rows = 24;
  kittyProtocolActive = false;
  output = '';
  start(input, resize) { this.input = input; this.resize = resize; }
  stop() { this.stopped = true; }
  write(data) { this.output += data; }
  async drainInput() {}
  moveBy() {}
  hideCursor() {}
  showCursor() {}
  clearLine() {}
  clearFromCursor() {}
  clearScreen() {}
  setTitle() {}
  setProgress() {}
}

test('Pi keyboard editor submits, queues, interrupts on Escape and detaches', {timeout: 15000}, async t => {
  const {client, call} = await fixture(t);
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20});
  t.after(() => chat.stop());
  const running = chat.run();
  for (let i = 0; i < 100 && !terminal.input; i++) await new Promise(r => setTimeout(r, 10));
  assert.equal(typeof terminal.input, 'function');
  // Exercise Pi's own input handling, not Chat.submit directly.
  terminal.input('Inspect'); terminal.input('\r');
  let state = await until(client, s => !!s.active);
  const id = state.active;
  await call({op: 'frame', frame: {kind: 'turn_started', turn_id: id, data: {}}});
  while (chat.busy) await new Promise(r => setTimeout(r, 5));
  terminal.input('Then wait'); terminal.input('\r');
  await until(client, s => s.messages.length === 2);
  while (chat.busy) await new Promise(r => setTimeout(r, 5));
  // Escape stops the turn at once, as in a coding agent's own terminal.
  terminal.input('\x1b');
  assert.equal(chat.panel, null);
  await until(client, s => s.messages[0].cancel_requested);
  await call({op: 'frame', frame: {kind: 'turn_finished', turn_id: id, data: {status: 'interrupted'}}});
  // ...and what was queued goes next.
  await until(client, s => s.messages[1].status === 'dispatching' || s.messages[1].status === 'running');
  const observer = new Client(client.origin, client.token);
  t.after(() => observer.close());
  terminal.input('\x04');
  await running;
  state = (await observer.snapshot()).state;
  assert.equal(state.closed, false);
  assert.equal(state.paused, false);
  assert.equal(terminal.stopped, true);
});

test('queued edits and question replies use the existing owner contracts', {timeout: 15000}, async t => {
  const {client, call} = await fixture(t);
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20, history: true});
  t.after(() => chat.stop());
  const running = chat.run();
  for (let i = 0; i < 100 && !terminal.input; i++) await new Promise(r => setTimeout(r, 10));
  await chat.submit('Inspect');
  const id = (await client.snapshot()).state.active;
  await call({op: 'frame', frame: {kind: 'turn_started', turn_id: id, data: {}}});
  await chat.submit('Later');
  await chat.submit('/queue');
  terminal.input('\r'); await new Promise(r => setTimeout(r, 10));
  terminal.input('\x1b[B'); terminal.input('\r'); await new Promise(r => setTimeout(r, 10));
  terminal.input('\x15'); terminal.input('Inspect only'); terminal.input('\r');
  await until(client, s => s.messages[1].text === 'Inspect only');
  await call({op: 'frame', frame: {kind: 'input_required', turn_id: id, data: {
    request_id: 'q', questions: [{id: 'target', text: 'Which target?', choices: ['cup', 'bowl']}]}}});
  await until(client, s => !!s.requests.q);
  await chat.refresh();
  terminal.input('\x1b[B'); terminal.input('\r'); await new Promise(r => setTimeout(r, 10));
  // Answers are collected, then explicitly confirmed.
  assert.equal((await call({op: 'writes'})).writes.filter(w => w.reply).length, 0);
  terminal.input('\x1b[B'); terminal.input('\r');
  await until(client, s => s.requests.q?.status !== 'pending');
  assert.deepEqual((await call({op: 'writes'})).writes.find(w => w.reply), {reply: 'q', answers: {target: ['bowl']}});
  await chat.submit('/resume');
  assert.deepEqual(await running, {action: 'history'});
});

test('a typed line answers the question the agent is waiting on', {timeout: 15000}, async t => {
  // A person typed an answer while the agent waited on a question; it went
  // into the queue as a new message and the agent kept waiting.
  const {client, call} = await fixture(t);
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20});
  t.after(() => chat.stop());
  const running = chat.run();
  for (let i = 0; i < 100 && !terminal.input; i++) await new Promise(r => setTimeout(r, 10));
  await chat.submit('Move joint 7');
  const id = (await client.snapshot()).state.active;
  await call({op: 'frame', frame: {kind: 'turn_started', turn_id: id, data: {}}});
  await call({op: 'frame', frame: {kind: 'input_required', turn_id: id, data: {
    request_id: 'q', questions: [{id: '0', text: 'How should I proceed?', choices: ['Look first', 'Retry'],
      allow_other: true}]}}});
  await until(client, s => !!s.requests.q);
  await chat.refresh();
  await chat.submit('use position control, then try the other joints');
  await until(client, s => s.requests.q?.status !== 'pending');
  const {writes} = await call({op: 'writes'});
  assert.deepEqual(writes.find(w => w.reply),
    {reply: 'q', answers: {0: ['use position control, then try the other joints']}});
  assert.equal((await client.snapshot()).state.messages.length, 1);   // not queued as a message
  terminal.input('\x04');
  await running;
});

test('archive mode cannot enqueue or end a session', async () => {
  const {ArchiveClient} = await import('../src/screens.mjs');
  const client = new ArchiveClient({snapshot: {state: {messages: [], requests: {}, closed: true}}, events: [], reason: 'Ended'});
  await assert.rejects(client.command('enqueue', {text: 'Move'}), /Read-only/);
  await assert.rejects(client.end(), /Read-only/);
});

test('keyboard configuration and filtered history use Pi selectors', async () => {
  const {Setup, History} = await import('../src/screens.mjs');
  const terminal = new Terminal();
  const spec = {values: {robot: 'panda', sim: 'robosuite', bench: '', agent: 'one', model: 'm1', name: 'id'},
    choices: {robot: ['panda', 'ur5'], agent: ['one', 'two']}, models: {two: 'm2'}, location: 'config.yaml', notice: ''};
  const setup = new Setup(spec, terminal);
  const running = setup.run();
  terminal.input('\r'); terminal.input('\x1b[B'); terminal.input('\r');
  assert.equal(setup.values.robot, 'ur5');
  // Robot, simulator, benchmark, then agent.
  for (let i = 0; i < 3; i++) terminal.input('\x1b[B');
  terminal.input('\r'); terminal.input('\x1b[B'); terminal.input('\r');
  assert.deepEqual(await running, {action: 'agent', agent: 'two', values: {...spec.values, robot: 'ur5'}});
  // Backend resolves the agent, then the user reviews the new selection.
  const nextTerminal = new Terminal();
  const nextValues = {...spec.values, robot: 'ur5', agent: 'two', model: 'm2'};
  const next = new Setup({...spec, values: nextValues}, nextTerminal);
  const checking = next.run();
  for (let i = 0; i < 7; i++) nextTerminal.input('\x1b[B');
  nextTerminal.input('\r');
  assert.deepEqual(await checking, {action: 'check', values: nextValues});
  const historyTerminal = new Terminal();
  const history = new History({rows: [{id: 'a', title: 'Cup', status: 'Ended', time: ''},
    {id: 'b', title: 'Bowl', status: 'Service recorded', time: ''}]}, historyTerminal);
  const selected = history.run();
  historyTerminal.input('Bowl'); historyTerminal.input('\r');
  assert.deepEqual(await selected, {action: 'select', id: 'b'});
});

test('authentication uses explicit keyboard selection and clears the path on native login', async () => {
  const {Setup} = await import('../src/screens.mjs');
  const terminal = new Terminal();
  const spec = {values: {robot: 'panda', sim: 'robosuite', bench: '', agent: 'one', model: '', name: 'id',
    auth_mode: 'native', api_key_file: ''}, choices: {}, models: {}, location: 'config.yaml', notice: '',
    authentication: {one: {api: true}, two: {api: true, auth_mode: 'api', api_key_file: '/keys/two.key'},
      third: {api: false}}};
  const setup = new Setup(spec, terminal);
  const running = setup.run();
  setup.edit('auth_mode', 'Authentication');
  terminal.input('\x1b[B'); terminal.input('\r');
  assert.equal(setup.values.auth_mode, 'api');
  setup.edit('api_key_file', 'API key file path');
  assert.match(setup.body.render(120).join('\n'), /Do not paste the key here/);
  terminal.input('/keys/one.key'); terminal.input('\r');
  assert.equal(setup.values.api_key_file, '/keys/one.key');
  setup.edit('auth_mode', 'Authentication');
  terminal.input('\x1b[A'); terminal.input('\r');
  assert.equal(setup.values.auth_mode, 'native');
  assert.equal(setup.values.api_key_file, '');
  terminal.input('\x04'); await running;
});

test('end requires confirmation and releases execution through the service', {timeout: 15000}, async t => {
  const {client} = await fixture(t);
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20});
  t.after(() => chat.stop());
  const running = chat.run();
  for (let i = 0; i < 100 && !terminal.input; i++) await new Promise(r => setTimeout(r, 10));
  await chat.submit('/end'); terminal.input('\r');
  await new Promise(r => setTimeout(r, 10));
  assert.equal((await client.snapshot()).state.closed, false);
  await chat.submit('/end'); terminal.input('\x1b[B'); terminal.input('\r');
  assert.deepEqual(await running, {action: 'quit'});
  assert.equal(terminal.stopped, true);
});

test('environment menus never offer untested pairs and selection repairs dependent fields', async () => {
  const {Setup} = await import('../src/screens.mjs');
  const terminal = new Terminal();
  const environments = [
    {selection: {robot: 'panda', sim: 'robosuite', bench: ''}},
    {selection: {robot: 'panda-omron', sim: 'robosuite', bench: 'robocasa365'}},
    {selection: {robot: 'widowx', sim: 'maniskill', bench: 'simpler'}},
  ];
  const spec = {values: {robot: 'panda', sim: 'robosuite', bench: '', agent: 'one', model: '', name: 'id'},
    environments, choices: {}, models: {}, location: 'config.yaml', notice: ''};
  const setup = new Setup(spec, terminal);
  const running = setup.run();
  terminal.input('\r'); terminal.input('\x1b[B'); terminal.input('\r');
  assert.equal(setup.values.robot, 'panda-omron');
  assert.equal(setup.values.sim, 'robosuite');
  assert.equal(setup.values.bench, 'robocasa365');
  assert.deepEqual(setup.options('sim'), ['robosuite']);
  assert.deepEqual(setup.options('bench'), ['robocasa365']);
  setup.update('robot', 'widowx');
  assert.equal(setup.values.sim, 'maniskill');
  assert.equal(setup.values.bench, 'simpler');
  terminal.input('\x04');
  await running;
});

test('workspace reads and keyboard previews preserve the conversation and draft', {timeout: 15000}, async t => {
  const {client, call} = await fixture(t);
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20});
  t.after(() => chat.stop());
  const running = chat.run();
  for (let i = 0; i < 100 && !terminal.input; i++) await new Promise(r => setTimeout(r, 10));
  const before = (await client.snapshot()).state;
  chat.editor.setText('Keep my draft');
  const entries = await client.workspaceList();
  assert.equal(entries.entries.find(e => e.name === 'camera.png').kind, 'file');
  assert.equal((await client.workspaceRead('notes #1.txt')).kind, 'text');
  await assert.rejects(client.workspaceRead('../endpoint.json'), /relative/);
  await assert.rejects(client.workspaceRead('blocked'), /symlinks/);
  await chat.submit('/files');
  // Refresh, blocked, camera, notes.
  terminal.input('\x1b[B'); terminal.input('\x1b[B'); terminal.input('\x1b[B'); terminal.input('\r');
  await new Promise(r => setTimeout(r, 80));
  assert.notEqual(chat.panel, null);
  assert.match(terminal.output, /measurement/);
  terminal.input('\x1b[6~'); // Page down through Pi's scroll view.
  terminal.columns = 50; terminal.rows = 18; terminal.resize();
  await new Promise(r => setTimeout(r, 40));
  assert.doesNotMatch(terminal.output, /\x1b\]52;/);
  terminal.input('\x1b'); await new Promise(r => setTimeout(r, 40));
  terminal.input('\x1b');
  assert.equal(chat.editor.getText(), 'Keep my draft');
  assert.deepEqual((await client.snapshot()).state, before);
  assert.equal((await call({op: 'writes'})).writes.filter(w => w.submit).length, 0);
  chat.stop(); await running;
});

test('saved image preview uses Pi image rendering and a terminal fallback', async () => {
  const {FilePreview} = await import('../src/workspace.mjs');
  const {setCapabilities} = await import('@earendil-works/pi-tui');
  setCapabilities({images: null, trueColor: false, hyperlinks: false});
  const preview = new FilePreview({path: 'camera.png', size: 1, kind: 'image', mime: 'image/png',
    data: 'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWZkAAAAASUVORK5CYII='}, () => {}, () => {}, () => {});
  assert.match(preview.render(60).join('\n'), /camera.png/);
  assert.match(preview.render(60).join('\n'), /not a live camera feed/);
});


test('a file preview names the file on this computer and how to open it', async () => {
  const {FilePreview} = await import('../src/workspace.mjs');
  const {setCapabilities} = await import('@earendil-works/pi-tui');
  setCapabilities({images: null, trueColor: false, hyperlinks: true});
  const preview = new FilePreview({path: 'snaps/front.png', size: 1, kind: 'image', mime: 'image/png',
    data: 'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aWZkAAAAASUVORK5CYII='},
    () => {}, () => {}, () => {}, () => 24, '/home/me/.openrua/sandboxes/s/workspace/workspace/snaps/front.png');
  const shown = preview.render(100).join('\n');
  assert.match(shown, /On this computer:.*\/home\/me\/\.openrua\/sandboxes\/s\/workspace\/workspace\/snaps\/front\.png/);
  assert.match(shown, /Ctrl\+click in Cursor or VS Code/);
});

test('the welcome says where you are, in OpenRUA\'s words, and welcomes you back', async () => {
  const {Welcome} = await import('../src/view.mjs');
  const plainText = lines => lines.join('\n').replace(/\x1b\[[0-9;]*m/g, '');
  const sim = new Welcome('s1', {robot: 'panda', robot_model: 'Franka Panda', simulator: 'robosuite',
    backend: 'sim', agent: 'claude-code', agent_title: 'Claude Code', task: 'lift the red cube'});
  let shown = plainText(sim.render(80));
  assert.match(shown, /Welcome to OpenRUA/);
  assert.match(shown, /connected to a Franka Panda, simulated in/);
  assert.match(shown, /Try "what do you see\?"/);
  assert.match(shown, /"lift the red cube"/);
  assert.match(shown, /Esc stops the robot/);
  assert.ok(sim.render(80).slice(0, -1).every(line => line.replace(/\x1b\[[0-9;]*m/g, '').length === 62));
  sim.returning = true;
  assert.match(plainText(sim.render(80)), /Welcome back to OpenRUA[\s\S]*Franka Panda is still up/);
  const real = plainText(new Welcome('s2', {robot: './arm.yaml', robot_model: 'Franka Panda',
    backend: 'real', agent: 'codex', agent_title: 'Codex'}).render(80));
  assert.match(real, /your Franka Panda, with Codex at/);
  assert.match(real, /stop button within reach/);
  assert.match(plainText(new Welcome('old', {}, true).render(80)), /past conversation, read-only/);
});

test('a workspace file the agent names shows with its path on this computer', async () => {
  const {mkdtempSync, mkdirSync, writeFileSync} = await import('node:fs');
  const {tmpdir} = await import('node:os');
  const {join} = await import('node:path');
  const workspace = mkdtempSync(join(tmpdir(), 'ws-'));
  mkdirSync(join(workspace, 'snaps')); writeFileSync(join(workspace, 'snaps', 'front.png'), 'x');
  const transcript = new Transcript({workspace});
  transcript.event({kind: 'agent_event', data: {turn_id: 't', kind: 'item', data: {item_id: 'm', kind: 'message',
    text: 'Here it is: [front](/workspace/snaps/front.png). No /workspace/missing.png though.'}}});
  const shown = transcript.render(120).join('\n').replace(/\x1b\[[0-9;]*m/g, '');
  assert.ok(shown.includes(`↳ ${join(workspace, 'snaps', 'front.png')}`));
  assert.ok(!shown.includes(join(workspace, 'missing.png')));
});

test('a native error emitted as both message and result is displayed once', () => {
  const view = new Transcript();
  const error = 'Failed to authenticate: OAuth session expired and could not be refreshed';
  view.event({kind: 'agent_event', data: {kind: 'item', turn_id: 'failed',
    data: {kind: 'message', item_id: 'auth', text: error}}});
  view.event({kind: 'agent_event', data: {kind: 'turn_finished', turn_id: 'failed',
    data: {status: 'failed', error: {message: error}}}});
  assert.equal(view.render(120).join('\n').split(error).length - 1, 1);
});

test('archived conversations browse files without gaining execution commands', {timeout: 15000}, async t => {
  const {client, call} = await fixture(t);
  const {ArchiveClient} = await import('../src/screens.mjs');
  const archive = new ArchiveClient({snapshot: await client.snapshot(), events: [], reason: 'Ended',
    workspace: {url: client.origin, token: client.token}});
  t.after(() => archive.close());
  const before = await call({op: 'writes'});
  assert.equal((await archive.workspaceRead('notes #1.txt')).kind, 'text');
  assert.equal((await archive.workspaceRead('camera.png')).kind, 'image');
  assert.ok((await archive.workspaceList()).entries.length > 0);
  await assert.rejects(archive.command('enqueue', {}), /Read-only/);
  await assert.rejects(archive.end(), /Read-only/);
  assert.deepEqual(await call({op: 'writes'}), before);
});

test('entering a plugin path requests backend resolution before changing authentication', async () => {
  const {Setup} = await import('../src/screens.mjs');
  const terminal = new Terminal();
  const values = {robot: 'panda', sim: 'robosuite', bench: '', agent: 'one',
    model: 'saved-model', name: 'id', auth_mode: 'api', api_key_file: '/keys/one.key'};
  const setup = new Setup({values, choices: {agent: ['one']}, models: {},
    authentication: {one: {api: true}}, location: 'config', notice: ''}, terminal);
  const running = setup.run();
  setup.edit('agent', 'Coding agent', true);
  terminal.input('./external/agent.yaml'); terminal.input('\r');
  await running;
  assert.equal(setup.done, true);
  assert.deepEqual(setup.result, {action: 'agent', agent: './external/agent.yaml', values});
  assert.equal(setup.values.agent, 'one');
});

test('keyboard questions preserve multiple choices and optional text until confirmed', {timeout: 15000}, async t => {
  const {client, call} = await fixture(t);
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20});
  t.after(() => chat.stop());
  void chat.run();
  for (let i = 0; i < 100 && !terminal.input; i++) await new Promise(r => setTimeout(r, 10));
  await chat.submit('Inspect');
  const id = (await client.snapshot()).state.active;
  await call({op: 'frame', frame: {kind: 'input_required', turn_id: id, data: {
    request_id: 'multi', questions: [
      {id: 'locations', text: 'Which locations?', choices: ['Near', 'Far'], multiple: true, allow_other: true},
      {id: 'color', text: 'Which color?', choices: ['Red', 'Blue'], allow_other: true},
    ]}}});
  await until(client, s => !!s.requests.multi); await chat.refresh();
  const key = async input => { terminal.input(input); await new Promise(r => setTimeout(r, 20)); };
  const replies = async () => (await call({op: 'writes'})).writes.filter(w => w.reply);
  // Empty selection cannot continue; Space toggles without submitting.
  await key('\x1b[A'); await key('\r');
  assert.match(terminal.output, /Choose at least one answer/);
  await key('\x1b[B'); await key(' '); await key(' '); await key(' ');
  await key('\x1b[B'); await key('\r');
  // Escape discards this unsubmitted answer set.
  await key('\x1b');
  assert.deepEqual(await replies(), []);
  await chat.submit('/questions'); await key('\r');
  await key(' '); await key('\x1b[B'); await key('\r');
  await key('\x1b[B'); await key('\r'); // Write another answer
  await key('Shelf'); await key('\r');
  await key('\x1b[B'); await key('\r'); // Continue to the color question
  await key('\x1b[B'); await key('\x1b[B'); await key('\r');
  await key('Green'); await key('\r');
  assert.deepEqual(await replies(), []);
  await key('\x1b[B'); await key('\r'); // Final explicit confirmation
  await until(client, s => s.requests.multi?.status !== 'pending');
  assert.deepEqual(await replies(), [{reply: 'multi', answers: {locations: ['Near', 'Far', 'Shelf'], color: ['Green']}}]);
  assert.equal((await client.snapshot()).state.messages.length, 1);
});

test('pending questions open automatically, preserve drafts and never approve on a stray Enter', {timeout: 15000}, async t => {
  const {client, call} = await fixture(t);
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20});
  t.after(() => chat.stop());
  void chat.run();
  for (let i = 0; i < 100 && !terminal.input; i++) await new Promise(r => setTimeout(r, 10));
  await chat.submit('Inspect');
  chat.editor.setText('accept');
  const id = (await client.snapshot()).state.active;
  await call({op: 'frame', frame: {kind: 'input_required', turn_id: id, data: {
    request_id: 'approval', questions: [{id: 'decision', text: 'Approve command: printf fixture?', choices: ['accept', 'decline']}]}}});
  await until(client, state => !!state.requests.approval);
  await chat.refresh();
  assert.equal(chat.panel.requestId, 'approval');
  assert.match(chat.panel.component.render(80).join('\n'), /Your response is needed/);
  assert.match(chat.panel.component.render(80).join('\n'), /printf fixture/);
  assert.match(chat.status.text, /Waiting for your response/);
  chat.lastEvent = Date.now() - 120000;
  await chat.refresh();
  assert.doesNotMatch(chat.status.text, /Working|no output/);
  const key = async input => { terminal.input(input); await new Promise(r => setTimeout(r, 20)); };
  await key('\r'); // Select an answer, but do not send it.
  assert.match(chat.panel.component.render(80).join('\n'), /Review your response/);
  assert.match(chat.panel.component.render(80).join('\n'), /Your answer: accept/);
  assert.equal((await call({op: 'writes'})).writes.filter(w => w.reply).length, 0);
  await key('\r'); // Confirmation initially selects Cancel.
  assert.equal(chat.panel, null);
  assert.equal(chat.editor.getText(), 'accept');
  await chat.refresh();
  assert.equal(chat.panel, null);
  assert.match(chat.status.text, /Waiting for your response/);
  await chat.submit('accept');
  assert.equal((await call({op: 'writes'})).writes.filter(w => w.reply).length, 0);
  await client.command('respond', {request_id: 'approval', answers: {decision: ['decline']}});
  await chat.refresh();
  assert.doesNotMatch(chat.status.text, /Waiting for your response/);
});

test('a waiting question opens on reconnect and closes when another client answers', {timeout: 15000}, async t => {
  const {client, call} = await fixture(t);
  const message = await client.command('enqueue', {client_id: 'other', request_id: 'start', text: 'Inspect'});
  await until(client, state => state.active === message.id);
  await call({op: 'frame', frame: {kind: 'input_required', turn_id: message.id, data: {
    request_id: 'waiting', questions: [{id: 'answer', text: 'Ready to continue?', choices: ['yes', 'no']}]}}});
  await until(client, state => !!state.requests.waiting);
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20});
  t.after(() => chat.stop());
  void chat.run();
  for (let i = 0; i < 100 && !terminal.input; i++) await new Promise(r => setTimeout(r, 10));
  assert.equal(chat.panel.requestId, 'waiting');
  terminal.input('\r'); await new Promise(r => setTimeout(r, 20));
  assert.match(chat.panel.component.render(80).join('\n'), /Review your response/);
  await client.command('respond', {request_id: 'waiting', answers: {answer: ['no']}});
  await chat.refresh();
  assert.equal(chat.panel, null);
});

test('new questions wait for an existing dialog and remain visible after dismissal', {timeout: 15000}, async t => {
  const {client, call} = await fixture(t);
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20});
  t.after(() => chat.stop());
  void chat.run();
  for (let i = 0; i < 100 && !terminal.input; i++) await new Promise(r => setTimeout(r, 10));
  await chat.submit('Inspect');
  chat.picker('Another dialog', [{value: 'back', label: 'Back'}], () => {});
  const old = chat.panel;
  const id = (await client.snapshot()).state.active;
  await call({op: 'frame', frame: {kind: 'input_required', turn_id: id, data: {
    request_id: 'later', questions: [{id: 'a', text: 'Answer this next.', choices: ['yes', 'no']}]}}});
  await until(client, state => !!state.requests.later); await chat.refresh();
  assert.equal(chat.panel, old);
  assert.match(chat.status.text, /Waiting for your response/);
  old.hide(); await chat.refresh();
  assert.equal(chat.panel.requestId, 'later');
});

test('follow-up messages leave the pending area and keep their place when history is replayed', async () => {
  const {PendingMessages} = await import('../src/view.mjs');
  const view = new Transcript();
  const pending = new PendingMessages();
  const active = {id: 'active', text: 'Inspect', status: 'running'};
  const text = 'left_wrist:\n  type: opencv\n  width: 960\nright_wrist:\n  type: opencv';
  const message = {id: 'extra', text, status: 'steering', joined: 'active'};
  view.snapshot({messages: [active, message]});
  view.event({kind: 'agent_event', data: {kind: 'item', turn_id: 'active', data: {
    item_id: 'before', kind: 'message', text: 'Before the new camera configuration.', phase: 'completed'}}});
  pending.messages = [message];
  assert.equal(pending.render(80).length, 1);
  assert.match(pending.render(80).join(''), /Adding to current turn/);
  assert.doesNotMatch(view.render(80).join('\n'), /left_wrist/);
  const delivered = {...message, status: 'steered'};
  view.snapshot({messages: [active, delivered]});
  pending.messages = [delivered];
  assert.deepEqual(pending.render(80), []);
  view.event({kind: 'agent_event', data: {kind: 'item', turn_id: 'active', data: {
    item_id: 'after', kind: 'message', text: 'After the new camera configuration.', phase: 'completed'}}});
  view.event({kind: 'message_accepted', data: {message: {...message, status: 'queued'}}});
  const rendered = view.render(80).join('\n');
  assert.ok(rendered.indexOf('Before the new') < rendered.indexOf('left_wrist'));
  assert.ok(rendered.indexOf('left_wrist') < rendered.indexOf('After the new'));
  assert.match(rendered, /Received by agent/);
  assert.equal(view.children.length, 1);

  const restored = new Transcript();
  restored.snapshot({messages: [active, delivered]});
  restored.event({kind: 'message_accepted', data: {message: {...message, status: 'queued'}}});
  restored.event({kind: 'agent_event', data: {kind: 'item', turn_id: 'active', data: {
    item_id: 'before', kind: 'message', text: 'Before the new camera configuration.', phase: 'completed'}}});
  restored.event({kind: 'agent_event', data: {kind: 'steer_delivered', turn_id: 'active', data: {message_id: 'extra'}}});
  restored.event({kind: 'agent_event', data: {kind: 'item', turn_id: 'active', data: {
    item_id: 'after', kind: 'message', text: 'After the new camera configuration.', phase: 'completed'}}});
  assert.equal(restored.render(80).join('\n'), rendered);
});

test('long approval prompts stay visible and can be read without approving', async () => {
  const writes = [];
  const chat = new Chat({readOnly: false, command: async (...args) => writes.push(args)}, {terminal: new Terminal()});
  const text = 'Run this command?\n' + Array.from({length: 40}, (_, i) => `command line ${i}`).join('\n');
  chat.controller.state = {requests: {long: {request_id: 'long', status: 'pending',
    questions: [{id: 'decision', text, choices: ['accept', 'decline', 'cancel']}]}}};
  chat.showPendingQuestion();
  let lines = chat.panel.component.render(80);
  assert.ok(lines.length < chat.rows(), `Question uses ${lines.length} rows`);
  assert.match(lines.join('\n'), /Your response is needed/);
  for (let i = 0; i < 20; i++) {
    chat.panel.component.handleInput('\x1b[6~');
    lines = chat.panel.component.render(80);
  }
  assert.match(lines.join('\n'), /command line 39/);
  assert.deepEqual(writes, []);
  chat.panel.component.handleInput('\r');
  await new Promise(resolve => setTimeout(resolve, 0));
  assert.match(chat.panel.component.render(80).join('\n'), /Review your response/);
  assert.match(chat.panel.component.render(80).join('\n'), /Your answer: accept/);
  assert.ok(chat.panel.component.render(80).length < chat.rows());
  chat.panel.component.handleInput('\r');
  await new Promise(resolve => setTimeout(resolve, 0));
  assert.deepEqual(writes, []);
});

test('temporary transport loss shows disconnected status and recovers without losing drafts or queue', {timeout: 15000}, async t => {
  const {client: owner, call} = await fixture(t);
  t.after(() => owner.close());
  let offline = false;
  const proxy = http.createServer((request, response) => {
    if (offline) { request.destroy(); return; }
    const upstream = http.request(owner.origin + request.url, {
      method: request.method, headers: {...request.headers, host: new URL(owner.origin).host},
    }, result => {
      response.writeHead(result.statusCode, result.headers); result.pipe(response);
    });
    upstream.on('error', () => response.destroy());
    request.pipe(upstream);
  }).listen(0, '127.0.0.1');
  await once(proxy, 'listening');
  t.after(() => proxy.close());
  const client = new Client(`http://127.0.0.1:${proxy.address().port}`, owner.token);
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20});
  t.after(() => chat.stop());
  const running = chat.run();
  const waitFor = async condition => {
    for (let i = 0; i < 300; i++) {
      if (condition()) return;
      await new Promise(resolve => setTimeout(resolve, 10));
    }
    assert.ok(condition(), 'client state did not converge');
  };
  await waitFor(() => Boolean(terminal.input));
  await chat.submit('Inspect the robot');
  const first = (await owner.snapshot()).state.active;
  await waitFor(() => /Working/.test(chat.status.render(120).join('\n')));
  chat.editor.setText('My unsent follow-up');
  chat.say('Use /files to inspect saved observations.');
  offline = true;
  await waitFor(() => /Disconnected.*retrying/.test(chat.status.render(120).join('\n')));
  assert.doesNotMatch(chat.status.render(120).join('\n'), /Working/);
  await owner.command('enqueue', {client_id: 'other-ui', request_id: 'queued', text: 'Then wait'});
  await call({op: 'frame', frame: {kind: 'item', turn_id: first, data: {
    item_id: 'during-disconnect', kind: 'tool', phase: 'completed', text: 'Read', output: 'camera saved'}}});
  offline = false;
  await waitFor(() => /Working.*1 queued/.test(chat.status.render(120).join('\n')) && chat.transcript.tools.size === 1);
  assert.equal(chat.editor.getText(), 'My unsent follow-up');
  assert.match(chat.notice.render(120).join('\n'), /Use \/files/);
  assert.doesNotMatch(chat.status.render(120).join('\n'), /Disconnected/);
  assert.equal(chat.controller.state.messages.length, 2);
  assert.equal((await call({op: 'writes'})).writes.filter(frame => frame.submit).length, 1);
  chat.stop(); await running;
});

test('tool completion retains invocation details alongside output and empty results', () => {
  const view = new Transcript();
  const event = data => view.event({kind: 'agent_event', data: {kind: 'item', turn_id: 'turn',
    data: {item_id: 'tool', kind: 'tool', text: 'Bash', ...data}}});
  event({phase: 'started', details: {command: 'ros2 topic list'}});
  view.toggleTool('turn:tool');
  event({phase: 'completed', output: '/camera'});
  let rendered = view.render(100).join('\n');
  assert.match(rendered, /ros2 topic list/);
  assert.match(rendered, /\/camera/);
  event({phase: 'completed'});
  assert.match(view.render(100).join('\n'), /\/camera/);
  event({phase: 'completed', output: ''});
  rendered = view.render(100).join('\n');
  assert.match(rendered, /ros2 topic list/);
  assert.doesNotMatch(rendered, /\/camera/);
  assert.equal(view.tools.size, 1);
  // A long line is cut to the width while folded and wrapped once opened.
  view.toggleTool('turn:tool');
  event({phase: 'completed', output: 'x'.repeat(150)});
  rendered = view.render(40).join('\n');
  assert.match(rendered, /x+.*…/);
  assert.doesNotMatch(rendered, /x{40}/);
  view.toggleTool('turn:tool');
  assert.match(view.render(40).join('\n'), /x{36}\n/);
});

test('dialogs open below the transcript in the editor slot and Escape closes them without interrupting', {timeout: 15000}, async t => {
  const {Text} = await import('@earendil-works/pi-tui');
  const {client, call} = await fixture(t);
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20});
  t.after(() => chat.stop());
  const running = chat.run();
  for (let i = 0; i < 100 && !terminal.input; i++) await new Promise(r => setTimeout(r, 10));
  await chat.submit('Inspect');
  const id = (await client.snapshot()).state.active;
  await call({op: 'frame', frame: {kind: 'turn_started', turn_id: id, data: {}}});
  chat.transcript.addChild(new Text('TRANSCRIPT '.repeat(20), 0, 0));
  const slot = chat.ui.children.indexOf(chat.editor);
  chat.editor.setText('Keep my draft');
  terminal.output = '';
  await chat.submit('/files');
  await new Promise(r => setTimeout(r, 80));
  assert.notEqual(chat.panel, null);
  assert.equal(chat.ui.children[slot], chat.panel.component);
  assert.equal(chat.ui.children.includes(chat.editor), false);
  assert.equal(chat.ui.hasOverlay(), false);
  const screen = terminal.output.replace(/\x1b\[[0-9;]*[A-Za-z]/g, '');
  assert.match(screen, /Workspace \//);
  assert.doesNotMatch(screen, /[│┌└]/);
  assert.ok(screen.indexOf('TRANSCRIPT') < screen.indexOf('Workspace /'));
  // Refresh, blocked, camera, notes: open the long text and page through it.
  terminal.input('\x1b[B'); terminal.input('\x1b[B'); terminal.input('\x1b[B'); terminal.input('\r');
  await new Promise(r => setTimeout(r, 80));
  const preview = chat.panel.component;
  assert.equal(preview.offset, 0);
  assert.match(preview.render(80).join('\n'), /lines 1-12 of 82/);
  terminal.input('\x1b[6~');
  assert.equal(preview.offset, 11);
  terminal.input('\x1b[F');
  assert.equal(preview.offset, 82 - 12);
  terminal.input('\x1b'); await new Promise(r => setTimeout(r, 80));
  assert.match(chat.panel.component.render(80).join('\n'), /Workspace \//);
  terminal.input('\x1b'); await new Promise(r => setTimeout(r, 20));
  assert.equal(chat.panel, null);
  assert.equal(chat.ui.children[slot], chat.editor);
  assert.equal(chat.editor.getText(), 'Keep my draft');
  assert.equal((await call({op: 'writes'})).writes.filter(w => w.interrupt).length, 0);
  assert.equal((await client.snapshot()).state.messages[0].cancel_requested, false);
  chat.stop(); await running;
});

test('/permissions and /model change the agent; its own commands go to it', {timeout: 15000}, async t => {
  const {client, call} = await fixture(t);
  const chat = new Chat(client, {terminal: new Terminal(), pollMs: 20});
  t.after(() => chat.stop());
  await chat.refresh();
  await call({op: 'frame', frame: {kind: 'commands', data: {commands: [
    {name: 'compact', description: 'summarize'}, {name: 'help', description: 'not shadowing ours'}]}}});
  await until(client, s => (s.agent_commands ?? []).length === 2);
  await chat.refresh();
  assert.deepEqual(chat.agentCommands().map(c => c.name), ['compact']);
  await call({op: 'frame', frame: {kind: 'setting', data: {setting: 'permissions', value: 'auto-review',
    choices: [{value: 'ask', label: 'Manual'}, {value: 'auto-review', label: 'Approve for me'}]}}});
  await until(client, s => s.setting_choices?.permissions?.length === 2);
  await chat.refresh();
  assert.match(chat.status.text, /Approve for me/);
  await chat.submit('/permissions');
  assert.match(chat.panel.component.render(80).join('\n'), /Approve for me/);
  assert.doesNotMatch(chat.panel.component.render(80).join('\n'), /Run commands without asking/);
  chat.panel.hide();
  await chat.submit('/permissions ask');
  assert.equal((await client.snapshot()).state.settings.permissions, 'ask');
  assert.match(chat.status.text, /asks before commands/);
  await chat.submit('/model other-model');
  assert.equal((await client.snapshot()).state.settings.model, 'other-model');
  const writes = (await call({op: 'writes'})).writes;
  assert.deepEqual(writes.filter(w => w.configure).map(w => [w.configure, w.value]),
    [['permissions', 'ask'], ['model', 'other-model']]);
  // The agent's own command is a message to it; an unknown one is refused.
  await chat.submit('/compact');
  const state = await until(client, s => s.messages.length === 1);
  assert.equal(state.messages[0].text, '/compact');
  await chat.submit('/nosuch');
  assert.match(chat.notice.text, /Unknown command/);
});

test('the terminal offers every command the shared list gives it', async () => {
  // One list of commands for every client (openrua/web/session.mjs); a
  // command added there without a handler here fails this.
  const {commandsFor} = await import('../src/session.mjs');
  const chat = new Chat({readOnly: false}, {terminal: new Terminal()});
  assert.deepEqual(Object.keys(chat.handlers()).sort(), commandsFor('terminal').map(c => c.name).sort());
});

test('what the session means is read the same way by every client', async () => {
  const {notice, parseCommand, status} = await import('../src/session.mjs');
  const state = {messages: [{status: 'queued'}], requests: {r: {status: 'pending'}}, connected: false,
    pause_reason: 'agent disconnected: exit 1', paused: true, pause_id: 'p', active: null, closed: false,
    settings: {permissions: 'ask'}, robot: {status: 'stopped', reason: 'driver exited'},
    checks: {host: {status: 'passed'}}, agent_commands: [{name: 'compact'}, {name: 'help'}]};
  const {head, notes} = status(state);
  assert.equal(head.kind, 'paused');
  assert.deepEqual(notes.map(n => n.text), ['robot stopped: driver exited', 'host ✓', 'asks before commands',
    '1 queued', 'agent disconnected: exit 1', '1 question(s): type the answer, or /questions to choose']);
  assert.equal(status(state, {readOnly: true}).head.kind, 'readonly');
  assert.deepEqual(parseCommand('/model m2', state), {kind: 'ours', name: 'model', args: ['m2']});
  assert.equal(parseCommand('/compact', state).kind, 'agent');
  assert.equal(parseCommand('/nope', state).kind, 'unknown');
  assert.equal(parseCommand('lift the cube', state), null);
  assert.equal(notice({kind: 'motion_canceled', data: {canceled: ['/fjt'], failed: []}}), 'Motion canceled: /fjt');
  assert.match(notice({kind: 'agent_event', data: {kind: 'setting_rejected',
    data: {setting: 'model', value: 'x', reason: 'unknown model'}}}), /did not take model x: unknown model/);
  assert.equal(notice({kind: 'agent_event', data: {kind: 'item'}}), null);
});

test('a typed line answers a question the same way in every client', async () => {
  const {typedAnswer, toolSummary} = await import('../src/session.mjs');
  const asking = questions => ({requests: {r: {request_id: 'r', status: 'pending', questions}}});
  assert.equal(typedAnswer({requests: {}}, 'hi'), null);
  assert.deepEqual(typedAnswer(asking([{id: 'q', text: 'How?'}]), ' position '),
    {kind: 'answer', request_id: 'r', answers: {q: ['position']}});
  assert.equal(typedAnswer(asking([{id: 'q', choices: ['Allow', 'Deny']}]), 'maybe').kind, 'choose');
  assert.deepEqual(typedAnswer(asking([{id: 'q', choices: ['allow', 'deny']}]), 'Allow').answers, {q: ['allow']});
  assert.equal(typedAnswer(asking([{id: 'q', secret: true}]), 'pw').kind, 'choose');
  assert.equal(toolSummary({command: 'ls\nmore'}), 'ls');
  assert.equal(toolSummary({file_path: '/workspace/a.md'}), '/workspace/a.md');
});

test('workspace consent can accept, choose another directory, or quit', async () => {
  const {Workspace} = await import('../src/screens.mjs');
  const firstTerminal = new Terminal();
  const first = new Workspace({path: '/project', notice: ''}, firstTerminal);
  const accepted = first.run(); firstTerminal.input('\r');
  assert.deepEqual(await accepted, {action: 'trust', path: '/project'});
  const otherTerminal = new Terminal();
  const other = new Workspace({path: '/project', notice: ''}, otherTerminal);
  const chosen = other.run(); otherTerminal.input('\x1b[B'); otherTerminal.input('\r');
  otherTerminal.input('/another'); otherTerminal.input('\r');
  assert.deepEqual(await chosen, {action: 'choose', path: '/another'});
  const quitTerminal = new Terminal();
  const quit = new Workspace({path: '/project', notice: ''}, quitTerminal);
  const canceled = quit.run(); quitTerminal.input('\x04');
  assert.deepEqual(await canceled, {action: 'quit'});
});

test('history scope switch is explicit and shows project identity', async () => {
  const {History} = await import('../src/screens.mjs');
  const terminal = new Terminal();
  const history = new History({rows: [{id: 'a', title: 'Cup', status: 'Ended', time: '', project: '/project'}],
    scope: '/project', can_all: true}, terminal);
  const result = history.run();
  assert.match(history.ui.children.map(c => c.render?.(100)?.join('\n') ?? '').join('\n'), /\/project/);
  terminal.input('\t');
  assert.deepEqual(await result, {action: 'scope'});
});

test('/resume --all opens all-project history without sending an agent instruction', async () => {
  const chat = new Chat({readOnly: true}, {terminal: new Terminal(), history: true});
  await chat.submit('/resume --all');
  assert.deepEqual(chat.result, {action: 'history_all'});
});
