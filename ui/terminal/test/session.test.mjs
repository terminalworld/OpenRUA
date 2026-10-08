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
  await call({op: 'frame', frame: {kind: 'item', turn_id: first.id, data: {item_id: 'tool', kind: 'tool', text: 'read image', phase: 'completed', output: 'frame saved'}}});
  for (let i = 0; i < 100 && view.tools.size === 0; i++) {
    await controller.refresh(); await new Promise(resolve => setTimeout(resolve, 5));
  }
  assert.equal(view.tools.size, 1);
  assert.match(view.render(80).join('\n'), /Inspecting/);
  assert.doesNotMatch(view.render(80).join('\n'), /frame saved/);
  view.toggleTool(`${first.id}:tool`);
  assert.match(view.render(80).join('\n'), /frame saved/);
  await client.command('interrupt', {message_id: first.id});
  await call({op: 'frame', frame: {kind: 'turn_finished', turn_id: first.id, data: {status: 'interrupted'}}});
  state = await until(client, s => s.paused && !s.active);
  assert.equal(state.messages[1].status, 'queued');
  const pause = state.pause_id;
  client.close();
  const reconnected = new Controller(other, new Transcript(), 'reconnected');
  await reconnected.refresh();
  assert.equal(reconnected.state.messages.length, 2);
  await other.command('resume', {pause_id: pause});
  await until(other, s => s.active === second.id);
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

test('Pi keyboard editor submits, queues, confirms interruption and detaches', {timeout: 15000}, async t => {
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
  terminal.input('\x1b');
  assert.equal(chat.ui.hasOverlay(), true);
  // Cancel is selected by default, so Escape alone cannot interrupt motion.
  let writes = (await call({op: 'writes'})).writes;
  assert.equal(writes.filter(w => w.interrupt).length, 0);
  terminal.input('\x1b[B'); terminal.input('\r');
  await until(client, s => s.messages[0].cancel_requested);
  await call({op: 'frame', frame: {kind: 'turn_finished', turn_id: id, data: {status: 'interrupted'}}});
  await until(client, s => s.paused && !s.active);
  const observer = new Client(client.origin, client.token);
  t.after(() => observer.close());
  terminal.input('\x04');
  await running;
  state = (await observer.snapshot()).state;
  assert.equal(state.closed, false);
  assert.equal(state.messages[1].status, 'queued');
  assert.equal(state.paused, true);
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
  await chat.refresh(); await chat.submit('/questions');
  terminal.input('\r'); await new Promise(r => setTimeout(r, 10));
  terminal.input('\x1b[B'); terminal.input('\r'); await new Promise(r => setTimeout(r, 10));
  // Answers are collected, then explicitly confirmed.
  assert.equal((await call({op: 'writes'})).writes.filter(w => w.reply).length, 0);
  terminal.input('\x1b[B'); terminal.input('\r');
  await until(client, s => s.requests.q?.status !== 'pending');
  assert.deepEqual((await call({op: 'writes'})).writes.find(w => w.reply), {reply: 'q', answers: {target: ['bowl']}});
  await chat.submit('/resume');
  assert.deepEqual(await running, {action: 'history'});
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
  assert.equal(chat.ui.hasOverlay(), true);
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
  await chat.submit('/questions'); await key('\r');
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
  assert.match(rendered, /Output/);
  assert.equal(view.tools.size, 1);
});

test('dialogs are framed and padded so the transcript cannot show through them', async t => {
  const {Frame} = await import('../src/view.mjs');
  const {Text} = await import('@earendil-works/pi-tui');
  const frame = new Frame(new Text('short\n' + 'x'.repeat(40) + '\nthird\nfourth', 0, 0), {maxLines: () => 5});
  const lines = frame.render(20);
  assert.equal(lines.length, 5);
  assert.match(lines[0], /┌─+┐/);
  assert.match(lines.at(-1), /└─+┘/);
  for (const line of lines) assert.equal((await import('@earendil-works/pi-tui')).visibleWidth(line), 20);
  assert.match(lines[1].replace(/\x1b\[[0-9;]*m/g, ''), /^│short {13}│$/);
  const {client} = await fixture(t);
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20});
  t.after(() => chat.stop());
  const running = chat.run();
  for (let i = 0; i < 100 && !terminal.input; i++) await new Promise(r => setTimeout(r, 10));
  chat.transcript.addChild(new Text('TRANSCRIPT '.repeat(20), 0, 0));
  terminal.output = '';
  await chat.submit('/files');
  await new Promise(r => setTimeout(r, 80));
  const rows = terminal.output.split('\n').map(row => row.replace(/\x1b\[[0-9;]*[A-Za-z]/g, ''));
  const framed = rows.filter(row => row.includes('│'));
  assert.ok(framed.some(row => row.includes('Workspace /')));
  for (const row of framed) assert.doesNotMatch(row.slice(row.indexOf('│'), row.lastIndexOf('│')), /TRANSCRIPT/);
  terminal.input('\x1b');
  chat.stop(); await running;
});
