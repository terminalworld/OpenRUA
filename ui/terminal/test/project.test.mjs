import assert from 'node:assert/strict';
import {test} from 'node:test';
import {Chat} from '../src/app.mjs';
import {projectMenu, showReview} from '../src/project.mjs';
import {schedulesMenu} from '../src/schedules.mjs';
import {parseCommand} from '../src/session.mjs';

class Terminal {
  columns = 100; rows = 30; output = '';
  start(input) { this.input = input; }
  stop() {} write(data) { this.output += data; }
  async drainInput() {}
  moveBy() {} hideCursor() {} showCursor() {} clearLine() {}
  clearFromCursor() {} clearScreen() {} setTitle() {} setProgress() {}
}

const tick = () => new Promise(resolve => setTimeout(resolve, 5));
async function choose(chat, index) {
  assert.ok(chat.panel, 'a menu must be visible');
  chat.panel.component.children[1].setSelectedIndex(index);
  chat.panel.component.handleInput('\r');
  await tick();
}

async function setup(t, respond = () => ({})) {
  const state = {messages: [], requests: {}, connected: true, closed: false,
    settings: {model: 'gpt-5.6-sol', effort: 'high'}, agent_commands: [{name: 'skills', description: 'Native skills'}],
    setting_choices: {model: [{value: 'gpt-6-astra', label: 'GPT-6 Astra', description: 'Native model'}],
      effort: [{value: 'medium', label: 'medium'}, {value: 'high', label: 'high'}]}};
  const calls = [];
  const client = {
    async snapshot() { return {state}; }, async events() { return []; }, close() {},
    async projectSkills(operation, params) { calls.push({operation, params}); return respond(operation, params); },
    async command(operation, params) {
      calls.push({command: operation, params});
      if (operation === 'configure') state.settings[params.setting] = params.value;
    },
  };
  const terminal = new Terminal();
  const chat = new Chat(client, {terminal, pollMs: 20});
  void chat.run();
  t.after(() => chat.stop());
  while (!terminal.input) await tick();
  return {chat, terminal, state, calls, client};
}

const identity = 'a'.repeat(32);
const candidate = {key: '.agents/skills/camera-check/SKILL.md', name: 'camera-check',
  description: 'Inspect a camera', can_import: true, existing: false, fingerprint: 'candidate-digest'};
const overview = {project: '/project', policy: 'auto', entries: [{key: 'saved', name: 'saved-check', scope: 'project', description: 'Existing method'}],
  reports: [{id: identity, status: 'completed', proposed_skills: 1, started_at: 'today'}], tasks: [],
  defaults: {agent: 'codex', model: 'gpt-6-astra', effort: 'high', image: 'prepared-image', wall_clock_min: 5}, agent_identity: 'codex'};
const detail = {report: {id: identity, status: 'completed'}, summary: 'Evidence from completed work',
  diff: '+new method', candidates: [candidate]};

test('project menu previews skills while /skills stays native', {timeout: 5000}, async t => {
  const {chat, calls, state} = await setup(t, operation => operation === 'overview' ? overview
    : {path: 'SKILL.md', kind: 'text', text: 'Saved skill instructions', size: 24});
  await chat.submit('/project');
  assert.match(chat.panel.component.render(100).join('\n'), /Project skills and reviews/);
  await choose(chat, 0); await choose(chat, 0);
  assert.match(chat.panel.component.render(100).join('\n'), /Saved skill instructions/);
  assert.equal(parseCommand('/skills', state).kind, 'agent');
  assert.deepEqual(calls.map(call => call.operation), ['overview', 'skill']);
});

test('starting a review requires confirmation and retries the same unconfirmed ID', {timeout: 5000}, async t => {
  let attempts = 0;
  const {chat, calls} = await setup(t, (operation, params) => {
    if (operation === 'overview') return overview;
    if (operation === 'start') {
      if (++attempts === 1) throw new Error('lost start reply');
      return {id: params.request_id, status: 'running'};
    }
    return {id: chat.reviewTask?.id, status: 'running'};
  });
  await projectMenu(chat); await choose(chat, 4); await choose(chat, 5);
  assert.match(chat.panel.component.render(100).join('\n'), /gpt-6-astra/);
  await choose(chat, 0);
  assert.equal(attempts, 0);
  await projectMenu(chat); await choose(chat, 4); await choose(chat, 5); await choose(chat, 1);
  assert.equal(attempts, 1);
  assert.ok(chat.pendingReview);
  await projectMenu(chat); await choose(chat, 5); await choose(chat, 1);
  assert.equal(attempts, 2);
  const starts = calls.filter(call => call.operation === 'start');
  assert.equal(starts[0].params.request_id, starts[1].params.request_id);
  assert.equal(chat.pendingReview, null);
  assert.equal(calls.some(call => call.command), false);
});

test('candidate import waits for explicit confirmation and never starts an agent turn', {timeout: 5000}, async t => {
  const {chat, calls} = await setup(t, operation => operation === 'report' ? detail
    : {name: candidate.name, imported: '.agents/skills/camera-check', issues: []});
  await showReview(chat, identity); await choose(chat, 2); await choose(chat, 2);
  await choose(chat, 0);
  assert.equal(calls.some(call => call.operation === 'import'), false);
  await showReview(chat, identity); await choose(chat, 2); await choose(chat, 2); await choose(chat, 1);
  assert.equal(calls.filter(call => call.operation === 'import').length, 1);
  assert.equal(calls.some(call => call.command), false);
});

test('existing skill edits prepare a draft without overwriting or sending it as an answer', {timeout: 5000}, async t => {
  const data = {...detail, candidates: [{...candidate, existing: true, source_changed: true, can_import: false}]};
  const {chat, calls, state} = await setup(t, operation => operation === 'report' ? data : {draft: 'Merge the chosen proposal with current edits.'});
  chat.editor.setText('Existing unsent draft');
  await showReview(chat, identity); await choose(chat, 2);
  assert.match(chat.panel.component.render(100).join('\n'), /changed since/);
  await choose(chat, 2); await choose(chat, 1);
  assert.match(chat.editor.getText(), /^Existing unsent draft\n\nMerge/);
  assert.equal(calls.some(call => call.command || call.operation === 'import'), false);
  state.requests.q = {status: 'pending', request_id: 'q', questions: [{id: 'q', text: 'Approve?', allow_other: true}]};
  await chat.submit(chat.editor.getText());
  assert.equal(calls.some(call => call.command), false);
  assert.match(chat.notice.render(100).join('\n'), /Answer the pending agent question/);
});

test('slow review status does not block the normal conversation poll', {timeout: 5000}, async t => {
  let release, snapshots = 0;
  const pending = new Promise(resolve => { release = resolve; });
  const {chat, client} = await setup(t, operation => operation === 'task' ? pending : {});
  const original = client.snapshot;
  client.snapshot = async () => { snapshots++; return original(); };
  chat.reviewTask = {id: identity, status: 'running'};
  await new Promise(resolve => setTimeout(resolve, 100));
  assert.ok(snapshots >= 2);
  release({id: identity, status: 'completed'});
});

test('model setting is visible and the native model menu changes the next-turn configuration', {timeout: 5000}, async t => {
  const {chat, calls} = await setup(t);
  assert.match(chat.status.render(140).join('\n'), /model: gpt-5.6-sol/);
  await chat.submit('/model');
  assert.match(chat.panel.component.render(100).join('\n'), /Select Model/);
  assert.match(chat.panel.component.render(100).join('\n'), /GPT-6 Astra/);
  assert.equal(chat.panel.component.children[1].getSelectedItem().value, 'gpt-5.6-sol');
  assert.equal(calls.some(call => call.command), false);
  await choose(chat, 1);
  assert.equal(calls.find(call => call.command)?.params.value, 'gpt-6-astra');
  assert.match(chat.status.render(140).join('\n'), /model: gpt-6-astra/);
});

test('a running review can be canceled explicitly without ending the robot conversation', {timeout: 5000}, async t => {
  let canceled = false;
  const task = () => ({id: identity, status: canceled ? 'cancel_requested' : 'running'});
  const {chat, calls} = await setup(t, operation => {
    if (operation === 'overview') return {...overview, tasks: [task()]};
    if (operation === 'cancel') canceled = true;
    return task();
  });
  await projectMenu(chat); await choose(chat, 2); await choose(chat, 0); await choose(chat, 1);
  await choose(chat, 0);
  assert.equal(canceled, false);
  await projectMenu(chat); await choose(chat, 2); await choose(chat, 0); await choose(chat, 1); await choose(chat, 1);
  assert.equal(canceled, true);
  assert.equal(chat.done, false);
  assert.equal(calls.some(call => call.command), false);
});

test('model ID entry remains usable when native choices are absent', {timeout: 5000}, async t => {
  const {chat, state, calls} = await setup(t);
  state.setting_choices.model = [];
  await chat.submit('/model'); await choose(chat, 1);
  const editor = chat.panel.component.children[1];
  editor.setText('gpt-6-astra');
  assert.equal(calls.some(call => call.command), false);
  chat.panel.component.handleInput('\r');
  await tick();
  assert.equal(calls.find(call => call.command)?.params.value, 'gpt-6-astra');
});

test('reasoning effort uses native choices and changes only on user selection', {timeout: 5000}, async t => {
  const {chat, calls} = await setup(t);
  await chat.submit('/model effort');
  assert.match(chat.panel.component.render(100).join('\n'), /Reasoning effort setting: high/);
  assert.equal(calls.some(call => call.command), false);
  await choose(chat, 0);
  assert.deepEqual(calls.find(call => call.command)?.params, {setting: 'effort', value: 'medium'});
  assert.match(chat.status.render(140).join('\n'), /effort: medium/);
  await chat.submit('/model effort high');
  assert.equal(calls.filter(call => call.command).at(-1).params.value, 'high');
});

test('changing review agents refreshes their model and effort without starting a task', {timeout: 5000}, async t => {
  const {chat, calls} = await setup(t, (operation, params) => operation === 'overview' ? overview : {
    agent_identity: 'claude-code', defaults: {agent: params.agent, model: 'claude-model', effort: 'medium'}});
  await projectMenu(chat); await choose(chat, 4); await choose(chat, 0);
  chat.panel.component.children[1].setText('claude-code');
  chat.panel.component.handleInput('\r'); await tick();
  const rendered = chat.panel.component.render(100).join('\n');
  assert.match(rendered, /Model: claude-model/);
  assert.match(rendered, /Effort: medium/);
  assert.doesNotMatch(rendered, /gpt-6-astra/);
  assert.equal(calls.some(call => call.operation === 'start' || call.command), false);
});

async function fill(chat, value) {
  chat.panel.component.children[1].setText(value);
  chat.panel.component.handleInput('\r'); await tick();
}

test('periodic review creation is confirmed and retries the same identity after a lost acknowledgement', {timeout: 5000}, async t => {
  const requests = [];
  const {chat, calls} = await setup(t, (operation, params) => {
    if (operation === 'schedules') return {schedules: [], scheduler: {state: 'stopped'}};
    if (operation === 'schedule-create') {
      requests.push(structuredClone(params));
      if (requests.length === 1) throw new Error('reply lost');
      return {name: params.definition.name, enabled: true, next_run: 'tomorrow'};
    }
    return overview;
  });
  const open = () => schedulesMenu(chat, overview, () => {}, () => {});
  await open(); await choose(chat, 0);
  await choose(chat, 0); await fill(chat, 'Camera methods');
  await choose(chat, 1); await fill(chat, '21:30');
  await choose(chat, 5);
  assert.match(chat.panel.component.render(100).join('\n'), /Time limits are not a monetary cap/);
  assert.equal(requests.length, 0);
  await choose(chat, 0); // Cancel the first confirmation.
  assert.equal(requests.length, 0);
  await open(); await choose(chat, 0);
  await choose(chat, 0); await fill(chat, 'Camera methods');
  await choose(chat, 1); await fill(chat, '21:30');
  await choose(chat, 5); await choose(chat, 1);
  assert.match(chat.notice.render(100).join('\n'), /reply lost/);
  await open(); await choose(chat, 1);
  assert.equal(requests.length, 2);
  assert.deepEqual(requests[0], requests[1]);
  assert.equal(chat.pendingSchedule, null);
  assert.equal(calls.some(call => call.command || call.operation === 'start'), false);
});

test('pausing a schedule is explicit and leaves the current review and robot conversation alone', {timeout: 5000}, async t => {
  const plan = {id: identity, name: 'Camera methods', enabled: true, revision: 1, next_run: 'tomorrow',
    calendar: {time: '21:30', days: 'mon', timezone: 'UTC'}, options: overview.defaults, active_task: 'active-review'};
  const {chat, calls} = await setup(t, operation => operation === 'schedules'
    ? {schedules: [plan], scheduler: {state: 'running'}} : plan);
  const open = () => schedulesMenu(chat, overview, () => {}, () => {});
  await open(); await choose(chat, 0); await choose(chat, 1);
  assert.match(chat.panel.component.render(100).join('\n'), /current review continues/);
  await choose(chat, 0);
  assert.equal(calls.some(call => call.operation === 'schedule-action'), false);
  await open(); await choose(chat, 0); await choose(chat, 1); await choose(chat, 1);
  assert.deepEqual(calls.find(call => call.operation === 'schedule-action').params, {id: identity, action: 'pause'});
  assert.equal(calls.some(call => call.command || call.operation === 'cancel'), false);
  assert.equal(chat.done, false);
});

test('robot restart confirms once and retries the same unconfirmed operation', {timeout: 5000}, async t => {
  const {chat, client, calls} = await setup(t);
  let attempts = 0;
  client.request = async (path, body) => {
    calls.push({path, body: {...body}});
    if (++attempts === 1) throw new Error('lost acknowledgement');
    return {result: {...body, status: 'accepted', summary: 'accepted'}};
  };
  await chat.submit('/robot restart');
  assert.equal(attempts, 0);
  await choose(chat, 1);
  assert.equal(attempts, 1);
  assert.ok(chat.pendingRobot);
  await chat.submit('/robot retry');
  assert.equal(attempts, 2);
  assert.equal(chat.pendingRobot, null);
  assert.deepEqual(calls[0].body, calls[1].body);
  assert.equal(calls[0].path, '/api/actions/robot');
});

test('connection failure keeps chat ready and held work distinct from new queued messages', async () => {
  const {status} = await import('../src/session.mjs');
  const {PendingMessages} = await import('../src/view.mjs');
  const state = {connected: true, closed: false, paused: false, active: null, requests: {},
    robot: {status: 'failed', reason: 'driver failed'}, messages: [
      {status: 'queued', text: 'move to bowl', held_reason: 'driver exited'},
      {status: 'queued', text: 'inspect logs'},
    ]};
  const {head, notes} = status(state);
  assert.equal(head.text, 'Ready to chat');
  assert.ok(notes.some(note => /1 held for review/.test(note.text)));
  assert.ok(notes.some(note => note.text === '1 queued'));
  const pending = new PendingMessages(); pending.messages = state.messages;
  assert.match(pending.render(100).join('\n'), /Held for review \(\/queue\): move to bowl/);
});
