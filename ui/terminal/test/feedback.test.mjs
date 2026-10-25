import assert from 'node:assert/strict';
import {test} from 'node:test';
import {SettingChanges, notice, status} from '../src/session.mjs';
import {Controller} from '../src/controller.mjs';

const event = (setting, value, kind = 'setting', extra = {}) => ({kind: 'agent_event',
  data: {kind, data: {setting, value, effective_from: 'next_turn', ...extra}}});

test('a model and effort change is confirmed once, including later catalog refreshes', () => {
  const messages = [], changes = new SettingChanges(message => messages.push(message));
  const state = {settings: {model: 'new-model', effort: 'high'}};
  const change = changes.begin(state.settings);
  changes.receive(event('model', 'new-model'), state);
  changes.receive(event('effort', 'high'), state);
  assert.equal(changes.complete(change, state), true);
  changes.receive(event('effort', 'high'), state);
  assert.deepEqual(messages, ['Model changed to new-model · high.']);
  assert.equal(notice(event('effort', 'high')), null);
});

test('pending native confirmation becomes one success or one rejection, never an early success', () => {
  const messages = [], changes = new SettingChanges(message => messages.push(message));
  const state = {settings: {model: 'old-model'}};
  const first = changes.begin({model: 'new-model'});
  assert.equal(changes.complete(first, state), false);
  assert.deepEqual(messages, ['Updating model…']);
  state.settings.model = 'new-model';
  changes.receive(event('model', 'new-model'), state);
  changes.receive(event('model', 'new-model'), state);
  assert.deepEqual(messages, ['Updating model…', 'Model changed to new-model.']);
  const second = changes.begin({model: 'unavailable'});
  changes.complete(second, state);
  changes.receive(event('model', 'unavailable', 'setting_rejected', {reason: 'Native model unavailable'}), state);
  assert.match(messages.at(-1), /Native model unavailable/);
  assert.equal(messages.some(message => message === 'Model changed to unavailable.'), false);
  assert.equal(changes.pending.size, 0);
});

test('a synchronous rejection throws once and superseded choices do not produce late confirmations', () => {
  const messages = [], changes = new SettingChanges(message => messages.push(message));
  const state = {settings: {model: 'old'}};
  const rejected = changes.begin({model: 'bad'});
  changes.receive(event('model', 'bad', 'setting_rejected', {reason: 'Not supported'}), state);
  assert.throws(() => changes.complete(rejected, state), /Not supported/);
  assert.deepEqual(messages, []);
  const first = changes.begin({model: 'first'});
  const last = changes.begin({model: 'last'});
  state.settings.model = 'first';
  changes.receive(event('model', 'first'), state);
  assert.equal(changes.complete(first, state), false);
  state.settings.model = 'last';
  changes.receive(event('model', 'last'), state);
  changes.complete(last, state);
  assert.deepEqual(messages, ['Model changed to last.']);
});

test('history remains a replay across batches while new events are delivered live', async () => {
  let cursor = 1002;
  const records = Array.from({length: cursor}, (_, index) => ({seq: index + 1, ...event('effort', 'high')}));
  const seen = [];
  const client = {async snapshot() { return {state: {messages: []}, cursor}; },
    async events(after) { return records.filter(record => record.seq > after).slice(0, 1000); }};
  const controller = new Controller(client, {snapshot() {}, event(record, options) { seen.push(options.replay); }});
  await controller.refresh();
  assert.equal(seen.length, 1002);
  assert.equal(seen.every(Boolean), true);
  records.push({seq: ++cursor, ...event('effort', 'medium')});
  await controller.refresh();
  assert.equal(seen.at(-1), false);
});

test('interruption is pending until execution finishes and does not leave a success notice behind', () => {
  const state = {connected: true, messages: [{id: 'active', cancel_requested: true}], active: 'active'};
  assert.equal(status(state).head.text, 'Interrupting…');
  state.active = null;
  assert.equal(status(state).head.text, 'Ready');
});


test('reopened conversations retain the connection-specific context supplied by the owner', () => {
  const note = 'The robot connection changed. Read the project connection notes and obtain a fresh camera image.';
  assert.equal(notice({kind: 'session_reopened', data: {note}}), note);
});

test('environment changes keep their supplied reason visible', () => {
  const reason = 'The project changed the driver tool image to include its new ROS package.';
  assert.ok(notice({kind: 'handed_over', data: {reason}}).includes(reason));
});
