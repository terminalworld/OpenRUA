import assert from 'node:assert/strict';
import {test} from 'node:test';
import {leave} from '../src/exit.mjs';

function output(answer = '') {
  const lines = [];
  const options = {write: line => lines.push(line), ask: async prompt => {
    lines.push(prompt); return answer;
  }};
  return {lines, options};
}

test('an acknowledged end never probes or ends the closed service again', async () => {
  const {lines, options} = output();
  const client = {snapshot: () => assert.fail('owner is already stopped'),
    end: () => assert.fail('do not repeat the end request')};
  options.ask = () => assert.fail('do not offer a second exit choice');
  assert.equal(await leave('robot', client, {...options, ended: true}), true);
  assert.match(lines.join('\n'), /Session robot ended/);
});

test('another client ending the session supersedes the old open UI state', async () => {
  const {lines, options} = output();
  options.ask = () => assert.fail('the current snapshot is closed');
  const client = {snapshot: async () => ({state: {closed: true}}),
    end: () => assert.fail('the session has ended')};
  assert.equal(await leave('robot', client, options), true);
  assert.doesNotMatch(lines.join('\n'), /is open|background/);
});

test('an unavailable owner has unknown status and is never described as running', async () => {
  const {lines, options} = output();
  options.ask = () => assert.fail('cannot offer a choice based on stale state');
  const client = {snapshot: async () => { throw new Error('connection refused'); },
    end: () => assert.fail('do not mutate an unknown session')};
  assert.equal(await leave('robot', client, options), false);
  assert.match(lines.join('\n'), /could not check its status/);
  assert.match(lines.join('\n'), /openrua session --name robot status/);
  assert.doesNotMatch(lines.join('\n'), /still running|stays under control|Session robot ended/);
});

test('No is the default, Yes keeps execution, and cancellation never changes resources', async () => {
  for (const answer of ['', 'no', 'yes', 'cancel']) {
    let ends = 0;
    const client = {snapshot: async () => ({state: {closed: false}}), end: async () => { ends++; }};
    const {lines, options} = output(answer);
    assert.equal(await leave('robot', client, options), answer !== 'cancel');
    assert.equal(ends, answer === '' || answer === 'no' ? 1 : 0);
    assert.doesNotMatch(lines.join('\n'), /powers off|stays under control/);
    if (ends) {
      assert.ok(lines.findIndex(line => line.startsWith('Ending session'))
        < lines.findIndex(line => line.startsWith('Session robot ended')));
    }
  }
});

test('a lost end reply stays unconfirmed without an automatic retry or success claim', async () => {
  let ends = 0;
  const {lines, options} = output('no');
  const client = {snapshot: async () => ({state: {closed: false}}), end: async () => {
    ends++; throw new Error('reply lost');
  }};
  assert.equal(await leave('robot', client, options), false);
  assert.equal(ends, 1);
  assert.match(lines.join('\n'), /could not confirm that it stopped/);
  assert.match(lines.join('\n'), /openrua session --name robot status/);
  assert.doesNotMatch(lines.join('\n'), /Session robot ended/);
});
