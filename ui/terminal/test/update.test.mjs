import assert from 'node:assert/strict';
import {test} from 'node:test';
import {Update} from '../src/update.mjs';
import {Progress} from '../src/application.mjs';
import {TerminalHost} from '../src/surface.mjs';

class Terminal {
  columns = 100; rows = 30; output = '';
  start(input) { this.input = input; }
  stop() {} write(data) { this.output += data; }
  async drainInput() {}
  moveBy() {} hideCursor() {} showCursor() {} clearLine() {}
  clearFromCursor() {} clearScreen() {} setTitle() {} setProgress() {}
}
const spec = {mode: 'update', name: 'Example Agent', installed: '1.0', target: '2.0', required: false, phase: 'offer'};

for (const [key, action] of [['\r', 'continue'], ['\x1b', 'continue'], ['1', 'update'], ['3', 'skip'], ['\x04', 'quit']]) {
  test(`optional update ${JSON.stringify(key)} selects ${action}`, async () => {
    const terminal = new Terminal(), screen = new Update(spec, terminal);
    const result = screen.run(); terminal.input(key);
    assert.equal((await result).action, action);
  });
}

test('required updates cannot silently continue and Escape quits', async () => {
  const terminal = new Terminal(), screen = new Update({...spec, required: true}, terminal);
  const result = screen.run(); terminal.input('\x1b');
  assert.equal((await result).action, 'quit');
});

test('a failed required update offers explicit retry, log and quit', async () => {
  const terminal = new Terminal(), screen = new Update({...spec, required: true, phase: 'result', success: false}, terminal);
  const result = screen.run();
  const text = screen.ui.children.map(child => child.render(100).join('\n')).join('\n');
  assert.match(text, /Retry update/); assert.match(text, /View update log/);
  assert.doesNotMatch(text, /Continue with current/);
  terminal.input('1'); assert.equal((await result).action, 'retry');
});

test('compact progress only reveals raw build output when logs are requested', async () => {
  const terminal = new Terminal(), host = new TerminalHost(terminal);
  const screen = new Progress(host);
  const result = screen.run();
  screen.update({title: 'Updating Example Agent', detail: '1.0 → 2.0', compact: true, summary: 'Installing selected version'});
  screen.update({line: 'apt and Docker details stay in the log'});
  assert.match(screen.output.text, /Installing selected version/);
  assert.doesNotMatch(screen.output.text, /apt and Docker/);
  terminal.input('l'); assert.match(screen.output.text, /apt and Docker/);
  terminal.input('l'); assert.doesNotMatch(screen.output.text, /apt and Docker/);
  screen.stop(); host.stop(); await result;
});
