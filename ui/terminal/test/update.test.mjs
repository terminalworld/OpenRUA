import assert from 'node:assert/strict';
import {test} from 'node:test';
import {Update} from '../src/update.mjs';
import {Leave, Progress} from '../src/application.mjs';
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

for (const [keys, value] of [['\r', 'no'], ['\x1b[B\r', 'yes'], ['\x1b[B\x1b[B\r', 'cancel'], ['\x1b', 'cancel']]) {
  test(`resource choice ${JSON.stringify(keys)} selects ${value}`, async () => {
    const terminal = new Terminal(), screen = new Leave('', null, terminal);
    const result = screen.run();
    await new Promise(resolve => setTimeout(resolve, 30));
    assert.match(terminal.output, /Keep this session running/);
    if (keys === '\x1b[B\r') { terminal.input('\x1b[B'); terminal.input('\r'); }
    else if (keys === '\x1b[B\x1b[B\r') { terminal.input('\x1b[B'); terminal.input('\x1b[B'); terminal.input('\r'); }
    else terminal.input(keys);
    assert.equal((await result).value, value);
  });
}

for (const [key, value] of [['\r', 'no'], ['\x1b', 'cancel']]) {
  test(`leave choice survives exit shortcut events until ${value}`, async t => {
    const terminal = new Terminal(), screen = new Leave('', null, terminal);
    t.after(() => screen.stop());
    const result = screen.run();
    for (const data of ['\x03', '\x04', '\x1b[99;5:1u', '\x1b[99;5:2u',
      '\x1b[99;5:3u', '\x1b[100;5:2u', '\x1b[100;5:3u']) {
      terminal.input(data);
      assert.equal(screen.done, false);
    }
    terminal.input(key);
    assert.equal((await result).value, value);
  });
}

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

test('startup phases replace the summary while elapsed time stays explicitly total', async () => {
  const terminal = new Terminal(), host = new TerminalHost(terminal);
  const screen = new Progress(host);
  const result = screen.run();
  screen.update({title: 'Starting session', compact: true, summary: 'Starting session'});
  screen.started = Date.now() - 33000;
  const started = screen.started;
  screen.update({summary: 'Starting the agent container', line: '[startup] Starting the agent container'});
  screen.update({line: '[startup timing] Preparing project files: completed in 0.250s'});
  assert.match(screen.output.text, /Starting the agent container/);
  assert.match(screen.output.text, /\d+s total/);
  assert.equal(screen.started, started);
  assert.doesNotMatch(screen.output.text, /Preparing project files/);
  terminal.input('l'); assert.match(screen.output.text, /Preparing project files: completed/);
  screen.update({summary: 'Connecting to the agent'});
  assert.match(screen.output.text, /Connecting to the agent/);
  assert.match(screen.output.text, /\d+s total/);
  assert.equal(screen.started, started);
  screen.stop(); host.stop(); await result;
});
