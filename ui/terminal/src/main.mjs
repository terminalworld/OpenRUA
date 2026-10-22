#!/usr/bin/env node
import {parseArgs} from 'node:util';
import {readFile, writeFile} from 'node:fs/promises';
import {createInterface} from 'node:readline/promises';
import {Client} from './client.mjs';
import {Chat} from './app.mjs';
import {ArchiveClient, History, Setup, Workspace} from './screens.mjs';

// Pi tells terminals that draw pictures apart by variables an SSH connection
// drops; these two travel: iTerm2 sends LC_TERMINAL, kitty sets TERM. Under
// tmux or screen pictures stay off, as Pi has them.
if (!process.env.PI_IMAGE_PROTOCOL && !process.env.TMUX && !/^(screen|tmux)/.test(process.env.TERM ?? '')) {
  if (process.env.LC_TERMINAL === 'iTerm2') process.env.PI_IMAGE_PROTOCOL = 'iterm2';
  else if (process.env.TERM === 'xterm-kitty') process.env.PI_IMAGE_PROTOCOL = 'kitty';
}

// Leaving a conversation that holds a robot: keep it running (Enter), or end
// it now. Anything but "e" keeps it, Ctrl-C and a closed input included.
async function leave(name, client, facts) {
  const holding = facts.backend === 'real' ? 'the robot stays under control' : 'robot and agent included';
  console.log(`Session ${name} is still running, ${holding}.`);
  const input = createInterface({input: process.stdin, output: process.stdout});
  input.on('SIGINT', () => input.close());
  let answer = '';
  try {
    answer = await input.question('  Enter  keep it running in the background\n'
      + '  e      end it now: the robot powers off, the files stay\n> ');
  } catch { answer = ''; } finally { input.close(); }
  if (answer.trim().toLowerCase().startsWith('e')) {
    await client.end();
    console.log(`Session ${name} ended; its files are kept (openrua --resume shows it).`);
  } else {
    console.log(`Still running. Come back: openrua --resume · end it: openrua session --name ${name} end`);
  }
}

try {
  const {values} = parseArgs({options: {endpoint: {type: 'string'}, input: {type: 'string'},
    output: {type: 'string'}, help: {type: 'boolean'}}});
  if (values.help) {
    console.log('Usage: openrua\nDevelopment: node src/main.mjs --endpoint /path/to/endpoint.json\n\nCtrl+D or /quit leaves; you choose whether the session keeps running.');
  } else {
    if (!process.stdin.isTTY || !process.stdout.isTTY) throw new Error('Run in an interactive terminal; use openrua session for scripts.');
    if (Boolean(values.input) !== Boolean(values.output) || Boolean(values.endpoint) === Boolean(values.input)) {
      throw new Error('Provide --endpoint, or both --input and --output. See --help.');
    }
    const spec = values.input ? JSON.parse(await readFile(values.input, 'utf8')) : {mode: 'chat'};
    let screen;
    if (spec.mode === 'setup') screen = new Setup(spec);
    else if (spec.mode === 'workspace') screen = new Workspace(spec);
    else if (spec.mode === 'history') screen = new History(spec);
    else if (spec.mode === 'chat') {
      const client = values.endpoint ? await Client.fromFile(values.endpoint)
        : spec.connection.read_only ? new ArchiveClient(spec.connection)
        : new Client(spec.connection.url, spec.connection.token);
      screen = new Chat(client, {history: Boolean(values.input), name: spec.name, facts: spec.facts ?? {}});
    } else throw new Error('Unknown terminal screen.');
    const stop = () => screen.stop();
    process.once('SIGINT', stop); process.once('SIGTERM', stop);
    let result;
    try { result = await screen.run(); }
    finally { screen.stop(); process.removeListener('SIGINT', stop); process.removeListener('SIGTERM', stop); }
    if (values.output) await writeFile(values.output, JSON.stringify(result ?? {action: 'quit'}), {mode: 0o600, flag: 'wx'});
    // Leaving the terminal leaves the session: on a real robot the arm stays
    // under control, so ask whether to keep it running.
    const left = screen instanceof Chat && !screen.client.readOnly && screen.controller?.state
      && !screen.controller.state.closed && !result?.action?.startsWith?.('history');
    if (left && spec.name) await leave(spec.name, screen.client, spec.facts ?? {});
  }
} catch (error) {
  console.error(error.message);
  process.exitCode = 1;
}
