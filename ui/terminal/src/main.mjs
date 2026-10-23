#!/usr/bin/env node
import {parseArgs} from 'node:util';
import {readFile, writeFile} from 'node:fs/promises';
import {Client} from './client.mjs';
import {Chat} from './app.mjs';
import {ArchiveClient, History, Setup, Workspace} from './screens.mjs';
import {leave} from './exit.mjs';

// Pi tells terminals that draw pictures apart by variables an SSH connection
// drops; these two travel: iTerm2 sends LC_TERMINAL, kitty sets TERM. Under
// tmux or screen pictures stay off, as Pi has them.
if (!process.env.PI_IMAGE_PROTOCOL && !process.env.TMUX && !/^(screen|tmux)/.test(process.env.TERM ?? '')) {
  if (process.env.LC_TERMINAL === 'iTerm2') process.env.PI_IMAGE_PROTOCOL = 'iterm2';
  else if (process.env.TERM === 'xterm-kitty') process.env.PI_IMAGE_PROTOCOL = 'kitty';
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
    const left = screen instanceof Chat && !screen.client.readOnly
      && !result?.action?.startsWith?.('history');
    if (left && spec.name) {
      const ended = screen.endConfirmed || Boolean(screen.controller?.state?.closed);
      if (!await leave(spec.name, screen.client, {ended})) process.exitCode = 1;
    }
  }
} catch (error) {
  console.error(error.message);
  process.exitCode = 1;
}
