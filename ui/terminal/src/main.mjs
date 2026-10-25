#!/usr/bin/env node
import {parseArgs} from 'node:util';
import {Text, SelectList} from '@earendil-works/pi-tui';
import {selectTheme} from './view.mjs';
import {readFile, writeFile} from 'node:fs/promises';
import {Client} from './client.mjs';
import {Chat} from './app.mjs';
import {Update} from './update.mjs';
import {ArchiveClient, History, Setup, Workspace} from './screens.mjs';
import {leave} from './exit.mjs';
import {serve, Leave, Progress} from './application.mjs';
import {TerminalHost} from './surface.mjs';

// Pi tells terminals that draw pictures apart by variables an SSH connection
// drops; these two travel: iTerm2 sends LC_TERMINAL, kitty sets TERM. Under
// tmux or screen pictures stay off, as Pi has them.
if (!process.env.PI_IMAGE_PROTOCOL && !process.env.TMUX && !/^(screen|tmux)/.test(process.env.TERM ?? '')) {
  if (process.env.LC_TERMINAL === 'iTerm2') process.env.PI_IMAGE_PROTOCOL = 'iterm2';
  else if (process.env.TERM === 'xterm-kitty') process.env.PI_IMAGE_PROTOCOL = 'kitty';
}

async function createScreen(spec, host) {
  if (spec.mode === 'update') return new Update(spec, undefined, host);
  if (spec.mode === 'setup') return new Setup(spec, undefined, host);
  if (spec.mode === 'workspace') return new Workspace(spec, undefined, host);
  if (spec.mode === 'history') return new History(spec, undefined, host);
  if (spec.mode === 'chat') {
    const client = spec.endpoint ? await Client.fromFile(spec.endpoint)
      : spec.connection.read_only ? new ArchiveClient(spec.connection)
      : new Client(spec.connection.url, spec.connection.token);
    return new Chat(client, {history: !spec.endpoint, name: spec.name, facts: spec.facts || {}, host});
  }
  throw new Error('Unknown terminal screen.');
}

async function finishScreen(screen, spec, result, host, showProgress) {
  if (!(screen instanceof Chat) || screen.client.readOnly || result?.action?.startsWith('history') || !spec.name) return;
  const ended = screen.endConfirmed || Boolean(screen.controller?.state?.closed);
  let waiting = false, stage;
  const messages = [];
  const ok = await leave(spec.name, screen.client, {ended,
    ask: async prompt => {
      const view = new Leave(prompt, host);
      const answer = await view.run();
      waiting = true;
      return answer.value || '';
    }, write: text => {
      messages.push(text);
      if (waiting || ended) stage = showProgress({title: 'Session status', line: text});
    }});
  if (!ok) {
    stage?.stop();
    const view = new Leave('', host);
    view.ui.setHeading('OpenRUA · Status unconfirmed');
    view.ui.setFooter(new Text('Enter Close · Esc Close', 0, 0));
    view.ui.children.splice(0, view.ui.children.length);
    view.ui.addChild(new Text(messages.join('\n'), 0, 1));
    const close = new SelectList([{value: '', label: 'Close interface'}], 1, selectTheme);
    close.onSelect = close.onCancel = () => view.finish();
    view.ui.addChild(close); view.ui.setFocus(close);
    await view.run();
    process.exitCode = 1;
  }
  return ok ? messages.slice(-1) : messages.slice(-2);
}

try {
  const {values} = parseArgs({options: {endpoint: {type: 'string'}, input: {type: 'string'},
    output: {type: 'string'}, help: {type: 'boolean'}, 'control-fd': {type: 'string'}, 'parent-pid': {type: 'string'}}});
  if (values.help) {
    console.log('Usage: openrua\nDevelopment: node src/main.mjs --endpoint /path/to/endpoint.json\n\nCtrl+D or /quit leaves; choose whether to keep the session running.');
  } else {
    if (!process.stdin.isTTY || !process.stdout.isTTY) throw new Error('Run in an interactive terminal; use openrua session for scripts.');
    if (values['control-fd']) {
      await serve(Number(values['control-fd']), Number(values['parent-pid']), createScreen, finishScreen);
    } else {
      if (Boolean(values.input) !== Boolean(values.output) || Boolean(values.endpoint) === Boolean(values.input))
        throw new Error('Provide --endpoint, or both --input and --output. See --help.');
      const spec = values.input ? JSON.parse(await readFile(values.input, 'utf8')) : {mode: 'chat', endpoint: values.endpoint};
      const host = new TerminalHost();
      const screen = await createScreen(spec, host);
      const stop = () => host.close();
      process.once('SIGINT', stop); process.once('SIGTERM', stop);
      let result, progress, summary;
      try {
        result = await screen.run();
        summary = await finishScreen(screen, spec, result, host, message => {
          if (!progress) { progress = new Progress(host); progress.run().catch(() => {}); }
          progress.update(message); return progress;
        });
      } finally {
        screen.stop(); progress?.stop(); host.stop();
        process.removeListener('SIGINT', stop); process.removeListener('SIGTERM', stop);
      }
      for (const line of summary || []) console.log(line);
      if (values.output) await writeFile(values.output, JSON.stringify(result || {action: 'quit'}), {mode: 0o600, flag: 'wx'});
    }
  }
} catch (error) {
  console.error(error.message);
  process.exitCode = 1;
}
