#!/usr/bin/env node
import {parseArgs} from 'node:util';
import {Client} from './client.mjs';
import {Chat} from './app.mjs';

try {
  const {values} = parseArgs({options: {endpoint: {type: 'string'}, help: {type: 'boolean'}}});
  if (values.help) {
    console.log('Usage: npm start -- --endpoint /path/to/endpoint.json\n\nAttach the Pi terminal prototype to an existing OpenRUA shared session.\nCtrl+D or /quit detaches without ending the session.');
  } else {
    if (!values.endpoint) throw new Error('Provide --endpoint /path/to/endpoint.json. See npm start -- --help.');
    if (!process.stdin.isTTY || !process.stdout.isTTY) throw new Error('Run in an interactive terminal; use openrua session for scripts.');
    const chat = new Chat(await Client.fromFile(values.endpoint));
    const stop = () => chat.stop();
    process.once('SIGINT', stop); process.once('SIGTERM', stop);
    try { await chat.run(); } finally { chat.stop(); process.removeListener('SIGINT', stop); process.removeListener('SIGTERM', stop); }
    console.log('Detached. The session remains in its execution service.');
  }
} catch (error) {
  console.error(error.message);
  process.exitCode = 1;
}
