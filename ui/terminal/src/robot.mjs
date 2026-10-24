import {randomUUID} from 'node:crypto';
import {FilePreview} from './workspace.mjs';
import {plain} from './view.mjs';
import {ResponseError} from './client.mjs';

const operations = new Set(['start', 'stop', 'restart', 'build']);

function show(chat, text) {
  const close = () => handle.hide();
  const panel = new FilePreview({path: 'Robot connection', text, kind: 'text', size: Buffer.byteLength(text)},
    close, close, () => chat.ui.requestRender(), () => chat.rows());
  const handle = chat.dialog(panel);
}

async function send(chat) {
  const body = chat.pendingRobot;
  if (!body) throw new Error('No unconfirmed connection request.');
  try {
    const {result} = await chat.client.request('/api/actions/robot', body);
    chat.pendingRobot = null;
    chat.say(`${result.operation}: ${result.status}. ${result.summary}\n/robot shows progress. You can keep chatting.`);
    await chat.refresh();
  } catch (error) {
    if (error instanceof ResponseError && error.status >= 400 && error.status < 500) chat.pendingRobot = null;
    throw error;
  }
}

export async function robotMenu(chat, [operation] = []) {
  if (chat.client.readOnly) throw new Error('This is read-only history.');
  if (operation === 'retry') return send(chat);
  if (operation && !operations.has(operation) && !['status', 'logs'].includes(operation))
    throw new Error('Use /robot status, logs, start, stop, restart, build or retry.');
  if (operations.has(operation)) {
    if (chat.pendingRobot) throw new Error('A connection request is unconfirmed. Inspect /robot or use /robot retry.');
    chat.confirm(`${operation} the configured robot resources?\n\nThe conversation stays open. Held instructions remain held. A stop follows the driver contract and is not a physical emergency stop.`, async () => {
      chat.pendingRobot = {operation, request_id: randomUUID()};
      await send(chat);
    });
    return;
  }
  const {result} = await chat.client.request('/api/actions/robot', {operation: operation || 'status'});
  if (operation) { show(chat, JSON.stringify(result, null, 2)); return; }
  const robot = result.robot;
  chat.picker(`Robot ${plain(robot?.status || 'not connected')}${robot?.reason ? '\n' + plain(robot.reason) : ''}`, [
    {value: 'status', label: 'Connection status and operation results'},
    {value: 'logs', label: 'Read driver and build logs'},
    {value: 'start', label: 'Connect robot…'},
    {value: 'restart', label: 'Apply configuration and restart connection…'},
    {value: 'stop', label: 'Stop owned robot resources…'},
    {value: 'build', label: 'Build declared robot images…', description: 'Does not restart a running driver'},
    ...(chat.pendingRobot ? [{value: 'retry', label: 'Retry unconfirmed request', description: chat.pendingRobot.request_id}] : []),
    {value: 'back', label: 'Back to conversation'},
  ], async key => { if (key !== 'back') await robotMenu(chat, [key]); });
}
