import {randomUUID} from 'node:crypto';
import {FilePreview} from './workspace.mjs';
import {plain} from './view.mjs';
import {ResponseError} from './client.mjs';
import {robotPrompt, displayStatus} from './session.mjs';

const operations = new Set(['start', 'stop', 'restart', 'build']);

function show(chat, text) {
  const close = () => handle.hide();
  const panel = new FilePreview({path: 'Robot connection', text, kind: 'text', size: Buffer.byteLength(text)},
    close, close, () => chat.ui.requestRender(), () => chat.rows());
  const handle = chat.dialog(panel);
}

async function send(chat) {
  const body = chat.pendingRobot;
  if (!body) throw new Error('There is no robot request waiting to be retried.');
  try {
    const {result} = await chat.client.request('/api/actions/robot', body);
    chat.pendingRobot = null;
    const action = {start: 'Robot startup', stop: 'Robot shutdown', restart: 'Robot restart', build: 'Software build'}[result.operation];
    chat.say(`${action}: ${displayStatus(result.status).toLowerCase()}.`
      + (result.summary ? `\n${plain(result.summary)}` : '')
      + '\nOpen /robot to check progress. You can keep chatting.');
    await chat.refresh();
  } catch (error) {
    if (error instanceof ResponseError && error.status >= 400 && error.status < 500) chat.pendingRobot = null;
    throw error;
  }
}

export async function robotMenu(chat, [operation] = []) {
  if (chat.client.readOnly) throw new Error('You can view this conversation, but you cannot control the robot here.');
  if (operation === 'retry') return send(chat);
  if (operation && !operations.has(operation) && !['status', 'logs'].includes(operation))
    throw new Error('Use /robot status, logs, start, stop, restart, build or retry.');
  if (operations.has(operation)) {
    if (chat.pendingRobot) throw new Error('We have not confirmed the last robot request. Check /robot or use /robot retry.');
    chat.confirm(robotPrompt(operation), async () => {
      chat.pendingRobot = {operation, request_id: randomUUID()};
      await send(chat);
    });
    return;
  }
  const {result} = await chat.client.request('/api/actions/robot', {operation: operation || 'status'});
  if (operation) { show(chat, JSON.stringify(result, null, 2)); return; }
  const robot = result.robot;
  chat.picker(`Robot ${plain(robot?.status || 'not connected')}${robot?.reason ? '\n' + plain(robot.reason) : ''}`, [
    {value: 'status', label: 'Connection status and request results'},
    {value: 'logs', label: 'Read driver and build logs'},
    {value: 'start', label: 'Connect robot…'},
    {value: 'restart', label: 'Apply settings and reconnect…'},
    {value: 'stop', label: 'Stop the simulator or drivers…'},
    {value: 'build', label: 'Prepare robot software…', description: 'Running drivers keep running'},
    ...(chat.pendingRobot ? [{value: 'retry', label: 'Retry the last robot request', description: `Request ID: ${plain(chat.pendingRobot.request_id)}`}] : []),
    {value: 'back', label: 'Back to conversation'},
  ], async key => { if (key !== 'back') await robotMenu(chat, [key]); });
}
