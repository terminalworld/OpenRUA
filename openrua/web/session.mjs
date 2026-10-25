// What a shared session means to the person in front of it, for every
// client: the commands a person can type, what the session's state says,
// and the notices its events carry. Pure: no terminal, no page. The browser
// loads this file as served; the terminal ships a copy of it (its source
// tree links here), so both read one text and only draw it their own way.

// `clients` says where a command means something: the terminal's own
// screens (tool folding, history, leaving) have no page counterpart, and
// the page retries an unconfirmed send with its Send button.
export const COMMANDS = [
  {name: 'help', description: 'Keyboard shortcuts and commands', clients: ['terminal', 'browser']},
  {name: 'setup', description: 'Configure a new session and keep this conversation in history', clients: ['terminal']},
  {name: 'tools', description: 'Expand or collapse a tool result', clients: ['terminal']},
  {name: 'files', description: 'Browse saved workspace files and images', clients: ['terminal', 'browser']},
  {name: 'project', description: 'Project skills, reviews and accumulation settings', clients: ['terminal']},
  {name: 'robot', description: 'Robot connection status, logs, build and restart', clients: ['terminal', 'browser']},
  {name: 'watch', description: "See the robot's cameras live, in a browser", clients: ['terminal', 'browser']},
  {name: 'queue', description: 'Inspect, edit or withdraw queued messages', clients: ['terminal', 'browser']},
  {name: 'questions', description: 'Answer pending agent questions', clients: ['terminal', 'browser']},
  {name: 'resume', description: 'Resume in this project; --all shows every project', clients: ['terminal']},
  {name: 'end', description: 'End execution while retaining the workspace and history', clients: ['terminal', 'browser']},
  {name: 'interrupt', description: 'Stop the current turn; queued messages go on', clients: ['terminal', 'browser']},
  {name: 'continue', description: 'Confirm continuation of the paused queue', clients: ['terminal', 'browser']},
  {name: 'permissions', description: "Choose the current agent's native permission mode", clients: ['terminal', 'browser']},
  {name: 'model', description: 'Show the model, or change it: /model <name>', clients: ['terminal', 'browser']},
  {name: 'retry', description: 'Retry an unconfirmed send with the same request ID', clients: ['terminal']},
  {name: 'quit', description: 'Leave; you choose whether the session keeps running', clients: ['terminal']},
];

export const commandsFor = client => COMMANDS.filter(c => c.clients.includes(client));

export function permissionChoices(state) {
  return state?.setting_choices?.permissions ?? [];
}

// The agent's own commands (its /compact, its skills...), as it reported
// them; typed, they go to the agent as a message, as in its own terminal.
export function agentCommands(state) {
  const ours = new Set(COMMANDS.map(c => c.name));
  return (state?.agent_commands ?? [])
    .filter(c => !ours.has(c.name))
    .map(c => ({name: c.name, description: c.description || "the agent's own command"}));
}

// Split a typed line into a command: ours, the agent's, or none (a message).
export function parseCommand(text, state) {
  const trimmed = String(text).trim();
  if (!trimmed.startsWith('/')) return null;
  const [word, ...rest] = trimmed.split(/\s+/);
  const name = word.slice(1);
  if (COMMANDS.some(c => c.name === name)) return {kind: 'ours', name, args: rest};
  if (agentCommands(state).some(c => c.name === name)) return {kind: 'agent', name, args: rest};
  return {kind: 'unknown', name, args: rest};
}

// The session's state as one headline and its notes. A tone is a hint for
// colour: plain, muted, good, warn, bad, busy. `readOnly` for a retained
// conversation shown as history.
export function status(state, {readOnly = false} = {}) {
  if (!state) return {head: {kind: 'connecting', tone: 'muted', text: 'Connecting…'}, notes: []};
  const notes = [];
  if (state.robot) notes.push({tone: state.robot.status === 'ready' ? 'good' : 'warn', kind: 'robot',
    text: `robot ${state.robot.status} · /robot${state.robot.status === 'failed' ? ' logs' : ''}`});
  const maintenance = Object.values(state.maintenance ?? {}).at(-1);
  const connectionFailure = state.robot?.status === 'failed' && maintenance?.status === 'failed'
    && ['start', 'restart'].includes(maintenance.operation);
  if (maintenance && !connectionFailure) notes.push({tone: maintenance.status === 'failed' || maintenance.status === 'unknown' ? 'warn' : 'plain',
    text: `${maintenance.operation}: ${maintenance.status} · /robot`});
  const checks = Object.entries(state.checks ?? {});
  if (checks.length) notes.push({tone: 'plain', kind: 'checks', checks,
    text: checks.map(([step, check]) => `${step} ${{passed: '✓', failed: '✗', running: '…'}[check.status] ?? ''}`).join(' ')});
  if (state.settings?.permissions && (!state.setting_controls || state.setting_controls.includes('permissions'))) {
    const current = permissionChoices(state).find(choice => choice.value === state.settings.permissions);
    notes.push({tone: 'muted', text: `Permissions: ${current?.label ?? state.settings.permissions}`});
  }
  const held = state.messages.filter(m => m.status === 'queued' && m.held_reason).length;
  if (held) notes.push({tone: 'warn', text: `${held} held for review · /queue`});
  const queued = state.messages.filter(m => m.status === 'queued' && !m.held_reason).length;
  if (queued) notes.push({tone: 'plain', text: `${queued} queued`});
  if (!(state.connected || state.closed || readOnly)) notes.push({tone: 'warn',
    text: state.pause_reason?.startsWith('agent disconnected') ? state.pause_reason : 'agent disconnected'});
  const asking = Object.values(state.requests ?? {}).filter(r => r.status === 'pending').length;
  if (asking) notes.push({tone: 'warn', kind: 'questions', text: `${asking} question(s): type the answer, or /questions to choose`});
  let head;
  if (readOnly) head = {kind: 'readonly', tone: 'muted', text: 'Read-only history'};
  else if (state.closed) head = {kind: 'closed', tone: 'muted', text: 'Closed'};
  else if (state.paused) head = {kind: 'paused', tone: 'warn',
    text: `Paused${state.pause_reason ? ': ' + state.pause_reason : ''} · /queue to review` +
      (state.connected ? ', /continue to go on' : ' · agent unavailable; inspect session logs before restarting')};
  else if (asking) head = {kind: 'waiting', tone: 'warn', text: 'Waiting for your response'};
  else if (state.active) head = {kind: 'working', tone: 'busy', text: 'Working…'};
  else head = {kind: 'ready', tone: state.robot && state.robot.status !== 'ready' ? 'plain' : 'good',
    text: state.robot && state.robot.status !== 'ready' ? 'Ready to chat' : 'Ready'};
  return {head, notes};
}

// A notice an event carries for the person, or null.
export function notice(record) {
  const data = record?.data ?? {};
  if (record?.kind === 'session_reopened') return data.note ?? 'Conversation reopened; observe the current scene before continuing.';
  if (record?.kind === 'motion_canceled') {
    const {canceled = [], failed = []} = data;
    return [canceled.length ? `Motion canceled: ${canceled.join(', ')}` : '',
      failed.length ? `No answer from ${failed.join(', ')}: use the emergency stop if it still moves` : '']
      .filter(Boolean).join('\n') || null;
  }
  if (record?.kind === 'handed_over') return `Updating the execution environment: ${data.reason || 'internal handover'}. This conversation stays open.`;
  if (record?.kind === 'agent_event' && data.kind === 'setting_rejected') {
    const {setting, value, reason} = data.data ?? {};
    return `The agent did not take ${setting} ${value}${reason ? `: ${reason}` : ''}`;
  }
  if (record?.kind === 'agent_event' && data.kind === 'setting' && data.data?.effective_from === 'next_turn')
    return `${data.data.setting} ${data.data.value} applies from the next turn.`;
  if (record?.kind === 'agent_event' && data.kind === 'setting')
    return `${data.data.setting}: ${data.data.value}.`;
  return null;
}

// What a typed line does while the agent waits on a question: answers it
// when it can (one question, no secret, free text or one of its choices),
// or needs the question's own view ("choose"); null when none is pending.
export function typedAnswer(state, text) {
  const pending = Object.values(state?.requests ?? {}).filter(r => r.status === 'pending');
  if (!pending.length) return null;
  const request = pending[0];
  const [question] = request.questions;
  const answer = String(text).trim();
  // A choice typed in any case is that choice, as the agent wrote it.
  const chosen = question.choices?.find(c => c.toLowerCase() === answer.toLowerCase());
  if (request.questions.length !== 1 || question.secret
      || (question.choices?.length && !question.allow_other && chosen === undefined))
    return {kind: 'choose', request_id: request.request_id};
  return {kind: 'answer', request_id: request.request_id, answers: {[question.id]: [chosen ?? answer]}};
}

// A tool call's one-line summary of what it was given: the command, the
// file, the pattern, or its arguments.
export function toolSummary(details) {
  if (details === undefined || details === null) return '';
  if (typeof details === 'object') {
    const value = details.command ?? details.file_path ?? details.path ?? details.pattern ?? details.url;
    if (typeof value === 'string') return value.split('\n')[0];
    const keys = Object.keys(details);
    if (!keys.length) return '';
    return keys.map(key => `${key}: ${typeof details[key] === 'string' ? details[key].split('\n')[0] : JSON.stringify(details[key])}`).join(', ');
  }
  return String(details);
}
