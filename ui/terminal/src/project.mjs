import {randomUUID} from 'node:crypto';
import {FilePreview} from './workspace.mjs';
import {plain, muted} from './view.mjs';
import {schedulesMenu} from './schedules.mjs';
import {displayStatus} from './session.mjs';

const policyNames = {auto: 'Automatically', suggest: 'Suggest methods first', manual: 'Only when I ask'};

const terminalStates = new Set(['completed', 'skipped', 'failed', 'busy', 'unknown', 'interrupted',
  'timed_out', 'quota_limited', 'authentication_required', 'turn_limit', 'invalid_output']);

function textFile(path, text) {
  return {path, text, kind: 'text', size: Buffer.byteLength(text)};
}

function preview(chat, file, back) {
  if (chat.done) return;
  const close = () => { handle.hide(); void back().catch(error => chat.say(error.message)); };
  const panel = new FilePreview(file, close, close, () => chat.ui.requestRender(), () => chat.rows());
  const handle = chat.dialog(panel);
}

export async function updateReview(chat, force = false) {
  const task = chat.reviewTask;
  if (!task || task.checking || (!force && (terminalStates.has(task.status) || Date.now() - (task.checked ?? 0) < 1500))) return;
  task.checked = Date.now();
  task.checking = true;
  try {
    const value = await chat.client.projectSkills('task', {id: task.id});
    if (chat.done || chat.reviewTask?.id !== task.id) return;
    chat.reviewTask = {...value, checked: Date.now()};
    chat.projectStatus.setText(muted(`Skill review: ${plain(displayStatus(value.status))}. Open /project for details.`
      + (value.reason || value.error ? ` · ${plain(value.reason || value.error)}` : '')));
    chat.ui.requestRender();
  } catch (error) {
    if (!chat.done) chat.projectStatus.setText(muted(`Could not check the review. Open /project for details. · ${plain(error.message)}`));
  } finally { task.checking = false; }
}

async function startReview(chat, options) {
  const title = `Review completed project work?\n\nAgent: ${options.agent}\nModel: ${options.model || '(agent default)'}\nEffort: ${options.effort || '(agent default)'}\nContainer image: ${options.image}\nReview time limit: ${options.wall_clock_min} minutes\n\n`
    + 'This starts a separate agent task using your account or API key. It reviews finished work and suggests skills. Work in progress is excluded. It does not start a robot.';
  chat.confirm(title, async () => {
    if (chat.pendingReview && JSON.stringify(chat.pendingReview.options) !== JSON.stringify(options))
      throw new Error('We have not confirmed that the last review started. Check it in /project before starting another.');
    chat.pendingReview ??= {request_id: randomUUID().replaceAll('-', ''), options: {...options}};
    const task = await chat.client.projectSkills('start', chat.pendingReview);
    chat.pendingReview = null;
    chat.reviewTask = task;
    await updateReview(chat, true);
    chat.say('Review requested. The task runs separately from this conversation. You can keep working or close this terminal. Open /project to check its status and results.');
  });
}

export function reviewOptions(chat, options, identity = null, flow = {}) {
  const rows = [
    {value: 'agent', label: `Agent: ${plain(options.agent)}`},
    {value: 'model', label: `Model: ${plain(options.model || '(agent default)')}`},
    {value: 'effort', label: `Effort: ${plain(options.effort || '(agent default)')}`},
    {value: 'image', label: `Container image: ${plain(options.image)}`},
    {value: 'wall_clock_min', label: `Review time limit: ${options.wall_clock_min} minutes`},
    {value: 'start', label: flow.label || 'Review completed work…'},
    {value: 'back', label: 'Back'},
  ];
  chat.picker('Review settings', rows, async key => {
    if (key === 'back') return (flow.back || projectMenu)(chat);
    if (key === 'start') return flow.finish ? flow.finish(options) : startReview(chat, options);
    chat.editText(key === 'wall_clock_min' ? 'Review time limit in minutes' : key,
      String(options[key]), async value => {
        if (key === 'wall_clock_min' && (!Number.isFinite(Number(value)) || Number(value) <= 0))
          throw new Error('Enter a time limit greater than zero, in minutes.');
        options[key] = key === 'wall_clock_min' ? Number(value) : value.trim();
        if (key === 'agent') {
          const resolved = await chat.client.projectSkills('defaults', {agent: options.agent});
          if (identity !== resolved.agent_identity) {
            options.model = resolved.defaults.model;
            options.effort = resolved.defaults.effort;
          }
          identity = resolved.agent_identity;
        }
        reviewOptions(chat, options, identity, flow);
      }, {allowEmpty: ['model', 'effort'].includes(key)});
  });
}

async function candidateMenu(chat, id, data, candidate) {
  if (chat.done) return;
  const rows = [{value: 'read', label: 'Read suggested skill'}, {value: 'diff', label: 'See changes and supporting evidence'}];
  if (candidate.can_import) rows.push({value: 'import', label: 'Import new skill into this project…'});
  rows.push({value: 'merge', label: 'Ask the agent to combine these skills…'}, {value: 'back', label: 'Back to review'});
  chat.picker(`${plain(candidate.name)}${candidate.existing ? ' · existing skill' : ' · proposed new skill'}`
    + (candidate.source_changed ? '\nProject files have changed since this review started.' : '')
    + (candidate.error ? `\n${plain(candidate.error)}` : ''), rows, async key => {
    const back = () => candidateMenu(chat, id, data, candidate);
    if (key === 'back') return showReview(chat, id);
    if (key === 'read') return preview(chat, await chat.client.projectSkills('candidate', {id, key: candidate.key}), back);
    if (key === 'diff') return preview(chat, textFile('Review changes and evidence', data.diff + '\n\n' + data.summary), back);
    if (key === 'import') return chat.confirm(`Import ${candidate.name} into this project?\n\nYour existing skills will not be overwritten. Saving these instructions does not test them on the robot.`, async () => {
      const result = await chat.client.projectSkills('import', {id, key: candidate.key, fingerprint: candidate.fingerprint});
      chat.say(`Imported ${plain(result.name)} at ${plain(result.imported)}.`
        + (result.issues?.length ? ` ${result.issues.map(plain).join('; ')}` : ' Your agent can use this skill through its own skills feature.'));
    });
    const {draft} = await chat.client.projectSkills('merge', {id, key: candidate.key});
    chat.confirm('Add this request to your draft?\n\nIt will not be sent yet. Your existing text stays above it.', async () => {
      const current = chat.editor.getText().trim();
      chat.projectDraft = (current ? current + '\n\n' : '') + draft;
      chat.editor.setText(chat.projectDraft);
      chat.say('The request is in your draft. Review it, then press Enter to send it.');
    });
  });
}

export async function showReview(chat, id) {
  const data = await chat.client.projectSkills('report', {id});
  if (chat.done) return;
  const rows = [{value: 'summary', label: 'Read review report and supporting evidence'}, {value: 'diff', label: 'See all changes'},
    ...data.candidates.map(row => ({value: row.key, label: plain(row.name),
      description: `${row.existing ? 'existing skill' : 'new suggestion'}${row.source_changed ? ' · project files have changed' : ''}`})),
    {value: 'refresh', label: 'Refresh report'}, {value: 'back', label: 'Back'}];
  chat.picker(`Skill review · ${plain(displayStatus(data.report.status))} · ${id.slice(0, 8)}`, rows, async key => {
    if (key === 'back') return projectMenu(chat);
    if (key === 'refresh') return showReview(chat, id);
    if (key === 'summary' || key === 'diff') return preview(chat,
      textFile(key === 'summary' ? 'Review report' : 'Review diff', data[key === 'summary' ? 'summary' : 'diff'] || 'No output yet.'),
      () => showReview(chat, id));
    return candidateMenu(chat, id, data, data.candidates.find(row => row.key === key));
  });
}

export async function projectMenu(chat) {
  if (chat.client.readOnly || !chat.client.projectSkills)
    throw new Error('Open a running conversation for this project to use this menu. You can also run openrua skills in the project directory.');
  const data = await chat.client.projectSkills('overview');
  if (chat.done) return;
  const active = data.tasks.find(task => ['running', 'starting', 'cancel_requested'].includes(task.status));
  if (active) { chat.reviewTask = active; await updateReview(chat, true); }
  const rows = [{value: 'skills', label: `Saved skills (${data.entries.length})`},
    {value: 'reviews', label: `Review reports (${data.reports.length})`},
    {value: 'tasks', label: `Review tasks (${data.tasks.length})`},
    {value: 'policy', label: `Save useful methods: ${policyNames[data.policy] || data.policy}`},
    ...(data.defaults ? [{value: 'start', label: 'Review completed work…'}] : []),
    ...(chat.pendingReview ? [{value: 'retry', label: 'Check the last review request…'}] : []),
    {value: 'schedules', label: 'Recurring review schedules…'},
    {value: 'refresh', label: 'Refresh'}];
  chat.picker(`Project skills and reviews\n${plain(data.project)}`
    + (data.review_error ? `\nReview unavailable: ${plain(data.review_error)}` : ''), rows, async key => {
    if (key === 'refresh') return projectMenu(chat);
    if (key === 'schedules') return schedulesMenu(chat, data, reviewOptions, showReview);
    if (key === 'start') return reviewOptions(chat, {...data.defaults}, data.agent_identity);
    if (key === 'retry') return startReview(chat, chat.pendingReview.options);
    if (key === 'policy') return chat.picker('How should the agent save useful methods? These instructions guide future work. Existing skills stay available.',
      ['auto', 'suggest', 'manual'].map(mode => ({value: mode, label: policyNames[mode]})), mode =>
        chat.confirm(`Use "${policyNames[mode]}" for saving useful methods?`, async () => {
          await chat.client.projectSkills('policy', {mode}); chat.say('Saved. This preference guides the agent in future work.');
        }));
    if (key === 'skills') return chat.picker('Saved skills\nNames and descriptions come from the saved files. They do not show whether a skill has been tested on this robot.',
      data.entries.map(entry => ({value: entry.key, label: `${plain(entry.name)} [${entry.scope}]`, description: plain(entry.error || entry.description)})),
      async key => preview(chat, await chat.client.projectSkills('skill', {key}), () => projectMenu(chat)));
    if (key === 'reviews') return chat.picker('Review reports', data.reports.map(report => ({value: report.id,
      label: `${report.id.slice(0, 8)} · ${plain(displayStatus(report.status))}`, description: `${report.proposed_skills ?? 0} suggested skills · ${plain(report.started_at)}`})),
      id => showReview(chat, id));
    if (key === 'tasks') return chat.picker('Background reviews\nYou can close this terminal while a review is running.',
      data.tasks.map(task => ({value: task.id, label: `${task.id.slice(0, 8)} · ${plain(displayStatus(task.status))}`, description: plain(task.reason || task.error || task.started_at)})),
      async id => {
        const task = await chat.client.projectSkills('task', {id});
        if (task.review_id) return showReview(chat, task.review_id);
        chat.reviewTask = task; await updateReview(chat, true);
        chat.picker(`Skill review: ${plain(displayStatus(task.status))}` + (task.reason || task.error ? `\n${plain(task.reason || task.error)}` : ''),
          [{value: 'refresh', label: 'Refresh task'},
            ...(['running', 'starting', 'cancel_requested'].includes(task.status) ? [{value: 'cancel', label: 'Cancel this review…'}] : []),
            {value: 'back', label: 'Back'}], async action => {
            if (action !== 'cancel') return projectMenu(chat);
            chat.confirm('Cancel this skill review?\n\nYour conversation stays open. Your saved skills are kept.', async () => {
              chat.reviewTask = await chat.client.projectSkills('cancel', {id});
              await updateReview(chat, true);
            });
          });
      });
  });
}
