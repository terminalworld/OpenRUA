import {randomUUID} from 'node:crypto';
import {FilePreview} from './workspace.mjs';
import {plain, muted} from './view.mjs';

const terminalStates = new Set(['completed', 'skipped', 'failed', 'busy', 'unknown', 'interrupted',
  'timed_out', 'quota_limited', 'turn_limit', 'invalid_output']);

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
    chat.projectStatus.setText(muted(`Skill review: ${plain(value.status)} · /project to inspect`
      + (value.reason || value.error ? ` · ${plain(value.reason || value.error)}` : '')));
    chat.ui.requestRender();
  } catch (error) {
    if (!chat.done) chat.projectStatus.setText(muted(`Skill review status unavailable · /project to inspect · ${plain(error.message)}`));
  } finally { task.checking = false; }
}

async function startReview(chat, options) {
  const title = `Review completed project work?\n\nAgent: ${options.agent}\nModel: ${options.model || '(agent default)'}\nEffort: ${options.effort || '(agent default)'}\nImage: ${options.image}\nWorker limit: ${options.wall_clock_min} minutes\n\n`
    + 'This is a separate model task using your configured login or API. Active work is excluded; results are proposals. No robot is started.';
  chat.confirm(title, async () => {
    if (chat.pendingReview && JSON.stringify(chat.pendingReview.options) !== JSON.stringify(options))
      throw new Error('Inspect or retry the previous unconfirmed review start first.');
    chat.pendingReview ??= {request_id: randomUUID().replaceAll('-', ''), options: {...options}};
    const task = await chat.client.projectSkills('start', chat.pendingReview);
    chat.pendingReview = null;
    chat.reviewTask = task;
    await updateReview(chat, true);
    chat.say('Review started separately. You can keep working or leave this terminal; /project shows its result.');
  });
}

function reviewOptions(chat, options, identity = null) {
  const rows = [
    {value: 'agent', label: `Agent: ${plain(options.agent)}`},
    {value: 'model', label: `Model: ${plain(options.model || '(agent default)')}`},
    {value: 'effort', label: `Effort: ${plain(options.effort || '(agent default)')}`},
    {value: 'image', label: `Image: ${plain(options.image)}`},
    {value: 'wall_clock_min', label: `Worker time limit: ${options.wall_clock_min} minutes`},
    {value: 'start', label: 'Review completed work…'},
    {value: 'back', label: 'Back'},
  ];
  chat.picker('Review settings', rows, async key => {
    if (key === 'back') return projectMenu(chat);
    if (key === 'start') return startReview(chat, options);
    chat.editText(key === 'wall_clock_min' ? 'Worker time limit in minutes' : key,
      String(options[key]), async value => {
        if (key === 'wall_clock_min' && (!Number.isFinite(Number(value)) || Number(value) <= 0))
          throw new Error('Use a finite positive number of minutes.');
        options[key] = key === 'wall_clock_min' ? Number(value) : value.trim();
        if (key === 'agent') {
          const resolved = await chat.client.projectSkills('defaults', {agent: options.agent});
          if (identity !== resolved.agent_identity) {
            options.model = resolved.defaults.model;
            options.effort = resolved.defaults.effort;
          }
          identity = resolved.agent_identity;
        }
        reviewOptions(chat, options, identity);
      }, {allowEmpty: ['model', 'effort'].includes(key)});
  });
}

async function candidateMenu(chat, id, data, candidate) {
  if (chat.done) return;
  const rows = [{value: 'read', label: 'Read proposed skill'}, {value: 'diff', label: 'Read changes and evidence'}];
  if (candidate.can_import) rows.push({value: 'import', label: 'Import new skill into this project…'});
  rows.push({value: 'merge', label: 'Prepare a merge request for the agent…'}, {value: 'back', label: 'Back to review'});
  chat.picker(`${plain(candidate.name)}${candidate.existing ? ' · existing skill' : ' · proposed new skill'}`
    + (candidate.source_changed ? '\nProject files changed since the review snapshot.' : '')
    + (candidate.error ? `\n${plain(candidate.error)}` : ''), rows, async key => {
    const back = () => candidateMenu(chat, id, data, candidate);
    if (key === 'back') return showReview(chat, id);
    if (key === 'read') return preview(chat, await chat.client.projectSkills('candidate', {id, key: candidate.key}), back);
    if (key === 'diff') return preview(chat, textFile('Review changes and evidence', data.diff + '\n\n' + data.summary), back);
    if (key === 'import') return chat.confirm(`Import ${candidate.name} into this project?\n\nExisting skills will not be overwritten. Saving does not verify the method on this robot.`, async () => {
      const result = await chat.client.projectSkills('import', {id, key: candidate.key, fingerprint: candidate.fingerprint});
      chat.say(`Imported ${plain(result.name)} at ${plain(result.imported)}.`
        + (result.issues?.length ? ` ${result.issues.map(plain).join('; ')}` : ' Use it through the agent’s native skills.'));
    });
    const {draft} = await chat.client.projectSkills('merge', {id, key: candidate.key});
    chat.confirm('Put this merge request in the composer?\n\nIt will not be sent. Any existing draft will be kept above it.', async () => {
      const current = chat.editor.getText().trim();
      chat.projectDraft = (current ? current + '\n\n' : '') + draft;
      chat.editor.setText(chat.projectDraft);
      chat.say('Merge request prepared. Review it and press Enter when ready; nothing has been sent.');
    });
  });
}

export async function showReview(chat, id) {
  const data = await chat.client.projectSkills('report', {id});
  if (chat.done) return;
  const rows = [{value: 'summary', label: 'Read report and evidence'}, {value: 'diff', label: 'Read full diff'},
    ...data.candidates.map(row => ({value: row.key, label: plain(row.name),
      description: `${row.existing ? 'existing skill' : 'new candidate'}${row.source_changed ? ' · current files changed' : ''}`})),
    {value: 'refresh', label: 'Refresh report'}, {value: 'back', label: 'Back'}];
  chat.picker(`Skill review · ${plain(data.report.status)} · ${id.slice(0, 8)}`, rows, async key => {
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
    throw new Error('Open a live project conversation for this menu, or use openrua skills in the project folder.');
  const data = await chat.client.projectSkills('overview');
  if (chat.done) return;
  const active = data.tasks.find(task => ['running', 'starting', 'cancel_requested'].includes(task.status));
  if (active) { chat.reviewTask = active; await updateReview(chat, true); }
  const rows = [{value: 'skills', label: `Saved skills (${data.entries.length})`},
    {value: 'reviews', label: `Review reports (${data.reports.length})`},
    {value: 'tasks', label: `Review tasks (${data.tasks.length})`},
    {value: 'policy', label: `Accumulate methods: ${data.policy}`},
    ...(data.defaults ? [{value: 'start', label: 'Review completed work…'}] : []),
    ...(chat.pendingReview ? [{value: 'retry', label: 'Inspect/retry unconfirmed review start…'}] : []),
    {value: 'refresh', label: 'Refresh'}];
  chat.picker(`Project skills and reviews\n${plain(data.project)}`
    + (data.review_error ? `\nReview unavailable: ${plain(data.review_error)}` : ''), rows, async key => {
    if (key === 'refresh') return projectMenu(chat);
    if (key === 'start') return reviewOptions(chat, {...data.defaults}, data.agent_identity);
    if (key === 'retry') return startReview(chat, chat.pendingReview.options);
    if (key === 'policy') return chat.picker('Accumulation guides future work; saved skills remain usable.',
      ['auto', 'suggest', 'manual'].map(mode => ({value: mode, label: mode})), mode =>
        chat.confirm(`Set this project's accumulation guidance to ${mode}?`, async () => {
          await chat.client.projectSkills('policy', {mode}); chat.say(`Accumulation guidance: ${mode}.`);
        }));
    if (key === 'skills') return chat.picker('Saved skills · file metadata, not a validation result',
      data.entries.map(entry => ({value: entry.key, label: `${plain(entry.name)} [${entry.scope}]`, description: plain(entry.error || entry.description)})),
      async key => preview(chat, await chat.client.projectSkills('skill', {key}), () => projectMenu(chat)));
    if (key === 'reviews') return chat.picker('Review reports', data.reports.map(report => ({value: report.id,
      label: `${report.id.slice(0, 8)} · ${plain(report.status)}`, description: `${report.proposed_skills ?? 0} proposed skills · ${plain(report.started_at)}`})),
      id => showReview(chat, id));
    if (key === 'tasks') return chat.picker('Review tasks · continue independently of this terminal',
      data.tasks.map(task => ({value: task.id, label: `${task.id.slice(0, 8)} · ${plain(task.status)}`, description: plain(task.reason || task.error || task.started_at)})),
      async id => {
        const task = await chat.client.projectSkills('task', {id});
        if (task.review_id) return showReview(chat, task.review_id);
        chat.reviewTask = task; await updateReview(chat, true);
        chat.picker(`Skill review: ${plain(task.status)}` + (task.reason || task.error ? `\n${plain(task.reason || task.error)}` : ''),
          [{value: 'refresh', label: 'Refresh task'},
            ...(['running', 'starting', 'cancel_requested'].includes(task.status) ? [{value: 'cancel', label: 'Cancel this review…'}] : []),
            {value: 'back', label: 'Back'}], async action => {
            if (action !== 'cancel') return projectMenu(chat);
            chat.confirm('Cancel this skill review?\n\nThe robot conversation continues. Existing skills will be retained.', async () => {
              chat.reviewTask = await chat.client.projectSkills('cancel', {id});
              await updateReview(chat, true);
            });
          });
      });
  });
}
