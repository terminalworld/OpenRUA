import {randomUUID} from 'node:crypto';
import {plain} from './view.mjs';
import {displayStatus} from './session.mjs';

const days = ['mon', 'tue', 'wed', 'thu', 'fri', 'sat', 'sun'];

export async function schedulesMenu(chat, context, reviewOptions, openReport) {
  const api = (operation, params = {}) => chat.client.projectSkills(operation, params);
  const back = () => schedulesMenu(chat, context, reviewOptions, openReport);
  const data = await api('schedules');
  if (chat.done) return;
  const rows = data.schedules.map(plan => ({value: plan.id,
    label: `${plan.enabled ? 'Enabled' : 'Paused'} · ${plain(plan.name)}`,
    description: `${plain(plan.calendar.time)} ${plain(plan.calendar.timezone)} · ${plain(plan.calendar.days)}`}));
  if (context.defaults) rows.push({value: 'create', label: 'Schedule a review…'});
  if (chat.pendingSchedule) rows.push({value: 'retry', label: 'Check the last schedule request…'});
  rows.push({value: 'refresh', label: 'Refresh'});
  chat.picker(`Scheduled skill reviews · scheduler: ${plain(displayStatus(data.scheduler.state))}\n`
    + 'Reviews keep running when you close this terminal. After restarting your computer, open OpenRUA to start scheduling again.\n'
    + 'Missed reviews are combined into one catch-up review. Sign-in or usage-limit problems pause the schedule.', rows, async key => {
    if (key === 'refresh') return back();
    if (key === 'retry') {
      const saved = await api('schedule-create', chat.pendingSchedule);
      chat.pendingSchedule = null;
      chat.say(`Schedule enabled: ${plain(saved.name)} · next ${plain(saved.next_run)}.`
        + (saved.scheduler && !saved.scheduler.running ? ' The scheduler is stopped. Run openrua skills scheduler start to resume your schedules.' : ''));
      return back();
    }
    if (key === 'create') return form({name: '', calendar: {time: '', days: days.join(','),
      timezone: Intl.DateTimeFormat().resolvedOptions().timeZone || 'UTC'}, options: {...context.defaults}});
    return details(data.schedules.find(plan => plan.id === key));
  });

  function details(plan) {
    const choices = [{value: 'edit', label: 'Edit name, time or review settings…'},
      {value: plan.enabled ? 'pause' : 'resume', label: plan.enabled ? 'Pause future reviews…' : 'Resume future reviews…'}];
    if (plan.last_result?.review_id) choices.push({value: 'report', label: 'Read last report and proposals'});
    choices.push({value: 'remove', label: 'Remove schedule…'}, {value: 'back', label: 'Back'});
    chat.picker(`${plain(plan.name)} · ${plan.enabled ? 'enabled' : 'paused'}\n`
      + `Next time: ${plain(plan.next_run)}\nAgent: ${plain(plan.options.agent)} · model: ${plain(plan.options.model)} · effort: ${plain(plan.options.effort || 'default')}\n`
      + `Current review: ${plain(plan.active_task || 'none')} · Last result: ${plain(displayStatus(plan.last_result?.status || 'none'))}`
      + (plan.reason ? `\n${plain(plan.reason)}` : ''), choices, async action => {
      if (action === 'back') return back();
      if (action === 'report') return openReport(chat, plan.last_result.review_id);
      if (action === 'edit') return form(structuredClone({name: plan.name, calendar: plan.calendar, options: plan.options}), plan);
      chat.confirm(`${action === 'resume' ? 'Resume' : action === 'pause' ? 'Pause' : 'Remove'} ${plain(plan.name)}?\n\n`
        + (action === 'resume' ? 'Future reviews use the account and model set in this schedule. They do not start a robot.'
          : 'This changes future reviews. A review already running will continue. Open Review tasks to cancel it. Your reports and skills are kept.'), async () => {
        await api('schedule-action', {id: plan.id, action});
        return back();
      });
    });
  }

  function form(definition, existing = null) {
    const calendar = definition.calendar;
    chat.picker(existing ? 'Edit review schedule' : 'Create review schedule', [
      {value: 'name', label: `Name: ${plain(definition.name || '(choose a name)')}`},
      {value: 'time', label: `Time: ${plain(calendar.time || '(choose HH:MM)')}`},
      {value: 'days', label: `Days: ${plain(calendar.days)}`},
      {value: 'timezone', label: `Time zone: ${plain(calendar.timezone)}`},
      {value: 'options', label: `Review: ${plain(definition.options.agent)} · ${plain(definition.options.model)} · effort ${plain(definition.options.effort || 'default')}`},
      {value: 'save', label: existing ? 'Save changes…' : 'Enable schedule…'},
      {value: 'back', label: 'Back'},
    ], async key => {
      if (key === 'back') return back();
      if (key === 'options') {
        const resolved = await api('defaults', {agent: definition.options.agent});
        return reviewOptions(chat, {...definition.options}, resolved.agent_identity,
          {label: 'Continue to schedule…', finish: options => { definition.options = options; form(definition, existing); },
            back: () => form(definition, existing)});
      }
      if (key === 'days') return chat.picker('Run on', [
        {value: days.join(','), label: 'Every day'}, {value: days.slice(0, 5).join(','), label: 'Weekdays'},
        ...days.map(day => ({value: day, label: `Weekly on ${day}`})),
      ], value => { calendar.days = value; form(definition, existing); }, {selected: calendar.days});
      if (key === 'save') {
        if (!definition.name.trim() || !/^(?:[01]\d|2[0-3]):[0-5]\d$/.test(calendar.time))
          throw new Error('Choose a name and an HH:MM time before enabling this schedule.');
        const options = definition.options;
        return chat.confirm(`${existing ? 'Save' : 'Enable'} this periodic skill review?\n\n`
          + `${plain(definition.name)}\n${plain(calendar.time)} ${plain(calendar.timezone)} · ${plain(calendar.days)}\n`
          + `Agent: ${plain(options.agent)}\nModel: ${plain(options.model || 'default')} · effort: ${plain(options.effort || 'default')}\n`
          + `Container image: ${plain(options.image)} · Review time limit: ${options.wall_clock_min} minutes\n\n`
          + 'Reviews run in the background using your account or API key. The time limit does not limit API costs. '
          + 'Reviews do not start a robot. You choose which suggested skills to save.', async () => {
          let saved;
          if (existing) saved = await api('schedule-update', {id: existing.id, revision: existing.revision, definition});
          else {
            if (chat.pendingSchedule && JSON.stringify(chat.pendingSchedule.definition) !== JSON.stringify(definition))
              throw new Error('We have not confirmed that the last schedule was saved. Check it before creating another.');
            chat.pendingSchedule ??= {request_id: randomUUID().replaceAll('-', ''), definition: structuredClone(definition)};
            saved = await api('schedule-create', chat.pendingSchedule);
            chat.pendingSchedule = null;
          }
          chat.say(`Schedule saved: ${plain(saved.name)} · ${saved.enabled ? 'enabled' : 'paused'} · next ${plain(saved.next_run)}.`);
          if (saved.enabled && saved.scheduler && !saved.scheduler.running)
            chat.say('The schedule is saved, but the scheduler is stopped. Run openrua skills scheduler start to start it.');
          return back();
        });
      }
      chat.editText(key === 'time' ? 'Local time (HH:MM)' : key === 'timezone' ? 'Time zone, such as UTC or Europe/London' : 'Schedule name',
        key === 'name' ? definition.name : calendar[key], value => {
          if (key === 'name') definition.name = value.trim(); else calendar[key] = value.trim();
          form(definition, existing);
        });
    });
  }
}
