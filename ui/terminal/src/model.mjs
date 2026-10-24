import {plain} from './view.mjs';

// Keep a proposed selection local until the last picker is confirmed.
// The native catalog owns model names, reasoning levels and defaults.
export function modelPicker(chat, selected) {
  const state = chat.controller.state;
  const current = state?.settings?.model || chat.facts.model;
  const choices = state?.setting_choices?.model ?? [];
  const nextTurn = state?.setting_effective_from?.model === 'next_turn';
  const apply = async (model, effort) => {
    if (effort === undefined) { await chat.configure('model', model); return; }
    else {
      await chat.client.command('select_model', {model, effort});
      await chat.refresh();
    }
    chat.say(`Selected ${model}${effort === undefined ? '' : ` · ${effort}`}${nextTurn ? ' for the next turn' : ''}.`);
  };
  const propose = async model => {
    model = model.trim();
    if (!model) throw new Error('Enter a model ID.');
    const preset = choices.find(choice => choice.value === model);
    const efforts = preset?.efforts?.length ? preset.efforts
      : preset?.default_effort ? [{value: preset.default_effort}] : [];
    if (!efforts.length || state?.settings?.effort === undefined) {
      await apply(model); return;
    }
    if (efforts.length === 1) { await apply(model, efforts[0].value); return; }
    const initial = (model === current && efforts.some(choice => choice.value === state.settings.effort))
      ? state.settings.effort : efforts.find(choice => choice.value === preset.default_effort)?.value ?? efforts[0].value;
    chat.picker(`Select Reasoning Level for ${plain(preset.label || model)}`
      + (nextTurn ? '\nApplies to the next turn.' : ''),
      efforts.map(choice => ({value: choice.value,
        label: `${model === current && choice.value === state.settings.effort ? '✓ ' : ''}${plain(choice.label || choice.value)}`
          + (choice.value === preset.default_effort ? ' (default)' : ''),
        description: plain(choice.description || '')})),
      effort => apply(model, effort), {selected: initial, onCancel: () => modelPicker(chat, model)});
  };
  chat.picker('Select Model' + (nextTurn ? '\nApplies to the next turn. An active turn keeps its model.' : ''), [
    ...(!choices.some(choice => choice.value === current) && current
      ? [{value: current, label: `✓ ${plain(current)}`}]: []),
    ...choices.map(choice => ({value: choice.value,
      label: `${choice.value === current ? '✓ ' : ''}${plain(choice.label || choice.value)}`,
      description: plain(choice.description || choice.value)})),
    {value: null, label: 'Enter a model ID…'},
  ], async value => {
    if (value === null) chat.editText('Model ID', current || '', propose,
      {onCancel: () => modelPicker(chat, null)});
    else await propose(value);
  }, {selected: selected === undefined ? current : selected});
}
