import {Container, SelectList, Text, matchesKey} from '@earendil-works/pi-tui';
import {muted, plain, selectTheme} from './view.mjs';

// Native question semantics are normalized by the agent plugin.
export function answerQuestion(chat, question, accept) {
  if (!question.choices?.length) {
    chat.editText(question.text, '', text => accept([text])); return;
  }
  if (!question.multiple && !question.allow_other) {
    chat.picker(question.text, question.choices.map(value => ({value, label: plain(value)})),
      value => accept([value])); return;
  }
  const selected = new Set();
  let other = '';
  const items = question.choices.map(value => ({value, label: plain(value), kind: 'choice'}));
  if (question.allow_other) items.push({value: 'other', label: 'Write another answer…', kind: 'other'});
  if (question.multiple) items.push({value: 'continue', label: 'Continue', kind: 'continue'});
  const menu = new SelectList(items, 8, selectTheme);
  const box = new Container();
  const notice = new Text('', 0, 0);
  box.addChild(new Text(plain(question.text), 0, 1)); box.addChild(menu); box.addChild(notice);
  box.addChild(new Text(muted(question.multiple
    ? '↑/↓ move · Space/Enter toggle · Continue to review · Esc cancel'
    : '↑/↓ select · Enter confirm · Esc cancel'), 0, 1));
  const handle = chat.dialog(box);
  const refresh = () => {
    for (const item of items) {
      if (item.kind === 'choice') item.label = `${question.multiple ? selected.has(item.value) ? '[x] ' : '[ ] ' : ''}${plain(item.value)}`;
      if (item.kind === 'other') item.label = other ? `[x] ${plain(other)} (toggle off)` : 'Write another answer…';
    }
    chat.ui.requestRender();
  };
  menu.onCancel = () => handle.hide();
  menu.onSelect = item => {
    notice.setText('');
    if (item.kind === 'continue') {
      const values = [...selected, ...(other ? [other] : [])];
      if (!values.length) { notice.setText('Choose at least one answer.'); return; }
      handle.hide(); accept(values);
    } else if (item.kind === 'other') {
      if (other) { other = ''; refresh(); return; }
      chat.editText(question.text, '', text => {
        if (!question.multiple) { handle.hide(); accept([text]); return; }
        if (question.choices.includes(text)) selected.add(text); else other = text;
        refresh();
      });
    } else if (question.multiple) {
      if (selected.has(item.value)) selected.delete(item.value); else selected.add(item.value);
      refresh();
    } else { handle.hide(); accept([item.value]); }
  };
  box.handleInput = data => {
    if (question.multiple && matchesKey(data, 'space')) {
      const item = menu.getSelectedItem();
      if (item && item.kind !== 'continue') menu.onSelect(item);
    } else menu.handleInput(data);
  };
  refresh();
}
