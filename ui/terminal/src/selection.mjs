import {SelectList} from '@earendil-works/pi-tui';
import {selectTheme} from './view.mjs';

export function selection(items, height, {selected, explicit = false} = {}) {
  const rows = items.map((item, index) => ({...item, label: `${index + 1}. ${item.label}`}));
  const menu = new SelectList(rows, height, selectTheme);
  const at = items.findIndex(item => item.value === selected);
  if (at >= 0) menu.setSelectedIndex(at);
  const handle = menu.handleInput.bind(menu);
  menu.handleInput = data => {
    const index = /^[1-9]$/.test(data) ? Number(data) - 1 : -1;
    if (index >= 0 && index < rows.length) {
      menu.setSelectedIndex(index);
      if (!explicit) menu.onSelect?.(rows[index]);
    } else handle(data);
  };
  return menu;
}
