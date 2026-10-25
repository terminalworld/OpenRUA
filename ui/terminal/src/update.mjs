import {Text, matchesKey} from '@earendil-works/pi-tui';
import {Screen} from './surface.mjs';
import {selection} from './selection.mjs';
import {plain, muted} from './view.mjs';

export class Update extends Screen {
  constructor(spec, terminal, host) {
    super(terminal, host);
    this.ui.setHeading('OpenRUA · Software update');
    this.ui.addChild(new Text(`${plain(spec.name)}\n${plain(spec.installed)} → ${plain(spec.target)}`, 0, 1));
    if (spec.phase === 'logs') {
      this.ui.addChild(new Text(plain(spec.log_path), 0, 1));
      this.ui.addChild(new Text(plain(spec.text), 0, 0));
      this.ui.addInputListener(data => {
        if (matchesKey(data, 'escape') || matchesKey(data, 'enter')) { this.finish({action: 'back'}); return {consume: true}; }
      });
      this.ui.setFooter(new Text(muted('PgUp/PgDn Scroll · Enter Back · Esc Back'), 0, 1));
      return;
    }
    const offer = spec.phase === 'offer';
    const message = offer ? spec.required ? 'This update is required for this agent to work with OpenRUA.'
      : 'An update is available. You can continue with the installed version.'
      : spec.success ? spec.restart ? 'Update installed. Close OpenRUA and run it again to use the new version.'
        : 'Update verified. New sessions will use this version. Running sessions keep their current environment.'
        : spec.restart ? 'Update failed. Inspect the log before retrying.'
          : 'Update failed. Running sessions keep their current environment. Inspect the log before retrying.';
    this.ui.addChild(new Text(message, 0, 1));
    const rows = offer ? [
      {value: 'update', label: 'Update now'},
      ...(spec.required ? [{value: 'quit', label: 'Quit'}] : [
        {value: 'continue', label: 'Continue with current version'},
        {value: 'skip', label: 'Skip this version'},
      ]),
    ] : [
      ...(spec.success ? [{value: spec.restart ? 'close' : 'continue', label: spec.restart ? 'Close OpenRUA' : 'Continue'}]
        : [...(!spec.required ? [{value: 'continue', label: 'Continue with current version'}] : []),
          {value: 'retry', label: 'Retry update'}]),
      {value: 'logs', label: 'View update log'},
      ...(!spec.success && spec.required ? [{value: 'quit', label: 'Quit'}] : []),
    ];
    const defaultAction = offer ? spec.required ? 'update' : 'continue'
      : spec.success ? spec.restart ? 'close' : 'continue' : spec.required ? 'quit' : 'continue';
    const menu = selection(rows, 6, {selected: defaultAction});
    menu.onSelect = item => this.finish({action: item.value});
    menu.onCancel = () => this.finish({action: defaultAction === 'update' ? 'quit' : defaultAction});
    this.ui.addChild(menu); this.ui.setFocus(menu);
    this.ui.setFooter(new Text(muted('↑/↓ Select · Enter Confirm · Esc Back · Ctrl+D Quit'), 0, 1));
  }
}
