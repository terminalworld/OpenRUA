import {Container, ProcessTerminal, ScrollView, Text, TuiAltScreen, VStack, isKeyRelease, isKeyRepeat, matchesKey, truncateToWidth} from '@earendil-works/pi-tui';
import {bold, muted, plain} from './view.mjs';
import {isImageLine} from '@earendil-works/pi-tui/dist/terminal-image.js';
import {brand, TerminalTheme} from './theme.mjs';

class Renderer extends TuiAltScreen {
  constructor(terminal, theme) { super(terminal); this.theme = theme; }
  applyLineResets(lines) {
    return super.applyLineResets(lines.map(line => isImageLine(line) ? line : this.theme.paint(line)));
  }
  handleViewportInput(data) {
    if (this.navigation?.(data)) return {consume: true};
    return super.handleViewportInput(data);
  }
}

export class TerminalHost {
  constructor(terminal = new ProcessTerminal(), {theme = new TerminalTheme()} = {}) {
    this.theme = theme;
    this.renderer = new Renderer(terminal, theme);
    this.active = false;
    if (theme.adaptive) {
      this.renderer.onTerminalColorSchemeChange(appearance => {
        if (theme.setAppearance(appearance)) this.renderer.requestRender(true);
      });
      this.renderer.setTerminalColorSchemeNotifications(true);
    }
  }
  start() {
    if (!this.active) { this.renderer.start(); this.active = true; }
    if (this.theme.adaptive && !this.queriedColors) {
      this.queriedColors = true;
      const update = colors => { if (this.theme.update(colors) && this.active) this.renderer.requestRender(true); };
      this.renderer.queryTerminalColors({timeoutMs: 150, onLateReply: update}).then(update);
    }
    this.renderer.requestRender();
  }
  stop() {
    if (!this.active) return;
    this.renderer.stop({preserveScreen: true});
    this.active = false;
  }
  close() { this.closePage?.(); this.stop(); }
}

export function surface(terminal, host = null, area = 'body') {
  const owner = host || new TerminalHost(terminal);
  const renderer = owner.renderer;
  const canvas = new Container(), empty = new Container();
  let header = new Text(brand(bold('OpenRUA')), 0, 1);
  let body = area === 'body' ? canvas : empty;
  let bottom = area === 'footer' ? canvas : new Container();
  const listeners = [];
  const mount = () => renderer.setLayoutRoot(new VStack([
    {component: header, basis: 'auto', shrink: 0},
    {component: new ScrollView(body, {follow: 'end', primary: true, overscroll: 'chain'}), basis: 0, grow: 1, minSize: 1},
    {component: bottom, basis: 'auto', shrink: 1, minSize: 1},
  ]));
  mount();
  const local = {
    children: canvas.children,
    addChild: component => canvas.addChild(component),
    setHeader(component) { header = component; mount(); },
    setHeading(title, detail = '') {
      header = {render: width => [brand(bold(plain(title))), ...(detail ? [muted(truncateToWidth(plain(detail), width))] : []), '']};
      mount();
    },
    setBody(component) { body = component; mount(); },
    setFooter(component) { bottom = component; mount(); },
    addInputListener(callback) {
      // Pi filters releases before focused components, after custom listeners.
      const remove = renderer.addInputListener(data => isKeyRelease(data) ? undefined : callback(data));
      listeners.push(remove); return remove;
    },
    setNavigationHandler(callback) {
      renderer.navigation = callback;
      listeners.push(() => { if (renderer.navigation === callback) renderer.navigation = null; });
    },
    setCloseHandler(callback) {
      owner.closePage = callback;
      listeners.push(() => { if (owner.closePage === callback) owner.closePage = null; });
    },
    start: () => owner.start(),
    stop() { for (const remove of listeners.splice(0)) remove(); if (!host) owner.stop(); },
  };
  return new Proxy(local, {get(target, key) {
    if (key in target) return target[key];
    const value = renderer[key];
    return typeof value === 'function' ? value.bind(renderer) : value;
  }});
}

export class Screen {
  constructor(terminal, host, {quitKeys = true} = {}) {
    this.ui = surface(terminal, host);
    this.result = {action: 'quit'}; this.done = false;
    this.ui.setCloseHandler(() => this.finish());
    this.ui.addInputListener(data => {
      if (quitKeys && (matchesKey(data, 'ctrl+c') || matchesKey(data, 'ctrl+d'))) {
        if (!isKeyRepeat(data)) this.finish();
        return {consume: true};
      }
    });
  }
  finish(result = {action: 'quit'}) {
    if (this.done) return;
    this.done = true; this.result = result; this.ui.stop(); this.resolve?.(result);
  }
  stop() { this.finish(); }
  async run() {
    if (this.done) return this.result;
    return new Promise(resolve => { this.resolve = resolve; this.ui.start(); });
  }
}
