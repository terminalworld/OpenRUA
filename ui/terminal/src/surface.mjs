import {Container, ProcessTerminal, ScrollView, Text, TuiAltScreen, VStack, matchesKey, truncateToWidth} from '@earendil-works/pi-tui';
import {bold, muted, plain} from './view.mjs';

class Renderer extends TuiAltScreen {
  handleViewportInput(data) {
    if (this.navigation?.(data)) return {consume: true};
    return super.handleViewportInput(data);
  }
}

export class TerminalHost {
  constructor(terminal = new ProcessTerminal()) {
    this.renderer = new Renderer(terminal);
    this.active = false;
  }
  start() {
    if (!this.active) { this.renderer.start(); this.active = true; }
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
  let header = new Text(bold('OpenRUA'), 0, 1);
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
      header = {render: width => [bold(plain(title)), ...(detail ? [muted(truncateToWidth(plain(detail), width))] : []), '']};
      mount();
    },
    setBody(component) { body = component; mount(); },
    setFooter(component) { bottom = component; mount(); },
    addInputListener(callback) { const remove = renderer.addInputListener(callback); listeners.push(remove); return remove; },
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
        this.finish(); return {consume: true};
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
