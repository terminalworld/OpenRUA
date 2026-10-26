import assert from 'node:assert/strict';
import {test} from 'node:test';
import {stripVTControlCharacters as strip} from 'node:util';
import {Editor, StdinBuffer, Text, visibleWidth} from '@earendil-works/pi-tui';
import {TerminalTheme, accent, bold, brand, colorMode, green, muted, panel, red, selected, yellow} from '../src/theme.mjs';
import {Composer, Welcome, editorTheme} from '../src/view.mjs';
import {selection} from '../src/selection.mjs';
import {TerminalHost, surface} from '../src/surface.mjs';

const dark = {background: {r: 24, g: 28, b: 25}};
const light = {background: {r: 255, g: 255, b: 255}};
const rgb = hex => `\x1b[38;2;${hex.match(/../g).map(value => parseInt(value, 16)).join(';')}m`;
const hasColor = /\x1b\[(?:3[0-9]|4[0-9]|9[0-7]|10[0-7]|5[89])(?:;[\d;]+)?m/;
const themeFor = (mode = 'truecolor', colors = dark) => { const theme = new TerminalTheme(mode); theme.update(colors); return theme; };

class Terminal {
  columns = 80; rows = 24; kittyProtocolActive = false; output = [];
  start(input) { this.buffer = new StdinBuffer(); this.buffer.on('data', input); }
  input(data) { this.buffer.process(data); }
  stop() { this.buffer?.destroy(); }
  write(data) { this.output.push(data); }
  async drainInput() {} moveBy() {} hideCursor() {} showCursor() {} clearLine() {}
  clearFromCursor() {} clearScreen() {} setTitle() {} setProgress() {}
}

test('color policy respects explicit opt-out and distinguishes 16, 256 and truecolor terminals', () => {
  assert.equal(colorMode({NO_COLOR: '1', TERM: 'xterm-256color'}, {trueColor: true}), 'none');
  assert.equal(colorMode({NO_COLOR: '0'}, {trueColor: true}), 'none');
  assert.equal(colorMode({TERM: 'dumb'}, {trueColor: true}), 'none');
  assert.equal(colorMode({NO_COLOR: '', TERM: 'xterm'}, {trueColor: true}), 'truecolor');
  assert.equal(colorMode({TERM: 'screen-256color'}, {trueColor: false}), '256color');
  assert.equal(colorMode({TERM: 'vt100'}, {trueColor: false}), 'ansi');
});

test('unknown appearance uses terminal colors without assuming a dark background', () => {
  const theme = new TerminalTheme('truecolor');
  const line = panel(brand('OpenRUA') + muted(' metadata'));
  assert.equal(theme.update({foreground: {r: 255, g: 255, b: 255}}), false);
  const output = theme.paint(line);
  assert.match(output, /\x1b\[32mOpenRUA/);
  assert.doesNotMatch(output, /\x1b\[(?:38;|48;|40m|42m)/);
  assert.equal(strip(output), 'OpenRUA metadata');
});

test('dark and light hosts have independent palettes and no global background', () => {
  const first = themeFor(), second = themeFor('truecolor', light);
  const text = brand('OpenRUA') + ' works with ' + accent('native interfaces');
  assert.ok(first.paint(text).includes(rgb('BDF36B')));
  assert.ok(second.paint(text).includes(rgb('396B10')));
  second.setAppearance('dark');
  first.setAppearance('light');
  assert.equal(second.appearance, 'dark');
  assert.equal(first.appearance, 'light');
  for (const theme of [first, second]) {
    assert.doesNotMatch(theme.paint(text), /\x1b\[(?:4[0-8]|10[0-7])(?:;|m)/);
    const statuses = [brand, accent, green, yellow, red].map(format => theme.paint(format('status')));
    assert.equal(new Set(statuses).size, 5);
  }
});

test('256-color output delegates quantization to Pi and no-color keeps structure and controls', () => {
  const line = selected(bold(brand('OpenRUA')) + muted(' ready')) + '\x1b[7m cursor\x1b[27m';
  const indexed = themeFor('256color').paint(line);
  assert.match(indexed, /\x1b\[38;5;\d+m/);
  assert.match(indexed, /\x1b\[48;5;\d+m/);
  assert.doesNotMatch(indexed, /\x1b\[(?:38|48);2;/);
  const monochrome = themeFor('none').paint(line + '\x1b[38;2;1;2;3mRGB\x1b[48;5;15mBG');
  assert.doesNotMatch(monochrome, hasColor);
  assert.ok(monochrome.includes('\x1b[1m'));
  assert.ok(monochrome.includes('\x1b[7m'));
  assert.equal(strip(monochrome), 'OpenRUA ready cursorRGBBG');
});

test('nested foreground and local surfaces restore their enclosing roles', () => {
  const theme = themeFor();
  const foreground = theme.paint(brand('one ' + muted('two') + ' three'));
  assert.ok(foreground.includes(rgb('A8BBB2') + 'two' + rgb('F0F2E9') + rgb('BDF36B') + ' three'));
  const surface = theme.paint(panel('one ' + selected('two') + ' three\x1b[0m four'));
  assert.ok(surface.includes('\x1b[48;2;36;59;43mtwo\x1b[49m\x1b[48;2;17;28;22m three'));
  assert.ok(surface.includes('\x1b[0m' + rgb('F0F2E9') + '\x1b[48;2;17;28;22m four'));
  assert.ok(surface.endsWith('\x1b[49m'));
  assert.ok(theme.paint(brand('one\x1b[39;22mtwo')).includes(rgb('BDF36B') + '\x1b[22mtwo'));
  assert.ok(theme.paint(panel('one\x1b[0;1mtwo')).includes('\x1b[48;2;17;28;22m\x1b[1mtwo'));
  const controls = '\x1b[2K\x1b[5;7H\x1b[?25l';
  assert.ok(theme.paint(brand(controls + 'text')).includes(controls));
  assert.ok(themeFor('none').paint(brand(controls + 'text')).includes(controls));
  assert.ok(brand('\x1b[38;2;0;39;22mRGB').includes('\x1b[38;2;0;39;22mRGB'));
});

test('welcome and selection retain text, widths and keyboard behavior in every palette', () => {
  const welcome = new Welcome('example', {agent_title: 'Claude Code', model: 'opus', project: '/workspace', backend: 'sim'});
  const menu = selection([{value: 'sim', label: 'Simulation', description: 'Use a simulator'},
    {value: 'real', label: 'Real robot', description: 'Connect to a robot'}], 2);
  let chosen;
  menu.onSelect = item => { chosen = item.value; };
  const active = themeFor().paint(menu.render(80)[0]);
  assert.ok(active.includes(rgb('BDF36B') + '\x1b[1m→ '));
  assert.ok(active.includes('\x1b[48;2;36;59;43m'));
  for (const width of [12, 40, 80, 120]) {
    const lines = [...welcome.render(width), ...menu.render(width)];
    for (const theme of [themeFor(), themeFor('truecolor', light), themeFor('256color'), new TerminalTheme(), themeFor('none')]) {
      for (const line of lines) {
        assert.equal(strip(theme.paint(line)), strip(line));
        assert.equal(visibleWidth(theme.paint(line)), visibleWidth(line));
        assert.ok(visibleWidth(theme.paint(line)) <= width);
      }
    }
  }
  menu.handleInput('\x1b[B'); menu.handleInput('\r');
  assert.equal(chosen, 'real');
  menu.handleInput('1');
  assert.equal(chosen, 'sim');
});

test('composer uses native rendering, editing and cursor markers without adding a prompt', () => {
  const ui = {terminal: {rows: 24}, requestRender() {}};
  const native = new Editor(ui, editorTheme), composer = new Composer(ui, editorTheme);
  for (const editor of [native, composer]) {
    editor.focused = true;
    editor.setText('Move the cup');
    editor.handleInput('\x01'); editor.handleInput('Please ');
  }
  assert.equal(composer.getText(), native.getText());
  for (const width of [20, 80]) {
    const original = native.render(width), styled = composer.render(width);
    assert.deepEqual(styled.map(line => strip(line).trimEnd()), original.map(line => strip(line).trimEnd()));
    assert.ok(styled.every(line => visibleWidth(line) <= width));
    assert.equal(styled.join('').includes('❯'), false);
    for (const line of styled) assert.equal(visibleWidth(themeFor().paint(line)), visibleWidth(line));
  }
});

test('Pi consumes fragmented late appearance replies while normal input and cached text survive', async t => {
  const terminal = new Terminal(), theme = new TerminalTheme('truecolor');
  const host = new TerminalHost(terminal, {theme});
  const ui = surface(terminal, host, 'footer');
  const heading = new Text(brand('Cached OpenRUA'), 0, 0);
  ui.setHeader(heading);
  const editor = new Composer(ui, editorTheme);
  ui.addChild(editor); ui.setFocus(editor);
  t.after(() => host.stop());
  const before = host.renderer.applyLineResets(heading.render(80))[0];
  ui.start();
  await new Promise(resolve => setTimeout(resolve, 170));
  assert.equal(theme.appearance, null);
  terminal.input('\x1b]11;rgb:1818/');
  terminal.input('1c1c/1919\x07x\x1b[?1;2c');
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(theme.appearance, 'dark');
  assert.equal(editor.getText(), 'x');
  const after = host.renderer.applyLineResets(heading.render(80))[0];
  assert.notEqual(before, after);
  assert.ok(after.includes(rgb('BDF36B')));
  assert.match(after, /\x1b\[0m/);
  terminal.input('\x1b[?997;2n');
  assert.equal(theme.appearance, 'light');
  assert.equal(editor.getText(), 'x');
  host.stop(); host.start();
  assert.equal(terminal.output.filter(data => data.includes('\x1b]11;?')).length, 1);
});

test('final line resets prevent color leakage and inline image protocols remain untouched', () => {
  const host = new TerminalHost(new Terminal(), {theme: themeFor()});
  const [line] = host.renderer.applyLineResets([panel(brand('OpenRUA'))]);
  assert.match(line, /\x1b\[0m\x1b\]8;;\x07$/);
  for (const image of ['\x1b_Ga=T;image\x1b\\', '\x1b]1337;File=inline=1:image\x07']) {
    assert.equal(host.renderer.applyLineResets([image])[0], image);
  }
});
