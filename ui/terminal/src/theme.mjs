import {backgroundAnsi, foregroundAnsi, parseColor} from '@earendil-works/pi-tui';

function sgrGroups(raw) {
  const values = raw ? raw.split(';').map(Number) : [0], groups = [];
  for (let index = 0; index < values.length; index++) {
    const code = values[index];
    const extended = code === 38 || code === 48 || code === 58;
    const extra = extended ? ({2: 4, 5: 2}[values[index + 1]] || 0) : 0;
    groups.push(values.slice(index, index + extra + 1));
    index += extra;
  }
  return groups;
}

// Semantic ANSI roles stay in component strings until the host paints a frame.
// Cached text can then follow terminal appearance changes without global state.
export const style = (open, close) => text => {
  const start = `\x1b[${open}m`;
  return start + String(text).replace(/\x1b\[([\d;]*)m/g, (sequence, raw) => {
    const groups = sgrGroups(raw);
    if (!groups.some(([code]) => code === close || code === 0)) return sequence;
    return groups.map(group => `\x1b[${group.join(';')}m` + (group[0] === close || group[0] === 0 ? start : '')).join('');
  })
    + `\x1b[${close}m`;
};
export const bold = style(1, 22), muted = style(90, 39), accent = style(36, 39);
export const brand = style(92, 39), border = style(94, 39);
export const green = style(32, 39), yellow = style(33, 39), red = style(31, 39);
export const panel = style(40, 49), selected = style(42, 49);

const palettes = {
  dark: {text: '#F0F2E9', brand: '#BDF36B', muted: '#A8BBB2', border: '#52685B',
    accent: '#63DACF', green: '#7DD99B', yellow: '#EDC776', red: '#F58C88', panel: '#111C16', selected: '#243B2B'},
  light: {text: '#24362B', brand: '#396B10', muted: '#52695C', border: '#718477',
    accent: '#087B74', green: '#237A43', yellow: '#916200', red: '#B63B37', panel: '#F0F6E9', selected: '#DEEDCE'},
};
const roles = {31: 'red', 32: 'green', 33: 'yellow', 36: 'accent', 90: 'muted', 92: 'brand', 94: 'border'};

export function colorMode(env, capabilities) {
  if (env.NO_COLOR || env.TERM === 'dumb') return 'none';
  if (capabilities.trueColor) return 'truecolor';
  return /256color/.test(env.TERM || '') ? '256color' : 'ansi';
}

function luminance({r, g, b}) {
  const linear = value => { const channel = value / 255; return channel <= 0.04045 ? channel / 12.92 : ((channel + 0.055) / 1.055) ** 2.4; };
  return linear(r) * 0.2126 + linear(g) * 0.7152 + linear(b) * 0.0722;
}

export class TerminalTheme {
  constructor(mode = 'ansi') {
    this.mode = mode;
    this.appearance = null;
    this.colors = null;
  }
  get adaptive() { return this.mode === 'truecolor' || this.mode === '256color'; }
  update(colors) {
    if (!colors.background) return false;
    return this.setAppearance(luminance(colors.background) > 0.18 ? 'light' : 'dark');
  }
  setAppearance(appearance) {
    if (!this.adaptive || !palettes[appearance] || this.appearance === appearance) return false;
    this.appearance = appearance;
    this.colors = Object.fromEntries(Object.entries(palettes[appearance]).map(([role, hex]) => [role,
      (role === 'panel' || role === 'selected' ? backgroundAnsi : foregroundAnsi)(parseColor(hex), this.mode)]));
    return true;
  }
  paint(line) {
    // Only resolve the SGR styles emitted by our components and Pi. Terminal
    // controls, cursor markers and links remain Pi's responsibility.
    const body = this.colors?.text || '';
    return body + line.replace(/\x1b\[([\d;]*)m/g, (sequence, raw) => {
      let result = '';
      for (const group of sgrGroups(raw)) {
        const [code] = group;
        if (code === 38 || code === 48 || code === 58) {
          if (this.mode !== 'none') result += `\x1b[${group.join(';')}m`;
        } else if (code === 0) {
          result += '\x1b[0m' + body;
        } else if ((code >= 30 && code <= 49) || (code >= 90 && code <= 107) || code === 59) {
          if (this.mode === 'none') continue;
          if (code === 39) result += body || '\x1b[39m';
          else if (code === 40 || code === 42) result += this.colors?.[code === 40 ? 'panel' : 'selected'] || '';
          else if (this.colors && roles[code]) result += this.colors[roles[code]];
          else result += `\x1b[${code === 92 ? 32 : code === 94 ? 90 : code}m`;
        } else result += `\x1b[${code}m`;
      }
      return result;
    });
  }
}
