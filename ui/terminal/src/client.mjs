import {readFile} from 'node:fs/promises';
import http from 'node:http';

export class ResponseError extends Error {
  constructor(status, message) { super(message); this.status = status; }
}

// This client uses the existing session API; it owns no execution or queue.
export class Client {
  constructor(url, token, timeout = 130_000) {
    const target = new URL(url);
    if (target.protocol !== 'http:' || target.hostname !== '127.0.0.1' || !target.port ||
        target.pathname !== '/' || target.search || target.hash || target.username || target.password) {
      throw new Error('Use a local HTTP session origin from an OpenRUA endpoint file.');
    }
    if (typeof token !== 'string' || !token || /[\r\n]/.test(token)) throw new Error('Invalid session credential.');
    this.origin = target.origin;
    this.token = token;
    this.timeout = timeout;
    this.requests = new Set();
  }

  static async fromFile(path) {
    const {url, token} = JSON.parse(await readFile(path, 'utf8'));
    return new Client(url, token);
  }

  request(path, body) {
    const raw = body === undefined ? undefined : Buffer.from(JSON.stringify(body));
    return new Promise((resolve, reject) => {
      // Node's direct HTTP client does not route credentials through proxy env vars.
      const request = http.request(this.origin + path, {
        method: raw ? 'POST' : 'GET',
        headers: {'Authorization': `Bearer ${this.token}`, 'Content-Type': 'application/json',
          ...(raw ? {'Content-Length': raw.length} : {})},
      }, response => {
        let size = 0;
        const chunks = [];
        response.on('data', chunk => {
          size += chunk.length;
          if (size > 32 * 1024 * 1024) response.destroy(new Error('Session response exceeds 32 MiB.'));
          else chunks.push(chunk);
        });
        response.on('error', reject);
        response.on('end', () => {
          try {
            const value = JSON.parse(Buffer.concat(chunks).toString('utf8'));
            if (response.statusCode !== 200) throw new ResponseError(response.statusCode, value.error ?? 'Session request rejected.');
            resolve(value);
          } catch (error) { reject(error); }
        });
      });
      this.requests.add(request);
      const timer = setTimeout(() => request.destroy(new Error('Session request timed out.')), this.timeout);
      request.once('close', () => { clearTimeout(timer); this.requests.delete(request); });
      request.on('error', () => reject(new Error('Session unavailable or command result unknown. Reconnect and inspect state before retrying.')));
      request.end(raw);
    });
  }

  close() { for (const request of this.requests) request.destroy(); }

  async end() { return (await this.request('/api/end', {})).result; }
  snapshot() { return this.request('/api/session'); }
  events(after = 0) { return this.request(`/api/events?after=${after}&limit=1000`); }
  async command(operation, params) { return (await this.request('/api/commands', {operation, params})).result; }
}
