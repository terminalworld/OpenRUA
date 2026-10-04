import {randomUUID} from 'node:crypto';
import {ResponseError} from './client.mjs';

export class Controller {
  constructor(client, view, clientId = randomUUID()) {
    this.client = client;
    this.view = view;
    this.clientId = clientId;
    this.cursor = 0;
    this.state = null;
    this.pending = null;
  }

  refresh() {
    if (!this.refreshing) this.refreshing = this.read().finally(() => { this.refreshing = null; });
    return this.refreshing;
  }

  async read() {
    const {state} = await this.client.snapshot();
    this.state = state;
    this.view.snapshot(state);
    if (this.pending && state.messages.some(m => m.client_id === this.clientId && m.request_id === this.pending.request_id)) {
      this.pending = null;
    }
    // Replay batches by their server cursor, including events accepted by another UI.
    for (let batch = 0; batch < 10; batch++) {
      const events = await this.client.events(this.cursor);
      for (const event of events) {
        if (event.seq <= this.cursor) continue;
        this.view.event(event);
        this.cursor = event.seq;
      }
      if (events.length < 1000) break;
    }
  }

  async send(text) {
    if (this.pending) throw new Error('The previous send is unconfirmed. Use /retry or wait for its acceptance.');
    this.pending = {client_id: this.clientId, request_id: randomUUID(), text};
    return this.retry();
  }

  async retry() {
    if (!this.pending) throw new Error('No unconfirmed send to retry.');
    try {
      const message = await this.client.command('enqueue', this.pending);
      this.pending = null;
      return message;
    } catch (error) {
      if (error instanceof ResponseError && error.status >= 400 && error.status < 500) this.pending = null;
      throw error;
    }
  }
}
