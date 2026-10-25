import {randomUUID} from 'node:crypto';
import {ResponseError} from './client.mjs';

export class Controller {
  constructor(client, view, clientId = randomUUID()) {
    this.client = client;
    this.view = view;
    this.clientId = clientId;
    this.cursor = 0;
    this.state = null;
    this.replayThrough = undefined;
    this.pending = null;
  }

  refresh() {
    // A refresh requested after a mutation must read after any earlier poll.
    // Reusing that poll can return a snapshot taken before the mutation.
    const previous = this.refreshing;
    const current = (async () => {
      if (previous) await previous.catch(() => {});
      await this.read();
    })();
    this.refreshing = current;
    return current.finally(() => {
      if (this.refreshing === current) this.refreshing = null;
    });
  }

  async read() {
    const {state, cursor: through} = await this.client.snapshot();
    if (this.replayThrough === undefined) this.replayThrough = through ?? 0;
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
        // An event newer than this state is read with the next snapshot.
        // Advancing past it would lose its delivery position on that poll.
        if (typeof through === 'number' && event.seq > through) return;
        this.view.event(event, {replay: event.seq <= this.replayThrough});
        this.cursor = event.seq;
      }
      if (events.length < 1000) break;
    }
  }

  async send(text) {
    if (this.pending) throw new Error('We have not confirmed that your last message arrived. Wait or use /retry.');
    this.pending = {client_id: this.clientId, request_id: randomUUID(), text};
    return this.retry();
  }

  async retry() {
    if (!this.pending) throw new Error('There is no message waiting to be retried.');
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
