const $ = (id) => document.getElementById(id);
const make = (tag, text, className) => {
  const e = document.createElement(tag);
  if (text !== undefined) e.textContent = text;
  if (className) e.className = className;
  return e;
};
const key = "openrua-draft";
let saved;
try {
  saved = JSON.parse(sessionStorage.getItem(key));
} catch {
  saved = null;
}
let clientId = saved?.clientId || crypto.randomUUID(),
  pending = saved?.pending || null;
let token = "",
  state = null,
  cursor = 0,
  timer = null,
  syncing = false,
  online = false,
  ending = false,
  sending = false,
  generation = 0;
let turns = new Map(),
  outputs = new Map(),
  questionVersion = "",
  editTarget = null;
function saveDraft() {
  sessionStorage.setItem(key, JSON.stringify({ clientId, pending }));
}
function notice(text) {
  $("notice").textContent = text;
  $("notice").hidden = !text;
}
function connection(text, connected) {
  online = connected;
  $("connection").textContent = text;
  $("connection-light").classList.toggle("online", connected);
}
async function api(path, body) {
  const response = await fetch(path, {
    method: body === undefined ? "GET" : "POST",
    headers: {
      Authorization: `Bearer ${token}`,
      "Content-Type": "application/json",
    },
    body: body === undefined ? undefined : JSON.stringify(body),
    signal: AbortSignal.timeout(130000),
  });
  const value = await response.json();
  if (!response.ok)
    throw new Error(value.error || `Request failed (${response.status})`);
  return value;
}
async function command(operation, params) {
  return (await api("/api/commands", { operation, params })).result;
}
async function act(operation, params) {
  if (!online || ending) return false;
  try {
    await command(operation, params);
    notice("");
    await sync();
    return true;
  } catch (e) {
    notice(`${e.message}. Refresh state before repeating an operation.`);
    return false;
  }
}
function turn(message) {
  let entry = turns.get(message.id);
  if (!entry) {
    $("thread").querySelector(".empty")?.remove();
    const article = make("article"),
      heading = make("div", undefined, "turn-heading"),
      status = make("span"),
      user = make("div", undefined, "user-text"),
      result = make("div", undefined, "turn-result");
    heading.append(make("span", "YOU"), status);
    article.append(heading, user, result);
    $("thread").append(article);
    entry = { article, user, status, result };
    turns.set(message.id, entry);
  }
  entry.user.textContent = message.text;
  entry.status.textContent = message.status;
  return entry;
}
function event(record) {
  if (record.kind !== "agent_event") return;
  const e = record.data,
    data = e.data || {},
    t = turns.get(e.turn_id);
  if (!t) return;
  if (e.kind === "turn_finished") {
    t.result.textContent = data.text || data.error?.message || data.status;
    return;
  }
  if (!["text_delta", "item"].includes(e.kind)) return;
  const id = `${e.turn_id}:${data.item_id || "text"}`;
  let out = outputs.get(id);
  const message = e.kind === "text_delta" || data.kind === "message";
  if (!out) {
    const el = make(
      message ? "div" : "details",
      undefined,
      message ? "agent-output" : "tool",
    );
    if (!message) el.append(make("summary"), make("pre"));
    t.article.insertBefore(el, t.result);
    out = { el, text: "" };
    outputs.set(id, out);
  }
  if (message) {
    out.text =
      e.kind === "text_delta"
        ? out.text + (data.text || "")
        : data.text || out.text;
    out.el.textContent = out.text;
  } else {
    out.el.querySelector("summary").textContent =
      `${data.phase} · ${data.text || data.kind}`;
    out.el.querySelector("pre").textContent =
      data.output || JSON.stringify(data.details || {}, null, 2);
  }
}
function button(text, action, disabled = false) {
  const b = make("button", text);
  b.type = "button";
  b.disabled = disabled;
  b.onclick = action;
  return b;
}
function edit(message, mode) {
  editTarget = { message, mode };
  $("edit-title").textContent =
    mode === "edit" ? "Edit queued message" : "Confirm what happened";
  $("edit-label").textContent =
    mode === "edit" ? "Instruction" : "What did you check and observe?";
  $("edit-text").value = mode === "edit" ? message.text : "";
  $("edit-dialog").showModal();
}
function renderQuestions() {
  const version = JSON.stringify(state.requests);
  if (version === questionVersion) return;
  questionVersion = version;
  $("question-list").replaceChildren();
  $("questions").hidden = !Object.keys(state.requests).length;
  for (const request of Object.values(state.requests)) {
    const form = make("form"),
      fields = [];
    for (const q of request.questions) {
      const label = make("label", q.text),
        field = make(q.choices?.length ? "select" : "input");
      if (q.choices?.length) {
        if (!q.multiple) {
          const option = make("option", "Choose an answer");
          option.value = "";
          field.append(option);
        }
        for (const choice of q.choices) {
          const o = make("option", choice);
          o.value = choice;
          field.append(o);
        }
        field.multiple = !!q.multiple;
      } else field.type = q.secret ? "password" : "text";
      field.required = true;
      field.disabled = request.status !== "pending";
      label.append(field);
      form.append(label);
      fields.push({ q, field });
    }
    const send = make("button", "Send answer", "primary");
    send.disabled = request.status !== "pending";
    form.append(send);
    form.onsubmit = async (e) => {
      e.preventDefault();
      send.disabled = true;
      const answers = {};
      for (const { q, field } of fields)
        answers[q.id] = field.multiple
          ? Array.from(field.selectedOptions, (o) => o.value)
          : [field.value];
      const ok = await act("respond", {
        request_id: request.request_id,
        answers,
      });
      if (!ok) {
        questionVersion = "";
        await sync();
        notice(
          "Answer delivery may be uncertain. Check the current request state before responding again.",
        );
      }
    };
    $("question-list").append(form);
  }
}
function render() {
  if (!state) return;
  const queued = state.messages.filter((m) => m.status === "queued"),
    unknown = state.messages.filter((m) => m.status === "unknown");
  const label = state.closed
    ? "Ended"
    : state.paused
      ? "Paused"
      : state.active
        ? "Working"
        : state.connected
          ? "Ready"
          : "Starting";
  $("state").textContent = label;
  $("summary").textContent =
    `${state.messages.length} messages · ${queued.length} queued · ${label.toLowerCase()}`;
  $("execution-description").textContent = state.closed
    ? "Resources stopped. Conversation and workspace retained."
    : state.paused
      ? "Queued instructions are retained. Review them before continuing."
      : state.active
        ? "The agent is working. New messages will wait their turn."
        : state.connected
          ? "Ready for your next instruction."
          : "Waiting for the native agent connection.";
  $("cursor").textContent = `Event ${cursor}`;
  $("interrupt").disabled = !online || !state.active || state.closed;
  $("resume").disabled =
    !online ||
    !state.paused ||
    !!state.active ||
    !!unknown.length ||
    !state.connected ||
    state.closed;
  $("end").disabled = !online || state.closed || ending;
  $("send").disabled = !online || state.closed || ending || sending;
  $("send").textContent = pending ? "Retry same message ↑" : "Send message ↑";
  $("message").readOnly = !!pending;
  $("send-hint").textContent = pending
    ? "Awaiting acceptance. Retry keeps the same request ID."
    : state.paused
      ? "Your message will remain queued until you continue."
      : "New messages join the shared queue.";
  $("queue-count").textContent = queued.length;
  $("queue").replaceChildren();
  if (!queued.length)
    $("queue").append(make("p", "No queued messages.", "hint"));
  for (const m of queued) {
    const row = make("div", undefined, "queue-item"),
      actions = make("div", undefined, "queue-actions");
    row.append(make("p", m.text));
    actions.append(
      button("Edit", () => edit(m, "edit"), !online || state.closed),
      button(
        "Withdraw",
        () => act("withdraw", { message_id: m.id }),
        !online || state.closed,
      ),
    );
    row.append(actions);
    $("queue").append(row);
  }
  $("unknown-panel").hidden = !unknown.length;
  $("unknown").replaceChildren();
  for (const m of unknown) {
    const row = make("div", undefined, "queue-item");
    row.append(
      make("p", m.text),
      button(
        "Record what happened",
        () => edit(m, "resolve_unknown"),
        !online || state.closed,
      ),
    );
    $("unknown").append(row);
  }
  renderQuestions();
}
async function sync() {
  if (syncing || !token || ending) return;
  syncing = true;
  const current = generation;
  try {
    const snapshot = await api("/api/session");
    if (current !== generation) return;
    state = snapshot.state;
    const atBottom =
      $("thread").scrollHeight -
        $("thread").scrollTop -
        $("thread").clientHeight <
      70;
    for (const message of state.messages) turn(message);
    // Read only up to this snapshot; anything newer belongs to the next poll.
    while (cursor < snapshot.cursor) {
      const records = await api(`/api/events?after=${cursor}&limit=1000`);
      if (current !== generation) return;
      if (!records.length) break;
      for (const record of records) {
        if (record.seq > snapshot.cursor) break;
        event(record);
        cursor = record.seq;
      }
    }
    if (
      pending &&
      state.messages.some(
        (m) => m.client_id === clientId && m.request_id === pending.request_id,
      )
    ) {
      pending = null;
      saveDraft();
      $("message").value = "";
    }
    if (!online) notice("");
    connection(state.closed ? "Session ended" : "Connected", true);
    render();
    if (atBottom) $("thread").scrollTop = $("thread").scrollHeight;
    if (state.closed) clearInterval(timer);
  } catch (e) {
    if (current !== generation) return;
    connection("Disconnected · retrying", false);
    notice(e.message);
    render();
  } finally {
    syncing = false;
  }
}
$("connect-form").onsubmit = async (e) => {
  e.preventDefault();
  generation++;
  token = $("token").value.trim();
  try {
    await api("/api/session");
    $("token").value = "";
    $("login").hidden = true;
    $("workspace").hidden = false;
    $("disconnect").hidden = false;
    $("login-error").textContent = "";
    if (pending) $("message").value = pending.text;
    await sync();
    clearInterval(timer);
    timer = setInterval(sync, 800);
  } catch (e) {
    token = "";
    $("login-error").textContent = e.message;
  }
};
$("disconnect").onclick = () => {
  generation++;
  clearInterval(timer);
  token = "";
  connection("Not connected", false);
  $("workspace").hidden = true;
  $("login").hidden = false;
  $("disconnect").hidden = true;
};
$("compose").onsubmit = async (e) => {
  e.preventDefault();
  if (!online || !state || state.closed || sending) return;
  if (!pending) {
    pending = {
      request_id: crypto.randomUUID(),
      text: $("message").value.trim(),
    };
    if (!pending.text) {
      pending = null;
      return;
    }
    saveDraft();
  }
  const message = pending;
  sending = true;
  render();
  try {
    await command("enqueue", { client_id: clientId, ...message });
    pending = null;
    saveDraft();
    $("message").value = "";
    notice("");
  } catch (e) {
    notice(`${e.message}. The original request is kept for a safe retry.`);
  }
  sending = false;
  await sync();
  render();
};
$("interrupt").onclick = () => {
  if (state?.active) act("interrupt", { message_id: state.active });
};
$("resume").onclick = () => {
  if (state?.paused && confirm("Continue the queued messages in order?"))
    act("resume", { pause_id: state.pause_id });
};
$("end").onclick = async () => {
  if (
    !confirm(
      "End this session and stop its resources? Your conversation and workspace will be retained.",
    )
  )
    return;
  generation++;
  ending = true;
  render();
  try {
    const closed = (await api("/api/end", {})).result;
    clearInterval(timer);
    state = closed.state;
    for (const message of state.messages) turn(message);
    connection("Session ended", true);
    notice("Session ended. Your conversation and workspace are retained.");
  } catch (e) {
    notice(`${e.message}. Check state before assuming resources have stopped.`);
  } finally {
    ending = false;
    render();
  }
};
$("edit-form").onsubmit = async (e) => {
  e.preventDefault();
  const { message, mode } = editTarget;
  const text = $("edit-text").value.trim();
  if (!text) return;
  const params =
    mode === "edit"
      ? { message_id: message.id, text, revision: message.revision }
      : { message_id: message.id, note: text };
  if (await act(mode, params)) $("edit-dialog").close();
};
$("edit-cancel").onclick = () => $("edit-dialog").close();
