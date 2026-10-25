import { workspaceBrowser } from "/workspace.js";
import { cameraView } from "/cameras.js";
import {
  permissionChoices,
  SettingChanges,
  agentCommands,
  commandsFor,
  notice as eventNotice,
  parseCommand,
  status,
  toolSummary,
  typedAnswer,
  displayStatus,
  robotPrompt,
} from "/session.mjs";
const $ = (id) => document.getElementById(id);
const make = (tag, text, className) => {
  const e = document.createElement(tag);
  if (text !== undefined) e.textContent = text;
  if (className) e.className = className;
  return e;
};
const key = "openrua-draft";
const settingChanges = new SettingChanges(notice);
let replayThrough = null;
let saved;
try {
  saved = JSON.parse(sessionStorage.getItem(key));
} catch {
  saved = null;
}
let clientId = saved?.clientId || crypto.randomUUID(),
  pending = saved?.pending || null,
  pendingRobot = saved?.pendingRobot || null;
let token = "",
  state = null,
  cursor = 0,
  timer = null,
  syncing = false,
  online = false,
  ending = false,
  sending = false,
  generation = 0,
  lastEvent = 0;
let turns = new Map(),
  outputs = new Map(),
  questionVersion = "",
  editTarget = null;
function saveDraft() {
  sessionStorage.setItem(key, JSON.stringify({ clientId, pending, pendingRobot }));
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
  if (!response.ok) {
    const error = new Error(value.error || `Request failed (${response.status})`);
    error.status = response.status;
    throw error;
  }
  return value;
}
const files = workspaceBrowser(api);
const cameras = cameraView(api);
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
    notice(`${e.message}\nCheck the session status before trying again.`);
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
  // A turn the agent started by itself has no instruction of its own.
  entry.user.textContent = message.origin === "agent" ? "(the agent continued working on its own)" : message.text;
  entry.status.textContent =
    displayStatus(message.status);
  return entry;
}
function event(record) {
  const handled = settingChanges.receive(record, state);
  // What an event says to the person, as every client says it.
  const said = eventNotice(record);
  const oldSetting = record.seq <= replayThrough && record.kind === 'agent_event' && record.data?.kind === 'setting_rejected';
  if (said && !handled && !oldSetting) notice(said);
  if (record.kind !== "agent_event") return;
  lastEvent = Date.now();
  const e = record.data,
    data = e.data || {},
    t = turns.get(e.turn_id);
  if (!t) return;
  if (e.kind === "turn_finished") {
    // A completed turn's closing text repeats what the agent already said;
    // it shows only when the turn showed nothing else, as in the terminal.
    const shown = [...outputs.keys()].some((id) => id.startsWith(`${e.turn_id}:`));
    t.result.textContent =
      data.status === "completed"
        ? shown
          ? ""
          : data.text || ""
        : data.error?.message || data.status;
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
    if (data.details !== undefined) out.given = data.details;
    const given = toolSummary(out.given);
    out.el.querySelector("summary").textContent =
      `${data.phase} · ${data.text || data.kind}${given ? `(${given})` : ""}`;
    if (data.details !== undefined) out.details = JSON.stringify(data.details, null, 2);
    if (data.output !== undefined) out.output = String(data.output ?? "");
    const sections = [];
    if (out.details !== undefined) sections.push(`Details\n${out.details}`);
    if (out.output !== undefined) sections.push(`Output\n${out.output}`);
    out.el.querySelector("pre").textContent = sections.join("\n\n");
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
      let other;
      if (q.choices?.length && q.allow_other) {
        other = make("input");
        other.type = q.secret ? "password" : "text";
        other.disabled = field.disabled;
        const otherLabel = make("label", `${q.text} (another answer)`);
        otherLabel.append(other);
        form.append(otherLabel);
        other.oninput = () => {
          field.required = !other.value.trim();
          if (!q.multiple && other.value.trim()) field.value = "";
        };
        field.onchange = () => {
          if (!q.multiple && field.value) other.value = "";
          field.required = !other.value.trim();
        };
      }
      fields.push({ q, field, other });
    }
    const send = make("button", "Send answer", "primary");
    send.disabled = request.status !== "pending";
    form.append(send);
    form.onsubmit = async (e) => {
      e.preventDefault();
      send.disabled = true;
      const answers = {};
      for (const { q, field, other } of fields) {
        const values = field.multiple
          ? Array.from(field.selectedOptions, (o) => o.value)
          : field.value ? [field.value] : [];
        if (other?.value.trim()) values.push(other.value.trim());
        answers[q.id] = [...new Set(values)];
      }
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
          ? (state.robot && state.robot.status !== 'ready' ? 'Ready to chat' : 'Ready')
          : "Starting";
  $("state").textContent = label;
  $("summary").textContent =
    `${state.messages.length} message${state.messages.length === 1 ? '' : 's'} · ${queued.length} waiting · ${label.toLowerCase()}`;
  $("execution-description").textContent = state.closed
    ? "This session has stopped. Your files and chat history are saved."
    : state.robot && state.robot.status !== 'ready'
      ? `Robot ${state.robot.status}: ${state.robot.reason}. You can keep chatting to check the problem and fix it. Earlier instructions are saved and will wait for your review before they can be sent.`
      : state.paused
      ? `Paused${state.pause_reason ? `: ${state.pause_reason}` : ""}. Your waiting messages are saved. Check them before continuing.`
      : state.active
        ? "The agent is working. New messages join its current response when supported. Otherwise, they wait."
        : state.connected
          ? "Ready for your next instruction."
          : "Connecting to the agent…";
  // The same notes the terminal's status line shows.
  const { notes } = status(state);
  if (state.active && lastEvent && Date.now() - lastEvent > 60000)
    notes.splice(1, 0, {
      tone: "warn",
      text: `no output for ${Math.round((Date.now() - lastEvent) / 1000)}s`,
    });
  $("notes").replaceChildren(
    ...notes.map((n) => make("span", n.text, `tone-${n.tone}`)),
  );
  // The agent: its permissions and model, and its own commands.
  const settings = state.settings || {};
  if (document.activeElement !== $("permissions")) {
    $("permissions").replaceChildren(...permissionChoices(state).map(choice => {
      const option = make('option', choice.label);
      option.value = choice.value;
      option.title = choice.description || '';
      return option;
    }));
    $("permissions").value = settings.permissions || "";
  }
    $("permissions").disabled = !online || state.closed || !permissionChoices(state).length;
  $("model").disabled = !online || state.closed;
  $("agent-model").textContent = settings.model || "";
  const theirs = agentCommands(state);
  $("agent-commands").textContent = theirs.length
    ? `Agent commands: ${theirs.map((c) => "/" + c.name).join(" ")}`
    : "";
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
    ? "We have not confirmed that this message arrived. Retry keeps the same request ID, so the message is not submitted twice."
    : state.paused
      ? "Your message will wait until you choose Continue."
      : "While the agent works, new messages join its current response when supported. Otherwise, they wait.";
  $("queue-count").textContent = queued.length;
  $("queue").replaceChildren();
  if (!queued.length)
    $("queue").append(make("p", "No messages are waiting.", "hint"));
  for (const m of queued) {
    const row = make("div", undefined, "queue-item"),
      actions = make("div", undefined, "queue-actions");
    row.append(make("p", m.text));
    if (m.held_reason) {
      row.append(make('p', `Not sent yet: ${m.held_reason}`, 'hint'));
      actions.append(button('Send this instruction now', () => {
        if (confirm('Send this instruction now? Check the robot and the result of earlier actions first.'))
          return act('release', {message_id: m.id, revision: m.revision});
      }, !online || state.closed));
    }
    actions.append(
      button("Edit", () => edit(m, "edit"), !online || state.closed),
      button(
        "Remove",
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
    if (replayThrough === null) replayThrough = snapshot.cursor;
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
  settingChanges.clear();
  replayThrough = null;
  token = $("token").value.trim();
  try {
    await api("/api/session");
    $("token").value = "";
    $("login").hidden = true;
    $("workspace").hidden = false;
    $("disconnect").hidden = false;
    $("login-error").textContent = "";
    if (pending) $("message").value = pending.text;
    files.connect();
    cameras.connect();
    await sync();
    clearInterval(timer);
    timer = setInterval(sync, 800);
  } catch (e) {
    token = "";
    $("login-error").textContent = e.message;
  }
};
$("disconnect").onclick = () => {
  files.disconnect();
  cameras.disconnect();
  generation++;
  clearInterval(timer);
  token = "";
  connection("Not connected", false);
  $("workspace").hidden = true;
  $("login").hidden = false;
  $("disconnect").hidden = true;
};
async function configure(setting, value) {
  if (state?.settings?.[setting] === value) return true;
  const change = settingChanges.begin({[setting]: value});
  const accepted = await act("configure", { setting, value });
  if (!accepted) { settingChanges.cancel(change); return false; }
  try { return settingChanges.complete(change, state); }
  catch (error) { notice(error.message); return false; }
}
// One handler for each command the page offers (commandsFor("browser"));
// the agent's own commands go to it as a message, as in its terminal.
export const handlers = {
  robot: async ([operation = 'status']) => {
    if (!['status', 'logs', 'start', 'stop', 'restart', 'build', 'retry'].includes(operation))
      throw new Error('Use /robot status, logs, start, stop, restart, build or retry.');
    if (operation === 'retry' && !pendingRobot) throw new Error('There is no robot request waiting to be retried.');
    let body = operation === 'retry' ? pendingRobot : {operation};
    if (!['status', 'logs', 'retry'].includes(operation)) {
      if (pendingRobot) throw new Error('We have not confirmed the last robot request. Check /robot status or use /robot retry.');
      if (!confirm(robotPrompt(operation))) return;
      body.request_id = crypto.randomUUID();
      pendingRobot = body;
      saveDraft();
    }
    try {
      const value = await api('/api/actions/robot', body);
      if (body.request_id) { pendingRobot = null; saveDraft(); }
      notice(JSON.stringify(value.result, null, 2));
    } catch (error) {
      if (body.request_id && error.status >= 400 && error.status < 500) {
        pendingRobot = null; saveDraft();
      }
      throw new Error(error.message + (pendingRobot ? ` We have not confirmed request ${pendingRobot.request_id}. Use /robot retry to retry it with the same request ID.` : ''));
    }
  },
  help: () => {
    const theirs = agentCommands(state);
    notice(
      commandsFor("browser")
        .map((c) => `/${c.name}: ${c.description}`)
        .concat(
          theirs.length
            ? ["", "The agent's own:", ...theirs.map((c) => `/${c.name}: ${c.description}`)]
            : [],
        )
        .join("\n"),
    );
  },
  files: () => {
    $("files-path").scrollIntoView({ block: "center" });
    $("files-path").focus();
  },
  queue: () => $("queue").scrollIntoView({ block: "center" }),
  watch: () => {
    $("camera-watch").scrollIntoView({ block: "center" });
    if ($("camera-watch").textContent === "Watch") $("camera-watch").click();
  },
  questions: () => {
    if ($("questions").hidden) throw new Error("No pending questions.");
    $("questions").scrollIntoView({ block: "center" });
  },
  end: () => $("end").click(),
  interrupt: async () => {
    if (!state?.active) { notice('No agent response to interrupt.'); return; }
    await act("interrupt", { message_id: state.active });
  },
  continue: () => {
    if (!state?.paused) { notice('Nothing to continue.'); return; }
    $("resume").click();
  },
  permissions: async ([value]) => {
    if (!value) {
      $("permissions").focus();
      return;
    }
    await configure("permissions", value);
  },
  model: async ([value]) => {
    if (!value) {
      notice(`The model: ${state?.settings?.model ?? "unknown"}. Change it with /model <name>.`);
      return;
    }
    await configure("model", value);
  },
};
for (const p of permissionChoices(null)) {
  const option = make("option", p.label);
  option.value = p.value;
  $("permissions").append(option);
}
$("permissions").onchange = () => configure("permissions", $("permissions").value);
$("model-form").onsubmit = async (e) => {
  e.preventDefault();
  const value = $("model").value.trim();
  if (value && (await configure("model", value))) $("model").value = "";
};
$("compose").onsubmit = async (e) => {
  e.preventDefault();
  if (!online || !state || state.closed || sending) return;
  const typed = pending ? null : parseCommand($("message").value, state);
  if (typed && typed.kind !== "agent") {
    $("message").value = ""; // as the terminal does, whatever the command says
    try {
      const handler = typed.kind === "ours" ? handlers[typed.name] : undefined;
      if (!handler)
        throw new Error(
          typed.kind === "ours"
            ? `/${typed.name} is a terminal command.`
            : "Unknown command. Use /help.",
        );
      await handler(typed.args);
    } catch (err) {
      notice(err.message);
    }
    return;
  }
  // While the agent waits on a question, a typed line answers it, as in
  // the terminal.
  const answer = pending ? null : typedAnswer(state, $("message").value);
  if (answer?.kind === "choose") {
    handlers.questions();
    notice("Answer this question in its form.");
    return;
  }
  if (answer) {
    if (await act("respond", { request_id: answer.request_id, answers: answer.answers }))
      $("message").value = "";
    return;
  }
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
    notice(`${e.message}\nThe original request is kept for a safe retry.`);
  }
  sending = false;
  await sync();
  render();
};
$("interrupt").onclick = () => {
  if (state?.active) act("interrupt", { message_id: state.active });
};
$("resume").onclick = () => {
  if (state?.paused && confirm("Send the waiting messages in order?"))
    act("resume", { pause_id: state.pause_id });
};
$("end").onclick = async () => {
  if (
    !confirm(
      "Stop this session and free its resources? Your files and chat history are saved.",
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
    files.disconnect();
    cameras.disconnect();
    connection("Session ended", true);
    notice("This session has stopped. Your files and chat history are saved.");
  } catch (e) {
    notice(`${e.message}\nCheck the session status to confirm it has stopped.`);
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
