// The robot's cameras, live: a read-only client of the cameras API. Frames
// are taken only while someone watches (Watch pressed, tab visible); the
// service stops reading the cameras a little after the last request.
export function cameraView(api) {
  const get = (id) => document.getElementById(id);
  let enabled = false,
    watching = false,
    timer = null;
  const status = (text) => {
    get("camera-status").textContent = text;
  };
  async function names() {
    status("Finding the cameras…");
    const { cameras } = await api("/api/cameras");
    const select = get("camera-name");
    select.replaceChildren();
    for (const name of cameras) {
      const option = document.createElement("option");
      option.value = option.textContent = name;
      select.append(option);
    }
    select.disabled = !cameras.length;
    return cameras;
  }
  async function poll() {
    timer = null;
    if (!enabled || !watching) return;
    if (!document.hidden && get("camera-name").value) {
      try {
        const frame = await api(
          `/api/cameras/frame?name=${encodeURIComponent(get("camera-name").value)}`,
        );
        if (!watching) return;
        if (frame.waiting) status("Waiting for the first frame…");
        else {
          get("camera-frame").src = `data:${frame.mime};base64,${frame.data}`;
          get("camera-frame").hidden = false;
          status(`Live · ${frame.age_s < 1 ? "now" : `${Math.round(frame.age_s)} s ago`}`);
        }
      } catch (e) {
        status(e.message);
      }
    }
    timer = setTimeout(poll, 300);
  }
  async function start() {
    watching = true;
    get("camera-watch").textContent = "Stop";
    try {
      if (!get("camera-name").options.length && !(await names()).length) {
        status("No cameras are configured for this robot.");
        stop();
        return;
      }
    } catch (e) {
      status(e.message);
      stop();
      return;
    }
    poll();
  }
  function stop() {
    watching = false;
    clearTimeout(timer);
    timer = null;
    get("camera-watch").textContent = "Watch";
    if (enabled) status("Press Watch to see the robot's cameras live.");
  }
  get("camera-watch").onclick = () => (watching ? stop() : start());
  get("camera-name").onchange = () => {
    get("camera-frame").hidden = true;
  };
  return {
    connect() {
      enabled = true;
      get("camera-watch").disabled = false;
      stop();
    },
    disconnect() {
      stop();
      enabled = false;
      get("camera-watch").disabled = true;
      get("camera-frame").hidden = true;
      status("");
    },
  };
}
