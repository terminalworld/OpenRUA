// A read-only client of the workspace API. Browsing never submits agent tasks.
export function workspaceBrowser(api) {
  const get = (id) => document.getElementById(id);
  let directory = "",
    revision = 0,
    enabled = false,
    downloadURL = null;
  const clearDownload = () => {
    if (downloadURL) URL.revokeObjectURL(downloadURL);
    downloadURL = null;
  };
  const status = (text) => {
    get("files-status").textContent = text;
  };
  async function list(path = directory) {
    if (!enabled) return;
    const current = ++revision;
    get("files-path").value = path;
    status("Reading workspace…");
    try {
      const result = await api(
        `/api/workspace/list?path=${encodeURIComponent(path)}`,
      );
      if (current !== revision) return;
      directory = result.path;
      get("files-list").replaceChildren();
      const row = (label, action, disabled = false) => {
        const button = document.createElement("button");
        button.type = "button";
        button.textContent = label;
        button.disabled = disabled;
        button.onclick = action;
        get("files-list").append(button);
      };
      if (directory)
        row("../", () => list(directory.split("/").slice(0, -1).join("/")));
      for (const entry of result.entries) {
        const path = [directory, entry.name].filter(Boolean).join("/");
        row(
          entry.name + (entry.kind === "directory" ? "/" : ""),
          () => (entry.kind === "directory" ? list(path) : read(path)),
          entry.kind === "unsupported",
        );
      }
      status(
        result.truncated
          ? "There are more files than this list can show. Open a subdirectory to see more."
          : result.entries.length
            ? `${result.entries.length} item${result.entries.length === 1 ? '' : 's'}. Select a file to open it.`
            : "This directory is empty.",
      );
    } catch (error) {
      if (current === revision) status(error.message);
    }
  }
  async function read(path) {
    if (!enabled) return;
    const current = ++revision;
    status("Reading file…");
    try {
      const file = await api(
        `/api/workspace/read?path=${encodeURIComponent(path)}`,
      );
      if (current !== revision) return;
      clearDownload();
      get("file-title").textContent = file.path;
      get("file-meta").textContent =
        `${file.size.toLocaleString()} bytes · Modified ${new Date(file.modified).toLocaleString()}`;
      get("file-content").replaceChildren();
      if (file.kind === "image") {
        const image = document.createElement("img");
        image.alt = `Saved workspace image: ${file.path}`;
        image.src = `data:${file.mime};base64,${file.data}`;
        image.onerror = () => {
          get("file-note").textContent =
            "This image could not be displayed. Download the file to open it.";
        };
        get("file-content").append(image);
      } else if (file.kind === "text") {
        const pre = document.createElement("pre");
        pre.textContent = file.text;
        get("file-content").append(pre);
      }
      get("file-note").textContent =
        file.kind === "image"
          ? "This is a saved image. Use Cameras to watch live video."
          : file.kind === "binary"
            ? "This file cannot be previewed here. Download it to open it."
            : "This is a preview. Opening a file here does not run it.";
      const bytes = Uint8Array.from(atob(file.data), (c) => c.charCodeAt(0));
      downloadURL = URL.createObjectURL(
        new Blob([bytes], { type: "application/octet-stream" }),
      );
      get("file-download").href = downloadURL;
      get("file-download").download = file.path.split("/").pop();
      get("file-dialog").showModal();
      status("File opened. Refresh the directory to see new files.");
    } catch (error) {
      if (current === revision) status(error.message);
    }
  }
  get("files-form").onsubmit = (event) => {
    event.preventDefault();
    list(get("files-path").value);
  };
  get("file-close").onclick = () => get("file-dialog").close();
  get("file-dialog").onclose = clearDownload;
  return {
    connect() {
      enabled = true;
      list(directory);
    },
    disconnect() {
      enabled = false;
      revision++;
      get("file-dialog").close();
      get("file-content").replaceChildren();
      get("files-list").replaceChildren();
      status("Reconnect to browse saved files.");
      clearDownload();
    },
  };
}
