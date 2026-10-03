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
          ? "Directory listing limited. Enter a subdirectory path to browse further."
          : result.entries.length
            ? `${result.entries.length} entries. Select a file to inspect it.`
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
            "The image could not be decoded. You can download the file for inspection.";
        };
        get("file-content").append(image);
      } else if (file.kind === "text") {
        const pre = document.createElement("pre");
        pre.textContent = file.text;
        get("file-content").append(pre);
      }
      get("file-note").textContent =
        file.kind === "image"
          ? "Saved observation or image, not a live camera feed."
          : file.kind === "binary"
            ? "Binary file. Download to inspect it with an appropriate tool."
            : "Read-only preview. Code is not executed.";
      const bytes = Uint8Array.from(atob(file.data), (c) => c.charCodeAt(0));
      downloadURL = URL.createObjectURL(
        new Blob([bytes], { type: "application/octet-stream" }),
      );
      get("file-download").href = downloadURL;
      get("file-download").download = file.path.split("/").pop();
      get("file-dialog").showModal();
      status("File loaded. Refresh the directory to see new artifacts.");
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
