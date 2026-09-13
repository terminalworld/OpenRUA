# tools/: minimal utilities

Two executable Python scripts, one per real gap where the `ros2` CLI has
no reasonable one-liner. Everything else is done with the CLI directly
(see docs). Plain code; read them, run them, adapt them.

| Tool | Usage | Fills which gap |
|---|---|---|
| `perception/cam_snap.py` | `./tools/perception/cam_snap.py <camera> [out.png]` | the CLI cannot save an image topic to a file (color → PNG; depth topics → normalized PNG + raw-meters `.npy`) |
| `action/fjt_send.py` | `./tools/action/fjt_send.py p1,...,pN <seconds>` | trajectory goals are impractical to write as CLI YAML (joints/port resolved from machine.yaml) |
