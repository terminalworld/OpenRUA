# 40. Working patterns

Cross-cutting habits that save turns and time.

## P1. Act → verify, every time

The clock only moves with activity (10-machine.md §4), so verification
is sensor re-reads, never waits:

1. command (30-action) → 2. re-read the relevant sensor (20-perception)
→ 3. compare with what you intended → 4. correct or continue.

Motion: compare the joint state against your target. Manipulation:
finger gap + a fresh camera frame. World changes: before/after frames
from the same camera.

## P2. Coarse-to-fine motion

Large repositioning with one trajectory move (fast, accurate over big
spans), final approach with short servo bursts (small, sensed,
correctable). Avoid many small trajectory hops and long blind servo
streams alike.

## P3. Approach–grasp–verify (generic pick pattern)

1. Snapshot cameras; locate the target object in the image (pixels +
   depth + intrinsics → world position, 20-perception B3).
2. Move above it (IK → trajectory), open the gripper.
3. Descend with short servo bursts, watching a wrist camera if present.
4. Close the gripper; verify the grasp: finger gap above `closed_m`,
   wrench change on contact.
5. Lift and confirm visually (fresh frame) before moving on.

Placing is the mirror: position above the destination, descend, open,
retreat, verify with a fresh frame.

## P4. Troubleshooting table

| Symptom | Likely cause | Fix |
|---|---|---|
| trajectory result `error_code != 0` | duration too short / target beyond a joint limit | slow down; check limits (machine.yaml) |
| arm lags the servo stream | per-message motion is tiny | keep streaming in a loop; verify between bursts |
| gripper "closed" but nothing held | closed on air | reopen, re-approach lower/centered, use the wrist camera |
| image looks stale | pulled an old frame | snapshot again after the motion completed |
| a port seems missing | wrong name assumed | `ros2 topic list` / `ros2 action list`, then `ros2 topic info` |

## Session-runtime facts

- NEVER end your turn while a motion or background task is pending: an
  ended turn ends the session; nothing resumes it, and the paused
  world means what you were "waiting for" will never arrive. Poll to
  completion first.
- Long commands: run them detached writing to a file with `python3 -u`
  (pipes buffer output invisibly), then poll the file with short
  commands. Build service/action clients once and reuse them; every
  rebuild costs seconds.
