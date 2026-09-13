# Scene notes (world frame, metres)

- Robot: Franka Panda, base panda_link0 at world (-0.66, 0, 0.912). Hand frame
  points down at start; fingers slide along hand-y. TCP = hand + 0.1034 hand-z.
  Hand housing ~0.2 (finger axis) x 0.06 x 0.06 tall.
- Table top z = 0.90.
- Black bowl (after settling): centre (-0.137, 0.025), rim radius 0.0525,
  rim z 0.955, bottom on table (height ~0.055). Too wide for the 0.08 gripper
  -> grasp the rim (one finger inside, one outside), fingers along world x.
- Cabinet body: x -0.215..0.025, y -0.41..-0.243, top z 1.127. Upper drawer
  fronts at y=-0.222; their handles protrude to y=-0.191, x -0.133..-0.044,
  z 1.01-1.02 and 1.084-1.099.
- Bottom drawer OPEN (slides along y, front faces +y): floor z 0.924,
  interior x -0.214..0.017, front wall inner face y~-0.088, outer ~-0.07,
  wall tops z 0.98. Drawer handle bar y -0.045..-0.035, x -0.131..-0.043,
  z 0.945-0.955. Closed position: front face at y=-0.222 (travel ~0.15).
- Bottle at (0.05, -0.08), radius ~0.03, z to 1.12 - keep x<0.0 near it.
- Cameras: agentview (+x side, looking -x), frontview, sideview (+y side),
  birdview (top), robot0_eye_in_hand (on hand, +0.05 hand-x offset).
