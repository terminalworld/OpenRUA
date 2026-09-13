#!/usr/bin/env python3
"""Pick each mug by its rim and place it on its plate, verifying each step.
Usage: python3 -u task.py <white|yellow|both>
"""
import sys, math
import numpy as np, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
from robot import Robot, down_quat, ik_down_quat

HOME = [0.0, -0.16, 0.0, -2.44, 0.0, 2.23, 0.785]

R_RIM = 0.045          # rim radius (wall midpoint), measured ~9 cm dia
TABLE = 0.425
PLATE_TOP = 0.443
GRASP_DEPTH = 0.03     # fingertips this far below the rim
Z_TRAVEL = 0.72

# measured from birdview (world xy, rim top z)
MUGS = {
    "white":  dict(c=(-0.084, -0.186), ztop=0.549, yaw=+math.pi / 2, plate=(0.011, 0.290)),   # -> right plate
    "yellow": dict(c=(-0.064, 0.096),  ztop=0.542, yaw=-math.pi / 2, plate=(-0.011, -0.290)),  # -> left plate
}


def snap(r, cam, out):
    got = []
    sub = r.node.create_subscription(Image, f"/{cam}/color/image_raw", got.append, 1)
    while not got:
        rclpy.spin_once(r.node, timeout_sec=0.5)
    r.node.destroy_subscription(sub)
    cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got[0], "bgr8"))
    print(f"  snapshot {out}")


def goto(r, xyz, quat, seconds, seed=None, label=""):
    q = r.ik_world(xyz, quat, seed=seed)
    if q is None:
        raise SystemExit(f"IK failed for {label} {np.round(xyz,3)}")
    code, err = r.move_q(q, seconds)
    tcp, _ = r.tcp_world()
    d = np.linalg.norm(tcp - np.asarray(xyz))
    print(f"  {label}: code={code} qerr={err:.4f} tcp={tcp.round(3)} target={np.round(xyz,3)} |d|={d*1000:.1f}mm")
    if d > 0.01:
        raise SystemExit(f"pose error too large at {label}")
    return q


def do_mug(r, name):
    m = MUGS[name]
    cx, cy = m["c"]
    gx, gy = cx - R_RIM, cy               # grasp the rim on the -x (robot) side
    zg = m["ztop"] - GRASP_DEPTH
    h = m["ztop"] - TABLE                 # mug height
    print(f"== {name}: grasp at ({gx:.3f},{gy:.3f},{zg:.3f})")

    # fingers along world x; +-pi/2 are equivalent, keep the least contorted
    best = None
    for yaw in (math.pi / 2, -math.pi / 2):
        qq = r.ik_world([gx, gy, Z_TRAVEL], ik_down_quat(yaw), seed=HOME)
        if qq is None:
            continue
        cost = sum(abs(a - b) for a, b in zip(qq[:6], HOME[:6]))
        print(f"  yaw {yaw:+.2f}: q={[round(v,2) for v in qq]} cost={cost:.2f}")
        if best is None or cost < best[0]:
            best = (cost, yaw, qq)
    quat = ik_down_quat(best[1])
    print("open gripper:", r.gripper(0.04))
    q = goto(r, [gx, gy, Z_TRAVEL], quat, 4.0, seed=best[2], label="pre-grasp high")
    q = goto(r, [gx, gy, m["ztop"] + 0.04], quat, 2.5, seed=q, label="above rim")
    snap(r, "robot0_eye_in_hand", f"{name}_eih_above.png")
    if "check" in sys.argv:
        print("CHECK STOP"); return
    q = goto(r, [gx, gy, zg], quat, 2.0, seed=q, label="grasp height")
    ok, stalled, gap = r.gripper(0.0)
    print(f"  close: reached={ok} stalled={stalled} gap={gap*1000:.1f}mm")
    if gap < 0.002:
        raise SystemExit("closed on air")
    q = goto(r, [gx, gy, Z_TRAVEL], quat, 3.0, seed=q, label="lift")
    gap = r.finger_gap()
    print(f"  gap after lift {gap*1000:.1f}mm")
    if gap < 0.002:
        raise SystemExit("lost the mug on lift")
    snap(r, "agentview", f"{name}_lifted.png")

    px, py = m["plate"]
    tx, ty = px - 0.040, py               # mug centre lands on the plate centre (0.040: measured from white run)
    z_place = PLATE_TOP + 0.006 + (h - GRASP_DEPTH)   # mug bottom ~6 mm above plate
    q = goto(r, [tx, ty, Z_TRAVEL], quat, 4.0, seed=q, label="above plate")
    q = goto(r, [tx, ty, z_place + 0.05], quat, 2.5, seed=q, label="lowering")
    q = goto(r, [tx, ty, z_place], quat, 2.0, seed=q, label="place height")
    print("  open:", r.gripper(0.04))
    q = goto(r, [tx, ty, Z_TRAVEL], quat, 3.0, seed=q, label="retreat")
    snap(r, "agentview", f"{name}_placed.png")


def main():
    which = sys.argv[1]
    r = Robot()
    try:
        for name in (["white", "yellow"] if which == "both" else [which]):
            do_mug(r, name)
        # park clear of the birdview
        goto(r, [-0.35, 0.0, 0.95], ik_down_quat(0), 4.0, seed=HOME, label="park")
    finally:
        r.shutdown()
    print("DONE")


if __name__ == "__main__":
    main()
