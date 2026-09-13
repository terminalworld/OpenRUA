#!/usr/bin/env python3
"""Staged task: put the yellow/white mug in the microwave, close the door.
usage: python3 tools/task.py <stage>
stages: pregrasp | grasp | lift | preinsert | insert | release | retract | park

Grasp geometry (from IK reach probes + 2D footprint check, see insert_geom.py):
hand approach a = yaw -10 deg from +y, pitched 40 deg down; fingers close along x on the
mug handle. Pitch raises the wrist (reach), small yaw keeps the hand body clear of the door.
"""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from planner import Planner, cylinder
from robot import mat_to_quat, TCP
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

YAW, PITCH = np.deg2rad(-15.0), np.deg2rad(35.0)


def R_of(yaw, pitch):
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    a = np.array([-sy * cp, cy * cp, -sp])            # hand z (approach)
    xh = -np.array([-sy * sp, cy * sp, cp])           # hand x (~ -world z)
    yh = np.cross(a, xh)                              # hand y (finger axis, ~ -world x)
    return np.stack([xh, yh, a], 1)


R_G = R_of(YAW, PITCH)
Q_G = mat_to_quat(R_G)
A_XY = R_G[:2, 2]                                     # horizontal approach direction
GRIP_TO_CENTRE = 0.0785                               # handle grip point -> mug axis

MUG_XY = np.array([-0.0006, 0.1851])                  # mug axis after relocation (agentview fit)
HANDLE = np.array([MUG_XY[0], MUG_XY[1] - 0.0785, 0.958])   # grip point on the handle bar
PREGRASP = HANDLE - np.array([*(0.12 * A_XY / np.linalg.norm(A_XY)), 0.0])
LIFT_DZ = 0.056                                       # mug bottom ends ~1.2 cm above cavity floor
MUG_ENTRY = np.array([-0.122, -0.385])                # mug axis just outside the opening
MUG_FINAL = np.array([-0.122, -0.225])                # mug axis inside (handle end ~2.5 cm behind face)
Z_CARRY = HANDLE[2] + LIFT_DZ


def tcp_for_mug(mug_xy):
    return np.array([*(np.asarray(mug_xy) - GRIP_TO_CENTRE * A_XY), Z_CARRY])


PREINSERT = tcp_for_mug(MUG_ENTRY)
INSERT = tcp_for_mug(MUG_FINAL)
RETRACT = np.array([PREINSERT[0], -0.44, Z_CARRY])
HOME_Q = [-0.002, 0.244, 0.003, -1.725, -0.001, 1.961, 0.778]

# top-down relocation grasp (the mug stands right behind the microwave; no room for a frontal grasp)
R_TD = np.array([[0, -1, 0],
                 [-1, 0, 0],
                 [0, 0, -1]], float)                  # hand x -> -y, hand y -> -x (fingers along x), hand z down
Q_TD = mat_to_quat(R_TD)
TD_GRASP = np.array([-0.001, -0.053, 0.970])          # pads on the handle bar (z 0.96-0.98)
DRAG_DY = 0.16                                        # slide the mug to y ~ 0.19


def servo_tcp(p, target, quat, iters=4, tol=0.004, seconds=3.0):
    """Closed-loop TCP positioning: re-command target + measured error until |err| < tol.
    Compensates the steady-state sag of the position controller at stretched poses."""
    target = np.asarray(target, float)
    cmd = target.copy()
    for i in range(iters):
        cur, _, _ = p.tcp()
        err = target - cur
        print(f"servo {i}: tcp {np.round(cur, 4)} err {np.round(err, 4)} |err| {np.linalg.norm(err):.4f}", flush=True)
        if np.linalg.norm(err) < tol:
            return True
        cmd = cmd + err
        q = p.ik(cmd, quat, seed=p.arm_q(), attempts=6)
        if q is None:
            print("servo: ik failed for", np.round(cmd, 4), flush=True)
            return False
        p.move(q, seconds)
    cur, _, _ = p.tcp()
    print(f"servo end: tcp {np.round(cur, 4)} err {np.round(target - cur, 4)}", flush=True)
    return np.linalg.norm(target - cur) < tol


def grasp_from_measure(centre, bar):
    """Handle grasp aligned with the measured handle: returns (tcp, R, quat)."""
    centre, bar = np.asarray(centre, float), np.asarray(bar, float)
    d = centre - bar; d /= np.linalg.norm(d)                 # horizontal approach direction (hand -> mug)
    yaw = np.arctan2(-d[0], d[1])
    R = R_of(yaw, PITCH)
    tcp = np.array([*(centre - GRIP_TO_CENTRE * d), HANDLE[2]])
    print(f"measured grasp: yaw {np.rad2deg(yaw):.1f} deg, tcp {np.round(tcp, 4)}", flush=True)
    return tcp, R, mat_to_quat(R)


MEAS_CENTRE = (-0.0308, 0.2342)                       # latest agentview fit
MEAS_BAR = (-0.047, 0.157)                            # bar centre from centre + 0.0785*dir
RELOC_CENTRE = np.array([-0.03, 0.24])                # where to set the mug down for the pitched grasp
TD_Z = 0.965                                          # TCP height for the top-down handle grasp


def Rz(t):
    c, s_ = np.cos(t), np.sin(t)
    return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])


def td_frames(centre, bar):
    """Top-down grasp of the handle bar with the finger axis normal to the handle plane,
    plus the place pose that leaves the handle pointing along -A_XY (insertion approach)."""
    centre, bar = np.asarray(centre, float), np.asarray(bar, float)
    d_g = bar - centre; d_g /= np.linalg.norm(d_g)               # handle direction now
    d_p = -A_XY / np.linalg.norm(A_XY)                             # handle direction wanted
    ang_g = np.arctan2(d_g[1], d_g[0]); ang_p = np.arctan2(d_p[1], d_p[0])
    R_grasp = Rz(ang_g + np.pi / 2) @ R_TD                          # R_TD closes along x for a handle along -y
    R_place = Rz(ang_p + np.pi / 2) @ R_TD
    tcp_g = np.array([*(centre + GRIP_TO_CENTRE * d_g), TD_Z])
    tcp_p = np.array([*(RELOC_CENTRE + GRIP_TO_CENTRE * d_p), TD_Z + 0.004])
    print(f"td grasp: handle dir {np.round(d_g,3)} -> {np.round(d_p,3)}, rotate {np.degrees(ang_p-ang_g):.1f} deg; "
          f"tcp_g {np.round(tcp_g,4)} tcp_p {np.round(tcp_p,4)}", flush=True)
    return tcp_g, R_grasp, tcp_p, R_place, d_g


def attach_mug(p, R=None, c_world_rel=None):
    """Attach the held mug as a vertical cylinder in the hand frame.
    R: hand rotation at grasp; c_world_rel: mug centre relative to the TCP (world axes)."""
    if R is None:
        R = R_G
    if c_world_rel is None:
        c_world_rel = np.array([*(GRIP_TO_CENTRE * A_XY), 0.9525 - HANDLE[2]])
    aco = AttachedCollisionObject()
    aco.link_name = "panda_hand"
    aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
    co = CollisionObject(); co.header.frame_id = "panda_hand"; co.id = "held_mug"
    prim = SolidPrimitive(); prim.type = SolidPrimitive.CYLINDER; prim.dimensions = [0.105, 0.05]
    pose = Pose()
    c_hand = R.T @ np.asarray(c_world_rel, float) + np.array([0, 0, TCP])
    pose.position.x, pose.position.y, pose.position.z = map(float, c_hand)
    zh = R.T @ np.array([0, 0, 1.0])                  # world z in hand frame = cylinder axis
    v = np.cross([0, 0, 1.0], zh); s = np.linalg.norm(v); c = np.dot([0, 0, 1.0], zh)
    if s < 1e-6:
        q = [0, 0, 0, 1] if c > 0 else [1, 0, 0, 0]
    else:
        ax = v / s; ang = np.arctan2(s, c)
        q = [*(ax * np.sin(ang / 2)), np.cos(ang / 2)]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    co.primitives.append(prim); co.primitive_poses.append(pose); co.operation = CollisionObject.ADD
    aco.object = co
    return p.apply_scene([], attached=[aco])


def main():
    stage = sys.argv[1]
    if stage == "info":
        print("R_G=\n", np.round(R_G, 3)); print("Q_G", np.round(Q_G, 4))
        print("PREGRASP", np.round(PREGRASP, 4)); print("HANDLE", HANDLE)
        print("PREINSERT", np.round(PREINSERT, 4)); print("INSERT", np.round(INSERT, 4)); print("RETRACT", RETRACT)
        return
    rclpy.init()
    p = Planner()
    print("start tcp", np.round(p.tcp()[0], 4), "gap", round(p.finger_gap(), 4), flush=True)

    if stage == "td_grasp":
        # top-down grasp of the handle bar: hand z down, fingers along x
        p.set_scene(microwave="solid", yellow=False)
        p.grip(True)
        ok = p.goto_tcp(TD_GRASP + np.array([0, 0, 0.10]), Q_TD, time_s=10.0)
        print("above ok", ok, flush=True)
        if ok:
            ok = p.line(TD_GRASP, quat=Q_TD, step=0.005, avoid=False, time_scale=2.0)
            print("descend ok", ok, flush=True)
            p.grip(False)
            print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "td_descend":
        ok = p.line(TD_GRASP, quat=Q_TD, step=0.005, avoid=False, time_scale=2.0)
        print("descend ok", ok, flush=True)
        p.grip(False)
        print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "drag":
        # drag the mug along the table (+y) so a frontal grasp has room behind it
        pos = p.tcp()[0]
        ok = p.line(pos + np.array([0, DRAG_DY, 0]), quat=Q_TD, step=0.005, avoid=False, time_scale=3.0)
        print("drag ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
        p.grip(True)
        pos = p.tcp()[0]
        ok = p.line(pos + np.array([0, 0, 0.10]), quat=Q_TD, step=0.005, avoid=False)
        print("raise ok", ok, flush=True)
    elif stage == "fgrasp":
        # frontal (pitched) handle grasp at the relocated mug, approached from above
        p.set_scene(microwave="solid", yellow=False)
        p.apply_scene([cylinder("yellow_mug", MUG_XY[0], MUG_XY[1], 0.055, 0.90, 1.006)])
        p.grip(True)
        ok = p.goto_tcp(HANDLE + np.array([0, 0, 0.08]), Q_G, time_s=10.0)
        print("above ok", ok, flush=True)
        if ok:
            p.apply_scene([], remove_ids=["yellow_mug"])
            ok = p.line(HANDLE, quat=Q_G, step=0.005, avoid=False, time_scale=2.0)
            print("descend ok", ok, flush=True)
            p.grip(False)
            print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "fgrasp2":
        # re-do the descent slowly (controller lag deflected the first attempt)
        p.grip(True)
        pos = p.tcp()[0]
        p.line(pos + np.array([0, 0, 0.05]), quat=Q_G, step=0.005, avoid=False, time_scale=3.0)
        p.line(HANDLE + np.array([0, 0, 0.05]), quat=Q_G, step=0.005, avoid=False, time_scale=3.0)
        ok = p.line(HANDLE, quat=Q_G, step=0.0025, avoid=False, time_scale=6.0)
        print("descend ok", ok, flush=True)
        p.grip(False)
        print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "fgrasp3":
        tcp, R, q = grasp_from_measure(MEAS_CENTRE, MEAS_BAR)
        p.grip(True)
        p.set_scene(microwave="solid", yellow=False)
        ok = p.goto_tcp(tcp + np.array([0, 0, 0.06]), q, time_s=10.0)
        print("above ok", ok, flush=True)
        if ok:
            ok = p.line(tcp, quat=q, step=0.0025, avoid=False, time_scale=6.0)
            print("descend ok", ok, flush=True)
            p.grip(False)
            print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "relocate":
        tcp_g, R_g, tcp_p, R_p, d_g = td_frames(MEAS_CENTRE, MEAS_BAR)
        q_g, q_p = mat_to_quat(R_g), mat_to_quat(R_p)
        p.set_scene(microwave="solid", yellow=False)
        p.grip(True)
        ok = p.goto_tcp(tcp_g + np.array([0, 0, 0.08]), q_g, time_s=10.0)
        print("above ok", ok, flush=True)
        if not ok: return
        ok = p.line(tcp_g, quat=q_g, step=0.004, avoid=False, time_scale=3.0)
        print("descend ok", ok, flush=True)
        p.grip(False)
        gap = p.finger_gap(); print("gap after close", round(gap, 4), flush=True)
        if not (0.010 < gap < 0.025):
            print("unexpected gap, stopping", flush=True); return
        c_rel = np.array([*(GRIP_TO_CENTRE * d_g), 0.9525 - TD_Z])
        attach_mug(p, R=R_g, c_world_rel=c_rel)
        ok = p.line(tcp_g + np.array([0, 0, 0.04]), quat=q_g, step=0.004, avoid=False, time_scale=3.0)
        print("lift ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
        ok = p.goto_tcp(tcp_p + np.array([0, 0, 0.04]), q_p, time_s=10.0)
        print("carry ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
        if not ok: return
        ok = p.line(tcp_p, quat=q_p, step=0.004, avoid=False, time_scale=3.0)
        print("lower ok", ok, flush=True)
        p.grip(True)
        p.detach_mug()
        ok = p.line(tcp_p + np.array([0, 0, 0.10]), quat=q_p, step=0.005, avoid=False, time_scale=2.0)
        print("raise ok", ok, flush=True)
    elif stage == "lift2":
        tcp, R, q = grasp_from_measure(MEAS_CENTRE, MEAS_BAR)
        d = R[:2, 2] / np.linalg.norm(R[:2, 2])
        c_rel = np.array([*(GRIP_TO_CENTRE * d), 0.9525 - HANDLE[2]])
        attach_mug(p, R=R, c_world_rel=c_rel)
        pos = p.tcp()[0]
        ok = p.line(pos + np.array([0, 0, LIFT_DZ]), quat=q, step=0.004, avoid=False, time_scale=3.0)
        print("lift ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "servo_test":
        tcp, R, q = grasp_from_measure(MEAS_CENTRE, MEAS_BAR)
        p.grip(True)
        ok = servo_tcp(p, tcp, q)
        print("servo ok", ok, flush=True)
        if ok:
            p.grip(False)
            print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "backoff":
        p.grip(True)
        pos = p.tcp()[0]
        p.line(pos + np.array([0, 0, 0.10]), quat=Q_G, step=0.005, avoid=False, time_scale=3.0)
    elif stage == "ikcheck":
        for name, pos in (("PREGRASP", PREGRASP), ("HANDLE", HANDLE), ("PREINSERT", PREINSERT), ("INSERT", INSERT)):
            q = p.ik(pos, Q_G, attempts=10)
            print(name, np.round(pos, 3), None if q is None else np.round(q, 3).tolist(), flush=True)
    elif stage == "pregrasp":
        p.set_scene(microwave="solid", yellow=True)
        p.grip(True)
        ok = p.goto_tcp(PREGRASP, Q_G)
        print("pregrasp ok", ok, flush=True)
    elif stage == "grasp":
        p.apply_scene([], remove_ids=["yellow_mug"])   # mug leaves the collision world for the approach
        ok = p.line(HANDLE, quat=Q_G, step=0.005, avoid=True)
        print("approach ok", ok, flush=True)
        p.grip(False)
        print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "lift":
        attach_mug(p)
        pos = p.tcp()[0]
        ok = p.line(pos + np.array([0, 0, LIFT_DZ]), quat=Q_G, step=0.005, avoid=False)
        print("lift ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "preinsert":
        p.set_scene(microwave="walls", yellow=False)
        ok = p.goto_tcp(PREINSERT, Q_G, time_s=10.0)
        print("preinsert ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "insert":
        ok = p.line(INSERT, quat=Q_G, step=0.005, avoid=False, time_scale=2.5)
        print("insert ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "release":
        p.grip(True)
        p.detach_mug()
        print("gap", round(p.finger_gap(), 4), flush=True)
    elif stage == "retract":
        ok = p.line(RETRACT, quat=Q_G, step=0.005, avoid=False)
        print("retract ok", ok, flush=True)
    elif stage == "park":
        p.set_scene(microwave="solid", yellow=False)
        ok = p.goto_q(HOME_Q, time_s=10.0)
        print("park ok", ok, flush=True)
    print("end tcp", np.round(p.tcp()[0], 4), "q", np.round(p.arm_q(), 3).tolist(), flush=True)
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
