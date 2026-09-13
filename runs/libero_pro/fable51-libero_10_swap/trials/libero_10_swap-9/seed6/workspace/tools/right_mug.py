#!/usr/bin/env python3
"""Recover the yellow mug lying on its side in front of the microwave.
Stages: pinch (rim-wall pinch, one finger inside the mouth), upright (rotate & set down).
usage: right_mug.py pinch|upright|info"""
import sys
import numpy as np
import rclpy
sys.path.insert(0, "/workspace/tools")
from robot import mat_to_quat, TCP
from planner import Planner, box, ALL_IDS, scene_objects
from task import attach_mug, Rz

# measured lying pose (birdview/agentview depth)
AX_X, AX_Z = -0.168, 0.952          # mug axis (along world y) at the rim end
RIM_Y = -0.437                      # rim plane
MUG_LEN, RIM_R = 0.105, 0.0485
PITCH = np.deg2rad(50.0)
ANG = np.deg2rad(25.0)              # pinch point angle above the +x side of the rim
INSIDE = 0.016                      # pad centre this far inside the mouth (along y)


def grasp_frame():
    a = np.array([0.0, np.cos(PITCH), -np.sin(PITCH)])                     # approach: +y, pitched down
    yh = np.array([np.cos(ANG), np.sin(ANG) * np.sin(PITCH), np.sin(ANG) * np.cos(PITCH)])  # closing dir (radial-ish)
    yh -= yh.dot(a) * a; yh /= np.linalg.norm(yh)
    xh = np.cross(yh, a)
    R = np.stack([xh, yh, a], 1)
    assert np.linalg.det(R) > 0.99
    rim_pt = np.array([AX_X + (RIM_R - 0.002) * np.cos(ANG), RIM_Y, AX_Z + (RIM_R - 0.002) * np.sin(ANG)])
    tcp = rim_pt + np.array([0, INSIDE, 0])
    # mug centre relative to the TCP (world axes) and cylinder axis (world y)
    c_rel = np.array([AX_X, RIM_Y + MUG_LEN / 2, AX_Z]) - tcp
    return tcp, R, a, c_rel


def attach_lying(p, R, c_rel):
    """Attach the lying mug (axis along world y) to the hand."""
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    aco = AttachedCollisionObject(); aco.link_name = "panda_hand"
    aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
    co = CollisionObject(); co.header.frame_id = "panda_hand"; co.id = "held_mug"
    prim = SolidPrimitive(); prim.type = SolidPrimitive.CYLINDER; prim.dimensions = [MUG_LEN, 0.05]
    pose = Pose()
    c_hand = R.T @ c_rel + np.array([0, 0, TCP])
    pose.position.x, pose.position.y, pose.position.z = map(float, c_hand)
    ax = R.T @ np.array([0, 1.0, 0])                     # world y in hand frame = cylinder axis
    v = np.cross([0, 0, 1.0], ax); s = np.linalg.norm(v); c = np.dot([0, 0, 1.0], ax)
    axn = v / s; ang = np.arctan2(s, c)
    q = [*(axn * np.sin(ang / 2)), np.cos(ang / 2)]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    co.primitives.append(prim); co.primitive_poses.append(pose); co.operation = CollisionObject.ADD
    aco.object = co
    return p.apply_scene([], attached=[aco])


def main():
    stage = sys.argv[1]
    rclpy.init(); p = Planner()
    tcp, R, a, c_rel = grasp_frame()
    q = mat_to_quat(R)
    print("pinch tcp", np.round(tcp, 4), "a", np.round(a, 3), "closing", np.round(R[:, 1], 3), flush=True)
    if stage == "info":
        s = p.ik(tcp, q, attempts=8); print("ik pinch", None if s is None else np.round(s, 3).tolist())
        s = p.ik(tcp - 0.05 * a, q, attempts=8); print("ik pre", None if s is None else np.round(s, 3).tolist())
    elif stage == "pinch":
        objs = scene_objects(microwave="solid", yellow=False)
        objs.append(box("yellow_mug", [-0.225, -0.435, 0.90], [-0.07, -0.33, 1.006]))
        p.apply_scene(objs, remove_ids=ALL_IDS)
        p.grip(True)
        ok = p.goto_tcp(tcp - 0.05 * a, q, time_s=10.0)
        print("pre ok", ok, flush=True)
        if not ok: return
        p.apply_scene([], remove_ids=["yellow_mug"])
        ok = p.line(tcp, quat=q, step=0.0025, avoid=False, time_scale=5.0)
        print("enter ok", ok, flush=True)
        p.grip(False)
        print("gap after close", round(p.finger_gap(), 4), flush=True)
    elif stage == "upright":
        gap = p.finger_gap(); print("gap", round(gap, 4))
        if gap < 0.002:
            print("nothing in the gripper"); return
        attach_lying(p, R, c_rel)
        pos = p.tcp()[0]
        ok = p.line(pos + np.array([0, 0, 0.03]), quat=q, step=0.004, avoid=False, time_scale=4.0)
        print("lift ok", ok, "gap", round(p.finger_gap(), 4), flush=True)
        # candidate upright poses: R_final = Rz(phi) @ Rx(-90deg) @ R ; mug bottom 6 mm above the table
        Rx = np.array([[1, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])            # rotation about x by -90deg: +y -> -z
        assert np.allclose(Rx @ np.array([0, 1.0, 0]), [0, 0, -1])
        best = None
        for phi_deg in [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, -20, -40, -60, -80, -100, -120, -140, -160]:
            Rf = Rz(np.deg2rad(phi_deg)) @ Rx @ R
            c_rel_f = Rz(np.deg2rad(phi_deg)) @ Rx @ c_rel            # mug centre rel. TCP after rotation
            for cx, cy in [(-0.15, -0.42), (-0.12, -0.42), (-0.18, -0.45), (-0.10, -0.46)]:
                tcp_f = np.array([cx, cy, 0.906 + MUG_LEN / 2]) - c_rel_f
                s = p.ik(tcp_f, mat_to_quat(Rf), attempts=4, avoid=True)
                if s is not None:
                    best = (phi_deg, (cx, cy), tcp_f, Rf, s); break
            if best: break
        if best is None:
            print("no collision-free upright pose found"); return
        phi_deg, c, tcp_f, Rf, s = best
        print(f"upright pose: phi {phi_deg} mug centre {c} tcp {np.round(tcp_f,4)} q {np.round(s,3).tolist()}", flush=True)
        ok = p.goto_q(s, time_s=10.0)
        print("rotate ok", ok, "gap", round(p.finger_gap(), 4), "tcp", np.round(p.tcp()[0], 4), flush=True)
        if not ok: return
        # lower 6 mm to set the base on the table, release, retreat along -a (up and away)
        qf = mat_to_quat(Rf)
        ok = p.line(tcp_f - np.array([0, 0, 0.005]), quat=qf, step=0.0025, avoid=False, time_scale=4.0)
        print("set down ok", ok, flush=True)
        p.grip(True)
        p.detach_mug()
        af = Rf[:, 2]
        ok = p.line(p.tcp()[0] - 0.06 * af + np.array([0, 0, 0.03]), quat=qf, step=0.004, avoid=False, time_scale=3.0)
        print("retreat ok", ok, flush=True)
    print("end tcp", np.round(p.tcp()[0], 4), "q", np.round(p.arm_q(), 3).tolist())
    p.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
