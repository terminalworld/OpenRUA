"""Attach/detach the white mug collision body to panda_hand in the planning scene."""
import sys, numpy as np, rclpy
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from rob import *; import scene

def attach_mug(r, mug_center_world, radius=0.05, height=0.12, axis_world=(0, 0, 1)):
    hand_pos, R = r.solve_fk(r.joints())
    d_hand = R.T @ (np.asarray(mug_center_world) - hand_pos)       # mug centre in hand frame
    # cylinder z-axis -> axis_world: build a world rotation whose z is the mug axis, express in hand frame
    z = np.asarray(axis_world, float); z /= np.linalg.norm(z)
    x = np.cross([0, 1, 0] if abs(z[1]) < 0.9 else [1, 0, 0], z); x /= np.linalg.norm(x)
    Rm = np.column_stack([x, np.cross(z, x), z])
    q = Rot.from_matrix(R.T @ Rm).as_quat()
    a = AttachedCollisionObject(); a.link_name = "panda_hand"
    a.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
    co = a.object; co.header.frame_id = "panda_hand"; co.id = "white_mug"
    co.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[height, radius])]
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, d_hand)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    co.primitive_poses = [p]; co.operation = CollisionObject.ADD
    ok0 = scene.apply(r.node, [scene.remove("white_mug")])
    ok = scene.apply(r.node, [], attached=a); print('remove', ok0)
    print("attached white_mug at hand-frame offset", np.round(d_hand,3), "ok", ok)

def detach_mug(r, world_center):
    ok = scene.apply(r.node, [scene.cyl("white_mug", world_center, 0.047, 0.112)], detach="white_mug")
    print("detached, world obj re-added at", np.round(world_center,3), "ok", ok)

if __name__ == "__main__":
    r = Robot("attach")
    if sys.argv[1] == "attach":
        ax = [float(v) for v in sys.argv[5:8]] if len(sys.argv) >= 8 else (0, 0, 1)
        attach_mug(r, [float(v) for v in sys.argv[2:5]], axis_world=ax)
    else: detach_mug(r, [float(v) for v in sys.argv[2:5]])
