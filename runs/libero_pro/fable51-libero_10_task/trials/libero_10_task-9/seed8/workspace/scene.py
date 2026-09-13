"""Planning-scene helpers: publish world collision boxes, validate joint states."""
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive

from lib import ARM, R_to_quat

# ---------- measured world geometry (metres, world frame) ----------
TABLE_Z = 0.90
MW = dict(x0=-0.175, x1=0.166, y0=0.267, y1=0.46, z0=TABLE_Z, z1=1.107)  # microwave exterior
CAV = dict(x0=-0.149, x1=0.055, y0=0.267, y1=0.435, z0=0.942, z1=1.087)  # cavity, measured with eye-in-hand
DOOR_HINGE = np.array([-0.182, 0.267])
DOOR_TIP = np.array([-0.304, 0.041])
DOOR_Z = (0.93, 1.108)
DOOR_THICK = 0.03
YELLOW = dict(x0=-0.03, x1=0.075, y0=-0.085, y1=0.05, z0=TABLE_Z, z1=1.0)
WHITE_MUG = dict(cx=-0.0985, cy=-0.2535, r=0.047, h=0.115)  # rim radius; tapers to ~0.03 at the base


def box(name, lo, hi, frame="world"):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, hi - lo)))
    pose = Pose()
    c = (lo + hi) / 2
    pose.position.x, pose.position.y, pose.position.z = map(float, c)
    pose.orientation.w = 1.0
    co.primitives = [prim]
    co.primitive_poses = [pose]
    co.operation = CollisionObject.ADD
    return co


def oriented_box(name, center, size, R, frame="world"):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, size)))
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, center)
    q = R_to_quat(R)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    co.primitives = [prim]
    co.primitive_poses = [pose]
    co.operation = CollisionObject.ADD
    return co


def door_object(hinge=DOOR_HINGE, tip=DOOR_TIP):
    d = tip - hinge
    L = np.linalg.norm(d)
    ang = np.arctan2(d[1], d[0])
    R = np.array([[np.cos(ang), -np.sin(ang), 0], [np.sin(ang), np.cos(ang), 0], [0, 0, 1]])
    c2 = (hinge + tip) / 2
    center = [c2[0], c2[1], (DOOR_Z[0] + DOOR_Z[1]) / 2]
    return oriented_box("door", center, [L, DOOR_THICK, DOOR_Z[1] - DOOR_Z[0]], R)


def world_objects(with_white_mug=True, cav=CAV, door=True):
    objs = [
        box("table", [-0.55, -0.65, TABLE_Z - 0.05], [0.45, 0.60, TABLE_Z]),
        box("mw_floor", [MW["x0"], MW["y0"], MW["z0"]], [MW["x1"], MW["y1"], cav["z0"]]),
        box("mw_ceiling", [MW["x0"], MW["y0"], cav["z1"]], [MW["x1"], MW["y1"], MW["z1"]]),
        box("mw_left", [MW["x0"], MW["y0"], MW["z0"]], [cav["x0"], MW["y1"], MW["z1"]]),
        box("mw_right", [cav["x1"], MW["y0"], MW["z0"]], [MW["x1"], MW["y1"], MW["z1"]]),
        box("mw_back", [MW["x0"], cav["y1"], MW["z0"]], [MW["x1"], MW["y1"], MW["z1"]]),
        box("yellow_mug", [YELLOW["x0"], YELLOW["y0"], YELLOW["z0"]], [YELLOW["x1"], YELLOW["y1"], YELLOW["z1"]]),
    ]
    if door:
        objs.append(door_object())
    if with_white_mug:
        w = WHITE_MUG
        objs.append(box("white_mug", [w["cx"] - w["r"], w["cy"] - w["r"], TABLE_Z],
                        [w["cx"] + w["r"], w["cy"] + w["r"], TABLE_Z + w["h"]]))
        objs.append(box("white_handle", [-0.106, -0.207, 0.915], [-0.088, -0.170, 0.995]))
    return objs


# yellow mug: body ~10 cm wide, handle on its -y side
YELLOW_BODY = dict(x0=-0.026, x1=0.074, y0=-0.052, y1=0.050, z0=TABLE_Z, z1=1.0)
YELLOW_HANDLE = dict(x0=0.018, x1=0.065, y0=-0.083, y1=-0.052, z0=0.92, z1=1.0)


def yellow_objects(dx=0.0, dy=0.0):
    """Yellow mug body + handle boxes, optionally shifted (after pushing it)."""
    b, h = YELLOW_BODY, YELLOW_HANDLE
    return [box("yellow_mug", [b["x0"] + dx, b["y0"] + dy, b["z0"]], [b["x1"] + dx, b["y1"] + dy, b["z1"]]),
            box("yellow_handle", [h["x0"] + dx, h["y0"] + dy, h["z0"]], [h["x1"] + dx, h["y1"] + dy, h["z1"]])]


class Scene:
    def __init__(self, node):
        self.node = node
        self.apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.valid = node.create_client(GetStateValidity, "/check_state_validity")
        self.apply.wait_for_service(10)
        self.valid.wait_for_service(10)

    def publish(self, objects, remove=()):
        ps = PlanningScene()
        ps.is_diff = True
        for name in remove:
            co = CollisionObject()
            co.header.frame_id = "world"
            co.id = name
            co.operation = CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        ps.world.collision_objects.extend(objects)
        req = ApplyPlanningScene.Request(scene=ps)
        fut = self.apply.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        return fut.result().success

    def attach(self, name, lo, hi, link="panda_hand", detach=False):
        """Attach a box (given in hand-frame lo/hi) to the hand, or detach it."""
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
        if detach:
            aco.object.id = name
            aco.object.operation = CollisionObject.REMOVE
        else:
            aco.object = box(name, lo, hi, frame=link)
        ps.robot_state.attached_collision_objects.append(aco)
        if detach:  # also drop it from the world so it does not linger
            co = CollisionObject()
            co.header.frame_id = "world"
            co.id = name
            co.operation = CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        fut = self.apply.call_async(ApplyPlanningScene.Request(scene=ps))
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        return fut.result().success

    def check(self, q, verbose=True):
        req = GetStateValidity.Request()
        req.group_name = "panda_arm"
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.valid.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None:
            raise RuntimeError("validity service timeout")
        if verbose and not res.valid:
            pairs = {(c.contact_body_1, c.contact_body_2) for c in res.contacts}
            print("    INVALID contacts:", sorted(pairs), flush=True)
        return res.valid, [(c.contact_body_1, c.contact_body_2) for c in res.contacts]


def path_valid(sc, q_from, q_to, steps=20, verbose=False):
    """Linear joint-space interpolation collision check."""
    q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
    bad = []
    for i in range(steps + 1):
        q = q_from + (q_to - q_from) * i / steps
        v, contacts = sc.check(q, verbose=False)
        if not v:
            bad.append((i, sorted({tuple(c) for c in contacts})))
    if verbose and bad:
        for b in bad:
            print(f"    step {b[0]}/{steps}: {b[1]}", flush=True)
    return not bad, bad
