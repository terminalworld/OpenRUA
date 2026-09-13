#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/ik_move.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
ros2 topic list; echo ---ACTIONS; ros2 action list; echo ---SERVICES; ros2 service list | head -60; echo ---NODES; ros2 node list

# openrua op 3
ros2 topic info /pipeline_state; ros2 topic echo /pipeline_state --once 2>&1 | head -40; echo ---; ros2 node info /robot_bridge 2>&1 | head -60; echo ---; ros2 topic echo /joint_states --once

# openrua op 4
cat /tmp/claude-1000/-workspace/1c92b845-a859-46fe-9318-98768cae257d/tasks/bmpmincj3.output

# openrua op 5
timeout 30 ros2 node info /robot_bridge 2>&1 | head -60; echo ---; timeout 30 ros2 topic echo /joint_states --once

# openrua op 6
mkdir -p snaps && for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done

# openrua op 7
timeout 60 python3 -c "
import rclpy
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
rclpy.init(); n=rclpy.create_node('tfl')
got=[]
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
n.create_subscription(TFMessage,'/tf_static',lambda m: got.append(('static',m)),qos)
n.create_subscription(TFMessage,'/tf',lambda m: got.append(('dyn',m)),10)
import time
t=time.time()
while time.time()-t<8: rclpy.spin_once(n,timeout_sec=0.2)
seen=set()
for k,m in got:
  for tr in m.transforms:
    key=(k,tr.header.frame_id,tr.child_frame_id)
    if key in seen: continue
    seen.add(key)
    t_=tr.transform.translation; q=tr.transform.rotation
    print(k,tr.header.frame_id,'->',tr.child_frame_id, f'{t_.x:.3f} {t_.y:.3f} {t_.z:.3f} | {q.x:.3f} {q.y:.3f} {q.z:.3f} {q.w:.3f}')
"

# openrua op 8
mkdir -p "$(dirname /workspace/px.py)"
cat > /workspace/px.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Batch pixel->world for one camera: python3 px.py <cam> u,v [u,v ...]
Also saves <cam>_depth.npy. Uses the TF world->optical frame."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    pts = [tuple(int(v) for v in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("px")
    buf = Buffer(); TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
    np.save(f"snaps/{cam}_depth.npy", depth)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    o = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print(f"cam {cam} {depth.shape} fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")
    for (u, v) in pts:
        z = depth[v, u]
        p = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
        w = R @ p + o
        print(f"px({u},{v}) depth={z:.3f} -> world {w[0]:.4f} {w[1]:.4f} {w[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 9
timeout 90 python3 px.py birdview 240,275 300,275 378,235 378,280 312,312 100,350 500,350 320,240

# openrua op 10
timeout 90 python3 px.py birdview 200,350 450,200 240,250 240,300 215,275 265,275 378,265 378,295 365,280 391,280 370,235 386,235 378,225 378,245

# openrua op 11
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot helper: one node, persistent clients.

    from rob import Robot
    r = Robot()
    r.joints()            # dict name->pos
    r.fk()                # (xyz, quat xyzw) of panda_hand in world
    r.ik(xyz, quat)       # arm joint list or None (hand frame, world coords)
    r.move_joints(list, secs)
    r.move_pose(xyz, quat, secs, tcp=False)
    r.gripper(width)      # per-finger position
    r.servo(vx,vy,vz, wx,wy,wz, n)  # twist bursts
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 (from TF)


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_z(theta):
    return np.array([0, 0, np.sin(theta / 2), np.cos(theta / 2)])


def down_quat(yaw):
    """Hand pointing straight down (hand +Z = world -Z); fingers close
    along the world axis obtained by rotating world Y by `yaw`."""
    return quat_mul(quat_z(yaw), np.array([1.0, 0.0, 0.0, 0.0]))


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.3):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._wr is None:
            return None
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    # ---- kinematics (world frame; planner works in panda_link0) ----
    def fk(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.header.frame_id = ""
        js = JointState()
        js.name = list(ARM)
        js.position = list(q if q is not None else self.arm_q())
        req.robot_state.joint_state = js
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print("FK failed", None if res is None else res.error_code.val)
            return None
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q_ = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return xyz, q_

    def ik(self, xyz, quat, seed=None, tcp=False, tries=3):
        xyz = np.array(xyz, dtype=float)
        if tcp:
            xyz = xyz - TCP * quat_R(quat)[:, 2]
        local = xyz - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, local)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        js = JointState()
        js.name = list(ARM)
        js.position = list(seed if seed is not None else self.arm_q())
        req.ik_request.robot_state.joint_state = js
        for _ in range(tries):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[n] for n in ARM]
            print("IK fail", None if res is None else res.error_code.val)
        return None

    # ---- motion ----
    def move_joints(self, q, secs=3.0, wait=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"move_joints code={code} max_err={err:.4f}")
        return code, err

    def move_pose(self, xyz, quat, secs=3.0, tcp=False, seed=None):
        q = self.ik(xyz, quat, seed=seed, tcp=tcp)
        if q is None:
            print("move_pose: no IK for", xyz)
            return None
        self.move_joints(q, secs)
        got = self.fk()
        print("  hand now", np.round(got[0], 4), "tcp", np.round(got[0] + TCP * quat_R(got[1])[:, 2], 4))
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}")
        return r

    def servo(self, lin, ang=(0, 0, 0), n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=dt)

    def tcp(self):
        xyz, q = self.fk()
        return xyz + TCP * quat_R(q)[:, 2], q
OPENRUA_EOF

# openrua op 12
timeout 300 python3 -u -c "
from rob import *
r=Robot()
print('joints', r.joints())
xyz,q=r.fk(); print('hand world', xyz, q)
xyz8,q8=r.fk(link='panda_link8'); print('link8 world', xyz8, q8)
print('tcp', r.tcp())
print('wrench', r.wrench())
# settle: tiny move to current config
q0=r.arm_q(); r.move_joints(q0, 1.0)
r.move_joints(q0, 1.0)
print('joints after', r.joints())
"

# openrua op 13
timeout 60 python3 -c "
import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('t'); b=Buffer(); TransformListener(b,n)
end=time.time()+10
while time.time()<end and not b.can_transform('world','panda_hand',rclpy.time.Time()): rclpy.spin_once(n,timeout_sec=0.2)
t=b.lookup_transform('world','panda_hand',rclpy.time.Time()); print(t.transform)
t=b.lookup_transform('panda_link0','panda_hand',rclpy.time.Time()); print(t.transform)
"

# openrua op 14
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 (from TF)", "new_string": "BASE = np.array([0.0, 0.0, 0.0])  # FK/IK on this machine are already in world (verified vs TF)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 15
timeout 300 python3 -u -c "
from rob import *
r=Robot()
xyz,q=r.fk(); print('hand', xyz, q)
sol=r.ik(xyz,q); print('ik of current', np.round(sol,3), 'cur', np.round(r.arm_q(),3))
# test a down-pose IK above the knob
kq=down_quat(0.0)
print('down quat', kq)
sol=r.ik([-0.218,0.204,1.10], kq); print('ik above knob', sol)
if sol: print('fk of sol', r.fk(sol))
" ; for c in agentview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done

# openrua op 16
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        local = xyz - BASE\\n        req = GetPositionIK.Request()", "new_string": "        local = xyz - BASE\\n        # IK tip link is panda_link8 (verified): convert the requested\\n        # panda_hand orientation (hand = link8 rotated -45deg about z)\\n        quat = quat_mul(np.array(quat, dtype=float), quat_z(np.pi / 4))\\n        req = GetPositionIK.Request()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 17
timeout 300 python3 -u -c "
from rob import *
r=Robot()
xyz,q=r.fk()
sol=r.ik(xyz,q); print('ik of current', np.round(sol,3), 'cur', np.round(r.arm_q(),3))
sol=r.ik([-0.218,0.204,1.10], down_quat(0.0)); print('fk of sol', r.fk(sol))
sol=r.ik([-0.218,0.204,1.10], down_quat(np.pi/2)); print('fk of sol yaw90', r.fk(sol))
"

# openrua op 18
timeout 90 python3 px.py birdview 378,235 >/dev/null; python3 -c "
import numpy as np
d=np.load('snaps/birdview_depth.npy')
# birdview: camera at z=3.0 looking straight down; world z = 3.0 - depth (approx, center pixel); x = -0.2 + (v-240)*depth/579.4 ; y = (u-320)*depth/579.4
np.set_printoptions(linewidth=250, precision=3, suppress=True)
z=3.0-d
print('knob region rows 222..250, cols 364..392 (world z):')
for v in range(222,251,2):
    print(v, ' '.join(f'{z[v,u]:.3f}' for u in range(364,393,2)))
"


# openrua op 19
timeout 600 python3 -u -c "
from rob import *
r=Robot()
KNOB=np.array([-0.211,0.204])
q=down_quat(0.0)
r.gripper(0.04)
r.move_pose([KNOB[0],KNOB[1],1.06], q, 4.0, tcp=True)
r.move_pose([KNOB[0],KNOB[1],0.975], q, 2.0, tcp=True)
print('tcp', r.tcp())
print('wrench', r.wrench())
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih.png && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agentview.png

# openrua op 20
timeout 600 python3 -u -c "
from rob import *
r=Robot()
KNOB=np.array([-0.211,0.204])
q=down_quat(0.0)
r.move_pose([KNOB[0],KNOB[1],0.940], q, 2.0, tcp=True)
print('wrench', r.wrench())
r.gripper(0.0)
print('gap', r.finger_gap())
print('joints', np.round(r.arm_q(),4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih.png

# openrua op 21
timeout 600 python3 -u -c "
from rob import *
r=Robot()
q=r.arm_q(); q[6]+=np.pi/2
r.move_joints(q, 3.0)
print('gap', r.finger_gap(), 'wrench', r.wrench())
print('tcp', r.tcp())
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih.png && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agentview.png && timeout 60 python3 tools/perception/cam_snap.py robot0_robotview snaps/robot0_robotview.png

# openrua op 22
timeout 600 python3 -u -c "
from rob import *
r=Robot()
q=r.arm_q(); q[6]=1.3746
r.move_joints(q, 2.0)
print('gap', r.finger_gap(), 'wrench', r.wrench(), 'tcp', r.tcp())
q[6]=1.3746-np.pi/2
r.move_joints(q, 3.0)
print('gap', r.finger_gap(), 'wrench', r.wrench())
print('tcp', r.tcp())
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih.png && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agentview.png

# openrua op 23
timeout 600 python3 -u -c "
from rob import *
r=Robot()
r.gripper(0.04)
xyz,q=r.tcp()
r.move_pose([xyz[0],xyz[1],1.10], q, 2.0, tcp=True)
" && timeout 60 python3 tools/perception/cam_snap.py birdview snaps/birdview.png && timeout 90 python3 px.py birdview 240,275 378,280 && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agentview.png

# openrua op 24
python3 -c "
import numpy as np
d=np.load('snaps/birdview_depth.npy')
z=3.0-d
np.set_printoptions(linewidth=300)
# object mask above table in region around pan: rows 230..330, cols 180..340
sub=z[230:330,180:340]
mask=sub>0.905
vs,us=np.nonzero(mask)
print('pan+handle bbox rows',vs.min()+230,vs.max()+230,'cols',us.min()+180,us.max()+180)
# print coarse height map every 4 px
for v in range(236,320,4):
    print(v, ''.join(('%3d'%int((z[v,u]-0.9)*1000)) if z[v,u]>0.902 else '  .' for u in range(190,330,4)))
"


# openrua op 25
python3 -c "
import numpy as np
d=np.load('snaps/birdview_depth.npy')
z=3.0-d
for v in range(268,282):
    print(v, ''.join(('%3d'%int((z[v,u]-0.9)*1000)) if z[v,u]>0.902 else '  .' for u in range(268,320,2)))
print('cols', list(range(268,320,2)))
# pan center precise: centroid of pixels with z between 0.925 and 0.95 in pan region (rim)
sub=z[240:310,205:280]
vs,us=np.nonzero((sub>0.925)&(sub<0.95))
print('rim centroid v,u', vs.mean()+240, us.mean()+205)
"


# openrua op 26
timeout 900 python3 -u -c "
from rob import *
r=Robot()
G=np.array([-0.0755,-0.108])
q=down_quat(np.pi/2)
r.gripper(0.04)
r.move_pose([G[0],G[1],1.10], q, 4.0, tcp=True)
r.move_pose([G[0],G[1],0.975], q, 2.0, tcp=True)
print('wrench', r.wrench())
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih.png

# openrua op 27
timeout 300 python3 -u -c "
from rob import *
r=Robot()
xyz,q=r.fk(); print('hand',xyz,q)
print('target q', down_quat(np.pi/2))
print('joints', np.round(r.arm_q(),3))
sol=r.ik([-0.0755,-0.108,0.975], down_quat(np.pi/2), tcp=True); print('ik', np.round(sol,3))
"

# openrua op 28
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def move_joints(self, q, secs=3.0, wait=True):\\n        goal = FollowJointTrajectory.Goal()", "new_string": "    def move_joints(self, q, secs=3.0, retries=3, tol=0.01):\\n        for i in range(retries):\\n            code, err = self._move_once(q, secs)\\n            if err < tol:\\n                break\\n            print(f\\"  retry {i + 1}: err {err:.3f}\\")\\n        return code, err\\n\\n    def _move_once(self, q, secs=3.0):\\n        goal = FollowJointTrajectory.Goal()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 29
timeout 900 python3 -u -c "
from rob import *
r=Robot()
G=np.array([-0.0755,-0.108])
q=down_quat(np.pi/2)
r.move_pose([G[0],G[1],0.975], q, 3.0, tcp=True)
print('hand', r.fk())
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih.png

# openrua op 30
timeout 900 python3 -u -c "
from rob import *
r=Robot()
G=np.array([-0.0755,-0.108])
q=down_quat(np.pi/2)
r.move_pose([G[0],G[1],0.932], q, 2.0, tcp=True)
print('wrench', r.wrench())
r.gripper(0.0)
print('gap', r.finger_gap())
# lift slowly a bit and check the pan comes along
r.move_pose([G[0],G[1],0.99], q, 2.0, tcp=True)
print('gap', r.finger_gap(), 'wrench', r.wrench())
" && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agentview.png && timeout 60 python3 tools/perception/cam_snap.py sideview snaps/sideview.png

# openrua op 31
timeout 900 python3 -u -c "
from rob import *
r=Robot()
q=down_quat(np.pi/2)
G=np.array([-0.0755,-0.108])
r.move_pose([G[0],G[1],1.13], q, 3.0, tcp=True)
print('gap', r.finger_gap())
P=np.array([-0.057,0.366])
r.move_pose([P[0],P[1],1.13], q, 5.0, tcp=True)
print('gap', r.finger_gap(), 'wrench', r.wrench())
" && timeout 60 python3 tools/perception/cam_snap.py birdview snaps/birdview.png && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agentview.png

# openrua op 32
timeout 900 python3 -u -c "
from rob import *
r=Robot()
q=down_quat(np.pi/2)
P=np.array([-0.057,0.366])
r.move_pose([P[0],P[1],1.00], q, 3.0, tcp=True)
print('wrench', r.wrench())
r.move_pose([P[0],P[1],0.968], q, 2.0, tcp=True)
print('wrench', r.wrench())
r.gripper(0.04)
r.move_pose([P[0],P[1],1.10], q, 2.0, tcp=True)
" && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agentview.png && timeout 60 python3 tools/perception/cam_snap.py birdview snaps/birdview.png

# openrua op 33
timeout 90 python3 px.py birdview 378,280 >/dev/null; python3 -c "
import numpy as np
d=np.load('snaps/birdview_depth.npy'); z=3.0-d
sub=z[240:330,330:430]
vs,us=np.nonzero((sub>0.955)&(sub<0.985))   # pan rim band above the 0.93 plate
v=vs.mean()+240; u=us.mean()+330
dep=d[int(v),int(u)]
print('pan rim centroid px',v,u,'-> world x=%.3f y=%.3f'%(-0.2+(v-240)*2.03/579.4,(u-320)*2.03/579.4))
print('rim z range', sub[(sub>0.955)&(sub<0.985)].min(), sub[(sub>0.955)&(sub<0.985)].max())
print('pan interior z at centroid', z[int(v),int(u)])
print('burner center (-0.057,0.207), plate top 0.930')
"; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/frontview.png

# openrua op 34
timeout 600 python3 -u -c "
from rob import *
r=Robot()
r.move_joints([0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854], 4.0)
print('joints', np.round(r.arm_q(),3))
" && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agentview.png
