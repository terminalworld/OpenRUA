#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ======; cat /workspace/tools/perception/cam_snap.py; echo ======; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ======; cat tools/action/gripper_cmd.py; echo ======; cat tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ====; ros2 action list; echo ====; ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ====; ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do python3 tools/perception/cam_snap.py $c; done

# openrua op 5
ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40; echo ===; timeout 5 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 6
timeout 10 ros2 topic echo /tf --once 2>&1 | head -60

# openrua op 7
mkdir -p "$(dirname /workspace/camlib.py)"
cat > /workspace/camlib.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera once; convert pixels to world."""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def _grab(node, topic, msg_type, timeout=20.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Cam:
    def __init__(self, node, name, tfbuf):
        self.name = name
        bridge = CvBridge()
        self.color = bridge.imgmsg_to_cv2(_grab(node, f"/{name}/color/image_raw", Image), "bgr8")
        self.depth = bridge.imgmsg_to_cv2(_grab(node, f"/{name}/depth/image_raw", Image), "passthrough").astype(np.float32)
        info = _grab(node, f"/{name}/color/camera_info", CameraInfo)
        self.fx, self.fy, self.cx, self.cy = info.k[0], info.k[4], info.k[2], info.k[5]
        frame = f"{name}_optical_frame"
        import time
        end = time.time() + 10
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
            if tfbuf.can_transform("world", frame, rclpy.time.Time()):
                break
        t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        self.T = np.eye(4)
        self.T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        self.T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]

    def px2world(self, u, v):
        z = float(self.depth[int(v), int(u)])
        if not np.isfinite(z) or z <= 0:
            return None
        p = np.array([(u - self.cx) * z / self.fx, (v - self.cy) * z / self.fy, z, 1.0])
        return (self.T @ p)[:3]

    def cloud(self, mask):
        vs, us = np.nonzero(mask)
        z = self.depth[vs, us]
        ok = np.isfinite(z) & (z > 0)
        us, vs, z = us[ok], vs[ok], z[ok]
        P = np.stack([(us - self.cx) * z / self.fx, (vs - self.cy) * z / self.fy, z, np.ones_like(z)])
        return (self.T @ P)[:3].T


def grab(names):
    rclpy.init()
    node = rclpy.create_node("camlib")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    cams = {n: Cam(node, n, tfbuf) for n in names}
    node.destroy_node()
    rclpy.shutdown()
    return cams


if __name__ == "__main__":
    name = sys.argv[1]
    cams = grab([name])
    c = cams[name]
    cv2.imwrite(f"{name}.png", c.color)
    np.save(f"{name}_depth.npy", c.depth)
    np.save(f"{name}_T.npy", c.T)
    np.save(f"{name}_K.npy", np.array([c.fx, c.fy, c.cx, c.cy]))
    for a in sys.argv[2:]:
        u, v = map(int, a.split(","))
        print(a, c.px2world(u, v))
OPENRUA_EOF

# openrua op 8
python3 camlib.py agentview 157,385 370,245 370,265 525,290 520,220 300,400 && python3 camlib.py birdview 275,312 333,272 383,285 320,300

# openrua op 9
python3 -c "
import numpy as np, cv2
from camlib import *
import rclpy
d=np.load('agentview_depth.npy'); T=np.load('agentview_T.npy'); fx,fy,cx,cy=np.load('agentview_K.npy')
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
z=d
P=np.stack([(us-cx)*z/fx,(vs-cy)*z/fy,z,np.ones_like(z)]).reshape(4,-1)
Wp=(T@P)[:3].T.reshape(H,W,3)
mask=(Wp[...,2]>0.435)&(Wp[...,2]<0.7)&(Wp[...,0]>-0.25)&(Wp[...,0]<0.35)&(np.abs(Wp[...,1])<0.45)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    pts=Wp[lab==i]
    print(i, 'px',cent[i].round(0), 'n',stats[i,4], 'xy min',pts[:,:2].min(0).round(3),'max',pts[:,:2].max(0).round(3),'zmax',pts[:,2].max().round(3), 'mean',pts.mean(0).round(3))
"


# openrua op 10
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); fx,fy,cx,cy=np.load('birdview_K.npy')
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
z=d
P=np.stack([(us-cx)*z/fx,(vs-cy)*z/fy,z,np.ones_like(z)]).reshape(4,-1)
Wp=(T@P)[:3].T.reshape(H,W,3)
print('table z sample', Wp[300,320], Wp[330,250])
mask=(Wp[...,2]>0.435)&(Wp[...,2]<0.66)&(Wp[...,0]>-0.3)&(Wp[...,0]<0.35)&(np.abs(Wp[...,1])<0.45)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<5: continue
    pts=Wp[lab==i]
    print(i, 'px',cent[i].round(0), 'n',stats[i,4], 'xy min',pts[:,:2].min(0).round(3),'max',pts[:,:2].max(0).round(3),'zmax',pts[:,2].max().round(3), 'mean',pts.mean(0).round(3))
"


# openrua op 11
python3 -c "
import numpy as np
d=np.load('agentview_depth.npy'); T=np.load('agentview_T.npy'); fx,fy,cx,cy=np.load('agentview_K.npy')
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
z=d
P=np.stack([(us-cx)*z/fx,(vs-cy)*z/fy,z,np.ones_like(z)]).reshape(4,-1)
Wp=(T@P)[:3].T.reshape(H,W,3)
m=(Wp[...,2]>0.512)&(Wp[...,0]>-0.2)&(Wp[...,0]<0)&(Wp[...,1]>0.0)&(Wp[...,1]<0.15)
pts=Wp[m]; print('can top n',len(pts),'min',pts.min(0).round(3),'max',pts.max(0).round(3),'mean',pts.mean(0).round(3))
m=(Wp[...,2]>0.447)&(Wp[...,0]>0.05)&(Wp[...,0]<0.2)&(Wp[...,1]>-0.25)&(Wp[...,1]<-0.15)
pts=Wp[m]; print('cheese top n',len(pts),'min',pts.min(0).round(3),'max',pts.max(0).round(3),'mean',pts.mean(0).round(3))
"


# openrua op 12
python3 -c "
import numpy as np
d=np.load('agentview_depth.npy'); T=np.load('agentview_T.npy'); fx,fy,cx,cy=np.load('agentview_K.npy')
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
z=d
P=np.stack([(us-cx)*z/fx,(vs-cy)*z/fy,z,np.ones_like(z)]).reshape(4,-1)
Wp=(T@P)[:3].T.reshape(H,W,3)
m=(Wp[...,2]>0.512)&(Wp[...,2]<0.53)&(Wp[...,0]>-0.2)&(Wp[...,0]<0)&(Wp[...,1]>0.0)&(Wp[...,1]<0.15)
pts=Wp[m]; print('can top n',len(pts),'min',pts.min(0).round(3),'max',pts.max(0).round(3),'mean',pts.mean(0).round(3))
"


# openrua op 13
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable clients: joint state, IK, FK, trajectory, gripper. World<->base."""
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load((Path(__file__).parent / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # /tf world->panda_link0
TCP = float(M["hand"]["tcp_offset_m"])
TOP_DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z down, fingers close along world y


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_quat(yaw):
    """Top-down hand orientation rotated by yaw about world z."""
    # q = Rz(yaw) * Rx(pi)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # Rz(yaw) = (0,0,s,c); Rx(pi) = (1,0,0,0); product (w1w2 - v1.v2, ...)
    # q1=(x=0,y=0,z=s,w=c), q2=(1,0,0,0)
    x = c * 1 + 0 + (0 * 0 - s * 0)
    y = 0 + 0 + (s * 1 - 0 * 0)
    z = c * 0 + s * 0 + (0 * 0 - 0 * 1)
    w = c * 0 - (0 * 1 + 0 + s * 0)
    # simplified: (x,y,z,w) = (c, s, 0, 0)
    return (float(c), float(s), 0.0, 0.0)


class Robot:
    def __init__(self, name="robot_ctl"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.traj.wait_for_server(10)
        self.grip.wait_for_server(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        end = time.time() + 15
        while "m" not in self._js and time.time() < end:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def solve_ik(self, pos_world, quat, seed=None, at_tcp=True, timeout=60):
        pos = np.array(pos_world, dtype=float)
        if at_tcp:
            R = quat_to_R(*quat)
            pos = pos - TCP * R[:, 2]
        pb = pos - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed_q = seed if seed is not None else self.arm_q()
        js = JointState()
        js.name = list(ARM)
        js.position = [float(v) for v in seed_q]
        req.ik_request.robot_state.joint_state = js
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            raise RuntimeError(f"IK failed code={res.error_code.val} for world {pos_world}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in ARM]

    def fk_hand(self, q=None):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        js = JointState()
        js.name = list(ARM)
        js.position = [float(v) for v in (q if q is not None else self.arm_q())]
        req.robot_state.joint_state = js
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, quat

    def tcp_world(self, q=None):
        pos, quat = self.fk_hand(q)
        return pos + TCP * quat_to_R(*quat)[:, 2], quat

    def move(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = list(waypoints or []) + [q]
        for i, wq in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move: code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def goto(self, pos_world, quat=TOP_DOWN, seconds=3.0, seed=None):
        q = self.solve_ik(pos_world, quat, seed=seed)
        code, err = self.move(q, seconds)
        tcp, _ = self.tcp_world()
        print(f"  tcp now {tcp.round(4)} target {np.array(pos_world).round(4)}", flush=True)
        return q
OPENRUA_EOF

# openrua op 14
timeout 120 python3 -c "
from robot import *
r=Robot()
print('q',np.round(r.arm_q(),3))
print('gap',r.finger_gap())
pos,quat=r.fk_hand(); print('hand world',pos.round(4),np.round(quat,4))
tcp,_=r.tcp_world(); print('tcp world',tcp.round(4))
for name,p in [('cheese_pre',(0.115,-0.193,0.60)),('cheese_grasp',(0.115,-0.193,0.44)),('can_pre',(-0.115,0.0725,0.65)),('can_grasp',(-0.115,0.0725,0.475)),('basket',(0.0,0.255,0.72))]:
    try:
        q=r.solve_ik(p,TOP_DOWN); print(name,'ok',np.round(q,3)); print('   fk tcp',r.tcp_world(q)[0].round(4))
    except Exception as e: print(name,'FAIL',e)
"

# openrua op 15
timeout 200 python3 -c "
from robot import *
r=Robot()
pos,quat=r.fk_hand(); print('hand world',pos.round(4),np.round(quat,4))
try:
    q=r.solve_ik(pos,quat,at_tcp=False); print('current pose IK ok',np.round(q,3))
except Exception as e: print('FAIL',e)
# try nearby poses
for dz in [0.0,-0.1,-0.2,-0.3]:
    p=pos+np.array([0.1,0,dz])
    try:
        q=r.solve_ik(p,TOP_DOWN,at_tcp=False); print('ok',p.round(3),np.round(q,3))
    except Exception as e: print('FAIL',p.round(3),e)
"

# openrua op 16
timeout 300 python3 -c "
from robot import *
r=Robot()
seeds={'ready':[0,-0.785,0,-2.356,0,1.571,0.785],'ext':[0,0.3,0,-1.8,0,2.1,0.785],'ext2':[0,0.6,0,-1.5,0,2.1,0.0]}
for sn,s in seeds.items():
    print('seed',sn,'fk tcp',r.tcp_world(s)[0].round(3))
    for name,p in [('cheese_pre',(0.115,-0.193,0.60)),('cheese_grasp',(0.115,-0.193,0.44)),('can_pre',(-0.115,0.0725,0.65)),('can_grasp',(-0.115,0.0725,0.475)),('basket',(0.0,0.255,0.72))]:
        try:
            q=r.solve_ik(p,TOP_DOWN,seed=s); print(' ',name,'ok',np.round(q,3))
        except Exception as e: print(' ',name,'FAIL',e)
"

# openrua op 17
python3 -c "
import numpy as np
def tf(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
def fk(q):
    A=[0,0,0,0.0825,-0.0825,0,0.088,0]; D=[0.333,0,0.316,0,0.384,0,0,0.107]; AL=[0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2,0]
    T=np.eye(4)
    for i in range(7): T=T@tf(A[i],D[i],AL[i],q[i])
    T=T@tf(A[7],D[7],AL[7],0)
    return T
for q in [[0,-0.161,0,-2.445,0,2.227,0.785],[0,-0.785,0,-2.356,0,1.571,0.785]]:
    T=fk(q); print('flange pos base',T[:3,3].round(4)); print('z axis',T[:3,2].round(3))
"


# openrua op 18
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # /tf world->panda_link0", "new_string": "# Verified empirically: /compute_fk and /compute_ik on this machine take and\\n# return poses in the WORLD frame (analytic Panda FK of the current config\\n# matched the service output only after adding /tf world->panda_link0).\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 19
timeout 300 python3 -c "
from robot import *
r=Robot()
tcp,_=r.tcp_world(); print('tcp world',tcp.round(4))
for name,p in [('cheese_pre',(0.115,-0.193,0.60)),('cheese_grasp',(0.115,-0.193,0.44)),('can_pre',(-0.115,0.0725,0.65)),('can_grasp',(-0.115,0.0725,0.475)),('basket',(0.0,0.255,0.72))]:
    try:
        q=r.solve_ik(p,TOP_DOWN); print(name,'ok',np.round(q,3),'fk tcp',r.tcp_world(q)[0].round(4))
    except Exception as e: print(name,'FAIL',e)
"

# openrua op 20
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick an object with a top-down grasp and drop it in the basket.

Usage: python3 -u pick_place.py <x> <y> <grasp_z_tcp> <yaw> [safe_z=0.65]
"""
import sys
import numpy as np
from robot import Robot, TOP_DOWN, yaw_quat, GRIP

BASKET = np.array([-0.005, 0.255, 0.76])  # TCP drop point above basket interior


def main():
    x, y, gz, yaw = map(float, sys.argv[1:5])
    safe_z = float(sys.argv[5]) if len(sys.argv) > 5 else 0.65
    quat = yaw_quat(yaw)
    r = Robot("pick_place")
    print("start tcp", r.tcp_world()[0].round(4), "gap", round(r.finger_gap(), 4), flush=True)

    print("1. open gripper", flush=True)
    r.gripper(GRIP["open_m"])

    print("2. pre-grasp above target", flush=True)
    r.goto((x, y, safe_z), quat, seconds=4.0)

    print("3. descend to grasp height", flush=True)
    r.goto((x, y, gz), quat, seconds=3.0)

    print("4. close gripper", flush=True)
    gap = r.gripper(GRIP["closed_m"])
    if gap < 0.005:
        print("GRASP FAILED: fingers closed on air (gap=%.4f)" % gap, flush=True)
        r.gripper(GRIP["open_m"])
        r.goto((x, y, safe_z), quat, seconds=3.0)
        sys.exit(2)
    print("grasp evidence: gap=%.4f" % gap, flush=True)

    print("5. lift", flush=True)
    r.goto((x, y, safe_z + 0.05), quat, seconds=3.0)
    gap = r.finger_gap()
    print("gap after lift %.4f" % gap, flush=True)
    if gap < 0.005:
        print("LOST OBJECT during lift", flush=True)
        sys.exit(3)

    print("6. move over basket", flush=True)
    r.goto(tuple(BASKET), TOP_DOWN, seconds=4.0)
    gap = r.finger_gap()
    print("gap over basket %.4f" % gap, flush=True)

    print("7. release", flush=True)
    r.gripper(GRIP["open_m"])

    print("8. retreat up", flush=True)
    r.goto((BASKET[0], BASKET[1], BASKET[2] + 0.08), TOP_DOWN, seconds=3.0)
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 21
nohup python3 -u pick_place.py 0.112 -0.193 0.44 0.0 0.65 > cheese.log 2>&1 &
sleep 45; cat /workspace/cheese.log

# openrua op 22
sleep 30; cat /workspace/cheese.log

# openrua op 23
python3 tools/perception/cam_snap.py agentview agentview2.png; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png

# openrua op 24
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "    def move(self, q, seconds=3.0, waypoints=None):\\n        goal = FollowJointTrajectory.Goal()", "new_string": "    def settle(self, max_reads=40):\\n        \\"\\"\\"Re-read joints until two consecutive reads agree (controller lag).\\"\\"\\"\\n        prev = np.array(self.arm_q())\\n        for _ in range(max_reads):\\n            cur = np.array(self.arm_q())\\n            if np.abs(cur - prev).max() < 1e-4:\\n                return cur\\n            prev = cur\\n        return prev\\n\\n    def move(self, q, seconds=3.0, waypoints=None, tol=0.01, retries=2):\\n        for attempt in range(retries + 1):\\n            code, err = self._move_once(q, seconds, waypoints)\\n            if err <= tol:\\n                break\\n            print(f\\"  move: retry {attempt + 1} (err={err:.4f})\\", flush=True)\\n            waypoints = None\\n        return code, err\\n\\n    def _move_once(self, q, seconds, waypoints):\\n        goal = FollowJointTrajectory.Goal()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 25
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "        code = res.result().result.error_code\\n        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()", "new_string": "        code = res.result().result.error_code\\n        err = np.abs(self.settle() - np.array(q)).max()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "        r = res.result().result\\n        gap = self.finger_gap()", "new_string": "        r = res.result().result\\n        gap = self.finger_gap()\\n        for _ in range(40):  # settle ticks\\n            g2 = self.finger_gap()\\n            if abs(g2 - gap) < 1e-5:\\n                break\\n            gap = g2", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
nohup python3 -u pick_place.py 0.112 -0.193 0.44 0.0 0.65 > cheese2.log 2>&1 &
sleep 60; cat /workspace/cheese2.log

# openrua op 28
sleep 40; cat /workspace/cheese2.log; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | head -20

# openrua op 29
timeout 120 python3 -c "
from robot import *
from moveit_msgs.srv import GetPositionFK
r=Robot()
def fk(link,q):
    req=GetPositionFK.Request(); req.header.frame_id=''; req.fk_link_names=[link]
    js=JointState(); js.name=list(ARM); js.position=[float(v) for v in q]; req.robot_state.joint_state=js
    fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
    p=fut.result().pose_stamped[0].pose
    return np.round([p.position.x,p.position.y,p.position.z],4), np.round([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],4)
q0=[0,-0.161,0,-2.445,0,2.227,0.785]
q1=[0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009]
for q in [q0,q1]:
    for l in ['panda_link8','panda_hand','panda_hand_tcp']:
        try: print(q[6], l, fk(l,q))
        except Exception as e: print(l,'err',e)
"

# openrua op 30
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)", "new_string": "        # /compute_ik solves for panda_link8 (verified via /compute_fk), which\\n        # is panda_hand rotated +45 deg about its z: q_link8 = q_hand * Rz(pi/4)\\n        q8 = quat_mul(quat, (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)))\\n        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 31
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "def yaw_quat(yaw):", "new_string": "def quat_mul(a, b):\\n    \\"\\"\\"Hamilton product a*b, quaternions as (x, y, z, w).\\"\\"\\"\\n    ax, ay, az, aw = a\\n    bx, by, bz, bw = b\\n    return (aw * bx + ax * bw + ay * bz - az * by,\\n            aw * by - ax * bz + ay * bw + az * bx,\\n            aw * bz + ax * by - ay * bx + az * bw,\\n            aw * bw - ax * bx - ay * by - az * bz)\\n\\n\\ndef yaw_quat(yaw):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 32
timeout 200 python3 -c "
from robot import *
r=Robot()
print('yaw_quat(0)',yaw_quat(0),'yaw_quat(pi/2)',yaw_quat(np.pi/2))
for name,p,yaw in [('cheese_grasp',(0.112,-0.193,0.44),0.0),('can_grasp',(-0.115,0.0725,0.475),0.0),('basket',(-0.005,0.255,0.76),0.0)]:
    q=r.solve_ik(p,yaw_quat(yaw)); pos,quat=r.fk_hand(q)
    print(name,'q',np.round(q,3),'hand quat',np.round(quat,3),'tcp',r.tcp_world(q)[0].round(4))
"

# openrua op 33
nohup python3 -u pick_place.py 0.112 -0.193 0.44 0.0 0.65 > cheese3.log 2>&1 &
sleep 75; cat /workspace/cheese3.log

# openrua op 34
sleep 40; cat /workspace/cheese3.log

# openrua op 35
python3 tools/perception/cam_snap.py agentview agentview3.png; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png

# openrua op 36
timeout 300 python3 -u -c "
from robot import *
r=Robot()
r.goto((-0.115,0.0725,0.70), TOP_DOWN, seconds=4.0)
print('final q', np.round(r.arm_q(),3))
" 2>&1 | tail -5; python3 camlib.py robot0_eye_in_hand 320,240 && python3 camlib.py agentview 370,245

# openrua op 37
python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand_depth.npy'); T=np.load('robot0_eye_in_hand_T.npy'); fx,fy,cx,cy=np.load('robot0_eye_in_hand_K.npy')
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
z=d
P=np.stack([(us-cx)*z/fx,(vs-cy)*z/fy,z,np.ones_like(z)]).reshape(4,-1)
Wp=(T@P)[:3].T.reshape(H,W,3)
print('cam T',T.round(3))
m=(Wp[...,2]>0.505)&(Wp[...,2]<0.54)&(np.abs(Wp[...,0]+0.115)<0.08)&(np.abs(Wp[...,1]-0.0725)<0.08)
pts=Wp[m]; print('can top n',len(pts),'min',pts.min(0).round(4),'max',pts.max(0).round(4),'mean',pts.mean(0).round(4))
print('center est', ((pts.min(0)+pts.max(0))/2).round(4))
# circle fit (algebraic) on xy
x,y=pts[:,0],pts[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('circle fit center',c[:2].round(4),'radius',R.round(4))
# also side wall points z 0.44..0.50
m2=(Wp[...,2]>0.44)&(Wp[...,2]<0.50)&(np.abs(Wp[...,0]+0.115)<0.08)&(np.abs(Wp[...,1]-0.0725)<0.08)
p2=Wp[m2]; print('wall n',len(p2),'min',p2.min(0).round(4),'max',p2.max(0).round(4))
"


# openrua op 38
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pick_place.py", "old_string": "    r.goto((x, y, gz), quat, seconds=3.0)\\n\\n    print(\\"4. close gripper\\", flush=True)", "new_string": "    r.goto((x, y, gz), quat, seconds=3.0)\\n    tcp = r.tcp_world()[0]\\n    if tcp[2] > gz + 0.01:\\n        print(\\"DESCENT BLOCKED at z=%.4f (fingers probably on top of object); retreating\\" % tcp[2], flush=True)\\n        r.goto((x, y, safe_z), quat, seconds=3.0)\\n        sys.exit(4)\\n\\n    print(\\"4. close gripper\\", flush=True)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 39
nohup python3 -u pick_place.py -0.114 0.073 0.475 0.0 0.70 > can1.log 2>&1 &
sleep 80; cat /workspace/can1.log

# openrua op 40
sleep 20; cat /workspace/can1.log | tail -4; python3 tools/perception/cam_snap.py agentview agentview4.png; python3 tools/perception/cam_snap.py sideview sideview4.png

# openrua op 41
timeout 200 python3 -c "
from robot import *
r=Robot()
q0=r.arm_q(); print('now',np.round(q0,3), r.tcp_world(q0)[0].round(4))
seed=q0
for z in [0.70,0.65,0.60,0.55,0.50,0.475]:
    q=r.solve_ik((-0.114,0.073,z),TOP_DOWN,seed=seed); print(z,np.round(q,3)); seed=q
print('direct from now:', np.round(r.solve_ik((-0.114,0.073,0.475),TOP_DOWN,seed=q0),3))
"

# openrua op 42
timeout 200 python3 -c "
from robot import *
r=Robot()
seed=[0.18,0.3,0.0,-2.4,0.0,2.7,0.97]
print('seed fk tcp', r.tcp_world(seed)[0].round(3))
for z in [0.80,0.70,0.65,0.60,0.55,0.50,0.475]:
    q=r.solve_ik((-0.114,0.073,z),TOP_DOWN,seed=seed); print(z,np.round(q,3)); seed=q
"

# openrua op 43
mkdir -p "$(dirname /workspace/can_pick.py)"
cat > /workspace/can_pick.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the tomato sauce can with a straight (Cartesian-waypoint) descent
from a planar arm configuration, then drop it in the basket."""
import sys
import numpy as np
from robot import Robot, TOP_DOWN, GRIP

X, Y = -0.114, 0.073
GZ = 0.475           # TCP grasp height (can mid-height; can spans 0.425..0.52)
HIGH = 0.80
NICE_SEED = [0.18, 0.3, 0.0, -2.4, 0.0, 2.7, 0.97]
BASKET = np.array([-0.005, 0.255, 0.76])


def chain(r, xy, zs, seed):
    qs = []
    for z in zs:
        seed = r.solve_ik((xy[0], xy[1], z), TOP_DOWN, seed=seed)
        qs.append(seed)
    return qs


def main():
    r = Robot("can_pick")
    print("start tcp", r.tcp_world()[0].round(4), "gap", round(r.finger_gap(), 4), flush=True)
    r.gripper(GRIP["open_m"])

    print("1. up to HIGH in current config", flush=True)
    r.goto((X, Y, HIGH), TOP_DOWN, seconds=3.0)

    print("2. reconfigure to planar config at HIGH", flush=True)
    q_high = r.solve_ik((X, Y, HIGH), TOP_DOWN, seed=NICE_SEED)
    print("   q_high", np.round(q_high, 3), flush=True)
    r.move(q_high, seconds=4.0)
    print("   tcp", r.tcp_world()[0].round(4), flush=True)

    print("3. straight descent to grasp height", flush=True)
    zs = [0.70, 0.65, 0.60, 0.55, 0.52, GZ]
    qs = chain(r, (X, Y), zs, q_high)
    r.move(qs[-1], seconds=6.0, waypoints=qs[:-1])
    tcp = r.tcp_world()[0]
    print("   tcp", tcp.round(4), flush=True)
    if tcp[2] > GZ + 0.01 or abs(tcp[0] - X) > 0.01 or abs(tcp[1] - Y) > 0.01:
        print("DESCENT OFF TARGET; retreating", flush=True)
        r.move(q_high, seconds=4.0, waypoints=qs[::-1][1:])
        sys.exit(4)

    print("4. close gripper", flush=True)
    gap = r.gripper(GRIP["closed_m"])
    if gap < 0.01:
        print("GRASP FAILED gap=%.4f" % gap, flush=True)
        r.gripper(GRIP["open_m"])
        r.move(q_high, seconds=4.0, waypoints=qs[::-1][1:])
        sys.exit(2)
    print("grasp evidence gap=%.4f" % gap, flush=True)

    print("5. straight lift", flush=True)
    up = qs[::-1][1:]  # back up through the same waypoints
    r.move(up[-1], seconds=5.0, waypoints=up[:-1])
    print("   tcp", r.tcp_world()[0].round(4), "gap", round(r.finger_gap(), 4), flush=True)
    if r.finger_gap() < 0.01:
        print("LOST OBJECT", flush=True)
        sys.exit(3)

    print("6. over basket", flush=True)
    r.goto(tuple(BASKET), TOP_DOWN, seconds=4.0)
    print("   gap", round(r.finger_gap(), 4), flush=True)

    print("7. release", flush=True)
    r.gripper(GRIP["open_m"])

    print("8. retreat", flush=True)
    r.goto((BASKET[0], BASKET[1], BASKET[2] + 0.08), TOP_DOWN, seconds=3.0)
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 44
nohup python3 -u can_pick.py > can2.log 2>&1 &
sleep 90; cat /workspace/can2.log

# openrua op 45
sleep 45; cat /workspace/can2.log | tail -8

# openrua op 46
python3 tools/perception/cam_snap.py agentview agentview5.png; python3 camlib.py robot0_eye_in_hand 320,240 >/dev/null; cp robot0_eye_in_hand.png eih5.png; python3 camlib.py agentview 1,1 > /dev/null; python3 -c "
import numpy as np
d=np.load('agentview_depth.npy'); T=np.load('agentview_T.npy'); fx,fy,cx,cy=np.load('agentview_K.npy')
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d,np.ones_like(d)]).reshape(4,-1)
Wp=(T@P)[:3].T.reshape(H,W,3)
# anything left on the table where the cheese / can were?
for name,(x,y) in {'cheese spot':(0.112,-0.193),'can spot':(-0.114,0.073)}.items():
    m=(np.abs(Wp[...,0]-x)<0.05)&(np.abs(Wp[...,1]-y)<0.05)&(Wp[...,2]>0.435)&(Wp[...,2]<0.7)
    print(name,'points above table:',m.sum())
"

# openrua op 47
pgrep -af "pick_place|can_pick" || echo "no pending motion scripts"
