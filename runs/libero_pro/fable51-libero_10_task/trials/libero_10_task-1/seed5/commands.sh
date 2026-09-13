#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ---; timeout 20 ros2 node list

# openrua op 3
timeout 30 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 4
python3 -c "
import cv2
im=cv2.imread('robot0_robotview.png')
cv2.imwrite('crop_can1.png', cv2.resize(im[30:150,330:430],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop_can2.png', cv2.resize(im[50:130,120:200],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im2=cv2.imread('robot0_eye_in_hand.png')
cv2.imwrite('crop_can3.png', cv2.resize(im2[250:360,430:560],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop_can4.png', cv2.resize(im2[260:370,180:310],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|translation|x:|y:|z:|w:" | head -80; echo; timeout 60 python3 tools/perception/px2world.py robot0_robotview 390 200; timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 250 170

# openrua op 6
timeout 60 python3 -c "
import rclpy
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('tfdump')
b=Buffer(); TransformListener(b,n)
import time
for _ in range(20): rclpy.spin_once(n,timeout_sec=0.2)
print(b.all_frames_as_yaml())
for f in ['panda_link0','panda_hand','robot0_eye_in_hand_optical_frame','robot0_robotview_optical_frame','agentview_optical_frame']:
    try:
        t=b.lookup_transform('world',f,rclpy.time.Time()).transform
        print(f, round(t.translation.x,4),round(t.translation.y,4),round(t.translation.z,4), 'q',round(t.rotation.x,4),round(t.rotation.y,4),round(t.rotation.z,4),round(t.rotation.w,4))
    except Exception as e: print(f,'ERR',e)
"

# openrua op 7
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+info+TF for a camera and back-project pixels to world.

Usage: python3 scene.py <camera> [u v]...   -> prints world xyz per pixel
       also saves <camera>_pts.npy (HxWx3 world coords) for offline use.
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, typ, timeout=20.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def cloud(cam):
    node = rclpy.create_node("scene_" + cam)
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    color = grab(node, f"/{cam}/color/image_raw", Image)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    img = br.imgmsg_to_cv2(color, "bgr8")
    d = br.imgmsg_to_cv2(depth, "passthrough").astype(np.float64)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time()).transform
    R = quat_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    T = np.array([t.translation.x, t.translation.y, t.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = d.shape
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    pc = np.stack([(us - cx) * d / fx, (vs - cy) * d / fy, d], -1)
    pw = pc @ R.T + T
    node.destroy_node()
    return img, d, pw


if __name__ == "__main__":
    cam = sys.argv[1]
    rclpy.init()
    img, d, pw = cloud(cam)
    cv2.imwrite(f"{cam}.png", img)
    np.save(f"{cam}_pts.npy", pw)
    px = [int(x) for x in sys.argv[2:]]
    for u, v in zip(px[::2], px[1::2]):
        print(u, v, "->", np.round(pw[v, u], 4), "depth", round(d[v, u], 4))
    rclpy.shutdown()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 scene.py birdview 320 240 380 285 285 228 && timeout 120 python3 scene.py robot0_eye_in_hand 250 170 495 300 240 320 && python3 -c "
import numpy as np
pw=np.load('birdview_pts.npy')
z=pw[:,:,2]
# table height: mode of z in table region
tz=z[150:350,150:500]
print('table z median', np.median(tz), np.percentile(tz,[5,50,95]))
"

# openrua op 9
python3 -c "
import numpy as np, cv2
for cam in ['robot0_eye_in_hand','birdview']:
    pw=np.load(cam+'_pts.npy'); img=cv2.imread(cam+'.png')
    z=pw[:,:,2]
    mask=((z>0.437)&(z<0.75)&np.isfinite(z)).astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
    print('==',cam)
    for i in range(1,n):
        if stats[i,4]<40: continue
        m=lab==i
        p=pw[m]
        print(f'comp {i}: px bbox {stats[i,:4]} area {stats[i,4]} centroid px {cent[i].round(0)}  world x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] mean {p.mean(0).round(3)}')
"


# openrua op 10
timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | tail -25

# openrua op 11
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot helpers: joints, FK, IK, trajectory, gripper (clients built once)."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(20); self.grip.wait_for_server(20)
        self.ik.wait_for_service(20); self.fk.wait_for_service(20)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def fk_hand(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(q if q is not None else self.arm_q())
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z]), \
            np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def tcp_world(self, q=None):
        p, quat = self.fk_hand(q)
        R = quat_R(*quat)
        return p + TCP * R[:, 2], quat

    def solve_ik(self, pos, quat, seed=None, at_tcp=True, timeout=60):
        pos = np.array(pos, float)
        if at_tcp:
            pos = pos - TCP * quat_R(*quat)[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = pos
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(seed if seed is not None else self.arm_q())
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None, (None if r is None else r.error_code.val)
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[n] for n in ARM], 1

    def move_joints(self, q, seconds=3.0, timeout=600):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result()
        code = r.result.error_code if r else None
        err = float(np.max(np.abs(np.array(self.arm_q()) - np.array(q))))
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q, code = self.solve_ik(pos, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED code={code} for {np.round(pos,3)}", flush=True)
            return None
        c, err = self.move_joints(q, seconds)
        tcp, _ = self.tcp_world()
        print(f"  move -> traj code={c} jerr={err:.4f} tcp_now={np.round(tcp,4)} target={np.round(pos,4)}", flush=True)
        return tcp

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result()
        f = self.fingers()
        print(f"  gripper({width}) reached={r.result.reached_goal if r else None} stalled={r.result.stalled if r else None} fingers={f}", flush=True)
        return f
OPENRUA_EOF

# openrua op 12
timeout 200 python3 -c "
from robot import *
r=Robot()
print('arm', np.round(r.arm_q(),3))
print('hand fk', r.fk_hand())
print('tcp', r.tcp_world())
# IK at current world hand pose
p,qt=r.fk_hand()
print('IK world coords ->', r.solve_ik(p,qt,at_tcp=False))
print('IK base coords ->', r.solve_ik(p+np.array([0.51,0,-0.42]),qt,at_tcp=False))
"

# openrua op 13
timeout 200 python3 -c "
from robot import *
r=Robot()
q=[0.058462119889661025, -0.16130609283377487, -0.05791992799008485, -2.4445840518080595, -0.011620247273772274, 2.226700917731669, 0.008905361657217602]
print('fk of IK sol', r.fk_hand(q))
# now test IK at the pregrasp poses
for name,pos in [('soup',(-0.08,-0.17,0.60)),('butter',(0.047,0.066,0.60)),('basket',(0.005,0.25,0.70)),('soup_grasp',(-0.08,-0.17,0.475)),('butter_grasp',(0.047,0.066,0.442))]:
    sol,c=r.solve_ik(pos,(1,0,0,0))
    print(name, c, None if sol is None else np.round(sol,3), None if sol is None else np.round(r.tcp_world(sol)[0],4))
"

# openrua op 14
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "        pos = np.array(pos, float)\\n        if at_tcp:", "new_string": "        pos = np.array(pos, float)\\n        quat = [float(v) for v in quat]\\n        if at_tcp:", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 15
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "        p.position.x, p.position.y, p.position.z = pos\\n        p.orientation.x", "new_string": "        p.position.x, p.position.y, p.position.z = [float(v) for v in pos]\\n        p.orientation.x", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 16
timeout 300 python3 -c "
from robot import *
r=Robot()
q=[0.058462119889661025, -0.16130609283377487, -0.05791992799008485, -2.4445840518080595, -0.011620247273772274, 2.226700917731669, 0.008905361657217602]
print('fk of IK sol', r.fk_hand(q))
for name,pos in [('soup',(-0.08,-0.17,0.60)),('butter',(0.047,0.066,0.60)),('basket',(0.005,0.25,0.70)),('soup_grasp',(-0.08,-0.17,0.475)),('butter_grasp',(0.047,0.066,0.442))]:
    sol,c=r.solve_ik(pos,(1,0,0,0))
    print(name, c, None if sol is None else np.round(sol,3), None if sol is None else np.round(r.tcp_world(sol)[0],4))
"

# openrua op 17
timeout 300 python3 -c "
from robot import *
from scipy.spatial.transform import Rotation as Rot
r=Robot()
for name,pos in [('soup',(-0.08,-0.17,0.60)),('basket',(0.005,0.25,0.70))]:
    for qt in [(1,0,0,0), tuple(Rot.from_quat([1,0,0,0]).__mul__(Rot.from_euler('z',45,degrees=True)).as_quat()), tuple((Rot.from_quat([1,0,0,0])*Rot.from_euler('z',-45,degrees=True)).as_quat())]:
        sol,c=r.solve_ik(pos,qt)
        if sol is None: print(name,'fail',c); continue
        p,q=r.fk_hand(sol)
        print(name, 'req',np.round(qt,3),'-> hand quat',np.round(q,3), 'euler xyz',np.round(Rot.from_quat(q).as_euler('xyz',degrees=True),1), 'tcp',np.round(r.tcp_world(sol)[0],3))
"

# openrua op 18
python3 -c "
import numpy as np
for cam in ['robot0_eye_in_hand','birdview']:
    pw=np.load(cam+'_pts.npy')
    m=(pw[:,:,2]>0.488)&(pw[:,:,2]<0.51)&(pw[:,:,0]>-0.13)&(pw[:,:,0]<-0.03)&(pw[:,:,1]>-0.22)&(pw[:,:,1]<-0.12)
    p=pw[m]; print(cam,'soup lid n',len(p),'center',p[:,:2].mean(0).round(4),'x range',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3))
    m=(pw[:,:,2]>0.443)&(pw[:,:,2]<0.46)&(pw[:,:,0]>-0.0)&(pw[:,:,0]<0.1)&(pw[:,:,1]>0.03)&(pw[:,:,1]<0.1)
    p=pw[m]; print(cam,'butter top n',len(p),'center',p[:,:2].mean(0).round(4),'x range',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3))
    m=(pw[:,:,2]>0.60)&(pw[:,:,2]<0.64)&(pw[:,:,1]>0.1)&(pw[:,:,1]<0.4)&(pw[:,:,0]>-0.2)&(pw[:,:,0]<0.2)
    p=pw[m]; print(cam,'basket rim n',len(p),'center',p[:,:2].mean(0).round(4),'x range',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3))
"

# openrua op 19
mkdir -p "$(dirname /workspace/task.py)"
cat > /workspace/task.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick alphabet soup and butter, place both in the basket."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from robot import Robot
import scene

# desired panda_hand orientation: z down, fingers along world y
HAND_DOWN = np.array([1.0, 0.0, 0.0, 0.0])
# IK target link is yawed 45deg from panda_hand: pre-rotate the request
IK_Q = (Rot.from_quat(HAND_DOWN) * Rot.from_euler("z", -45, degrees=True)).as_quat()

TABLE_Z = 0.425
BASKET = np.array([0.005, 0.25])
BASKET_DROP_Z = 0.72

OBJECTS = {
    "soup":   dict(xy=np.array([-0.078, -0.174]), top=0.50, grasp_z=0.475, lid=(0.488, 0.515)),
    "butter": dict(xy=np.array([0.047, 0.067]),  top=0.449, grasp_z=0.442, lid=(0.443, 0.46)),
}


def refine(r, xy, lid):
    """Look straight down from the eye-in-hand camera; centroid of the top face."""
    img, d, pw = scene.cloud("robot0_eye_in_hand")
    z = pw[:, :, 2]
    m = (z > lid[0]) & (z < lid[1]) & (np.abs(pw[:, :, 0] - xy[0]) < 0.06) & (np.abs(pw[:, :, 1] - xy[1]) < 0.06)
    p = pw[m]
    if len(p) < 50:
        print(f"  refine: only {len(p)} top points, keeping {xy}", flush=True)
        return xy
    c = p[:, :2].mean(0)
    print(f"  refine: n={len(p)} top-face centroid {np.round(c,4)} (prior {xy}) "
          f"x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}]", flush=True)
    if np.linalg.norm(c - xy) > 0.03:
        print("  refine: shift >3cm, suspicious; averaging", flush=True)
        c = (c + xy) / 2
    return c


def pick_place(r, name):
    o = OBJECTS[name]
    xy = o["xy"]
    print(f"== {name}: open gripper", flush=True)
    r.gripper(0.04)
    print(f"== {name}: pre-grasp above", flush=True)
    r.move_tcp([xy[0], xy[1], 0.60], IK_Q, 4.0)
    xy = refine(r, xy, o["lid"])
    r.move_tcp([xy[0], xy[1], 0.60], IK_Q, 1.5)
    print(f"== {name}: descend", flush=True)
    r.move_tcp([xy[0], xy[1], o["top"] + 0.02], IK_Q, 2.0)
    r.move_tcp([xy[0], xy[1], o["grasp_z"]], IK_Q, 2.0)
    print(f"== {name}: close", flush=True)
    f = r.gripper(0.0)
    gap = abs(f[0]) + abs(f[1])
    print(f"  finger gap {gap:.4f}", flush=True)
    if gap < 0.008:
        print("  GRASP FAILED (closed on air)", flush=True)
        return False
    print(f"== {name}: lift", flush=True)
    r.move_tcp([xy[0], xy[1], 0.72], IK_Q, 3.0)
    f = r.fingers(); print(f"  fingers after lift {f}", flush=True)
    print(f"== {name}: over basket", flush=True)
    r.move_tcp([BASKET[0], BASKET[1], BASKET_DROP_Z], IK_Q, 4.0)
    f = r.fingers(); print(f"  fingers over basket {f}", flush=True)
    print(f"== {name}: release", flush=True)
    r.gripper(0.04)
    r.move_tcp([BASKET[0], BASKET[1], 0.78], IK_Q, 1.5)
    return True


if __name__ == "__main__":
    names = sys.argv[1:] or ["soup", "butter"]
    r = Robot()
    for n in names:
        ok = pick_place(r, n)
        print(f"#### {n}: {'OK' if ok else 'FAIL'}", flush=True)
        if not ok:
            break
    print("#### DONE", flush=True)
OPENRUA_EOF

# openrua op 20
nohup python3 -u task.py soup > task_soup.log 2>&1 &
sleep 45; cat /workspace/task_soup.log

# openrua op 21
sleep 20; cat /workspace/task_soup.log; echo; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 22
pkill -f task.py; sleep 1; cat /workspace/task_soup.log | tail -5; timeout 60 python3 tools/perception/cam_snap.py agentview snap1.png

# openrua op 23
cd /workspace; tail -5 task_soup.log; pgrep -fa task.py; timeout 60 python3 tools/perception/cam_snap.py agentview snap1.png

# openrua op 24
cd /workspace; timeout 600 python3 -u -c "
from robot import *
import time
r=Robot()
print('start', np.round(r.arm_q(),3))
home=[0.0,-0.161,0.0,-2.445,0.0,2.227,0.785]
t0=time.time()
code,err=r.move_joints(home,4.0)
print('code',code,'err',err,'wall',round(time.time()-t0,1))
for i in range(5):
    q=r.arm_q(); print(i, np.round(np.array(q)-np.array(home),3))
"


# openrua op 25
cd /workspace; timeout 600 python3 -u -c "
from robot import *
import time
r=Robot()
home=[0.0,-0.161,0.0,-2.445,0.0,2.227,0.785]
for k in range(3):
    t0=time.time()
    code,err=r.move_joints(home,3.0)
    print('try',k,'code',code,'err',round(err,4),'wall',round(time.time()-t0,1), np.round(np.array(r.arm_q())-np.array(home),3))
    if err<0.01: break
"


# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "    def move_joints(self, q, seconds=3.0, timeout=600):\\n        goal = FollowJointTrajectory.Goal()", "new_string": "    def move_joints(self, q, seconds=3.0, timeout=600, tol=0.01, tries=4):\\n        \\"\\"\\"Send the goal; the controller often stops short (code -5) on the\\n        first pass, so re-send the same goal until the joints converge.\\"\\"\\"\\n        for k in range(tries):\\n            code, err = self._send_traj(q, seconds, timeout)\\n            if err < tol:\\n                return code, err\\n            print(f\\"    retry {k}: code={code} jerr={err:.4f}\\", flush=True)\\n            seconds = max(1.5, seconds * 0.6)\\n        return code, err\\n\\n    def _send_traj(self, q, seconds, timeout):\\n        goal = FollowJointTrajectory.Goal()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "        c, err = self.move_joints(q, seconds)", "new_string": "        delta = float(np.max(np.abs(np.array(q) - np.array(self.arm_q()))))\\n        seconds = max(seconds, delta / 0.5)\\n        c, err = self.move_joints(q, seconds)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 28
cd /workspace; nohup python3 -u task.py soup > task_soup.log 2>&1 &
sleep 90; cat /workspace/task_soup.log

# openrua op 29
sleep 60; cat /workspace/task_soup.log | tail -12

# openrua op 30
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py agentview snap2.png && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png

# openrua op 31
cd /workspace; nohup python3 -u task.py butter > task_butter.log 2>&1 &
sleep 120; cat /workspace/task_butter.log

# openrua op 32
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py agentview snap3.png && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png

# openrua op 33
cd /workspace; timeout 120 python3 scene.py robot0_eye_in_hand 320 285 320 350 >/dev/null; timeout 120 python3 scene.py birdview 380 290 >/dev/null; python3 -c "
import numpy as np
pw=np.load('robot0_eye_in_hand_pts.npy')
inb=(np.abs(pw[:,:,0]-0.005)<0.05)&(np.abs(pw[:,:,1]-0.25)<0.06)&(pw[:,:,2]>0.43)&(pw[:,:,2]<0.60)
p=pw[inb]; print('eih: points inside basket footprint below rim, n',len(p),'z range',p[:,2].min().round(3),p[:,2].max().round(3))
top=p[p[:,2]>p[:,2].max()-0.01]; print(' top-most cluster center',top.mean(0).round(3))
pw=np.load('birdview_pts.npy')
for name,x,y in [('soup_spot',-0.078,-0.174),('butter_spot',0.047,0.067)]:
    m=(np.abs(pw[:,:,0]-x)<0.03)&(np.abs(pw[:,:,1]-y)<0.02)
    print(name,'max z now',pw[m][:,2].max().round(3),'(table=0.425)')
inb=(np.abs(pw[:,:,0]-0.005)<0.05)&(np.abs(pw[:,:,1]-0.25)<0.06)&(pw[:,:,2]>0.43)&(pw[:,:,2]<0.60)
p=pw[inb]; print('birdview inside basket n',len(p),'z',p[:,2].min().round(3),p[:,2].max().round(3))
"

# openrua op 34
pgrep -fa "task.py|scene.py" | grep -v pgrep || echo "no background tasks"
