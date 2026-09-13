#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -v parameter | head -40; echo ----; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); cv2.imwrite('crop_can.png', cv2.resize(im[170:290,490:640],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im2=cv2.imread('agentview.png'); cv2.imwrite('crop_bottle.png', cv2.resize(im2[180:330,190:290],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_can2.png', cv2.resize(im[185:245,200:255],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('birdview.png'); cv2.imwrite('crop_bird.png', cv2.resize(im[200:320,240:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 7
for p in "282 225" "267 281" "382 290" "290 264" "300 150"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 8
timeout 60 python3 -c "
import rclpy, numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
rclpy.init(); n=rclpy.create_node('tfq'); b=Buffer(); TransformListener(b,n)
import time
for _ in range(20): rclpy.spin_once(n,timeout_sec=0.2)
print(b.all_frames_as_string())
for f in ['panda_link0','panda_hand','birdview_optical_frame','agentview_optical_frame','robot0_eye_in_hand_optical_frame']:
    try:
        t=b.lookup_transform('world',f,Time()).transform
        print(f, t.translation.x,t.translation.y,t.translation.z, t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)
    except Exception as e: print(f,'ERR',e)
"

# openrua op 9
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 20 ros2 topic echo /birdview/color/camera_info --once | head -30

# openrua op 10
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); fx=579.4112549695428; cx,cy=320,240
# birdview optical frame at (-0.2,0,3.0), quat (0.707,0.707,0,0): R = rotation... compute
q=np.array([0.7071067811865476,0.7071067811865475,0,0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print(R)
def w(u,v):
    Z=d[v,u]; p=np.array([(u-cx)*Z/fx,(v-cy)*Z/fx,Z]); return R@p+np.array([-0.2,0,3.0])
H=np.zeros_like(d)
for v in range(0,480,1):
    Z=d[v,:]; us=np.arange(640)
    P=np.stack([(us-cx)*Z/fx,(v-cy)*Z/fx,Z],1)@R.T+np.array([-0.2,0,3.0])
    H[v]=P[:,2]
np.save('birdH.npy',H)
print('table z sample', H[300,200], H[350,450], H[150,350])
# objects: threshold above table
tab=np.median(H[280:340,150:230]); print('table',tab)
mask=(H>tab+0.02)&(H<tab+0.35)
import cv2
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i; zs=H[m]; vs,us=np.nonzero(m)
    c=w(int(cent[i][0]),int(cent[i][1]))
    print(i,'px',cent[i].round(1),'area',stats[i,4],'ztop',zs.max().round(3),'zmed',np.median(zs).round(3),'world xy',c[:2].round(3), 'bbox',stats[i,:4])
"


# openrua op 11
python3 -c "
import numpy as np
H=np.load('birdH.npy')
for v in [150,200,250,300,350]:
    print(v, [round(float(H[v,u]),3) for u in range(150,520,40)])
"


# openrua op 12
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Persistent helpers: joint state, FK, IK, trajectory, gripper. World<->base."""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = TRAJ["joints"]
BASE = np.array([-0.51, 0.0, 0.42])  # world position of panda_link0 (from TF)
TCP = M["hand"]["tcp_offset_m"]
DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z pointing down, fingers along world y


class Arm:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("arm_helper")
        self._js = None
        self.n.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.n, FollowJointTrajectory, TRAJ["port"])
        self.gr = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10) and self.gr.wait_for_server(10)
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.n, timeout_sec=t)

    def js(self):
        self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.js()
        return [j[k] for k in JOINTS]

    def fingers(self):
        j = self.js()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def hand_world(self):
        """FK: hand pose in world (pos, quat)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = self.arm_q()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def solve_ik(self, world_xyz, quat=DOWN, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        b = np.array(world_xyz) - BASE
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state.joint_state.name = JOINTS
        req.ik_request.robot_state.joint_state.position = seed or self.arm_q()
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK FAILED {None if r is None else r.error_code.val} for {world_xyz}", flush=True)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def traj(self, q, seconds=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  traj code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_to(self, world_xyz, quat=DOWN, seconds=3.0):
        q = self.solve_ik(world_xyz, quat)
        if q is None:
            return False
        self.traj(q, seconds)
        pos, _ = self.hand_world()
        print(f"  hand now {pos.round(4)} target {np.array(world_xyz).round(4)}", flush=True)
        return True

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}", flush=True)
        return f
OPENRUA_EOF

# openrua op 13
timeout 120 python3 -u -c "
from arm import *
a=Arm()
print('q',np.round(a.arm_q(),3)); print('fingers',a.fingers())
pos,quat=a.hand_world(); print('hand world',pos.round(4),np.round(quat,4))
q=a.solve_ik([-0.269,-0.164,0.70]); print('ik above soup',None if q is None else np.round(q,3))
"

# openrua op 14
timeout 120 python3 -u -c "
from arm import *
import arm
a=Arm()
req = GetPositionFK.Request(); req.fk_link_names=['panda_hand','panda_link0']
req.robot_state.joint_state.name=JOINTS; req.robot_state.joint_state.position=a.arm_q()
fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.n,fut,timeout_sec=30); r=fut.result()
for ps in r.pose_stamped: print(ps.header.frame_id, ps.pose.position)
arm.BASE[:]=0
q=a.solve_ik([-0.269,-0.164,0.70]); print('ik above soup (world coords direct)',None if q is None else np.round(q,3))
"

# openrua op 15
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE = np.array([-0.51, 0.0, 0.42])  # world position of panda_link0 (from TF)", "new_string": "BASE = np.zeros(3)  # verified: MoveIt model frame == world on this machine (FK of panda_link0 = (-0.51,0,0.42))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 16
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    def gripper(self, width):", "new_string": "    def grab(self, topic, msg_type, timeout=30.0):\\n        got = {}\\n        sub = self.n.create_subscription(msg_type, topic, lambda m: got.setdefault(\\"m\\", m), 1)\\n        t0 = time.time()\\n        while \\"m\\" not in got and time.time() - t0 < timeout:\\n            self.spin(0.2)\\n        self.n.destroy_subscription(sub)\\n        return got.get(\\"m\\")\\n\\n    def cloud_world(self, cam):\\n        \\"\\"\\"Depth frame of <cam> -> Nx3 world points (uses live TF), plus color image.\\"\\"\\"\\n        from sensor_msgs.msg import Image, CameraInfo\\n        from tf2_ros import Buffer, TransformListener\\n        from rclpy.time import Time\\n        from cv_bridge import CvBridge\\n        if not hasattr(self, \\"tfb\\"):\\n            self.tfb = Buffer(); self.tfl = TransformListener(self.tfb, self.n)\\n        depth = self.grab(f\\"/{cam}/depth/image_raw\\", Image)\\n        color = self.grab(f\\"/{cam}/color/image_raw\\", Image)\\n        info = self.grab(f\\"/{cam}/color/camera_info\\", CameraInfo)\\n        frame = f\\"{cam}_optical_frame\\"\\n        t0 = time.time()\\n        while not self.tfb.can_transform(\\"world\\", frame, Time()) and time.time() - t0 < 10:\\n            self.spin(0.2)\\n        t = self.tfb.lookup_transform(\\"world\\", frame, Time()).transform\\n        q = t.rotation; x, y, z, w = q.x, q.y, q.z, q.w\\n        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],\\n                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],\\n                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])\\n        T = np.array([t.translation.x, t.translation.y, t.translation.z])\\n        D = CvBridge().imgmsg_to_cv2(depth, \\"passthrough\\").astype(np.float64)\\n        C = CvBridge().imgmsg_to_cv2(color, \\"bgr8\\")\\n        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]\\n        vs, us = np.mgrid[0:D.shape[0], 0:D.shape[1]]\\n        P = np.stack([(us - cx) * D / fx, (vs - cy) * D / fy, D], -1)\\n        W = P @ R.T + T\\n        return W, C, D\\n\\n    def locate(self, cam, guess, radius=0.06, zmin=0.445, zmax=0.60):\\n        \\"\\"\\"Centroid (world xy) and top z of the blob above the table near guess.\\"\\"\\"\\n        W, C, D = self.cloud_world(cam)\\n        ok = np.isfinite(D) & (D > 0.05)\\n        near = (np.hypot(W[..., 0] - guess[0], W[..., 1] - guess[1]) < radius)\\n        m = ok & near & (W[..., 2] > zmin) & (W[..., 2] < zmax)\\n        if m.sum() < 10:\\n            print(\\"  locate: nothing found\\", flush=True); return None\\n        pts = W[m]\\n        # use the top slice (cap) to get an unbiased xy centroid of a cylinder\\n        ztop = np.percentile(pts[:, 2], 98)\\n        cap = pts[pts[:, 2] > ztop - 0.015]\\n        c = cap.mean(0)\\n        ext = (pts[:, 0].max() - pts[:, 0].min(), pts[:, 1].max() - pts[:, 1].min())\\n        print(f\\"  locate {cam}: n={m.sum()} center=({c[0]:.4f},{c[1]:.4f}) ztop={ztop:.4f} extent=({ext[0]:.3f},{ext[1]:.3f})\\", flush=True)\\n        return c[0], c[1], ztop\\n\\n    def gripper(self, width):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 17
timeout 120 python3 -u -c "
from arm import *
a=Arm()
print('soup', a.locate('birdview',(-0.269,-0.164)))
print('tomato', a.locate('birdview',(-0.024,-0.227)))
print('basket', a.locate('birdview',(0.008,0.268), radius=0.15, zmax=0.8))
"

# openrua op 18
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""pick_place.py <name> <x> <y>   -- pick the object near world (x,y) and drop it in the basket."""
import sys
from arm import *

name, gx, gy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
BASKET = (0.003, 0.260)
Z_TRAVEL = 0.83      # hand z for carrying (object bottom clears basket rim 0.626)
Z_LOOK = 0.78        # hand z for the eye-in-hand refinement look
Z_DROP = 0.70        # hand z over the basket when releasing

a = Arm()
print(f"== {name}: start q={np.round(a.arm_q(),3)} fingers={a.fingers()}", flush=True)

print("-- open gripper", flush=True)
a.gripper(GRIP["open_m"])

print("-- move above guess", flush=True)
assert a.move_to((gx, gy, Z_LOOK), seconds=3.0)

print("-- refine with eye-in-hand", flush=True)
loc = a.locate("robot0_eye_in_hand", (gx, gy), radius=0.06)
if loc is None:
    loc = a.locate("birdview", (gx, gy), radius=0.06)
x, y, ztop = loc
d = np.hypot(x - gx, y - gy)
print(f"   correction {d*1000:.1f} mm", flush=True)
if d > 0.004:
    assert a.move_to((x, y, Z_LOOK), seconds=2.0)
    loc2 = a.locate("robot0_eye_in_hand", (x, y), radius=0.06)
    if loc2 is not None:
        x, y, ztop = loc2

z_grasp = ztop - 0.035 + TCP          # TCP 3.5 cm below the can top
z_grasp = max(z_grasp, 0.425 + 0.015 + TCP)  # never below 1.5 cm above the table
print(f"-- descend to grasp: xy=({x:.4f},{y:.4f}) hand z={z_grasp:.4f}", flush=True)
assert a.move_to((x, y, z_grasp + 0.08), seconds=2.0)
assert a.move_to((x, y, z_grasp), seconds=2.0)

print("-- close gripper", flush=True)
f = a.gripper(GRIP["closed_m"])
gap = abs(f[0]) + abs(f[1])
print(f"   finger gap after close = {gap*1000:.1f} mm", flush=True)
if gap < 0.01:
    print("!! grasp FAILED (closed on air)", flush=True)
    a.gripper(GRIP["open_m"])
    a.move_to((x, y, Z_TRAVEL), seconds=2.0)
    sys.exit(2)

print("-- lift", flush=True)
assert a.move_to((x, y, Z_TRAVEL), seconds=2.5)
f = a.fingers(); print(f"   fingers after lift {f}", flush=True)

print("-- carry to basket", flush=True)
assert a.move_to((BASKET[0], BASKET[1], Z_TRAVEL), seconds=3.5)
assert a.move_to((BASKET[0], BASKET[1], Z_DROP), seconds=2.0)

print("-- release", flush=True)
a.gripper(GRIP["open_m"])

print("-- retreat", flush=True)
assert a.move_to((BASKET[0], BASKET[1], Z_TRAVEL), seconds=2.0)
assert a.move_to((-0.15, 0.10, Z_TRAVEL), seconds=2.5)

print("-- verify: birdview look into basket", flush=True)
W, C, D = a.cloud_world("birdview")
inb = (np.abs(W[..., 0] - BASKET[0]) < 0.06) & (np.abs(W[..., 1] - BASKET[1]) < 0.06) & np.isfinite(D) & (D > 0.05)
zs = W[..., 2][inb]
print(f"   basket interior z: median={np.median(zs):.3f} max={zs.max():.3f} n={inb.sum()}", flush=True)
import cv2; cv2.imwrite(f"after_{name}.png", C)
print(f"== {name}: DONE", flush=True)
OPENRUA_EOF

# openrua op 19
nohup python3 -u pick_place.py soup -0.269 -0.162 > soup.log 2>&1 &
sleep 45; cat /workspace/soup.log

# openrua op 20
sleep 60; cat /workspace/soup.log | tail -n +16

# openrua op 21
sleep 20; tail -n 4 /workspace/soup.log

# openrua op 22
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    def traj(self, q, seconds=3.0):\\n        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = JOINTS\\n        pt = JointTrajectoryPoint(positions=[float(x) for x in q])\\n        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))\\n        goal.trajectory.points = [pt]\\n        send = self.fjt.send_goal_async(goal)\\n        rclpy.spin_until_future_complete(self.n, send, timeout_sec=60)\\n        res = send.result().get_result_async()\\n        rclpy.spin_until_future_complete(self.n, res, timeout_sec=600)\\n        code = res.result().result.error_code\\n        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()\\n        print(f\\"  traj code={code} max_joint_err={err:.4f}\\", flush=True)\\n        return code, err", "new_string": "    def traj(self, q, seconds=3.0, tol=0.02, tries=4):\\n        \\"\\"\\"Send the goal; the controller lags and reports -5 short of the\\n        target, so re-send (with the remaining distance) until converged.\\"\\"\\"\\n        for i in range(tries):\\n            goal = FollowJointTrajectory.Goal()\\n            goal.trajectory.joint_names = JOINTS\\n            pt = JointTrajectoryPoint(positions=[float(x) for x in q])\\n            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))\\n            goal.trajectory.points = [pt]\\n            send = self.fjt.send_goal_async(goal)\\n            rclpy.spin_until_future_complete(self.n, send, timeout_sec=60)\\n            res = send.result().get_result_async()\\n            rclpy.spin_until_future_complete(self.n, res, timeout_sec=600)\\n            code = res.result().result.error_code\\n            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()\\n            print(f\\"  traj[{i}] code={code} max_joint_err={err:.4f}\\", flush=True)\\n            if err < tol:\\n                break\\n            seconds = max(1.5, seconds * min(1.0, err / 0.5 + 0.3))\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 23
timeout 120 python3 -u -c "
from arm import *
a=Arm()
print('soup?', a.locate('birdview',(0.02,0.21), radius=0.05))
print('tomato', a.locate('birdview',(-0.02,-0.229)))
print('basket', a.locate('birdview',(0.003,0.26), radius=0.13, zmax=0.8))
W,C,D=a.cloud_world('birdview')
inb=(np.abs(W[...,0]-0.003)<0.06)&(np.abs(W[...,1]-0.26)<0.06)
print('basket interior max z', W[...,2][inb].max())
pos,_=a.hand_world(); print('hand', pos)
"

# openrua op 24
timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png; timeout 120 python3 -u -c "
from arm import *
a=Arm()
W,C,D=a.cloud_world('birdview')
# profile along y at x=0.0 and along x at y=0.30
for yy in np.arange(0.15,0.42,0.01):
    m=(np.abs(W[...,0]-0.0)<0.005)&(np.abs(W[...,1]-yy)<0.005)
    print(f'y={yy:.2f} z={W[...,2][m].max():.3f}', end=' | ')
print()
for xx in np.arange(-0.12,0.13,0.01):
    m=(np.abs(W[...,0]-xx)<0.005)&(np.abs(W[...,1]-0.30)<0.005)
    print(f'x={xx:.2f} z={W[...,2][m].max():.3f}', end=' | ')
"

# openrua op 25
timeout 120 python3 -u -c "
from arm import *
a=Arm()
W,C,D=a.cloud_world('birdview')
xs=np.arange(-0.12,0.13,0.01); ys=np.arange(0.16,0.50,0.01)
print('      '+' '.join(f'{x*100:4.0f}' for x in xs))
for yy in ys:
    row=[]
    for xx in xs:
        m=(np.abs(W[...,0]-xx)<0.005)&(np.abs(W[...,1]-yy)<0.005)
        row.append(f'{(W[...,2][m].max()-0.425)*100:4.0f}' if m.any() else '   .')
    print(f'y={yy*100:3.0f} '+' '.join(row))
"

# openrua op 26
python3 - <<'EOF'
import re
s=open('pick_place.py').read()
s=s.replace("BASKET = (0.003, 0.260)","BASKET = (0.0, 0.35)   # re-measured after the basket got nudged")
s=s.replace("Z_DROP = 0.70        # hand z over the basket when releasing","Z_DROP = 0.67        # hand body bottom (~hand z - 0.066) stays above rim z 0.575")
s=s.replace("z_grasp = ztop - 0.035 + TCP          # TCP 3.5 cm below the can top","z_grasp = ztop - 0.040 + TCP          # TCP 4 cm below the can top (cans are 9.1 cm tall)")
s=s.replace("""inb = (np.abs(W[..., 0] - BASKET[0]) < 0.06) & (np.abs(W[..., 1] - BASKET[1]) < 0.06) & np.isfinite(D) & (D > 0.05)
zs = W[..., 2][inb]
print(f"   basket interior z: median={np.median(zs):.3f} max={zs.max():.3f} n={inb.sum()}", flush=True)""",
"""xs = np.arange(-0.12, 0.13, 0.01); ys = np.arange(0.16, 0.50, 0.01)
print('      ' + ' '.join(f'{x*100:4.0f}' for x in xs))
for yy in ys:
    row = []
    for xx in xs:
        m = (np.abs(W[..., 0] - xx) < 0.005) & (np.abs(W[..., 1] - yy) < 0.005)
        row.append(f'{(W[..., 2][m].max()-0.425)*100:4.0f}' if m.any() else '   .')
    print(f'y={yy*100:3.0f} ' + ' '.join(row), flush=True)""")
open('pick_place.py','w').write(s)
EOF
grep -n "BASKET =\|Z_DROP =\|z_grasp = ztop" pick_place.py
nohup python3 -u pick_place.py soup2 0.0105 0.2213 > soup2.log 2>&1 &
sleep 90; cat soup2.log

# openrua op 27
sleep 45; tail -n +30 soup2.log

# openrua op 28
timeout 120 python3 -u -c "
from arm import *
a=Arm()
W,C,D=a.cloud_world('birdview')
ok=np.isfinite(D)&(D>0.05)
h=W[...,2]-0.425
# basket floor: 1cm<h<5cm in the y>0.25 region
m=ok&(h>0.01)&(h<0.05)&(W[...,1]>0.25)
print('floor cells',m.sum(),'centroid',W[m][:,:2].mean(0).round(4), 'x range',W[m][:,0].min().round(3),W[m][:,0].max().round(3),'y range',W[m][:,1].min().round(3),W[m][:,1].max().round(3))
r=ok&(h>0.12)&(W[...,1]>0.25)
print('rim cells',r.sum(),'centroid',W[r][:,:2].mean(0).round(4))
print('soup', a.locate('birdview',(0.01,0.22),radius=0.045,zmax=0.55))
print('tomato', a.locate('birdview',(-0.019,-0.229),radius=0.045,zmax=0.55))
"

# openrua op 29
timeout 120 python3 -u -c "
from arm import *
a=Arm()
W,C,D=a.cloud_world('birdview')
ok=np.isfinite(D)&(D>0.05); h=W[...,2]-0.425
r=ok&(h>0.12)&(W[...,1]>0.25); rc=W[r][:,:2].mean(0)
near=np.hypot(W[...,0]-rc[0],W[...,1]-rc[1])<0.10
m=ok&(h>0.01)&(h<0.05)&near
print('rim centroid',rc.round(4),'floor cells',m.sum(),'floor centroid',W[m][:,:2].mean(0).round(4))
"

# openrua op 30
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""pick_place.py <name> <x> <y>   -- pick the can near world (x,y) and drop it in the basket."""
import sys
import cv2
from arm import *

name, gx, gy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
TABLE = 0.425
Z_TRAVEL = 0.83      # hand z for carrying: TCP 0.727, can bottom ~0.67 > rim 0.575
Z_LOOK = 0.78        # hand z for the eye-in-hand refinement look
Z_DROP = 0.67        # hand body bottom (~hand z - 0.066) stays above rim z 0.575
RIM_MIN_H = 0.12     # basket rim height above table (measured 0.15)


def basket_center(a):
    """Interior-floor centroid of the basket from the birdview (basket = tallest thing in y>0.25)."""
    W, C, D = a.cloud_world("birdview")
    ok = np.isfinite(D) & (D > 0.05)
    h = W[..., 2] - TABLE
    r = ok & (h > RIM_MIN_H) & (W[..., 1] > 0.25)
    rc = W[r][:, :2].mean(0)
    near = np.hypot(W[..., 0] - rc[0], W[..., 1] - rc[1]) < 0.10
    m = ok & (h > 0.01) & (h < 0.05) & near
    fc = W[m][:, :2].mean(0)
    print(f"   basket rim centroid {rc.round(4)} floor centroid {fc.round(4)} (n={m.sum()})", flush=True)
    return fc, W, C, D


def height_grid(W, D):
    xs = np.arange(-0.16, 0.13, 0.01); ys = np.arange(0.16, 0.50, 0.01)
    print('      ' + ' '.join(f'{x*100:4.0f}' for x in xs))
    for yy in ys:
        row = []
        for xx in xs:
            m = (np.abs(W[..., 0] - xx) < 0.005) & (np.abs(W[..., 1] - yy) < 0.005) & np.isfinite(D)
            row.append(f'{(W[..., 2][m].max()-TABLE)*100:4.0f}' if m.any() else '   .')
        print(f'y={yy*100:3.0f} ' + ' '.join(row), flush=True)


def can_check(loc):
    """Sanity: a can top sits 0.49-0.54, and is 6-8 cm across; the basket rim is at 0.575."""
    x, y, ztop = loc
    return 0.49 < ztop < 0.545


a = Arm()
print(f"== {name}: start q={np.round(a.arm_q(),3)} fingers={a.fingers()}", flush=True)
BASKET, _, _, _ = basket_center(a)

print("-- open gripper", flush=True)
a.gripper(GRIP["open_m"])

print("-- move above guess", flush=True)
assert a.move_to((gx, gy, Z_LOOK), seconds=3.0)

print("-- refine with eye-in-hand", flush=True)
loc = a.locate("robot0_eye_in_hand", (gx, gy), radius=0.045, zmax=0.55)
if loc is None or not can_check(loc):
    print("!! refinement did not see a can-like blob; falling back to birdview", flush=True)
    loc = a.locate("birdview", (gx, gy), radius=0.045, zmax=0.55)
assert loc is not None and can_check(loc), f"no can near guess: {loc}"
x, y, ztop = loc
d = np.hypot(x - gx, y - gy)
print(f"   correction {d*1000:.1f} mm", flush=True)
if d > 0.004:
    assert a.move_to((x, y, Z_LOOK), seconds=2.0)
    loc2 = a.locate("robot0_eye_in_hand", (x, y), radius=0.045, zmax=0.55)
    if loc2 is not None and can_check(loc2):
        x, y, ztop = loc2

z_grasp = ztop - 0.040 + TCP          # TCP 4 cm below the can top (cans are 9.1 cm tall)
z_grasp = max(z_grasp, TABLE + 0.015 + TCP)  # never below 1.5 cm above the table
print(f"-- descend to grasp: xy=({x:.4f},{y:.4f}) hand z={z_grasp:.4f}", flush=True)
assert a.move_to((x, y, z_grasp + 0.08), seconds=2.0)
assert a.move_to((x, y, z_grasp), seconds=2.0)

print("-- close gripper", flush=True)
f = a.gripper(GRIP["closed_m"])
gap = abs(f[0]) + abs(f[1])
print(f"   finger gap after close = {gap*1000:.1f} mm", flush=True)
if not 0.045 < gap < 0.08:
    print("!! grasp FAILED (gap not can-sized)", flush=True)
    a.gripper(GRIP["open_m"])
    a.move_to((x, y, Z_TRAVEL), seconds=2.0)
    sys.exit(2)

print("-- lift", flush=True)
assert a.move_to((x, y, Z_TRAVEL), seconds=2.5)
f = a.fingers(); print(f"   fingers after lift {f}", flush=True)

print("-- carry to basket", flush=True)
assert a.move_to((BASKET[0], BASKET[1], Z_TRAVEL), seconds=3.5)
assert a.move_to((BASKET[0], BASKET[1], Z_DROP), seconds=2.0)

print("-- release", flush=True)
a.gripper(GRIP["open_m"])

print("-- retreat", flush=True)
assert a.move_to((BASKET[0], BASKET[1], Z_TRAVEL), seconds=2.0)
assert a.move_to((-0.15, 0.05, Z_TRAVEL), seconds=2.5)

print("-- verify: birdview", flush=True)
BASKET2, W, C, D = basket_center(a)
height_grid(W, D)
cv2.imwrite(f"after_{name}.png", C)
print(f"== {name}: DONE (basket moved {np.hypot(*(BASKET2-BASKET))*1000:.0f} mm during place)", flush=True)
OPENRUA_EOF

# openrua op 31
nohup python3 -u pick_place.py soup3 0.0104 0.2215 > soup3.log 2>&1 &
sleep 100; cat soup3.log

# openrua op 32
timeout 120 python3 -u -c "
from arm import *
a=Arm()
print('soup', a.locate('birdview',(0.014,0.223),radius=0.05,zmax=0.55))
W,C,D=a.cloud_world('birdview'); ok=np.isfinite(D)&(D>0.05); h=W[...,2]-0.425
r=ok&(h>0.12)&(W[...,1]>0.25); print('rim centroid',W[r][:,:2].mean(0).round(4), 'rim n',r.sum())
print('tomato', a.locate('birdview',(-0.019,-0.229),radius=0.045,zmax=0.55))
q=a.solve_ik((0.014,0.223,0.58),(0.7071068,0.7071068,0,0)); print('ik yaw90 grasp', None if q is None else np.round(q,3))
q=a.solve_ik((-0.053,0.382,0.67),(0.7071068,0.7071068,0,0)); print('ik yaw90 drop', None if q is None else np.round(q,3))
"

# openrua op 33
timeout 120 python3 -u -c "
from arm import *
import cv2
a=Arm()
W,C,D=a.cloud_world('birdview'); cv2.imwrite('bird3.png',C)
ok=np.isfinite(D)&(D>0.05); h=W[...,2]-0.425
mask=(ok&(h>0.03)&(h<0.30)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<20: continue
    m=lab==i; pts=W[m]
    print(i,'px',cent[i].round(0),'area',stats[i,4],'xy',pts[:,:2].mean(0).round(3),'ztop',pts[:,2].max().round(3),'ext',(pts[:,0].max()-pts[:,0].min()).round(3),(pts[:,1].max()-pts[:,1].min()).round(3))
"

# openrua op 34
timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview front3.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('bird3.png'); cv2.imwrite('crop_base.png', cv2.resize(im[110:180,290:370],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 35
sed -i 's/        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat/        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)/' arm.py && sed -i 's/^DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z pointing down, fingers along world y/DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z pointing down, fingers along world y\nDOWN_X = (0.7071068, 0.7071068, 0.0, 0.0)  # hand z down, yawed 90deg: fingers along world x/' arm.py && grep -n "DOWN" arm.py | head; timeout 120 python3 -u -c "
from arm import *
a=Arm()
for name,p,q in [('grasp90',(0.014,0.223,0.58),DOWN_X),('look90',(0.014,0.223,0.78),DOWN_X),('drop90',(-0.045,0.41,0.67),DOWN_X),('drop0',(-0.045,0.41,0.67),DOWN),('travel90',(-0.045,0.41,0.83),DOWN_X)]:
    s=a.solve_ik(p,q); print(name, None if s is None else np.round(s,3))
"

# openrua op 36
python3 - <<'EOF'
s=open('pick_place.py').read()
s=s.replace("a.move_to((gx, gy, Z_LOOK), seconds=3.0)","a.move_to((gx, gy, Z_LOOK), Q, seconds=3.0)")
s=s.replace("a.move_to((x, y, Z_LOOK), seconds=2.0)","a.move_to((x, y, Z_LOOK), Q, seconds=2.0)")
s=s.replace("a.move_to((x, y, z_grasp + 0.08), seconds=2.0)","a.move_to((x, y, z_grasp + 0.08), Q, seconds=2.0)")
s=s.replace("a.move_to((x, y, z_grasp), seconds=2.0)","a.move_to((x, y, z_grasp), Q, seconds=2.0)")
s=s.replace("a.move_to((x, y, Z_TRAVEL), seconds=2.0)","a.move_to((x, y, Z_TRAVEL), Q, seconds=2.0)")
s=s.replace("a.move_to((x, y, Z_TRAVEL), seconds=2.5)","a.move_to((x, y, Z_TRAVEL), Q, seconds=2.5)")
s=s.replace("a.move_to((BASKET[0], BASKET[1], Z_TRAVEL), seconds=3.5)","a.move_to((BASKET[0], BASKET[1], Z_TRAVEL), Q, seconds=3.5)")
s=s.replace("a.move_to((BASKET[0], BASKET[1], Z_DROP), seconds=2.0)","a.move_to((BASKET[0], BASKET[1], Z_DROP), Q, seconds=2.0)")
s=s.replace("a.move_to((BASKET[0], BASKET[1], Z_TRAVEL), seconds=2.0)","a.move_to((BASKET[0], BASKET[1], Z_TRAVEL), Q, seconds=2.0)")
s=s.replace("a.move_to((-0.15, 0.05, Z_TRAVEL), seconds=2.5)","a.move_to((-0.15, 0.05, Z_TRAVEL), Q, seconds=2.5)")
s=s.replace("RIM_MIN_H = 0.12     # basket rim height above table (measured 0.15)","RIM_MIN_H = 0.12     # basket rim height above table (measured 0.15)\nQ = DOWN_X           # fingers along world x: hand is narrow in y (basket side) -> no rim collision")
open('pick_place.py','w').write(s)
EOF
grep -c "Q, seconds" pick_place.py
nohup python3 -u pick_place.py soup4 0.014 0.223 > soup4.log 2>&1 &
sleep 120; cat soup4.log | grep -v "^y=\|^      "

# openrua op 37
sleep 30; tail -n 45 soup4.log

# openrua op 38
sleep 40; tail -n 40 soup4.log

# openrua op 39
python3 -c "
import cv2
im=cv2.imread('after_soup4.png'); cv2.imwrite('crop_basket4.png', cv2.resize(im[230:330,360:470],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 40
timeout 120 python3 -u -c "
from arm import *
a=Arm()
W,C,D=a.cloud_world('birdview'); ok=np.isfinite(D)&(D>0.05); h=W[...,2]-0.425
r=ok&(h>0.12)&(W[...,1]>0.25); rc=W[r][:,:2].mean(0); print('rim centroid',rc.round(4))
near=np.hypot(W[...,0]-rc[0],W[...,1]-rc[1])<0.10
fl=ok&(h>0.01)&(h<0.05)&near; print('free floor centroid',W[fl][:,:2].mean(0).round(4),'n',fl.sum(), 'x',W[fl][:,0].min().round(3),W[fl][:,0].max().round(3),'y',W[fl][:,1].min().round(3),W[fl][:,1].max().round(3))
can=ok&(h>0.09)&(h<0.12)&near; print('soup in basket centroid',W[can][:,:2].mean(0).round(4),'n',can.sum())
print('tomato', a.locate('birdview',(-0.019,-0.229),radius=0.045,zmax=0.55))
"

# openrua op 41
timeout 120 python3 -u -c "
from arm import *
import cv2
a=Arm()
W,C,D=a.cloud_world('birdview'); ok=np.isfinite(D)&(D>0.05); h=W[...,2]-0.425
r=ok&(h>0.12)&(W[...,1]>0.25); rc=W[r][:,:2].mean(0)
# rasterize basket region at 5mm
res=0.005; xs=np.arange(rc[0]-0.12,rc[0]+0.12,res); ys=np.arange(rc[1]-0.12,rc[1]+0.12,res)
G=np.zeros((len(ys),len(xs)),np.uint8)  # 1 = free floor
for i,yy in enumerate(ys):
    for j,xx in enumerate(xs):
        m=(np.abs(W[...,0]-xx)<res)&(np.abs(W[...,1]-yy)<res)&ok
        if m.any() and h[m].max()<0.05 and np.hypot(xx-rc[0],yy-rc[1])<0.09: G[i,j]=1
dt=cv2.distanceTransform(G,cv2.DIST_L2,5)*res
i,j=np.unravel_index(dt.argmax(),dt.shape)
print('best free spot',round(xs[j],4),round(ys[i],4),'clearance radius',round(float(dt[i,j]),4))
for row in G[::2,::2]: print(''.join('#' if v else '.' for v in row))
"

# openrua op 42
timeout 120 python3 -u -c "
from arm import *
a=Arm()
W,C,D=a.cloud_world('birdview'); ok=np.isfinite(D)&(D>0.05); h=W[...,2]-0.425
r=ok&(h>0.12)&(W[...,1]>0.25); rc=W[r][:,:2].mean(0)
near=np.hypot(W[...,0]-rc[0],W[...,1]-rc[1])<0.10
can=ok&(h>0.09)&(h<0.12)&near; sc=W[can][:,:2].mean(0); print('soup',sc.round(4),'rim c',rc.round(4))
def hmax(x,y,rad=0.004):
    m=(np.abs(W[...,0]-x)<rad)&(np.abs(W[...,1]-y)<rad)&ok
    return h[m].max() if m.any() else np.nan
best=None
for ang in np.arange(0,360,10):
    d=np.array([np.cos(np.radians(ang)),np.sin(np.radians(ang))])
    reach=0.04
    while reach<0.15:
        p=sc+d*reach
        hm=hmax(*p)
        if np.isnan(hm) or hm>0.05: break
        reach+=0.005
    print(f'ang {ang:3.0f} free floor until {reach*100:.1f} cm from soup center', end=' | ')
    if best is None or reach>best[0]: best=(reach,ang)
print(); print('best',best)
"

# openrua op 43
python3 - <<'EOF'
s=open('pick_place.py').read()
s=s.replace('name, gx, gy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])',
 'name, gx, gy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])\nDROP_XY = (float(sys.argv[4]), float(sys.argv[5])) if len(sys.argv) > 5 else None  # explicit drop point')
s=s.replace('Z_DROP = 0.67        # hand body bottom (~hand z - 0.066) stays above rim z 0.575',
 'Z_DROP = 0.70        # fingertips (hand z - 0.1034) stay above rim z 0.579 even near the rim')
s=s.replace('BASKET, _, _, _ = basket_center(a)','BASKET, _, _, _ = basket_center(a)\nif DROP_XY is not None:\n    BASKET = np.array(DROP_XY); print(f"   using explicit drop point {BASKET}", flush=True)')
open('pick_place.py','w').write(s)
EOF
nohup python3 -u pick_place.py tomato -0.019 -0.229 -0.0055 0.3495 > tomato.log 2>&1 &
sleep 150; grep -v "^y=\|^      " tomato.log

# openrua op 44
sleep 45; tail -n 40 tomato.log

# openrua op 45
for c in agentview frontview sideview galleryview; do timeout 60 python3 tools/perception/cam_snap.py $c ${c}_final.png >/dev/null; done; python3 -c "
import cv2
im=cv2.imread('agentview_final.png'); cv2.imwrite('crop_final_agent.png', cv2.resize(im[150:400,400:640],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('after_tomato.png'); cv2.imwrite('crop_final_bird.png', cv2.resize(im[220:330,350:470],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 46
timeout 120 python3 -u -c "
from arm import *
import cv2
a=Arm()
W,C,D=a.cloud_world('birdview'); ok=np.isfinite(D)&(D>0.05); h=W[...,2]-0.425
mask=(ok&(h>0.03)&(h<0.30)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
print('objects >3cm tall on the table (birdview):')
for i in range(1,n):
    if stats[i,4]<20: continue
    m=lab==i; pts=W[m]
    print(f'  xy=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) ztop={pts[:,2].max():.3f} ext=({pts[:,0].max()-pts[:,0].min():.3f},{pts[:,1].max()-pts[:,1].min():.3f}) area={stats[i,4]}')
r=ok&(h>0.12)&(h<0.155)&(W[...,1]>0.25); rc=W[r][:,:2].mean(0)
print('basket rim centroid',rc.round(3),'rim x',W[r][:,0].min().round(3),W[r][:,0].max().round(3),'y',W[r][:,1].min().round(3),W[r][:,1].max().round(3))
near=np.hypot(W[...,0]-rc[0],W[...,1]-rc[1])<0.08
t=ok&near&(h>0.155); print('tomato (lying, >15.5cm) centroid',W[t][:,:2].mean(0).round(3),'n',t.sum(),'z range',(W[t][:,2].min()).round(3),(W[t][:,2].max()).round(3))
pos,_=a.hand_world(); print('hand parked at',pos.round(3),'fingers',a.fingers())
"

# openrua op 47
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-libero-machine-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-libero-machine-quirks.md <<'OPENRUA_EOF'
---
name: panda-libero-machine-quirks
description: Hard-won facts about the Franka Panda / LIBERO tabletop sim in /workspace that contradict or extend the starter docs (IK frame, controller -5 behaviour, basket geometry, hand clearance)
metadata:
  type: project
---

Verified 2026-09-12 on the /workspace Panda workstation (LIBERO-style tabletop, basket task):

- MoveIt `/compute_ik` and `/compute_fk` poses are in the **world** frame (FK of panda_link0 = (-0.51, 0, 0.42)), NOT the arm base as docs/30-action.md claims. Pass world coords with empty frame_id.
- `follow_joint_trajectory` often returns error_code -5 with the arm 0.3–0.9 rad short; re-sending the same goal converges (see `/workspace/arm.py` `traj()` retry loop). Always check joint error, never trust the result code alone.
- Objects settle ~1.5 cm after the first commands; the very first birdview depth frame is stale (basket rim read 0.626 then 0.577). Re-measure after any motion.
- Panda hand is ~20 cm wide along the finger axis; body bottom ≈ hand_z − 0.066, fingertips = hand_z − 0.1034. Yaw the grasp (quat (0.707,0.707,0,0) → fingers along world x) when an obstacle sits beside the target in y.
- Gripper opens to 0.04/finger (8 cm gap); the cans are 6.5 cm ⌀ × 9.1 cm; a held can reads ~32 mm/finger. Basket interior ≈ 12 cm — two cans cannot both stand on its floor; the second ends up lying on the first (still inside).
- Eye-in-hand refinement must exclude taller neighbours (zmax below the basket rim) or it centres on the rim and grasps the basket.

**Why:** each of these cost a failed attempt (grasped the basket, dragged it, collided the hand with the rim).
**How to apply:** reuse `/workspace/arm.py` + `/workspace/pick_place.py` as the starting point for pick-and-place here.
OPENRUA_EOF

# openrua op 48
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no MEMORY.md yet)")

# openrua op 49
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda/LIBERO machine quirks](panda-libero-machine-quirks.md) — IK is world-frame, FJT returns -5 short of target (re-send), hand width vs basket rim, basket fits ~1.5 cans
OPENRUA_EOF
