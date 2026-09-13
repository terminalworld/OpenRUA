#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
ros2 topic list && echo --- && ros2 action list && echo --- && ros2 service list | grep -iE "ik|fk|plan|scene" && echo --- && ros2 node list

# openrua op 2
ros2 topic echo /joint_states --once && for c in agentview birdview frontview sideview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c; done

# openrua op 3
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -8; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -8; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -8

# openrua op 4
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+info from a camera, build a world-frame point cloud,
save color PNG, and print table height + object-ish cluster stats.
Usage: python3 scene.py <camera> [zmin_above_table]"""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, typ, timeout=20.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    z = depth
    pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], -1)  # cam frame
    pw = pc @ R.T + T
    np.save(f"{cam}_pw.npy", pw)
    cv2.imwrite(f"{cam}.png", color)
    np.save(f"{cam}_color.npy", color)
    print("image", w, h, "depth range", np.nanmin(z), np.nanmax(z))
    # table height: most common z
    zs = pw[..., 2][np.isfinite(pw[..., 2])]
    hist, edges = np.histogram(zs, bins=400)
    table_z = edges[np.argmax(hist)]
    print(f"dominant plane z ~ {table_z:.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 5
python3 scene.py birdview && python3 scene.py agentview

# openrua op 6
python3 -c "
import numpy as np, cv2
pw=np.load('birdview_pw.npy'); col=np.load('birdview_color.npy')
z=pw[...,2]
# table region
tab=(z>0.35)&(z<0.5)
zs=z[tab]; h,e=np.histogram(zs,bins=100); print('table z mode', e[np.argmax(h)])
tz=e[np.argmax(h)]
# objects: above table by >1cm
obj=(z>tz+0.012)&(z<tz+0.4)
obj=obj.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(obj)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    p=pw[m]; c=col[m].mean(0)
    print(f'comp {i}: px={stats[i,4]} centroid_px=({cent[i][0]:.0f},{cent[i][1]:.0f}) world x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] zmax={p[:,2].max():.3f} mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) bgr={c.astype(int)}')
"


# openrua op 7
python3 -c "
import numpy as np, cv2
pw=np.load('agentview_pw.npy'); col=np.load('agentview_color.npy')
z=pw[...,2]; x=pw[...,0]; y=pw[...,1]
tz=0.424
obj=(z>tz+0.008)&(z<tz+0.35)&(x>-0.4)&(x<0.5)&(np.abs(y)<0.6)
obj=obj.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(obj)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    p=pw[m]; c=col[m].mean(0)
    print(f'comp {i}: px={stats[i,4]} bbox_px={stats[i,:4]} world x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) bgr={c.astype(int)}')
"


# openrua op 8
python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 2.0 && python3 scene.py agentview && python3 -c "
import numpy as np, cv2
pw=np.load('agentview_pw.npy'); col=np.load('agentview_color.npy')
z=pw[...,2]; x=pw[...,0]; y=pw[...,1]
tz=0.424
obj=(z>tz+0.008)&(z<tz+0.35)&(x>-0.4)&(x<0.5)&(np.abs(y)<0.6)
obj=obj.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(obj)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    p=pw[m]; c=col[m].mean(0)
    print(f'comp {i}: px={stats[i,4]} bbox_px={stats[i,:4]} world x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) bgr={c.astype(int)}')
"

# openrua op 9
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""World-frame arm helpers for this Panda (base at world (-0.51,0,0.42)).

CLI:
  python3 arm.py go <x> <y> <z> [yaw_deg] [seconds]   # HAND frame, top-down
  python3 arm.py tcp <x> <y> <z> [yaw_deg] [seconds]  # fingertip point
  python3 arm.py grip <per_finger_m>
  python3 arm.py state                                # hand pose + fingers
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_W = np.array([-0.51, 0.0, 0.42])  # world position of panda_link0 (TF)


def topdown_quat(yaw_deg):
    """Hand Z down; fingers close along world axis rotated yaw from +y."""
    # q = qz(yaw) * (1,0,0,0)
    h = math.radians(yaw_deg) / 2
    qz = (0, 0, math.sin(h), math.cos(h))
    qx = (1.0, 0.0, 0.0, 0.0)
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_seed(self):
        j = self.joints()
        s = JointState()
        for n in JOINTS:
            s.name.append(n); s.position.append(j[n])
        return s, j

    def hand_pose_world(self):
        seed, j = self.arm_seed()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = seed
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), j

    def solve_ik(self, xw, yw, zw, yaw_deg, at_tcp=False):
        q = topdown_quat(yaw_deg)
        p = np.array([xw, yw, zw]) - BASE_W
        if at_tcp:
            p = p + np.array([0, 0, TCP])  # hand is TCP m above fingertips
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, p)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = q
        req.ik_request.robot_state.joint_state, _ = self.arm_seed()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"IK failed code={code} for {xw,yw,zw,yaw_deg}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def send_traj(self, points, seconds):
        """points: list of joint vectors; evenly spaced in time."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        n = len(points)
        for i, p in enumerate(points):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in p])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        j = self.joints()
        err = max(abs(j[n] - v) for n, v in zip(JOINTS, points[-1]))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def go(self, xw, yw, zw, yaw_deg=0.0, seconds=3.0, at_tcp=False):
        sol = self.solve_ik(xw, yw, zw, yaw_deg, at_tcp)
        code, err = self.send_traj([sol], seconds)
        pos, _, _ = self.hand_pose_world()
        print(f"hand now at world {pos.round(4)}")
        return pos

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        j = self.joints()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={j['panda_finger_joint1']:.4f},{j['panda_finger_joint2']:.4f}")
        return j

    def close(self):
        self.node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    try:
        if cmd in ("go", "tcp"):
            x, y, z = map(float, sys.argv[2:5])
            yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
            sec = float(sys.argv[6]) if len(sys.argv) > 6 else 3.0
            a.go(x, y, z, yaw, sec, at_tcp=(cmd == "tcp"))
        elif cmd == "grip":
            a.gripper(float(sys.argv[2]))
        elif cmd == "state":
            pos, q, j = a.hand_pose_world()
            print("hand world", pos.round(4), "quat", np.round(q, 3))
            print("fingers", j["panda_finger_joint1"], j["panda_finger_joint2"])
            print("arm", [round(j[n], 4) for n in JOINTS])
    finally:
        a.close()
OPENRUA_EOF

# openrua op 10
timeout 120 python3 arm.py state

# openrua op 11
timeout 120 python3 -c "
from arm import *
a=Arm()
import numpy as np
a.BASE_W=None
# try IK at the current hand pose expressed in WORLD coords (no base offset)
import arm
arm.BASE_W[:]=0
try:
    sol=a.solve_ik(-0.053,0.0,0.7776,0.0)
    print('world-frame IK sol', np.round(sol,3))
except Exception as e: print('world-frame IK', e)
arm.BASE_W[:]=[-0.51,0,0.42]
try:
    sol=a.solve_ik(-0.053,0.0,0.7776,0.0)
    print('base-frame IK sol', np.round(sol,3))
except Exception as e: print('base-frame IK', e)
print('current', [round(v,3) for v in a.arm_seed()[0].position])
a.close()
"

# openrua op 12
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE_W = np.array([-0.51, 0.0, 0.42])  # world position of panda_link0 (TF)", "new_string": "# Verified: FK/IK on this machine are expressed in WORLD (FK of the hand\\n# matches TF world->panda_hand; IK with a base offset fails), so no offset.\\nBASE_W = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 13
timeout 300 python3 arm.py go -0.19 0.0 0.92 0 3 && timeout 60 python3 scene.py robot0_eye_in_hand && timeout 60 python3 scene.py birdview

# openrua op 14
python3 -c "
import numpy as np, cv2
for cam in ['robot0_eye_in_hand','birdview']:
    pw=np.load(cam+'_pw.npy'); col=np.load(cam+'_color.npy')
    z=pw[...,2]; x=pw[...,0]; y=pw[...,1]
    tz=0.424
    obj=(z>tz+0.008)&(z<tz+0.30)&(x>-0.4)&(x<0.5)&(np.abs(y)<0.6)&np.isfinite(z)
    obj=obj.astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(obj)
    print('==',cam)
    for i in range(1,n):
        if stats[i,4]<30: continue
        m=lab==i
        p=pw[m]; c=col[m].mean(0)
        print(f'comp {i}: px={stats[i,4]} bbox_px={stats[i,:4]} world x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) bgr={c.astype(int)}')
"


# openrua op 15
python3 -c "
import numpy as np, cv2
pw=np.load('robot0_eye_in_hand_pw.npy'); col=np.load('robot0_eye_in_hand_color.npy')
z=pw[...,2]
m=(z>0.555)&(z<0.60)&(pw[...,0]<-0.1)&(np.abs(pw[...,1])<0.1)
p=pw[m]
print('rim pts',len(p),'x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max())
# fit circle to rim xy
A=np.c_[2*p[:,0],2*p[:,1],np.ones(len(p))]; b=p[:,0]**2+p[:,1]**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center',c[:2],'radius',r)
# slices of the mug body at several heights
for zl in [0.46,0.48,0.50,0.52,0.54,0.56]:
    mm=(z>zl)&(z<zl+0.015)&(pw[...,0]<-0.1)&(pw[...,0]>-0.3)&(np.abs(pw[...,1])<0.12)
    q=pw[mm]
    if len(q): print(f'z {zl}: n={len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]')
# annotate image
img=col.copy(); ys,xs=np.where(m); 
for u,v in zip(xs,ys): img[v,u]=(0,255,0)
cv2.imwrite('wrist_annot.png',img)
"


# openrua op 16
timeout 200 python3 -c "
from arm import *
a=Arm()
poses={'mug_pregrasp':(-0.192,0.004-0.041,0.72,0),'mug_grasp':(-0.192,0.004-0.041,0.55,0),
'plate_above':(0.17,-0.018-0.041,0.75,0),'plate_place':(0.17,-0.018-0.041,0.58,0),
'pud_above':(-0.05,0.121,0.60,0),'pud_grasp':(-0.05,0.121,0.44,0),
'pud_place_above':(0.17,0.115,0.60,0),'pud_place':(0.17,0.115,0.445,0)}
for k,(x,y,z,yaw) in poses.items():
    try:
        s=a.solve_ik(x,y,z,yaw,at_tcp=True); print(k,'OK',np.round(s,2))
    except Exception as e: print(k,'FAIL',e)
a.close()
"

# openrua op 17
python3 - <<'EOF'
import re
s=open('arm.py').read()
s=s.replace('''    def solve_ik(self, xw, yw, zw, yaw_deg, at_tcp=False):''','''    def solve_ik(self, xw, yw, zw, yaw_deg, at_tcp=False, seed=None):''')
s=s.replace('''        req.ik_request.robot_state.joint_state, _ = self.arm_seed()
        req.ik_request.avoid_collisions = False''','''        if seed is None:
            req.ik_request.robot_state.joint_state, _ = self.arm_seed()
        else:
            js = JointState(); js.name = list(JOINTS); js.position = [float(v) for v in seed]
            req.ik_request.robot_state.joint_state = js
        req.ik_request.avoid_collisions = False''')
s=s.replace('''    def gripper(self, width):''','''    def line(self, p0, p1, yaw_deg, seconds, n=4, at_tcp=True):
        """Straight-ish TCP line p0->p1 as n IK waypoints, each seeded by
        the previous so the arm stays on one branch."""
        _, _, j = self.hand_pose_world()
        seed = [j[k] for k in JOINTS]
        pts = []
        for i in range(1, n + 1):
            p = np.array(p0) + (np.array(p1) - np.array(p0)) * i / n
            seed = self.solve_ik(*p, yaw_deg, at_tcp=at_tcp, seed=seed)
            pts.append(seed)
        code, err = self.send_traj(pts, seconds)
        pos, _, _ = self.hand_pose_world()
        print(f"hand now at world {pos.round(4)} (tcp z {pos[2]-TCP:.4f})")
        return pos

    def tcp(self):
        pos, q, j = self.hand_pose_world()
        return pos - np.array([0, 0, TCP])

    def gripper(self, width):''')
open('arm.py','w').write(s)
EOF
echo ok

# openrua op 18
timeout 600 python3 -u -c "
from arm import *
a=Arm()
mx,my=-0.192,0.004
gx,gy=mx,my-0.041
a.gripper(0.04)
a.go(gx,gy,0.72,0,3,at_tcp=True)
a.line((gx,gy,0.72),(gx,gy,0.55),0,4,n=4)
print('tcp',a.tcp().round(4))
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 19
timeout 600 python3 -u -c "
from arm import *
a=Arm()
mx,my=-0.192,0.004
t=a.tcp()
a.line(t,(t[0],t[1],0.72),0,3,n=2)
gx,gy=mx+0.041,my
a.go(gx,gy,0.72,90,3,at_tcp=True)
a.line((gx,gy,0.72),(gx,gy,0.55),90,4,n=4)
print('tcp',a.tcp().round(4))
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 20
timeout 300 python3 -u -c "
from arm import *
a=Arm()
a.gripper(0.0)
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 21
timeout 600 python3 -u -c "
from arm import *
a=Arm()
t=a.tcp()
a.line(t,(t[0],t[1],0.76),90,4,n=3)
j=a.joints(); print('fingers',j['panda_finger_joint1'],j['panda_finger_joint2'])
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 scene.py sideview >/dev/null && python3 -c "
import numpy as np
pw=np.load('sideview_pw.npy'); z=pw[...,2]; x=pw[...,0]; y=pw[...,1]
m=(x>-0.3)&(x<0.0)&(np.abs(y)<0.08)&(z>0.43)&np.isfinite(z)
p=pw[m]; print('points in mug column: z range',p[:,2].min().round(3),p[:,2].max().round(3),'n',len(p))
"

# openrua op 22
timeout 900 python3 -u -c "
from arm import *
a=Arm()
px,py=0.170,-0.018
hx,hy=px+0.043,py
t=a.tcp()
a.line(t,(hx,hy,0.76),90,5,n=4)
a.line((hx,hy,0.76),(hx,hy,0.58),90,4,n=4)
print('tcp',a.tcp().round(4))
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 23
timeout 600 python3 -u -c "
from arm import *
a=Arm()
a.gripper(0.04)
t=a.tcp()
a.line(t,(t[0],t[1],0.78),90,4,n=3)
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 scene.py birdview > /dev/null && python3 -c "
import numpy as np
pw=np.load('birdview_pw.npy'); z=pw[...,2]; x=pw[...,0]; y=pw[...,1]
# mug top rim near the plate
m=(z>0.55)&(z<0.62)&(x>0.0)&(x<0.35)&(np.abs(y)<0.2)&np.isfinite(z)
p=pw[m]; print('mug rim pts',len(p),'center',p[:,:2].mean(0).round(3),'z',p[:,2].mean().round(3))
m=(z>0.44)&(z<0.47)&(x>0.0)&(x<0.35)&(np.abs(y)<0.2)&np.isfinite(z)
p=pw[m]; print('plate pts',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3))
"

# openrua op 24
timeout 900 python3 -u -c "
from arm import *
a=Arm()
bx,by=-0.052,0.121
t=a.tcp()
a.line(t,(bx,by,0.62),0,5,n=4)
a.line((bx,by,0.62),(bx,by,0.44),0,4,n=4)
print('tcp',a.tcp().round(4))
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 25
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        code = res.result().result.error_code\\n        j = self.joints()\\n        err = max(abs(j[n] - v) for n, v in zip(JOINTS, points[-1]))\\n        print(f\\"traj error_code={code} max_joint_err={err:.4f}\\")\\n        return code, err", "new_string": "        code = res.result().result.error_code\\n        j = self.joints()\\n        err = max(abs(j[n] - v) for n, v in zip(JOINTS, points[-1]))\\n        print(f\\"traj error_code={code} max_joint_err={err:.4f}\\")\\n        # machine fact: tolerance violations on long goals are controller\\n        # lag; resend the final point until the joints converge\\n        tries = 0\\n        while err > 0.01 and tries < 4:\\n            tries += 1\\n            goal.trajectory.points = [JointTrajectoryPoint(\\n                positions=[float(v) for v in points[-1]],\\n                time_from_start=Duration(sec=2))]\\n            send = self.traj.send_goal_async(goal)\\n            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)\\n            res = send.result().get_result_async()\\n            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)\\n            code = res.result().result.error_code\\n            j = self.joints()\\n            err = max(abs(j[n] - v) for n, v in zip(JOINTS, points[-1]))\\n            print(f\\"  retry {tries}: error_code={code} max_joint_err={err:.4f}\\")\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 26
timeout 900 python3 -u -c "
from arm import *
a=Arm()
t=a.tcp()
a.line(t,(t[0],t[1],0.75),0,3,n=2)
a.close()
" && timeout 60 python3 scene.py robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np, cv2
pw=np.load('robot0_eye_in_hand_pw.npy'); col=np.load('robot0_eye_in_hand_color.npy')
z=pw[...,2]; x=pw[...,0]; y=pw[...,1]
m=(z>0.435)&(z<0.50)&(x>-0.2)&(x<0.1)&(y>0.03)&(y<0.25)&np.isfinite(z)
p=pw[m]; print('box pts',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'ztop',p[:,2].max().round(3),'mean',p[:,:2].mean(0).round(3))
top=p[p[:,2]>0.455]
if len(top)>5:
    c=np.cov(top[:,:2].T); w,v=np.linalg.eigh(c); ax=v[:,1]; print('top center',top[:,:2].mean(0).round(3),'long axis',ax.round(3),'angle deg',np.degrees(np.arctan2(ax[1],ax[0])).round(1))
"

# openrua op 27
timeout 900 python3 -u -c "
from arm import *
a=Arm()
bx,by,yaw=-0.040,0.129,-5.2
t=a.tcp()
a.line(t,(bx,by,0.62),yaw,4,n=3)
a.line((bx,by,0.62),(bx,by,0.44),yaw,4,n=4)
print('tcp',a.tcp().round(4))
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 28
timeout 900 python3 -u -c "
from arm import *
a=Arm()
a.gripper(0.0)
t=a.tcp()
a.line(t,(t[0],t[1],0.62),-5.2,3,n=2)
j=a.joints(); print('fingers after lift',round(j['panda_finger_joint1'],4),round(j['panda_finger_joint2'],4))
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 29
timeout 900 python3 -u -c "
from arm import *
a=Arm()
tx,ty=0.17,0.115
t=a.tcp()
a.line(t,(tx,ty,0.62),0,5,n=4)
a.line((tx,ty,0.62),(tx,ty,0.45),0,4,n=4)
print('tcp',a.tcp().round(4))
j=a.joints(); print('fingers',round(j['panda_finger_joint1'],4))
a.gripper(0.04)
t=a.tcp()
a.line(t,(t[0],t[1],0.65),0,3,n=2)
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 30
timeout 900 python3 -u -c "
from arm import *
a=Arm()
t=a.tcp()
a.line(t,(-0.30,0.0,0.75),0,5,n=3)
a.close()
" && timeout 60 python3 scene.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import numpy as np, cv2
pw=np.load('birdview_pw.npy'); col=np.load('birdview_color.npy')
z=pw[...,2]; x=pw[...,0]; y=pw[...,1]
tz=0.424
obj=(z>tz+0.008)&(z<tz+0.30)&(x>-0.4)&(x<0.5)&(np.abs(y)<0.5)&np.isfinite(z)
n,lab,stats,cent=cv2.connectedComponentsWithStats(obj.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i; p=pw[m]
    print(f'comp {i}: px={stats[i,4]} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) bgr={col[m].mean(0).astype(int)}')
# mug rim (z>0.55) near plate
m=(z>0.55)&(z<0.60)&(x>0.0)&(x<0.35)&(np.abs(y)<0.2)&np.isfinite(z); p=pw[m]
A=np.c_[2*p[:,0],2*p[:,1],np.ones(len(p))]; b=p[:,0]**2+p[:,1]**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('MUG rim circle center',c[:2].round(3),'r',round(r,3),'ztop',p[:,2].max().round(3))
# plate: z in 0.44..0.46 and pale color
m=(z>0.44)&(z<0.462)&(x>0.05)&(x<0.3)&(np.abs(y)<0.12)&(col[...,0]>140)&np.isfinite(z); p=pw[m]
print('PLATE pts',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3))
m=(z>0.44)&(z<0.48)&(x>0.05)&(x<0.3)&(y>0.06)&(y<0.2)&np.isfinite(z); p=pw[m]
print('BOX pts',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'ztop',p[:,2].max().round(3),'center',p[:,:2].mean(0).round(3))
"
