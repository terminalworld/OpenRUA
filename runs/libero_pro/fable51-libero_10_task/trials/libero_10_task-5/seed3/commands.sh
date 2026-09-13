#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
ros2 topic list 2>&1; echo ----; ros2 action list 2>&1; echo ----; ros2 service list 2>&1 | head -40; echo ----; ros2 node list 2>&1

# openrua op 4
for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once 2>&1 | head -40

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[130:300,180:400]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); crop=im[140:300,100:400]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"; ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12 & sleep 4; kill %1 2>/dev/null; ros2 topic echo /birdview/color/camera_info --once 2>&1 | grep -A12 '^k:'

# openrua op 6
cat /tmp/claude-1000/-workspace/83e5bb5d-708a-47b3-b586-55da160b4fe2/tasks/bxqwcsw8a.output; ls *.png

# openrua op 7
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab depth+intrinsics+TF for a camera, save world-frame point cloud as npz.
Usage: python3 cloud.py <camera>   -> <camera>_cloud.npz (xyz: HxWx3, rgb: HxWx3)
"""
import sys, struct
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, T, timeout=30.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time; end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    cam = sys.argv[1]
    rclpy.init(); node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    rgb = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "rgb8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time; end = time.time() + 15
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation; R = quat_R(q.x, q.y, q.z, q.w)
    p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    xyz = pc @ R.T + p
    np.savez(f"{cam}_cloud.npz", xyz=xyz, rgb=rgb, depth=depth, K=np.array(info.k).reshape(3, 3), R=R, p=p)
    print(cam, "cam pos", p, "depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
for c in birdview agentview frontview sideview robot0_eye_in_hand; do timeout 120 python3 cloud.py $c 2>&1 | tail -1; done

# openrua op 9
python3 -c "
import numpy as np
for c in ['birdview','agentview','frontview','sideview']:
    d=np.load(c+'_cloud.npz'); xyz=d['xyz'].reshape(-1,3)
    ok=np.isfinite(xyz).all(1); xyz=xyz[ok]
    print(c, 'x',np.percentile(xyz[:,0],[1,50,99]).round(3),'y',np.percentile(xyz[:,1],[1,50,99]).round(3),'z',np.percentile(xyz[:,2],[1,50,99]).round(3))
    # table height: most common z
    h,e=np.histogram(xyz[:,2],bins=np.arange(-0.5,1.5,0.005)); i=h.argmax(); print('  mode z',e[i].round(3),h[i])
"


# openrua op 10
python3 -c "
import numpy as np
res=0.01
x0,x1,y0,y1=-0.55,0.35,-0.55,0.35
nx=int((x1-x0)/res); ny=int((y1-y0)/res)
H=np.full((nx,ny),np.nan)
for c in ['birdview','agentview','frontview','sideview']:
    d=np.load(c+'_cloud.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
    m=(xyz[:,0]>=x0)&(xyz[:,0]<x1)&(xyz[:,1]>=y0)&(xyz[:,1]<y1)&(xyz[:,2]<1.15)
    xyz=xyz[m]
    ix=((xyz[:,0]-x0)/res).astype(int); iy=((xyz[:,1]-y0)/res).astype(int)
    for a,b,z in zip(ix,iy,xyz[:,2]):
        if np.isnan(H[a,b]) or z>H[a,b]: H[a,b]=z
chars=' .:-=+*#%@'
print('rows: x from %.2f (top) step %.2f; cols: y from %.2f step %.2f; heights above table 0.88 in cm'%(x0,res,y0,res))
print('     '+''.join(str(int(round((y0+j*res)*100))%10) if j%5==0 else ' ' for j in range(ny)))
for i in range(nx):
    row=''
    for j in range(ny):
        z=H[i,j]
        if np.isnan(z): row+='?'
        else:
            h=z-0.88
            if h<0.01: row+='.'
            else: row+=chr(ord('a')+min(25,int(h*100)))  # a=1cm,b=2cm,...
    print('%5.2f %s'%(x0+i*res,row))
"


# openrua op 11
python3 -c "
import numpy as np
res=0.005
x0,x1,y0,y1=-0.50,-0.30,-0.40,0.12
nx=int(round((x1-x0)/res)); ny=int(round((y1-y0)/res))
for c in ['birdview']:
    H=np.full((nx,ny),np.nan)
    d=np.load(c+'_cloud.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
    m=(xyz[:,0]>=x0)&(xyz[:,0]<x1)&(xyz[:,1]>=y0)&(xyz[:,1]<y1)&(xyz[:,2]<1.15)
    xyz=xyz[m]
    ix=((xyz[:,0]-x0)/res).astype(int); iy=((xyz[:,1]-y0)/res).astype(int)
    for a,b,z in zip(ix,iy,xyz[:,2]):
        if np.isnan(H[a,b]) or z>H[a,b]: H[a,b]=z
    print(c)
    print('       '+''.join(('%d'%(abs(int(round((y0+j*res)*100)))%10)) if j%2==0 else ' ' for j in range(ny)))
    for i in range(nx):
        row=''
        for j in range(ny):
            z=H[i,j]
            if np.isnan(z): row+='?'
            else:
                h=z-0.88
                row+='.' if h<0.01 else chr(ord('a')+min(25,int(h*100)))
        print('%6.3f %s'%(x0+i*res,row))
"


# openrua op 12
python3 -c "
import numpy as np
for c in ['agentview','frontview','sideview','birdview']:
    d=np.load(c+'_cloud.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
    m=(xyz[:,0]>-0.19)&(xyz[:,0]<-0.03)&(xyz[:,1]>-0.08)&(xyz[:,1]<0.10)&(xyz[:,2]>0.885)&(xyz[:,2]<1.2)
    p=xyz[m]
    print(c, len(p), 'z range', p[:,2].min().round(3), p[:,2].max().round(3))
    for z0 in np.arange(0.89,1.06,0.01):
        s=p[(p[:,2]>=z0)&(p[:,2]<z0+0.01)]
        if len(s): print('  z %.2f n=%4d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
"


# openrua op 13
python3 -c "
import numpy as np
for c in ['agentview','frontview','birdview','sideview']:
    d=np.load(c+'_cloud.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
    m=(xyz[:,1]>-0.19)&(xyz[:,1]<-0.11)&(xyz[:,0]>-0.49)&(xyz[:,0]<-0.30)&(xyz[:,2]>0.885)&(xyz[:,2]<1.2)
    p=xyz[m]
    print(c, 'middle section x-z profile (per 5mm x bin: n, zmin, zmax)')
    for x0 in np.arange(-0.49,-0.30,0.005):
        s=p[(p[:,0]>=x0)&(p[:,0]<x0+0.005)]
        if len(s): print('  x %.3f n=%4d z[%.3f,%.3f]'%(x0,len(s),s[:,2].min(),s[:,2].max()))
" 2>&1 | head -120

# openrua op 14
python3 -c "
import cv2
im=cv2.imread('sideview.png'); crop=im[220:340,340:500]; cv2.imwrite('side_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('robot0_robotview.png'); crop=im[0:160,0:400]; cv2.imwrite('robotview_crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 15
(timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -m1 -A1 Translation; timeout 20 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -m1 -A3 Translation; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -m1 -A3 Translation) 

# openrua op 16
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once 2>&1 | grep -A9 position

# openrua op 17
for c in birdview agentview frontview sideview; do timeout 120 python3 cloud.py $c 2>&1 | tail -1; done; python3 -c "
import numpy as np
# cup
for c in ['agentview','frontview','sideview','birdview']:
    d=np.load(c+'_cloud.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
    m=(xyz[:,0]>-0.19)&(xyz[:,0]<-0.03)&(xyz[:,1]>-0.08)&(xyz[:,1]<0.10)&(xyz[:,2]>0.885)&(xyz[:,2]<1.2)
    p=xyz[m]
    print(c, 'cup z range', p[:,2].min().round(3), p[:,2].max().round(3))
    for z0 in np.arange(0.88,1.0,0.01):
        s=p[(p[:,2]>=z0)&(p[:,2]<z0+0.01)]
        if len(s): print('  z %.2f n=%4d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
# caddy middle profile from birdview
d=np.load('birdview_cloud.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
m=(xyz[:,1]>-0.19)&(xyz[:,1]<-0.11)&(xyz[:,0]>-0.49)&(xyz[:,0]<-0.30)&(xyz[:,2]>0.885)&(xyz[:,2]<1.2)
p=xyz[m]
print('caddy middle x-z profile birdview')
for x0 in np.arange(-0.49,-0.30,0.005):
    s=p[(p[:,0]>=x0)&(p[:,0]<x0+0.005)]
    if len(s): print('  x %.3f n=%4d z[%.3f,%.3f]'%(x0,len(s),s[:,2].min(),s[:,2].max()))
"


# openrua op 18
timeout 600 python3 tools/action/ik_move.py $(python3 -c "print(-0.059+0.75, 0.02, 1.06-0.912)") 0.7071068 0.7071068 0 0 4 --at tcp; ros2 topic echo /joint_states --once 2>&1 | grep -A9 position

# openrua op 19
mkdir -p "$(dirname /workspace/move_tcp.py)"
cat > /workspace/move_tcp.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Move the TCP (fingertip centre) to a WORLD-frame pose: IK -> trajectory -> FK check.

Usage: python3 move_tcp.py <x> <y> <z> <qx> <qy> <qz> <qw> [seconds=4]
       python3 move_tcp.py fk            # just print the current TCP pose (world)
World -> planner base frame handled here (base at world BASE_P).
"""
import sys, time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_P = np.array([-0.75, 0.0, 0.912])  # world position of panda_link0 (tf2_echo)
M = yaml.safe_load(open(Path(__file__).parent / "machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("move_tcp")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.traj.wait_for_server(10)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_state(self):
        j = self.joints()
        s = JointState(); s.name = list(ARM); s.position = [j[n] for n in ARM]
        return s

    def fk_tcp(self):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_state()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        R = quat_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        hand_w = np.array([p.position.x, p.position.y, p.position.z]) + BASE_P
        tcp_w = hand_w + TCP * R[:, 2]
        return tcp_w, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def move(self, x, y, z, qx, qy, qz, qw, secs=4.0):
        R = quat_R(qx, qy, qz, qw)
        hand_w = np.array([x, y, z]) - TCP * R[:, 2]
        hand_b = hand_w - BASE_P
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        req.ik_request.robot_state.joint_state = self.arm_state()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print("IK FAILED", None if res is None else res.error_code.val)
            return False
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        target = [sol[j] for j in ARM]
        print("IK ok, joints:", np.round(target, 3).tolist())
        return self.move_joints(target, secs)

    def move_joints(self, target, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in target])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        j = self.joints()
        err = max(abs(j[n] - t) for n, t in zip(ARM, target))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code == 0 and err < 0.02


def main():
    c = Ctl()
    if sys.argv[1] == "fk":
        pass
    else:
        vals = list(map(float, sys.argv[1:]))
        secs = vals[7] if len(vals) > 7 else 4.0
        c.move(*vals[:7], secs=secs)
    tcp, q = c.fk_tcp()
    print("TCP world:", np.round(tcp, 4).tolist(), "hand quat:", np.round(q, 4).tolist())
    j = c.joints()
    print("fingers:", round(j["panda_finger_joint1"], 4), round(j["panda_finger_joint2"], 4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 20
timeout 120 python3 move_tcp.py fk

# openrua op 21
cat > /tmp/fktest.py <<'EOF'
import sys; sys.path.insert(0,'/workspace')
import numpy as np, rclpy
from move_tcp import Ctl, ARM, M
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from builtin_interfaces.msg import Duration
c=Ctl()
req=GetPositionFK.Request(); req.fk_link_names=["panda_link0","panda_hand"]; req.robot_state.joint_state=c.arm_state()
fut=c.fk.call_async(req); rclpy.spin_until_future_complete(c.node,fut,timeout_sec=30); res=fut.result()
for ps in res.pose_stamped: print('FK', ps.header.frame_id, ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
print('fk names', res.fk_link_names, res.error_code.val)
hand=res.pose_stamped[1].pose
for label,off in [('as-is',np.zeros(3)),('minus base',np.array([-0.75,0,0.912]))]:
    req=GetPositionIK.Request(); req.ik_request.group_name=M["planning"]["group"]; req.ik_request.pose_stamped.header.frame_id=""
    p=req.ik_request.pose_stamped.pose; p.position.x=hand.position.x-off[0]; p.position.y=hand.position.y-off[1]; p.position.z=hand.position.z-off[2]
    p.orientation=hand.orientation; req.ik_request.robot_state.joint_state=c.arm_state(); req.ik_request.timeout=Duration(sec=2)
    fut=c.ik.call_async(req); rclpy.spin_until_future_complete(c.node,fut,timeout_sec=60); r=fut.result()
    print(label, 'code', r.error_code.val, np.round([dict(zip(r.solution.joint_state.name,r.solution.joint_state.position)).get(j,np.nan) for j in ARM],3).tolist() if r.error_code.val==1 else '')
rclpy.shutdown()
EOF
timeout 200 python3 /tmp/fktest.py

# openrua op 22
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/move_tcp.py", "old_string": "BASE_P = np.array([-0.75, 0.0, 0.912])  # world position of panda_link0 (tf2_echo)", "new_string": "BASE_P = np.array([0.0, 0.0, 0.0])  # verified: /compute_fk & /compute_ik use the WORLD frame here", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 23
timeout 600 python3 move_tcp.py -0.059 0.02 1.06 0.7071068 0.7071068 0 0 4

# openrua op 24
timeout 600 python3 move_tcp.py -0.059 0.02 1.06 0.7071068 0.7071068 0 0 4

# openrua op 25
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 26
timeout 600 python3 move_tcp.py -0.059 0.02 0.96 0.7071068 0.7071068 0 0 2.5

# openrua op 27
timeout 600 python3 move_tcp.py -0.059 0.02 0.96 0.7071068 0.7071068 0 0 2.5

# openrua op 28
python3 - <<'EOF'
import re
s=open('move_tcp.py').read()
s=s.replace('''        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw''',
'''        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        # IK solves for panda_link8, which is panda_hand yawed by +45 deg
        # about the shared z axis (verified via FK); convert hand quat -> link8 quat
        from scipy.spatial.transform import Rotation as Rot
        q8 = (Rot.from_quat([qx, qy, qz, qw]) * Rot.from_euler("z", np.pi / 4)).as_quat()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)''')
open('move_tcp.py','w').write(s)
EOF
timeout 600 python3 move_tcp.py -0.059 0.02 0.96 0.7071068 0.7071068 0 0 2.5

# openrua op 29
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/move_tcp.py", "old_string": "    def move_joints(self, target, secs):\\n        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = list(ARM)\\n        pt = JointTrajectoryPoint(positions=[float(v) for v in target])\\n        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))\\n        goal.trajectory.points = [pt]\\n        send = self.traj.send_goal_async(goal)\\n        rclpy.spin_until_future_complete(self.node, send)\\n        res = send.result().get_result_async()\\n        rclpy.spin_until_future_complete(self.node, res)\\n        code = res.result().result.error_code\\n        j = self.joints()\\n        err = max(abs(j[n] - t) for n, t in zip(ARM, target))\\n        print(f\\"traj error_code={code} max_joint_err={err:.4f}\\")\\n        return code == 0 and err < 0.02", "new_string": "    def move_joints(self, target, secs, tries=4):\\n        # the controller lags on long goals (tolerance violation -5); resending\\n        # the same goal converges, so retry until the joints actually match\\n        for _ in range(tries):\\n            goal = FollowJointTrajectory.Goal()\\n            goal.trajectory.joint_names = list(ARM)\\n            pt = JointTrajectoryPoint(positions=[float(v) for v in target])\\n            pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))\\n            goal.trajectory.points = [pt]\\n            send = self.traj.send_goal_async(goal)\\n            rclpy.spin_until_future_complete(self.node, send)\\n            res = send.result().get_result_async()\\n            rclpy.spin_until_future_complete(self.node, res)\\n            code = res.result().result.error_code\\n            j = self.joints()\\n            err = max(abs(j[n] - t) for n, t in zip(ARM, target))\\n            print(f\\"traj error_code={code} max_joint_err={err:.4f}\\")\\n            if err < 0.02:\\n                return True\\n            secs = max(secs, 2.0)\\n        return False", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 30
timeout 900 python3 move_tcp.py -0.059 0.02 0.96 0.7071068 0.7071068 0 0 2.5

# openrua op 31
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 32
timeout 300 python3 tools/action/gripper_cmd.py 0.0; ros2 topic echo /joint_states --once 2>&1 | grep -A9 position | tail -2; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force

# openrua op 33
timeout 900 python3 move_tcp.py -0.059 0.02 1.20 0.7071068 0.7071068 0 0 3 && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 34
python3 - <<'EOF'
import numpy as np
# 2D x-z cross-section of the cup (relative to bottom centre, axis up): (radius, h)
prof=[(0.028,0.0),(0.039,0.012),(0.05,0.10)]
def r(h):
    return np.interp(h,[p[1] for p in prof],[p[0] for p in prof])
hs=np.linspace(0,0.10,101)
# slot: back wall inner top edge E1=(-0.455,1.056), back wall inner face x=-0.455 ; divider top edge E2=(-0.40,0.989), divider left face x=-0.40
E1=np.array([-0.455,1.056]); E2=np.array([-0.40,0.989])
def cup_pts(B,th):  # th>0 leans back (top toward -x)
    ax=np.array([-np.sin(th),np.cos(th)]); rad=np.array([np.cos(th),np.sin(th)])
    left=[B+h*ax-r(h)*rad for h in hs]; right=[B+h*ax+r(h)*rad for h in hs]
    return np.array(left),np.array(right)
def penetrates(B,th):
    L,R=cup_pts(B,th)
    P=np.vstack([L,R])
    # back wall region: x<-0.455 and z<1.056 ; divider: x>-0.40 and z<0.989 (x<-0.39 ignore beyond) ; floor z<0.90
    bad=((P[:,0]<-0.455)&(P[:,1]<1.056))|((P[:,0]>-0.40)&(P[:,1]<0.989))|(P[:,1]<0.90)
    return bad.any()
best=None
for th in np.radians(np.arange(0,45,1.0)):
    # lower the cup from high for each x offset; find the lowest non-penetrating z; choose x giving lowest CoM x-behind
    for bx in np.arange(-0.46,-0.39,0.002):
        z=1.10
        while z>0.90 and not penetrates(np.array([bx,z-0.001]),th): z-=0.001
        B=np.array([bx,z]); ax=np.array([-np.sin(th),np.cos(th)])
        com=B+0.045*ax
        L,R=cup_pts(B,th)
        # stability heuristic: com x must lie between support contacts; require both edges touched approx
        touchE1=np.min(np.linalg.norm(L-E1,axis=1))<0.003 or np.min(np.abs(L[:,0]-(-0.455))+np.abs(np.clip(L[:,1]-1.056,0,None)))<0.003
        touchE2=np.min(np.linalg.norm(np.vstack([L,R])-E2,axis=1))<0.003
        if touchE1 and touchE2 and com[0]<-0.40:
            if best is None or com[1]<best[0]:
                best=(com[1],np.degrees(th),bx,z,com)
print('lowest CoM leaning pose:',best)
EOF

# openrua op 35
python3 - <<'EOF'
import numpy as np
prof=[(0.028,0.0),(0.039,0.012),(0.05,0.10)]
def r(h): return np.interp(h,[p[1] for p in prof],[p[0] for p in prof])
hs=np.linspace(0,0.10,101)
def cup_pts(B,th):
    ax=np.array([-np.sin(th),np.cos(th)]); rad=np.array([np.cos(th),np.sin(th)])
    return np.array([B+h*ax-r(h)*rad for h in hs]),np.array([B+h*ax+r(h)*rad for h in hs])
def pen(B,th):
    L,R=cup_pts(B,th); P=np.vstack([L,R])
    return (((P[:,0]<-0.455)&(P[:,1]<1.056))|((P[:,0]>-0.40)&(P[:,1]<0.989))|(P[:,1]<0.90)).any()
for thd in [0,10,20,30,40]:
    th=np.radians(thd)
    print('theta',thd)
    for bx in np.arange(-0.46,-0.385,0.005):
        z=1.12
        while z>0.90 and not pen(np.array([bx,z-0.001]),th): z-=0.001
        B=np.array([bx,z]); ax=np.array([-np.sin(th),np.cos(th)]); com=B+0.045*ax
        L,R=cup_pts(B,th)
        # which contacts: closest points to E1 and E2
        d1=np.min(np.hypot(L[:,0]+0.455,L[:,1]-1.056)); d2=np.min(np.hypot(np.vstack([L,R])[:,0]+0.40,np.vstack([L,R])[:,1]-0.989))
        print('  bx %.3f  bottom z %.3f  com (%.3f,%.3f)  dE1 %.3f dE2 %.3f'%(bx,z,com[0],com[1],d1,d2))
EOF

# openrua op 36
timeout 900 python3 move_tcp.py -0.059 0.02 1.25 0.3826834 0.9238795 0 0 3

# openrua op 37
timeout 900 python3 move_tcp.py -0.374 -0.107 1.25 0.3826834 0.9238795 0 0 4 && timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 tools/perception/cam_snap.py sideview

# openrua op 38
w() { ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force | tr '\n' ' '; echo; }
w; timeout 900 python3 move_tcp.py -0.374 -0.107 1.10 0.3826834 0.9238795 0 0 3 | tail -2; w

# openrua op 39
w() { ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force | tr '\n' ' '; echo; }
timeout 900 python3 move_tcp.py -0.374 -0.107 1.07 0.3826834 0.9238795 0 0 2 | tail -2; w; timeout 120 python3 tools/perception/cam_snap.py sideview

# openrua op 40
w() { ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force | tr '\n' ' '; echo; }
timeout 900 python3 move_tcp.py -0.374 -0.107 1.06 0.3826834 0.9238795 0 0 2 | tail -2; w

# openrua op 41
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 900 python3 move_tcp.py -0.374 -0.107 1.25 0.3826834 0.9238795 0 0 3 | tail -2; for c in agentview sideview birdview frontview; do timeout 120 python3 tools/perception/cam_snap.py $c; done

# openrua op 42
timeout 120 python3 cloud.py birdview | tail -1; timeout 120 python3 cloud.py agentview | tail -1; timeout 120 python3 cloud.py sideview | tail -1; python3 -c "
import numpy as np
for c in ['birdview','agentview','sideview']:
    d=np.load(c+'_cloud.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
    # cup: points above the caddy walls region near the middle slot, z>1.06 (above all caddy walls) plus lower
    m=(xyz[:,0]>-0.50)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.05)&(xyz[:,2]>0.95)&(xyz[:,2]<1.2)
    p=xyz[m]
    print(c, len(p))
    for z0 in np.arange(0.95,1.12,0.01):
        s=p[(p[:,2]>=z0)&(p[:,2]<z0+0.01)]
        if len(s): print('  z %.2f n=%4d x[%.3f,%.3f] y[%.3f,%.3f]'%(z0,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
"

# openrua op 43
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[130:260,180:360]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('frontview.png'); crop=im[200:360,150:420]; cv2.imwrite('front_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 44
timeout 900 python3 move_tcp.py -0.30 0.30 1.30 0.7071068 0.7071068 0 0 4 | tail -2; for c in birdview frontview agentview; do timeout 120 python3 tools/perception/cam_snap.py $c; done; python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[130:260,180:360]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('frontview.png'); crop=im[200:360,150:420]; cv2.imwrite('front_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 45
timeout 120 python3 cloud.py birdview | tail -1; python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
m=(xyz[:,0]>-0.50)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.05)&(xyz[:,2]>1.06)
p=xyz[m]; print('rim pts',len(p),'z',p[:,2].min().round(3),p[:,2].max().round(3))
print('x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3))
# inside-bottom of the cup visible from above: lowest points within the rim footprint
cx,cy=(p[:,0].min()+p[:,0].max())/2,(p[:,1].min()+p[:,1].max())/2
print('rim centre approx',round(cx,3),round(cy,3))
m2=(np.hypot(xyz[:,0]-cx,xyz[:,1]-cy)<0.03)&(xyz[:,2]>0.9)
q=xyz[m2]; print('inside-bottom z',q[:,2].min().round(3))
"

# openrua op 46
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
cx,cy=-0.404,-0.136
m2=(np.hypot(xyz[:,0]-cx,xyz[:,1]-cy)<0.035)&(xyz[:,2]>0.89)
q=xyz[m2]
h,e=np.histogram(q[:,2],bins=np.arange(0.89,1.11,0.01))
for a,b in zip(e,h): print('%.2f %d'%(a,b))
"
