#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace/tools/action && cat fjt_send.py; echo ----; cat gripper_cmd.py; echo ----; cat ik_move.py

# openrua op 3
cd /workspace && timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | head -40; echo ----; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|child_frame_id" | sort | uniq -c; echo ---; timeout 15 ros2 topic echo /tf --once 2>/dev/null | grep -E "frame_id" | sort | uniq -c

# openrua op 6
timeout 30 ros2 topic echo /tf --once 2>/dev/null | python3 -c "
import sys,yaml
docs=yaml.safe_load(sys.stdin)
for t in docs['transforms']:
    tr=t['transform']['translation']; r=t['transform']['rotation']
    print(t['child_frame_id'], [round(tr[k],3) for k in 'xyz'], [round(r[k],3) for k in 'xyzw'])
"

# openrua op 7
timeout 30 ros2 topic echo /tf --once 2>/dev/null | python3 -c "
import sys,yaml
for docs in yaml.safe_load_all(sys.stdin):
  if not docs: continue
  for t in docs['transforms']:
    tr=t['transform']['translation']; r=t['transform']['rotation']
    print(t['child_frame_id'], [round(tr[k],3) for k in 'xyz'], [round(r[k],3) for k in 'xyzw'])
  break
"

# openrua op 8
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+info+TF for a camera, save world-frame point cloud as npz.

Usage: python3 cloud.py <camera>
Writes <camera>_cloud.npz with keys: xyz (H,W,3) world coords, bgr (H,W,3).
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


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


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1)
    xyz = pc @ R.T + p0
    np.savez(f"{cam}_cloud.npz", xyz=xyz, bgr=color, depth=depth)
    print(f"{cam}_cloud.npz  {W}x{H}  depth range {np.nanmin(depth):.3f}..{np.nanmax(depth):.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 9
timeout 90 python3 cloud.py birdview && timeout 90 python3 cloud.py agentview && timeout 90 python3 cloud.py robot0_robotview

# openrua op 10
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
# table height: mode of z in central region
z=xyz[...,2]
h,_=np.histogram(z[np.isfinite(z)], bins=np.arange(0,1.5,0.01))
for i in np.argsort(h)[-6:][::-1]: print(f'z~{i*0.01:.2f} count {h[i]}')
# find objects: pixels above table height + 2cm, and within table region
"


# openrua op 11
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]
mask=((z>0.435)&(z<0.75)).astype(np.uint8)
# exclude robot: robot base around x=-0.51,y=0 ; robot arm is grey; keep everything and print
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    p=xyz[m]; c=bgr[m].mean(0)
    print(f'comp {i}: px={stats[i,4]} centroid px=({cent[i][0]:.0f},{cent[i][1]:.0f}) x={p[:,0].min():.3f}..{p[:,0].max():.3f} y={p[:,1].min():.3f}..{p[:,1].max():.3f} ztop={p[:,2].max():.3f} bgr={c.astype(int)}')
"


# openrua op 12
python3 -c "
import numpy as np, cv2
for cam in ['agentview','robot0_robotview']:
    d=np.load(cam+'_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
    z=xyz[...,2]
    mask=((z>0.44)&(z<0.75)&(xyz[...,0]>-0.4)).astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
    print('==',cam)
    for i in range(1,n):
        if stats[i,4]<40: continue
        m=lab==i
        p=xyz[m]; c=bgr[m].mean(0)
        print(f'comp {i}: px={stats[i,4]} centroid px=({cent[i][0]:.0f},{cent[i][1]:.0f}) x={p[:,0].min():.3f}..{p[:,0].max():.3f} y={p[:,1].min():.3f}..{p[:,1].max():.3f} ztop={p[:,2].max():.3f} bgr={c.astype(int)}')
"


# openrua op 13
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
got=[]
q=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),q)
import time; t=time.time()
while not got and time.time()-t<15: rclpy.spin_once(n,timeout_sec=0.2)
open('robot.urdf','w').write(got[0]); print(len(got[0]))
" && grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -i "camera\|eye_in_hand" robot.urdf | head

# openrua op 14
timeout 90 python3 -c "
import rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node('fk')
cli=n.create_client(GetPositionFK,'/compute_fk'); cli.wait_for_service(10)
req=GetPositionFK.Request()
req.fk_link_names=['panda_hand']
req.header.frame_id=''
js=JointState(); js.name=['panda_joint%d'%i for i in range(1,8)]
js.position=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
req.robot_state.joint_state=js
f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=60)
r=f.result(); print(r.error_code, r.pose_stamped[0].header.frame_id, r.pose_stamped[0].pose)
"
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
# table z around butter and can
for name,(x,y) in {'butter':(0.07,0.04),'can':(-0.12,-0.16),'basket':(0.01,0.26)}.items():
    m=(abs(xyz[...,0]-x)<0.06)&(abs(xyz[...,1]-y)<0.06)&(xyz[...,2]<0.435)
    print(name,'table z median',np.median(xyz[m][:,2]) if m.sum() else None, m.sum())
"

# openrua op 15
python3 -c "
import numpy as np
def T(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
def fk(q):
    A=[0,0,0,0.0825,-0.0825,0,0.088]; D=[0.333,0,0.316,0,0.384,0,0]; AL=[0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2]
    M=np.eye(4)
    for i in range(7): M=M@T(A[i],D[i],AL[i],q[i])
    M=M@T(0,0.107,0,0)@T(0,0,0,-np.pi/4)
    return M
M=fk([0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483])
print(M[:3,3])
"


# openrua op 16
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable arm helper: one node, IK/FJT/gripper clients built once.

Usage as CLI:
  python3 arm.py js                         # print joint state
  python3 arm.py ik x y z qx qy qz qw       # solve only, print joints
  python3 arm.py tcp x y z [yaw_deg] [secs] # move TCP (top-down) to world xyz
  python3 arm.py grip open|close
"""
import sys, time, math
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def topdown_quat(yaw_deg=0.0):
    """Hand z pointing down (world -z); yaw rotates finger axis about world z.
    yaw=0 -> fingers open along world y."""
    # q = rotz(yaw) * rotx(180deg)
    h = math.radians(yaw_deg) / 2
    qz = np.array([0, 0, math.sin(h), math.cos(h)])  # x,y,z,w
    qx = np.array([1, 0, 0, 0])
    # quaternion multiply qz * qx
    x1, y1, z1, w1 = qz; x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.update(dict(zip(m.name, m.position))), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def joints(self):
        self._js.clear()
        t = time.time()
        while len(self._js) < 9 and time.time() - t < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def solve_ik(self, x, y, z, q, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        js = self.joints() if seed is None else seed
        s = JointState(); s.name = list(JOINTS); s.position = [float(js[j]) for j in JOINTS]
        req.ik_request.robot_state.joint_state = s
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        f = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        r = f.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"IK failed: {None if r is None else r.error_code.val}")
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move_joints(self, positions, seconds=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        f = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        js = self.joints()
        err = max(abs(js[j] - p) for j, p in zip(JOINTS, positions))
        print(f"  fjt error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, x, y, z, yaw_deg=0.0, seconds=3.0, seed=None):
        q = topdown_quat(yaw_deg)
        R = quat_R(*q)
        hx, hy, hz = np.array([x, y, z]) - TCP * R[:, 2]
        sol = self.solve_ik(hx, hy, hz, q, seed)
        print(f"  tcp->({x:.3f},{y:.3f},{z:.3f}) yaw={yaw_deg} joints={np.round(sol,3).tolist()}")
        return self.move_joints(sol, seconds)

    def gripper(self, open_):
        goal = GripperCommand.Goal()
        goal.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        goal.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        js = self.joints()
        print(f"  gripper {'open' if open_ else 'close'}: reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={js.get('panda_finger_joint1'):.4f},{js.get('panda_finger_joint2'):.4f}")
        return js


if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "js":
        print(a.joints())
    elif cmd == "ik":
        x, y, z, qx, qy, qz, qw = map(float, sys.argv[2:9])
        print(a.solve_ik(x, y, z, (qx, qy, qz, qw)))
    elif cmd == "tcp":
        x, y, z = map(float, sys.argv[2:5])
        yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
        secs = float(sys.argv[6]) if len(sys.argv) > 6 else 3.0
        a.move_tcp(x, y, z, yaw, secs)
    elif cmd == "grip":
        a.gripper(sys.argv[2] == "open")
    rclpy.shutdown()
OPENRUA_EOF

# openrua op 17
timeout 120 python3 arm.py ik -0.05298564807835497 0 0.7776238083193787 0.9995966352021468 0 -0.028400121347386207 0

# openrua op 18
python3 -c "
import numpy as np
def T(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
def fk(q):
    A=[0,0,0,0.0825,-0.0825,0,0.088]; D=[0.333,0,0.316,0,0.384,0,0]; AL=[0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2]
    M=np.eye(4)
    for i in range(7): M=M@T(A[i],D[i],AL[i],q[i])
    return M@T(0,0.107,0,0)@T(0,0,0,-np.pi/4)
for q in [[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483],[0.0584621198952894, -0.16130609283375688, -0.057919927987371105, -2.4445840518080897, -0.011620247260552972, 2.2267009177317854, 0.008905361655137594]]:
    M=fk(q); print(np.round(M[:3,3]+[-0.51,0,0.42],4)); print(np.round(M[:3,:3],3))
"


# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.ik_link_name = M[\\"frames\\"][\\"hand\\"]  # group tip is link8 (45deg off)\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
timeout 120 python3 arm.py ik -0.05298564807835497 0 0.7776238083193787 0.9995966352021468 0 -0.028400121347386207 0

# openrua op 21
python3 -c "
import numpy as np, struct
f='/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/hand.stl'
b=open(f,'rb').read()
if b[:5]==b'solid':
    import re
    v=np.array([[float(x) for x in l.split()[1:]] for l in b.decode().splitlines() if l.strip().startswith('vertex')])
else:
    n=struct.unpack_from('<I',b,80)[0]
    arr=np.frombuffer(b[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    v=arr['v'].reshape(-1,3)
print('hand extents min',v.min(0),'max',v.max(0))
"
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
m=(xyz[...,2]>0.44)&(xyz[...,2]<0.47)&(abs(xyz[...,0]-0.067)<0.06)&(abs(xyz[...,1]-0.04)<0.05)
p=xyz[m][:,:2]; c=p.mean(0); u,s,vt=np.linalg.svd(p-c)
print('butter center',c,'axis',vt[0],'angle deg',np.degrees(np.arctan2(vt[0][1],vt[0][0])), 'n',len(p))
"

# openrua op 22
mkdir -p "$(dirname /workspace/pick_can.py)"
cat > /workspace/pick_can.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
import sys
sys.path.insert(0, "/workspace")
from arm import Arm
import rclpy

CAN = (-0.122, -0.163)      # world xy of alphabet soup can (birdview)
GRASP_Z = 0.465             # TCP height: hand bottom 0.51 > can top 0.50
PRE_Z = 0.62
CARRY_Z = 0.76
DROP = (-0.015, 0.232)      # inside basket, -y side
DROP_Z = 0.70               # hand bottom 0.745, can bottom ~0.66 > rim 0.628

a = Arm()
print("open gripper"); a.gripper(True)
print("pre-grasp"); a.move_tcp(CAN[0], CAN[1], PRE_Z, 0, 4)
print("descend"); a.move_tcp(CAN[0], CAN[1], GRASP_Z, 0, 2.5)
print("close"); js = a.gripper(False)
gap = js["panda_finger_joint1"]
if gap < 0.01:
    print("GRASP FAILED: fingers closed to", gap); rclpy.shutdown(); sys.exit(1)
print("lift"); a.move_tcp(CAN[0], CAN[1], CARRY_Z, 0, 2.5)
js = a.joints(); print("  fingers after lift", js["panda_finger_joint1"])
print("to basket"); a.move_tcp(DROP[0], DROP[1], CARRY_Z, 0, 4)
print("lower"); a.move_tcp(DROP[0], DROP[1], DROP_Z, 0, 2)
print("release"); a.gripper(True)
print("retreat"); a.move_tcp(DROP[0], DROP[1], CARRY_Z, 0, 2)
print("DONE")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 23
nohup python3 -u pick_can.py > pick_can.log 2>&1 &
sleep 45; cat /workspace/pick_can.log

# openrua op 24
sleep 40; cat /workspace/pick_can.log

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_robotview

# openrua op 26
mkdir -p "$(dirname /workspace/pick_butter.py)"
cat > /workspace/pick_butter.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
import sys
sys.path.insert(0, "/workspace")
from arm import Arm
import rclpy

BUT = (0.067, 0.048)        # butter center (0.067,0.038) +1cm y for milk clearance
GRASP_Z = 0.44              # fingertips ~0.431 (table 0.425), hand bottom 0.485 > top 0.449
PRE_Z = 0.60
CARRY_Z = 0.76
DROP = (0.01, 0.295)        # inside basket, +y side (can is at y~0.23)
DROP_Z = 0.70

a = Arm()
print("open gripper"); a.gripper(True)
print("pre-grasp"); a.move_tcp(BUT[0], BUT[1], PRE_Z, 0, 4)
print("descend"); a.move_tcp(BUT[0], BUT[1], GRASP_Z, 0, 2.5)
js = a.joints()
print("close"); js = a.gripper(False)
gap = js["panda_finger_joint1"]
if gap < 0.008:
    print("GRASP FAILED: fingers closed to", gap); rclpy.shutdown(); sys.exit(1)
print("lift"); a.move_tcp(BUT[0], BUT[1], CARRY_Z, 0, 3)
js = a.joints(); print("  fingers after lift", js["panda_finger_joint1"])
print("to basket"); a.move_tcp(DROP[0], DROP[1], CARRY_Z, 0, 4)
print("lower"); a.move_tcp(DROP[0], DROP[1], DROP_Z, 0, 2)
print("release"); a.gripper(True)
print("retreat"); a.move_tcp(DROP[0], DROP[1], CARRY_Z, 0, 2)
print("DONE")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 27
nohup python3 -u pick_butter.py > pick_butter.log 2>&1 &
sleep 50; cat /workspace/pick_butter.log

# openrua op 28
sleep 35; cat /workspace/pick_butter.log | tail -8

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 90 python3 cloud.py birdview && python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]
# basket footprint from earlier: x -0.073..0.083, y 0.174..0.345 ; interior above table
m=(xyz[...,0]>-0.05)&(xyz[...,0]<0.065)&(xyz[...,1]>0.195)&(xyz[...,1]<0.325)&(z>0.43)&(z<0.62)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m.astype(np.uint8))
print('objects inside basket footprint (below rim, above floor):')
for i in range(1,n):
    if stats[i,4]<10: continue
    p=xyz[lab==i]; c=bgr[lab==i].mean(0)
    print(f'  x={p[:,0].mean():.3f} y={p[:,1].mean():.3f} ztop={p[:,2].max():.3f} px={stats[i,4]} bgr={c.astype(int)}')
# anything left at the original spots?
for name,(x,y) in {'can spot':(-0.122,-0.163),'butter spot':(0.067,0.038)}.items():
    mm=(abs(xyz[...,0]-x)<0.04)&(abs(xyz[...,1]-y)<0.03)&(z>0.44)&(z<0.7)
    print(name,'pixels above table:',int(mm.sum()))
"

# openrua op 30
pgrep -af "pick_|arm.py|cloud.py" || echo "no pending processes"; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position | head -10
