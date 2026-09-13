#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list 2>&1; echo ---; ros2 action list 2>&1; echo ---; ros2 service list 2>&1 | head -40; echo ---; ros2 node list 2>&1

# openrua op 4
ros2 topic echo /joint_states --once 2>&1; for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview galleryview paperview; do python3 tools/perception/cam_snap.py $c & done; wait; ls *.png

# openrua op 5
echo "left pot lid:"; python3 tools/perception/px2world.py birdview 330 318; echo "right pot lid:"; python3 tools/perception/px2world.py birdview 385 278; echo "stove center:"; python3 tools/perception/px2world.py birdview 262 283; echo "stove corner top-left:"; python3 tools/perception/px2world.py birdview 238 258; echo "stove corner bottom-right:"; python3 tools/perception/px2world.py birdview 287 308; echo "table:"; python3 tools/perception/px2world.py birdview 400 380; echo "knob:"; python3 tools/perception/px2world.py birdview 262 238

# openrua op 6
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ---; ros2 topic echo /tf_static --once 2>&1 | head -80

# openrua op 7
mkdir -p "$(dirname /workspace/scan.py)"
cat > /workspace/scan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects above the table in a top-down camera; print world stats."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"


def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


rclpy.init()
node = rclpy.create_node("scan")
buf = Buffer(); TransformListener(buf, node)
d = grab(node, f"/{cam}/depth/image_raw", Image)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
frame = f"{cam}_optical_frame"
while not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation
R = quat_R(q.x, q.y, q.z, q.w)
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
v, u = np.mgrid[0:d.height, 0:d.width]
z = depth
pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], -1)
W = pc @ R.T + tr
np.save(f"{cam}_world.npy", W)
print("camera at", tr, "R=\n", R)
# objects above table (z>0.905) within table region
mask = (W[..., 2] > 0.905) & np.isfinite(W[..., 2])
from scipy import ndimage
lab, n = ndimage.label(mask)
for i in range(1, n + 1):
    m = lab == i
    if m.sum() < 30:
        continue
    P = W[m]
    print(f"blob {i}: n={m.sum()} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] "
          f"y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}] "
          f"centroid=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) "
          f"px u[{u[m].min()},{u[m].max()}] v[{v[m].min()},{v[m].max()}]")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 8
python3 scan.py birdview 2>&1

# openrua op 9
python3 -c "
import numpy as np
W=np.load('birdview_world.npy')
for name,(x0,x1,y0,y1) in {'left':(0.0,0.14,-0.08,0.12),'right':(-0.14,0.0,0.12,0.32)}.items():
    m=(W[...,0]>x0)&(W[...,0]<x1)&(W[...,1]>y0)&(W[...,1]<y1)&(W[...,2]>0.905)
    P=W[m]
    print(name, 'top z', P[:,2].max())
    for zlo in [0.905,0.95,0.98,1.0,1.02,1.04,1.05]:
        Q=P[P[:,2]>zlo]
        if len(Q)==0: continue
        print(f'  z>{zlo}: n={len(Q)} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}] c=({Q[:,0].mean():.3f},{Q[:,1].mean():.3f})')
"


# openrua op 10
python3 scan.py sideview 2>&1 | head -3; python3 scan.py frontview 2>&1 | head -3; python3 scan.py agentview 2>&1 | head -3

# openrua op 11
python3 -c "
import numpy as np
for cam in ['sideview','frontview','agentview']:
    W=np.load(f'{cam}_world.npy')
    print('==',cam)
    for name,(x0,x1,y0,y1) in {'left':(0.0,0.14,-0.08,0.12),'right':(-0.14,0.0,0.12,0.32),'stove':(-0.16,0.06,-0.32,-0.10)}.items():
        m=(W[...,0]>x0)&(W[...,0]<x1)&(W[...,1]>y0)&(W[...,1]<y1)&(W[...,2]>0.902)&np.isfinite(W[...,2])
        P=W[m]
        if len(P)==0: print(name,'none'); continue
        print(name, 'n',len(P),'top z %.3f'%P[:,2].max())
        for zlo in np.arange(0.90,1.06,0.01):
            Q=P[(P[:,2]>zlo)&(P[:,2]<=zlo+0.01)]
            if len(Q)<3: continue
            print(f'  z[{zlo:.2f},{zlo+0.01:.2f}]: n={len(Q):4d} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}]')
"


# openrua op 12
ros2 service call /get_urdf std_srvs/srv/Trigger 2>&1 | head -5; ros2 service type /get_urdf; python3 - <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('u')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
import time
t=time.time()
while not got and time.time()-t<10: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data if got else '')
print(len(got[0].data) if got else 'none')
EOF
grep -n -A12 'link name="panda_hand"\|link name="panda_leftfinger"\|link name="panda_rightfinger"' robot.urdf | head -80

# openrua op 13
timeout 30 python3 - <<'EOF'
import rclpy, time
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('u')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
t=time.time()
while not got and time.time()-t<15: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data if got else '')
print(len(got[0].data) if got else 'none')
EOF
grep -n -B2 -A14 'link name="panda_hand"\|link name="panda_leftfinger"\|link name="panda_rightfinger"' robot.urdf | grep -v '^--$' | head -90

# openrua op 14
python3 -c "
import numpy as np
W=np.load('frontview_world.npy')
m=(W[...,0]>0.02)&(W[...,0]<0.13)&(W[...,1]>-0.08)&(W[...,1]<0.0)&(W[...,2]>0.94)&(W[...,2]<1.06)&np.isfinite(W[...,2])
P=W[m]
# occupancy grid y (cols, 5mm) vs z (rows, 5mm)
ys=np.arange(-0.08,0.0,0.005); zs=np.arange(1.06,0.94,-0.005)
print('      '+''.join(f'{y*100:5.1f}' for y in ys[::2]))
for z in zs:
    row=''
    for y in ys:
        q=P[(P[:,1]>=y)&(P[:,1]<y+0.005)&(P[:,2]>=z-0.005)&(P[:,2]<z)]
        row+= (f'{q[:,0].mean()*100:4.1f} ' if len(q)>0 else '  .  ')[:5] if False else ('  #  ' if len(q)>0 else '  .  ')[:5]
    print(f'{z:.3f} '+row.replace('  #  ','# ').replace('  .  ','. '))
# x extent of handle points
H=P[(P[:,1]<-0.015)]
print('handle x range', H[:,0].min(), H[:,0].max(), 'mean', H[:,0].mean())
"


# openrua op 15
python3 -c "
import numpy as np, struct
def stl(path):
    d=open(path,'rb').read()
    if d[:5]==b'solid':
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)',d)])
    else:
        n=struct.unpack('<I',d[80:84])[0]
        a=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
        v=a['v'].reshape(-1,3)
    return v
for f in ['hand','finger']:
    v=stl(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl')
    print(f, 'min',v.min(0),'max',v.max(0))
"


# openrua op 16
find / -iname 'hand*.stl' -o -iname 'finger*.stl' -o -iname '*moka*' 2>/dev/null | grep -v proc | head; ls /opt/ros/jazzy/share/ | grep -i panda

# openrua op 17
python3 -c "
import numpy as np
W=np.load('birdview_world.npy')
m=(W[...,0]>-0.35)&(W[...,0]<-0.05)&(np.abs(W[...,1])<0.2)&(W[...,2]>1.1)&(W[...,2]<1.35)&np.isfinite(W[...,2])
P=W[m]
for zlo in np.arange(1.15,1.34,0.01):
    Q=P[(P[:,2]>zlo)&(P[:,2]<=zlo+0.01)]
    if len(Q)<3: continue
    print(f'z[{zlo:.2f},{zlo+0.01:.2f}] n={len(Q):4d} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}]')
"


# openrua op 18
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position | tail -2; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_open.png

# openrua op 19
timeout 120 python3 - <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
rclpy.init(); n=rclpy.create_node('e')
def grab(topic,T):
    got={}
    s=n.create_subscription(T,topic,lambda m:got.setdefault('m',m),1)
    while 'm' not in got: rclpy.spin_once(n,timeout_sec=0.2)
    n.destroy_subscription(s); return got['m']
d=grab('/robot0_eye_in_hand/depth/image_raw',Image); info=grab('/robot0_eye_in_hand/color/camera_info',CameraInfo)
depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
print('K',fx,fy,cx,cy, 'frame', d.header.frame_id)
np.save('eih_depth.npy',depth)
# near objects (fingers/hand): depth < 0.15
v,u=np.mgrid[0:d.height,0:d.width]
X=(u-cx)*depth/fx; Y=(v-cy)*depth/fy
for row in [370,380,390,400,410,420,440,470]:
    dd=depth[row]; 
    print(row, ' '.join(f'{x:.3f}' for x in dd[::40]))
# Find finger pixels: depth<0.12 and row>350
m=(depth<0.13)&(v>340)
print('near px n',m.sum())
for lab,(u0,u1) in {'left':(0,320),'right':(320,640)}.items():
    mm=m&(u>=u0)&(u<u1)
    if mm.sum()==0: print(lab,'none'); continue
    print(lab,'X[%.4f,%.4f] Y[%.4f,%.4f] Z[%.4f,%.4f]'%(X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max(),depth[mm].min(),depth[mm].max()))
EOF

# openrua op 20
python3 - <<'EOF'
import numpy as np
depth=np.load('eih_depth.npy'); fx=fy=312.774; cx,cy=320,240
v,u=np.mgrid[0:480,0:640]
X=(u-cx)*depth/fx; Y=(v-cy)*depth/fy
m=(depth>0.07)&(depth<0.12)&(v<408)
for lab,sel in {'left':m&(u<320),'right':m&(u>=320)}.items():
    print(lab,'n',sel.sum(),'X[%.4f,%.4f] Y[%.4f,%.4f] Z[%.4f,%.4f]'%(X[sel].min(),X[sel].max(),Y[sel].min(),Y[sel].max(),depth[sel].min(),depth[sel].max()))
    # inner edge at each depth band
    for z0 in np.arange(0.075,0.11,0.005):
        s=sel&(depth>=z0)&(depth<z0+0.005)
        if s.sum(): print(f'   z[{z0:.3f}] inner X={X[s].max() if lab=="left" else X[s].min():.4f} Y[{Y[s].min():.3f},{Y[s].max():.3f}]')
EOF

# openrua op 21
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Robot helper: joint state, FK/IK (MoveIt), trajectories, gripper.

World <-> base: base = world - BASE_IN_WORLD.  IK/FK work in the base frame.
"""
import math, sys, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.660, 0.0, 0.912])
TCP = M["hand"]["tcp_offset_m"]


def R_to_quat(R):
    m = R
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s; x = (m[2, 1] - m[1, 2]) / s; y = (m[0, 2] - m[2, 0]) / s; z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s; x = 0.25 * s; y = (m[0, 1] + m[1, 0]) / s; z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s; x = (m[0, 1] + m[1, 0]) / s; y = 0.25 * s; z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s; x = (m[0, 2] + m[2, 0]) / s; y = (m[1, 2] + m[2, 1]) / s; z = 0.25 * s
    return np.array([x, y, z, w])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def frame_from_axes(Yh, Zh):
    """Rotation whose hand-Y (finger axis) and hand-Z (approach) are given."""
    Yh = np.asarray(Yh, float); Yh /= np.linalg.norm(Yh)
    Zh = np.asarray(Zh, float); Zh /= np.linalg.norm(Zh)
    Xh = np.cross(Yh, Zh)
    return np.column_stack([Xh, Yh, Zh])


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.update(zip(m.name, m.position)), 1)
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.joints()

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        self._js.clear()
        while not all(j in self._js for j in ARM):
            self.spin(0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return np.array([js[j] for j in ARM])

    def finger_gap(self):
        js = self.joints()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    # ---------- kinematics (base frame) ----------
    def fk(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = list(map(float, self.arm_q() if q is None else q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def fk_world(self, q=None):
        p, quat = self.fk(q)
        return p + BASE_IN_WORLD, quat

    def ik(self, p_base, quat, seed=None, timeout=5.0, attempts=1):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        seed = self.arm_q() if seed is None else np.asarray(seed, float)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = list(map(float, seed))
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    def ik_world(self, p_world, quat, **kw):
        return self.ik(np.asarray(p_world) - BASE_IN_WORLD, quat, **kw)

    # ---------- motion ----------
    def move_joints(self, q_list, times):
        """Send one trajectory through q_list (list of 7-vectors) at cumulative times."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(q_list, times):
            pt = JointTrajectoryPoint(positions=list(map(float, q)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        if not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q_now = self.arm_q()
        err = np.abs(q_now - np.asarray(q_list[-1])).max()
        print(f"  traj done code={code} max|dq|={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        gap = self.finger_gap()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap


if __name__ == "__main__":
    r = Robot()
    q = r.arm_q()
    print("q", np.round(q, 4))
    p, quat = r.fk_world()
    print("hand world", np.round(p, 4), "quat", np.round(quat, 4))
    print("R\n", np.round(quat_to_R(quat), 3))
    print("finger gap", r.finger_gap())
OPENRUA_EOF

# openrua op 22
timeout 120 python3 rob.py

# openrua op 23
timeout 120 python3 -c "
from rob import *
r=Robot()
print('link0', r.fk(link='panda_link0'))
print('hand ', r.fk(link='panda_hand'))
import rclpy
from moveit_msgs.srv import GetPositionFK
req=GetPositionFK.Request(); req.header.frame_id=''; req.fk_link_names=['panda_hand']
req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=list(map(float,r.arm_q()))
f=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30)
print('fk header frame:', f.result().pose_stamped[0].header.frame_id)
"

# openrua op 24
sed -i 's/^BASE_IN_WORLD = np.array(\[-0.660, 0.0, 0.912\])/BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # MoveIt model frame is world here (verified via FK of panda_link0)/' rob.py && grep -n BASE_IN_WORLD rob.py | head -3

# openrua op 25
timeout 600 python3 - <<'EOF'
from rob import *
r=Robot()
q0=r.arm_q()
pot=np.array([0.072,0.027]); waist=0.972
cands={}
for sgn,name in [(1,'Y=+x'),(-1,'Y=-x')]:
    R=frame_from_axes([sgn,0,0],[0,-1,0]); q=R_to_quat(R)
    pads=np.array([pot[0]+0.0025, pot[1]+0.0043, waist])
    fl=pads+TCP*np.array([0,1,0])
    cands[f'side {name} grasp']=(fl,q)
    cands[f'side {name} pregrasp z1.10']=(fl+[0,0,0.128],q)
    cands[f'side {name} pregrasp y+0.06']=(fl+[0,0.06,0],q)
    cands[f'side {name} lift']=(fl+[0,0,0.15],q)
    stove=np.array([-0.046,-0.207])
    flp=np.array([stove[0]+0.0025, stove[1]+0.0043+TCP, 1.012])
    cands[f'side {name} place']=(flp,q)
    cands[f'side {name} place high']=(flp+[0,0,0.11],q)
    # diagonal 45 from above
    Zh=np.array([0,-1,-1])/np.sqrt(2); R=frame_from_axes([sgn,0,0],Zh); q=R_to_quat(R)
    fl=pads-TCP*Zh
    cands[f'diag {name} grasp']=(fl,q)
for k,(p,q) in cands.items():
    sol=r.ik_world(p,q,seed=q0,timeout=2.0)
    if sol is None:
        # retry with a different seed (elbow-up ready pose)
        sol=r.ik_world(p,q,seed=[0,-0.785,0,-2.356,0,1.571,0.785],timeout=2.0)
    print(f'{k:32s} p={np.round(p,3)} -> {None if sol is None else np.round(sol,3)}')
EOF

# openrua op 26
timeout 900 python3 - <<'EOF'
from rob import *
r=Robot()
q0=r.arm_q()
ready=[0,-0.785,0,-2.356,0,1.571,0.785]
pot=np.array([0.072,0.027])
def pose(theta_deg, z, back=0.005, sgn=1):
    th=np.radians(theta_deg)
    Zh=np.array([np.sin(th),-np.cos(th),0.0])
    Yh=sgn*np.array([np.cos(th),np.sin(th),0.0])
    pads=np.array([pot[0],pot[1],z]) - back*Zh
    fl=pads - TCP*Zh
    return fl, R_to_quat(frame_from_axes(Yh,Zh)), pads
for sgn in (1,-1):
  for th in [0,10,15,20,30]:
    for z in [0.965,0.972,1.0,1.05]:
        fl,q,pads=pose(th,z,sgn=sgn)
        sol=None
        for seed in (q0,ready,[0.3,1.2,0,-1.0,-1.0,1.3,2.0],[0.3,1.7,0.5,-0.05,-2.0,1.2,-1.4]):
            sol=r.ik_world(fl,q,seed=seed,timeout=1.0)
            if sol is not None: break
        if sol is None:
            print(f'sgn={sgn:+d} th={th:2d} z={z:.3f} flange={np.round(fl,3)} -> NONE'); continue
        p7,_=r.fk(sol,'panda_link7'); p6,_=r.fk(sol,'panda_link6'); p5,_=r.fk(sol,'panda_link5')
        print(f'sgn={sgn:+d} th={th:2d} z={z:.3f} flange={np.round(fl,3)} q={np.round(sol,2)} link7={np.round(p7,3)} link6={np.round(p6,3)} link5={np.round(p5,3)}')
EOF

# openrua op 27
mkdir -p "$(dirname /workspace/go.py)"
cat > /workspace/go.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Stage runner: python3 go.py <stage> [args]. Poses in world frame."""
import sys, json
import numpy as np
from rob import *

R_DOWN_X = frame_from_axes([1, 0, 0], [0, 0, -1])   # fingers close along world x, hand points down
Q_DOWN_X = R_to_quat(R_DOWN_X)


def goto(r, p_flange, quat, secs, seed=None, via=None):
    """IK then one trajectory (optionally through a via joint config)."""
    sol = r.ik_world(p_flange, quat, seed=r.arm_q() if seed is None else seed, timeout=3.0)
    if sol is None:
        for s in ([0, -0.785, 0, -2.356, 0, 1.571, 0.785], [0.3, 0.5, 0, -1.8, 0, 2.3, 0.8]):
            sol = r.ik_world(p_flange, quat, seed=s, timeout=3.0)
            if sol is not None:
                break
    if sol is None:
        raise SystemExit(f"IK failed for {p_flange}")
    print("  target q", np.round(sol, 3), flush=True)
    if via is not None:
        code, err = r.move_joints([via, sol], [secs * 0.5, secs])
    else:
        code, err = r.move_joints([sol], [secs])
    p, q = r.fk_world()
    print("  now flange", np.round(p, 4), "quat", np.round(q, 3), flush=True)
    return sol


def tcp_to_flange(p_tcp, quat):
    R = quat_to_R(quat)
    return np.asarray(p_tcp) - TCP * R[:, 2]


if __name__ == "__main__":
    r = Robot("go")
    st = sys.argv[1]
    a = [float(x) for x in sys.argv[2:]]
    if st == "tcp":            # go.py tcp x y z [secs]  (fingers along x, pointing down)
        secs = a[3] if len(a) > 3 else 4.0
        goto(r, tcp_to_flange(a[:3], Q_DOWN_X), Q_DOWN_X, secs)
    elif st == "tcpq":         # go.py tcpq x y z qx qy qz qw [secs]
        q = np.array(a[3:7]); secs = a[7] if len(a) > 7 else 4.0
        goto(r, tcp_to_flange(a[:3], q), q, secs)
    elif st == "grip":
        r.gripper(a[0])
    elif st == "state":
        p, q = r.fk_world()
        print("flange", np.round(p, 4), "quat", np.round(q, 4), "tcp", np.round(p + TCP * quat_to_R(q)[:, 2], 4))
        print("q", np.round(r.arm_q(), 4), "gap", round(r.finger_gap(), 4))
OPENRUA_EOF

# openrua op 28
timeout 900 python3 -u go.py tcp 0.072 -0.035 1.20 5 && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_above.png && python3 tools/perception/cam_snap.py agentview agent1.png

# openrua op 29
timeout 600 python3 -u go.py state

# openrua op 30
timeout 900 python3 -u go.py tcp 0.072 -0.035 1.20 8 && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_above.png

# openrua op 31
timeout 600 python3 -u go.py state

# openrua op 32
timeout 600 python3 -u -c "
from rob import *
r=Robot()
q=r.arm_q(); print('before', np.round(q,4))
t=q.copy(); t[6]=-1.572
r.move_joints([t],[2.0])
print('after ', np.round(r.arm_q(),4))
r.move_joints([t],[2.0])
print('after2', np.round(r.arm_q(),4))
"

# openrua op 33
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/go.py", "old_string": "    if via is not None:\\n        code, err = r.move_joints([via, sol], [secs * 0.5, secs])\\n    else:\\n        code, err = r.move_joints([sol], [secs])\\n    p, q = r.fk_world()", "new_string": "    if via is not None:\\n        code, err = r.move_joints([via, sol], [secs * 0.5, secs])\\n    else:\\n        code, err = r.move_joints([sol], [secs])\\n    for _ in range(4):                      # controller lag: resend short goals until converged\\n        if err < 0.01:\\n            break\\n        code, err = r.move_joints([sol], [2.0])\\n    if err >= 0.01:\\n        raise SystemExit(f\\"did not converge: max|dq|={err:.4f}\\")\\n    p, q = r.fk_world()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 34
python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_above.png && python3 tools/perception/cam_snap.py agentview agent1.png

# openrua op 35
timeout 120 python3 scan.py robot0_eye_in_hand 2>&1 | tail -20

# openrua op 36
python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_world.npy')
# handle region: x 0.05..0.10, y -0.07..-0.005, z 1.0..1.05
m=(W[...,0]>0.04)&(W[...,0]<0.11)&(W[...,1]>-0.075)&(W[...,1]<-0.008)&(W[...,2]>1.0)&(W[...,2]<1.05)
P=W[m]; print('n',len(P))
for lo in np.arange(-0.07,-0.008,0.005):
    s=P[(P[:,1]>=lo)&(P[:,1]<lo+0.005)]
    if len(s): print(f'y[{lo:.3f},{lo+0.005:.3f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] xmean={s[:,0].mean():.3f} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
# finger pads: z > 1.15
f=W[(W[...,2]>1.15)&(W[...,2]<1.28)]
print('fingers x',f[:,0].min(),f[:,0].max(),'y',f[:,1].min(),f[:,1].max())
"


# openrua op 37
timeout 300 python3 go.py tcp 0.070 -0.040 1.10 4 2>&1 | grep -v "^\[" ; timeout 300 python3 go.py tcp 0.070 -0.040 1.020 4 2>&1 | grep -v "^\["

# openrua op 38
timeout 60 python3 go.py state 2>&1 | grep -v "^\["; timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench | grep -A7 wrench; timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py frontview front2.png >/dev/null 2>&1; echo ok

# openrua op 39
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py sideview side2.png >/dev/null 2>&1; echo ok

# openrua op 40
timeout 120 python3 scan.py robot0_eye_in_hand 2>&1 | grep -E "camera|blob" ; python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_world.npy')
m=(W[...,0]>0.02)&(W[...,0]<0.12)&(W[...,1]>-0.08)&(W[...,1]<0.09)&np.isfinite(W[...,2])
P=W[m]
print('pot region max z',P[:,2].max())
top=P[P[:,2]>1.045]; print('top pts',len(top),'x',top[:,0].min(),top[:,0].max(),'y',top[:,1].min(),top[:,1].max())
# fingers
f=W[(W[...,2]>1.04)&(W[...,2]<1.14)&np.isfinite(W[...,2])]
print('finger-ish pts z',f[:,2].min(),f[:,2].max(),'x',f[:,0].min(),f[:,0].max(),'y',f[:,1].min(),f[:,1].max())
"

# openrua op 41
python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_world.npy')
m=(W[...,2]>1.0)&np.isfinite(W[...,2])
P=W[m]
for xlo in np.arange(0.0,0.13,0.01):
    s=P[(P[:,0]>=xlo)&(P[:,0]<xlo+0.01)]
    if len(s): print(f'x[{xlo:.2f},{xlo+0.01:.2f}] n={len(s)} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
print('---- z>1.045 by y')
Q=P[P[:,2]>1.045]
for ylo in np.arange(-0.08,0.04,0.01):
    s=Q[(Q[:,1]>=ylo)&(Q[:,1]<ylo+0.01)]
    if len(s): print(f'y[{ylo:.2f},{ylo+0.01:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
"

# openrua op 42
python3 - <<'EOF' 2>&1 | grep -v "^\["
from rob import *
r = Robot("t")
q = r.arm_q()
for link in ("panda_hand","panda_link8"):
    p,quat = r.fk(q, link); print(link, np.round(p,4), np.round(quat,4))
EOF

# openrua op 43
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.ik_link_name = \\"panda_hand\\"   # group tip is panda_link8 (45 deg off from the hand)\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 44
timeout 300 python3 go.py tcp 0.070 -0.040 1.15 4 2>&1 | grep -v "^\["

# openrua op 45
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png >/dev/null 2>&1; echo ok

# openrua op 46
timeout 300 python3 go.py tcp 0.070 -0.040 1.10 3 2>&1 | grep -v "^\["; timeout 300 python3 go.py tcp 0.070 -0.040 1.020 3 2>&1 | grep -v "^\["

# openrua op 47
timeout 120 python3 go.py grip 0.0 2>&1 | grep -v "^\["; timeout 60 python3 go.py state 2>&1 | grep -v "^\["

# openrua op 48
timeout 300 python3 go.py tcp 0.070 -0.040 1.07 3 2>&1 | grep -v "^\["; timeout 60 python3 go.py state 2>&1 | grep gap; timeout 60 python3 tools/perception/cam_snap.py frontview front3.png >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py sideview side3.png >/dev/null 2>&1; echo ok

# openrua op 49
timeout 120 python3 scan.py sideview 2>&1 | grep blob

# openrua op 50
python3 -c "
import numpy as np
W=np.load('sideview_world.npy')
m=(W[...,0]>0.02)&(W[...,0]<0.13)&(W[...,1]>-0.10)&(W[...,1]<0.10)&np.isfinite(W[...,2])&(W[...,2]>0.85)&(W[...,2]<1.12)
P=W[m]
for zlo in np.arange(0.88,1.12,0.01):
    s=P[(P[:,2]>=zlo)&(P[:,2]<zlo+0.01)]
    if len(s): print(f'z[{zlo:.2f},{zlo+0.01:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
"

# openrua op 51
timeout 300 python3 go.py tcp 0.070 -0.040 1.15 3 2>&1 | grep -v "^\["; timeout 300 python3 go.py tcp -0.048 -0.274 1.15 5 2>&1 | grep -v "^\["; timeout 60 python3 go.py state 2>&1 | grep gap

# openrua op 52
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py frontview front4.png >/dev/null 2>&1; echo ok

# openrua op 53
timeout 300 python3 go.py tcp -0.048 -0.274 1.058 4 2>&1 | grep -v "^\["; timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench | grep -A3 force

# openrua op 54
timeout 120 python3 go.py grip 0.08 2>&1 | grep -v "^\["; timeout 300 python3 go.py tcp -0.048 -0.274 1.20 3 2>&1 | grep -v "^\["; timeout 300 python3 go.py tcp 0.05 -0.15 1.30 4 2>&1 | grep -v "^\["

# openrua op 55
timeout 60 python3 tools/perception/cam_snap.py frontview front5.png >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py agentview agent5.png >/dev/null 2>&1; timeout 120 python3 scan.py birdview 2>&1 | grep blob

# openrua op 56
python3 -c "
import numpy as np
W=np.load('birdview_world.npy')
# stove region excluding robot arm: points near burner
m=(W[...,0]>-0.16)&(W[...,0]<0.05)&(W[...,1]>-0.30)&(W[...,1]<-0.11)&np.isfinite(W[...,2])
P=W[m]
for zlo in np.arange(0.92,1.10,0.01):
    s=P[(P[:,2]>=zlo)&(P[:,2]<zlo+0.01)]
    if len(s): print(f'z[{zlo:.2f},{zlo+0.01:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}')
# anything left at the old pot location?
o=W[(W[...,0]>0.02)&(W[...,0]<0.13)&(W[...,1]>-0.08)&(W[...,1]<0.10)&(W[...,2]>0.905)&(W[...,2]<1.2)]
print('points at old pot spot above table:',len(o))
"

# openrua op 57
timeout 120 python3 scan.py frontview >/dev/null 2>&1; python3 -c "
import numpy as np
W=np.load('frontview_world.npy')
m=(W[...,0]>-0.16)&(W[...,0]<0.05)&(W[...,1]>-0.32)&(W[...,1]<-0.10)&np.isfinite(W[...,2])&(W[...,2]>0.92)&(W[...,2]<1.12)
P=W[m]
for zlo in np.arange(0.92,1.10,0.01):
    s=P[(P[:,2]>=zlo)&(P[:,2]<zlo+0.01)]
    if len(s)>3: print(f'z[{zlo:.2f},{zlo+0.01:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
"
