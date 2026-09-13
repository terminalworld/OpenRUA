#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/ik_move.py; echo ----; cat /workspace/tools/action/gripper_cmd.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 30 ros2 action list; echo ----; timeout 30 ros2 service list | grep -v -E "parameter|describe_|get_type" ; echo ----; timeout 20 ros2 node list

# openrua op 4
timeout 60 ros2 topic echo /joint_states --once; for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 60 python3 -c "
import rclpy, yaml
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('tfdump')
buf=Buffer(); TransformListener(buf,n)
import time
for _ in range(40): rclpy.spin_once(n, timeout_sec=0.1)
print(buf.all_frames_as_yaml())
"

# openrua op 6
cat > tfq.py <<'EOF'
#!/usr/bin/env python3
"""Print transform parent->child as x y z qx qy qz qw."""
import sys, rclpy
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('tfq')
buf=Buffer(); TransformListener(buf,n)
pairs = [tuple(a.split(':')) for a in sys.argv[1:]] or [('world','panda_link0'),('world','panda_hand')]
for _ in range(50):
    rclpy.spin_once(n, timeout_sec=0.1)
    if all(buf.can_transform(p,c,rclpy.time.Time()) for p,c in pairs): break
for p,c in pairs:
    t=buf.lookup_transform(p,c,rclpy.time.Time())
    tr,q=t.transform.translation,t.transform.rotation
    print(f"{p}->{c}: {tr.x:.4f} {tr.y:.4f} {tr.z:.4f}  q {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}")
EOF
timeout 60 python3 tfq.py world:panda_link0 world:panda_hand world:birdview_optical_frame

# openrua op 7
cat > cloud.py <<'EOF'
#!/usr/bin/env python3
"""Grab depth+color+info from a camera, save world-frame XYZ array (H,W,3) as <cam>_xyz.npy and color png."""
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
cam = sys.argv[1]
rclpy.init(); n=rclpy.create_node('cloud'); buf=Buffer(); TransformListener(buf,n)
got={}
def sub(topic,T,key): n.create_subscription(T,topic,lambda m: got.setdefault(key,m),1)
sub(f'/{cam}/depth/image_raw',Image,'d'); sub(f'/{cam}/color/image_raw',Image,'c'); sub(f'/{cam}/color/camera_info',CameraInfo,'i')
frame=f'{cam}_optical_frame'
while len(got)<3 or not buf.can_transform('world',frame,rclpy.time.Time()): rclpy.spin_once(n,timeout_sec=0.2)
br=CvBridge()
depth=br.imgmsg_to_cv2(got['d'],'passthrough').astype(np.float64)
color=br.imgmsg_to_cv2(got['c'],'bgr8')
k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
X=(u-cx)*depth/fx; Y=(v-cy)*depth/fy; Z=depth
t=buf.lookup_transform('world',frame,rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
P=np.stack([X,Y,Z],-1)@R.T+tr
np.save(f'{cam}_xyz.npy',P); cv2.imwrite(f'{cam}.png',color)
print('saved',P.shape, 'depth range',np.nanmin(depth),np.nanmax(depth))
EOF
timeout 90 python3 cloud.py birdview && python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy'); z=P[...,2]
print('z percentiles', np.nanpercentile(z,[1,5,25,50,75,95,99]))
# table height ~ mode
h,e=np.histogram(z[np.isfinite(z)],bins=200); print('mode z', e[h.argmax()], e[h.argmax()+1])
"

# openrua op 8
python3 -c "
import numpy as np, cv2
P=np.load('birdview_xyz.npy'); z=P[...,2]
tbl = (z>0.85)&(z<0.92)
h,e=np.histogram(z[tbl],bins=70); print('table z', e[h.argmax()], e[h.argmax()+1])
ys,xs=np.where(tbl); print('table x range',P[tbl][:,0].min(),P[tbl][:,0].max(),'y range',P[tbl][:,1].min(),P[tbl][:,1].max())
# objects: above table
obj=(z>0.915)&(z<1.15)
m=(obj*255).astype('uint8')
n,lab,stats,cent=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    if stats[i,4]<30: continue
    sel=lab==i
    pts=P[sel]
    print(f'comp {i}: px area {stats[i,4]}, centroid px {cent[i]}, world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]')
"


# openrua op 9
python3 -c "
import cv2
im=cv2.imread('birdview.png')
crop=im[200:340,200:420]; crop=cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST); cv2.imwrite('bird_crop.png',crop)
im=cv2.imread('robot0_robotview.png'); crop=im[80:200,430:560]; cv2.imwrite('knob_crop.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 10
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy'); z=P[...,2]
# knob
sel=(z>0.94)&(P[...,0]<-0.15)&(P[...,1]>0.1)&(P[...,1]<0.3)
pts=P[sel]; print('knob top n',len(pts),'center',pts[:,:2].mean(0),'z',pts[:,2].min(),pts[:,2].max(), 'x range',pts[:,0].min(),pts[:,0].max(),'y range',pts[:,1].min(),pts[:,1].max())
sel=(z>0.915)&(P[...,0]<-0.15)&(P[...,1]>0.1)&(P[...,1]<0.3)
pts=P[sel]; print('knob all center',pts[:,:2].mean(0))
# stove
sel=(z>0.92)&(z<0.935)&(P[...,0]>-0.15)&(P[...,1]>0.1)
pts=P[sel]; print('stove center',pts[:,:2].mean(0),'z',np.median(pts[:,2]))
# pan body (exclude handle y>0.0? handle is at y> ~ -0.13)
sel=(z>0.95)&(P[...,1]<0.0)&(P[...,0]<0.1)&(P[...,0]>-0.3)
pts=P[sel]; print('pan rim n',len(pts),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z',pts[:,2].min(),pts[:,2].max())
sel=(z>0.95)&(P[...,1]<-0.13)
pts=P[sel]; print('pan body center',pts[:,:2].mean(0),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max())
# pan handle
sel=(z>0.95)&(P[...,1]>-0.13)&(P[...,1]<0.05)&(P[...,0]<0.1)
pts=P[sel]; print('handle n',len(pts),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z',pts[:,2].min(),pts[:,2].max())
"


# openrua op 11
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy'); z=P[...,2]
box=lambda x0,x1,y0,y1,z0,z1: (P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(z>z0)&(z<z1)
sel=box(-0.3,-0.14,0.1,0.3,0.945,1.0); pts=P[sel]; print('knob bar n',len(pts),'center',pts[:,:2].mean(0),'z',pts[:,2].min(),pts[:,2].max(),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max())
sel=box(-0.3,-0.14,0.1,0.3,0.912,0.945); pts=P[sel]; print('knob base n',len(pts),'center',pts[:,:2].mean(0),'z',pts[:,2].min(),pts[:,2].max(),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max())
sel=box(-0.2,0.1,-0.13,0.05,0.95,1.1); pts=P[sel]; print('handle n',len(pts),'center',pts[:,:2].mean(0),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z',pts[:,2].min(),pts[:,2].max())
sel=box(-0.2,0.1,-0.4,-0.13,0.95,1.1); pts=P[sel]; print('pan body n',len(pts),'center',pts[:,:2].mean(0),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z',pts[:,2].min(),pts[:,2].max())
sel=box(-0.2,0.1,-0.4,-0.13,1.0,1.1); pts=P[sel]; print('pan rim n',len(pts),'center',pts[:,:2].mean(0),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z',pts[:,2].min(),pts[:,2].max())
"


# openrua op 12
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy'); z=P[...,2]
sel=(P[...,0]>-0.3)&(P[...,0]<-0.14)&(P[...,1]>0.1)&(P[...,1]<0.3)&(z>0.905)
h,e=np.histogram(z[sel],bins=np.arange(0.905,0.965,0.0025))
for c,a in zip(h,e): print(f'{a:.4f} {c}')
# fin footprint at z>0.95
s2=sel&(z>0.95); pts=P[s2]; print('fin>0.95 center',pts[:,:2].mean(0),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max())
s2=sel&(z>0.93)&(z<0.95); pts=P[s2]; print('0.93-0.95 center',pts[:,:2].mean(0),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max())
"


# openrua op 13
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 300 python3 tools/action/ik_move.py -0.20 0.196 1.00 1 0 0 0 3 --at tcp; timeout 60 python3 tfq.py world:panda_hand

# openrua op 14
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small arm controller for this machine (Franka Panda, MoveIt IK).

Usage:
  python3 arm.py tcp X Y Z QX QY QZ QW [SECONDS]   # move fingertip point (world frame)
  python3 arm.py joints J1,...,J7 [SECONDS]        # joint-space move
  python3 arm.py j7 DELTA [SECONDS]                # rotate joint7 by DELTA rad, others held
  python3 arm.py grip WIDTH                        # per-finger position (0.04 open, 0.0 closed)
  python3 arm.py state                             # joints + hand/tcp pose (world)
World -> base conversion uses TF world->panda_link0; IK requests leave
frame_id empty (planner model frame == arm base).
"""
import sys
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
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


class Arm:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("arm_ctl")
        self.js = {}
        self.n.create_subscription(JointState, "/joint_states",
                                   lambda m: self.js.__setitem__("m", m), 1)
        self.buf = Buffer()
        TransformListener(self.buf, self.n)
        self.fjt = ActionClient(self.n, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])

    def spin(self, t=0.1):
        rclpy.spin_once(self.n, timeout_sec=t)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            self.spin()
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def tf(self, parent, child):
        while not self.buf.can_transform(parent, child, rclpy.time.Time()):
            self.spin()
        t = self.buf.lookup_transform(parent, child, rclpy.time.Time()).transform
        p = np.array([t.translation.x, t.translation.y, t.translation.z])
        q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        return p, q

    def state(self):
        j = self.joints()
        print("joints:", ",".join(f"{j[n]:.4f}" for n in JOINTS))
        print("fingers:", j.get("panda_finger_joint1"), j.get("panda_finger_joint2"))
        p, q = self.tf("world", "panda_hand")
        tcp = p + TCP * quat_R(*q)[:, 2]
        print(f"hand(world): {p.round(4)} q {np.round(q, 4)}")
        print(f"tcp(world):  {tcp.round(4)}")

    def solve_ik(self, hand_world, q):
        base, _ = self.tf("world", "panda_link0")  # base has identity rotation
        p = np.asarray(hand_world) - base
        if not self.ik.wait_for_service(timeout_sec=10):
            raise SystemExit("no IK service")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q)
        req.ik_request.avoid_collisions = False
        cur = self.joints()
        seed = JointState()
        for n in JOINTS:
            seed.name.append(n)
            seed.position.append(cur[n])
        req.ik_request.robot_state.joint_state = seed
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"IK FAILED code={None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in JOINTS]

    def move_joints(self, positions, seconds):
        lim = FJT["limits_rad"]
        for p, (lo, hi) in zip(positions, lim):
            if not lo <= p <= hi:
                raise SystemExit(f"target {p} outside limit [{lo},{hi}]")
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        f = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, f)
        r = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, r)
        code = r.result().result.error_code
        cur = self.joints()
        err = max(abs(cur[n] - p) for n, p in zip(JOINTS, positions))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code

    def move_tcp(self, xyz, q, seconds):
        R = quat_R(*q)
        hand = np.asarray(xyz, float) - TCP * R[:, 2]
        sol = self.solve_ik(hand, q)
        print("ik:", ",".join(f"{x:.4f}" for x in sol))
        self.move_joints(sol, seconds)
        self.state()

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise SystemExit("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=30)
        r = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, r, timeout_sec=120)
        res = r.result().result
        j = self.joints()
        print(f"gripper reached={res.reached_goal} stalled={res.stalled} "
              f"fingers={j['panda_finger_joint1']:.4f},{j['panda_finger_joint2']:.4f}")


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    arm = Arm()
    cmd = a[0]
    if cmd == "state":
        arm.state()
    elif cmd == "tcp":
        xyz = list(map(float, a[1:4])); q = list(map(float, a[4:8]))
        sec = float(a[8]) if len(a) > 8 else 3.0
        arm.move_tcp(xyz, q, sec)
    elif cmd == "joints":
        pos = list(map(float, a[1].split(",")))
        sec = float(a[2]) if len(a) > 2 else 3.0
        arm.move_joints(pos, sec); arm.state()
    elif cmd == "j7":
        d = float(a[1]); sec = float(a[2]) if len(a) > 2 else 2.0
        cur = arm.joints(); pos = [cur[n] for n in JOINTS]; pos[6] += d
        arm.move_joints(pos, sec); arm.state()
    elif cmd == "grip":
        arm.gripper(float(a[1]))
    else:
        raise SystemExit(__doc__)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 15
timeout 120 python3 arm.py state && timeout 300 python3 arm.py tcp -0.20 0.196 1.00 1 0 0 0 3

# openrua op 16
timeout 300 python3 arm.py tcp -0.20 0.196 1.00 0.9996 0 -0.0284 0 3 2>&1 | tail -5; echo ---; timeout 300 python3 arm.py tcp -0.20 0.196 1.05 1 0 0 0 3 2>&1 | tail -5

# openrua op 17
cat > fk.py <<'EOF'
import rclpy, sys
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node('fk')
cli=n.create_client(GetPositionFK,'/compute_fk'); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand','panda_link8']
req.header.frame_id=''
js=JointState(); js.name=[f'panda_joint{i}' for i in range(1,8)]
js.position=[float(x) for x in sys.argv[1].split(',')] if len(sys.argv)>1 else [0.0,-0.1610,0.0,-2.4446,0.0,2.2268,0.7854]
req.robot_state.joint_state=js
f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=30)
r=f.result(); print(r.error_code)
for nm,ps in zip(r.fk_link_names,r.pose_stamped):
    p=ps.pose.position;q=ps.pose.orientation; print(nm,ps.header.frame_id,f'{p.x:.4f} {p.y:.4f} {p.z:.4f} q {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}')
EOF
timeout 60 python3 fk.py

# openrua op 18
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        base, _ = self.tf(\\"world\\", \\"panda_link0\\")  # base has identity rotation\\n        p = np.asarray(hand_world) - base", "new_string": "        # verified via /compute_fk: the planner\'s model frame is `world`\\n        # on this machine (FK of panda_hand == TF world->panda_hand)\\n        p = np.asarray(hand_world)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 19
timeout 300 python3 arm.py tcp -0.20 0.196 1.00 1 0 0 0 3 2>&1 | tail -6

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "        req.ik_request.pose_stamped.header.frame_id = \\"\\"\\n        req.ik_request.ik_link_name = \\"panda_hand\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
timeout 300 python3 arm.py tcp -0.20 0.196 1.00 1 0 0 0 2 2>&1 | tail -6

# openrua op 22
timeout 300 python3 arm.py joints 0.3911,0.3028,0.0108,-2.3961,-0.0075,2.6988,1.1936 2 2>&1 | tail -5

# openrua op 23
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 24
timeout 300 python3 arm.py tcp -0.20 0.196 0.93 1 0 0 0 2 2>&1 | tail -5

# openrua op 25
timeout 120 python3 arm.py grip 0.0 2>&1 | tail -2; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 26
timeout 300 python3 arm.py j7 1.5708 2 2>&1 | tail -5; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | head -12

# openrua op 27
timeout 300 python3 arm.py j7 -1.8 3 2>&1 | tail -5; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 28
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; python3 -c "
import cv2; im=cv2.imread('agentview.png'); cv2.imwrite('ag_crop.png', cv2.resize(im[150:300,330:530],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 29
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        goal.trajectory.points = [pt]\\n        f = self.fjt.send_goal_async(goal)", "new_string": "        goal.trajectory.points = [pt]\\n        # loads (e.g. turning a knob) make the controller lag; do not let\\n        # the path tolerance abort the goal, only judge the final state\\n        for n in JOINTS:\\n            goal.path_tolerance.append(JointTolerance(name=n, position=3.0))\\n            goal.goal_tolerance.append(JointTolerance(name=n, position=0.05))\\n        goal.goal_time_tolerance = Duration(sec=10)\\n        f = self.fjt.send_goal_async(goal)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 30
sed -i 's/^from control_msgs.action import FollowJointTrajectory, GripperCommand/from control_msgs.action import FollowJointTrajectory, GripperCommand\nfrom control_msgs.msg import JointTolerance/' arm.py && timeout 600 python3 arm.py j7 -1.3 6 2>&1 | tail -5

# openrua op 31
timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; python3 -c "
import cv2; im=cv2.imread('agentview.png'); cv2.imwrite('ag_crop.png', cv2.resize(im[180:360,340:540],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 32
timeout 600 python3 arm.py joints 0.3922,0.4943,0.0054,-2.3553,-0.0218,2.8546,-0.40 4 2>&1 | tail -4; timeout 120 python3 arm.py grip 0.04 | tail -1; timeout 300 python3 arm.py tcp -0.20 0.196 1.05 0.7071 0.7071 0 0 3 2>&1 | tail -3

# openrua op 33
timeout 90 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy'); z=P[...,2]
box=lambda x0,x1,y0,y1,z0,z1: (P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(z>z0)&(z<z1)
def rep(name,sel):
    pts=P[sel]; print(f'{name}: n={len(pts)} center={pts[:,:2].mean(0).round(4)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]')
rep('pan body', box(-0.3,0.15,-0.45,-0.12,0.905,1.1))
rep('pan rim', box(-0.3,0.15,-0.45,-0.12,0.99,1.1))
rep('pan handle', box(-0.12,0.0,-0.13,0.02,0.905,1.1))
h,e=np.histogram(P[box(-0.12,0.0,-0.13,0.02,0.905,1.1)][:,2],bins=np.arange(0.9,1.06,0.005)); print([f'{a:.3f}:{c}' for a,c in zip(e,h) if c])
rep('moka', box(0.0,0.15,-0.12,0.06,0.905,1.2))
rep('stove', box(-0.15,0.05,0.09,0.31,0.915,0.94))
rep('knob', box(-0.3,-0.14,0.1,0.3,0.905,1.0))
rep('knob fin', box(-0.3,-0.14,0.1,0.3,0.945,1.0))
"

# openrua op 34
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy'); z=P[...,2]
box=lambda x0,x1,y0,y1,z0,z1: (P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(z>z0)&(z<z1)
def rep(name,sel):
    pts=P[sel]
    if len(pts)==0: print(name,'EMPTY'); return
    print(f'{name}: n={len(pts)} center={pts[:,:2].mean(0).round(4)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]')
rep('pan rim', box(-0.3,0.15,-0.45,-0.12,0.93,1.1))
sel=box(-0.3,0.15,-0.45,-0.12,0.905,1.1); h,e=np.histogram(P[sel][:,2],bins=np.arange(0.9,0.96,0.005)); print('pan z hist',[f'{a:.3f}:{c}' for a,c in zip(e,h) if c])
rep('pan handle', box(-0.12,0.0,-0.13,0.02,0.905,1.1))
sel=box(-0.12,0.0,-0.13,0.02,0.905,1.1); h,e=np.histogram(P[sel][:,2],bins=np.arange(0.9,0.96,0.005)); print('handle z hist',[f'{a:.3f}:{c}' for a,c in zip(e,h) if c])
for y0 in np.arange(-0.13,0.02,0.02):
    s=box(-0.15,0.05,y0,y0+0.02,0.915,1.0); pts=P[s]
    if len(pts): print(f' handle slice y[{y0:.2f},{y0+0.02:.2f}] n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] xmean={pts[:,0].mean():.3f} z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]')
rep('moka', box(0.0,0.15,-0.12,0.06,0.905,1.2))
rep('stove', box(-0.15,0.05,0.09,0.31,0.915,0.94))
rep('knob', box(-0.3,-0.14,0.1,0.3,0.905,1.0))
rep('knob fin', box(-0.3,-0.14,0.1,0.3,0.945,1.0))
"

# openrua op 35
timeout 300 python3 arm.py tcp -0.056 -0.09 1.02 0.7071 0.7071 0 0 3 2>&1 | tail -4; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 36
timeout 300 python3 arm.py tcp -0.056 -0.075 0.925 0.7071 0.7071 0 0 2.5 2>&1 | tail -3; timeout 120 python3 arm.py grip 0.0 | tail -1; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 37
timeout 300 python3 arm.py tcp -0.056 -0.075 1.15 0.7071 0.7071 0 0 3 2>&1 | tail -3; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 38
timeout 400 python3 arm.py tcp -0.038 0.028 1.15 0.7071 -0.7071 0 0 5 2>&1 | tail -4

# openrua op 39
timeout 400 python3 arm.py tcp -0.038 0.028 1.15 0.7071 -0.7071 0 0 5 2>&1

# openrua op 40
timeout 400 python3 arm.py joints 0.2413,0.3199,-0.2123,-1.8876,0.0820,2.1992,2.3474 6 2>&1

# openrua op 41
timeout 400 python3 arm.py joints 0.2413,0.3199,-0.2123,-1.8876,0.0820,2.1992,2.3474 4 2>&1

# openrua op 42
timeout 90 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy'); z=P[...,2]
sel=(z>1.08)&(z<1.20)&(P[...,0]>-0.3)&(P[...,0]<0.2)&(P[...,1]>-0.1)&(P[...,1]<0.45)
pts=P[sel]; print('pan n',len(pts),'center',pts[:,:2].mean(0).round(4),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
# body only: y>0.1
s2=sel&(P[...,1]>0.1); pts=P[s2]; print('pan body center',pts[:,:2].mean(0).round(4),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3))
"

# openrua op 43
timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null; python3 -c "
import cv2; im=cv2.imread('birdview.png'); cv2.imwrite('bird_crop.png', cv2.resize(im[150:350,200:480],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 44
timeout 90 python3 cloud.py sideview >/dev/null; timeout 90 python3 cloud.py agentview >/dev/null; python3 -c "
import numpy as np
for cam in ['sideview','agentview']:
    P=np.load(f'{cam}_xyz.npy'); z=P[...,2]
    sel=(z>1.0)&(z<1.2)&(P[...,0]>-0.25)&(P[...,0]<0.2)&(P[...,1]>-0.1)&(P[...,1]<0.45)
    pts=P[sel]
    print(cam,'n',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
    for zlo in np.arange(1.0,1.2,0.02):
        s=sel&(z>=zlo)&(z<zlo+0.02); p=P[s]
        if len(p)>20: print(f'  z[{zlo:.2f},{zlo+0.02:.2f}] n={len(p)} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}]')
"

# openrua op 45
timeout 400 python3 arm.py tcp -0.038 0.028 1.00 0.7071 -0.7071 0 0 3 2>&1 | tail -3; timeout 400 python3 arm.py tcp -0.038 0.028 0.955 0.7071 -0.7071 0 0 3 2>&1 | tail -3; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 46
timeout 120 python3 arm.py grip 0.04 | tail -1; timeout 400 python3 arm.py tcp -0.038 0.028 1.15 0.7071 -0.7071 0 0 3 2>&1 | tail -2; timeout 90 python3 cloud.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null; python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy'); z=P[...,2]
sel=(z>0.94)&(z<1.0)&(P[...,0]>-0.25)&(P[...,0]<0.2)&(P[...,1]>0.0)&(P[...,1]<0.45)
pts=P[sel]; print('pan n',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
s2=sel&(z>0.96); pts=P[s2]; print('rim center', pts[:,:2].mean(0).round(3), 'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
sel=(z>0.93)&(z<1.0)&(P[...,0]>-0.25)&(P[...,0]<0.2)&(P[...,1]>0.0)&(P[...,1]<0.45)
h,e=np.histogram(P[sel][:,2],bins=np.arange(0.93,1.0,0.005)); print([f'{a:.3f}:{c}' for a,c in zip(e,h) if c])
"

# openrua op 47
timeout 120 python3 arm.py grip 0.0 | tail -1; timeout 400 python3 arm.py tcp 0.125 0.20 1.05 1 0 0 0 4 2>&1 | tail -1; timeout 400 python3 arm.py tcp 0.125 0.20 0.95 1 0 0 0 2 2>&1 | tail -1

# openrua op 48
timeout 400 python3 arm.py tcp 0.06 0.20 1.02 1 0 0 0 4 2>&1 | tail -2

# openrua op 49
timeout 400 python3 arm.py tcp 0.06 0.20 1.02 1 0 0 0 3 2>&1 | grep -E "ik|traj|joints"

# openrua op 50
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 51
timeout 400 python3 arm.py tcp 0.06 0.20 0.955 1 0 0 0 2 2>&1 | grep -E "traj|tcp"; timeout 400 python3 arm.py tcp 0.025 0.20 0.955 1 0 0 0 3 2>&1 | grep -E "traj|tcp"; timeout 400 python3 arm.py tcp 0.025 0.20 1.10 1 0 0 0 3 2>&1 | grep -E "traj|tcp"

# openrua op 52
timeout 90 python3 cloud.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null; python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy'); z=P[...,2]
sel=(z>0.955)&(z<1.0)&(P[...,0]>-0.25)&(P[...,0]<0.2)&(P[...,1]>0.0)&(P[...,1]<0.45)
pts=P[sel]; print('pan rim n',len(pts),'center',pts[:,:2].mean(0).round(3),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
print('pan center est', ((pts[:,0].min()+pts[:,0].max())/2).round(3), ((pts[:,1].min()+pts[:,1].max())/2).round(3))
sel=(z>0.92)&(z<0.94)&(P[...,0]>-0.15)&(P[...,0]<0.05)&(P[...,1]>0.09)&(P[...,1]<0.31)
pts=P[sel]; print('stove visible plate x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3))
"

# openrua op 53
timeout 90 python3 cloud.py sideview >/dev/null; timeout 90 python3 cloud.py agentview >/dev/null; python3 -c "
import numpy as np
for cam in ['sideview','agentview','birdview']:
    P=np.load(f'{cam}_xyz.npy'); z=P[...,2]
    sel=(z>0.945)&(z<1.0)&(P[...,0]>-0.25)&(P[...,0]<0.2)&(P[...,1]>0.09)&(P[...,1]<0.35)
    pts=P[sel]
    print(cam,'rim n',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
    h,e=np.histogram(pts[:,2],bins=np.arange(0.945,1.0,0.005)); print('   ',[f'{a:.3f}:{c}' for a,c in zip(e,h) if c])
"

# openrua op 54
timeout 400 python3 arm.py tcp -0.07 0.20 1.02 1 0 0 0 3 2>&1 | grep -E "traj|tcp"; timeout 400 python3 arm.py tcp -0.07 0.20 0.955 1 0 0 0 2 2>&1 | grep -E "traj|tcp"; timeout 400 python3 arm.py tcp -0.115 0.20 0.955 1 0 0 0 3 2>&1 | grep -E "traj|tcp"; timeout 400 python3 arm.py tcp -0.115 0.20 1.10 1 0 0 0 3 2>&1 | grep -E "traj|tcp"

# openrua op 55
timeout 90 python3 cloud.py sideview >/dev/null; timeout 90 python3 cloud.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview >/dev/null; python3 -c "
import numpy as np
for cam in ['sideview','agentview']:
    P=np.load(f'{cam}_xyz.npy'); z=P[...,2]
    sel=(z>0.945)&(z<1.0)&(P[...,0]>-0.17)&(P[...,0]<0.2)&(P[...,1]>0.09)&(P[...,1]<0.35)
    pts=P[sel]
    print(cam,'rim n',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
    h,e=np.histogram(pts[:,2],bins=np.arange(0.945,1.0,0.005)); print('   ',[f'{a:.3f}:{c}' for a,c in zip(e,h) if c])
"

# openrua op 56
timeout 400 python3 arm.py joints 0,-0.161,0,-2.4446,0,2.2268,0.7854 4 2>&1 | grep -E "traj"; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null; python3 -c "
import cv2; im=cv2.imread('sideview.png'); cv2.imwrite('side_crop.png', cv2.resize(im[280:400,200:420],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 57
timeout 90 python3 cloud.py birdview >/dev/null; python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy'); z=P[...,2]
sel=(P[...,0]>-0.3)&(P[...,0]<-0.14)&(P[...,1]>0.1)&(P[...,1]<0.3)&(z>0.945)&(z<1.0)
pts=P[sel]; print('knob fin: center',pts[:,:2].mean(0).round(3),'x extent',(pts[:,0].max()-pts[:,0].min()).round(3),'y extent',(pts[:,1].max()-pts[:,1].min()).round(3))
sel=(z>0.955)&(z<1.0)&(P[...,0]>-0.2)&(P[...,0]<0.2)&(P[...,1]>0.09)&(P[...,1]<0.35)
pts=P[sel]; print('pan rim: x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
"
