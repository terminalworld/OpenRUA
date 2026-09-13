#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/action/ik_move.py tools/action/gripper_cmd.py tools/action/fjt_send.py tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iE "ik|fk|plan|scene" ; echo ---; timeout 20 ros2 node list

# openrua op 3
timeout 20 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 30 python3 tools/perception/cam_snap.py $c; done

# openrua op 4
python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[130:290,100:380]; cv2.imwrite('crop_caddy_agent.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('birdview.png'); crop=im[140:220,200:380]; cv2.imwrite('crop_caddy_bird.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); crop=im[220:340,280:520]; cv2.imwrite('crop_caddy_side.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 5
timeout 60 python3 tools/perception/px2world.py birdview 318 272; timeout 60 python3 tools/perception/px2world.py birdview 280 165; timeout 60 python3 tools/perception/px2world.py birdview 280 195; timeout 60 python3 tools/perception/px2world.py birdview 240 180; timeout 60 python3 tools/perception/px2world.py birdview 400 400

# openrua op 6
timeout 15 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 15 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -12

# openrua op 7
cat > /workspace/depth_world.py <<'EOF'
"""Project a camera's full depth frame into world coords; save npz."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def grab(node, topic, T, timeout=15.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds/1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds/1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("dw")
buf = Buffer(); TransformListener(buf, node)
depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
end = node.get_clock().now().nanoseconds/1e9 + 10
while node.get_clock().now().nanoseconds/1e9 < end:
    rclpy.spin_once(node, timeout_sec=0.2)
    if buf.can_transform("world", frame, rclpy.time.Time()): break
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; R = quat_R(q.x,q.y,q.z,q.w)
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
h,w = depth.shape
u,v = np.meshgrid(np.arange(w), np.arange(h))
pc = np.stack([(u-cx)*depth/fx, (v-cy)*depth/fy, depth], -1)
pw = pc @ R.T + tr
np.savez(f"{cam}_world.npz", xyz=pw, depth=depth)
print("saved", cam, "cam pos", tr, "shape", pw.shape)
rclpy.shutdown()
EOF
timeout 60 python3 depth_world.py birdview && timeout 60 python3 depth_world.py agentview

# openrua op 8
python3 -c "
import numpy as np
d=np.load('birdview_world.npz'); xyz=d['xyz']
# height map over table region
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
# cup: things with z between 0.9 and 1.0 near (-0.09,0)
m=(z>0.89)&(np.abs(x+0.09)<0.1)&(np.abs(y)<0.1)
print('cup pts',m.sum(),'x',x[m].min(),x[m].max(),'y',y[m].min(),y[m].max(),'z',z[m].min(),z[m].max())
# cup top rim ring: highest points
mt=m&(z>z[m].max()-0.01)
print('cup rim x',x[mt].min(),x[mt].max(),'y',y[mt].min(),y[mt].max(), 'center',x[mt].mean(),y[mt].mean())
# caddy: region x in [-0.55,-0.3], y in [-0.35, 0.0]
mc=(z>0.885)&(x>-0.56)&(x<-0.28)&(y>-0.36)&(y<-0.02)
print('caddy z range',z[mc].min(),z[mc].max())
# print coarse height map of caddy: bins of 1cm
xs=np.arange(-0.56,-0.28,0.01); ys=np.arange(-0.36,-0.02,0.01)
H=np.full((len(xs),len(ys)),np.nan)
for i,xa in enumerate(xs):
  for j,ya in enumerate(ys):
    mm=(x>=xa)&(x<xa+0.01)&(y>=ya)&(y<ya+0.01)&(z>0.5)
    if mm.sum(): H[i,j]=z[mm].max()
np.set_printoptions(linewidth=250,precision=2)
print('rows=x from -0.56 up; cols=y from -0.36 up; values=(z-0.88)*100 cm')
for i,xa in enumerate(xs):
  print(f'{xa:6.2f}',' '.join(f'{(v-0.88)*100:3.0f}' if np.isfinite(v) else '  .' for v in H[i]))
"


# openrua op 9
python3 -c "
import numpy as np
d=np.load('birdview_world.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.89)&(np.abs(x+0.09)<0.12)&(np.abs(y)<0.12)
h,_=np.histogram(z[m],bins=np.arange(0.89,1.06,0.01))
for b,c in zip(np.arange(0.89,1.06,0.01),h): print(f'{b:.2f} {c}')
for lo in [0.94,0.96,0.98,1.0,1.02]:
  mm=m&(z>=lo)&(z<lo+0.02)
  if mm.sum(): print(lo, 'x',x[mm].min().round(3),x[mm].max().round(3),'y',y[mm].min().round(3),y[mm].max().round(3),'n',mm.sum())
# 5mm grid heightmap around cup
xs=np.arange(-0.20,0.0,0.01); ys=np.arange(-0.10,0.11,0.01)
print('cup height map cm; rows x, cols y from -0.10')
for xa in xs:
  row=[]
  for ya in ys:
    mm=(x>=xa)&(x<xa+0.01)&(y>=ya)&(y<ya+0.01)&(z>0.5)
    row.append(f'{(z[mm].max()-0.88)*100:3.0f}' if mm.sum() else '  .')
  print(f'{xa:6.2f}',' '.join(row))
"


# openrua op 10
python3 -c "
import numpy as np
d=np.load('agentview_world.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.89)&(x>-0.25)&(x<0.05)&(np.abs(y)<0.15)
print('cup z max',z[m].max())
xs=np.arange(-0.20,0.0,0.01); ys=np.arange(-0.10,0.11,0.01)
print('cup height map cm (agentview); rows x, cols y from -0.10')
for xa in xs:
  row=[]
  for ya in ys:
    mm=(x>=xa)&(x<xa+0.01)&(y>=ya)&(y<ya+0.01)&(z>0.5)
    row.append(f'{(z[mm].max()-0.88)*100:3.0f}' if mm.sum() else '  .')
  print(f'{xa:6.2f}',' '.join(row))
print('caddy from agentview: rows x from -0.50, cols y from -0.36')
xs=np.arange(-0.50,-0.32,0.01); ys=np.arange(-0.36,0.06,0.01)
for xa in xs:
  row=[]
  for ya in ys:
    mm=(x>=xa)&(x<xa+0.01)&(y>=ya)&(y<ya+0.01)&(z>0.5)
    row.append(f'{(z[mm].max()-0.88)*100:3.0f}' if mm.sum() else '  .')
  print(f'{xa:6.2f}',' '.join(row))
"


# openrua op 11
timeout 20 ros2 topic echo /birdview/color/camera_info --once | head -20; python3 -c "
import numpy as np
d=np.load('birdview_world.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
# hand region
m=(z>1.15)&(z<1.45)&(x>-0.4)&(x<-0.15)&(np.abs(y)<0.12)
print('hand pts',m.sum(),'x',x[m].min(),x[m].max(),'y',y[m].min(),y[m].max(),'z',z[m].min(),z[m].max())
for zz in [1.16,1.2,1.25,1.3,1.35,1.4]:
  mm=m&(z>=zz)&(z<zz+0.05)
  if mm.sum(): print(zz,'x',x[mm].min().round(3),x[mm].max().round(3),'y',y[mm].min().round(3),y[mm].max().round(3),'n',mm.sum())
"

# openrua op 12
python3 -c "
import numpy as np
d=np.load('birdview_world.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
xs=np.arange(-0.50,-0.33,0.005); ys=np.arange(-0.24,-0.06,0.005)
print('5mm bins; rows x, cols y from -0.24 to -0.06; cm above table')
for xa in xs:
  row=[]
  for ya in ys:
    mm=(x>=xa)&(x<xa+0.005)&(y>=ya)&(y<ya+0.005)&(z>0.5)
    row.append(f'{(z[mm].max()-0.88)*100:3.0f}' if mm.sum() else '  .')
  print(f'{xa:7.3f}',' '.join(row))
"

# openrua op 13
timeout 120 find / -xdev \( -iname "*caddy*" -o -iname "*libero*" -o -iname "*robosuite*" \) 2>/dev/null | head -20; ps aux | head -30

# openrua op 14
for c in sideview frontview galleryview paperview agentview; do echo "== $c"; timeout 15 ros2 run tf2_ros tf2_echo world ${c}_optical_frame 2>&1 | grep -A1 -E "Translation|RPY \(degree" | head -4; done

# openrua op 15
python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[280:390,240:370]; cv2.imwrite('crop_cup_agent.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('frontview.png'); crop=im[290:370,260:360]; cv2.imwrite('crop_cup_front.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 16
timeout 60 python3 depth_world.py frontview; timeout 60 python3 depth_world.py sideview; python3 -c "
import numpy as np
from scipy.optimize import least_squares
d=np.load('birdview_world.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>1.03)&(x>-0.2)&(x<0.0)&(y>-0.065)&(y<0.1)
px,py=x[m],y[m]
def res(p): return np.hypot(px-p[0],py-p[1])-p[2]
r=least_squares(res,[-0.09,0,0.05]); print('rim circle fit (outer-ish):',r.x, 'n',m.sum())
cx,cy=r.x[0],r.x[1]
for cam in ['agentview','frontview','sideview','birdview']:
  d=np.load(f'{cam}_world.npz'); xyz=d['xyz']; z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
  print(cam)
  for lo in np.arange(0.885,1.05,0.01):
    m=(z>=lo)&(z<lo+0.01)&(np.hypot(x-cx,y-cy)<0.08)&~((y<cy-0.045)&(x>cx-0.03)&(x<cx+0.02))  # exclude handle side
    if m.sum()>3:
      r_=np.hypot(x[m]-cx,y[m]-cy)
      print(f'  z {lo:.3f}-{lo+0.01:.3f}: n={m.sum():4d} r max {r_.max():.3f} r 95pct {np.percentile(r_,95):.3f}')
"


# openrua op 17
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small controller: IK -> trajectory, gripper, joint-state, all in one node.

Usage:
  python3 ctl.py js                              # print arm + finger joints
  python3 ctl.py grip <width_m>                  # per-finger width
  python3 ctl.py goto <x> <y> <z> <yaw_deg> [secs]   # TCP (fingertip point) target in WORLD frame,
                                                 # hand pointing straight down, fingers along
                                                 # world Y rotated by yaw about Z
  python3 ctl.py joints p1,...,p7 [secs]
Poses are converted world -> panda_link0 using the fixed base offset.
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
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])
TCP = float(M["hand"]["tcp_offset_m"])


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])

    def js(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        js = self.js()
        return [js[j] for j in JOINTS]

    def solve_ik(self, pos_world, quat_xyzw, seed=None):
        if not self.ik.wait_for_service(timeout_sec=10):
            raise SystemExit("IK service unavailable")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_xyzw)
        seed = seed if seed is not None else self.arm_q()
        s = JointState(); s.name = list(JOINTS); s.position = [float(v) for v in seed]
        req.ik_request.robot_state.joint_state = s
        req.ik_request.timeout.sec = 5
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise SystemExit("IK no answer")
        if res.error_code.val != 1:
            raise SystemExit(f"IK FAILED error_code={res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move(self, q, seconds):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"move done error_code={code} max_joint_err={err:.4f}")
        return code

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise SystemExit("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        js = self.js()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={js['panda_finger_joint1']:.4f},{js['panda_finger_joint2']:.4f}")

    def goto_tcp(self, x, y, z, yaw_deg, seconds=4.0):
        # hand pointing down: 180deg about X, then yaw about world Z
        R = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_euler("x", 180, degrees=True)
        q = R.as_quat()  # xyzw
        # TCP is +TCP along hand Z (pointing down) -> hand origin is TCP - TCP*hand_z
        hand_z = R.as_matrix()[:, 2]
        hand_pos = np.array([x, y, z]) - TCP * hand_z
        qj = self.solve_ik(hand_pos, q)
        return self.move(qj, seconds)


if __name__ == "__main__":
    c = Ctl()
    cmd = sys.argv[1]
    if cmd == "js":
        js = c.js()
        for k, v in js.items():
            print(f"{k}: {v:.4f}")
    elif cmd == "grip":
        c.gripper(float(sys.argv[2]))
    elif cmd == "goto":
        x, y, z, yaw = map(float, sys.argv[2:6])
        secs = float(sys.argv[6]) if len(sys.argv) > 6 else 4.0
        c.goto_tcp(x, y, z, yaw, secs)
    elif cmd == "joints":
        q = [float(v) for v in sys.argv[2].split(",")]
        secs = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
        c.move(q, secs)
    rclpy.shutdown()
OPENRUA_EOF

# openrua op 18
timeout 300 python3 ctl.py grip 0.04 && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 depth_world.py birdview && python3 -c "
import numpy as np
d=np.load('birdview_world.npz'); xyz=d['xyz']; z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.89)&(x>-0.2)&(x<0.0)&(np.abs(y)<0.12)
print('cup z range',z[m].min(),z[m].max())
xs=np.arange(-0.16,-0.01,0.01); ys=np.arange(-0.11,0.07,0.01)
for xa in xs:
  print(f'{xa:6.2f}',' '.join(f'{(z[(x>=xa)&(x<xa+0.01)&(y>=ya)&(y<ya+0.01)&(z>0.5)].max()-0.88)*100:3.0f}' if ((x>=xa)&(x<xa+0.01)&(y>=ya)&(y<ya+0.01)&(z>0.5)).sum() else '  .' for ya in ys))
"

# openrua op 19
timeout 600 python3 ctl.py goto -0.09 -0.085 1.06 90 4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 20
cat > /workspace/iktest.py <<'EOF'
import sys, numpy as np, rclpy
sys.path.insert(0,'/workspace')
from ctl import Ctl
from scipy.spatial.transform import Rotation as Rot
c=Ctl()
q0=c.arm_q()
for yaw in [90,-90,0,180,45,-45]:
    for z in [1.06,1.15]:
        R = Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
        hand = np.array([-0.09,-0.085,z]) - 0.1034*R.as_matrix()[:,2]
        try:
            sol=c.solve_ik(hand, R.as_quat(), seed=q0)
            print(yaw,z,'OK',np.round(sol,3))
        except SystemExit as e:
            print(yaw,z,e)
rclpy.shutdown()
EOF
timeout 600 python3 iktest.py

# openrua op 21
cat > /workspace/iktest2.py <<'EOF'
import sys, numpy as np, rclpy
sys.path.insert(0,'/workspace')
from ctl import Ctl, BASE_IN_WORLD
c=Ctl()
q0=c.arm_q(); print('q0',np.round(q0,3))
# current hand pose from tf2_echo: base (0.457,0,0.358) quat (1,0,-0.028,0)
for pos,quat in [((0.457,0,0.358),(1,0,-0.028,0)), ((0.457,0,0.358),(1,0,0,0)), ((0.5,0,0.3),(1,0,0,0)), ((0.66,-0.085,0.25),(1,0,0,0)), ((0.66,-0.085,0.15),(1,0,0,0)), ((0.6,0,0.15),(1,0,0,0))]:
    try:
        sol=c.solve_ik(np.array(pos)+BASE_IN_WORLD, quat, seed=q0); print(pos,quat,'OK',np.round(sol,3))
    except SystemExit as e: print(pos,quat,e)
rclpy.shutdown()
EOF
timeout 600 python3 iktest2.py

# openrua op 22
timeout 300 python3 tools/action/ik_move.py 0.457 0 0.358 1 0 -0.028 0 2; echo "exit=$?"

# openrua op 23
timeout 120 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand','panda_link8'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | tail -40

# openrua op 24
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "        pb = np.asarray(pos_world) - BASE_IN_WORLD", "new_string": "        # verified via /compute_fk: the planner\'s model frame here IS world\\n        pb = np.asarray(pos_world)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 25
cat > /workspace/iktest.py <<'EOF'
import sys, numpy as np, rclpy
sys.path.insert(0,'/workspace')
from ctl import Ctl
from scipy.spatial.transform import Rotation as Rot
c=Ctl()
q0=c.arm_q()
for yaw in [90,-90]:
    for z in [1.06,0.955]:
        R = Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
        hand = np.array([-0.09,-0.085,z]) - 0.1034*R.as_matrix()[:,2]
        try:
            sol=c.solve_ik(hand, R.as_quat(), seed=q0)
            print(yaw,z,'OK',np.round(sol,3))
        except SystemExit as e:
            print(yaw,z,e)
rclpy.shutdown()
EOF
timeout 600 python3 iktest.py

# openrua op 26
timeout 600 python3 ctl.py goto -0.09 -0.085 1.06 -90 4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 27
timeout 60 python3 depth_world.py robot0_eye_in_hand && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_world.npz'); xyz=d['xyz']; z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.90)&(z<1.0)&(x>-0.2)&(x<0.0)&(y>-0.15)&(y<0.1)
print('cup-ish pts',m.sum())
xs=np.arange(-0.16,-0.01,0.01); ys=np.arange(-0.12,0.07,0.01)
print('cols y from -0.12')
for xa in xs:
  print(f'{xa:6.2f}',' '.join(f'{(z[(x>=xa)&(x<xa+0.01)&(y>=ya)&(y<ya+0.01)&(z>0.5)&(z<1.02)].max()-0.88)*100:3.0f}' if ((x>=xa)&(x<xa+0.01)&(y>=ya)&(y<ya+0.01)&(z>0.5)&(z<1.02)).sum() else '  .' for ya in ys))
# handle top points
mh=(z>0.955)&(z<0.985)&(y<-0.06)&(y>-0.11)&(x>-0.13)&(x<-0.05)
print('handle pts',mh.sum(), 'x',x[mh].min() if mh.sum() else None, x[mh].max() if mh.sum() else None,'y',y[mh].min() if mh.sum() else None,y[mh].max() if mh.sum() else None,'zmax',z[mh].max() if mh.sum() else None)
"

# openrua op 28
timeout 600 python3 ctl.py goto -0.084 -0.08 0.955 -90 3 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 29
python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[200:400,180:420]; cv2.imwrite('crop_a.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); crop=im[150:330,220:420]; cv2.imwrite('crop_s.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"; timeout 30 python3 ctl.py js | head -9

# openrua op 30
timeout 300 python3 ctl.py grip 0.0

# openrua op 31
timeout 600 python3 ctl.py goto -0.084 -0.08 1.12 -90 3 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 30 python3 ctl.py js | tail -2

# openrua op 32
timeout 60 python3 depth_world.py birdview && python3 -c "
import numpy as np
d=np.load('birdview_world.npz'); xyz=d['xyz']; z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
xs=np.arange(-0.16,-0.01,0.01); ys=np.arange(-0.12,0.07,0.01)
print('cols y from -0.12')
for xa in xs:
  print(f'{xa:6.2f}',' '.join(f'{(z[(x>=xa)&(x<xa+0.01)&(y>=ya)&(y<ya+0.01)&(z>0.5)&(z<1.05)].max()-0.88)*100:3.0f}' if ((x>=xa)&(x<xa+0.01)&(y>=ya)&(y<ya+0.01)&(z>0.5)&(z<1.05)).sum() else '  .' for ya in ys))
"; python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[300:420,230:380]; cv2.imwrite('crop_a.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 33
timeout 60 python3 tools/perception/cam_snap.py frontview && python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[280:380,250:370]; cv2.imwrite('crop_f.png', cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))"

# openrua op 34
cat > /workspace/iktest.py <<'EOF'
import sys, numpy as np, rclpy
sys.path.insert(0,'/workspace')
from ctl import Ctl
from scipy.spatial.transform import Rotation as Rot
c=Ctl()
q0=c.arm_q()
for name,(x,y,z,yaw) in {'pregrasp':(-0.087,0.046,1.06,0),'grasp':(-0.087,0.046,0.965,0),'lift':(-0.087,0.046,1.15,0),
                         'preplace':(-0.4695,-0.1425,1.20,90),'place':(-0.4695,-0.1425,1.075,90),
                         'preplace_b':(-0.4695,-0.1425,1.20,-270)}.items():
    R = Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    hand = np.array([x,y,z]) - 0.1034*R.as_matrix()[:,2]
    try:
        sol=c.solve_ik(hand, R.as_quat(), seed=q0); print(name,'OK',np.round(sol,3))
    except SystemExit as e: print(name,e)
rclpy.shutdown()
EOF
timeout 600 python3 iktest.py

# openrua op 35
cat > /workspace/iktest.py <<'EOF'
import sys, numpy as np, rclpy
sys.path.insert(0,'/workspace')
from ctl import Ctl
from scipy.spatial.transform import Rotation as Rot
c=Ctl()
q0=c.arm_q()
seed=q0
for name,(x,y,z,yaw) in {'lift_hi':(-0.087,0.046,1.30,0),'mid':(-0.28,-0.05,1.30,45),'preplace_hi':(-0.4695,-0.1425,1.30,90),'place':(-0.4695,-0.1425,1.075,90)}.items():
    R = Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    hand = np.array([x,y,z]) - 0.1034*R.as_matrix()[:,2]
    try:
        sol=c.solve_ik(hand, R.as_quat(), seed=seed); print(name,'OK',np.round(sol,3)); seed=sol
    except SystemExit as e: print(name,e)
rclpy.shutdown()
EOF
timeout 600 python3 iktest.py

# openrua op 36
timeout 300 python3 ctl.py grip 0.04 && timeout 600 python3 ctl.py goto -0.087 0.046 1.06 0 4 && timeout 600 python3 ctl.py goto -0.087 0.046 0.965 0 2.5 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 37
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 30 python3 ctl.py js; timeout 15 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A1 Translation | head -2

# openrua op 38
timeout 120 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [-0.1923,0.7602,0.2848,-1.7280,-0.3053,2.4423,0.2825]}}}" 2>&1 | grep -o "position=.*" | head -2; timeout 120 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [-0.105,0.746,0.189,-1.732,-0.203,2.458,0.193]}}}" 2>&1 | grep -o "position=.*" | head -2

# openrua op 39
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "        # hand pointing down: 180deg about X, then yaw about world Z\\n        R = Rot.from_euler(\\"z\\", yaw_deg, degrees=True) * Rot.from_euler(\\"x\\", 180, degrees=True)\\n        q = R.as_quat()  # xyzw", "new_string": "        # hand pointing down: 180deg about X, then yaw about world Z\\n        R = Rot.from_euler(\\"z\\", yaw_deg, degrees=True) * Rot.from_euler(\\"x\\", 180, degrees=True)\\n        # verified via /compute_fk: the IK tip link is panda_link8, which sits\\n        # 45deg yawed relative to panda_hand (same origin) -> request link8\\n        # orientation = Rz(-45) * desired hand orientation\\n        q = (Rot.from_euler(\\"z\\", -45, degrees=True) * R).as_quat()  # xyzw", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 40
timeout 600 python3 ctl.py goto -0.087 0.046 1.03 0 2.5 && timeout 600 python3 ctl.py goto -0.087 0.046 0.965 0 2.5 && timeout 120 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: $(python3 -c "
import sys;sys.path.insert(0,'/workspace')
import rclpy
from ctl import Ctl
c=Ctl(); print([round(v,4) for v in c.arm_q()]); rclpy.shutdown()" | tail -1)}}}" 2>&1 | grep -o "Point.*" | head -1; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 41
timeout 300 python3 ctl.py grip 0.0 && timeout 30 python3 ctl.py js | tail -2

# openrua op 42
timeout 600 python3 ctl.py goto -0.087 0.046 1.10 0 3 && timeout 30 python3 ctl.py js | tail -2 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 43
timeout 600 python3 ctl.py goto -0.087 0.046 1.30 0 3 && timeout 60 python3 depth_world.py birdview && python3 -c "
import numpy as np
d=np.load('birdview_world.npz'); xyz=d['xyz']; z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
# cup rim should be near z 1.30+0.025=1.325 ; base at ~1.22
for lo in np.arange(1.18,1.36,0.02):
  m=(z>=lo)&(z<lo+0.02)&(x>-0.25)&(x<0.05)&(y>-0.15)&(y<0.15)
  if m.sum()>5: print(f'z {lo:.2f}: n={m.sum():4d} x {x[m].min():.3f}..{x[m].max():.3f} y {y[m].min():.3f}..{y[m].max():.3f}')
"

# openrua op 44
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py frontview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[80:330,180:460]; cv2.imwrite('crop_a.png', cv2.resize(crop,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('frontview.png'); crop=im[100:330,200:440]; cv2.imwrite('crop_f.png', cv2.resize(crop,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 45
cat > ikprobe.py <<'EOF'
import sys, numpy as np
from ctl import Ctl, TCP
from scipy.spatial.transform import Rotation as Rot
c = Ctl()
seed = c.arm_q()
for spec in sys.argv[1:]:
    x,y,z,yaw = map(float, spec.split(","))
    R = Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    q = (Rot.from_euler("z", -45, degrees=True) * R).as_quat()
    hp = np.array([x,y,z]) - TCP*R.as_matrix()[:,2]
    try:
        sol = c.solve_ik(hp, q, seed=seed)
        print(spec, "OK", np.round(sol,3).tolist()); seed = sol
    except SystemExit as e:
        print(spec, "FAIL", e)
EOF
timeout 300 python3 ikprobe.py -0.28,-0.05,1.30,45 -0.4895,-0.1425,1.30,90 -0.4895,-0.1425,1.10,90 -0.4795,-0.1425,1.10,90 -0.4895,-0.1425,1.08,90

# openrua op 46
timeout 600 python3 ctl.py goto -0.28 -0.05 1.30 45 4 && timeout 600 python3 ctl.py goto -0.4895 -0.1425 1.30 90 5 && timeout 60 python3 ctl.py js | grep finger

# openrua op 47
timeout 600 python3 ctl.py goto -0.4895 -0.1425 1.30 90 5 && timeout 60 python3 ctl.py js | grep finger

# openrua op 48
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 depth_world.py birdview >/dev/null && python3 -c "
import cv2, numpy as np
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[40:330,120:460],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
d=np.load('birdview_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
for lo,hi in [(1.15,1.22),(1.22,1.27),(1.27,1.32),(1.32,1.36)]:
    s=d[(d[:,2]>lo)&(d[:,2]<hi)&(d[:,0]<-0.30)&(d[:,0]>-0.60)&(d[:,1]<0.0)&(d[:,1]>-0.30)]
    if len(s): print(lo,hi,len(s),'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
"

# openrua op 49
timeout 600 python3 ctl.py goto -0.4895 -0.1425 1.10 90 4; timeout 60 python3 ctl.py js | grep finger; timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('frontview.png'); cv2.imwrite('crop_f.png', cv2.resize(im[120:360,150:450],None,fx=2.2,fy=2.2,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[40:330,120:460],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))"

# openrua op 50
timeout 300 python3 ctl.py grip 0.04 && timeout 600 python3 ctl.py goto -0.4895 -0.1425 1.30 90 3; timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py frontview && python3 -c "
import cv2
im=cv2.imread('frontview.png'); cv2.imwrite('crop_f.png', cv2.resize(im[120:360,150:450],None,fx=2.2,fy=2.2,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[40:330,120:460],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))"

# openrua op 51
timeout 120 python3 depth_world.py birdview >/dev/null && python3 -c "
import numpy as np
d=np.load('birdview_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
s=d[(d[:,0]<-0.33)&(d[:,0]>-0.50)&(d[:,1]<-0.07)&(d[:,1]>-0.215)]
# height map 1cm bins in x, report max z per x column for y in middle column
for x0 in np.arange(-0.50,-0.33,0.01):
    c=s[(s[:,0]>=x0)&(s[:,0]<x0+0.01)]
    if len(c): print(f'x {x0:.2f}: zmax {c[:,2].max():.3f} zmed {np.median(c[:,2]):.3f} y@zmax {c[c[:,2].argmax(),1]:.3f} n {len(c)}')
"

# openrua op 52
timeout 600 python3 ctl.py goto -0.20 -0.35 1.35 0 4; timeout 120 python3 depth_world.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import numpy as np, cv2
d=np.load('birdview_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
s=d[(d[:,0]<-0.30)&(d[:,0]>-0.50)&(d[:,1]<-0.07)&(d[:,1]>-0.215)&(d[:,2]>1.0)]
for x0 in np.arange(-0.50,-0.30,0.01):
    c=s[(s[:,0]>=x0)&(s[:,0]<x0+0.01)]
    if len(c): print(f'x {x0:.2f}: zmax {c[:,2].max():.3f} y@zmax {c[c[:,2].argmax(),1]:.3f} yrange {c[:,1].min():.3f} {c[:,1].max():.3f} n {len(c)}')
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[150:330,150:400],None,fx=2.8,fy=2.8,interpolation=cv2.INTER_CUBIC))
"

# openrua op 53
python3 -c "
import numpy as np
d=np.load('birdview_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
s=d[(d[:,0]<-0.30)&(d[:,0]>-0.50)&(d[:,1]<-0.075)&(d[:,1]>-0.20)&(d[:,2]<1.3)]
for x0 in np.arange(-0.50,-0.30,0.01):
    c=s[(s[:,0]>=x0)&(s[:,0]<x0+0.01)]
    if len(c):
        zs=np.round(np.percentile(c[:,2],[0,25,50,75,100]),3)
        print(f'x {x0:.2f}: z pct {zs} y@zmax {c[c[:,2].argmax(),1]:.3f} n {len(c)}')
"

# openrua op 54
timeout 600 python3 ctl.py goto -0.20 -0.35 1.35 0 5; timeout 600 python3 ctl.py goto -0.20 -0.35 1.35 0 3

# openrua op 55
timeout 120 python3 depth_world.py birdview >/dev/null && timeout 120 python3 depth_world.py agentview >/dev/null && python3 -c "
import numpy as np
for cam in ['birdview','agentview']:
    d=np.load(cam+'_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
    s=d[(d[:,0]<-0.30)&(d[:,0]>-0.50)&(d[:,1]<-0.075)&(d[:,1]>-0.20)&(d[:,2]<1.3)]
    print(cam)
    for x0 in np.arange(-0.50,-0.30,0.01):
        c=s[(s[:,0]>=x0)&(s[:,0]<x0+0.01)]
        if len(c):
            zs=np.round(np.percentile(c[:,2],[0,25,50,75,100]),3)
            print(f'x {x0:.2f}: z pct {zs} y@zmax {c[c[:,2].argmax(),1]:.3f} n {len(c)}')
"

# openrua op 56
cat > ikprobe2.py <<'EOF'
import sys, numpy as np
from ctl import Ctl, TCP
from scipy.spatial.transform import Rotation as Rot
c = Ctl()
seed0 = c.arm_q()
# spec: x,y,z,yaw,pitch  -> R = Ry(pitch) * Rz(yaw) * Rx(180)  (pitch about world y, applied last)
for spec in sys.argv[1:]:
    x,y,z,yaw,pitch = map(float, spec.split(","))
    R = Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    q = (Rot.from_euler("z", -45, degrees=True) * R).as_quat()
    hz = R.as_matrix()[:,2]
    hp = np.array([x,y,z]) - TCP*hz
    try:
        sol = c.solve_ik(hp, q, seed=seed0)
        print(spec, "OK hand_z", np.round(hz,2), np.round(sol,3).tolist())
    except SystemExit as e:
        print(spec, "FAIL", e)
EOF
timeout 300 python3 ikprobe2.py -0.12,-0.10,0.98,0,-90 -0.12,-0.10,0.98,0,-60 -0.15,-0.15,1.0,0,-90 -0.12,-0.10,0.98,0,90 -0.10,-0.10,1.05,0,-90 -0.12,-0.10,0.98,180,-90

# openrua op 57
timeout 300 python3 ctl.py grip 0.0 && timeout 600 python3 ctl.py goto -0.455 -0.13 1.20 0 4 && timeout 600 python3 ctl.py goto -0.455 -0.13 1.20 0 3 && timeout 600 python3 ctl.py goto -0.455 -0.13 1.00 0 3

# openrua op 58
timeout 600 python3 ctl.py goto -0.455 -0.13 1.00 0 3; timeout 60 python3 ctl.py js | grep finger; timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[100:330,150:400],None,fx=2.8,fy=2.8,interpolation=cv2.INTER_CUBIC))"

# openrua op 59
cat > fk.py <<'EOF'
import numpy as np, rclpy
from ctl import Ctl, JOINTS, TCP
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as Rot
c = Ctl()
cl = c.node.create_client(GetPositionFK, "/compute_fk"); cl.wait_for_service(10)
req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
s = JointState(); s.name = list(JOINTS); s.position = c.arm_q(); req.robot_state.joint_state = s
f = cl.call_async(req); rclpy.spin_until_future_complete(c.node, f, timeout_sec=30)
p = f.result().pose_stamped[0].pose
R = Rot.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
hand = np.array([p.position.x,p.position.y,p.position.z]); tcp = hand + TCP*R.as_matrix()[:,2]
print("hand", hand.round(4), "tcp", tcp.round(4), "hand_z", R.as_matrix()[:,2].round(3), "hand_y", R.as_matrix()[:,1].round(3))
EOF
timeout 60 python3 fk.py

# openrua op 60
timeout 600 python3 ctl.py goto -0.455 -0.13 1.25 0 3; timeout 600 python3 ctl.py goto -0.455 -0.13 1.25 90 4; timeout 600 python3 ctl.py goto -0.455 -0.13 1.25 90 3; timeout 60 python3 fk.py

# openrua op 61
timeout 600 python3 ctl.py goto -0.455 -0.13 1.10 90 3; timeout 600 python3 ctl.py goto -0.455 -0.13 1.10 90 3; timeout 60 python3 fk.py; timeout 600 python3 ctl.py goto -0.455 -0.13 1.018 90 3; timeout 600 python3 ctl.py goto -0.455 -0.13 1.018 90 3; timeout 60 python3 fk.py

# openrua op 62
timeout 600 python3 ctl.py goto -0.459 -0.13 1.10 90 3; timeout 600 python3 ctl.py goto -0.459 -0.13 1.10 90 2; timeout 60 python3 fk.py; timeout 600 python3 ctl.py goto -0.459 -0.13 1.02 90 3; timeout 600 python3 ctl.py goto -0.459 -0.13 1.02 90 3; timeout 60 python3 fk.py

# openrua op 63
timeout 600 python3 ctl.py goto -0.459 -0.13 1.15 90 3; timeout 300 python3 ctl.py grip 0.04; timeout 600 python3 ctl.py goto -0.459 -0.13 1.15 0 3; timeout 600 python3 ctl.py goto -0.459 -0.13 1.15 0 2; timeout 60 python3 fk.py; timeout 600 python3 ctl.py goto -0.459 -0.13 1.024 0 3; timeout 600 python3 ctl.py goto -0.459 -0.13 1.024 0 3; timeout 60 python3 fk.py

# openrua op 64
timeout 600 python3 ctl.py goto -0.459 -0.13 1.20 0 3; timeout 600 python3 ctl.py goto -0.459 -0.13 1.20 0 3; timeout 600 python3 ctl.py goto -0.459 -0.13 1.20 0 3; timeout 60 python3 fk.py; timeout 60 python3 ctl.py js

# openrua op 65
timeout 600 python3 ctl.py goto -0.459 -0.13 1.10 0 3; timeout 600 python3 ctl.py goto -0.459 -0.13 1.10 0 2; timeout 60 python3 fk.py; timeout 600 python3 ctl.py goto -0.459 -0.13 1.024 0 3; timeout 600 python3 ctl.py goto -0.459 -0.13 1.024 0 2; timeout 60 python3 fk.py

# openrua op 66
timeout 600 python3 ctl.py goto -0.432 -0.13 1.024 0 3; timeout 60 python3 fk.py; timeout 600 python3 ctl.py goto -0.432 -0.13 1.024 0 2; timeout 60 python3 fk.py

# openrua op 67
timeout 600 python3 ctl.py goto -0.432 -0.13 1.25 0 3; timeout 600 python3 ctl.py goto -0.20 -0.35 1.35 0 4; timeout 600 python3 ctl.py goto -0.20 -0.35 1.35 0 3; timeout 120 python3 depth_world.py birdview >/dev/null && timeout 120 python3 depth_world.py agentview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import numpy as np, cv2
for cam in ['birdview','agentview']:
    d=np.load(cam+'_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
    s=d[(d[:,0]<-0.25)&(d[:,0]>-0.50)&(d[:,1]<-0.075)&(d[:,1]>-0.20)&(d[:,2]<1.3)&(d[:,2]>0.97)]
    print(cam)
    for x0 in np.arange(-0.50,-0.25,0.01):
        c=s[(s[:,0]>=x0)&(s[:,0]<x0+0.01)]
        if len(c):
            zs=np.round(np.percentile(c[:,2],[0,50,100]),3)
            print(f'x {x0:.2f}: z pct {zs} y@zmax {c[c[:,2].argmax(),1]:.3f} yrange {c[:,1].min():.3f} {c[:,1].max():.3f} n {len(c)}')
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[100:330,150:400],None,fx=2.8,fy=2.8,interpolation=cv2.INTER_CUBIC))
"

# openrua op 68
timeout 600 python3 ctl.py goto -0.331 -0.079 1.25 0 4; timeout 600 python3 ctl.py goto -0.331 -0.079 1.25 0 3; timeout 600 python3 ctl.py goto -0.331 -0.079 1.09 0 3; timeout 600 python3 ctl.py goto -0.331 -0.079 1.09 0 2; timeout 60 python3 fk.py; timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[100:330,150:400],None,fx=2.8,fy=2.8,interpolation=cv2.INTER_CUBIC))"

# openrua op 69
timeout 120 python3 depth_world.py agentview >/dev/null && python3 -c "
import numpy as np
d=np.load('agentview_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
# fingers: z 1.085..1.14
for lo,hi,yl,yh in [(1.085,1.14,-0.06,-0.02),(1.085,1.14,-0.14,-0.10)]:
    s=d[(d[:,2]>lo)&(d[:,2]<hi)&(d[:,1]>yl)&(d[:,1]<yh)&(d[:,0]>-0.40)&(d[:,0]<-0.25)]
    print('finger', yl,yh, 'x', s[:,0].min().round(4), s[:,0].max().round(4), 'y', s[:,1].min().round(3), s[:,1].max().round(3), len(s))
# front wall of right compartment / divider front face: points z 1.0..1.06 near x -0.35
s=d[(d[:,2]>0.98)&(d[:,2]<1.065)&(d[:,1]>-0.085)&(d[:,1]<-0.055)&(d[:,0]>-0.40)&(d[:,0]<-0.30)]
print('divider y-0.07 region: x', s[:,0].min().round(4), s[:,0].max().round(4), 'zmax', s[:,2].max().round(3), len(s))
# cup rim: max x of cup points at z>1.0 y -0.19..-0.08
s=d[(d[:,2]>1.0)&(d[:,2]<1.085)&(d[:,1]>-0.19)&(d[:,1]<-0.085)&(d[:,0]>-0.36)&(d[:,0]<-0.28)]
print('cup rim region x max', s[:,0].max().round(4), 'z at xmax', s[s[:,0].argmax(),2].round(3))
for x0 in np.arange(-0.34,-0.30,0.005):
    c=s[(s[:,0]>=x0)&(s[:,0]<x0+0.005)]
    if len(c): print(f'  x {x0:.3f}: zmax {c[:,2].max():.3f} n {len(c)}')
"

# openrua op 70
timeout 600 python3 ctl.py goto -0.331 -0.079 1.027 0 3; timeout 600 python3 ctl.py goto -0.331 -0.079 1.027 0 2; timeout 60 python3 fk.py

# openrua op 71
cat >> ctl.py <<'EOF'


def goto_tcp_pitch(c, x, y, z, yaw_deg, pitch_deg, seconds=4.0):
    """Like goto_tcp but with an extra pitch about world Y applied last
    (pitch>0 tilts the hand's pointing direction toward -X)."""
    R = (Rot.from_euler("y", pitch_deg, degrees=True) * Rot.from_euler("z", yaw_deg, degrees=True)
         * Rot.from_euler("x", 180, degrees=True))
    q = (Rot.from_euler("z", -45, degrees=True) * R).as_quat()
    hand_z = R.as_matrix()[:, 2]
    hand_pos = np.array([x, y, z]) - TCP * hand_z
    qj = c.solve_ik(hand_pos, q)
    return c.move(qj, seconds)


if __name__ == "__main__" and sys.argv[1] == "gotop":
    c = Ctl()
    x, y, z, yaw, pitch = map(float, sys.argv[2:7])
    secs = float(sys.argv[7]) if len(sys.argv) > 7 else 4.0
    goto_tcp_pitch(c, x, y, z, yaw, pitch, secs)
    rclpy.shutdown()
EOF
python3 - <<'EOF'
# guard: the original main block calls rclpy.shutdown(); make it skip unknown cmds gracefully
src=open('/workspace/ctl.py').read()
src=src.replace('''if __name__ == "__main__":
    c = Ctl()
    cmd = sys.argv[1]''','''if __name__ == "__main__" and sys.argv[1] != "gotop":
    c = Ctl()
    cmd = sys.argv[1]''')
open('/workspace/ctl.py','w').write(src)
EOF
timeout 600 python3 ctl.py gotop -0.322 -0.079 1.15 0 30 4; timeout 600 python3 ctl.py gotop -0.322 -0.079 1.15 0 30 3; timeout 60 python3 fk.py

# openrua op 72
python3 - <<'EOF'
src=open('/workspace/ctl.py').read()
src=src.replace('''    q = (Rot.from_euler("z", -45, degrees=True) * R).as_quat()
    hand_z = R.as_matrix()[:, 2]
    hand_pos = np.array([x, y, z]) - TCP * hand_z
    qj = c.solve_ik(hand_pos, q)''','''    # link8 = hand * Rz(+45) in the hand's local frame (same as the world
    # Rz(-45) pre-multiplication in goto_tcp when the hand points straight down)
    q = (R * Rot.from_euler("z", 45, degrees=True)).as_quat()
    hand_z = R.as_matrix()[:, 2]
    hand_pos = np.array([x, y, z]) - TCP * hand_z
    qj = c.solve_ik(hand_pos, q)''')
open('/workspace/ctl.py','w').write(src)
EOF
timeout 600 python3 ctl.py gotop -0.322 -0.079 1.15 0 30 4; timeout 600 python3 ctl.py gotop -0.322 -0.079 1.15 0 30 3; timeout 60 python3 fk.py

# openrua op 73
timeout 600 python3 ctl.py gotop -0.322 -0.079 1.026 0 30 3; timeout 600 python3 ctl.py gotop -0.322 -0.079 1.026 0 30 2; timeout 60 python3 fk.py; timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[100:330,150:400],None,fx=2.8,fy=2.8,interpolation=cv2.INTER_CUBIC))"

# openrua op 74
timeout 600 python3 ctl.py gotop -0.322 -0.079 1.20 0 30 3; timeout 600 python3 ctl.py gotop -0.322 -0.079 1.20 0 0 3; cat > ikprobe3.py <<'EOF'
import sys, numpy as np
from ctl import Ctl, TCP
from scipy.spatial.transform import Rotation as Rot
c = Ctl(); seed = c.arm_q()
for spec in sys.argv[1:]:
    x,y,z,yaw,pitch = map(float, spec.split(","))
    R = Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    q = (R * Rot.from_euler("z", 45, degrees=True)).as_quat()
    hz = R.as_matrix()[:,2]; hp = np.array([x,y,z]) - TCP*hz
    try:
        sol = c.solve_ik(hp, q, seed=seed); print(spec, "OK", np.round(sol,2).tolist()); seed = sol
    except SystemExit as e: print(spec, "FAIL", e)
EOF
timeout 300 python3 ikprobe3.py -0.28,-0.131,1.20,90,75 -0.28,-0.131,1.078,90,75 -0.325,-0.131,1.078,90,75 -0.325,-0.131,1.25,90,75 -0.20,-0.15,1.25,90,30 -0.20,-0.15,1.25,90,-15

# openrua op 75
timeout 600 python3 ctl.py gotop -0.28 -0.131 1.20 90 75 5; timeout 600 python3 ctl.py gotop -0.28 -0.131 1.20 90 75 3; timeout 60 python3 fk.py; timeout 600 python3 ctl.py gotop -0.28 -0.131 1.078 90 75 3; timeout 600 python3 ctl.py gotop -0.28 -0.131 1.078 90 75 2; timeout 60 python3 fk.py; timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('frontview.png'); cv2.imwrite('crop_f.png', cv2.resize(im[120:360,150:450],None,fx=2.2,fy=2.2,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[100:330,150:400],None,fx=2.8,fy=2.8,interpolation=cv2.INTER_CUBIC))"

# openrua op 76
timeout 600 python3 ctl.py gotop -0.28 -0.131 1.30 90 75 3; timeout 600 python3 ctl.py goto -0.20 -0.35 1.40 0 5; timeout 600 python3 ctl.py goto -0.20 -0.35 1.40 0 3; timeout 120 python3 depth_world.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import numpy as np, cv2
d=np.load('birdview_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
s=d[(d[:,0]>-0.36)&(d[:,0]<-0.05)&(d[:,1]>-0.30)&(d[:,1]<0.05)&(d[:,2]>0.90)&(d[:,2]<1.15)]
print('objects above table in front of caddy:')
for z0 in np.arange(0.90,1.12,0.02):
    c=s[(s[:,2]>=z0)&(s[:,2]<z0+0.02)]
    if len(c): print(f'z {z0:.2f}: x {c[:,0].min():.3f}..{c[:,0].max():.3f} y {c[:,1].min():.3f}..{c[:,1].max():.3f} n {len(c)}')
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[100:400,100:450],None,fx=2.2,fy=2.2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 77
timeout 300 python3 ctl.py grip 0.0; timeout 600 python3 ctl.py goto -0.276 -0.235 1.15 0 4; timeout 600 python3 ctl.py goto -0.276 -0.235 1.15 0 3; timeout 600 python3 ctl.py goto -0.276 -0.235 0.972 0 3; timeout 600 python3 ctl.py goto -0.276 -0.235 0.972 0 2; timeout 60 python3 fk.py

# openrua op 78
timeout 600 python3 ctl.py goto -0.276 -0.12 0.972 0 1.5; timeout 60 python3 fk.py; timeout 600 python3 ctl.py goto -0.276 -0.12 1.20 0 3; timeout 600 python3 ctl.py goto -0.20 -0.40 1.40 0 4; timeout 600 python3 ctl.py goto -0.20 -0.40 1.40 0 3; timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[100:400,100:450],None,fx=2.2,fy=2.2,interpolation=cv2.INTER_CUBIC))"

# openrua op 79
timeout 120 python3 depth_world.py birdview >/dev/null && timeout 120 python3 depth_world.py agentview >/dev/null && python3 -c "
import numpy as np
for cam in ['birdview','agentview']:
    d=np.load(cam+'_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
    top=d[(d[:,2]>0.975)&(d[:,2]<1.0)&(d[:,0]>-0.34)&(d[:,0]<-0.15)&(d[:,1]>-0.30)&(d[:,1]<0.05)]
    print(cam,'base disc: x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3),'center',top[:,:2].mean(0).round(4),'zmax',top[:,2].max().round(3),len(top))
    cx,cy=top[:,:2].mean(0)
    h=d[(d[:,2]>0.885)&(d[:,2]<0.98)&(d[:,0]>cx-0.03)&(d[:,0]<cx+0.03)&(d[:,1]<cy-0.045)&(d[:,1]>cy-0.12)]
    print('  handle region: y',h[:,1].min().round(3),h[:,1].max().round(3),'x',h[:,0].min().round(3),h[:,0].max().round(3),'z',h[:,2].min().round(3),h[:,2].max().round(3),len(h))
    for z0 in np.arange(0.885,0.98,0.01):
        c=h[(h[:,2]>=z0)&(h[:,2]<z0+0.01)]
        if len(c): print(f'   z {z0:.3f}: ymin {c[:,1].min():.3f} x {c[:,0].min():.3f}..{c[:,0].max():.3f} n {len(c)}')
"

# openrua op 80
timeout 120 python3 depth_world.py sideview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py sideview && python3 -c "
import numpy as np, cv2
d=np.load('sideview_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
cx,cy=-0.266,-0.131
h=d[(d[:,2]>0.885)&(d[:,2]<0.985)&(d[:,0]>cx-0.04)&(d[:,0]<cx+0.04)&(d[:,1]<cy-0.045)&(d[:,1]>cy-0.13)]
print('handle pts',len(h))
for z0 in np.arange(0.885,0.985,0.01):
    c=h[(h[:,2]>=z0)&(h[:,2]<z0+0.01)]
    if len(c): print(f'z {z0:.3f}: y {c[:,1].min():.3f}..{c[:,1].max():.3f}  x {c[:,0].min():.3f}..{c[:,0].max():.3f} n {len(c)}')
im=cv2.imread('sideview.png'); print(im.shape)
cv2.imwrite('crop_s.png', im)
"

# openrua op 81
timeout 600 python3 ctl.py goto -0.266 -0.19 1.25 0 4; timeout 600 python3 ctl.py goto -0.266 -0.19 1.25 0 3; timeout 120 python3 depth_world.py robot0_eye_in_hand >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
cx,cy=-0.266,-0.131
top=d[(d[:,2]>0.975)&(d[:,2]<1.0)&(d[:,0]>-0.34)&(d[:,0]<-0.15)&(d[:,1]>-0.30)&(d[:,1]<0.05)]
print('base disc center',top[:,:2].mean(0).round(4),'x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3))
h=d[(d[:,2]>0.885)&(d[:,2]<0.985)&(d[:,0]>cx-0.04)&(d[:,0]<cx+0.04)&(d[:,1]<cy-0.045)&(d[:,1]>cy-0.13)]
print('handle pts',len(h))
for z0 in np.arange(0.885,0.985,0.01):
    c=h[(h[:,2]>=z0)&(h[:,2]<z0+0.01)]
    if len(c): print(f'z {z0:.3f}: y {c[:,1].min():.3f}..{c[:,1].max():.3f}  x {c[:,0].min():.3f}..{c[:,0].max():.3f} n {len(c)}')
"

# openrua op 82
python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
cx,cy=-0.266,-0.131
r=np.hypot(d[:,0]-cx,d[:,1]-cy); ang=np.degrees(np.arctan2(d[:,1]-cy,d[:,0]-cx))
m=(r>0.056)&(r<0.11)&(d[:,2]>0.885)&(d[:,2]<0.99)
h=d[m]; rr=r[m]; aa=ang[m]
print(len(h))
for z0 in np.arange(0.885,0.99,0.01):
    s=(h[:,2]>=z0)&(h[:,2]<z0+0.01)
    if s.sum(): print(f'z {z0:.3f}: r {rr[s].min():.3f}..{rr[s].max():.3f} ang {aa[s].min():.0f}..{aa[s].max():.0f} x {h[s,0].min():.3f}..{h[s,0].max():.3f} y {h[s,1].min():.3f}..{h[s,1].max():.3f} n {s.sum()}')
# also body top edge for axis refinement: points z 0.975-0.99
t=d[(d[:,2]>0.975)&(d[:,2]<0.995)&(r<0.06)]
print('disc center refined', t[:,:2].mean(0).round(4), 'r max', np.hypot(t[:,0]-cx,t[:,1]-cy).max().round(3))
"

# openrua op 83
python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_world.npz')['xyz'].reshape(-1,3); d=d[np.isfinite(d).all(1)]
cx,cy=-0.261,-0.1305
r=np.hypot(d[:,0]-cx,d[:,1]-cy); ang=np.degrees(np.arctan2(d[:,1]-cy,d[:,0]-cx))
m=(r>0.056)&(r<0.11)&(d[:,2]>0.885)&(d[:,2]<0.99)&(d[:,0]>-0.32)
h=d[m]; rr=r[m]; aa=ang[m]
print(len(h))
for z0 in np.arange(0.885,0.99,0.01):
    s=(h[:,2]>=z0)&(h[:,2]<z0+0.01)
    if s.sum(): print(f'z {z0:.3f}: r {rr[s].min():.3f}..{rr[s].max():.3f} ang {np.percentile(aa[s],[5,50,95]).round(0)} x {h[s,0].min():.3f}..{h[s,0].max():.3f} y {h[s,1].min():.3f}..{h[s,1].max():.3f} n {s.sum()}')
"

# openrua op 84
timeout 300 python3 ctl.py grip 0.0; timeout 600 python3 ctl.py goto -0.318 -0.1305 1.15 90 4; timeout 600 python3 ctl.py goto -0.318 -0.1305 1.15 90 3; timeout 600 python3 ctl.py goto -0.318 -0.1305 0.965 90 3; timeout 600 python3 ctl.py goto -0.318 -0.1305 0.965 90 2; timeout 60 python3 fk.py

# openrua op 85
timeout 600 python3 ctl.py goto -0.20 -0.14 0.974 90 2; timeout 60 python3 fk.py; timeout 600 python3 ctl.py goto -0.20 -0.14 1.20 90 3; timeout 60 python3 fk.py; timeout 120 python3 depth_world.py birdview >/dev/null; python3 - <<'EOF'
import numpy as np
d=np.load('birdview_world.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
m=(xyz[:,2]>0.9)&(xyz[:,0]>-0.34)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.3)&(xyz[:,1]<0.05)
p=xyz[m]; print(len(p)); 
top=p[p[:,2]>0.975]; print('top pts',len(top),'center',top[:,:2].mean(0),'x',top[:,0].min(),top[:,0].max(),'y',top[:,1].min(),top[:,1].max(),'z',top[:,2].min(),top[:,2].max())
lo=p[(p[:,2]>0.9)&(p[:,2]<0.975)]; print('lower pts',len(lo)); 
if len(lo):
  c=top[:,:2].mean(0); r=np.hypot(lo[:,0]-c[0],lo[:,1]-c[1]); ang=np.degrees(np.arctan2(lo[:,1]-c[1],lo[:,0]-c[0]))
  h=lo[r>0.06]; print('handle pts',len(h), 'ang med',np.median(ang[r>0.06]) if len(h) else None, 'r',h[:,0].size and (r[r>0.06].min(), r[r>0.06].max()))
EOF

# openrua op 86
python3 - <<'EOF'
import numpy as np
d=np.load('birdview_world.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
m=(xyz[:,2]>0.895)&(xyz[:,2]<1.05)&(xyz[:,0]>-0.34)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.3)&(xyz[:,1]<0.05)
p=xyz[m]; print(len(p))
for zb in np.arange(0.895,1.05,0.01):
    s=p[(p[:,2]>=zb)&(p[:,2]<zb+0.01)]
    if len(s): print(f"z {zb:.3f} n{len(s):4d} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null

# openrua op 87
timeout 600 python3 ctl.py goto -0.30 -0.14 0.974 90 3; timeout 60 python3 fk.py; for i in 1 2 3 4; do timeout 600 python3 ctl.py goto -0.20 -0.14 0.974 90 3; timeout 60 python3 fk.py; done

# openrua op 88
timeout 600 python3 ctl.py goto -0.20 -0.14 1.25 90 3; timeout 600 python3 ctl.py goto -0.05 -0.25 1.30 90 3; timeout 60 python3 fk.py; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; timeout 120 python3 depth_world.py birdview >/dev/null; python3 - <<'EOF'
import numpy as np
d=np.load('birdview_world.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
m=(xyz[:,2]>0.895)&(xyz[:,2]<1.05)&(xyz[:,0]>-0.34)&(xyz[:,0]<0.05)&(xyz[:,1]>-0.3)&(xyz[:,1]<0.08)
p=xyz[m]; print(len(p))
for zb in np.arange(0.895,1.05,0.01):
    s=p[(p[:,2]>=zb)&(p[:,2]<zb+0.01)]
    if len(s): print(f"z {zb:.3f} n{len(s):4d} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
top=p[p[:,2]>0.975]; c=top[:,:2].mean(0); print('top center',c)
lo=p[(p[:,2]>0.9)&(p[:,2]<0.97)]; r=np.hypot(lo[:,0]-c[0],lo[:,1]-c[1]); ang=np.degrees(np.arctan2(lo[:,1]-c[1],lo[:,0]-c[0]))
h=r>0.065; print('handle n',h.sum(),'ang',np.percentile(ang[h],[10,50,90]) if h.sum() else None,'r',np.percentile(r[h],[10,50,90]) if h.sum() else None)
EOF

# openrua op 89
timeout 600 python3 ctl.py goto -0.30 0.10 1.40 0 4; timeout 600 python3 ctl.py goto -0.30 0.10 1.40 0 3; timeout 60 python3 fk.py; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; timeout 120 python3 depth_world.py birdview >/dev/null; timeout 120 python3 depth_world.py agentview >/dev/null; python3 - <<'EOF'
import numpy as np
for cam in ['birdview','agentview']:
    d=np.load(f'{cam}_world.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
    m=(xyz[:,2]>0.885)&(xyz[:,2]<1.05)&(xyz[:,0]>-0.335)&(xyz[:,0]<0.05)&(xyz[:,1]>-0.3)&(xyz[:,1]<0.08)
    p=xyz[m]; print(cam,len(p))
    for zb in np.arange(0.885,1.05,0.01):
        s=p[(p[:,2]>=zb)&(p[:,2]<zb+0.01)]
        if len(s): print(f"z {zb:.3f} n{len(s):4d} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
    print('x-bins (top z, y range):')
    for xb in np.arange(-0.33,0.0,0.01):
        s=p[(p[:,0]>=xb)&(p[:,0]<xb+0.01)]
        if len(s): print(f"x {xb:.2f} n{len(s):4d} zmax {s[:,2].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}  y@top {s[np.argmax(s[:,2]),1]:.3f}")
EOF

# openrua op 90
python3 - <<'EOF'
import numpy as np
d=np.load('agentview_world.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
m=(xyz[:,2]>0.89)&(xyz[:,2]<1.05)&(xyz[:,0]>-0.23)&(xyz[:,0]<-0.07)&(xyz[:,1]>-0.20)&(xyz[:,1]<-0.02)
p=xyz[m]
ys=np.arange(-0.19,-0.02,0.01); xs=np.arange(-0.22,-0.07,0.01)
print('     '+' '.join(f'{y:6.2f}' for y in ys))
for xb in xs:
    row=[]
    for yb in ys:
        s=p[(p[:,0]>=xb)&(p[:,0]<xb+0.01)&(p[:,1]>=yb)&(p[:,1]<yb+0.01)]
        row.append(f'{s[:,2].max():6.3f}' if len(s) else '   .  ')
    print(f'{xb:5.2f} '+' '.join(row))
EOF

# openrua op 91
cat ikprobe3.py | head -40

# openrua op 92
timeout 300 python3 ikprobe3.py -0.252,-0.046,1.10,0,-45 -0.252,-0.046,0.99,0,-45 -0.195,-0.046,0.934,0,-45 2>&1 | grep -v '^\[' ; echo ---; timeout 300 python3 ikprobe3.py -0.252,-0.046,1.10,180,-45 -0.252,-0.046,0.99,180,-45 -0.195,-0.046,0.934,180,-45 2>&1 | grep -v '^\['; echo ---; timeout 300 python3 ikprobe3.py -0.252,-0.046,1.10,0,-50 -0.252,-0.046,0.99,0,-50 -0.195,-0.046,0.934,0,-50 2>&1 | grep -v '^\['

# openrua op 93
cat > ikseed.py <<'EOF'
import sys, numpy as np
from ctl import Ctl, TCP, M
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
import rclpy
c = Ctl()
lim = np.array(M["actuators"][0]["limits_rad"])
def ik(hp, q, seed, t=1.0):
    req = GetPositionIK.Request(); req.ik_request.group_name = M["planning"]["group"]
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = map(float, hp)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    s = JointState(); s.name = list(M["actuators"][0]["joints"]); s.position = [float(v) for v in seed]
    req.ik_request.robot_state.joint_state = s
    req.ik_request.timeout.sec = int(t); req.ik_request.timeout.nanosec = int((t % 1) * 1e9)
    fut = c.ik.call_async(req); rclpy.spin_until_future_complete(c.node, fut, timeout_sec=30)
    r = fut.result()
    if r is None or r.error_code.val != 1: return None
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return [sol[j] for j in s.name]
x, y, z, yaw, pitch = map(float, sys.argv[1].split(","))
n = int(sys.argv[2]) if len(sys.argv) > 2 else 20
R = Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
q = (R * Rot.from_euler("z", 45, degrees=True)).as_quat(); hz = R.as_matrix()[:, 2]; hp = np.array([x, y, z]) - TCP * hz
rng = np.random.default_rng(0); sols = []
seeds = [c.arm_q()] + [rng.uniform(lim[:, 0], lim[:, 1]) for _ in range(n)]
for sd in seeds:
    s = ik(hp, q, sd)
    if s is not None and not any(np.abs(np.array(s) - np.array(o)).max() < 0.05 for o in sols):
        sols.append(s); print("SOL", np.round(s, 2).tolist(), flush=True)
print("n_sols", len(sols))
EOF
timeout 600 python3 ikseed.py -0.195,-0.046,0.934,0,-45 25 2>&1 | grep -v '^\['

# openrua op 94
python3 - <<'EOF'
s=open('ctl.py').read()
s=s.replace('''def goto_tcp_pitch(c, x, y, z, yaw_deg, pitch_deg, seconds=4.0):''','''def goto_tcp_pitch(c, x, y, z, yaw_deg, pitch_deg, seconds=4.0, seed=None):''')
s=s.replace('''    qj = c.solve_ik(hand_pos, q)
    return c.move(qj, seconds)


if __name__ == "__main__" and sys.argv[1] == "gotop":
    c = Ctl()
    x, y, z, yaw, pitch = map(float, sys.argv[2:7])
    secs = float(sys.argv[7]) if len(sys.argv) > 7 else 4.0
    goto_tcp_pitch(c, x, y, z, yaw, pitch, secs)''','''    qj = c.solve_ik(hand_pos, q, seed=seed)
    print("ik q", np.round(qj, 2).tolist())
    return c.move(qj, seconds)


if __name__ == "__main__" and sys.argv[1] == "gotop":
    # gotop x y z yaw pitch [secs] [seed q1,...,q7]
    c = Ctl()
    x, y, z, yaw, pitch = map(float, sys.argv[2:7])
    secs = float(sys.argv[7]) if len(sys.argv) > 7 else 4.0
    seed = [float(v) for v in sys.argv[8].split(",")] if len(sys.argv) > 8 else None
    goto_tcp_pitch(c, x, y, z, yaw, pitch, secs, seed)''')
open('ctl.py','w').write(s)
EOF
S=0.16,0.59,-0.21,-2.17,2.9,2.74,-2.06
timeout 300 python3 ctl.py grip 0.04
timeout 600 python3 ctl.py gotop -0.252 -0.0485 1.20 0 -45 5 $S; timeout 600 python3 ctl.py gotop -0.252 -0.0485 1.20 0 -45 3 $S; timeout 60 python3 fk.py

# openrua op 95
S=0.16,0.59,-0.21,-2.17,2.9,2.74,-2.06
timeout 600 python3 ctl.py gotop -0.252 -0.0485 1.20 0 -45 3 $S; timeout 600 python3 ctl.py gotop -0.252 -0.0485 1.20 0 -45 3 $S; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | head -7

# openrua op 96
timeout 600 python3 ctl.py gotop -0.252 -0.0485 0.991 0 -45 4; timeout 600 python3 ctl.py gotop -0.252 -0.0485 0.991 0 -45 3; timeout 60 python3 fk.py; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null

# openrua op 97
cat >> ctl.py <<'EOF'


def goto_tcp_frame(c, tcp, f, d, seconds=4.0, seed=None):
    """General pose: f = finger (hand y) axis direction, d = hand pointing (hand z) direction, world frame."""
    f = np.asarray(f, float); d = np.asarray(d, float)
    d = d / np.linalg.norm(d); f = f - d * (f @ d); f = f / np.linalg.norm(f)
    x = np.cross(f, d)
    R = Rot.from_matrix(np.column_stack([x, f, d]))
    q = (R * Rot.from_euler("z", 45, degrees=True)).as_quat()
    hand_pos = np.asarray(tcp, float) - TCP * d
    qj = c.solve_ik(hand_pos, q, seed=seed)
    print("ik q", np.round(qj, 2).tolist())
    return c.move(qj, seconds)


if __name__ == "__main__" and sys.argv[1] == "gotof":
    # gotof x y z fx fy fz dx dy dz [secs] [seed]
    c = Ctl()
    v = list(map(float, sys.argv[2:11]))
    secs = float(sys.argv[11]) if len(sys.argv) > 11 else 4.0
    seed = [float(t) for t in sys.argv[12].split(",")] if len(sys.argv) > 12 else None
    goto_tcp_frame(c, v[0:3], v[3:6], v[6:9], secs, seed)
    rclpy.shutdown()
EOF
sed -i 's/if __name__ == "__main__" and sys.argv\[1\] != "gotop":/if __name__ == "__main__" and sys.argv[1] not in ("gotop", "gotof"):/' ctl.py
F="0.208 -0.978 0"; D="0.691 0.147 -0.707"
timeout 600 python3 ctl.py gotof -0.242 -0.056 0.976 $F $D 3; timeout 600 python3 ctl.py gotof -0.242 -0.056 0.976 $F $D 3; timeout 60 python3 fk.py

# openrua op 98
timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>&1 | head -20; timeout 300 python3 ctl.py js | head -7

# openrua op 99
python3 - <<'EOF'
s=open('ikseed.py').read()
s=s.replace('''x, y, z, yaw, pitch = map(float, sys.argv[1].split(","))
n = int(sys.argv[2]) if len(sys.argv) > 2 else 20
R = Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
q = (R * Rot.from_euler("z", 45, degrees=True)).as_quat(); hz = R.as_matrix()[:, 2]; hp = np.array([x, y, z]) - TCP * hz''','''v = list(map(float, sys.argv[1].split(",")))
n = int(sys.argv[2]) if len(sys.argv) > 2 else 20
if len(v) == 5:
    x, y, z, yaw, pitch = v
    R = Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
else:
    tcp = np.array(v[0:3]); f = np.array(v[3:6]); d = np.array(v[6:9])
    d = d / np.linalg.norm(d); f = f - d * (f @ d); f = f / np.linalg.norm(f)
    R = Rot.from_matrix(np.column_stack([np.cross(f, d), f, d])); x, y, z = tcp
q = (R * Rot.from_euler("z", 45, degrees=True)).as_quat(); hz = R.as_matrix()[:, 2]; hp = np.array([x, y, z]) - TCP * hz''')
open('ikseed.py','w').write(s)
EOF
timeout 900 python3 ikseed.py -0.198,-0.0615,0.970,-0.15,0.7,0.7,0.489,0.716,-0.612 30 2>&1 | grep -v '^\['

# openrua op 100
cat fk.py | head -30

# openrua op 101
cat > fkq.py <<'EOF'
import sys, numpy as np, rclpy
from ctl import Ctl, JOINTS
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
c = Ctl()
cl = c.node.create_client(GetPositionFK, "/compute_fk"); cl.wait_for_service(10)
links = ["panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]
for spec in sys.argv[1:]:
    q = [float(v) for v in spec.split(",")]
    req = GetPositionFK.Request(); req.fk_link_names = links
    s = JointState(); s.name = list(JOINTS); s.position = q; req.robot_state.joint_state = s
    f = cl.call_async(req); rclpy.spin_until_future_complete(c.node, f, timeout_sec=30)
    print(spec)
    for ln, ps in zip(links, f.result().pose_stamped):
        p = ps.pose.position; print(f"   {ln:12s} {p.x:7.3f} {p.y:7.3f} {p.z:7.3f}")
EOF
timeout 120 python3 fkq.py 0.24,0.64,-0.58,-2.18,1.73,1.96,2.35 -0.9,0.66,0.62,-2.19,1.29,2.54,2.38 0.39,0.71,-0.74,-2.18,1.8,1.86,2.36 2>&1 | grep -v '^\['

# openrua op 102
F="0.208 -0.978 0"; D="0.691 0.147 -0.707"
timeout 600 python3 ctl.py gotof -0.242 -0.056 1.20 $F $D 3; timeout 600 python3 ctl.py gotof -0.242 -0.056 1.20 $F $D 3; timeout 60 python3 fk.py
F2="-0.15 0.7 0.7"; D2="0.489 0.716 -0.612"; S2=0.24,0.64,-0.58,-2.18,1.73,1.96,2.35
timeout 600 python3 ctl.py gotof -0.227 -0.1045 1.15 $F2 $D2 5 $S2; timeout 600 python3 ctl.py gotof -0.227 -0.1045 1.15 $F2 $D2 3 $S2; timeout 60 python3 fk.py

# openrua op 103
F2="-0.15 0.7 0.7"; D2="0.489 0.716 -0.612"; S2=0.24,0.64,-0.58,-2.18,1.73,1.96,2.35
for i in 1 2 3; do timeout 600 python3 ctl.py gotof -0.227 -0.1045 1.15 $F2 $D2 4 $S2; done; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | head -7

# openrua op 104
F2="-0.15 0.7 0.7"; D2="0.489 0.716 -0.612"
timeout 600 python3 ctl.py gotof -0.227 -0.1045 1.0067 $F2 $D2 4; timeout 600 python3 ctl.py gotof -0.227 -0.1045 1.0067 $F2 $D2 3; timeout 60 python3 fk.py; timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>&1 | grep -A3 force; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null

# openrua op 105
F2="-0.15 0.7 0.7"; D2="0.489 0.716 -0.612"
timeout 600 python3 ctl.py gotof -0.198 -0.0615 0.970 $F2 $D2 3; timeout 600 python3 ctl.py gotof -0.198 -0.0615 0.970 $F2 $D2 2; timeout 60 python3 fk.py; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png >/dev/null

# openrua op 106
timeout 300 python3 ctl.py grip 0.0; timeout 300 python3 ctl.py js | grep finger; timeout 60 python3 fk.py

# openrua op 107
timeout 300 python3 ctl.py js | grep finger; F2="-0.15 0.7 0.7"; D2="0.489 0.716 -0.612"
timeout 600 python3 ctl.py gotof -0.198 -0.0615 1.00 $F2 $D2 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null

# openrua op 108
F2="-0.15 0.7 0.7"; D2="0.489 0.716 -0.612"
timeout 600 python3 ctl.py gotof -0.198 -0.0615 1.10 $F2 $D2 4; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null

# openrua op 109
timeout 600 python3 ctl.py gotof -0.198 -0.0611 1.15 -0.21 0.98 0 0 0 -1 5; timeout 600 python3 ctl.py gotof -0.198 -0.0611 1.15 -0.21 0.98 0 0 0 -1 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null

# openrua op 110
timeout 600 python3 ctl.py gotof -0.198 -0.0611 1.30 -0.21 0.98 0 0 0 -1 3; timeout 600 python3 ctl.py gotof -0.198 -0.0611 1.30 -0.21 0.98 0 0 0 -1 3; timeout 60 python3 fk.py; timeout 600 python3 ctl.py gotof -0.25 -0.10 1.30 -0.98 -0.21 0 0 0 -1 5; timeout 600 python3 ctl.py gotof -0.25 -0.10 1.30 -0.98 -0.21 0 0 0 -1 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null

# openrua op 111
timeout 120 python3 depth_world.py agentview >/dev/null; python3 - <<'EOF'
import numpy as np
d=np.load('agentview_world.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
m=(xyz[:,2]>0.89)&(xyz[:,2]<1.05)&(xyz[:,0]>-0.34)&(xyz[:,0]<0.05)&(xyz[:,1]>-0.30)&(xyz[:,1]<0.10)
p=xyz[m]; print(len(p), 'x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max())
ys=np.arange(-0.30,0.10,0.01); xs=np.arange(-0.34,0.05,0.01)
ys=ys[(ys>=p[:,1].min()-0.01)&(ys<=p[:,1].max())]; xs=xs[(xs>=p[:,0].min()-0.01)&(xs<=p[:,0].max())]
print('      '+' '.join(f'{y:5.2f}' for y in ys))
for xb in xs:
    row=[]
    for yb in ys:
        s=p[(p[:,0]>=xb)&(p[:,0]<xb+0.01)&(p[:,1]>=yb)&(p[:,1]<yb+0.01)]
        row.append(f'{s[:,2].max():5.3f}' if len(s) else '  .  ')
    print(f'{xb:5.2f} '+' '.join(row))
EOF

# openrua op 112
timeout 900 python3 ikseed.py -0.1914,-0.135,0.9704,-0.707,0,0.707,-0.612,0.5,-0.612 30 2>&1 | grep -v '^\['

# openrua op 113
timeout 120 python3 fkq.py -0.26,1.05,-0.14,-1.16,0.37,1.32,2.12 -0.42,1.05,0.11,-1.17,0.2,1.36,2.21 -0.05,1.12,-0.47,-1.14,0.6,1.24,2.0 2>&1 | grep -v '^\['

# openrua op 114
timeout 300 python3 ctl.py grip 0.04
F3="-0.707 0 0.707"; D3="-0.612 0.5 -0.612"; S3=-0.26,1.05,-0.14,-1.16,0.37,1.32,2.12
for i in 1 2 3; do timeout 600 python3 ctl.py gotof -0.1547 -0.165 1.20 $F3 $D3 5 $S3; done; timeout 60 python3 fk.py

# openrua op 115
timeout 900 python3 ikseed.py -0.1547,-0.165,1.10,-0.707,0,0.707,-0.612,0.5,-0.612 10 2>&1 | grep -v '^\[' | head; echo ---; timeout 900 python3 ikseed.py -0.1547,-0.165,1.0071,-0.707,0,0.707,-0.612,0.5,-0.612 10 2>&1 | grep -v '^\[' | head

# openrua op 116
timeout 300 python3 ikprobe3.py 2>/dev/null | head -1; python3 - <<'EOF'
import subprocess
EOF
timeout 600 python3 - <<'EOF' 2>&1 | grep -v '^\['
import numpy as np
from ctl import Ctl, TCP
from scipy.spatial.transform import Rotation as Rot
c = Ctl()
def pose(tcp, f, d):
    f=np.array(f,float); d=np.array(d,float); d/=np.linalg.norm(d); f=f-d*(f@d); f/=np.linalg.norm(f)
    R=Rot.from_matrix(np.column_stack([np.cross(f,d),f,d])); return np.array(tcp)-TCP*d, (R*Rot.from_euler("z",45,degrees=True)).as_quat()
F=[-0.707,0,0.707]; D=[-0.612,0.5,-0.612]
seed=[-0.17,1.25,-0.67,-0.56,0.83,0.81,2.13]
for tcp in [(-0.1547,-0.165,1.10),(-0.1547,-0.165,1.0071),(-0.1914,-0.135,0.9704),(-0.1914,-0.135,1.00),(-0.1914,-0.135,1.10)]:
    hp,q=pose(tcp,F,D)
    try:
        seed=c.solve_ik(hp,q,seed=seed); print(tcp,"OK",np.round(seed,2).tolist())
    except SystemExit as e: print(tcp,"FAIL",e)
EOF

# openrua op 117
F3="-0.707 0 0.707"; D3="-0.612 0.5 -0.612"; S3=-0.17,1.25,-0.67,-0.56,0.83,0.81,2.13
for i in 1 2 3; do timeout 600 python3 ctl.py gotof -0.1547 -0.165 1.10 $F3 $D3 5 $S3; done; timeout 60 python3 fk.py
timeout 600 python3 ctl.py gotof -0.1547 -0.165 1.0071 $F3 $D3 3; timeout 600 python3 ctl.py gotof -0.1547 -0.165 1.0071 $F3 $D3 3; timeout 60 python3 fk.py; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null

# openrua op 118
F3="-0.707 0 0.707"; D3="-0.612 0.5 -0.612"
timeout 600 python3 ctl.py gotof -0.1914 -0.135 0.9704 $F3 $D3 3; timeout 600 python3 ctl.py gotof -0.1914 -0.135 0.9704 $F3 $D3 2; timeout 60 python3 fk.py; timeout 300 python3 ctl.py grip 0.0; timeout 300 python3 ctl.py js | grep finger

# openrua op 119
F3="-0.707 0 0.707"; D3="-0.612 0.5 -0.612"
timeout 600 python3 ctl.py gotof -0.1914 -0.135 1.03 $F3 $D3 4; timeout 300 python3 ctl.py js | grep finger; timeout 600 python3 ctl.py gotof -0.1914 -0.135 1.12 $F3 $D3 5; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null

# openrua op 120
timeout 600 python3 ctl.py gotof -0.1914 -0.135 1.20 -1 0 0 0 0 -1 7; timeout 600 python3 ctl.py gotof -0.1914 -0.135 1.20 -1 0 0 0 0 -1 4; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null

# openrua op 121
timeout 600 python3 ctl.py gotof -0.1914 -0.135 1.20 -1 0 0 0 -0.643 -0.766 7; timeout 600 python3 ctl.py gotof -0.1914 -0.135 1.20 -1 0 0 0 -0.643 -0.766 4; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null

# openrua op 122
timeout 120 python3 depth_world.py agentview >/dev/null; timeout 120 python3 depth_world.py frontview >/dev/null; python3 - <<'EOF'
import numpy as np
for cam in ['agentview','frontview']:
    d=np.load(f'{cam}_world.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
    m=(xyz[:,2]>1.08)&(xyz[:,2]<1.23)&(xyz[:,0]>-0.30)&(xyz[:,0]<-0.05)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.02)
    p=xyz[m]; print(cam,len(p))
    for zb in np.arange(1.08,1.23,0.01):
        s=p[(p[:,2]>=zb)&(p[:,2]<zb+0.01)]
        if len(s): print(f"  z {zb:.2f} n{len(s):4d} x {s[:,0].min():.3f}..{s[:,0].max():.3f} (mid {(s[:,0].min()+s[:,0].max())/2:.3f})  y {s[:,1].min():.3f}..{s[:,1].max():.3f} (mid {(s[:,1].min()+s[:,1].max())/2:.3f})")
EOF

# openrua op 123
D4="0 -0.643 -0.766"
timeout 600 python3 ctl.py gotof -0.1918 -0.1347 1.30 -1 0 0 $D4 5; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 600 python3 ctl.py gotof -0.34 -0.14 1.30 -1 0 0 $D4 6; timeout 600 python3 ctl.py gotof -0.488 -0.1475 1.30 -1 0 0 $D4 6; timeout 600 python3 ctl.py gotof -0.488 -0.1475 1.30 -1 0 0 $D4 4; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null

# openrua op 124
D4="0 -0.643 -0.766"
timeout 600 python3 ctl.py gotof -0.488 -0.1475 1.17 -1 0 0 $D4 5; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 120 python3 depth_world.py frontview >/dev/null; timeout 120 python3 depth_world.py birdview >/dev/null; python3 - <<'EOF'
import numpy as np
for cam in ['frontview','birdview']:
    d=np.load(f'{cam}_world.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
    m=(xyz[:,2]>1.05)&(xyz[:,2]<1.20)&(xyz[:,0]>-0.60)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.05)
    p=xyz[m]; print(cam,len(p))
    for zb in np.arange(1.05,1.20,0.01):
        s=p[(p[:,2]>=zb)&(p[:,2]<zb+0.01)]
        if len(s): print(f"  z {zb:.2f} n{len(s):4d} x {s[:,0].min():.3f}..{s[:,0].max():.3f} (mid {(s[:,0].min()+s[:,0].max())/2:.3f})  y {s[:,1].min():.3f}..{s[:,1].max():.3f} (mid {(s[:,1].min()+s[:,1].max())/2:.3f})")
EOF

# openrua op 125
timeout 600 python3 ctl.py gotof -0.488 -0.1475 1.17 -0.978 0 0.208 -0.159 -0.643 -0.749 6; timeout 600 python3 ctl.py gotof -0.488 -0.1475 1.17 -0.978 0 0.208 -0.159 -0.643 -0.749 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 120 python3 depth_world.py frontview >/dev/null; timeout 120 python3 depth_world.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null; python3 - <<'EOF'
import numpy as np
for cam in ['frontview','birdview']:
    d=np.load(f'{cam}_world.npz'); xyz=d['xyz'].reshape(-1,3); xyz=xyz[np.isfinite(xyz).all(1)]
    m=(xyz[:,2]>1.05)&(xyz[:,2]<1.20)&(xyz[:,0]>-0.60)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.05)
    p=xyz[m]; print(cam,len(p))
    for zb in np.arange(1.05,1.20,0.01):
        s=p[(p[:,2]>=zb)&(p[:,2]<zb+0.01)]
        if len(s): print(f"  z {zb:.2f} n{len(s):4d} x {s[:,0].min():.3f}..{s[:,0].max():.3f} (mid {(s[:,0].min()+s[:,0].max())/2:.3f})  y {s[:,1].min():.3f}..{s[:,1].max():.3f} (mid {(s[:,1].min()+s[:,1].max())/2:.3f})")
EOF

# openrua op 126
F5="-0.978 0 0.208"; D5="-0.159 -0.643 -0.749"
timeout 600 python3 ctl.py gotof -0.4876 -0.1477 1.30 $F5 $D5 5; timeout 600 python3 ctl.py gotof -0.33 -0.153 1.30 $F5 $D5 6; timeout 600 python3 ctl.py gotof -0.33 -0.153 1.30 $F5 $D5 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger

# openrua op 127
F5="-0.978 0 0.208"; D5="-0.159 -0.643 -0.749"
timeout 600 python3 ctl.py gotof -0.33 -0.153 1.00 $F5 $D5 5; timeout 600 python3 ctl.py gotof -0.33 -0.153 1.00 $F5 $D5 3; timeout 60 python3 fk.py; timeout 600 python3 ctl.py gotof -0.33 -0.153 0.985 $F5 $D5 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py grip 0.04

# openrua op 128
timeout 120 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null; timeout 120 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null; timeout 120 python3 depth_world.py birdview >/dev/null; timeout 120 python3 depth_world.py agentview > /dev/null
python3 - <<'EOF'
import numpy as np
for cam in ["birdview","agentview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
    m=(xyz[:,2]>0.89)&(xyz[:,0]>-0.34)&(xyz[:,0]<0.1)&(xyz[:,1]>-0.4)&(xyz[:,1]<0.05)
    p=xyz[m]
    print(cam,len(p))
    if len(p):
        print(" x",p[:,0].min().round(3),p[:,0].max().round(3)," y",p[:,1].min().round(3),p[:,1].max().round(3)," z max",p[:,2].max().round(3))
        # top height grid per 1cm in x,y
        for zlo,zhi in [(0.89,0.93),(0.93,0.97),(0.97,1.0)]:
            q=p[(p[:,2]>zlo)&(p[:,2]<zhi)]
            if len(q): print(f"  z{zlo}-{zhi}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
EOF

# openrua op 129
timeout 600 python3 ctl.py goto -0.25 -0.30 1.30 0 5; timeout 600 python3 ctl.py goto -0.25 -0.30 1.30 0 3; timeout 60 python3 fk.py
timeout 120 python3 depth_world.py birdview >/dev/null; timeout 120 python3 depth_world.py agentview > /dev/null; timeout 120 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null
python3 - <<'EOF'
import numpy as np
for cam in ["birdview","agentview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
    m=(xyz[:,2]>0.89)&(xyz[:,0]>-0.34)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.3)&(xyz[:,1]<0.05)
    p=xyz[m]
    print(cam,len(p), "z max", p[:,2].max().round(3))
    for zlo,zhi in [(0.89,0.93),(0.93,0.97),(0.97,1.0),(0.96,1.0)]:
        q=p[(p[:,2]>zlo)&(p[:,2]<zhi)]
        if len(q): print(f"  z{zlo}-{zhi}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)} cx {(q[:,0].min()+q[:,0].max())/2:.3f} cy {(q[:,1].min()+q[:,1].max())/2:.3f}")
EOF

# openrua op 130
python3 - <<'EOF'
import numpy as np
for cam in ["birdview","agentview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
    m=(xyz[:,2]>0.885)&(xyz[:,0]>-0.335)&(xyz[:,0]<-0.15)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.07)
    p=xyz[m]
    print(cam,len(p), "z max", p[:,2].max().round(3))
    for zlo,zhi in [(0.885,0.92),(0.92,0.95),(0.95,0.975),(0.975,1.0),(1.0,1.1)]:
        q=p[(p[:,2]>zlo)&(p[:,2]<zhi)]
        if len(q): print(f"  z{zlo}-{zhi}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)} cx {(q[:,0].min()+q[:,0].max())/2:.3f} cy {(q[:,1].min()+q[:,1].max())/2:.3f}")
    # x-bins of max z
    print("  x-bins max z:", [(round(x,2), round(p[(p[:,0]>=x)&(p[:,0]<x+0.01),2].max(),3)) for x in np.arange(-0.335,-0.15,0.01) if ((p[:,0]>=x)&(p[:,0]<x+0.01)).any()])
    print("  y-bins max z:", [(round(y,2), round(p[(p[:,1]>=y)&(p[:,1]<y+0.01),2].max(),3)) for y in np.arange(-0.25,-0.07,0.01) if ((p[:,1]>=y)&(p[:,1]<y+0.01)).any()])
EOF

# openrua op 131
timeout 600 python3 ctl.py goto -0.278 -0.1865 1.15 0 5; timeout 600 python3 ctl.py goto -0.278 -0.1865 1.15 0 3; timeout 60 python3 fk.py

# openrua op 132
timeout 600 python3 ctl.py goto -0.278 -0.1865 1.02 0 4; timeout 600 python3 ctl.py goto -0.278 -0.1865 0.985 0 3; timeout 600 python3 ctl.py goto -0.278 -0.1865 0.985 0 2; timeout 60 python3 fk.py; timeout 120 ./tools/perception/cam_snap.py robot0_eye_in_hand eih.png >/dev/null

# openrua op 133
timeout 300 python3 ctl.py grip 0.0; timeout 300 python3 ctl.py js | grep finger

# openrua op 134
timeout 600 python3 ctl.py goto -0.278 -0.1865 1.05 0 4; timeout 600 python3 ctl.py goto -0.278 -0.1865 1.20 0 5; timeout 600 python3 ctl.py goto -0.278 -0.1865 1.20 0 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 120 python3 depth_world.py frontview >/dev/null; timeout 120 python3 depth_world.py agentview >/dev/null; timeout 120 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null
python3 - <<'EOF'
import numpy as np
for cam in ["frontview","agentview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
    m=(xyz[:,2]>1.0)&(xyz[:,2]<1.25)&(xyz[:,0]>-0.40)&(xyz[:,0]<-0.15)&(xyz[:,1]>-0.30)&(xyz[:,1]<-0.05)
    p=xyz[m]
    print(cam,len(p))
    for zlo in np.arange(1.08,1.22,0.02):
        q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.02)]
        if len(q): print(f"  z{zlo:.2f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)} cx {(q[:,0].min()+q[:,0].max())/2:.3f} cy {(q[:,1].min()+q[:,1].max())/2:.3f}")
    print("  zmin", p[:,2].min().round(3))
EOF

# openrua op 135
timeout 600 python3 ctl.py gotof -0.278 -0.1865 1.20 0 -1 0 -0.342 0 -0.94 6; timeout 600 python3 ctl.py gotof -0.278 -0.1865 1.20 0 -1 0 -0.342 0 -0.94 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 120 python3 depth_world.py frontview >/dev/null; timeout 120 python3 depth_world.py agentview >/dev/null; timeout 120 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null
python3 - <<'EOF'
import numpy as np
for cam in ["frontview","agentview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
    m=(xyz[:,2]>1.0)&(xyz[:,2]<1.25)&(xyz[:,0]>-0.40)&(xyz[:,0]<-0.15)&(xyz[:,1]>-0.30)&(xyz[:,1]<-0.05)
    p=xyz[m]
    print(cam,len(p))
    for zlo in np.arange(1.08,1.22,0.02):
        q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.02)]
        if len(q): print(f"  z{zlo:.2f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)} cx {(q[:,0].min()+q[:,0].max())/2:.3f} cy {(q[:,1].min()+q[:,1].max())/2:.3f}")
    print("  zmin", p[:,2].min().round(3))
EOF

# openrua op 136
timeout 600 python3 ctl.py gotof -0.278 -0.1865 1.20 0 -1 0 -0.53 0 -0.848 5; timeout 600 python3 ctl.py gotof -0.278 -0.1865 1.20 0 -1 0 -0.53 0 -0.848 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 120 python3 depth_world.py frontview >/dev/null; timeout 120 python3 depth_world.py agentview >/dev/null; timeout 120 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null
python3 - <<'EOF'
import numpy as np
for cam in ["frontview","agentview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
    m=(xyz[:,2]>1.0)&(xyz[:,2]<1.25)&(xyz[:,0]>-0.40)&(xyz[:,0]<-0.15)&(xyz[:,1]>-0.30)&(xyz[:,1]<-0.05)
    p=xyz[m]
    print(cam,len(p))
    for zlo in np.arange(1.08,1.22,0.02):
        q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.02)]
        if len(q): print(f"  z{zlo:.2f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)} cx {(q[:,0].min()+q[:,0].max())/2:.3f} cy {(q[:,1].min()+q[:,1].max())/2:.3f}")
    print("  zmin", p[:,2].min().round(3))
EOF

# openrua op 137
python3 - <<'EOF'
import numpy as np
xyz=np.load("frontview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>1.06)&(xyz[:,2]<1.30)&(xyz[:,0]>-0.40)&(xyz[:,0]<-0.15)&(xyz[:,1]>-0.30)&(xyz[:,1]<-0.05)
p=xyz[m]
xs=np.arange(-0.40,-0.15,0.005); zs=np.arange(1.30,1.06,-0.005)
for z in zs:
    row=""
    for x in xs:
        q=p[(p[:,0]>=x)&(p[:,0]<x+0.005)&(p[:,2]>=z-0.005)&(p[:,2]<z)]
        row+= "#" if len(q)>2 else ("." if len(q) else " ")
    print(f"{z:.3f} {row}")
print("      x from -0.40 to -0.15, 5mm cells")
EOF

# openrua op 138
python3 -c "
from PIL import Image
im=Image.open('frontview.png').crop((170,150,370,300)).resize((800,600),Image.LANCZOS); im.save('crop.png')
im=Image.open('agentview.png')
" ; timeout 120 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null; python3 -c "
from PIL import Image
im=Image.open('agentview.png').crop((150,100,400,320)).resize((800,704),Image.LANCZOS); im.save('crop2.png')"

# openrua op 139
timeout 120 python3 depth_world.py birdview >/dev/null
python3 - <<'EOF'
import numpy as np
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>1.05)&(xyz[:,2]<1.25)&(xyz[:,0]>-0.40)&(xyz[:,0]<-0.15)&(xyz[:,1]>-0.30)&(xyz[:,1]<-0.05)
p=xyz[m]
print(len(p))
for zlo in np.arange(1.08,1.24,0.02):
    q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.02)]
    if len(q): print(f"  z{zlo:.2f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)} cx {(q[:,0].min()+q[:,0].max())/2:.3f} cy {(q[:,1].min()+q[:,1].max())/2:.3f} mean {q[:,0].mean():.3f},{q[:,1].mean():.3f}")
# ASCII top-down map of z
xs=np.arange(-0.36,-0.18,0.005); ys=np.arange(-0.22,-0.07,0.005)
for y in ys[::-1]:
    row=""
    for x in xs:
        q=p[(p[:,0]>=x)&(p[:,0]<x+0.005)&(p[:,1]>=y)&(p[:,1]<y+0.005)]
        if len(q)==0: row+=" "
        else:
            z=q[:,2].max()
            row+= "R" if z>1.17 else ("b" if z<1.13 else "m")
    print(f"{y:.3f} {row}")
print("x -0.36..-0.18")
EOF

# openrua op 140
D="-0.53 0 -0.848"
timeout 600 python3 ctl.py gotof -0.419 -0.186 1.22 0 -1 0 $D 6; timeout 600 python3 ctl.py gotof -0.419 -0.186 1.22 0 -1 0 $D 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 120 python3 depth_world.py birdview >/dev/null
python3 - <<'EOF'
import numpy as np
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>1.07)&(xyz[:,2]<1.26)&(xyz[:,0]>-0.52)&(xyz[:,0]<-0.33)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.05)
p=xyz[m]
xs=np.arange(-0.52,-0.33,0.005); ys=np.arange(-0.25,-0.05,0.005)
for y in ys[::-1]:
    row=""
    for x in xs:
        q=p[(p[:,0]>=x)&(p[:,0]<x+0.005)&(p[:,1]>=y)&(p[:,1]<y+0.005)]
        if len(q)==0: row+=" "
        else:
            z=q[:,2].max()
            row+= "R" if z>1.19 else ("b" if z<1.15 else "m")
    print(f"{y:.3f} {row}")
print("x -0.52..-0.33 (5mm)")
b=p[p[:,2]<1.15]; print("inner base x",b[:,0].min().round(3),b[:,0].max().round(3),"y",b[:,1].min().round(3),b[:,1].max().round(3))
EOF

# openrua op 141
timeout 120 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null; timeout 120 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null
python3 -c "
from PIL import Image
Image.open('frontview.png').crop((200,130,400,280)).resize((800,600),Image.LANCZOS).save('crop.png')
Image.open('agentview.png').crop((130,80,380,300)).resize((800,704),Image.LANCZOS).save('crop2.png')"

# openrua op 142
timeout 120 python3 depth_world.py frontview >/dev/null
python3 - <<'EOF'
import numpy as np
xyz=np.load("frontview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>1.06)&(xyz[:,2]<1.26)&(xyz[:,0]>-0.55)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.05)
p=xyz[m]
for zlo in np.arange(1.10,1.24,0.02):
    q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.02)]
    if len(q): print(f"  z{zlo:.2f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
EOF

# openrua op 143
D="-0.53 0 -0.848"
timeout 600 python3 ctl.py gotof -0.422 -0.186 1.14 0 -1 0 $D 5; timeout 600 python3 ctl.py gotof -0.422 -0.186 1.14 0 -1 0 $D 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 120 python3 depth_world.py frontview >/dev/null
python3 - <<'EOF'
import numpy as np
xyz=np.load("frontview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>1.0)&(xyz[:,2]<1.20)&(xyz[:,0]>-0.55)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.05)
p=xyz[m]
for zlo in np.arange(1.02,1.18,0.02):
    q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.02)]
    if len(q): print(f"  z{zlo:.2f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
EOF

# openrua op 144
D="-0.53 0 -0.848"
timeout 600 python3 ctl.py gotof -0.422 -0.186 1.10 0 -1 0 $D 4; timeout 600 python3 ctl.py gotof -0.422 -0.186 1.10 0 -1 0 $D 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 600 python3 ctl.py gotof -0.422 -0.186 1.093 0 -1 0 $D 3; timeout 600 python3 ctl.py gotof -0.422 -0.186 1.093 0 -1 0 $D 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger

# openrua op 145
timeout 120 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null; timeout 120 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null
python3 -c "
from PIL import Image
Image.open('frontview.png').crop((200,150,400,300)).resize((800,600),Image.LANCZOS).save('crop.png')
Image.open('agentview.png').crop((130,80,380,300)).resize((800,704),Image.LANCZOS).save('crop2.png')"

# openrua op 146
D="-0.53 0 -0.848"
timeout 600 python3 ctl.py gotof -0.412 -0.196 1.16 0 -1 0 $D 5; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 120 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null; python3 -c "
from PIL import Image
Image.open('frontview.png').crop((200,130,400,280)).resize((800,600),Image.LANCZOS).save('crop.png')"

# openrua op 147
timeout 600 python3 ctl.py gotof -0.412 -0.196 1.16 0 -1 0 0 0 -1 6; timeout 600 python3 ctl.py gotof -0.412 -0.196 1.16 0 -1 0 0 0 -1 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 120 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null; python3 -c "
from PIL import Image
Image.open('frontview.png').crop((200,130,400,280)).resize((800,600),Image.LANCZOS).save('crop.png')"
timeout 120 python3 depth_world.py frontview >/dev/null
python3 - <<'EOF'
import numpy as np
xyz=np.load("frontview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>1.0)&(xyz[:,2]<1.20)&(xyz[:,0]>-0.55)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.05)
p=xyz[m]
for zlo in np.arange(1.04,1.18,0.02):
    q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.02)]
    if len(q): print(f"  z{zlo:.2f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
EOF

# openrua op 148
timeout 120 python3 depth_world.py frontview >/dev/null && timeout 120 python3 depth_world.py birdview >/dev/null
python3 - <<'EOF'
import numpy as np
xyz=np.load("frontview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>1.0)&(xyz[:,2]<1.20)&(xyz[:,0]>-0.55)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.05)
p=xyz[m]
print("frontview")
for zlo in np.arange(1.04,1.18,0.02):
    q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.02)]
    if len(q): print(f"  z{zlo:.2f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>1.0)&(xyz[:,2]<1.22)&(xyz[:,0]>-0.52)&(xyz[:,0]<-0.33)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.05)
p=xyz[m]
print("birdview")
for zlo in np.arange(1.04,1.20,0.02):
    q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.02)]
    if len(q): print(f"  z{zlo:.2f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
xs=np.arange(-0.52,-0.33,0.005); ys=np.arange(-0.25,-0.05,0.005)
for y in ys[::-1]:
    row=""
    for x in xs:
        q=p[(p[:,0]>=x)&(p[:,0]<x+0.005)&(p[:,1]>=y)&(p[:,1]<y+0.005)]
        if len(q)==0: row+=" "
        else:
            z=q[:,2].max()
            row+= "R" if z>1.14 else ("b" if z<1.08 else "m")
    print(f"{y:.3f} {row}")
print("x -0.52..-0.33 (5mm); R>1.14 m b<1.08")
EOF

# openrua op 149
cat > /workspace/hang.py <<'EOF'
import numpy as np, subprocess, sys
# measure hanging cup from frontview: base ring (x range at z in [tcp_z-0.09, tcp_z-0.05]) and rim (x-min near tcp_z-0.02..tcp_z)
tcp=np.array(list(map(float,sys.argv[1:4])))
subprocess.run(["timeout","120","python3","depth_world.py","frontview"],stdout=subprocess.DEVNULL,check=True)
xyz=np.load("frontview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>tcp[2]-0.12)&(xyz[:,2]<tcp[2]+0.03)&(xyz[:,0]>tcp[0]-0.13)&(xyz[:,0]<tcp[0]+0.13)&(xyz[:,1]>tcp[1]-0.06)&(xyz[:,1]<tcp[1]+0.15)
p=xyz[m]
for zlo in np.arange(tcp[2]-0.12,tcp[2]+0.03,0.01):
    q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.01)]
    if len(q): print(f"  z{zlo:.3f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} (w {q[:,0].max()-q[:,0].min():.3f} c {(q[:,0].min()+q[:,0].max())/2:.3f}) y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
EOF
timeout 600 python3 ctl.py gotof -0.412 -0.196 1.16 0 -1 0 -0.309 0 -0.951 6; timeout 600 python3 ctl.py gotof -0.412 -0.196 1.16 0 -1 0 -0.309 0 -0.951 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
python3 hang.py -0.412 -0.196 1.16

# openrua op 150
sed -i 's/check=True)/check=False)\nfor _ in range(3):\n    if subprocess.run(["timeout","120","python3","depth_world.py","frontview"],stdout=subprocess.DEVNULL).returncode==0: break/' hang.py; python3 hang.py -0.412 -0.196 1.16

# openrua op 151
timeout 600 python3 ctl.py gotof -0.30 -0.196 1.16 0 -1 0 -0.309 0 -0.951 6; timeout 600 python3 ctl.py gotof -0.30 -0.196 1.16 0 -1 0 -0.309 0 -0.951 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
python3 hang.py -0.30 -0.196 1.16

# openrua op 152
timeout 120 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null; timeout 120 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null
python3 -c "
from PIL import Image
Image.open('frontview.png').crop((150,130,350,280)).resize((800,600),Image.LANCZOS).save('crop.png')
Image.open('agentview.png').crop((130,60,380,280)).resize((800,704),Image.LANCZOS).save('crop2.png')"

# openrua op 153
cat > /workspace/cuptilt.py <<'EOF'
import numpy as np, subprocess, sys
from PIL import Image
cam=sys.argv[1] if len(sys.argv)>1 else "frontview"
subprocess.run(["timeout","120","./tools/perception/cam_snap.py",cam,f"{cam}.png"],stdout=subprocess.DEVNULL)
im=np.asarray(Image.open(f"{cam}.png").convert("RGB")).astype(int)
r,g,b=im[...,0],im[...,1],im[...,2]
yellow=(r>120)&(g>100)&(b<90)&(r-b>60)
rows=np.where(yellow.any(axis=1))[0]
print("yellow rows",rows.min(),rows.max())
for y in range(rows.min(),rows.max()+1,4):
    xs=np.where(yellow[y])[0]
    if len(xs): print(y, xs.min(), xs.max(), "w",xs.max()-xs.min(),"c",(xs.min()+xs.max())/2)
EOF
python3 cuptilt.py frontview

# openrua op 154
python3 - <<'EOF'
import numpy as np
from PIL import Image
im=np.asarray(Image.open("frontview.png").convert("RGB")).astype(int)
patch=im[235:260,240:275].reshape(-1,3)
print(patch.mean(0), patch.min(0), patch.max(0))
# hsv-ish: yellow hue: r>g>b, r-b large
r,g,b=im[...,0],im[...,1],im[...,2]
mask=(r-b>40)&(g-b>25)&(r>80)&(r<230)
mask[:150]=False
ys=np.where(mask.any(1))[0]; print(ys.min(),ys.max())
for y in range(ys.min(),ys.max()+1,3):
    xs=np.where(mask[y])[0]
    if len(xs)>3: print(y,xs.min(),xs.max(),"w",xs.max()-xs.min(),"c",(xs.min()+xs.max())/2, "n",len(xs))
EOF

# openrua op 155
cat > /workspace/bird.py <<'EOF'
import numpy as np, subprocess, sys
from scipy.optimize import least_squares
tcp=np.array(list(map(float,sys.argv[1:4])))
for _ in range(3):
    if subprocess.run(["timeout","120","python3","depth_world.py","birdview"],stdout=subprocess.DEVNULL).returncode==0: break
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,0]>tcp[0]-0.09)&(xyz[:,0]<tcp[0]+0.09)&(xyz[:,1]>tcp[1]-0.03)&(xyz[:,1]<tcp[1]+0.14)
p=xyz[m]
def fit(q,r0,label):
    if len(q)<8: print(label,"too few",len(q)); return None
    f=lambda c: np.hypot(q[:,0]-c[0],q[:,1]-c[1])-r0
    c=least_squares(f,[q[:,0].mean(),q[:,1].mean()]).x
    print(f"{label}: n={len(q)} center {c[0]:.4f},{c[1]:.4f} resid {np.abs(f(c)).mean():.4f}  xrange {q[:,0].min():.3f}..{q[:,0].max():.3f} yrange {q[:,1].min():.3f}..{q[:,1].max():.3f}")
    return c
for zlo in np.arange(tcp[2]-0.12,tcp[2]+0.03,0.01):
    q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.01)]
    if len(q): print(f"  z{zlo:.3f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
rim=p[(p[:,2]>tcp[2]-0.005)&(p[:,2]<tcp[2]+0.02)]
base=p[(p[:,2]>tcp[2]-0.105)&(p[:,2]<tcp[2]-0.08)]
cr=fit(rim,0.052,"rim(r=.052)"); cb=fit(base,0.037,"innerbase(r=.037)")
if cr is not None and cb is not None: print("base - rim offset:", np.round(cb-cr,4))
EOF
python3 bird.py -0.3007 -0.196 1.1595

# openrua op 156
timeout 600 python3 ctl.py gotof -0.30 -0.196 1.16 0 -1 0 0 0 -1 6; timeout 600 python3 ctl.py gotof -0.30 -0.196 1.16 0 -1 0 0 0 -1 3; timeout 60 python3 fk.py
timeout 600 python3 ctl.py goto -0.30 -0.196 1.00 0 5; timeout 600 python3 ctl.py goto -0.30 -0.196 1.00 0 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 600 python3 ctl.py goto -0.30 -0.196 0.988 0 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger

# openrua op 157
timeout 300 python3 ctl.py grip 0.04; timeout 600 python3 ctl.py goto -0.29 -0.20 1.15 0 4; timeout 600 python3 ctl.py goto -0.29 -0.20 1.15 0 3; timeout 60 python3 fk.py
timeout 120 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null
for i in 1 2 3; do timeout 120 python3 depth_world.py birdview >/dev/null && break; done
python3 - <<'EOF'
import numpy as np
from scipy.optimize import least_squares
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>0.95)&(xyz[:,2]<1.02)&(xyz[:,0]>-0.34)&(xyz[:,0]<-0.15)&(xyz[:,1]>-0.30)&(xyz[:,1]<-0.05)
p=xyz[m]
print(len(p),"z",p[:,2].min().round(3),p[:,2].max().round(3))
for zlo in np.arange(0.95,1.02,0.01):
    q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.01)]
    if len(q): print(f"  z{zlo:.3f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
q=p[p[:,2]>0.985]
f=lambda c: np.hypot(q[:,0]-c[0],q[:,1]-c[1])-0.052
c=least_squares(f,[q[:,0].mean(),q[:,1].mean()]).x
print("rim fit center",c.round(4),"resid",np.abs(f(c)).mean().round(4))
EOF

# openrua op 158
timeout 600 python3 ctl.py goto -0.283 -0.2175 1.10 0 4; timeout 600 python3 ctl.py goto -0.283 -0.2175 1.02 0 3; timeout 600 python3 ctl.py goto -0.283 -0.2175 0.987 0 3; timeout 600 python3 ctl.py goto -0.283 -0.2175 0.987 0 2; timeout 60 python3 fk.py; timeout 120 ./tools/perception/cam_snap.py robot0_eye_in_hand eih.png >/dev/null

# openrua op 159
timeout 300 python3 ctl.py grip 0.0; timeout 600 python3 ctl.py goto -0.283 -0.2175 1.05 0 4; timeout 600 python3 ctl.py goto -0.283 -0.2175 1.22 0 5; timeout 600 python3 ctl.py goto -0.283 -0.2175 1.22 0 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 120 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null; timeout 120 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null
python3 -c "
from PIL import Image
Image.open('frontview.png').crop((150,100,350,250)).resize((800,600),Image.LANCZOS).save('crop.png')
Image.open('agentview.png').crop((100,40,350,260)).resize((800,704),Image.LANCZOS).save('crop2.png')"

# openrua op 160
for i in 1 2 3; do timeout 120 python3 depth_world.py birdview >/dev/null && break; done
python3 - <<'EOF'
import numpy as np
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
tcp=np.array([-0.2832,-0.2174,1.2192]); rc=np.array([tcp[0], tcp[1]+0.0565])
m=(xyz[:,2]>1.05)&(xyz[:,2]<1.25)&(xyz[:,0]>-0.40)&(xyz[:,0]<-0.15)&(xyz[:,1]>-0.30)&(xyz[:,1]<-0.05)
p=xyz[m]
d=np.hypot(p[:,0]-rc[0],p[:,1]-rc[1])
print("rim-ish points",(d<0.058).sum(),"outside",(d>0.06).sum())
q=p[d>0.06]
ang=np.degrees(np.arctan2(q[:,1]-rc[1],q[:,0]-rc[0]))
for a in range(-180,180,30):
    s=q[(ang>=a)&(ang<a+30)]
    if len(s): print(f"angle {a}..{a+30}: n={len(s)} dist {d[d>0.06][(ang>=a)&(ang<a+30)].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}")
xs=np.arange(-0.38,-0.17,0.005); ys=np.arange(-0.30,-0.05,0.005)
for y in ys[::-1]:
    row=""
    for x in xs:
        s=p[(p[:,0]>=x)&(p[:,0]<x+0.005)&(p[:,1]>=y)&(p[:,1]<y+0.005)]
        row+= " " if len(s)==0 else ("R" if s[:,2].max()>1.20 else ("m" if s[:,2].max()>1.13 else "b"))
    print(f"{y:.3f} {row}")
EOF

# openrua op 161
timeout 600 python3 ctl.py goto -0.424 -0.204 1.22 0 6; timeout 600 python3 ctl.py goto -0.424 -0.204 1.22 0 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 120 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null; timeout 120 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null
python3 -c "
from PIL import Image
Image.open('frontview.png').crop((200,100,400,250)).resize((800,600),Image.LANCZOS).save('crop.png')
Image.open('agentview.png').crop((100,40,350,260)).resize((800,704),Image.LANCZOS).save('crop2.png')"

# openrua op 162
python3 - <<'EOF'
import numpy as np
xyz=np.load("frontview_world.npz")["xyz"]
H,W,_=xyz.shape
for name,sl in [("left cols",slice(0,W//3)),("right cols",slice(2*W//3,W)),("top rows",None),("bottom rows",None)]:
    pass
tbl=(np.abs(xyz[...,2]-0.88)<0.01)
L=xyz[:, :W//3][tbl[:, :W//3]]; R=xyz[:, 2*W//3:][tbl[:, 2*W//3:]]
T=xyz[:H//2][tbl[:H//2]]; B=xyz[H//2:][tbl[H//2:]]
print("table pts: left cols mean x,y",L[:,:2].mean(0).round(3)," right cols",R[:,:2].mean(0).round(3))
print("top rows mean x,y",T[:,:2].mean(0).round(3)," bottom rows",B[:,:2].mean(0).round(3))
EOF

# openrua op 163
python3 - <<'EOF'
import numpy as np
for cam in ["agentview","birdview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"]
    H,W,_=xyz.shape
    tbl=(np.abs(xyz[...,2]-0.88)<0.01)
    L=xyz[:, :W//3][tbl[:, :W//3]]; R=xyz[:, 2*W//3:][tbl[:, 2*W//3:]]
    T=xyz[:H//2][tbl[:H//2]]; B=xyz[H//2:][tbl[H//2:]]
    print(cam,"left cols x,y",L[:,:2].mean(0).round(3)," right cols",R[:,:2].mean(0).round(3)," top rows",T[:,:2].mean(0).round(3)," bottom rows",B[:,:2].mean(0).round(3))
EOF

# openrua op 164
cat > /workspace/xtilt.py <<'EOF'
import numpy as np, subprocess, sys
# x-tilt of hanging cup from a +x camera: +x face x-position vs z at y = yc + dy
tcp=np.array(list(map(float,sys.argv[1:4]))); cam=sys.argv[4] if len(sys.argv)>4 else "frontview"
yc=tcp[1]+0.054
for _ in range(3):
    if subprocess.run(["timeout","120","python3","depth_world.py",cam],stdout=subprocess.DEVNULL).returncode==0: break
xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
for dy in (-0.03,-0.02,0.0):
    m=(np.abs(xyz[:,1]-(yc+dy))<0.004)&(xyz[:,0]>tcp[0]-0.02)&(xyz[:,0]<tcp[0]+0.12)&(xyz[:,2]>tcp[2]-0.13)&(xyz[:,2]<tcp[2]+0.02)
    p=xyz[m]
    print(f"dy={dy}: expected offset for vertical cup: rim {np.sqrt(0.054**2-dy**2):.3f} base {np.sqrt(0.042**2-dy**2) if abs(dy)<0.042 else 0:.3f}")
    for zlo in np.arange(tcp[2]-0.13,tcp[2]+0.02,0.01):
        q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.01)]
        if len(q)>=3: print(f"   z{zlo:.3f}: xmax {q[:,0].max():.3f}  (xmax-tcpx {q[:,0].max()-tcp[0]:+.3f}) n={len(q)}")
EOF
python3 xtilt.py -0.4239 -0.2043 1.2196 frontview

# openrua op 165
timeout 600 python3 ctl.py goto -0.408 -0.204 1.14 0 5; timeout 600 python3 ctl.py goto -0.408 -0.204 1.14 0 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger

# openrua op 166
timeout 600 python3 ctl.py goto -0.408 -0.204 1.115 0 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
python3 xtilt.py -0.4084 -0.204 1.115 frontview 2>&1 | grep -A20 "dy=-0.02" | head -14

# openrua op 167
timeout 600 python3 ctl.py goto -0.408 -0.204 1.10 0 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger
timeout 600 python3 ctl.py goto -0.408 -0.204 1.092 0 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger

# openrua op 168
timeout 600 python3 ctl.py goto -0.408 -0.204 1.087 0 3; timeout 60 python3 fk.py; timeout 300 python3 ctl.py js | grep finger

# openrua op 169
timeout 300 python3 ctl.py grip 0.04; timeout 600 python3 ctl.py goto -0.406 -0.204 1.20 0 4; timeout 600 python3 ctl.py goto -0.406 -0.204 1.20 0 3; timeout 60 python3 fk.py
timeout 120 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null; timeout 120 ./tools/perception/cam_snap.py frontview frontview.png >/dev/null
python3 -c "
from PIL import Image
Image.open('agentview.png').crop((100,80,350,300)).resize((800,704),Image.LANCZOS).save('crop2.png')
Image.open('frontview.png').crop((200,150,400,300)).resize((800,600),Image.LANCZOS).save('crop.png')"

# openrua op 170
timeout 600 python3 ctl.py goto -0.25 -0.35 1.30 0 4; timeout 600 python3 ctl.py goto -0.25 -0.35 1.30 0 3; timeout 60 python3 fk.py
for i in 1 2 3; do timeout 120 python3 depth_world.py birdview >/dev/null && break; done; for i in 1 2 3; do timeout 120 python3 depth_world.py frontview >/dev/null && break; done
python3 - <<'EOF'
import numpy as np
from scipy.optimize import least_squares
for cam in ["birdview","frontview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
    m=(xyz[:,2]>1.062)&(xyz[:,2]<1.15)&(xyz[:,0]>-0.50)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.25)&(xyz[:,1]<-0.05)
    p=xyz[m]
    print(cam,"points above walls:",len(p),"z max",p[:,2].max().round(3))
    for zlo in np.arange(1.06,1.12,0.01):
        q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.01)]
        if len(q): print(f"  z{zlo:.3f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
    q=p[p[:,2]>1.085]
    if len(q)>10:
        f=lambda c: np.hypot(q[:,0]-c[0],q[:,1]-c[1])-0.052
        c=least_squares(f,[q[:,0].mean(),q[:,1].mean()]).x
        print("  rim fit center",c.round(4),"resid",np.abs(f(c)).mean().round(4))
# inner base from birdview through the mouth
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>0.98)&(xyz[:,2]<1.03)&(xyz[:,0]>-0.47)&(xyz[:,0]<-0.36)&(xyz[:,1]>-0.21)&(xyz[:,1]<-0.09)
q=xyz[m]; print("inner floor pts",len(q), "z",q[:,2].min().round(3),q[:,2].max().round(3),"x",q[:,0].min().round(3),q[:,0].max().round(3),"y",q[:,1].min().round(3),q[:,1].max().round(3))
EOF

# openrua op 171
python3 - <<'EOF'
import numpy as np
for cam in ["birdview","frontview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
    m=(xyz[:,2]>0.95)&(xyz[:,0]>-0.472)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.20)&(xyz[:,1]<-0.085)
    p=xyz[m]
    print(cam)
    # per x-bin (1cm) max z within the column
    for x in np.arange(-0.47,-0.30,0.01):
        q=p[(p[:,0]>=x)&(p[:,0]<x+0.01)]
        if len(q): print(f"  x{x:.2f}: zmax {q[:,2].max():.3f} at y {q[np.argmax(q[:,2]),1]:.3f}  n={len(q)}")
    for y in np.arange(-0.20,-0.085,0.01):
        q=p[(p[:,1]>=y)&(p[:,1]<y+0.01)]
        if len(q): print(f"  y{y:.2f}: zmax {q[:,2].max():.3f} at x {q[np.argmax(q[:,2]),0]:.3f}  n={len(q)}")
EOF

# openrua op 172
python3 - <<'EOF'
import numpy as np
for cam in ["birdview","frontview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
    m=(xyz[:,2]>0.95)&(xyz[:,2]<1.2)&(xyz[:,0]>-0.472)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.20)&(xyz[:,1]<-0.085)
    p=xyz[m]
    print(cam)
    for x in np.arange(-0.47,-0.30,0.01):
        q=p[(p[:,0]>=x)&(p[:,0]<x+0.01)]
        if len(q): print(f"  x{x:.2f}: zmax {q[:,2].max():.3f} at y {q[np.argmax(q[:,2]),1]:.3f}  n={len(q)}")
EOF

# openrua op 173
timeout 120 ./tools/perception/cam_snap.py agentview agentview.png >/dev/null; python3 -c "
from PIL import Image
Image.open('agentview.png').crop((100,80,350,300)).resize((800,704),Image.LANCZOS).save('crop2.png')"

# openrua op 174
for i in 1 2 3; do timeout 120 python3 depth_world.py birdview >/dev/null && break; done
python3 - <<'EOF'
import numpy as np
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>0.95)&(xyz[:,2]<1.2)&(xyz[:,0]>-0.472)&(xyz[:,0]<-0.25)&(xyz[:,1]>-0.24)&(xyz[:,1]<-0.085)
p=xyz[m]
# rim = highest points per y-bin -> gives rim ring where it's the top surface
for y in np.arange(-0.24,-0.085,0.01):
    q=p[(p[:,1]>=y)&(p[:,1]<y+0.01)]
    if len(q): 
        i=np.argmax(q[:,2]); print(f"  y{y:.2f}: zmax {q[i,2]:.3f} at x {q[i,0]:.3f}; xrange {q[:,0].min():.3f}..{q[:,0].max():.3f} n={len(q)}")
EOF

# openrua op 175
python3 ctl.py grip 0.04 && python3 ctl.py goto -0.325 -0.2215 1.15 0 5 && python3 ctl.py goto -0.325 -0.2215 1.08 0 4 && python3 fk.py

# openrua op 176
python3 ctl.py goto -0.325 -0.2215 1.06 0 3 && python3 fk.py && python3 ctl.py grip 0.0 && python3 fk.py

# openrua op 177
python3 ctl.py goto -0.325 -0.2215 1.25 0 6 && python3 ctl.py js | grep finger && python3 ctl.py goto -0.25 -0.25 1.25 0 5 && python3 ctl.py js | grep finger && python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null

# openrua op 178
for i in 1 2 3; do timeout 120 python3 depth_world.py birdview >/dev/null 2>&1 && break; done
python3 - <<'EOF'
import numpy as np
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>0.95)&(xyz[:,2]<1.2)&(xyz[:,0]>-0.472)&(xyz[:,0]<-0.25)&(xyz[:,1]>-0.24)&(xyz[:,1]<-0.085)
p=xyz[m]
for y in np.arange(-0.24,-0.085,0.01):
    q=p[(p[:,1]>=y)&(p[:,1]<y+0.01)]
    if len(q):
        i=np.argmax(q[:,2]); print(f"  y{y:.2f}: zmax {q[i,2]:.3f} at x {q[i,0]:.3f}; xrange {q[:,0].min():.3f}..{q[:,0].max():.3f} n={len(q)}")
print("per x:")
for x in np.arange(-0.47,-0.25,0.01):
    q=p[(p[:,0]>=x)&(p[:,0]<x+0.01)]
    if len(q): print(f"  x{x:.2f}: zmax {q[:,2].max():.3f} yrange {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
EOF

# openrua op 179
python3 - <<'EOF'
import numpy as np
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>0.905)&(xyz[:,2]<1.2)&(xyz[:,0]>-0.50)&(xyz[:,0]<-0.25)&(xyz[:,1]>-0.24)&(xyz[:,1]<-0.06)
p=xyz[m]
xs=np.arange(-0.50,-0.25,0.01); ys=np.arange(-0.24,-0.06,0.01)
print("z-max map (cm above table), rows x, cols y from -0.24 to -0.07")
print("      "+" ".join(f"{y:5.2f}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        q=p[(p[:,0]>=x)&(p[:,0]<x+0.01)&(p[:,1]>=y)&(p[:,1]<y+0.01)]
        row.append(f"{(q[:,2].max()-0.88)*100:5.1f}" if len(q) else "    .")
    print(f"{x:5.2f} "+" ".join(row))
EOF

# openrua op 180
for i in 1 2 3; do timeout 120 python3 depth_world.py frontview >/dev/null 2>&1 && break; done
python3 - <<'EOF'
import numpy as np
for cam in ["birdview","frontview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
    m=(xyz[:,2]>0.89)&(xyz[:,2]<1.2)&(xyz[:,0]>-0.50)&(xyz[:,0]<-0.20)&(np.abs(xyz[:,1]+0.165)<0.006)
    p=xyz[m]
    print(cam, "x-z slice at y=-0.165 (x: zmin..zmax)")
    for x in np.arange(-0.50,-0.20,0.01):
        q=p[(p[:,0]>=x)&(p[:,0]<x+0.01)]
        if len(q): print(f"  x{x:.2f}: z {q[:,2].min():.3f}..{q[:,2].max():.3f} n={len(q)}")
EOF

# openrua op 181
python3 -c "
from PIL import Image
im=Image.open('agentview.png'); im.crop((150,140,330,280)).resize((720,560),Image.LANCZOS).save('crop3.png')"

# openrua op 182
python3 - <<'EOF'
import numpy as np
for cam in ["frontview","birdview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
    m=(xyz[:,2]>0.95)&(xyz[:,2]<1.2)&(xyz[:,0]>-0.44)&(xyz[:,0]<-0.28)&(xyz[:,1]>-0.235)&(xyz[:,1]<-0.09)
    p=xyz[m]
    print(cam)
    for z in np.arange(0.95,1.11,0.01):
        q=p[(p[:,2]>=z)&(p[:,2]<z+0.01)]
        if len(q):
            j=np.argmin(q[:,1]); k=np.argmax(q[:,1])
            print(f"  z{z:.2f}: ymin {q[j,1]:.3f}(x{q[j,0]:.3f}) ymax {q[k,1]:.3f}(x{q[k,0]:.3f}) xrange {q[:,0].min():.3f}..{q[:,0].max():.3f} n={len(q)}")
EOF

# openrua op 183
python3 - <<'EOF'
import numpy as np, sys
sys.argv=["x","gotof"]
from ctl import Ctl, goto_tcp_frame, TCP
from scipy.spatial.transform import Rotation as Rot
c=Ctl()
a=np.array([0.82,0,0.57]); a/=np.linalg.norm(a)
f=np.array([-a[2],0,a[0]]); d=-a
P=np.array([-0.374,-0.162,1.096])-0.012*a
for name,tcp in [("pre",P+0.05*(-d)),("grasp",P)]:
    x=np.cross(f,d); R=Rot.from_matrix(np.column_stack([x,f,d]))
    q=(R*Rot.from_euler("z",45,degrees=True)).as_quat()
    hand=tcp-TCP*d
    try:
        qj=c.solve_ik(hand,q); print(name, tcp.round(3), "IK ok", np.round(qj,2).tolist())
    except SystemExit as e: print(name, tcp.round(3), e)
EOF

# openrua op 184
python3 ctl.py grip 0.04 >/dev/null; python3 ctl.py gotof -0.343 -0.162 1.118 -0.57 0 0.82 -0.82 0 -0.57 6 ; python3 fk.py

# openrua op 185
python3 ctl.py gotof -0.343 -0.162 1.118 -0.57 0 0.82 -0.82 0 -0.57 4 ; python3 fk.py; python3 ctl.py js | head -7

# openrua op 186
python3 ctl.py gotof -0.384 -0.162 1.089 -0.57 0 0.82 -0.82 0 -0.57 4 ; python3 fk.py; python3 ctl.py grip 0.0; python3 fk.py

# openrua op 187
python3 ctl.py gotof -0.384 -0.162 1.15 -0.57 0 0.82 -0.82 0 -0.57 4 ; python3 fk.py; python3 ctl.py js | grep finger; python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; python3 -c "
from PIL import Image
im=Image.open('agentview.png'); im.crop((120,60,380,300)).resize((780,720),Image.LANCZOS).save('crop3.png')"

# openrua op 188
python3 ctl.py gotof -0.384 -0.162 1.20 -0.57 0 0.82 -0.82 0 -0.57 4 && python3 ctl.py gotof -0.22 -0.162 1.20 -0.57 0 0.82 -0.82 0 -0.57 6 ; python3 fk.py; python3 ctl.py js | grep finger

# openrua op 189
python3 ctl.py gotof -0.22 -0.162 1.20 -0.88 0 0.47 -0.47 0 -0.88 4 ; python3 ctl.py js | grep finger; python3 ctl.py gotof -0.22 -0.162 1.20 -1 0 0 0 0 -1 4 ; python3 fk.py; python3 ctl.py js | grep finger

# openrua op 190
python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; python3 -c "
from PIL import Image
im=Image.open('agentview.png'); im.crop((150,40,450,300)).resize((900,780),Image.LANCZOS).save('crop3.png')"; python3 xtilt.py -0.2204 -0.1615 1.1996 frontview 2>&1 | tail -15

# openrua op 191
python3 - <<'EOF'
import numpy as np
xyz=np.load("frontview_world.npz")["xyz"].reshape(-1,3)
tcp=np.array([-0.2204,-0.1615,1.1996])
m=(xyz[:,2]>1.0)&(xyz[:,2]<1.25)&(xyz[:,0]>tcp[0]-0.04)&(xyz[:,0]<tcp[0]+0.16)&(xyz[:,1]>-0.35)&(xyz[:,1]<0.0)
p=xyz[m]
for z in np.arange(1.0,1.25,0.01):
    q=p[(p[:,2]>=z)&(p[:,2]<z+0.01)]
    if len(q): print(f"z{z:.2f}: y {q[:,1].min():.3f}..{q[:,1].max():.3f} mid {(q[:,1].min()+q[:,1].max())/2:.3f}  x {q[:,0].min():.3f}..{q[:,0].max():.3f} n={len(q)}")
EOF

# openrua op 192
python3 ctl.py gotof -0.22 -0.162 1.00 -1 0 0 0 0 -1 6 ; python3 ctl.py js | grep finger; python3 ctl.py gotof -0.22 -0.162 0.99 -1 0 0 0 0 -1 3 ; python3 fk.py; python3 ctl.py js | grep finger

# openrua op 193
python3 ctl.py gotof -0.22 -0.162 0.984 -1 0 0 0 0 -1 3 ; python3 fk.py; python3 ctl.py js | grep finger

# openrua op 194
for z in 0.979 0.975 0.971; do python3 ctl.py gotof -0.22 -0.162 $z -1 0 0 0 0 -1 3 | grep -v "ik q"; python3 fk.py | grep -o "tcp.*hand_z \[[^]]*\]"; python3 ctl.py js | grep finger | tr '\n' ' '; echo; done

# openrua op 195
python3 ctl.py grip 0.04; python3 ctl.py gotof -0.22 -0.162 1.10 -1 0 0 0 0 -1 4 | grep -v "ik q"; python3 ctl.py gotof -0.22 -0.30 1.25 -1 0 0 0 0 -1 5 | grep -v "ik q"; python3 fk.py
for i in 1 2 3; do timeout 120 python3 depth_world.py birdview >/dev/null 2>&1 && break; done
python3 - <<'EOF'
import numpy as np
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>0.9)&(xyz[:,2]<1.1)&(xyz[:,0]>-0.34)&(xyz[:,0]<-0.10)&(xyz[:,1]>-0.30)&(xyz[:,1]<-0.02)
p=xyz[m]
print("zmax",p[:,2].max())
for z in np.arange(0.9,1.1,0.02):
    q=p[(p[:,2]>=z)&(p[:,2]<z+0.02)]
    if len(q): print(f"z{z:.2f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
# rim circle fit on top points
top=p[p[:,2]>p[:,2].max()-0.012]
A=np.c_[2*top[:,0],2*top[:,1],np.ones(len(top))]; b=(top[:,:2]**2).sum(1)
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx*cx+cy*cy)
print(f"rim fit center ({cx:.4f},{cy:.4f}) r={r:.4f} n={len(top)}")
EOF

# openrua op 196
python3 - <<'EOF'
import numpy as np
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>0.975)&(xyz[:,2]<1.01)&(xyz[:,0]>-0.30)&(xyz[:,0]<-0.08)&(xyz[:,1]>-0.30)&(xyz[:,1]<-0.02)
top=xyz[m]
print("n",len(top),"z",top[:,2].min(),top[:,2].max())
A=np.c_[2*top[:,0],2*top[:,1],np.ones(len(top))]; b=(top[:,:2]**2).sum(1)
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx*cx+cy*cy)
d=np.hypot(top[:,0]-cx,top[:,1]-cy)
print(f"rim fit center ({cx:.4f},{cy:.4f}) r={r:.4f} resid {np.abs(d-r).mean():.4f}")
# refit excluding outliers (handle)
k=np.abs(d-r)<0.006; top=top[k]
A=np.c_[2*top[:,0],2*top[:,1],np.ones(len(top))]; b=(top[:,:2]**2).sum(1)
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx*cx+cy*cy)
print(f"refit center ({cx:.4f},{cy:.4f}) r={r:.4f} n={len(top)}")
# handle direction: outlier points
o=xyz[m][~k]
if len(o): print("outliers mean", o[:,:2].mean(0).round(3), "dir", np.degrees(np.arctan2(o[:,1].mean()-cy,o[:,0].mean()-cx)).round(0))
EOF
python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; python3 -c "
from PIL import Image
im=Image.open('agentview.png'); im.crop((200,150,500,400)).resize((900,750),Image.LANCZOS).save('crop3.png')"

# openrua op 197
python3 ctl.py goto -0.1718 -0.1081 1.10 0 5 && python3 ctl.py goto -0.1718 -0.1081 0.987 0 4 && python3 fk.py && python3 ctl.py grip 0.0 && python3 ctl.py goto -0.1718 -0.1081 1.22 0 6 && python3 ctl.py js | grep finger

# openrua op 198
python3 ctl.py goto -0.1718 -0.1081 1.22 45 5 && python3 ctl.py goto -0.1718 -0.1081 1.22 90 5 && python3 fk.py && python3 ctl.py js | grep finger
for i in 1 2 3; do timeout 120 python3 depth_world.py frontview >/dev/null 2>&1 && break; done
python3 - <<'EOF'
import numpy as np
xyz=np.load("frontview_world.npz")["xyz"].reshape(-1,3)
tcp=np.array([-0.1718,-0.1081,1.22])
m=(xyz[:,2]>1.09)&(xyz[:,2]<1.26)&(xyz[:,0]>tcp[0]-0.03)&(xyz[:,0]<tcp[0]+0.20)&(xyz[:,1]>-0.30)&(xyz[:,1]<0.0)
p=xyz[m]
for z in np.arange(1.09,1.26,0.01):
    q=p[(p[:,2]>=z)&(p[:,2]<z+0.01)]
    if len(q): print(f"z{z:.2f}: y {q[:,1].min():.3f}..{q[:,1].max():.3f} mid {(q[:,1].min()+q[:,1].max())/2:.3f}  xmax {q[:,0].max():.3f} n={len(q)}")
EOF

# openrua op 199
python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; python3 -c "
from PIL import Image
im=Image.open('agentview.png'); im.crop((100,100,540,420)).resize((880,640),Image.LANCZOS).save('crop3.png')"

# openrua op 200
for i in 1 2 3; do timeout 120 python3 depth_world.py birdview >/dev/null 2>&1 && break; done
python3 - <<'EOF'
import numpy as np
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>0.975)&(xyz[:,2]<1.01)&(xyz[:,0]>-0.34)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.35)&(xyz[:,1]<0.05)
top=xyz[m]
print("n",len(top),"z",top[:,2].min().round(3),top[:,2].max().round(3), "x",top[:,0].min().round(3),top[:,0].max().round(3),"y",top[:,1].min().round(3),top[:,1].max().round(3))
A=np.c_[2*top[:,0],2*top[:,1],np.ones(len(top))]; b=(top[:,:2]**2).sum(1)
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx*cx+cy*cy)
d=np.hypot(top[:,0]-cx,top[:,1]-cy)
k=np.abs(d-r)<0.006; t=top[k]
A=np.c_[2*t[:,0],2*t[:,1],np.ones(len(t))]; b=(t[:,:2]**2).sum(1)
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx*cx+cy*cy)
print(f"rim center ({cx:.4f},{cy:.4f}) r={r:.4f} n={len(t)} outliers {(~k).sum()}")
o=top[~k]
if len(o): print("handle dir deg", np.degrees(np.arctan2(o[:,1].mean()-cy,o[:,0].mean()-cx)).round(0), o[:,:2].mean(0).round(3))
EOF

# openrua op 201
python3 ctl.py grip 0.04 >/dev/null; python3 ctl.py goto -0.30 -0.40 1.30 0 5 | grep -v "ik q"
for i in 1 2 3; do timeout 120 python3 depth_world.py birdview >/dev/null 2>&1 && break; done
python3 - <<'EOF'
import numpy as np
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>0.975)&(xyz[:,2]<1.01)&(xyz[:,0]>-0.34)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.35)&(xyz[:,1]<0.05)
top=xyz[m]
print("n",len(top),"z",top[:,2].min().round(3),top[:,2].max().round(3), "x",top[:,0].min().round(3),top[:,0].max().round(3),"y",top[:,1].min().round(3),top[:,1].max().round(3))
A=np.c_[2*top[:,0],2*top[:,1],np.ones(len(top))]; b=(top[:,:2]**2).sum(1)
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx*cx+cy*cy)
d=np.hypot(top[:,0]-cx,top[:,1]-cy)
k=np.abs(d-r)<0.006; t=top[k]
A=np.c_[2*t[:,0],2*t[:,1],np.ones(len(t))]; b=(t[:,:2]**2).sum(1)
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx*cx+cy*cy)
print(f"rim center ({cx:.4f},{cy:.4f}) r={r:.4f} n={len(t)} outliers {(~k).sum()}")
o=top[~k]
if len(o): print("handle dir deg", np.degrees(np.arctan2(o[:,1].mean()-cy,o[:,0].mean()-cx)).round(0), o[:,:2].mean(0).round(3))
# handle from lower band
m2=(xyz[:,2]>0.93)&(xyz[:,2]<0.97)&(np.hypot(xyz[:,0]-cx,xyz[:,1]-cy)>0.058)&(np.hypot(xyz[:,0]-cx,xyz[:,1]-cy)<0.10)
h=xyz[m2]
if len(h): print("handle band pts", len(h), "dir", np.degrees(np.arctan2(h[:,1].mean()-cy,h[:,0].mean()-cx)).round(0))
EOF

# openrua op 202
python3 ctl.py goto -0.1396 -0.1837 1.10 0 6 | grep -v "ik q"; python3 ctl.py goto -0.1396 -0.1837 1.10 0 4 | grep -v "ik q"; python3 ctl.py goto -0.1396 -0.1837 0.982 0 4 | grep -v "ik q"; python3 fk.py; python3 ctl.py grip 0.0

# openrua op 203
python3 ctl.py goto -0.1396 -0.1837 1.22 0 6 | grep -v "ik q"; python3 ctl.py js | grep finger | tr '\n' ' '; echo
for yaw in -22 -44 -66; do python3 ctl.py goto -0.1396 -0.1837 1.22 $yaw 5 | grep -v "ik q"; python3 ctl.py js | grep finger | tr '\n' ' '; echo; done; python3 fk.py

# openrua op 204
for i in 1 2 3; do timeout 120 python3 depth_world.py frontview >/dev/null 2>&1 && break; done
for i in 1 2 3; do timeout 120 python3 depth_world.py birdview >/dev/null 2>&1 && break; done
python3 - <<'EOF'
import numpy as np
tcp=np.array([-0.1399,-0.1839,1.2192])
cx,cy=tcp[0]+0.0493,tcp[1]+0.022
print("expected cup center",round(cx,4),round(cy,4))
xyz=np.load("frontview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>1.09)&(xyz[:,2]<1.24)&(xyz[:,0]>cx-0.07)&(xyz[:,0]<cx+0.12)&(xyz[:,1]>cy-0.08)&(xyz[:,1]<cy+0.08)
p=xyz[m]
print("frontview: per z, y extent and +x face")
for z in np.arange(1.09,1.24,0.01):
    q=p[(p[:,2]>=z)&(p[:,2]<z+0.01)]
    if len(q): print(f"  z{z:.2f}: y {q[:,1].min():.3f}..{q[:,1].max():.3f} mid {(q[:,1].min()+q[:,1].max())/2:.3f} xmax {q[:,0].max():.3f} n={len(q)}")
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>1.20)&(xyz[:,2]<1.24)&(xyz[:,0]>cx-0.10)&(xyz[:,0]<cx+0.10)&(xyz[:,1]>cy-0.10)&(xyz[:,1]<cy+0.10)
t=xyz[m]
d=np.hypot(t[:,0]-cx,t[:,1]-cy)
print("birdview rim-level pts",len(t)," r<0.06:",(d<0.06).sum()," r 0.06-0.10:",(d>=0.06).sum())
o=t[d>=0.06]
if len(o): print("outer pts dir deg", np.degrees(np.arctan2(o[:,1]-cy,o[:,0]-cx)).round(0)[:20])
r=t[d<0.06]
if len(r)>10:
    A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,:2]**2).sum(1)
    a,bb,c=np.linalg.lstsq(A,b,rcond=None)[0]; print("rim fit", round(a,4),round(bb,4), "r",round(np.sqrt(c+a*a+bb*bb),4))
m=(xyz[:,2]>1.10)&(xyz[:,2]<1.14)&(np.hypot(xyz[:,0]-cx,xyz[:,1]-cy)<0.06)
bpts=xyz[m]; print("base-level pts", len(bpts), "mean xy", bpts[:,:2].mean(0).round(4) if len(bpts) else None)
EOF

# openrua op 205
for i in 1 2 3; do timeout 120 python3 depth_world.py sideview >/dev/null 2>&1 && break; done
python3 - <<'EOF'
import numpy as np
xyz=np.load("sideview_world.npz")["xyz"].reshape(-1,3)
ok=np.isfinite(xyz).all(1)&(np.abs(xyz).max(1)<5)
print("sideview cloud extents x",xyz[ok][:,0].min().round(2),xyz[ok][:,0].max().round(2),"y",xyz[ok][:,1].min().round(2),xyz[ok][:,1].max().round(2))
cx,cy=-0.099,-0.166
m=ok&(xyz[:,2]>1.09)&(xyz[:,2]<1.24)&(xyz[:,0]>cx-0.09)&(xyz[:,0]<cx+0.09)&(xyz[:,1]>cy-0.08)&(xyz[:,1]<cy+0.08)
p=xyz[m]
print("n",len(p))
for z in np.arange(1.09,1.24,0.01):
    q=p[(p[:,2]>=z)&(p[:,2]<z+0.01)]
    if len(q): print(f"  z{z:.2f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} mid {(q[:,0].min()+q[:,0].max())/2:.3f}  y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
EOF

# openrua op 206
python3 ctl.py goto -0.30 -0.155 1.22 -66 6 | grep -v "ik q"; python3 ctl.py js | grep finger | tr '\n' ' '; echo; python3 ctl.py goto -0.4568 -0.155 1.22 -66 6 | grep -v "ik q"; python3 fk.py; python3 ctl.py js | grep finger | tr '\n' ' '

# openrua op 207
for i in 1 2 3; do timeout 120 python3 depth_world.py birdview >/dev/null 2>&1 && break; done
for i in 1 2 3; do timeout 120 python3 depth_world.py sideview >/dev/null 2>&1 && break; done
python3 - <<'EOF'
import numpy as np
tcp=np.array([-0.4567,-0.1553,1.2197]); cx,cy=tcp[0]+0.0408,tcp[1]+0.0175
print("expected rim center",round(cx,4),round(cy,4))
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>1.20)&(xyz[:,2]<1.245)&(np.hypot(xyz[:,0]-cx,xyz[:,1]-cy)<0.062)
r=xyz[m]
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=(r[:,:2]**2).sum(1)
a,bb,c=np.linalg.lstsq(A,b,rcond=None)[0]; print("birdview rim fit", round(a,4),round(bb,4), "r",round(np.sqrt(c+a*a+bb*bb),4),"n",len(r))
xyz=np.load("sideview_world.npz")["xyz"].reshape(-1,3)
ok=np.isfinite(xyz).all(1)&(np.abs(xyz).max(1)<5)
m=ok&(xyz[:,2]>1.09)&(xyz[:,2]<1.25)&(xyz[:,0]>cx-0.09)&(xyz[:,0]<cx+0.09)&(xyz[:,1]>cy-0.08)&(xyz[:,1]<cy+0.08)
p=xyz[m]
for z in np.arange(1.10,1.25,0.01):
    q=p[(p[:,2]>=z)&(p[:,2]<z+0.01)]
    if len(q): print(f"  z{z:.2f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f}  y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
EOF

# openrua op 208
for z in 1.15 1.11 1.098; do python3 ctl.py goto -0.4568 -0.155 $z -66 4 | grep -v "ik q"; python3 fk.py | grep -o "tcp.*hand_z \[[^]]*\]"; python3 ctl.py js | grep finger | tr '\n' ' '; echo; done

# openrua op 209
python3 ctl.py goto -0.4568 -0.155 1.16 -66 4 | grep -v "ik q"; python3 fk.py | grep -o "tcp.*hand_z \[[^]]*\]"; python3 ctl.py js | grep finger | tr '\n' ' '; echo; python3 ctl.py goto -0.4568 -0.155 1.16 -66 3 | grep -v "ik q"; python3 fk.py | grep -o "tcp.*hand_z \[[^]]*\]"; python3 ctl.py js | grep finger | tr '\n' ' '

# openrua op 210
python3 ctl.py goto -0.4488 -0.155 1.16 -66 4 | grep -v "ik q"; python3 ctl.py js | grep finger | tr '\n' ' '; echo
for z in 1.13 1.11 1.10 1.095; do python3 ctl.py goto -0.4488 -0.155 $z -66 4 | grep -v "ik q"; python3 fk.py | grep -o "tcp.*hand_z \[[^]]*\]"; python3 ctl.py js | grep finger | tr '\n' ' '; echo; done

# openrua op 211
for z in 1.092 1.089; do python3 ctl.py goto -0.4488 -0.155 $z -66 3 | grep -v "ik q"; python3 fk.py | grep -o "tcp.*hand_z \[[^]]*\]"; python3 ctl.py js | grep finger | tr '\n' ' '; echo; done

# openrua op 212
python3 ctl.py grip 0.04; python3 ctl.py goto -0.4488 -0.155 1.16 -66 4 | grep -v "ik q"; python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; python3 -c "
from PIL import Image
im=Image.open('agentview.png'); im.crop((120,100,420,320)).resize((900,660),Image.LANCZOS).save('crop3.png')"

# openrua op 213
python3 ctl.py goto -0.30 -0.40 1.30 0 6 | grep -v "ik q"; python3 ctl.py goto -0.30 -0.40 1.30 0 4 | grep -v "ik q"; python3 fk.py
for i in 1 2 3; do timeout 120 python3 depth_world.py birdview >/dev/null 2>&1 && break; done
for i in 1 2 3; do timeout 120 python3 depth_world.py frontview >/dev/null 2>&1 && break; done
python3 - <<'EOF'
import numpy as np
for cam in ["birdview","frontview"]:
    xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
    m=(xyz[:,2]>0.95)&(xyz[:,2]<1.2)&(xyz[:,0]>-0.472)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.21)&(xyz[:,1]<-0.075)
    p=xyz[m]
    print(cam,"zmax",p[:,2].max().round(4))
    top=p[p[:,2]>p[:,2].max()-0.012]
    A=np.c_[2*top[:,0],2*top[:,1],np.ones(len(top))]; b=(top[:,:2]**2).sum(1)
    a,bb,c=np.linalg.lstsq(A,b,rcond=None)[0]; print("  rim fit center",round(a,4),round(bb,4),"r",round(np.sqrt(c+a*a+bb*bb),4),"n",len(top))
    for x in np.arange(-0.47,-0.33,0.01):
        q=top[(top[:,0]>=x)&(top[:,0]<x+0.01)]
        if len(q): print(f"  rim x{x:.2f}: z {q[:,2].mean():.4f} n={len(q)}")
    for y in np.arange(-0.21,-0.08,0.01):
        q=top[(top[:,1]>=y)&(top[:,1]<y+0.01)]
        if len(q): print(f"  rim y{y:.2f}: z {q[:,2].mean():.4f} n={len(q)}")
EOF

# openrua op 214
python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; python3 -c "
from PIL import Image
im=Image.open('agentview.png'); im.crop((120,100,420,320)).resize((900,660),Image.LANCZOS).save('crop3.png')"
