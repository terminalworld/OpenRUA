#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
find /workspace/tools /workspace/.ros -type f; echo ---; ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | head -40; echo ---; ros2 node list

# openrua op 2
mkdir -p snaps && for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; ros2 topic echo /joint_states --once

# openrua op 3
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -8; ros2 topic echo /birdview/color/camera_info --once | grep -A12 '^k:'

# openrua op 4
mkdir -p "$(dirname /workspace/px_batch.py)"
cat > /workspace/px_batch.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""px_batch.py <camera> u,v [u,v ...]  -> world xyz for each pixel (one node)."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def main():
    cam = sys.argv[1]
    pts = [tuple(map(int, a.split(","))) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("px_batch")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    for (u, v) in pts:
        zc = float(D[v, u])
        p = R @ np.array([(u-cx)*zc/fx, (v-cy)*zc/fy, zc]) + tr
        print(f"({u},{v}) depth={zc:.3f} -> {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 5
timeout 60 python3 px_batch.py birdview 320,305 313,300 422,298 215,215 280,215 215,320 280,320 247,265 230,213 150,185 190,200 320,240 100,400 500,400

# openrua op 6
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw snaps/birdview_depth.png && python3 - <<'EOF'
import numpy as np
D=np.load('snaps/birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
# camera at height: table pixel depth 2.1 at z=0.9 -> cam z=3.0; cam looking straight down; image down=+x, right=+y
# world x = -0.2 + (v-240)*z/fx ; world y = (u-320)*z/fx  (approx, verify with earlier points)
H=3.0-D  # height above floor
# microwave region print, coarse grid
for v in range(170,340,5):
    print(f"{v:3d} "+"".join('#' if H[v,u]>1.0 else ('+' if H[v,u]>0.93 else '.') for u in range(120,300,2)))
EOF

# openrua op 7
python3 - <<'EOF'
import numpy as np
D=np.load('snaps/birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
# camera pose from earlier: world = R*p_cam + t ; empirically x = -0.2 + (v-cy)*z/fx, y=(u-cx)*z/fx, zw = 3.0 - z
vv,uu=np.mgrid[0:480,0:640]
X=-0.2+(vv-cy)*D/fx; Y=(uu-cx)*D/fx; Z=3.0-D
def box(mask,name):
    print(name, "x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] n=%d"%(X[mask].min(),X[mask].max(),Y[mask].min(),Y[mask].max(),Z[mask].min(),Z[mask].max(),mask.sum()))
body=(Z>1.0)&(uu>=218)&(uu<=290)&(vv>=213)&(vv<=318)
box(body,"body")
door=(Z>0.95)&(uu>=140)&(uu<=232)&(vv>=175)&(vv<=214)
box(door,"door")
# door points list to fit line
pts=np.c_[X[door],Y[door],Z[door]]
for p in pts[::15]: print("  door pt %.3f %.3f %.3f"%tuple(p))
# mug yellow
mug=(Z>0.93)&(uu>=295)&(uu<=345)&(vv>=280)&(vv<=330)
box(mug,"mug")
print("mug top-only (Z>1.0):"); box(mug&(Z>1.0),"mugtop")
# body top surface heights histogram
print(np.percentile(Z[body],[5,50,95]))
# side feature bottom-left
sf=(Z>0.93)&(uu>=205)&(uu<=222)&(vv>=290)&(vv<=320); 
if sf.sum(): box(sf,"sidefeat")
EOF

# openrua op 8
for c in paperview galleryview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done

# openrua op 9
timeout 60 python3 tools/perception/cam_snap.py /robot0_robotview/depth/image_raw snaps/robotview_depth.png >/dev/null && timeout 60 python3 - <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
rclpy.init(); node=rclpy.create_node('pc')
buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(CameraInfo,'/robot0_robotview/color/camera_info',lambda m: got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(node,timeout_sec=0.2)
while not buf.can_transform('world','robot0_robotview_optical_frame',Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world','robot0_robotview_optical_frame',Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
k=got['m'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
D=np.load('snaps/robotview_depth.npy'); h,wd=D.shape
vv,uu=np.mgrid[0:h,0:wd]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D],-1)@R.T+tr
X,Y,Z=P[...,0],P[...,1],P[...,2]
print("cam pos",tr)
mug=(X>-0.06)&(X<0.10)&(Y>-0.10)&(Y<0.09)&(Z>0.905)&(Z<1.3)
print("mug: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"%(X[mug].min(),X[mug].max(),Y[mug].min(),Y[mug].max(),Z[mug].min(),Z[mug].max()))
# per-height slices of the mug: y extent at various z (camera sees +x side face)
for zlo in np.arange(0.91,1.06,0.01):
    s=mug&(Z>=zlo)&(Z<zlo+0.01)
    if s.sum(): print("z %.2f: y[%.3f,%.3f] xmin %.3f n=%d"%(zlo,Y[s].min(),Y[s].max(),X[s].min(),s.sum()))
np.save('snaps/robotview_xyz.npy',P)
EOF

# openrua op 10
timeout 60 python3 - <<'EOF'
import numpy as np
P=np.load('snaps/robotview_xyz.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
tab=(X>-0.1)&(X<0.3)&(Y>0.1)&(Y<0.3)
print("table region z percentiles", np.percentile(Z[tab],[5,50,95]))
tab2=(X>0.08)&(X<0.2)&(Y>-0.05)&(Y<0.05)
print("table just +x of mug", np.percentile(Z[tab2],[5,50,95]))
# mug column: points with y in [-0.03,0.03], x in [-0.03,0.08]
col=(Y>-0.03)&(Y<0.03)&(X>-0.03)&(X<0.09)
zs=Z[col]; print("column z hist", np.histogram(zs,bins=np.arange(0.85,1.12,0.01)))
D=np.load('snaps/birdview_depth.npy')
print("birdview mug centre depth", D[300:312,315:330].min(), D[300:312,315:330].max())
EOF

# openrua op 11
timeout 120 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position | tail -3; timeout 60 python3 tools/perception/cam_snap.py /robot0_robotview/depth/image_raw snaps/robotview_depth.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw snaps/birdview_depth.png > /dev/null; python3 - <<'EOF'
import numpy as np
D=np.load('snaps/birdview_depth.npy'); print("birdview mug centre depth", D[300:312,315:330].min())
fx=579.4112549695428
vv,uu=np.mgrid[0:480,0:640]; X=-0.2+(vv-240)*D/fx; Y=(uu-320)*D/fx; Z=3.0-D
mug=(Z>0.93)&(uu>=290)&(uu<=350)&(vv>=275)&(vv<=335)
print("mug x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"%(X[mug].min(),X[mug].max(),Y[mug].min(),Y[mug].max(),Z[mug].min(),Z[mug].max()))
EOF

# openrua op 12
python3 - <<'EOF'
import numpy as np
D=np.load('snaps/birdview_depth.npy'); fx=579.4112549695428
vv,uu=np.mgrid[0:480,0:640]; X=-0.2+(vv-240)*D/fx; Y=(uu-320)*D/fx; Z=3.0-D
reg=(uu>=290)&(uu<=350)&(vv>=275)&(vv<=335)
rim=reg&(Z>0.995)
print("rim x[%.3f,%.3f] y[%.3f,%.3f] n=%d"%(X[rim].min(),X[rim].max(),Y[rim].min(),Y[rim].max(),rim.sum()))
# fit circle to rim points
pts=np.c_[X[rim],Y[rim]]
A=np.c_[2*pts,np.ones(len(pts))]; b=(pts**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx*cx+cy*cy)
print("rim circle centre (%.4f,%.4f) radius %.4f"%(cx,cy,r))
# handle: points with Z between 0.93 and 0.99 outside the circle
hd=reg&(Z>0.93)&(np.hypot(X-cx,Y-cy)>r+0.005)
print("handle x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"%(X[hd].min(),X[hd].max(),Y[hd].min(),Y[hd].max(),Z[hd].min(),Z[hd].max()))
# inner floor depth
inner=reg&(np.hypot(X-cx,Y-cy)<r-0.015)
print("inner z", np.percentile(Z[inner],[5,50,95]))
# grey mug for reference
reg2=(uu>=400)&(uu<=445)&(vv>=275)&(vv<=325)&(Z>0.93)
print("grey mug x[%.3f,%.3f] y[%.3f,%.3f] z max %.3f"%(X[reg2].min(),X[reg2].max(),Y[reg2].min(),Y[reg2].max(),Z[reg2].max()))
EOF

# openrua op 13
timeout 30 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | sed -n 2,4p; timeout 30 ros2 run tf2_ros tf2_echo panda_hand panda_leftfinger 2>&1 | sed -n 2,3p; ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 '^k:'; timeout 10 ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|child" | head -40

# openrua op 14
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small persistent helper around IK + FollowJointTrajectory + gripper.

World frame in, base-frame IK under the hood. Import and use:
    from arm import Arm; a = Arm(); a.move_hand(pos, R, secs)
"""
import sys, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE = np.array([-0.660, 0.0, 0.912])   # world -> panda_link0 (from TF)
TCP = M["hand"]["tcp_offset_m"]


def R_from_axes(z, y=None, x=None):
    """Rotation whose columns are hand x,y,z axes expressed in world."""
    z = np.asarray(z, float); z /= np.linalg.norm(z)
    if y is not None:
        y = np.asarray(y, float); y -= z * (y @ z); y /= np.linalg.norm(y)
        x = np.cross(y, z)
    else:
        x = np.asarray(x, float); x -= z * (x @ z); x /= np.linalg.norm(x)
        y = np.cross(z, x)
    return np.c_[x, y, z]


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, M["sensors"][1]["port"], self._on_wr, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.spin_until(lambda: self.js is not None)

    def _on_js(self, m): self.js = m
    def _on_wr(self, m): self.wr = m

    def spin_until(self, pred, timeout=30):
        t0 = time.time()
        while not pred() and time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return pred()

    def q(self):
        """Current arm joint positions in manifest order."""
        self.js = None; self.spin_until(lambda: self.js is not None)
        d = dict(zip(self.js.name, self.js.position))
        return np.array([d[j] for j in JOINTS])

    def fingers(self):
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self.wr = None; self.spin_until(lambda: self.wr is not None)
        f = self.wr.wrench.force; t = self.wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def hand_pose(self, q=None):
        """FK: world position + rotation of panda_hand."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(self.q() if q is None else q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        R = Rot.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
        return pos, R

    def solve_ik(self, pos, R, seed=None, at_tcp=False):
        """World pose of hand (or tcp point) -> joint vector or None."""
        pos = np.asarray(pos, float)
        if at_tcp:
            pos = pos - TCP * R[:, 2]
        p_base = pos - BASE
        qx, qy, qz, qw = Rot.from_matrix(R).as_quat()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = p_base
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = list(self.q() if seed is None else seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"  IK failed: {None if r is None else r.error_code.val}")
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    def traj(self, points, secs, verbose=True):
        """points: list of joint vectors; secs: total duration (or list of times)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(points)
        times = secs if isinstance(secs, (list, tuple)) else [secs * (i + 1) / n for i in range(n)]
        for p, t in zip(points, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in p])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(self.q() - np.asarray(points[-1])).max()
        if verbose:
            print(f"  traj done code={code} max joint err={err:.4f}")
        return code, err

    def move_hand(self, pos, R, secs=3.0, at_tcp=False, max_jump=2.5):
        q0 = self.q()
        sol = self.solve_ik(pos, R, seed=q0, at_tcp=at_tcp)
        if sol is None:
            return False
        jump = np.abs(sol - q0).max()
        if jump > max_jump:
            print(f"  IK solution jumps {jump:.2f} rad; refusing")
            return False
        code, err = self.traj([sol], secs)
        return code == 0 and err < 0.02

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        self.q()
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def snap(self, cam, out):
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        import cv2
        got = []
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw", got.append, 1)
        self.spin_until(lambda: got)
        self.node.destroy_subscription(sub)
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got[0], "bgr8"))
        return out
OPENRUA_EOF

# openrua op 15
timeout 300 python3 -u - <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
print("q", a.q().round(3))
pos,R=a.hand_pose(); print("hand pos", pos.round(3)); print(R.round(3))
# look into the microwave: hand above/in front of the opening, tilted 40deg toward +x
R1=R_from_axes(z=[np.sin(np.radians(40)),0,-np.cos(np.radians(40))], y=[0,-1,0])
ok=a.move_hand([-0.42,-0.23,1.30], R1, 3.0)
print("moved", ok)
pos,R=a.hand_pose(); print("hand pos", pos.round(3)); print(R.round(3))
a.snap("robot0_eye_in_hand","snaps/eih_cavity.png")
EOF

# openrua op 16
timeout 300 python3 -u - <<'EOF'
import numpy as np, arm
from arm import Arm
a=Arm()
q0=a.q()
# FK returned world coords -> set BASE to zero and re-check
arm.BASE=np.zeros(3)
pos,R=a.hand_pose(); print("hand (model frame)", pos.round(3))
print("IK at current pose, model frame coords:", None if a.solve_ik(pos,R,seed=q0) is None else "ok")
print("IK at current pose shifted to base coords:", None if a.solve_ik(pos-np.array([-0.66,0,0.912]),R,seed=q0) is None else "ok")
EOF

# openrua op 17
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE = np.array([-0.660, 0.0, 0.912])   # world -> panda_link0 (from TF)", "new_string": "# MoveIt\'s model frame here IS world (FK/IK verified against TF), so no offset\\nBASE = np.zeros(3)\\nBASE_POS = np.array([-0.660, 0.0, 0.912])   # world position of panda_link0 (TF)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 18
timeout 300 python3 -u - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
R1=R_from_axes(z=[np.sin(np.radians(40)),0,-np.cos(np.radians(40))], y=[0,-1,0])
ok=a.move_hand([-0.42,-0.23,1.30], R1, 3.0)
print("moved", ok)
pos,R=a.hand_pose(); print("hand pos", pos.round(3)); print(R.round(3))
a.snap("robot0_eye_in_hand","snaps/eih_cavity.png")
EOF

# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        code, err = self.traj([sol], secs)\\n        return code == 0 and err < 0.02", "new_string": "        code, err = self.traj([sol], secs)\\n        for _ in range(2):\\n            if err < 0.02:\\n                break\\n            # controller lag on long goals: resend the same target\\n            code, err = self.traj([sol], max(2.0, secs / 2))\\n        return err < 0.02", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
timeout 300 python3 -u - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
R1=R_from_axes(z=[np.sin(np.radians(40)),0,-np.cos(np.radians(40))], y=[0,-1,0])
ok=a.move_hand([-0.42,-0.23,1.30], R1, 4.0)
print("moved", ok)
pos,R=a.hand_pose(); print("hand pos", pos.round(3)); print(R.round(3))
a.snap("robot0_eye_in_hand","snaps/eih_cavity.png")
EOF

# openrua op 21
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        p_base = pos - BASE\\n        qx, qy, qz, qw = Rot.from_matrix(R).as_quat()", "new_string": "        p_base = pos - BASE\\n        # IK\'s tip link is panda_link8; panda_hand = link8 * Rz(-45deg)\\n        R8 = np.asarray(R) @ Rot.from_euler(\\"z\\", np.pi / 4).as_matrix()\\n        qx, qy, qz, qw = Rot.from_matrix(R8).as_quat()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 22
timeout 300 python3 -u - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
# camera looks along z_hand; put image "up" (-y_cam = +x_hand... ) sensible: y_hand=-y world
R1=R_from_axes(z=[np.sin(np.radians(45)),0,-np.cos(np.radians(45))], y=[0,-1,0])
ok=a.move_hand([-0.45,-0.23,1.25], R1, 4.0)
print("moved", ok)
pos,R=a.hand_pose(); print("hand pos", pos.round(3)); print(R.round(3))
a.snap("robot0_eye_in_hand","snaps/eih_cavity2.png")
EOF
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_depth.png

# openrua op 23
timeout 300 python3 -u - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
# look along +y, tilted 40deg down, from the -y side of the microwave
R1=R_from_axes(z=[0,np.cos(np.radians(40)),-np.sin(np.radians(40))], y=[1,0,0])
for p in ([-0.12,-0.62,1.22],[-0.12,-0.58,1.18],[-0.15,-0.66,1.25]):
    ok=a.move_hand(p, R1, 4.0); print("moved", ok, p)
    if ok: break
pos,R=a.hand_pose(); print("hand pos", pos.round(3)); print(R.round(3))
a.snap("robot0_eye_in_hand","snaps/eih_side.png")
EOF
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_depth.png

# openrua op 24
timeout 300 python3 -u - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
R1=R_from_axes(z=[0,np.cos(np.radians(45)),-np.sin(np.radians(45))], y=[1,0,0])
for p in ([-0.22,-0.52,1.20],[-0.25,-0.50,1.18],[-0.20,-0.48,1.15]):
    ok=a.move_hand(p, R1, 4.0); print("moved", ok, p)
    if ok: break
pos,R=a.hand_pose(); print("hand pos", pos.round(3)); print(R.round(3))
a.snap("robot0_eye_in_hand","snaps/eih_side.png")
EOF
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_depth.png

# openrua op 25
timeout 120 python3 - 2>/dev/null <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
rclpy.init(); node=rclpy.create_node('pc')
buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(CameraInfo,'/robot0_eye_in_hand/color/camera_info',lambda m: got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(node,timeout_sec=0.2)
fr='robot0_eye_in_hand_optical_frame'
while not buf.can_transform('world',fr,Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',fr,Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
print("cam pos",tr.round(3)); print("cam R",R.round(3))
k=got['m'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
D=np.load('snaps/eih_depth.npy'); h,wd=D.shape
vv,uu=np.mgrid[0:h,0:wd]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D],-1)@R.T+tr
np.save('snaps/eih_xyz.npy',P)
X,Y,Z=P[...,0],P[...,1],P[...,2]
# sample some pixels: the diagonal frame
for (u,v) in [(300,250),(400,300),(500,330),(560,350),(250,230),(330,230),(450,280),(600,300),(620,320),(280,320),(350,360),(420,380)]:
    print((u,v),"->",P[v,u].round(3))
# what is at y in [-0.36,-0.33] (the -y face plane) : z-x extent of solid points
face=(Y>-0.37)&(Y<-0.33)&(Z>0.9)&(Z<1.12)
print("face pts x[%.3f,%.3f] z[%.3f,%.3f]"%(X[face].min(),X[face].max(),Z[face].min(),Z[face].max()))
# points inside cavity region y>-0.33 and within body footprint
cav=(Y>-0.33)&(Y<-0.09)&(X>-0.29)&(X<0.06)&(Z>0.9)&(Z<1.11)
print("cavity pts n=%d x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"%(cav.sum(),X[cav].min(),X[cav].max(),Y[cav].min(),Y[cav].max(),Z[cav].min(),Z[cav].max()))
EOF

# openrua op 26
timeout 60 python3 tools/perception/cam_snap.py birdview snaps/birdview2.png && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw snaps/birdview_depth.png >/dev/null && python3 - <<'EOF'
import numpy as np
D=np.load('snaps/birdview_depth.npy'); fx=579.4112549695428
vv,uu=np.mgrid[0:480,0:640]; X=-0.2+(vv-240)*D/fx; Y=(uu-320)*D/fx; Z=3.0-D
door=(Z>0.95)&(Z<1.15)&(Y<-0.355)&(X>-0.6)&(X<0.2)
print("door pts x[%.3f,%.3f] y[%.3f,%.3f] n=%d"%(X[door].min(),X[door].max(),Y[door].min(),Y[door].max(),door.sum()))
pts=np.c_[X[door],Y[door]]
# PCA line fit
c=pts.mean(0); u,s,vt=np.linalg.svd(pts-c); d=vt[0]
proj=(pts-c)@d
print("centre",c.round(3),"dir",d.round(3),"ends",(c+d*proj.min()).round(3),(c+d*proj.max()).round(3))
EOF

# openrua op 27
python3 - <<'EOF'
import numpy as np
P=np.load('snaps/eih_xyz.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
inside=(Y>-0.33)&(Y<-0.10)&(X>-0.30)&(X<0.08)&(Z>0.85)&(Z<1.12)
print("interior floor-ish (Z<0.95) : x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"%tuple(np.r_[X[inside&(Z<0.95)].min(),X[inside&(Z<0.95)].max(),Y[inside&(Z<0.95)].min(),Y[inside&(Z<0.95)].max(),Z[inside&(Z<0.95)].min(),Z[inside&(Z<0.95)].max()]))
for ylo in np.arange(-0.33,-0.10,0.02):
    s=inside&(Y>=ylo)&(Y<ylo+0.02)
    if s.sum()>20:
        print("y %.2f: n=%5d x[%.3f,%.3f] z[%.3f,%.3f]  z-hist:"%(ylo,s.sum(),X[s].min(),X[s].max(),Z[s].min(),Z[s].max()), np.histogram(Z[s],bins=[0.85,0.91,0.93,0.95,1.0,1.03,1.05,1.07,1.09,1.12])[0])
# ceiling: points with z>1.0 inside cavity and y>-0.30
c=inside&(Y>-0.30)&(Z>0.98)&(Z<1.10)
print("ceiling-ish z percentiles", np.percentile(Z[c],[5,50,95]) if c.sum() else None)
# walls: x extents at mid height
w=inside&(Y>-0.30)&(Z>0.93)&(Z<1.03)
print("mid-height x percentiles", np.percentile(X[w],[1,5,50,95,99]))
EOF

# openrua op 28
python3 - <<'EOF'
import numpy as np
P=np.load('snaps/eih_xyz.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
face=(Y>-0.37)&(Y<-0.335)&(X>-0.24)&(X<-0.10)&(Z>0.85)
zs=Z[face]; print("face z hist", np.histogram(zs,bins=np.arange(0.88,1.12,0.01)))
# lintel: min z of face points above 1.0
print("lintel bottom ~", np.percentile(zs[zs>1.0],[1,5]))
# floor lip: face points below 0.95
print("lip z range", zs[zs<0.96].min() if (zs<0.96).any() else None, zs[zs<0.96].max() if (zs<0.96).any() else None)
# opening side edges: face points at mid height 0.96-1.02
m=(Y>-0.37)&(Y<-0.335)&(Z>0.96)&(Z<1.02)&(X>-0.5)&(X<0.1)
print("side frame x values at mid height:", np.unique(np.round(X[m],2)))
EOF

# openrua op 29
timeout 300 python3 -u - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
for th,p in ((20,[-0.135,-0.56,1.12]),(25,[-0.135,-0.52,1.14]),(30,[-0.16,-0.50,1.16])):
    R1=R_from_axes(z=[0,np.cos(np.radians(th)),-np.sin(np.radians(th))], y=[1,0,0])
    ok=a.move_hand(p, R1, 4.0); print("moved", ok, p)
    if ok: break
pos,R=a.hand_pose(); print("hand pos", pos.round(3)); print(R.round(3))
a.snap("robot0_eye_in_hand","snaps/eih_front.png")
EOF
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_depth2.png

# openrua op 30
timeout 120 python3 - 2>/dev/null <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
rclpy.init(); node=rclpy.create_node('pc')
buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(CameraInfo,'/robot0_eye_in_hand/color/camera_info',lambda m: got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(node,timeout_sec=0.2)
fr='robot0_eye_in_hand_optical_frame'
while not buf.can_transform('world',fr,Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',fr,Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
k=got['m'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
D=np.load('snaps/eih_depth2.npy'); h,wd=D.shape
vv,uu=np.mgrid[0:h,0:wd]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D],-1)@R.T+tr
np.save('snaps/eih_xyz2.npy',P)
X,Y,Z=P[...,0],P[...,1],P[...,2]
print("cam", tr.round(3))
face=(Y>-0.37)&(Y<-0.335)&(X>-0.24)&(X<-0.05)&(Z>0.85)
print("face z hist", np.histogram(Z[face],bins=np.arange(0.88,1.12,0.01))[0])
# interior ceiling: points y>-0.33 inside footprint with z>1.0
inside=(Y>-0.33)&(Y<-0.10)&(X>-0.26)&(X<0.0)&(Z>1.0)&(Z<1.104)
print("interior high pts n=%d"%inside.sum(), np.histogram(Z[inside],bins=np.arange(1.0,1.11,0.01))[0] if inside.sum() else "")
if inside.sum(): print(" y range", Y[inside].min().round(3), Y[inside].max().round(3), "x range", X[inside].min().round(3), X[inside].max().round(3))
# door now
door=(Y<-0.355)&(Z>0.95)&(Z<1.12)&(X>-0.5)&(X<0.2)
pts=np.c_[X[door],Y[door]]; c=pts.mean(0); u,s,vt=np.linalg.svd(pts-c); d=vt[0]; proj=(pts-c)@d
print("door ends",(c+d*proj.min()).round(3),(c+d*proj.max()).round(3), "z", Z[door].min().round(3), Z[door].max().round(3))
EOF

# openrua op 31
timeout 60 python3 tools/perception/cam_snap.py /paperview/depth/image_raw snaps/paperview_depth.png >/dev/null; timeout 120 python3 - 2>/dev/null <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
rclpy.init(); node=rclpy.create_node('pc')
buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(CameraInfo,'/paperview/color/camera_info',lambda m: got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(node,timeout_sec=0.2)
fr='paperview_optical_frame'
while not buf.can_transform('world',fr,Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',fr,Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
k=got['m'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
D=np.load('snaps/paperview_depth.npy'); h,wd=D.shape
vv,uu=np.mgrid[0:h,0:wd]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D],-1)@R.T+tr
X,Y,Z=P[...,0],P[...,1],P[...,2]
print("cam", tr.round(3), "fx", fx)
face=(Y>-0.38)&(Y<-0.33)&(X>-0.24)&(X<-0.05)&(Z>0.85)&(Z<1.15)
print("face pts n", face.sum(), "z hist", np.histogram(Z[face],bins=np.arange(0.88,1.13,0.01))[0])
# interior points (inside footprint) - ceiling?
inside=(Y>-0.33)&(Y<-0.10)&(X>-0.26)&(X<0.0)&(Z>0.9)&(Z<1.104)
print("interior n", inside.sum(), np.histogram(Z[inside],bins=np.arange(0.9,1.11,0.01))[0])
# also pixels: where does the face fall in the image?
ys,xs=np.where(face); print("face pixels u[%d,%d] v[%d,%d]"%(xs.min(),xs.max(),ys.min(),ys.max()) if face.sum() else "")
EOF

# openrua op 32
timeout 60 python3 - 2>/dev/null <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open('snaps/robot.urdf','w').write(got[0].data)
print(len(got[0].data))
EOF
grep -n -A12 '<link name="panda_hand"' snaps/robot.urdf | head -40; grep -n -A12 '<link name="panda_leftfinger"' snaps/robot.urdf | head -30; grep -n -B2 -A6 'name="panda_finger_joint1"' snaps/robot.urdf | head -20; grep -n -B2 -A8 'name="panda_hand_joint"' snaps/robot.urdf | head

# openrua op 33
python3 - <<'EOF'
import numpy as np, struct
def stl_bounds(path):
    d=open(path,'rb').read()
    if d[:5]==b'solid' and b'facet' in d[:300]:
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',d)])
    else:
        n=struct.unpack_from('<I',d,80)[0]
        arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0),v.max(0)
base='/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/'
for f in ['hand.stl','finger.stl','link7.stl']:
    lo,hi=stl_bounds(base+f); print(f,"min",lo.round(4),"max",hi.round(4))
EOF
grep -o '<joint name="panda_finger_joint1"[^>]*>.\{0,400\}' snaps/robot.urdf | grep -o 'origin[^/]*/' | head -2; grep -o '<joint name="panda_hand_joint"[^>]*>.\{0,200\}' snaps/robot.urdf | grep -o 'origin[^/]*/'

# openrua op 34
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ 2>/dev/null; find / -iname "hand*.stl" -o -iname "finger*.stl" 2>/dev/null | head

# openrua op 35
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Scene measurements from the birdview depth camera (top-down)."""
import numpy as np, rclpy
from sensor_msgs.msg import Image

FX = 579.4112549695428
HINGE = np.array([-0.290, -0.365])   # microwave door hinge (world x,y)
BOX = dict(x=(-0.293, 0.058), y=(-0.35, -0.09), top=1.107, floor=0.944,
           open_x=(-0.255, -0.046))  # cavity opening on the -y face


def birdview_cloud(node=None):
    own = node is None
    if own:
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node("bird")
    got = []
    sub = node.create_subscription(Image, "/birdview/depth/image_raw", got.append, 1)
    while not got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    m = got[0]
    D = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
    vv, uu = np.mgrid[0:m.height, 0:m.width]
    X = -0.2 + (vv - 240) * D / FX
    Y = (uu - 320) * D / FX
    Z = 3.0 - D
    if own:
        node.destroy_node()
    return X, Y, Z


def door_state(X, Y, Z):
    """Door opening angle (deg, 0 = closed along +x) and tip point."""
    m = (Z > 0.95) & (Z < 1.15) & (Y < -0.355) & (X > -0.7) & (X < 0.2)
    # exclude the robot arm: keep points within 0.30 m of the hinge
    r = np.hypot(X - HINGE[0], Y - HINGE[1])
    m &= (r < 0.30) & (r > 0.03)
    if m.sum() < 20:
        return None, None
    pts = np.c_[X[m], Y[m]]
    d = pts - HINGE
    far = d[np.argsort(np.hypot(*d.T))[-40:]].mean(0)
    ang = np.degrees(np.arctan2(-far[1], far[0]))  # cw from +x
    return ang, HINGE + far


def mug_state(X, Y, Z, region=None):
    """Yellow/white mug rim circle (centre x,y, radius, rim z)."""
    if region is None:
        region = (X > -0.15, X < 0.2, Y > -0.15, Y < 0.15)
    m = np.ones_like(Z, bool)
    for c in region:
        m &= c
    m &= (Z > 0.93) & (Z < 1.2)
    if m.sum() < 20:
        return None
    ztop = np.percentile(Z[m], 98)
    rim = m & (Z > ztop - 0.012)
    pts = np.c_[X[rim], Y[rim]]
    A = np.c_[2 * pts, np.ones(len(pts))]; b = (pts ** 2).sum(1)
    c = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = c[0], c[1]; r = np.sqrt(c[2] + cx * cx + cy * cy)
    return np.array([cx, cy]), r, ztop


if __name__ == "__main__":
    X, Y, Z = birdview_cloud()
    ang, tip = door_state(X, Y, Z)
    print(f"door angle {ang:.1f} deg, tip {tip.round(3)}")
    ms = mug_state(X, Y, Z)
    print("mug", None if ms is None else (ms[0].round(3), round(ms[1], 3), round(ms[2], 3)))
    ms2 = mug_state(X, Y, Z, region=(X > -0.30, X < 0.06, Y > -0.35, Y < -0.09))
    print("mug-in-cavity-region", None if ms2 is None else (ms2[0].round(3), round(ms2[1], 3), round(ms2[2], 3)))
OPENRUA_EOF

# openrua op 36
timeout 60 python3 scene.py 2>/dev/null

# openrua op 37
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/scene.py", "old_string": "        region = (X > -0.15, X < 0.2, Y > -0.15, Y < 0.15)", "new_string": "        region = (X > -0.15, X < 0.2, Y > -0.08, Y < 0.15)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 38
mkdir -p "$(dirname /workspace/door.py)"
cat > /workspace/door.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Door helpers: geometry of the push poses + the sweep."""
import sys
import numpy as np
from arm import Arm, R_from_axes
from scene import HINGE, birdview_cloud, door_state

Z_TIPS = 1.02        # push height on the door face (door spans 0.90..1.107)
Z_CLEAR = 1.20       # tips height when travelling above the box (top 1.107)


def d_n(alpha_deg):
    a = np.radians(alpha_deg)
    d = np.array([np.cos(a), -np.sin(a), 0.0])     # door direction from hinge
    n = np.array([np.sin(a), np.cos(a), 0.0])      # inner normal (+y when closed)
    return d, n


def push_pose(alpha_deg, r, side, off=0.02, z=Z_TIPS):
    """Tips point + hand rotation for pushing the door at angle alpha.
    side=+1: fingers on the inner face (opening), -1: on the outer face (closing)."""
    d, n = d_n(alpha_deg)
    tips = np.array([HINGE[0], HINGE[1], z]) + r * d + side * off * n
    R = R_from_axes(z=[0, 0, -1], y=d)
    return tips, R


def hand_from_tips(tips, R):
    return np.asarray(tips) - 0.1034 * R[:, 2]


def sweep(a, alphas, rs, side, off=0.02, secs_per=0.7):
    """IK every waypoint (seeded by the previous), check continuity, run."""
    q = a.q(); pts = []
    for al, r in zip(alphas, rs):
        tips, R = push_pose(al, r, side, off)
        sol = a.solve_ik(hand_from_tips(tips, R), R, seed=q)
        if sol is None:
            print(f"  no IK at alpha={al:.1f}"); return False
        jump = np.abs(sol - q).max()
        if jump > 0.6:
            print(f"  joint jump {jump:.2f} at alpha={al:.1f}; abort"); return False
        pts.append(sol); q = sol
    code, err = a.traj(pts, [secs_per * (i + 1) for i in range(len(pts))])
    return err < 0.05


if __name__ == "__main__":
    pass
OPENRUA_EOF

# openrua op 39
timeout 600 python3 -u - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm
from door import *
a=Arm()
# 1. close the fingers for pushing
a.gripper(0.0)
# 2. lift clear first (current pose is tilted near the box)
pos,R=a.hand_pose(); print("hand now", pos.round(3))
alpha0=12.2
tips,Rp=push_pose(alpha0, 0.235, +1, off=0.024, z=Z_CLEAR)
print("tips above wedge", tips.round(3))
ok=a.move_hand(hand_from_tips(tips,Rp), Rp, 4.0); print("above wedge", ok)
a.snap("robot0_eye_in_hand","snaps/eih_wedge.png")
pos,R=a.hand_pose(); print("hand", pos.round(3)); print(R.round(2))
EOF

# openrua op 40
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_depth3.png >/dev/null; timeout 120 python3 - 2>/dev/null <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
rclpy.init(); node=rclpy.create_node('pc')
buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(CameraInfo,'/robot0_eye_in_hand/color/camera_info',lambda m: got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(node,timeout_sec=0.2)
fr='robot0_eye_in_hand_optical_frame'
while not buf.can_transform('world',fr,Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',fr,Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
k=got['m'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
D=np.load('snaps/eih_depth3.npy'); h,wd=D.shape
vv,uu=np.mgrid[0:h,0:wd]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D],-1)@R.T+tr
X,Y,Z=P[...,0],P[...,1],P[...,2]
print("cam",tr.round(3))
H=np.array([-0.29,-0.365]); a=np.radians(12.2); d=np.array([np.cos(a),-np.sin(a)]); n=np.array([np.sin(a),np.cos(a)])
rr=(X-H[0])*d[0]+(Y-H[1])*d[1]; nn=(X-H[0])*n[0]+(Y-H[1])*n[1]
top=(Z>1.08)&(Z<1.13)&(Y<-0.30)
for r0 in (0.10,0.14,0.18,0.20,0.22,0.24,0.26):
    s=top&(rr>r0-0.01)&(rr<r0+0.01)
    if s.sum(): 
        vals=np.sort(nn[s]); 
        # cluster: door (n small) vs box (n larger)
        print("r=%.2f  n values: door-ish [%.3f,%.3f]  all max %.3f  z[%.3f,%.3f] n=%d"%(r0, vals[vals<0.035].min() if (vals<0.035).any() else np.nan, vals[vals<0.035].max() if (vals<0.035).any() else np.nan, vals.max(), Z[s].min(), Z[s].max(), s.sum()))
# the block: points with z between 1.0 and 1.08 near the tip
blk=(Z>0.96)&(Z<1.10)&(rr>0.14)&(rr<0.30)&(nn>-0.05)&(nn<0.10)
print("mid-height pts near tip: n=%d"%blk.sum())
if blk.sum():
    for zlo in (0.96,1.0,1.04,1.08):
        s=blk&(Z>=zlo)&(Z<zlo+0.04)
        if s.sum(): print("  z %.2f: r[%.3f,%.3f] n[%.3f,%.3f] cnt %d"%(zlo, rr[s].min(),rr[s].max(),nn[s].min(),nn[s].max(),s.sum()))
np.save('snaps/eih_xyz3.npy',P)
EOF

# openrua op 41
timeout 60 python3 - <<'EOF'
import numpy as np
P=np.load('snaps/eih_xyz3.npy')
for (u,v) in [(360,220),(400,220),(450,220),(480,220),(400,200),(400,240),(400,260),(400,300),(400,180),(300,280),(300,300),(300,330),(300,360),(300,380)]:
    print((u,v), P[v,u].round(3))
EOF

# openrua op 42
sed -i 's/^Z_TIPS = 1.02 .*/Z_TIPS = 1.06        # push height: fingers overlap door top (1.107), hand body clears box top/' door.py && timeout 300 python3 - <<'EOF'
import numpy as np
from arm import Arm
from door import *
a=Arm()
print("fingers",a.fingers())
tips,R=push_pose(12.2,0.235,+1,off=0.04,z=1.06)
print("tips",tips.round(3))
ok=a.move_hand(hand_from_tips(tips,R),R,2.5); print("descend",ok)
print("wrench",a.wrench().round(1))
p,Rh=a.hand_pose(); print("hand",p.round(3),"tips",(p+0.1034*Rh[:,2]).round(3))
EOF

# openrua op 43
timeout 120 python3 scene.py 2>/dev/null

# openrua op 44
timeout 600 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm
from door import *
a=Arm()
alphas=np.arange(20,136,8.0); n=len(alphas)
rs=np.linspace(0.23,0.19,n)
ok=sweep(a,alphas,rs,+1,off=0.035,secs_per=1.0); print("sweep",ok)
print("wrench",a.wrench().round(1))
p,Rh=a.hand_pose(); print("hand",p.round(3),"tips",(p+0.1034*Rh[:,2]).round(3))
EOF

# openrua op 45
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm
from door import *
from scene import *
a=Arm()
p,Rh=a.hand_pose()
ok=a.move_hand(p+[0,0,0.15],Rh,2.0); print("lift",ok, "wrench",a.wrench().round(1))
X,Y,Z=birdview_cloud(a.node)
ang,tip=door_state(X,Y,Z); print("door",ang,tip)
a.snap('birdview','snaps/bird_open.png')
EOF

# openrua op 46
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
R=R_from_axes(z=[0,1,0], y=[1,0,0])
q=a.q()
for name,tips in [("pregrasp",(0.0185,-0.14,0.965)),("grasp",(0.0185,-0.058,0.962)),("lift",(0.0185,-0.058,1.05)),("transit",(-0.15,-0.42,1.05)),("insert_start",(-0.15,-0.42,0.965)),("insert_end",(-0.15,-0.355,0.965)),("insert_deeper",(-0.15,-0.33,0.965))]:
    sol=a.solve_ik(np.array(tips)-0.1034*R[:,2],R,seed=q)
    print(name, None if sol is None else sol.round(2))
EOF

# openrua op 47
ls -la snaps/ | head -50; ls tools/*

# openrua op 48
timeout 60 python3 - <<'EOF'
import numpy as np
for f in ['snaps/eih_xyz2.npy','snaps/eih_xyz.npy']:
    P=np.load(f); X,Y,Z=P[...,0],P[...,1],P[...,2]
    # points inside the box footprint with the cavity (x in opening range)
    m=(X>-0.25)&(X<-0.05)&(Y>-0.33)&(Y<-0.12)&np.isfinite(Z)
    print(f, m.sum())
    if m.sum():
        z=Z[m]
        hist,edges=np.histogram(z,bins=np.arange(0.90,1.15,0.01))
        for h,e in zip(hist,edges): 
            if h>20: print("  z %.2f: %d"%(e,h))
        # ceiling: points near top
        top=m&(Z>1.05)
        if top.sum(): print("  ceiling-ish pts z range",Z[top].min().round(3),Z[top].max().round(3),"y range",Y[top].min().round(3),Y[top].max().round(3))
EOF

# openrua op 49
timeout 60 python3 - <<'EOF'
import numpy as np
for f in ['snaps/eih_xyz2.npy','snaps/eih_xyz.npy','snaps/robotview_xyz.npy']:
    P=np.load(f); X,Y,Z=P[...,0],P[...,1],P[...,2]
    face=(Y>-0.365)&(Y<-0.34)&(X>-0.24)&(X<-0.06)&np.isfinite(Z)&(Z>0.9)&(Z<1.12)
    print(f,"face pts",face.sum())
    if face.sum():
        z=Z[face]; hist,edges=np.histogram(z,bins=np.arange(0.90,1.12,0.005))
        print("  ",[(round(e,3),h) for h,e in zip(hist,edges) if h>5])
EOF

# openrua op 50
cat >> arm.py <<'EOF'


def look_pose(cam, target, y_axis=(1, 0, 0)):
    """Hand pose that puts the eye-in-hand camera at `cam` looking at `target`.
    Camera sits at +0.05 along hand x; image right = hand y."""
    cam = np.asarray(cam, float); z = np.asarray(target, float) - cam
    R = R_from_axes(z=z, y=y_axis)
    return cam - 0.05 * R[:, 0], R
EOF
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, look_pose
a=Arm()
for cam,tgt in [((-0.15,-0.62,1.10),(-0.15,-0.35,1.02)),((-0.15,-0.55,1.20),(-0.15,-0.32,1.00)),((-0.15,-0.50,1.28),(-0.15,-0.28,1.00))]:
    p,R=look_pose(cam,tgt)
    sol=a.solve_ik(p,R)
    print(cam, None if sol is None else "ok")
    if sol is not None:
        ok=a.move_hand(p,R,3.0); print(" moved",ok); a.snap('robot0_eye_in_hand','snaps/eih_lintel.png'); break
EOF

# openrua op 51
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, look_pose
a=Arm()
done=False
for cam,tgt in [((-0.15,-0.55,1.20),(-0.15,-0.32,1.00)),((-0.15,-0.50,1.28),(-0.15,-0.28,1.00)),((-0.20,-0.48,1.30),(-0.15,-0.28,1.00))]:
    for ya in [(-1,0,0),(0,0,1),(0,0,-1),(1,0,1),(-1,0,1)]:
        p,R=look_pose(cam,tgt,ya)
        sol=a.solve_ik(p,R)
        print(cam, ya, None if sol is None else "ok")
        if sol is not None:
            ok=a.move_hand(p,R,3.0); print(" moved",ok); a.snap('robot0_eye_in_hand','snaps/eih_lintel.png'); done=True; break
    if done: break
EOF

# openrua op 52
timeout 120 python3 px_batch.py robot0_eye_in_hand 236,240 240,240 244,240 248,240 252,240 300,240 360,240 368,240 372,240 376,240 380,240 300,120 300,160 300,300 250,300 350,300 2>/dev/null

# openrua op 53
timeout 120 python3 - 2>/dev/null <<'EOF'
import numpy as np
from scene import *
X,Y,Z=birdview_cloud()
c,r,zt=mug_state(X,Y,Z); print("mug",c.round(4),round(r,4),round(zt,4))
m=(X>c[0]-0.06)&(X<c[0]+0.06)&(Y>c[1]-0.12)&(Y<c[1]-0.04)&(Z>0.92)&(Z<0.999)
print("handle pts",m.sum())
if m.sum():
    print(" x[%.4f,%.4f] y[%.4f,%.4f] z[%.3f,%.3f]"%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
    for zlo in np.arange(0.92,1.0,0.01):
        s=m&(Z>=zlo)&(Z<zlo+0.01)
        if s.sum(): print("  z%.2f: n=%d x[%.4f,%.4f] y[%.4f,%.4f]"%(zlo,s.sum(),X[s].min(),X[s].max(),Y[s].min(),Y[s].max()))
# mug body radius vs height (outer wall from above: only rim visible) 
EOF

# openrua op 54
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
Rv=R_from_axes(z=[0,0,-1], y=[1,0,0])
th=np.radians(10); Rt=R_from_axes(z=[0,np.sin(th),-np.cos(th)], y=[1,0,0])
def tips2hand(t,R): return np.array(t)-0.1034*R[:,2]
q=a.q()
poses=[("pre",(0.0185,-0.046,1.10),Rv),("grasp",(0.0185,-0.046,0.966),Rv),("lift",(0.0185,-0.046,1.25),Rv),
       ("over_ins",(-0.14,-0.458,1.25),Rv),("ins_out",(-0.14,-0.458,1.02),Rv),("ins_in",(-0.14,-0.358,1.02),Rv),
       ("nudge0",(-0.14,-0.43,1.00),Rt),("nudge1",(-0.14,-0.365,1.00),Rt)]
prev=q
for name,t,R in poses:
    sol=a.solve_ik(tips2hand(t,R),R,seed=prev)
    print(name, None if sol is None else (sol.round(2), "jump %.2f"%np.abs(sol-prev).max()))
    if sol is not None: prev=sol
EOF

# openrua op 55
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
print("current q",a.q().round(2))
Rv=R_from_axes(z=[0,0,-1], y=[1,0,0])
th=np.radians(10); Rt=R_from_axes(z=[0,np.sin(th),-np.cos(th)], y=[1,0,0])
def tips2hand(t,R): return np.array(t)-0.1034*R[:,2]
poses=[("pre",(0.0185,-0.046,1.10),Rv),("grasp",(0.0185,-0.046,0.966),Rv),("lift",(0.0185,-0.046,1.25),Rv),
       ("over_ins",(-0.14,-0.458,1.25),Rv),("ins_out",(-0.14,-0.458,1.02),Rv),("ins_in",(-0.14,-0.358,1.02),Rv),
       ("nudge0",(-0.14,-0.43,1.00),Rt),("nudge1",(-0.14,-0.365,1.00),Rt)]
for seed in ([0,-0.3,0,-2.0,0,1.7,0.8],[-0.1,0.3,0,-1.8,0,2.1,0.7]):
    prev=np.array(seed,float); print("seed",seed)
    for name,t,R in poses:
        sol=a.solve_ik(tips2hand(t,R),R,seed=prev)
        print(" ",name, None if sol is None else (sol.round(2), "jump %.2f"%np.abs(sol-prev).max()))
        if sol is not None: prev=sol
EOF

# openrua op 56
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np, rclpy
from arm import Arm, R_from_axes, JOINTS
from moveit_msgs.srv import GetPositionFK
a=Arm()
def fk_links(q, links=("panda_link4","panda_link5","panda_link6","panda_link7","panda_hand")):
    req=GetPositionFK.Request(); req.fk_link_names=list(links)
    req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=list(map(float,q))
    fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=30)
    r=fut.result(); return {l:np.array([p.pose.position.x,p.pose.position.y,p.pose.position.z]) for l,p in zip(links,r.pose_stamped)}
Rv=R_from_axes(z=[0,0,-1], y=[1,0,0])
q0=a.q(); p0,R0=a.hand_pose()
q1=a.solve_ik(p0+[0,0,0.25],R0,seed=q0)        # lift straight up
q2=a.solve_ik(np.array([0.0185,-0.046,1.35])-0.1034*Rv[:,2],Rv,seed=[0,-0.3,0,-2.0,0,1.7,0.8])
print("q1",None if q1 is None else q1.round(2)); print("q2",None if q2 is None else q2.round(2))
def boxhit(p):  # inside box/door danger zone?
    return (p[0]>-0.45 and p[0]<0.1 and p[1]>-0.6 and p[1]<-0.05 and p[2]<1.2)
for name,(qa,qb) in [("lift",(q0,q1)),("swing",(q1,q2))]:
    print(name)
    for s in np.linspace(0,1,9):
        q=qa+(qb-qa)*s; L=fk_links(q)
        flag=[l for l,p in L.items() if boxhit(p)]
        print("  s=%.2f hand %s  minz %.2f  %s"%(s,L["panda_hand"].round(2),min(p[2] for p in L.values()), "HIT "+",".join(flag) if flag else ""))
np.save('/tmp/q12.npy',np.array([q1,q2]))
EOF

# openrua op 57
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np, rclpy
from arm import Arm, R_from_axes, JOINTS
from moveit_msgs.srv import GetPositionFK
a=Arm()
def fk_links(q, links=("panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand")):
    req=GetPositionFK.Request(); req.fk_link_names=list(links)
    req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=list(map(float,q))
    fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=30)
    r=fut.result(); return {l:np.array([p.pose.position.x,p.pose.position.y,p.pose.position.z]) for l,p in zip(links,r.pose_stamped)}
Rv=R_from_axes(z=[0,0,-1], y=[1,0,0])
q0=a.q(); p0,R0=a.hand_pose(); print("hand now",p0.round(3))
q1=a.solve_ik(p0+[0.05,0.05,0.12],R0,seed=q0)
q2=a.solve_ik(np.array([0.0185,-0.046,1.35])-0.1034*Rv[:,2],Rv,seed=[0,-0.3,0,-2.0,0,1.7,0.8])
print("q1",None if q1 is None else q1.round(2)); print("q2",q2.round(2))
def boxhit(p): return (p[0]>-0.45 and p[0]<0.1 and p[1]>-0.6 and p[1]<-0.05 and p[2]<1.2)
paths=[("direct",(q0,q2))]+([("lift",(q0,q1)),("swing",(q1,q2))] if q1 is not None else [])
for name,(qa,qb) in paths:
    print(name)
    for s in np.linspace(0,1,9):
        q=qa+(qb-qa)*s; L=fk_links(q)
        flag=[l for l,p in L.items() if boxhit(p)]
        print("  s=%.2f hand %s  minz %.2f  %s"%(s,L["panda_hand"].round(2),min(p[2] for p in L.values()), "HIT "+",".join(flag) if flag else ""))
np.save('/tmp/q2.npy',q2)
EOF

# openrua op 58
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
q2=np.load('/tmp/q2.npy')
code,err=a.traj([q2],4.0)
if err>0.02: code,err=a.traj([q2],2.0)
p,R=a.hand_pose(); print("hand",p.round(3),"tips",(p+0.1034*R[:,2]).round(3))
print("R",R.round(2))
a.gripper(0.04)
a.snap('robot0_eye_in_hand','snaps/eih_mug_top.png')
EOF

# openrua op 59
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm
a=Arm()
q2=np.load('/tmp/q2.npy')
for i in range(4):
    code,err=a.traj([q2],3.0)
    if err<0.02: break
p,R=a.hand_pose(); print("hand",p.round(3),"tips",(p+0.1034*R[:,2]).round(3)); print(R.round(2))
a.snap('robot0_eye_in_hand','snaps/eih_mug_top.png')
EOF

# openrua op 60
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
Rv=R_from_axes(z=[0,0,-1], y=[1,0,0])
def T(t): return np.array(t)-0.1034*Rv[:,2]
print("w0",a.wrench().round(1))
print(a.move_hand(T((0.0185,-0.056,1.05)),Rv,3.0), a.wrench().round(1))
print(a.move_hand(T((0.0185,-0.056,0.966)),Rv,2.0), a.wrench().round(1))
p,R=a.hand_pose(); print("tips",(p+0.1034*R[:,2]).round(4))
a.snap('robot0_eye_in_hand','snaps/eih_pregrasp.png')
EOF

# openrua op 61
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm
a=Arm()
f=a.gripper(0.0)
print("wrench",a.wrench().round(1))
EOF

# openrua op 62
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
from scene import *
a=Arm()
Rv=R_from_axes(z=[0,0,-1], y=[1,0,0])
def T(t): return np.array(t)-0.1034*Rv[:,2]
print(a.move_hand(T((0.0185,-0.056,1.05)),Rv,2.5), a.wrench().round(1), a.fingers())
X,Y,Z=birdview_cloud(a.node)
ms=mug_state(X,Y,Z); print("mug",ms[0].round(3),round(ms[1],3),"rim z",round(ms[2],3))
a.snap('robot0_eye_in_hand','snaps/eih_lifted.png')
EOF

# openrua op 63
timeout 600 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
Rv=R_from_axes(z=[0,0,-1], y=[1,0,0])
def T(t): return np.array(t)-0.1034*Rv[:,2]
steps=[("lift",(0.0185,-0.056,1.25),3.0),("transit",(-0.14,-0.473,1.25),4.0),("descend",(-0.14,-0.473,1.02),3.0),("insert",(-0.14,-0.373,1.02),3.0),("lower",(-0.14,-0.373,1.012),1.5)]
for name,t,s in steps:
    ok=a.move_hand(T(t),Rv,s)
    p,R=a.hand_pose()
    print(name, ok, "tips",(p+0.1034*R[:,2]).round(3),"wrench",a.wrench().round(1),"fingers %.4f"%a.fingers()[0])
    if not ok: break
a.snap('robot0_eye_in_hand','snaps/eih_inserted.png')
EOF

# openrua op 64
timeout 120 python3 - 2>/dev/null <<'EOF'
from arm import Arm
a=Arm(); a.snap('birdview','snaps/bird_ins.png'); a.snap('agentview','snaps/agent_ins.png')
EOF
echo done

# openrua op 65
timeout 120 python3 - 2>/dev/null <<'EOF'
from arm import Arm
a=Arm()
for c in ['frontview','sideview','galleryview','paperview','robot0_robotview']:
    try: a.snap(c,f'snaps/{c}_ins.png')
    except Exception as e: print(c,e)
EOF
echo ok

# openrua op 66
timeout 600 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
Rv=R_from_axes(z=[0,0,-1], y=[1,0,0])
th=np.radians(14); Rt=R_from_axes(z=[0,np.sin(th),-np.cos(th)], y=[1,0,0])
def T(t,R): return np.array(t)-0.1034*R[:,2]
for name,t,R,s in [("retract",(-0.14,-0.42,1.04),Rv,3.0),("tilt",(-0.14,-0.42,1.04),Rt,2.5)]:
    ok=a.move_hand(T(t,R),R,s); p,Rh=a.hand_pose()
    print(name,ok,"tips",(p+0.1034*Rh[:,2]).round(3),"wrench",a.wrench().round(1),"fingers %.4f"%a.fingers()[0])
a.snap('robot0_eye_in_hand','snaps/eih_tilted.png')
EOF

# openrua op 67
timeout 600 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
th=np.radians(14); Rt=R_from_axes(z=[0,np.sin(th),-np.cos(th)], y=[1,0,0])
def T(t,R): return np.array(t)-0.1034*R[:,2]
for name,t,s in [("insert_a",(-0.14,-0.39,1.03),3.0),("insert_b",(-0.14,-0.355,1.03),3.0)]:
    ok=a.move_hand(T(t,Rt),Rt,s); p,Rh=a.hand_pose()
    print(name,ok,"tips",(p+0.1034*Rh[:,2]).round(3),"wrench",a.wrench().round(1),"fingers %.4f"%a.fingers()[0])
    if not ok: break
a.snap('robot0_eye_in_hand','snaps/eih_ins2.png')
EOF

# openrua op 68
timeout 600 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
th=np.radians(14); Rt=R_from_axes(z=[0,np.sin(th),-np.cos(th)], y=[1,0,0])
def T(t,R): return np.array(t)-0.1034*R[:,2]
ok=a.move_hand(T((-0.14,-0.355,1.015),Rt),Rt,2.0); p,Rh=a.hand_pose()
print("lower",ok,"tips",(p+0.1034*Rh[:,2]).round(3),"wrench",a.wrench().round(1))
a.gripper(0.04)
ok=a.move_hand(T((-0.14,-0.43,1.03),Rt),Rt,2.5); p,Rh=a.hand_pose()
print("retract",ok,"tips",(p+0.1034*Rh[:,2]).round(3),"wrench",a.wrench().round(1))
a.snap('robot0_eye_in_hand','snaps/eih_released.png')
EOF

# openrua op 69
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_depth4.png >/dev/null; timeout 120 python3 - 2>/dev/null <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
rclpy.init(); node=rclpy.create_node('pc')
buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(CameraInfo,'/robot0_eye_in_hand/color/camera_info',lambda m: got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(node,timeout_sec=0.2)
fr='robot0_eye_in_hand_optical_frame'
while not buf.can_transform('world',fr,Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',fr,Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
k=got['m'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
D=np.load('snaps/eih_depth4.npy'); h,wd=D.shape
vv,uu=np.mgrid[0:h,0:wd]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D],-1)@R.T+tr
X,Y,Z=P[...,0],P[...,1],P[...,2]
np.save('snaps/eih_xyz4.npy',P)
# mug rim: highest points in the cavity region excluding box top
m=(X>-0.25)&(X<-0.05)&(Y>-0.40)&(Y<-0.15)&(Z>1.0)&(Z<1.08)
print("rim-ish pts",m.sum(), "z range",Z[m].min().round(3),Z[m].max().round(3))
zt=np.percentile(Z[m],98); rim=m&(Z>zt-0.01)
pts=np.c_[X[rim],Y[rim]]; A=np.c_[2*pts,np.ones(len(pts))]; b=(pts**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cxm,cym=c[0],c[1]; r=np.sqrt(c[2]+cxm**2+cym**2)
print("rim fit centre (%.4f,%.4f) r=%.4f ztop=%.4f"%(cxm,cym,r,zt))
# handle
hm=(X>cxm-0.03)&(X<cxm+0.03)&(Y<cym-0.05)&(Y>cym-0.12)&(Z>0.95)&(Z<1.04)
print("handle pts",hm.sum(), "y[%.3f,%.3f] z[%.3f,%.3f]"%(Y[hm].min(),Y[hm].max(),Z[hm].min(),Z[hm].max()) if hm.sum() else "")
# floor edge visible?
fl=(Z>0.94)&(Z<0.95)&(X>-0.25)&(X<-0.05)
print("floor pts",fl.sum(),"y range",Y[fl].min().round(3),Y[fl].max().round(3))
EOF

# openrua op 70
timeout 600 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
th=np.radians(30); Rp=R_from_axes(z=[0,np.sin(th),-np.cos(th)], y=[1,0,0])
def T(t,R): return np.array(t)-0.1034*R[:,2]
a.gripper(0.0)
for name,t,s in [("pre",(-0.146,-0.44,1.06),Rp and 3.0),("down",(-0.146,-0.44,1.00),2.0),("push",(-0.146,-0.323,1.00),4.0)]:
    ok=a.move_hand(T(t,Rp),Rp,s); p,Rh=a.hand_pose()
    print(name,ok,"tips",(p+0.1034*Rh[:,2]).round(3),"wrench",a.wrench().round(1))
    if not ok: break
ok=a.move_hand(T((-0.146,-0.40,1.03),Rp),Rp,2.0); print("back",ok)
a.snap('robot0_eye_in_hand','snaps/eih_pushed.png')
EOF

# openrua op 71
timeout 600 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
th=np.radians(30); Rp=R_from_axes(z=[0,np.sin(th),-np.cos(th)], y=[1,0,0])
def T(t,R): return np.array(t)-0.1034*R[:,2]
for name,t,s in [("pre",(-0.146,-0.44,1.06),3.0),("down",(-0.146,-0.44,1.00),2.0),("push",(-0.146,-0.323,1.00),4.0)]:
    ok=a.move_hand(T(t,Rp),Rp,s); p,Rh=a.hand_pose()
    print(name,ok,"tips",(p+0.1034*Rh[:,2]).round(3),"wrench",a.wrench().round(1))
    if not ok: break
ok=a.move_hand(T((-0.146,-0.40,1.03),Rp),Rp,2.0); print("back",ok)
a.snap('robot0_eye_in_hand','snaps/eih_pushed.png')
EOF

# openrua op 72
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_depth5.png >/dev/null; timeout 120 python3 - 2>/dev/null <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
rclpy.init(); node=rclpy.create_node('pc')
buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(CameraInfo,'/robot0_eye_in_hand/color/camera_info',lambda m: got.setdefault('m',m),1)
while 'm' not in got: rclpy.spin_once(node,timeout_sec=0.2)
fr='robot0_eye_in_hand_optical_frame'
while not buf.can_transform('world',fr,Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',fr,Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
k=got['m'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
D=np.load('snaps/eih_depth5.npy'); h,wd=D.shape
vv,uu=np.mgrid[0:h,0:wd]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D],-1)@R.T+tr
X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.25)&(X<-0.05)&(Y>-0.40)&(Y<-0.15)&(Z>1.0)&(Z<1.08)
print("rim-ish pts",m.sum())
if m.sum()>50:
    zt=np.percentile(Z[m],98); rim=m&(Z>zt-0.01)
    pts=np.c_[X[rim],Y[rim]]; A=np.c_[2*pts,np.ones(len(pts))]; b=(pts**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; cxm,cym=c[0],c[1]; r=np.sqrt(c[2]+cxm**2+cym**2)
    print("rim fit centre (%.4f,%.4f) r=%.4f ztop=%.4f"%(cxm,cym,r,zt))
    hm=(X>cxm-0.03)&(X<cxm+0.03)&(Y<cym-0.05)&(Y>cym-0.12)&(Z>0.95)&(Z<1.04)
    print("handle pts",hm.sum(), "y[%.3f,%.3f] z[%.3f,%.3f]"%(Y[hm].min(),Y[hm].max(),Z[hm].min(),Z[hm].max()) if hm.sum() else "")
EOF

# openrua op 73
timeout 120 python3 px_batch.py robot0_eye_in_hand 290,245 295,235 280,200 300,300 200,150 420,150 240,120 250,110 2>/dev/null

# openrua op 74
timeout 600 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
a=Arm()
th=np.radians(40); Rp=R_from_axes(z=[0,np.sin(th),-np.cos(th)], y=[1,0,0])
def T(t,R): return np.array(t)-0.1034*R[:,2]
for name,t,s in [("pre",(-0.146,-0.40,0.99),3.0),("push",(-0.146,-0.308,0.99),3.0)]:
    ok=a.move_hand(T(t,Rp),Rp,s); p,Rh=a.hand_pose()
    print(name,ok,"tips",(p+0.1034*Rh[:,2]).round(3),"wrench",a.wrench().round(1))
    if not ok: break
ok=a.move_hand(T((-0.146,-0.42,1.03),Rp),Rp,2.0); print("back",ok)
EOF
timeout 120 python3 px_batch.py robot0_eye_in_hand 290,245 300,300 2>/dev/null; timeout 60 python3 - 2>/dev/null <<'EOF'
from arm import Arm
a=Arm(); a.snap('robot0_eye_in_hand','snaps/eih_pushed2.png')
EOF

# openrua op 75
timeout 120 python3 px_batch.py robot0_eye_in_hand 290,225 292,220 288,230 290,235 200,120 2>/dev/null

# openrua op 76
timeout 900 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm
from door import *
from scene import *
a=Arm()
X,Y,Z=birdview_cloud(a.node); ang,tip=door_state(X,Y,Z); print("door before",round(ang,1),tip.round(3))
al0=ang
tips,R=push_pose(al0,0.19,-1,off=0.03,z=Z_CLEAR)
ok=a.move_hand(hand_from_tips(tips,R),R,4.0); print("above",ok)
tips,R=push_pose(al0,0.19,-1,off=0.03,z=1.06)
ok=a.move_hand(hand_from_tips(tips,R),R,2.5); print("down",ok,"wrench",a.wrench().round(1))
alphas=np.arange(al0-6,-5,-7.0); n=len(alphas)
ok=sweep(a,alphas,[0.19]*n,-1,off=0.03,secs_per=1.0); print("sweep",ok,"wrench",a.wrench().round(1))
p,Rh=a.hand_pose(); print("tips",(p+0.1034*Rh[:,2]).round(3))
EOF

# openrua op 77
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm
from scene import *
a=Arm()
p,Rh=a.hand_pose()
a.move_hand(p+[0,0,0.2],Rh,2.5)
X,Y,Z=birdview_cloud(a.node); ang,tip=door_state(X,Y,Z); print("door",round(ang,1),tip.round(3))
a.snap('birdview','snaps/bird_check.png')
EOF

# openrua op 78
timeout 900 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm
from door import *
from scene import *
a=Arm()
al0=130.9
tips,R=push_pose(al0,0.19,-1,off=0.03,z=Z_CLEAR)
ok=a.move_hand(hand_from_tips(tips,R),R,4.0); print("above",ok)
tips,R=push_pose(al0,0.19,-1,off=0.03,z=1.06)
ok=a.move_hand(hand_from_tips(tips,R),R,2.5); print("down",ok,"wrench",a.wrench().round(1))
alphas=np.arange(al0-6,-5,-7.0); n=len(alphas)
ok=sweep(a,alphas,[0.19]*n,-1,off=0.03,secs_per=1.0); print("sweep",ok,"wrench",a.wrench().round(1))
p,Rh=a.hand_pose(); print("tips",(p+0.1034*Rh[:,2]).round(3))
EOF

# openrua op 79
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm
from scene import *
a=Arm()
p,Rh=a.hand_pose()
a.move_hand(p+[0,0,0.2],Rh,2.5)
X,Y,Z=birdview_cloud(a.node); ang,tip=door_state(X,Y,Z); print("door",round(ang,1),tip.round(3))
a.snap('birdview','snaps/bird_check2.png')
EOF

# openrua op 80
timeout 600 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, R_from_axes
from scene import *
a=Arm()
Rv=R_from_axes(z=[0,0,-1], y=[1,0,0])
def T(t): return np.array(t)-0.1034*Rv[:,2]
for name,t,s in [("pre",(-0.10,-0.43,1.20),3.0),("down",(-0.10,-0.43,1.06),2.0),("push",(-0.10,-0.362,1.06),3.0)]:
    ok=a.move_hand(T(t),Rv,s); p,Rh=a.hand_pose()
    print(name,ok,"tips",(p+0.1034*Rh[:,2]).round(3),"wrench",a.wrench().round(1))
ok=a.move_hand(T((-0.10,-0.43,1.25)),Rv,2.5); print("away",ok)
X,Y,Z=birdview_cloud(a.node); ang,tip=door_state(X,Y,Z); print("door",round(ang,1),tip.round(3))
EOF

# openrua op 81
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm
from scene import *
a=Arm()
X,Y,Z=birdview_cloud(a.node)
m=(Z>0.95)&(Z<1.15)&(Y<-0.355)&(X>-0.7)&(X<0.2)
r=np.hypot(X-HINGE[0],Y-HINGE[1]); m&=(r<0.30)&(r>0.03)
print("door pts",m.sum())
if m.sum(): print(" x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
# door top strip: points at z~1.10 with y<-0.33
s=(Z>1.09)&(Z<1.12)&(Y<-0.33)&(Y>-0.42)&(X>-0.30)&(X<0.0)
print("top strip pts",s.sum())
if s.sum():
    for x0 in (-0.25,-0.15,-0.05):
        t=s&(X>x0-0.02)&(X<x0+0.02)
        if t.sum(): print("  x=%.2f: y[%.3f,%.3f]"%(x0,Y[t].min(),Y[t].max()))
print(door_state(X,Y,Z))
a.snap('birdview','snaps/bird_final.png')
EOF

# openrua op 82
timeout 300 python3 - 2>/dev/null <<'EOF'
import numpy as np
from arm import Arm, look_pose
a=Arm()
for cam,tgt in [((-0.12,-0.58,1.22),(-0.12,-0.34,1.0)),((-0.15,-0.55,1.20),(-0.15,-0.32,1.00))]:
    for ya in [(0,0,-1),(1,0,0),(-1,0,0)]:
        p,R=look_pose(cam,tgt,ya)
        if a.solve_ik(p,R) is not None:
            print("moving",cam,ya, a.move_hand(p,R,3.5)); a.snap('robot0_eye_in_hand','snaps/eih_final_door.png'); raise SystemExit
EOF
echo ok
