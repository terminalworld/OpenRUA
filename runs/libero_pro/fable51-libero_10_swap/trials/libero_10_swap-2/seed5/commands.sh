#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---ACTIONS; timeout 30 ros2 action list; echo ---SERVICES; timeout 30 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---NODES; timeout 30 ros2 node list

# openrua op 4
timeout 20 ros2 topic info /pipeline_state -v 2>&1 | head -30; echo ---; timeout 20 ros2 topic echo /pipeline_state --once 2>&1 | head -40; echo ---; timeout 30 ros2 node info /robot_bridge 2>&1

# openrua op 5
for c in agentview frontview sideview birdview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 6
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | sort | uniq -c; echo ---; timeout 15 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | sort | uniq -c

# openrua op 7
timeout 15 ros2 topic echo /tf --once 2>&1 | grep -A 12 "child_frame_id: panda_link0"; timeout 15 ros2 topic echo /tf --once 2>&1 | grep -B2 -A 12 "child_frame_id: birdview_optical_frame"

# openrua op 8
mkdir -p "$(dirname /workspace/px_batch.py)"
cat > /workspace/px_batch.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Batch pixel->world for one camera. Usage: px_batch.py <cam> u,v [u,v ...]"""
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
    pts = [tuple(int(x) for x in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("px_batch")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    for (u, v) in pts:
        zc = float(D[v, u])
        pc = np.array([(u-cx)*zc/fx, (v-cy)*zc/fy, zc])
        p = R @ pc + tr
        print(f"px({u},{v}) depth={zc:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 9
timeout 60 python3 px_batch.py birdview 248,283 378,285 378,237 325,315 320,240 200,350 355,262 400,305 320,285

# openrua op 10
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import numpy as np
D=np.load('birdview_depth.npy'); print(D.shape, np.nanmin(D), np.nanmax(D))
# camera at z=3.0 looking down; world z ~ 3.0 - depth (approx for pinhole center; use exact via intrinsics)
"
timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A 10 "^k:"

# openrua op 11
python3 -c "
import numpy as np, cv2
D=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
H,W=D.shape
v,u=np.mgrid[0:H,0:W]
# optical frame: x right, y down, z forward. rotation (0.707,0.707,0,0): maps optical x->world x? compute:
# q=(x=.707,y=.707,z=0,w=0): R = [[0,1,0],[1,0,0],[0,0,-1]] => world = (opt_y, opt_x, -opt_z) + (-0.2,0,3.0)
X=(u-cx)*D/fx; Y=(v-cy)*D/fx; Z=D
wx=Y-0.2; wy=X; wz=3.0-Z
# objects above table (z>0.905)
mask=(wz>0.905)&(wz<1.3)
# connected components
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    m=lab==i
    if stats[i,4]<20: continue
    print(f'comp{i}: px area={stats[i,4]} centroid px=({cent[i][0]:.0f},{cent[i][1]:.0f}) world x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] zmax={wz[m].max():.3f} zmed={np.median(wz[m]):.3f}')
"


# openrua op 12
python3 -c "
import numpy as np, cv2
D=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
H,W=D.shape
v,u=np.mgrid[0:H,0:W]
X=(u-cx)*D/fx; Y=(v-cy)*D/fx
wx=Y-0.2; wy=X; wz=3.0-D
def comps(mask,label):
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for i in range(1,n):
        m=lab==i
        if stats[i,4]<30: continue
        print(f'{label} comp{i}: area={stats[i,4]} px=({cent[i][0]:.0f},{cent[i][1]:.0f}) x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] z[{wz[m].min():.3f},{wz[m].max():.3f}] zmed={np.median(wz[m]):.3f}')
comps((wz>0.905)&(wz<0.945)&(wx>-0.15)&(wx<0.2)&(wy>-0.1)&(wy<0.35),'low')
comps((wz>=0.945)&(wz<1.1)&(wx>-0.15)&(wx<0.2)&(wy>-0.1)&(wy<0.35),'high')
# stove plate rows: print z along y through stove center x=-0.04
r=int(np.argmin(np.abs(wx[:,378]+0.04)))
print('row',r)
for c in range(340,420,4): print(c, f'{wy[r,c]:.3f} {wz[r,c]:.3f}')
"


# openrua op 13
python3 -c "
import numpy as np
D=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
H,W=D.shape
v,u=np.mgrid[0:H,0:W]
X=(u-cx)*D/fx; Y=(v-cy)*D/fx
wx=Y-0.2; wy=X; wz=3.0-D
m=(wz>0.945)&(wz<1.1)&(wy>0.12)&(wy<0.35)&(wx>-0.15)&(wx<0.2)
print('handle: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(wx[m].min(),wx[m].max(),wy[m].min(),wy[m].max(),wz[m].min(),wz[m].max()))
for yy in np.arange(0.12,0.30,0.02):
    mm=m&(np.abs(wy-yy)<0.01)
    if mm.sum(): print(f'y={yy:.2f}: x[{wx[mm].min():.3f},{wx[mm].max():.3f}] z[{wz[mm].min():.3f},{wz[mm].max():.3f}]')
# pan body
m2=(wz>0.945)&(wz<1.1)&(wy<0.12)&(wx>-0.15)&(wx<0.2)&(wy>-0.15)
print('pan body: x[%.3f,%.3f] y[%.3f,%.3f]'%(wx[m2].min(),wx[m2].max(),wy[m2].min(),wy[m2].max()))
# moka pot
m3=(wz>0.945)&(wx>-0.15)&(wx<0.05)&(wy<-0.15)&(wy>-0.4)
print('moka: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(wx[m3].min(),wx[m3].max(),wy[m3].min(),wy[m3].max(),wz[m3].min(),wz[m3].max()))
for zz in [0.95,1.0,1.03,1.05]:
    mm=m3&(wz>zz)
    print(f'z>{zz}: x[{wx[mm].min():.3f},{wx[mm].max():.3f}] y[{wy[mm].min():.3f},{wy[mm].max():.3f}] centroid ({wx[mm].mean():.3f},{wy[mm].mean():.3f})')
# knob
m4=(wz>0.905)&(wx>-0.3)&(wx<-0.15)&(wy>0.1)&(wy<0.3)
for zz in [0.905,0.93,0.95]:
    mm=m4&(wz>zz)
    print(f'knob z>{zz}: x[{wx[mm].min():.3f},{wx[mm].max():.3f}] y[{wy[mm].min():.3f},{wy[mm].max():.3f}] centroid ({wx[mm].mean():.3f},{wy[mm].mean():.3f}) n={mm.sum()}')
"


# openrua op 14
timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw frontview_depth.png && timeout 20 ros2 topic echo /frontview/color/camera_info --once | grep -A 3 "^k:" && timeout 15 ros2 topic echo /tf --once 2>&1 | grep -A 12 "child_frame_id: frontview_optical_frame"

# openrua op 15
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
"""Helpers: load a saved depth .npy for a camera and produce world-frame points."""
import numpy as np

FX = 579.4112549695428
CX, CY = 320.0, 240.0

CAMS = {
    # cam: (translation, quaternion xyzw) of <cam>_optical_frame in world
    "birdview": ((-0.2, 0.0, 3.0), (0.7071067811865476, 0.7071067811865477, 0.0, 0.0)),
    "frontview": ((1.0, 0.0, 1.48), (0.5608418947374367, 0.5608418947374367, -0.4306464548876746, -0.4306464548876746)),
}


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def world_points(cam, npy=None):
    D = np.load(npy or f"{cam}_depth.npy")
    H, W = D.shape
    v, u = np.mgrid[0:H, 0:W]
    pc = np.stack([(u - CX) * D / FX, (v - CY) * D / FX, D], axis=-1)
    t, q = CAMS[cam]
    R = quat_R(*q)
    pw = pc @ R.T + np.array(t)
    return pw[..., 0], pw[..., 1], pw[..., 2], D
OPENRUA_EOF

# openrua op 16
python3 -c "
import numpy as np
from cloud import world_points
wx,wy,wz,D=world_points('frontview')
# moka pot region
m=(wx>-0.15)&(wx<0.05)&(wy<-0.15)&(wy>-0.4)&(wz>0.905)&(wz<1.1)
print('moka n',m.sum())
for z0 in np.arange(0.90,1.06,0.01):
    mm=m&(wz>=z0)&(wz<z0+0.01)
    if mm.sum()>3:
        print(f'z {z0:.2f}: y[{wy[mm].min():.3f},{wy[mm].max():.3f}] width={wy[mm].max()-wy[mm].min():.3f} x[{wx[mm].min():.3f},{wx[mm].max():.3f}] n={mm.sum()}')
"


# openrua op 17
python3 -c "
import numpy as np
from cloud import world_points
wx,wy,wz,D=world_points('birdview')
m=(wx>-0.15)&(wx<0.05)&(wy<-0.15)&(wy>-0.4)&(wz>0.905)
for y0 in np.arange(-0.34,-0.18,0.01):
    mm=m&(wy>=y0)&(wy<y0+0.01)
    if mm.sum()>2: print(f'y {y0:.2f}: x[{wx[mm].min():.3f},{wx[mm].max():.3f}] z[{wz[mm].min():.3f},{wz[mm].max():.3f}] n={mm.sum()}')
"


# openrua op 18
python3 -c "
import numpy as np
from cloud import world_points
wx,wy,wz,D=world_points('birdview')
m=(wx>-0.28)&(wx<-0.14)&(wy>0.14)&(wy<0.27)&(wz>0.905)
print('knob all: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] c=(%.3f,%.3f)'%(wx[m].min(),wx[m].max(),wy[m].min(),wy[m].max(),wz[m].min(),wz[m].max(),wx[m].mean(),wy[m].mean()))
for z0 in [0.91,0.92,0.93,0.94,0.95]:
    mm=m&(wz>z0)
    print(f'z>{z0}: x[{wx[mm].min():.3f},{wx[mm].max():.3f}] y[{wy[mm].min():.3f},{wy[mm].max():.3f}] c=({wx[mm].mean():.3f},{wy[mm].mean():.3f}) n={mm.sum()}')
# print a small ascii map of height in knob region
xs=np.arange(-0.26,-0.14,0.005); ys=np.arange(0.14,0.27,0.005)
for x0 in xs:
    row=''
    for y0 in ys:
        mm=(np.abs(wx-x0)<0.0025)&(np.abs(wy-y0)<0.0025)
        h=wz[mm].max() if mm.sum() else 0
        row+= '.' if h<0.905 else ('o' if h<0.94 else '#')
    print(f'{x0:.3f} {row}')
"


# openrua op 19
timeout 300 python3 tools/action/ik_move.py 0.457 0.203 0.15 1 0 0 0 3 --at tcp; timeout 20 ros2 topic echo /joint_states --once | grep -A 9 "^position"

# openrua op 20
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small control library for this Panda: IK, FK, trajectories, gripper, sensing.

All poses are WORLD frame; converted to the arm base (panda_link0) for MoveIt.
"""
import sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import WrenchStamped, TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK

BASE = np.array([-0.66, 0.0, 0.912])   # panda_link0 in world (no rotation)
ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)          # hand z down, fingers along world y
Q_DOWN_X = (0.7071068, 0.7071068, 0.0, 0.0)  # hand z down, fingers along world x


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Ctl:
    def __init__(self, name="ctl"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def js(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin()
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.js()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.js()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin()
        f = self._wr.wrench.force; t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk(self, q=None):
        """world-frame position + quaternion of panda_hand."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [float(v) for v in (q or self.arm_q())]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        return pos + TCP * R[:, 2], quat

    # ---------- planning ----------
    def ik(self, pos_world, quat, at_tcp=True, seed=None, tries=3):
        pos = np.array(pos_world, dtype=float)
        if at_tcp:
            pos = pos - TCP * quat_R(*quat)[:, 2]
        pos_b = pos - BASE
        seed = seed or self.arm_q()
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos_b)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = ARM
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.timeout.sec = 2
            req.ik_request.avoid_collisions = False
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[n] for n in ARM]
            print(f"IK try {k}: error {r and r.error_code.val}", file=sys.stderr)
        return None

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i + 1) / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"move_q: code={code} max_err={err:.4f}")
        return code, err

    def move_pose(self, pos_world, quat, seconds=3.0, at_tcp=True, seed=None):
        q = self.ik(pos_world, quat, at_tcp=at_tcp, seed=seed)
        if q is None:
            print("move_pose: IK FAILED, no motion")
            return None
        self.move_q(q, seconds)
        return q

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        f = self.fingers()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def servo(self, lin, ang=(0, 0, 0), ticks=20, frame="panda_link0"):
        msg = TwistStamped(); msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            self.twist_pub.publish(msg); self.spin(0.05)

    def snap(self, cam, out=None):
        from cv_bridge import CvBridge
        import cv2
        got = []
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw", got.append, 1)
        while not got:
            self.spin(0.5)
        self.node.destroy_subscription(sub)
        out = out or f"{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got[0], "bgr8"))
        return out
OPENRUA_EOF

# openrua op 21
timeout 120 python3 -c "
from ctl import *
c=Ctl()
print('hand', c.fk())
print('tcp', c.tcp())
print('fingers', c.fingers())
"

# openrua op 22
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String, '/robot_description', got.append, qos)
while not got: rclpy.spin_once(n, timeout_sec=0.5)
s=got[0].data
import re
print(len(s))
for m in re.finditer(r'<link name=\"(\w+)\"', s): print('link', m.group(1))
i=s.find('panda_link0'); print(s[max(0,i-800):i+200])
" 2>&1 | head -80

# openrua op 23
python3 -c "
import numpy as np
def dh(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483]
P=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
T=np.eye(4)
for (a,d,al),th in zip(P,q): T=T@dh(a,d,al,th)
T=T@dh(0,0.107,0,0)  # flange
Th=T@dh(0,0,0,-np.pi/4)  # hand
print('flange',T[:3,3]); print('hand R',np.round(Th[:3,:3],3))
"


# openrua op 24
timeout 300 python3 -c "
from ctl import *
c=Ctl()
q0=c.arm_q()
pos,quat=c.fk()
print('fk raw', pos-BASE, quat)
# test A: pose as returned by FK raw (subtract BASE back out => raw numbers)
raw=pos-BASE
for label,p in [('raw(world-like)', raw), ('raw+base offset', raw+BASE)]:
    c.ik.__func__  # noop
    # call ik with at_tcp False; ik() subtracts BASE, so pass p+BASE to send p raw
    q=c.ik(p+BASE, quat, at_tcp=False, tries=1)
    print(label, 'sent', p, '->', None if q is None else np.round(q,3))
print('current', np.round(q0,3))
" 2>&1 | grep -v "^\[" 

# openrua op 25
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "BASE = np.array([-0.66, 0.0, 0.912])   # panda_link0 in world (no rotation)", "new_string": "# Verified empirically: /compute_fk and /compute_ik on this machine speak\\n# WORLD coordinates (FK of the start pose matched the birdview camera), so\\n# no base offset is applied.\\nBASE = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 26
timeout 600 python3 -c "
from ctl import *
c=Ctl()
q=c.move_pose((-0.203, 0.203, 1.08), Q_DOWN_Y, seconds=4)
print('q', np.round(q,3) if q else None)
print('tcp now', c.tcp())
c.snap('robot0_eye_in_hand','eih1.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 27
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "            req.ik_request.group_name = \\"panda_arm\\"\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "            req.ik_request.group_name = \\"panda_arm\\"\\n            req.ik_request.ik_link_name = \\"panda_hand\\"   # group tip is link8 (45deg off)\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 28
timeout 600 python3 -c "
from ctl import *
c=Ctl()
q=c.move_pose((-0.203, 0.203, 1.08), Q_DOWN_Y, seconds=3)
print('q', np.round(q,3) if q else None)
print('tcp now', c.tcp())
c.snap('robot0_eye_in_hand','eih1.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 29
timeout 600 python3 -c "
from ctl import *
c=Ctl()
q=[0.288,0.113,0.129,-2.392,-0.024,2.504,1.221]
c.move_q(q, 2)
print('tcp now', c.tcp())
c.snap('robot0_eye_in_hand','eih1.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 30
timeout 600 python3 -c "
from ctl import *
c=Ctl()
q=c.move_pose((-0.203, 0.203, 0.947), Q_DOWN_Y, seconds=3)
print('q', np.round(q,3) if q else None)
print('tcp now', c.tcp())
print('wrench', c.wrench())
c.snap('robot0_eye_in_hand','eih2.png'); c.snap('sideview','side2.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 31
timeout 600 python3 -c "
from ctl import *
c=Ctl()
f=c.gripper(0.0)
print('fingers after close', f)
" 2>&1 | grep -v "^\[INFO"

# openrua op 32
timeout 600 python3 -c "
from ctl import *
c=Ctl()
q=c.arm_q(); print('before', np.round(q,3))
q[6]=q[6]-1.5708
c.move_q(q, 3)
print('after', np.round(c.arm_q(),3), 'fingers', c.fingers())
c.snap('robot0_eye_in_hand','eih3.png'); c.snap('agentview','agent3.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 33
timeout 600 python3 -c "
from ctl import *
c=Ctl()
c.gripper(0.04)
q=c.move_pose((-0.203, 0.203, 1.08), Q_DOWN_Y, seconds=3)
print('tcp', c.tcp())
c.snap('robot0_eye_in_hand','eih4.png'); c.snap('agentview','agent4.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 34
python3 -c "
import numpy as np
from cloud import world_points
wx,wy,wz,D=world_points('frontview')
m=(wx>-0.15)&(wx<0.05)&(np.abs(wy+0.249)<0.012)&(wz>0.90)&(wz<1.07)
for z0 in np.arange(0.90,1.06,0.005):
    mm=m&(wz>=z0)&(wz<z0+0.005)
    if mm.sum()>2:
        print(f'z {z0:.3f}: front x_max={wx[mm].max():.4f} radius={wx[mm].max()+0.054:.4f} n={mm.sum()}')
print('--- y extent (full width) per 5mm z-bin, body only |x+0.054|<0.05')
m=(np.abs(wx+0.054)<0.06)&(wy<-0.1)&(wy>-0.4)&(wz>0.90)&(wz<1.07)
for z0 in np.arange(0.90,1.06,0.005):
    mm=m&(wz>=z0)&(wz<z0+0.005)
    if mm.sum()>2:
        print(f'z {z0:.3f}: y[{wy[mm].min():.3f},{wy[mm].max():.3f}] n={mm.sum()}')
"


# openrua op 35
timeout 600 python3 -c "
from ctl import *
c=Ctl()
q=c.move_pose((-0.054, -0.249, 1.25), Q_DOWN_X, seconds=4)
print('tcp', c.tcp())
c.snap('robot0_eye_in_hand','eih5.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 36
timeout 600 python3 -c "
from ctl import *
c=Ctl()
q=c.ik((-0.054, -0.249, 1.25), Q_DOWN_X)
print('target q', np.round(q,3)); print('cur', np.round(c.arm_q(),3))
c.move_q(q, 3)
print('tcp', c.tcp())
c.snap('robot0_eye_in_hand','eih5.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 37
timeout 600 python3 -c "
from ctl import *
c=Ctl()
q=[-0.374,0.324,-0.018,-1.607,0.006,1.931,-1.179]
print('cur', np.round(c.arm_q(),3))
c.move_q(q, 5)
print('cur', np.round(c.arm_q(),3))
print('tcp', c.tcp())
c.snap('robot0_eye_in_hand','eih5.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 38
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = ARM\\n        pts = []", "new_string": "        # machine quirk: joint7 tracks at only ~0.17 rad/s; budget time for it\\n        d7 = abs(q[6] - self.arm_q()[6])\\n        seconds = max(seconds, d7 / 0.15)\\n        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = ARM\\n        pts = []", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 39
timeout 600 python3 -c "
from ctl import *
c=Ctl()
q=[-0.374,0.324,-0.018,-1.607,0.006,1.931,-1.179]
c.move_q(q, 3)
print('cur', np.round(c.arm_q(),3))
print('tcp', c.tcp())
c.snap('robot0_eye_in_hand','eih5.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 40
timeout 120 python3 -c "
import numpy as np, rclpy
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from cloud import quat_R
rclpy.init(); n=rclpy.create_node('eihd'); buf=Buffer(); TransformListener(buf,n)
got={}
n.create_subscription(Image,'/robot0_eye_in_hand/depth/image_raw',lambda m:got.setdefault('d',m),1)
n.create_subscription(CameraInfo,'/robot0_eye_in_hand/color/camera_info',lambda m:got.setdefault('i',m),1)
while 'd' not in got or 'i' not in got: rclpy.spin_once(n,timeout_sec=0.2)
while not buf.can_transform('world','robot0_eye_in_hand_optical_frame',rclpy.time.Time()): rclpy.spin_once(n,timeout_sec=0.2)
t=buf.lookup_transform('world','robot0_eye_in_hand_optical_frame',rclpy.time.Time())
q=t.transform.rotation; tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
print('cam pose', tr, q)
d=got['d']; D=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]; print('K',fx,fy,cx,cy)
H,W=D.shape; v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D],-1); R=quat_R(q.x,q.y,q.z,q.w); pw=pc@R.T+tr
wx,wy,wz=pw[...,0],pw[...,1],pw[...,2]
np.save('eih_depth.npy',D)
m=(wz>0.95)&(wz<1.1)&(np.abs(wx+0.054)<0.08)&(np.abs(wy+0.25)<0.1)
print('pot: n',m.sum())
for z0 in [0.95,1.0,1.02,1.03,1.035,1.04]:
    mm=m&(wz>z0)
    print(f'z>{z0}: x[{wx[mm].min():.4f},{wx[mm].max():.4f}] y[{wy[mm].min():.4f},{wy[mm].max():.4f}] c=({wx[mm].mean():.4f},{wy[mm].mean():.4f}) n={mm.sum()}')
# body only (exclude handle/spout): |y+0.249|<0.03
mm=m&(np.abs(wy+0.249)<0.03)&(wz>1.02)
print(f'body top ring: x[{wx[mm].min():.4f},{wx[mm].max():.4f}] cx={(wx[mm].min()+wx[mm].max())/2:.4f} zmax={wz[mm].max():.4f}')
mm=m&(np.abs(wx+0.054)<0.03)&(wz>1.02)
print(f'body y ext (incl spout/handle): y[{wy[mm].min():.4f},{wy[mm].max():.4f}]')
# print per-y-slice x-extents
for y0 in np.arange(-0.30,-0.19,0.005):
    mm=m&(np.abs(wy-y0)<0.0025)&(wz>1.0)
    if mm.sum(): print(f' y={y0:.3f}: x[{wx[mm].min():.4f},{wx[mm].max():.4f}] w={wx[mm].max()-wx[mm].min():.4f} zmax={wz[mm].max():.3f}')
"


# openrua op 41
timeout 300 python3 -c "
from ctl import *
from moveit_msgs.srv import GetPositionFK
c=Ctl()
def fk_links(q, links):
    req=GetPositionFK.Request(); req.fk_link_names=links
    req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=[float(v) for v in q]
    fut=c.fk_cli.call_async(req); rclpy.spin_until_future_complete(c.node,fut,timeout_sec=30)
    r=fut.result(); return {l:(np.round([p.pose.position.x,p.pose.position.y,p.pose.position.z],3)) for l,p in zip(links,r.pose_stamped)}
QA=(0.5,0.5,-0.5,0.5); QB=(0.5,-0.5,0.5,0.5)
for lab,Q in [('A',QA),('B',QB)]:
    for pos in [(-0.052,-0.13,0.967),(-0.052,-0.249,0.967)]:
        q=c.ik(pos,Q,tries=2)
        print(lab,pos,'q',None if q is None else np.round(q,3))
        if q: print('   links',fk_links(q,['panda_link5','panda_link6','panda_link7','panda_hand']))
" 2>&1 | grep -v "^\[INFO"

# openrua op 42
timeout 300 python3 -c "
from ctl import *
from scipy.spatial.transform import Rotation as Rot
psi=np.deg2rad(45)
fy=np.array([np.cos(psi),np.sin(psi),0]); a=np.array([np.sin(psi),-np.cos(psi),0]); hx=np.cross(fy,a)
R=np.column_stack([hx,fy,a]); Q=Rot.from_matrix(R).as_quat(); print('Q',np.round(Q,4),'a',a,'hx',hx)
c=Ctl()
pot=np.array([-0.0517,-0.2493,0.967])
pre=pot-0.13*a
high=pre.copy(); high[2]=1.15
for lab,p in [('high',high),('pre',pre),('grasp',pot+0.005*a)]:
    q=c.ik(p,Q,tries=2); print(lab,np.round(p,3),None if q is None else np.round(q,3))
" 2>&1 | grep -v "^\[INFO"

# openrua op 43
timeout 300 python3 -c "
from ctl import *
from moveit_msgs.srv import GetPositionFK
Q=(0.2706,0.6533,-0.2706,0.6533); a=np.array([0.70710678,-0.70710678,0])
c=Ctl()
def fk_links(q, links=['panda_link4','panda_link5','panda_link7','panda_hand']):
    req=GetPositionFK.Request(); req.fk_link_names=links
    req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=[float(v) for v in q]
    fut=c.fk_cli.call_async(req); rclpy.spin_until_future_complete(c.node,fut,timeout_sec=30)
    r=fut.result(); return {l:(np.round([p.pose.position.x,p.pose.position.y,p.pose.position.z],3)) for l,p in zip(links,r.pose_stamped)}
pot=np.array([-0.0517,-0.2493,0.967]); pre=pot-0.13*a; high=pre.copy(); high[2]=1.15
seed=[0.887,1.372,-0.907,-1.679,-0.621,2.074,0.004]
for seed0 in [seed,[0,0,0,-1.5,0,1.5,0.785],[0.5,0.5,-0.5,-2.0,0.3,2.4,0.0]]:
  print('SEED',seed0)
  s=seed0
  for lab,p in [('high',high),('pre',pre),('grasp',pot+0.005*a)]:
    q=c.ik(p,Q,seed=s,tries=2); print(' ',lab,np.round(p,3),None if q is None else np.round(q,3))
    if q: print('    ',fk_links(q)); s=q
" 2>&1 | grep -v "^\[INFO"

# openrua op 44
timeout 900 python3 -c "
from ctl import *
c=Ctl()
high=[0.024,0.112,-0.098,-2.603,-2.381,1.897,0.504]
pre=[0.036,0.697,-0.056,-2.396,-2.361,1.63,0.777]
c.move_q(high, 6)
print('tcp', np.round(c.tcp()[0],4), np.round(c.tcp()[1],3))
c.move_q(pre, 4)
print('tcp', np.round(c.tcp()[0],4), np.round(c.tcp()[1],3))
c.snap('robot0_eye_in_hand','eih6.png'); c.snap('sideview','side6.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 45
timeout 120 python3 -c "
import numpy as np, rclpy
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from cloud import quat_R
rclpy.init(); n=rclpy.create_node('eihd'); buf=Buffer(); TransformListener(buf,n)
got={}
n.create_subscription(Image,'/robot0_eye_in_hand/depth/image_raw',lambda m:got.setdefault('d',m),1)
n.create_subscription(CameraInfo,'/robot0_eye_in_hand/color/camera_info',lambda m:got.setdefault('i',m),1)
while 'd' not in got or 'i' not in got: rclpy.spin_once(n,timeout_sec=0.2)
while not buf.can_transform('world','robot0_eye_in_hand_optical_frame',rclpy.time.Time()): rclpy.spin_once(n,timeout_sec=0.2)
t=buf.lookup_transform('world','robot0_eye_in_hand_optical_frame',rclpy.time.Time())
q=t.transform.rotation; tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
print('cam pose', np.round(tr,4))
d=got['d']; D=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
H,W=D.shape; v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D],-1); R=quat_R(q.x,q.y,q.z,q.w); pw=pc@R.T+tr
wx,wy,wz=pw[...,0],pw[...,1],pw[...,2]
a=np.array([0.70710678,-0.70710678,0]); fyv=np.array([0.70710678,0.70710678,0])
pot=np.array([-0.0517,-0.2493,0.0])
# points of the pot: within 6cm horizontally of the pot axis
rel=np.stack([wx-pot[0],wy-pot[1]],-1); rad=np.linalg.norm(rel,axis=-1)
m=(rad<0.06)&(wz>0.905)&(wz<1.06)
print('pot pts',m.sum())
# distance along approach axis (depth along a) and lateral along finger axis
along=rel[...,0]*a[0]+rel[...,1]*a[1]; lat=rel[...,0]*fyv[0]+rel[...,1]*fyv[1]
for z0 in np.arange(0.905,1.06,0.005):
    mm=m&(wz>=z0)&(wz<z0+0.005)&(np.abs(lat)<0.01)
    if mm.sum()>2:
        # nearest surface point along -a is the visible face; radius = -along
        print(f'z {z0:.3f}: visible face at along={along[mm].min():.4f} (radius {-along[mm].min():.4f}) lat width {lat[m&(wz>=z0)&(wz<z0+0.005)].min():.3f}..{lat[m&(wz>=z0)&(wz<z0+0.005)].max():.3f} n={mm.sum()}')
"


# openrua op 46
timeout 900 python3 -c "
from ctl import *
c=Ctl()
Q=(0.2706,0.6533,-0.2706,0.6533); a=np.array([0.70710678,-0.70710678,0])
pot=np.array([-0.0517,-0.2493,0.955])
pre=pot-0.13*a; mid=pot-0.06*a; g=pot+0.01*a
q0=c.arm_q()
qpre=c.ik(pre,Q,seed=q0); print('pre',np.round(qpre,3))
c.move_q(qpre,2); print('tcp',np.round(c.tcp()[0],4))
qmid=c.ik(mid,Q,seed=qpre); qg=c.ik(g,Q,seed=qmid)
print('mid',np.round(qmid,3)); print('g',np.round(qg,3))
c.move_q(qg,4,via=[qmid]); print('tcp',np.round(c.tcp()[0],4), np.round(c.tcp()[1],3))
print('wrench',np.round(c.wrench()[0],2))
c.snap('robot0_eye_in_hand','eih7.png'); c.snap('frontview','front7.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 47
timeout 600 python3 -c "
from ctl import *
c=Ctl()
f=c.gripper(0.0)
print('tcp',np.round(c.tcp()[0],4))
c.snap('robot0_eye_in_hand','eih8.png'); c.snap('agentview','agent8.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 48
timeout 120 python3 -c "
from ctl import *
c=Ctl()
print('fingers',c.fingers()); print('tcp',c.tcp()[0]); print('wrench',c.wrench())
" 2>&1 | grep -v "^\[INFO"; timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw frontview_depth.png >/dev/null && python3 -c "
import numpy as np
from cloud import world_points
wx,wy,wz,D=world_points('frontview')
m=(wx>-0.12)&(wx<0.02)&(np.abs(wy+0.249)<0.012)&(wz>0.90)&(wz<1.07)
for z0 in np.arange(0.90,1.07,0.01):
    mm=m&(wz>=z0)&(wz<z0+0.01)
    if mm.sum()>2: print(f'z {z0:.3f}: front x_max={wx[mm].max():.4f} n={mm.sum()}')
"

# openrua op 49
python3 -c "
import cv2
im=cv2.imread('agent8.png'); crop=im[180:340,80:300]; cv2.imwrite('crop8a.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('front7.png'); crop=im[280:420,120:320]; cv2.imwrite('crop7f.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 50
timeout 600 python3 -c "
from ctl import *
c=Ctl()
print('before',c.fingers())
c.gripper(0.0)
for i in range(5):
    c.spin(0.3); print('fingers',c.fingers())
c.gripper(0.0)
print('after 2nd',c.fingers())
" 2>&1 | grep -v "^\[INFO"

# openrua op 51
python3 -c "
import numpy as np
from cloud import world_points
wx,wy,wz,D=world_points('frontview')
# finger near (-0.024,-0.222) (+x side of the pot along finger axis); pot center now ~(-0.042,-0.259)?
for (fx_,fy_) in [(-0.024,-0.222),(-0.015,-0.215),(-0.03,-0.23)]:
    m=(np.abs(wx-fx_)<0.012)&(np.abs(wy-fy_)<0.012)&(wz>0.905)
    if m.sum(): print((fx_,fy_),'z range',wz[m].min().round(3),wz[m].max().round(3),'n',m.sum())
# whole scene slice: points with y in [-0.24,-0.20], x in [-0.05,0.0]: print z histogram
m=(wy>-0.24)&(wy<-0.19)&(wx>-0.06)&(wx<0.01)&(wz>0.905)
zs=wz[m]; print(np.histogram(zs,bins=np.arange(0.9,1.1,0.01)))
# the hand body: near (-0.118,-0.181)
m=(np.abs(wx+0.118)<0.04)&(np.abs(wy+0.181)<0.04)&(wz>0.905)
print('hand z range',wz[m].min().round(3),wz[m].max().round(3))
"

# openrua op 52
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview bird8.png >/dev/null; python3 -c "
import numpy as np
from cloud import world_points
wx,wy,wz,D=world_points('birdview')
# map region around pot & hand: print max z on a 1cm grid
xs=np.arange(-0.20,0.0,0.01); ys=np.arange(-0.32,-0.12,0.01)
print('      '+' '.join(f'{y:6.2f}' for y in ys))
for x0 in xs:
    row=[]
    for y0 in ys:
        m=(np.abs(wx-x0)<0.005)&(np.abs(wy-y0)<0.005)
        row.append(f'{wz[m].max():6.3f}' if m.sum() else '   -  ')
    print(f'{x0:6.2f} '+' '.join(row))
"

# openrua op 53
python3 -c "
import numpy as np
from cloud import world_points
wx,wy,wz,D=world_points('frontview')
for (x0,y0) in [(-0.02,-0.21),(-0.015,-0.205),(-0.025,-0.215)]:
    m=(np.abs(wx-x0)<0.012)&(np.abs(wy-y0)<0.008)&(wz>0.902)
    if m.sum(): print((x0,y0),'z',wz[m].min().round(3),wz[m].max().round(3),'n',m.sum(), np.histogram(wz[m],bins=np.arange(0.90,1.0,0.01))[0])
# Also the finger from birdview: the 0.964 patch extents
wx,wy,wz,D=world_points('birdview')
m=(wz>0.955)&(wz<0.969)&(wx>-0.04)&(wx<0.0)&(wy>-0.24)&(wy<-0.18)
print('finger+ top: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] n=%d'%(wx[m].min(),wx[m].max(),wy[m].min(),wy[m].max(),wz[m].min(),wz[m].max(),m.sum()))
m=(wz>0.955)&(wz<0.969)&(wx>-0.12)&(wx<-0.08)&(wy>-0.31)&(wy<-0.26)
print('finger- top: x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] n=%d'%(wx[m].min(),wx[m].max(),wy[m].min(),wy[m].max(),wz[m].min(),wz[m].max(),m.sum()))
"

# openrua op 54
timeout 600 python3 -c "
from ctl import *
c=Ctl()
c.gripper(0.04)
print('fingers',c.fingers())
Q=(0.2706,0.6533,-0.2706,0.6533); a=np.array([0.70710678,-0.70710678,0])
tcp=c.tcp()[0]
back=tcp-0.10*a
q=c.ik(back,Q); c.move_q(q,3)
print('tcp',np.round(c.tcp()[0],4),'fingers',c.fingers())
c.snap('robot0_eye_in_hand','eih9.png')
" 2>&1 | grep -v "^\[INFO"; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; python3 -c "
import numpy as np
from cloud import world_points
wx,wy,wz,D=world_points('birdview')
m=(wz>1.045)&(wx>-0.15)&(wx<0.05)&(wy<-0.15)&(wy>-0.4)
print('lid knob centroid', wx[m].mean().round(4), wy[m].mean().round(4), 'zmax', wz[m].max().round(3), m.sum())
m=(wz>0.95)&(wz<1.045)&(wx>-0.15)&(wx<0.05)&(wy<-0.15)&(wy>-0.4)
print('pot top x[%.3f,%.3f] y[%.3f,%.3f]'%(wx[m].min(),wx[m].max(),wy[m].min(),wy[m].max()))
"

# openrua op 55
timeout 600 python3 -c "
from ctl import *
import time
c=Ctl()
print('fingers',c.fingers())
t=time.time(); c.gripper(0.0); print('close took',round(time.time()-t,1),'s')
for i in range(3): c.spin(0.3); print('  ',c.fingers())
t=time.time(); c.gripper(0.04); print('open took',round(time.time()-t,1),'s')
for i in range(3): c.spin(0.3); print('  ',c.fingers())
" 2>&1 | grep -v "^\[INFO"

# openrua op 56
timeout 600 python3 -c "
from ctl import *
from scipy.spatial.transform import Rotation as Rot
psi=np.deg2rad(20)
fy=np.array([np.cos(psi),np.sin(psi),0]); a=np.array([np.sin(psi),-np.cos(psi),0]); hx=np.cross(fy,a)
Q=Rot.from_matrix(np.column_stack([hx,fy,a])).as_quat(); print('Q',np.round(Q,4),'a',np.round(a,4))
c=Ctl()
pot=np.array([-0.0446,-0.2562,0.96])
pre=pot-0.14*a
q0=c.arm_q()
qpre=c.ik(pre,Q,seed=q0); print('pre q',np.round(qpre,3))
c.move_q(qpre,4)
print('tcp',np.round(c.tcp()[0],4),np.round(c.tcp()[1],4),'fingers',c.fingers())
c.snap('robot0_eye_in_hand','eih10.png')
" 2>&1 | grep -v "^\[INFO"

# openrua op 57
cat > /workspace/eih_profile.py <<'EOF'
"""Profile the pot from the wrist depth cam: lateral offset & radius vs z relative to the hand line."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from cloud import quat_R
from ctl import Ctl, TCP

def main():
    c = Ctl('prof')
    buf = Buffer(); TransformListener(buf, c.node)
    got = {}
    c.node.create_subscription(Image, '/robot0_eye_in_hand/depth/image_raw', lambda m: got.setdefault('d', m), 1)
    c.node.create_subscription(CameraInfo, '/robot0_eye_in_hand/color/camera_info', lambda m: got.setdefault('i', m), 1)
    while 'd' not in got or 'i' not in got: c.spin()
    while not buf.can_transform('world', 'robot0_eye_in_hand_optical_frame', rclpy.time.Time()): c.spin()
    t = buf.lookup_transform('world', 'robot0_eye_in_hand_optical_frame', rclpy.time.Time())
    q = t.transform.rotation; tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    d = got['d']; D = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
    k = got['i'].k; fx, fy_, cx, cy = k[0], k[4], k[2], k[5]
    H, W = D.shape; v, u = np.mgrid[0:H, 0:W]
    pc = np.stack([(u-cx)*D/fx, (v-cy)*D/fy_, D], -1); R = quat_R(q.x, q.y, q.z, q.w); pw = pc@R.T+tr
    wx, wy, wz = pw[..., 0], pw[..., 1], pw[..., 2]
    tcp, quat = c.tcp(); Rh = quat_R(*quat); a = Rh[:, 2]; fy = Rh[:, 1]
    print('tcp', np.round(tcp, 4), 'a', np.round(a, 3), 'fy', np.round(fy, 3))
    rel = np.stack([wx-tcp[0], wy-tcp[1]], -1)
    along = rel[..., 0]*a[0]+rel[..., 1]*a[1]; lat = rel[..., 0]*fy[0]+rel[..., 1]*fy[1]
    m = (along > 0.02) & (along < 0.30) & (np.abs(lat) < 0.08) & (wz > 0.905) & (wz < 1.06)
    print('pts', m.sum())
    for z0 in np.arange(0.905, 1.05, 0.005):
        mm = m & (wz >= z0) & (wz < z0+0.005)
        if mm.sum() > 5:
            mc = mm & (np.abs(lat) < 0.008)
            near = along[mc].min() if mc.sum() else float('nan')
            print(f'z {z0:.3f}: lat[{lat[mm].min():+.4f},{lat[mm].max():+.4f}] mid={(lat[mm].min()+lat[mm].max())/2:+.4f} halfw={(lat[mm].max()-lat[mm].min())/2:.4f} nearest along={near:.4f} n={mm.sum()}')
    rclpy.shutdown()

main()
EOF
timeout 120 python3 /workspace/eih_profile.py 2>&1 | grep -v "^\[INFO"

# openrua op 58
timeout 600 python3 -c "
from ctl import *
c=Ctl()
Q=(0.4056,0.5792,-0.4056,0.5792)
tcp,quat=c.tcp(); R=quat_R(*quat); a=R[:,2]; fy=R[:,1]
axis=tcp+0.1325*a+0.015*fy; axis[2]=0.96
print('pot axis est',np.round(axis,4))
a0=np.array([0.342,-0.9397,0]); fy0=np.array([0.9397,0.342,0])
t1=axis-0.045*a0
q=c.ik(t1,Q); c.move_q(q,3); c.move_q(q,2)
print('tcp',np.round(c.tcp()[0],4),np.round(c.tcp()[1],4),'fingers',c.fingers())
np.save('axis.npy',axis)
" 2>&1 | grep -v "^\[INFO"; timeout 120 python3 /workspace/eih_profile.py 2>&1 | grep -v "^\[INFO" | grep -E "tcp|z 0.9[4-7]"

# openrua op 59
timeout 600 python3 -c "
from ctl import *
c=Ctl()
Q=(0.4056,0.5792,-0.4056,0.5792)
axis=np.load('axis.npy'); a0=np.array([0.342,-0.9397,0])
g=axis+0.01*a0
q=c.ik(g,Q); c.move_q(q,3); c.move_q(q,2)
print('tcp',np.round(c.tcp()[0],4),'fingers',c.fingers())
c.gripper(0.0)
print('fingers',c.fingers())
c.snap('agentview','agent11.png')
" 2>&1 | grep -v "^\[INFO"; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; python3 -c "
import numpy as np
from cloud import world_points
wx,wy,wz,D=world_points('birdview')
m=(wz>1.045)&(wx>-0.15)&(wx<0.05)&(wy<-0.15)&(wy>-0.4)
print('lid knob centroid', wx[m].mean().round(4), wy[m].mean().round(4), 'zmax', wz[m].max().round(3), m.sum())
"

# openrua op 60
find / -iname "*panda*gripper*" -o -iname "*moka*" 2>/dev/null | grep -v proc | head -20; pip list 2>/dev/null | grep -i -E "robosuite|robocasa|mujoco" 

# openrua op 61
cat > /workspace/eih_fingers.py <<'EOF'
"""Measure fingertip lateral extents relative to the hand line from the wrist depth camera."""
import numpy as np, rclpy
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from cloud import quat_R
from ctl import Ctl

c = Ctl('fing')
buf = Buffer(); TransformListener(buf, c.node)
got = {}
c.node.create_subscription(Image, '/robot0_eye_in_hand/depth/image_raw', lambda m: got.setdefault('d', m), 1)
c.node.create_subscription(CameraInfo, '/robot0_eye_in_hand/color/camera_info', lambda m: got.setdefault('i', m), 1)
while 'd' not in got or 'i' not in got: c.spin()
while not buf.can_transform('world', 'robot0_eye_in_hand_optical_frame', rclpy.time.Time()): c.spin()
t = buf.lookup_transform('world', 'robot0_eye_in_hand_optical_frame', rclpy.time.Time())
q = t.transform.rotation; tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
d = got['d']; D = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
k = got['i'].k; fx, fy_, cx, cy = k[0], k[4], k[2], k[5]
H, W = D.shape; v, u = np.mgrid[0:H, 0:W]
pc = np.stack([(u-cx)*D/fx, (v-cy)*D/fy_, D], -1); R = quat_R(q.x, q.y, q.z, q.w); pw = pc@R.T+tr
tcp, quat = c.tcp(); Rh = quat_R(*quat); a = Rh[:, 2]; fy = Rh[:, 1]; hx = Rh[:, 0]
rel = pw - tcp
along = rel@a; lat = rel@fy; up = rel@hx
print('tcp', np.round(tcp, 4), 'fingers', np.round(c.fingers(), 4), 'cam rel tcp: along %.3f lat %.3f hx %.3f' % ((tr-tcp)@a, (tr-tcp)@fy, (tr-tcp)@hx))
m = (along > -0.06) & (along < 0.02) & (np.abs(lat) < 0.07) & (np.abs(up) < 0.03) & (D < 0.2)
print('near pts', m.sum())
for lo, hi in [(-0.06, -0.05), (-0.05, -0.04), (-0.04, -0.03), (-0.03, -0.02), (-0.02, -0.01), (-0.01, 0.0), (0.0, 0.01)]:
    mm = m & (along >= lo) & (along < hi)
    if mm.sum() < 5: continue
    L = lat[mm]; U = up[mm]
    neg = L[L < 0]; pos = L[L > 0]
    print(f'along[{lo:+.2f},{hi:+.2f}) n={mm.sum()} lat- [{neg.min() if neg.size else np.nan:+.4f},{neg.max() if neg.size else np.nan:+.4f}]  lat+ [{pos.min() if pos.size else np.nan:+.4f},{pos.max() if pos.size else np.nan:+.4f}]  up[{U.min():+.4f},{U.max():+.4f}]')
rclpy.shutdown()
EOF
timeout 120 python3 /workspace/eih_fingers.py 2>&1 | grep -v "^\[INFO"

# openrua op 62
cat > lift_test.py <<'EOF'
import numpy as np, rclpy
from ctl import Ctl
c = Ctl('lift')
tcp, quat = c.tcp()
print('tcp', np.round(tcp,4), 'fingers', np.round(c.fingers(),4))
q = c.ik(tcp + [0,0,0.04], quat)
print('ik', q is not None)
if q: c.move_q(q, 3.0)
tcp2, _ = c.tcp(); print('tcp now', np.round(tcp2,4), 'fingers', np.round(c.fingers(),4))
c.snap('agentview', 'agent12.png'); c.snap('robot0_eye_in_hand', 'eih12.png')
rclpy.shutdown()
EOF
timeout 120 python3 lift_test.py 2>&1 | grep -v "^\[INFO"

# openrua op 63
cat > lidz.py <<'EOF'
import numpy as np, rclpy, sys
from sensor_msgs.msg import Image
from ctl import Ctl
from cloud import world_points
c = Ctl('lid')
got=[]
sub=c.node.create_subscription(Image,'/birdview/depth/image_raw',got.append,1)
while not got: c.spin()
m=got[0]; D=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.width); np.save('birdview_depth.npy',D)
x,y,z,_=world_points('birdview')
# pot region: near tcp, excluding the gripper: highest points in a 12cm box around the pot's old center
m2=(x>-0.13)&(x<0.03)&(y>-0.36)&(y<-0.17)
zz=z[m2]; xx=x[m2]; yy=y[m2]
order=np.argsort(-zz)[:20]
print('top pts in pot box:'); 
for i in order[:8]: print(round(xx[i],4),round(yy[i],4),round(zz[i],4))
# table-level check: any points at z in [0.9,0.92] inside pot base footprint?
base=(np.hypot(x+0.04,y+0.262)<0.03)
print('z hist under old pot center:', np.histogram(z[base],bins=[0.89,0.91,0.93,0.95,0.97,1.0,1.03,1.06,1.1,1.2])[0])
rclpy.shutdown()
EOF
timeout 60 python3 lidz.py 2>&1 | grep -v "^\[INFO"

# openrua op 64
cat > place.py <<'EOF'
import numpy as np, rclpy
from ctl import Ctl
c = Ctl('place')
tcp, quat = c.tcp(); print('start tcp', np.round(tcp,4), 'fingers', np.round(c.fingers(),4))
def go(p, secs=4.0):
    q = c.ik(p, quat, seed=c.arm_q())
    if q is None: raise SystemExit('IK failed for %s' % p)
    code, err = c.move_q(q, secs)
    if code != 0 or err > 0.02:
        code, err = c.move_q(q, secs)
    t,_ = c.tcp(); print('  tcp', np.round(t,4), 'fingers', np.round(c.fingers(),4))
    return q
go([tcp[0], tcp[1], 1.10], 4.0)                 # lift clear of pan handle
go([-0.045, -0.03, 1.10], 4.0)                  # midway
go([-0.05, 0.204, 1.10], 4.0)                   # over burner
c.snap('agentview', 'agent13.png')
rclpy.shutdown()
EOF
timeout 300 python3 place.py 2>&1 | grep -v "^\[INFO"

# openrua op 65
cat > place2.py <<'EOF'
import numpy as np, rclpy
from ctl import Ctl
from cloud import quat_R
c = Ctl('place2')
tcp, quat = c.tcp(); a = quat_R(*quat)[:,2]
print('tcp', np.round(tcp,4), 'a', np.round(a,3))
def go(p, secs=4.0):
    q = c.ik(p, quat, seed=c.arm_q())
    if q is None: raise SystemExit('IK failed')
    code, err = c.move_q(q, secs)
    if code != 0 or err > 0.02: code, err = c.move_q(q, secs)
    t,_ = c.tcp(); print('  tcp', np.round(t,4), 'fingers', np.round(c.fingers(),4), 'F', np.round(c.wrench()[0],1))
go([-0.05, 0.204, 1.03], 3.0)
go([-0.05, 0.204, 0.995], 3.0)
c.gripper(0.08)
c.spin(1.0)
go(np.array([-0.05, 0.204, 0.995]) - 0.09*a, 4.0)   # retreat along -a
print('fingers after retreat', np.round(c.fingers(),4))
go(np.array([-0.05, 0.204, 1.12]) - 0.12*a, 4.0)     # lift away
c.snap('agentview', 'agent14.png'); c.snap('birdview', 'bird14.png'); c.snap('frontview','front14.png')
rclpy.shutdown()
EOF
timeout 300 python3 place2.py 2>&1 | grep -v "^\[INFO"

# openrua op 66
cat > verify.py <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import Image
from ctl import Ctl
from cloud import world_points
c = Ctl('verify')
got=[]; sub=c.node.create_subscription(Image,'/birdview/depth/image_raw',got.append,1)
while not got: c.spin()
m=got[0]; D=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.width); np.save('birdview_depth.npy',D)
x,y,z,_=world_points('birdview')
# stove plate box
box=(x>-0.143)&(x<0.044)&(y>0.111)&(y<0.297)
zz=z[box]; print('stove box z hist', np.histogram(zz,bins=[0.9,0.92,0.935,0.96,1.0,1.03,1.045,1.06,1.2])[0])
top=box&(z>1.045)
print('lid knob top pts:', top.sum(), 'center', np.round([x[top].mean(),y[top].mean(),z[top].max()],4))
body=box&(z>0.99)&(z<1.045)
print('upper chamber center', np.round([x[body].mean(),y[body].mean()],4), 'x range', np.round([x[body].min(),x[body].max()],3), 'y range', np.round([y[body].min(),y[body].max()],3))
# knob region
kb=(np.hypot(x+0.203,y-0.203)<0.05)&(z>0.94)
print('knob bar pts', kb.sum(), 'x range', np.round([x[kb].min(),x[kb].max()],3), 'y range', np.round([y[kb].min(),y[kb].max()],3))
rclpy.shutdown()
EOF
timeout 60 python3 verify.py 2>&1 | grep -v "^\[INFO"

# openrua op 67
cat > verify2.py <<'EOF'
import numpy as np
from cloud import world_points
x,y,z,_=world_points('birdview')
box=(x>-0.143)&(x<0.044)&(y>0.111)&(y<0.297)
top=box&(z>1.045)
print(np.histogram(z[top],bins=[1.045,1.06,1.07,1.08,1.09,1.1,1.11,1.13])[0])
hi=box&(z>1.10); print('z>1.10 pts', hi.sum(), 'xy', np.round([x[hi].mean(),y[hi].mean()],3), 'x', np.round([x[hi].min(),x[hi].max()],3),'y', np.round([y[hi].min(),y[hi].max()],3))
lid=box&(z>1.06)&(z<1.10); print('lid pts', lid.sum(), 'xy center', np.round([x[lid].mean(),y[lid].mean()],4), 'zmax', round(z[lid].max(),4))
# pot body ring at z 1.0-1.04 (upper chamber) center & radius
ring=box&(z>1.0)&(z<1.04)&(np.hypot(x+0.055,y-0.2)<0.06)
print('upper chamber pts', ring.sum(), 'center', np.round([x[ring].mean(),y[ring].mean()],4), 'radius max', round(np.hypot(x[ring]-x[ring].mean(),y[ring]-y[ring].mean()).max(),4))
EOF
python3 verify2.py
