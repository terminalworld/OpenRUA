#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools /workspace/.ros

# openrua op 2
cat tools/action/ik_move.py tools/action/fjt_send.py tools/action/gripper_cmd.py

# openrua op 3
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -v parameter | head -40; echo ---; timeout 20 ros2 node list

# openrua op 5
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 60 python3 -c "
import rclpy, yaml
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
rclpy.init(); n=rclpy.create_node('tfq'); b=Buffer(); TransformListener(b,n)
import time
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.2)
print(b.all_frames_as_string())
for f in ['panda_link0','panda_hand','agentview_optical_frame','birdview_optical_frame','frontview_optical_frame']:
    try:
        t=b.lookup_transform('world',f,Time()).transform
        print(f, round(t.translation.x,4), round(t.translation.y,4), round(t.translation.z,4), '|', round(t.rotation.x,4), round(t.rotation.y,4), round(t.rotation.z,4), round(t.rotation.w,4))
    except Exception as e: print(f, 'ERR', e)
"

# openrua op 7
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera, save arrays, and print
world coords for requested pixels.  Usage: scene.py <cam> [u,v ...]"""
import sys, struct
import numpy as np, rclpy, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


def grab(node, topic, typ, timeout=30.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    cam = sys.argv[1]
    rclpy.init(); node = rclpy.create_node("scene")
    buf = Buffer(); TransformListener(buf, node)
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end and not buf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, Time()).transform
    T = np.eye(4); T[:3, :3] = quat_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    K = np.array(info.k).reshape(3, 3)
    cv2.imwrite(f"{cam}.png", color)
    np.save(f"{cam}_depth.npy", depth); np.save(f"{cam}_T.npy", T); np.save(f"{cam}_K.npy", K)
    print("K", K.tolist()); print("T", T.round(4).tolist())
    for a in sys.argv[2:]:
        u, v = map(int, a.split(","))
        z = depth[v, u]
        p = T @ np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, 1.0])
        print(f"px({u},{v}) depth={z:.4f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 scene.py birdview 280,262 312,245 345,278 247,283 390,285 320,400 320,150

# openrua op 9
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
img=cv2.imread('birdview.png')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
z=d
X=(u-K[0,2])*z/K[0,0]; Y=(v-K[1,2])*z/K[1,1]
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
wz=P[...,2]
# table height: mode near 0.48
tab=wz[(wz>0.4)&(wz<0.6)]
print('table z median', np.median(tab))
mask=(wz>0.50)&(wz<0.62)  # objects above table but below arm
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<20: continue
    m=lab==i
    cx,cy=cent[i]
    bgr=img[m].mean(0)
    print(f'comp {i} area={stats[i,4]} px=({cx:.0f},{cy:.0f}) world x={P[...,0][m].mean():.3f} y={P[...,1][m].mean():.3f} zmax={wz[m].max():.3f} bgr={bgr.round()}')
"


# openrua op 10
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
img=cv2.imread('birdview.png')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
z=d
P=np.stack([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z,np.ones_like(z)],-1)@T.T
wz=P[...,2]
print('bare table (320,330):', wz[330,320], ' (200,250):', wz[250,200])
h=np.histogram(wz[(wz>0.3)&(wz<0.7)],bins=40,range=(0.3,0.7))
for c,e in zip(*h): 
    if c>50: print(f'{e:.2f} {c}')
# rim analysis for each mug: pixels with wz>0.56
for name,(cx,cy) in {'white':(279,262),'red':(316,238),'yellow':(354,278)}.items():
    win=wz[cy-25:cy+25,cx-25:cx+25]
    m=win>0.555
    ys,xs=np.nonzero(m)
    xs=xs+cx-25; ys=ys+cy-25
    print(name,'rim px count',m.sum(),'center px',xs.mean().round(1),ys.mean().round(1),'world',P[...,0][ys,xs].mean().round(3),P[...,1][ys,xs].mean().round(3), 'xrange',P[...,0][ys,xs].min().round(3),P[...,0][ys,xs].max().round(3),'yrange',P[...,1][ys,xs].min().round(3),P[...,1][ys,xs].max().round(3))
"


# openrua op 11
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
img=cv2.imread('birdview.png')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)@T.T
wz=P[...,2]
crop=img[200:320,220:420].copy()
m=((wz>0.50)&(wz<0.62))[200:320,220:420]
crop[m]=(0,255,0)
m2=((wz>0.46)&(wz<0.50))[200:320,220:420]
crop[m2]=(255,0,0)
cv2.imwrite('bird_crop.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
# heights per column of mug: print the wz values on a horizontal line through white mug
np.set_printoptions(linewidth=250,precision=3)
print(wz[262,260:300])
print(wz[278,335:375])
print('plate row', wz[283,225:270])
"


# openrua op 12
timeout 120 python3 scene.py agentview 200,270 200,190 90,300 400,285 400,215 320,400 540,300 60,300 260,210 > /dev/null; python3 -c "
import numpy as np, cv2
d=np.load('agentview_depth.npy'); T=np.load('agentview_T.npy'); K=np.load('agentview_K.npy')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)@T.T
np.set_printoptions(linewidth=250,precision=3,suppress=True)
# vertical line through white mug center u=200, rows 170..300
for r in range(170,300,5): print(r, P[r,200].round(3), '| yellow u=410:', P[r,410].round(3))
print('plate left col 90:')
for r in range(260,320,4): print(r, P[r,90].round(3), P[r,540].round(3))
"


# openrua op 13
timeout 300 python3 tools/action/fjt_send.py 0,-0.785,0,-2.356,0,1.571,0.785 3 && timeout 30 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 14
timeout 300 python3 tools/action/fjt_send.py 0,-0.785,0,-2.356,0,1.571,0.785 2 && timeout 30 ros2 topic echo /joint_states --once | grep -A9 position && timeout 120 python3 scene.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 15
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
img=cv2.imread('birdview.png')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)@T.T
wz=P[...,2]
crop=img[200:320,220:420].copy()
m=((wz>0.50)&(wz<0.65))[200:320,220:420]; crop[m]=(0,255,0)
m2=((wz>0.46)&(wz<0.50))[200:320,220:420]; crop[m2]=(255,0,0)
cv2.imwrite('bird_crop.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
# inner-bottom components (0.46-0.50)
mask=((wz>0.46)&(wz<0.50)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<20: continue
    m=lab==i
    print(f'inner comp area={stats[i,4]} px=({cent[i][0]:.1f},{cent[i][1]:.1f}) world x={P[...,0][m].mean():.4f} y={P[...,1][m].mean():.4f} z={wz[m].mean():.3f} xr=[{P[...,0][m].min():.3f},{P[...,0][m].max():.3f}] yr=[{P[...,1][m].min():.3f},{P[...,1][m].max():.3f}]')
mask=((wz>0.50)&(wz<0.65)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<20: continue
    m=lab==i
    print(f'rim comp area={stats[i,4]} px=({cent[i][0]:.1f},{cent[i][1]:.1f}) world x={P[...,0][m].mean():.4f} y={P[...,1][m].mean():.4f} zmax={wz[m].max():.3f} xr=[{P[...,0][m].min():.3f},{P[...,0][m].max():.3f}] yr=[{P[...,1][m].min():.3f},{P[...,1][m].max():.3f}]')
"


# openrua op 16
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)@T.T
wz=P[...,2]
h=np.histogram(wz[(wz>0.40)&(wz<0.7)],bins=60,range=(0.40,0.7))
for c,e in zip(*h):
    if c>30: print(f'{e:.3f} {c}')
mask=((wz>0.43)&(wz<0.46)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<20: continue
    m=lab==i
    print(f'plate comp area={stats[i,4]} px=({cent[i][0]:.1f},{cent[i][1]:.1f}) world x={P[...,0][m].mean():.4f} y={P[...,1][m].mean():.4f} z={wz[m].mean():.3f} zmax={wz[m].max():.3f} xr=[{P[...,0][m].min():.3f},{P[...,0][m].max():.3f}] yr=[{P[...,1][m].min():.3f},{P[...,1][m].max():.3f}]')
"


# openrua op 17
timeout 120 python3 scene.py agentview >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('agentview_depth.npy'); T=np.load('agentview_T.npy'); K=np.load('agentview_K.npy')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)@T.T
wz=P[...,2]
np.set_printoptions(linewidth=250,precision=3,suppress=True)
for r in [225,240,255,265,275,285,295]:
    row=wz[r]; m=(row>0.45)&(row<0.6)
    # segments
    segs=[]; start=None
    for i in range(W):
        if m[i] and start is None: start=i
        if (not m[i]) and start is not None: segs.append((start,i-1)); start=None
    out=[]
    for a,b in segs:
        if b-a<3: continue
        out.append(f'u[{a}-{b}] y[{P[r,a,1]:.3f},{P[r,b,1]:.3f}] x[{P[r,a,0]:.3f},{P[r,b,0]:.3f}] z~{row[a:b].mean():.3f}')
    print(r, out)
# top rim rows: find min row per mug
for name,(u0,u1) in {'white':(150,260),'yellow':(370,460)}.items():
    m=(wz[:,u0:u1]>0.45)&(wz[:,u0:u1]<0.7)
    rows=np.nonzero(m.any(1))[0]
    print(name,'top row',rows.min(),'z at top', wz[rows.min(),u0:u1][m[rows.min()]].max(), 'bottom row', rows.max())
"


# openrua op 18
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Persistent-node arm controller helpers (IK/FK/trajectory/gripper/state).

Usage (CLI):
  ctl.py state                      # joints + hand/tcp pose (world)
  ctl.py fk                         # FK of current joints
  ctl.py tcp x y z qx qy qz qw [sec] # move TCP to world pose (IK+FJT)
  ctl.py grip open|close
  ctl.py wrench
"""
import sys, time
from pathlib import Path
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK

ROOT = Path(__file__).resolve().parent
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])   # panda_link0 in world (TF, identity rot)
DOWN_X = (0.7071068, 0.7071068, 0.0, 0.0)      # hand down, fingers close along world x
DOWN_Y = (1.0, 0.0, 0.0, 0.0)                  # hand down, fingers close along world y


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, next(s for s in M["sensors"] if s["kind"] == "wrench")["port"], self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.spin_until(lambda: self.js is not None, 15)

    def _on_js(self, m): self.js = m
    def _on_wr(self, m): self.wr = m

    def spin_until(self, pred, timeout):
        end = time.time() + timeout
        while not pred() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return pred()

    def joints(self):
        self.js = None
        self.spin_until(lambda: self.js is not None, 10)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in ARM], d.get("panda_finger_joint1"), d

    def wrench(self):
        self.wr = None
        self.spin_until(lambda: self.wr is not None, 10)
        f = self.wr.wrench.force
        return np.array([f.x, f.y, f.z])

    # ---- kinematics -------------------------------------------------
    def fk_hand(self, q=None):
        if q is None:
            q = self.joints()[0]
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = list(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP_OFF * quat_R(*quat)[:, 2]
        return pos, quat, tcp

    def ik_hand(self, pos_world, quat, seed=None):
        """IK for HAND frame pose in world. Returns joint list or None."""
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        b = np.asarray(pos_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        if seed is None:
            seed = self.joints()[0]
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = list(map(float, seed))
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK failed code={r and r.error_code.val} for {pos_world} {quat}")
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp(self, tcp_world, quat, seed=None):
        hand = np.asarray(tcp_world) - TCP_OFF * quat_R(*quat)[:, 2]
        return self.ik_hand(hand, quat, seed)

    # ---- motion -----------------------------------------------------
    def move_joints(self, q, seconds=3.0, retries=2):
        self.fjt.wait_for_server(10)
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM)
            pt = JointTrajectoryPoint(positions=list(map(float, q)))
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            gh = send.result()
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            cur = np.array(self.joints()[0])
            err = np.abs(cur - np.array(q)).max()
            print(f"  fjt code={code} max_joint_err={err:.4f}")
            if err < 0.02:
                return True
            seconds = max(1.5, seconds * 0.7)
        return err < 0.05

    def move_tcp(self, tcp_world, quat, seconds=3.0, seed=None):
        q = self.ik_tcp(tcp_world, quat, seed)
        if q is None:
            return False
        ok = self.move_joints(q, seconds)
        pos, qt, tcp = self.fk_hand()
        print(f"  tcp now {tcp.round(4)} (target {np.asarray(tcp_world).round(4)})")
        return ok

    def gripper(self, open_):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        gap = self.joints()[1]
        print(f"  gripper {'open' if open_ else 'close'}: reached={r.reached_goal} stalled={r.stalled} finger={gap:.4f}")
        return gap


def main():
    c = Ctl()
    cmd = sys.argv[1]
    if cmd == "state":
        q, f, _ = c.joints()
        print("joints", np.round(q, 4).tolist(), "finger", round(f, 4))
        pos, quat, tcp = c.fk_hand(q)
        print("hand", pos.round(4), np.round(quat, 4), "tcp", tcp.round(4))
    elif cmd == "tcp":
        v = list(map(float, sys.argv[2:9]))
        sec = float(sys.argv[9]) if len(sys.argv) > 9 else 3.0
        print(c.move_tcp(v[:3], v[3:7], sec))
    elif cmd == "grip":
        c.gripper(sys.argv[2] == "open")
    elif cmd == "wrench":
        print(c.wrench())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 19
timeout 120 python3 ctl.py state; timeout 60 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -8

# openrua op 20
timeout 120 python3 -c "
import numpy as np
from ctl import *
c=Ctl()
q0=c.joints()[0]
for label,pos in [('world',(-0.203,0,1.01)),('base',(0.307,0,0.59))]:
    c_pos=np.array(pos)+BASE_IN_WORLD  # ik_hand subtracts BASE_IN_WORLD, so cancel
    q=c.ik_hand(c_pos,(1,0,0,0),q0)
    print(label, None if q is None else np.round(q,3).tolist())
print('current', np.round(q0,3).tolist())
rclpy.shutdown()
"

# openrua op 21
python3 - <<'EOF'
import re
s=open('ctl.py').read()
s=s.replace('BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])   # panda_link0 in world (TF, identity rot)',
 'BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])   # verified: IK/FK with empty frame_id are in WORLD on this machine')
open('ctl.py','w').write(s)
EOF
timeout 120 python3 ctl.py state

# openrua op 22
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)@T.T
wz=P[...,2]
for name,(cx,cy) in {'white':(-0.109,-0.176),'yellow':(-0.054,0.1225)}.items():
    r=np.hypot(P[...,0]-cx,P[...,1]-cy)
    ring=(r<0.06)&(wz>0.50)
    print(name,'ring z percentiles',np.percentile(wz[ring],[50,75,90,98]).round(3), 'n',ring.sum())
    # radius of ring pixels with z>0.52
    rr=r[(r<0.07)&(wz>0.52)]
    print('  ring radius range',rr.min().round(3),rr.max().round(3), 'mean', rr.mean().round(3))
"


# openrua op 23
mkdir -p "$(dirname /workspace/pickplace.py)"
cat > /workspace/pickplace.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Rim-grasp pick and place of a mug.
Usage: pickplace.py <mug_cx> <mug_cy> <rim_top_z> <plate_cx> <plate_cy> [--dry]
Grasp: hand down, fingers along world x, TCP on the -x rim wall.
"""
import sys
import numpy as np, rclpy
from ctl import Ctl, DOWN_X

WALL_R = 0.044          # mid-wall radius of the mug rim
GRASP_DEPTH = 0.025     # fingertips this far below the rim top
SAFE_Z = 0.70
PLACE_TCP_Z = 0.545     # rim top when mug sits on plate ~0.56 -> tcp 0.535; release 1 cm above


def main():
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    cx, cy, rim, px, py = map(float, a)
    dry = "--dry" in sys.argv
    c = Ctl()
    q = DOWN_X
    gx = cx - WALL_R          # TCP x so that the wall sits between the fingers
    grasp_z = rim - GRASP_DEPTH
    wps = [
        ("pre-grasp", (gx, cy, SAFE_Z)),
        ("descend-1", (gx, cy, rim + 0.03)),
        ("grasp", (gx, cy, grasp_z)),
        ("lift", (gx, cy, SAFE_Z)),
        ("over-plate", (px - WALL_R, py, SAFE_Z)),
        ("lower-1", (px - WALL_R, py, PLACE_TCP_Z + 0.05)),
        ("place", (px - WALL_R, py, PLACE_TCP_Z)),
        ("retreat", (px - WALL_R, py, SAFE_Z)),
    ]
    # pre-solve IK chain
    seed = c.joints()[0]
    sols = {}
    for name, p in wps:
        s = c.ik_tcp(p, q, seed)
        if s is None:
            print("IK FAIL at", name, p); sys.exit(2)
        d = np.abs(np.array(s) - np.array(seed)).max() if seed is not None else 0
        print(f"IK {name:10s} {np.round(p,3)} -> {np.round(s,3).tolist()} (dmax {d:.2f})")
        sols[name] = s
        seed = s
    if dry:
        rclpy.shutdown(); return

    def go(name, secs=3.0):
        print(f"== {name}")
        ok = c.move_joints(sols[name], secs)
        pos, qt, tcp = c.fk_hand()
        print(f"   tcp {tcp.round(4)} target {np.round(dict(wps)[name],4)} ok={ok}")
        return ok

    f = c.joints()[1]
    if f < 0.035:
        c.gripper(True)
    go("pre-grasp", 4.0)
    go("descend-1", 2.5)
    go("grasp", 2.0)
    w0 = c.wrench()
    gap = c.gripper(False)
    print(f"   finger after close: {gap:.4f}  wrench {c.wrench().round(2)} (before {w0.round(2)})")
    if gap < 0.002:
        print("GRASP FAILED (closed on air); reopening and stopping")
        c.gripper(True); go("lift", 2.5); sys.exit(3)
    go("lift", 3.0)
    print(f"   holding: finger={c.joints()[1]:.4f} wrench {c.wrench().round(2)}")
    go("over-plate", 4.0)
    go("lower-1", 2.5)
    go("place", 2.0)
    c.gripper(True)
    go("retreat", 3.0)
    print("DONE")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 24
timeout 300 python3 pickplace.py -0.054 0.1225 0.541 -0.025 -0.309 --dry

# openrua op 25
python3 - <<'EOF'
s=open('pickplace.py').read()
s=s.replace('SAFE_Z = 0.70','SAFE_Z = 0.78')
s=s.replace('''        ("over-plate", (px - WALL_R, py, SAFE_Z)),''','''        ("mid", ((gx + px - WALL_R) / 2, (cy + py) / 2, SAFE_Z + 0.03)),
        ("over-plate", (px - WALL_R, py, SAFE_Z)),''')
s=s.replace('''    go("over-plate", 4.0)''','''    go("mid", 3.0)
    go("over-plate", 3.0)''')
open('pickplace.py','w').write(s)
EOF
nohup python3 -u pickplace.py -0.054 0.1225 0.541 -0.025 -0.309 > pp_yellow.log 2>&1 &
echo started

# openrua op 26
sleep 60; cat pp_yellow.log

# openrua op 27
sleep 45; tail -n +28 pp_yellow.log

# openrua op 28
timeout 120 python3 scene.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
img=cv2.imread('birdview.png')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)@T.T
wz=P[...,2]
mask=((wz>0.50)&(wz<0.65)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<20: continue
    m=lab==i
    print(f'rim comp area={stats[i,4]} px=({cent[i][0]:.1f},{cent[i][1]:.1f}) world x={P[...,0][m].mean():.4f} y={P[...,1][m].mean():.4f} zmax={wz[m].max():.3f} xr=[{P[...,0][m].min():.3f},{P[...,0][m].max():.3f}] yr=[{P[...,1][m].min():.3f},{P[...,1][m].max():.3f}]')
cv2.imwrite('bird_crop.png',cv2.resize(img[200:320,220:420],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 29
timeout 300 python3 tools/action/fjt_send.py 0,-0.785,0,-2.356,0,1.571,0.785 4; timeout 300 python3 tools/action/fjt_send.py 0,-0.785,0,-2.356,0,1.571,0.785 2; timeout 120 python3 scene.py birdview >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
img=cv2.imread('birdview.png')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)@T.T
wz=P[...,2]
for lo,hi,lab_ in [(0.50,0.65,'rim'),(0.43,0.46,'low')]:
    mask=((wz>lo)&(wz<hi)).astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
    for i in range(1,n):
        if stats[i,4]<20 or stats[i,4]>5000: continue
        m=lab==i
        print(f'{lab_} area={stats[i,4]} px=({cent[i][0]:.1f},{cent[i][1]:.1f}) world x={P[...,0][m].mean():.4f} y={P[...,1][m].mean():.4f} zmax={wz[m].max():.3f} xr=[{P[...,0][m].min():.3f},{P[...,0][m].max():.3f}] yr=[{P[...,1][m].min():.3f},{P[...,1][m].max():.3f}]')
for name,(cx,cy) in {'white':(-0.109,-0.176),'yellow':(-0.025,-0.309)}.items():
    r=np.hypot(P[...,0]-cx,P[...,1]-cy); ring=(r<0.07)&(wz>0.50)
    print(name,'ring z pct',np.percentile(wz[ring],[50,90,98]).round(3),'n',ring.sum())
cv2.imwrite('bird_crop.png',cv2.resize(img[200:320,220:420],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 30
nohup python3 -u pickplace.py -0.109 -0.176 0.549 -0.011 0.304 > pp_white.log 2>&1 & sleep 90; cat pp_white.log

# openrua op 31
sleep 15; tail -3 pp_white.log; timeout 300 python3 tools/action/fjt_send.py 0,-0.785,0,-2.356,0,1.571,0.785 4 >/dev/null; timeout 120 python3 scene.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
img=cv2.imread('birdview.png')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)@T.T
wz=P[...,2]
mask=((wz>0.50)&(wz<0.65)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<20 or stats[i,4]>5000: continue
    m=lab==i
    print(f'rim area={stats[i,4]} px=({cent[i][0]:.1f},{cent[i][1]:.1f}) world x={P[...,0][m].mean():.4f} y={P[...,1][m].mean():.4f} zmax={wz[m].max():.3f} xr=[{P[...,0][m].min():.3f},{P[...,0][m].max():.3f}] yr=[{P[...,1][m].min():.3f},{P[...,1][m].max():.3f}]')
cv2.imwrite('bird_crop.png',cv2.resize(img[200:320,220:420],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"
