#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---- && ros2 action list && echo ---- && ros2 service list | grep -iv parameter | head -40 && echo ---- && ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
ros2 topic echo /birdview/color/camera_info --once | head -30; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -60

# openrua op 6
python3 -c "
import numpy as np
d=np.load('birdview.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
fx=579.4112549695428; cx=320; cy=240
# world: x = -0.2 + (v-cy)*z/fy ; y = (u-cx)*z/fx ; zw = 3 - z
def w(u,v):
    z=d[v,u]; return (-0.2+(v-cy)*z/fx, (u-cx)*z/fx, 3-z)
# table height
print('table', w(100,400), w(500,150))
# caddy region rows 150-215, cols 210-310: print height map coarse
for v in range(150,215,3):
    print(v, ' '.join(f'{(3-d[v,u]):.2f}' for u in range(212,310,4)))
"


# openrua op 7
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png && ls

# openrua op 8
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
fx=579.4112549695428; cx=320; cy=240
def w(u,v):
    z=d[v,u]; return (round(-0.2+(v-cy)*z/fx,3), round((u-cx)*z/fx,3), round(3-z,3))
print('table', w(100,400), w(500,150))
print('    '+' '.join(f'{u:4d}' for u in range(212,312,4)))
for v in range(148,216,2):
    print(v, ' '.join(f'{(3-d[v,u]):.2f}' for u in range(212,312,4)))
print('cup center approx', w(318,262))
"


# openrua op 9
for c in agentview frontview sideview robot0_eye_in_hand; do echo "== $c"; timeout 15 ros2 run tf2_ros tf2_echo world ${c}_optical_frame 2>&1 | grep -A5 "Translation" | head -8; ros2 topic echo /$c/color/camera_info --once | grep -A1 "^k:" ; done

# openrua op 10
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab depth+intrinsics+TF for a camera, save world-frame point cloud as
<cam>_cloud.npy (N x 3) and print a top-down height map for a region.

Usage: python3 cloud.py <cam> [xmin xmax ymin ymax [res]]
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, mt, timeout=30.0):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
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
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    d = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    v, u = np.mgrid[0:depth.height, 0:depth.width]
    z = d
    X = (u - cx) * z / fx; Y = (v - cy) * z / fy
    P = np.stack([X, Y, z], -1).reshape(-1, 3)
    ok = np.isfinite(P[:, 2]) & (P[:, 2] > 0)
    W = (R @ P[ok].T).T + T
    np.save(f"{cam}_cloud.npy", W)
    uv = np.stack([u.reshape(-1)[ok], v.reshape(-1)[ok]], -1)
    np.save(f"{cam}_cloud_uv.npy", uv)
    print("saved", W.shape, "cam at", T)
    if len(sys.argv) >= 6:
        xmin, xmax, ymin, ymax = map(float, sys.argv[2:6])
        res = float(sys.argv[6]) if len(sys.argv) > 6 else 0.01
        sel = (W[:, 0] >= xmin) & (W[:, 0] < xmax) & (W[:, 1] >= ymin) & (W[:, 1] < ymax)
        S = W[sel]
        nx = int(round((xmax - xmin) / res)); ny = int(round((ymax - ymin) / res))
        H = np.full((nx, ny), np.nan)
        ix = ((S[:, 0] - xmin) / res).astype(int).clip(0, nx - 1)
        iy = ((S[:, 1] - ymin) / res).astype(int).clip(0, ny - 1)
        for a, b, zz in zip(ix, iy, S[:, 2]):
            if np.isnan(H[a, b]) or zz > H[a, b]:
                H[a, b] = zz
        print("rows = x from %.2f (top) to %.2f; cols = y from %.2f (left) to %.2f; cell=%.2fm; value=max z*100 (cm)" % (xmin, xmax, ymin, ymax, res))
        print("      " + " ".join(f"{ymin + j * res:5.2f}"[-3:] for j in range(ny)))
        for i in range(nx):
            row = " ".join(("  ." if np.isnan(H[i, j]) else f"{H[i, j] * 100:3.0f}") for j in range(ny))
            print(f"{xmin + i * res:5.2f} {row}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 11
timeout 120 python3 cloud.py agentview -0.55 -0.25 -0.40 -0.05 0.01 2>&1 | tail -40

# openrua op 12
python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[140:280,110:390]; cv2.imwrite('caddy_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('robot0_eye_in_hand.png'); crop=im[270:420,180:640]; cv2.imwrite('caddy_crop2.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 13
timeout 120 python3 cloud.py robot0_eye_in_hand -0.50 -0.30 -0.40 -0.02 0.01 2>&1 | tail -30

# openrua op 14
python3 - <<'EOF'
import numpy as np
W=np.load('robot0_eye_in_hand_cloud.npy')
def hmap(xmin,xmax,ymin,ymax,res=0.01):
    sel=(W[:,0]>=xmin)&(W[:,0]<xmax)&(W[:,1]>=ymin)&(W[:,1]<ymax); S=W[sel]
    nx=int(round((xmax-xmin)/res)); ny=int(round((ymax-ymin)/res))
    H=np.full((nx,ny),np.nan)
    ix=((S[:,0]-xmin)/res).astype(int).clip(0,nx-1); iy=((S[:,1]-ymin)/res).astype(int).clip(0,ny-1)
    for a,b,z in zip(ix,iy,S[:,2]):
        if np.isnan(H[a,b]) or z>H[a,b]: H[a,b]=z
    print("      "+" ".join(f"{ymin+j*res:5.2f}"[-3:] for j in range(ny)))
    for i in range(nx):
        print(f"{xmin+i*res:5.2f} "+" ".join(("  ." if np.isnan(H[i,j]) else f"{H[i,j]*100:3.0f}") for j in range(ny)))
print("cup region"); hmap(-0.20,-0.04,-0.10,0.08)
print("caddy right end"); hmap(-0.48,-0.30,-0.10,0.06)
# cup top rim points
sel=(W[:,2]>0.93)&(W[:,0]>-0.2)&(W[:,0]<-0.04)&(np.abs(W[:,1])<0.1)
S=W[sel]; print("rim pts", len(S), "x range", S[:,0].min(), S[:,0].max(), "y range", S[:,1].min(), S[:,1].max(), "zmax", S[:,2].max())
EOF

# openrua op 15
python3 - <<'EOF'
import numpy as np
for cam in ['agentview','robot0_eye_in_hand']:
    W=np.load(f'{cam}_cloud.npy')
    sel=(W[:,0]>-0.2)&(W[:,0]<-0.04)&(np.abs(W[:,1])<0.1)&(W[:,2]>0.885)
    S=W[sel]
    print(cam, 'cup pts', len(S), 'zmax', S[:,2].max())
    for lo in np.arange(0.88,1.06,0.02):
        s=S[(S[:,2]>=lo)&(S[:,2]<lo+0.02)]
        if len(s): print(f'  z {lo:.2f}-{lo+0.02:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
EOF

# openrua op 16
ros2 node info /robot_bridge 2>&1 | head -60; ros2 param list /robot_bridge 2>&1 | head

# openrua op 17
timeout 120 python3 cloud.py frontview >/dev/null && timeout 120 python3 cloud.py sideview > /dev/null && python3 - <<'EOF'
import numpy as np
W=np.concatenate([np.load(f'{c}_cloud.npy') for c in ['agentview','robot0_eye_in_hand','frontview','sideview']])
def hmap(xmin,xmax,ymin,ymax,res=0.01):
    sel=(W[:,0]>=xmin)&(W[:,0]<xmax)&(W[:,1]>=ymin)&(W[:,1]<ymax); S=W[sel]
    nx=int(round((xmax-xmin)/res)); ny=int(round((ymax-ymin)/res))
    H=np.full((nx,ny),np.nan)
    ix=((S[:,0]-xmin)/res).astype(int).clip(0,nx-1); iy=((S[:,1]-ymin)/res).astype(int).clip(0,ny-1)
    for a,b,z in zip(ix,iy,S[:,2]):
        if np.isnan(H[a,b]) or z>H[a,b]: H[a,b]=z
    print("      "+" ".join(f"{ymin+j*res:6.3f}"[-3:] for j in range(ny)))
    for i in range(nx):
        print(f"{xmin+i*res:6.3f} "+" ".join(("  ." if np.isnan(H[i,j]) else f"{H[i,j]*100:3.0f}") for j in range(ny)))
hmap(-0.50,-0.31,-0.37,0.12,0.01)
EOF

# openrua op 18
find / -iname "*caddy*" 2>/dev/null | head; find / -iname "*libero*" -maxdepth 6 2>/dev/null | head; find / -name "*.bddl" 2>/dev/null | head; ls /opt /home /usr/local/lib 2>/dev/null

# openrua op 19
ls -la /home/robot /workspace/.ros/log | head -30; grep -il "caddy\|region\|mug\|libero" /workspace/.ros/log/* 2>/dev/null | head; pip list 2>/dev/null | grep -i -E "libero|robosuite|mujoco" 

# openrua op 20
timeout 15 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 Translation | head -3; timeout 15 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -A2 Translation | head -3; timeout 15 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A2 Translation | head -3

# openrua op 21
python3 - <<'EOF'
import numpy as np
E=np.load('robot0_eye_in_hand_cloud.npy'); A=np.load('agentview_cloud.npy'); F=np.load('frontview_cloud.npy'); S=np.load('sideview_cloud.npy')
def sl(W,lo,hi):
    s=W[(W[:,0]>-0.22)&(W[:,0]<-0.03)&(np.abs(W[:,1])<0.1)&(W[:,2]>=lo)&(W[:,2]<hi)]
    return s
print(" z      eye:-x   agent:+x  front:+x  side:y+  side:y-  ")
for lo in np.arange(0.88,1.05,0.01):
    e=sl(E,lo,lo+0.01); a=sl(A,lo,lo+0.01); f=sl(F,lo,lo+0.01); s=sl(S,lo,lo+0.01)
    row=f"{lo:.2f} "
    row+=f"{e[:,0].min():8.3f}" if len(e) else "      --"
    row+=f"{a[:,0].max():8.3f}" if len(a) else "      --"
    row+=f"{f[:,0].max():8.3f}" if len(f) else "      --"
    row+=f"{s[:,1].max():8.3f}{s[:,1].min():8.3f}" if len(s) else "      --      --"
    print(row)
EOF

# openrua op 22
python3 - <<'EOF'
import numpy as np
for cam in ['agentview','frontview','sideview']:
    W=np.load(f'{cam}_cloud.npy')
    s=W[(W[:,0]>-0.50)&(W[:,0]<-0.32)&(W[:,2]>0.95)&(W[:,2]<1.10)&(W[:,1]>-0.5)&(W[:,1]<0.5)]
    print(cam, 'caddy-ish points y range', s[:,1].min(), s[:,1].max(), ' x range', s[:,0].min(), s[:,0].max())
    # right compartment interior floor
    f=W[(W[:,0]>-0.46)&(W[:,0]<-0.35)&(W[:,2]>0.895)&(W[:,2]<0.915)&(W[:,1]>-0.06)&(W[:,1]<0.3)]
    if len(f): print('   right compartment floor y range', f[:,1].min(), f[:,1].max())
    # book
    b=W[(W[:,0]>-0.45)&(W[:,0]<-0.2)&(W[:,1]>0.05)&(W[:,1]<0.3)&(W[:,2]>0.9)]
    if len(b): print('   book x',b[:,0].min(),b[:,0].max(),'y',b[:,1].min(),b[:,1].max(),'z',b[:,2].max())
EOF

# openrua op 23
ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; ros2 interface show control_msgs/action/FollowJointTrajectory | grep -A3 "path_tolerance\|goal_tolerance\|goal_time" | head -20

# openrua op 24
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Robot helper library for this Panda workstation.

World frame = 'world'; MoveIt IK/FK operate in the arm base frame
(panda_link0), which sits at BASE_IN_WORLD. All public functions take
and return WORLD-frame poses.

Import and use:
    from rob import Robot
    r = Robot()
    r.joints()                 -> dict name->pos
    r.hand_pose()              -> (xyz, quat_xyzw) of panda_hand in world (FK)
    r.ik(xyz, quat)            -> list of 7 joint positions or None
    r.move_joints(q, seconds)  -> error_code
    r.move_pose(xyz, quat, seconds) -> (ok, info)
    r.gripper(width_per_finger)-> (reached_goal, stalled, gap)
    r.wrench()                 -> (force xyz, torque xyz)
"""
import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])
ARM = [f"panda_joint{i}" for i in range(1, 8)]
FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]
TCP_OFFSET = 0.1034  # hand frame -> fingertip point along hand +z


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    m = R
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# Hand orientation with fingers along world X and approach (hand +z)
# pointing straight down: R = rot_x(pi) gives hand x=+X, y=-Y, z=-Z.
R_DOWN = rot_x(math.pi)


def hand_R(yaw=0.0, tilt_axis=None, tilt=0.0):
    """Top-down hand, yawed about world Z by `yaw` (finger axis = X
    rotated by yaw), then tilted about a world axis by `tilt` rad."""
    R = rot_z(yaw) @ R_DOWN
    if tilt_axis is not None and tilt != 0.0:
        ax = np.asarray(tilt_axis, float)
        ax = ax / np.linalg.norm(ax)
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        Rt = np.eye(3) + math.sin(tilt) * K + (1 - math.cos(tilt)) * K @ K
        R = Rt @ R
    return R


class Robot:
    def __init__(self, name="rob"):
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

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        end = time.time() + 10
        while self._js is None and time.time() < end:
            self.spin(0.1)
        if self._js is None:
            raise RuntimeError("no /joint_states")
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j[FINGERS[0]] - j[FINGERS[1]] if j[FINGERS[1]] < 0 else j[FINGERS[0]] + j[FINGERS[1]]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            self.spin(0.1)
        if self._wr is None:
            return None
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk(self, q=None, link="panda_hand"):
        if q is None:
            q = self.arm_q()
        if not self.fk_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return xyz, q

    def hand_pose(self):
        return self.fk()

    def tcp_pose(self):
        xyz, q = self.fk()
        R = quat_to_R(q)
        return xyz + TCP_OFFSET * R[:, 2], q

    # ---------------- IK ----------------
    def ik(self, xyz_world, quat_xyzw, seed=None, attempts=3, timeout=2.0):
        if not self.ik_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        if seed is None:
            seed = self.arm_q()
        p_base = np.asarray(xyz_world, float) - BASE_IN_WORLD
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.pose_stamped.header.frame_id = ""
            pp = req.ik_request.pose_stamped.pose
            pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
            pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, quat_xyzw)
            req.ik_request.robot_state.joint_state.name = ARM
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[n] for n in ARM]
        return None

    def ik_tcp(self, tcp_xyz_world, R, **kw):
        """IK for a fingertip (TCP) position with hand rotation matrix R."""
        R = np.asarray(R)
        hand_xyz = np.asarray(tcp_xyz_world, float) - TCP_OFFSET * R[:, 2]
        return self.ik(hand_xyz, R_to_quat(R), **kw)

    # ---------------- motion ----------------
    def move_joints(self, q, seconds=3.0, waypoints=None):
        """Send one trajectory (optionally several waypoints, each a
        (q, t) pair) and wait. Returns error_code (0 = ok)."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if waypoints is None:
            waypoints = [(q, seconds)]
        for qq, t in waypoints:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("FJT goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = r.result.error_code if r is not None else -99
        return code

    def move_pose(self, xyz_world, quat_xyzw, seconds=3.0, seed=None):
        q = self.ik(xyz_world, quat_xyzw, seed=seed)
        if q is None:
            return False, "IK failed"
        code = self.move_joints(q, seconds)
        now = np.array(self.arm_q())
        err = np.abs(now - np.array(q)).max()
        return code == 0 and err < 0.02, f"code={code} max_joint_err={err:.4f}"

    def move_tcp(self, tcp_xyz_world, R, seconds=3.0, seed=None):
        R = np.asarray(R)
        hand_xyz = np.asarray(tcp_xyz_world, float) - TCP_OFFSET * R[:, 2]
        return self.move_pose(hand_xyz, R_to_quat(R), seconds, seed)

    def gripper(self, width, effort=30.0):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(effort)
        send = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result().result
        j = self.joints()
        return res.reached_goal, res.stalled, (j[FINGERS[0]], j[FINGERS[1]])

    def close(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
OPENRUA_EOF

# openrua op 25
timeout 120 python3 -c "
from rob import *
r=Robot()
print('joints', r.joints())
xyz,q=r.hand_pose(); print('hand world', xyz, q)
print('tcp', r.tcp_pose()[0])
# IK sanity: same pose
sol=r.ik(xyz,q); print('ik same pose', sol)
print('cur', r.arm_q())
print('wrench', r.wrench())
r.close()
"

# openrua op 26
sed -i 's/^BASE_IN_WORLD = np.array(\[-0.75, 0.0, 0.912\])/BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK\/IK model frame == world here (verified)/' rob.py && grep -n "BASE_IN_WORLD =" rob.py && timeout 120 python3 -c "
from rob import *
r=Robot()
xyz,q=r.hand_pose(); print('hand world', xyz, q)
sol=[0.058462119889661025, -0.16130609283377492, -0.05791992799008485, -2.4445840518080595, -0.01162024727377228, 2.226700917731669, 0.008905361657217607]
print('fk of ik sol', r.fk(sol))
r.close()
"

# openrua op 27
python3 - <<'EOF'
import re
s=open('rob.py').read()
s=s.replace('''        p_base = np.asarray(xyz_world, float) - BASE_IN_WORLD
        for _ in range(attempts):''','''        p_base = np.asarray(xyz_world, float) - BASE_IN_WORLD
        # IK tip link is panda_link8; panda_hand = link8 * rot_z(-pi/4)
        # (same origin). Convert the requested HAND orientation.
        R8 = quat_to_R(quat_xyzw) @ rot_z(math.pi / 4)
        quat_xyzw = R_to_quat(R8)
        for _ in range(attempts):''')
open('rob.py','w').write(s)
EOF
timeout 120 python3 -c "
from rob import *
r=Robot()
xyz,q=r.hand_pose(); print('hand world', xyz, q)
sol=r.ik(xyz,q); print('ik', sol)
print('fk of ik sol', r.fk(sol))
r.close()
"

# openrua op 28
timeout 300 python3 -c "
from rob import *
r=Robot()
print('gripper open ->', r.gripper(0.04))
print('gap', r.finger_gap())
r.close()
" && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 cloud.py robot0_eye_in_hand > /dev/null && timeout 120 python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
E=np.load('robot0_eye_in_hand_cloud.npy'); A=np.load('agentview_cloud.npy')
def sl(W,lo,hi):
    return W[(W[:,0]>-0.22)&(W[:,0]<-0.03)&(np.abs(W[:,1])<0.1)&(W[:,2]>=lo)&(W[:,2]<hi)]
print(" z     eye:-x  eye:+x  agent:+x  agent:-x  y-  y+")
for lo in np.arange(0.88,1.02,0.01):
    e=sl(E,lo,lo+0.01); a=sl(A,lo,lo+0.01)
    row=f"{lo:.2f} "
    row+=f"{e[:,0].min():8.3f}{e[:,0].max():8.3f}" if len(e) else "      --      --"
    row+=f"{a[:,0].max():8.3f}{a[:,0].min():8.3f}" if len(a) else "      --      --"
    allp=np.concatenate([e,a]) if len(e) or len(a) else e
    row+=f"{allp[:,1].min():8.3f}{allp[:,1].max():8.3f}" if len(allp) else ""
    print(row)
EOF

# openrua op 29
cat > pick.py <<'EOF'
from rob import *
import sys
r = Robot()
cx, cy = -0.122, -0.001          # cup center (world)
rim_r, rim_z = 0.050, 0.985
R = hand_R(0.0)                  # fingers along world X, approach down
wall_x = cx + rim_r - 0.001      # pinch point on +x side of the rim
pre = np.array([wall_x, cy, 1.12])
grasp = np.array([wall_x, cy, rim_z - 0.015])

print("pre-grasp", pre)
ok, info = r.move_tcp(pre, R, 4.0); print(" ->", ok, info)
if not ok: sys.exit(1)
print("tcp now", r.tcp_pose()[0])
print("descend", grasp)
ok, info = r.move_tcp(grasp, R, 3.0); print(" ->", ok, info)
print("tcp now", r.tcp_pose()[0])
w0 = r.wrench(); print("wrench before close", w0)
print("close ->", r.gripper(0.0))
print("gap after close", r.finger_gap())
print("wrench after close", r.wrench())
EOF
timeout 900 python3 -u pick.py

# openrua op 30
timeout 120 python3 -c "
from rob import *
r=Robot()
print('tcp', r.tcp_pose())
print('q', r.arm_q())
r.close()"

# openrua op 31
python3 - <<'EOF'
s=open('rob.py').read()
s=s.replace('''        code = self.move_joints(q, seconds)
        now = np.array(self.arm_q())
        err = np.abs(now - np.array(q)).max()
        return code == 0 and err < 0.02, f"code={code} max_joint_err={err:.4f}"''','''        code = self.move_joints(q, seconds)
        now = np.array(self.arm_q())
        err = np.abs(now - np.array(q)).max()
        if err > 0.01:  # controller lag: re-send the same goal once to converge
            code = self.move_joints(q, max(1.5, seconds / 2))
            now = np.array(self.arm_q())
            err = np.abs(now - np.array(q)).max()
        xyz_now, _ = self.fk()
        cart = np.linalg.norm(xyz_now - np.asarray(xyz_world, float))
        return code == 0 and err < 0.02, f"code={code} max_joint_err={err:.4f} cart_err={cart*1000:.1f}mm"''')
open('rob.py','w').write(s)
EOF
sed -i 's/^if not ok: sys.exit(1)$/if not ok: print("WARN: not converged")/' pick.py
timeout 900 python3 -u pick.py

# openrua op 32
timeout 60 python3 -c "
from rob import *
r=Robot(); print(r.joints()); r.close()"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 33
python3 -u tools/perception/cam_snap.py sideview && python3 -u tools/perception/cam_snap.py frontview && python3 -u cloud.py birdview -0.25 0.05 -0.10 0.10 0.01 2>&1 | tail -40

# openrua op 34
python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('side_crop.png', cv2.resize(im[200:340,260:420], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))
im=cv2.imread('frontview.png'); print(im.shape)
"

# openrua op 35
python3 -u -c "
from rob import *
r=Robot()
print('before', r.finger_gap(), r.wrench())
print(r.gripper(0.0))
print('after', r.finger_gap(), r.wrench())
print('tcp', r.tcp_pose()[0])
r.close()
"

# openrua op 36
python3 -u tools/perception/cam_snap.py sideview && python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('side_crop.png', cv2.resize(im[200:340,260:420], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))"

# openrua op 37
python3 -u -c "
from rob import *
r=Robot()
print(r.move_tcp([-0.072,-0.001,1.22], hand_R(0.0), 4.0))
print('gap', r.finger_gap(), 'wrench', r.wrench()[0])
print('tcp', r.tcp_pose()[0])
r.close()
" && python3 -u tools/perception/cam_snap.py sideview && python3 -u tools/perception/cam_snap.py agentview

# openrua op 38
python3 -u -c "
from rob import *
r=Robot()
print(r.gripper(0.08))
print(r.move_tcp([-0.35,-0.10,1.30], hand_R(0.0), 4.0))
r.close()
" && python3 -u cloud.py birdview -0.25 0.05 -0.15 0.10 0.01 2>&1 | tail -34

# openrua op 39
python3 -c "
import numpy as np
W=np.load('birdview_cloud.npy')
sel=(W[:,0]>-0.20)&(W[:,0]<-0.07)&(W[:,1]>-0.07)&(W[:,1]<0.07)&(W[:,2]>0.975)
P=W[sel]
print(len(P), 'rim z range', P[:,2].min(), P[:,2].max())
# fit circle in xy
x,y=P[:,0],P[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
c=np.linalg.lstsq(A,b,rcond=None)[0]
cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx*cx+cy*cy)
print('center',cx,cy,'r',r)
sel2=(W[:,0]>-0.20)&(W[:,0]<-0.07)&(W[:,1]>-0.07)&(W[:,1]<0.07)&(W[:,2]>0.885)&(W[:,2]<0.93)
Q=W[sel2]; print('floor z', np.percentile(Q[:,2],[5,50,95]), 'x range', Q[:,0].min(), Q[:,0].max())
"

# openrua op 40
grep -n "def hand_R" -A 14 rob.py

# openrua op 41
mkdir -p "$(dirname /workspace/pick2.py)"
cat > /workspace/pick2.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Rim pinch on the cup's +x wall with the finger pads tilted to match the
cup's wall taper (rim r 5.0 cm -> base r 3.75 cm over 10.5 cm ~ 6.8 deg)."""
import math
import numpy as np
from rob import *

cx, cy = -0.128, -0.0004      # rim circle centre (birdview fit)
rim_z = 0.990
taper = math.atan2(0.0125, 0.105)   # ~6.8 deg
depth = 0.025                       # pinch this far below the rim
outer_r = 0.050 - depth * math.tan(taper)
wall_x = cx + outer_r - 0.0023      # wall centreline

R = hand_R(0.0, tilt_axis=[0, 1, 0], tilt=taper)   # hand z leans toward -x
print("hand z axis", R[:, 2])
r = Robot()
print("open", r.gripper(0.08))
pre = [wall_x, cy, 1.15]
grasp = [wall_x, cy, rim_z - depth]
print("pre", pre, r.move_tcp(pre, R, 4.0))
print("grasp", grasp, r.move_tcp(grasp, R, 3.0))
print("tcp", r.tcp_pose()[0], "wrench", r.wrench()[0])
for i in range(3):
    print("close", r.gripper(0.0), "gap", r.finger_gap())
print("wrench", r.wrench()[0])
r.close()
OPENRUA_EOF

# openrua op 42
python3 -u pick2.py 2>&1 | grep -v "^\[" 

# openrua op 43
python3 -u tools/perception/cam_snap.py robot0_eye_in_hand && python3 -u tools/perception/cam_snap.py sideview && python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('side_crop.png', cv2.resize(im[200:340,260:420], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))"

# openrua op 44
python3 -u -c "
from rob import *
r=Robot()
for i in range(6):
    g=r.gripper(0.0); print(i, g[2], 'gap %.4f'%r.finger_gap(), 'F', np.round(r.wrench()[0],2))
r.close()
" 2>&1 | grep -v "^\["

# openrua op 45
python3 -u -c "
from rob import *
r=Robot()
print(r.gripper(0.08)); print(r.gripper(0.08))
print(r.move_tcp([-0.083,-0.0004,1.15], hand_R(0.0, [0,1,0], 0.118), 3.0))
print(r.move_tcp([-0.35,-0.10,1.30], hand_R(0.0), 4.0))
r.close()
" 2>&1 | grep -v "^\[" && python3 -c "
import subprocess, numpy as np
subprocess.run(['python3','cloud.py','birdview'],check=True,capture_output=True)
W=np.load('birdview_cloud.npy')
sel=(W[:,0]>-0.25)&(W[:,0]<-0.05)&(W[:,1]>-0.08)&(W[:,1]<0.08)&(W[:,2]>0.975)
P=W[sel]; x,y=P[:,0],P[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rr=np.sqrt(c[2]+cx*cx+cy*cy)
print(len(P),'rim center',cx,cy,'r',rr,'zmax',P[:,2].max())
"

# openrua op 46
sed -i 's/^cx, cy = .*/cx, cy = -0.134, 0.0        # rim circle centre (birdview fit)/; s/^R = hand_R(0.0, tilt_axis/R = hand_R(math.pi \/ 2, tilt_axis/' pick2.py && sed -i 's/^print("hand z axis", R\[:, 2\])/print("hand z axis", R[:, 2], "finger axis (hand y)", R[:, 1])/' pick2.py && grep -n "hand_R\|cx, cy" pick2.py && python3 -u pick2.py 2>&1 | grep -v "^\["

# openrua op 47
python3 -u tools/perception/cam_snap.py robot0_eye_in_hand && python3 -u tools/perception/cam_snap.py sideview && python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('side_crop.png', cv2.resize(im[200:340,260:420], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))"

# openrua op 48
python3 -u -c "
from rob import *
import math
R = hand_R(math.pi/2, [0,1,0], math.atan2(0.0125,0.105))
r=Robot()
print(r.move_tcp([-0.0893,0.0,1.03], R, 3.0))
print('gap %.4f'%r.finger_gap(), 'F', np.round(r.wrench()[0],2))
print(r.move_tcp([-0.0893,0.0,1.22], R, 3.0))
print('gap %.4f'%r.finger_gap(), 'F', np.round(r.wrench()[0],2))
r.close()
" 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py sideview

# openrua op 49
python3 -u cloud.py birdview -0.51 -0.30 -0.36 0.10 0.01 2>&1 | tail -24

# openrua op 50
mkdir -p "$(dirname /workspace/place.py)"
cat > /workspace/place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Carry the cup (held by its wall) to the caddy's left compartment,
handle pointing -x (toward the back wall), and release it just above
the floor.  Run steps one at a time: python3 place.py <step>"""
import math
import sys
import numpy as np
from rob import *

taper = math.atan2(0.0125, 0.105)
R_grasp = hand_R(math.pi / 2, [0, 1, 0], taper)   # current: cup at -x of TCP
R_place = hand_R(0.0, [1, 0, 0], taper)           # after -90 deg yaw: cup at +y of TCP
print("R_place hand z", np.round(R_place[:, 2], 3), "finger axis", np.round(R_place[:, 1], 3))

cup_xy = np.array([-0.385, -0.272])       # target cup centre in the left compartment
tcp_xy = cup_xy + np.array([0.0, -0.0447])  # grasp point is on the cup's -y wall after the yaw

step = sys.argv[1]
r = Robot()
q0 = r.arm_q()
print("q0", np.round(q0, 3), "gap %.4f" % r.finger_gap())

if step == "rotate":
    tcp = r.tcp_pose()[0]
    q = r.ik_tcp(tcp, R_place, seed=q0)
    print("ik", None if q is None else np.round(q, 3))
    if q is not None:
        print("max joint delta", np.max(np.abs(np.array(q) - q0)))
        if np.max(np.abs(np.array(q) - q0)) < 2.0:
            print(r.move_joints(q, 5.0))
            print("tcp now", r.tcp_pose()[0], "gap %.4f" % r.finger_gap())
elif step == "carry":
    print(r.move_tcp([tcp_xy[0], tcp_xy[1], 1.22], R_place, 5.0))
    print("gap %.4f" % r.finger_gap(), "F", np.round(r.wrench()[0], 2))
elif step == "lower":
    for z in (1.08, 1.03, 1.00):
        print(z, r.move_tcp([tcp_xy[0], tcp_xy[1], z], R_place, 2.5))
        print("  gap %.4f" % r.finger_gap(), "F", np.round(r.wrench()[0], 2))
elif step == "release":
    print(r.gripper(0.08)); print(r.gripper(0.08))
    print("gap %.4f" % r.finger_gap())
    print(r.move_tcp([tcp_xy[0], tcp_xy[1], 1.25], R_place, 3.0))
    print(r.move_tcp([-0.20, -0.30, 1.30], hand_R(0.0), 4.0))
r.close()
OPENRUA_EOF

# openrua op 51
grep -n "def ik_tcp\|def ik(\|def move_joints\|def arm_q" rob.py && python3 -u place.py rotate 2>&1 | grep -v "^\["

# openrua op 52
python3 -u -c "
from rob import *
r=Robot()
print('q', np.round(r.arm_q(),3)); print('tcp', r.tcp_pose()[0], 'gap %.4f'%r.finger_gap())
r.close()
" 2>&1 | grep -v "^\["; ros2 interface show control_msgs/action/FollowJointTrajectory 2>/dev/null | grep -n "int32" 

# openrua op 53
python3 -u -c "
from rob import *
r=Robot()
q=[0.203,0.375,-0.273,-1.627,0.237,1.973,0.638]
for i in range(3):
    code=r.move_joints(q, 3.0); now=np.array(r.arm_q())
    print(i, 'code', code, 'err', np.round(now-np.array(q),3))
    if np.abs(now-np.array(q)).max()<0.01: break
print('tcp', r.tcp_pose()[0], 'gap %.4f'%r.finger_gap(), 'hand R', np.round(quat_to_R(r.hand_pose()[1]),2).tolist())
r.close()
" 2>&1 | grep -v "^\["

# openrua op 54
python3 -u tools/perception/cam_snap.py sideview && python3 -u place.py carry 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py agentview

# openrua op 55
python3 -u tools/perception/cam_snap.py robot0_eye_in_hand && python3 -u tools/perception/cam_snap.py frontview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[0:160,0:220], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))"

# openrua op 56
python3 -u place.py lower 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[60:260,60:300], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))"

# openrua op 57
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R_place = hand_R(0.0, [1,0,0], taper)
r=Robot()
print(r.move_tcp([-0.385,-0.3167,1.22], R_place, 3.0))
print('gap %.4f'%r.finger_gap(), 'F', np.round(r.wrench()[0],2), 'tcp', r.tcp_pose()[0])
r.close()
" 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[0:260,0:300], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))"

# openrua op 58
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R_place = hand_R(0.0, [1,0,0], taper)
r=Robot()
print(r.move_tcp([-0.15,-0.05,1.22], R_place, 4.0))
for z in (1.05, 1.00, 0.97, 0.95):
    ok,info=r.move_tcp([-0.15,-0.05,z], R_place, 2.0)
    F=r.wrench()[0]; print(z, info, 'F', np.round(F,2))
    if abs(F[2]+5.3)>2.0 or abs(F[1])>2.0: break
r.close()
" 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py sideview && python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('side_crop.png', cv2.resize(im[100:340,180:460], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))"

# openrua op 59
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R_place = hand_R(0.0, [1,0,0], taper)
r=Robot()
tcp=r.tcp_pose()[0]; print('tcp', tcp)
q0=r.arm_q()
for deg in (25, 50, 75):
    Rn = rot_y(math.radians(deg)) @ R_place
    q = r.ik_tcp([tcp[0], tcp[1], 1.00], Rn, seed=q0)
    print(deg, None if q is None else np.round(q,3), None if q is None else 'delta %.2f'%np.max(np.abs(np.array(q)-q0)))
r.close()
" 2>&1 | grep -v "^\["

# openrua op 60
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R_place = hand_R(0.0, [1,0,0], taper)
r=Robot()
q50=[0.064,1.223,-0.374,-0.798,0.321,1.093,0.615]
for deg in (65, 75):
  for pos in ([-0.15,-0.05,1.0],[-0.15,-0.05,1.1],[-0.25,-0.05,1.05],[-0.35,-0.05,1.05],[-0.15,-0.2,1.05]):
    Rn = rot_y(math.radians(deg)) @ R_place
    q = r.ik_tcp(pos, Rn, seed=q50, attempts=5)
    print(deg, pos, None if q is None else np.round(q,3))
r.close()
" 2>&1 | grep -v "^\["

# openrua op 61
python3 -u tools/perception/px2world.py agentview 415 290 2>&1 | tail -1; python3 -u tools/perception/px2world.py agentview 430 265 2>&1 | tail -1; python3 -u tools/perception/px2world.py agentview 395 330 2>&1 | tail -1

# openrua op 62
mkdir -p "$(dirname /workspace/right_cup.py)"
cat > /workspace/right_cup.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""The cup is held by its rim and has swung ~80 deg about the finger axis
(base toward +x). The pinch acts as a friction hinge stiff enough to hold
the cup's weight, so rotate the whole hand about world Y to bring the cup
back upright, then set it down on the open table."""
import math
import sys
import numpy as np
from rob import *

taper = math.atan2(0.0125, 0.105)
R_place = hand_R(0.0, [1, 0, 0], taper)
P = [-0.30, -0.05, 1.08]
r = Robot()
step = sys.argv[1]
if step == "move":
    print(r.move_tcp([-0.15, -0.05, 1.08], R_place, 3.0))
    print(r.move_tcp(P, R_place, 4.0))
    print("gap %.4f" % r.finger_gap(), "F", np.round(r.wrench()[0], 2))
elif step == "rotate":
    q0 = r.arm_q()
    for deg in (25, 50, 65, 75):
        Rn = rot_y(math.radians(deg)) @ R_place
        ok, info = r.move_tcp(P, Rn, 4.0, seed=q0)
        q0 = r.arm_q()
        print(deg, info, "gap %.4f" % r.finger_gap(), "F", np.round(r.wrench()[0], 2))
        if not ok:
            break
    print("hand z", np.round(quat_to_R(r.hand_pose()[1])[:, 2], 3))
elif step == "down":
    deg = float(sys.argv[2])
    Rn = rot_y(math.radians(deg)) @ R_place
    for z in (1.0, 0.975, 0.96):
        ok, info = r.move_tcp([P[0], P[1], z], Rn, 2.5)
        F = r.wrench()[0]
        print(z, info, "F", np.round(F, 2))
        if abs(F[2] + 5.3) > 2.0:
            break
r.close()
OPENRUA_EOF

# openrua op 63
python3 -u right_cup.py move 2>&1 | grep -v "^\[" && python3 -u right_cup.py rotate 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py sideview && python3 -u tools/perception/cam_snap.py agentview

# openrua op 64
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R_place = hand_R(0.0, [1,0,0], taper)
r=Robot()
Rn = rot_y(math.radians(25)) @ R_place
print(r.move_tcp([-0.12,-0.05,1.12], Rn, 4.0))
print('gap %.4f'%r.finger_gap(), 'F', np.round(r.wrench()[0],2), 'tcp', r.tcp_pose()[0])
r.close()
" 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py sideview && python3 -u tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('side_crop.png', cv2.resize(im[100:340,180:460], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[0:300,100:400], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))"

# openrua op 65
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R_place = hand_R(0.0, [1,0,0], taper)
r=Robot()
P=[-0.12,-0.05,1.12]
for deg in (10, -5, -15):
    Rn = rot_y(math.radians(deg)) @ R_place
    ok,info=r.move_tcp(P, Rn, 3.0)
    print(deg, info, 'gap %.4f'%r.finger_gap(), 'F', np.round(r.wrench()[0],2))
r.close()
" 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py sideview && python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('side_crop.png', cv2.resize(im[100:340,180:460], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))"

# openrua op 66
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R_place = hand_R(0.0, [1,0,0], taper)
r=Robot()
P=[-0.12,-0.05,1.12]
for deg in (0, 10):
    Rn = rot_y(math.radians(deg)) @ R_place
    ok,info=r.move_tcp(P, Rn, 3.0)
    print(deg, info, 'gap %.4f'%r.finger_gap(), 'F', np.round(r.wrench()[0],2))
r.close()
" 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py sideview && python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('side_crop.png', cv2.resize(im[100:340,180:460], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))"

# openrua op 67
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R_place = hand_R(0.0, [1,0,0], taper)
r=Robot()
P=[-0.12,-0.05,1.12]
for deg in (-10, -25, -40):
    Rn = rot_y(math.radians(deg)) @ R_place
    ok,info=r.move_tcp(P, Rn, 3.0)
    print(deg, info, 'gap %.4f'%r.finger_gap(), 'F', np.round(r.wrench()[0],2))
r.close()
" 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py sideview && python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('side_crop.png', cv2.resize(im[100:340,180:460], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))"

# openrua op 68
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R_place = hand_R(0.0, [1,0,0], taper)
Rn = rot_y(math.radians(-40)) @ R_place
r=Robot()
for z in (1.05, 1.0, 0.98, 0.965, 0.955):
    ok,info=r.move_tcp([-0.12,-0.05,z], Rn, 2.5)
    F=r.wrench()[0]; print(z, info, 'F', np.round(F,2))
    if abs(F[2]+4.1)>2.0: break
print(r.gripper(0.08)); print(r.gripper(0.08)); print('gap %.4f'%r.finger_gap())
print(r.move_tcp([-0.12,-0.05,1.15], Rn, 3.0))
print(r.move_tcp([-0.25,-0.20,1.30], hand_R(0.0), 4.0))
r.close()
" 2>&1 | grep -v "^\[" && python3 -u cloud.py birdview -0.25 0.00 -0.15 0.10 0.01 2>&1 | tail -27

# openrua op 69
python3 -u -c "
from rob import *
r=Robot()
for i in range(3):
    ok,info=r.move_tcp([-0.25,-0.20,1.30], hand_R(0.0), 3.0); print(info)
    if ok: break
print('q', np.round(r.arm_q(),3))
r.close()
" 2>&1 | grep -v "^\[" && python3 -c "
import subprocess, numpy as np
subprocess.run(['python3','cloud.py','birdview'],check=True,capture_output=True)
W=np.load('birdview_cloud.npy')
sel=(W[:,0]>-0.22)&(W[:,0]<-0.09)&(W[:,1]>-0.08)&(W[:,1]<0.08)&(W[:,2]>0.975)
P=W[sel]; x,y=P[:,0],P[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rr=np.sqrt(c[2]+cx*cx+cy*cy)
print(len(P),'rim center %.4f %.4f r %.4f zmax %.4f'%(cx,cy,rr,P[:,2].max()))
h=(W[:,0]>-0.25)&(W[:,0]<-0.20)&(W[:,1]>-0.06)&(W[:,1]<0.04)&(W[:,2]>0.90)
H=W[h]; print('handle x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(H[:,0].min(),H[:,0].max(),H[:,1].min(),H[:,1].max(),H[:,2].min(),H[:,2].max()))
"

# openrua op 70
sed -i 's/^cx, cy = .*/cx, cy = float(sys.argv[1]), float(sys.argv[2])   # rim circle centre (birdview fit)/; s/^import math$/import math, sys/' pick2.py && python3 -u pick2.py -0.1456 -0.0061 2>&1 | grep -v "^\["

# openrua op 71
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R = hand_R(math.pi/2, [0,1,0], taper)
r=Robot()
print(r.move_tcp([-0.1009,-0.0061,1.05], R, 3.0)); print('gap %.4f'%r.finger_gap())
print(r.move_tcp([-0.1009,-0.0061,1.22], R, 3.0)); print('gap %.4f'%r.finger_gap())
print(r.move_tcp([-0.340,-0.275,1.22], R, 6.0)); print('gap %.4f'%r.finger_gap(), 'F', np.round(r.wrench()[0],2))
r.close()
" 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py agentview && python3 -u tools/perception/cam_snap.py sideview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[0:280,0:320], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); cv2.imwrite('side_crop.png', cv2.resize(im[60:340,300:560], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))"

# openrua op 72
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R = hand_R(math.pi/2, [0,1,0], taper)
r=Robot()
base=np.array([0.64,0.01,-5.37])
for z in (1.15, 1.10, 1.06, 1.03, 1.00):
    ok,info=r.move_tcp([-0.340,-0.275,z], R, 2.5)
    F=r.wrench()[0]; d=np.abs(F-base).max()
    print(z, info, 'F', np.round(F,2), 'dev %.2f'%d, 'gap %.4f'%r.finger_gap())
    if d>1.5 or not ok: print('STOP'); break
r.close()
" 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[60:280,40:320], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))"

# openrua op 73
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R = hand_R(math.pi/2, [0,1,0], taper)
r=Robot()
print(r.move_tcp([-0.340,-0.275,1.22], R, 4.0)); print('gap %.4f'%r.finger_gap(), 'F', np.round(r.wrench()[0],2))
print(r.move_tcp([-0.105,-0.275,1.22], R, 6.0)); print('gap %.4f'%r.finger_gap())
base=np.array([0.64,0.01,-5.37])
for z in (1.05, 1.00, 0.98, 0.965):
    ok,info=r.move_tcp([-0.105,-0.275,z], R, 2.5)
    F=r.wrench()[0]; d=np.abs(F-base).max()
    print(z, info, 'F', np.round(F,2), 'dev %.2f'%d)
    if d>1.5 or not ok: break
print(r.gripper(0.08)); print(r.gripper(0.08)); print('gap %.4f'%r.finger_gap())
print(r.move_tcp([-0.105,-0.275,1.15], R, 3.0))
print(r.move_tcp([-0.25,-0.45,1.30], hand_R(0.0), 4.0))
r.close()
" 2>&1 | grep -v "^\[" && python3 -c "
import subprocess, numpy as np
subprocess.run(['python3','cloud.py','birdview'],check=True,capture_output=True)
W=np.load('birdview_cloud.npy')
sel=(W[:,0]>-0.25)&(W[:,0]<-0.02)&(W[:,1]>-0.36)&(W[:,1]<-0.18)&(W[:,2]>0.975)
P=W[sel]; x,y=P[:,0],P[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rr=np.sqrt(c[2]+cx*cx+cy*cy)
print(len(P),'rim center %.4f %.4f r %.4f zmax %.4f zmin %.4f'%(cx,cy,rr,P[:,2].max(),P[:,2].min()))
"

# openrua op 74
python3 -u -c "
from rob import *
r=Robot()
for i in range(3):
    ok,info=r.move_tcp([-0.20,-0.55,1.30], hand_R(0.0), 3.0); print(info)
    if ok: break
r.close()
" 2>&1 | grep -v "^\[" && python3 -c "
import subprocess, numpy as np
subprocess.run(['python3','cloud.py','birdview'],check=True,capture_output=True)
W=np.load('birdview_cloud.npy')
sel=(W[:,0]>-0.25)&(W[:,0]<-0.02)&(W[:,1]>-0.36)&(W[:,1]<-0.18)&(W[:,2]>0.975)&(W[:,2]<1.05)
P=W[sel]; x,y=P[:,0],P[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rr=np.sqrt(c[2]+cx*cx+cy*cy)
print(len(P),'rim center %.4f %.4f r %.4f zmax %.4f zmin %.4f'%(cx,cy,rr,P[:,2].max(),P[:,2].min()))
h=(W[:,0]>-0.30)&(W[:,0]<0.0)&(W[:,1]>-0.40)&(W[:,1]<-0.15)&(W[:,2]>0.90)&(W[:,2]<0.98)
H=W[h]; 
# points outside rim circle -> handle
d=np.hypot(H[:,0]-cx,H[:,1]-cy); Hh=H[d>0.052]
print('handle pts',len(Hh), 'x %.3f..%.3f y %.3f..%.3f'%(Hh[:,0].min(),Hh[:,0].max(),Hh[:,1].min(),Hh[:,1].max()) if len(Hh) else '')
" && python3 -u tools/perception/cam_snap.py agentview

# openrua op 75
python3 -u pick2.py -0.1528 -0.2726 2>&1 | grep -v "^\[" && python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R = hand_R(math.pi/2, [0,1,0], taper)
wx=-0.1528+0.050-0.025*math.tan(taper)-0.0023
r=Robot()
print('lift', r.move_tcp([wx,-0.2726,1.05], R, 3.0), 'gap %.4f'%r.finger_gap())
print('lift', r.move_tcp([wx,-0.2726,1.22], R, 3.0), 'gap %.4f'%r.finger_gap())
print('carry', r.move_tcp([-0.3403,-0.2726,1.22], R, 6.0), 'gap %.4f'%r.finger_gap(), 'F', np.round(r.wrench()[0],2))
r.close()
" 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[0:280,0:320], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))"

# openrua op 76
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R = hand_R(math.pi/2, [0,1,0], taper)
r=Robot()
base=np.array([0.65,0.0,-5.37]); stopped=False
for z in (1.15, 1.10, 1.06, 1.03, 1.00, 0.985):
    ok,info=r.move_tcp([-0.3403,-0.2726,z], R, 2.5)
    F=r.wrench()[0]; d=np.abs(F-base).max()
    print(z, info, 'F', np.round(F,2), 'dev %.2f'%d, 'gap %.4f'%r.finger_gap())
    if d>1.5 or not ok: print('STOP'); stopped=True; break
print('tcp', r.tcp_pose()[0])
r.close()
" 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[60:280,40:320], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))"

# openrua op 77
python3 -u cloud.py agentview -0.50 -0.30 -0.40 -0.15 0.01 2>&1 | tail -23; python3 -c "
import numpy as np
W=np.load('agentview_cloud.npy')
# cup: points above z 1.0 near x -0.45..-0.30, y -0.35..-0.20 but below hand (z<1.10)
s=(W[:,0]>-0.46)&(W[:,0]<-0.30)&(W[:,1]>-0.36)&(W[:,1]<-0.18)&(W[:,2]>1.005)&(W[:,2]<1.09)
C=W[s]; print('cup-ish pts',len(C))
for zlo,zhi in ((1.005,1.03),(1.03,1.06),(1.06,1.09)):
    m=(C[:,2]>=zlo)&(C[:,2]<zhi); 
    if m.sum(): print(zlo,zhi,'x %.3f..%.3f y %.3f..%.3f n %d'%(C[m,0].min(),C[m,0].max(),C[m,1].min(),C[m,1].max(),m.sum()))
"

# openrua op 78
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R = hand_R(math.pi/2, [0,1,0], taper)
r=Robot()
print(r.move_tcp([-0.3403,-0.2726,1.22], R, 3.0), 'gap %.4f'%r.finger_gap())
print(r.move_tcp([-0.15,-0.2726,1.22], R, 5.0), 'gap %.4f'%r.finger_gap())
r.close()
" 2>&1 | grep -v "^\[" && python3 -u cloud.py birdview -0.50 -0.32 -0.10 0.10 0.01 2>&1 | tail -20

# openrua op 79
python3 -u cloud.py sideview > /dev/null 2>&1; python3 -c "
import numpy as np
W=np.load('sideview_cloud.npy')
cx,cy=-0.15-0.0447,-0.2726
# cup points: z in 1.12..1.26, near center
s=(W[:,2]>1.10)&(W[:,2]<1.26)&(np.abs(W[:,1]-cy)<0.08)&(W[:,0]>cx-0.15)&(W[:,0]<cx+0.08)
C=W[s]; print('pts',len(C), 'x %.3f..%.3f z %.3f..%.3f'%(C[:,0].min(),C[:,0].max(),C[:,2].min(),C[:,2].max()))
# per z-slice, min x (handle extent)
for z in np.arange(1.12,1.26,0.01):
    m=(C[:,2]>=z)&(C[:,2]<z+0.01)
    if m.sum(): print('z %.2f  xmin %.3f  xmax %.3f  n %d'%(z, C[m,0].min(), C[m,0].max(), m.sum()))
"

# openrua op 80
cat >> rob.py <<'EOF'


def _minjerk(s):
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def move_tcp_smooth(r, p0, p1, R, seconds, n=12):
    """Straight-line TCP move with a minimum-jerk time profile sent as one
    multi-waypoint trajectory (bounded acceleration -> no swing of a hinged
    object in the grasp). Returns (ok, info)."""
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    q = r.arm_q(); wps = []
    for i in range(1, n + 1):
        s = i / n
        p = p0 + _minjerk(s) * (p1 - p0)
        q = r.ik_tcp(p, R, seed=q)
        if q is None:
            return False, f"IK failed at waypoint {i}"
        wps.append((q, seconds * s))
    code = r.move_joints(wps[-1][0], seconds, waypoints=wps)
    now = np.array(r.arm_q()); err = np.abs(now - np.array(wps[-1][0])).max()
    if err > 0.01:
        code = r.move_joints(wps[-1][0], 1.5)
        now = np.array(r.arm_q()); err = np.abs(now - np.array(wps[-1][0])).max()
    cart = np.linalg.norm(r.tcp_pose()[0] - p1)
    return code == 0 and err < 0.02, f"code={code} max_joint_err={err:.4f} cart_err={cart*1000:.1f}mm"
EOF
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R = hand_R(math.pi/2, [0,1,0], taper)
r=Robot()
p0=r.tcp_pose()[0]; print('tcp', p0, 'gap %.4f'%r.finger_gap())
print(move_tcp_smooth(r, p0, [-0.155,-0.14,1.22], R, 8.0))
print('gap %.4f'%r.finger_gap(), 'F', np.round(r.wrench()[0],2))
r.close()
" 2>&1 | grep -v "^\[" && python3 -u tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[0:300,100:400], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))"

# openrua op 81
python3 -u -c "
from rob import *
import math
taper = math.atan2(0.0125, 0.105); R = hand_R(math.pi/2, [0,1,0], taper)
r=Robot()
base=np.array([0.65,0.0,-5.37])
for z in (1.10, 1.02, 0.985, 0.97):
    ok,info=r.move_tcp([-0.155,-0.14,z], R, 3.0)
    F=r.wrench()[0]; d=np.abs(F-base).max(); print(z, info, 'F', np.round(F,2), 'dev %.2f'%d)
    if d>1.5 or not ok: break
print(r.gripper(0.08)); print(r.gripper(0.08)); print('gap %.4f'%r.finger_gap())
print(r.move_tcp([-0.155,-0.14,1.15], R, 3.0))
print(r.move_tcp([-0.30,-0.35,1.30], hand_R(0.0), 4.0))
r.close()
" 2>&1 | grep -v "^\[" && python3 -c "
import subprocess, numpy as np
subprocess.run(['python3','cloud.py','birdview'],check=True,capture_output=True)
W=np.load('birdview_cloud.npy')
sel=(W[:,0]>-0.28)&(W[:,0]<-0.10)&(W[:,1]>-0.25)&(W[:,1]<-0.03)&(W[:,2]>0.975)&(W[:,2]<1.05)
P=W[sel]; x,y=P[:,0],P[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; rr=np.sqrt(c[2]+cx*cx+cy*cy)
print(len(P),'rim center %.4f %.4f r %.4f zmax %.4f zmin %.4f'%(cx,cy,rr,P[:,2].max(),P[:,2].min()))
" && python3 -u tools/perception/cam_snap.py agentview

# openrua op 82
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-quirks
description: Hard-won facts about the /workspace Panda + paused-clock sim (finger axis, gripper/FJT early returns, thin-wall pinch behaves like a hinge, caddy is too small for the mug)
metadata:
  type: project
---

Facts learned driving the Franka Panda in the /workspace robot_bridge sim (Sept 2026):

- Panda fingers travel along the **hand Y axis** (not X). With `hand_R(yaw=0)` (R_DOWN = rot_x(pi)) the
  fingers straddle along world Y; use yaw = pi/2 for fingers along world X. Getting this wrong made
  the gripper pinch the cup's side bulge instead of its wall.
- Gripper action and FollowJointTrajectory often return before the motion is finished (paused clock):
  re-issue `gripper()` until the finger position stops changing; FJT code -5 with large joint error
  just needs the same goal re-sent (esp. big joint-7 moves).
- A pinch on a thin cup wall is a friction **hinge** about the finger axis: accelerations perpendicular to
  the fingers or any wall contact swing the cup and it stays tilted. Carry along the finger axis, use
  `move_tcp_smooth` (min-jerk waypoints in rob.py), and re-seat the cup on the table if it tilts.
- Scene: caddy interior compartments are ~11x11 cm (left), 11x12 cm (right, 16 cm deep walls), middle
  column split by a low divider into 5 cm and 4 cm slots. The mug (base 7.5 cm, rim 10 cm, handle
  reaching 8.3 cm from centre, handle bottom only 2 cm above base) fits upright in none of them.

**Why:** these cost several failed grasps/placements; none is documented in docs/ or machine.yaml.
**How to apply:** check finger axis and re-issue gripper commands first; don't retry the mug-in-caddy
placement without a different object or caddy.
OPENRUA_EOF

# openrua op 83
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index)") && grep -q panda-sim-quirks MEMORY.md 2>/dev/null || echo "- [Panda sim quirks](panda-sim-quirks.md) — finger axis is hand Y, re-issue gripper/FJT goals, thin-wall pinch is a hinge, mug doesn't fit the caddy" >> MEMORY.md; tail -3 MEMORY.md
