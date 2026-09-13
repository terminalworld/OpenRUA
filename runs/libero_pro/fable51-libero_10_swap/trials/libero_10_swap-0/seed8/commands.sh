#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la tools .ros

# openrua op 2
ls -la tools/action tools/perception; cat tools/action/*.py tools/perception/*.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | grep -v -E 'parameter|describe|list_|get_type' ; echo ---; timeout 30 ros2 node list

# openrua op 4
timeout 60 ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[170:420,90:380]
cv2.imwrite('crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 60 ros2 topic echo /tf_static --once 2>&1 | grep -E 'frame_id' | sort | uniq -c; echo ---; timeout 20 ros2 topic echo /tf --once 2>&1 | grep -E 'frame_id' | sort | uniq -c; echo ---; timeout 60 ros2 topic echo /agentview/color/camera_info --once

# openrua op 7
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+info from a camera, print world coords for pixels.

Usage: python3 scene.py <camera> u,v [u,v ...]
Also prints the world->panda_link0 transform and saves <camera>_cloud.npy
(HxWx3 world xyz) for offline analysis.
"""
import struct
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def T_of(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    return T


def main():
    cam = sys.argv[1]
    pix = [tuple(int(x) for x in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float64)
    import time
    end = time.time() + 15
    frame = f"{cam}_optical_frame"
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()) and \
           tfbuf.can_transform("world", "panda_link0", rclpy.time.Time()):
            break
    Tc = T_of(tfbuf.lookup_transform("world", frame, rclpy.time.Time()))
    Tb = T_of(tfbuf.lookup_transform("world", "panda_link0", rclpy.time.Time()))
    print("world->panda_link0 translation:", np.round(Tb[:3, 3], 4))
    print("world->panda_link0 rotation:\n", np.round(Tb[:3, :3], 4))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    P = np.stack([X, Y, depth, np.ones_like(depth)], -1)
    Pw = P @ Tc.T
    cloud = Pw[..., :3]
    np.save(f"{cam}_cloud.npy", cloud)
    np.save(f"{cam}_depth.npy", depth)
    for (u, v) in pix:
        print(f"px ({u},{v}) depth={depth[v, u]:.4f} world=", np.round(cloud[v, u], 4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 scene.py agentview 133,300 240,205 530,265 340,420 130,330 240,240

# openrua op 9
timeout 120 python3 scene.py birdview 320,240 >/dev/null && python3 -c "
import numpy as np, cv2
for cam in ['agentview','birdview']:
    c=np.load(f'{cam}_cloud.npy'); im=cv2.imread(f'{cam}.png')
    mask=(c[...,2]>0.445)&(c[...,2]<0.75)&np.isfinite(c[...,2])&(c[...,0]>-0.45)&(c[...,0]<0.4)&(np.abs(c[...,1])<0.5)
    m8=mask.astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(m8)
    print(cam)
    for i in range(1,n):
        if stats[i,4]<40: continue
        pts=c[lab==i]
        print(f'  comp {i}: px={stats[i,4]} centroid_px={cent[i].round(0)} world_xy_mean={pts[:,:2].mean(0).round(3)} zmin={pts[:,2].min():.3f} zmax={pts[:,2].max():.3f} xrange=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yrange=({pts[:,1].min():.3f},{pts[:,1].max():.3f})')
"


# openrua op 10
python3 -c "
import numpy as np, cv2
c=np.load('birdview_cloud.npy')
for name,(x0,x1,y0,y1,zt) in {'alphabet_soup':(-0.32,-0.2,-0.2,-0.08,0.48),'tomato_sauce':(-0.03,0.14,-0.29,-0.17,0.49),'basket':(-0.12,0.12,0.14,0.36,0.58)}.items():
    m=(c[...,0]>x0)&(c[...,0]<x1)&(c[...,1]>y0)&(c[...,1]<y1)&(c[...,2]>zt)&(c[...,2]<0.7)
    p=c[m]
    print(name, 'n=',len(p),'center=',p[:,:2].mean(0).round(4),'min=',p.min(0).round(3),'max=',p.max(0).round(3))
# table height
m=(np.abs(c[...,0]-0.2)<0.05)&(np.abs(c[...,1])<0.05)
print('table z', np.median(c[m][:,2]).round(4))
"


# openrua op 11
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Arm helper for this Panda: FK, IK-goto, gripper, joint read.

Usage:
  python3 arm.py fk                          # print hand + tcp pose (base frame)
  python3 arm.py js                          # print joint state
  python3 arm.py goto x y z [qx qy qz qw] [--tcp] [--sec S]   # base frame
  python3 arm.py grip open|close
  python3 arm.py lin x y z [--tcp] [--sec S] [--steps N]   # straight-line, keeps orientation
All poses in the planning frame (panda_link0). World = base + (-0.51, 0, 0.42).
"""
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
# default "hand pointing down" orientation, set from FK of the home pose
DOWN_Q = None


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.update(zip(m.name, m.position)), 1)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.wait_js()

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        end = time.time() + 20
        while time.time() < end and not all(j in self.js for j in JOINTS):
            self.spin(0.2)
        if not all(j in self.js for j in JOINTS):
            raise SystemExit("no /joint_states")

    def arm_q(self):
        self.js.clear()
        self.wait_js()
        return [self.js[j] for j in JOINTS]

    def fingers(self):
        self.js.clear()
        self.wait_js()
        return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")

    def seed_state(self, q=None):
        s = JointState()
        s.name = list(JOINTS)
        s.position = list(q if q is not None else self.arm_q())
        return s

    def fk(self, q=None):
        self.fk_cli.wait_for_service(10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.seed_state(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def ik(self, pos, q, seed=None, at_tcp=False):
        pos = np.array(pos, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(q)[:, 2]
        self.ik_cli.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = self.seed_state(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise SystemExit("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def traj(self, points, seconds):
        """points: list of joint vectors; seconds: total duration."""
        if not self.fjt.wait_for_server(10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(points)
        for i, p in enumerate(points):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(x) for x in p])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q = np.array(self.arm_q())
        err = np.abs(q - np.array(points[-1])).max()
        print(f"traj done error_code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, pos, q, at_tcp=False, seconds=4.0, steps=1):
        """IK to pose; if steps>1 interpolate straight line from current pose."""
        cur_pos, cur_q = self.fk()
        if at_tcp:
            cur_pos = cur_pos + TCP * quat_to_R(cur_q)[:, 2]
        seed = self.arm_q()
        pts = []
        for i in range(1, steps + 1):
            a = i / steps
            p = (1 - a) * cur_pos + a * np.array(pos, float)
            sol = self.ik(p, q, seed=seed, at_tcp=at_tcp)
            if sol is None:
                raise SystemExit(f"IK FAILED at step {i}/{steps} pos={np.round(p,4)}")
            pts.append(sol)
            seed = sol
        code, err = self.traj(pts, seconds)
        fpos, fq = self.fk()
        tcp = fpos + TCP * quat_to_R(fq)[:, 2]
        print(f"hand={np.round(fpos,4)} tcp={np.round(tcp,4)} q={np.round(fq,4)}")
        return code, err

    def gripper(self, width):
        if not self.grip.wait_for_server(10):
            raise SystemExit("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        for _ in range(5):
            self.spin(0.1)
        f = self.fingers()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f


def main():
    a = Arm()
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    sec = float(sys.argv[sys.argv.index("--sec") + 1]) if "--sec" in sys.argv else 4.0
    steps = int(sys.argv[sys.argv.index("--steps") + 1]) if "--steps" in sys.argv else 1
    at_tcp = "--tcp" in sys.argv
    cmd = args[0]
    if cmd == "fk":
        pos, q = a.fk()
        tcp = pos + TCP * quat_to_R(q)[:, 2]
        print("hand", np.round(pos, 4), "tcp", np.round(tcp, 4), "q(xyzw)", np.round(q, 4))
        print("R\n", np.round(quat_to_R(q), 3))
    elif cmd == "js":
        print("arm", np.round(a.arm_q(), 4), "fingers", a.fingers())
    elif cmd == "goto":
        pos = [float(x) for x in args[1:4]]
        if len(args) >= 8:
            q = [float(x) for x in args[4:8]]
        else:
            _, q = a.fk()
        a.goto(pos, q, at_tcp=at_tcp, seconds=sec, steps=steps)
    elif cmd == "lin":
        pos = [float(x) for x in args[1:4]]
        _, q = a.fk()
        a.goto(pos, q, at_tcp=at_tcp, seconds=sec, steps=max(steps, 4))
    elif cmd == "grip":
        a.gripper(GRIP["open_m"] if args[1] == "open" else GRIP["closed_m"])
    elif cmd == "joints":
        q = [float(x) for x in args[1].split(",")]
        a.traj([q], sec)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 12
timeout 120 python3 arm.py fk

# openrua op 13
python3 -c "
import numpy as np
c=np.load('birdview_cloud.npy')
def box(name,x0,x1,y0,y1,zt=0.435):
    m=(c[...,0]>x0)&(c[...,0]<x1)&(c[...,1]>y0)&(c[...,1]<y1)&(c[...,2]>zt)&(c[...,2]<0.7)
    p=c[m]; b=p.min(0).round(3); t=p.max(0).round(3)
    print(f'{name:14s} n={len(p):4d} world min={b} max={t}   base-frame center=({(b[0]+t[0])/2+0.51:.3f},{(b[1]+t[1])/2:.3f}) top={t[2]-0.42:.3f}')
box('soup',-0.32,-0.2,-0.2,-0.08)
box('ketchup',-0.19,-0.06,-0.2,-0.08)
box('milk',-0.01,0.1,-0.17,-0.08)
box('tomato',-0.03,0.07,-0.29,-0.19,0.46)
box('creamcheese',0.0,0.16,-0.30,-0.15,0.435)
box('OJ',-0.16,-0.03,-0.02,0.08)
box('butter',0.0,0.12,0.0,0.08)
"


# openrua op 14
python3 -c "
import numpy as np
c=np.load('birdview_cloud.npy')
def box(name,x0,x1,y0,y1,z0=0.435,z1=0.7):
    m=(c[...,0]>x0)&(c[...,0]<x1)&(c[...,1]>y0)&(c[...,1]<y1)&(c[...,2]>z0)&(c[...,2]<z1)
    p=c[m]
    if not len(p): print(name,'empty'); return
    b=p.min(0).round(3); t=p.max(0).round(3)
    print(f'{name:14s} n={len(p):4d} world min={b} max={t}   base-frame center=({(b[0]+t[0])/2+0.51:.3f},{(b[1]+t[1])/2:.3f}) top={t[2]-0.42:.3f}')
box('creamcheese',0.0,0.16,-0.30,-0.15,0.435,0.462)
box('OJ',-0.16,-0.03,-0.02,0.08)
box('butter',0.0,0.12,0.0,0.08,0.435)
"


# openrua op 15
python3 -c "
import cv2
im=cv2.imread('birdview.png')
crop=im[200:340,230:420]
cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 16
timeout 300 python3 arm.py grip open && timeout 600 python3 arm.py goto 0.252 -0.134 0.20 1 0 0 0 --tcp --sec 4

# openrua op 17
cat > ikprobe.py <<'EOF'
import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm, quat_to_R
from scipy.spatial.transform import Rotation as Rot
a=Arm()
tx,ty,tz=0.252,-0.134,0.20
for tilt in [0,10,20]:
    for yaw in [0,45,90,135,180,-45,-90,-135]:
        # hand down, tilted toward -x by tilt degrees, yawed
        R=Rot.from_euler('z',yaw,degrees=True)*Rot.from_euler('y',tilt,degrees=True)*Rot.from_quat([1,0,0,0])
        q=R.as_quat()
        sol=a.ik([tx,ty,tz],q,at_tcp=True)
        print(f"tilt={tilt} yaw={yaw} -> {'OK '+str(np.round(sol,3)) if sol else 'fail'}", flush=True)
rclpy.shutdown()
EOF
timeout 600 python3 ikprobe.py

# openrua op 18
cat > ikprobe2.py <<'EOF'
import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm
a=Arm()
pos,q=a.fk()
print("current", np.round(pos,3), np.round(q,3))
print("ik current ->", a.ik(pos,q))
for p in [(0.4,0,0.4),(0.5,0,0.3),(0.3,0,0.5),(0.3,-0.13,0.4),(0.252,-0.134,0.4)]:
    print(p, "->", a.ik(p,[1,0,0,0]))
rclpy.shutdown()
EOF
timeout 600 python3 ikprobe2.py

# openrua op 19
cat > ikprobe3.py <<'EOF'
import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from builtin_interfaces.msg import Duration
import arm
from arm import Arm
a=Arm()
ready=[0,-0.785,0,-2.356,0,1.571,0.785]
pos,q=a.fk(ready); print("fk(ready)", np.round(pos,3), np.round(q,3))
# patch longer timeout
orig=a.ik
def ik2(p,q,seed=None,at_tcp=False,t=5):
    return orig(p,q,seed=seed,at_tcp=at_tcp)
for p in [(0.4,0,0.4),(0.5,0,0.3),(0.3,-0.13,0.4),(0.252,-0.134,0.30)]:
    print(p, "seed ready ->", a.ik(p,[1,0,0,0],seed=ready))
    print(p, "seed ready q=fk(ready) ->", a.ik(p,q,seed=ready))
rclpy.shutdown()
EOF
timeout 600 python3 ikprobe3.py

# openrua op 20
cat > ikprobe4.py <<'EOF'
import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm
from scipy.spatial.transform import Rotation as Rot
a=Arm()
for p in [(0.0,0,0.7),(-0.258,-0.135,0.62),(-0.258,-0.135,0.57),(0.017,-0.236,0.60),(0.0,0.255,0.80)]:
    print(p, "->", None if a.ik(p,[1,0,0,0]) is None else np.round(a.ik(p,[1,0,0,0]),3))
# soup pregrasp with tcp
print("soup tcp 0.62 yaw variants")
for yaw in [0,45,90,-45,-90]:
    q=(Rot.from_euler('z',yaw,degrees=True)*Rot.from_quat([1,0,0,0])).as_quat()
    s=a.ik((-0.258,-0.135,0.62),q,at_tcp=True)
    print(yaw, None if s is None else np.round(s,3))
rclpy.shutdown()
EOF
timeout 600 python3 ikprobe4.py

# openrua op 21
cat > ikprobe5.py <<'EOF'
import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm, JOINTS
from scipy.spatial.transform import Rotation as Rot
a=Arm()
lim=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(s): s=np.array(s); return np.min(np.minimum(s-lim[:,0], lim[:,1]-s))
for z in [0.465,0.60]:
  for tilt in [0,15,30,45]:
    for yaw in [0,90,-90]:
        q=(Rot.from_euler('z',yaw,degrees=True)*Rot.from_euler('y',tilt,degrees=True)*Rot.from_quat([1,0,0,0])).as_quat()
        s=a.ik((-0.258,-0.135,z),q,at_tcp=True)
        print(f"z={z} tilt={tilt} yaw={yaw}: ", "fail" if s is None else f"{np.round(s,2)} limit-margin={margin(s):.2f}", flush=True)
rclpy.shutdown()
EOF
timeout 900 python3 ikprobe5.py

# openrua op 22
cat > ikprobe6.py <<'EOF'
import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm
from scipy.spatial.transform import Rotation as Rot
a=Arm()
lim=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(s): s=np.array(s); return np.min(np.minimum(s-lim[:,0], lim[:,1]-s))
seed=[-0.1,0.2,-0.28,-2.73,-0.21,2.42,-0.2]
for tilt in [20,25,30]:
    q=(Rot.from_euler('y',tilt,degrees=True)*Rot.from_quat([1,0,0,0])).as_quat()
    print("tilt",tilt,"q",np.round(q,4))
    for z in [0.60,0.53,0.47]:
        s=a.ik((-0.258,-0.135,z),q,seed=seed,at_tcp=True)
        print(f"  z={z}: ", "fail" if s is None else f"{np.round(s,3)} margin={margin(s):.2f}", flush=True)
rclpy.shutdown()
EOF
timeout 900 python3 ikprobe6.py

# openrua op 23
mkdir -p "$(dirname /workspace/pick.py)"
cat > /workspace/pick.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick an object with a tilted top-down grasp and lift it.

Usage: python3 pick.py x y z_grasp z_pre tilt_deg yaw_deg seed_csv
World frame. Seed = joint vector to seed IK with (keeps the branch).
"""
import sys
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation as Rot

argv = sys.argv[1:]
sys.argv = [sys.argv[0]]
from arm import Arm, GRIP, quat_to_R, TCP

x, y, zg, zp, tilt, yaw = map(float, argv[:6])
seed = [float(v) for v in argv[6].split(",")]
q = (Rot.from_euler('z', yaw, degrees=True) * Rot.from_euler('y', tilt, degrees=True)
     * Rot.from_quat([1, 0, 0, 0])).as_quat()
a = Arm()
print("fingers before:", a.fingers())

# 1. pre-grasp
s_pre = a.ik((x, y, zp), q, seed=seed, at_tcp=True)
if s_pre is None:
    raise SystemExit("IK pregrasp failed")
print("pregrasp sol", np.round(s_pre, 3))
code, err = a.traj([s_pre], 4.0)
if err > 0.05:
    raise SystemExit("pregrasp not reached")
pos, fq = a.fk()
print("tcp now", np.round(pos + TCP * quat_to_R(fq)[:, 2], 4))

# 2. descend in steps
zs = np.linspace(zp, zg, 4)[1:]
pts, s = [], s_pre
for z in zs:
    s = a.ik((x, y, z), q, seed=s, at_tcp=True)
    if s is None:
        raise SystemExit(f"IK descend failed at z={z}")
    pts.append(s)
code, err = a.traj(pts, 3.0)
pos, fq = a.fk()
tcp = pos + TCP * quat_to_R(fq)[:, 2]
print("tcp at grasp", np.round(tcp, 4), "target", (x, y, zg))

# 3. close
f = a.gripper(GRIP["closed_m"])
gap = f[0] - f[1]
print(f"finger gap after close: {gap:.4f} m")
if gap < 0.01:
    raise SystemExit("GRASP FAILED: closed on air")

# 4. lift straight up to zp
s_lift = a.ik((x, y, zp), q, seed=s, at_tcp=True)
code, err = a.traj([s_lift], 3.0)
f = a.fingers()
print(f"lifted; finger gap now {f[0]-f[1]:.4f}")
pos, fq = a.fk()
print("tcp after lift", np.round(pos + TCP * quat_to_R(fq)[:, 2], 4))
print("joints", ",".join(f"{v:.4f}" for v in a.arm_q()))
rclpy.shutdown()
OPENRUA_EOF

# openrua op 24
timeout 1500 python3 -u pick.py -0.258 -0.135 0.47 0.60 25 0 -0.066,-0.314,-0.348,-2.774,-0.295,2.045,-0.183 2>&1 | tee pick_soup.log

# openrua op 25
timeout 600 python3 arm.py js; timeout 600 python3 arm.py joints -0.066,-0.314,-0.348,-2.774,-0.295,2.045,-0.183 --sec 4; timeout 300 python3 arm.py fk

# openrua op 26
python3 - <<'EOF'
import re
s=open('arm.py').read()
s=s.replace('''        pos = np.array(pos, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(q)[:, 2]
        self.ik_cli.wait_for_service(10)''','''        pos = np.array(pos, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(q)[:, 2]
        # the IK tip link is panda_link8, yawed +45deg (about its z) from
        # panda_hand; convert the requested HAND orientation to link8
        from scipy.spatial.transform import Rotation as _R
        q = (_R.from_quat(q) * _R.from_euler('z', 45, degrees=True)).as_quat()
        self.ik_cli.wait_for_service(10)''')
open('arm.py','w').write(s)
EOF
cat > ikprobe7.py <<'EOF'
import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm, quat_to_R
from scipy.spatial.transform import Rotation as Rot
a=Arm()
q=(Rot.from_euler('y',25,degrees=True)*Rot.from_quat([1,0,0,0])).as_quat()
s=a.ik((-0.258,-0.135,0.60),q,seed=a.arm_q(),at_tcp=True)
print("sol",np.round(s,3))
pos,fq=a.fk(s)
print("requested hand q",np.round(q,4),"\nFK hand q       ",np.round(fq,4))
print("R\n",np.round(quat_to_R(fq),3))
print("tcp", np.round(pos+0.1034*quat_to_R(fq)[:,2],4))
rclpy.shutdown()
EOF
timeout 600 python3 ikprobe7.py

# openrua op 27
timeout 1500 python3 -u pick.py -0.258 -0.135 0.47 0.60 25 0 -0.163,-0.306,-0.246,-2.776,-0.263,2.053,0.575 2>&1 | tee pick_soup.log

# openrua op 28
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        send = self.fjt.send_goal_async(goal)\\n        rclpy.spin_until_future_complete(self.node, send)\\n        res = send.result().get_result_async()\\n        rclpy.spin_until_future_complete(self.node, res)\\n        code = res.result().result.error_code\\n        q = np.array(self.arm_q())\\n        err = np.abs(q - np.array(points[-1])).max()\\n        print(f\\"traj done error_code={code} max_joint_err={err:.4f}\\")\\n        return code, err", "new_string": "        for attempt in range(3):\\n            send = self.fjt.send_goal_async(goal)\\n            rclpy.spin_until_future_complete(self.node, send)\\n            res = send.result().get_result_async()\\n            rclpy.spin_until_future_complete(self.node, res)\\n            code = res.result().result.error_code\\n            q = np.array(self.arm_q())\\n            err = np.abs(q - np.array(points[-1])).max()\\n            print(f\\"traj done error_code={code} max_joint_err={err:.4f}\\")\\n            if err < 0.02:\\n                break\\n            # controller lag on the first pass (machine fact): resend the\\n            # final point only, from where we are\\n            goal.trajectory.points = [goal.trajectory.points[-1]]\\n            t = max(2.0, seconds / 2)\\n            goal.trajectory.points[0].time_from_start = Duration(\\n                sec=int(t), nanosec=int((t % 1) * 1e9))\\n            print(\\"  resending final point\\")\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 29
timeout 1500 python3 -u pick.py -0.258 -0.135 0.47 0.60 25 0 -0.163,-0.306,-0.246,-2.776,-0.263,2.053,0.575 2>&1 | tee pick_soup.log

# openrua op 30
timeout 120 python3 tools/perception/cam_snap.py agentview agent2.png && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png

# openrua op 31
mkdir -p "$(dirname /workspace/place.py)"
cat > /workspace/place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Carry the held object to a world xy over the basket and release.

Usage: python3 place.py x y z_release tilt_deg
"""
import sys
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation as Rot

argv = sys.argv[1:]
sys.argv = [sys.argv[0]]
from arm import Arm, GRIP, quat_to_R, TCP

x, y, zr, tilt = map(float, argv[:4])
q = (Rot.from_euler('y', tilt, degrees=True) * Rot.from_quat([1, 0, 0, 0])).as_quat()
a = Arm()
f = a.fingers()
print("fingers before (gap):", f[0] - f[1])

pos, fq = a.fk()
tcp0 = pos + TCP * quat_to_R(fq)[:, 2]
mid = np.array([(tcp0[0] + x) / 2, (tcp0[1] + y) / 2, 0.80])
seed = a.arm_q()
s_mid = a.ik(mid, q, seed=seed, at_tcp=True)
if s_mid is None:
    raise SystemExit("IK mid failed")
s_end = a.ik((x, y, zr), q, seed=s_mid, at_tcp=True)
if s_end is None:
    raise SystemExit("IK release pose failed")
print("mid", np.round(s_mid, 3), "\nend", np.round(s_end, 3))
a.traj([s_mid], 4.0)
a.traj([s_end], 3.0)
pos, fq = a.fk()
tcp = pos + TCP * quat_to_R(fq)[:, 2]
print("tcp at release", np.round(tcp, 4), "target", (x, y, zr))
print("gap before release", (lambda f: f[0] - f[1])(a.fingers()))
if np.linalg.norm(tcp[:2] - np.array([x, y])) > 0.03:
    raise SystemExit("not over target; NOT releasing")
f = a.gripper(GRIP["open_m"])
print("gap after release", f[0] - f[1])
# retreat up
s_up = a.ik((x, y, 0.85), q, seed=s_end, at_tcp=True)
a.traj([s_up], 3.0)
print("joints", ",".join(f"{v:.4f}" for v in a.arm_q()))
rclpy.shutdown()
OPENRUA_EOF

# openrua op 32
timeout 1500 python3 -u place.py 0.0 0.255 0.72 0 2>&1 | tee place_soup.log

# openrua op 33
timeout 120 python3 tools/perception/cam_snap.py agentview agent3.png && timeout 120 python3 tools/perception/cam_snap.py birdview bird3.png

# openrua op 34
cat > ikprobe8.py <<'EOF'
import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm, quat_to_R
from scipy.spatial.transform import Rotation as Rot
a=Arm()
lim=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(s): s=np.array(s); return np.min(np.minimum(s-lim[:,0], lim[:,1]-s))
seed=a.arm_q()
for yaw in [90,-90]:
    q=(Rot.from_euler('z',yaw,degrees=True)*Rot.from_quat([1,0,0,0])).as_quat()
    s=seed
    for z in [0.65,0.57,0.49]:
        s=a.ik((0.017,-0.236,z),q,seed=s,at_tcp=True)
        if s is None: print(yaw,z,"fail"); break
        pos,fq=a.fk(s)
        print(f"yaw={yaw} z={z}: {np.round(s,3)} margin={margin(s):.2f} fingeraxis={np.round(quat_to_R(fq)[:,1],2)}")
rclpy.shutdown()
EOF
timeout 600 python3 ikprobe8.py

# openrua op 35
timeout 1500 python3 -u pick.py 0.017 -0.236 0.49 0.65 0 90 -0.146,0.208,-0.285,-2.076,0.076,2.274,-1.259 2>&1 | tee pick_tomato.log

# openrua op 36
timeout 300 python3 arm.py js; timeout 900 python3 arm.py joints -0.146,0.208,-0.285,-2.076,0.076,2.274,-1.259 --sec 10; timeout 300 python3 arm.py fk

# openrua op 37
timeout 1500 python3 -u pick.py 0.017 -0.236 0.49 0.65 0 90 -0.146,0.208,-0.285,-2.076,0.076,2.274,-1.259 2>&1 | tee pick_tomato.log

# openrua op 38
timeout 120 python3 scene.py birdview 320,240 >/dev/null; python3 -c "
import numpy as np, cv2
c=np.load('birdview_cloud.npy')
m=(c[...,0]>-0.08)&(c[...,0]<0.08)&(c[...,1]>0.17)&(c[...,1]<0.34)&(c[...,2]>0.44)&(c[...,2]<0.60)
p=c[m]; print('inside basket, z<0.60: n=',len(p)); 
if len(p): print('xy center',p[:,:2].mean(0).round(3),'zmax',p[:,2].max().round(3),'xrange',p[:,0].min().round(3),p[:,0].max().round(3),'yrange',p[:,1].min().round(3),p[:,1].max().round(3))
im=cv2.imread('birdview.png'); cv2.imwrite('bird_basket.png', cv2.resize(im[250:330,340:420],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"

# openrua op 39
timeout 120 python3 scene.py agentview 320,240 >/dev/null; python3 -c "
import numpy as np
c=np.load('agentview_cloud.npy')
m=(c[...,0]>-0.07)&(c[...,0]<0.07)&(c[...,1]>0.18)&(c[...,1]<0.33)&(c[...,2]>0.43)&(c[...,2]<0.58)&np.isfinite(c[...,2])
p=c[m]; print('n',len(p))
import collections
for z0 in np.arange(0.43,0.58,0.02):
    q=p[(p[:,2]>=z0)&(p[:,2]<z0+0.02)]
    if len(q): print(f'z {z0:.2f}-{z0+0.02:.2f}: n={len(q)} xy mean {q[:,:2].mean(0).round(3)} x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f}')
"

# openrua op 40
timeout 1500 python3 -u place.py -0.03 0.29 0.70 0 2>&1 | tee place_tomato.log

# openrua op 41
timeout 120 python3 tools/perception/cam_snap.py agentview agent4.png && timeout 120 python3 tools/perception/cam_snap.py frontview front4.png && timeout 120 python3 tools/perception/cam_snap.py sideview side4.png

# openrua op 42
timeout 120 python3 scene.py birdview 320,240 >/dev/null; python3 -c "
import numpy as np, cv2
c=np.load('birdview_cloud.npy')
def zmax(name,x0,x1,y0,y1):
    m=(c[...,0]>x0)&(c[...,0]<x1)&(c[...,1]>y0)&(c[...,1]<y1)&np.isfinite(c[...,2])
    p=c[m]; print(f'{name:22s} zmax={p[:,2].max():.3f} (table=0.425)')
zmax('old soup spot',-0.30,-0.21,-0.18,-0.09)
zmax('old tomato spot',-0.03,0.06,-0.28,-0.19)
zmax('basket footprint',-0.09,0.09,0.16,0.35)
zmax('basket interior',-0.06,0.06,0.19,0.32)
m=(c[...,0]>-0.06)&(c[...,0]<0.06)&(c[...,1]>0.19)&(c[...,1]<0.32)&(c[...,2]>0.50)&(c[...,2]<0.62)
p=c[m]; print('interior pts above 0.50 (can tops): n=',len(p),'xy',p[:,:2].mean(0).round(3) if len(p) else '-')
im=cv2.imread('agent4.png'); cv2.imwrite('agent4_basket.png', cv2.resize(im[190:360,430:630],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 43
python3 -c "
import numpy as np
c=np.load('birdview_cloud.npy')
m=(c[...,0]>-0.09)&(c[...,0]<0.09)&(c[...,1]>0.16)&(c[...,1]<0.35)&(c[...,2]>0.44)&(c[...,2]<0.9)
p=c[m]; print('basket-footprint points below arm: zmax=%.3f (rim=0.627)'%p[:,2].max())
q=p[p[:,2]>0.60]; print('points above 0.60: n=%d'%len(q), 'x %.3f..%.3f y %.3f..%.3f'%(q[:,0].min(),q[:,0].max(),q[:,1].min(),q[:,1].max()) if len(q) else '')
"

# openrua op 44
timeout 900 python3 arm.py joints 0,-0.161,0,-2.4446,0,2.2268,0.7854 --sec 6 && timeout 120 python3 scene.py birdview 320,240 >/dev/null && python3 -c "
import numpy as np
c=np.load('birdview_cloud.npy')
m=(c[...,0]>-0.10)&(c[...,0]<0.10)&(c[...,1]>0.15)&(c[...,1]<0.36)&(c[...,2]>0.44)&np.isfinite(c[...,2])
p=c[m]; print('basket footprint zmax=%.3f (rim=0.627)'%p[:,2].max())
q=p[p[:,2]>0.505]; print('points above 0.505 (can tops etc): n=%d x %.3f..%.3f y %.3f..%.3f'%(len(q),q[:,0].min(),q[:,0].max(),q[:,1].min(),q[:,1].max()))
" && timeout 120 python3 tools/perception/cam_snap.py agentview agent5.png
