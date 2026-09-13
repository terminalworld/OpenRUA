#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
find /workspace/tools /workspace/.ros -type f | xargs ls -la; echo ---; ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | head -40; echo ---; ros2 node list

# openrua op 2
mkdir -p snaps && for c in agentview frontview birdview sideview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c snaps/$c.png & done; wait; ros2 topic echo /joint_states --once

# openrua op 3
python3 -c "
import cv2
im=cv2.imread('snaps/agentview.png')
crop=im[150:420,80:420]
cv2.imwrite('snaps/agent_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 4
python3 -c "
import cv2
im=cv2.imread('snaps/agentview.png')
cv2.imwrite('snaps/can_crop.png', cv2.resize(im[220:310,320:400],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
cv2.imwrite('snaps/cc_crop.png', cv2.resize(im[335:410,140:210],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"; for c in galleryview paperview robot0_robotview; do python3 tools/perception/cam_snap.py $c snaps/$c.png & done; wait

# openrua op 5
cat > tools/perception/px2world_batch.py <<'EOF'
#!/usr/bin/env python3
"""Batch pixel->world: python3 px2world_batch.py <camera> u,v [u,v ...]
Also prints a depth patch median option: use u,v,r to take median depth over (2r+1)^2 window."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

def _grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]

def main():
    cam = sys.argv[1]
    rclpy.init(); node = rclpy.create_node("px2world_batch")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth = _grab(node, f"/{cam}/depth/image_raw", Image)
    info = _grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()): break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation; x, y, zz, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+zz*zz), 2*(x*y-zz*w), 2*(x*zz+y*w)],
                  [2*(x*y+zz*w), 1-2*(x*x+zz*zz), 2*(y*zz-x*w)],
                  [2*(x*zz-y*w), 2*(y*zz+x*w), 1-2*(x*x+y*y)]])
    T = np.eye(4); T[:3,:3] = R
    T[:3,3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    for a in sys.argv[2:]:
        parts = [int(v) for v in a.split(",")]
        u, v = parts[0], parts[1]; r = parts[2] if len(parts) > 2 else 0
        patch = D[max(0,v-r):v+r+1, max(0,u-r):u+r+1]
        patch = patch[np.isfinite(patch) & (patch > 0)]
        if patch.size == 0:
            print(f"{u},{v}: no depth"); continue
        z = float(np.median(patch))
        p = T @ np.array([(u-cx)*z/fx, (v-cy)*z/fy, z, 1.0])
        print(f"{u},{v}: depth={z:.3f} world=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
EOF
python3 tools/perception/px2world_batch.py agentview 355,238,2 355,262,3 175,372,3 178,392,2 530,270,3 300,420,3 400,150,3

# openrua op 6
timeout 8 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 8 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ---; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -60

# openrua op 7
cat > tools/perception/heightmap.py <<'EOF'
#!/usr/bin/env python3
"""Top-down height map from a camera's depth: python3 heightmap.py <camera> <out_prefix>
Saves <out_prefix>.npy (world points Nx3 above table) and prints object clusters."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2

def _grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got: raise SystemExit(f"no message on {topic}")
    return got["m"]

def main():
    cam, out = sys.argv[1], sys.argv[2]
    zmin = float(sys.argv[3]) if len(sys.argv) > 3 else 0.435
    rclpy.init(); node = rclpy.create_node("heightmap")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth = _grab(node, f"/{cam}/depth/image_raw", Image)
    info = _grab(node, f"/{cam}/color/camera_info", CameraInfo)
    color = _grab(node, f"/{cam}/color/image_raw", Image)
    C = np.frombuffer(color.data, dtype=np.uint8).reshape(color.height, color.width, -1)[:, :, :3]
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()): break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation; x, y, zz, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+zz*zz), 2*(x*y-zz*w), 2*(x*zz+y*w)],
                  [2*(x*y+zz*w), 1-2*(x*x+zz*zz), 2*(y*zz-x*w)],
                  [2*(x*zz-y*w), 2*(y*zz+x*w), 1-2*(x*x+y*y)]])
    tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    vs, us = np.mgrid[0:depth.height, 0:depth.width]
    Z = D; ok = np.isfinite(Z) & (Z > 0)
    X = (us - cx) * Z / fx; Y = (vs - cy) * Z / fy
    P = np.stack([X, Y, Z], -1)[ok] @ R.T + tr
    col = C[ok]
    sel = (P[:, 2] > zmin) & (P[:, 2] < 0.75) & (np.abs(P[:, 0]) < 0.6) & (np.abs(P[:, 1]) < 0.6)
    P, col = P[sel], col[sel]
    np.save(out + ".npy", np.hstack([P, col]))
    # top-down raster: 2.5 mm cells over x,y in [-0.6,0.6]
    res = 0.0025; n = int(1.2 / res)
    img = np.zeros((n, n, 3), np.uint8); hm = np.zeros((n, n), np.float32)
    ix = ((P[:, 0] + 0.6) / res).astype(int).clip(0, n-1)
    iy = ((P[:, 1] + 0.6) / res).astype(int).clip(0, n-1)
    order = np.argsort(P[:, 2])
    img[n-1-iy[order], ix[order]] = col[order]  # y up, x right
    hm[n-1-iy[order], ix[order]] = P[order, 2]
    # grid lines every 0.1 m
    for k in range(0, n, int(0.1/res)):
        img[k, :] = (60, 60, 60); img[:, k] = (60, 60, 60)
    cv2.imwrite(out + ".png", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    np.save(out + "_hm.npy", hm)
    print("saved", out, "points:", len(P))

if __name__ == "__main__":
    main()
EOF
python3 tools/perception/heightmap.py agentview snaps/hm_agent && python3 tools/perception/heightmap.py frontview snaps/hm_front

# openrua op 8
python3 -c "
import cv2
im=cv2.imread('snaps/hm_agent.png')
# world x in [-0.6,0.6] -> col 0..480 ; y in [-0.6,0.6] -> row 480..0
# crop x in [-0.3,0.4], y in [-0.35,0.35]
def c(x): return int((x+0.6)/0.0025)
def r(y): return 480-1-int((y+0.6)/0.0025)
crop=im[r(0.35):r(-0.35), c(-0.3):c(0.4)]
cv2.imwrite('snaps/hm_agent_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 9
python3 -c "
import numpy as np
for f in ['snaps/hm_agent.npy','snaps/hm_front.npy']:
    P=np.load(f)[:,:3]
    print(f)
    def region(name,x0,x1,y0,y1,zmin=0.435):
        s=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>zmin)
        Q=P[s]
        if len(Q)<5: print(name,'none'); return
        top=Q[Q[:,2]>np.percentile(Q[:,2],80)]
        c=top[:,:2].mean(0); 
        cov=np.cov(top[:,:2].T); w,v=np.linalg.eigh(cov)
        ang=np.degrees(np.arctan2(v[1,1],v[0,1]))
        print(f'{name}: n={len(Q)} top-center=({c[0]:.4f},{c[1]:.4f}) zmax={Q[:,2].max():.4f} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}] major-axis-angle={ang:.1f}deg extents={2*np.sqrt(w)*1.7}')
    region('tomato_can',-0.16,-0.05,0.0,0.10)
    region('cream_cheese',0.04,0.15,-0.21,-0.14)
    region('basket',-0.1,0.12,0.15,0.36,0.5)
    region('table', -0.3,0.4,-0.4,0.4,0.0)
"


# openrua op 10
python3 -c "
import numpy as np
for f in ['snaps/hm_agent.npy','snaps/hm_front.npy']:
    P=np.load(f)[:,:3]
    print(f)
    def region(name,x0,x1,y0,y1,zmin,zmax):
        s=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>zmin)&(P[:,2]<zmax)
        Q=P[s]
        if len(Q)<5: print(name,'none'); return
        c=Q[:,:2].mean(0); 
        cov=np.cov(Q[:,:2].T); w,v=np.linalg.eigh(cov)
        ang=np.degrees(np.arctan2(v[1,1],v[0,1]))
        print(f'{name}: n={len(Q)} center=({c[0]:.4f},{c[1]:.4f}) z[{Q[:,2].min():.3f},{Q[:,2].max():.3f}] x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}] major-axis={ang:.1f}deg')
    region('tomato_can_top',-0.16,-0.04,0.0,0.10,0.50,0.53)
    region('tomato_can_all',-0.16,-0.04,0.0,0.10,0.435,0.53)
    region('cream_cheese_top',0.03,0.15,-0.215,-0.14,0.448,0.47)
    region('cream_cheese_all',0.03,0.15,-0.215,-0.14,0.435,0.47)
    region('oj',0.0,0.06,-0.30,-0.20,0.435,0.6)
    region('milk',0.03,0.15,-0.15,-0.05,0.435,0.6)
"


# openrua op 11
python3 tools/action/gripper_cmd.py 0.04 && python3 tools/action/ik_move.py -0.104 0.046 0.62 1 0 0 0 4 --at tcp && timeout 6 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A1 Translation | head -2

# openrua op 12
mkdir -p "$(dirname /workspace/tools/action/arm.py)"
cat > /workspace/tools/action/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""World-frame TCP mover for this Panda.

Usage:
  python3 tools/action/arm.py tcp <x> <y> <z> <yaw_deg> [seconds=4]
      top-down grasp pose: fingers axis = world y rotated by yaw_deg about z
  python3 tools/action/arm.py where
      print current hand + TCP pose in world (via TF)
Coordinates are WORLD frame; converted to panda_link0 for IK using the
world->panda_link0 TF. Verifies the reached pose from TF afterwards.
"""
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parents[2]
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
TCP_OFF = float(M["hand"]["tcp_offset_m"])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_tool")
        self.tf = Buffer()
        TransformListener(self.tf, self.node)
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            self.spin()
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def lookup(self, parent, child):
        end = self.node.get_clock().now().nanoseconds / 1e9 + 10
        while not self.tf.can_transform(parent, child, Time()):
            self.spin()
            if self.node.get_clock().now().nanoseconds / 1e9 > end:
                raise SystemExit(f"no TF {parent}->{child}")
        t = self.tf.lookup_transform(parent, child, Time())
        tr = t.transform.translation
        q = t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), (q.x, q.y, q.z, q.w)

    def where(self):
        # spin a bit to refresh the TF buffer with fresh stamps
        for _ in range(10):
            self.spin(0.1)
        p, q = self.lookup("world", "panda_hand")
        R = quat_to_R(*q)
        tcp = p + TCP_OFF * R[:, 2]
        j = self.joints()
        print(f"hand world=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}) q=({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})")
        print(f"tcp  world=({tcp[0]:.4f}, {tcp[1]:.4f}, {tcp[2]:.4f})")
        print("fingers:", j.get("panda_finger_joint1"), j.get("panda_finger_joint2"))
        return tcp

    def tcp(self, x, y, z, yaw_deg, seconds=4.0):
        th = math.radians(yaw_deg)
        q = (math.cos(th / 2), math.sin(th / 2), 0.0, 0.0)  # Rz(yaw)*Rx(180)
        R = quat_to_R(*q)
        hand_w = np.array([x, y, z]) - TCP_OFF * R[:, 2]
        base_p, base_q = self.lookup("world", "panda_link0")
        assert abs(base_q[3] - 1) < 1e-3, "base rotated; handle rotation"
        hand_b = hand_w - base_p
        if not self.ik.wait_for_service(timeout_sec=10):
            raise SystemExit("IK service unavailable")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        cur = self.joints()
        seed = JointState()
        for n in TRAJ["joints"]:
            seed.name.append(n)
            seed.position.append(cur[n])
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"IK FAILED code={None if res is None else res.error_code.val}; no motion")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        target = [sol[j] for j in TRAJ["joints"]]
        print("IK ok, target joints:", [round(v, 3) for v in target])
        self.goto(target, seconds)
        after = self.joints()
        err = max(abs(after[j] - t) for j, t in zip(TRAJ["joints"], target))
        print(f"max joint err after move: {err:.4f} rad")
        return self.where()

    def goto(self, target, seconds):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(TRAJ["joints"])
        pt = JointTrajectoryPoint(positions=[float(v) for v in target])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        print(f"trajectory done error_code={code}")


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    arm = Arm()
    if a[0] == "where":
        arm.where()
    elif a[0] == "tcp":
        x, y, z, yaw = map(float, a[1:5])
        secs = float(a[5]) if len(a) > 5 else 4.0
        arm.tcp(x, y, z, yaw, secs)
    else:
        raise SystemExit(__doc__)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 13
python3 tools/action/arm.py where && python3 tools/action/arm.py tcp -0.104 0.046 0.62 0 4

# openrua op 14
cat > /tmp/iktest.py <<'EOF'
import sys; sys.path.insert(0,'/workspace/tools/action')
import arm as A, rclpy, math, numpy as np
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
a=A.Arm()
a.ik.wait_for_service(timeout_sec=10)
cur=a.joints()
def try_ik(hb, q, seed=None, timeout=5):
    req=GetPositionIK.Request(); r=req.ik_request
    r.group_name='panda_arm'; r.pose_stamped.header.frame_id=''
    p=r.pose_stamped.pose; p.position.x,p.position.y,p.position.z=hb
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q
    s=JointState()
    for n in A.TRAJ['joints']: s.name.append(n); s.position.append((seed or cur)[n])
    r.robot_state.joint_state=s; r.avoid_collisions=False; r.timeout.sec=timeout
    f=a.ik.call_async(req); rclpy.spin_until_future_complete(a.node,f,timeout_sec=60)
    res=f.result(); return res.error_code.val, (dict(zip(res.solution.joint_state.name,res.solution.joint_state.position)) if res.error_code.val==1 else None)
# sanity: current hand pose in base
p,q=a.lookup('panda_link0','panda_hand'); print('cur hand in base',p,q)
print('cur pose ->', try_ik(tuple(p), q)[0])
print('target straight down ->', try_ik((0.406,0.046,0.3034),(1,0,0,0))[0])
print('target w/ small pitch ->', try_ik((0.406,0.046,0.3034),(1,0,-0.028,0))[0])
print('target higher ->', try_ik((0.406,0.046,0.40),(1,0,0,0))[0])
print('target 45deg ->', try_ik((0.406,0.046,0.3034),(0.9239,0.3827,0,0))[0])
EOF
python3 /tmp/iktest.py

# openrua op 15
sed -i 's/p.position.x,p.position.y,p.position.z=hb/p.position.x,p.position.y,p.position.z=map(float,hb)/; s/p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q$/p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q)/' /tmp/iktest.py && python3 /tmp/iktest.py

# openrua op 16
ros2 param get /move_group robot_description_kinematics.panda_arm.kinematics_solver 2>&1; ros2 param list /move_group 2>/dev/null | grep -i -E "kinematics|solver" | head; cat >> /tmp/iktest.py <<'EOF'
def try_ik2(hb,q,link='',empty_seed=False,timeout=0):
    req=GetPositionIK.Request(); r=req.ik_request
    r.group_name='panda_arm'; r.pose_stamped.header.frame_id=''; r.ik_link_name=link
    p=r.pose_stamped.pose; p.position.x,p.position.y,p.position.z=map(float,hb)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q)
    if not empty_seed:
        s=JointState()
        for n in A.TRAJ['joints']: s.name.append(n); s.position.append(cur[n])
        r.robot_state.joint_state=s
    r.avoid_collisions=False; r.timeout.sec=timeout
    f=a.ik.call_async(req); rclpy.spin_until_future_complete(a.node,f,timeout_sec=60)
    res=f.result(); return res.error_code.val
print('link panda_hand', try_ik2(tuple(p),q,'panda_hand'))
print('link panda_link8', try_ik2(tuple(p),q,'panda_link8'))
print('empty seed', try_ik2(tuple(p),q,'',True))
print('timeout 0 default', try_ik2(tuple(p),q,'',False,0))
EOF
python3 /tmp/iktest.py 2>&1 | tail -5

# openrua op 17
cat > /tmp/fktest.py <<'EOF'
import sys; sys.path.insert(0,'/workspace/tools/action')
import arm as A, rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
a=A.Arm()
cli=a.node.create_client(GetPositionFK,'/compute_fk'); cli.wait_for_service(timeout_sec=10)
cur=a.joints()
req=GetPositionFK.Request(); req.header.frame_id=''; req.fk_link_names=['panda_hand','panda_link8','panda_link0']
s=JointState()
for n in A.TRAJ['joints']: s.name.append(n); s.position.append(cur[n])
req.robot_state.joint_state=s
f=cli.call_async(req); rclpy.spin_until_future_complete(a.node,f,timeout_sec=60)
r=f.result(); print('code',r.error_code.val)
for n,ps in zip(r.fk_link_names,r.pose_stamped):
    p=ps.pose.position;q=ps.pose.orientation; print(n,ps.header.frame_id,(round(p.x,4),round(p.y,4),round(p.z,4)),(round(q.x,3),round(q.y,3),round(q.z,3),round(q.w,3)))
EOF
python3 /tmp/fktest.py; ros2 param get /move_group robot_description_kinematics.panda_arm.kinematics_solver_timeout; ros2 param get /move_group robot_description_kinematics.panda_arm.epsilon; ros2 param get /move_group robot_description_kinematics.panda_arm.max_solver_iterations; ros2 param get /move_group robot_description_kinematics.panda_arm.joints

# openrua op 18
cat >> /tmp/iktest.py <<'EOF'
print('WORLD cur pose ->', try_ik2((-0.053,0,0.7776),(1,0,-0.028,0)))
print('WORLD target above can ->', try_ik2((-0.104,0.046,0.7234),(1,0,0,0)))
EOF
python3 /tmp/iktest.py 2>&1 | tail -2

# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/tools/action/arm.py", "old_string": "        base_p, base_q = self.lookup(\\"world\\", \\"panda_link0\\")\\n        assert abs(base_q[3] - 1) < 1e-3, \\"base rotated; handle rotation\\"\\n        hand_b = hand_w - base_p", "new_string": "        # verified via /compute_fk: move_group\'s model frame IS world here\\n        # (panda_link0 reported at world (-0.51, 0, 0.42)), so pass world\\n        # coordinates straight through with an empty frame_id\\n        hand_b = hand_w", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
python3 tools/action/arm.py tcp -0.104 0.046 0.62 0 4

# openrua op 21
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/tools/action/arm.py", "old_string": "        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "        req.ik_request.pose_stamped.header.frame_id = \\"\\"\\n        req.ik_request.ik_link_name = \\"panda_hand\\"  # group tip is link8 (45deg off)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 22
python3 tools/action/arm.py tcp -0.104 0.046 0.60 0 3 && python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/hand1.png

# openrua op 23
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/tools/action/arm.py", "old_string": "        self.goto(target, seconds)\\n        after = self.joints()\\n        err = max(abs(after[j] - t) for j, t in zip(TRAJ[\\"joints\\"], target))\\n        print(f\\"max joint err after move: {err:.4f} rad\\")", "new_string": "        for attempt in range(3):\\n            self.goto(target, seconds if attempt == 0 else 2.0)\\n            after = self.joints()\\n            err = max(abs(after[j] - t) for j, t in zip(TRAJ[\\"joints\\"], target))\\n            print(f\\"max joint err after move: {err:.4f} rad\\")\\n            if err < 0.02:\\n                break\\n            print(\\"resending (controller lag)\\")", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 24
python3 tools/action/arm.py tcp -0.104 0.046 0.60 0 3 && python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/hand1.png

# openrua op 25
python3 tools/perception/heightmap.py robot0_eye_in_hand snaps/hm_hand 0.435 && python3 -c "
import numpy as np
P=np.load('snaps/hm_hand.npy')[:,:3]
s=(P[:,0]>-0.16)&(P[:,0]<-0.04)&(P[:,1]>0.0)&(P[:,1]<0.10)&(P[:,2]>0.50)&(P[:,2]<0.53)
Q=P[s]; print('can top n',len(Q),'center',Q[:,:2].mean(0),'x',Q[:,0].min(),Q[:,0].max(),'y',Q[:,1].min(),Q[:,1].max(),'z',Q[:,2].min(),Q[:,2].max())
s=(P[:,0]>-0.16)&(P[:,0]<-0.04)&(P[:,1]>0.0)&(P[:,1]<0.10)&(P[:,2]>0.44)&(P[:,2]<0.53)
Q=P[s]; print('can all n',len(Q),'x',Q[:,0].min(),Q[:,0].max(),'y',Q[:,1].min(),Q[:,1].max())
"

# openrua op 26
python3 tools/action/arm.py tcp -0.1045 0.0465 0.475 0 3 && python3 tools/action/gripper_cmd.py 0.0 && python3 tools/action/arm.py where

# openrua op 27
python3 tools/action/arm.py tcp -0.1045 0.0465 0.72 0 3 && python3 tools/action/arm.py tcp 0.016 0.263 0.74 0 4 && python3 tools/perception/cam_snap.py agentview snaps/carry1.png

# openrua op 28
python3 tools/action/arm.py tcp 0.016 0.263 0.68 0 2 && python3 tools/action/gripper_cmd.py 0.04 && python3 tools/action/arm.py tcp 0.016 0.263 0.80 0 2 && python3 tools/perception/cam_snap.py agentview snaps/after1.png

# openrua op 29
python3 tools/action/arm.py tcp 0.094 -0.178 0.60 12 4 && python3 tools/perception/heightmap.py robot0_eye_in_hand snaps/hm_hand2 0.435 && python3 -c "
import numpy as np
P=np.load('snaps/hm_hand2.npy')[:,:3]
s=(P[:,0]>0.03)&(P[:,0]<0.16)&(P[:,1]>-0.23)&(P[:,1]<-0.13)&(P[:,2]>0.45)&(P[:,2]<0.48)
Q=P[s]; c=Q[:,:2].mean(0); cov=np.cov(Q[:,:2].T); w,v=np.linalg.eigh(cov)
print('cc top n',len(Q),'center',c,'z',Q[:,2].min(),Q[:,2].max(),'x',Q[:,0].min(),Q[:,0].max(),'y',Q[:,1].min(),Q[:,1].max())
print('major axis angle deg', np.degrees(np.arctan2(v[1,1],v[0,1])), 'extents ~', 4*np.sqrt(w))
"

# openrua op 30
python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/hand2.png

# openrua op 31
python3 -c "
import numpy as np
P=np.load('snaps/hm_hand2.npy'); C=P[:,3:]; P=P[:,:3]
s=(P[:,0]>0.05)&(P[:,0]<0.16)&(P[:,1]>-0.23)&(P[:,1]<-0.15)&(P[:,2]>0.455)&(P[:,2]<0.475)
Q=P[s]; c=Q[:,:2].mean(0); cov=np.cov(Q[:,:2].T); w,v=np.linalg.eigh(cov)
print('cc top n',len(Q),'center',c,'z',Q[:,2].min(),Q[:,2].max(),'x',Q[:,0].min(),Q[:,0].max(),'y',Q[:,1].min(),Q[:,1].max())
ang=np.degrees(np.arctan2(v[1,1],v[0,1])); print('major axis angle deg', ang, 'extents ~', 2*np.sqrt(3*w))
# project to axes for extents
d=Q[:,:2]-c; a=np.radians(ang); u=d@np.array([np.cos(a),np.sin(a)]); vv=d@np.array([-np.sin(a),np.cos(a)])
print('along',u.min(),u.max(),'across',vv.min(),vv.max())
print('mean color',C[s].mean(0))
"

# openrua op 32
python3 -c "
import numpy as np
P=np.load('snaps/hm_hand2.npy'); C=P[:,3:]; P=P[:,:3]
s=(P[:,0]>0.05)&(P[:,0]<0.16)&(P[:,1]>-0.23)&(P[:,1]<-0.15)&(P[:,2]>0.43)
Q=P[s]
h,e=np.histogram(Q[:,2],bins=np.arange(0.43,0.50,0.005)); 
for hh,ee in zip(h,e): print(f'{ee:.3f} {hh}')
for lo,hi in [(0.435,0.445),(0.445,0.452),(0.452,0.458)]:
    R=Q[(Q[:,2]>lo)&(Q[:,2]<hi)]
    if len(R): print(lo,hi,'n',len(R),'x',R[:,0].min().round(3),R[:,0].max().round(3),'y',R[:,1].min().round(3),R[:,1].max().round(3),'center',R[:,:2].mean(0).round(4))
"

# openrua op 33
python3 -c "
import numpy as np
P=np.load('snaps/hm_hand2.npy')[:,:3]
s=(P[:,0]>0.05)&(P[:,0]<0.16)&(P[:,1]>-0.23)&(P[:,1]<-0.15)&(P[:,2]>0.45)&(P[:,2]<0.458)
Q=P[s]; c=Q[:,:2].mean(0); cov=np.cov(Q[:,:2].T); w,v=np.linalg.eigh(cov)
ang=np.degrees(np.arctan2(v[1,1],v[0,1])); ang=(ang+180)%180
a=np.radians(ang); d=Q[:,:2]-c; u=d@np.array([np.cos(a),np.sin(a)]); vv=d@np.array([-np.sin(a),np.cos(a)])
print('center',c.round(4),'angle',round(ang,1),'along',u.min().round(3),u.max().round(3),'across',vv.min().round(3),vv.max().round(3))
"

# openrua op 34
python3 - <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('urdf_get')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time
end=time.time()+10
while not got and time.time()<end: rclpy.spin_once(n,timeout_sec=0.5)
open('snaps/robot.urdf','w').write(got[0] if got else '')
print(len(got[0]) if got else 'none')
EOF
grep -n -A12 'link name="panda_hand"' snaps/robot.urdf | head -60; grep -n -A14 'link name="panda_leftfinger"' snaps/robot.urdf | head -40; grep -n -B2 -A6 'joint name="panda_finger_joint1"' snaps/robot.urdf

# openrua op 35
python3 - <<'EOF'
import numpy as np, struct, re
def stl_bounds(path):
    d=open(path,'rb').read()
    if d[:5]==b'solid' and b'facet' in d[:300]:
        v=np.array([list(map(float,m.groups())) for m in re.finditer(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)',d)])
    else:
        n=struct.unpack_from('<I',d,80)[0]
        arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0),v.max(0)
base='/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/'
for m in ['hand.stl','finger.stl','link7.stl']:
    print(m, stl_bounds(base+m))
EOF
grep -o '<joint name="panda_hand_joint"[^/]*/>.\{0,300\}' snaps/robot.urdf | head -3; grep -o '<joint name="panda_finger_joint1".\{0,400\}' snaps/robot.urdf | head -2

# openrua op 36
python3 - <<'EOF'
import numpy as np
# hand body footprint: half-length L along finger axis, half-width W across; padding p
L,W,p=0.10,0.03,0.01
obst={'milk':(0.049,0.102,-0.119,-0.065),'oj':(0.0,0.044,-0.261,-0.208),'bluecan':(-0.110,-0.044,-0.21,-0.14)}
def rect_corners(c,ang,L,W):
    d=np.array([np.cos(ang),np.sin(ang)]); n=np.array([-d[1],d[0]])
    return np.array([c+L*d+W*n,c+L*d-W*n,c-L*d-W*n,c-L*d+W*n])
def sat(A,B):
    for poly in (A,B):
        for i in range(len(poly)):
            e=poly[(i+1)%len(poly)]-poly[i]; ax=np.array([-e[1],e[0]])
            pa=A@ax; pb=B@ax
            if pa.max()<pb.min() or pb.max()<pa.min(): return False
    return True
box=np.array([0.0933,-0.1763])
for th in range(-90,91,15):
    for dy in (0.0,-0.01,-0.02):
        c=box+np.array([0,dy])
        ang=np.radians(90+th)
        H=rect_corners(c,ang,L+p,W+p)
        hits=[k for k,(x0,x1,y0,y1) in obst.items() if sat(H,np.array([[x0,y0],[x1,y0],[x1,y1],[x0,y1]]))]
        print(f'yaw {th:4d} dy {dy:+.2f}: {"clear" if not hits else hits}')
EOF

# openrua op 37
python3 -c "
import numpy as np
P=np.load('snaps/hm_hand2.npy')[:,:3]
s=(P[:,0]>0.03)&(P[:,0]<0.13)&(P[:,1]>-0.14)&(P[:,1]<-0.04)&(P[:,2]>0.54)
Q=P[s]; print('milk top n',len(Q),'center',Q[:,:2].mean(0).round(4),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].max().round(3))
s=(P[:,0]>-0.13)&(P[:,0]<-0.03)&(P[:,1]>-0.23)&(P[:,1]<-0.12)&(P[:,2]>0.47)
Q=P[s]; print('bluecan n',len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].max().round(3))
s=(P[:,0]>0.15)&(P[:,1]>-0.3)&(P[:,1]<0.1)&(P[:,2]>0.435); print('points at x>0.15:',s.sum())
"

# openrua op 38
python3 -c "
import numpy as np
P=np.load('snaps/hm_hand2.npy')[:,:3]
s=(P[:,0]>0.03)&(P[:,0]<0.13)&(P[:,1]>-0.14)&(P[:,1]<-0.04)&(P[:,2]>0.54)&(P[:,2]<0.57)
Q=P[s]; print('milk top n',len(Q),'center',Q[:,:2].mean(0).round(4),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].min().round(3),Q[:,2].max().round(3))
s=(P[:,0]>0.15)&(P[:,1]>-0.3)&(P[:,1]<0.1)&(P[:,2]>0.435)&(P[:,2]<0.6); print('points at x>0.15:',s.sum())
"; python3 -c "
import numpy as np
P=np.load('snaps/hm_agent.npy')[:,:3]
s=(P[:,0]>0.15)&(P[:,1]>-0.3)&(P[:,1]<0.1)&(P[:,2]>0.435)&(P[:,2]<0.6); print('agentview points at x>0.15:',s.sum())
s=(P[:,0]>-0.13)&(P[:,0]<-0.03)&(P[:,1]>-0.23)&(P[:,1]<-0.12)&(P[:,2]>0.44)&(P[:,2]<0.6)
Q=P[s]; print('bluecan n',len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].max().round(3))
"

# openrua op 39
python3 -c "
import numpy as np
P=np.load('snaps/hm_hand2.npy')[:,:3]
s=(P[:,0]>0.15)&(P[:,1]>-0.3)&(P[:,1]<0.1)&(P[:,2]>0.435)&(P[:,2]<0.6); Q=P[s]
print('x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].min().round(3),Q[:,2].max().round(3))
P=np.load('snaps/hm_front.npy')[:,:3]
s=(P[:,0]>0.15)&(P[:,1]>-0.3)&(P[:,1]<0.1)&(P[:,2]>0.435)&(P[:,2]<0.6); print('frontview pts x>0.15:',s.sum())
"

# openrua op 40
python3 tools/action/arm.py tcp 0.0755 -0.0915 0.65 0 4 && python3 tools/action/arm.py tcp 0.0755 -0.0915 0.52 0 3 && python3 tools/action/gripper_cmd.py 0.0 && python3 tools/action/arm.py where

# openrua op 41
python3 tools/action/arm.py tcp 0.0755 -0.0915 0.70 0 3 && python3 tools/action/arm.py tcp 0.20 -0.05 0.70 0 4 && python3 tools/action/arm.py tcp 0.20 -0.05 0.54 0 3 && python3 tools/action/gripper_cmd.py 0.04 && python3 tools/action/arm.py tcp 0.20 -0.05 0.70 0 3

# openrua op 42
python3 tools/action/arm.py tcp 0.0933 -0.1763 0.60 0 4 && python3 tools/perception/heightmap.py robot0_eye_in_hand snaps/hm_hand3 0.43 && python3 -c "
import numpy as np
P=np.load('snaps/hm_hand3.npy')[:,:3]
s=(P[:,0]>0.04)&(P[:,0]<0.16)&(P[:,1]>-0.23)&(P[:,1]<-0.13)&(P[:,2]>0.448)&(P[:,2]<0.46)
Q=P[s]; c=Q[:,:2].mean(0); cov=np.cov(Q[:,:2].T); w,v=np.linalg.eigh(cov)
ang=(np.degrees(np.arctan2(v[1,1],v[0,1]))+180)%180
print('cc n',len(Q),'center',c.round(4),'angle',round(ang,1),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].min().round(3),Q[:,2].max().round(3))
s=(P[:,0]>0.03)&(P[:,0]<0.13)&(P[:,1]>-0.14)&(P[:,1]<-0.04)&(P[:,2]>0.46)&(P[:,2]<0.6); print('anything left at old milk spot:',s.sum())
s=(P[:,0]>0.15)&(P[:,0]<0.25)&(P[:,1]>-0.1)&(P[:,1]<0.0)&(P[:,2]>0.54)&(P[:,2]<0.57); print('milk at new spot pts:',s.sum())
" && python3 tools/perception/cam_snap.py agentview snaps/mid2.png

# openrua op 43
python3 -c "
import numpy as np
P=np.load('snaps/hm_hand3.npy')[:,:3]
s=(P[:,0]>0.05)&(P[:,0]<0.16)&(P[:,1]>-0.21)&(P[:,1]<-0.14)&(P[:,2]>0.45)&(P[:,2]<0.458)
Q=P[s]; c=Q[:,:2].mean(0); cov=np.cov(Q[:,:2].T); w,v=np.linalg.eigh(cov)
ang=(np.degrees(np.arctan2(v[1,1],v[0,1]))+180)%180
print('cc n',len(Q),'center',c.round(4),'angle',round(ang,1),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3))
s=(P[:,0]>0.13)&(P[:,1]>-0.3)&(P[:,1]<0.1)&(P[:,2]>0.44)&(P[:,2]<0.6); Q=P[s]
print('fallen milk footprint: x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].max().round(3))
"

# openrua op 44
python3 -c "
import numpy as np
P=np.load('snaps/hm_hand3.npy')[:,:3]
s=(P[:,0]>0.13)&(P[:,0]<0.32)&(P[:,1]>-0.3)&(P[:,1]<0.1)&(P[:,2]>0.46)&(P[:,2]<0.6); Q=P[s]
print('fallen milk: n',len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].max().round(3))
"

# openrua op 45
python3 tools/action/arm.py tcp 0.0935 -0.1765 0.50 0 2 && python3 tools/action/arm.py tcp 0.0935 -0.1765 0.438 0 2 && python3 tools/action/gripper_cmd.py 0.0 && python3 tools/action/arm.py where

# openrua op 46
python3 tools/action/arm.py tcp 0.0935 -0.1765 0.70 0 3 && python3 tools/action/arm.py tcp 0.03 0.28 0.74 0 4 && python3 tools/perception/cam_snap.py agentview snaps/carry2.png

# openrua op 47
python3 tools/action/arm.py tcp 0.03 0.28 0.66 0 2 && python3 tools/action/gripper_cmd.py 0.04 && python3 tools/action/arm.py tcp 0.03 0.28 0.82 0 3 && python3 tools/action/arm.py tcp -0.15 0.15 0.85 0 3 && for c in agentview frontview birdview; do python3 tools/perception/cam_snap.py $c snaps/final_$c.png & done; wait

# openrua op 48
python3 tools/perception/heightmap.py agentview snaps/hm_final 0.43 && python3 -c "
import numpy as np
P=np.load('snaps/hm_final.npy'); C=P[:,3:]; P=P[:,:3]
# inside basket footprint, below rim
s=(P[:,0]>-0.06)&(P[:,0]<0.07)&(P[:,1]>0.19)&(P[:,1]<0.33)&(P[:,2]>0.43)&(P[:,2]<0.60)
Q=P[s]; Cq=C[s]
print('points inside basket below rim:',len(Q),'z range',Q[:,2].min().round(3),Q[:,2].max().round(3))
# cluster by height band
for lo,hi in [(0.43,0.47),(0.47,0.50),(0.50,0.55),(0.55,0.60)]:
    m=(Q[:,2]>=lo)&(Q[:,2]<hi)
    if m.sum(): print(f' z[{lo},{hi}) n={m.sum()} xy-center={Q[m][:,:2].mean(0).round(3)} mean RGB={Cq[m].mean(0).round(0)}')
# original spots empty?
for name,(x0,x1,y0,y1) in {'can_origin':(-0.14,-0.07,0.01,0.085),'cc_origin':(0.05,0.135,-0.20,-0.15)}.items():
    m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.44)
    print(name,'points above table:',m.sum())
"

# openrua op 49
python3 -c "
import cv2
im=cv2.imread('snaps/final_agentview.png'); cv2.imwrite('snaps/final_basket_crop.png', cv2.resize(im[190:330,440:620],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('snaps/final_birdview.png'); cv2.imwrite('snaps/final_bird_crop.png', cv2.resize(im[255:315,350:415],None,fx=8,fy=8,interpolation=cv2.INTER_CUBIC))"
