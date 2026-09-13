#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -v parameter | head -40; echo ----; timeout 20 ros2 node list

# openrua op 4
timeout 30 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40; echo ----; timeout 20 ros2 topic echo /agentview/color/camera_info --once

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Scene helper: TF tree dump + pixel->world for several cameras/pixels.

Usage: python3 scene.py tf
       python3 scene.py px <camera> u,v [u,v ...]
       python3 scene.py fk            # hand pose in base frame via /compute_fk
"""
import struct
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy


def grab(node, topic, msg_type, timeout=15.0, qos=1):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), qos)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic} within {timeout}s")
    return got["m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_to_T(t):
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    return T


def main():
    rclpy.init()
    node = rclpy.create_node("scene")
    cmd = sys.argv[1]
    if cmd == "tf":
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, q in (("/tf_static", qos), ("/tf", 10)):
            try:
                m = grab(node, topic, TFMessage, timeout=5, qos=q)
            except SystemExit as e:
                print(e); continue
            for t in m.transforms:
                tr, r = t.transform.translation, t.transform.rotation
                print(f"{topic}: {t.header.frame_id} -> {t.child_frame_id}: "
                      f"t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) "
                      f"q=({r.x:.4f},{r.y:.4f},{r.z:.4f},{r.w:.4f})")
    elif cmd == "px":
        cam = sys.argv[2]
        tfbuf = Buffer(); TransformListener(tfbuf, node)
        depth = grab(node, f"/{cam}/depth/image_raw", Image)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        frame = f"{cam}_optical_frame"
        import time
        end = time.time() + 10
        while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
            rclpy.spin_once(node, timeout_sec=0.2)
        T = tf_to_T(tfbuf.lookup_transform("world", frame, rclpy.time.Time()))
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        for uv in sys.argv[3:]:
            u, v = map(int, uv.split(","))
            z = struct.unpack_from("<f", depth.data, (v * depth.width + u) * 4)[0]
            if not np.isfinite(z) or z <= 0:
                print(f"({u},{v}): no depth ({z})"); continue
            p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
            print(f"({u},{v}): depth={z:.4f} world=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
    elif cmd == "fk":
        js = grab(node, "/joint_states", JointState)
        cli = node.create_client(GetPositionFK, "/compute_fk")
        cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand", "panda_link8"]
        arm = [f"panda_joint{i}" for i in range(1, 8)]
        for n, p in zip(js.name, js.position):
            if n in arm:
                req.robot_state.joint_state.name.append(n)
                req.robot_state.joint_state.position.append(p)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
        res = fut.result()
        print("error", res.error_code.val)
        for name, ps in zip(res.fk_link_names, res.pose_stamped):
            p, o = ps.pose.position, ps.pose.orientation
            print(f"{name} [{ps.header.frame_id}]: p=({p.x:.4f},{p.y:.4f},{p.z:.4f}) "
                  f"q=({o.x:.4f},{o.y:.4f},{o.z:.4f},{o.w:.4f})")
        print("joints:", dict(zip(js.name, [round(x, 4) for x in js.position])))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 60 python3 scene.py tf 2>&1 | grep -v "panda_link[1-7] ->" ; echo ----; timeout 60 python3 scene.py fk

# openrua op 8
timeout 60 python3 -c "
import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('t'); b=Buffer(); TransformListener(b,n)
end=time.time()+8
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.1)
for a,c in [('world','panda_hand'),('panda_link0','panda_hand'),('world','panda_link0'),('panda_hand','robot0_eye_in_hand_optical_frame')]:
    try:
        t=b.lookup_transform(a,c,rclpy.time.Time()).transform
        print(a,'->',c, round(t.translation.x,4),round(t.translation.y,4),round(t.translation.z,4), '| q',round(t.rotation.x,4),round(t.rotation.y,4),round(t.rotation.z,4),round(t.rotation.w,4))
    except Exception as e: print(a,c,'ERR',e)
"

# openrua op 9
timeout 60 python3 scene.py px agentview 220,235 220,260 310,232 310,245 405,230 405,260 340,385 340,360 340,410 300,385 380,385 340,300 150,300 500,300 340,450

# openrua op 10
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[230:350,240:380]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); crop=im[170:440,150:470]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"

# openrua op 11
timeout 60 python3 scene.py px birdview 285,268 283,266 287,270 326,321 326,315 326,327 320,321 332,321 200,321

# openrua op 12
mkdir -p "$(dirname /workspace/objs.py)"
cat > /workspace/objs.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment above-table blobs in a camera's depth frame and print world
bounding boxes. Usage: python3 objs.py <camera> [min_height=0.01]"""
import sys, time
import numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scene import grab, tf_to_T

cam = sys.argv[1]
minh = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01
TABLE_Z = 0.4255
rclpy.init(); node = rclpy.create_node("objs")
tfbuf = Buffer(); TransformListener(tfbuf, node)
depth = grab(node, f"/{cam}/depth/image_raw", Image)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
end = time.time() + 10
while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
T = tf_to_T(tfbuf.lookup_transform("world", frame, rclpy.time.Time()))
D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
v, u = np.mgrid[0:depth.height, 0:depth.width]
P = np.stack([(u - cx) * D / fx, (v - cy) * D / fy, D, np.ones_like(D)], -1)
W = P @ T.T
Z = W[..., 2]
mask = (Z > TABLE_Z + minh) & np.isfinite(D) & (D > 0)
n, lab = cv2.connectedComponents(mask.astype(np.uint8))
for i in range(1, n):
    m = lab == i
    if m.sum() < 15: continue
    pts = W[m]
    vs, us = np.nonzero(m)
    print(f"blob{i}: px u[{us.min()}-{us.max()}] v[{vs.min()}-{vs.max()}] n={m.sum()} "
          f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 13
timeout 60 python3 objs.py birdview 0.02; echo ----; timeout 60 python3 objs.py agentview 0.02

# openrua op 14
timeout 60 python3 scene.py px birdview 330,200 330,250 330,300 330,340 330,360 250,300 400,300 200,321 450,250; echo ---; timeout 60 python3 scene.py px agentview 340,450 340,470 340,320 340,300 200,400 480,400 100,250 550,250 340,190

# openrua op 15
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 30 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 16
timeout 60 python3 objs.py agentview 0.015; echo ----; timeout 60 python3 objs.py birdview 0.03

# openrua op 17
timeout 60 python3 objs.py birdview 0.09; echo ---; timeout 60 python3 objs.py agentview 0.09

# openrua op 18
timeout 60 python3 -c "
import numpy as np, rclpy, time, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scene import grab, tf_to_T
rclpy.init(); node=rclpy.create_node('m'); tfbuf=Buffer(); TransformListener(tfbuf,node)
cam='birdview'
depth=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
fr=f'{cam}_optical_frame'; end=time.time()+10
while time.time()<end and not tfbuf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
T=tf_to_T(tfbuf.lookup_transform('world',fr,rclpy.time.Time()))
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
v,u=np.mgrid[0:depth.height,0:depth.width]
W=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D,np.ones_like(D)],-1)@T.T
np.save('bird_world.npy',W)
# white mug: region x>-0.2, y<-0.05, z between 0.50 and 0.60
m=(W[...,0]>-0.2)&(W[...,1]<-0.05)&(W[...,2]>0.50)&(W[...,2]<0.60)
pts=W[m]; print('white mug rim n',m.sum(),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z',pts[:,2].min(),pts[:,2].max(),'mean',pts[:,0].mean(),pts[:,1].mean())
# fit circle to rim points (x,y)
x,y=pts[:,0],pts[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); print('circle fit center',c[0],c[1],'radius',r)
# what is inside the mug (depth at center)?
vs,us=np.nonzero(m); print('px range u',us.min(),us.max(),'v',vs.min(),vs.max())
# red mug region
m2=(W[...,0]>-0.2)&(W[...,1]>0.03)&(W[...,1]<0.25)&(W[...,2]>0.50)&(W[...,2]<0.60)
pts=W[m2]; print('red mug n',m2.sum(), (pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max()) if m2.sum() else '')
# pudding region
m3=(W[...,0]>-0.3)&(W[...,0]<-0.1)&(abs(W[...,1])<0.06)&(W[...,2]>0.44)&(W[...,2]<0.50)
pts=W[m3]; print('pudding n',m3.sum(), (pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].max(), pts[:,0].mean(), pts[:,1].mean()) if m3.sum() else '')
"


# openrua op 19
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Arm helper: IK (with frame test), trajectory execution, gripper, state.

  python3 arm.py iktest
  python3 arm.py goto <x> <y> <z> [seconds] [--hand]   # world TCP pose, hand down, fingers along world y
  python3 arm.py grip <per_finger_m>
  python3 arm.py state
"""
import sys, time
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # from TF world->panda_link0
TCP = M["hand"]["tcp_offset_m"]
# hand pointing straight down, fingers (hand Y) along world Y: 180 deg about X
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grp = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")

    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
        end = time.time() + 15
        while "m" not in self.js and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_joints(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def solve_ik(self, pos, quat, seed=None, frame_pos_is_world=True):
        """pos: TCP position (world if frame_pos_is_world). Returns joint list or None."""
        p = np.array(pos, float)
        R = quat_R(*quat)
        hand = p - TCP * R[:, 2]          # hand frame = TCP back along hand +Z
        if frame_pos_is_world:
            hand = hand - BASE_IN_WORLD   # IK works in the base frame
        self.ik.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, hand)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        s = JointState(); s.name = list(ARM)
        s.position = list(seed) if seed is not None else self.arm_joints()
        req.ik_request.robot_state.joint_state = s
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print("IK failed", None if res is None else res.error_code.val)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in ARM]

    def fk_hand(self, q=None):
        self.fk.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = list(q) if q is not None else self.arm_joints()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        ps = fut.result().pose_stamped[0].pose
        return (np.array([ps.position.x, ps.position.y, ps.position.z]),
                (ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w))

    def tcp_world(self):
        p, q = self.fk_hand()
        return p + TCP * quat_R(*q)[:, 2], q

    def move_joints(self, q, seconds=3.0, waypoints=None):
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = list(waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(x) for x in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        cur = self.arm_joints()
        err = max(abs(a - b) for a, b in zip(cur, q))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, pos, seconds=3.0, quat=Q_DOWN, seed=None):
        q = self.solve_ik(pos, quat, seed=seed)
        if q is None:
            return None
        code, err = self.move_joints(q, seconds)
        p, _ = self.tcp_world()
        print(f"TCP now world=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f}) target={tuple(round(x,4) for x in pos)}")
        return q

    def grip(self, width):
        self.grp.wait_for_server(timeout_sec=10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grp.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"grip reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "iktest":
        cur = a.arm_joints()
        p, q = a.fk_hand()
        print("FK hand (as returned):", p, q)
        tcp = p + TCP * quat_R(*q)[:, 2]
        for label, world in (("as-world", True), ("as-base", False)):
            sol = a.solve_ik(tcp, q, frame_pos_is_world=world)
            if sol:
                print(label, "max dev from current:", max(abs(x - y) for x, y in zip(sol, cur)))
    elif cmd == "goto":
        pos = [float(x) for x in sys.argv[2:5]]
        sec = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
        a.goto(pos, sec)
    elif cmd == "grip":
        a.grip(float(sys.argv[2]))
    elif cmd == "state":
        p, q = a.tcp_world()
        print("TCP world:", p, "hand quat:", q)
        print("fingers:", a.fingers())
        print("arm:", [round(x, 4) for x in a.arm_joints()])
    rclpy.shutdown()
OPENRUA_EOF

# openrua op 20
timeout 120 python3 arm.py iktest

# openrua op 21
timeout 120 python3 -c "
from arm import *
a=Arm(); cur=a.arm_joints(); print('cur',[round(x,3) for x in cur])
p,q=a.fk_hand(); tcp=p+TCP*quat_R(*q)[:,2]; print('tcp',tcp)
sol=a.solve_ik(tcp,q,frame_pos_is_world=False); print('sol',[round(x,3) for x in sol])
p2,q2=a.fk_hand(sol); print('fk of sol',p2,q2)
sol2=a.solve_ik(tcp,Q_DOWN,frame_pos_is_world=False); print('sol down',[round(x,3) for x in sol2] if sol2 else None)
if sol2: print('fk of sol2', a.fk_hand(sol2))
rclpy.shutdown()"

# openrua op 22
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        self.ik.wait_for_service(timeout_sec=10)\\n        req = GetPositionIK.Request()", "new_string": "        # IK solves for panda_link8 = hand rotated +45deg about Z (TF link8->hand is -45deg)\\n        q8 = quat_mul(quat, (0.0, 0.0, 0.3826834, 0.9238795))\\n        self.ik.wait_for_service(timeout_sec=10)\\n        req = GetPositionIK.Request()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 23
python3 - <<'EOF'
import re
s=open('arm.py').read()
s=s.replace("pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, quat)",
            "pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q8)")
s=s.replace("""def quat_R(x, y, z, w):""","""def quat_mul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)


def quat_R(x, y, z, w):""")
# BASE offset no longer applies: IK is in world
s=s.replace("""        if frame_pos_is_world:
            hand = hand - BASE_IN_WORLD   # IK works in the base frame
""","""        if not frame_pos_is_world:
            hand = hand + BASE_IN_WORLD   # IK (like FK) works in world on this machine
""")
open('arm.py','w').write(s)
EOF
timeout 120 python3 -c "
from arm import *
a=Arm(); cur=a.arm_joints(); print('cur',[round(x,3) for x in cur])
tcp,q=a.tcp_world(); print('tcp',tcp,'q',q)
sol=a.solve_ik(tcp,q); print('sol',[round(x,3) for x in sol]); print('dev',max(abs(x-y) for x,y in zip(sol,cur)))
sol2=a.solve_ik(tcp,Q_DOWN); print('sol down',[round(x,3) for x in sol2]); print('fk', a.fk_hand(sol2))
rclpy.shutdown()"

# openrua op 24
timeout 300 python3 -c "
from arm import *
a=Arm(); cur=a.arm_joints()
targets={'mug_pre':(-0.085,-0.184,0.66),'mug_grasp':(-0.085,-0.184,0.525),'plate_pre':(0.151,-0.023,0.66),'plate_place':(0.151,-0.023,0.575),
'pud_pre':(-0.217,-0.020,0.60),'pud_grasp':(-0.217,-0.020,0.445),'pudplace_pre':(0.15,0.16,0.60),'pudplace':(0.15,0.16,0.455)}
for k,v in targets.items():
    s=a.solve_ik(v,Q_DOWN,seed=cur)
    if s:
        p,q=a.fk_hand(s); tcp=p+TCP*quat_R(*q)[:,2]
        print(k, [round(x,3) for x in s], 'tcp', tcp.round(4))
    else: print(k,'NO IK')
rclpy.shutdown()"

# openrua op 25
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    elif cmd == \\"grip\\":", "new_string": "    elif cmd == \\"line\\":\\n        # straight TCP line from current TCP to target via n IK waypoints\\n        pos = np.array([float(x) for x in sys.argv[2:5]])\\n        n = int(sys.argv[5]) if len(sys.argv) > 5 else 4\\n        sec = float(sys.argv[6]) if len(sys.argv) > 6 else 3.0\\n        start, _ = a.tcp_world()\\n        seed = a.arm_joints(); wps = []\\n        for i in range(1, n + 1):\\n            p = start + (pos - start) * i / n\\n            q = a.solve_ik(p, Q_DOWN, seed=seed)\\n            if q is None:\\n                raise SystemExit(f\\"no IK at waypoint {p}\\")\\n            seed = q\\n            wps.append((q, sec * i / n))\\n        a.move_joints(wps[-1][0], sec, waypoints=wps[:-1])\\n        p, _ = a.tcp_world()\\n        print(f\\"TCP now world=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f}) target={pos}\\")\\n    elif cmd == \\"grip\\":", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 26
timeout 600 python3 arm.py goto -0.085 -0.184 0.66 3

# openrua op 27
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 28
timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:"; timeout 60 python3 -c "
import numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scene import grab, tf_to_T
rclpy.init(); node=rclpy.create_node('m'); tfbuf=Buffer(); TransformListener(tfbuf,node)
cam='robot0_eye_in_hand'
depth=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
fr=f'{cam}_optical_frame'; end=time.time()+10
while time.time()<end and not tfbuf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=tfbuf.lookup_transform('world',fr,rclpy.time.Time()); print('cam world', t.transform.translation)
T=tf_to_T(t)
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
v,u=np.mgrid[0:depth.height,0:depth.width]
W=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D,np.ones_like(D)],-1)@T.T
m=(W[...,2]>0.50)&(W[...,2]<0.60)&(W[...,0]>-0.2)&(W[...,1]<-0.05)
pts=W[m]; print('rim n',m.sum(),'x',pts[:,0].min().round(4),pts[:,0].max().round(4),'y',pts[:,1].min().round(4),pts[:,1].max().round(4),'z',pts[:,2].min().round(4),pts[:,2].max().round(4))
x,y=pts[:,0],pts[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; print('circle', c[0].round(4), c[1].round(4), 'r', np.sqrt(c[2]+c[0]**2+c[1]**2).round(4))
# exclude handle: y<-0.10
mm=m&(W[...,1]<-0.10); pts=W[mm]; x,y=pts[:,0],pts[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; print('circle no handle', c[0].round(4), c[1].round(4), 'r', np.sqrt(c[2]+c[0]**2+c[1]**2).round(4))
# table z seen by this cam
print('table z sample', W[50,320,2], W[100,600,2])
# inside-bottom of mug
mi=(W[...,2]<0.50)&(W[...,2]>0.40)&(((W[...,0]+0.085)**2+(W[...,1]+0.14)**2)<0.03**2); print('inside z', W[mi][:,2].min() if mi.sum() else None, W[mi][:,2].max() if mi.sum() else None)
"

# openrua op 29
timeout 600 python3 arm.py line -0.080 -0.187 0.52 5 4

# openrua op 30
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 31
timeout 300 python3 arm.py grip 0.0

# openrua op 32
timeout 600 python3 arm.py line -0.080 -0.187 0.68 3 3 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 33
timeout 600 python3 arm.py goto 0.151 -0.020 0.68 4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 34
timeout 60 python3 -c "
import numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scene import grab, tf_to_T
rclpy.init(); node=rclpy.create_node('m'); tfbuf=Buffer(); TransformListener(tfbuf,node)
for cam in ['birdview','agentview']:
    depth=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
    fr=f'{cam}_optical_frame'; end=time.time()+10
    while time.time()<end and not tfbuf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
    T=tf_to_T(tfbuf.lookup_transform('world',fr,rclpy.time.Time()))
    D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    v,u=np.mgrid[0:depth.height,0:depth.width]
    W=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D,np.ones_like(D)],-1)@T.T
    # held mug: z between 0.60 and 0.72, x>0.0
    m=(W[...,2]>0.58)&(W[...,2]<0.73)&(W[...,0]>0.0)
    if m.sum():
        pts=W[m]; print(cam,'mug pts',m.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
        # mug bottom (lowest)
        lo=pts[pts[:,2]<pts[:,2].min()+0.01]; print('  bottom-ish x,y mean',lo[:,0].mean().round(3),lo[:,1].mean().round(3))
    # plate
    m=(W[...,2]>0.44)&(W[...,2]<0.47)&(W[...,0]>0.05)&(abs(W[...,1])<0.12)
    pts=W[m]; print(cam,'plate pts',m.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'mean',pts[:,0].mean().round(3),pts[:,1].mean().round(3))
"

# openrua op 35
timeout 600 python3 arm.py line 0.151 -0.020 0.562 4 4 && timeout 300 python3 arm.py grip 0.04

# openrua op 36
timeout 600 python3 arm.py line 0.151 -0.020 0.70 3 3 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 37
timeout 60 python3 -c "
import numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scene import grab, tf_to_T
rclpy.init(); node=rclpy.create_node('m'); tfbuf=Buffer(); TransformListener(tfbuf,node)
for cam in ['birdview','agentview','sideview']:
    depth=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
    fr=f'{cam}_optical_frame'; end=time.time()+10
    while time.time()<end and not tfbuf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
    T=tf_to_T(tfbuf.lookup_transform('world',fr,rclpy.time.Time()))
    D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    v,u=np.mgrid[0:depth.height,0:depth.width]
    W=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D,np.ones_like(D)],-1)@T.T
    m=(W[...,0]>-0.30)&(W[...,0]<-0.13)&(abs(W[...,1])<0.07)&(W[...,2]>0.435)&(W[...,2]<0.50)
    if m.sum():
        pts=W[m]; top=pts[pts[:,2]>pts[:,2].max()-0.008]
        print(cam,'pudding n',m.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'ztop',pts[:,2].max().round(3),'| top face x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3),'mean',top[:,0].mean().round(3),top[:,1].mean().round(3))
    else: print(cam,'none')
    # red mug
    m=(W[...,1]>0.03)&(W[...,0]>-0.2)&(W[...,0]<0.0)&(W[...,2]>0.50)&(W[...,2]<0.60)
    if m.sum(): pts=W[m]; print(cam,'red mug x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3))
"

# openrua op 38
timeout 600 python3 arm.py goto -0.2025 -0.0185 0.62 4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 39
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 arm.py state

# openrua op 40
timeout 600 python3 arm.py goto -0.2025 -0.0185 0.62 5

# openrua op 41
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 -c "
import numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scene import grab, tf_to_T
rclpy.init(); node=rclpy.create_node('m'); tfbuf=Buffer(); TransformListener(tfbuf,node)
cam='robot0_eye_in_hand'
depth=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
fr=f'{cam}_optical_frame'; end=time.time()+10
while time.time()<end and not tfbuf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
T=tf_to_T(tfbuf.lookup_transform('world',fr,rclpy.time.Time()))
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
v,u=np.mgrid[0:depth.height,0:depth.width]
W=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D,np.ones_like(D)],-1)@T.T
m=(W[...,0]>-0.30)&(W[...,0]<-0.13)&(abs(W[...,1])<0.07)&(W[...,2]>0.435)&(W[...,2]<0.50)
pts=W[m]; top=pts[pts[:,2]>pts[:,2].max()-0.006]
print('pudding n',m.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'ztop',pts[:,2].max().round(3))
print('top face x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3),'mean',top[:,0].mean().round(4),top[:,1].mean().round(4), 'n',len(top))
print('table z', W[20,20,2], W[400,600,2])
"

# openrua op 42
timeout 600 python3 arm.py line -0.2026 -0.0178 0.443 5 4 && timeout 300 python3 arm.py grip 0.0

# openrua op 43
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview && timeout 60 python3 arm.py state

# openrua op 44
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; python3 -c "
import cv2; im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[150:300,200:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 45
timeout 300 python3 arm.py grip 0.04 && timeout 600 python3 arm.py line -0.1636 -0.0378 0.60 3 5 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 46
timeout 60 python3 -c "
import numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scene import grab, tf_to_T
rclpy.init(); node=rclpy.create_node('m'); tfbuf=Buffer(); TransformListener(tfbuf,node)
for cam in ['agentview','sideview','robot0_eye_in_hand']:
    depth=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
    fr=f'{cam}_optical_frame'; end=time.time()+10
    while time.time()<end and not tfbuf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
    T=tf_to_T(tfbuf.lookup_transform('world',fr,rclpy.time.Time()))
    D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    v,u=np.mgrid[0:depth.height,0:depth.width]
    W=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D,np.ones_like(D)],-1)@T.T
    m=(W[...,0]>-0.32)&(W[...,0]<-0.10)&(abs(W[...,1])<0.08)&(W[...,2]>0.435)&(W[...,2]<0.52)
    if m.sum():
        pts=W[m]; top=pts[pts[:,2]>pts[:,2].max()-0.006]
        print(cam,'pudding n',m.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'ztop',pts[:,2].max().round(3),'| top mean',top[:,0].mean().round(4),top[:,1].mean().round(4),'n',len(top))
    else: print(cam,'none')
"; python3 -c "
import cv2; im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[180:300,220:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 47
timeout 900 python3 -c "
from arm import *
a=Arm()
cx,cy=-0.202,-0.026
qg=a.solve_ik((cx,cy,0.443),Q_DOWN); print('grasp q',[round(x,3) for x in qg])
qp=a.solve_ik((cx,cy,0.60),Q_DOWN,seed=qg); print('pre q',[round(x,3) for x in qp])
print('branch dev', max(abs(x-y) for x,y in zip(qg,qp)))
a.move_joints(qp,5.0); p,_=a.tcp_world(); print('TCP',p.round(4))
# descend in a straight line, seeded from previous, slow
start,_=a.tcp_world(); seed=qp; wps=[]; n=5; sec=6.0
tgt=np.array([cx,cy,0.443])
for i in range(1,n+1):
    pt=start+(tgt-start)*i/n; q=a.solve_ik(pt,Q_DOWN,seed=seed); seed=q; wps.append((q,sec*i/n))
print('descent branch dev', max(abs(x-y) for x,y in zip(wps[0][0],wps[-1][0])))
code,err=a.move_joints(wps[-1][0],sec,waypoints=wps[:-1]); p,q=a.tcp_world(); print('TCP',p.round(4),'quat',[round(x,3) for x in q])
rclpy.shutdown()"

# openrua op 48
timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import cv2; im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[120:300,180:420],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 49
timeout 300 python3 arm.py grip 0.0

# openrua op 50
timeout 600 python3 arm.py line -0.2025 -0.026 0.60 4 5 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 arm.py state | head -2

# openrua op 51
timeout 300 python3 -c "
from arm import *
a=Arm(); cur=a.arm_joints()
for name,q in [('yaw+90',(0.7071068,0.7071068,0,0)),('yaw-90',(0.7071068,-0.7071068,0,0))]:
    for z in (0.60,0.458):
        s=a.solve_ik((0.15,0.16,z),q,seed=cur)
        if s:
            p,qq=a.fk_hand(s); print(name,z,[round(x,3) for x in s],'hand quat',[round(x,3) for x in qq], 'R col1(fingers)',quat_R(*qq)[:,1].round(2))
        else: print(name,z,'NO IK')
rclpy.shutdown()"

# openrua op 52
timeout 900 python3 -c "
from arm import *
a=Arm()
QY=(0.7071068,0.7071068,0.0,0.0)
tx,ty=0.15,0.16
qg=a.solve_ik((tx,ty,0.458),QY); qp=a.solve_ik((tx,ty,0.62),QY,seed=qg)
print('pre q',[round(x,3) for x in qp],'dev from grasp',max(abs(x-y) for x,y in zip(qg,qp)))
code,err=a.move_joints(qp,6.0); p,q=a.tcp_world(); print('TCP',p.round(4),'quat',[round(x,3) for x in q]); print('fingers',a.fingers())
rclpy.shutdown()"

# openrua op 53
timeout 900 python3 -c "
from arm import *
a=Arm()
QY=(0.7071068,0.7071068,0.0,0.0)
qp=a.solve_ik((0.15,0.16,0.62),QY)
code,err=a.move_joints(qp,5.0); p,q=a.tcp_world(); print('TCP',p.round(4),'quat',[round(x,3) for x in q]); print('fingers',a.fingers())
rclpy.shutdown()" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 54
timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py sideview && timeout 60 python3 -c "
import numpy as np, rclpy, time, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scene import grab, tf_to_T
rclpy.init(); node=rclpy.create_node('m'); tfbuf=Buffer(); TransformListener(tfbuf,node)
for cam in ['agentview','sideview','birdview']:
    depth=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
    fr=f'{cam}_optical_frame'; end=time.time()+10
    while time.time()<end and not tfbuf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
    T=tf_to_T(tfbuf.lookup_transform('world',fr,rclpy.time.Time()))
    D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    v,u=np.mgrid[0:depth.height,0:depth.width]
    W=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D,np.ones_like(D)],-1)@T.T
    # everything above table in x>-0.05 region, z<0.60 (exclude arm high up), y>-0.15
    m=(W[...,2]>0.435)&(W[...,2]<0.60)&(W[...,0]>-0.1)&(W[...,0]<0.4)&(W[...,1]>-0.15)&(W[...,1]<0.35)
    n,lab=cv2.connectedComponents(m.astype(np.uint8))
    for i in range(1,n):
        mm=lab==i
        if mm.sum()<30: continue
        pts=W[mm]; print(cam,f'blob{i} n={mm.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]')
"

# openrua op 55
timeout 900 python3 -c "
from arm import *
a=Arm()
QY=(0.7071068,0.7071068,0.0,0.0)
def line(tgt,n,sec):
    start,_=a.tcp_world(); seed=a.arm_joints(); wps=[]
    for i in range(1,n+1):
        pt=start+(np.array(tgt)-start)*i/n; q=a.solve_ik(pt,QY,seed=seed); seed=q; wps.append((q,sec*i/n))
    a.move_joints(wps[-1][0],sec,waypoints=wps[:-1]); p,q=a.tcp_world(); print('TCP',p.round(4),'quat',[round(x,3) for x in q],'fingers',[round(x,4) for x in a.fingers()])
line((0.205,0.165,0.62),3,4.0)
line((0.205,0.165,0.458),5,6.0)
rclpy.shutdown()"

# openrua op 56
timeout 300 python3 arm.py grip 0.04 && timeout 900 python3 -c "
from arm import *
a=Arm()
QY=(0.7071068,0.7071068,0.0,0.0)
start,_=a.tcp_world(); seed=a.arm_joints(); wps=[]; tgt=np.array([0.205,0.165,0.68]); n=3; sec=4.0
for i in range(1,n+1):
    pt=start+(tgt-start)*i/n; q=a.solve_ik(pt,QY,seed=seed); seed=q; wps.append((q,sec*i/n))
a.move_joints(wps[-1][0],sec,waypoints=wps[:-1]); p,_=a.tcp_world(); print('TCP',p.round(4))
rclpy.shutdown()" && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 57
timeout 900 python3 -c "
from arm import *
a=Arm(); QY=(0.7071068,0.7071068,0.0,0.0)
q=a.solve_ik((0.12,0.20,0.80),QY); a.move_joints(q,4.0); p,_=a.tcp_world(); print('TCP',p.round(4))
rclpy.shutdown()" && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 -c "
import numpy as np, rclpy, time, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scene import grab, tf_to_T
rclpy.init(); node=rclpy.create_node('m'); tfbuf=Buffer(); TransformListener(tfbuf,node)
for cam in ['birdview','agentview']:
    depth=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
    fr=f'{cam}_optical_frame'; end=time.time()+10
    while time.time()<end and not tfbuf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
    T=tf_to_T(tfbuf.lookup_transform('world',fr,rclpy.time.Time()))
    D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    v,u=np.mgrid[0:depth.height,0:depth.width]
    W=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D,np.ones_like(D)],-1)@T.T
    def rep(name,m):
        if m.sum()<10: print(cam,name,'none'); return
        pts=W[m]; print(cam,f'{name}: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})')
    inreg=(W[...,0]>0.0)&(W[...,0]<0.35)&(W[...,1]>-0.15)&(W[...,1]<0.30)
    rep('white mug rim (z 0.55-0.60)', inreg&(W[...,2]>0.55)&(W[...,2]<0.60)&(W[...,1]<0.11))
    rep('plate (z 0.44-0.47, y<0.10)', inreg&(W[...,2]>0.44)&(W[...,2]<0.47)&(W[...,1]<0.10)&(W[...,0]>0.06))
    rep('pudding (z 0.44-0.48, y>0.11)', inreg&(W[...,2]>0.44)&(W[...,2]<0.48)&(W[...,1]>0.11))
    rep('red mug lying', (W[...,0]>-0.1)&(W[...,0]<0.10)&(W[...,1]>0.03)&(W[...,1]<0.16)&(W[...,2]>0.44)&(W[...,2]<0.56))
"

# openrua op 58
timeout 900 python3 -c "
from arm import *
a=Arm()
q=a.solve_ik((-0.15,0.0,0.85),Q_DOWN); print([round(x,3) for x in q]); a.move_joints(q,5.0); p,_=a.tcp_world(); print('TCP',p.round(4))
rclpy.shutdown()" && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 -c "
import numpy as np, rclpy, time, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scene import grab, tf_to_T
rclpy.init(); node=rclpy.create_node('m'); tfbuf=Buffer(); TransformListener(tfbuf,node)
for cam in ['birdview','agentview']:
    depth=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
    fr=f'{cam}_optical_frame'; end=time.time()+10
    while time.time()<end and not tfbuf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
    T=tf_to_T(tfbuf.lookup_transform('world',fr,rclpy.time.Time()))
    D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    v,u=np.mgrid[0:depth.height,0:depth.width]
    W=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D,np.ones_like(D)],-1)@T.T
    def rep(name,m,circle=False):
        if m.sum()<10: print(cam,name,'none'); return
        pts=W[m]; print(cam,f'{name}: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mid=({(pts[:,0].min()+pts[:,0].max())/2:.3f},{(pts[:,1].min()+pts[:,1].max())/2:.3f})')
        if circle:
            x,y=pts[:,0],pts[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2; c=np.linalg.lstsq(A,b,rcond=None)[0]; print('   circle center',c[0].round(3),c[1].round(3),'r',np.sqrt(c[2]+c[0]**2+c[1]**2).round(3))
    rep('white mug rim', (W[...,0]>0.10)&(W[...,0]<0.25)&(W[...,1]>-0.06)&(W[...,1]<0.07)&(W[...,2]>0.54)&(W[...,2]<0.60), True)
    rep('plate', (W[...,0]>0.06)&(W[...,0]<0.25)&(W[...,1]>-0.08)&(W[...,1]<0.10)&(W[...,2]>0.44)&(W[...,2]<0.47))
    rep('pudding', (W[...,0]>0.15)&(W[...,0]<0.26)&(W[...,1]>0.11)&(W[...,1]<0.25)&(W[...,2]>0.44)&(W[...,2]<0.48))
    rep('red mug lying', (W[...,0]>-0.1)&(W[...,0]<0.11)&(W[...,1]>0.02)&(W[...,1]<0.17)&(W[...,2]>0.44)&(W[...,2]<0.57))
"

# openrua op 59
timeout 900 python3 -c "
from arm import *
a=Arm()
q=a.solve_ik((-0.15,0.0,0.85),Q_DOWN); a.move_joints(q,4.0); p,_=a.tcp_world(); print('TCP',p.round(4),'fingers',[round(x,4) for x in a.fingers()])
rclpy.shutdown()"
