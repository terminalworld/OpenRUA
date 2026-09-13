#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 node list

# openrua op 4
cd /workspace; mkdir -p img; for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c img/$c.png & done; wait; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
cd /workspace; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo ----; timeout 10 ros2 topic echo /tf --once 2>&1 | head -40

# openrua op 6
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab depth+color+intrinsics+TF for a camera, save a world-frame point
cloud (npz) and print table-plane stats + object clusters above it.

Usage: python3 cloud.py <camera> [zmin_above_table=0.01]
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
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
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    z = depth
    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
    pc = np.stack([X, Y, z], -1) @ R.T + T
    np.savez(f"img/{cam}_cloud.npz", pc=pc, color=color, depth=depth, K=np.array(info.k).reshape(3, 3), R=R, T=T)
    valid = np.isfinite(z) & (z > 0)
    print("cam pos", T, "K", fx, fy, cx, cy, "shape", depth.shape)
    zs = pc[..., 2][valid]
    hist, edges = np.histogram(zs, bins=60)
    top = edges[np.argmax(hist)]
    print("most common z (table?)", round(top, 3), " z range", zs.min().round(3), zs.max().round(3))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
cd /workspace; timeout 90 python3 cloud.py birdview; timeout 90 python3 cloud.py agentview

# openrua op 8
cd /workspace; python3 -c "
import numpy as np
d=np.load('img/birdview_cloud.npz'); pc=d['pc']; col=d['color']
z=pc[...,2]
valid=np.isfinite(z)&(z>0.43)&(z<0.9)
# table region only : x in [-1.2,1.0], y in [-0.8,0.8]
valid &= (np.abs(pc[...,1])<0.8)&(pc[...,0]>-1.3)&(pc[...,0]<1.0)
import scipy.ndimage as ndi
lab,n=ndi.label(valid)
print('clusters',n)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<5: continue
    p=pc[m]; c=col[m].mean(0)
    vs,us=np.nonzero(m)
    print(i,'n',m.sum(),'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].max()),'ctr px',us.mean().round(),vs.mean().round(),'bgr',c.round())
"


# openrua op 9
cd /workspace; python3 -c "
import numpy as np
d=np.load('img/agentview_cloud.npz'); pc=d['pc']; col=d['color']
z=pc[...,2]
valid=np.isfinite(z)&(z>0.43)&(z<0.75)
valid &= (np.abs(pc[...,1])<0.8)&(pc[...,0]>-0.45)&(pc[...,0]<0.6)
import scipy.ndimage as ndi
lab,n=ndi.label(valid)
print('clusters',n)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<20: continue
    p=pc[m]; c=col[m].mean(0)
    vs,us=np.nonzero(m)
    print(i,'n',m.sum(),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()),'ctr px',us.mean().round(),vs.mean().round(),'bgr',c.round())
"


# openrua op 10
cd /workspace; timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -8; timeout 15 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -8

# openrua op 11
cd /workspace; python3 -c "
import numpy as np
d=np.load('img/agentview_cloud.npz'); pc=d['pc']
def box(xr,yr,zmin):
    m=(pc[...,0]>xr[0])&(pc[...,0]<xr[1])&(pc[...,1]>yr[0])&(pc[...,1]<yr[1])&(pc[...,2]>zmin)&np.isfinite(pc[...,2])
    p=pc[m]
    print('n',m.sum(),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()), 'mean',p.mean(0).round(3))
print('box top'); box((-0.25,-0.05),(0.03,0.13),0.45)
print('ketchup >0.50'); box((-0.3,-0.15),(-0.2,-0.05),0.50)
print('ketchup 0.46-0.50'); m=(pc[...,0]>-0.3)&(pc[...,0]<-0.15)&(pc[...,1]>-0.2)&(pc[...,1]<-0.05)&(pc[...,2]>0.46)&(pc[...,2]<0.50); p=pc[m]; print('x %.3f..%.3f y %.3f..%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()))
print('ketchup 0.43-0.46'); m=(pc[...,0]>-0.3)&(pc[...,0]<-0.15)&(pc[...,1]>-0.2)&(pc[...,1]<-0.05)&(pc[...,2]>0.43)&(pc[...,2]<0.46); p=pc[m]; print('x %.3f..%.3f y %.3f..%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()))
print('basket rim >0.6'); box((-0.2,0.2),(0.1,0.4),0.60)
"


# openrua op 12
cd /workspace; python3 -c "
import numpy as np
d=np.load('img/agentview_cloud.npz'); pc=d['pc']
m=(pc[...,0]>-0.25)&(pc[...,0]<-0.05)&(pc[...,1]>0.03)&(pc[...,1]<0.13)&(pc[...,2]>0.448)&(pc[...,2]<0.47)
p=pc[m]; print('box top n',m.sum(),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()),'mean',p.mean(0).round(3))
# percentiles to drop outliers
print('x pct', np.percentile(p[:,0],[2,50,98]).round(3),'y pct',np.percentile(p[:,1],[2,50,98]).round(3))
"


# openrua op 13
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small controller: chain commands in one process (clients built once).

Usage: python3 ctl.py "cmd args; cmd args; ..."
  tcp X Y Z [secs]      move so the TCP (fingertip point) is at world XYZ,
                        hand pointing straight down, fingers along world Y
  tcpyaw X Y Z YAW [s]  same, with a yaw (rad) about world Z
  grip W                per-finger position (0.04 open, 0.0 closed)
  pose                  print world hand/TCP pose + finger gap
  js                    print arm joint positions
"""
import sys
import time

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
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (from /tf)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.buf = Buffer()
        TransformListener(self.buf, self.node)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)
        assert self.ik.wait_for_service(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            self.spin()
        m = self.js["m"]
        d = dict(zip(m.name, m.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def pose(self):
        end = time.time() + 10
        while time.time() < end and not self.buf.can_transform("world", "panda_hand", rclpy.time.Time()):
            self.spin(0.2)
        t = self.buf.lookup_transform("world", "panda_hand", rclpy.time.Time()).transform
        q = t.rotation
        R = quat_R(q.x, q.y, q.z, q.w)
        hand = np.array([t.translation.x, t.translation.y, t.translation.z])
        tcp = hand + TCP * R[:, 2]
        d = self.joints()
        gap = d.get("panda_finger_joint1", float("nan"))
        print(f"hand {hand.round(4)} tcp {tcp.round(4)} quat ({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f}) finger1 {gap:.4f}")
        return hand, tcp, R

    def solve_ik(self, xyz_world, quat):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        xyz = np.array(xyz_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = JointState()
        d = self.joints()
        for j in ARM:
            seed.name.append(j)
            seed.position.append(d[j])
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print("IK FAILED", None if res is None else res.error_code.val)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        q = [sol[j] for j in ARM]
        delta = np.abs(np.array(q) - np.array(seed.position))
        print("IK ok, max joint delta %.2f rad" % delta.max())
        return q

    def move_q(self, q, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"traj error_code={code} max joint err after={err:.4f}")
        return code

    def tcp(self, x, y, z, yaw=0.0, secs=3.0):
        # hand pointing down: quat (1,0,0,0) then yaw about world Z
        cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
        # q_yaw * q_down ; q_down=(1,0,0,0) (x,y,z,w); q_yaw=(0,0,sy,cy)
        # product (w1,v1)*(w2,v2): w=w1w2 - v1.v2, v = w1 v2 + w2 v1 + v1 x v2
        w1, v1 = cy, np.array([0, 0, sy])
        w2, v2 = 0.0, np.array([1.0, 0, 0])
        w = w1 * w2 - v1 @ v2
        v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
        quat = (v[0], v[1], v[2], w)
        R = quat_R(*quat)
        hand = np.array([x, y, z]) - TCP * R[:, 2]
        q = self.solve_ik(hand, quat)
        if q is None:
            return 99
        code = self.move_q(q, secs)
        self.pose()
        return code

    def gripper(self, w):
        goal = GripperCommand.Goal()
        goal.command.position = float(w)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        d = self.joints()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} pos={r.position:.4f} finger1={d['panda_finger_joint1']:.4f}")


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    c = Ctl()
    for cmd in sys.argv[1].split(";"):
        parts = cmd.split()
        if not parts:
            continue
        op, a = parts[0], [float(x) for x in parts[1:]]
        print(">>", cmd.strip(), flush=True)
        if op == "tcp":
            code = c.tcp(a[0], a[1], a[2], 0.0, a[3] if len(a) > 3 else 3.0)
            if code != 0:
                print("STOPPING chain"); break
        elif op == "tcpyaw":
            code = c.tcp(a[0], a[1], a[2], a[3], a[4] if len(a) > 4 else 3.0)
            if code != 0:
                print("STOPPING chain"); break
        elif op == "grip":
            c.gripper(a[0])
        elif op == "pose":
            c.pose()
        elif op == "js":
            print(np.array(c.arm_q()).round(4))
        else:
            print("unknown", op)
        sys.stdout.flush()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 14
cd /workspace; timeout 120 python3 -u ctl.py "pose; js" 2>&1 | grep -v "^\[" 

# openrua op 15
cd /workspace; timeout 600 python3 -u ctl.py "grip 0.04; tcp -0.21 -0.126 0.62 4" 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 16
cd /workspace; cat > iktest.py <<'EOF'
import sys, numpy as np
sys.argv=[sys.argv[0],""]
from ctl import Ctl, quat_R, TCP
c=Ctl()
hand,tcp,R=c.pose()
tests=[("current hand",hand,(1,0,-0.028,0)),
       ("current hand pure down",hand,(1,0,0,0)),
       ("ketchup hi",np.array([-0.21,-0.126,0.62+TCP]),(1,0,0,0)),
       ("ketchup hi yaw45",np.array([-0.21,-0.126,0.62+TCP]),(0.924,0.383,0,0)),
       ("ketchup hi yaw-45",np.array([-0.21,-0.126,0.62+TCP]),(0.924,-0.383,0,0)),
       ("center hi",np.array([-0.1,0.0,0.62+TCP]),(1,0,0,0)),
]
for name,p,q in tests:
    print(name, p.round(3)); c.solve_ik(p,q)
EOF
timeout 300 python3 -u iktest.py 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 17
cd /workspace; timeout 300 python3 -u tools/action/ik_move.py 0.457 0.0 0.358 1 0 0 0 3 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 18
cd /workspace; timeout 60 ros2 param get /move_group robot_description_kinematics.panda_arm.kinematics_solver 2>&1 | tail -2; timeout 60 ros2 param list /move_group 2>&1 | grep -i -E "kinematics|panda_arm" | head -20; timeout 30 ros2 param get /move_group robot_description_semantic 2>&1 | grep -E "group name|chain|tip|end_effector|link name" | head -20

# openrua op 19
cd /workspace; timeout 30 ros2 param get /move_group robot_description_semantic 2>&1 | grep -o "openrua assembly note.*" | head -5; timeout 30 ros2 param get /move_group robot_description_kinematics.panda_arm.kinematics_solver_timeout; timeout 30 ros2 param get /move_group robot_description_kinematics.panda_arm.position_only_ik; timeout 30 ros2 param get /move_group robot_description_kinematics.panda_arm.joints

# openrua op 20
cd /workspace; timeout 30 ros2 param get /move_group robot_description_semantic 2>&1 | tr '\n' ' ' | grep -o "openrua assembly note.\{0,1500\}"

# openrua op 21
cd /workspace; cat > fktest.py <<'EOF'
import sys, numpy as np, rclpy
sys.argv=[sys.argv[0],""]
from ctl import Ctl, ARM
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
c=Ctl()
cli=c.node.create_client(GetPositionFK,"/compute_fk"); print(cli.wait_for_service(10))
req=GetPositionFK.Request(); req.fk_link_names=["panda_link8","panda_hand"]
d=c.joints(); js=JointState()
for j in ARM: js.name.append(j); js.position.append(d[j])
req.robot_state.joint_state=js
for fid in ["", "world", "panda_link0"]:
    req.header.frame_id=fid
    f=cli.call_async(req); rclpy.spin_until_future_complete(c.node,f,timeout_sec=30)
    r=f.result()
    print("frame",repr(fid),"code",r.error_code.val if r else None)
    if r:
        for n,p in zip(r.fk_link_names,r.pose_stamped):
            print(" ",n,p.header.frame_id,[round(v,3) for v in (p.pose.position.x,p.pose.position.y,p.pose.position.z)],[round(v,3) for v in (p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w)])
EOF
timeout 120 python3 -u fktest.py 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 22
cd /workspace; cat > iktest2.py <<'EOF'
import sys, numpy as np, rclpy
sys.argv=[sys.argv[0],""]
from ctl import Ctl, ARM
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
c=Ctl()
def ik(fid, xyz, quat, link=""):
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.pose_stamped.header.frame_id=fid
    if link: req.ik_request.ik_link_name=link
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=xyz
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=quat
    d=c.joints(); js=JointState()
    for j in ARM: js.name.append(j); js.position.append(d[j])
    req.ik_request.robot_state.joint_state=js
    req.ik_request.timeout.sec=1
    f=c.ik.call_async(req); rclpy.spin_until_future_complete(c.node,f,timeout_sec=60)
    r=f.result()
    code=r.error_code.val if r else None
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position)) if r and code==1 else {}
    print(repr(fid),link,xyz,quat,"->",code,[round(sol.get(j,0),3) for j in ARM] if sol else "")
ik("", (-0.053,0.0,0.778),(0.924,-0.383,-0.026,0.011))
ik("", (-0.053,0.0,0.778),(1.0,0.0,0.0,0.0))
ik("world", (-0.053,0.0,0.778),(0.924,-0.383,-0.026,0.011))
ik("panda_link0", (0.457,0.0,0.358),(0.924,-0.383,-0.026,0.011))
ik("", (-0.053,0.0,0.778),(1.0,0.0,-0.028,0.0),"panda_hand")
ik("", (-0.21,-0.126,0.723),(1.0,0.0,0.0,0.0),"panda_hand")
ik("", (-0.21,-0.126,0.723),(0.924,-0.383,0,0))
EOF
timeout 300 python3 -u iktest2.py 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 23
cd /workspace; python3 - <<'EOF'
s=open('ctl.py').read()
s=s.replace('BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (from /tf)',
 'BASE_IN_WORLD = np.zeros(3)  # verified via /compute_fk: empty frame_id == world on this machine')
s=s.replace('req.ik_request.pose_stamped.header.frame_id = ""\n',
 'req.ik_request.pose_stamped.header.frame_id = ""\n        req.ik_request.ik_link_name = "panda_hand"\n')
open('ctl.py','w').write(s)
EOF
grep -n "ik_link_name\|BASE_IN_WORLD =" ctl.py

# openrua op 24
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.21 -0.126 0.62 4" 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 25
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih1.png; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent1.png

# openrua op 26
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.21 -0.126 0.47 3; grip 0.0" 2>&1 | grep -v "^\[INFO\|^\[WARN"; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent2.png

# openrua op 27
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.21 -0.126 0.47 5" 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 28
cd /workspace; cat > dbg.py <<'EOF'
import sys, numpy as np
sys.argv=[sys.argv[0],""]
from ctl import Ctl, TCP
c=Ctl()
print("joints now", np.array(c.arm_q()).round(3))
q=c.solve_ik((-0.21,-0.126,0.47+TCP),(1,0,0,0)); print("ik sol", np.array(q).round(3) if q else None)
EOF
timeout 120 python3 -u dbg.py 2>&1 | grep -v "^\[INFO\|^\[WARN"; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | head -20

# openrua op 29
cd /workspace; timeout 20 ros2 service list | grep -v parameter

# openrua op 30
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.21 -0.126 0.62 4; js" 2>&1 | grep -v "^\[INFO\|^\[WARN"; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 31
cd /workspace; cat > ikexplore.py <<'EOF'
import sys, numpy as np, rclpy
sys.argv=[sys.argv[0],""]
from ctl import Ctl, ARM, TCP, quat_R
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as Rot
c=Ctl()
def ik(xyz, quat, seed):
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.ik_link_name="panda_hand"
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=map(float,xyz)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quat)
    js=JointState(); js.name=list(ARM); js.position=[float(v) for v in seed]
    req.ik_request.robot_state.joint_state=js
    req.ik_request.timeout.sec=1
    f=c.ik.call_async(req); rclpy.spin_until_future_complete(c.node,f,timeout_sec=60)
    r=f.result()
    if not r or r.error_code.val!=1: return None
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
    return np.array([sol[j] for j in ARM])
tcp=np.array([-0.21,-0.126,0.47])
cur=np.array(c.arm_q())
seeds={"cur":cur,"ready":[0,-0.785,0,-2.356,0,1.571,0.785],"ready_j5pi":[0,-0.785,0,-2.356,np.pi/2,1.571,0.785],
       "ready_j5mpi":[0,-0.785,0,-2.356,-np.pi/2,1.571,0.785],"j1flip":[np.pi/2,-0.785,-np.pi/2,-2.356,0,1.571,0.785],
       "home":[0,-0.161,0,-2.44,0,2.23,0.785],"alt":[0.5,0.3,-0.8,-2.5,0.5,2.6,0.0],"alt2":[-0.5,0.3,0.8,-2.5,-0.5,2.6,1.5]}
for pitch_deg in [0,15,25,35]:
  for yaw_deg in [0,45,-45,90]:
    # rotation: start from down (1,0,0,0) i.e. R=diag(1,-1,-1); pitch about world y (tilt fingers toward -x when negative?)
    Rd=quat_R(1,0,0,0)
    R=Rot.from_euler('z',yaw_deg,degrees=True).as_matrix()@Rot.from_euler('y',pitch_deg,degrees=True).as_matrix()@Rd
    q=Rot.from_matrix(R).as_quat()
    hand=tcp-TCP*R[:,2]
    for name,s in seeds.items():
        sol=ik(hand,q,s)
        if sol is not None:
            print(f"pitch{pitch_deg} yaw{yaw_deg} seed {name}: ", sol.round(2), "zaxis",R[:,2].round(2))
EOF
timeout 600 python3 -u ikexplore.py 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 32
cd /workspace; python3 - <<'EOF'
s=open('ctl.py').read()
old_start=s.index("    def solve_ik(self, xyz_world, quat):")
old_end=s.index("    def move_q(self, q, secs):")
new='''    def solve_ik(self, xyz_world, quat, seed_q=None, verbose=True):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        xyz = np.array(xyz_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = JointState()
        seed.name = list(ARM)
        seed.position = [float(v) for v in (seed_q if seed_q is not None else self.arm_q())]
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            if verbose:
                print("IK FAILED", None if res is None else res.error_code.val)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    HOME = np.array([0, -0.161, 0, -2.44, 0, 2.23, 0.785])

    def best_ik(self, hand, quat):
        """Try several seeds; prefer solutions that stay out of the
        physically self-colliding fold (j4 < -2.8 with j6 > 2.9) and
        close to the current configuration."""
        cur = np.array(self.arm_q())
        seeds = [cur, self.HOME, self.HOME + [0.3, 0, -0.3, 0, 0, 0, 0],
                 self.HOME + [-0.3, 0, 0.3, 0, 0, 0, 0], self.HOME + [0, 0, 0, 0, 0.5, 0, 0]]
        best, best_cost = None, 1e9
        for s in seeds:
            q = self.solve_ik(hand, quat, s, verbose=False)
            if q is None:
                continue
            cost = np.abs(q - cur).sum()
            if q[3] < -2.8:
                cost += 10 * (-2.8 - q[3])
            if q[5] > 2.85:
                cost += 10 * (q[5] - 2.85)
            if abs(q[6]) > 2.5 or abs(q[0]) > 2.0:
                cost += 5
            if cost < best_cost:
                best, best_cost = q, cost
        if best is None:
            print("IK FAILED for all seeds")
        else:
            print("IK ok:", best.round(3), "cost %.2f" % best_cost)
        return best

'''
s=s[:old_start]+new+s[old_end:]
old_start=s.index("    def tcp(self, x, y, z, yaw=0.0, secs=3.0):")
old_end=s.index("    def gripper(self, w):")
new='''    def tcp(self, x, y, z, pitch_deg=0.0, yaw_deg=0.0, secs=3.0):
        """Hand pointing down; pitch tilts the approach axis toward -x
        (wrist moves away from the base), yaw rotates the finger axis
        (0 = fingers close along world Y)."""
        from scipy.spatial.transform import Rotation as Rot
        R = (Rot.from_euler("z", yaw_deg, degrees=True).as_matrix()
             @ Rot.from_euler("y", pitch_deg, degrees=True).as_matrix()
             @ quat_R(1, 0, 0, 0))
        quat = Rot.from_matrix(R).as_quat()
        hand = np.array([x, y, z]) - TCP * R[:, 2]
        q = self.best_ik(hand, quat)
        if q is None:
            return 99
        code = self.move_q(q, secs)
        self.pose()
        return code

'''
s=s[:old_start]+new+s[old_end:]
s=s.replace('''        if op == "tcp":
            code = c.tcp(a[0], a[1], a[2], 0.0, a[3] if len(a) > 3 else 3.0)
            if code != 0:
                print("STOPPING chain"); break
        elif op == "tcpyaw":
            code = c.tcp(a[0], a[1], a[2], a[3], a[4] if len(a) > 4 else 3.0)
            if code != 0:
                print("STOPPING chain"); break
''','''        if op == "tcp":  # tcp X Y Z [pitch_deg] [yaw_deg] [secs]
            code = c.tcp(*a[:3], *(a[3:6] + [0.0, 0.0, 3.0][len(a) - 3:]))
            if code != 0 and not FORCE:
                print("STOPPING chain"); break
''')
s=s.replace("def main():\n    c = Ctl()", "FORCE = \"--force\" in sys.argv  # keep going after a tolerance-violation result\n\n\ndef main():\n    c = Ctl()")
open('ctl.py','w').write(s)
EOF
python3 -c "import ast;ast.parse(open('/workspace/ctl.py').read())" && echo OK

# openrua op 33
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.21 -0.126 0.60 25 0 4" 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 34
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.21 -0.126 0.47 25 0 3" 2>&1 | grep -v "^\[INFO\|^\[WARN"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih2.png; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent3.png

# openrua op 35
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.30 -0.05 0.72 25 0 4" 2>&1 | grep -v "^\[INFO\|^\[WARN"; timeout 90 python3 cloud.py agentview; timeout 90 python3 cloud.py birdview; timeout 60 python3 tools/perception/cam_snap.py robot0_robotview img/robotview2.png

# openrua op 36
cd /workspace; timeout 60 python3 -u ctl.py "pose; js" 2>&1 | grep -v "^\[INFO\|^\[WARN\|XMLPARSER"; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 37
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.30 -0.05 0.72 25 0 6" 2>&1 | grep -v "^\[INFO\|^\[WARN\|XMLPARSER"

# openrua op 38
cd /workspace; timeout 90 python3 cloud.py agentview 2>&1 | grep -v XMLPARSER; python3 -c "
import numpy as np
d=np.load('img/agentview_cloud.npz'); pc=d['pc']; col=d['color']
z=pc[...,2]
valid=np.isfinite(z)&(z>0.43)&(z<0.60)
valid &= (np.abs(pc[...,1])<0.8)&(pc[...,0]>-0.45)&(pc[...,0]<0.6)
import scipy.ndimage as ndi
lab,n=ndi.label(valid)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<20: continue
    p=pc[m]; c=col[m].mean(0)
    vs,us=np.nonzero(m)
    print(i,'n',m.sum(),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()),'ctr px',us.mean().round(),vs.mean().round(),'bgr',c.round())
"; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent4.png

# openrua op 39
cd /workspace; python3 -c "
import numpy as np
d=np.load('img/agentview_cloud.npz'); pc=d['pc']; col=d['color'].astype(float)
m=(pc[...,0]>-0.25)&(pc[...,0]<-0.03)&(pc[...,1]>-0.27)&(pc[...,1]<-0.09)&(pc[...,2]>0.44)&(pc[...,2]<0.50)
p=pc[m]; c=col[m]
sat=c.max(1)-c.min(1)
body=sat>40; cap=~body
print('body n',body.sum(),'cap n',cap.sum())
pb=p[body]; ctr=pb[:,:2].mean(0); 
u,s,vt=np.linalg.svd(pb[:,:2]-ctr); ax=vt[0]
proj=(pb[:,:2]-ctr)@ax
print('body center',ctr.round(3),'axis',ax.round(3),'len along axis',proj.min().round(3),proj.max().round(3),'ztop',pb[:,2].max().round(3))
perp=(pb[:,:2]-ctr)@vt[1]; print('perp extent',perp.min().round(3),perp.max().round(3))
pcap=p[cap]; print('cap center',pcap[:,:2].mean(0).round(3), 'cap z',pcap[:,2].max().round(3))
# thickest part: bin along axis, measure perp extent
for lo in np.arange(proj.min(),proj.max(),0.01):
    sel=(proj>=lo)&(proj<lo+0.01)
    if sel.sum()>3: print('  axis %.3f width %.3f zmax %.3f'%(lo, perp[sel].max()-perp[sel].min(), pb[sel,2].max()))
"


# openrua op 40
cd /workspace; timeout 90 python3 cloud.py birdview 2>&1 | grep -v XMLPARSER; python3 -c "
import numpy as np
for cam in ['birdview','agentview']:
    d=np.load(f'img/{cam}_cloud.npz'); pc=d['pc']
    m=(pc[...,0]>-0.25)&(pc[...,0]<-0.03)&(pc[...,1]>-0.27)&(pc[...,1]<-0.09)&(pc[...,2]>0.465)&(pc[...,2]<0.50)
    p=pc[m]
    print(cam,'ridge n',m.sum(),'center',p[:,:2].mean(0).round(3),'z',p[:,2].mean().round(3))
    m=(pc[...,0]>-0.25)&(pc[...,0]<-0.03)&(pc[...,1]>-0.27)&(pc[...,1]<-0.09)&(pc[...,2]>0.44)&(pc[...,2]<0.50)
    p=pc[m]; ctr=p[:,:2].mean(0); u,s,vt=np.linalg.svd(p[:,:2]-ctr); ax=vt[0]; proj=(p[:,:2]-ctr)@ax; perp=(p[:,:2]-ctr)@vt[1]
    print('   all n',m.sum(),'center',ctr.round(3),'axis',ax.round(3),'proj',proj.min().round(3),proj.max().round(3),'perp',perp.min().round(3),perp.max().round(3))
"


# openrua op 41
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.141 -0.168 0.58 25 -44 5; tcp -0.141 -0.168 0.455 25 -44 3" 2>&1 | grep -v "^\[INFO\|^\[WARN\|XMLPARSER"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih3.png; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent5.png

# openrua op 42
cd /workspace; python3 - <<'EOF'
s=open('ctl.py').read()
old=s[s.index("    def move_q(self, q, secs):"):s.index("    def tcp(self, x, y, z, pitch_deg")]
new='''    def move_q(self, q, secs, retries=2):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        for attempt in range(retries + 1):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"traj error_code={code} max joint err after={err:.4f}")
            if err < 0.02:
                return 0  # converged (a -5 with tiny residual is controller lag)
            if attempt < retries:
                print("  residual too large, resending same goal")
        return code if code != 0 else -99

'''
s=s.replace(old,new)
open('ctl.py','w').write(s)
EOF
python3 -c "import ast;ast.parse(open('/workspace/ctl.py').read())" && echo OK

# openrua op 43
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.141 -0.168 0.455 25 -44 3" 2>&1 | grep -v "^\[INFO\|^\[WARN\|XMLPARSER"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih4.png; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent6.png

# openrua op 44
cd /workspace; timeout 600 python3 -u ctl.py "grip 0.0; pose" 2>&1 | grep -v "^\[INFO\|^\[WARN\|XMLPARSER"

# openrua op 45
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.141 -0.168 0.66 25 -44 4" 2>&1 | grep -v "^\[INFO\|^\[WARN\|XMLPARSER"; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent7.png

# openrua op 46
cd /workspace; timeout 900 python3 -u ctl.py "tcp 0.0 0.255 0.78 25 -44 6" 2>&1 | grep -v "^\[INFO\|^\[WARN\|XMLPARSER"; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent8.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih5.png

# openrua op 47
cd /workspace; timeout 900 python3 -u ctl.py "tcp 0.0 0.255 0.71 25 -44 3; grip 0.04; pose" 2>&1 | grep -v "^\[INFO\|^\[WARN\|XMLPARSER"; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent9.png

# openrua op 48
cd /workspace; timeout 90 python3 cloud.py birdview 2>&1 | grep -v XMLPARSER; python3 -c "
import numpy as np
d=np.load('img/birdview_cloud.npz'); pc=d['pc']
m=(pc[...,0]>-0.25)&(pc[...,0]<-0.05)&(pc[...,1]>0.02)&(pc[...,1]<0.15)&(pc[...,2]>0.43)&(pc[...,2]<0.5)
p=pc[m]; print('box n',m.sum(),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()),'center',p[:,:2].mean(0).round(3))
ctr=p[:,:2].mean(0); u,s,vt=np.linalg.svd(p[:,:2]-ctr); print('axis',vt[0].round(3))
"


# openrua op 49
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.14 0.082 0.58 25 0 6; tcp -0.14 0.082 0.443 25 0 3" 2>&1 | grep -v "^\[INFO\|^\[WARN\|XMLPARSER"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih6.png; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent10.png

# openrua op 50
cd /workspace; timeout 900 python3 -u ctl.py "grip 0.0; tcp -0.14 0.082 0.66 25 0 4" 2>&1 | grep -v "^\[INFO\|^\[WARN\|XMLPARSER"; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent11.png

# openrua op 51
cd /workspace; timeout 900 python3 -u ctl.py "tcp 0.0 0.255 0.76 25 0 6; tcp 0.0 0.255 0.70 25 0 3; grip 0.04; tcp 0.0 0.255 0.80 25 0 3" 2>&1 | grep -v "^\[INFO\|^\[WARN\|XMLPARSER"; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent12.png

# openrua op 52
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih7.png; timeout 60 python3 tools/perception/cam_snap.py birdview img/bird2.png 2>&1 | grep -v XMLPARSER; timeout 90 python3 cloud.py agentview 2>&1 | grep -v XMLPARSER; python3 -c "
import numpy as np
d=np.load('img/agentview_cloud.npz'); pc=d['pc']
m=(pc[...,0]>-0.25)&(pc[...,0]<-0.05)&(pc[...,1]>0.02)&(pc[...,1]<0.15)&(pc[...,2]>0.43)&(pc[...,2]<0.5)
print('points at old box spot:',m.sum())
m=(pc[...,0]>-0.25)&(pc[...,0]<-0.03)&(pc[...,1]>-0.27)&(pc[...,1]<-0.09)&(pc[...,2]>0.43)&(pc[...,2]<0.5)
print('points at old ketchup spot:',m.sum())
"

# openrua op 53
cd /workspace; timeout 900 python3 -u ctl.py "tcp -0.25 -0.05 0.75 25 0 6" 2>&1 | grep -v "^\[INFO\|^\[WARN\|XMLPARSER"; timeout 90 python3 cloud.py birdview 2>&1 | grep -v XMLPARSER; python3 -c "
import numpy as np
d=np.load('img/birdview_cloud.npz'); pc=d['pc']; col=d['color']
# basket interior footprint
m=(pc[...,0]>-0.07)&(pc[...,0]<0.07)&(pc[...,1]>0.18)&(pc[...,1]<0.33)&np.isfinite(pc[...,2])
z=pc[...,2][m]; print('basket interior z: min %.3f max %.3f'%(z.min(),z.max()))
import scipy.ndimage as ndi
sel=m&(pc[...,2]>0.44)&(pc[...,2]<0.60)
lab,n=ndi.label(sel)
for i in range(1,n+1):
    mm=lab==i
    if mm.sum()<10: continue
    p=pc[mm]; c=col[mm].mean(0)
    print(' object in basket: n',mm.sum(),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()),'bgr',c.round())
# whole table: anything above table outside basket and outside robot?
sel=(pc[...,2]>0.43)&(pc[...,2]<0.60)&(np.abs(pc[...,1])<0.7)&(pc[...,0]>-0.35)&(pc[...,0]<0.6)&~((pc[...,1]>0.15)&(pc[...,1]<0.36)&(pc[...,0]>-0.1)&(pc[...,0]<0.1))
lab,n=ndi.label(sel)
for i in range(1,n+1):
    mm=lab==i
    if mm.sum()<10: continue
    p=pc[mm]
    print(' on table elsewhere: n',mm.sum(),'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].max()))
"; timeout 60 python3 tools/perception/cam_snap.py birdview img/bird3.png 2>&1 | grep -v XMLPARSER
