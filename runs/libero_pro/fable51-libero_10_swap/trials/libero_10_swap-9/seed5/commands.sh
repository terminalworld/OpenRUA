#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
find /workspace/tools /workspace/.ros -type f; echo ---; ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | head -40; echo ---; ros2 node list

# openrua op 2
mkdir -p snaps && for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c snaps/$c.png; done; ros2 topic echo /joint_states --once

# openrua op 3
cat > tf_dump.py <<'EOF'
import rclpy, sys
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = rclpy.create_node("tfd")
buf = Buffer(); TransformListener(buf, n)
import time
for _ in range(30): rclpy.spin_once(n, timeout_sec=0.2)
print(buf.all_frames_as_string())
for f in sys.argv[1:]:
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f, "t=", round(tr.x,4), round(tr.y,4), round(tr.z,4), "q=", round(q.x,4), round(q.y,4), round(q.z,4), round(q.w,4))
    except Exception as e:
        print(f, "ERR", e)
EOF
python3 tf_dump.py panda_link0 panda_hand birdview_optical_frame agentview_optical_frame frontview_optical_frame sideview_optical_frame

# openrua op 4
cat > scene.py <<'EOF'
import rclpy, numpy as np, struct, sys
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]

def cam_model(cam):
    rclpy.init(); node=rclpy.create_node("scene")
    buf=Buffer(); TransformListener(buf,node)
    depth=grab(node,f"/{cam}/depth/image_raw",Image)
    color=grab(node,f"/{cam}/color/image_raw",Image)
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    frame=f"{cam}_optical_frame"
    while not buf.can_transform("world",frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
    t=buf.lookup_transform("world",frame,rclpy.time.Time())
    q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    tt=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    D=CvBridge().imgmsg_to_cv2(depth,"passthrough").astype(np.float64)
    C=CvBridge().imgmsg_to_cv2(color,"bgr8")
    K=np.array(info.k).reshape(3,3)
    rclpy.shutdown()
    return D,C,K,R,tt

def to_world(D,K,R,tt):
    h,w=D.shape
    u,v=np.meshgrid(np.arange(w),np.arange(h))
    Z=D
    X=(u-K[0,2])*Z/K[0,0]; Y=(v-K[1,2])*Z/K[1,1]
    P=np.stack([X,Y,Z],-1)@R.T+tt
    return P

if __name__=="__main__":
    cam=sys.argv[1]
    D,C,K,R,tt=cam_model(cam)
    P=to_world(D,K,R,tt)
    np.save(f"snaps/{cam}_P.npy",P); cv2.imwrite(f"snaps/{cam}_c.png",C)
    print("K",K.tolist()); print("depth range",np.nanmin(D),np.nanmax(D))
    for a in sys.argv[2:]:
        u,v=map(int,a.split(","))
        print((u,v),"->",np.round(P[v,u],4), "bgr",C[v,u])
EOF
python3 scene.py birdview 318,300 420,300 240,270 170,210 320,400 320,240 100,300

# openrua op 5
for c in galleryview paperview; do python3 tools/perception/cam_snap.py $c snaps/$c.png; done
cat > extents.py <<'EOF'
import numpy as np
P=np.load("snaps/birdview_P.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
def ext(mask,name):
    print(name, "n=",mask.sum(), "x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]"%(x[mask].min(),x[mask].max(),y[mask].min(),y[mask].max(),z[mask].min(),z[mask].max()))
# microwave body: tall stuff at y<-0.05
ext((z>1.05)&(y<-0.05)&(x<0.3),"microwave top")
ext((z>0.93)&(z<1.05)&(y<-0.35)&(x<0.3),"door?")
ext((z>0.93)&(y<-0.35)&(x<0.3),"door all")
# yellow mug region
ext((z>0.92)&(abs(y)<0.1)&(abs(x)<0.1),"yellow mug")
ext((z>0.92)&(abs(y-0.35)<0.1)&(abs(x)<0.1),"grey mug")
# table
tb=(abs(z-0.9)<0.005)
print("table x[%.3f %.3f] y[%.3f %.3f]"%(x[tb].min(),x[tb].max(),y[tb].min(),y[tb].max()))
# door height profile along y at x~-0.3
m=(z>0.93)&(y<-0.3)&(x<-0.2)&(x>-0.4)
for yy in np.arange(-0.5,-0.28,0.02):
    mm=m&(abs(y-yy)<0.01)
    if mm.sum(): print("y=%.2f x[%.3f %.3f] zmax=%.3f"%(yy,x[mm].min(),x[mm].max(),z[mm].max()))
EOF
python3 extents.py

# openrua op 6
cat > extents2.py <<'EOF'
import numpy as np
P=np.load("snaps/birdview_P.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
def ext(mask,name):
    print(name, "n=",mask.sum(), "x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]"%(x[mask].min(),x[mask].max(),y[mask].min(),y[mask].max(),z[mask].min(),z[mask].max()))
ext((z>1.0)&(z<1.2)&(y<-0.05)&(y>-0.36)&(x<0.2)&(x>-0.4),"microwave body top")
ext((z>1.0)&(z<1.2)&(y<-0.05)&(y>-0.36)&(x<0.2)&(x>-0.4)&(z>1.09),"microwave top face z>1.09")
ext((z>0.95)&(z<1.0)&(abs(y)<0.1)&(abs(x)<0.1),"yellow mug rim")
ext((z>0.95)&(z<1.0)&(abs(y-0.35)<0.1)&(abs(x)<0.1),"grey mug rim")
# mug body pixels at z between 0.92 and 1.0
for name,yc in [("yellow",0.0),("grey",0.35)]:
    m=(z>0.91)&(z<1.0)&(abs(y-yc)<0.1)&(abs(x)<0.1)
    print(name,"centroid",x[m].mean(),y[m].mean(),"zmax",z[m].max())
# profile microwave along x at y=-0.2
m=(z>1.0)&(y<-0.05)&(y>-0.36)&(x<0.2)&(x>-0.4)
for xx in np.arange(-0.32,0.12,0.02):
    mm=m&(abs(x-xx)<0.01)
    if mm.sum(): print("x=%.2f y[%.3f %.3f] z[%.3f %.3f]"%(xx,y[mm].min(),y[mm].max(),z[mm].min(),z[mm].max()))
EOF
python3 extents2.py

# openrua op 7
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper around IK / FJT / gripper for this Panda (machine.yaml facts).

import arm; a = arm.Arm()
a.js()                      -> dict joint->pos
a.fk()                      -> (xyz, quat) of panda_hand in world
a.ik(xyz, quat, at_tcp=True, seed=None) -> list of 7 joint positions or None
a.move_joints([q1, q2,...], seconds_per_seg) -> error_code
a.move_poses([(xyz,quat),...], seconds_per_seg, at_tcp=True) -> error_code
a.grip(width_per_finger)    -> (reached, stalled)
"""
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parent
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
PLAN = M["planning"]
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (from TF)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    # returns (x, y, z, w)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def quat_from_axes(xaxis, yaxis, zaxis):
    R = np.stack([np.asarray(xaxis, float), np.asarray(yaxis, float),
                  np.asarray(zaxis, float)], axis=1)
    return R_to_quat(R)


class Arm:
    def __init__(self, name="arm_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, PLAN["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.gr.wait_for_server(10)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def js(self, fresh=True):
        if fresh:
            self._js = {}
        while not self._js:
            self.spin(0.2)
        return dict(self._js)

    def arm_q(self):
        j = self.js()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.js()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def _seed(self, q):
        s = JointState()
        s.name = list(JOINTS)
        s.position = [float(v) for v in q]
        return s

    def fk(self, q=None):
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z,
                         p.orientation.w])
        return xyz, quat

    def ik(self, xyz, quat, at_tcp=True, seed=None, timeout=20.0):
        xyz = np.asarray(xyz, float)
        quat = np.asarray(quat, float)
        if at_tcp:
            xyz = xyz - TCP * quat_to_R(quat)[:, 2]
        xyz = xyz - BASE  # planner frame is the arm base
        req = GetPositionIK.Request()
        req.ik_request.group_name = PLAN["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        (p.orientation.x, p.orientation.y, p.orientation.z,
         p.orientation.w) = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(
            seed if seed is not None else self.arm_q())
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            print("IK: no answer", file=sys.stderr)
            return None
        if res.error_code.val != 1:
            print(f"IK failed code={res.error_code.val}", file=sys.stderr)
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        q = [sol[j] for j in JOINTS]
        for i, (v, (lo, hi)) in enumerate(zip(q, LIMITS)):
            if v < lo or v > hi:
                print(f"IK: joint{i+1}={v:.3f} outside [{lo},{hi}]",
                      file=sys.stderr)
        return q

    def move_joints(self, qs, seconds_per_seg=3.0, first_seconds=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        t = 0.0
        for i, q in enumerate(qs):
            dt = first_seconds if (i == 0 and first_seconds) else seconds_per_seg
            t += dt
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        if not gh.accepted:
            print("FJT goal rejected", file=sys.stderr)
            return -100
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        code = rf.result().result.error_code
        q_now = np.array(self.arm_q())
        err = np.abs(q_now - np.array(qs[-1])).max()
        print(f"FJT done code={code} max_joint_err={err:.4f}")
        return code

    def move_poses(self, poses, seconds_per_seg=3.0, at_tcp=True,
                   first_seconds=None):
        qs = []
        seed = self.arm_q()
        for xyz, quat in poses:
            q = self.ik(xyz, quat, at_tcp=at_tcp, seed=seed)
            if q is None:
                print(f"IK failed for {np.round(xyz,3)}; no motion",
                      file=sys.stderr)
                return None
            qs.append(q)
            seed = q
        return self.move_joints(qs, seconds_per_seg, first_seconds)

    def grip(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"grip reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return r.reached_goal, r.stalled


# handy orientations (x,y,z,w), hand Z = approach, hand Y = finger axis
DOWN = np.array([1.0, 0.0, 0.0, 0.0])            # Z down, fingers along world y
DOWN_FX = quat_from_axes([0, -1, 0], [1, 0, 0], [0, 0, -1])  # Z down, fingers along x
FWD_Y = quat_from_axes([0, 0, 1], [1, 0, 0], [0, 1, 0])      # Z -> +y, fingers along x, camera up
OPENRUA_EOF

# openrua op 8
cat > s1.py <<'EOF'
import arm, numpy as np
a = arm.Arm()
print("q", np.round(a.arm_q(),3))
print("fk", a.fk())
print("fingers", a.fingers())
a.grip(arm.GRIP["open_m"])
print("fingers after open", a.fingers())
# test IK for a viewing pose in front of the microwave opening
for xyz in [(-0.17,-0.50,1.0),(-0.17,-0.46,1.0),(-0.17,-0.45,1.05)]:
    q=a.ik(xyz, arm.FWD_Y, at_tcp=True)
    print(xyz, None if q is None else np.round(q,3))
    if q is not None: print("  fk check", a.fk(q))
EOF
python3 s1.py 2>&1 | grep -v "^\[" 

# openrua op 9
sed -i 's/^BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.array([0.0, 0.0, 0.0])  # FK\/IK poses come back in world coords (verified vs TF)/' arm.py && grep -n "^BASE" arm.py
cat > s2.py <<'EOF'
import arm, numpy as np
a = arm.Arm()
xyz,quat=a.fk(); print("fk", np.round(xyz,4), np.round(quat,4))
q=a.ik(xyz,quat,at_tcp=False); print("ik of current pose", None if q is None else np.round(q,3), "current", np.round(a.arm_q(),3))
for xyz in [(-0.17,-0.50,1.0),(-0.17,-0.46,1.0),(-0.17,-0.45,1.05)]:
    q=a.ik(xyz, arm.FWD_Y, at_tcp=True)
    print(xyz, None if q is None else np.round(q,3))
    if q is not None: print("  fk check", np.round(a.fk(q)[0],4), np.round(a.fk(q)[1],3))
EOF
python3 s2.py 2>&1 | grep -v "^\[" 

# openrua op 10
cat > s3.py <<'EOF'
import arm, numpy as np
a = arm.Arm()
tests = [
 ("down above mug", (0.01,0.0,1.10), arm.DOWN),
 ("down above mug fx", (0.01,0.0,1.10), arm.DOWN_FX),
 ("fwd_y easy", (-0.2,-0.2,1.1), arm.FWD_Y),
 ("fwd_y -0.40", (-0.17,-0.40,1.0), arm.FWD_Y),
 ("fwd_y -0.45", (-0.17,-0.45,1.0), arm.FWD_Y),
 ("fwd_y -0.45 z1.1", (-0.17,-0.45,1.1), arm.FWD_Y),
 ("fwd_y -0.45 x-0.25", (-0.25,-0.45,1.0), arm.FWD_Y),
 ("fwd_y at mug", (0.01,-0.10,0.955), arm.FWD_Y),
 ("fwd_y at mug flipped", (0.01,-0.10,0.955), arm.quat_from_axes([0,0,-1],[-1,0,0],[0,1,0])),
 ("fwd_y -0.45 flipped", (-0.17,-0.45,1.0), arm.quat_from_axes([0,0,-1],[-1,0,0],[0,1,0])),
]
for name,xyz,q in tests:
    for trial in range(2):
        sol=a.ik(xyz,q,at_tcp=True)
        if sol is not None: break
    print(f"{name:28s} {xyz} ->", None if sol is None else np.round(sol,3))
EOF
python3 s3.py 2>&1 | grep -v "^\[" 

# openrua op 11
cat > s4.py <<'EOF'
import arm, numpy as np
a = arm.Arm()
def tilt(deg, fx=True):
    s,c=np.sin(np.radians(deg)),np.cos(np.radians(deg))
    Z=np.array([0,c,-s]); Y=np.array([1.0,0,0]) if fx else np.array([-1.0,0,0])
    X=np.cross(Y,Z)
    return arm.quat_from_axes(X,Y,Z)
tests=[]
for deg in (30,45,60,75):
    for xyz in [(-0.17,-0.45,1.0),(-0.17,-0.45,0.93),(-0.17,-0.31,0.93),(-0.17,-0.5,1.05)]:
        tests.append((f"tilt{deg}",xyz,tilt(deg)))
for name,xyz,q in tests:
    sol=a.ik(xyz,q,at_tcp=True)
    if sol is None: sol=a.ik(xyz,q,at_tcp=True)
    print(f"{name:8s} {xyz} ->", None if sol is None else np.round(sol,3))
EOF
python3 s4.py 2>&1 | grep -v "^\[" | grep -v "IK failed"

# openrua op 12
cat >> arm.py <<'EOF'


def tilt_y(deg, fingers_x=True):
    """Hand Z pointing +y and rotated `deg` downward; fingers along world x."""
    s, c = np.sin(np.radians(deg)), np.cos(np.radians(deg))
    Z = np.array([0.0, c, -s])
    Y = np.array([1.0, 0, 0]) if fingers_x else np.array([-1.0, 0, 0])
    X = np.cross(Y, Z)
    return quat_from_axes(X, Y, Z)


def chain_ik(a, poses, at_tcp=True, seed=None, max_jump=1.6):
    """IK for a list of (xyz, quat), each seeded with the previous solution.
    Returns list of q or None; prints per-segment max joint jump."""
    qs = []
    seed = a.arm_q() if seed is None else list(seed)
    for i, (xyz, quat) in enumerate(poses):
        q = None
        for _ in range(3):
            q = a.ik(xyz, quat, at_tcp=at_tcp, seed=seed)
            if q is not None and np.abs(np.array(q) - np.array(seed)).max() <= max_jump:
                break
        if q is None:
            print(f"chain_ik: no IK for wp{i} {np.round(xyz,3)}")
            return None
        jump = np.abs(np.array(q) - np.array(seed)).max()
        print(f"wp{i} {np.round(xyz,3)} jump={jump:.2f} q={np.round(q,3)}")
        if jump > max_jump:
            print("chain_ik: jump too large, aborting")
            return None
        qs.append(q)
        seed = q
    return qs
EOF
cat > s5.py <<'EOF'
import arm, numpy as np
a = arm.Arm()
poses=[((-0.17,-0.22,1.32), arm.DOWN_FX),
       ((-0.17,-0.45,1.18), arm.tilt_y(45)),
       ((-0.17,-0.45,1.02), arm.tilt_y(45))]
qs=arm.chain_ik(a,poses)
if qs:
    code=a.move_joints(qs, seconds_per_seg=3.0)
    print("fk now", np.round(a.fk()[0],3))
EOF
python3 s5.py 2>&1 | grep -v "^\[" | grep -v "IK failed"

# openrua op 13
python3 - <<'EOF'
import re
s=open("arm.py").read()
s=s.replace('DOWN_FX = quat_from_axes([0, -1, 0], [1, 0, 0], [0, 0, -1])  # Z down, fingers along x',
            'DOWN_FX = quat_from_axes([0, 1, 0], [1, 0, 0], [0, 0, -1])   # Z down, fingers along x')
s+='''

# --- crude collision check: sample joint-space segments, FK several links ---
LINKS = ["panda_link4", "panda_link5", "panda_link6", "panda_link7", "panda_hand"]
# boxes as (name, xmin,xmax, ymin,ymax, zmin,zmax) already inflated for link radius
OBST = [
    ("microwave", -0.36, 0.15, -0.43, -0.08, 0.0, 1.17),
    ("door", -0.38, -0.22, -0.68, -0.36, 0.0, 1.17),
    ("table", -0.80, 0.56, -0.66, 0.66, 0.0, 0.96),
    ("grey_mug", -0.09, 0.11, 0.24, 0.44, 0.0, 1.06),
]
YELLOW_MUG = ("yellow_mug", -0.09, 0.11, -0.09, 0.11, 0.0, 1.06)


def fk_links(a, q, links=LINKS):
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = list(links)
    req.robot_state.joint_state = a._seed(q)
    fut = a.fk_cli.call_async(req)
    rclpy.spin_until_future_complete(a.node, fut, timeout_sec=30)
    res = fut.result()
    out = {}
    for name, ps in zip(res.fk_link_names, res.pose_stamped):
        p = ps.pose
        xyz = np.array([p.position.x, p.position.y, p.position.z])
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        out[name] = (xyz, quat)
    if "panda_hand" in out:
        h, hq = out["panda_hand"]
        out["tcp"] = (h + TCP * quat_to_R(hq)[:, 2], hq)
        out["fingers_mid"] = (h + 0.07 * quat_to_R(hq)[:, 2], hq)
    return out


def in_box(p, b):
    return b[1] <= p[0] <= b[2] and b[3] <= p[1] <= b[4] and b[5] <= p[2] <= b[6]


def check_path(a, q_start, qs, n=8, obst=None, allow=()):
    """Sample linear joint interpolation; report link points inside obstacle
    boxes. `allow` = set of (link, obstacle) pairs to ignore (e.g. tcp near
    table when grasping). Returns list of violations."""
    obst = OBST if obst is None else obst
    viol = []
    prev = np.array(q_start, float)
    for si, q in enumerate(qs):
        q = np.array(q, float)
        for k in range(1, n + 1):
            qi = prev + (q - prev) * k / n
            L = fk_links(a, qi)
            for name, (p, _) in L.items():
                for b in obst:
                    if (name, b[0]) in allow:
                        continue
                    # links above the flange only checked against tall obstacles
                    if in_box(p, b):
                        viol.append((si, k, name, b[0], np.round(p, 3)))
        prev = q
    return viol
'''
open("arm.py","w").write(s)
EOF
cat > s5.py <<'EOF'
import arm, numpy as np
a = arm.Arm()
print("fk now", np.round(a.fk()[0],3), np.round(a.fk()[1],3))
poses=[((-0.17,-0.20,1.32), arm.tilt_y(90,False)),
       ((-0.17,-0.45,1.22), arm.tilt_y(60,False)),
       ((-0.17,-0.45,1.02), arm.tilt_y(45,False))]
qs=arm.chain_ik(a,poses,max_jump=2.5)
if qs:
    v=arm.check_path(a,a.arm_q(),qs,obst=arm.OBST+[arm.YELLOW_MUG])
    print("violations:",len(v)); [print(x) for x in v[:20]]
EOF
python3 s5.py 2>&1 | grep -v "^\[" | grep -v "IK failed"

# openrua op 14
cat > s6.py <<'EOF'
import arm, numpy as np
a = arm.Arm()
poses=[((-0.17,-0.20,1.32), arm.tilt_y(90,False)),
       ((-0.17,-0.45,1.22), arm.tilt_y(60,False)),
       ((-0.17,-0.45,1.02), arm.tilt_y(45,False))]
qs=arm.chain_ik(a,poses,max_jump=2.5)
code=a.move_joints(qs, seconds_per_seg=3.0)
xyz,q=a.fk(); print("hand", np.round(xyz,3), "tcp", np.round(xyz+arm.TCP*arm.quat_to_R(q)[:,2],3))
EOF
python3 s6.py 2>&1 | grep -v "^\[" | grep -v "IK failed" && python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png && python3 tools/perception/cam_snap.py agentview snaps/agent1.png

# openrua op 15
cat > s7.py <<'EOF'
import arm, numpy as np, sys
a = arm.Arm()
target=((-0.17,-0.45,1.02), arm.tilt_y(45,False))
q=a.ik(*target, at_tcp=True)
print("jump", np.abs(np.array(q)-np.array(a.arm_q())).max())
a.move_joints([q], seconds_per_seg=2.0)
xyz,qq=a.fk(); print("hand", np.round(xyz,3), "tcp", np.round(xyz+arm.TCP*arm.quat_to_R(qq)[:,2],3))
EOF
python3 s7.py 2>&1 | grep -v "^\[" | grep -v "IK failed"; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih2.png && python3 -c "
import cv2; im=cv2.imread('snaps/eih2.png'); cv2.imwrite('snaps/eih2r.png', cv2.rotate(im, cv2.ROTATE_180))"

# openrua op 16
python3 tf_dump.py panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -E "^(panda_hand|robot0)" ; python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from arm import quat_to_R
# hand
print("hand R cols (X,Y,Z):")
EOF

# openrua op 17
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from arm import quat_to_R
Rh=quat_to_R([0.8533,-0.3532,-0.1466,-0.3543]); Rc=quat_to_R([-0.3539,0.8531,0.3541,0.147])
np.set_printoptions(precision=3,suppress=True)
print("hand X,Y,Z:\n",Rh.T); print("cam X,Y,Z (optical: X right, Y down, Z fwd):\n",Rc.T)
# camera in hand frame
th=np.array([-0.1702,-0.523,1.0933]); tc=np.array([-0.1348,-0.5483,1.0687])
print("cam offset in hand frame", Rh.T@(tc-th))
print("cam axes in hand frame:\n", Rh.T@Rc)
EOF

# openrua op 18
python3 - <<'EOF'
s=open("arm.py").read()
old="""        if at_tcp:
            xyz = xyz - TCP * quat_to_R(quat)[:, 2]
        xyz = xyz - BASE  # planner frame is the arm base
"""
new="""        if at_tcp:
            xyz = xyz - TCP * quat_to_R(quat)[:, 2]
        xyz = xyz - BASE  # planner frame is the arm base
        # the IK tip is panda_link8; panda_hand = link8 rotated -45deg about Z
        # (verified vs TF), so request R_link8 = R_hand * Rz(+45deg)
        quat = R_to_quat(quat_to_R(quat) @ RZ45)
"""
assert old in s
s=s.replace(old,new)
s=s.replace("BASE = np.array([0.0, 0.0, 0.0])","RZ45 = np.array([[np.cos(np.pi/4), -np.sin(np.pi/4), 0], [np.sin(np.pi/4), np.cos(np.pi/4), 0], [0, 0, 1]])\nBASE = np.array([0.0, 0.0, 0.0])")
open("arm.py","w").write(s)
EOF
cat > s8.py <<'EOF'
import arm, numpy as np
a = arm.Arm()
xyz,q=a.fk()
sol=a.ik(xyz,q,at_tcp=False); print("ik(current) vs current:", np.round(sol,3), np.round(a.arm_q(),3))
target=((-0.17,-0.45,1.02), arm.tilt_y(45,False))
q=a.ik(*target, at_tcp=True); print("jump", np.abs(np.array(q)-np.array(a.arm_q())).max())
a.move_joints([q], seconds_per_seg=2.0)
xyz,qq=a.fk(); R=arm.quat_to_R(qq); print("hand", np.round(xyz,3), "tcp", np.round(xyz+arm.TCP*R[:,2],3)); print("hand axes\n", np.round(R.T,3))
EOF
python3 s8.py 2>&1 | grep -v "^\[" | grep -v "IK failed"; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih3.png && python3 -c "
import cv2; im=cv2.imread('snaps/eih3.png'); cv2.imwrite('snaps/eih3r.png', cv2.rotate(im, cv2.ROTATE_180))"

# openrua op 19
cat > s9.py <<'EOF'
import arm, numpy as np
a = arm.Arm()
target=((-0.17,-0.45,1.02), arm.tilt_y(45,False))
q=a.ik(*target, at_tcp=True)
for i in range(3):
    print("jump", np.abs(np.array(q)-np.array(a.arm_q())).max())
    code=a.move_joints([q], seconds_per_seg=2.0)
    if code==0: break
xyz,qq=a.fk(); R=arm.quat_to_R(qq); print("hand", np.round(xyz,3), "tcp", np.round(xyz+arm.TCP*R[:,2],3)); print("hand axes\n", np.round(R.T,3))
EOF
python3 s9.py 2>&1 | grep -v "^\[" | grep -v "IK failed"; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih3.png && python3 -c "
import cv2; im=cv2.imread('snaps/eih3.png'); cv2.imwrite('snaps/eih3r.png', cv2.rotate(im, cv2.ROTATE_180))"

# openrua op 20
cat > cav.py <<'EOF'
import numpy as np, sys
sys.argv=["scene.py","robot0_eye_in_hand"]
import scene
D,C,K,R,tt=scene.cam_model("robot0_eye_in_hand")
P=scene.to_world(D,K,R,tt)
np.save("snaps/eih_P.npy",P)
x,y,z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(z)&(D>0.05)
print("depth range", D[ok].min(), D[ok].max())
# points inside cavity footprint
m=ok&(x>-0.27)&(x<-0.07)&(y>-0.35)&(y<-0.16)
print("cavity pts", m.sum())
if m.sum():
    zs=z[m]
    print("z percentiles", np.percentile(zs,[0,5,25,50,75,95,100]).round(3))
    for lo in np.arange(0.88,1.12,0.02):
        mm=m&(z>=lo)&(z<lo+0.02)
        if mm.sum(): print(f"z[{lo:.2f},{lo+0.02:.2f}) n={mm.sum()} y[{y[mm].min():.3f},{y[mm].max():.3f}] x[{x[mm].min():.3f},{x[mm].max():.3f}]")
# opening frame: points near the face plane y in [-0.37,-0.35]
f=ok&(y>-0.38)&(y<-0.35)&(x>-0.32)&(x<0.1)
print("face pts", f.sum(), "z range", z[f].min().round(3), z[f].max().round(3))
for xx in np.arange(-0.30,0.10,0.02):
    mm=f&(abs(x-xx)<0.01)
    if mm.sum(): print(f"face x={xx:.2f} z[{z[mm].min():.3f},{z[mm].max():.3f}] n={mm.sum()}")
EOF
python3 cav.py 2>&1 | grep -v "^\["

# openrua op 21
cat > cav2.py <<'EOF'
import numpy as np
P=np.load("snaps/eih_P.npy"); x,y,z=P[...,0],P[...,1],P[...,2]
# find pixels with x close to -0.17 and print (y,z) profile sorted by y
m=(abs(x+0.17)<0.004)&np.isfinite(z)
ys,zs=y[m],z[m]
o=np.argsort(ys)
ys,zs=ys[o],zs[o]
# bin by y
for lo in np.arange(-0.60,-0.15,0.01):
    mm=(ys>=lo)&(ys<lo+0.01)
    if mm.sum(): print(f"y[{lo:.2f}] z min {zs[mm].min():.3f} max {zs[mm].max():.3f} n={mm.sum()}")
EOF
python3 cav2.py

# openrua op 22
python3 - <<'EOF'
import numpy as np, cv2
P=np.load("snaps/eih_P.npy"); z=P[...,2]; y=P[...,1]
zz=np.nan_to_num(z,nan=0.0)
img=((np.clip(zz,0.88,1.12)-0.88)/0.24*255).astype(np.uint8)
img=cv2.applyColorMap(img,cv2.COLORMAP_JET)
img[~np.isfinite(z)]=0
img=cv2.rotate(img,cv2.ROTATE_180)
cv2.imwrite("snaps/eih_z.png",img)
# also the color image rotated with contour of z==0.943 region
EOF
echo done

# openrua op 23
cat > urdf.py <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node("u")
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,"/robot_description",lambda m: got.append(m.data),qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open("snaps/robot.urdf","w").write(got[0]); print(len(got[0]))
EOF
python3 urdf.py 2>&1 | grep -v "^\[" ; grep -n -A12 '<link name="panda_hand"' snaps/robot.urdf | head -60; grep -n -A12 '<link name="panda_leftfinger"' snaps/robot.urdf | head -40; grep -n -B2 -A6 'joint name="panda_hand_joint"\|joint name="panda_finger_joint1"\|joint name="panda_hand_tcp' snaps/robot.urdf | head -60

# openrua op 24
python3 - <<'EOF'
import re
s=open("snaps/robot.urdf").read()
for m in re.finditer(r'<joint name="(panda_hand_joint|panda_finger_joint1|panda_finger_joint2|panda_joint8|panda_hand_tcp_joint)"[^>]*>(.*?)</joint>', s, re.S):
    print(m.group(1), re.sub(r'\s+',' ',m.group(2))[:400]); print()
print([l for l in re.findall(r'<link name="([^"]+)"',s)])
EOF
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ 2>/dev/null

# openrua op 25
python3 - <<'EOF'
import numpy as np
P=np.load("snaps/birdview_P.npy"); x,y,z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(z)&(abs(x+0.2)<0.15)&(abs(y)<0.2)
for lo in np.arange(1.15,1.45,0.02):
    mm=m&(z>=lo)&(z<lo+0.02)
    if mm.sum(): print(f"z[{lo:.2f}] n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
EOF

# openrua op 26
python3 - <<'EOF'
import numpy as np
P=np.load("snaps/eih_P.npy"); x,y,z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(z)
# floor plate front edge region
m=ok&(abs(z-0.943)<0.004)&(y>-0.36)&(y<-0.33)
print("lip x range", x[m].min().round(3), x[m].max().round(3))
m=ok&(abs(z-0.943)<0.004)
print("floor x range", x[m].min().round(3), x[m].max().round(3), "y range", y[m].min().round(3), y[m].max().round(3))
# vertical surfaces near the face plane: for x bins, max z with y in [-0.37,-0.34]
f=ok&(y>-0.37)&(y<-0.33)
for xx in np.arange(-0.32,0.10,0.01):
    mm=f&(abs(x-xx)<0.005)
    if mm.sum(): print(f"x={xx:.2f} zmax={z[mm].max():.3f} n={mm.sum()}")
# interior side walls: points with z in [0.95,1.05] inside y>-0.34
w=ok&(z>0.95)&(z<1.06)&(y>-0.34)&(y<-0.16)
print("interior wall pts", w.sum())
for xx in np.arange(-0.32,0.0,0.01):
    mm=w&(abs(x-xx)<0.005)
    if mm.sum(): print(f"  wall x={xx:.2f} n={mm.sum()} y[{y[mm].min():.3f},{y[mm].max():.3f}] z[{z[mm].min():.3f},{z[mm].max():.3f}]")
EOF

# openrua op 27
cat > s10.py <<'EOF'
import arm, numpy as np, itertools
a = arm.Arm()
seed=a.arm_q()
res=[]
for th in (0,10,20):
  for fx in (True,False):
    for x in (-0.22,-0.18,-0.14):
      for y in (-0.42,-0.46,-0.50):
        for z in (1.0,1.05):
            q=a.ik((x,y,z), arm.tilt_y(th,fx), at_tcp=True, seed=seed, timeout=5)
            ok = q is not None
            print(f"th={th:2d} fx={int(fx)} tcp=({x:.2f},{y:.2f},{z:.2f}) -> {'OK '+str(np.round(q,2)) if ok else '--'}", flush=True)
EOF
python3 s10.py 2>&1 | grep -v "^\[" | grep -v "IK failed"

# openrua op 28
cat > s11.py <<'EOF'
import arm, numpy as np
a = arm.Arm()
MUG=(0.009,0.007)
def T(th,fx): return arm.tilt_y(th,fx)
plan=[
 ("above mug topdown fx", (MUG[0],MUG[1],1.12), T(90,True)),
 ("grasp mug topdown fx", (MUG[0],MUG[1],0.948), T(90,True)),
 ("above stage yaw180",  (-0.15,-0.41,1.12), T(90,False)),
 ("place stage yaw180",  (-0.15,-0.41,0.952), T(90,False)),
 ("tilt45 above stage",  (-0.15,-0.41,1.05), T(45,True)),
 ("tilt45 grasp stage",  (-0.15,-0.41,0.956), T(45,True)),
 ("tilt45 lift",         (-0.15,-0.41,1.01), T(45,True)),
 ("tilt45 enter",        (-0.15,-0.36,1.01), T(45,True)),
 ("tilt30 enter",        (-0.15,-0.36,1.01), T(30,True)),
 ("tilt20 enter",        (-0.15,-0.36,1.01), T(20,True)),
 ("tilt15 enter",        (-0.15,-0.36,1.01), T(15,True)),
 ("tilt20 deep",         (-0.15,-0.332,1.0), T(20,True)),
 ("tilt15 deep",         (-0.15,-0.332,1.0), T(15,True)),
 ("tilt15 deep low",     (-0.15,-0.332,0.985), T(15,True)),
]
for fx in (True,False):
  print("=== fingers_x", fx)
  seed=a.arm_q()
  for name,xyz,q in plan:
    q=arm.quat_from_axes(*[arm.quat_to_R(q)[:,i] for i in range(3)]) if fx else q
    if not fx:
        # flip finger direction: rotate 180 about hand Z
        R=arm.quat_to_R(q); R=np.stack([-R[:,0],-R[:,1],R[:,2]],1); q=arm.R_to_quat(R)
    sol=None
    for _ in range(2):
        sol=a.ik(xyz,q,at_tcp=True,seed=seed,timeout=5)
        if sol is not None: break
    print(f"{name:24s} {xyz} -> {'OK '+str(np.round(sol,2)) if sol is not None else '--'}", flush=True)
    if sol is not None: seed=sol
EOF
python3 s11.py 2>&1 | grep -v "^\[" | grep -v "IK failed"

# openrua op 29
cat > s12.py <<'EOF'
import arm, numpy as np, sys
a = arm.Arm()
MUG=(0.009,0.007)
T=arm.tilt_y
def go(name,xyz,quat,secs=4,obst=None,allow=()):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    v=arm.check_path(a,a.arm_q(),[q],n=10,obst=obst,allow=allow)
    if v: print("ABORT path",name,v[:5]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    print(name,"->",np.round(a.fk()[0],3),"code",code,flush=True)
    return code
# leave the viewing pose: go up/back first (hand ~ tilt45 fx=False now)
go("retreat up",(-0.17,-0.45,1.15),T(45,False),3)
go("high mid",(-0.05,-0.15,1.25),T(90,True),4)
go("above mug",(MUG[0],MUG[1],1.12),T(90,True),4)
print("fingers",a.fingers())
EOF
python3 s12.py 2>&1 | grep -v "^\[INFO"

# openrua op 30
cat > s12b.py <<'EOF'
import arm, numpy as np, sys
a = arm.Arm()
MUG=(0.009,0.007)
T=arm.tilt_y
def go(name,xyz,quat,secs=4,obst=None,allow=()):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    v=arm.check_path(a,a.arm_q(),[q],n=10,obst=obst,allow=allow)
    if v: print("ABORT path",name,v[:5]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    print(name,"->",np.round(a.fk()[0],3),"code",code,flush=True)
    return code
go("higher",(-0.17,-0.45,1.28),T(45,False),3)
go("rotate down",(-0.17,-0.45,1.32),T(90,True),4)
go("high mid",(-0.05,-0.15,1.28),T(90,True),4)
go("above mug",(MUG[0],MUG[1],1.12),T(90,True),4)
print("fingers",a.fingers())
EOF
python3 s12b.py 2>&1 | grep -v "^\[INFO"

# openrua op 31
cat > s12c.py <<'EOF'
import arm, numpy as np, sys
a = arm.Arm()
T=arm.tilt_y
print("now", np.round(a.fk()[0],3), np.round(a.arm_q(),2))
for name,xyz,q in [("h45f",(-0.17,-0.45,1.28),T(45,False)),("h45t",(-0.17,-0.45,1.28),T(45,True)),
                   ("h60f",(-0.17,-0.45,1.25),T(60,False)),("h90t",(-0.17,-0.40,1.28),T(90,True)),
                   ("h90t2",(-0.17,-0.45,1.3),T(90,True)),("h90f",(-0.17,-0.45,1.3),T(90,False)),
                   ("h70f",(-0.17,-0.45,1.25),T(70,False)),("h90t3",(-0.12,-0.35,1.3),T(90,True))]:
    s=a.ik(xyz,q,at_tcp=True,seed=a.arm_q(),timeout=8)
    print(name, None if s is None else np.round(s,2), flush=True)
EOF
python3 s12c.py 2>&1 | grep -v "^\[INFO"

# openrua op 32
cat > s12d.py <<'EOF'
import arm, numpy as np, sys
a = arm.Arm()
MUG=(0.009,0.007)
T=arm.tilt_y
def go(name,xyz,quat,secs=4,obst=None,allow=()):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    v=arm.check_path(a,a.arm_q(),[q],n=10,obst=obst,allow=allow)
    if v: print("ABORT path",name,v[:5]); sys.exit(1)
    print(name,"jump",np.round(np.abs(np.array(q)-a.arm_q()).max(),2))
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    print(name,"->",np.round(a.fk()[0],3),"code",code,flush=True)
    return code
go("h60f",(-0.17,-0.45,1.25),T(60,False),3)
go("h90f",(-0.17,-0.45,1.3),T(90,False),3)
go("high mid",(-0.05,-0.15,1.28),T(90,False),4)
go("above mug",(MUG[0],MUG[1],1.12),T(90,False),4)
print("fingers",a.fingers())
EOF
python3 s12d.py 2>&1 | grep -v "^\[INFO"

# openrua op 33
python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_above.png 2>&1 | grep -v INFO

# openrua op 34
cat > s13.py <<'EOF'
import arm, numpy as np, sys
a = arm.Arm()
MUG=(0.009,0.007)
T=arm.tilt_y
def go(name,xyz,quat,secs=4):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    print(name,"jump",np.round(np.abs(np.array(q)-a.arm_q()).max(),2))
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    print(name,"->",np.round(a.fk()[0],3),"code",code,flush=True)
go("pregrasp",(MUG[0],MUG[1],1.00),T(90,False),3)
go("grasp",(MUG[0],MUG[1],0.948),T(90,False),3)
print("fingers before",a.fingers())
print("grip",a.grip(0.0))
print("fingers after",a.fingers())
EOF
python3 s13.py 2>&1 | grep -v "^\[INFO"

# openrua op 35
python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_g.png 2>&1 | grep -v INFO; python3 tools/perception/cam_snap.py agentview snaps/agent_g.png 2>&1 | grep -v INFO; python3 -c "
import arm,numpy as np
a=arm.Arm(); print(np.round(a.arm_q(),3)); print(a.fk())" 2>&1 | grep -v INFO

# openrua op 36
cat > s14.py <<'EOF'
import arm, numpy as np
a=arm.Arm()
print("grip",a.grip(0.08))
q=a.arm_q()
xyz,quat=a.fk()
print("hand",np.round(xyz,3))
s=a.ik((xyz[0],xyz[1],xyz[2]+0.12),quat,at_tcp=False,seed=q,timeout=10)
print("code",a.move_joints([s],seconds_per_seg=3))
print("hand",np.round(a.fk()[0],3),"fingers",a.fingers())
EOF
python3 s14.py 2>&1 | grep -v "^\[INFO"; python3 tools/perception/cam_snap.py birdview snaps/bird_g.png 2>&1 | grep -v INFO; python3 tools/perception/cam_snap.py agentview snaps/agent_g2.png 2>&1 | grep -v INFO

# openrua op 37
cat > s15.py <<'EOF'
import numpy as np, subprocess, sys
sys.argv=['scene.py','birdview']
import scene
D,C,K,R,t=scene.cam_model('birdview')
P=scene.to_world(D,K,R,t)
np.save('snaps/bird_P2.npy',P)
X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(np.abs(X-0.009)<0.08)&(np.abs(Y-0.007)<0.08)&(Z>0.91)
print("mug pts",m.sum(),"zmax",Z[m].max())
for zt in [0.93,0.95,0.96,0.97,0.975,0.98,0.99,1.0]:
    mm=m&(Z>zt)
    if mm.sum(): print(zt,mm.sum(),"x",np.round([X[mm].min(),X[mm].max()],3),"y",np.round([Y[mm].min(),Y[mm].max()],3),"c",np.round([X[mm].mean(),Y[mm].mean()],3))
EOF
python3 s15.py 2>&1 | grep -v INFO

# openrua op 38
cat > s16.py <<'EOF'
import numpy as np, sys
import scene
D,C,K,R,t=scene.cam_model('robot0_eye_in_hand')
P=scene.to_world(D,K,R,t)
X,Y,Z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(Z)
m=ok&(np.abs(X-0.009)<0.09)&(np.abs(Y-0.007)<0.09)&(Z>0.905)&(Z<1.1)
print("pts",m.sum(),"zmax",Z[m].max())
for zt in [0.92,0.94,0.95,0.96,0.965,0.97,0.975,0.98,0.985,0.99,1.0]:
    mm=m&(Z>zt)
    if mm.sum(): print(zt,mm.sum(),"x",np.round([X[mm].min(),X[mm].max()],3),"y",np.round([Y[mm].min(),Y[mm].max()],3),"c",np.round([X[mm].mean(),Y[mm].mean()],3))
# table height near mug
tb=ok&(np.abs(X-0.009)<0.15)&(np.abs(Y-0.007)<0.15)&(Z<0.92)
print("table z", np.round(np.median(Z[tb]),4))
import cv2; cv2.imwrite('snaps/eih_top.png',C)
EOF
python3 s16.py 2>&1 | grep -v INFO

# openrua op 39
cat > s17.py <<'EOF'
import numpy as np, scene
D,C,K,R,t=scene.cam_model('robot0_eye_in_hand')
P=scene.to_world(D,K,R,t)
X,Y,Z=P[...,0],P[...,1],P[...,2]
# find pixel closest to mug center (0.01,0.01) at rim level
ok=np.isfinite(Z)
m=ok&(Z>0.985)&(Z<1.02)
print("rim-level pts", m.sum(), "x",np.round([X[m].min(),X[m].max()],3),"y",np.round([Y[m].min(),Y[m].max()],3), "zmed",np.round(np.median(Z[m]),3))
# histogram of z for points within 0.06 of center
mm=ok&(np.hypot(X-0.01,Y-0.01)<0.07)&(Z>0.905)
h,e=np.histogram(Z[mm],bins=np.arange(0.90,1.03,0.005))
for c,lo in zip(h,e[:-1]): print(f"{lo:.3f} {c}")
# radial profile: for points with z>0.99 (rim), radius from center
r=np.hypot(X[m]-0.01,Y[m]-0.01)
print("rim radius pct", np.round(np.percentile(r,[5,50,95,99]),3))
# body: points z in 0.93..0.98
b=ok&(Z>0.93)&(Z<0.98)&(np.hypot(X-0.01,Y-0.01)<0.08)
r=np.hypot(X[b]-0.01,Y[b]-0.01); print("body radius pct", np.round(np.percentile(r,[5,50,95,99]),3), "n",b.sum())
EOF
python3 s17.py 2>&1 | grep -v INFO

# openrua op 40
cat > s18.py <<'EOF'
import numpy as np, scene
D,C,K,R,t=scene.cam_model('robot0_eye_in_hand')
P=scene.to_world(D,K,R,t)
X,Y,Z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(Z)
print("cam t",np.round(t,3))
for lo,hi in [(0.93,0.95),(0.95,0.97),(0.97,0.98),(0.98,0.99),(0.99,0.999),(0.999,1.01)]:
    b=ok&(Z>lo)&(Z<hi)&(np.hypot(X-0.01,Y-0.01)<0.1)
    if b.sum()==0: continue
    r=np.hypot(X[b]-0.01,Y[b]-0.01)
    ang=np.degrees(np.arctan2(Y[b]-0.01,X[b]-0.01))
    print(f"z {lo}-{hi} n={b.sum()} r pct {np.round(np.percentile(r,[5,50,95]),3)} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
# unique depth values near 0.25
d=D[np.isfinite(D)]
u,c=np.unique(np.round(d,4),return_counts=True)
print("most common depths",[(float(a),int(b)) for a,b in sorted(zip(u,c),key=lambda x:-x[1])[:5]], "dmax",d.max())
EOF
python3 s18.py 2>&1 | grep -v INFO

# openrua op 41
cat > s19.py <<'EOF'
import numpy as np, scene, sys
for cam in ['frontview','sideview','agentview']:
    try:
        D,C,K,R,t=scene.cam_model(cam)
    except SystemExit as e:
        print(cam,"fail",e); continue
    P=scene.to_world(D,K,R,t)
    X,Y,Z=P[...,0],P[...,1],P[...,2]
    ok=np.isfinite(Z)
    print(cam,"cam at",np.round(t,3))
    m=ok&(np.abs(X-0.01)<0.09)&(np.abs(Y-0.01)<0.09)&(Z>0.905)&(Z<1.1)
    if m.sum()==0: print(" no mug pts"); continue
    print(" n",m.sum(),"zmax",np.round(Z[m].max(),3),"x",np.round([X[m].min(),X[m].max()],3),"y",np.round([Y[m].min(),Y[m].max()],3))
    for lo,hi in [(0.92,0.95),(0.95,0.97),(0.97,0.98),(0.98,0.99),(0.99,1.0),(1.0,1.02)]:
        b=m&(Z>lo)&(Z<hi)
        if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
    # grey mug too
    g=ok&(np.abs(X-0.007)<0.09)&(np.abs(Y-0.338)<0.09)&(Z>0.905)&(Z<1.1)
    if g.sum(): print(" grey zmax",np.round(Z[g].max(),3),"x",np.round([X[g].min(),X[g].max()],3),"y",np.round([Y[g].min(),Y[g].max()],3))
EOF
python3 s19.py 2>&1 | grep -v INFO

# openrua op 42
cat > s20.py <<'EOF'
import numpy as np, scene
for cam in ['frontview','agentview']:
    D,C,K,R,t=scene.cam_model(cam)
    P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
    # hand body: z in 1.245..1.30 near (0.022,-0.001)
    h=ok&(Z>1.25)&(Z<1.30)&(np.abs(X-0.022)<0.2)&(np.abs(Y)<0.1)
    print(cam,"hand slice n",h.sum(),"x",np.round([X[h].min(),X[h].max()],3),"y",np.round([Y[h].min(),Y[h].max()],3))
    # fingers: z 1.14..1.24
    f=ok&(Z>1.15)&(Z<1.23)&(np.abs(X-0.022)<0.2)&(np.abs(Y)<0.1)
    if f.sum(): print("  fingers x",np.round([X[f].min(),X[f].max()],3),"y",np.round([Y[f].min(),Y[f].max()],3))
    # mug base slices
    for lo,hi in [(0.902,0.91),(0.91,0.92),(0.92,0.93),(0.93,0.94)]:
        b=ok&(Z>lo)&(Z<hi)&(np.abs(X-0.012)<0.09)&(np.abs(Y-0.01)<0.09)
        if b.sum(): print(f"  base z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
    # handle: points with y<-0.045 near mug
    hd=ok&(Z>0.905)&(Z<1.02)&(np.abs(X-0.012)<0.09)&(Y<-0.045)&(Y>-0.1)
    if hd.sum():
        print("  handle n",hd.sum(),"x",np.round([X[hd].min(),X[hd].max()],3),"y",np.round([Y[hd].min(),Y[hd].max()],3),"z",np.round([Z[hd].min(),Z[hd].max()],3))
        for lo,hi in [(0.92,0.94),(0.94,0.96),(0.96,0.98),(0.98,1.0)]:
            s=hd&(Z>lo)&(Z<hi)
            if s.sum(): print(f"   handle z{lo}-{hi} n={s.sum()} x[{X[s].min():.3f},{X[s].max():.3f}] y[{Y[s].min():.3f},{Y[s].max():.3f}]")
EOF
python3 s20.py 2>&1 | grep -v INFO

# openrua op 43
grep -n -i -A12 "<link name=\"panda_leftfinger\"\|<link name=\"panda_rightfinger\"\|<link name=\"panda_hand\"" snaps/robot.urdf | head -80; grep -n -i "finger_joint1\|finger_joint2\|finger_joint" snaps/robot.urdf | head

# openrua op 44
cat > s21.py <<'EOF'
import arm, numpy as np
a=arm.Arm()
def ori(sign, alpha_deg, ysign=1):
    al=np.radians(alpha_deg)
    Z=np.array([sign*np.cos(al),0,-np.sin(al)]); Y=np.array([0,ysign,0.0]); X=np.cross(Y,Z)
    return arm.quat_from_axes(X,Y,Z)
BAR=(0.019,-0.0735,0.962)
tests=[]
for sign in (+1,-1):
  for al in (0,20,40):
    for ys in (1,-1):
      tests.append((f"pick sign{sign} a{al} y{ys}",BAR,ori(sign,al,ys)))
      tests.append((f"mw   sign{sign} a{al} y{ys}",(-0.164,-0.385,1.0),ori(sign,al,ys)))
      tests.append((f"mwhi sign{sign} a{al} y{ys}",(-0.164,-0.43,1.05),ori(sign,al,ys)))
seed=a.arm_q()
for name,xyz,q in tests:
    s=a.ik(xyz,q,at_tcp=True,seed=seed,timeout=6)
    if s is None: s=a.ik(xyz,q,at_tcp=True,seed=[0,-0.785,0,-2.356,0,1.571,0.785],timeout=6)
    print(f"{name:26s} {'OK '+str(np.round(s,2)) if s is not None else '--'}",flush=True)
EOF
python3 s21.py 2>&1 | grep -v "^\[INFO" | grep -v "IK failed"

# openrua op 45
cat > s22.py <<'EOF'
import arm, numpy as np, sys
a=arm.Arm()
T=arm.tilt_y
BAR=(0.016,-0.0735)
def go(name,xyz,quat,secs=3):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    print(name,"jump",np.round(np.abs(np.array(q)-a.arm_q()).max(),2))
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    print(name,"->",np.round(a.fk()[0],3),"code",code,flush=True)
    return code
go("above bar",(BAR[0],BAR[1],1.10),T(90,False),3)
go("pinch height",(BAR[0],BAR[1],0.962),T(90,False),3)
print("grip",a.grip(0.0))
go("lift",(BAR[0],BAR[1],1.04),T(90,False),3)
print("fingers",a.fingers())
EOF
python3 s22.py 2>&1 | grep -v "^\[INFO"; python3 tools/perception/cam_snap.py agentview snaps/agent_h.png 2>&1 | grep -v INFO; python3 tools/perception/cam_snap.py frontview snaps/front_h.png 2>&1 | grep -v INFO

# openrua op 46
cat > s23.py <<'EOF'
import numpy as np, scene
for cam in ['frontview','agentview']:
    D,C,K,R,t=scene.cam_model(cam)
    P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
    m=ok&(np.abs(X-0.012)<0.09)&(Y>-0.05)&(Y<0.08)&(Z>0.905)&(Z<1.2)
    print(cam,"mug body n",m.sum(),"z",np.round([Z[m].min(),Z[m].max()],3))
    for lo,hi in [(0.93,0.96),(0.96,0.99),(0.99,1.02),(1.02,1.05),(1.05,1.08),(1.08,1.1)]:
        b=m&(Z>lo)&(Z<hi)
        if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}] cx={X[b].mean():.3f}")
EOF
python3 s23.py 2>&1 | grep -v INFO

# openrua op 47
cat > s24.py <<'EOF'
import arm, numpy as np
a=arm.Arm(); T=arm.tilt_y
seq=[("pre top",(-0.131,-0.50,1.06),T(90,False)),
     ("pre pitched",(-0.131,-0.50,1.06),T(60,False)),
     ("mid pitched",(-0.131,-0.45,1.02),T(60,False)),
     ("place pitched",(-0.131,-0.398,0.98),T(60,False)),
     ("place low",(-0.131,-0.398,0.972),T(60,False)),
     ("retreat",(-0.131,-0.46,1.02),T(60,False))]
seed=a.arm_q(); print("now",np.round(seed,2))
for name,xyz,q in seq:
    s=a.ik(xyz,q,at_tcp=True,seed=seed,timeout=8)
    print(f"{name:14s} {'OK '+str(np.round(s,2))+' jump=%.2f'%np.abs(np.array(s)-seed).max() if s is not None else '--'}",flush=True)
    if s is not None: seed=s
EOF
python3 s24.py 2>&1 | grep -v "^\[INFO"

# openrua op 48
cat > s25.py <<'EOF'
import arm, numpy as np
a=arm.Arm(); T=arm.tilt_y
seq=[("lift high",(0.016,-0.0735,1.19),T(90,False)),
     ("over mw",(-0.06,-0.30,1.19),T(90,False)),
     ("front high",(-0.131,-0.51,1.19),T(90,False)),
     ("front mid",(-0.131,-0.51,1.10),T(90,False)),
     ("front low",(-0.131,-0.51,1.02),T(90,False)),
     ("pitch75",(-0.131,-0.48,1.02),T(75,False)),
     ("pitch60",(-0.131,-0.45,1.02),T(60,False)),
     ("place",(-0.131,-0.398,0.98),T(60,False))]
seed=a.arm_q(); print("now",np.round(seed,2))
for name,xyz,q in seq:
    s=a.ik(xyz,q,at_tcp=True,seed=seed,timeout=8)
    print(f"{name:12s} {'OK '+str(np.round(s,2))+' jump=%.2f'%np.abs(np.array(s)-seed).max() if s is not None else '--'}",flush=True)
    if s is not None: seed=s
EOF
python3 s25.py 2>&1 | grep -v "^\[INFO"

# openrua op 49
cat > s26.py <<'EOF'
import arm, numpy as np, sys
a=arm.Arm(); T=arm.tilt_y
OB=[("mw",-0.30,0.086,-0.37,-0.14,0,1.117),("door",-0.32,-0.28,-0.62,-0.36,0,1.117),("table",-0.8,0.56,-0.66,0.66,0,0.905)]
def go(name,xyz,quat,secs=4):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    v=arm.check_path(a,a.arm_q(),[q],n=10,obst=OB)
    if v: print("ABORT path",name,v[:4]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    print(name,"->",np.round(a.fk()[0],3),"code",code,"fingers",np.round(a.fingers(),4),flush=True)
    if code!=0: sys.exit(1)
go("lift high",(0.016,-0.0735,1.19),T(90,False),4)
go("over mw",(-0.06,-0.30,1.19),T(90,False),4)
go("front high",(-0.131,-0.51,1.19),T(90,False),4)
go("front mid",(-0.131,-0.51,1.10),T(90,False),3)
go("front low",(-0.131,-0.51,1.02),T(90,False),3)
EOF
python3 s26.py 2>&1 | grep -v "^\[INFO"; python3 tools/perception/cam_snap.py agentview snaps/agent_i.png 2>&1 | grep -v INFO; python3 tools/perception/cam_snap.py birdview snaps/bird_i.png 2>&1 | grep -v INFO

# openrua op 50
cat > s27.py <<'EOF'
import numpy as np, scene
for cam in ['birdview','frontview']:
    D,C,K,R,t=scene.cam_model(cam)
    P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
    m=ok&(X>-0.25)&(X<-0.02)&(Y>-0.52)&(Y<-0.37)&(Z>0.93)&(Z<1.08)
    print(cam,"n",m.sum())
    if m.sum():
        print(" z",np.round([Z[m].min(),Z[m].max()],3))
        for lo,hi in [(0.94,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.08)]:
            b=m&(Z>lo)&(Z<hi)
            if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
    # microwave face: points with z in 1.0..1.08, x in opening, y near -0.36
    f=ok&(X>-0.30)&(X<0.08)&(Y>-0.40)&(Y<-0.30)&(Z>1.095)&(Z<1.12)
    if f.sum(): print(" top front edge y min", np.round(Y[f].min(),3), "x", np.round([X[f].min(),X[f].max()],3))
EOF
python3 s27.py 2>&1 | grep -v INFO

# openrua op 51
cat > s28.py <<'EOF'
import arm, numpy as np, sys
a=arm.Arm(); T=arm.tilt_y
OB=[("mw",-0.30,0.086,-0.37,-0.14,0,1.117),("door",-0.32,-0.28,-0.62,-0.36,0,1.117),("table",-0.8,0.56,-0.66,0.66,0,0.905)]
def go(name,xyz,quat,secs=4):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    v=arm.check_path(a,a.arm_q(),[q],n=10,obst=OB)
    if v: print("ABORT path",name,v[:4]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print(name,"-> hand",np.round(xyz_,3),"Z",np.round(R[:,2],2),"code",code,"fingers",np.round(a.fingers(),4),flush=True)
    if code!=0: sys.exit(1)
go("pitch75",(-0.131,-0.48,1.02),T(75,False),4)
go("pitch60",(-0.131,-0.45,1.02),T(60,False),4)
EOF
python3 s28.py 2>&1 | grep -v "^\[INFO"; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_p.png 2>&1 | grep -v INFO; python3 tools/perception/cam_snap.py sideview snaps/side_p.png 2>&1 | grep -v INFO

# openrua op 52
cat > s29.py <<'EOF'
import numpy as np, scene
D,C,K,R,t=scene.cam_model('robot0_eye_in_hand')
P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
print("cam",np.round(t,3))
m=ok&(X>-0.20)&(X<-0.07)&(Y>-0.47)&(Y<-0.25)&(Z>0.93)&(Z<1.12)
print("mug-ish n",m.sum())
for lo,hi in [(0.94,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.09),(1.09,1.12)]:
    b=m&(Z>lo)&(Z<hi)
    if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
# cavity plate visible?
p=ok&(X>-0.23)&(X<-0.04)&(Y>-0.36)&(Y<-0.16)&(Z>0.92)&(Z<0.96)
print("plate pts",p.sum(), "z med",np.round(np.median(Z[p]),3) if p.sum() else None, "y range",np.round([Y[p].min(),Y[p].max()],3) if p.sum() else None)
# lip / front face
f=ok&(X>-0.23)&(X<-0.04)&(Y>-0.40)&(Y<-0.33)&(Z>0.90)&(Z<0.95)
if f.sum(): print("lip region z",np.round([Z[f].min(),Z[f].max()],3),"y",np.round([Y[f].min(),Y[f].max()],3))
EOF
python3 s29.py 2>&1 | grep -v INFO

# openrua op 53
cat > s30.py <<'EOF'
import numpy as np, scene
D,C,K,R,t=scene.cam_model('birdview')
P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
m=ok&(X>-0.21)&(X<-0.06)&(Y>-0.50)&(Y<-0.28)&(Z>0.93)&(Z<1.3)
print("n",m.sum())
for lo,hi in [(0.93,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.09),(1.09,1.12),(1.12,1.16),(1.16,1.3)]:
    b=m&(Z>lo)&(Z<hi)
    if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
import cv2; cv2.imwrite('snaps/bird_p.png',C)
EOF
python3 s30.py 2>&1 | grep -v INFO

# openrua op 54
cat > s31.py <<'EOF'
import numpy as np, scene, cv2
D,C,K,R,t=scene.cam_model('agentview')
P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
m=ok&(X>-0.21)&(X<-0.06)&(Y>-0.50)&(Y<-0.27)&(Z>0.93)&(Z<1.13)
print("n",m.sum())
for lo,hi in [(0.93,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.09),(1.09,1.12)]:
    b=m&(Z>lo)&(Z<hi)
    if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
cv2.imwrite('snaps/agent_p.png',C)
EOF
python3 s31.py 2>&1 | grep -v INFO

# openrua op 55
cat > s32.py <<'EOF'
import arm, numpy as np, sys
a=arm.Arm(); T=arm.tilt_y
OB=[("mw",-0.30,0.086,-0.37,-0.14,0,1.117),("door",-0.32,-0.28,-0.62,-0.36,0,1.117),("table",-0.8,0.56,-0.66,0.66,0,0.905)]
def go(name,xyz,quat,secs=4):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    v=arm.check_path(a,a.arm_q(),[q],n=10,obst=OB)
    if v: print("ABORT path",name,v[:4]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print(name,"-> tcp",np.round(xyz_+0.1034*R[:,2],3),"Z",np.round(R[:,2],2),"code",code,"fingers",np.round(a.fingers(),4),flush=True)
    if code!=0: sys.exit(1)
go("place",(-0.131,-0.398,0.98),T(60,False),4)
print("release",a.grip(0.08))
go("retreat1",(-0.131,-0.45,1.02),T(60,False),3)
go("retreat2",(-0.131,-0.51,1.06),T(75,False),3)
EOF
python3 s32.py 2>&1 | grep -v "^\[INFO"; python3 tools/perception/cam_snap.py agentview snaps/agent_q.png 2>&1 | grep -v INFO

# openrua op 56
cat > s33.py <<'EOF'
import arm, numpy as np, sys
a=arm.Arm(); T=arm.tilt_y
q=a.ik((-0.14,-0.47,1.03),T(45,False),at_tcp=True,seed=a.arm_q(),timeout=10)
print("code",a.move_joints([q],seconds_per_seg=3))
xyz,quat=a.fk(); R=arm.quat_to_R(quat); print("tcp",np.round(xyz+0.1034*R[:,2],3),"Z",np.round(R[:,2],2))
EOF
python3 s33.py 2>&1 | grep -v "^\[INFO"; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_q.png 2>&1 | grep -v INFO; python3 -c "
import cv2; im=cv2.imread('snaps/eih_q.png'); cv2.imwrite('snaps/eih_q_rot.png', cv2.rotate(im, cv2.ROTATE_180))"

# openrua op 57
cat > s33.py <<'EOF'
import arm, numpy as np, sys
a=arm.Arm(); T=arm.tilt_y
for xyz in [(-0.17,-0.45,1.02),(-0.14,-0.46,1.02),(-0.14,-0.48,1.04)]:
    q=a.ik(xyz,T(45,False),at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is not None: break
if q is None: sys.exit("no ik")
print("code",a.move_joints([q],seconds_per_seg=3))
xyz,quat=a.fk(); R=arm.quat_to_R(quat); print("tcp",np.round(xyz+0.1034*R[:,2],3),"Z",np.round(R[:,2],2))
EOF
python3 s33.py 2>&1 | grep -v "^\[INFO"; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_q.png 2>&1 | grep -v INFO; python3 -c "
import cv2; im=cv2.imread('snaps/eih_q.png'); cv2.imwrite('snaps/eih_q_rot.png', cv2.rotate(im, cv2.ROTATE_180))"

# openrua op 58
cat > s34.py <<'EOF'
import numpy as np, scene
D,C,K,R,t=scene.cam_model('robot0_eye_in_hand')
P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
print("cam",np.round(t,3))
m=ok&(X>-0.25)&(X<-0.02)&(Y>-0.40)&(Y<-0.15)&(Z>0.93)&(Z<1.10)
# exclude points that are the microwave walls: keep x in opening interior
print("n",m.sum())
for lo,hi in [(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.01),(1.01,1.03),(1.03,1.05),(1.05,1.07),(1.07,1.10)]:
    b=m&(Z>lo)&(Z<hi)
    if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
# yellow color mask to isolate mug
hsv=__import__('cv2').cvtColor(C,__import__('cv2').COLOR_BGR2HSV)
yel=(hsv[...,0]>15)&(hsv[...,0]<40)&(hsv[...,1]>80)&(hsv[...,2]>60)
ym=yel&ok
print("yellow pts",ym.sum())
if ym.sum():
    print(" x",np.round([X[ym].min(),X[ym].max()],3),"y",np.round([Y[ym].min(),Y[ym].max()],3),"z",np.round([Z[ym].min(),Z[ym].max()],3))
    for lo,hi in [(0.93,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.10)]:
        b=ym&(Z>lo)&(Z<hi)
        if b.sum(): print(f"  yel z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}] cx={X[b].mean():.3f}")
# white parts of mug (the body is white/yellow)
wh=(hsv[...,1]<40)&(hsv[...,2]>150)&ok&(Y>-0.40)&(Y<-0.15)&(Z>0.93)&(Z<1.1)&(X>-0.25)&(X<-0.02)
print("white pts",wh.sum())
if wh.sum():
    for lo,hi in [(0.93,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.10)]:
        b=wh&(Z>lo)&(Z<hi)
        if b.sum(): print(f"  wh z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
EOF
python3 s34.py 2>&1 | grep -v INFO

# openrua op 59
cat > s35.py <<'EOF'
import arm, numpy as np, sys
a=arm.Arm(); T=arm.tilt_y
OB=[("mw",-0.30,0.086,-0.37,-0.14,0,1.117),("door",-0.32,-0.28,-0.62,-0.36,0,1.117),("table",-0.8,0.56,-0.66,0.66,0,0.905)]
Q=T(60,False); Zax=np.array([0,0.5,-0.866])
BAR=np.array([-0.1315,-0.4085,1.003])
def go(name,xyz,quat,secs=3,check=True):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    if check:
        v=arm.check_path(a,a.arm_q(),[q],n=10,obst=OB)
        if v: print("ABORT path",name,v[:4]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print(name,"-> tcp",np.round(xyz_+0.1034*R[:,2],3),"Z",np.round(R[:,2],2),"code",code,"fingers",np.round(a.fingers(),4),flush=True)
    if code!=0: sys.exit(1)
go("approach",BAR-0.06*Zax,Q,4)
go("at bar",BAR,Q,3)
print("pinch",a.grip(0.0))
f=a.fingers()
if abs(f[0])>0.012: print("pinch missed? fingers",f); sys.exit(1)
go("push",BAR+np.array([0,0.03,0]),Q,4,check=False)
print("release",a.grip(0.08))
go("retreat",BAR+np.array([0,0.03,0])-0.06*Zax,Q,3,check=False)
EOF
python3 s35.py 2>&1 | grep -v "^\[INFO"

# openrua op 60
cat > s36.py <<'EOF'
import arm, scene, numpy as np, sys, cv2
a=arm.Arm(); T=arm.tilt_y
def go(name,xyz,quat,secs=3):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print(name,"-> tcp",np.round(xyz_+0.1034*R[:,2],3),"code",code,flush=True)
go("view",np.array([-0.17,-0.45,1.02]),T(45,False),4)
D,C,K,R,t=scene.cam_model("robot0_eye_in_hand")
cv2.imwrite("snaps/eih_r.png",C)
P=scene.to_world(D,K,R,t)
hsv=cv2.cvtColor(C,cv2.COLOR_BGR2HSV)
yel=(hsv[...,0]>18)&(hsv[...,0]<38)&(hsv[...,1]>90)&(hsv[...,2]>90)
ok=np.isfinite(P[...,2])
m=yel&ok
pts=P[m]
print("yellow n",m.sum(),"x",np.round([pts[:,0].min(),pts[:,0].max()],3),"y",np.round([pts[:,1].min(),pts[:,1].max()],3),"z",np.round([pts[:,2].min(),pts[:,2].max()],3))
for lo,hi in [(0.94,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06)]:
    s=pts[(pts[:,2]>=lo)&(pts[:,2]<hi)]
    if len(s): print(f" z{lo}-{hi} n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] ymin5%={np.percentile(s[:,1],5):.3f}")
# anything (any colour) inside the opening region protruding past face plane?
allp=P[ok]
sel=allp[(allp[:,0]>-0.23)&(allp[:,0]<-0.04)&(allp[:,2]>0.95)&(allp[:,2]<1.085)&(allp[:,1]<-0.36)&(allp[:,1]>-0.50)]
print("non-face pts in opening x-range, y<-0.36, z 0.95-1.085:",len(sel))
if len(sel): print("  y range",np.round([sel[:,1].min(),sel[:,1].max()],3),"z",np.round([sel[:,2].min(),sel[:,2].max()],3),"x",np.round([sel[:,0].min(),sel[:,0].max()],3))
EOF
python3 s36.py 2>&1 | grep -v "^\[INFO"

# openrua op 61
sed -n 1,40p scene.py; grep -n "shutdown\|def close\|def __init__" arm.py | head

# openrua op 62
python3 - <<'EOF'
import re
s=open("scene.py").read()
s=s.replace('    rclpy.init(); node=rclpy.create_node("scene")','    own=not rclpy.ok()\n    if own: rclpy.init()\n    node=rclpy.create_node("scene_"+str(np.random.randint(1e6)))')
s=s.replace('    rclpy.shutdown()\n    return D,C,K,R,tt','    node.destroy_node()\n    if own: rclpy.shutdown()\n    return D,C,K,R,tt')
open("scene.py","w").write(s)
EOF
sed -i 's/^go("view".*$/#&/' s36.py
python3 s36.py 2>&1 | grep -v "^\[INFO"

# openrua op 63
cat > s37.py <<'EOF'
import scene, numpy as np, cv2
D,C,K,R,t=scene.cam_model("robot0_eye_in_hand")
cv2.imwrite("snaps/eih_r.png",C)
P=scene.to_world(D,K,R,t)
hsv=cv2.cvtColor(C,cv2.COLOR_BGR2HSV)
ok=np.isfinite(P[...,2])&(D>0.12)
yel=(hsv[...,0]>15)&(hsv[...,0]<40)&(hsv[...,1]>60)&(hsv[...,2]>35)&ok
pts=P[yel]
print("yellow n",yel.sum(),"x",np.round([pts[:,0].min(),pts[:,0].max()],3),"y",np.round([pts[:,1].min(),pts[:,1].max()],3),"z",np.round([pts[:,2].min(),pts[:,2].max()],3))
for lo,hi in [(0.94,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06)]:
    s=pts[(pts[:,2]>=lo)&(pts[:,2]<hi)]
    if len(s): print(f" z{lo}-{hi} n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] ymin1%={np.percentile(s[:,1],1):.3f}")
# everything (any colour) in the cavity box, beyond the face plane inward
allp=P[ok]
cav=allp[(allp[:,0]>-0.22)&(allp[:,0]<-0.05)&(allp[:,2]>0.95)&(allp[:,2]<1.08)&(allp[:,1]>-0.42)&(allp[:,1]<-0.17)]
print("cavity pts",len(cav),"y range",np.round([cav[:,1].min(),cav[:,1].max()],3),"y1%",np.round(np.percentile(cav[:,1],1),3))
# points that are in the opening x-range and z-range but y in [-0.40,-0.355] (would protrude)
pro=cav[cav[:,1]<-0.355]
print("protruding candidates",len(pro))
if len(pro): print(" x",np.round([pro[:,0].min(),pro[:,0].max()],3),"y",np.round([pro[:,1].min(),pro[:,1].max()],3),"z",np.round([pro[:,2].min(),pro[:,2].max()],3))
EOF
python3 s37.py 2>&1 | grep -v "^\[INFO"

# openrua op 64
cat > s38.py <<'EOF'
import arm, scene, numpy as np, sys, cv2
a=arm.Arm(); T=arm.tilt_y
OB=[("mw",-0.30,0.086,-0.37,-0.14,0,1.117),("door",-0.32,-0.28,-0.62,-0.36,0,1.117),("table",-0.8,0.56,-0.66,0.66,0,0.905)]
th=np.deg2rad(20); Q=T(20,False); Zax=np.array([0,np.cos(th),-np.sin(th)])
BAR=np.array([-0.132,-0.3815,1.003])
def go(name,xyz,quat,secs=3,check=True):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    if check:
        v=arm.check_path(a,a.arm_q(),[q],n=10,obst=OB)
        if v: print("ABORT path",name,v[:4]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print(name,"-> tcp",np.round(xyz_+0.1034*R[:,2],3),"Z",np.round(R[:,2],2),"code",code,"fingers",np.round(a.fingers(),4),flush=True)
    if code!=0: sys.exit(1)
go("approach",BAR-0.06*Zax,Q,4)
go("at bar",BAR,Q,3)
print("pinch",a.grip(0.0))
f=a.fingers()
if abs(f[0])>0.012: print("pinch missed? fingers",f); a.grip(0.08); sys.exit(1)
go("push",BAR+np.array([0,0.05,0]),Q,4,check=False)
print("release",a.grip(0.08))
go("retreat",BAR+np.array([0,0.05,0])-0.06*Zax,Q,3,check=False)
go("view",np.array([-0.17,-0.45,1.02]),T(45,False),4,check=False)
EOF
python3 s38.py 2>&1 | grep -v "^\[INFO" && python3 s37.py 2>&1 | grep -v "^\[INFO"

# openrua op 65
cat > s39.py <<'EOF'
import arm, numpy as np
a=arm.Arm(); T=arm.tilt_y
BAR=np.array([-0.132,-0.3815,1.003])
for deg in [20,25,30,35,40]:
    th=np.deg2rad(deg); Zax=np.array([0,np.cos(th),-np.sin(th)])
    for fx in [False,True]:
        Q=T(deg,fx); ok=[]
        seed=a.arm_q()
        for name,p in [("app",BAR-0.06*Zax),("bar",BAR),("push",BAR+[0,0.05,0])]:
            q=a.ik(p,Q,at_tcp=True,seed=seed,timeout=8)
            ok.append(q is not None)
            if q is not None: seed=q
        print(deg,fx,ok,flush=True)
EOF
python3 s39.py 2>&1 | grep -v "^\[INFO\|IK failed"

# openrua op 66
cat > s40.py <<'EOF'
import arm, numpy as np
a=arm.Arm(); T=arm.tilt_y
BAR=np.array([-0.132,-0.3815,1.003])
for deg in [30,35]:
    for fx in [False,True]:
        Q=T(deg,fx); ok=[]; seed=a.arm_q()
        for name,p in [("above",BAR+[0,0,0.06]),("mid",BAR+[0,0,0.03]),("bar",BAR),("push",BAR+[0,0.05,0]),("up",BAR+[0,0.05,0.05])]:
            q=a.ik(p,Q,at_tcp=True,seed=seed,timeout=8)
            ok.append(q is not None)
            if q is not None: seed=q
        print(deg,fx,ok,flush=True)
EOF
python3 s40.py 2>&1 | grep -v "^\[INFO\|IK failed"

# openrua op 67
cat > s41.py <<'EOF'
import arm, numpy as np, sys
a=arm.Arm(); T=arm.tilt_y
OB=[("mw",-0.30,0.086,-0.37,-0.14,0,1.117),("door",-0.32,-0.28,-0.62,-0.36,0,1.117),("table",-0.8,0.56,-0.66,0.66,0,0.905)]
Q=T(30,False)
BAR=np.array([-0.132,-0.3815,1.003]); PUSH=np.array([0,0.045,0])
def go(name,xyz,quat,secs=3,check=True):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    if check:
        v=arm.check_path(a,a.arm_q(),[q],n=10,obst=OB)
        if v: print("ABORT path",name,v[:4]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print(name,"-> tcp",np.round(xyz_+0.1034*R[:,2],3),"Z",np.round(R[:,2],2),"code",code,"fingers",np.round(a.fingers(),4),flush=True)
    if code!=0: sys.exit(1)
go("above",BAR+[0,0,0.06],Q,4)
go("mid",BAR+[0,0,0.03],Q,2,check=False)
go("at bar",BAR,Q,2,check=False)
print("pinch",a.grip(0.0))
f=a.fingers()
if abs(f[0])>0.012: print("pinch missed? fingers",f); a.grip(0.08); sys.exit(1)
go("push",BAR+PUSH,Q,4,check=False)
print("release",a.grip(0.08))
go("up",BAR+PUSH+[0,0,0.05],Q,3,check=False)
go("view",np.array([-0.17,-0.45,1.02]),T(45,False),4,check=False)
EOF
python3 s41.py 2>&1 | grep -v "^\[INFO" && python3 s37.py 2>&1 | grep -v "^\[INFO"

# openrua op 68
python3 tools/perception/cam_snap.py agentview snaps/agent_r.png 2>&1 | grep -v INFO; python3 tools/perception/cam_snap.py birdview snaps/bird_r.png 2>&1 | grep -v INFO

# openrua op 69
cat > s42.py <<'EOF'
import scene, numpy as np
D,C,K,R,t=scene.cam_model("birdview")
P=scene.to_world(D,K,R,t); ok=np.isfinite(P[...,2])
pts=P[ok]
# door region: x in [-0.36,-0.26], y in [-0.65,-0.36], z>0.92
s=pts[(pts[:,0]>-0.36)&(pts[:,0]<-0.26)&(pts[:,1]>-0.66)&(pts[:,1]<-0.365)&(pts[:,2]>0.915)]
print("door pts",len(s),"x",np.round([s[:,0].min(),s[:,0].max()],3),"y",np.round([s[:,1].min(),s[:,1].max()],3),"z",np.round([s[:,2].min(),np.percentile(s[:,2],50),s[:,2].max()],3))
for ylo in np.arange(-0.64,-0.36,0.04):
    q=s[(s[:,1]>=ylo)&(s[:,1]<ylo+0.04)]
    if len(q): print(f" y{ylo:.2f}: n={len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] ztop={np.percentile(q[:,2],95):.3f}")
# microwave top
m=pts[(pts[:,0]>-0.28)&(pts[:,0]<0.07)&(pts[:,1]>-0.35)&(pts[:,1]<-0.16)&(pts[:,2]>1.0)]
print("mw top z",np.round(np.percentile(m[:,2],[5,50,95]),3),"x",np.round([m[:,0].min(),m[:,0].max()],3),"y",np.round([m[:,1].min(),m[:,1].max()],3))
# face plane: points with y in [-0.40,-0.35], z 0.92-1.11, x in [-0.29,0.08]
f=pts[(pts[:,0]>-0.29)&(pts[:,0]<0.08)&(pts[:,1]>-0.42)&(pts[:,1]<-0.34)&(pts[:,2]>0.92)&(pts[:,2]<1.12)]
print("front region pts",len(f))
if len(f): print(" y",np.round(np.percentile(f[:,1],[1,50,99]),3),"z",np.round(np.percentile(f[:,2],[1,50,99]),3))
EOF
python3 s42.py 2>&1 | grep -v "^\[INFO"

# openrua op 70
cat > s43.py <<'EOF'
import arm, numpy as np, sys
a=arm.Arm(); T=arm.tilt_y
OB=[("mw",-0.30,0.086,-0.37,-0.14,0,1.117),("table",-0.8,0.56,-0.66,0.66,0,0.905)]
Q=T(90,False)
H=np.array([-0.29,-0.36]); r=0.20; eps=0.025; ZP=1.09
def P(deg):
    th=np.deg2rad(deg); d=np.array([np.cos(th),np.sin(th)]); n=np.array([np.sin(th),-np.cos(th)])
    xy=H+r*d+eps*n; return np.array([xy[0],xy[1],ZP])
def go(name,xyz,quat,secs=3,check=True):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    if check:
        v=arm.check_path(a,a.arm_q(),[q],n=10,obst=OB)
        if v: print("ABORT path",name,v[:4]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print(name,"-> tcp",np.round(xyz_+0.1034*R[:,2],3),"code",code,"fingers",np.round(a.fingers(),4),flush=True)
    if code!=0: sys.exit(1)
    return q
s0=P(-97); print("start",np.round(s0,3))
# pre-plan arc IK
seed=a.arm_q(); qs=[]
q=a.ik(s0,Q,at_tcp=True,seed=seed,timeout=10)
if q is None: print("ABORT ik start"); sys.exit(1)
seed=q
for deg in list(range(-90,1,10))+[0]:
    p=P(deg); q=a.ik(p,Q,at_tcp=True,seed=seed,timeout=10)
    if q is None: print("ABORT ik arc",deg,np.round(p,3)); sys.exit(1)
    jump=np.abs(np.array(q)-np.array(seed)).max()
    print(f"arc {deg:4d} p={np.round(p,3)} jump={jump:.2f}")
    if jump>0.6: print("ABORT big jump"); sys.exit(1)
    qs.append(q); seed=q
go("high start",s0+[0,0,0.11],Q,5)
print("close",a.grip(0.0))
go("start",s0,Q,3)
code=a.move_joints(qs,seconds_per_seg=1.5)
xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
print("arc done code",code,"tcp",np.round(xyz_+0.1034*R[:,2],3),flush=True)
if code!=0:
    code=a.move_joints([qs[-1]],seconds_per_seg=2)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print("retry final code",code,"tcp",np.round(xyz_+0.1034*R[:,2],3),flush=True)
go("lift",np.array([xyz_[0],xyz_[1],0])+0.1034*R[:,2]*np.array([1,1,0])+[0,0,1.22],Q,3,check=False)
EOF
python3 s43.py 2>&1 | grep -v "^\[INFO"

# openrua op 71
cat > s44.py <<'EOF'
import scene, numpy as np, cv2
D,C,K,R,t=scene.cam_model("birdview"); cv2.imwrite("snaps/bird_s.png",C)
P=scene.to_world(D,K,R,t); pts=P[np.isfinite(P[...,2])]
# anything left where the open door was?
s=pts[(pts[:,0]>-0.36)&(pts[:,0]<-0.26)&(pts[:,1]>-0.66)&(pts[:,1]<-0.40)&(pts[:,2]>0.915)&(pts[:,2]<1.12)]
print("old door region pts (z<1.12)",len(s))
# door closed: front region
for xlo in np.arange(-0.30,0.08,0.05):
    f=pts[(pts[:,0]>=xlo)&(pts[:,0]<xlo+0.05)&(pts[:,1]>-0.45)&(pts[:,1]<-0.30)&(pts[:,2]>1.05)&(pts[:,2]<1.12)]
    if len(f): print(f" x{xlo:.2f}: n={len(f)} ymin={f[:,1].min():.3f} y1%={np.percentile(f[:,1],1):.3f} ztop={np.percentile(f[:,2],95):.3f}")
EOF
python3 s44.py 2>&1 | grep -v "^\[INFO"; python3 tools/perception/cam_snap.py agentview snaps/agent_s.png 2>&1 | grep -v INFO

# openrua op 72
cat > s45.py <<'EOF'
import arm, scene, numpy as np, sys, cv2
a=arm.Arm(); T=arm.tilt_y
def go(name,xyz,quat,secs=4):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("IK none",name); return False
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print(name,"-> tcp",np.round(xyz_+0.1034*R[:,2],3),"Z",np.round(R[:,2],2),"code",code,flush=True); return code==0
for p in [(-0.15,-0.62,1.15),(-0.15,-0.58,1.2),(-0.17,-0.55,1.25)]:
    if go("look",np.array(p),T(30,False)): break
D,C,K,R,t=scene.cam_model("robot0_eye_in_hand"); cv2.imwrite("snaps/eih_s.png",C)
P=scene.to_world(D,K,R,t); pts=P[np.isfinite(P[...,2])&(D>0.12)]
f=pts[(pts[:,0]>-0.30)&(pts[:,0]<0.08)&(pts[:,1]>-0.50)&(pts[:,1]<-0.30)&(pts[:,2]>0.93)&(pts[:,2]<1.10)]
print("front face pts",len(f))
for xlo in np.arange(-0.30,0.08,0.04):
    q=f[(f[:,0]>=xlo)&(f[:,0]<xlo+0.04)]
    if len(q): print(f" x{xlo:.2f}: n={len(q)} y[{np.percentile(q[:,1],1):.3f},{np.percentile(q[:,1],50):.3f},{np.percentile(q[:,1],99):.3f}] z[{q[:,2].min():.3f},{q[:,2].max():.3f}]")
# any cavity visible (points with y > -0.34 inside opening footprint)?
c=pts[(pts[:,0]>-0.23)&(pts[:,0]<-0.04)&(pts[:,1]>-0.34)&(pts[:,1]<-0.15)&(pts[:,2]>0.94)&(pts[:,2]<1.09)]
print("cavity-interior pts visible",len(c))
EOF
python3 s45.py 2>&1 | grep -v "^\[INFO\|IK failed"
