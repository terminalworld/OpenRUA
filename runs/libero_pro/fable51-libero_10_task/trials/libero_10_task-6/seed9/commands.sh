#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -v parameter | head -40; echo ---; ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c; done

# openrua op 5
cat > tfdump.py <<'EOF'
import rclpy, sys
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node("tfd"); b=Buffer(); TransformListener(b,n)
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.2)
frames = b.all_frames_as_yaml()
print(frames)
for f in ["panda_link0","panda_hand","agentview_optical_frame","birdview_optical_frame","frontview_optical_frame","robot0_eye_in_hand_optical_frame"]:
    try:
        t=b.lookup_transform("world",f,rclpy.time.Time())
        tr=t.transform.translation; q=t.transform.rotation
        print(f, f"{tr.x:.4f} {tr.y:.4f} {tr.z:.4f}", f"q={q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}")
    except Exception as e: print(f, "ERR", e)
EOF
python3 tfdump.py

# openrua op 6
cat > cloud.py <<'EOF'
import rclpy, numpy as np, sys, cv2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
cam = sys.argv[1]
rclpy.init(); n=rclpy.create_node("cloud"); b=Buffer(); TransformListener(b,n)
got={}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d",m),1)
n.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c",m),1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i",m),1)
while len(got)<3 or not b.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()):
    rclpy.spin_once(n, timeout_sec=0.2)
br=CvBridge()
d=br.imgmsg_to_cv2(got["d"],"passthrough").astype(np.float64)
c=br.imgmsg_to_cv2(got["c"],"bgr8")
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
t=b.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
H,W=d.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1).reshape(-1,3)
Pw=(P@R.T+T).reshape(H,W,3)
np.save(f"{cam}_xyz.npy",Pw); np.save(f"{cam}_bgr.npy",c)
print("intrinsics",fx,fy,cx,cy,"size",W,H)
zs=Pw[...,2]; print("z range",np.nanmin(zs),np.nanmax(zs))
# table height guess: mode of z
hist,edges=np.histogram(zs[np.isfinite(zs)],bins=200)
i=hist.argmax(); print("dominant z",edges[i],edges[i+1])
EOF
python3 cloud.py birdview

# openrua op 7
cat > seg.py <<'EOF'
import numpy as np, cv2
P=np.load("birdview_xyz.npy"); c=np.load("birdview_bgr.npy")
z=P[...,2]
# table top height: look at z values in the table region (0.3..0.6)
m=(z>0.3)&(z<0.6)
hist,edges=np.histogram(z[m],bins=60); i=hist.argmax(); table=(edges[i]+edges[i+1])/2
print("table z ~",table)
obj=(z>table+0.01)&(z<table+0.4)
obj=obj.astype(np.uint8)
nl,lab,stats,cent=cv2.connectedComponentsWithStats(obj)
for j in range(1,nl):
    if stats[j,cv2.CC_STAT_AREA]<15: continue
    mm=lab==j
    pts=P[mm]; col=c[mm].mean(0)
    print(f"blob{j} area={stats[j,4]} px_c=({cent[j][0]:.0f},{cent[j][1]:.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} bgr={col.round()}")
vis=c.copy(); vis[obj>0]=(0,255,0); cv2.imwrite("birdview_seg.png",vis)
EOF
python3 seg.py

# openrua op 8
python3 cloud.py agentview && cat > seg2.py <<'EOF'
import numpy as np, cv2, sys
cam=sys.argv[1]; table=float(sys.argv[2])
P=np.load(f"{cam}_xyz.npy"); c=np.load(f"{cam}_bgr.npy")
z=P[...,2]
obj=(z>table+0.012)&(z<table+0.35)&(P[...,0]>-0.4)&(P[...,0]<0.5)&(np.abs(P[...,1])<0.6)
obj=obj.astype(np.uint8)
nl,lab,stats,cent=cv2.connectedComponentsWithStats(obj)
for j in range(1,nl):
    if stats[j,cv2.CC_STAT_AREA]<15: continue
    mm=lab==j
    pts=P[mm]; col=c[mm].mean(0)
    print(f"blob{j} area={stats[j,4]} px_c=({cent[j][0]:.0f},{cent[j][1]:.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} bgr={col.round()}")
vis=c.copy(); vis[obj>0]=(0,255,0); cv2.imwrite(f"{cam}_seg.png",vis)
EOF
python3 seg2.py agentview 0.426

# openrua op 9
cat > red.py <<'EOF'
import numpy as np, cv2, sys
for cam in ["agentview","birdview"]:
    P=np.load(f"{cam}_xyz.npy"); c=np.load(f"{cam}_bgr.npy").astype(int)
    b,g,r=c[...,0],c[...,1],c[...,2]
    red=(r>90)&(r>1.5*g)&(r>1.5*b)&(P[...,2]>0.43)&(P[...,2]<0.7)
    red=red.astype(np.uint8)
    nl,lab,stats,cent=cv2.connectedComponentsWithStats(red)
    for j in range(1,nl):
        if stats[j,4]<10: continue
        pts=P[lab==j]
        print(cam,f"red{j} area={stats[j,4]} px=({cent[j][0]:.0f},{cent[j][1]:.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
    vis=np.load(f"{cam}_bgr.npy").copy(); vis[red>0]=(0,255,0); cv2.imwrite(f"{cam}_red.png",vis)
EOF
python3 red.py

# openrua op 10
python3 tools/action/gripper_cmd.py 0.04 && ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 11
python3 cloud.py birdview >/dev/null && python3 cloud.py agentview >/dev/null && python3 seg2.py agentview 0.424 && echo --- && python3 seg2.py birdview 0.424 && python3 red.py | grep -v "z\[0.4"

# openrua op 12
ps aux | grep -v grep | grep -iE "bridge|python|ros|sim|mujoco" | head -20; echo ---; ls / ; ls /opt 2>/dev/null; ls ~ 2>/dev/null

# openrua op 13
ls /opt/ros /log /ros_entrypoint.sh 2>/dev/null | head; cat /ros_entrypoint.sh 2>/dev/null; pip list 2>/dev/null | grep -iE "libero|robosuite|mujoco|bddl"; python3 -c "import libero, os; print(os.path.dirname(libero.__file__))" 2>&1 | tail -1; find / -iname "*living_room*" -o -iname "*.bddl" 2>/dev/null | head

# openrua op 14
cat > ctl.py <<'EOF'
"""Reusable control helpers: IK (MoveIt), FK, trajectory, gripper, TF hand pose.
World<->base: base = panda_link0 at world (-0.51, 0, 0.42), identity rotation.
"""
import sys, time, numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = ARM["joints"]
TCP = M["hand"]["tcp_offset_m"]
BASE_W = np.array([-0.51, 0.0, 0.42])
DOWN = (1.0, 0.0, 0.0, 0.0)  # hand pointing down, fingers along world y

class Ctl:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("ctl")
        self.js = {}
        self.n.create_subscription(JointState, "/joint_states", self._js, 10)
        self.ik = self.n.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.n, FollowJointTrajectory, ARM["port"])
        self.grip = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.tf = Buffer(); TransformListener(self.tf, self.n)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.spin(0.5)

    def _js(self, m): self.js = dict(zip(m.name, m.position))
    def spin(self, t):
        end = time.time() + t
        while time.time() < end: rclpy.spin_once(self.n, timeout_sec=0.05)
    def joints(self):
        self.js = {}
        while not self.js: rclpy.spin_once(self.n, timeout_sec=0.1)
        return self.js
    def arm_q(self): j = self.joints(); return [j[k] for k in JOINTS]
    def fingers(self): j = self.joints(); return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def hand_world(self):
        """world pose of panda_hand from TF (fresh)."""
        self.spin(0.3)
        t = self.tf.lookup_transform("world", "panda_hand", rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), (q.x, q.y, q.z, q.w)
    def tcp_world(self):
        p, q = self.hand_world(); R = quat_R(*q); return p + TCP * R[:, 2], q

    def solve_ik(self, xyz_world, quat=DOWN, at_tcp=True, seed=None):
        xyz = np.array(xyz_world, float)
        if at_tcp: xyz = xyz - TCP * quat_R(*quat)[:, 2]
        b = xyz - BASE_W
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = M["planning"]["group"]; r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = b
        o = r.pose_stamped.pose.orientation; o.x, o.y, o.z, o.w = quat
        r.avoid_collisions = False
        r.timeout.sec = 2
        s = JointState(); s.name = list(JOINTS); s.position = list(seed if seed is not None else self.arm_q())
        r.robot_state.joint_state = s
        fut = self.ik.call_async(req); rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None, (None if res is None else res.error_code.val)
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS], 1

    def traj(self, q, secs, q_via=None):
        g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = list(JOINTS)
        pts = []
        if q_via is not None:
            for i, qv in enumerate(q_via):
                p = JointTrajectoryPoint(positions=list(qv)); t = secs * (i + 1) / (len(q_via) + 1)
                p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)); pts.append(p)
        p = JointTrajectoryPoint(positions=list(q)); p.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        pts.append(p); g.trajectory.points = pts
        f = self.fjt.send_goal_async(g); rclpy.spin_until_future_complete(self.n, f)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.n, rf)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        return code, err

    def move(self, xyz_world, quat=DOWN, secs=3.0, at_tcp=True, retries=2):
        q, code = self.solve_ik(xyz_world, quat, at_tcp)
        if q is None: print(f"IK FAILED code={code} for {xyz_world}"); return False
        for i in range(retries + 1):
            c, e = self.traj(q, secs)
            print(f"  traj code={c} maxerr={e:.4f}")
            if c == 0 and e < 0.02: break
        p, _ = self.tcp_world()
        print(f"  tcp now {p.round(4)} target {np.round(xyz_world,4)} d={np.linalg.norm(p-np.array(xyz_world)):.4f}")
        return True

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.n, f, timeout_sec=30)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.n, rf, timeout_sec=120)
        r = rf.result().result; self.spin(0.3)
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={np.round(self.fingers(),4)}")
        return r

def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

def yaw_down(yaw):
    """quaternion: hand pointing down, fingers rotated by yaw about world z from DOWN."""
    # DOWN = 180deg about x. compose Rz(yaw) * Rx(pi)
    cz, sz = np.cos(yaw / 2), np.sin(yaw / 2)
    # q_z = (0,0,sz,cz), q_x = (1,0,0,0); q = q_z * q_x
    return (cz, sz, 0.0, 0.0)  # (x,y,z,w) of Rz(yaw)*Rx(pi) => (cz*1, sz*1, 0, 0)... verify numerically in test
EOF
cat > t_fk.py <<'EOF'
from ctl import *
c = Ctl()
print("arm q", np.round(c.arm_q(),4)); print("fingers", c.fingers())
p,q = c.hand_world(); print("hand world", p.round(4), np.round(q,4))
t,_ = c.tcp_world(); print("tcp world", t.round(4))
# FK check in planner frame
req = GetPositionFK.Request(); req.fk_link_names=["panda_hand"]; s=JointState(); s.name=list(JOINTS); s.position=c.arm_q(); req.robot_state.joint_state=s
f=c.fk.call_async(req); rclpy.spin_until_future_complete(c.n,f,timeout_sec=30); r=f.result()
pp=r.pose_stamped[0].pose.position; print("FK panda_hand frame", r.pose_stamped[0].header.frame_id, round(pp.x,4),round(pp.y,4),round(pp.z,4), "code", r.error_code.val)
# IK sanity: solve for current tcp, expect near current joints
q,code=c.solve_ik(t, q, at_tcp=True); print("IK back to current:", code, None if q is None else np.round(np.array(q)-np.array(c.arm_q()),3))
print("yaw_down(0)", yaw_down(0), "R col2", quat_R(*yaw_down(0))[:,2], "R col1", quat_R(*yaw_down(0))[:,1])
print("yaw_down(pi/2) R col1", quat_R(*yaw_down(np.pi/2))[:,1].round(3), "col2", quat_R(*yaw_down(np.pi/2))[:,2].round(3))
EOF
python3 t_fk.py

# openrua op 15
cat > t_ik.py <<'EOF'
from ctl import *
import ctl
c = Ctl()
t,q = c.tcp_world()
ctl.BASE_W = np.zeros(3)
sol,code=c.solve_ik(t, q, at_tcp=True); print("IK (world) back to current:", code, None if sol is None else np.round(np.array(sol)-np.array(c.arm_q()),3))
sol,code=c.solve_ik(t, DOWN, at_tcp=True); print("IK DOWN:", code, None if sol is None else np.round(sol,3))
sol,code=c.solve_ik([-0.2255,0.035,0.70], DOWN, at_tcp=True); print("IK above mug:", code, None if sol is None else np.round(sol,3))
sol,code=c.solve_ik([0.131,0.035,0.70], DOWN, at_tcp=True); print("IK above plate:", code, None if sol is None else np.round(sol,3))
sol,code=c.solve_ik([0.13,0.15,0.445], DOWN, at_tcp=True); print("IK pudding goal low:", code, None if sol is None else np.round(sol,3))
EOF
python3 t_ik.py

# openrua op 16
cat > t_fk2.py <<'EOF'
from ctl import *
c = Ctl()
p,q = c.hand_world(); print("TF  hand", p.round(4), np.round(q,4))
for link in ["panda_hand","panda_link8","panda_leftfinger","panda_rightfinger"]:
    req = GetPositionFK.Request(); req.fk_link_names=[link]; s=JointState(); s.name=list(JOINTS); s.position=c.arm_q(); req.robot_state.joint_state=s
    f=c.fk.call_async(req); rclpy.spin_until_future_complete(c.n,f,timeout_sec=30); r=f.result()
    pp=r.pose_stamped[0].pose.position; o=r.pose_stamped[0].pose.orientation
    print("FK ", link, round(pp.x,4),round(pp.y,4),round(pp.z,4), "q", np.round([o.x,o.y,o.z,o.w],4))
for fr in ["panda_link8","panda_leftfinger","panda_rightfinger"]:
    t=c.tf.lookup_transform("world",fr,rclpy.time.Time()); tr=t.transform.translation; qq=t.transform.rotation
    print("TF ", fr, round(tr.x,4),round(tr.y,4),round(tr.z,4), "q", np.round([qq.x,qq.y,qq.z,qq.w],4))
EOF
python3 t_fk2.py

# openrua op 17
cat > s1.py <<'EOF'
from ctl import *
import ctl; ctl.BASE_W = np.zeros(3)
c = Ctl()
print("move above mug"); c.move([-0.2255,0.035,0.70], DOWN, secs=3.0)
p,q = c.hand_world(); R=quat_R(*q); print("hand q", np.round(q,4), "finger axis (hand y) in world", R[:,1].round(3), "approach", R[:,2].round(3))
print("joints", np.round(c.arm_q(),3))
EOF
python3 -u s1.py

# openrua op 18
python3 - <<'EOF'
import re
s=open("ctl.py").read()
s=s.replace("BASE_W = np.array([-0.51, 0.0, 0.42])","BASE_W = np.zeros(3)  # planner frame is world here (verified via FK)")
s=s.replace("        o = r.pose_stamped.pose.orientation; o.x, o.y, o.z, o.w = quat\n",
"        # IK tip link is panda_link8; panda_hand = link8 * Rz(-45deg) (verified). request link8 = hand * Rz(+45deg)\n        o = r.pose_stamped.pose.orientation; o.x, o.y, o.z, o.w = qmul(quat, (0.0, 0.0, np.sin(np.pi/8), np.cos(np.pi/8)))\n")
s=s.replace('''def yaw_down(yaw):
    """quaternion: hand pointing down, fingers rotated by yaw about world z from DOWN."""
    # DOWN = 180deg about x. compose Rz(yaw) * Rx(pi)
    cz, sz = np.cos(yaw / 2), np.sin(yaw / 2)
    # q_z = (0,0,sz,cz), q_x = (1,0,0,0); q = q_z * q_x
    return (cz, sz, 0.0, 0.0)  # (x,y,z,w) of Rz(yaw)*Rx(pi) => (cz*1, sz*1, 0, 0)... verify numerically in test
''','''def qmul(a, b):
    """quaternion product a*b, (x,y,z,w)."""
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + w2*x1 + y1*z2 - z1*y2,
            w1*y2 + w2*y1 + z1*x2 - x1*z2,
            w1*z2 + w2*z1 + x1*y2 - y1*x2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)

def yaw_down(yaw):
    """hand quaternion: pointing down, finger axis rotated by yaw about world z from DOWN (fingers along y)."""
    return qmul((0.0, 0.0, np.sin(yaw/2), np.cos(yaw/2)), DOWN)
''')
open("ctl.py","w").write(s)
EOF
cat > s1b.py <<'EOF'
from ctl import *
c = Ctl()
print(np.round(yaw_down(0),4), quat_R(*yaw_down(np.pi/2))[:,1].round(3))
print("re-orient above mug"); c.move([-0.2255,0.035,0.70], DOWN, secs=3.0)
p,q = c.hand_world(); R=quat_R(*q); print("hand q", np.round(q,4), "finger axis", R[:,1].round(3), "approach", R[:,2].round(3))
EOF
python3 -u s1b.py

# openrua op 19
python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 tools/perception/cam_snap.py agentview

# openrua op 20
cat > s2.py <<'EOF'
from ctl import *
from geometry_msgs.msg import WrenchStamped
c = Ctl()
w={}
c.n.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: w.update(f=(m.wrench.force.x,m.wrench.force.y,m.wrench.force.z)), 10)
def wr(): c.spin(0.4); return np.round(w.get("f",(0,0,0)),2)
print("wrench before", wr())
c.move([-0.2255,0.035,0.60], DOWN, secs=2.5); print("wrench", wr())
c.move([-0.2255,0.035,0.545], DOWN, secs=2.5); print("wrench", wr())
c.gripper(0.0)
print("wrench after close", wr())
EOF
python3 -u s2.py && python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 21
cat > s3.py <<'EOF'
from ctl import *
c = Ctl()
c.move([-0.2255,0.035,0.72], DOWN, secs=3.0)
print("fingers", np.round(c.fingers(),4))
EOF
python3 -u s3.py && python3 cloud.py agentview >/dev/null && python3 red.py | grep agentview && python3 tools/perception/cam_snap.py agentview

# openrua op 22
cat > s4.py <<'EOF'
from ctl import *
c = Ctl()
c.move([0.131,0.035,0.72], DOWN, secs=4.0); print("fingers", np.round(c.fingers(),4))
c.move([0.131,0.035,0.585], DOWN, secs=3.0); print("fingers", np.round(c.fingers(),4))
c.gripper(0.04)
c.move([0.131,0.035,0.75], DOWN, secs=3.0)
EOF
python3 -u s4.py && python3 cloud.py agentview >/dev/null && python3 cloud.py birdview >/dev/null && python3 seg2.py agentview 0.424 && python3 red.py | grep -v "z\[0.4[45]" && python3 tools/perception/cam_snap.py agentview

# openrua op 23
python3 seg2.py birdview 0.424; python3 - <<'EOF'
import numpy as np
P=np.load("birdview_xyz.npy"); z=P[...,2]
m=(z>0.50)&(z<0.60)&(P[...,0]>0.0)&(P[...,0]<0.3)   # mug rim region above plate
pts=P[m]; print("mug top pts x[%.3f,%.3f] y[%.3f,%.3f] n=%d"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),len(pts)))
m=(z>0.435)&(z<0.47)&(P[...,0]>0.0)&(P[...,0]<0.3)&(np.abs(P[...,1])<0.1)
pts=P[m]; print("plate pts x[%.3f,%.3f] y[%.3f,%.3f]"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max()))
EOF

# openrua op 24
cat > s5.py <<'EOF'
from ctl import *
c = Ctl()
c.move([-0.037,0.091,0.70], DOWN, secs=4.0)
EOF
python3 -u s5.py && python3 cloud.py birdview >/dev/null && python3 seg2.py birdview 0.424 && python3 - <<'EOF'
import numpy as np
P=np.load("birdview_xyz.npy"); z=P[...,2]
m=(z>0.50)&(z<0.62)&(P[...,0]>0.0)&(P[...,0]<0.3)
pts=P[m]; print("mug rim pts x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f n=%d -> center (%.3f, %.3f)"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].max(),len(pts),(pts[:,0].min()+pts[:,0].max())/2,(pts[:,1].min()+pts[:,1].max())/2))
m=(z>0.435)&(z<0.47)&(P[...,0]>0.0)&(P[...,0]<0.3)&(np.abs(P[...,1])<0.1)
pts=P[m]; print("plate pts x[%.3f,%.3f] y[%.3f,%.3f] -> center (%.3f, %.3f)"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),(pts[:,0].min()+pts[:,0].max())/2,(pts[:,1].min()+pts[:,1].max())/2))
EOF
python3 tools/perception/cam_snap.py robot0_eye_in_hand; python3 tools/perception/cam_snap.py birdview

# openrua op 25
cat > s6.py <<'EOF'
from ctl import *
c = Ctl()
c.move([-0.037,0.091,0.50], DOWN, secs=2.5)
c.move([-0.037,0.091,0.443], DOWN, secs=2.5)
c.gripper(0.0)
c.move([-0.037,0.091,0.65], DOWN, secs=3.0); print("fingers", np.round(c.fingers(),4))
EOF
python3 -u s6.py && python3 cloud.py agentview >/dev/null && python3 seg2.py agentview 0.424 && python3 tools/perception/cam_snap.py agentview

# openrua op 26
cat > s7.py <<'EOF'
from ctl import *
c = Ctl()
c.move([0.13,0.15,0.65], DOWN, secs=4.0); print("fingers", np.round(c.fingers(),4))
c.move([0.13,0.15,0.449], DOWN, secs=3.0); print("fingers", np.round(c.fingers(),4))
c.gripper(0.04)
c.move([0.13,0.15,0.62], DOWN, secs=3.0)
c.move([-0.15,0.0,0.75], DOWN, secs=4.0)   # park clear of the scene for verification
EOF
python3 -u s7.py && python3 cloud.py agentview >/dev/null && python3 cloud.py birdview >/dev/null && python3 seg2.py agentview 0.424 && echo --- && python3 seg2.py birdview 0.424 && python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py birdview
