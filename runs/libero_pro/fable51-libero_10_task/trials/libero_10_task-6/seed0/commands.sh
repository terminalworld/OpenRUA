#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace; echo ---; find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la .ros tools

# openrua op 2
ls -la tools/action tools/perception; cat tools/action/* tools/perception/*

# openrua op 3
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 5
timeout 30 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
end = time.time()+5
while time.time()<end: rclpy.spin_once(node, timeout_sec=0.1)
print(buf.all_frames_as_string())
for f in ["panda_link0","panda_hand","agentview_optical_frame","birdview_optical_frame","frontview_optical_frame","sideview_optical_frame","robot0_eye_in_hand_optical_frame"]:
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f, f"{tr.x:.4f} {tr.y:.4f} {tr.z:.4f}  q {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}")
    except Exception as e:
        print(f, "ERR", e)
rclpy.shutdown()
EOF
timeout 60 python3 tfdump.py

# openrua op 7
cat > cloud.py <<'EOF'
"""Grab color+depth+info from a camera, produce world-frame point cloud; save npz."""
import sys, time, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(node, topic, T, timeout=20):
    got={}
    sub=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    end=time.time()+timeout
    while "m" not in got and time.time()<end: rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return got["m"]

def main():
    cam=sys.argv[1]
    rclpy.init(); node=rclpy.create_node("cloud")
    buf=Buffer(); TransformListener(buf,node)
    br=CvBridge()
    col=br.imgmsg_to_cv2(grab(node,f"/{cam}/color/image_raw",Image),"bgr8")
    dep=br.imgmsg_to_cv2(grab(node,f"/{cam}/depth/image_raw",Image),"passthrough").astype(np.float32)
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    frame=f"{cam}_optical_frame"
    end=time.time()+10
    while time.time()<end and not buf.can_transform("world",frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.1)
    t=buf.lookup_transform("world",frame,rclpy.time.Time())
    q=t.transform.rotation; R=quat_R(q.x,q.y,q.z,q.w)
    p=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    h,w=dep.shape
    u,v=np.meshgrid(np.arange(w),np.arange(h))
    X=(u-cx)*dep/fx; Y=(v-cy)*dep/fy; Z=dep
    pc=np.stack([X,Y,Z],-1).reshape(-1,3)@R.T+p
    pc=pc.reshape(h,w,3)
    np.savez(f"{cam}_cloud.npz",pc=pc,col=col,dep=dep,K=np.array(info.k).reshape(3,3),R=R,p=p)
    print("saved", f"{cam}_cloud.npz", "depth range", np.nanmin(dep), np.nanmax(dep))
    print("optical axes in world: x(right)=",R[:,0].round(3)," y(down)=",R[:,1].round(3)," z(fwd)=",R[:,2].round(3))
    rclpy.shutdown()
main()
EOF
for c in agentview birdview; do timeout 90 python3 cloud.py $c; done

# openrua op 8
cat > seg.py <<'EOF'
import numpy as np, sys, cv2
from scipy import ndimage
cam=sys.argv[1]
d=np.load(f"{cam}_cloud.npz"); pc=d["pc"]; col=d["col"]
z=pc[...,2]
valid=np.isfinite(z)
# table height: mode of z in the central region
zz=z[valid]; hist,edges=np.histogram(zz,bins=400,range=(0,1.5))
table_z=edges[np.argmax(hist)]
print("table z ~",round(table_z,4))
mask=valid&(z>table_z+0.008)&(z<table_z+0.4)
# exclude robot: x < -0.25 roughly (robot base at -0.51) -- keep but label
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<40: continue
    P=pc[m]; C=col[m]
    xmin,ymin,zmin=P.min(0); xmax,ymax,zmax=P.max(0)
    bgr=C.mean(0)
    vs,us=np.nonzero(m)
    print(f"blob{i}: n={m.sum()} px(u,v)=({us.mean():.0f},{vs.mean():.0f}) x[{xmin:.3f},{xmax:.3f}] y[{ymin:.3f},{ymax:.3f}] z[{zmin:.3f},{zmax:.3f}] center=({(xmin+xmax)/2:.3f},{(ymin+ymax)/2:.3f}) meanBGR={bgr.round(0)}")
EOF
python3 seg.py agentview; echo ----; python3 seg.py birdview

# openrua op 9
cat > seg2.py <<'EOF'
import numpy as np, cv2
from scipy import ndimage
d=np.load("agentview_cloud.npz"); pc=d["pc"]; col=d["col"]
z=pc[...,2]
# table z near plate: pixels around plate but outside it
for (u,v) in [(334,430),(200,400),(450,400),(334,300),(100,250),(500,250)]:
    print("table px",(u,v),"->",pc[v,u].round(3))
# red mug: red-dominant pixels
b,g,r=col[...,0].astype(int),col[...,1].astype(int),col[...,2].astype(int)
red=(r>90)&(r>g+40)&(r>b+40)&np.isfinite(z)
lab,n=ndimage.label(red)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<100: continue
    P=pc[m]; vs,us=np.nonzero(m)
    print(f"red blob{i}: n={m.sum()} px=({us.mean():.0f},{vs.mean():.0f}) x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
# all points in the region of the red mug (x -0.35..0.0, y -0.1..0.1, z 0.43..0.65) excluding robot gray
reg=np.isfinite(z)&(pc[...,0]>-0.3)&(pc[...,0]<0.0)&(np.abs(pc[...,1])<0.12)&(z>0.44)&(z<0.62)
P=pc[reg]; print("region n",reg.sum(), "x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"z",P[:,2].min().round(3),P[:,2].max().round(3))
EOF
python3 seg2.py

# openrua op 10
cat > seg3.py <<'EOF'
import numpy as np
from scipy import ndimage
d=np.load("birdview_cloud.npz"); pc=d["pc"]; col=d["col"]; dep=d["dep"]
z=pc[...,2]
tbl=np.isfinite(z)&(np.abs(pc[...,0])<0.6)&(np.abs(pc[...,1])<0.8)&(z>0.3)&(z<0.46)
print("birdview table z median", np.median(z[tbl]).round(4), "n",tbl.sum())
mask=np.isfinite(z)&(np.abs(pc[...,0])<0.7)&(np.abs(pc[...,1])<0.9)&(z>0.44)&(z<1.5)
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<5: continue
    P=pc[m]; C=col[m]; vs,us=np.nonzero(m)
    print(f"blob{i}: n={m.sum()} px=({us.mean():.0f},{vs.mean():.0f}) x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}] ctr=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) BGR={C.mean(0).round(0)}")
# depth at plate center pixel
u,v=int(d["K"][0,2]+0.016*d["K"][0,0]/2.5),0
EOF
python3 seg3.py

# openrua op 11
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 30 ros2 topic echo /joint_states --once | grep -A9 position; timeout 90 python3 cloud.py birdview && python3 seg3.py

# openrua op 12
timeout 90 python3 cloud.py agentview >/dev/null && python3 seg.py agentview && python3 seg2.py

# openrua op 13
cat > seg4.py <<'EOF'
import numpy as np
from scipy import ndimage
d=np.load("agentview_cloud.npz"); pc=d["pc"]; col=d["col"]
z=pc[...,2]
reg=np.isfinite(z)&(pc[...,0]>-0.3)&(pc[...,0]<0.0)&(np.abs(pc[...,1])<0.15)&(z>0.44)&(z<0.60)
lab,n=ndimage.label(reg)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<50: continue
    P=pc[m]; C=col[m]; vs,us=np.nonzero(m)
    print(f"blob{i}: n={m.sum()} px=({us.min()}-{us.max()},{vs.min()}-{vs.max()}) x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}] BGR={C.mean(0).round(0)}")
    # body vs handle: histogram of y at mid height
    mid=m&(z>0.48)&(z<0.52)
    ys=pc[mid][:,1]; xs=pc[mid][:,0]
    print("   mid-height y range",ys.min().round(3),ys.max().round(3)," x range",xs.min().round(3),xs.max().round(3))
    # top rim
    top=m&(z>P[:,2].max()-0.01)
    print("   rim pts x",pc[top][:,0].min().round(3),pc[top][:,0].max().round(3)," y",pc[top][:,1].min().round(3),pc[top][:,1].max().round(3), " n",top.sum())
EOF
python3 seg4.py

# openrua op 14
cat > seg5.py <<'EOF'
import numpy as np
d=np.load("birdview_cloud.npz"); pc=d["pc"]; col=d["col"]
z=pc[...,2]
reg=np.isfinite(z)&(pc[...,0]>-0.32)&(pc[...,0]<-0.08)&(np.abs(pc[...,1])<0.15)&(z>0.44)&(z<0.62)
P=pc[reg]; C=col[reg]
print("n",reg.sum())
print("x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"z",P[:,2].min().round(3),P[:,2].max().round(3))
# print occupancy grid 1cm
xs=np.arange(-0.32,-0.08,0.01); ys=np.arange(-0.15,0.15,0.01)
H,_,_=np.histogram2d(P[:,0],P[:,1],bins=[xs,ys])
print("     y:", " ".join(f"{y*100:+.0f}"[-2:] for y in ys[:-1]))
for i,x in enumerate(xs[:-1]):
    print(f"x={x:+.2f} "+"".join(" #" if H[i,j]>0 else " ." for j in range(len(ys)-1)))
# top rim only (z>0.55)
rim=reg&(z>0.555)
R=pc[rim]; print("rim n",rim.sum(),"x",R[:,0].min().round(3),R[:,0].max().round(3),"y",R[:,1].min().round(3),R[:,1].max().round(3),"ctr",R[:,0].mean().round(3),R[:,1].mean().round(3))
EOF
python3 seg5.py

# openrua op 15
cat > fit.py <<'EOF'
import numpy as np
from scipy.optimize import least_squares
d=np.load("agentview_cloud.npz"); pc=d["pc"]; col=d["col"]; z=pc[...,2]
for zl,zh in [(0.46,0.50),(0.50,0.54),(0.54,0.57)]:
    m=np.isfinite(z)&(pc[...,0]>-0.3)&(pc[...,0]<0.0)&(np.abs(pc[...,1])<0.15)&(z>zl)&(z<zh)
    P=pc[m][:,:2]
    # exclude handle: keep y>-0.03 for fit
    Q=P[P[:,1]>-0.025]
    f=lambda c: np.hypot(Q[:,0]-c[0],Q[:,1]-c[1])-c[2]
    r=least_squares(f,[-0.2,0.01,0.045])
    print(f"z[{zl},{zh}] n={len(Q)} center=({r.x[0]:.3f},{r.x[1]:.3f}) radius={r.x[2]:.3f} rms={np.sqrt(np.mean(r.fun**2)):.4f}  handle-side y min {P[:,1].min():.3f}")
EOF
python3 fit.py

# openrua op 16
cat > robot.py <<'EOF'
"""Reusable control layer for this Panda (long-lived node, clients built once)."""
import time, math, yaml, numpy as np, rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, TwistStamped
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE = np.array([-0.51, 0.0, 0.42])   # panda_link0 in world (from TF)
TCP = M["hand"]["tcp_offset_m"]

class Robot:
    def __init__(self, name="ctl"):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None; self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 10)
        self.node.create_subscription(WrenchStamped, M["sensors"][1]["port"], self._wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)
        self.spin(0.5)
        while self.js is None: self.spin(0.2)

    def _js(self, m): self.js = m
    def _wr(self, m): self.wr = m
    def spin(self, t):
        end = time.time() + t
        while time.time() < end: rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---- sensing
    def q(self):
        self.spin(0.1)
        d = dict(zip(self.js.name, self.js.position))
        return np.array([d[j] for j in ARM])
    def fingers(self):
        self.spin(0.1)
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]
    def wrench(self):
        self.spin(0.1)
        if self.wr is None: return None
        f = self.wr.wrench.force; t = self.wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])
    def fk(self, q=None):
        """hand pose in WORLD: (pos xyz, quat xyzw)"""
        if q is None: q = self.q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1: raise RuntimeError(f"FK failed {r}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = Rot.from_quat(quat).as_matrix()
        return pos + TCP * R[:, 2], quat

    # ---- planning
    def ik(self, pos_world, quat, seed=None, at_tcp=True, tries=3):
        pos = np.array(pos_world, float)
        R = Rot.from_quat(quat).as_matrix()
        if at_tcp: pos = pos - TCP * R[:, 2]
        pb = pos - BASE
        if seed is None: seed = self.q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pb)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return np.array([sol[j] for j in ARM])
        return None

    # ---- acting
    def move(self, q_target, seconds=3.0, via=()):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = list(via) + [q_target]
        for i, q in enumerate(pts):
            t = seconds * (i + 1) / len(pts)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = np.abs(self.q() - np.array(q_target)).max()
        print(f"  move: code={code} max_joint_err={err:.4f}")
        return code, err
    def goto(self, pos_world, quat, seconds=3.0, seed=None, at_tcp=True):
        q = self.ik(pos_world, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            print("  IK FAILED for", np.round(pos_world, 3)); return None
        self.move(q, seconds)
        tp, _ = self.tcp()
        print(f"  tcp now {np.round(tp,4)} (target {np.round(pos_world,4)})")
        return q
    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        self.spin(0.3)
        print(f"  gripper: reached={r.reached_goal} stalled={r.stalled} fingers={np.round(self.fingers(),4)}")
        return r
    def servo(self, v, n=20, frame=None):
        """stream twist (linear m/s in base frame) for n ticks"""
        msg = TwistStamped(); msg.header.frame_id = frame or TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            self.tw_pub.publish(msg); rclpy.spin_once(self.node, timeout_sec=0.05)

def down_quat(yaw_deg=0.0):
    """hand z pointing down (world -z); yaw rotates fingers' closing axis: 0 -> fingers close along world y"""
    return (Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])).as_quat()
EOF
cat > t_fk.py <<'EOF'
from robot import *
r = Robot()
print("q", np.round(r.q(),4))
print("hand", r.fk()); print("tcp", r.tcp())
print("fingers", r.fingers()); print("wrench", r.wrench())
print("down_quat", np.round(down_quat(0),4), np.round(down_quat(90),4))
# test IK for pre-grasp above red mug rim (+y side)
q = r.ik([-0.196, 0.056, 0.70], down_quat(0))
print("IK pregrasp", None if q is None else np.round(q,3))
if q is not None: print("  fk check", r.tcp(q))
EOF
timeout 120 python3 t_fk.py

# openrua op 17
sed -i 's/^BASE = np.array(\[-0.51, 0.0, 0.42\])/BASE = np.array([0.0, 0.0, 0.0])/' robot.py
cat > t_ik.py <<'EOF'
from robot import *
r = Robot()
pos, quat = r.fk(); print("hand world", np.round(pos,4), np.round(quat,4))
q = r.ik(pos, quat, at_tcp=False); print("IK roundtrip", None if q is None else np.round(q,3), "current", np.round(r.q(),3))
for z in [0.75, 0.70, 0.66]:
    q = r.ik([-0.196, 0.056, z], down_quat(0))
    print("IK pregrasp z",z, None if q is None else np.round(q,3))
    if q is not None: print("  tcp check", np.round(r.tcp(q)[0],4))
q = r.ik([-0.057, 0.115, 0.60], down_quat(0)); print("IK above pudding", None if q is None else np.round(q,3))
q = r.ik([0.126, 0.056, 0.70], down_quat(0)); print("IK above plate", None if q is None else np.round(q,3))
q = r.ik([0.126, 0.17, 0.55], down_quat(0)); print("IK right of plate", None if q is None else np.round(q,3))
EOF
timeout 300 python3 t_ik.py

# openrua op 18
cat > t_ik2.py <<'EOF'
from robot import *
r = Robot()
def tilt_quat(yaw_deg, pitch_deg):
    # hand down, then pitch about world y (positive tilts the approach so the hand leans toward -x / robot side)
    return (Rot.from_euler("y", pitch_deg, degrees=True) * Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
seed = np.array([0.146,-0.686,0.031,-2.831,0.023,2.145,0.158])
for pitch in [0, 10, 20]:
    for z in [0.62, 0.58, 0.555, 0.53]:
        q = r.ik([-0.196, 0.056, z], tilt_quat(0, pitch), seed=seed)
        print(f"rim grasp pitch={pitch} z={z}:", None if q is None else np.round(q,3))
for pitch in [0, 10, 20]:
    for z in [0.50, 0.46, 0.445]:
        q = r.ik([-0.057, 0.115, z], tilt_quat(0, pitch), seed=seed)
        print(f"pudding pitch={pitch} z={z}:", None if q is None else np.round(q,3))
for z in [0.62, 0.59]:
    q = r.ik([0.126, 0.056, z], tilt_quat(0, 0), seed=seed); print(f"plate place z={z}:", None if q is None else np.round(q,3))
for y in [0.17, 0.16]:
    for z in [0.48, 0.45]:
        q = r.ik([0.126, y, z], tilt_quat(0, 0), seed=seed); print(f"pudding place y={y} z={z}:", None if q is None else np.round(q,3))
EOF
timeout 600 python3 t_ik2.py

# openrua op 19
cat > pick_mug.py <<'EOF'
from robot import *
r = Robot("pick")
def tq(pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
MUG = np.array([-0.196, 0.016]); RIM_Z = 0.575
G = np.array([MUG[0], MUG[1] + 0.040])        # gripper centre over the +y wall
print("wrench before", np.round(r.wrench(),2))
print("1. pre-grasp high")
q1 = r.goto([G[0], G[1], 0.72], tq(10), 3.0)
print("   wrench", np.round(r.wrench(),2))
print("2. descend to rim height +1cm (tips just above rim)")
q2 = r.goto([G[0], G[1], 0.595], tq(10), 2.5, seed=q1)
print("   wrench", np.round(r.wrench(),2))
print("3. descend into rim")
q3 = r.goto([G[0], G[1], 0.555], tq(10), 2.0, seed=q2)
print("   wrench", np.round(r.wrench(),2))
np.save("q_grasp.npy", q3)
EOF
timeout 900 python3 -u pick_mug.py 2>&1 | tee pick_mug.log

# openrua op 20
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 21
cat > lift.py <<'EOF'
from robot import *
r = Robot("lift")
def tq(pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
r.goto([-0.196, 0.056, 0.70], tq(10), 2.5)
print("wrench", np.round(r.wrench(),2))
EOF
timeout 300 python3 -u lift.py && timeout 90 python3 cloud.py agentview >/dev/null && python3 fit.py && timeout 90 python3 cloud.py birdview > /dev/null && python3 seg5.py | tail -3

# openrua op 22
cat > probe.py <<'EOF'
from robot import *
r = Robot("probe")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
C = np.array([-0.195, 0.016])
Q = tq(90, 0)
base_fz = r.wrench()[2]
q = r.goto([C[0], C[1], 0.62], Q, 3.0)
print("   wrench", np.round(r.wrench(),2))
for z in [0.585, 0.57, 0.555, 0.54, 0.525]:
    q = r.goto([C[0], C[1], z], Q, 1.5, seed=q)
    w = r.wrench(); tp,_ = r.tcp()
    print(f"   z target {z}: tcp z {tp[2]:.4f} dz={tp[2]-z:+.4f}  Fz={w[2]:.2f} (dF={w[2]-base_fz:+.2f})")
    if abs(tp[2]-z) > 0.006 or abs(w[2]-base_fz) > 4:
        print("   BLOCKED"); break
np.save("q_probe.npy", q)
EOF
timeout 900 python3 -u probe.py 2>&1 | tee probe.log

# openrua op 23
timeout 90 python3 cloud.py sideview > /dev/null; cat > gap.py <<'EOF'
import numpy as np
d=np.load("sideview_cloud.npz"); pc=d["pc"]; col=d["col"]; z=pc[...,2]
# points near the gripper fingers: around mug x, y, z 0.56-0.62
m=np.isfinite(z)&(np.abs(pc[...,0]+0.195)<0.12)&(np.abs(pc[...,1]-0.016)<0.06)&(z>0.545)&(z<0.60)
P=pc[m]; C=col[m]
print("n",m.sum())
# histogram along x at 5mm
xs=np.arange(-0.31,-0.08,0.005)
H,_=np.histogram(P[:,0],bins=xs)
for i,x in enumerate(xs[:-1]):
    sel=(P[:,0]>=x)&(P[:,0]<xs[i+1])
    if H[i]>0:
        print(f"x={x:+.3f} n={H[i]:4d} zmean={P[sel][:,2].mean():.3f} zmin={P[sel][:,2].min():.3f} BGR={C[sel].mean(0).round(0)}")
EOF
python3 gap.py

# openrua op 24
timeout 120 find / -xdev \( -iname "*coffee_mug*" -o -iname "*libero*" -o -iname "*chocolate_pudding*" \) 2>/dev/null | head -20; echo ---; timeout 60 find / -xdev -iname "*.xml" -path "*mug*" 2>/dev/null | head; pip list 2>/dev/null | grep -i -E "libero|robosuite|mujoco"; ps aux | grep -v grep | grep -i -E "sim|mujoco|robosuite|libero|bridge" | head

# openrua op 25
timeout 30 ros2 node info /robot_bridge 2>&1 | head -60; timeout 20 ros2 param list /robot_bridge 2>&1 | head -30

# openrua op 26
timeout 90 python3 cloud.py agentview >/dev/null; cat > handle.py <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); pc=d["pc"]; col=d["col"]; z=pc[...,2]
# handle region: -y side of the mug, beyond the body wall (y < 0.016-0.040-0.003)
m=np.isfinite(z)&(pc[...,0]>-0.28)&(pc[...,0]<-0.12)&(pc[...,1]<-0.028)&(pc[...,1]>-0.10)&(z>0.43)&(z<0.60)
P=pc[m]; print("handle pts",m.sum())
print("x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"z",P[:,2].min().round(3),P[:,2].max().round(3))
for zl in np.arange(0.43,0.60,0.01):
    s=(P[:,2]>=zl)&(P[:,2]<zl+0.01)
    if s.sum(): print(f"z {zl:.2f}: n={s.sum():3d} x[{P[s][:,0].min():.3f},{P[s][:,0].max():.3f}] y[{P[s][:,1].min():.3f},{P[s][:,1].max():.3f}]")
EOF
python3 handle.py

# openrua op 27
timeout 90 python3 cloud.py robot0_eye_in_hand; cat > wristgap.py <<'EOF'
import numpy as np
d=np.load("robot0_eye_in_hand_cloud.npz"); pc=d["pc"]; col=d["col"]; dep=d["dep"]; z=pc[...,2]
print("depth range", np.nanmin(dep), np.nanmax(dep), "K", d["K"][0,0], d["K"][0,2], d["K"][1,2])
m=np.isfinite(z)&(z>0.50)&(z<0.70)&(np.abs(pc[...,1]-0.016)<0.08)&(pc[...,0]>-0.32)&(pc[...,0]<-0.06)
P=pc[m]; C=col[m]
gray=(np.abs(C[:,0].astype(int)-C[:,2].astype(int))<12)&(C[:,2]<120)
G=P[gray]; R=P[~gray]
print("gray n",gray.sum(),"other n",(~gray).sum())
xs=np.arange(-0.30,-0.08,0.005)
for i in range(len(xs)-1):
    g=G[(G[:,0]>=xs[i])&(G[:,0]<xs[i+1])]; o=R[(R[:,0]>=xs[i])&(R[:,0]<xs[i+1])]
    if len(g)+len(o)>3:
        print(f"x={xs[i]:+.3f} gray n={len(g):4d} z[{g[:,2].min() if len(g) else 0:.3f},{g[:,2].max() if len(g) else 0:.3f}] | other n={len(o):4d} z[{o[:,2].min() if len(o) else 0:.3f},{o[:,2].max() if len(o) else 0:.3f}] y[{o[:,1].min() if len(o) else 0:.3f},{o[:,1].max() if len(o) else 0:.3f}]")
EOF
python3 wristgap.py

# openrua op 28
python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand_cloud.npz'); col=d['col'].copy(); pc=d['pc']
for u in range(0,640,80): cv2.line(col,(u,0),(u,479),(0,255,0),1); cv2.putText(col,str(u),(u+2,12),0,0.4,(0,255,0),1)
for v in range(0,480,80): cv2.line(col,(0,v),(639,v),(0,255,0),1); cv2.putText(col,str(v),(2,v+12),0,0.4,(0,255,0),1)
cv2.imwrite('wrist_grid.png',col)
"

# openrua op 29
cat > look_handle.py <<'EOF'
from robot import *
r = Robot("look")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
q = r.goto([-0.195, 0.016, 0.66], tq(90,0), 2.0)
q = r.goto([-0.19, -0.05, 0.72], tq(90,0), 2.5, seed=q)
print("wrench", np.round(r.wrench(),2))
EOF
timeout 300 python3 -u look_handle.py && timeout 90 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand_cloud.npz'); col=d['col'].copy()
for u in range(0,640,80): cv2.line(col,(u,0),(u,479),(0,255,0),1); cv2.putText(col,str(u),(u+2,12),0,0.4,(0,255,0),1)
for v in range(0,480,80): cv2.line(col,(0,v),(639,v),(0,255,0),1); cv2.putText(col,str(v),(2,v+12),0,0.4,(0,255,0),1)
cv2.imwrite('wrist_grid.png',col)"

# openrua op 30
cat > handle2.py <<'EOF'
import numpy as np
d=np.load("robot0_eye_in_hand_cloud.npz"); pc=d["pc"]; col=d["col"]; z=pc[...,2]
m=np.isfinite(z)&(pc[...,0]>-0.26)&(pc[...,0]<-0.13)&(pc[...,1]<-0.026)&(pc[...,1]>-0.12)&(z>0.44)&(z<0.60)
P=pc[m]; C=col[m]; print("pts",m.sum())
print("x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"z",P[:,2].min().round(3),P[:,2].max().round(3))
# top-down occupancy 2.5mm in x,y for z>0.50 (handle top region)
for zl,zh in [(0.54,0.60),(0.50,0.54),(0.46,0.50)]:
    s=(P[:,2]>=zl)&(P[:,2]<zh); Q=P[s]
    if not len(Q): continue
    print(f"--- z[{zl},{zh}] n={len(Q)} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}]")
    xs=np.arange(-0.24,-0.14,0.005); ys=np.arange(-0.10,-0.02,0.005)
    H,_,_=np.histogram2d(Q[:,0],Q[:,1],bins=[xs,ys])
    print("        y:"+"".join(f"{abs(y)*1000:3.0f}"[-3:] for y in ys[:-1]))
    for i,x in enumerate(xs[:-1]): print(f"x={x:+.3f} "+"".join("  #" if H[i,j]>0 else "  ." for j in range(len(ys)-1)))
# body wall position on -y side at z 0.50-0.54: min y of body points near x=-0.195
b=np.isfinite(z)&(np.abs(pc[...,0]+0.195)<0.01)&(z>0.50)&(z<0.56)&(pc[...,1]<0.0)
print("body/handle pts near x=-0.195, y values sorted sample:", np.unique(np.round(pc[b][:,1],3))[:40])
EOF
python3 handle2.py

# openrua op 31
timeout 90 python3 cloud.py sideview >/dev/null; timeout 90 python3 cloud.py agentview >/dev/null; cat > hand_dims.py <<'EOF'
import numpy as np
for cam in ["sideview","agentview"]:
    d=np.load(f"{cam}_cloud.npz"); pc=d["pc"]; z=pc[...,2]
    # hand frame at tcp(-0.19,-0.05,0.72)+0.1034 up = z 0.823, yaw 90 -> fingers along x
    m=np.isfinite(z)&(np.abs(pc[...,0]+0.19)<0.2)&(np.abs(pc[...,1]+0.05)<0.15)&(z>0.72)&(z<0.95)
    P=pc[m]
    print(cam, "n", len(P))
    for zl in np.arange(0.72,0.95,0.01):
        s=(P[:,2]>=zl)&(P[:,2]<zl+0.01)
        if s.sum()>3: print(f"  z {zl:.2f}: n={s.sum():4d} x[{P[s][:,0].min():.3f},{P[s][:,0].max():.3f}] w={P[s][:,0].max()-P[s][:,0].min():.3f}  y[{P[s][:,1].min():.3f},{P[s][:,1].max():.3f}] w={P[s][:,1].max()-P[s][:,1].min():.3f}")
EOF
python3 hand_dims.py

# openrua op 32
cat > hand_dims2.py <<'EOF'
import numpy as np
for cam in ["sideview","agentview"]:
    d=np.load(f"{cam}_cloud.npz"); pc=d["pc"]; z=pc[...,2]; col=d["col"]
    m=np.isfinite(z)&(np.abs(pc[...,0]+0.19)<0.2)&(np.abs(pc[...,1]+0.05)<0.15)&(z>0.72)&(z<0.755)
    P=pc[m]
    xs=np.arange(-0.26,-0.12,0.01); ys=np.arange(-0.13,0.05,0.01)
    H,_,_=np.histogram2d(P[:,0],P[:,1],bins=[xs,ys])
    print(cam, "finger slice z 0.72-0.755  (rows x, cols y)")
    print("        y:"+"".join(f"{y*100:+4.0f}" for y in ys[:-1]))
    for i,x in enumerate(xs[:-1]): print(f"x={x:+.2f} "+"".join(f"{int(H[i,j]):4d}" if H[i,j]>0 else "   ." for j in range(len(ys)-1)))
EOF
python3 hand_dims2.py

# openrua op 33
cat > t_orient.py <<'EOF'
from robot import *
r = Robot("orient")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
pos, quat = r.fk()
print("current hand pos", np.round(pos,4), "quat", np.round(quat,4))
print("requested quat", np.round(tq(90,0),4))
R = Rot.from_quat(quat).as_matrix()
print("hand axes in world: x=",R[:,0].round(3)," y=",R[:,1].round(3)," z=",R[:,2].round(3))
Rr = Rot.from_quat(tq(90,0)).as_matrix()
print("requested axes:     x=",Rr[:,0].round(3)," y=",Rr[:,1].round(3)," z=",Rr[:,2].round(3))
print("angle between:", np.degrees((Rot.from_quat(quat).inv()*Rot.from_quat(tq(90,0))).magnitude()).round(2))
print("q", np.round(r.q(),4))
# also check link8 vs hand: FK for panda_link8
from moveit_msgs.srv import GetPositionFK
req = GetPositionFK.Request(); req.fk_link_names=["panda_link8","panda_hand","panda_leftfinger","panda_rightfinger"]
req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=[float(v) for v in r.q()]
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
for n,ps in zip(fut.result().fk_link_names, fut.result().pose_stamped):
    p=ps.pose.position; o=ps.pose.orientation
    print(n, np.round([p.x,p.y,p.z],4), np.round([o.x,o.y,o.z,o.w],4))
EOF
timeout 120 python3 t_orient.py

# openrua op 34
python3 - <<'EOF'
import re
s=open("robot.py").read()
s=s.replace('req.ik_request.group_name = M["planning"]["group"]\n', 'req.ik_request.group_name = M["planning"]["group"]\n            req.ik_request.ik_link_name = "panda_hand"\n')
open("robot.py","w").write(s)
EOF
grep -n "ik_link_name" robot.py
cat > t_orient2.py <<'EOF'
from robot import *
r = Robot("orient2")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
for yaw in [0, 90]:
    q = r.ik([-0.19, -0.05, 0.72], tq(yaw, 0))
    if q is None: print("IK fail"); continue
    pos, quat = r.fk(q); R = Rot.from_quat(quat).as_matrix()
    print(f"yaw {yaw}: hand y axis (finger closing dir) in world:", R[:,1].round(3), " z:", R[:,2].round(3), " tcp", np.round(r.tcp(q)[0],4))
EOF
timeout 120 python3 t_orient2.py

# openrua op 35
cat > rim2.py <<'EOF'
from robot import *
r = Robot("rim2")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
MUG = np.array([-0.195, 0.016]); G = np.array([MUG[0], MUG[1] + 0.040])
base_fz = r.wrench()[2]
Q = tq(0, 0)
q = r.ik([G[0], G[1], 0.62], Q)
if q is None:
    print("pitch0 infeasible, using pitch 10"); Q = tq(0, 10); q = r.ik([G[0], G[1], 0.62], Q)
q = r.goto([G[0], G[1], 0.62], Q, 3.0)
pos, quat = r.fk(); R = Rot.from_quat(quat).as_matrix(); print("   finger axis", R[:,1].round(3))
for z in [0.59, 0.575, 0.56, 0.545]:
    q = r.goto([G[0], G[1], z], Q, 1.5, seed=q)
    w = r.wrench(); tp,_ = r.tcp()
    print(f"   z target {z}: tcp {np.round(tp,4)} dz={tp[2]-z:+.4f}  Fz={w[2]:.2f} (dF={w[2]-base_fz:+.2f})")
    if abs(tp[2]-z) > 0.006 or abs(w[2]-base_fz) > 4:
        print("   BLOCKED"); break
np.save("q_rim.npy", q)
EOF
timeout 900 python3 -u rim2.py 2>&1 | tee rim2.log

# openrua op 36
cat > t_settle.py <<'EOF'
from robot import *
import time
r = Robot("settle")
q0 = r.q(); print("q now", np.round(q0,3), "tcp", np.round(r.tcp()[0],4))
# small move up 3cm at same orientation, then watch joint state vs wall time
pos, quat = r.fk()
qt = r.ik(pos + [0,0,0.03], quat, at_tcp=False, seed=q0)
t0 = time.time()
code, err = r.move(qt, 1.5)
print(f"result at {time.time()-t0:.2f}s code={code} err={err:.4f}")
for i in range(30):
    r.spin(0.2)
    e = np.abs(r.q()-qt).max()
    print(f"  t={time.time()-t0:5.2f}s err={e:.4f} clock={r.node.get_clock().now().nanoseconds/1e9:.3f}")
    if e < 0.003: break
print("tcp", np.round(r.tcp()[0],4))
EOF
timeout 300 python3 -u t_settle.py

# openrua op 37
python3 - <<'EOF'
s=open("robot.py").read()
old='''        code = rf.result().result.error_code
        err = np.abs(self.q() - np.array(q_target)).max()
        print(f"  move: code={code} max_joint_err={err:.4f}")
        return code, err'''
new='''        code = rf.result().result.error_code
        err = self.settle(q_target)
        print(f"  move: code={code} max_joint_err={err:.4f}")
        return code, err
    def settle(self, q_target, tol=0.004, budget=40.0):
        """poll joint state until converged to q_target or stalled (no progress for ~2s)"""
        import time
        t0 = time.time(); last = None; still = 0
        while time.time() - t0 < budget:
            e = np.abs(self.q() - np.array(q_target)).max()
            if e < tol: return e
            if last is not None and abs(last - e) < 1e-4:
                still += 1
                if still >= 8: return e   # stalled
            else: still = 0
            last = e; self.spin(0.25)
        return e'''
assert old in s
open("robot.py","w").write(s.replace(old,new))
EOF
cat > rim3.py <<'EOF'
from robot import *
r = Robot("rim3")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
def axes():
    pos, quat = r.fk(); R = Rot.from_quat(quat).as_matrix(); return R[:,1].round(3), R[:,2].round(3)
MUG = np.array([-0.195, 0.016]); G = np.array([MUG[0], MUG[1] + 0.040])
Q = tq(0, 0)
print("finger axis now", axes())
q = r.goto([G[0], G[1], 0.62], Q, 4.0)
print("   finger axis, approach:", axes(), " fingers", np.round(r.fingers(),4))
base_fz = r.wrench()[2]
for z in [0.59, 0.575, 0.565, 0.555, 0.545]:
    q = r.goto([G[0], G[1], z], Q, 1.5, seed=q)
    w = r.wrench(); tp,_ = r.tcp()
    print(f"   z target {z}: tcp {np.round(tp,4)} dz={tp[2]-z:+.4f}  Fz={w[2]:.2f} (dF={w[2]-base_fz:+.2f}) axis {axes()[0]}")
    if abs(tp[2]-z) > 0.006 or abs(w[2]-base_fz) > 4:
        print("   BLOCKED"); break
np.save("q_rim.npy", q)
EOF
timeout 900 python3 -u rim3.py 2>&1 | tee rim3.log

# openrua op 38
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand wrist_rim.png

# openrua op 39
cat > grasp_mug.py <<'EOF'
from robot import *
r = Robot("grasp")
print("fingers before", np.round(r.fingers(),4), "wrench", np.round(r.wrench(),2))
r.gripper(0.0)
f = r.fingers(); print("fingers after close", np.round(f,4), " gap=", round(abs(f[0])+abs(f[1]),4))
print("wrench", np.round(r.wrench(),2))
EOF
timeout 300 python3 -u grasp_mug.py

# openrua op 40
cat > lift_mug.py <<'EOF'
from robot import *
r = Robot("liftmug")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
Q = tq(0,0); G = [-0.195, 0.056]
q = r.goto([G[0], G[1], 0.60], Q, 2.0)
print("fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench(),2))
q = r.goto([G[0], G[1], 0.72], Q, 2.5, seed=q)
print("fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench(),2))
EOF
timeout 300 python3 -u lift_mug.py && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 41
cat > place_mug.py <<'EOF'
from robot import *
r = Robot("placemug")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
Q = tq(0,0)
PLATE = np.array([0.126, 0.016]); G = PLATE + [0, 0.040]
q = r.goto([G[0], G[1], 0.72], Q, 4.0)
print("fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench(),2))
base_fz = r.wrench()[2]
for z in [0.64, 0.60, 0.585, 0.575, 0.565]:
    q = r.goto([G[0], G[1], z], Q, 1.5, seed=q)
    w = r.wrench(); tp,_ = r.tcp()
    print(f"   z {z}: tcp {np.round(tp,4)} dz={tp[2]-z:+.4f} Fz={w[2]:.2f} dF={w[2]-base_fz:+.2f}")
    if abs(tp[2]-z) > 0.005 or (w[2]-base_fz) > 3:
        print("   touchdown"); break
r.gripper(0.04)
print("fingers", np.round(r.fingers(),4))
q = r.goto([G[0], G[1], 0.70], Q, 2.5, seed=q)
print("wrench", np.round(r.wrench(),2))
EOF
timeout 900 python3 -u place_mug.py 2>&1 | tee place_mug.log && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 42
timeout 90 python3 cloud.py agentview >/dev/null; timeout 90 python3 cloud.py birdview >/dev/null; cat > check_mug.py <<'EOF'
import numpy as np
from scipy.optimize import least_squares
d=np.load("agentview_cloud.npz"); pc=d["pc"]; col=d["col"]; z=pc[...,2]
# red mug region near plate
reg=np.isfinite(z)&(np.abs(pc[...,0]-0.126)<0.12)&(np.abs(pc[...,1]-0.016)<0.12)&(z>0.47)
P=pc[reg]; print("mug pts",len(P),"z top",P[:,2].max().round(4))
for zl,zh in [(0.50,0.54),(0.54,0.58),(0.58,0.62)]:
    s=(P[:,2]>zl)&(P[:,2]<zh); Q=P[s][:,:2]
    if len(Q)<30: continue
    f=lambda c: np.hypot(Q[:,0]-c[0],Q[:,1]-c[1])-c[2]
    r=least_squares(f,[0.126,0.016,0.04])
    print(f"z[{zl},{zh}] n={len(Q)} center=({r.x[0]:.3f},{r.x[1]:.3f}) r={r.x[2]:.3f} rms={np.sqrt(np.mean(r.fun**2)):.4f}")
b=np.load("birdview_cloud.npz"); pcb=b["pc"]; zb=pcb[...,2]
m=np.isfinite(zb)&(np.abs(pcb[...,0]-0.126)<0.12)&(np.abs(pcb[...,1]-0.016)<0.12)&(zb>0.50)
B=pcb[m]; print("birdview mug: n",len(B),"x",B[:,0].min().round(3),B[:,0].max().round(3),"y",B[:,1].min().round(3),B[:,1].max().round(3),"z max",B[:,2].max().round(3), "rim ctr", B[B[:,2]>B[:,2].max()-0.015][:,:2].mean(0).round(3))
# plate check
m2=np.isfinite(zb)&(np.abs(pcb[...,0]-0.126)<0.12)&(np.abs(pcb[...,1]-0.016)<0.12)&(zb>0.44)&(zb<0.47)
Pl=pcb[m2]; print("plate pts x",Pl[:,0].min().round(3),Pl[:,0].max().round(3),"y",Pl[:,1].min().round(3),Pl[:,1].max().round(3),"ctr",((Pl[:,0].min()+Pl[:,0].max())/2).round(3),((Pl[:,1].min()+Pl[:,1].max())/2).round(3))
EOF
python3 check_mug.py

# openrua op 43
cat > pudding.py <<'EOF'
import numpy as np
for cam in ["agentview","birdview"]:
    d=np.load(f"{cam}_cloud.npz"); pc=d["pc"]; z=pc[...,2]
    m=np.isfinite(z)&(pc[...,0]>-0.13)&(pc[...,0]<0.0)&(pc[...,1]>0.06)&(pc[...,1]<0.18)&(z>0.445)&(z<0.50)
    P=pc[m]; 
    if len(P)<10: print(cam,"few pts",len(P)); continue
    top=P[P[:,2]>P[:,2].max()-0.008]
    c=top[:,:2].mean(0); X=top[:,:2]-c
    w,v=np.linalg.eigh(X.T@X/len(X))
    ax=v[:,1]; ang=np.degrees(np.arctan2(ax[1],ax[0]))
    proj=X@v
    print(f"{cam}: n={len(P)} top z={P[:,2].max():.3f} center=({c[0]:.3f},{c[1]:.3f}) long-axis angle={ang:.1f}deg  extents long={proj[:,1].max()-proj[:,1].min():.3f} short={proj[:,0].max()-proj[:,0].min():.3f}  zmin={P[:,2].min():.3f}")
EOF
python3 pudding.py

# openrua op 44
python3 - <<'EOF'
s=open("robot.py").read()
old='''    def move(self, q_target, seconds=3.0, via=()):
        goal = FollowJointTrajectory.Goal()'''
new='''    def move(self, q_target, seconds=3.0, via=(), retries=2):
        for i in range(retries + 1):
            code, err = self._move_once(q_target, seconds, via)
            if code == 0 and err < 0.01: return code, err
            print(f"  (retry {i+1}: code={code} err={err:.4f})"); via = (); seconds = max(seconds, 3.0)
        return code, err
    def _move_once(self, q_target, seconds, via):
        goal = FollowJointTrajectory.Goal()'''
assert old in s; s=s.replace(old,new)
open("robot.py","w").write(s)
EOF
cat > pick_pudding.py <<'EOF'
from robot import *
r = Robot("pickpud")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
Q = tq(0,0); BOX = np.array([-0.054, 0.114])
print("fingers", np.round(r.fingers(),4))
q = r.goto([BOX[0], BOX[1], 0.62], Q, 5.0)
pos, quat = r.fk(); print("   finger axis", Rot.from_quat(quat).as_matrix()[:,1].round(3))
base_fz = r.wrench()[2]
for z in [0.52, 0.48, 0.446]:
    q = r.goto([BOX[0], BOX[1], z], Q, 2.0, seed=q)
    w = r.wrench(); tp,_ = r.tcp()
    print(f"   z {z}: tcp {np.round(tp,4)} dz={tp[2]-z:+.4f} Fz={w[2]:.2f} dF={w[2]-base_fz:+.2f}")
    if abs(tp[2]-z) > 0.006 or abs(w[2]-base_fz) > 4: print("   BLOCKED"); break
np.save("q_pud.npy", q)
EOF
timeout 900 python3 -u pick_pudding.py 2>&1 | tee pick_pudding.log && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand wrist_pud.png

# openrua op 45
cat > grasp_pud.py <<'EOF'
from robot import *
r = Robot("grasppud")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
Q = tq(0,0); BOX = np.array([-0.054, 0.114])
r.gripper(0.0)
f = r.fingers(); print("gap", round(abs(f[0])+abs(f[1]),4))
q = r.goto([BOX[0], BOX[1], 0.52], Q, 2.0)
print("fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench(),2))
q = r.goto([BOX[0], BOX[1], 0.66], Q, 2.5, seed=q)
print("fingers", np.round(r.fingers(),4), "wrench", np.round(r.wrench(),2))
EOF
timeout 600 python3 -u grasp_pud.py && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 46
cat > place_pud.py <<'EOF'
from robot import *
r = Robot("placepud")
def tq(yaw, pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
Q = tq(0,0); T = np.array([0.126, 0.17])
q = r.goto([T[0], T[1], 0.66], Q, 5.0)
print("fingers", np.round(r.fingers(),4))
base_fz = r.wrench()[2]
for z in [0.55, 0.49, 0.465, 0.452]:
    q = r.goto([T[0], T[1], z], Q, 2.0, seed=q)
    w = r.wrench(); tp,_ = r.tcp()
    print(f"   z {z}: tcp {np.round(tp,4)} dz={tp[2]-z:+.4f} Fz={w[2]:.2f} dF={w[2]-base_fz:+.2f}")
    if abs(tp[2]-z) > 0.005 or (w[2]-base_fz) > 3: print("   touchdown"); break
r.gripper(0.04)
print("fingers", np.round(r.fingers(),4))
q = r.goto([T[0], T[1], 0.62], Q, 2.5, seed=q)
# park the arm up and away so cameras see the scene
q = r.goto([-0.05, 0.0, 0.80], Q, 4.0, seed=q)
print("wrench", np.round(r.wrench(),2))
EOF
timeout 900 python3 -u place_pud.py 2>&1 | tee place_pud.log && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 47
timeout 90 python3 cloud.py agentview >/dev/null; timeout 90 python3 cloud.py birdview >/dev/null; cat > final_check.py <<'EOF'
import numpy as np
from scipy.optimize import least_squares
from scipy import ndimage
d=np.load("agentview_cloud.npz"); pc=d["pc"]; col=d["col"]; z=pc[...,2]
# table height
print("table z:", np.round(np.median(z[np.isfinite(z)&(np.abs(pc[...,0]-0.1)<0.05)&(np.abs(pc[...,1]+0.3)<0.05)]),4))
# plate (flat, z 0.44-0.46, light color) footprint
pl=np.isfinite(z)&(z>0.440)&(z<0.462)&(np.abs(pc[...,0]-0.126)<0.12)&(np.abs(pc[...,1]-0.016)<0.12)&(col[...,0]>120)
P=pc[pl]; print(f"plate: n={len(P)} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] -> center ({(P[:,0].min()+P[:,0].max())/2:.3f},{(P[:,1].min()+P[:,1].max())/2:.3f}) radius~{(P[:,1].max()-P[:,1].min())/2:.3f}")
plate_c=np.array([(P[:,0].min()+P[:,0].max())/2,(P[:,1].min()+P[:,1].max())/2]); plate_r=(P[:,1].max()-P[:,1].min())/2
# red mug: circle fits at several heights
reg=np.isfinite(z)&(np.abs(pc[...,0]-0.126)<0.12)&(np.abs(pc[...,1]-0.016)<0.10)&(z>0.47)&(z<0.65)
M=pc[reg]; print("mug top z", M[:,2].max().round(4), " mug bottom-ish z", M[:,2].min().round(4))
for zl,zh in [(0.48,0.52),(0.52,0.56),(0.56,0.60)]:
    s=(M[:,2]>zl)&(M[:,2]<zh); Q=M[s][:,:2]
    f=lambda c: np.hypot(Q[:,0]-c[0],Q[:,1]-c[1])-c[2]
    r=least_squares(f,[0.126,0.02,0.04])
    print(f"  mug z[{zl},{zh}] center=({r.x[0]:.3f},{r.x[1]:.3f}) r={r.x[2]:.3f} rms={np.sqrt(np.mean(r.fun**2)):.4f}  dist to plate center={np.hypot(*(r.x[:2]-plate_c)):.3f} (plate r {plate_r:.3f})")
# pudding
pd=np.isfinite(z)&(z>0.44)&(z<0.48)&(pc[...,0]>0.02)&(pc[...,0]<0.25)&(pc[...,1]>0.10)&(pc[...,1]<0.30)
B=pc[pd]; print(f"pudding: n={len(B)} x[{B[:,0].min():.3f},{B[:,0].max():.3f}] y[{B[:,1].min():.3f},{B[:,1].max():.3f}] top z={B[:,2].max():.3f} center=({(B[:,0].min()+B[:,0].max())/2:.3f},{(B[:,1].min()+B[:,1].max())/2:.3f})")
print(f"pudding y-min {B[:,1].min():.3f} vs plate y-max {P[:,1].max():.3f} -> gap {B[:,1].min()-P[:,1].max():.3f} m; pudding is at +y (agentview image-right) of plate")
EOF
python3 final_check.py

# openrua op 48
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-gotchas.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-gotchas.md <<'OPENRUA_EOF'
---
name: panda-sim-gotchas
description: Hard-won facts about the /workspace Franka Panda sim robot (MoveIt IK frame/link, FJT lag, gripper vs object sizes) not stated in the docs
metadata:
  type: project
---

Facts learned on the /workspace Panda (LIBERO-style tabletop sim, robot_bridge node), Sept 2026:

- `/compute_ik` and `/compute_fk` work in WORLD coordinates here (FK output matches TF world poses; do NOT subtract the panda_link0 offset), despite docs saying planning frame = arm base.
- `/compute_ik` solves for `panda_link8` by default, which is rotated 45° about z from `panda_hand`. Set `ik_request.ik_link_name = "panda_hand"` or every commanded yaw is 45° off (fingers end up on a diagonal).
- With hand quat (1,0,0,0) and ik_link_name=panda_hand, fingers close along world y; yaw +90° → along x.
- FollowJointTrajectory can return error_code -5 while the arm is still >1 rad from target; the paused sim then leaves the arm short. Re-send the goal (it converges) and poll /joint_states until settled before trusting any contact/force check.
- Gripper max gap measured ≈7.75 cm (finger 0.0387). LIBERO red mug body is ≈8 cm → body grasp impossible; the mug IS hollow, so a rim grasp (one finger inside, close radially on the wall, ~1 cm gap when closed) works and hangs the mug upright. Chocolate pudding box is 8×4.8×4 cm, easy pinch.
- Agentview camera image-right = world +y; "right of the plate" was placed at +y.
- Objects float ~6 cm above the table until the first command advances the sim; re-measure after any motion.

**Why:** each of these cost multiple failed attempts to discover.
**How to apply:** use the `/workspace/robot.py` helper (Robot class: ik/goto/move with settle+retry, gripper, wrench) if it still exists; otherwise rebuild with these settings.
OPENRUA_EOF

# openrua op 49
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md || touch MEMORY.md) && grep -q panda-sim-gotchas MEMORY.md || echo "- [Panda sim gotchas](panda-sim-gotchas.md) — IK link must be panda_hand, FK/IK in world frame, FJT -5 lag, 7.75 cm gripper vs 8 cm mug → rim grasp" >> MEMORY.md; cat MEMORY.md
