#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---- && ros2 action list && echo ---- && ros2 service list | grep -v parameter && echo ---- && ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        tr=t.transform.translation; r=t.transform.rotation
        seen[(t.header.frame_id,t.child_frame_id)] = (round(tr.x,4),round(tr.y,4),round(tr.z,4),round(r.x,4),round(r.y,4),round(r.z,4),round(r.w,4))
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
end=time.time()+5
while time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
for k,v in sorted(seen.items()): print(k, v)
EOF
python3 tfdump.py

# openrua op 7
cat > pxw.py <<'EOF'
"""Usage: python3 pxw.py <camera> u,v [u,v ...]  -> world xyz for each pixel (depth+intrinsics+TF)."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
def grab(node, topic, T, qos=1):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), qos)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node("pxw")
depth=grab(node,f"/{cam}/depth/image_raw",Image)
info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
frame=f"{cam}_optical_frame"
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
tf=None
while tf is None:
    m=grab(node,"/tf_static",TFMessage,qos)
    for t in m.transforms:
        if t.child_frame_id==frame: tf=t
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
R=qR(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w)
t=np.array([tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z])
for a in sys.argv[2:]:
    u,v=map(int,a.split(","))
    z=float(D[v,u])
    p=R@np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])+t
    print(f"({u},{v}) depth={z:.3f} world= {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
np.save(f"{cam}_depth.npy", D)
EOF
python3 pxw.py agentview 228,225 228,250 355,268 355,285 530,260 530,300 300,400 100,300 140,360 245,355

# openrua op 8
cat /tmp/claude-1000/-workspace/7063777f-528f-4e08-89ca-ff0f4f200312/tasks/b1g85fp7a.output; ls -la /workspace/*.npy 2>/dev/null

# openrua op 9
python3 - <<'EOF'
import re
s=open("pxw.py").read()
s=s.replace('''tf=None
while tf is None:
    m=grab(node,"/tf_static",TFMessage,qos)
    for t in m.transforms:
        if t.child_frame_id==frame: tf=t
''','''seen={}
def cb(m):
    for t in m.transforms: seen[t.child_frame_id]=t
node.create_subscription(TFMessage,"/tf_static",cb,qos)
node.create_subscription(TFMessage,"/tf",cb,100)
import time; end=time.time()+4
while time.time()<end and frame not in seen: rclpy.spin_once(node,timeout_sec=0.2)
tf=seen[frame]
''')
open("pxw.py","w").write(s)
EOF
timeout 120 python3 pxw.py agentview 228,225 228,250 355,268 355,285 530,260 530,300 300,400 100,300 140,360 245,355

# openrua op 10
timeout 120 python3 pxw.py birdview 383,288 284,238 270,313 297,300 320,150 200,200 400,350 312,283

# openrua op 11
cat > seg.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from scipy import ndimage
def grab(node, topic, T, qos=1):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), qos)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cam=sys.argv[1]; zmin=float(sys.argv[2]) if len(sys.argv)>2 else 0.44
rclpy.init(); node=rclpy.create_node("seg")
depth=grab(node,f"/{cam}/depth/image_raw",Image)
info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
frame=f"{cam}_optical_frame"
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
seen={}
def cb(m):
    for t in m.transforms: seen[t.child_frame_id]=t
node.create_subscription(TFMessage,"/tf_static",cb,qos)
node.create_subscription(TFMessage,"/tf",cb,100)
import time; end=time.time()+4
while time.time()<end and frame not in seen: rclpy.spin_once(node,timeout_sec=0.2)
tf=seen[frame]
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
R=qR(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w)
t=np.array([tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z])
v,u=np.mgrid[0:depth.height,0:depth.width]
P=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D],-1)@R.T+t
X,Y,Z=P[...,0],P[...,1],P[...,2]
mask=(Z>zmin)&(Z<0.75)&(X>-0.35)&(X<0.45)&(np.abs(Y)<0.5)&np.isfinite(D)
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<4: continue
    print(f"blob{i}: px={m.sum()} u={u[m].mean():.0f} v={v[m].mean():.0f} x[{X[m].min():.3f},{X[m].max():.3f}] y[{Y[m].min():.3f},{Y[m].max():.3f}] z[{Z[m].min():.3f},{Z[m].max():.3f}] centroid=({X[m].mean():.3f},{Y[m].mean():.3f})")
EOF
timeout 120 python3 seg.py birdview 0.44

# openrua op 12
timeout 120 python3 seg.py agentview 0.44

# openrua op 13
cat > fk.py <<'EOF'
import rclpy, yaml
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); node=rclpy.create_node("fk")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand"]
arm=[f"panda_joint{i}" for i in range(1,8)]
d=dict(zip(js["m"].name,js["m"].position))
req.robot_state.joint_state.name=arm; req.robot_state.joint_state.position=[d[a] for a in arm]
f=cli.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=30)
r=f.result()
print("joints:",[round(d[a],4) for a in arm], "fingers:", [round(d[n],4) for n in d if "finger" in n])
for ps in r.pose_stamped:
    p=ps.pose.position; q=ps.pose.orientation
    print(f"frame={ps.header.frame_id} pos=({p.x:.4f},{p.y:.4f},{p.z:.4f}) quat=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
timeout 60 python3 fk.py

# openrua op 14
cat > chain.py <<'EOF'
import numpy as np, rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rclpy.init(); node=rclpy.create_node("chain")
seen={}
def cb(m):
    for t in m.transforms: seen[t.child_frame_id]=t
node.create_subscription(TFMessage,"/tf_static",cb,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(TFMessage,"/tf",cb,100)
end=time.time()+4
while time.time()<end: rclpy.spin_once(node,timeout_sec=0.2)
def T(child):
    t=seen[child]; M=np.eye(4); r=t.transform.rotation; tr=t.transform.translation
    M[:3,:3]=qR(r.x,r.y,r.z,r.w); M[:3,3]=[tr.x,tr.y,tr.z]; return M
M=np.eye(4)
for c in ["panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_link8","panda_hand"]:
    M=M@T(c)
print("base->hand pos", M[:3,3].round(4)); print("R=\n",M[:3,:3].round(3))
W=T("panda_link0")@M
print("world->hand pos", W[:3,3].round(4))
EOF
timeout 60 python3 chain.py

# openrua op 15
cat > arm.py <<'EOF'
"""Arm helper: IK (base frame), trajectory, gripper, state. Import or run as CLI.
CLI: python3 arm.py ik x y z            -> print IK joints for TCP at base-frame xyz, hand pointing down
     python3 arm.py tcp x y z [secs]    -> IK + move TCP (world frame coords!) 
     python3 arm.py grip open|close
     python3 arm.py state
"""
import sys, time, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

ARM=[f"panda_joint{i}" for i in range(1,8)]
TCP=0.1034
WORLD2BASE=np.array([-0.51,0.0,0.42])  # world->panda_link0 translation
DOWN=(1.0,0.0,0.0,0.0)  # hand Z down, fingers close along Y

class Arm:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node=rclpy.create_node("arm_helper")
        self.js={}
        self.node.create_subscription(JointState,"/joint_states",self._js,1)
        self.ik=self.node.create_client(GetPositionIK,"/compute_ik")
        self.fk=self.node.create_client(GetPositionFK,"/compute_fk")
        self.traj=ActionClient(self.node,FollowJointTrajectory,"/panda_arm_controller/follow_joint_trajectory")
        self.grip=ActionClient(self.node,GripperCommand,"/franka_gripper/gripper_action")
        self.ik.wait_for_service(); self.traj.wait_for_server(); self.grip.wait_for_server()
    def _js(self,m): self.js=dict(zip(m.name,m.position))
    def state(self):
        self.js={}
        while not self.js: rclpy.spin_once(self.node,timeout_sec=0.2)
        return dict(self.js)
    def joints(self):
        s=self.state(); return [s[a] for a in ARM]
    def fingers(self):
        s=self.state(); return s["panda_finger_joint1"], s["panda_finger_joint2"]
    def hand_pose_world(self):
        req=GetPositionFK.Request(); req.fk_link_names=["panda_hand"]
        req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=self.joints()
        f=self.fk.call_async(req); rclpy.spin_until_future_complete(self.node,f,timeout_sec=60)
        p=f.result().pose_stamped[0].pose
        return np.array([p.position.x,p.position.y,p.position.z]), (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)
    def solve_ik_base(self, pos_base, quat=DOWN, seed=None, at_tcp=True):
        x,y,z=pos_base
        if at_tcp:
            R=_qR(*quat); x,y,z=np.array([x,y,z])-TCP*R[:,2]
        req=GetPositionIK.Request(); r=req.ik_request
        r.group_name="panda_arm"; r.pose_stamped.header.frame_id=""
        r.pose_stamped.pose.position.x=float(x); r.pose_stamped.pose.position.y=float(y); r.pose_stamped.pose.position.z=float(z)
        r.pose_stamped.pose.orientation.x,r.pose_stamped.pose.orientation.y,r.pose_stamped.pose.orientation.z,r.pose_stamped.pose.orientation.w=[float(q) for q in quat]
        r.robot_state.joint_state.name=ARM; r.robot_state.joint_state.position=[float(v) for v in (seed or self.joints())]
        r.avoid_collisions=False
        r.timeout=Duration(sec=2)
        f=self.ik.call_async(req); rclpy.spin_until_future_complete(self.node,f,timeout_sec=120)
        res=f.result()
        if res is None: raise RuntimeError("IK no answer")
        if res.error_code.val!=1: raise RuntimeError(f"IK failed code={res.error_code.val}")
        sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
        return [sol[a] for a in ARM]
    def solve_ik_world(self, pos_world, quat=DOWN, **kw):
        return self.solve_ik_base(np.array(pos_world)-WORLD2BASE, quat, **kw)
    def move(self, joints, secs=3.0, wait_timeout=600):
        g=FollowJointTrajectory.Goal(); g.trajectory.joint_names=ARM
        pt=JointTrajectoryPoint(positions=[float(v) for v in joints]); pt.time_from_start=Duration(sec=int(secs),nanosec=int((secs%1)*1e9))
        g.trajectory.points=[pt]
        f=self.traj.send_goal_async(g); rclpy.spin_until_future_complete(self.node,f,timeout_sec=120)
        rf=f.result().get_result_async(); rclpy.spin_until_future_complete(self.node,rf,timeout_sec=wait_timeout)
        code=rf.result().result.error_code
        cur=self.joints(); err=max(abs(a-b) for a,b in zip(cur,joints))
        print(f"move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err
    def move_tcp_world(self, pos, quat=DOWN, secs=3.0):
        j=self.solve_ik_world(pos,quat); return self.move(j,secs)
    def gripper(self, width, timeout=300):
        g=GripperCommand.Goal(); g.command.position=float(width); g.command.max_effort=30.0
        f=self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node,f,timeout_sec=120)
        rf=f.result().get_result_async(); rclpy.spin_until_future_complete(self.node,rf,timeout_sec=timeout)
        r=rf.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}", flush=True)
        return r

def _qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

if __name__=="__main__":
    a=Arm(); cmd=sys.argv[1]
    if cmd=="state":
        print(a.state()); p,q=a.hand_pose_world(); print("hand world",p.round(4),np.round(q,4))
    elif cmd=="ik":
        print(a.solve_ik_base([float(v) for v in sys.argv[2:5]]))
    elif cmd=="tcp":
        secs=float(sys.argv[5]) if len(sys.argv)>5 else 3.0
        a.move_tcp_world([float(v) for v in sys.argv[2:5]],secs=secs); p,q=a.hand_pose_world(); print("hand world",p.round(4))
    elif cmd=="grip":
        a.gripper(0.04 if sys.argv[2]=="open" else 0.0)
EOF
timeout 120 python3 arm.py ik 0.457 0 0.4610

# openrua op 16
cat > iktest.py <<'EOF'
from arm import *
a=Arm()
for label,pos in [("base",[0.457,0,0.3576]),("world",[-0.053,0,0.7776])]:
    for q in [(1.0,0,0,0),(0.9996,0,-0.0284,0)]:
        try: print(label,q,np.round(a.solve_ik_base(pos,q,at_tcp=False),4))
        except Exception as e: print(label,q,e)
EOF
timeout 300 python3 iktest.py

# openrua op 17
cat > iktest2.py <<'EOF'
from arm import *
a=Arm()
req=GetPositionFK.Request(); req.fk_link_names=["panda_link8","panda_hand"]
req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=a.joints()
f=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,f,timeout_sec=60)
for ps in f.result().pose_stamped:
    p=ps.pose.position; q=ps.pose.orientation; print(ps.header.frame_id,(round(p.x,4),round(p.y,4),round(p.z,4)),(round(q.x,4),round(q.y,4),round(q.z,4),round(q.w,4)))
print(np.round(a.solve_ik_base([-0.053,0,0.7776],(0.9239,-0.3827,0,0),at_tcp=False),4))
EOF
timeout 300 python3 iktest2.py

# openrua op 18
python3 - <<'EOF'
s=open("arm.py").read()
s=s.replace('WORLD2BASE=np.array([-0.51,0.0,0.42])  # world->panda_link0 translation\nDOWN=(1.0,0.0,0.0,0.0)  # hand Z down, fingers close along Y',
 '# MACHINE FACT (verified): /compute_ik solves for panda_link8 in the WORLD frame (frame_id empty).\n# panda_link8 quaternion for hand pointing down with fingers closing along world Y:\nDOWN=(0.9239,-0.3827,0.0,0.0)\n# fingers closing along world X (hand yawed 90 deg):\nDOWN_X=(0.6533,-0.6533,0.2706,0.2706)')
s=s.replace('        return self.solve_ik_base(np.array(pos_world)-WORLD2BASE, quat, **kw)','        return self.solve_ik_base(np.array(pos_world), quat, **kw)')
open("arm.py","w").write(s)
EOF
grep -n "DOWN" arm.py | head

# openrua op 19
sed -i 's/DOWN_X=(0.6533,-0.6533,0.2706,0.2706)/DOWN_X=(0.3827,-0.9239,0.0,0.0)/' arm.py; timeout 120 python3 seg.py robot0_eye_in_hand 0.44

# openrua op 20
python3 - <<'EOF'
s=open("seg.py").read()
s=s.replace('cam=sys.argv[1]; zmin=float(sys.argv[2]) if len(sys.argv)>2 else 0.44',
'cam=sys.argv[1]; zmin=float(sys.argv[2]) if len(sys.argv)>2 else 0.44\nzmax=float(sys.argv[3]) if len(sys.argv)>3 else 0.75\nroi=[int(x) for x in sys.argv[4].split(",")] if len(sys.argv)>4 else None')
s=s.replace('mask=(Z>zmin)&(Z<0.75)','mask=(Z>zmin)&(Z<zmax)')
s=s.replace('lab,n=ndimage.label(mask)','if roi: mask&=(u>=roi[0])&(u<=roi[1])&(v>=roi[2])&(v<=roi[3])\nlab,n=ndimage.label(mask)')
open("seg.py","w").write(s)
EOF
timeout 120 python3 seg.py robot0_eye_in_hand 0.44 0.47 200,340,280,400

# openrua op 21
cat > pick_box.py <<'EOF'
from arm import *
a=Arm()
BOX=np.array([-0.137,0.056]); TOP=0.455
print("state",np.round(a.joints(),3),a.fingers(),flush=True)
a.gripper(0.04)
print("-> above box",flush=True)
a.move_tcp_world([BOX[0],BOX[1],TOP+0.10],DOWN,secs=3)
p,q=a.hand_pose_world(); print("hand",p.round(4),np.round(q,4),flush=True)
EOF
timeout 900 python3 -u pick_box.py 2>&1 | tee pick_box.log

# openrua op 22
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png && timeout 120 python3 seg.py robot0_eye_in_hand 0.44 0.47

# openrua op 23
cat > grasp_box.py <<'EOF'
from arm import *
a=Arm()
BOX=np.array([-0.137,0.056])
print("-> descend",flush=True)
a.move_tcp_world([BOX[0],BOX[1],0.434],DOWN,secs=2.5)
p,q=a.hand_pose_world(); print("hand",p.round(4),"tcp z",round(p[2]-TCP,4),flush=True)
a.gripper(0.0)
print("fingers after close",a.fingers(),flush=True)
print("-> lift",flush=True)
a.move_tcp_world([BOX[0],BOX[1],0.60],DOWN,secs=2.5)
p,q=a.hand_pose_world(); print("hand",p.round(4),flush=True)
print("fingers after lift",a.fingers(),flush=True)
EOF
timeout 900 python3 -u grasp_box.py 2>&1 | tee grasp_box.log

# openrua op 24
timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png

# openrua op 25
cat > place_box.py <<'EOF'
from arm import *
a=Arm()
B=np.array([-0.009,0.253])
print("-> above basket",flush=True)
a.move_tcp_world([B[0],B[1],0.72],DOWN,secs=3)
p,q=a.hand_pose_world(); print("hand",p.round(4),"fingers",a.fingers(),flush=True)
print("-> lower",flush=True)
a.move_tcp_world([B[0],B[1],0.66],DOWN,secs=2)
p,q=a.hand_pose_world(); print("hand",p.round(4),"fingers",a.fingers(),flush=True)
a.gripper(0.04)
print("-> retreat up",flush=True)
a.move_tcp_world([B[0],B[1],0.75],DOWN,secs=2)
p,q=a.hand_pose_world(); print("hand",p.round(4),"fingers",a.fingers(),flush=True)
EOF
timeout 900 python3 -u place_box.py 2>&1 | tee place_box.log

# openrua op 26
timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png

# openrua op 27
cat > above_ketchup.py <<'EOF'
from arm import *
a=Arm()
K=np.array([-0.206,-0.148])
print("-> above ketchup",flush=True)
a.move_tcp_world([K[0],K[1],0.70],DOWN,secs=3.5)
p,q=a.hand_pose_world(); print("hand",p.round(4),np.round(q,3),"fingers",a.fingers(),flush=True)
EOF
timeout 900 python3 -u above_ketchup.py 2>&1 | tee above_ketchup.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png; timeout 120 python3 seg.py robot0_eye_in_hand 0.44 0.60

# openrua op 28
timeout 900 python3 -u above_ketchup.py 2>&1 | tee above_ketchup2.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih5.png; timeout 120 python3 seg.py robot0_eye_in_hand 0.53 0.60 200,450,200,450; timeout 120 python3 seg.py robot0_eye_in_hand 0.46 0.50 200,450,200,450

# openrua op 29
cat > grasp_ketchup.py <<'EOF'
from arm import *
a=Arm()
K=np.array([-0.200,-0.148])
print("-> rotate above ketchup",flush=True)
for i in range(2):
    code,err=a.move_tcp_world([K[0],K[1],0.66],DOWN_X,secs=3)
    if code==0: break
p,q=a.hand_pose_world(); print("hand",p.round(4),np.round(q,3),"fingers",a.fingers(),flush=True)
EOF
timeout 900 python3 -u grasp_ketchup.py 2>&1 | tee grasp_ketchup.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih6.png; timeout 120 python3 seg.py robot0_eye_in_hand 0.44 0.60 150,500,150,450

# openrua op 30
sed -i 's/secs=3)/secs=4)/' grasp_ketchup.py; timeout 900 python3 -u grasp_ketchup.py 2>&1 | tee grasp_ketchup2.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih6.png; timeout 120 python3 seg.py robot0_eye_in_hand 0.44 0.60 150,500,150,450

# openrua op 31
cat > grasp_ketchup3.py <<'EOF'
from arm import *
a=Arm()
K=np.array([-0.200,-0.150])
def go(z,secs):
    for i in range(3):
        code,err=a.move_tcp_world([K[0],K[1],z],DOWN_X,secs=secs)
        if code==0 and err<0.01: break
    p,q=a.hand_pose_world(); print("hand",p.round(4),"tcp z",round(p[2]-TCP,4),np.round(q,3),flush=True)
print("-> descend",flush=True); go(0.56,3); go(0.475,3)
a.gripper(0.0)
f=a.fingers(); print("fingers after close",f,flush=True)
print("-> lift",flush=True); go(0.70,3)
print("fingers after lift",a.fingers(),flush=True)
EOF
timeout 1500 python3 -u grasp_ketchup3.py 2>&1 | tee grasp_ketchup3.log

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih7.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent7.png; timeout 120 python3 seg.py robot0_eye_in_hand 0.44 0.60 100,540,100,450

# openrua op 33
cat > profile.py <<'EOF'
import sys, numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
def grab(node, topic, T, qos=1):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), qos)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cam=sys.argv[1]; x0,x1,y0,y1=map(float,sys.argv[2:6]); zmin=float(sys.argv[6])
rclpy.init(); node=rclpy.create_node("prof")
depth=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
frame=f"{cam}_optical_frame"; seen={}
def cb(m):
    for t in m.transforms: seen[t.child_frame_id]=t
node.create_subscription(TFMessage,"/tf_static",cb,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(TFMessage,"/tf",cb,100)
end=time.time()+4
while time.time()<end and frame not in seen: rclpy.spin_once(node,timeout_sec=0.2)
tf=seen[frame]
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
R=qR(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w)
t=np.array([tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z])
v,u=np.mgrid[0:depth.height,0:depth.width]
P=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D],-1)@R.T+t
X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>x0)&(X<x1)&(Y>y0)&(Y<y1)&(Z>zmin)&np.isfinite(D)
print("overall x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f n=%d"%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max(),m.sum()))
for xa in np.arange(x0,x1,0.01):
    mm=m&(X>=xa)&(X<xa+0.01)
    if mm.sum()>3: print(f"x={xa:.2f}: n={mm.sum():4d} y[{Y[mm].min():.3f},{Y[mm].max():.3f}] ymid={0.5*(Y[mm].min()+Y[mm].max()):.3f} zmax={Z[mm].max():.3f}")
# PCA of xy
pts=np.stack([X[m],Y[m]],1); c=pts.mean(0); w,vec=np.linalg.eigh(np.cov((pts-c).T))
print("centroid",c.round(4),"axis",vec[:,1].round(3),"angle_deg",np.degrees(np.arctan2(vec[1,1],vec[0,1])).round(1))
EOF
timeout 120 python3 profile.py robot0_eye_in_hand -0.20 -0.02 -0.23 -0.12 0.44

# openrua op 34
cat > look.py <<'EOF'
from arm import *
a=Arm()
a.gripper(0.04)
for i in range(3):
    code,err=a.move_tcp_world([-0.11,-0.17,0.72],DOWN,secs=4)
    if code==0 and err<0.01: break
p,q=a.hand_pose_world(); print("hand",p.round(4),np.round(q,3),flush=True)
EOF
timeout 900 python3 -u look.py 2>&1 | tee look.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih8.png; timeout 120 python3 profile.py robot0_eye_in_hand -0.25 -0.02 -0.25 -0.10 0.44

# openrua op 35
python3 - <<'EOF'
s=open("arm.py").read()
s=s.replace('''    def move_tcp_world(self, pos, quat=DOWN, secs=3.0):
        j=self.solve_ik_world(pos,quat); return self.move(j,secs)''',
'''    def move_tcp_world(self, pos, quat=DOWN, secs=3.0, max_delta=1.0, tries=5):
        cur=self.joints()
        for k in range(tries):
            j=self.solve_ik_world(pos,quat,seed=cur)
            d=max(abs(a-b) for a,b in zip(j,cur))
            if d<=max_delta: break
            print(f"IK branch far from current (max delta {d:.2f}); retrying", flush=True)
        else:
            raise RuntimeError(f"no nearby IK branch (delta {d:.2f})")
        return self.move(j,secs)
    def go(self, pos, quat=DOWN, secs=3.0, retries=3):
        """move_tcp_world with re-send on tracking lag; returns final hand pose"""
        for i in range(retries):
            code,err=self.move_tcp_world(pos,quat,secs)
            if code==0 and err<0.01: break
        p,q=self.hand_pose_world(); print("hand",p.round(4),"tcp z",round(p[2]-TCP,4),np.round(q,3),flush=True)
        return p,q''')
s=s.replace('DOWN_X=(0.3827,-0.9239,0.0,0.0)','''DOWN_X=(0.3827,-0.9239,0.0,0.0)
def yaw_quat(theta_deg):
    """link8 quaternion: hand down, closing axis rotated theta from world Y about world Z"""
    import math
    s,c=math.sin(math.radians(theta_deg)/2),math.cos(math.radians(theta_deg)/2)
    x2,y2=DOWN[0],DOWN[1]
    return (c*x2-s*y2, c*y2+s*x2, 0.0, 0.0)''')
open("arm.py","w").write(s)
EOF
python3 -c "
from arm import yaw_quat; print(yaw_quat(14), yaw_quat(90))"

# openrua op 36
cat > grasp_k2.py <<'EOF'
from arm import *
a=Arm()
Q=yaw_quat(14)
G=np.array([-0.15,-0.184])
print("fingers",a.fingers(),flush=True)
print("-> above",flush=True); p,q=a.go([G[0],G[1],0.60],Q,secs=4)
R=_qR(*q); print("hand Y axis (closing dir) world:",R[:,1].round(3),flush=True)
EOF
timeout 900 python3 -u grasp_k2.py 2>&1 | tee grasp_k2.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih9.png

# openrua op 37
cat > grasp_k3.py <<'EOF'
from arm import *
a=Arm()
Q=yaw_quat(14)
G=np.array([-0.15,-0.184])
print("-> descend",flush=True); a.go([G[0],G[1],0.52],Q,secs=3); a.go([G[0],G[1],0.45],Q,secs=3)
a.gripper(0.0)
print("fingers after close",a.fingers(),flush=True)
print("-> lift",flush=True); a.go([G[0],G[1],0.70],Q,secs=3)
print("fingers after lift",a.fingers(),flush=True)
EOF
timeout 1500 python3 -u grasp_k3.py 2>&1 | tee grasp_k3.log

# openrua op 38
cat > lift_k.py <<'EOF'
from arm import *
a=Arm()
Q=yaw_quat(14)
G=np.array([-0.15,-0.184])
print("fingers",a.fingers(),flush=True)
for z in (0.50,0.56,0.62,0.70):
    try: a.go([G[0],G[1],z],Q,secs=2.5)
    except RuntimeError as e: print("z",z,e,flush=True)
print("fingers after lift",a.fingers(),flush=True)
EOF
timeout 1500 python3 -u lift_k.py 2>&1 | tee lift_k.log

# openrua op 39
timeout 60 python3 tools/perception/cam_snap.py agentview agent10.png

# openrua op 40
cat > place_k.py <<'EOF'
from arm import *
a=Arm()
Q=yaw_quat(45)
T=np.array([-0.037,0.225])
print("fingers",a.fingers(),flush=True)
a.go([-0.15,-0.184,0.75],yaw_quat(14),secs=2.5)
a.go([-0.10,0.02,0.75],yaw_quat(30),secs=3)
a.go([T[0],T[1],0.75],Q,secs=3)
print("fingers over basket",a.fingers(),flush=True)
EOF
timeout 1500 python3 -u place_k.py 2>&1 | tee place_k.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih11.png

# openrua op 41
cat > drop_k.py <<'EOF'
from arm import *
a=Arm()
Q=yaw_quat(45)
T=np.array([-0.037,0.225])
a.go([T[0],T[1],0.66],Q,secs=2.5)
print("fingers before release",a.fingers(),flush=True)
a.gripper(0.04)
a.go([T[0],T[1],0.78],Q,secs=2.5)
print("fingers after",a.fingers(),flush=True)
EOF
timeout 1500 python3 -u drop_k.py 2>&1 | tee drop_k.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih12.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent12.png

# openrua op 42
cat > home.py <<'EOF'
from arm import *
a=Arm()
home=[0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854]
for i in range(3):
    code,err=a.move(home,4)
    if code==0 and err<0.01: break
print("fingers",a.fingers(),flush=True)
EOF
timeout 900 python3 -u home.py 2>&1 | tee home.log; timeout 60 python3 tools/perception/cam_snap.py birdview bird_final.png; timeout 120 python3 seg.py birdview 0.44 0.75

# openrua op 43
timeout 60 python3 tools/perception/cam_snap.py frontview front_final.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent_final.png

# openrua op 44
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-ik-frame-facts.md)"
cat > /claude-config/projects/-workspace/memory/panda-ik-frame-facts.md <<'OPENRUA_EOF'
---
name: panda-ik-frame-facts
description: Verified quirks of the /workspace Panda sim - /compute_ik solves panda_link8 in WORLD frame (docs say base), trajectory controller lags (-5 = resend), IK branch jumps
metadata:
  type: project
---

On the /workspace Franka Panda workstation (machine.yaml, OpenRUA sim), verified 2026-09-12:

- `/compute_ik` (group panda_arm, frame_id empty) solves for **panda_link8**, and poses are in the **world** frame, not panda_link0 as docs/30-action.md "Planning facts" claims. `/compute_fk` also returns world-frame poses. world->panda_link0 = (-0.51, 0, 0.42).
- Hand-down with fingers closing along world Y: link8 quat (0.9239, -0.3827, 0, 0) (panda_hand then reads ~(1,0,0,0)). TCP is 0.1034 m along hand +Z.
- Table top is at world z≈0.426. Fingertip (TCP) at z≈0.434 works for 3 cm-tall boxes.
- FollowJointTrajectory often returns error_code -5 with 0.1–0.5 rad residual on big moves; re-sending the same goal converges (as docs say). Use 3–4 s durations.
- IK from a low pose to a high pose (>20 cm jump) can return a far joint branch (>1 rad delta) that sweeps through objects; move in ~6–10 cm increments and reject solutions far from the current joints.
- /workspace/arm.py (written that session) wraps IK/FK/trajectory/gripper with these conventions.

**Why:** the docs were wrong on the IK frame; blindly following them gives -31 NO_IK_SOLUTION, and a large-branch IK jump knocked over an object once.
**How to apply:** trust these facts over docs/30-action.md; verify once with FK at session start since the sim may change.
OPENRUA_EOF

# openrua op 45
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index yet)")

# openrua op 46
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda IK frame facts](panda-ik-frame-facts.md) — /compute_ik = panda_link8 in WORLD frame (docs wrong); -5 = resend; move in small z steps
OPENRUA_EOF
