#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | head -40; echo ---; ros2 node list

# openrua op 3
ros2 topic echo /joint_states --once 2>&1 | head -40; for c in agentview frontview birdview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 4
for p in "365 320" "365 285" "455 300" "455 265" "200 295" "205 235" "320 400"; do echo -n "agentview $p -> "; timeout 60 python3 tools/perception/px2world.py agentview $p 2>&1 | tail -1; done

# openrua op 5
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -8

# openrua op 6
cat > /workspace/cloud.py <<'EOF'
#!/usr/bin/env python3
"""Dump a world-frame point cloud from a camera's depth + intrinsics + TF, save npy."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def grab(node, topic, T, timeout=30):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time; end = time.time()+timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
buf = Buffer(); TransformListener(buf, node)
depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
color = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
import time; end=time.time()+10
while time.time()<end and not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q=t.transform.rotation; R=quat_R(q.x,q.y,q.z,q.w)
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
H,W = depth.shape
v,u = np.mgrid[0:H,0:W]
z = depth
X = (u-cx)*z/fx; Y=(v-cy)*z/fy
P = np.stack([X,Y,z],-1) @ R.T + tr
np.save(f"{cam}_cloud.npy", P); np.save(f"{cam}_color.npy", color)
print("saved", P.shape, "cam at", tr)
rclpy.shutdown()
EOF
timeout 120 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np
P=np.load("/workspace/birdview_cloud.npy"); C=np.load("/workspace/birdview_color.npy")
H,W,_=P.shape
z=P[...,2]
print("table z stats over table region:", np.percentile(z[200:400,200:450],[5,50,95]))
# objects above table
mask=(z>0.915)&(P[...,0]>-0.35)&(P[...,0]<0.5)&(np.abs(P[...,1])<0.5)
from scipy import ndimage
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<30: continue
    pts=P[m]
    vs,us=np.where(m)
    print(f"blob {i}: n={m.sum()} px=({us.mean():.0f},{vs.mean():.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} center=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
EOF

# openrua op 7
timeout 120 python3 cloud.py frontview && timeout 120 python3 cloud.py sideview && python3 - <<'EOF'
import numpy as np
for cam in ["frontview","sideview"]:
    P=np.load(f"/workspace/{cam}_cloud.npy")
    print(cam)
    # left pot region: x in [-0.02,0.15], y in [-0.03,0.14]
    m=(P[...,0]>-0.03)&(P[...,0]<0.16)&(P[...,1]>-0.03)&(P[...,1]<0.15)&(P[...,2]>0.905)
    pts=P[m]
    for zlo in np.arange(0.90,1.07,0.01):
        s=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
        if len(s)<3: continue
        print(f" z {zlo:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 8
timeout 20 ros2 run tf2_ros tf2_echo panda_hand panda_leftfinger 2>&1 | grep -A1 Translation | head -2; timeout 20 ros2 run tf2_ros tf2_echo panda_hand panda_hand_tcp 2>&1 | grep -A1 Translation | head -2; timeout 20 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 Translation | head -4; ros2 topic echo /robot_description --qos-durability transient_local --qos-reliability reliable --once 2>/dev/null | grep -iE "hand|finger" | grep -iE "mesh|box|cylinder|origin" | head -30

# openrua op 9
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once 2>&1 | grep -A9 "^position" | tail -2

# openrua op 10
ros2 node info /robot_bridge 2>&1 | head -60; echo ---; ros2 interface show ros_gz_interfaces/srv/LoadGeometryFromFile 2>/dev/null | head; ros2 service type /load_geometry_from_file; ros2 service type /load_map; ros2 service type /get_urdf; ros2 topic echo /pipeline_state --once 2>&1 | head -20

# openrua op 11
sleep 20; cat /tmp/claude-1000/-workspace/1952cb5a-f007-4c98-8041-db16845578a2/tasks/b1hplxnih.output

# openrua op 12
cat > mv.py <<'EOF'
#!/usr/bin/env python3
"""IK (base frame) -> trajectory -> verify. Usage: mv.py x y z qx qy qz qw [sec] [--tcp]
Coordinates are WORLD frame; converted to base frame internally."""
import sys, time, numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation
TCP_OFF = 0.1034
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]

def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def get_js(node):
    got = {}
    sub = node.create_subscription(JointState, "/joint_states", lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return dict(zip(got["m"].name, got["m"].position))

def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    x, y, z, qx, qy, qz, qw = map(float, args[:7])
    sec = float(args[7]) if len(args) > 7 else 4.0
    p = np.array([x, y, z])
    if "--tcp" in sys.argv:
        R = quat_R(qx, qy, qz, qw)
        p = p - TCP_OFF * R[:, 2]
    pb = p - BASE
    rclpy.init(); node = rclpy.create_node("mv")
    js = get_js(node)
    cli = node.create_client(GetPositionIK, "/compute_ik")
    cli.wait_for_service(10)
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    pp = req.ik_request.pose_stamped.pose
    pp.position.x, pp.position.y, pp.position.z = map(float, pb)
    pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw
    seed = JointState(); seed.name = JOINTS; seed.position = [js[j] for j in JOINTS]
    req.ik_request.robot_state.joint_state = seed
    req.ik_request.avoid_collisions = False
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    res = fut.result()
    if res is None or res.error_code.val != 1:
        raise SystemExit(f"IK FAILED code={None if res is None else res.error_code.val}")
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    target = [sol[j] for j in JOINTS]
    print("IK target:", np.round(target, 4))
    ac = ActionClient(node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
    ac.wait_for_server(10)
    goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
    pt = JointTrajectoryPoint(positions=target)
    pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
    goal.trajectory.points = [pt]
    f = ac.send_goal_async(goal); rclpy.spin_until_future_complete(node, f)
    rf = f.result().get_result_async(); rclpy.spin_until_future_complete(node, rf)
    print("error_code", rf.result().result.error_code)
    js = get_js(node)
    err = np.array([js[j] for j in JOINTS]) - np.array(target)
    print("joint err max", np.abs(err).max().round(4), "fingers", round(js["panda_finger_joint1"], 4))
    rclpy.shutdown()

if __name__ == "__main__":
    main()
EOF
timeout 900 python3 -u mv.py 0.06 0.071 1.20 0.7071068 0.7071068 0 0 4 --tcp

# openrua op 13
cat > iktest.py <<'EOF'
import sys, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
BASE = np.array([-0.66, 0.0, 0.912])
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
rclpy.init(); node = rclpy.create_node("ikt")
got={}
sub=node.create_subscription(JointState,"/joint_states",lambda m: got.setdefault("m",m),1)
while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
js=dict(zip(got["m"].name,got["m"].position))
cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service(10)
def ik(pw, q, timeout=5.0):
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.pose_stamped.header.frame_id=""
    pp=req.ik_request.pose_stamped.pose
    pb=np.array(pw)-BASE
    pp.position.x,pp.position.y,pp.position.z=map(float,pb)
    pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=q
    seed=JointState(); seed.name=JOINTS; seed.position=[js[j] for j in JOINTS]
    req.ik_request.robot_state.joint_state=seed
    req.ik_request.timeout.sec=int(timeout); req.ik_request.timeout.nanosec=int((timeout%1)*1e9)
    req.ik_request.avoid_collisions=False
    f=cli.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=60)
    r=f.result()
    if r is None: return None,None
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
    return r.error_code.val, [round(sol.get(j,float('nan')),3) for j in JOINTS]
tests=[
 ((0.06,0.071,1.3034),(0.7071068,0.7071068,0,0)),
 ((0.06,0.071,1.3034),(1,0,0,0)),
 ((0.06,0.071,1.3034),(0.9238795,0.3826834,0,0)),
 ((0.06,0.071,1.3034),(0,1,0,0)),
 ((-0.203,0,1.27),(1,0,-0.028,0)),
 ((-0.1,0.0,1.25),(0.7071068,0.7071068,0,0)),
 ((0.0,0.07,1.25),(0.7071068,0.7071068,0,0)),
 ((0.06,0.071,1.25),(0.7071068,0.7071068,0,0)),
]
for pw,q in tests:
    print(pw,q,"->",ik(pw,q))
rclpy.shutdown()
EOF
timeout 600 python3 -u iktest.py

# openrua op 14
sed -i 's/pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=q/pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=map(float,q)/' iktest.py && timeout 600 python3 -u iktest.py

# openrua op 15
cat > fk.py <<'EOF'
import numpy as np, rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
rclpy.init(); node = rclpy.create_node("fkt")
got={}
sub=node.create_subscription(JointState,"/joint_states",lambda m: got.setdefault("m",m),1)
while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
js=dict(zip(got["m"].name,got["m"].position))
cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service(10)
req=GetPositionFK.Request(); req.fk_link_names=["panda_link8","panda_hand","panda_hand_tcp","panda_link0"]
req.robot_state.joint_state.name=JOINTS; req.robot_state.joint_state.position=[js[j] for j in JOINTS]
f=cli.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=60)
r=f.result(); print("code",r.error_code.val)
for n,p in zip(r.fk_link_names,r.pose_stamped):
    print(n,p.header.frame_id,round(p.pose.position.x,4),round(p.pose.position.y,4),round(p.pose.position.z,4),
          [round(v,4) for v in (p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w)])
rclpy.shutdown()
EOF
timeout 300 python3 -u fk.py; ros2 param get /move_group robot_description_kinematics.panda_arm.kinematics_solver 2>&1 | tail -1; ros2 param list /move_group 2>/dev/null | grep -i -E "kinematics|tip|group" | head

# openrua op 16
sed -i 's/BASE = np.array(\[-0.66, 0.0, 0.912\])/BASE = np.array([0.0, 0.0, 0.0])/; s/req.ik_request.group_name="panda_arm"/req.ik_request.group_name="panda_arm"; req.ik_request.ik_link_name="panda_hand"/' iktest.py && timeout 600 python3 -u iktest.py

# openrua op 17
sed -i 's/BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.array([0.0, 0.0, 0.0])  # IK model frame is world here (verified via FK)/; s/req.ik_request.group_name = "panda_arm"/req.ik_request.group_name = "panda_arm"; req.ik_request.ik_link_name = "panda_hand"/; s/pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw/pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, (qx, qy, qz, qw))/' mv.py && grep -n "BASE =\|ik_link\|orientation" mv.py && timeout 900 python3 -u mv.py 0.06 0.071 1.20 0.7071068 0.7071068 0 0 4 --tcp

# openrua op 18
ros2 topic echo /joint_states --once 2>&1 | grep -A9 "^position"; timeout 300 python3 -u fk.py

# openrua op 19
timeout 900 python3 -u mv.py 0.06 0.071 1.20 0.7071068 0.7071068 0 0 4 --tcp; timeout 300 python3 -u fk.py | grep panda_hand

# openrua op 20
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png && timeout 120 python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
P=np.load("/workspace/robot0_eye_in_hand_cloud.npy")
z=P[...,2]
m=(z>0.905)&(np.abs(P[...,0]-0.06)<0.09)&(np.abs(P[...,1]-0.071)<0.1)&np.isfinite(z)
pts=P[m]
print("n",len(pts),"zmax",pts[:,2].max())
for zlo in np.arange(0.90,1.07,0.01):
    s=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
    if len(s)<3: continue
    # restrict to body: exclude handle/spout by y window around center for x-extent
    b=s[np.abs(s[:,1]-0.071)<0.02]
    xw = (b[:,0].min(), b[:,0].max()) if len(b) else (np.nan,np.nan)
    print(f" z {zlo:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] body-x[{xw[0]:.3f},{xw[1]:.3f}] w={xw[1]-xw[0]:.3f}")
EOF

# openrua op 21
python3 - <<'EOF'
import numpy as np, cv2
P=np.load("/workspace/robot0_eye_in_hand_cloud.npy"); C=np.load("/workspace/robot0_eye_in_hand_color.npy")
import subprocess
# knob center pixel: find z>1.045 points
z=P[...,2]
m=(z>1.045)&(z<1.07)&(np.abs(P[...,0]-0.06)<0.05)&(np.abs(P[...,1]-0.071)<0.05)
vs,us=np.where(m); cu,cv=int(us.mean()),int(vs.mean()); print("knob px",cu,cv, "knob world", P[cv,cu])
# lid pixels: z in [1.025,1.04]
ml=(z>1.024)&(z<1.042)&(np.abs(P[...,0]-0.06)<0.06)&(np.abs(P[...,1]-0.071)<0.06)
vs,us=np.where(ml)
print("lid px rows",vs.min(),vs.max(),"cols",us.min(),us.max())
# for rows near knob center, x extent
for dv in range(-30,31,10):
    r=cv+dv; cols=us[vs==r]
    if len(cols)==0: continue
    print(f"row {r}: cols {cols.min()}-{cols.max()} n={len(cols)} x {P[r,cols.min(),0]:.4f}..{P[r,cols.max(),0]:.4f} y={P[r,cu,1]:.4f} w={P[r,cols.max(),0]-P[r,cols.min(),0]:.4f}")
# color-based: white pixels around the pot
gray=cv2.cvtColor(C,cv2.COLOR_BGR2GRAY)
white=(gray>170)
for dv in range(-30,31,10):
    r=cv+dv; row=white[r, cu-80:cu+80]; cols=np.where(row)[0]+cu-80
    if len(cols)==0: continue
    print(f"white row {r}: cols {cols.min()}-{cols.max()} n={len(cols)}")
# intrinsics: print distance-based pixel size
print("lid dist from cam", 1.3038-1.03)
EOF
ros2 topic echo /robot0_eye_in_hand/color/camera_info --once 2>/dev/null | grep -A3 "^k:"

# openrua op 22
cat > wrench.py <<'EOF'
import rclpy
from geometry_msgs.msg import WrenchStamped
rclpy.init(); n=rclpy.create_node("w"); got=[]
n.create_subscription(WrenchStamped,"/franka_robot_state_broadcaster/external_wrench",got.append,1)
while not got: rclpy.spin_once(n,timeout_sec=0.2)
f=got[0].wrench.force; t=got[0].wrench.torque
print(f"force ({f.x:.2f},{f.y:.2f},{f.z:.2f}) torque ({t.x:.2f},{t.y:.2f},{t.z:.2f})")
rclpy.shutdown()
EOF
timeout 60 python3 wrench.py; timeout 900 python3 -u mv.py 0.058 0.073 1.10 0.7071068 0.7071068 0 0 3 --tcp; timeout 300 python3 -u fk.py | grep panda_hand; timeout 60 python3 wrench.py

# openrua op 23
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png >/dev/null && timeout 120 python3 cloud.py robot0_eye_in_hand >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load("/workspace/robot0_eye_in_hand_cloud.npy")
z=P[...,2]
m=(z>1.045)&(z<1.07)&(np.abs(P[...,0]-0.06)<0.05)&(np.abs(P[...,1]-0.071)<0.05)
print("knob center", P[m].mean(0), "zmax", P[m][:,2].max())
ml=(z>1.022)&(z<1.042)&(np.abs(P[...,0]-0.06)<0.06)&(np.abs(P[...,1]-0.073)<0.025)
pts=P[ml]; print("lid x extent", pts[:,0].min(), pts[:,0].max(), "center", (pts[:,0].min()+pts[:,0].max())/2)
ml=(z>1.022)&(z<1.042)&(np.abs(P[...,0]-0.058)<0.02)&(np.abs(P[...,1]-0.073)<0.06)
pts=P[ml]; print("lid y extent (incl spout/handle)", pts[:,1].min(), pts[:,1].max())
EOF

# openrua op 24
python3 - <<'EOF'
import numpy as np, cv2
C=np.load("/workspace/robot0_eye_in_hand_color.npy"); P=np.load("/workspace/robot0_eye_in_hand_cloud.npy")
hsv=cv2.cvtColor(C,cv2.COLOR_BGR2HSV)
sat=hsv[...,1]; val=hsv[...,2]
white=(sat<40)&(val>150)
z=P[...,2]; m=(z>1.045)&(z<1.07)&(np.abs(P[...,0]-0.06)<0.05)&(np.abs(P[...,1]-0.071)<0.05)
vs,us=np.where(m); cu,cv=int(us.mean()),int(vs.mean()); print("knob px",cu,cv)
fx=312.774; d=1.2038-1.03  # cam z ~ hand z? use cloud cam pos
import yaml
for r in range(cv-40,cv+41,10):
    row=white[r,cu-90:cu+90]; cols=np.where(row)[0]+cu-90
    if len(cols)==0: continue
    # contiguous segment containing cu
    segs=np.split(cols, np.where(np.diff(cols)>1)[0]+1)
    seg=[s for s in segs if s.min()<=cu<=s.max()]
    if not seg: continue
    s=seg[0]
    print(f"row {r}: white cols {s.min()}-{s.max()} npx={s.max()-s.min()+1} width_m={(s.max()-s.min()+1)*d/fx:.4f}  x_edges {P[r,s.min(),0]:.4f} {P[r,s.max(),0]:.4f}")
print("cam-lid dist used", d)
EOF

# openrua op 25
python3 - <<'EOF'
import numpy as np
P=np.load("/workspace/robot0_eye_in_hand_cloud.npy")
z=P[...,2]
# fingers: z between 1.08 and 1.16 (hand at 1.2038, fingers extend to ~1.09)
m=(z>1.07)&(z<1.18)&np.isfinite(z)
pts=P[m]; print("n finger pts", len(pts))
for zlo in np.arange(1.07,1.18,0.01):
    s=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
    if len(s)<3: continue
    left=s[s[:,0]<0.058]; right=s[s[:,0]>0.058]
    print(f"z {zlo:.2f}: n={len(s)} leftfinger x[{left[:,0].min():.4f},{left[:,0].max():.4f}] rightfinger x[{right[:,0].min():.4f},{right[:,0].max():.4f}] inner gap={right[:,0].min()-left[:,0].max():.4f} y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 26
timeout 900 python3 -u mv.py 0.059 0.0735 1.045 0.7071068 0.7071068 0 0 3 --tcp; timeout 300 python3 -u fk.py | grep panda_hand; timeout 60 python3 wrench.py

# openrua op 27
timeout 900 python3 -u mv.py 0.059 0.0735 1.030 0.7071068 0.7071068 0 0 2 --tcp; timeout 300 python3 -u fk.py | grep panda_hand; timeout 60 python3 wrench.py

# openrua op 28
timeout 900 python3 -u mv.py 0.059 0.0735 1.10 0.7071068 0.7071068 0 0 2 --tcp; timeout 60 python3 wrench.py; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png >/dev/null; timeout 120 python3 cloud.py robot0_eye_in_hand >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load("/workspace/robot0_eye_in_hand_cloud.npy")
z=P[...,2]
m=(z>1.045)&(z<1.07)&(np.abs(P[...,0]-0.06)<0.05)&(np.abs(P[...,1]-0.071)<0.05)
print("knob center", P[m].mean(0), "zmax", P[m][:,2].max())
EOF

# openrua op 29
timeout 60 ros2 topic echo /robot_description --qos-durability transient_local --qos-reliability reliable --once --field data 2>/dev/null > urdf.txt; wc -c urdf.txt; grep -n -E "<link name=\"panda_(hand|leftfinger|rightfinger)\"" -A 25 urdf.txt | grep -E "link name|mesh|box|origin|cylinder" | head -30

# openrua op 30
python3 - <<'EOF'
import numpy as np, struct
def stl_bbox(path):
    d=open(path,'rb').read()
    n=struct.unpack_from('<I',d,80)[0]
    arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
    v=arr['v'].reshape(-1,3)
    return v.min(0),v.max(0)
for f in ["hand","finger"]:
    lo,hi=stl_bbox(f"/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl")
    print(f, "min",lo.round(4),"max",hi.round(4),"size",(hi-lo).round(4))
EOF

# openrua op 31
find / -iname "hand*.stl" -o -iname "finger*.stl" -o -iname "hand.dae" 2>/dev/null | grep -v proc | head; find / -iname "*moka*" 2>/dev/null | head

# openrua op 32
python3 - <<'EOF'
import numpy as np
for cam in ["frontview","sideview","agentview"]:
    try: P=np.load(f"/workspace/{cam}_cloud.npy")
    except: continue
    print(cam)
    m=(P[...,0]>-0.03)&(P[...,0]<0.16)&(P[...,1]>-0.03)&(P[...,1]<0.15)&(P[...,2]>0.905)
    pts=P[m]
    for zlo in np.arange(0.90,1.04,0.005):
        s=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.005)]
        if len(s)<3: continue
        body=s[np.abs(s[:,1]-0.0735)<0.045]
        print(f" z {zlo:.3f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 33
python3 - <<'EOF'
import numpy as np
P=np.load("/workspace/robot0_eye_in_hand_cloud.npy")
z=P[...,2]
cx,cy=0.0589,0.0735
m=(z>1.0)&(z<1.07)&(np.hypot(P[...,0]-cx,P[...,1]-cy)<0.05)
pts=P[m]; r=np.hypot(pts[:,0]-cx,pts[:,1]-cy)
for rlo in np.arange(0,0.045,0.0025):
    s=pts[(r>=rlo)&(r<rlo+0.0025)]
    if len(s)<3: continue
    print(f"r {rlo:.4f}-{rlo+0.0025:.4f}: n={len(s)} z median {np.median(s[:,2]):.4f} max {s[:,2].max():.4f} min {s[:,2].min():.4f}")
# knob: points along x axis through center (|y-cy|<0.003)
s=pts[np.abs(pts[:,1]-cy)<0.003]
for zlo in np.arange(1.030,1.056,0.003):
    q=s[(s[:,2]>=zlo)&(s[:,2]<zlo+0.003)]
    if len(q)<2: continue
    print(f"knob slice z {zlo:.3f}: x[{q[:,0].min():.4f},{q[:,0].max():.4f}] w={q[:,0].max()-q[:,0].min():.4f}")
EOF

# openrua op 34
timeout 900 python3 -u mv.py 0.0589 0.0735 1.048 0.7071068 0.7071068 0 0 2 --tcp; timeout 300 python3 -u fk.py | grep panda_hand; timeout 60 python3 wrench.py

# openrua op 35
timeout 300 python3 tools/action/gripper_cmd.py 0.0; ros2 topic echo /joint_states --once 2>&1 | grep -A9 "^position" | tail -2; timeout 60 python3 wrench.py

# openrua op 36
timeout 900 python3 -u mv.py 0.0589 0.0735 1.10 0.7071068 0.7071068 0 0 2 --tcp; timeout 60 python3 wrench.py; timeout 120 python3 tools/perception/cam_snap.py agentview agent2.png >/dev/null; timeout 120 python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load("/workspace/agentview_cloud.npy"); z=P[...,2]
m=(z>0.905)&(z<1.2)&(P[...,0]>-0.02)&(P[...,0]<0.16)&(P[...,1]>0.0)&(P[...,1]<0.15)
pts=P[m]
print("points in left-pot column: n",len(pts),"zmin",pts[:,2].min() if len(pts) else None,"zmax",pts[:,2].max() if len(pts) else None)
EOF

# openrua op 37
timeout 900 python3 -u mv.py 0.0589 0.0735 1.16 0.7071068 0.7071068 0 0 2 --tcp && timeout 900 python3 -u mv.py -0.044 -0.21 1.16 0.7071068 0.7071068 0 0 4 --tcp; timeout 300 python3 -u fk.py | grep panda_hand; ros2 topic echo /joint_states --once 2>&1 | grep -A9 "^position" | tail -2

# openrua op 38
timeout 120 python3 tools/perception/cam_snap.py agentview agent3.png >/dev/null; timeout 120 python3 cloud.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load("/workspace/birdview_cloud.npy"); z=P[...,2]
# pot hanging: points between z 1.0 and 1.17 near the stove
m=(z>0.98)&(z<1.17)&(P[...,0]>-0.15)&(P[...,0]<0.06)&(P[...,1]>-0.32)&(P[...,1]<-0.1)
pts=P[m]
print("hanging pot: n",len(pts),"x",pts[:,0].min().round(3),pts[:,0].max().round(3),"y",pts[:,1].min().round(3),pts[:,1].max().round(3),"zmax",pts[:,2].max().round(3))
# lid-level points (top of pot ~1.16+0.005-0.15... top = TCP z (1.16) + ~0.005)
lid=pts[pts[:,2]>1.135]
print("pot top region center", lid[:,0].mean().round(4), lid[:,1].mean().round(4), "n", len(lid))
EOF

# openrua op 39
timeout 900 python3 -u mv.py -0.044 -0.21 1.085 0.7071068 0.7071068 0 0 3 --tcp; timeout 60 python3 wrench.py; timeout 900 python3 -u mv.py -0.044 -0.21 1.079 0.7071068 0.7071068 0 0 2 --tcp; timeout 60 python3 wrench.py; timeout 300 python3 -u fk.py | grep panda_hand

# openrua op 40
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 900 python3 -u mv.py -0.044 -0.21 1.20 0.7071068 0.7071068 0 0 3 --tcp; timeout 60 python3 wrench.py

# openrua op 41
timeout 120 python3 tools/perception/cam_snap.py agentview agent_final.png >/dev/null; timeout 120 python3 tools/perception/cam_snap.py sideview side_final.png >/dev/null; timeout 120 python3 cloud.py birdview >/dev/null && timeout 120 python3 cloud.py frontview >/dev/null && python3 - <<'EOF'
import numpy as np
from scipy import ndimage
P=np.load("/workspace/birdview_cloud.npy"); z=P[...,2]
mask=(z>0.935)&(P[...,0]>-0.35)&(P[...,0]<0.5)&(np.abs(P[...,1])<0.5)
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<30: continue
    pts=P[m]
    print(f"blob: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} center=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
F=np.load("/workspace/frontview_cloud.npy")
m=(F[...,0]>-0.12)&(F[...,0]<0.03)&(F[...,1]>-0.30)&(F[...,1]<-0.12)&(F[...,2]>0.935)&(F[...,2]<1.1)
pts=F[m]
print("pot on stove (frontview): zmin", pts[:,2].min().round(4), "zmax", pts[:,2].max().round(4))
k=pts[pts[:,2]>1.075]; print("knob top center", k[:,0].mean().round(3), k[:,1].mean().round(3), k[:,2].max().round(3))
EOF

# openrua op 42
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-kitchen-moka-pot-grasp.md)"
cat > /claude-config/projects/-workspace/memory/panda-kitchen-moka-pot-grasp.md <<'OPENRUA_EOF'
---
name: panda-kitchen-moka-pot-grasp
description: Verified facts for the Franka Panda LIBERO-kitchen workstation — IK frame quirk, gripper true opening, and that moka pots must be pinched by the lid knob
metadata:
  type: project
---

On the Panda kitchen workstation (moka pots + flat stove scene, session 2026-09-12):

- `/compute_ik` poses with empty `frame_id` were interpreted in the WORLD frame (panda_link0 at world (-0.66, 0, 0.912) per `/compute_fk`), contrary to docs/30-action.md "Planning facts". Set `ik_link_name="panda_hand"` or IK returns -31.
- Gripper true finger gap at full open is 0.080 m (measured from wrist depth camera). Moka pot lid rim is ~0.076 m: a top-down body grasp physically collides with the rim. Side grasps are blocked by the pot's handle (at waist height) and by the second pot.
- What works: pinch the lid knob (~1.7 cm cylinder, top at lid+0.02) top-down, TCP ≈ knob_top − 0.005, fingers close to ~0.0057 each. Pot hangs vertically (COM below grasp). Pot bottom ≈ TCP − 0.148.
- "Left/right" in task text follows the agentview camera (image-left = world −y), matching LIBERO BDDL region names.

**Why:** Saves an hour of grasp trial-and-error; the docs' frame claim was wrong for this machine.
**How to apply:** Reuse `mv.py`/`cloud.py`-style IK-in-world + depth-cloud measurement; go straight to the knob pinch for moka pots.
OPENRUA_EOF

# openrua op 43
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda kitchen moka pot grasp](panda-kitchen-moka-pot-grasp.md) — IK is world-frame here; gripper gap 0.080 m; pinch moka pots by the lid knob" >> MEMORY.md && cat MEMORY.md
