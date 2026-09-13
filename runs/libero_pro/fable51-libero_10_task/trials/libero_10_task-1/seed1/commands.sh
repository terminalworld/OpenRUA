#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[220:330,170:380]; cv2.imwrite('crop_cans.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('robot0_eye_in_hand.png')
crop=im[240:360,220:560]; cv2.imwrite('crop_eih.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo ----; timeout 10 ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:"

# openrua op 7
cat > /workspace/tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def cb(m):
    for t in m.transforms:
        got[(t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
for _ in range(20): rclpy.spin_once(node, timeout_sec=0.2)
for (p,c),t in sorted(got.items()):
    print(f"{p} -> {c}: t=({t.translation.x:.4f},{t.translation.y:.4f},{t.translation.z:.4f}) q=({t.rotation.x:.4f},{t.rotation.y:.4f},{t.rotation.z:.4f},{t.rotation.w:.4f})")
EOF
timeout 60 python3 /workspace/tfdump.py

# openrua op 8
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
fx=579.4112549695428; cx=320; cy=240
# camera at z=3.0 looking straight down (q=(.7071,.7071,0,0) rotates optical z-> -world z)
# optical x -> world? compute via rotation
import numpy as np
x,y,z,w=0.7071,0.7071,0,0
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print(R)
# table height: mode of depth in central region
tbl=d[200:260,200:260]; print('table depth median', np.median(tbl), '-> z=', 3.0-np.median(tbl))
"


# openrua op 9
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); f=579.4112549695428; cx=320; cy=240
H=3.0-d
tab=0.425
mask=(H>tab+0.008).astype(np.uint8)
# exclude robot: robot occupies roughly u 270-360, v 120-280
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    x,y,w,h,a=stats[i]
    if a<15: continue
    u,v=cent[i]
    reg=H[lab==i]; 
    # top height = 95th percentile
    top=np.percentile(reg,95)
    z=d[int(v),int(u)]
    wx=-0.2+(v-cy)*z/f; wy=(u-cx)*z/f
    print(f'blob {i}: px=({u:.0f},{v:.0f}) size={w}x{h} area={a} top_z={top:.3f} world=({wx:.3f},{wy:.3f})')
"


# openrua op 10
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); H=3.0-d
for v in [150,200,250,280,300,320]:
    print(v, [round(float(H[v,u]),3) for u in range(150,500,25)])
"


# openrua op 11
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); f=579.4112549695428; cx=320; cy=240
H=3.0-d
mask=np.zeros_like(H,dtype=np.uint8); 
sub=(H>0.445)&(H<0.8); mask[200:340,230:460]=sub[200:340,230:460]
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    x,y,w,h,a=stats[i]
    if a<10: continue
    u,v=cent[i]
    reg=H[lab==i]; top=np.percentile(reg,90)
    z=3.0-top
    wx=-0.2+(v-cy)*z/f; wy=(u-cx)*z/f
    print(f'blob {i}: px=({u:.0f},{v:.0f}) bbox=({x},{y},{w}x{h}) area={a} top_z={top:.3f} world=({wx:.3f},{wy:.3f})')
"


# openrua op 12
cat > /workspace/handpose.py <<'EOF'
import rclpy
from tf2_ros import Buffer, TransformListener
rclpy.init(); node=rclpy.create_node("hp"); buf=Buffer(); TransformListener(buf,node)
import rclpy.time
for _ in range(40):
    rclpy.spin_once(node,timeout_sec=0.1)
    if buf.can_transform("panda_link0","panda_hand",rclpy.time.Time()) and buf.can_transform("world","panda_hand",rclpy.time.Time()): break
for a,b in [("panda_link0","panda_hand"),("world","panda_hand"),("world","panda_leftfinger"),("world","panda_rightfinger")]:
    t=buf.lookup_transform(a,b,rclpy.time.Time()).transform
    print(f"{a}->{b}: t=({t.translation.x:.4f},{t.translation.y:.4f},{t.translation.z:.4f}) q=({t.rotation.x:.4f},{t.rotation.y:.4f},{t.rotation.z:.4f},{t.rotation.w:.4f})")
EOF
timeout 60 python3 /workspace/handpose.py

# openrua op 13
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2

# openrua op 14
timeout 300 python3 tools/action/ik_move.py 0.421 -0.158 0.19 1 0 0 0 3 --at tcp && timeout 60 python3 /workspace/handpose.py | head -2

# openrua op 15
timeout 300 python3 tools/action/ik_move.py 0.421 -0.158 0.2934 1 0 0 0 3 && timeout 60 python3 /workspace/handpose.py | head -2

# openrua op 16
cat > /workspace/ik_try.py <<'EOF'
"""Try IK for a pose with several yaw variants; print solutions. Usage: ik_try.py x y z [yaw_deg...]"""
import sys, math, numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
x,y,z=map(float,sys.argv[1:4]); yaws=[float(a) for a in sys.argv[4:]] or [0]
rclpy.init(); node=rclpy.create_node("iktry")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service(10)
arm=[f"panda_joint{i}" for i in range(1,8)]
cur=dict(zip(js["m"].name,js["m"].position))
for yaw in yaws:
    # top-down: q = (1,0,0,0) rotated about world z by yaw
    h=math.radians(yaw)/2
    # q_z(yaw) * (1,0,0,0): (w,x,y,z) = (cos h,0,0,sin h)*(0,1,0,0) = (0, cos h, sin h, 0)
    qx,qy,qz,qw=math.cos(h),math.sin(h),0.0,0.0
    req=GetPositionIK.Request(); r=req.ik_request
    r.group_name="panda_arm"; r.pose_stamped.header.frame_id=""
    p=r.pose_stamped.pose; p.position.x,p.position.y,p.position.z=x,y,z
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=qx,qy,qz,qw
    r.robot_state.joint_state.name=arm; r.robot_state.joint_state.position=[cur[j] for j in arm]
    r.timeout.sec=2; r.avoid_collisions=False
    fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=60)
    res=fut.result()
    if res is None: print(yaw,"timeout"); continue
    if res.error_code.val!=1: print(f"yaw {yaw}: FAIL {res.error_code.val}"); continue
    sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
    print(f"yaw {yaw}: OK", ",".join(f"{sol[j]:.4f}" for j in arm))
EOF
timeout 300 python3 /workspace/ik_try.py 0.421 -0.158 0.2934 0 10 -10 30 -30 45 -45 90

# openrua op 17
timeout 300 python3 /workspace/ik_try.py 0.457 0.0 0.3576 0 3.25; timeout 300 python3 /workspace/ik_try.py 0.457 0.0 0.30 0

# openrua op 18
timeout 60 ros2 param get /move_group robot_description_semantic 2>&1 | grep -iE "group name|chain|tip|end_effector|link name" | head -20; timeout 30 ros2 param list /move_group 2>/dev/null | grep -iE "kinematics|solver|panda_arm" | head

# openrua op 19
timeout 60 ros2 param get /move_group robot_description_semantic 2>&1 | grep -oE "openrua assembly note.{0,1500}" ; timeout 60 ros2 param get /move_group robot_description_kinematics.panda_arm.kinematics_solver; timeout 60 ros2 param get /move_group robot_description_kinematics.panda_arm.kinematics_solver_timeout

# openrua op 20
timeout 60 ros2 param get /move_group robot_description_semantic 2>&1 | tr '\n' ' ' | grep -oE "openrua assembly note.{0,2000}"

# openrua op 21
cat > /workspace/ik_try2.py <<'EOF'
import sys, math, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
rclpy.init(); node=rclpy.create_node("iktry")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service(10)
arm=[f"panda_joint{i}" for i in range(1,8)]
cur=dict(zip(js["m"].name,js["m"].position))
def ik(label,pos,q,frame="",link="",avoid=False):
    req=GetPositionIK.Request(); r=req.ik_request
    r.group_name="panda_arm"; r.pose_stamped.header.frame_id=frame; r.ik_link_name=link
    p=r.pose_stamped.pose; p.position.x,p.position.y,p.position.z=pos
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q
    r.robot_state.joint_state.name=arm; r.robot_state.joint_state.position=[cur[j] for j in arm]
    r.timeout.sec=1; r.avoid_collisions=avoid
    fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=60)
    res=fut.result()
    if res is None: print(label,"timeout"); return
    if res.error_code.val!=1: print(f"{label}: FAIL {res.error_code.val}"); return
    sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
    print(f"{label}: OK", ",".join(f"{sol[j]:.4f}" for j in arm))
qh=(0.9996,0,-0.0284,0)
# link8 quat = qh * Rz(45)
def qmul(a,b):
    x1,y1,z1,w1=a; x2,y2,z2,w2=b
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)
q8=qmul(qh,(0,0,0.38268,0.92388)); print("q8",q8)
ik("base,hand q",(0.457,0,0.3576),qh)
ik("base,link8 q",(0.457,0,0.3576),q8)
ik("world,hand q",(-0.053,0,0.7776),qh)
ik("world,link8 q",(-0.053,0,0.7776),q8)
ik("world frame_id,link8 q",(-0.053,0,0.7776),q8,frame="world")
ik("base frame_id,link8 q",(0.457,0,0.3576),q8,frame="panda_link0")
ik("base,link8 q, link=panda_hand",(0.457,0,0.3576),qh,link="panda_hand")
EOF
timeout 300 python3 /workspace/ik_try2.py

# openrua op 22
sed -i 's/p.position.x,p.position.y,p.position.z=pos/p.position.x,p.position.y,p.position.z=[float(v) for v in pos]/; s/p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q$/p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=[float(v) for v in q]/' ik_try2.py && timeout 300 python3 /workspace/ik_try2.py

# openrua op 23
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small arm helper for this Panda (verified conventions):
 - /compute_ik: frame_id "" == WORLD frame; tip link is panda_link8
   (hand quat * Rz(+45deg)).
 - TCP = hand origin + 0.1034 along hand +Z (down for top-down grasps).

CLI:
  arm.py tcp X Y Z [yaw_deg] [secs]    move fingertip point to world XYZ, top-down,
                                       fingers separating along world y rotated by yaw
  arm.py grip W                        gripper per-finger width (0.04 open, 0.0 close)
  arm.py js                            print joints / finger gap / hand pose
"""
import math, sys, time
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
import rclpy.time

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034


def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2, w1*w2 - x1*x2 - y1*y2 - z1*z2)


def topdown_hand_q(yaw_deg):
    h = math.radians(yaw_deg) / 2
    return (math.cos(h), math.sin(h), 0.0, 0.0)   # Rz(yaw) * Rx(180)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grp = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.buf = Buffer(); TransformListener(self.buf, self.node)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
        while "m" not in self.js:
            self.spin(0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def hand_pose(self):
        for _ in range(50):
            self.spin(0.1)
            if self.buf.can_transform("world", "panda_hand", rclpy.time.Time()):
                break
        t = self.buf.lookup_transform("world", "panda_hand", rclpy.time.Time()).transform
        return ((t.translation.x, t.translation.y, t.translation.z),
                (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))

    def solve_ik(self, hand_pos_world, hand_q, seed=None):
        self.ik.wait_for_service(10)
        q8 = qmul(hand_q, (0, 0, 0.3826834, 0.9238795))
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = "panda_arm"; r.pose_stamped.header.frame_id = ""
        p = r.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = [float(v) for v in hand_pos_world]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = [float(v) for v in q8]
        cur = seed or self.joints(fresh=False)
        r.robot_state.joint_state.name = ARM
        r.robot_state.joint_state.position = [float(cur[j]) for j in ARM]
        r.timeout.sec = 1; r.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"IK failed ({code}) for {hand_pos_world}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, positions, secs):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        cur = self.joints()
        err = max(abs(cur[j] - p) for j, p in zip(ARM, positions))
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def tcp(self, x, y, z, yaw=0.0, secs=3.0):
        hq = topdown_hand_q(yaw)
        # hand +Z (approach) points down for top-down: hand origin is TCP above by TCP
        hand = (x, y, z + TCP)
        sol = self.solve_ik(hand, hq)
        code, err = self.move_joints(sol, secs)
        pos, q = self.hand_pose()
        print(f"  hand now world=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}) "
              f"tcp z={pos[2]-TCP:.4f}")
        return pos

    def grip(self, width):
        self.grp.wait_for_server(10)
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
        fut = self.grp.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        j = self.joints()
        gap = j["panda_finger_joint1"] - j["panda_finger_joint2"]
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def status(self):
        j = self.joints()
        print("joints:", ",".join(f"{j[n]:.4f}" for n in ARM))
        print(f"finger gap: {j['panda_finger_joint1'] - j['panda_finger_joint2']:.4f}")
        pos, q = self.hand_pose()
        print(f"hand world=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}) q=({q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f}) tcp z={pos[2]-TCP:.4f}")


if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "tcp":
        v = [float(s) for s in sys.argv[2:]]
        a.tcp(*v)
    elif cmd == "grip":
        a.grip(float(sys.argv[2]))
    elif cmd == "js":
        a.status()
    rclpy.shutdown()
OPENRUA_EOF

# openrua op 24
timeout 300 python3 arm.py tcp -0.089 -0.158 0.62 0 3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:" && python3 -c "
import numpy as np, cv2
d=np.load('eih_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
# camera ~0.77 above table(0.425)-> table depth ~0.345; can top at 0.501 -> ~0.27
m=((d>0.2)&(d<0.30)).astype(np.uint8)
n,lab,st,cen=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    if st[i,4]>200: print(i, st[i], cen[i], np.median(d[lab==i]))
"

# openrua op 26
python3 -c "
import numpy as np
d=np.load('eih_depth.npy')
h,e=np.histogram(d,bins=30,range=(0,0.31)); 
for c,lo in zip(h,e): print(f'{lo:.3f} {c}')
print(d[300,318], d[100,200], d[250,60])
"

# openrua op 27
python3 -c "
import numpy as np, cv2
d=np.load('eih_depth.npy')
m=((d>0.205)&(d<0.225)).astype(np.uint8)
n,lab,st,cen=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    if st[i,4]>200: print(i, st[i], cen[i])
"

# openrua op 28
timeout 60 python3 /workspace/tfdump.py | grep -E "eye_in_hand|panda_hand:" ; python3 -c "
import cv2, numpy as np
im=cv2.imread('robot0_eye_in_hand.png'); g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
row=g[304,200:440]; print('row304', ''.join('#' if v>90 else '.' for v in row))
col=g[200:400,318]; print('col318', ''.join('#' if v>90 else '.' for v in col))
"

# openrua op 29
python3 -c "
import cv2, numpy as np
im=cv2.imread('robot0_eye_in_hand.png'); g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY).astype(int)
print('row304', list(g[304,250:390:4]))
print('col318', list(g[240:370:4,318]))
"

# openrua op 30
timeout 300 python3 arm.py tcp -0.085 -0.158 0.52 0 2 && timeout 300 python3 arm.py tcp -0.085 -0.158 0.472 0 2 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 31
timeout 300 python3 arm.py grip 0.0

# openrua op 32
timeout 300 python3 arm.py tcp -0.085 -0.158 0.70 0 2.5 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2

# openrua op 33
timeout 300 python3 arm.py tcp -0.014 0.26 0.74 0 3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 34
timeout 300 python3 arm.py tcp -0.014 0.26 0.68 0 2 && timeout 300 python3 arm.py grip 0.04 && timeout 300 python3 arm.py tcp -0.014 0.26 0.80 0 2 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 35
timeout 300 python3 arm.py tcp 0.060 0.038 0.62 0 3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 /workspace/tfdump.py | grep eye_in_hand

# openrua op 36
python3 -c "
import cv2, numpy as np
im=cv2.imread('robot0_eye_in_hand.png')
hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
# butter box: red/yellow/blue-white; segment by 'not wood' in a window around it
win=im[220:360,260:380]
h,s,v=cv2.split(cv2.cvtColor(win,cv2.COLOR_BGR2HSV))
# wood is brownish, low saturation moderately dark; the box has high sat red/yellow or bright white/blue
m=((s>120)&(v>120))|(v>150)
m=m.astype(np.uint8)*255
m=cv2.morphologyEx(m,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
cnts,_=cv2.findContours(m,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
c=max(cnts,key=cv2.contourArea)
(cx,cy),(w,hh),ang=cv2.minAreaRect(c)
print('center px',cx+260,cy+220,'size',w,hh,'angle',ang, 'area',cv2.contourArea(c))
"


# openrua op 37
timeout 300 python3 arm.py tcp 0.064 0.040 0.50 0 2 && timeout 300 python3 arm.py tcp 0.064 0.040 0.440 0 2 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 38
timeout 300 python3 arm.py grip 0.0 && timeout 300 python3 arm.py tcp 0.064 0.040 0.60 0 2.5 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2

# openrua op 39
timeout 300 python3 arm.py tcp -0.014 0.305 0.74 0 3 && timeout 300 python3 arm.py tcp -0.014 0.305 0.67 0 2 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 40
timeout 300 python3 arm.py grip 0.04 && timeout 300 python3 arm.py tcp -0.014 0.26 0.82 0 2.5 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 41
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth2.png >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth2.npy'); f=579.4112549695428; cx=320; cy=240
H=3.0-d
# basket footprint in birdview px: bbox (363,266,42x40) -> interior
print('basket interior heights (rows 270..302 step 4, cols 367..401 step 4):')
for v in range(270,304,4):
    print(v, ' '.join(f'{H[v,u]:.3f}' for u in range(367,402,4)))
# objects on the open table (outside basket & robot)
mask=np.zeros_like(H,dtype=np.uint8); sub=(H>0.445)&(H<0.8); mask[200:340,230:460]=sub[200:340,230:460]
n,lab,st,cen=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    x,y,w,h,a=st[i]
    if a<10: continue
    u,v=cen[i]; top=np.percentile(H[lab==i],90); z=3.0-top
    print(f'blob px=({u:.0f},{v:.0f}) bbox=({x},{y},{w}x{h}) top_z={top:.3f} world=({-0.2+(v-cy)*z/f:.3f},{(u-cx)*z/f:.3f})')
"

# openrua op 42
timeout 300 python3 arm.py tcp -0.10 0.0 0.72 0 3 && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth3.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth3.npy'); f=579.4112549695428; cx=320; cy=240
H=3.0-d
print('basket interior heights (rows 266..306 step 4, cols 363..405 step 4):')
for v in range(266,308,4):
    print(v, ' '.join(f'{H[v,u]:.3f}' for u in range(363,406,4)))
mask=np.zeros_like(H,dtype=np.uint8); sub=(H>0.445)&(H<0.8); mask[200:340,230:460]=sub[200:340,230:460]
n,lab,st,cen=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    x,y,w,h,a=st[i]
    if a<10: continue
    u,v=cen[i]; top=np.percentile(H[lab==i],90); z=3.0-top
    print(f'blob px=({u:.0f},{v:.0f}) bbox=({x},{y},{w}x{h}) top_z={top:.3f} world=({-0.2+(v-cy)*z/f:.3f},{(u-cx)*z/f:.3f})')
"

# openrua op 43
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[250:320,340:420]; cv2.imwrite('crop_basket.png', cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))"
