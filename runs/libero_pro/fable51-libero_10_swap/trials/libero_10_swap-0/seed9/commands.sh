#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | head -50; echo ----; timeout 20 ros2 node list

# openrua op 4
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('robot0_robotview.png')
crop=im[0:260,0:260]; cv2.imwrite('crop_cans.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im2=cv2.imread('agentview.png')
crop=im2[190:400,60:420]; cv2.imwrite('crop_agent.png', cv2.resize(crop,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 30 ros2 topic echo /tf_static --once --qos-durability transient_local 2>&1 | grep -E "frame_id|translation|rotation|x:|y:|z:|w:" | head -120

# openrua op 7
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node=rclpy.create_node("tfdump")
got={}
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def cb(m, key):
    for t in m.transforms:
        got[(key,t.header.frame_id,t.child_frame_id)]=t
node.create_subscription(TFMessage,"/tf_static",lambda m: cb(m,"static"),qos)
node.create_subscription(TFMessage,"/tf",lambda m: cb(m,"dyn"),10)
import time
end=time.time()+5
while time.time()<end: rclpy.spin_once(node,timeout_sec=0.2)
for k,t in sorted(got.items()):
    tr=t.transform.translation; q=t.transform.rotation
    print(k, f"t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
timeout 60 python3 tfdump.py

# openrua op 8
cat > depthworld.py <<'EOF'
"""Compute world xyz for every pixel of a camera's depth frame; save npy."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node("dw")
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m: got.setdefault("d",m),1)
node.create_subscription(Image,f"/{cam}/color/image_raw",lambda m: got.setdefault("c",m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m: got.setdefault("i",m),1)
def tfcb(m):
    for t in m.transforms:
        if t.child_frame_id==f"{cam}_optical_frame": got["tf"]=t
node.create_subscription(TFMessage,"/tf",tfcb,10)
while not all(k in got for k in "d c i tf".split()): rclpy.spin_once(node,timeout_sec=0.2)
d=CvBridge().imgmsg_to_cv2(got["d"],"passthrough").astype(np.float64)
c=CvBridge().imgmsg_to_cv2(got["c"],"bgr8")
K=np.array(got["i"].k).reshape(3,3)
t=got["tf"].transform; q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
H,W=d.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
X=(u-K[0,2])*d/K[0,0]; Y=(v-K[1,2])*d/K[1,1]
P=np.stack([X,Y,d],-1)@R.T+T
np.save(f"{cam}_world.npy",P); np.save(f"{cam}_rgb.npy",c)
print("K",K.tolist()); print("shape",d.shape, "depth range",np.nanmin(d),np.nanmax(d))
print("center world",P[H//2,W//2])
rclpy.shutdown()
EOF
timeout 120 python3 depthworld.py birdview && timeout 120 python3 depthworld.py agentview

# openrua op 9
cat > segment.py <<'EOF'
import numpy as np, cv2, sys
cam=sys.argv[1]
P=np.load(f"{cam}_world.npy"); c=np.load(f"{cam}_rgb.npy")
z=P[...,2]
mask=(z>0.445)&(z<0.9)&np.isfinite(z)
# exclude robot: robot base around world x<-0.3 ... keep only x>-0.45? robot base at x=-0.51
mask&=(P[...,0]>-0.35)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<15: continue
    m=lab==i
    pts=P[m]; col=c[m].mean(0)
    print(f"blob {i}: px area={stats[i,4]} centroid px=({cent[i][0]:.0f},{cent[i][1]:.0f}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} meanBGR={col.astype(int)}")
cv2.imwrite(f"{cam}_mask.png",mask*255)
EOF
python3 segment.py birdview

# openrua op 10
python3 segment.py agentview

# openrua op 11
python3 - <<'EOF'
import numpy as np
for cam in ["agentview","birdview"]:
    P=np.load(f"{cam}_world.npy")
    z=P[...,2]
    m=(z>0.445)&(z<0.9)&(P[...,0]<-0.17)&(P[...,0]>-0.3)&(P[...,1]<-0.08)&(P[...,1]>-0.25)
    pts=P[m]
    print(cam,"alphabet soup pts",len(pts),"x",pts[:,0].min(),pts[:,0].max(),"y",pts[:,1].min(),pts[:,1].max(),"zmax",pts[:,2].max())
    top=pts[pts[:,2]>pts[:,2].max()-0.01]
    print("  top surface center",top[:,:2].mean(0), "n",len(top))
    m=(z>0.445)&(z<0.9)&(P[...,0]<0.06)&(P[...,0]>-0.06)&(P[...,1]<-0.22)&(P[...,1]>-0.33)
    pts=P[m]
    print(cam,"tomato sauce pts",len(pts),"x",pts[:,0].min(),pts[:,0].max(),"y",pts[:,1].min(),pts[:,1].max(),"zmax",pts[:,2].max())
    top=pts[pts[:,2]>pts[:,2].max()-0.01]
    print("  top surface center",top[:,:2].mean(0), "n",len(top))
    # basket
    m=(z>0.445)&(z<0.9)&(P[...,1]>0.15)&(P[...,0]>-0.15)&(P[...,0]<0.15)
    pts=P[m]
    print(cam,"basket x",pts[:,0].min(),pts[:,0].max(),"y",pts[:,1].min(),pts[:,1].max(),"zmax",pts[:,2].max())
    rim=pts[pts[:,2]>0.60]
    print("  rim center",rim[:,:2].mean(0))
EOF

# openrua op 12
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small controller: IK -> trajectory, gripper, FK readout.

Usage:
  ctl.py fk                         print hand + TCP pose (world frame), finger gap
  ctl.py goto X Y Z YAW [SECONDS]   move TCP (fingertip point) to world x,y,z,
                                    hand pointing down, fingers sliding along
                                    world axis rotated YAW deg from +Y
  ctl.py joints p1,...,p7 [SECONDS] raw joint move
  ctl.py open | close               gripper
"""
import sys, math
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]
BASE = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (from /tf)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    w = math.sqrt(max(0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.sqrt(max(0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = math.sqrt(max(0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = math.sqrt(max(0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = math.copysign(x, R[2, 1] - R[1, 2])
    y = math.copysign(y, R[0, 2] - R[2, 0])
    z = math.copysign(z, R[1, 0] - R[0, 1])
    return x, y, z, w


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])

    def joint_state(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_seed(self):
        js = self.joint_state()
        s = JointState()
        for j in JOINTS:
            s.name.append(j); s.position.append(js[j])
        return s, js

    def fk_pose(self):
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        seed, js = self.arm_seed()
        req.robot_state.joint_state = seed
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        R = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        hand = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        tcp = hand + TCP * R[:, 2]
        return hand, tcp, R, js

    def print_fk(self):
        hand, tcp, R, js = self.fk_pose()
        gap = js.get("panda_finger_joint1", float("nan"))
        print(f"hand(world)={hand.round(4).tolist()} tcp(world)={tcp.round(4).tolist()} "
              f"hand_z_axis={R[:,2].round(3).tolist()} hand_y_axis={R[:,1].round(3).tolist()} finger1={gap:.4f}")
        print("joints:", [round(js[j], 4) for j in JOINTS])
        return hand, tcp

    def solve_ik(self, tcp_world, yaw_deg):
        yaw = math.radians(yaw_deg)
        # hand z down; hand y (finger slide axis) along world +Y rotated by yaw about Z
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        R = Rz @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)
        hand_world = np.array(tcp_world, float) - TCP * R[:, 2]
        hand_base = hand_world - BASE
        qx, qy, qz, qw = R_to_quat(R)
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = hand_base.tolist()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        seed, js = self.arm_seed()
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK FAILED code={None if res is None else res.error_code.val} target hand_base={hand_base.round(3).tolist()}")
            return None, js
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        q = [sol[j] for j in JOINTS]
        dist = sum(abs(sol[j] - js[j]) for j in JOINTS)
        print(f"IK ok: hand_base={hand_base.round(3).tolist()} q={[round(v,3) for v in q]} L1 move={dist:.2f} rad")
        return q, js

    def move_joints(self, q, seconds):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        result = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result)
        code = result.result().result.error_code
        js = self.joint_state()
        err = max(abs(js[j] - v) for j, v in zip(JOINTS, q))
        print(f"trajectory done error_code={code} max joint err={err:.4f} rad")
        return code, err

    def goto(self, tcp_world, yaw_deg, seconds):
        q, js = self.solve_ik(tcp_world, yaw_deg)
        if q is None:
            return False
        self.move_joints(q, seconds)
        hand, tcp, R, _ = self.fk_pose()
        e = np.linalg.norm(tcp - np.array(tcp_world))
        print(f"reached tcp(world)={tcp.round(4).tolist()} target={np.round(tcp_world,4).tolist()} err={e*1000:.1f} mm")
        return e < 0.01

    def gripper(self, width):
        self.grip.wait_for_server(10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res_fut = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res_fut, timeout_sec=300)
        res = res_fut.result().result
        js = self.joint_state()
        print(f"gripper cmd={width} reached_goal={res.reached_goal} stalled={res.stalled} "
              f"finger1={js['panda_finger_joint1']:.4f} finger2={js['panda_finger_joint2']:.4f}")
        return js["panda_finger_joint1"]


def main():
    a = sys.argv[1:]
    c = Ctl()
    cmd = a[0]
    if cmd == "fk":
        c.print_fk()
    elif cmd == "goto":
        x, y, z, yaw = map(float, a[1:5])
        sec = float(a[5]) if len(a) > 5 else 4.0
        c.goto([x, y, z], yaw, sec)
    elif cmd == "joints":
        q = [float(v) for v in a[1].split(",")]
        sec = float(a[2]) if len(a) > 2 else 4.0
        c.move_joints(q, sec)
        c.print_fk()
    elif cmd == "open":
        c.gripper(GRIP["open_m"])
    elif cmd == "close":
        c.gripper(GRIP["closed_m"])
    else:
        raise SystemExit(__doc__)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 13
timeout 120 python3 -u ctl.py fk

# openrua op 14
timeout 300 python3 -u ctl.py open

# openrua op 15
timeout 600 python3 -u ctl.py goto -0.230 -0.155 0.60 0 4

# openrua op 16
python3 - <<'EOF'
import re
s=open("ctl.py").read()
s=s.replace("""        seed, js = self.arm_seed()
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK FAILED code={None if res is None else res.error_code.val} target hand_base={hand_base.round(3).tolist()}")
            return None, js
""","""        seed, js = self.arm_seed()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        seeds = [[js[j] for j in JOINTS], [0, -0.785, 0, -2.356, 0, 1.571, 0.785],
                 [0, -0.3, 0, -2.2, 0, 2.0, 0.785], [0.3, 0.2, -0.3, -1.8, 0.2, 2.0, 0.5]]
        rng = np.random.default_rng(0)
        lim = np.array(FJT["limits_rad"])
        for _ in range(6):
            seeds.append(rng.uniform(lim[:, 0], lim[:, 1]).tolist())
        res = None
        for sd in seeds:
            seed.position = [float(v) for v in sd]
            req.ik_request.robot_state.joint_state = seed
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                break
            print(f"  IK attempt failed code={None if res is None else res.error_code.val}")
        if res is None or res.error_code.val != 1:
            print(f"IK FAILED code={None if res is None else res.error_code.val} target hand_base={hand_base.round(3).tolist()}")
            return None, js
""")
open("ctl.py","w").write(s)
EOF
timeout 600 python3 -u ctl.py goto -0.230 -0.155 0.60 0 4

# openrua op 17
cat > iktest.py <<'EOF'
import rclpy, numpy as np, sys
sys.argv=["x"]
import ctl
c=ctl.Ctl()
from moveit_msgs.srv import GetPositionIK, GetPositionFK
# FK current
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand","panda_link8"]
seed,js=c.arm_seed(); req.robot_state.joint_state=seed
fut=c.fk.call_async(req); rclpy.spin_until_future_complete(c.node,fut,timeout_sec=30)
res=fut.result()
print("fk frame", res.pose_stamped[0].header.frame_id)
for ps in res.pose_stamped: 
    p=ps.pose; print(ps.header.frame_id, p.position, p.orientation)
pose=res.pose_stamped[0].pose
def ik(pose, tip=None, frame=""):
    r=GetPositionIK.Request(); r.ik_request.group_name="panda_arm"
    r.ik_request.pose_stamped.header.frame_id=frame
    r.ik_request.pose_stamped.pose=pose
    if tip: r.ik_request.ik_link_name=tip
    r.ik_request.robot_state.joint_state=seed
    r.ik_request.timeout.sec=2
    f=c.ik.call_async(r); rclpy.spin_until_future_complete(c.node,f,timeout_sec=60)
    o=f.result(); print("ik code",o.error_code.val, [round(v,3) for v in o.solution.joint_state.position[:7]] if o.error_code.val==1 else "")
ik(pose)
ik(pose, tip="panda_hand")
ik(pose, tip="panda_link8")
import copy
p2=copy.deepcopy(pose); p2.position.z-=0.2; ik(p2)
p3=copy.deepcopy(pose); p3.position.x+=0.3; p3.position.z-=0.4; ik(p3)
EOF
timeout 300 python3 -u iktest.py

# openrua op 18
sed -i 's/req.fk_link_names=\["panda_hand","panda_link8"\]/req.fk_link_names=["panda_hand"]/; s/timeout_sec=30)/timeout_sec=120)/' iktest.py && timeout 600 python3 -u iktest.py

# openrua op 19
cat >> iktest.py <<'EOF'
print("---- panda_hand tip tests")
from geometry_msgs.msg import Pose
def mk(x,y,z,q=(1,0,0,0)):
    p=Pose(); p.position.x,p.position.y,p.position.z=x,y,z
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q; return p
for tgt in [(0.3,0,0.5),(0.3,0,0.4),(0.3,0,0.3),(0.28,-0.155,0.283),(0.4,-0.2,0.3),(0.5,-0.28,0.25)]:
    print(tgt, end=" "); ik(mk(*tgt), tip="panda_hand")
seed.position=[0.0,-0.785,0.0,-2.356,0.0,1.571,0.785]
print("ready seed")
for tgt in [(0.3,0,0.5),(0.3,0,0.3),(0.28,-0.155,0.283),(0.5,-0.28,0.25)]:
    print(tgt, end=" "); ik(mk(*tgt), tip="panda_hand")
EOF
timeout 600 python3 -u iktest.py 2>&1 | tail -14

# openrua op 20
python3 - <<'EOF'
s=open("ctl.py").read()
s=s.replace("""    def joint_state(self):""","""    def call(self, client, req, tries=6, timeout=20.0):
        client.wait_for_service(10)
        for i in range(tries):
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            if fut.done() and fut.result() is not None:
                return fut.result()
            print(f"  service call attempt {i+1} timed out; retrying")
        return None

    def joint_state(self):""")
s=s.replace("""        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose""","""        res = self.call(self.fk, req)
        p = res.pose_stamped[0].pose""")
s=s.replace("""            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None""","""            res = self.call(self.ik, req)
            if res is not None""")
s=s.replace("""        req.ik_request.pose_stamped.header.frame_id = \"\"""","""        req.ik_request.pose_stamped.header.frame_id = \"\"
        req.ik_request.ik_link_name = "panda_hand"  # default tip is link8 (45 deg off)""")
open("ctl.py","w").write(s)

t=open("iktest.py").read()
t=t.replace("""fut=c.fk.call_async(req); rclpy.spin_until_future_complete(c.node,fut,timeout_sec=120)
res=fut.result()""","""res=c.call(c.fk,req)""")
t=t.replace("""    f=c.ik.call_async(r); rclpy.spin_until_future_complete(c.node,f,timeout_sec=60)
    o=f.result();""","""    o=c.call(c.ik,r);""")
open("iktest.py","w").write(t)
EOF
timeout 900 python3 -u iktest.py 2>&1 | tail -16

# openrua op 21
sed -i 's/p.position.x,p.position.y,p.position.z=x,y,z/p.position.x,p.position.y,p.position.z=float(x),float(y),float(z)/; s/p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q; return p/p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=[float(v) for v in q]; return p/' iktest.py && timeout 900 python3 -u iktest.py 2>&1 | tail -12

# openrua op 22
python3 - <<'EOF'
import numpy as np
def q2R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
chain=[((0,0,0.333),(0,0,0,1)),((0,0,0),(-0.7048,-0.0569,-0.0569,0.7048)),((0,-0.316,0),(0.7071,0,0,0.7071)),((0.0825,0,0),(-0.2415,-0.6646,0.6646,-0.2415)),((-0.0825,0.384,0),(-0.7071,0,0,0.7071)),((0,0,0),(0.3123,-0.6344,0.6344,0.3123)),((0.088,0,0),(0.6533,-0.2706,0.2706,0.6533)),((0,0,0.107),(0,0,0,1)),((0,0,0),(0,0,-0.3827,0.9239))]
T=np.eye(4)
for t,q in chain:
    A=np.eye(4); A[:3,:3]=q2R(*q); A[:3,3]=t; T=T@A
print("hand in link0 from TF chain:",T[:3,3].round(4)); print(T[:3,:3].round(3))
EOF

# openrua op 23
python3 - <<'EOF'
s=open("ctl.py").read()
s=s.replace('BASE = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (from /tf)',
 '# MoveIt FK/IK poses are in the WORLD frame (verified against the TF chain:\n# link0 sits at world (-0.51, 0, 0.42) and the planner includes that offset).\nBASE = np.array([0.0, 0.0, 0.0])')
open("ctl.py","w").write(s)
EOF
timeout 300 python3 -u ctl.py fk

# openrua op 24
timeout 900 python3 -u ctl.py goto -0.230 -0.155 0.60 0 4

# openrua op 25
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png && timeout 120 python3 tools/perception/cam_snap.py agentview agent1.png

# openrua op 26
timeout 900 python3 -u ctl.py goto -0.230 -0.155 0.467 0 3 && timeout 300 python3 -u ctl.py close

# openrua op 27
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png && timeout 120 python3 tools/perception/cam_snap.py agentview agent2.png && timeout 300 python3 -u ctl.py fk

# openrua op 28
python3 - <<'EOF'
s=open("ctl.py").read()
old_start=s.index("        seed, js = self.arm_seed()\n        req.ik_request.avoid_collisions = False")
old_end=s.index("        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))")
new='''        seed, js = self.arm_seed()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        cur = np.array([js[j] for j in JOINTS])
        lim = np.array(FJT["limits_rad"])
        rng = np.random.default_rng(0)
        seeds = [cur.tolist()]
        for _ in range(5):  # small perturbations of the current config
            seeds.append(np.clip(cur + rng.normal(0, 0.15, 7), lim[:, 0], lim[:, 1]).tolist())
        seeds += [[0, -0.785, 0, -2.356, 0, 1.571, 0.785], [0, -0.3, 0, -2.2, 0, 2.0, 0.785]]
        for _ in range(4):
            seeds.append(rng.uniform(lim[:, 0], lim[:, 1]).tolist())
        best = None
        for sd in seeds:
            seed.position = [float(v) for v in sd]
            req.ik_request.robot_state.joint_state = seed
            res = self.call(self.ik, req)
            if res is None or res.error_code.val != 1:
                continue
            sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            q = np.array([sol[j] for j in JOINTS])
            dist = np.abs(q - cur).sum()
            if best is None or dist < best[0]:
                best = (dist, q)
            if dist < 0.8:
                break
        if best is None:
            print(f"IK FAILED target hand(world)={hand_base.round(3).tolist()}")
            return None, js
        dist, q = best
        print(f"IK ok: hand(world)={hand_base.round(3).tolist()} q={[round(v,3) for v in q]} L1 move={dist:.2f} rad")
        return q.tolist(), js
'''
s=s[:old_start]+new+s[s.index("    def move_joints"):]
open("ctl.py","w").write(s)
EOF
grep -n "def solve_ik" -A 60 ctl.py | sed -n '1,70p' | tail -20

# openrua op 29
timeout 300 python3 -u ctl.py open && timeout 900 python3 -u ctl.py goto -0.230 -0.155 0.60 0 4

# openrua op 30
timeout 120 python3 tools/perception/cam_snap.py agentview agent3.png

# openrua op 31
timeout 120 python3 depthworld.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load("agentview_world.npy"); z=P[...,2]
m=(z>0.445)&(z<0.53)&(P[...,0]<-0.17)&(P[...,0]>-0.3)&(P[...,1]<-0.08)&(P[...,1]>-0.25)
pts=P[m]
print("alphabet soup pts",len(pts),"x",pts[:,0].min().round(3),pts[:,0].max().round(3),"y",pts[:,1].min().round(3),pts[:,1].max().round(3),"zmax",pts[:,2].max().round(3))
top=pts[pts[:,2]>pts[:,2].max()-0.01]; print(" top center",top[:,:2].mean(0).round(3),len(top))
EOF

# openrua op 32
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png

# openrua op 33
timeout 900 python3 -u ctl.py goto -0.229 -0.154 0.53 0 3 && timeout 900 python3 -u ctl.py goto -0.229 -0.154 0.467 0 3

# openrua op 34
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png

# openrua op 35
timeout 300 python3 -u ctl.py close

# openrua op 36
timeout 900 python3 -u ctl.py goto -0.229 -0.154 0.72 0 3 && timeout 120 python3 tools/perception/cam_snap.py agentview agent4.png

# openrua op 37
timeout 900 python3 -u ctl.py goto -0.229 -0.154 0.72 0 3 && timeout 900 python3 -u ctl.py goto -0.025 0.24 0.72 0 6

# openrua op 38
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih5.png && timeout 120 python3 tools/perception/cam_snap.py agentview agent5.png && timeout 60 ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 39
timeout 900 python3 -u ctl.py goto -0.025 0.24 0.55 0 3 && timeout 300 python3 -u ctl.py open && timeout 900 python3 -u ctl.py goto -0.025 0.24 0.72 0 3

# openrua op 40
timeout 120 python3 tools/perception/cam_snap.py agentview agent6.png && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih6.png

# openrua op 41
timeout 900 python3 -u ctl.py goto 0.001 -0.277 0.72 0 6 && timeout 900 python3 -u ctl.py goto 0.001 -0.277 0.60 0 3 && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih7.png

# openrua op 42
timeout 900 python3 -u ctl.py goto 0.001 -0.277 0.54 0 3 && timeout 900 python3 -u ctl.py goto 0.001 -0.277 0.48 0 3 && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih8.png

# openrua op 43
timeout 300 python3 -u ctl.py close && timeout 900 python3 -u ctl.py goto 0.001 -0.277 0.72 0 4 && timeout 120 python3 tools/perception/cam_snap.py agentview agent7.png

# openrua op 44
timeout 900 python3 -u ctl.py goto 0.0 0.27 0.74 0 6 && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih9.png

# openrua op 45
timeout 120 python3 depthworld.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load("agentview_world.npy"); z=P[...,2]
m=(z>0.44)&(z<0.60)&(P[...,0]>-0.08)&(P[...,0]<0.07)&(P[...,1]>0.18)&(P[...,1]<0.35)
pts=P[m]
print("pts inside basket footprint",len(pts))
if len(pts):
    print("x",pts[:,0].min().round(3),pts[:,0].max().round(3),"y",pts[:,1].min().round(3),pts[:,1].max().round(3),"z",pts[:,2].min().round(3),pts[:,2].max().round(3))
    top=pts[pts[:,2]>pts[:,2].max()-0.015]; print("top center",top[:,:2].mean(0).round(3),len(top))
EOF

# openrua op 46
python3 - <<'EOF'
import numpy as np
P=np.load("agentview_world.npy"); z=P[...,2]; c=np.load("agentview_rgb.npy")
m=(z>0.44)&(z<0.60)&(P[...,0]>-0.06)&(P[...,0]<0.05)&(P[...,1]>0.20)&(P[...,1]<0.33)
pts=P[m]
h,e=np.histogram(pts[:,2],bins=16,range=(0.44,0.60))
for a,b in zip(e,h): print(f"{a:.3f} {b}")
sel=pts[(pts[:,2]>0.49)&(pts[:,2]<0.53)]
print("can-top-height pts",len(sel),"center",sel[:,:2].mean(0).round(3), "x",sel[:,0].min().round(3),sel[:,0].max().round(3),"y",sel[:,1].min().round(3),sel[:,1].max().round(3))
EOF

# openrua op 47
python3 -c "
import cv2
im=cv2.imread('agent7.png'); crop=im[200:360,440:640]; cv2.imwrite('crop_basket.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 48
timeout 900 python3 -u ctl.py goto 0.005 0.30 0.74 0 3 && timeout 900 python3 -u ctl.py goto 0.005 0.30 0.55 0 3 && timeout 300 python3 -u ctl.py open && timeout 900 python3 -u ctl.py goto 0.005 0.30 0.76 0 3

# openrua op 49
timeout 900 python3 -u ctl.py goto -0.25 0.0 0.75 0 5 && timeout 120 python3 tools/perception/cam_snap.py agentview agent8.png && timeout 120 python3 tools/perception/cam_snap.py birdview bird8.png && timeout 120 python3 tools/perception/cam_snap.py frontview front8.png

# openrua op 50
python3 -c "
import cv2
im=cv2.imread('agent8.png'); crop=im[200:360,440:640]; cv2.imwrite('crop_basket2.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('bird8.png'); crop=im[240:330,340:430]; cv2.imwrite('crop_bird2.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 51
timeout 120 python3 depthworld.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np, cv2
P=np.load("birdview_world.npy"); z=P[...,2]
# inside basket interior footprint, below rim, above floor
m=(z>0.47)&(z<0.60)&(P[...,0]>-0.07)&(P[...,0]<0.06)&(P[...,1]>0.19)&(P[...,1]<0.34)
mask=m.astype(np.uint8); n,lab,st,cen=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if st[i,4]<10: continue
    pts=P[lab==i]; print(f"object in basket: center xy={pts[:,:2].mean(0).round(3)} top z={pts[:,2].max():.3f} px={st[i,4]}")
# original spots on table
for name,(x,y) in {"alphabet soup spot":(-0.23,-0.155),"tomato sauce spot":(0.0,-0.277)}.items():
    m=(np.abs(P[...,0]-x)<0.05)&(np.abs(P[...,1]-y)<0.05)&np.isfinite(z)
    print(name, "max z now =", z[m].max().round(3), "(table=0.425)")
EOF

# openrua op 52
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy"); z=P[...,2]
m=(z>0.47)&(z<0.60)&(P[...,0]>-0.07)&(P[...,0]<0.06)&(P[...,1]>0.19)&(P[...,1]<0.34)
pts=P[m]
h,e=np.histogram(pts[:,2],bins=13,range=(0.47,0.60))
for a,b in zip(e,h): print(f"{a:.3f} {b}")
for lo,hi in [(0.545,0.565),(0.57,0.59)]:
    s=pts[(pts[:,2]>lo)&(pts[:,2]<hi)]
    if len(s): print(f"z in [{lo},{hi}]: n={len(s)} center={s[:,:2].mean(0).round(3)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
# basket floor height: interior pixels at low z
fl=P[(z>0.43)&(z<0.50)&(P[...,0]>-0.07)&(P[...,0]<0.06)&(P[...,1]>0.19)&(P[...,1]<0.34)]
print("floor-ish pts", len(fl), fl[:,2].mean().round(3) if len(fl) else None)
EOF

# openrua op 53
python3 - <<'EOF'
import numpy as np, cv2
P=np.load("birdview_world.npy"); z=P[...,2]
for lo,hi in [(0.505,0.535),(0.535,0.56)]:
    m=(z>lo)&(z<hi)&(P[...,0]>-0.06)&(P[...,0]<0.05)&(P[...,1]>0.2)&(P[...,1]<0.33)
    n,lab,st,cen=cv2.connectedComponentsWithStats(m.astype(np.uint8))
    for i in range(1,n):
        if st[i,4]<15: continue
        pts=P[lab==i]; print(f"band [{lo},{hi}]: blob center xy={pts[:,:2].mean(0).round(3)} zmean={pts[:,2].mean():.3f} px={st[i,4]} extent x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}]")
EOF

# openrua op 54
python3 -c "
import cv2
im=cv2.imread('front8.png'); crop=im[170:270,360:480]; cv2.imwrite('crop_front2.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"
