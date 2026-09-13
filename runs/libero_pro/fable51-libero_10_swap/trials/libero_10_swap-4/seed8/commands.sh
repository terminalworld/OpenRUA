#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools

# openrua op 2
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 3
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 4
ros2 topic list 2>&1; echo ---; ros2 action list 2>&1; echo ---; ros2 service list 2>&1 | grep -iE "ik|fk|plan|scene" ; echo ---; ros2 node list 2>&1

# openrua op 5
ros2 topic echo /joint_states --once 2>&1 | head -40; for c in agentview birdview frontview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[200:330,200:440]; crop=cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST); cv2.imwrite('bird_crop.png',crop)
"

# openrua op 7
for p in "253 284 leftplate" "386 284 rightplate" "286 261 whitemug" "329 237 yellowmug" "352 271 redmug" "320 200 robotbody"; do set -- $p; echo -n "$3: "; timeout 60 python3 tools/perception/px2world.py birdview $1 $2 2>&1 | tail -1; done

# openrua op 8
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -8

# openrua op 9
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png; python3 -c "
import numpy as np
d=np.load('bird_depth.npy'); print(d.shape,d.dtype,np.nanmin(d),np.nanmax(d))
for (u,v,n) in [(253,284,'lp'),(386,284,'rp'),(286,261,'wm'),(329,237,'ym'),(352,271,'rm'),(320,200,'body'),(320,150,'body2'),(100,100,'table'),(20,20,'floor')]:
    print(n,d[v,u])
"; ros2 topic echo /birdview/color/camera_info --once 2>&1 | grep -A9 "^k:"

# openrua op 10
python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
np.set_printoptions(linewidth=250,precision=3)
print('row 284, cols 200..300 step 4:'); print(d[284,200:300:4])
print('row 261 cols 270..310 step2 (white mug):'); print(d[261,270:310:2])
print('col 286 rows 240..285 step2 (white mug):'); print(d[240:285:2,286])
print('col 329 rows 215..260 (yellow):'); print(d[215:260:2,329])
"

# openrua op 11
timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agent_depth.png; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | grep -A1 Translation | head -3; python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[150:340,30:610]; crop=cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST); cv2.imwrite('agent_crop.png',crop)"

# openrua op 12
python3 -c "
import cv2
im=cv2.imread('agentview.png')
for (u,v) in [(100,295),(535,295),(215,190),(340,160),(415,190)]:
    cv2.circle(im,(u,v),3,(0,255,0),-1)
cv2.imwrite('agent_marks.png',im)"
for p in "100 295 leftplate" "535 295 rightplate" "215 190 whitemug_rim" "340 160 yellow_rim" "415 190 red_rim" "300 400 table"; do set -- $p; echo -n "$3: "; timeout 60 python3 tools/perception/px2world.py agentview $1 $2 2>&1 | tail -1; done

# openrua op 13
timeout 120 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once 2>&1 | grep -A9 position: | tail -2

# openrua op 14
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null; python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
np.set_printoptions(linewidth=250,precision=3)
print('row 284 plate:', d[284,240:270:3])
print('white mug col:', d[240:285:2,286])
print('yellow col:', d[215:260:2,329])
"

# openrua op 15
python3 -c "
import cv2, numpy as np
im=cv2.imread('birdview.png'); d=np.load('bird_depth.npy')
# mask of things between rim height and above table (depth < 2.50 and > 2.3) near mugs
crop=im[200:330,200:440].copy()
m=((d>2.40)&(d<2.50))[200:330,200:440]
crop[m]=(0,255,0)
crop=cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST); cv2.imwrite('bird_crop2.png',crop)
"

# openrua op 16
cat > /workspace/locate.py <<'EOF'
import numpy as np, cv2, sys
d=np.load('bird_depth.npy')
f=579.4112549695428; cx,cy=320.0,240.0
camx,camy,camz=-0.2,0.0,3.0
def px2w(u,v,z):
    # birdview: u -> world y, v -> world x (px2world showed v=200 -> x=-0.329, v=284 -> x=-0.009)
    depth=z
    X=(u-cx)*depth/f; Y=(v-cy)*depth/f
    return (camx+Y, camy+X, camz-depth)
def ring(name,u0,v0,r=16,lo=2.40,hi=2.50):
    win=d[v0-r:v0+r,u0-r:u0+r]
    m=(win>lo)&(win<hi)
    vs,us=np.nonzero(m)
    us=us+u0-r; vs=vs+v0-r
    # circle fit (algebraic)
    A=np.c_[2*us,2*vs,np.ones(len(us))]; b=us**2+vs**2
    sol=np.linalg.lstsq(A,b,rcond=None)[0]
    ucen,vcen=sol[0],sol[1]; rad=np.sqrt(sol[2]+ucen**2+vcen**2)
    zr=win[m].mean()
    w=px2w(ucen,vcen,zr)
    print(f"{name}: n={len(us)} center px=({ucen:.1f},{vcen:.1f}) r_px={rad:.1f} -> world x={w[0]:.4f} y={w[1]:.4f} rim_z={w[2]:.4f} rim_diam={2*rad*zr/f:.3f}")
ring('white',286,261)
ring('yellow',329,237,r=14)
ring('red',352,271)
# plates: centroid of pixels within plate depth band
for name,u0,v0 in [('leftplate',253,284),('rightplate',386,284)]:
    win=d[v0-25:v0+25,u0-25:u0+25]; m=(win>2.545)&(win<2.565)
    vs,us=np.nonzero(m); u=us.mean()+u0-25; v=vs.mean()+v0-25; z=win[m].mean()
    w=px2w(u,v,z); print(f"{name}: px=({u:.1f},{v:.1f}) n={len(us)} world x={w[0]:.4f} y={w[1]:.4f} top_z={w[2]:.4f}")
EOF
python3 /workspace/locate.py

# openrua op 17
cat > /workspace/arm.py <<'EOF'
#!/usr/bin/env python3
"""Small manipulation helper for this Panda.

  arm.py fk                         hand + TCP pose in WORLD
  arm.py goto X Y Z YAWDEG [SECS]   move so the TCP (fingertip centre) is at
                                    world X Y Z, hand pointing down, fingers
                                    opening along world y rotated by YAWDEG
  arm.py grip open|close
World -> base offset comes from TF (world->panda_link0 = -0.51 0 0.42).
"""
import math, sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE = np.array([-0.51, 0.0, 0.42])      # world position of panda_link0
TCP = 0.1034
LIMITS = [(-2.9, 2.9), (-1.76, 1.76), (-2.9, 2.9), (-3.07, -0.07),
          (-2.9, 2.9), (-0.02, 3.75), (-2.9, 2.9)]


def quat_down(yaw_deg):
    # (1,0,0,0) = hand pointing down, fingers along world y; then yaw about world z
    s, c = math.sin(math.radians(yaw_deg) / 2), math.cos(math.radians(yaw_deg) / 2)
    # q = qz * (1,0,0,0)  ->  (c, s, 0, 0)
    return (c, s, 0.0, 0.0)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.update(zip(m.name, m.position)), 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fkc = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory,
                                 "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js.clear()
        while not all(j in self.js for j in ARM):
            self.spin(0.2)
        return [self.js[j] for j in ARM]

    def seed(self):
        s = JointState()
        s.name = list(ARM)
        s.position = self.joints()
        return s

    def fk(self, q=None):
        self.fkc.wait_for_service(5)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = q if q is not None else self.joints()
        fut = self.fkc.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise SystemExit(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        x, y, z, w = q
        # hand z axis in world (3rd column of R)
        zaxis = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
        return pos, q, pos + TCP * zaxis

    def solve_ik(self, tcp_world, yaw_deg):
        hand_world = np.array(tcp_world) + np.array([0, 0, TCP])   # hand points down
        p_base = hand_world - BASE
        qx, qy, qz, qw = quat_down(yaw_deg)
        self.ik.wait_for_service(5)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw
        req.ik_request.robot_state.joint_state = self.seed()
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 5
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            raise SystemExit("IK: no answer")
        if r.error_code.val != 1:
            raise SystemExit(f"IK FAILED code={r.error_code.val} for tcp={tcp_world} yaw={yaw_deg}")
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        q = [sol[j] for j in ARM]
        for v, (lo, hi) in zip(q, LIMITS):
            if not lo <= v <= hi:
                raise SystemExit(f"IK solution outside limits: {q}")
        return q

    def execute(self, q, secs):
        self.traj.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        now = np.array(self.joints())
        err = np.abs(now - np.array(q)).max()
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, tcp_world, yaw_deg, secs=3.0):
        q = self.solve_ik(tcp_world, yaw_deg)
        print("IK q =", np.round(q, 3).tolist())
        code, err = self.execute(q, secs)
        if err > 0.02:
            print("large residual; resending")
            self.execute(q, secs)
        pos, quat, tcp = self.fk()
        print(f"hand world={np.round(pos,4).tolist()} TCP world={np.round(tcp,4).tolist()}")
        return tcp

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        self.js.clear()
        self.joints()
        while "panda_finger_joint1" not in self.js:
            self.spin(0.2)
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"finger1={self.js['panda_finger_joint1']:.4f}")
        return self.js["panda_finger_joint1"]


def main():
    a = sys.argv[1:]
    arm = Arm()
    if a[0] == "fk":
        pos, q, tcp = arm.fk()
        print("joints", np.round(arm.joints(), 4).tolist())
        print("hand world", np.round(pos, 4).tolist(), "quat", np.round(q, 3).tolist())
        print("TCP world", np.round(tcp, 4).tolist())
    elif a[0] == "goto":
        x, y, z, yaw = map(float, a[1:5])
        secs = float(a[5]) if len(a) > 5 else 3.0
        arm.goto((x, y, z), yaw, secs)
    elif a[0] == "ik":
        x, y, z, yaw = map(float, a[1:5])
        print(np.round(arm.solve_ik((x, y, z), yaw), 3).tolist())
    elif a[0] == "grip":
        arm.gripper(0.04 if a[1] == "open" else 0.0)
    elif a[0] == "fingers":
        arm.joints(); print(arm.js.get("panda_finger_joint1"))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
EOF
timeout 120 python3 arm.py fk

# openrua op 18
python3 -c "
import numpy as np
def dh(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854]
P=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
T=np.eye(4)
for (a,d,al),th in zip(P,q): T=T@dh(a,d,al,th)
T=T@dh(0,0.107,0,0)  # flange
T=T@dh(0,0,0,-np.pi/4) # hand rotated -45deg
print('hand in base:',T[:3,3].round(4)); print(T[:3,:3].round(3))
"

# openrua op 19
cat > /workspace/iktest.py <<'EOF'
import sys; sys.argv=['x','fk']
import numpy as np, rclpy
exec(open('arm.py').read().split("def main")[0])
arm=Arm()
from moveit_msgs.srv import GetPositionIK
for label,p in [('world',(-0.053,0,0.7776)),('base',(0.457,0,0.3576))]:
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.pose_stamped.header.frame_id=""
    pp=req.ik_request.pose_stamped.pose
    pp.position.x,pp.position.y,pp.position.z=p
    pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=1.0,0.0,-0.028,0.0
    req.ik_request.robot_state.joint_state=arm.seed(); req.ik_request.timeout.sec=5
    fut=arm.ik.call_async(req); rclpy.spin_until_future_complete(arm.node,fut,timeout_sec=60)
    r=fut.result(); print(label, r.error_code.val, r.error_code.val==1 and np.round([dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))[j] for j in ARM],3).tolist())
EOF
timeout 200 python3 iktest.py

# openrua op 20
sed -i 's/pp.position.x,pp.position.y,pp.position.z=p/pp.position.x,pp.position.y,pp.position.z=map(float,p)/' iktest.py && timeout 200 python3 iktest.py

# openrua op 21
sed -i 's/timeout_sec=60)/timeout_sec=180)/' iktest.py && time timeout 400 python3 iktest.py

# openrua op 22
time timeout 120 ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{ik_request: {group_name: panda_arm, robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854]}}, pose_stamped: {header: {frame_id: ''}, pose: {position: {x: -0.053, y: 0.0, z: 0.7776}, orientation: {x: 1.0, y: 0.0, z: -0.028, w: 0.0}}}, timeout: {sec: 5}}}" 2>&1 | grep -E "position|error_code|val" | head -20

# openrua op 23
cat > /workspace/iktest.py <<'EOF'
import sys; sys.argv=['x','fk']
import numpy as np, rclpy, time
exec(open('arm.py').read().split("def main")[0])
arm=Arm()
from moveit_msgs.srv import GetPositionIK
for label,p,ac in [('world_noac',(-0.053,0,0.7776),False),('world_ac',(-0.053,0,0.7776),True)]:
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.pose_stamped.header.frame_id=""
    pp=req.ik_request.pose_stamped.pose
    pp.position.x,pp.position.y,pp.position.z=map(float,p)
    pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=1.0,0.0,-0.028,0.0
    req.ik_request.robot_state.joint_state=arm.seed(); req.ik_request.timeout.sec=5
    req.ik_request.avoid_collisions=ac
    t=time.time(); fut=arm.ik.call_async(req); rclpy.spin_until_future_complete(arm.node,fut,timeout_sec=40)
    r=fut.result(); print(label, time.time()-t, r and r.error_code.val)
    if r and r.error_code.val==1:
        q=[dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))[j] for j in ARM]
        print(np.round(q,3).tolist()); pos,quat,tcp=arm.fk(q); print('fk of sol', pos.round(4).tolist(), np.round(quat,3).tolist())
EOF
timeout 200 python3 iktest.py

# openrua op 24
python3 - <<'EOF'
s=open('arm.py').read()
s=s.replace('BASE = np.array([-0.51, 0.0, 0.42])      # world position of panda_link0',
            'BASE = np.array([0.0, 0.0, 0.0])  # IK/FK on this machine are already in WORLD (virtual_joint)')
s=s.replace('req.ik_request.group_name = "panda_arm"\n', 'req.ik_request.group_name = "panda_arm"\n        req.ik_request.ik_link_name = "panda_hand"   # group tip is link8 (45 deg off)\n')
s=s.replace('req.ik_request.avoid_collisions = True','req.ik_request.avoid_collisions = False')
open('arm.py','w').write(s)
EOF
grep -n "ik_link_name\|BASE =" arm.py; timeout 100 python3 arm.py ik -0.053 0 0.6742 0

# openrua op 25
for p in "-0.1104 -0.1865 0.65 0" "-0.1104 -0.1865 0.515 0" "-0.0101 -0.3374 0.70 0" "-0.0101 -0.3374 0.542 0" "-0.2108 0.0796 0.65 0" "-0.2108 0.0796 0.515 0" "-0.1658 0.0346 0.515 90" "-0.0098 0.329 0.542 0" "0.035 0.2841 0.542 90"; do echo -n "$p => "; timeout 100 python3 arm.py ik $p 2>&1 | tail -1; done

# openrua op 26
timeout 600 python3 arm.py goto -0.1104 -0.1865 0.65 0 3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 27
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null; python3 -c "
import numpy as np; d=np.load('eih_depth.npy'); np.set_printoptions(precision=3,linewidth=200)
print('row 305:', d[305,180:350:6]); print('col 260:', d[230:380:6,260])"; for p in "330 305 wall_-y" "195 305 wall_+y" "260 240 wall_-x" "260 365 wall_+x" "425 180 plate"; do set -- $p; echo -n "$3: "; timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand $1 $2 2>&1 | tail -1; done

# openrua op 28
for p in "324 305 rim_a" "186 305 rim_b" "260 242 rim_c" "260 374 rim_d"; do set -- $p; echo -n "$3: "; timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand $1 $2 2>&1 | tail -1; done

# openrua op 29
timeout 600 python3 arm.py goto -0.106 -0.188 0.58 0 2 && timeout 600 python3 arm.py goto -0.106 -0.188 0.517 0 2 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 30
python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[80:320,100:400]; crop=cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC); cv2.imwrite('agent_crop.png',crop)"

# openrua op 31
timeout 300 python3 arm.py grip close

# openrua op 32
timeout 600 python3 arm.py goto -0.106 -0.188 0.70 0 2.5 && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 arm.py fingers

# openrua op 33
timeout 600 python3 arm.py goto -0.0101 -0.338 0.70 0 3 && timeout 600 python3 arm.py goto -0.0101 -0.338 0.555 0 2.5 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 34
timeout 300 python3 arm.py grip open && timeout 600 python3 arm.py goto -0.0101 -0.338 0.70 0 2.5 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 35
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null && python3 locate.py 2>&1

# openrua op 36
python3 -c "
import numpy as np; d=np.load('bird_depth.npy'); np.set_printoptions(precision=3,linewidth=200)
print('row 237:', d[237,312:346]); print('col 329:', d[220:255,329])
# also the white mug now
print('white region min depth:', d[250:300,240:290].min())"

# openrua op 37
timeout 600 python3 arm.py goto -0.2127 0.086 0.65 0 3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png

# openrua op 38
timeout 100 python3 arm.py fk; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 39
timeout 600 python3 tools/action/fjt_send.py 0,-0.161,0,-2.4446,0,2.2268,0.7854 3 && timeout 100 python3 arm.py fk

# openrua op 40
timeout 600 python3 tools/action/fjt_send.py 0,-0.161,0,-2.4446,0,2.2268,0.7854 3 && ros2 topic echo /joint_states --once 2>&1 | grep -A8 "^position" 

# openrua op 41
timeout 600 python3 arm.py goto -0.2127 0.086 0.65 0 4 && timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 42
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && python3 -c "
import numpy as np, cv2; d=np.load('eih_depth.npy')
# rim pixels: depth well above table & interior, i.e. the ring of closest points near mug
m=(d>0.15)&(d<0.24)
vs,us=np.nonzero(m); print('n',len(us), 'u range',us.min(),us.max(),'v range',vs.min(),vs.max())
im=cv2.imread('robot0_eye_in_hand.png'); im[m]=(0,255,0); cv2.imwrite('eih_marks.png',im)
np.set_printoptions(precision=3,linewidth=220)
"

# openrua op 43
python3 -c "
import numpy as np; d=np.load('eih_depth.npy')
sub=d[200:400,250:520]; m=(sub>0.1)&(sub<0.20)
vs,us=np.nonzero(m); us=us+250; vs=vs+200
print('n',len(us),'u',us.min(),us.max(),'v',vs.min(),vs.max(), 'depth min', sub[m].min())
print('row300:', np.round(d[300,300:470:5],3))
"
for p in "319 300 wall_py" "444 300 wall_ny" "380 237 wall_a" "380 368 wall_b"; do set -- $p; echo -n "$3: "; timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand $1 $2 2>&1 | tail -1; done

# openrua op 44
timeout 600 python3 arm.py goto -0.209 0.084 0.58 0 2.5 && timeout 600 python3 arm.py goto -0.209 0.084 0.51 0 2.5 && timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null && python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[60:300,180:480]; crop=cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC); cv2.imwrite('agent_crop.png',crop)"

# openrua op 45
timeout 300 python3 arm.py grip close && timeout 600 python3 arm.py goto -0.209 0.084 0.70 0 2.5 && python3 arm.py fingers && timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 46
timeout 600 python3 arm.py goto -0.0098 0.328 0.70 0 3 && timeout 600 python3 arm.py goto -0.0098 0.328 0.55 0 2.5 && timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 47
timeout 300 python3 arm.py grip open && timeout 600 python3 arm.py goto -0.0098 0.328 0.72 0 2.5 && timeout 600 python3 tools/action/fjt_send.py 0,-0.161,0,-2.4446,0,2.2268,0.7854 4; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null

# openrua op 48
python3 - <<'EOF'
import numpy as np
d=np.load('bird_depth.npy'); f=579.4112549695428
def px2w(u,v,z): return (-0.2+(v-240)*z/f, (u-240+240-320)*z/f, 3.0-z)
def ring(name,u0,v0,r=18):
    win=d[v0-r:v0+r,u0-r:u0+r]; m=(win>2.40)&(win<2.50)
    vs,us=np.nonzero(m); us=us+u0-r; vs=vs+v0-r
    A=np.c_[2*us,2*vs,np.ones(len(us))]; b=us**2+vs**2
    s=np.linalg.lstsq(A,b,rcond=None)[0]; uc,vc=s[0],s[1]; rad=np.sqrt(s[2]+uc**2+vc**2)
    zr=win[m].min(); w=px2w(uc,vc,zr)
    print(f"{name}: n={len(us)} px=({uc:.1f},{vc:.1f}) world x={w[0]:.4f} y={w[1]:.4f} rim_z={w[2]:.3f} r={rad*zr/f:.3f}")
    return w
def plate(name,u0,v0):
    win=d[v0-30:v0+30,u0-30:u0+30]; m=(win>2.545)&(win<2.565)
    vs,us=np.nonzero(m); u=us.mean()+u0-30; v=vs.mean()+v0-30; z=win[m].mean()
    # plate radius from extent
    r=max(us.max()-us.min(), vs.max()-vs.min())/2*z/f
    w=px2w(u,v,z); print(f"{name}: n={len(us)} world x={w[0]:.4f} y={w[1]:.4f} top_z={w[2]:.3f} visible-extent r>={r:.3f}")
    return w
wm=ring('white mug',253,284); ym=ring('yellow mug',386,284)
lp=plate('left plate',253,284); rp=plate('right plate',386,284)
import math
print('white mug centre offset from left plate centre: %.3f m'%math.hypot(wm[0]-lp[0],wm[1]-lp[1]))
print('yellow mug centre offset from right plate centre: %.3f m'%math.hypot(ym[0]-rp[0],ym[1]-rp[1]))
EOF
