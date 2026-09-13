#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ------; cat /workspace/tools/action/fjt_send.py; echo ------; cat /workspace/tools/action/ik_move.py; echo ------; cat /workspace/tools/action/gripper_cmd.py; echo -----; cat /workspace/tools/perception/cam_snap.py; echo -----; cat /workspace/tools/perception/px2world.py

# openrua op 2
ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -v -E 'parameter|describe|get_type' ; echo ----; ros2 node list

# openrua op 3
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c & done; wait; ros2 topic echo /joint_states --once

# openrua op 4
for p in "373 280" "373 240" "252 288" "325 300" "385 298"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 5
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -20; echo ----; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo; for p in "200 380" "373 300" "300 240"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 6
timeout 10 ros2 topic echo /pipeline_state --once 2>&1 | head -30; echo ----; ros2 node info /robot_bridge 2>&1 | head -60

# openrua op 7
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a camera's full depth frame as world-frame XYZ (HxWx3 .npy) + color PNG.

Usage: python3 cloud.py <camera>   ->  <camera>_xyz.npy, <camera>.png
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
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
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    xyz = pc @ R.T + p0
    np.save(f"{cam}_xyz.npy", xyz)
    cv2.imwrite(f"{cam}.png", color)
    print(f"{cam}_xyz.npy {xyz.shape}; cam at {p0}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
python3 cloud.py birdview && python3 cloud.py agentview && python3 - <<'EOF'
import numpy as np
xyz=np.load('birdview_xyz.npy')
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
# objects above table (0.90) in the table region
def region(mask,name):
    if mask.sum()==0: print(name,'none'); return
    pts=xyz[mask]
    print(name, 'n=%d'%len(pts), 'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()), 'centroid',pts.mean(0).round(3))
table=(X>-0.45)&(X<0.5)&(np.abs(Y)<0.6)
# moka pot
region(table&(Y<-0.1)&(Z>0.905),'moka')
region(table&(Y<-0.1)&(Z>1.0),'moka top')
# knob
region(table&(X<-0.13)&(Y>0.1)&(Z>0.905),'knob')
region(table&(X<-0.13)&(Y>0.1)&(Z>0.945),'knob top')
# stove
region(table&(X>-0.13)&(X<0.03)&(Y>0.1)&(Z>0.905)&(Z<0.94),'stove')
# pan
region(table&(Y>-0.1)&(Y<0.35)&(X>-0.13)&(Z>0.94),'pan')
region(table&(Y>0.12)&(X>-0.13)&(Z>0.935),'pan handle over stove')
EOF

# openrua op 9
python3 - <<'EOF'
import numpy as np
xyz=np.load('birdview_xyz.npy')
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
def region(mask,name):
    pts=xyz[mask]
    if len(pts)==0: print(name,'none'); return
    print(name, 'n=%d'%len(pts), 'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()), 'centroid',pts.mean(0).round(3))
table=(X>-0.45)&(X<0.5)&(np.abs(Y)<0.6)
region(table&(Y<-0.15)&(Z>0.905)&(Z<1.1),'moka')
region(table&(Y<-0.15)&(Z>1.03)&(Z<1.1),'moka top')
for zl in [0.905,0.92,0.94,0.95]:
    region(table&(X<-0.13)&(X>-0.3)&(Y>0.1)&(Y<0.3)&(Z>zl)&(Z<1.0),'knob z>%.3f'%zl)
# knob slices in z
m=table&(X<-0.13)&(X>-0.3)&(Y>0.1)&(Y<0.3)&(Z>0.905)&(Z<1.0)
pts=xyz[m]
for lo in np.arange(0.905,0.97,0.01):
    s=pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.01)]
    if len(s): print('  z %.3f-%.3f n=%d x[%.3f,%.3f] y[%.3f,%.3f]'%(lo,lo+0.01,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
EOF

# openrua op 10
python3 - <<'EOF'
import numpy as np
xyz=np.load('birdview_xyz.npy')
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(X>-0.2)&(X<0.1)&(Y<-0.12)&(Y>-0.35)&(Z>0.903)&(Z<1.1)
pts=xyz[m]
for lo in np.arange(0.90,1.06,0.01):
    s=pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.01)]
    if len(s): print('  z %.3f-%.3f n=%d x[%.3f,%.3f] y[%.3f,%.3f] c=(%.3f,%.3f)'%(lo,lo+0.01,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max(),s[:,0].mean(),s[:,1].mean()))
# top-down occupancy: print a coarse map (1cm cells) of max z
import collections
grid=collections.defaultdict(float)
for x,y,z in pts:
    grid[(round(x*100),round(y*100))]=max(grid[(round(x*100),round(y*100))],z)
xs=sorted(set(k[0] for k in grid)); ys=sorted(set(k[1] for k in grid))
print('     '+''.join('%4d'%y for y in ys))
for x in xs:
    print('%4d '%x+''.join(('%4d'%round((grid[(x,y)]-0.9)*100) if (x,y) in grid else '   .') for y in ys))
EOF

# openrua op 11
python3 - <<'EOF'
import numpy as np
for cam in ['agentview']:
    xyz=np.load(f'{cam}_xyz.npy')
    X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
    m=(X>-0.2)&(X<0.1)&(Y<-0.12)&(Y>-0.36)&(Z>0.903)&(Z<1.1)
    pts=xyz[m]
    print(cam)
    for lo in np.arange(0.90,1.07,0.005):
        s=pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.005)]
        if len(s): print('  z %.3f n=%d x[%.3f,%.3f] y[%.3f,%.3f]  ymid=%.3f width=%.3f'%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max(),(s[:,1].min()+s[:,1].max())/2,s[:,1].max()-s[:,1].min()))
EOF

# openrua op 12
python3 tools/action/gripper_cmd.py 0.04 && ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 13
python3 tools/action/ik_move.py 0.447 0.190 0.108 1 0 0 0 4 --at tcp && ros2 topic echo /joint_states --once | grep -A8 position: && timeout 15 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A1 "At time" | head -3

# openrua op 14
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""World-frame arm helper for this Panda (base at world (-0.66, 0, 0.912)).

  arm.py fk                                  print TCP pose in world
  arm.py goto x y z qx qy qz qw [secs] [--tcp]   IK -> trajectory -> FK check
  arm.py joints j1,...,j7 secs
  arm.py rot7 delta secs                     rotate joint7 by delta (rad)
  arm.py servo dx dy dz [secs]               cartesian twist burst (world m)
"""
import sys
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
JOINTS = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world position of panda_link0
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fkc = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.twist = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    def _js(self, m):
        self.js = m

    def joint_state(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return self.js

    def arm_positions(self):
        js = self.joint_state()
        d = dict(zip(js.name, js.position))
        return [d[j] for j in JOINTS], d

    def seed(self):
        q, _ = self.arm_positions()
        s = JointState(); s.name = list(JOINTS); s.position = list(q)
        return s

    def fk(self):
        self.fkc.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.seed()
        fut = self.fkc.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        R = quat_R(*q)
        tcp = pos + TCP * R[:, 2]
        return pos, tcp, q

    def solve_ik(self, world_xyz, quat, at_tcp=True):
        self.ik.wait_for_service(10)
        R = quat_R(*quat)
        p = np.array(world_xyz, float)
        if at_tcp:
            p = p - TCP * R[:, 2]
        p = p - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.pose_stamped.pose.position.x = float(p[0])
        req.ik_request.pose_stamped.pose.position.y = float(p[1])
        req.ik_request.pose_stamped.pose.position.z = float(p[2])
        o = req.ik_request.pose_stamped.pose.orientation
        o.x, o.y, o.z, o.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self.seed()
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"IK failed: {None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move_joints(self, q, secs):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        cur, _ = self.arm_positions()
        err = np.abs(np.array(cur) - np.array(q)).max()
        print(f"trajectory error_code={code} max_joint_err={err:.4f}")
        return code

    def goto(self, xyz, quat, secs=4.0, at_tcp=True):
        q = self.solve_ik(xyz, quat, at_tcp)
        print("IK:", np.round(q, 4).tolist())
        self.move_joints(q, secs)
        self.report()

    def servo(self, dxyz, secs=1.0, rate=20):
        msg = TwistStamped(); msg.header.frame_id = "panda_link0"
        v = np.array(dxyz, float) / secs
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(int(secs * rate)):
            self.twist.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=1.0 / rate)
        self.report()

    def report(self):
        pos, tcp, q = self.fk()
        _, d = self.arm_positions()
        print(f"hand world={np.round(pos,4).tolist()} tcp world={np.round(tcp,4).tolist()} quat={np.round(q,4).tolist()}")
        print(f"fingers={d['panda_finger_joint1']:.4f},{d['panda_finger_joint2']:.4f}")


def main():
    a = sys.argv[1:]
    arm = Arm()
    cmd = a[0]
    if cmd == "fk":
        arm.report()
    elif cmd == "goto":
        nums = [float(x) for x in a[1:] if not x.startswith("--")]
        secs = nums[7] if len(nums) > 7 else 4.0
        arm.goto(nums[:3], nums[3:7], secs, at_tcp="--hand" not in a)
    elif cmd == "joints":
        q = [float(x) for x in a[1].split(",")]
        arm.move_joints(q, float(a[2])); arm.report()
    elif cmd == "rot7":
        q, _ = arm.arm_positions()
        q[6] += float(a[1])
        arm.move_joints(q, float(a[2])); arm.report()
    elif cmd == "servo":
        d = [float(x) for x in a[1:4]]
        arm.servo(d, float(a[4]) if len(a) > 4 else 1.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 15
python3 arm.py fk

# openrua op 16
python3 - <<'EOF'
import rclpy, numpy as np
from arm import Arm, BASE
from moveit_msgs.srv import GetPositionFK
arm=Arm()
req=GetPositionFK.Request(); req.fk_link_names=["panda_link0","panda_hand"]; req.robot_state.joint_state=arm.seed()
fut=arm.fkc.call_async(req); rclpy.spin_until_future_complete(arm.node,fut,timeout_sec=30)
r=fut.result()
for ps in r.pose_stamped: print(ps.header.frame_id, ps.pose.position)
cur,_=arm.arm_positions(); print('cur',np.round(cur,3))
# hypothesis A: IK pose in base frame -> hand at (0.457,0,0.358)
hand=r.pose_stamped[1].pose
q=(hand.orientation.x,hand.orientation.y,hand.orientation.z,hand.orientation.w)
for name,p in [('as-returned',(hand.position.x,hand.position.y,hand.position.z)),('minus base',(hand.position.x-BASE[0],hand.position.y-BASE[1],hand.position.z-BASE[2]))]:
    try:
        sol=arm.solve_ik(np.array(p)+BASE,q,at_tcp=False)  # solve_ik subtracts BASE
        print(name,'->',np.round(sol,3))
    except SystemExit as e: print(name,'->',e)
EOF

# openrua op 17
python3 - <<'EOF'
import rclpy, numpy as np
from arm import Arm, BASE
from moveit_msgs.srv import GetPositionFK
arm=Arm()
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand"]; req.robot_state.joint_state=arm.seed()
fut=arm.fkc.call_async(req); rclpy.spin_until_future_complete(arm.node,fut,timeout_sec=30)
r=fut.result()
for ps in r.pose_stamped: print(repr(ps.header.frame_id), ps.pose.position)
cur,_=arm.arm_positions(); print('cur',np.round(cur,3))
hand=r.pose_stamped[0].pose
q=(hand.orientation.x,hand.orientation.y,hand.orientation.z,hand.orientation.w)
for name,p in [('as-returned',(hand.position.x,hand.position.y,hand.position.z)),('minus base',(hand.position.x-BASE[0],hand.position.y-BASE[1],hand.position.z-BASE[2]))]:
    try:
        sol=arm.solve_ik(np.array(p)+BASE,q,at_tcp=False)  # solve_ik subtracts BASE
        print(name,'->',np.round(sol,3))
    except SystemExit as e: print(name,'->',e)
EOF

# openrua op 18
timeout 120 python3 arm.py fk; echo exit=$?

# openrua op 19
python3 - <<'EOF'
import rclpy, numpy as np
from arm import Arm, BASE
from moveit_msgs.srv import GetPositionFK
arm=Arm()
arm.fkc.wait_for_service(10)
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand"]; req.robot_state.joint_state=arm.seed()
fut=arm.fkc.call_async(req); rclpy.spin_until_future_complete(arm.node,fut,timeout_sec=30)
r=fut.result()
for ps in r.pose_stamped: print(repr(ps.header.frame_id), ps.pose.position)
cur,_=arm.arm_positions(); print('cur',np.round(cur,3))
hand=r.pose_stamped[0].pose
q=(hand.orientation.x,hand.orientation.y,hand.orientation.z,hand.orientation.w)
for name,p in [('as-returned',(hand.position.x,hand.position.y,hand.position.z)),('minus base',(hand.position.x-BASE[0],hand.position.y-BASE[1],hand.position.z-BASE[2]))]:
    try:
        sol=arm.solve_ik(np.array(p)+BASE,q,at_tcp=False)  # solve_ik subtracts BASE
        print(name,'->',np.round(sol,3))
    except SystemExit as e: print(name,'->',e)
EOF

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE = np.array([-0.66, 0.0, 0.912])  # world position of panda_link0", "new_string": "# verified: /compute_fk and /compute_ik on this machine use the WORLD frame\\n# (hand FK == TF world->panda_hand), so no base offset is needed.\\nBASE = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
timeout 600 python3 arm.py goto -0.213 0.190 1.02 1 0 0 0 4

# openrua op 22
timeout 600 python3 arm.py rot7 0.784 2

# openrua op 23
timeout 600 python3 arm.py rot7 0.369 3

# openrua op 24
python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 25
timeout 600 python3 arm.py servo 0 0 -0.073 1.5

# openrua op 26
timeout 600 python3 arm.py goto -0.213 0.190 0.947 1 0 0 0 3

# openrua op 27
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        o = req.ik_request.pose_stamped.pose.orientation\\n        o.x, o.y, o.z, o.w = map(float, quat)", "new_string": "        # the IK tip is panda_link8, which sits 45 deg (about z) from\\n        # panda_hand: request link8 = hand * Rz(+45deg)\\n        o = req.ik_request.pose_stamped.pose.orientation\\n        o.x, o.y, o.z, o.w = map(float, quat_mul(quat, (0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8))))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 28
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "class Arm:", "new_string": "def quat_mul(a, b):\\n    \\"\\"\\"Hamilton product a*b, quaternions as (x, y, z, w).\\"\\"\\"\\n    ax, ay, az, aw = a\\n    bx, by, bz, bw = b\\n    return (aw * bx + ax * bw + ay * bz - az * by,\\n            aw * by - ax * bz + ay * bw + az * bx,\\n            aw * bz + ax * by - ay * bx + az * bw,\\n            aw * bw - ax * bx - ay * by - az * bz)\\n\\n\\nclass Arm:", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 29
timeout 600 python3 arm.py goto -0.213 0.190 1.0 1 0 0 0 3

# openrua op 30
timeout 600 python3 arm.py goto -0.213 0.190 0.947 1 0 0 0 3 && python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 31
timeout 300 python3 tools/action/gripper_cmd.py 0.0 && timeout 120 python3 arm.py fk

# openrua op 32
timeout 600 python3 arm.py rot7 1.3 3; python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 33
python3 tools/perception/cam_snap.py agentview

# openrua op 34
timeout 600 python3 arm.py rot7 0.5 4

# openrua op 35
timeout 600 python3 arm.py rot7 -1.1 5

# openrua op 36
timeout 600 python3 arm.py rot7 -0.6 5; python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 37
timeout 300 python3 tools/action/gripper_cmd.py 0.04 && timeout 600 python3 arm.py goto -0.213 0.190 1.05 1 0 0 0 3 && python3 tools/perception/cam_snap.py agentview

# openrua op 38
python3 cloud.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np, collections
xyz=np.load('birdview_xyz.npy')
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(X>-0.15)&(X<0.15)&(Y>0.08)&(Y<0.3)&(Z>0.903)&(Z<1.1)
pts=xyz[m]
grid=collections.defaultdict(float)
for x,y,z in pts:
    k=(round(x*100),round(y*100)); grid[k]=max(grid[k],z)
xs=sorted(set(k[0] for k in grid)); ys=sorted(set(k[1] for k in grid))
print('height above 0.90 in cm; rows x, cols y')
print('     '+''.join('%3d'%y for y in ys))
for x in xs:
    print('%4d '%x+''.join(('%3d'%round((grid[(x,y)]-0.9)*100) if (x,y) in grid else '  .') for y in ys))
EOF

# openrua op 39
python3 cloud.py sideview >/dev/null && python3 cloud.py frontview > /dev/null && python3 - <<'EOF'
import numpy as np
for cam in ['sideview','frontview']:
    xyz=np.load(f'{cam}_xyz.npy')
    X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
    m=(X>-0.15)&(X<0.08)&(Y<-0.12)&(Y>-0.36)&(Z>0.903)&(Z<1.07)
    pts=xyz[m]
    print(cam, len(pts))
    for lo in np.arange(0.90,1.07,0.005):
        s=pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.005)]
        if len(s): print('  z %.3f n=%3d x[%.3f,%.3f] xw=%.3f xmid=%.3f | y[%.3f,%.3f] yw=%.3f'%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,0].max()-s[:,0].min(),(s[:,0].max()+s[:,0].min())/2,s[:,1].min(),s[:,1].max(),s[:,1].max()-s[:,1].min()))
EOF

# openrua op 40
timeout 600 python3 arm.py goto -0.034 -0.233 1.08 0.7071 0.7071 0 0 4 && python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 41
timeout 600 python3 arm.py joints -0.1107,0.5569,-0.2803,-1.7278,0.1903,2.258,-1.2574 4

# openrua op 42
timeout 600 python3 arm.py joints -0.1107,0.5569,-0.2803,-1.7278,0.1903,2.258,-1.2574 6

# openrua op 43
python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 44
timeout 600 python3 arm.py goto -0.034 -0.233 0.965 0.7071 0.7071 0 0 4

# openrua op 45
timeout 600 python3 arm.py goto -0.033 -0.233 1.075 0.7071 0.7071 0 0 3 && timeout 600 python3 arm.py goto -0.033 -0.233 1.043 0.7071 0.7071 0 0 3 && python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 46
timeout 300 python3 tools/action/gripper_cmd.py 0.0 && timeout 120 python3 arm.py fk

# openrua op 47
timeout 600 python3 arm.py goto -0.033 -0.233 1.09 0.7071 0.7071 0 0 3 && python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py sideview

# openrua op 48
timeout 600 python3 arm.py goto -0.033 -0.233 1.25 0.7071 0.7071 0 0 4 && timeout 600 python3 arm.py goto -0.065 0.19 1.25 0.7071 0.7071 0 0 5 && python3 tools/perception/cam_snap.py agentview

# openrua op 49
timeout 600 python3 arm.py goto -0.065 0.19 1.12 0.7071 0.7071 0 0 4 && timeout 600 python3 arm.py goto -0.065 0.19 1.075 0.7071 0.7071 0 0 3

# openrua op 50
timeout 300 python3 tools/action/gripper_cmd.py 0.04 && timeout 600 python3 arm.py goto -0.065 0.19 1.20 0.7071 0.7071 0 0 3 && python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py sideview

# openrua op 51
python3 cloud.py birdview >/dev/null && python3 cloud.py sideview >/dev/null && python3 - <<'EOF'
import numpy as np
xyz=np.load('birdview_xyz.npy')
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
# moka lid (z~1.055-1.09 now if base on stove at 0.93)
m=(X>-0.15)&(X<0.05)&(Y>0.1)&(Y<0.3)&(Z>1.03)&(Z<1.1)
p=xyz[m]; print('moka top from birdview: n=%d x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] centroid %s'%(len(p),p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max(),p.mean(0).round(3)))
# knob bar orientation
m=(X<-0.15)&(X>-0.3)&(Y>0.1)&(Y<0.3)&(Z>0.945)&(Z<1.0)
p=xyz[m]; print('knob bar: n=%d x[%.3f,%.3f] y[%.3f,%.3f]'%(len(p),p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()))
c=p[:,:2].mean(0); u,s,vt=np.linalg.svd(p[:,:2]-c); print('bar axis angle from x: %.1f deg'%np.degrees(np.arctan2(vt[0,1],vt[0,0])))
xyz=np.load('sideview_xyz.npy')
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(X>-0.15)&(X<0.05)&(Y>0.1)&(Y<0.3)&(Z>0.925)&(Z<1.1)
p=xyz[m]
for lo in np.arange(0.925,1.09,0.01):
    s=p[(p[:,2]>=lo)&(p[:,2]<lo+0.01)]
    if len(s): print('  z %.3f n=%3d x[%.3f,%.3f] xmid=%.3f'%(lo,len(s),s[:,0].min(),s[:,0].max(),(s[:,0].max()+s[:,0].min())/2))
EOF

# openrua op 52
timeout 600 python3 arm.py joints 0,-0.161,0,-2.4446,0,2.2268,0.7854 5 && python3 cloud.py birdview >/dev/null && python3 tools/perception/cam_snap.py agentview && python3 - <<'EOF'
import numpy as np
xyz=np.load('birdview_xyz.npy')
X,Y,Z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(X>-0.15)&(X<0.05)&(Y>0.08)&(Y<0.3)&(Z>1.03)&(Z<1.1)
p=xyz[m]; print('moka top: n=%d x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] centroid %s'%(len(p),p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max(),p.mean(0).round(3)))
print('burner center ~(-0.055, 0.190), radius ~0.06')
EOF
