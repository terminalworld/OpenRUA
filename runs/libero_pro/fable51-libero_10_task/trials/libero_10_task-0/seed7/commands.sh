#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -iv parameter; echo ---; ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png')
cv2.imwrite('crop_cans.png', cv2.resize(im[250:420,220:540],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop_cc.png', cv2.resize(im[50:170,470:580],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agentview.png')
cv2.imwrite('crop_agent.png', cv2.resize(im[160:420,60:400],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 120 python3 tools/perception/px2world.py agentview 340 255; timeout 60 python3 tools/perception/px2world.py agentview 145 380; timeout 60 python3 tools/perception/px2world.py agentview 540 260

# openrua op 7
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -20; echo ---; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80

# openrua op 8
timeout 60 python3 tools/perception/px2world.py agentview 300 420; timeout 60 python3 tools/perception/px2world.py birdview 320 380; timeout 30 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | grep -A2 "Translation" | head -3; timeout 30 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | grep -A2 "Translation" | head -3

# openrua op 9
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects above the table from the birdview depth camera.

Usage: python3 scene.py [zmin=0.445]
Prints clusters: centroid world x y, top z, pixel bbox, pixel count.
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, msg_type, timeout=60.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    zmin = float(sys.argv[1]) if len(sys.argv) > 1 else 0.445
    cam = "birdview"
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    import time
    end = time.time() + 20
    frame = f"{cam}_optical_frame"
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])

    H, W = depth_msg.height, depth_msg.width
    depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(H, W)
    color = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(H, W, -1)[:, :, :3]
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T
    Z = P[..., 2]
    mask = (Z > zmin) & (Z < 0.9) & np.isfinite(depth)
    # exclude the robot: it is a big blob; we'll report all and let the user judge
    n, lab = cv2.connectedComponents(mask.astype(np.uint8))
    out = []
    for i in range(1, n):
        m = lab == i
        cnt = m.sum()
        if cnt < 8:
            continue
        pts = P[m]
        vv, uu = np.where(m)
        top = pts[:, 2].max()
        # centroid of the top-most part (within 2cm of top) is a better XY for grasping
        topm = pts[:, 2] > top - 0.03
        c = pts[topm].mean(0)
        col = color[m].mean(0)
        out.append((cnt, c, top, uu.min(), uu.max(), vv.min(), vv.max(), col))
    out.sort(key=lambda o: -o[0])
    for cnt, c, top, u0, u1, v0, v1, col in out:
        print(f"px={cnt:5d} xy=({c[0]:+.3f},{c[1]:+.3f}) top_z={top:.3f} "
              f"bbox u[{u0}-{u1}] v[{v0}-{v1}] rgb={col.astype(int)}")
    np.save("birdview_P.npy", P)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 10
timeout 200 python3 scene.py

# openrua op 11
sed -i 's/    cam = "birdview"/    cam = sys.argv[2] if len(sys.argv) > 2 else "birdview"/; s/np.save("birdview_P.npy", P)/np.save(f"{cam}_P.npy", P)/' scene.py && timeout 200 python3 scene.py 0.44 agentview

# openrua op 12
python3 -c "
import numpy as np
P=np.load('birdview_P.npy'); Z=P[...,2]
def info(name, u0,u1,v0,v1, zmin):
    sub=P[v0:v1+1,u0:u1+1]; m=sub[...,2]>zmin
    pts=sub[m]
    print(name, 'n',len(pts),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
    xy=pts[:,:2]; c=xy.mean(0); w,v=np.linalg.eigh(np.cov((xy-c).T)); 
    print('  center',c,'axes',v[:,1],'yaw_deg',np.degrees(np.arctan2(v[1,1],v[0,1])), 'sd',np.sqrt(w))
info('basket',360,410,264,311,0.55)
info('creamcheese',265,280,298,322,0.447)
info('OJ',248,268,280,299,0.5)
# pixel scale
print('pixel scale m/px:', P[300,321,0]-P[300,320,0], P[301,320,1]-P[300,320,1])
"


# openrua op 13
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small controller CLI for the Panda (world-frame targets).

  python3 ctl.py fk                         # hand + tcp pose in world
  python3 ctl.py move X Y Z [yaw_deg] [T]   # TCP to world pose, top-down grasp orientation
  python3 ctl.py joints p1,...,p7 [T]       # raw joint trajectory
  python3 ctl.py grip open|close
  python3 ctl.py js                         # joint state
World->base offset: base = world - (-0.51, 0, 0.42)  (from TF world->panda_link0).
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = ARM["joints"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def topdown_quat(yaw_deg):
    """Hand Z down; yaw rotates the finger-opening axis about world Z.
    yaw=0 -> fingers open along world Y."""
    # q = Rz(yaw) * Rx(pi)
    h = math.radians(yaw_deg) / 2
    qz = (0, 0, math.sin(h), math.cos(h))
    qx = (1.0, 0, 0, 0)
    # quaternion multiply qz * qx
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, ARM["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fkc = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        end = time.time() + 30
        while "m" not in self._js and time.time() < end:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_state(self):
        js = self.joints()
        s = JointState()
        for j in JOINTS:
            s.name.append(j)
            s.position.append(js[j])
        return s, js

    def fk(self):
        self.fkc.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state, js = self.arm_state()
        fut = self.fkc.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        R = quat_to_R(*q)
        tcp = pos + TCP * R[:, 2]
        return pos + BASE_IN_WORLD, tcp + BASE_IN_WORLD, q, R, js

    def ik_solve(self, tcp_world, quat):
        self.ik.wait_for_service(10)
        R = quat_to_R(*quat)
        hand_base = np.array(tcp_world) - TCP * R[:, 2] - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state, _ = self.arm_state()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK failed: {None if res is None else res.error_code.val}")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def traj(self, positions, seconds):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        result = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result)
        code = result.result().result.error_code
        js = self.joints()
        err = max(abs(js[j] - p) for j, p in zip(JOINTS, positions))
        print(f"traj done error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move(self, tcp_world, yaw_deg=0.0, seconds=3.0):
        q = topdown_quat(yaw_deg)
        sol = self.ik_solve(tcp_world, q)
        if sol is None:
            return False
        code, err = self.traj(sol, seconds)
        hand, tcp, _, _, _ = self.fk()
        print(f"tcp now world=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f}) "
              f"target=({tcp_world[0]:.3f},{tcp_world[1]:.3f},{tcp_world[2]:.3f})")
        return code == 0

    def grip(self, what):
        self.gr.wait_for_server(10)
        goal = GripperCommand.Goal()
        goal.command.position = float(GRIP["open_m"] if what == "open" else GRIP["closed_m"])
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=180)
        r = rf.result().result
        js = self.joints()
        f1, f2 = js["panda_finger_joint1"], js["panda_finger_joint2"]
        print(f"grip {what}: reached={r.reached_goal} stalled={r.stalled} "
              f"fingers=({f1:.4f},{f2:.4f}) gap={f1 - f2:.4f}")
        return f1, f2


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    c = Ctl()
    cmd = a[0]
    if cmd == "fk":
        hand, tcp, q, R, js = c.fk()
        print("hand world", np.round(hand, 4))
        print("tcp  world", np.round(tcp, 4))
        print("quat xyzw", np.round(q, 4))
        print("R", np.round(R, 3))
        print("fingers", js["panda_finger_joint1"], js["panda_finger_joint2"])
    elif cmd == "js":
        print(c.joints())
    elif cmd == "move":
        x, y, z = map(float, a[1:4])
        yaw = float(a[4]) if len(a) > 4 else 0.0
        T = float(a[5]) if len(a) > 5 else 3.0
        ok = c.move((x, y, z), yaw, T)
        sys.exit(0 if ok else 1)
    elif cmd == "joints":
        pos = [float(v) for v in a[1].split(",")]
        T = float(a[2]) if len(a) > 2 else 3.0
        c.traj(pos, T)
    elif cmd == "grip":
        c.grip(a[1])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 14
timeout 120 python3 ctl.py fk

# openrua op 15
python3 -c "
import numpy as np
def dh(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
def fk(q):
    P=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
    T=np.eye(4)
    for (a,d,al),th in zip(P,q): T=T@dh(a,d,al,th)
    T=T@dh(0,0.107,0,0)  # link8
    T=T@dh(0,0,0,-np.pi/4) # hand
    return T
T=fk([0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483])
print(np.round(T,3))
print('world hand', T[:3,3]+[-0.51,0,0.42])
T=fk([0,-0.785,0,-2.356,0,1.571,0.785]); print('ready', np.round(T[:3,3],3))
"


# openrua op 16
sed -i 's/^BASE_IN_WORLD = np.array(\[-0.51, 0.0, 0.42\])/BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK check: planner model frame == world here/' ctl.py && timeout 120 python3 -c "
import ctl, numpy as np
c=ctl.Ctl()
hand,tcp,q,R,js=c.fk(); print('tcp world',tcp, 'q',q)
sol=c.ik_solve(tcp, q); print('ik sol', np.round(sol,3))
print('cur    ', np.round([js[j] for j in ctl.JOINTS],3))
sol=c.ik_solve(tcp, ctl.topdown_quat(0)); print('ik sol topdown', np.round(sol,3) if sol else sol)
"

# openrua op 17
python3 -c "
import numpy as np
P=np.load('birdview_P.npy')
sub=P[255:278,274:298]; m=(sub[...,2]>0.47)&(sub[...,2]<0.6); pts=sub[m]
print('bluecan n',len(pts),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()), 'center', pts[:,:2].mean(0))
print('px scale x', P[301,320,0]-P[300,320,0], 'y', P[300,321,1]-P[300,320,1])
"

# openrua op 18
timeout 300 python3 ctl.py grip open && timeout 300 python3 ctl.py move -0.12 0.03 0.62 0 3

# openrua op 19
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 20
timeout 20 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 Translation | head -4; timeout 20 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 Translation | head -4; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A3 Translation | head -4

# openrua op 21
timeout 200 python3 scene.py 0.47 robot0_eye_in_hand && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_P.npy')
# tomato sauce can: around pixel (318,312)
sub=P[250:380,240:400]; m=(sub[...,2]>0.49)&(sub[...,2]<0.53); pts=sub[m]
print('top face n',len(pts),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()), 'center', pts[:,:2].mean(0))
"

# openrua op 22
timeout 300 python3 ctl.py move -0.121 0.033 0.47 0 3 && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 23
timeout 300 python3 ctl.py grip close

# openrua op 24
timeout 300 python3 ctl.py move -0.121 0.033 0.72 0 3 && timeout 300 python3 ctl.py js | tr ',' '\n' | grep finger && timeout 300 python3 ctl.py move 0.0 0.27 0.74 0 4 && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 25
sleep 3; timeout 300 python3 ctl.py fk 2>&1 | grep -v XMLPARSER

# openrua op 26
timeout 300 python3 ctl.py move -0.121 0.033 0.72 0 3 2>&1 | grep -v XMLPARSER && timeout 300 python3 ctl.py js 2>&1 | tr ',' '\n' | grep finger

# openrua op 27
timeout 300 python3 ctl.py move -0.121 0.033 0.72 0 3 2>&1 | grep -v XMLPARSER

# openrua op 28
timeout 300 python3 ctl.py move 0.0 0.27 0.74 0 4 2>&1 | grep -v XMLPARSER; timeout 300 python3 ctl.py js 2>&1 | tr ',' '\n' | grep finger; timeout 120 python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v XMLPARSER

# openrua op 29
timeout 300 python3 ctl.py move 0.0 0.27 0.69 0 2 2>&1 | grep -v XMLPARSER; timeout 300 python3 ctl.py grip open 2>&1 | grep -v XMLPARSER; timeout 300 python3 ctl.py move 0.0 0.27 0.78 0 2 2>&1 | grep -v XMLPARSER; timeout 120 python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v XMLPARSER

# openrua op 30
timeout 300 python3 ctl.py move 0.108 -0.209 0.60 0 4 2>&1 | grep -v XMLPARSER; timeout 200 python3 scene.py 0.44 robot0_eye_in_hand 2>&1 | grep -v XMLPARSER; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>&1 | grep -v XMLPARSER

# openrua op 31
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_P.npy')
sub=P[230:360,270:370]; m=(sub[...,2]>0.443)&(sub[...,2]<0.47); pts=sub[m]
print('cc n',len(pts),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()), 'center', pts[:,:2].mean(0))
xy=pts[:,:2]; c=xy.mean(0); w,v=np.linalg.eigh(np.cov((xy-c).T)); print('long axis',v[:,1],'yaw',np.degrees(np.arctan2(v[1,1],v[0,1])),'sd',np.sqrt(w))
# table height under it
sub=P[100:200,100:250]; print('table z', np.median(sub[...,2]))
"

# openrua op 32
timeout 300 python3 ctl.py move 0.114 -0.206 0.445 0 3 2>&1 | grep -v XMLPARSER; timeout 120 python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v XMLPARSER

# openrua op 33
timeout 300 python3 ctl.py grip close 2>&1 | grep -v XMLPARSER

# openrua op 34
timeout 300 python3 ctl.py move 0.114 -0.206 0.72 0 3 2>&1 | grep -v XMLPARSER; timeout 300 python3 ctl.py js 2>&1 | tr ',' '\n' | grep finger

# openrua op 35
timeout 300 python3 ctl.py move -0.02 0.28 0.74 0 4 2>&1 | grep -v XMLPARSER; timeout 300 python3 ctl.py js 2>&1 | tr ',' '\n' | grep finger; timeout 120 python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v XMLPARSER

# openrua op 36
timeout 300 python3 ctl.py move -0.02 0.28 0.70 0 2 2>&1 | grep -v XMLPARSER; timeout 300 python3 ctl.py grip open 2>&1 | grep -v XMLPARSER; timeout 300 python3 ctl.py move -0.02 0.28 0.80 0 2 2>&1 | grep -v XMLPARSER; timeout 120 python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v XMLPARSER

# openrua op 37
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>&1 | grep -v XMLPARSER; timeout 200 python3 scene.py 0.44 robot0_eye_in_hand 2>&1 | grep -v XMLPARSER

# openrua op 38
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_P.npy')
for name,(u,v) in {'can top':(330,262),'cream cheese':(318,290),'basket floor':(275,220),'table':(150,150)}.items():
    p=P[v-2:v+3,u-2:u+3].reshape(-1,3).mean(0); print(name, np.round(p,3))
"

# openrua op 39
timeout 300 python3 ctl.py move -0.25 -0.05 0.85 0 3 2>&1 | grep -v XMLPARSER; timeout 120 python3 tools/perception/cam_snap.py birdview 2>&1 | grep -v XMLPARSER; timeout 120 python3 tools/perception/cam_snap.py frontview 2>&1 | grep -v XMLPARSER; python3 -c "
import cv2
im=cv2.imread('birdview.png'); cv2.imwrite('crop_bird.png', cv2.resize(im[240:340,340:430],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('frontview.png'); cv2.imwrite('crop_front.png', cv2.resize(im[150:300,150:500],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))"
