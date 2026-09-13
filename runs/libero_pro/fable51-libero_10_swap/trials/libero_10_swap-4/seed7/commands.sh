#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools /workspace/.ros

# openrua op 2
cat tools/action/ik_move.py tools/action/fjt_send.py tools/action/gripper_cmd.py

# openrua op 3
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 4
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -iE "ik|fk|plan|scene" ; echo ---; ros2 node list

# openrua op 5
ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -8; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; ros2 topic echo /agentview/color/camera_info --once | grep -A3 "^k:"

# openrua op 7
mkdir -p "$(dirname /workspace/locate.py)"
cat > /workspace/locate.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Convert several pixels of one camera to world coords in one go.

Usage: python3 locate.py <camera> u,v[,label] [u,v[,label] ...]
Also saves <camera>.png and <camera>_depth.npy for offline inspection.
"""
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
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
    cam = sys.argv[1]
    pix = []
    for a in sys.argv[2:]:
        parts = a.split(",")
        pix.append((int(parts[0]), int(parts[1]),
                    parts[2] if len(parts) > 2 else ""))
    rclpy.init()
    node = rclpy.create_node("locate")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    bridge = CvBridge()
    color = grab(node, f"/{cam}/color/image_raw", Image)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    img = bridge.imgmsg_to_cv2(color, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
    cv2.imwrite(f"{cam}.png", img)
    np.save(f"{cam}_depth.npy", depth)

    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print(f"camera {cam} at world {T[:3, 3].round(3)}")
    for u, v, label in pix:
        z = float(depth[v, u])
        if not np.isfinite(z) or z <= 0:
            print(f"{label:>12s} ({u},{v}): no depth")
            continue
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"{label:>12s} ({u},{v}) d={z:.3f} -> "
              f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
python3 locate.py birdview 250,283,lplate 390,283,rplate 285,268,white 322,240,yellow 318,285,red 200,283,table 2>&1 | grep -v INFO; python3 locate.py agentview 98,292,lplate 545,292,rplate 215,200,white_rim 335,175,yellow_rim 385,195,red_rim 320,400,table 2>&1 | grep -v INFO

# openrua op 9
mkdir -p "$(dirname /workspace/heightmap.py)"
cat > /workspace/heightmap.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Project a camera's depth frame to world, then report blobs above the
table between zmin and zmax (mugs/plates), with color samples.

Usage: python3 heightmap.py <camera> [zmin zmax]
"""
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2

sys.path.insert(0, "/workspace")
from locate import grab, quat_to_R  # noqa: E402


def main():
    cam = sys.argv[1]
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.44
    zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 0.70
    rclpy.init()
    node = rclpy.create_node("heightmap")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    bridge = CvBridge()
    color = grab(node, f"/{cam}/color/image_raw", Image)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    img = bridge.imgmsg_to_cv2(color, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    tr = np.array([t.transform.translation.x, t.transform.translation.y,
                   t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    pc = np.stack([(us - cx) * depth / fx, (vs - cy) * depth / fy, depth], -1)
    pw = pc @ R.T + tr
    np.save(f"{cam}_world.npy", pw)
    z = pw[..., 2]
    mask = ((z > zmin) & (z < zmax)).astype(np.uint8)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 15:
            continue
        m = lab == i
        pts = pw[m]
        col = img[m].mean(0)[::-1].astype(int)  # RGB
        x0, y0, bw, bh = stats[i, :4]
        print(f"blob {i}: px({cent[i][0]:.0f},{cent[i][1]:.0f}) "
              f"box({x0},{y0},{bw}x{bh}) area={area} "
              f"world x={pts[:,0].mean():.3f}[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] "
              f"y={pts[:,1].mean():.3f}[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
              f"zmax={pts[:,2].max():.3f} rgb={col}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 10
python3 heightmap.py birdview 0.43 0.75 2>&1 | grep -v INFO

# openrua op 11
python3 heightmap.py birdview 0.46 0.66 2>&1 | grep -v INFO; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "At time"

# openrua op 12
python3 heightmap.py agentview 0.46 0.66 2>&1 | grep -v INFO && python3 -c "
import numpy as np
pw=np.load('agentview_world.npy')
# white mug region: y in [-0.25,-0.05], x in [-0.2,0.0]
for name,(y0,y1) in {'white':(-0.25,-0.05),'red':(0.02,0.2),'yellow':(-0.06,0.09)}.items():
    print(name)
    for z0 in np.arange(0.44,0.60,0.02):
        m=(pw[...,2]>z0)&(pw[...,2]<z0+0.02)&(pw[...,1]>y0)&(pw[...,1]<y1)&(pw[...,0]>-0.35)&(pw[...,0]<0.0)
        if name=='yellow': m&=(pw[...,0]<-0.15)
        if name!='yellow': m&=(pw[...,0]>-0.15)
        if m.sum()<3: continue
        ys=pw[m][:,1]; xs=pw[m][:,0]
        print(f'  z {z0:.2f}-{z0+0.02:.2f}: n={m.sum()} y[{ys.min():.3f},{ys.max():.3f}] w={ys.max()-ys.min():.3f} xmin={xs.min():.3f}')
"

# openrua op 13
python3 -c "
import cv2, numpy as np
img=cv2.imread('agentview.png'); d=np.load('agentview_depth.npy')
crop=img[150:300,150:450]; cv2.imwrite('crop_mugs.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
# depth profile along row 215 (white mug body) and 230
for row in (200,215,240,260):
    prof=d[row,170:270]
    print(row, ' '.join(f'{v:.2f}' for v in prof[::2]))
"

# openrua op 14
python3 tools/action/gripper_cmd.py 0.04 2>&1 | grep -v INFO; ros2 topic echo /joint_states --once | grep -A9 position | tail -3; python3 heightmap.py birdview 0.44 0.66 2>&1 | grep -v INFO

# openrua op 15
python3 heightmap.py agentview 0.46 0.66 2>&1 | grep -v INFO | grep -v "x=-1"; python3 -c "
import numpy as np
pw=np.load('agentview_world.npy'); 
for name,(y0,y1,x0,x1) in {'white':(-0.25,-0.05,-0.15,0.0),'red':(0.0,0.2,-0.15,0.0),'yellow':(-0.08,0.1,-0.35,-0.15)}.items():
    m=(pw[...,2]>0.50)&(pw[...,2]<0.60)&(pw[...,1]>y0)&(pw[...,1]<y1)&(pw[...,0]>x0)&(pw[...,0]<x1)
    p=pw[m]; print(name, 'n',m.sum(),'zmax %.3f'%p[:,2].max(), 'x[%.3f,%.3f] y[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()))
    # rim points: top 1cm
    r=p[p[:,2]>p[:,2].max()-0.01]; print('   rim x[%.3f,%.3f] y[%.3f,%.3f]'%(r[:,0].min(),r[:,0].max(),r[:,1].min(),r[:,1].max()))
"

# openrua op 16
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper for this Panda: world-frame TCP moves via IK + FJT.

Usage:
  python3 arm.py pose                       # current TCP pose (world)
  python3 arm.py goto x,y,z[,secs] [x,y,z[,secs] ...]
        # TCP waypoints in world frame, hand pointing down; each
        # waypoint is one IK + one trajectory, seeded from the last
  python3 arm.py grip open|close
Options: --yaw <deg> rotate the hand about world z (0 = fingers along y)
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
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # panda_link0 in world (TF)


def quat_down(yaw_deg=0.0):
    """Hand z pointing down (-world z); yaw about world z."""
    # base: 180deg about x -> (1,0,0,0). Then yaw about z: q_z * q_x
    h = math.radians(yaw_deg) / 2
    qz = np.array([0, 0, math.sin(h), math.cos(h)])  # x y z w
    qx = np.array([1.0, 0, 0, 0])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_to_R(x, y, z, w):
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
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.wait_js()

    def _on_js(self, m):
        self.js = m

    def wait_js(self):
        self.js = None
        t0 = time.time()
        while self.js is None and time.time() - t0 < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self.js is None:
            raise SystemExit("no /joint_states")
        return dict(zip(self.js.name, self.js.position))

    def arm_state(self):
        d = self.wait_js()
        s = JointState()
        for j in JOINTS:
            s.name.append(j)
            s.position.append(d[j])
        return s

    def fingers(self):
        d = self.wait_js()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def tcp_pose(self):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_state()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise SystemExit(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        q = p.orientation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        hand = np.array([p.position.x, p.position.y, p.position.z])
        tcp = hand + TCP * R[:, 2] + BASE_IN_WORLD
        return tcp, np.array([q.x, q.y, q.z, q.w])

    def solve_ik(self, tcp_world, quat, seed=None):
        q = np.asarray(quat, float)
        R = quat_to_R(*q)
        hand = np.asarray(tcp_world, float) - TCP * R[:, 2] - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = hand
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        req.ik_request.robot_state.joint_state = seed or self.arm_state()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            raise SystemExit("IK no answer")
        if r.error_code.val != 1:
            raise SystemExit(f"IK failed code={r.error_code.val} for tcp "
                             f"{np.round(tcp_world, 3)}")
        sol = dict(zip(r.solution.joint_state.name,
                       r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move_joints(self, positions, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(secs),
                                      nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        # verify
        d = self.wait_js()
        err = max(abs(d[j] - p) for j, p in zip(JOINTS, positions))
        print(f"  traj code={code} max joint err={err:.4f} rad")
        return code, err

    def goto(self, tcp_world, quat, secs=3.0):
        sol = self.solve_ik(tcp_world, quat)
        code, err = self.move_joints(sol, secs)
        if err > 0.02:
            print("  large joint error, resending")
            code, err = self.move_joints(sol, secs)
        tcp, q = self.tcp_pose()
        d = np.linalg.norm(tcp - np.asarray(tcp_world))
        print(f"  TCP now {np.round(tcp, 4)} (target {np.round(tcp_world, 4)}"
              f", off {d*1000:.1f} mm)")
        return tcp

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={f[0]:.4f},{f[1]:.4f}")
        return f


def main():
    args = sys.argv[1:]
    yaw = 0.0
    if "--yaw" in args:
        i = args.index("--yaw")
        yaw = float(args[i + 1])
        del args[i:i + 2]
    if not args:
        raise SystemExit(__doc__)
    arm = Arm()
    cmd = args[0]
    if cmd == "pose":
        tcp, q = arm.tcp_pose()
        print(f"TCP world {np.round(tcp, 4)} quat {np.round(q, 3)} "
              f"fingers {arm.fingers()}")
    elif cmd == "goto":
        q = quat_down(yaw)
        for wp in args[1:]:
            v = [float(x) for x in wp.split(",")]
            secs = v[3] if len(v) > 3 else 3.0
            print(f"goto {v[:3]} in {secs}s")
            arm.goto(v[:3], q, secs)
    elif cmd == "grip":
        arm.gripper(GRIP["open_m"] if args[1] == "open" else GRIP["closed_m"])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 17
python3 arm.py pose 2>&1 | grep -v INFO

# openrua op 18
python3 -c "
import sys; sys.argv=['x']; sys.path.insert(0,'/workspace')
import arm, rclpy
from moveit_msgs.srv import GetPositionFK
a=arm.Arm()
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand','panda_link0','panda_link8']
req.header.frame_id=''
req.robot_state.joint_state=a.arm_state()
fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=30)
r=fut.result(); print(r.error_code)
for n,ps in zip(r.fk_link_names,r.pose_stamped): print(n, ps.header.frame_id, ps.pose.position, ps.pose.orientation)
print(a.arm_state().position)
" 2>&1 | grep -v INFO

# openrua op 19
python3 -c "
import sys; sys.argv=['x']; sys.path.insert(0,'/workspace')
import arm, rclpy, numpy as np
a=arm.Arm()
cur=np.array(a.arm_state().position)
q=[0.9996,0,-0.0284,0]
for label,hand in (('world',[-0.053,0,0.7776]),('base',[0.457,0,0.3576])):
    try:
        # bypass tcp offset/base handling: pass tcp such that hand==given
        R=arm.quat_to_R(*q); tcp=np.array(hand)+arm.TCP*R[:,2]+arm.BASE_IN_WORLD
        sol=np.array(a.solve_ik(tcp,q)); print(label, np.round(sol-cur,3))
    except SystemExit as e: print(label, e)
" 2>&1 | grep -v INFO

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # panda_link0 in world (TF)", "new_string": "# verified live: /compute_fk and /compute_ik on this machine both speak\\n# the WORLD frame (frame_id left empty), so no base offset is needed\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        print(f\\"  TCP now {np.round(tcp, 4)} (target {np.round(tcp_world, 4)}\\"\\n              f\\", off {d*1000:.1f} mm)\\")", "new_string": "        print(f\\"  TCP now {np.round(tcp, 4)} (target {np.round(tcp_world, 4)}\\"\\n              f\\", off {d*1000:.1f} mm) quat {np.round(q, 3)}\\")", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 22
python3 arm.py goto -0.08,-0.197,0.70,3 2>&1 | grep -v INFO && python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>&1 | grep -v INFO

# openrua op 23
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    def goto(self, tcp_world, quat, secs=3.0):\\n        sol = self.solve_ik(tcp_world, quat)", "new_string": "    def goto(self, tcp_world, quat, secs=3.0):\\n        # machine quirk (verified by FK): /compute_ik applies the requested\\n        # orientation to panda_link8, which sits 45deg about z from\\n        # panda_hand; pre-rotate so the HAND ends up at `quat`\\n        q = np.asarray(quat, float)\\n        h = math.radians(-45.0) / 2\\n        x1, y1, z1, w1 = 0.0, 0.0, math.sin(h), math.cos(h)\\n        x2, y2, z2, w2 = q\\n        q_req = np.array([\\n            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,\\n            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,\\n            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,\\n            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,\\n        ])\\n        sol = self.solve_ik(tcp_world, q_req)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 24
python3 arm.py goto -0.08,-0.197,0.70,3 2>&1 | grep -v INFO && python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>&1 | grep -v INFO

# openrua op 25
python3 heightmap.py robot0_eye_in_hand 0.50 0.60 2>&1 | grep -v INFO; timeout 15 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A1 "At time" | head -2

# openrua op 26
python3 arm.py goto -0.079,-0.196,0.60,2 -0.079,-0.196,0.52,2 2>&1 | grep -v INFO && python3 arm.py grip close 2>&1 | grep -v INFO && python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v INFO

# openrua op 27
python3 arm.py goto -0.079,-0.196,0.68,2 2>&1 | grep -v INFO && python3 heightmap.py birdview 0.50 0.75 2>&1 | grep -v INFO

# openrua op 28
python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v INFO

# openrua op 29
python3 arm.py goto -0.021,-0.341,0.68,3 -0.021,-0.341,0.565,2 2>&1 | grep -v INFO && python3 arm.py grip open 2>&1 | grep -v INFO && python3 arm.py goto -0.021,-0.341,0.70,2 2>&1 | grep -v INFO && python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v INFO

# openrua op 30
python3 heightmap.py birdview 0.44 0.66 2>&1 | grep -v INFO

# openrua op 31
python3 arm.py goto -0.209,0.073,0.70,3 2>&1 | grep -v INFO && python3 heightmap.py birdview 0.44 0.66 2>&1 | grep -v INFO && python3 heightmap.py robot0_eye_in_hand 0.50 0.60 2>&1 | grep -v INFO

# openrua op 32
python3 arm.py goto -0.2065,0.074,0.60,2 -0.2065,0.074,0.512,2 2>&1 | grep -v INFO && python3 arm.py grip close 2>&1 | grep -v INFO

# openrua op 33
python3 arm.py goto -0.2065,0.074,0.70,2 2>&1 | grep -v INFO && python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v INFO

# openrua op 34
python3 arm.py goto -0.2065,0.074,0.76,2 -0.013,0.344,0.76,3 -0.013,0.344,0.55,2 2>&1 | grep -v INFO && python3 arm.py grip open 2>&1 | grep -v INFO && python3 arm.py goto -0.013,0.344,0.72,2 -0.15,0.15,0.80,3 2>&1 | grep -v INFO && python3 tools/perception/cam_snap.py agentview 2>&1 | grep -v INFO

# openrua op 35
python3 heightmap.py birdview 0.50 0.66 2>&1 | grep -v INFO; python3 -c "
import numpy as np
pw=np.load('birdview_world.npy')
for name,(x,y) in {'left plate':(-0.021,-0.296),'right plate':(-0.013,0.298)}.items():
    m=(pw[...,2]>0.50)&(pw[...,2]<0.66)&(np.hypot(pw[...,0]-x,pw[...,1]-y)<0.12)
    p=pw[m]; rim=p[p[:,2]>p[:,2].max()-0.012]
    cx,cy=rim[:,0].mean(),rim[:,1].mean()
    print(f'{name}: mug rim top z={p[:,2].max():.3f}, rim center ({cx:.3f},{cy:.3f}), offset from plate center {np.hypot(cx-x,cy-y)*100:.1f} cm, z spread {rim[:,2].max()-rim[:,2].min():.3f}')
"; python3 arm.py pose 2>&1 | grep -v INFO

# openrua op 36
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Verified-live quirks of the Franka Panda mug/plate workstation (IK frame, link8 yaw offset, settle step, rim grasp) that contradict or extend the starter docs
metadata:
  type: project
---

Facts verified live on the /workspace Panda workstation (2026-09-12), not in the docs:

- `/compute_ik` and `/compute_fk` both speak the WORLD frame (base at world (-0.51,0,0.42)); a base-frame pose fails with -31. docs/30-action.md says "planner frame is the arm base" — wrong for this machine.
- `/compute_ik` applies the requested orientation to `panda_link8`, which is rotated 45° about z from `panda_hand`. Pre-rotate the request by -45° yaw to get the hand where you want it; verify with FK. `/workspace/arm.py` already does this.
- Scene objects float ~5 cm at session start; send one harmless command (e.g. gripper open) before measuring anything.
- First trajectory to a far target often returns error -5 (tolerance); resending the same goal converges (docs are right about this).
- LIBERO mugs here are ~9.5–10 cm wide, wider than the 8 cm gripper opening. A rim grasp (one finger inside, one outside, opposite the handle, fingertips ~3 cm below the rim) holds them level; finger readings stop at ~4–6 mm per finger when gripping the wall.
- Agentview camera sits at world +x looking toward the robot: image-left = world −y = robot's right. "Left/right plate" in task text follows the agentview image.

**Why:** rediscovering these cost several turns of trial and error.
**How to apply:** reuse `/workspace/arm.py` (goto/grip/pose helpers with these fixes baked in) and `/workspace/heightmap.py` (birdview segmentation to world coords) if they still exist.
OPENRUA_EOF

# openrua op 37
cat /claude-config/projects/-workspace/memory/MEMORY.md 2>/dev/null; echo "- [Panda workstation quirks](panda-workstation-quirks.md) — IK is world-frame, link8 45° yaw offset, settle step, rim grasp for wide mugs" >> /claude-config/projects/-workspace/memory/MEMORY.md
