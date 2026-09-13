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
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -iv parameter; echo ---; ros2 node list

# openrua op 5
ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview galleryview paperview; do python3 tools/perception/cam_snap.py $c; done

# openrua op 6
python3 -c "
import cv2
im=cv2.imread('agentview.png')
cv2.imwrite('crop_cans.png', cv2.resize(im[190:300,190:400], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))
im2=cv2.imread('robot0_robotview.png')
cv2.imwrite('crop_rv.png', cv2.resize(im2[0:270,40:440], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC))
"

# openrua op 7
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -12; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 8
mkdir -p "$(dirname /workspace/px_batch.py)"
cat > /workspace/px_batch.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Batch pixel->world for one camera. Usage: px_batch.py <cam> u,v [u,v ...]
Also saves <cam>_depth.npy for offline use."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=15.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def quat_T(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4); T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cam = sys.argv[1]
    pts = [tuple(map(int, a.split(","))) for a in sys.argv[2:]]
    rclpy.init(); node = rclpy.create_node("px_batch")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    np.save(f"{cam}_depth.npy", D)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform("world", frame, rclpy.time.Time()):
            break
    T = quat_T(buf.lookup_transform("world", frame, rclpy.time.Time()))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    for u, v in pts:
        z = float(D[v, u])
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"px({u},{v}) depth={z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 9
python3 px_batch.py agentview 225,215 225,240 365,240 365,255 138,375 240,330 540,230 540,280 320,420 460,300 620,300 470,190 600,190 2>&1 | grep -v INFO

# openrua op 10
python3 px_batch.py birdview 285,237 295,293 268,305 385,290 370,275 400,305 320,200 250,250 2>&1 | grep -v INFO; python3 -c "
import cv2
im=cv2.imread('birdview.png')
cv2.imwrite('crop_bird.png', cv2.resize(im[200:340,220:430], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))"

# openrua op 11
sleep 2; python3 px_batch.py birdview 285,237 295,293 268,305 385,290 370,275 400,305 320,200 250,250 2>&1 | grep -v INFO

# openrua op 12
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Save a world-frame XYZ cloud (H,W,3) for a camera as <cam>_xyz.npy."""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from px_batch import grab, quat_T


def main():
    cam = sys.argv[1]
    rclpy.init(); node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform("world", frame, rclpy.time.Time()):
            break
    T = quat_T(buf.lookup_transform("world", frame, rclpy.time.Time()))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    v, u = np.mgrid[0:depth.height, 0:depth.width]
    P = np.stack([(u - cx) * D / fx, (v - cy) * D / fy, D, np.ones_like(D)], -1)
    W = P @ T.T
    np.save(f"{cam}_xyz.npy", W[..., :3].astype(np.float32))
    print("saved", f"{cam}_xyz.npy", W.shape)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 13
python3 cloud.py birdview 2>&1 | grep -v INFO && python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy')
z=W[...,2]
# alphabet soup can region: around px (285,237)
def blob(u0,v0,r,zmin):
    sub=W[v0-r:v0+r,u0-r:u0+r]
    m=(sub[...,2]>zmin)&(sub[...,2]<0.7)
    pts=sub[m]
    print('n',m.sum(),'xy mean',pts[:,:2].mean(0),'x range',pts[:,0].min(),pts[:,0].max(),'y range',pts[:,1].min(),pts[:,1].max(),'z max',pts[:,2].max(), 'z med', np.median(pts[:,2]))
print('alphabet'); blob(285,237,14,0.46)
print('cream cheese'); blob(268,305,14,0.44)
print('basket'); blob(385,290,30,0.58)
"


# openrua op 14
python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy')
sub=W[305-16:305+16,268-16:268+16]
m=(sub[...,2]>0.44)&(sub[...,2]<0.5)
pts=sub[m][:,:2]
c=pts.mean(0); X=pts-c
ev,evec=np.linalg.eigh(X.T@X)
print('center',c,'axes',evec,'ev',ev)
print('long axis angle deg (from x)', np.degrees(np.arctan2(evec[1,1],evec[0,1])))
"
python3 - <<'EOF' 2>&1 | grep -v INFO
import rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node('fk')
js={}
n.create_subscription(JointState,'/joint_states',lambda m: js.setdefault('m',m),1)
while 'm' not in js: rclpy.spin_once(n,timeout_sec=0.2)
cli=n.create_client(GetPositionFK,'/compute_fk'); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand','panda_link8']
s=JointState()
for a,b in zip(js['m'].name,js['m'].position):
    if 'finger' not in a: s.name.append(a); s.position.append(b)
req.robot_state.joint_state=s
f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=30)
r=f.result()
print(r.error_code)
for ps in r.pose_stamped:
    p=ps.pose.position; q=ps.pose.orientation
    print(ps.header.frame_id, round(p.x,4),round(p.y,4),round(p.z,4),'q',round(q.x,4),round(q.y,4),round(q.z,4),round(q.w,4))
EOF

# openrua op 15
python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy')
z=W[...,2]
m=(z>0.70)&(z<0.90)&(np.abs(W[...,1])<0.3)&(W[...,0]>-0.3)
pts=W[m]
print('n',m.sum(),'xyz mean',pts.mean(0),'x range',pts[:,0].min(),pts[:,0].max())
"; timeout 15 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A1 Translation | head -3; ros2 topic echo /robot_description --qos-durability transient_local --once 2>/dev/null | grep -oE 'link name="world"|<joint name="[^"]*" type="fixed">|origin[^/]*panda_link0' | head

# openrua op 16
cat /tmp/claude-1000/-workspace/f8df0ecc-a41a-4f29-881b-d03c77585f4e/tasks/b972ok1nr.output

# openrua op 17
python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy')
sub=W[240:340,340:430]
z=sub[...,2]
floor=(z>0.47)&(z<0.52)
p=sub[floor]
print('floor n',floor.sum(),'x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max(),'z',np.median(p[:,2]))
rim=(z>0.58)
p=sub[rim]
print('rim n',rim.sum(),'x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max(),'z',np.median(p[:,2]), p[:,2].max())
# wall: pixels with z between
"


# openrua op 18
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Mini command driver for the Panda: one node, reused clients.

Usage: python3 -u arm.py "<cmd>" ["<cmd>" ...]
  grip <per_finger_m>                      open/close gripper, report finger gap
  goto <x> <y> <z> [secs] [qx qy qz qw]    TCP to world pose (default top-down)
  fk                                       print current hand + TCP pose
  js                                       print joint state
Poses are in the planner model frame (== world here, verified by FK).
"""
import sys
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034
TOPDOWN = (1.0, 0.0, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Arm:
    def __init__(self):
        self.node = rclpy.create_node("arm_driver")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fkc = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)
        assert self.ik.wait_for_service(10) and self.fkc.wait_for_service(10)
        self.spin_until(lambda: self.js is not None, 10)

    def _on_js(self, m):
        self.js = m

    def spin_until(self, pred, timeout):
        end = self.node.get_clock().now().nanoseconds / 1e9 + timeout
        while not pred() and self.node.get_clock().now().nanoseconds / 1e9 < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return pred()

    def fresh_js(self):
        old = self.js
        self.spin_until(lambda: self.js is not old, 5)
        return dict(zip(self.js.name, self.js.position))

    def arm_state(self):
        j = self.fresh_js()
        s = JointState()
        s.name = list(ARM)
        s.position = [j[n] for n in ARM]
        return s

    def call(self, cli, req, timeout=60):
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=timeout)
        return f.result()

    def fk(self):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_state()
        r = self.call(self.fkc, req)
        p, q = r.pose_stamped[0].pose.position, r.pose_stamped[0].pose.orientation
        R = quat_R(q.x, q.y, q.z, q.w)
        hand = np.array([p.x, p.y, p.z])
        tcp = hand + TCP * R[:, 2]
        return hand, tcp, (q.x, q.y, q.z, q.w)

    def goto(self, x, y, z, secs=4.0, quat=TOPDOWN):
        R = quat_R(*quat)
        hx, hy, hz = np.array([x, y, z]) - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = float(hx), float(hy), float(hz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self.arm_state()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        r = self.call(self.ik, req)
        if r is None or r.error_code.val != 1:
            print(f"IK FAILED for TCP ({x},{y},{z}): {None if r is None else r.error_code.val}")
            return False
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        target = [sol[n] for n in ARM]
        ok = self.move(target, secs)
        hand, tcp, q = self.fk()
        err = np.linalg.norm(tcp - np.array([x, y, z]))
        print(f"  TCP now {tcp.round(4)} (err {err*1000:.1f} mm) q={np.round(q,3)}")
        return ok and err < 0.01

    def move(self, target, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in target])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        for attempt in range(2):
            f = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, f, timeout_sec=30)
            rf = f.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
            code = rf.result().result.error_code
            j = self.fresh_js()
            jerr = max(abs(j[n] - t) for n, t in zip(ARM, target))
            print(f"  traj error_code={code} max joint err={jerr:.4f} rad")
            if code == 0 and jerr < 0.02:
                return True
        return False

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        f = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=30)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        res = rf.result().result
        j = self.fresh_js()
        f1, f2 = j["panda_finger_joint1"], j["panda_finger_joint2"]
        print(f"  gripper reached={res.reached_goal} stalled={res.stalled} "
              f"fingers={f1:.4f},{f2:.4f} gap={f1 - f2:.4f}")
        return f1, f2


def main():
    rclpy.init()
    a = Arm()
    for cmd in sys.argv[1:]:
        print(">>", cmd, flush=True)
        t = cmd.split()
        if t[0] == "grip":
            a.gripper(float(t[1]))
        elif t[0] == "goto":
            x, y, z = map(float, t[1:4])
            secs = float(t[4]) if len(t) > 4 else 4.0
            quat = tuple(map(float, t[5:9])) if len(t) > 8 else TOPDOWN
            if not a.goto(x, y, z, secs, quat):
                print("!! goto did not converge; stopping sequence")
                break
        elif t[0] == "fk":
            hand, tcp, q = a.fk()
            print(f"  hand {hand.round(4)} tcp {tcp.round(4)} q {np.round(q,4)}")
        elif t[0] == "js":
            j = a.fresh_js()
            print("  " + " ".join(f"{k}={v:.4f}" for k, v in j.items()))
        sys.stdout.flush()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 19
python3 -u arm.py fk js "grip 0.04" "goto -0.218 -0.155 0.60 4" fk 2>&1 | grep -v INFO

# openrua op 20
python3 -u arm.py js 2>&1 | grep -v INFO; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png

# openrua op 21
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    rclpy.init()\\n    a = Arm()", "new_string": "    import time\\n    for attempt in range(5):  # DDS peer resolution flakes right after another process exits\\n        try:\\n            rclpy.init()\\n            a = Arm()\\n            break\\n        except Exception as e:  # noqa\\n            print(\\"node init retry:\\", str(e).splitlines()[0])\\n            try:\\n                rclpy.shutdown()\\n            except Exception:\\n                pass\\n            time.sleep(2)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 22
python3 -u arm.py "goto -0.218 -0.155 0.50 3" "goto -0.218 -0.155 0.455 3" "grip 0.0" fk 2>&1 | grep -v INFO

# openrua op 23
python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png; python3 tools/perception/cam_snap.py agentview ag2.png; python3 -c "
import cv2
im=cv2.imread('ag2.png'); cv2.imwrite('ag2c.png', cv2.resize(im[150:330,120:340],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 24
python3 -u arm.py "goto -0.218 -0.155 0.56 3" fk 2>&1 | grep -v INFO && sleep 1 && python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v INFO && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png && python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_xyz.npy')
z=W[...,2]
m=(z>0.485)&(z<0.52)&(np.isfinite(z))
pts=W[m]
print('can top n',m.sum(),'center',pts[:,:2].mean(0),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z',np.median(pts[:,2]))
vs,us=np.nonzero(m); print('pixel center',us.mean(),vs.mean())
"

# openrua op 25
python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_xyz.npy')
z=W[...,2]
m=(z>0.485)&(z<0.52)&(np.isfinite(z))&(np.hypot(W[...,0]+0.218,W[...,1]+0.155)<0.06)
pts=W[m]
print('can top n',m.sum(),'center',pts[:,:2].mean(0),'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z',np.median(pts[:,2]))
vs,us=np.nonzero(m); print('pixel center',us.mean(),vs.mean(), 'u range',us.min(),us.max(),'v range',vs.min(),vs.max())
# table z check
t=(np.hypot(W[...,0]+0.15,W[...,1]+0.10)<0.03)
print('table z near', np.median(z[t]))
"

# openrua op 26
python3 -u arm.py "goto -0.2135 -0.1535 0.52 3" "goto -0.2135 -0.1535 0.49 3" "goto -0.2135 -0.1535 0.46 3" 2>&1 | grep -v INFO

# openrua op 27
python3 - <<'EOF' 2>&1 | grep -v INFO
import rclpy, time
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
for i in range(5):
    try:
        rclpy.init(); n=rclpy.create_node('fk2'); break
    except Exception as e:
        time.sleep(2)
js={}
n.create_subscription(JointState,'/joint_states',lambda m: js.setdefault('m',m),1)
while 'm' not in js: rclpy.spin_once(n,timeout_sec=0.2)
cli=n.create_client(GetPositionFK,'/compute_fk'); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=['panda_link8','panda_hand','panda_hand_tcp','panda_leftfinger','panda_rightfinger']
s=JointState()
for a,b in zip(js['m'].name,js['m'].position):
    if 'finger' not in a: s.name.append(a); s.position.append(b)
req.robot_state.joint_state=s
f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=30)
r=f.result()
print(r.error_code, r.fk_link_names)
for name,ps in zip(r.fk_link_names, r.pose_stamped):
    p=ps.pose.position; q=ps.pose.orientation
    print(name, round(p.x,4),round(p.y,4),round(p.z,4),'q',round(q.x,4),round(q.y,4),round(q.z,4),round(q.w,4))
print('fingers', [(a,round(b,4)) for a,b in zip(js['m'].name,js['m'].position) if 'finger' in a])
EOF
ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>/dev/null | head -20

# openrua op 28
python3 -u arm.py "grip 0.0" fk 2>&1 | grep -v INFO

# openrua op 29
python3 -u arm.py "goto -0.208 -0.155 0.70 3" "goto 0.008 0.247 0.72 4" "goto 0.008 0.247 0.655 3" 2>&1 | grep -v INFO && python3 tools/perception/cam_snap.py agentview ag3.png

# openrua op 30
python3 -u arm.py "grip 0.04" "goto 0.008 0.247 0.78 3" 2>&1 | grep -v INFO && python3 tools/perception/cam_snap.py agentview ag4.png

# openrua op 31
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        elif t[0] == \\"fk\\":", "new_string": "        elif t[0] == \\"j7\\":  # rotate about the hand axis: joint7 += delta rad\\n            j = a.fresh_js()\\n            target = [j[n] for n in ARM]\\n            target[6] += float(t[1])\\n            a.move(target, float(t[2]) if len(t) > 2 else 2.0)\\n            hand, tcp, q = a.fk()\\n            print(f\\"  hand {hand.round(4)} tcp {tcp.round(4)} q {np.round(q,4)} \\"\\n                  f\\"fingerY={np.round(quat_R(*q)[:,1],3)}\\")\\n        elif t[0] == \\"fk\\":", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 32
python3 -u arm.py js "goto 0.086 -0.218 0.60 4" js "j7 0.7854 2" 2>&1 | grep -v INFO

# openrua op 33
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))\\n        target = [sol[n] for n in ARM]\\n        ok = self.move(target, secs)", "new_string": "        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))\\n        target = [sol[n] for n in ARM]\\n        # the IK plugin returns solutions yawed about the hand axis; joint7 IS\\n        # that axis, so correct it in joint space before moving\\n        for _ in range(2):\\n            Rs = quat_R(*self.fk_of(target)[1])\\n            Rrel = Rs.T @ R\\n            yaw = float(np.arctan2(Rrel[1, 0], Rrel[0, 0]))\\n            if abs(yaw) < 0.01:\\n                break\\n            cand = target[6] + yaw\\n            if not (-2.85 < cand < 2.85):\\n                cand = target[6] + yaw - np.sign(yaw) * 2 * np.pi\\n            if not (-2.85 < cand < 2.85):\\n                print(f\\"  yaw fix {yaw:.3f} would exceed joint7 limit; leaving as is\\")\\n                break\\n            target[6] = cand\\n        ok = self.move(target, secs)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 34
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    def fk(self):\\n        req = GetPositionFK.Request()\\n        req.fk_link_names = [\\"panda_hand\\"]\\n        req.robot_state.joint_state = self.arm_state()\\n        r = self.call(self.fkc, req)", "new_string": "    def fk_of(self, target):\\n        s = JointState()\\n        s.name = list(ARM)\\n        s.position = [float(v) for v in target]\\n        req = GetPositionFK.Request()\\n        req.fk_link_names = [\\"panda_hand\\"]\\n        req.robot_state.joint_state = s\\n        r = self.call(self.fkc, req)\\n        p, q = r.pose_stamped[0].pose.position, r.pose_stamped[0].pose.orientation\\n        return np.array([p.x, p.y, p.z]), (q.x, q.y, q.z, q.w)\\n\\n    def fk(self):\\n        req = GetPositionFK.Request()\\n        req.fk_link_names = [\\"panda_hand\\"]\\n        req.robot_state.joint_state = self.arm_state()\\n        r = self.call(self.fkc, req)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 35
sleep 1; python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v INFO && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png && python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_xyz.npy')
z=W[...,2]
m=(z>0.44)&(z<0.47)&np.isfinite(z)&(np.hypot(W[...,0]-0.086,W[...,1]+0.218)<0.08)
pts=W[m]
c=pts[:,:2].mean(0)
print('box top n',m.sum(),'center',c,'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z med',np.median(pts[:,2]),'z max',pts[:,2].max())
X=pts[:,:2]-c; ev,evec=np.linalg.eigh(X.T@X); print('long axis angle deg', np.degrees(np.arctan2(evec[1,1],evec[0,1])))
vs,us=np.nonzero(m); print('pixel center',us.mean(),vs.mean())
"

# openrua op 36
python3 -u arm.py "goto 0.09 -0.2175 0.50 3" "goto 0.09 -0.2175 0.437 3" "grip 0.0" fk 2>&1 | grep -v INFO

# openrua op 37
python3 -u arm.py "goto 0.09 -0.2175 0.65 3" "goto 0.008 0.293 0.70 4" "goto 0.008 0.293 0.655 3" 2>&1 | grep -v INFO && python3 tools/perception/cam_snap.py agentview ag5.png

# openrua op 38
python3 -u arm.py "grip 0.04" "goto 0.008 0.293 0.80 3" "goto -0.15 0.05 0.80 4" 2>&1 | grep -v INFO && python3 tools/perception/cam_snap.py agentview ag6.png && python3 tools/perception/cam_snap.py birdview bird6.png

# openrua op 39
python3 -c "
import cv2
im=cv2.imread('bird6.png'); cv2.imwrite('bird6c.png', cv2.resize(im[230:340,330:440],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))"

# openrua op 40
sleep 1; python3 cloud.py birdview 2>&1 | grep -v INFO && python3 -c "
import numpy as np
W=np.load('birdview_xyz.npy'); z=W[...,2]
inb=(W[...,0]>-0.06)&(W[...,0]<0.07)&(W[...,1]>0.20)&(W[...,1]<0.335)
can=inb&(z>0.56)&(z<0.60)   # can top ~ floor 0.498 + 0.084
box=inb&(z>0.515)&(z<0.545) # box top ~ floor + 0.03
for n,m in [('can',can),('box',box)]:
    p=W[m]; print(n,'n',m.sum(),'center',p[:,:2].mean(0).round(3) if m.sum() else None,'z med',np.median(p[:,2]).round(3) if m.sum() else None)
# and confirm nothing at the old table spots
for n,(x,y) in [('old can spot',(-0.218,-0.155)),('old box spot',(0.086,-0.218))]:
    m=(np.hypot(W[...,0]-x,W[...,1]-y)<0.03); print(n,'max z',z[m].max().round(3))
"
