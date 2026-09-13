#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list 2>&1; echo ---; ros2 action list 2>&1; echo ---; ros2 service list 2>&1 | grep -iv parameter; echo ---; ros2 node list 2>&1

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once 2>&1 | head -40

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('agentview.png')
cv2.imwrite('crop1.png', cv2.resize(im[230:400,100:420],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop2.png', cv2.resize(im[240:390,300:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
for c in robot0_robotview galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 7
python3 -c "
import cv2
im=cv2.imread('birdview.png')
cv2.imwrite('crop3.png', cv2.resize(im[200:330,230:420],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('robot0_robotview.png')
cv2.imwrite('crop4.png', cv2.resize(im[0:260,20:460],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 8
timeout 60 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -8; timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | head -40

# openrua op 9
cat > getpts.py <<'EOF'
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rclpy.init(); node=rclpy.create_node('pts'); buf=Buffer(); TransformListener(buf,node)
cam=sys.argv[1]; pts=[tuple(map(int,a.split(','))) for a in sys.argv[2:]]
d=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
D=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
fr=f'{cam}_optical_frame'
while not buf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',fr,rclpy.time.Time()); q=t.transform.rotation
R=qR(q.x,q.y,q.z,q.w); o=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
for u,v in pts:
    z=D[v,u]; p=R@np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])+o
    print(f'({u},{v}) depth={z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}')
rclpy.shutdown()
EOF
timeout 120 python3 getpts.py birdview 268,287 271,303 291,298 336,299 383,290 291,264 281,229 200,200 320,320

# openrua op 10
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -4; timeout 20 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -4; timeout 30 ros2 topic echo /tf --once 2>&1 | grep -A1 frame_id | head; timeout 120 python3 getpts.py agentview 343,280 375,335 125,340 230,265 240,375 320,420

# openrua op 11
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small control helper: one node, clients built once, act -> verify.

Usage as a library (see run_*.py) or:
  python3 rob.py fk                      # print hand + tcp pose in base frame
  python3 rob.py grip <w>                # gripper per-finger width
  python3 rob.py tcp <x> <y> <z> [sec]   # IK (top-down, fingers along y) + FJT
"""
import sys, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_W = np.array([-0.51, 0.0, 0.42])   # panda_link0 in world (from /tf)
# top-down grasp orientation: hand z = -world z, hand x = world x
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_yaw_down(yaw):
    """Top-down orientation with the hand x axis rotated by yaw about world z."""
    # R = Rz(yaw) @ Rx(pi)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # q = qz * qx  with qx=(1,0,0,0), qz=(0,0,s,c)
    # (w1 w2 - v1.v2, w1 v2 + w2 v1 + v1 x v2)
    w = c * 0 - (0 * 1 + 0 * 0 + s * 0)
    v = c * np.array([1, 0, 0]) + 0 * np.array([0, 0, s]) + np.cross([0, 0, s], [1, 0, 0])
    return (v[0], v[1], v[2], w)


class Rob:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no fjt server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no ik"
        assert self.fk.wait_for_service(10), "no fk"

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
        while "m" not in self.js:
            self.spin(0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def _seed(self):
        s = JointState()
        j = self.joints()
        for n in ARM:
            s.name.append(n); s.position.append(j[n])
        return s

    def fk_pose(self):
        """hand pose in base frame -> (pos, quat), plus tcp position."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        assert r is not None and r.error_code.val == 1, f"fk failed {r}"
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP * quat_R(*q)[:, 2]
        return pos, q, tcp

    def ik_tcp(self, x, y, z, q=Q_DOWN):
        """IK for a TCP position (base frame) with orientation q. Returns joints or None."""
        R = quat_R(*q)
        hx, hy, hz = np.array([x, y, z]) - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = float(hx), float(hy), float(hz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = self._seed()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK failed: {None if r is None else r.error_code.val}")
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[n] for n in ARM]

    def move_q(self, q, sec=3.0, retries=1):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        for attempt in range(retries + 1):
            t0 = time.time()
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"  fjt code={code} max_joint_err={err:.4f} ({time.time()-t0:.0f}s)")
            if code == 0 and err < 0.02:
                return True
        return err < 0.05

    def move_tcp(self, x, y, z, q=Q_DOWN, sec=3.0):
        sol = self.ik_tcp(x, y, z, q)
        if sol is None:
            return False
        ok = self.move_q(sol, sec)
        pos, _, tcp = self.fk_pose()
        print(f"  tcp now {tcp.round(4)} target {np.array([x,y,z]).round(4)}")
        return ok

    def gripper(self, w):
        g = GripperCommand.Goal()
        g.command.position = float(w)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=180)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper -> {w}: reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def shutdown(self):
        rclpy.shutdown()


if __name__ == "__main__":
    r = Rob()
    cmd = sys.argv[1]
    if cmd == "fk":
        pos, q, tcp = r.fk_pose()
        print("hand", pos.round(4), "q", np.round(q, 4), "tcp", tcp.round(4))
        print("joints", np.round(r.arm_q(), 4), "fingers", r.fingers())
    elif cmd == "grip":
        r.gripper(float(sys.argv[2]))
    elif cmd == "tcp":
        x, y, z = map(float, sys.argv[2:5])
        sec = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
        r.move_tcp(x, y, z, sec=sec)
    r.shutdown()
OPENRUA_EOF

# openrua op 12
timeout 120 python3 -u rob.py fk 2>&1 | grep -v "^\[" 

# openrua op 13
python3 -c "
import numpy as np
def T(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0,-0.161,0,-2.4446,0,2.2268,0.7854]
A=[0,0,0,0.0825,-0.0825,0,0.088]; D=[0.333,0,0.316,0,0.384,0,0]; AL=[0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2]
M=np.eye(4)
for i in range(7): M=M@T(A[i],D[i],AL[i],q[i])
M=M@T(0,0.107,0,0)  # flange link8
print('link8', M[:3,3].round(4)); print(M[:3,:3].round(3))
"


# openrua op 14
cat > iktest.py <<'EOF'
import numpy as np
from rob import *
r = Rob()
pos, q, tcp = r.fk_pose()
print("fk hand", pos.round(4), "q", np.round(q,4))
cur = np.array(r.arm_q())
# hypothesis A: IK expects world frame -> use fk pos directly (hand pose, so pass tcp)
solA = r.ik_tcp(*tcp, q=q)
print("A (world) sol", None if solA is None else np.round(solA,3), "diff", None if solA is None else np.abs(np.array(solA)-cur).max().round(3))
# hypothesis B: base frame -> subtract base offset
tcpB = tcp - BASE_W
solB = r.ik_tcp(*tcpB, q=q)
print("B (base) sol", None if solB is None else np.round(solB,3), "diff", None if solB is None else np.abs(np.array(solB)-cur).max().round(3))
r.shutdown()
EOF
timeout 200 python3 -u iktest.py 2>&1 | grep -v "^\[INFO"

# openrua op 15
cat > step1.py <<'EOF'
import numpy as np
from rob import *
r = Rob()
r.gripper(0.04)
ok = r.move_tcp(0.0065, -0.2285, 0.60, sec=4)
print("ok", ok)
pos, q, tcp = r.fk_pose(); print("hand q", np.round(q,3))
r.shutdown()
EOF
timeout 600 python3 -u step1.py 2>&1 | grep -v "^\[INFO"; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 16
timeout 60 python3 tools/perception/cam_snap.py birdview && python3 -c "
import cv2
im=cv2.imread('birdview.png')
cv2.imwrite('crop3.png', cv2.resize(im[200:330,230:420],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 17
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.ik_link_name = \\"panda_hand\\"  # group tip is link8 (45deg off)\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 18
cat > step2.py <<'EOF'
import numpy as np
from rob import *
r = Rob()
ok = r.move_tcp(0.0065, -0.2285, 0.60, sec=3)
pos, q, tcp = r.fk_pose(); print("ok", ok, "hand q", np.round(q,3), "joints", np.round(r.arm_q(),3))
r.shutdown()
EOF
timeout 600 python3 -u step2.py 2>&1 | grep -v "^\[INFO"; timeout 120 python3 getpts.py birdview 329,260 326,255 333,265 268,287

# openrua op 19
cat > pick_cc.py <<'EOF'
import numpy as np
from rob import *
r = Rob()
x, y = 0.0065, -0.2285
print("descend"); r.move_tcp(x, y, 0.48, sec=3)
print("final"); r.move_tcp(x, y, 0.440, sec=3)
f = r.gripper(0.0)
print("lift"); r.move_tcp(x, y, 0.60, sec=3)
print("fingers after lift", r.fingers())
r.shutdown()
EOF
timeout 900 python3 -u pick_cc.py 2>&1 | grep -v "^\[INFO"; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 20
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 21
cat > park.py <<'EOF'
from rob import *
r = Rob(); r.gripper(0.04); r.move_tcp(-0.15, 0.0, 0.80, sec=4); r.shutdown()
EOF
timeout 600 python3 -u park.py 2>&1 | grep -v "^\[INFO"; timeout 60 python3 tools/perception/cam_snap.py birdview; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 22
for c in galleryview sideview paperview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 23
python3 -c "
import cv2
im=cv2.imread('galleryview.png')
cv2.imwrite('crop5.png', cv2.resize(im[320:480,340:460],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('birdview.png')
cv2.imwrite('crop6.png', cv2.resize(im[380:480,480:640],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 24
python3 -c "
import cv2
im=cv2.imread('galleryview.png')
cv2.imwrite('crop7.png', cv2.resize(im[240:480,0:640],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('paperview.png')
cv2.imwrite('crop8.png', cv2.resize(im[280:480,0:640],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 25
python3 -c "
import cv2
im=cv2.imread('birdview.png')
cv2.imwrite('crop9.png', cv2.resize(im[200:330,230:420],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 26
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 27
python3 -c "
import cv2
im=cv2.imread('frontview.png')
cv2.imwrite('crop10.png', cv2.resize(im[300:480,440:640],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png')
cv2.imwrite('crop11.png', cv2.resize(im[250:480,0:640],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 28
ls -la /workspace/.ros/log/; for f in /workspace/.ros/log/*.log; do echo "== $f"; head -c 1500 "$f"; echo; done 2>/dev/null | head -150

# openrua op 29
timeout 60 ros2 node info /robot_bridge 2>&1; timeout 30 ros2 param list /robot_bridge 2>&1 | head -30

# openrua op 30
python3 -c "
import cv2, numpy as np
for c in ['galleryview','paperview','frontview','sideview','birdview','agentview']:
    im=cv2.imread(c+'.png'); hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    # blue hue ~100-130 in opencv, saturation moderate
    m=((hsv[...,0]>95)&(hsv[...,0]<135)&(hsv[...,1]>60)&(hsv[...,2]>40)).astype(np.uint8)
    n,lab,st,cen=cv2.connectedComponentsWithStats(m)
    print(c,[ (tuple(cen[i].astype(int)),st[i,4]) for i in range(1,n) if st[i,4]>=4])
"

# openrua op 31
python3 -c "
import cv2, numpy as np
for c in ['galleryview','paperview','frontview','sideview','birdview']:
    im=cv2.imread(c+'.png'); hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    # floor: light wood, hue ~15-30, high value. find non-floor, non-table dark/colored small blobs
    floor=(hsv[...,0]>10)&(hsv[...,0]<35)&(hsv[...,2]>120)
    table=(hsv[...,2]<110)&(hsv[...,1]<200)
    m=(~floor).astype(np.uint8)
    n,lab,st,cen=cv2.connectedComponentsWithStats(m)
    print(c,[ (tuple(cen[i].astype(int)),st[i,4]) for i in range(1,n) if 4<=st[i,4]<=400])
"

# openrua op 32
python3 -c "
import cv2
im=cv2.imread('galleryview.png'); cv2.imwrite('c12.png', cv2.resize(im[380:480,500:600],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); cv2.imwrite('c13.png', cv2.resize(im[320:420,0:160],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('paperview.png'); cv2.imwrite('c14.png', cv2.resize(im[340:420,60:160],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 33
cat > floorscan.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from getpts import grab, qR
EOF
cat > floorscan.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rclpy.init(); node=rclpy.create_node('scan'); buf=Buffer(); TransformListener(buf,node)
for cam in sys.argv[1:]:
    d=grab(node,f'/{cam}/depth/image_raw',Image); info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
    D=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
    fr=f'{cam}_optical_frame'
    while not buf.can_transform('world',fr,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
    t=buf.lookup_transform('world',fr,rclpy.time.Time()); q=t.transform.rotation
    R=qR(q.x,q.y,q.z,q.w); o=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    v,u=np.mgrid[0:d.height,0:d.width]
    P=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D],-1).reshape(-1,3)@R.T+o
    P=P.reshape(d.height,d.width,3)
    np.save(f'{cam}_xyz.npy',P)
    z=P[...,2]; ok=np.isfinite(z)
    print(cam,'z range',np.nanmin(z[ok]),np.nanmax(z[ok]), 'cam at',o.round(2))
    # floor-level small objects: 0.015<z<0.2, and not the table pedestal (which is a cylinder around table center)
    m=(z>0.015)&(z<0.25)&ok
    r=np.hypot(P[...,0]+0.245,P[...,1])
    m&= r>0.5   # outside pedestal radius (~0.45)
    import cv2
    n,lab,st,cen=cv2.connectedComponentsWithStats(m.astype(np.uint8))
    for i in range(1,n):
        if st[i,4]>=3:
            pts=P[lab==i]; print('  blob px',cen[i].astype(int),'n',st[i,4],'world',pts.mean(0).round(3))
rclpy.shutdown()
EOF
timeout 300 python3 floorscan.py galleryview paperview frontview sideview birdview 2>&1 | grep -v INFO

# openrua op 34
python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('c15.png', cv2.resize(im[340:420,560:640],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('galleryview.png'); cv2.imwrite('c16.png', cv2.resize(im[180:240,270:350],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"

# openrua op 35
cat > geturdf.py <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('u'); got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data); print(len(got[0].data))
EOF
timeout 60 python3 geturdf.py 2>&1 | grep -v INFO; grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A12 'link name="panda_leftfinger"' robot.urdf | head -40

# openrua op 36
python3 -c "
import numpy as np, cv2
for c in ['galleryview','paperview','frontview','sideview']:
    P=np.load(c+'_xyz.npy'); z=P[...,2]
    ok=np.isfinite(z)
    inside=((P[...,0]+0.245)/0.9)**2+(P[...,1]/0.62)**2<1
    r=np.hypot(P[...,0]+0.245,P[...,1])
    m=ok&inside&(z>0.012)&(z<0.40)
    # pedestal: find radius stats of those points
    print(c,'under-table pts',m.sum(),'r min',r[m].min() if m.sum() else None, 'r percentiles',np.percentile(r[m],[1,5,50]) if m.sum() else None)
    mm=m&(r<0.30)
    print('   pts r<0.30:',mm.sum())
    n,lab,st,cen=cv2.connectedComponentsWithStats((m&(r<0.36)).astype(np.uint8))
    for i in range(1,n):
        if st[i,4]>=3: print('   blob',cen[i].astype(int),st[i,4],P[lab==i].mean(0).round(3))
"

# openrua op 37
python3 -c "
import numpy as np, cv2
im=cv2.imread('birdview.png'); hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
sub=im[240:285,310:350]
# butter: orange/red saturated
m=((hsv[...,1]>120)&(hsv[...,2]>90)).astype(np.uint8); m[:240]=0; m[285:]=0; m[:, :310]=0; m[:,350:]=0
n,lab,st,cen=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    if st[i,4]>5: print(cen[i], st[i,:4])
ys,xs=np.where(m>0); print('bbox',xs.min(),xs.max(),ys.min(),ys.max())
"
timeout 60 python3 getpts.py birdview 329,260 329,252 329,268 325,260 334,260 2>&1 | grep -v INFO

# openrua op 38
timeout 30 ros2 topic echo /tf --once 2>&1 | grep child_frame_id

# openrua op 39
cat > hover_b.py <<'EOF'
from rob import *
r = Rob(); r.move_tcp(-0.112, 0.040, 0.60, sec=4); r.shutdown()
EOF
timeout 600 python3 -u hover_b.py 2>&1 | grep -v "^\[INFO"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 40
python3 -c "
import numpy as np, cv2
im=cv2.imread('robot0_eye_in_hand.png'); hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
m=((hsv[...,1]>150)&(hsv[...,2]>120)&((hsv[...,0]<25)|(hsv[...,0]>160))).astype(np.uint8)
m[360:]=0
n,lab,st,cen=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    if st[i,4]>50: print(cen[i].round(1), st[i,:4])
"
timeout 60 python3 getpts.py robot0_eye_in_hand 318,301 318,262 318,340 300,301 336,301 2>&1 | grep -v INFO

# openrua op 41
cat > pick_b.py <<'EOF'
import numpy as np
from rob import *
r = Rob()
x, y = -0.111, 0.041
print("descend"); r.move_tcp(x, y, 0.50, sec=3)
print("final"); r.move_tcp(x, y, 0.442, sec=3)
f = r.gripper(0.0)
for i in range(3): print("fingers", r.fingers())
print("lift"); r.move_tcp(x, y, 0.60, sec=3)
print("fingers after lift", r.fingers())
r.shutdown()
EOF
timeout 900 python3 -u pick_b.py 2>&1 | grep -v "^\[INFO"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 42
cat > place_b.py <<'EOF'
import numpy as np
from rob import *
r = Rob()
print("up"); r.move_tcp(-0.111, 0.041, 0.66, sec=3)
print("over basket"); r.move_tcp(0.016, 0.28, 0.66, sec=4)
print("fingers", r.fingers())
print("lower"); r.move_tcp(0.016, 0.28, 0.56, sec=3)
print("fingers", r.fingers())
r.gripper(0.04)
print("retreat"); r.move_tcp(0.016, 0.28, 0.70, sec=3)
r.shutdown()
EOF
timeout 900 python3 -u place_b.py 2>&1 | grep -v "^\[INFO"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 43
python3 -c "
import numpy as np, cv2
known=[(-0.21,1.03),(0.85,1.27)]
for c in ['galleryview','paperview','frontview','sideview','birdview']:
    P=np.load(c+'_xyz.npy'); z=P[...,2]; ok=np.isfinite(z)
    m=ok&(z>0.008)&(z<0.09)
    n,lab,st,cen=cv2.connectedComponentsWithStats(m.astype(np.uint8))
    out=[]
    for i in range(1,n):
        if st[i,4]>=2:
            p=P[lab==i].mean(0)
            if min(np.hypot(p[0]-k[0],p[1]-k[1]) for k in known)<0.15: continue
            if np.hypot(p[0]+0.245,p[1])<0.5: continue
            out.append((tuple(cen[i].astype(int)),int(st[i,4]),tuple(p.round(2))))
    print(c,out)
"

# openrua op 44
python3 -c "
import cv2
im=cv2.imread('galleryview.png'); cv2.imwrite('c17.png', cv2.resize(im[195:225,285:330],None,fx=12,fy=12,interpolation=cv2.INTER_CUBIC))
"

# openrua op 45
python3 -c "
import numpy as np, cv2
res=0.05; xs=np.arange(-2.5,3.5,res); ys=np.arange(-3.5,3.0,res)
cov=np.zeros((len(ys),len(xs)),bool); anyp=np.zeros_like(cov)
for c in ['galleryview','paperview','frontview','sideview','birdview','agentview']:
    try: P=np.load(c+'_xyz.npy')
    except: continue
    z=P[...,2]; ok=np.isfinite(z)&(z<0.012)&(z>-0.05)
    pts=P[ok]
    ix=((pts[:,0]-xs[0])/res).astype(int); iy=((pts[:,1]-ys[0])/res).astype(int)
    good=(ix>=0)&(ix<len(xs))&(iy>=0)&(iy<len(ys))
    cov[iy[good],ix[good]]=True
# table footprint
X,Y=np.meshgrid(xs,ys); table=((X+0.245)/0.9)**2+(Y/0.62)**2<1
img=np.zeros(cov.shape+(3,),np.uint8); img[cov]=(255,255,255); img[table]=(80,40,0); img[table&cov]=(200,200,100)
img=cv2.flip(img,0)
cv2.imwrite('coverage.png', cv2.resize(img,None,fx=6,fy=6,interpolation=cv2.INTER_NEAREST))
print('floor cells seen',cov.sum(),'of',cov.size)
# list unseen cells near table (within 1.2m of table center) excluding under-table
near=(np.hypot(X+0.245,Y)<1.5)&~table&~cov
print('unseen near-table cells:',near.sum()); 
ys_,xs_=np.where(near); print(sorted(set(zip(np.round(X[near],1),np.round(Y[near],1))))[:80])
"

# openrua op 46
cat > look.py <<'EOF'
import sys, numpy as np
from rob import *
from scipy.spatial.transform import Rotation as Rot
def q_from_dir(d, up=(0,0,1)):
    z=np.array(d,float); z/=np.linalg.norm(z)
    y=np.cross(z,np.array(up,float)); y/=np.linalg.norm(y)   # horizontal
    x=np.cross(y,z)
    R=np.stack([x,y,z],1)
    return tuple(Rot.from_matrix(R).as_quat())  # x,y,z,w
r=Rob()
px,py,pz=map(float,sys.argv[1:4]); dx,dy,dz=map(float,sys.argv[4:7])
q=q_from_dir((dx,dy,dz))
print("q",np.round(q,3))
ok=r.move_tcp(px,py,pz,q=q,sec=4); print("ok",ok)
pos,qq,tcp=r.fk_pose(); print("hand",pos.round(3),"q",np.round(qq,3))
r.shutdown()
EOF
timeout 600 python3 -u look.py -0.2 0.3 0.95 1 1 -0.5 2>&1 | grep -v "^\[INFO"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 47
cat > sweep.py <<'EOF'
import numpy as np, cv2
from rob import *
from look import q_from_dir
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
r=Rob()
def snap(name):
    got=[]
    s=r.node.create_subscription(Image,'/robot0_eye_in_hand/color/image_raw',got.append,1)
    while not got: r.spin(0.2)
    r.node.destroy_subscription(s)
    cv2.imwrite(name, CvBridge().imgmsg_to_cv2(got[0],'bgr8')); print('saved',name)
dirs={'py':(0,1,-0.45),'px':(1,0,-0.45),'mxpy':(-1,1,-0.45),'pxmy':(1,-1,-0.45),'my':(0,-1,-0.45),'mxmy':(-1,-1,-0.45),'mx':(-1,0,-0.3)}
for k,d in dirs.items():
    # stand the hand up high, offset opposite the look direction
    p=np.array([-0.35,0.0,0.95])
    q=q_from_dir(d)
    ok=r.move_tcp(*p,q=q,sec=4); print(k,'ok',ok)
    snap(f'look_{k}.png')
r.shutdown()
EOF
sed -i 's/^r=Rob()$/if __name__=="__main__": r=Rob()/; s/^px,py,pz=/if __name__=="__main__": px,py,pz=/; s/^q=q_from_dir((dx,dy,dz))$/if __name__=="__main__": q=q_from_dir((dx,dy,dz))/' look.py
python3 - <<'EOF'
# rewrite look.py main into a guard properly
src=open('look.py').read()
head,rest=src.split('r=Rob()',1) if 'if __name__' not in src else (None,None)
EOF
cat > look.py <<'EOF'
import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
def q_from_dir(d, up=(0,0,1)):
    z=np.array(d,float); z/=np.linalg.norm(z)
    y=np.cross(z,np.array(up,float)); y/=np.linalg.norm(y)   # horizontal
    x=np.cross(y,z)
    R=np.stack([x,y,z],1)
    return tuple(Rot.from_matrix(R).as_quat())  # x,y,z,w
if __name__=="__main__":
    from rob import *
    r=Rob()
    px,py,pz=map(float,sys.argv[1:4]); dx,dy,dz=map(float,sys.argv[4:7])
    q=q_from_dir((dx,dy,dz)); print("q",np.round(q,3))
    ok=r.move_tcp(px,py,pz,q=q,sec=4); print("ok",ok)
    pos,qq,tcp=r.fk_pose(); print("hand",pos.round(3),"q",np.round(qq,3))
    r.shutdown()
EOF
timeout 1500 python3 -u sweep.py 2>&1 | grep -v "^\[INFO"

# openrua op 48
python3 -c "
import cv2, numpy as np
ims=[cv2.imread(f'look_{k}.png') for k in ['py','px','mxpy','pxmy','my','mxmy','mx']]
ims.append(np.zeros_like(ims[0]))
rows=[np.hstack(ims[i:i+2]) for i in range(0,8,2)]
g=np.vstack(rows); cv2.imwrite('sweep.png', cv2.resize(g,None,fx=0.75,fy=0.75))
"

# openrua op 49
cat > side.py <<'EOF'
from rob import *
r=Rob(); print(r.move_tcp(-0.30,-0.45,0.75,sec=4)); r.shutdown()
EOF
timeout 600 python3 -u side.py 2>&1 | grep -v "^\[INFO"; timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 50
python3 -c "
import cv2
im=cv2.imread('birdview.png')
cv2.imwrite('crop18.png', cv2.resize(im[90:380,120:530],None,fx=2.3,fy=2.3,interpolation=cv2.INTER_CUBIC))
"
