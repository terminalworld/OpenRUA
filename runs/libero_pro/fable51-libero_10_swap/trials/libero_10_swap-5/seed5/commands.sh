#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -iv parameter | head -40; echo ----; ros2 node list

# openrua op 4
mkdir -p img && for c in agentview birdview frontview sideview galleryview paperview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c img/$c.png & done; wait; ls -la img; ros2 topic echo /joint_states --once

# openrua op 5
ros2 topic echo /birdview/color/camera_info --once | head -30; ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12 & sleep 4; kill %1 2>/dev/null; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 6
cat /tmp/claude-1000/-workspace/7cdd4be4-9d0f-49c9-ad1e-d99e08103427/tasks/b0a006lyr.output; pkill -f tf2_echo; pkill -f "topic echo"

# openrua op 7
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab depth+info+TF for a camera, save world-frame XYZ array (H,W,3) to <cam>_xyz.npy."""
import sys
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=30.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
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
    buf = Buffer(); TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not buf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u = np.arange(W)[None, :].repeat(H, 0)
    v = np.arange(H)[:, None].repeat(W, 1)
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1)
    world = pc @ R.T + p
    np.save(f"img/{cam}_xyz.npy", world)
    np.save(f"img/{cam}_depth.npy", depth)
    print("cam pos", p, "R", R.round(3).tolist())
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 90 python3 cloud.py birdview && timeout 90 python3 cloud.py agentview

# openrua op 9
python3 -c "
import numpy as np
W=np.load('img/birdview_xyz.npy')
z=W[...,2]
# table height: mode of z in table region
import collections
print('z hist', np.histogram(z, bins=np.arange(0,1.6,0.05)))
# print z at some pixels: robot base, caddy, book, mug
for name,(u,v) in {'robot':(320,50),'caddy_L':(235,180),'caddy_R':(290,185),'book':(315,270),'mug':(375,245),'table':(320,350)}.items():
    print(name, W[v,u].round(3))
"


# openrua op 10
python3 -c "
import numpy as np
np.set_printoptions(linewidth=250)
W=np.load('img/birdview_xyz.npy')
z=W[...,2]
# caddy region approx u 205..320, v 150..215
sub=z[150:215:3, 205:325:3]
print('rows v=150..215 step3, cols u=205..325 step3; heights above table (cm)')
print(((sub-0.88)*100).round(0).astype(int))
"


# openrua op 11
python3 -c "
import cv2
im=cv2.imread('img/agentview.png'); cv2.imwrite('img/agent_crop.png', cv2.resize(im[140:290,100:380],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('img/birdview.png'); cv2.imwrite('img/bird_crop.png', cv2.resize(im[140:300,190:400],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('img/sideview.png'); cv2.imwrite('img/side_crop.png', cv2.resize(im[200:340,260:520],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 12
python3 -c "
import numpy as np
np.set_printoptions(linewidth=250)
W=np.load('img/birdview_xyz.npy')
z=W[...,2]
sub=z[245:300:2, 296:336:2]
print('book region v=245..300 step2, u=296..336 step2 (cm above table)')
print(((sub-0.88)*100).round(0).astype(int))
m=(z[230:310,290:345]>0.95)
vs,us=np.nonzero(m); vs+=230; us+=290
pts=W[vs,us]
print('book pts n=',len(vs))
print('x range',pts[:,0].min().round(3),pts[:,0].max().round(3),'y range',pts[:,1].min().round(3),pts[:,1].max().round(3),'z max',pts[:,2].max().round(3))
print('centroid',pts.mean(0).round(3))
# PCA for orientation
c=pts[:,:2]-pts[:,:2].mean(0); u_,s,vt=np.linalg.svd(c,full_matrices=False); print('principal dir',vt[0].round(3),'sv',s.round(2))
print('angle of long axis from x axis (deg)', np.degrees(np.arctan2(vt[0][1],vt[0][0])).round(1))
"


# openrua op 13
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library: joint state, FK, IK, trajectory, gripper (one node, reused clients)."""
import sys
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    # returns x,y,z,w
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(R).as_quat()
    return q


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")

    def _on_js(self, m):
        self._js = dict(zip(m.name, m.position))

    def _on_wr(self, m):
        f = m.wrench.force
        t = m.wrench.torque
        self._wr = np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def spin(self, s=0.1):
        rclpy.spin_once(self.node, timeout_sec=s)

    def joints(self, fresh=True):
        if fresh:
            self._js = {}
            end = time.time() + 20
            while not self._js and time.time() < end:
                self.spin(0.2)
        return self._js

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in ARM]

    def fingers(self):
        js = self.joints()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    def wrench(self):
        self._wr = {}
        end = time.time() + 10
        while isinstance(self._wr, dict) and time.time() < end:
            self.spin(0.2)
        return self._wr

    def _seed(self, q=None):
        js = JointState()
        q = q if q is not None else self.arm_q()
        js.name = list(ARM)
        js.position = [float(x) for x in q]
        return js

    def fk(self, q=None, link="panda_hand"):
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def ik_solve(self, pos, quat, seed=None, timeout=30):
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.timeout.sec = 2
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_q(self, q, seconds=3.0, waypoints=None):
        """Send trajectory; waypoints: list of (q, t) before the final point."""
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        for wq, wt in (waypoints or []):
            pt = JointTrajectoryPoint(positions=[float(x) for x in wq])
            pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        h = send.result()
        res = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qa = np.array(self.arm_q())
        err = np.abs(qa - np.array(q)).max()
        print(f"move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik_solve(pos, quat, seed)
        if q is None:
            print("IK FAILED for", pos, quat, flush=True)
            return None
        return self.move_q(q, seconds)

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}", flush=True)
        return r


def topdown_quat(yaw_deg=0.0):
    """Hand z pointing down (world -z), hand x rotated by yaw about world z."""
    from scipy.spatial.transform import Rotation
    R = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix() @ np.diag([1.0, -1.0, -1.0])
    return R_quat(R)
OPENRUA_EOF

# openrua op 14
timeout 120 python3 -c "
from rob import *
r=Robot()
q=r.arm_q(); print('q',np.round(q,3))
pos,quat=r.fk(q); print('hand pos',pos.round(4),'quat',quat.round(4))
R=quat_R(*quat); print('hand axes in base frame:\n',R.round(3))
print('fingers',r.fingers())
print('wrench',r.wrench())
"

# openrua op 15
sed -i 's/self.ik = self.node.create_client/self.ik_cli = self.node.create_client/; s/self.fk = self.node.create_client/self.fk_cli = self.node.create_client/; s/self.fk.wait_for_service/self.fk_cli.wait_for_service/; s/fut = self.fk.call_async/fut = self.fk_cli.call_async/; s/self.ik.wait_for_service/self.ik_cli.wait_for_service/; s/fut = self.ik.call_async/fut = self.ik_cli.call_async/' rob.py && grep -n "_cli" rob.py && timeout 120 python3 -c "
from rob import *
r=Robot()
q=r.arm_q(); print('q',np.round(q,3))
pos,quat=r.fk(q); print('hand pos',pos.round(4),'quat',quat.round(4))
R=quat_R(*quat); print('hand axes in base frame:\n',R.round(3))
print('fingers',r.fingers())
print('wrench',r.wrench())
"

# openrua op 16
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -A2 "Translation" | head -3; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A1 "Translation" | head -2

# openrua op 17
timeout 300 python3 -u -c "
from rob import *
r=Robot()
q=r.ik_solve([-0.3,0.45,1.25], topdown_quat(0))
print('ik',None if q is None else np.round(q,3))
if q: 
    r.move_q(q, 3.0)
    pos,quat=r.fk(); print('hand now',pos.round(3),quat.round(3))
" && timeout 60 python3 tools/perception/cam_snap.py birdview img/bird2.png && timeout 90 python3 cloud.py birdview

# openrua op 18
timeout 120 python3 -c "
from rob import *
r=Robot()
p8,q8=r.fk(link='panda_link8'); ph,qh=r.fk(link='panda_hand')
print('link8',p8.round(3),q8.round(3)); print('hand ',ph.round(3),qh.round(3))
from scipy.spatial.transform import Rotation as Ro
print('link8 yaw-ish euler', Ro.from_quat(q8).as_euler('xyz',degrees=True).round(1))
print('hand yaw-ish euler', Ro.from_quat(qh).as_euler('xyz',degrees=True).round(1))
"

# openrua op 19
python3 -c "
import numpy as np, cv2
np.set_printoptions(linewidth=250)
W=np.load('img/birdview_xyz.npy'); z=W[...,2]
sub=z[150:215:3, 205:355:3]
print('rows v=150..215 step3, cols u=205..355 step3; cm above table 0.88')
print(((sub-0.88)*100).round(0).astype(int))
im=cv2.imread('img/bird2.png'); cv2.imwrite('img/bird2_crop.png', cv2.resize(im[140:300,190:400],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 20
python3 -c "
import numpy as np, sys
np.set_printoptions(linewidth=300, threshold=sys.maxsize)
W=np.load('img/birdview_xyz.npy'); z=W[...,2]
sub=z[156:210:3, 210:355:3]
print('rows v=156..210 step3, cols u=210..355 step3; cm above table 0.88')
print(((sub-0.88)*100).round(0).astype(int))
"

# openrua op 21
python3 -c "
import numpy as np, cv2
W=np.load('img/birdview_xyz.npy')
im=cv2.imread('img/bird2.png')
g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
roi=g[240:300,290:340]
m=roi<70
vs,us=np.nonzero(m); vs+=240; us+=290
print('dark px',len(vs))
# use table-plane geometry: book top at z=1.075 -> depth 1.925 ; compute world xy for those pixels at that depth
d=3.0-1.075; f=579.4112549695428
x=-0.2+(vs-240)*d/f; y=(us-320)*d/f
pts=np.stack([x,y],1)
c=pts.mean(0); print('centroid',c.round(4))
cc=pts-c; u_,s,vt=np.linalg.svd(cc,full_matrices=False)
ax=vt[0]; 
if ax[0]<0: ax=-ax
print('long axis',ax.round(3),'angle deg',np.degrees(np.arctan2(ax[1],ax[0])).round(1))
proj=cc@ax; perp=cc@np.array([-ax[1],ax[0]])
print('length',(proj.max()-proj.min()).round(4),'thick',(perp.max()-perp.min()).round(4))
print('ends', (c+ax*proj.min()).round(3),(c+ax*proj.max()).round(3))
cv2.imwrite('img/book_crop.png', cv2.resize(im[240:300,290:340],None,fx=8,fy=8,interpolation=cv2.INTER_NEAREST))
"

# openrua op 22
python3 -c "
import numpy as np, sys
np.set_printoptions(linewidth=300, threshold=sys.maxsize)
W=np.load('img/birdview_xyz.npy'); z=W[...,2]
for v in (170,174,178):
    print('v',v,'u=255..305:', ((z[v,255:306]-0.88)*100).round(0).astype(int))
    print('   y at u=255..305 step5:', W[v,255:306:5,1].round(3))
print('x along v=160..190 at u=280:')
print(((z[160:191,280]-0.88)*100).round(0).astype(int))
print(W[160:191:2,280,0].round(3))
"

# openrua op 23
timeout 600 python3 -u -c "
from rob import *
r=Robot()
r.gripper(0.04)
TCP=0.1034
yaw_hand=-10.0
q8=topdown_quat(yaw_hand-45)
target=np.array([-0.087,-0.011,1.075+0.10+TCP])
q=r.ik_solve(target,q8)
print('ik',None if q is None else np.round(q,3))
if q:
    r.move_q(q,3.0)
    ph,qh=r.fk(link='panda_hand'); print('hand',ph.round(4),qh.round(3))
    from scipy.spatial.transform import Rotation as Ro
    print('hand euler',Ro.from_quat(qh).as_euler('xyz',degrees=True).round(1))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih1.png

# openrua op 24
timeout 600 python3 -u -c "
from rob import *
r=Robot()
q=[-0.35,0.432,0.388,-1.678,-0.182,2.075,1.054]
r.move_q(q,3.0)
print('q now',np.round(r.arm_q(),3))
ph,qh=r.fk(link='panda_hand'); print('hand',ph.round(4),qh.round(3))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih1.png

# openrua op 25
timeout 600 python3 -u -c "
from rob import *
r=Robot()
TCP=0.1034
q8=topdown_quat(-10.0-45)
target=np.array([-0.087,-0.011,1.055+TCP])
q=r.ik_solve(target,q8)
print('ik',None if q is None else np.round(q,3))
if q:
    code,err=r.move_q(q,2.5)
    if code!=0 or err>0.01: r.move_q(q,2.5)
    ph,qh=r.fk(link='panda_hand'); print('hand',ph.round(4),qh.round(3))
    print('wrench',r.wrench().round(2))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih2.png && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent2.png

# openrua op 26
timeout 600 python3 -u -c "
from rob import *
r=Robot()
r.gripper(0.0)
print('fingers',r.fingers())
print('wrench',r.wrench().round(2))
"

# openrua op 27
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih3.png && timeout 60 python3 tools/perception/cam_snap.py sideview img/side3.png

# openrua op 28
timeout 600 python3 -u -c "
from rob import *
r=Robot()
TCP=0.1034
q8=topdown_quat(-10.0-45)
target=np.array([-0.087,-0.011,1.20+TCP])
q=r.ik_solve(target,q8)
code,err=r.move_q(q,2.5)
if code!=0 or err>0.01: r.move_q(q,2.5)
ph,qh=r.fk(link='panda_hand'); print('hand',ph.round(4),qh.round(3))
print('fingers',r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent3.png && timeout 90 python3 cloud.py birdview

# openrua op 29
timeout 90 python3 cloud.py birdview 2>&1 | grep -v XMLPARSER; timeout 90 python3 cloud.py agentview 2>&1 | grep -v XMLPARSER; python3 -c "
import numpy as np
W=np.load('img/birdview_xyz.npy'); z=W[...,2]
m=(z[230:310,280:350]>0.95); vs,us=np.nonzero(m); vs+=230; us+=280
pts=W[vs,us]; print('bird: book n',len(vs),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'ztop',pts[:,2].max().round(4), 'z pct',np.percentile(pts[:,2],[50,90,99]).round(4))
A=np.load('img/agentview_xyz.npy')
# agentview: select points near book location
sel=(np.abs(A[...,0]+0.09)<0.08)&(np.abs(A[...,1]+0.01)<0.05)&(A[...,2]>0.9)
p=A[sel]; print('agent: book n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'ztop',p[:,2].max().round(4))
# thickness: y extent at height 1.0..1.05
q=p[(p[:,2]>1.0)&(p[:,2]<1.05)]; print('y at z 1.0-1.05:',np.percentile(q[:,1],[1,50,99]).round(4), 'x', np.percentile(q[:,0],[1,50,99]).round(4))
"


# openrua op 30
python3 -c "
import numpy as np
A=np.load('img/agentview_xyz.npy')
sel=(np.abs(A[...,0]+0.09)<0.08)&(np.abs(A[...,1]+0.01)<0.05)&(A[...,2]>0.9)&(A[...,2]<1.15)
p=A[sel]; print('agent: book n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'ztop',p[:,2].max().round(4), np.percentile(p[:,2],[90,99,99.9]).round(4))
W=np.load('img/birdview_xyz.npy'); z=W[...,2]
m=(z[230:310,280:350]>0.95)&(z[230:310,280:350]<1.15); vs,us=np.nonzero(m); vs+=230; us+=280
pts=W[vs,us]; print('bird: book n',len(vs),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'ztop',pts[:,2].max().round(4), 'z pct',np.percentile(pts[:,2],[50,90,99]).round(4))
# top-edge slice: points with z>1.06 in agent cloud
q=p[p[:,2]>1.06]; print('agent top-edge pts', len(q), 'x',np.percentile(q[:,0],[1,50,99]).round(3),'y',np.percentile(q[:,1],[1,50,99]).round(3))
"


# openrua op 31
python3 -c "
import numpy as np, sys
np.set_printoptions(linewidth=300, threshold=sys.maxsize)
W=np.load('img/birdview_xyz.npy'); z=W[...,2]
print('table z samples', z[350,320].round(4), z[300,200].round(4), z[220,280].round(4))
print('v=174 u=255..305 cm:', ((z[174,255:306]-0.88)*100).round(0).astype(int))
print('u=280 v=158..190 cm:', ((z[158:191,280]-0.88)*100).round(0).astype(int))
print('u=280 x at v=158..190:', W[158:191,280,0].round(3))
print('v=174 y at u=255..305:', W[174,255:306,1].round(3))
A=np.load('img/agentview_xyz.npy')
sel=(np.abs(A[...,0]+0.09)<0.08)&(np.abs(A[...,1]+0.01)<0.05)&(A[...,2]>0.86)&(A[...,2]<1.15)
p=A[sel]; print('agent book z pct', np.percentile(p[:,2],[0.5,2,10,50,98,99.9]).round(4))
"


# openrua op 32
timeout 900 python3 -u -c "
from rob import *
r=Robot()
TCP=0.1034
q8=topdown_quat(-10.0-45)
r.gripper(0.04)
# pre-grasp just above, then descend
for zt in (1.06, 0.992):
    target=np.array([-0.089,-0.011,zt+TCP])
    q=r.ik_solve(target,q8)
    code,err=r.move_q(q,2.5)
    if code!=0 or err>0.01: r.move_q(q,2.5)
ph,qh=r.fk(link='panda_hand'); print('hand',ph.round(4),qh.round(3))
r.gripper(0.0)
for i in range(5):
    for _ in range(5): r.spin(0.1)
    print('fingers',r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih4.png && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent4.png

# openrua op 33
timeout 900 python3 -u -c "
from rob import *
r=Robot()
TCP=0.1034
q8=topdown_quat(-10.0-45)
target=np.array([-0.089,-0.011,1.20+TCP])
q=r.ik_solve(target,q8)
code,err=r.move_q(q,3.0)
if code!=0 or err>0.01: r.move_q(q,3.0)
ph,qh=r.fk(link='panda_hand'); print('hand',ph.round(4),qh.round(3))
print('fingers',r.fingers())
print('wrench',r.wrench().round(2))
" && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent5.png && timeout 90 python3 cloud.py birdview 2>&1 | grep -v XMLPARSER

# openrua op 34
timeout 900 python3 -u -c "
from rob import *
r=Robot()
TCP=0.1034
q8=topdown_quat(-90-45)
target=np.array([-0.089,-0.011,1.25+TCP])
q=r.ik_solve(target,q8)
print('ik',None if q is None else np.round(q,3))
code,err=r.move_q(q,3.0)
if code!=0 or err>0.01: r.move_q(q,3.0)
ph,qh=r.fk(link='panda_hand'); print('hand',ph.round(4),qh.round(3))
from scipy.spatial.transform import Rotation as Ro
print('hand euler',Ro.from_quat(qh).as_euler('xyz',degrees=True).round(1))
print('fingers',r.fingers())
" && timeout 90 python3 cloud.py agentview 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent6.png

# openrua op 35
timeout 900 python3 -u -c "
from rob import *
r=Robot()
q=[-0.221,0.352,0.242,-1.577,-0.088,1.918,2.393]
for i in range(4):
    code,err=r.move_q(q,3.0)
    if code==0 and err<0.01: break
print('q',np.round(r.arm_q(),3))
ph,qh=r.fk(link='panda_hand'); print('hand',ph.round(4),qh.round(3))
from scipy.spatial.transform import Rotation as Ro
print('hand euler',Ro.from_quat(qh).as_euler('xyz',degrees=True).round(1))
print('fingers',r.fingers())
" && timeout 90 python3 cloud.py agentview 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent6.png

# openrua op 36
python3 -c "
import numpy as np
A=np.load('img/agentview_xyz.npy')
sel=(A[...,2]>1.15)&(A[...,2]<1.225)&(A[...,0]>-0.25)&(A[...,0]<0.1)&(np.abs(A[...,1])<0.2)
p=A[sel]; print('n',len(p))
print('x pct',np.percentile(p[:,0],[0.5,2,50,98,99.5]).round(4))
print('y pct',np.percentile(p[:,1],[0.2,1,50,99,99.8]).round(4))
print('z pct',np.percentile(p[:,2],[0.5,50,99.5]).round(4))
# face points: those with x within 5mm of the 2nd percentile... the visible face is the +x face
face=p[np.abs(p[:,0]-np.median(p[:,0]))<0.006]
print('face n',len(face),'y range',face[:,1].min().round(4),face[:,1].max().round(4),'center y',((face[:,1].min()+face[:,1].max())/2).round(4),'x med',np.median(face[:,0]).round(4))
# bottom edge
sel2=(A[...,2]>1.10)&(A[...,2]<1.30)&(A[...,0]>-0.25)&(A[...,0]<0.1)&(np.abs(A[...,1])<0.12)
q=A[sel2]; print('book z min',q[:,2].min().round(4),'z max',q[:,2].max().round(4))
"


# openrua op 37
timeout 900 python3 -u -c "
from rob import *
r=Robot()
TCP=0.1034
q8=topdown_quat(-90-45)
target=np.array([-0.440,-0.1415,1.25+TCP])
q=r.ik_solve(target,q8)
print('ik',None if q is None else np.round(q,3))
for i in range(4):
    code,err=r.move_q(q,3.0)
    if code==0 and err<0.01: break
ph,qh=r.fk(link='panda_hand'); print('hand',ph.round(4),qh.round(3))
from scipy.spatial.transform import Rotation as Ro
print('hand euler',Ro.from_quat(qh).as_euler('xyz',degrees=True).round(1))
print('fingers',r.fingers())
" && timeout 90 python3 cloud.py agentview 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent7.png && python3 -c "
import numpy as np
A=np.load('img/agentview_xyz.npy')
sel=(A[...,2]>1.15)&(A[...,2]<1.225)&(A[...,0]>-0.6)&(A[...,0]<-0.3)&(A[...,1]>-0.3)&(A[...,1]<0.0)
p=A[sel]; print('n',len(p))
print('x pct',np.percentile(p[:,0],[0.5,50,99.5]).round(4))
print('y range',p[:,1].min().round(4),p[:,1].max().round(4),'center',((p[:,1].min()+p[:,1].max())/2).round(4))
print('z min',p[:,2].min().round(4))
"

# openrua op 38
timeout 900 python3 -u -c "
from rob import *
r=Robot()
TCP=0.1034
q8=topdown_quat(-90-45)
w0=r.wrench(); print('wrench before',w0.round(2))
target=np.array([-0.440,-0.1415,1.08+TCP])
q=r.ik_solve(target,q8)
print('ik',None if q is None else np.round(q,3))
for i in range(4):
    code,err=r.move_q(q,3.0)
    if code==0 and err<0.01: break
ph,qh=r.fk(link='panda_hand'); print('hand',ph.round(4),qh.round(3))
print('wrench after',r.wrench().round(2))
print('fingers',r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent8.png && timeout 60 python3 tools/perception/cam_snap.py sideview img/side8.png

# openrua op 39
timeout 900 python3 -u -c "
from rob import *
r=Robot()
TCP=0.1034
r.gripper(0.04)
q8=topdown_quat(-90-45)
target=np.array([-0.440,-0.1415,1.30+TCP])
q=r.ik_solve(target,q8)
for i in range(4):
    code,err=r.move_q(q,3.0)
    if code==0 and err<0.01: break
ph,qh=r.fk(link='panda_hand'); print('hand',ph.round(4))
print('fingers',r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent9.png && timeout 60 python3 tools/perception/cam_snap.py frontview img/front9.png

# openrua op 40
timeout 900 python3 -u -c "
from rob import *
r=Robot()
q=r.ik_solve([-0.3,0.45,1.25], topdown_quat(0))
for i in range(4):
    code,err=r.move_q(q,3.0)
    if code==0 and err<0.01: break
print('hand',r.fk(link='panda_hand')[0].round(3))
" && timeout 90 python3 cloud.py birdview 2>&1 | grep -v XMLPARSER && timeout 60 python3 tools/perception/cam_snap.py birdview img/bird10.png && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent10.png && python3 -c "
import numpy as np, sys, cv2
np.set_printoptions(linewidth=300, threshold=sys.maxsize)
W=np.load('img/birdview_xyz.npy'); z=W[...,2]
sub=z[156:210:3, 210:355:3]
print('caddy height map (cm above table):')
print(((sub-0.88)*100).round(0).astype(int))
# book: points inside caddy middle section region above the floor but not walls: search x in [-0.47,-0.40], y in [-0.21,-0.07], z between 0.93 and 1.05
sel=(W[...,0]>-0.475)&(W[...,0]<-0.40)&(W[...,1]>-0.21)&(W[...,1]<-0.075)&(z>0.925)&(z<1.05)
p=W[sel]; print('candidate book-top pts',len(p))
if len(p): print('x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',np.percentile(p[:,2],[5,50,95]).round(3))
im=cv2.imread('img/bird10.png'); cv2.imwrite('img/bird10_crop.png', cv2.resize(im[140:230,200:360],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"
