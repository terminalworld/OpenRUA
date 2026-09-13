#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -iv parameter | head -40; echo ----; ros2 node list

# openrua op 4
cd /workspace; for c in agentview frontview birdview sideview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c & done; wait; ros2 topic echo /joint_states --once

# openrua op 5
cd /workspace; timeout 20 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -12; echo ----; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ----; ros2 topic echo /agentview/color/camera_info --once | head -30

# openrua op 6
cd /workspace; python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png & python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png & wait; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | grep -A5 Translation | head -6; ros2 topic echo /birdview/color/camera_info --once | grep -A9 "^k:"

# openrua op 7
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera; save world-frame point cloud.

Usage: python3 scene.py <camera>  ->  <camera>.png, <camera>_xyz.npy (HxWx3 world)
"""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=20.0):
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
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene_grab")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    xyz = pc @ R.T + T
    cv2.imwrite(f"{cam}.png", color)
    np.save(f"{cam}_xyz.npy", xyz)
    np.save(f"{cam}_bgr.npy", color)
    print(f"saved {cam}.png {cam}_xyz.npy; cam at {T}, depth range {np.nanmin(depth):.2f}-{np.nanmax(depth):.2f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
cd /workspace; python3 scene.py agentview & python3 scene.py birdview & wait

# openrua op 9
cd /workspace; python3 -c "
import numpy as np
xyz=np.load('agentview_xyz.npy'); bgr=np.load('agentview_bgr.npy')
# table height: mode of z in lower image
z=xyz[...,2]
print('table z estimate (median lower half):', np.median(z[300:,:]))
# sample pixels: plate center, pudding, red mug, white mug
for name,(u,v) in {'plate':(320,380),'pudding':(385,295),'redmug_body':(345,200),'whitemug':(205,220),'redmug_top':(345,150)}.items():
    print(name, (u,v), xyz[v,u].round(3), bgr[v,u])
"


# openrua op 10
cd /workspace; python3 -c "
import numpy as np
for cam in ['agentview','birdview']:
    xyz=np.load(f'{cam}_xyz.npy'); bgr=np.load(f'{cam}_bgr.npy')
    z=xyz[...,2]
    print(cam)
    for (u,v) in [(100,450),(320,450),(550,450),(320,300),(100,300),(600,250),(320,330)]:
        print('  px',(u,v),'->',xyz[v,u].round(3), bgr[v,u])
"


# openrua op 11
cd /workspace; python3 tools/action/gripper_cmd.py 0.04; python3 scene.py birdview; python3 scene.py agentview

# openrua op 12
mkdir -p "$(dirname /workspace/segment.py)"
cat > /workspace/segment.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment table objects in birdview point cloud by height above table; report clusters."""
import numpy as np
import cv2

TABLE_Z = 0.426
xyz = np.load("birdview_xyz.npy")
bgr = np.load("birdview_bgr.npy")
z = xyz[..., 2]
# table region: within table footprint (x in -0.6..0.6, y in -0.8..0.8), exclude the robot (x < -0.3 near y=0 handled by color later)
above = (z > TABLE_Z + 0.008) & (z < TABLE_Z + 0.4) & np.isfinite(z)
mask = above.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
out = bgr.copy()
for i in range(1, n):
    area = stats[i, cv2.CC_STAT_AREA]
    if area < 15:
        continue
    m = lab == i
    pts = xyz[m]
    col = bgr[m].mean(0)
    x0, y0, x1, y1 = pts[:, 0].min(), pts[:, 1].min(), pts[:, 0].max(), pts[:, 1].max()
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    print(f"comp {i}: px_area={area} centroid_px=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
          f"world center=({cx:.3f},{cy:.3f}) x[{x0:.3f},{x1:.3f}] y[{y0:.3f},{y1:.3f}] "
          f"ztop={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} bgr={col.round(0)}")
    x, y, w, h = stats[i, 0], stats[i, 1], stats[i, 2], stats[i, 3]
    cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 1)
    cv2.putText(out, str(i), (x, y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
cv2.imwrite("birdview_seg.png", out)
OPENRUA_EOF

# openrua op 13
cd /workspace; python3 segment.py

# openrua op 14
cd /workspace; python3 -c "
import numpy as np, cv2
xyz=np.load('agentview_xyz.npy'); bgr=np.load('agentview_bgr.npy')
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
above=(z>0.434)&(z<0.75)&(x>-0.3)&(x<0.35)&(abs(y)<0.5)
n,lab,stats,cents=cv2.connectedComponentsWithStats(above.astype(np.uint8),8)
out=bgr.copy()
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i; p=xyz[m]; c=bgr[m].mean(0)
    print(f'comp {i} area={stats[i,4]} px=({cents[i][0]:.0f},{cents[i][1]:.0f}) x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} bgr={c.round(0)}')
    X,Y,W,H=stats[i,:4]; cv2.rectangle(out,(X,Y),(X+W,Y+H),(0,255,0),1); cv2.putText(out,str(i),(X,Y-2),0,0.4,(0,255,255),1)
cv2.imwrite('agentview_seg.png',out)
"


# openrua op 15
cd /workspace; timeout 20 ros2 param list /robot_bridge 2>&1 | head -30; echo ----; timeout 10 ros2 topic echo /pipeline_state --once 2>&1 | head -20; echo ----; ros2 interface show moveit_msgs/srv/GetPositionFK | head -30

# openrua op 16
cd /workspace; python3 -c "
import numpy as np, cv2
b=np.load('birdview_bgr.npy'); a=np.load('agentview_bgr.npy')
cv2.imwrite('zoom_bird.png', cv2.resize(b[200:280,280:380], None, fx=6, fy=6, interpolation=cv2.INTER_NEAREST))
cv2.imwrite('zoom_agent.png', cv2.resize(a[120:260,270:420], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
xyz=np.load('birdview_xyz.npy'); z=xyz[...,2]
# red mug top rim: pixels with z>0.55 near mug
m=(z>0.54)&(z<0.6)&(xyz[...,0]<-0.15)&(xyz[...,0]>-0.3)&(abs(xyz[...,1])<0.12)
p=xyz[m]; print('rim pts',m.sum(),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3))
# print rows
ys,xs=np.where(m); print('px u',xs.min(),xs.max(),'v',ys.min(),ys.max())
"


# openrua op 17
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Persistent helper: FK/IK, trajectories, gripper, joint/wrench reads. World frame."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE = np.array([-0.510, 0.0, 0.420])  # panda_link0 in world (tf2_echo)
TCP = M["hand"]["tcp_offset_m"]
HAND = "panda_hand"


def q_down(yaw_deg=0.0):
    """Quaternion (x,y,z,w) for hand pointing straight down; yaw about world z.
    yaw=0 -> fingers open along world y; yaw=90 -> along world x."""
    R = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])
    return R.as_quat()


class Robot:
    def __init__(self, name="robot_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self.js = m

    def _on_wr(self, m):
        self.wr = m

    def spin(self, t=0.3):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self.spin(0.3)
        d = dict(zip(self.js.name, self.js.position))
        return np.array([d[j] for j in ARM])

    def fingers(self):
        self.spin(0.3)
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self.spin(0.3)
        if self.wr is None:
            return None
        f = self.wr.wrench.force
        t = self.wrench_t = self.wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    # ---------- kinematics ----------
    def _seed(self, q=None):
        q = self.joints() if q is None else q
        s = JointState()
        s.name = list(ARM)
        s.position = [float(v) for v in q]
        return s

    def fk(self, q=None, tcp=True):
        """World pose (xyz, quat xyzw) of hand (or TCP)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [HAND]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        if tcp:
            xyz = xyz + Rot.from_quat(q).as_matrix()[:, 2] * TCP
        return xyz, q

    def ik(self, xyz, quat, tcp=True, seed=None, attempts=3):
        """Joint solution for world pose of TCP (or hand). Returns np.array or None."""
        xyz = np.asarray(xyz, float)
        if tcp:
            xyz = xyz - Rot.from_quat(quat).as_matrix()[:, 2] * TCP
        p_base = xyz - BASE
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.ik_link_name = HAND
            req.ik_request.pose_stamped.header.frame_id = ""
            pp = req.ik_request.pose_stamped.pose
            pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
            pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state = self._seed(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.array([sol[j] for j in ARM])
        return None

    # ---------- motion ----------
    def move_joints(self, waypoints, seconds):
        """waypoints: list of joint arrays; seconds: total time (spread evenly) or list."""
        waypoints = [np.asarray(w, float) for w in waypoints]
        if np.isscalar(seconds):
            times = np.linspace(0, seconds, len(waypoints) + 1)[1:]
        else:
            times = list(seconds)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for w, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("FJT goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = np.abs(self.joints() - waypoints[-1]).max()
        return code, err

    def move_tcp(self, xyz, quat, seconds=3.0, n=1, seed=None):
        """Straight-ish line to TCP pose via n IK waypoints. Returns (code, joint_err, final_tcp_err)."""
        start, _ = self.fk()
        wps = []
        seed_q = self.joints() if seed is None else seed
        for i in range(1, n + 1):
            p = start + (np.asarray(xyz) - start) * i / n
            q = self.ik(p, quat, seed=seed_q)
            if q is None:
                raise RuntimeError(f"IK failed at waypoint {i}/{n}: {p}")
            wps.append(q)
            seed_q = q
        code, jerr = self.move_joints(wps, seconds)
        now, _ = self.fk()
        return code, jerr, np.asarray(xyz) - now

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])
OPENRUA_EOF

# openrua op 18
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
print('joints', r.joints().round(3))
print('fingers', r.fingers())
xyz,q=r.fk(); print('TCP world', xyz.round(3), 'quat', q.round(3))
xyz,q=r.fk(tcp=False); print('hand world', xyz.round(3))
print('wrench', r.wrench())
# test IK roundtrip at current pose
sol=r.ik(*r.fk()); print('ik roundtrip', None if sol is None else sol.round(3))
print('q_down(0)', q_down(0).round(3), 'q_down(90)', q_down(90).round(3))
" 2>&1 | tail -20

# openrua op 19
cd /workspace; python3 -c "
import numpy as np
def dh(a,d,alpha,theta):
    ca,sa,ct,st=np.cos(alpha),np.sin(alpha),np.cos(theta),np.sin(theta)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0,-0.161,0,-2.445,0,2.227,0.785]
# Panda modified DH (Craig): (a, d, alpha)
P=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
T=np.eye(4)
for (a,d,al),th in zip(P,q): T=T@dh(a,d,al,th)
T=T@dh(0,0.107,0,0)  # flange (link8)
print('flange in base', T[:3,3].round(3))
Th=T@dh(0,0,0,-np.pi/4)  # panda_hand
print('hand z axis', Th[:3,2].round(3))
print('tcp in base', (T[:3,3]+Th[:3,2]*0.1034).round(3))
"


# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "BASE = np.array([-0.510, 0.0, 0.420])  # panda_link0 in world (tf2_echo)", "new_string": "# Verified: /compute_fk and /compute_ik poses are already WORLD-frame here\\n# (DH FK + camera cross-check), despite the docs\' base-frame note.\\nBASE = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
print('close on air ->', r.close())
print('open ->', r.open())
print('TCP', r.fk()[0].round(3))
" 2>&1 | tail -5

# openrua op 22
cd /workspace; python3 -c "
import numpy as np
for cam in ['agentview','birdview']:
    xyz=np.load(f'{cam}_xyz.npy')
    m=(xyz[...,2]>0.555)&(xyz[...,2]<0.59)&(xyz[...,0]<-0.12)&(xyz[...,0]>-0.3)&(xyz[...,1]>-0.02)&(xyz[...,1]<0.12)
    p=xyz[m][:,:2]
    # algebraic circle fit
    A=np.c_[2*p, np.ones(len(p))]; b=(p**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
    print(cam, 'n',len(p),'center',round(cx,3),round(cy,3),'radius',round(r,3), 'xrange',p[:,0].min().round(3),p[:,0].max().round(3),'yrange',p[:,1].min().round(3),p[:,1].max().round(3))
"


# openrua op 23
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
tgt=[-0.201,0.078,0.70]
code,jerr,perr=r.move_tcp(tgt,q_down(0),seconds=4.0,n=1)
print('code',code,'joint err',round(jerr,4),'tcp err',perr.round(4))
print('joints',r.joints().round(3)); print('TCP',r.fk()[0].round(3),'quat',r.fk()[1].round(3))
" 2>&1 | tail -4; python3 tools/perception/cam_snap.py agentview >/dev/null; python3 tools/perception/cam_snap.py sideview > /dev/null

# openrua op 24
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
w0=r.wrench()
code,jerr,perr=r.move_tcp([-0.201,0.078,0.59],q_down(0),seconds=3.0,n=2)
print('to 0.59: code',code,'jerr',round(jerr,4),'perr',perr.round(4),'wrench d',(r.wrench()-w0).round(2))
code,jerr,perr=r.move_tcp([-0.201,0.078,0.545],q_down(0),seconds=2.5,n=2)
print('to 0.545: code',code,'jerr',round(jerr,4),'perr',perr.round(4),'wrench d',(r.wrench()-w0).round(2))
print('TCP',r.fk()[0].round(4))
" 2>&1 | tail -3; python3 tools/perception/cam_snap.py agentview >/dev/null; python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 25
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
print('close ->', r.close())
r.spin(1.0); print('fingers settled', r.fingers())
print('wrench', r.wrench().round(2))
" 2>&1 | tail -3

# openrua op 26
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
code,jerr,perr=r.move_tcp([-0.201,0.078,0.72],q_down(0),seconds=3.0,n=2)
print('lift: code',code,'jerr',round(jerr,4),'perr',perr.round(4)); print('fingers',r.fingers()); print('wrench',r.wrench().round(2))
" 2>&1 | tail -3; python3 tools/perception/cam_snap.py agentview >/dev/null; python3 tools/perception/cam_snap.py sideview >/dev/null

# openrua op 27
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
code,jerr,perr=r.move_tcp([0.145,0.044,0.72],q_down(0),seconds=6.0,n=3)
print('transit: code',code,'jerr',round(jerr,4),'perr',perr.round(4)); print('fingers',r.fingers()); print('TCP',r.fk()[0].round(3))
" 2>&1 | tail -3; python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 28
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
for i in range(2):
    code,jerr,perr=r.move_tcp([0.145,0.044,0.72],q_down(0),seconds=3.0,n=1)
    print('resend',i,': code',code,'jerr',round(jerr,4),'perr',perr.round(4))
    if abs(perr).max()<0.005: break
print('fingers',r.fingers()); print('TCP',r.fk()[0].round(3))
" 2>&1 | tail -4

# openrua op 29
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
w0=r.wrench()
code,jerr,perr=r.move_tcp([0.145,0.044,0.575],q_down(0),seconds=4.0,n=3)
print('descend: code',code,'jerr',round(jerr,4),'perr',perr.round(4),'wrench d',(r.wrench()-w0).round(2))
if abs(perr).max()>0.01:
    code,jerr,perr=r.move_tcp([0.145,0.044,0.575],q_down(0),seconds=2.0,n=1)
    print('resend: code',code,'jerr',round(jerr,4),'perr',perr.round(4),'wrench d',(r.wrench()-w0).round(2))
print('TCP',r.fk()[0].round(3))
print('open ->', r.open())
code,jerr,perr=r.move_tcp([0.145,0.044,0.72],q_down(0),seconds=3.0,n=1)
print('retreat: code',code,'perr',perr.round(4))
" 2>&1 | tail -5; python3 tools/perception/cam_snap.py agentview >/dev/null; python3 tools/perception/cam_snap.py frontview >/dev/null

# openrua op 30
cd /workspace; python3 scene.py birdview >/dev/null & python3 scene.py agentview > /dev/null & wait; python3 -c "
import numpy as np, cv2
for cam in ['birdview','agentview']:
    xyz=np.load(f'{cam}_xyz.npy'); bgr=np.load(f'{cam}_bgr.npy')
    z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
    m=(z>0.434)&(z<0.48)&(x>-0.12)&(x<0.06)&(y>0.02)&(y<0.16)
    p=xyz[m]
    print(cam,'pudding pts',m.sum(),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'ztop',p[:,2].max().round(3))
    top=p[p[:,2]>0.455][:,:2]
    c=top.mean(0); u,s,vt=np.linalg.svd(top-c)
    print('  top-face center',c.round(3),'principal axis',vt[0].round(3),'yaw deg',round(np.degrees(np.arctan2(vt[0][1],vt[0][0])),1),'extent along axes',(2*np.sqrt(s**2/len(top))*np.sqrt(3)).round(3))
    ys,xs=np.where(m); print('  px',xs.min(),xs.max(),ys.min(),ys.max())
"


# openrua op 31
cd /workspace; python3 -c "
import numpy as np, cv2
b=np.load('birdview_bgr.npy'); a=np.load('agentview_bgr.npy')
cv2.imwrite('zoom_bird.png', cv2.resize(b[255:305,305:375], None, fx=8, fy=8, interpolation=cv2.INTER_NEAREST))
cv2.imwrite('zoom_agent.png', cv2.resize(a[270:350,340:430], None, fx=6, fy=6, interpolation=cv2.INTER_NEAREST))
"

# openrua op 32
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
code,jerr,perr=r.move_tcp([0.0,-0.30,0.72],q_down(0),seconds=4.0,n=2)
print('park: code',code,'perr',perr.round(4))
" 2>&1 | tail -1; python3 scene.py birdview >/dev/null; python3 -c "
import numpy as np, cv2
xyz=np.load('birdview_xyz.npy'); bgr=np.load('birdview_bgr.npy')
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.434)&(z<0.48)&(x>-0.12)&(x<0.06)&(y>0.02)&(y<0.16)
p=xyz[m]
print('pudding pts',m.sum(),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'ztop',p[:,2].max().round(3))
top=p[p[:,2]>0.455][:,:2]
c=top.mean(0); u,s,vt=np.linalg.svd(top-c)
ax=vt[0]; 
print('top-face center',c.round(4),'yaw deg',round(np.degrees(np.arctan2(ax[1],ax[0])),1))
# extents along principal axes
proj=(top-c)@vt.T; print('extent along',(proj.max(0)-proj.min(0)).round(3))
# minAreaRect in pixel space
ys,xs=np.where(m); rect=cv2.minAreaRect(np.c_[xs,ys].astype(np.float32)); print('minAreaRect px',rect)
cv2.imwrite('zoom_bird.png', cv2.resize(bgr[255:305,305:375], None, fx=8, fy=8, interpolation=cv2.INTER_NEAREST))
"

# openrua op 33
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
P=[-0.029,0.0875]
code,jerr,perr=r.move_tcp(P+[0.60],q_down(0),seconds=4.0,n=2); print('pregrasp: code',code,'perr',perr.round(4))
if abs(perr).max()>0.01:
    code,jerr,perr=r.move_tcp(P+[0.60],q_down(0),seconds=2.0,n=1); print('resend: code',code,'perr',perr.round(4))
print('fingers open', r.fingers())
code,jerr,perr=r.move_tcp(P+[0.44],q_down(0),seconds=3.0,n=2); print('descend: code',code,'perr',perr.round(4))
if abs(perr).max()>0.005:
    code,jerr,perr=r.move_tcp(P+[0.44],q_down(0),seconds=2.0,n=1); print('resend: code',code,'perr',perr.round(4))
print('TCP',r.fk()[0].round(4))
" 2>&1 | tail -6; python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 34
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
print('close ->', r.close())
code,jerr,perr=r.move_tcp([-0.029,0.0875,0.60],q_down(0),seconds=3.0,n=2); print('lift: code',code,'perr',perr.round(4),'fingers',r.fingers())
T=[0.145,0.18]
code,jerr,perr=r.move_tcp(T+[0.60],q_down(0),seconds=5.0,n=3); print('transit: code',code,'perr',perr.round(4))
for i in range(2):
    if abs(perr).max()<0.005: break
    code,jerr,perr=r.move_tcp(T+[0.60],q_down(0),seconds=2.5,n=1); print('resend: code',code,'perr',perr.round(4))
print('fingers',r.fingers(),'TCP',r.fk()[0].round(4))
" 2>&1 | tail -6; python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 35
cd /workspace; python3 -u -c "
from robot import *
r=Robot()
T=[0.145,0.18]
code,jerr,perr=r.move_tcp(T+[0.45],q_down(0),seconds=3.0,n=2); print('lower: code',code,'perr',perr.round(4))
if abs(perr).max()>0.005:
    code,jerr,perr=r.move_tcp(T+[0.45],q_down(0),seconds=2.0,n=1); print('resend: code',code,'perr',perr.round(4))
print('TCP',r.fk()[0].round(4))
print('open ->', r.open())
code,jerr,perr=r.move_tcp(T+[0.65],q_down(0),seconds=3.0,n=1); print('retreat: code',code,'perr',perr.round(4))
code,jerr,perr=r.move_tcp([-0.10,-0.35,0.75],q_down(0),seconds=4.0,n=2); print('park: code',code,'perr',perr.round(4))
" 2>&1 | tail -6; for c in agentview birdview frontview sideview; do python3 tools/perception/cam_snap.py $c >/dev/null; done; python3 scene.py birdview > /dev/null

# openrua op 36
cd /workspace; python3 -c "
import numpy as np
xyz=np.load('birdview_xyz.npy'); z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
# plate: low ring near (0.145,0)
m=(z>0.434)&(z<0.47)&(abs(x-0.145)<0.12)&(abs(y)<0.10); p=xyz[m]
print('plate+mugbase region: x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3))
# red mug rim: z ~0.575+0.03 (sitting on plate surface ~0.44-0.457)
m=(z>0.56)&(z<0.64)&(abs(x-0.145)<0.12)&(abs(y)<0.12); p=xyz[m][:,:2]
A=np.c_[2*p,np.ones(len(p))]; b=(p**2).sum(1); c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('red mug rim: n',len(p),'center',c[:2].round(3),'radius',r.round(3),'rim z',xyz[m][:,2].max().round(3))
# pudding
m=(z>0.434)&(z<0.48)&(abs(x-0.145)<0.12)&(y>0.10)&(y<0.30); p=xyz[m]
print('pudding: n',m.sum(),'center',p[:,:2].mean(0).round(3),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'ztop',p[:,2].max().round(3))
"
