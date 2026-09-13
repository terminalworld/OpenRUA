#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace; find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ----; timeout 20 ros2 node list

# openrua op 4
mkdir -p snaps && for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40; echo ----; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child" | head; echo ----; timeout 10 ros2 topic echo /agentview/color/camera_info --once

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a camera's depth frame as a world-frame point cloud + color image.

Usage: python3 scene.py <camera>
Saves snaps/<camera>_pts.npy (H,W,3 world xyz) and snaps/<camera>.png,
and prints the world->panda_link0 transform.
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


def tf_mat(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer()
    TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "bgr8")
    import time
    end = time.time() + 15
    frame = f"{cam}_optical_frame"
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform("world", frame, rclpy.time.Time()) and \
           buf.can_transform("world", "panda_link0", rclpy.time.Time()):
            break
    T = tf_mat(buf.lookup_transform("world", frame, rclpy.time.Time()))
    Tb = tf_mat(buf.lookup_transform("world", "panda_link0", rclpy.time.Time()))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth, np.ones_like(depth)], -1)
    Pw = P @ T.T
    np.save(f"snaps/{cam}_pts.npy", Pw[..., :3])
    cv2.imwrite(f"snaps/{cam}.png", color)
    print("cam T:\n", np.round(T, 4))
    print("world->panda_link0:\n", np.round(Tb, 4))
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 scene.py birdview && timeout 120 python3 scene.py agentview

# openrua op 8
python3 -c "
import numpy as np, cv2
P=np.load('snaps/birdview_pts.npy'); img=cv2.imread('snaps/birdview.png')
z=P[...,2]
# table height: mode of z in table region
zs=z[np.isfinite(z)]
h,e=np.histogram(zs,bins=200); print('top z bins', [(round(e[i],3),h[i]) for i in np.argsort(h)[-6:]])
# points above table
table=np.median(z[200:330,250:400]); print('table z', table)
mask=(z>table+0.01)&(z<table+0.4)
# exclude robot: robot base at x=-0.51.. we look for objects with x>-0.35 roughly
X=P[...,0];Y=P[...,1]
mask &= (X>-0.35)
ys,xs=np.nonzero(mask)
print('n',len(ys))
# connected components
m8=mask.astype(np.uint8); n,lab,stats,cent=cv2.connectedComponentsWithStats(m8)
for i in range(1,n):
    if stats[i,4]<10: continue
    sel=lab==i
    print(i,'px',stats[i,4],'centroid px',cent[i].round(1),'world x',X[sel].mean().round(3),'y',Y[sel].mean().round(3),'zmax',z[sel].max().round(3),'zmin',z[sel].min().round(3), 'xrange',X[sel].min().round(3),X[sel].max().round(3),'yrange',Y[sel].min().round(3),Y[sel].max().round(3))
"


# openrua op 9
python3 -c "
import numpy as np, cv2
P=np.load('snaps/agentview_pts.npy')
X,Y,Z=P[...,0],P[...,1],P[...,2]
# plate region in agentview image roughly px (260-380, 340-430)
for name,(u0,u1,v0,v1) in {'plate':(270,370,350,420),'pudding':(305,335,210,250),'whitemug':(180,240,190,250),'table_near_plate':(150,250,380,450),'table_far':(400,500,300,330)}.items():
    s=(slice(v0,v1),slice(u0,u1))
    print(name,'x',np.nanmin(X[s]).round(3),np.nanmax(X[s]).round(3),'y',np.nanmin(Y[s]).round(3),np.nanmax(Y[s]).round(3),'z',np.nanmin(Z[s]).round(3),np.nanmax(Z[s]).round(3),'zmed',np.nanmedian(Z[s]).round(3))
"


# openrua op 10
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position; timeout 100 python3 scene.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
P=np.load('snaps/birdview_pts.npy'); X,Y,z=P[...,0],P[...,1],P[...,2]
table=np.median(z[200:330,250:400]); print('table z', table)
mask=(z>table+0.01)&(z<table+0.4)&(X>-0.35)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<10: continue
    sel=lab==i
    print(i,'px',stats[i,4],'world x',X[sel].mean().round(3),'y',Y[sel].mean().round(3),'zmax',z[sel].max().round(3),'zmin',z[sel].min().round(3), 'xr',X[sel].min().round(3),X[sel].max().round(3),'yr',Y[sel].min().round(3),Y[sel].max().round(3))
"

# openrua op 11
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Robot helper: FK / IK / trajectory / gripper against machine.yaml ports.

Positions given to the CLI are in the WORLD frame; converted to the arm
base (planning frame) internally using the world->panda_link0 offset.

  python3 rob.py fk                         # hand + tcp pose (world)
  python3 rob.py js                         # joint dict
  python3 rob.py goto X Y Z [yaw_deg] [secs] # TCP to world pose, top-down
  python3 rob.py joints p1,...,p7 [secs]
  python3 rob.py grip open|close
"""
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
LIMITS = FJT["limits_rad"]
TCP = M["hand"]["tcp_offset_m"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # from TF world->panda_link0


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def topdown_R(yaw_deg):
    """Hand Z pointing down, hand X rotated by yaw about world Z."""
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # flip: Z down
    return Rz @ Rx


class Rob:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.update(zip(m.name, m.position)), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js.clear()
        end = time.time() + 10
        while not all(j in self.js for j in JOINTS) and time.time() < end:
            self.spin()
        return [self.js[j] for j in JOINTS]

    def fingers(self):
        self.joints()
        return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")

    def _seed(self, q=None):
        q = q if q is not None else self.joints()
        s = JointState()
        s.name = list(JOINTS)
        s.position = [float(v) for v in q]
        return s

    def fk(self, q=None, link="panda_hand"):
        self.fk_cli.wait_for_service(10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        R = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, R

    def fk_world(self, q=None):
        pos, R = self.fk(q)
        tcp = pos + TCP * R[:, 2]
        return pos + BASE_IN_WORLD, tcp + BASE_IN_WORLD, R

    def ik(self, tcp_world, R, seed=None, tries=1):
        """IK for TCP at world position with hand rotation R. Returns joints or None."""
        pos = np.asarray(tcp_world) - BASE_IN_WORLD - TCP * R[:, 2]
        q = R_to_quat(R)
        self.ik_cli.wait_for_service(10)
        seed_q = seed if seed is not None else self.joints()
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state = self._seed(seed_q)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 1
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                out = [sol[j] for j in JOINTS]
                if all(lo <= v <= hi for v, (lo, hi) in zip(out, LIMITS)):
                    return out
            seed_q = [np.clip(v + np.random.uniform(-0.3, 0.3), lo, hi)
                      for v, (lo, hi) in zip(seed_q, LIMITS)]
        return None

    def move(self, q, secs=3.0, waypoints=None):
        """Execute trajectory to q (optionally via intermediate waypoints)."""
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = list(waypoints or []) + [q]
        n = len(pts)
        for i, p in enumerate(pts):
            t = secs * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in p])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        now = self.joints()
        err = float(np.max(np.abs(np.array(now) - np.array(q))))
        print(f"move: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, tcp_world, yaw_deg=0.0, secs=3.0, seed=None):
        R = topdown_R(yaw_deg)
        q = self.ik(tcp_world, R, seed=seed, tries=8)
        if q is None:
            print(f"IK FAILED for {tcp_world} yaw={yaw_deg}")
            return None
        self.move(q, secs)
        _, tcp, _ = self.fk_world()
        print(f"goto: tcp now {np.round(tcp, 4)} (target {np.round(tcp_world, 4)})")
        return q

    def grip(self, open_):
        self.gr.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"grip {'open' if open_ else 'close'}: reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f


def main():
    a = sys.argv[1:]
    r = Rob()
    if a[0] == "fk":
        hand, tcp, R = r.fk_world()
        print("hand world", np.round(hand, 4))
        print("tcp  world", np.round(tcp, 4))
        print("R\n", np.round(R, 3))
    elif a[0] == "js":
        print(dict(zip(JOINTS, np.round(r.joints(), 4))), r.fingers())
    elif a[0] == "goto":
        x, y, z = map(float, a[1:4])
        yaw = float(a[4]) if len(a) > 4 else 0.0
        secs = float(a[5]) if len(a) > 5 else 3.0
        r.goto(np.array([x, y, z]), yaw, secs)
    elif a[0] == "joints":
        q = [float(v) for v in a[1].split(",")]
        r.move(q, float(a[2]) if len(a) > 2 else 3.0)
    elif a[0] == "grip":
        r.grip(a[1] == "open")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 12
timeout 120 python3 rob.py fk

# openrua op 13
python3 -c "
import numpy as np
P=np.load('snaps/birdview_pts.npy'); X,Y,z=P[...,0],P[...,1],P[...,2]
# white mug rim: z>0.52 in region
sel=(z>0.52)&(z<0.6)&(X>-0.2)&(X<-0.05)&(Y<-0.05)&(Y>-0.25)
print('rim n',sel.sum(),'x',X[sel].min().round(3),X[sel].max().round(3),'y',Y[sel].min().round(3),Y[sel].max().round(3),'zmax',z[sel].max().round(3))
print('rim center', X[sel].mean().round(3), Y[sel].mean().round(3))
# lower part (handle) z 0.45..0.52
sel2=(z>0.44)&(z<0.52)&(X>-0.2)&(X<-0.05)&(Y<-0.05)&(Y>-0.25)
print('body/handle n',sel2.sum(),'x',X[sel2].min().round(3),X[sel2].max().round(3),'y',Y[sel2].min().round(3),Y[sel2].max().round(3))
# pudding
sel3=(z>0.44)&(X>-0.3)&(X<-0.15)&(abs(Y)<0.06)
print('pudding n',sel3.sum(),'x',X[sel3].min().round(3),X[sel3].max().round(3),'y',Y[sel3].min().round(3),Y[sel3].max().round(3),'z',z[sel3].min().round(3),z[sel3].max().round(3),'center',X[sel3].mean().round(3),Y[sel3].mean().round(3))
# plate
sel4=(z>0.435)&(z<0.47)&(X>0.05)&(X<0.25)&(abs(Y)<0.1)
print('plate n',sel4.sum(),'x',X[sel4].min().round(3),X[sel4].max().round(3),'y',Y[sel4].min().round(3),Y[sel4].max().round(3),'center',X[sel4].mean().round(3),Y[sel4].mean().round(3),'z',z[sel4].min().round(3),z[sel4].max().round(3))
"


# openrua op 14
timeout 300 python3 rob.py goto -0.123 -0.201 0.66 0 4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png

# openrua op 15
timeout 300 python3 -c "
import numpy as np, rob
r=rob.Rob()
hand,tcp,R=r.fk_world()
print('tcp',tcp)
q=r.ik(tcp,R,tries=1); print('ik current pose ->',q)
R0=rob.topdown_R(0); print(R0, rob.R_to_quat(R0))
q=r.ik(np.array([-0.2,-0.1,0.7]),R0,tries=3); print('ik test1', q)
q=r.ik(np.array([-0.123,-0.201,0.66]),R0,tries=3); print('ik test2', q)
"

# openrua op 16
timeout 300 python3 -c "
import numpy as np, rob, rclpy
from moveit_msgs.srv import GetPositionIK
r=rob.Rob()
def raw(pos_base,q,seed,timeout=2):
    req=GetPositionIK.Request(); req.ik_request.group_name='panda_arm'; req.ik_request.pose_stamped.header.frame_id=''
    p=req.ik_request.pose_stamped.pose; p.position.x,p.position.y,p.position.z=map(float,pos_base)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q)
    req.ik_request.robot_state.joint_state=r._seed(seed); req.ik_request.timeout.sec=timeout
    fut=r.ik_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60); res=fut.result()
    print('code',res.error_code.val, dict(zip(res.solution.joint_state.name,np.round(res.solution.joint_state.position,3))) if res.error_code.val==1 else '')
home=r.joints()
raw([0.31,-0.1,0.28+0.1034],[1,0,0,0],home)
raw([0.31,-0.1,0.38],[1,0,0,0],home,5)
raw([0.4,0.0,0.4],[1,0,0,0],home)
raw([0.3,0.0,0.5],[1,0,0,0],home)
# use seed with elbow more typical
seed=[0,-0.785,0,-2.356,0,1.571,0.785]
raw([0.31,-0.1,0.38],[1,0,0,0],seed)
raw([0.4,0.0,0.3],[1,0,0,0],seed)
"

# openrua op 17
timeout 300 python3 -c "
import numpy as np, rob, rclpy, time
from moveit_msgs.srv import GetPositionIK
r=rob.Rob()
def raw(pos_base,q,seed,timeout=2,wait=20):
    req=GetPositionIK.Request(); req.ik_request.group_name='panda_arm'; req.ik_request.pose_stamped.header.frame_id=''
    p=req.ik_request.pose_stamped.pose; p.position.x,p.position.y,p.position.z=map(float,pos_base)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q)
    req.ik_request.robot_state.joint_state=r._seed(seed); req.ik_request.timeout.sec=timeout
    t=time.time(); fut=r.ik_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=wait); res=fut.result()
    print(f'{time.time()-t:.1f}s', 'None' if res is None else ('code',res.error_code.val, np.round(res.solution.joint_state.position,3) if res.error_code.val==1 else ''))
home=r.joints(); print('home',np.round(home,3))
raw([0.4,0.0,0.4],[1,0,0,0],home)
raw([0.4,0.0,0.4],[1,0,0,0],home)
raw([0.3,0.0,0.5],[1,0,0,0],home)
"

# openrua op 18
timeout 120 python3 -c "
import numpy as np, rob
r=rob.Rob()
for l in ['panda_link0','panda_link1','panda_link4','panda_link8','panda_hand']:
    p,R=r.fk(link=l); print(l, np.round(p,4))
"; timeout 30 ros2 service call /get_urdf std_srvs/srv/Trigger 2>/dev/null | head -c 300; echo; timeout 30 ros2 service type /get_urdf

# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # from TF world->panda_link0", "new_string": "# Verified: FK(panda_link0) with empty frame_id returns (-0.51, 0, 0.42),\\n# i.e. the planner\'s model frame IS world on this machine -> no offset.\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
timeout 300 python3 -c "
import numpy as np, rob
r=rob.Rob()
hand,tcp,R=r.fk_world(); print('hand',np.round(hand,4),'tcp',np.round(tcp,4))
R0=rob.topdown_R(0)
for p in [[-0.123,-0.201,0.66],[-0.218,-0.004,0.6],[0.149,-0.043,0.6],[0.149,0.2,0.55]]:
    q=r.ik(np.array(p),R0,tries=3); print(p,'->',None if q is None else np.round(q,3))
"

# openrua op 21
timeout 300 python3 rob.py goto -0.123 -0.201 0.66 0 4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png

# openrua op 22
timeout 60 python3 rob.py js

# openrua op 23
timeout 300 python3 rob.py joints -0.146,-0.22,-0.335,-2.55,-0.1,2.34,-0.404 3 && timeout 60 python3 rob.py js

# openrua op 24
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def move(self, q, secs=3.0, waypoints=None):\\n        \\"\\"\\"Execute trajectory to q (optionally via intermediate waypoints).\\"\\"\\"\\n        self.fjt.wait_for_server(10)", "new_string": "    def move(self, q, secs=3.0, waypoints=None, retries=2):\\n        \\"\\"\\"Execute trajectory to q (optionally via intermediate waypoints).\\n        Controller lag can leave a joint short (error_code -5); resend.\\"\\"\\"\\n        code, err = self._move(q, secs, waypoints)\\n        while err > 0.02 and retries > 0:\\n            print(\\"  retrying (controller lag)\\")\\n            code, err = self._move(q, max(2.0, secs / 2), None)\\n            retries -= 1\\n        return code, err\\n\\n    def _move(self, q, secs=3.0, waypoints=None):\\n        self.fjt.wait_for_server(10)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png

# openrua op 26
timeout 60 python3 rob.py fk

# openrua op 27
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "            req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "            req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n            # the group\'s default tip is panda_link8, which is yawed 45 deg\\n            # from panda_hand; solve for the hand frame explicitly\\n            req.ik_request.ik_link_name = \\"panda_hand\\"\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 28
timeout 300 python3 -c "
import numpy as np, rob
r=rob.Rob()
R0=rob.topdown_R(0)
q=r.ik(np.array([-0.123,-0.201,0.66]),R0,tries=3); print('q',np.round(q,3))
hand,tcp,R=r.fk_world(q); print('fk tcp',np.round(tcp,4)); print(np.round(R,3))
"

# openrua op 29
timeout 300 python3 rob.py goto -0.123 -0.201 0.66 0 3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih2.png

# openrua op 30
timeout 300 python3 rob.py goto -0.123 -0.201 0.58 0 2 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih3.png && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent2.png

# openrua op 31
timeout 300 python3 -c "
import numpy as np, rob
r=rob.Rob()
r.goto(np.array([-0.123,-0.201,0.52]),0,2)
f=r.grip(False)
print('finger gap', f[0]-f[1])
"

# openrua op 32
timeout 300 python3 rob.py goto -0.123 -0.201 0.68 0 3 && timeout 60 python3 rob.py js && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent3.png

# openrua op 33
timeout 400 python3 -c "
import numpy as np, rob
r=rob.Rob()
r.goto(np.array([0.149,-0.043,0.68]),0,4)
r.goto(np.array([0.149,-0.043,0.565]),0,2.5)
" && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent4.png

# openrua op 34
timeout 400 python3 -c "
import numpy as np, rob
r=rob.Rob()
r.grip(True)
r.goto(np.array([0.149,-0.043,0.70]),0,3)
" && timeout 100 python3 scene.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent5.png && python3 -c "
import numpy as np, cv2
P=np.load('snaps/birdview_pts.npy'); X,Y,z=P[...,0],P[...,1],P[...,2]
sel=(z>0.50)&(z<0.6)&(X>0.0)&(X<0.3)&(abs(Y)<0.15)
print('mug rim n',sel.sum(),'x',X[sel].min().round(3),X[sel].max().round(3),'y',Y[sel].min().round(3),Y[sel].max().round(3),'zmax',z[sel].max().round(3),'center',X[sel].mean().round(3),Y[sel].mean().round(3))
sel4=(z>0.435)&(z<0.47)&(X>0.05)&(X<0.25)&(abs(Y)<0.1)
print('plate visible n',sel4.sum(),'x',X[sel4].min().round(3),X[sel4].max().round(3),'y',Y[sel4].min().round(3),Y[sel4].max().round(3))
"

# openrua op 35
timeout 400 python3 rob.py goto -0.218 -0.004 0.60 0 4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih4.png

# openrua op 36
timeout 400 python3 rob.py goto -0.218 -0.004 0.50 0 2.5 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih5.png

# openrua op 37
timeout 400 python3 -c "
import numpy as np, rob
r=rob.Rob()
r.goto(np.array([-0.218,-0.004,0.442]),0,2)
f=r.grip(False); print('finger gap', f[0]-f[1])
r.goto(np.array([-0.218,-0.004,0.60]),0,2.5)
print('fingers after lift', r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent6.png

# openrua op 38
timeout 500 python3 -c "
import numpy as np, rob
r=rob.Rob()
r.goto(np.array([-0.218,-0.004,0.70]),0,2.5)
r.goto(np.array([0.149,0.20,0.70]),0,4)
r.goto(np.array([0.149,0.20,0.50]),0,2.5)
print('fingers', r.fingers())
" && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent7.png

# openrua op 39
timeout 500 python3 -c "
import numpy as np, rob
r=rob.Rob()
r.goto(np.array([0.149,0.20,0.446]),0,2)
r.grip(True)
r.goto(np.array([0.149,0.20,0.62]),0,2.5)
# park the arm back near home, out of the cameras' way
r.move([0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854],4)
" && timeout 100 python3 scene.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent8.png

# openrua op 40
python3 -c "
import numpy as np
P=np.load('snaps/birdview_pts.npy'); X,Y,z=P[...,0],P[...,1],P[...,2]
table=0.425
def rep(name,sel):
    print(f'{name}: n={sel.sum()} x[{X[sel].min():.3f},{X[sel].max():.3f}] y[{Y[sel].min():.3f},{Y[sel].max():.3f}] z[{z[sel].min():.3f},{z[sel].max():.3f}] center=({X[sel].mean():.3f},{Y[sel].mean():.3f})')
rep('plate (visible ring)', (z>0.435)&(z<0.47)&(X>0.05)&(X<0.25)&(abs(Y)<0.1))
rep('white mug rim', (z>0.52)&(z<0.60)&(X>0.0)&(X<0.3)&(abs(Y)<0.15))
rep('pudding', (z>0.44)&(z<0.48)&(X>0.05)&(X<0.25)&(Y>0.1)&(Y<0.3))
rep('anything left at old mug spot', (z>table+0.01)&(X>-0.2)&(X<-0.05)&(Y<-0.08)&(Y>-0.25))
rep('anything left at old pudding spot', (z>table+0.01)&(z<0.5)&(X>-0.26)&(X<-0.17)&(abs(Y)<0.04))
"
