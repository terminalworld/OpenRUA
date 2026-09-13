#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace; echo ---; find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; echo ---; find /workspace/tools -type f; echo ---; ls -la /workspace/.ros

# openrua op 2
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 3
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 node list

# openrua op 5
timeout 60 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 60 ros2 topic echo /agentview/color/camera_info --once | head -30; timeout 30 ros2 topic echo /tf_static --once 2>&1 | head -80

# openrua op 7
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a camera's color, depth and world-frame point cloud helpers.

Usage: python3 scene.py <camera>
Saves <camera>_rgb.png, <camera>_depth.npy, <camera>_T.npy (4x4 world<-optical),
<camera>_K.npy. Then prints world coords for a grid of pixel probes on request.
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, typ, timeout=30.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
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
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    rgb = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    K = np.array(info.k).reshape(3, 3)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    cv2.imwrite(f"{cam}_rgb.png", rgb)
    np.save(f"{cam}_depth.npy", depth)
    np.save(f"{cam}_T.npy", T)
    np.save(f"{cam}_K.npy", K)
    print("T world<-optical:\n", np.round(T, 4))
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 scene.py agentview && timeout 120 python3 scene.py birdview

# openrua op 9
python3 -c "
import numpy as np, cv2
cam='agentview'
d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); K=np.load(f'{cam}_K.npy')
rgb=cv2.imread(f'{cam}_rgb.png')
H,W=d.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
z=d
pc=np.stack([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z,np.ones_like(z)],-1)
pw=pc@T.T
X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
# table height: mode of Z in lower half
zz=Z[300:,:].ravel(); zz=zz[np.isfinite(zz)]
hist,edges=np.histogram(zz,bins=200)
print('table z mode', edges[np.argmax(hist)])
def probe(uu,vv):
    print((uu,vv),'->',np.round(pw[vv,uu,:3],3), 'rgb',rgb[vv,uu][::-1])
# probe mug tops / plates from image
for p in [(185,195),(185,230),(335,165),(335,190),(425,200),(425,250),(85,305),(535,295),(180,265),(415,300),(325,220)]:
    probe(*p)
"


# openrua op 10
python3 -c "
import numpy as np, cv2
cam='birdview'
d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); K=np.load(f'{cam}_K.npy')
rgb=cv2.imread(f'{cam}_rgb.png')
H,W=d.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
z=d
pc=np.stack([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z,np.ones_like(z)],-1)
pw=pc@T.T
Z=pw[...,2]
print('K',K[0,0])
zz=Z.ravel(); zz=zz[np.isfinite(zz)]
hist,edges=np.histogram(zz,bins=300)
top=np.argsort(hist)[-5:]
print('z modes', [(round(edges[i],3),hist[i]) for i in top])
# objects: z between 0.44 and 0.7, exclude robot (near center column x)
for name,(u0,v0) in {'white mug':(272,265),'yw mug':(322,235),'red mug':(352,283),'lplate':(250,287),'rplate':(388,287)}.items():
    print(name, np.round(pw[v0,u0,:3],3), rgb[v0,u0][::-1])
"


# openrua op 11
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 60 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 12
timeout 120 python3 scene.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
cam='birdview'
d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); K=np.load(f'{cam}_K.npy')
rgb=cv2.imread(f'{cam}_rgb.png')
H,W=d.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
z=d
pc=np.stack([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z,np.ones_like(z)],-1)
pw=pc@T.T
Z=pw[...,2]
zz=Z.ravel(); zz=zz[np.isfinite(zz)]
hist,edges=np.histogram(zz,bins=300)
top=np.argsort(hist)[-5:]
print('z modes', [(round(edges[i],3),hist[i]) for i in top])
for name,(u0,v0) in {'white mug':(272,265),'yw mug':(322,235),'red mug':(352,283),'lplate':(250,287),'rplate':(388,287)}.items():
    print(name, np.round(pw[v0,u0,:3],3), rgb[v0,u0][::-1])
"


# openrua op 13
timeout 120 python3 scene.py agentview >/dev/null && python3 -c "
import numpy as np, cv2
def cloud(cam):
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); K=np.load(f'{cam}_K.npy')
    rgb=cv2.imread(f'{cam}_rgb.png')
    H,W=d.shape
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    pc=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)
    return (pc@T.T)[...,:3], rgb
pw,rgb=cloud('birdview')
X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
# objects above table, in table region, exclude robot body (x < -0.3 roughly? robot base location?)
mask=(Z>0.43)&(Z<0.7)&np.isfinite(Z)
# connected components
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    print(i,'px',stats[i,4],'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] cen(%.3f,%.3f)'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max(),X[m].mean(),Y[m].mean()), 'rgb',rgb[m].mean(0)[::-1].astype(int))
"


# openrua op 14
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -12; echo ---; timeout 10 ros2 service list | grep -i -E "fk|ik|state"

# openrua op 15
python3 -c "
import numpy as np, cv2
def cloud(cam):
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); K=np.load(f'{cam}_K.npy')
    rgb=cv2.imread(f'{cam}_rgb.png')
    H,W=d.shape
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    pc=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)
    return (pc@T.T)[...,:3], rgb
pw,rgb=cloud('birdview')
X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
mugs={'yw':((-0.27,-0.17),(-0.07,0.09)),'white':((-0.15,-0.03),(-0.25,-0.10)),'red':((-0.09,0.02),(0.08,0.19))}
for n,((x0,x1),(y0,y1)) in mugs.items():
    box=(X>x0)&(X<x1)&(Y>y0)&(Y<y1)&np.isfinite(Z)
    floor=box&(Z>0.435)&(Z<0.47)
    rim=box&(Z>Z[box].max()-0.015)
    print(n,'floor cen (%.3f,%.3f) z %.3f n=%d'%(X[floor].mean(),Y[floor].mean(),Z[floor].mean(),floor.sum()),
      'floor ext x %.3f y %.3f'%(X[floor].max()-X[floor].min(),Y[floor].max()-Y[floor].min()),
      'rim z %.3f cen (%.3f,%.3f) n=%d'%(Z[rim].mean(),X[rim].mean(),Y[rim].mean(),rim.sum()))
    # rim circle fit
    xs,ys=X[rim],Y[rim]
    A=np.c_[2*xs,2*ys,np.ones_like(xs)]; b=xs**2+ys**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print('   rim circle cen (%.3f,%.3f) r %.3f'%(c[0],c[1],r))
for n,((x0,x1),(y0,y1)) in {'lplate':((-0.08,0.10),(-0.40,-0.20)),'rplate':((-0.10,0.08),(0.20,0.37))}.items():
    box=(X>x0)&(X<x1)&(Y>y0)&(Y<y1)&(Z>0.435)&np.isfinite(Z)
    print(n,'cen (%.3f,%.3f) ztop %.3f ext x %.3f y %.3f'%(X[box].mean(),Y[box].mean(),Z[box].max(),X[box].max()-X[box].min(),Y[box].max()-Y[box].min()))
"


# openrua op 16
python3 -c "
import numpy as np, cv2
def cloud(cam):
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); K=np.load(f'{cam}_K.npy')
    rgb=cv2.imread(f'{cam}_rgb.png')
    H,W=d.shape
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    pc=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)
    return (pc@T.T)[...,:3], rgb
pw,rgb=cloud('agentview')
X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
for n,((x0,x1),(y0,y1)) in {'yw':((-0.29,-0.15),(-0.07,0.09)),'white':((-0.15,-0.03),(-0.25,-0.10)),'red':((-0.09,0.02),(0.08,0.19))}.items():
    box=(X>x0)&(X<x1)&(Y>y0)&(Y<y1)&np.isfinite(Z)&(Z>0.43)&(Z<0.7)
    zs=Z[box]
    print(n,'zmax %.3f  z99 %.3f  xrange [%.3f,%.3f] yrange [%.3f,%.3f]'%(zs.max(),np.percentile(zs,99),X[box].min(),X[box].max(),Y[box].min(),Y[box].max()))
    top=box&(Z>np.percentile(zs,97))
    print('   top pixels cen (%.3f,%.3f)'%(X[top].mean(),Y[top].mean()))
"


# openrua op 17
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small control library: IK (hand frame, base-frame poses), FK, trajectories,
gripper, joint state. World<->base offset from TF (panda_link0 in world)."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # panda_link0 in world (tf2_echo)
TCP = float(M["hand"]["tcp_offset_m"])
DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z -> world -z, hand y -> world -y (finger axis = world y)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down(yaw):
    """Quaternion: DOWN orientation then rotated by yaw about world z."""
    # q = Rz(yaw) * Rx(pi)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    # Rz(yaw) = (0,0,sy,cy); Rx(pi) = (1,0,0,0); product (w1w2 - v1.v2, ...)
    w = -0.0 * 0 + cy * 0 - 0  # placeholder, compute properly below
    q1 = np.array([cy, 0, 0, sy])  # w,x,y,z
    q2 = np.array([0, 1, 0, 0])
    w = q1[0] * q2[0] - np.dot(q1[1:], q2[1:])
    v = q1[0] * q2[1:] + q2[0] * q1[1:] + np.cross(q1[1:], q2[1:])
    return (v[0], v[1], v[2], w)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, M["sensors"][1]["port"], self._on_wr, 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.spin(0.5)

    def _on_js(self, m):
        self._js["m"] = m

    def _on_wr(self, m):
        self._wr["m"] = m

    def spin(self, t):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self._js.clear()
        while "m" not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr.clear()
        end = time.time() + 3
        while "m" not in self._wr and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if "m" not in self._wr:
            return None
        f = self._wr["m"].wrench.force
        return np.array([f.x, f.y, f.z])

    # ---- kinematics ----
    def ik_hand_world(self, pos_w, quat, seed=None):
        """IK for the panda_hand frame at world position pos_w."""
        pos_b = np.asarray(pos_w) - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        s = JointState()
        s.name = list(JOINTS)
        s.position = [float(v) for v in (seed if seed is not None else self.arm_q())]
        req.ik_request.robot_state.joint_state = s
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"IK failed {None if res is None else res.error_code.val} for {pos_w}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def ik_tcp_world(self, tcp_w, quat, seed=None):
        R = quat_R(*quat)
        hand = np.asarray(tcp_w) - TCP * R[:, 2]
        return self.ik_hand_world(hand, quat, seed)

    def fk_world(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        s = JointState(); s.name = list(JOINTS)
        s.position = [float(v) for v in (q if q is not None else self.arm_q())]
        req.robot_state.joint_state = s
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError("FK failed")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, quat

    def tcp_world(self, q=None):
        pos, quat = self.fk_world(q)
        R = quat_R(*quat)
        return pos + TCP * R[:, 2], quat

    # ---- motion ----
    def traj(self, qs, times):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for q, t in zip(qs, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(qs[-1])).max()
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code

    def move_tcp_line(self, tcp_from, tcp_to, quat, n=4, speed=0.15, seed=None, min_t=1.0):
        """Straight TCP line via n IK waypoints, one multi-point trajectory."""
        tcp_from, tcp_to = np.asarray(tcp_from), np.asarray(tcp_to)
        seed = list(seed if seed is not None else self.arm_q())
        qs, ts = [], []
        dist = np.linalg.norm(tcp_to - tcp_from)
        total = max(min_t, dist / speed)
        for i in range(1, n + 1):
            a = i / n
            p = tcp_from + a * (tcp_to - tcp_from)
            q = self.ik_tcp_world(p, quat, seed)
            seed = q
            qs.append(q); ts.append(total * a)
        code = self.traj(qs, ts)
        pos, _ = self.tcp_world()
        print(f"  tcp now {np.round(pos, 4)} target {np.round(tcp_to, 4)} err {np.linalg.norm(pos - tcp_to):.4f}")
        return code

    def move_tcp(self, tcp_to, quat, **kw):
        pos, _ = self.tcp_world()
        return self.move_tcp_line(pos, tcp_to, quat, **kw)

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        self.spin(0.3)
        f = self.fingers()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
OPENRUA_EOF

# openrua op 18
timeout 300 python3 -c "
from ctl import *
c=Ctl()
q=c.arm_q(); print('q',np.round(q,3))
pos,quat=c.fk_world(); print('hand world',np.round(pos,4),np.round(quat,3))
tcp,_=c.tcp_world(); print('tcp world',np.round(tcp,4))
print('yaw_down(0)',np.round(yaw_down(0),3))
sol=c.ik_hand_world(pos,quat,q); print('ik roundtrip',np.round(sol,3))
# test IK for grasp poses
for name,p in {'white_pre':(-0.088,-0.233,0.65),'white_grasp':(-0.088,-0.233,0.52),'yw_pre':(-0.222,0.069,0.65),'yw_grasp':(-0.222,0.069,0.52),'lplate':(0.017,-0.260,0.56),'rplate':(-0.006,0.327,0.56)}.items():
    try:
        s=c.ik_tcp_world(p,DOWN,q); t,_=c.tcp_world(s); print(name,'ok',np.round(s,3),'fk tcp',np.round(t,4))
    except Exception as e: print(name,'FAIL',e)
print('wrench',c.wrench()); print('fingers',c.fingers())
c.close()
"

# openrua op 19
sed -i 's/^BASE_IN_WORLD = .*/BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # MoveIt model frame == world (verified via FK vs TF)/' ctl.py && timeout 300 python3 -c "
from ctl import *
c=Ctl()
q=c.arm_q()
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand']; s=JointState(); s.name=list(JOINTS); s.position=[float(v) for v in q]; req.robot_state.joint_state=s
fut=c.fk.call_async(req); rclpy.spin_until_future_complete(c.node,fut,timeout_sec=30); print('fk frame:',fut.result().pose_stamped[0].header.frame_id)
pos,quat=c.fk_world(); print('hand world',np.round(pos,4),np.round(quat,3))
tcp,_=c.tcp_world(); print('tcp world',np.round(tcp,4))
for name,p in {'white_pre':(-0.088,-0.233,0.65),'white_grasp':(-0.088,-0.233,0.52),'yw_pre':(-0.222,0.069,0.65),'yw_grasp':(-0.222,0.069,0.52),'lplate':(0.017,-0.260,0.56),'rplate':(-0.006,0.327,0.56)}.items():
    try:
        s=c.ik_tcp_world(p,DOWN,q); t,qq=c.tcp_world(s); print(name,'ok',np.round(s,3),'fk tcp',np.round(t,4),np.round(qq,3))
    except Exception as e: print(name,'FAIL',e)
c.close()
"

# openrua op 20
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Rim-grasp a mug and place it on a plate.

Usage: python3 pick_place.py <mug_x> <mug_y> <rim_z> <side:+1|-1> <plate_x> <plate_y>
side: which y-side of the mug to grasp (+1 = +y side, -1 = -y side; pick the
side away from the handle). Grasp point is on the wall at radius WALL_R.
"""
import sys
import numpy as np
from ctl import Ctl, DOWN

WALL_R = 0.041       # radius of the mug wall centre
GRASP_BELOW_RIM = 0.03
TABLE_Z = 0.422
PLATE_TOP = 0.456
CARRY_Z = 0.70
OPEN, CLOSED = 0.04, 0.0


def main():
    mx, my, rim_z, side, px, py = map(float, sys.argv[1:7])
    mug_h = rim_z - TABLE_Z
    grasp = np.array([mx, my + side * WALL_R, rim_z - GRASP_BELOW_RIM])
    pre = grasp.copy(); pre[2] = CARRY_Z
    # when placed, TCP sits (mug_h - GRASP_BELOW_RIM) above the mug bottom
    place = np.array([px, py + side * WALL_R, PLATE_TOP + (mug_h - GRASP_BELOW_RIM) + 0.008])
    place_pre = place.copy(); place_pre[2] = CARRY_Z
    print("grasp", grasp, "place", place)

    c = Ctl()
    print("[0] open gripper"); c.gripper(OPEN)
    print("[1] to pre-grasp", pre); c.move_tcp(pre, DOWN, n=4, speed=0.2)
    w0 = c.wrench(); print("  wrench", w0)
    print("[2] descend", grasp); c.move_tcp(grasp, DOWN, n=4, speed=0.08)
    w1 = c.wrench(); print("  wrench", w1, "delta", None if w0 is None else np.round(w1 - w0, 2))
    print("[3] close"); f = c.gripper(CLOSED)
    gap = f[0] - f[1]
    print(f"  finger gap {gap:.4f} (0 => closed on air)")
    if gap < 0.002:
        print("GRASP FAILED: nothing held"); c.gripper(OPEN); c.move_tcp(pre, DOWN); c.close(); sys.exit(2)
    print("[4] lift", pre); c.move_tcp(pre, DOWN, n=4, speed=0.08)
    f = c.fingers(); print(f"  gap after lift {f[0]-f[1]:.4f}")
    print("[5] transport", place_pre); c.move_tcp(place_pre, DOWN, n=6, speed=0.15)
    f = c.fingers(); print(f"  gap after transport {f[0]-f[1]:.4f}")
    print("[6] lower", place); c.move_tcp(place, DOWN, n=4, speed=0.08)
    w2 = c.wrench(); print("  wrench", w2)
    print("[7] release"); c.gripper(OPEN)
    print("[8] retreat", place_pre); c.move_tcp(place_pre, DOWN, n=3, speed=0.1)
    print("DONE")
    c.close()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 21
nohup python3 -u pick_place.py -0.088 -0.192 0.550 -1 0.017 -0.301 > white.log 2>&1 &
sleep 45; cat /workspace/white.log

# openrua op 22
sleep 10; tail -3 /workspace/white.log; timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 23
timeout 120 python3 scene.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
def cloud(cam):
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); K=np.load(f'{cam}_K.npy')
    rgb=cv2.imread(f'{cam}_rgb.png')
    H,W=d.shape
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    pc=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)
    return (pc@T.T)[...,:3], rgb
pw,rgb=cloud('birdview')
X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
box=(X>-0.1)&(X<0.12)&(Y>-0.42)&(Y<-0.2)&np.isfinite(Z)
floor=box&(Z>0.46)&(Z<0.49)
print('white mug floor cen (%.3f,%.3f) z %.3f n=%d'%(X[floor].mean(),Y[floor].mean(),Z[floor].mean(),floor.sum()))
rim=box&(Z>0.55)
print('rim z %.3f n=%d'%(Z[rim].mean(),rim.sum()))
# yw mug recheck
box=(X>-0.29)&(X<-0.15)&(Y>-0.07)&(Y<0.09)&np.isfinite(Z)&(Z<0.7)
floor=box&(Z>0.435)&(Z<0.47)
print('yw floor cen (%.3f,%.3f) n=%d  ext x %.3f y %.3f'%(X[floor].mean(),Y[floor].mean(),floor.sum(),X[floor].max()-X[floor].min(),Y[floor].max()-Y[floor].min()))
print('yw xrange [%.3f,%.3f] yrange [%.3f,%.3f] zmax %.3f'%(X[box].min(),X[box].max(),Y[box].min(),Y[box].max(),Z[box].max()))
"

# openrua op 24
timeout 120 python3 scene.py agentview >/dev/null && python3 -c "
import numpy as np, cv2
def cloud(cam):
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); K=np.load(f'{cam}_K.npy')
    rgb=cv2.imread(f'{cam}_rgb.png')
    H,W=d.shape
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    pc=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)
    return (pc@T.T)[...,:3], rgb
pw,rgb=cloud('agentview')
X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
box=(X>-0.1)&(X<0.12)&(Y>-0.42)&(Y<-0.2)&np.isfinite(Z)&(Z>0.46)&(Z<0.62)
print('white mug xrange [%.3f,%.3f] yrange [%.3f,%.3f] zmax %.3f'%(X[box].min(),X[box].max(),Y[box].min(),Y[box].max(),Z[box].max()))
top=box&(Z>Z[box].max()-0.01); print('rim z %.3f cen (%.3f,%.3f)'%(Z[top].mean(),X[top].mean(),Y[top].mean()))
plate=(X>-0.1)&(X<0.12)&(Y>-0.42)&(Y<-0.2)&np.isfinite(Z)&(Z>0.435)&(Z<0.46)
print('plate visible xrange [%.3f,%.3f] yrange [%.3f,%.3f]'%(X[plate].min(),X[plate].max(),Y[plate].min(),Y[plate].max()))
"

# openrua op 25
sed -i 's/c.move_tcp(pre, DOWN, n=4, speed=0.2)/c.move_tcp(pre, DOWN, n=6, speed=0.1)/; s/c.move_tcp(place_pre, DOWN, n=6, speed=0.15)/c.move_tcp(place_pre, DOWN, n=8, speed=0.08)/' pick_place.py && grep -n "speed" pick_place.py && nohup python3 -u pick_place.py -0.223 0.027 0.542 1 -0.006 0.286 > yw.log 2>&1 &
sleep 60; cat /workspace/yw.log

# openrua op 26
sleep 5; tail -2 yw.log; timeout 120 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 120 python3 tools/perception/cam_snap.py birdview >/dev/null; echo ok

# openrua op 27
python3 - <<'EOF'
import re
s=open('ctl.py').read()
s=s.replace("""            qs.append(q); ts.append(total * a)
        code = self.traj(qs, ts)""","""            qs.append(q); ts.append(total * a)
        qs.append(qs[-1]); ts.append(total + 1.0)  # hold point: let the controller settle
        code = self.traj(qs, ts)""")
s=s.replace('''def yaw_down(yaw):''','''def qmul(a, b):
    """quaternion product, (x,y,z,w) convention"""
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def pitch_down(theta):
    """DOWN orientation tilted about world y by theta: hand approach axis
    becomes (-sin t, 0, -cos t), i.e. leaning in from +x for t>0."""
    ry = (0.0, np.sin(theta / 2), 0.0, np.cos(theta / 2))
    return qmul(ry, DOWN)


def yaw_down(yaw):''')
open('ctl.py','w').write(s)
EOF
timeout 300 python3 -c "
from ctl import *
c=Ctl()
q=c.arm_q()
lim=np.array(FJT['limits_rad'])
for th in [0,10,20,30,40]:
    quat=pitch_down(np.radians(th))
    R=quat_R(*quat); print('theta',th,'approach axis',np.round(R[:,2],3),'finger axis',np.round(R[:,1],3))
    for name,p in {'pre':(-0.223,0.068,0.70),'grasp':(-0.223,0.068,0.512)}.items():
        try:
            s=c.ik_tcp_world(p,quat,q); t,_=c.tcp_world(s)
            marg=np.minimum(np.array(s)-lim[:,0],lim[:,1]-np.array(s))
            print('  ',name,'q',np.round(s,2),'min margin j%d %.3f'%(marg.argmin()+1,marg.min()),'fk err %.4f'%np.linalg.norm(t-p))
        except Exception as e: print('  ',name,'FAIL',e)
c.close()
"

# openrua op 28
timeout 120 python3 scene.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
def cloud(cam):
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); K=np.load(f'{cam}_K.npy')
    rgb=cv2.imread(f'{cam}_rgb.png')
    H,W=d.shape
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    pc=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)
    return (pc@T.T)[...,:3], rgb
pw,rgb=cloud('birdview')
X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
box=(X>-0.30)&(X<-0.14)&(Y>-0.08)&(Y<0.10)&np.isfinite(Z)&(Z<0.7)&(Z>0.43)
floor=box&(Z>0.435)&(Z<0.47)
print('yw floor cen (%.3f,%.3f) n=%d ext x %.3f y %.3f'%(X[floor].mean(),Y[floor].mean(),floor.sum(),X[floor].max()-X[floor].min(),Y[floor].max()-Y[floor].min()))
print('yw xrange [%.3f,%.3f] yrange [%.3f,%.3f] zmax %.3f'%(X[box].min(),X[box].max(),Y[box].min(),Y[box].max(),Z[box].max()))
rim=box&(Z>0.53); xs,ys=X[rim],Y[rim]
A=np.c_[2*xs,2*ys,np.ones_like(xs)]; b=xs**2+ys**2
cc=np.linalg.lstsq(A,b,rcond=None)[0]; print('rim circle (%.3f,%.3f) r %.3f n=%d'%(cc[0],cc[1],np.sqrt(cc[2]+cc[0]**2+cc[1]**2),rim.sum()))
"

# openrua op 29
python3 -c "
import cv2; im=cv2.imread('birdview_rgb.png'); crop=im[200:280,270:370]; cv2.imwrite('crop.png',cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))"

# openrua op 30
mkdir -p "$(dirname /workspace/pick_place_tilt.py)"
cat > /workspace/pick_place_tilt.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Rim-grasp a mug near the base with a pitched (leaning-in) hand, then place
it flat on a plate.  Same args as pick_place.py plus tilt degrees.

Usage: python3 pick_place_tilt.py <mug_x> <mug_y> <rim_z> <side> <plate_x> <plate_y> <tilt_deg>
"""
import sys
import numpy as np
from ctl import Ctl, DOWN, pitch_down

WALL_R = 0.041
GRASP_BELOW_RIM = 0.03
TABLE_Z = 0.422
PLATE_TOP = 0.456
CARRY_Z = 0.70
OPEN, CLOSED = 0.04, 0.0


def main():
    mx, my, rim_z, side, px, py, tilt = map(float, sys.argv[1:8])
    TILT = pitch_down(np.radians(tilt))
    mug_h = rim_z - TABLE_Z
    grasp = np.array([mx, my + side * WALL_R, rim_z - GRASP_BELOW_RIM])
    pre = grasp.copy(); pre[2] = CARRY_Z
    place = np.array([px, py + side * WALL_R, PLATE_TOP + (mug_h - GRASP_BELOW_RIM) + 0.008])
    place_pre = place.copy(); place_pre[2] = CARRY_Z
    print("grasp", grasp, "place", place, "tilt", tilt)

    c = Ctl()
    print("[0] open gripper"); c.gripper(OPEN)
    print("[1] to pre-grasp (tilted)", pre); c.move_tcp(pre, TILT, n=6, speed=0.1)
    w0 = c.wrench(); print("  wrench", w0)
    print("[2] descend", grasp); c.move_tcp(grasp, TILT, n=5, speed=0.06)
    w1 = c.wrench(); print("  wrench", w1, "delta", None if w0 is None else np.round(w1 - w0, 2))
    pos, _ = c.tcp_world()
    if np.linalg.norm(pos - grasp) > 0.01:
        print("  retrying descent for accuracy"); c.move_tcp(grasp, TILT, n=2, speed=0.03)
    print("[3] close"); f = c.gripper(CLOSED)
    gap = f[0] - f[1]
    print(f"  finger gap {gap:.4f} (0 => closed on air)")
    if gap < 0.002:
        print("GRASP FAILED: nothing held"); c.gripper(OPEN); c.move_tcp(pre, TILT); c.close(); sys.exit(2)
    print("[4] lift", pre); c.move_tcp(pre, TILT, n=4, speed=0.06)
    f = c.fingers(); print(f"  gap after lift {f[0]-f[1]:.4f}")
    if f[0] - f[1] < 0.002:
        print("LOST OBJECT during lift"); c.close(); sys.exit(3)
    print("[5] un-tilt to vertical at carry height")
    c.move_tcp(pre, DOWN, n=4, speed=0.05, min_t=2.0)
    f = c.fingers(); print(f"  gap after untilt {f[0]-f[1]:.4f}")
    print("[6] transport", place_pre); c.move_tcp(place_pre, DOWN, n=8, speed=0.08)
    f = c.fingers(); print(f"  gap after transport {f[0]-f[1]:.4f}")
    print("[7] lower", place); c.move_tcp(place, DOWN, n=4, speed=0.06)
    pos, _ = c.tcp_world()
    if np.linalg.norm(pos - place) > 0.01:
        print("  retrying lower for accuracy"); c.move_tcp(place, DOWN, n=2, speed=0.03)
    w2 = c.wrench(); print("  wrench", w2)
    print("[8] release"); c.gripper(OPEN)
    print("[9] retreat", place_pre); c.move_tcp(place_pre, DOWN, n=3, speed=0.1)
    print("DONE")
    c.close()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 31
nohup python3 -u pick_place_tilt.py -0.223 0.026 0.542 1 -0.006 0.286 20 > yw2.log 2>&1 &
sleep 50; cat /workspace/yw2.log

# openrua op 32
sleep 20; cat /workspace/yw2.log | tail -8; timeout 10 ros2 service list | grep -i -E "cartesian|plan"

# openrua op 33
timeout 120 python3 tools/perception/cam_snap.py agentview >/dev/null; echo done

# openrua op 34
timeout 20 ros2 interface show moveit_msgs/srv/GetCartesianPath | grep -v "^\s*#" | grep -v "^\s*$" | head -60

# openrua op 35
timeout 20 ros2 interface show moveit_msgs/srv/GetCartesianPath | grep -v "^\s*#" | grep -v "^\s*$" | grep -E "^(string|float|bool|geometry_msgs/Pose\[|Constraints|RobotTrajectory|MoveItErrorCodes|---|int|builtin|\S)" 

# openrua op 36
python3 - <<'EOF'
s=open('ctl.py').read()
s=s.replace("from moveit_msgs.srv import GetPositionIK, GetPositionFK","from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetCartesianPath\nfrom geometry_msgs.msg import Pose")
s=s.replace('''        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")''','''        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.cart = self.node.create_client(GetCartesianPath, "/compute_cartesian_path")''')
s=s.replace('''    def move_tcp(self, tcp_to, quat, **kw):''','''    def cart_path(self, tcp_pts, quats, start_q=None, max_step=0.01):
        """Joint-continuous Cartesian path through TCP waypoints via MoveIt.
        Returns (list of q, fraction)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.group_name = M["planning"]["group"]
        req.link_name = "panda_hand"
        s = JointState(); s.name = list(JOINTS)
        s.position = [float(v) for v in (start_q if start_q is not None else self.arm_q())]
        req.start_state.joint_state = s
        for p, quat in zip(tcp_pts, quats):
            R = quat_R(*quat)
            hand = np.asarray(p) - TCP * R[:, 2]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, hand)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quat)
            req.waypoints.append(pose)
        req.max_step = max_step
        req.jump_threshold = 3.0
        req.avoid_collisions = False
        fut = self.cart.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
        res = fut.result()
        if res is None:
            raise RuntimeError("cartesian path service timeout")
        jt = res.solution.joint_trajectory
        idx = [jt.joint_names.index(j) for j in JOINTS]
        qs = [[pt.positions[i] for i in idx] for pt in jt.points]
        return qs, res.fraction, res.error_code.val

    def move_cart(self, tcp_to, quat, speed=0.08, quat_from=None, n_rot=1, min_t=1.0):
        """Move TCP on a straight line (optionally interpolating orientation)
        using compute_cartesian_path; retimed at constant speed; hold point."""
        pos, q0 = self.tcp_world()
        tcp_to = np.asarray(tcp_to)
        pts, quats = [], []
        if quat_from is None:
            pts, quats = [tcp_to], [quat]
        else:
            for i in range(1, n_rot + 1):
                a = i / n_rot
                pts.append(pos + a * (tcp_to - pos))
                quats.append(slerp(quat_from, quat, a))
        qs, frac, code = self.cart_path(pts, quats)
        if frac < 0.999 or len(qs) < 2:
            raise RuntimeError(f"cartesian path incomplete: fraction={frac:.3f} code={code} n={len(qs)}")
        # joint-space continuity check
        dq = np.abs(np.diff(np.array(qs), axis=0)).max()
        dist = np.linalg.norm(tcp_to - pos)
        total = max(min_t, dist / speed)
        ts = list(np.linspace(0, total, len(qs))[1:])
        qs = qs[1:]
        qs.append(qs[-1]); ts.append(total + 1.0)
        print(f"  cart path: {len(qs)} pts, max dq step {dq:.3f}, T={total:.1f}s")
        code = self.traj(qs, ts)
        p2, _ = self.tcp_world()
        print(f"  tcp now {np.round(p2, 4)} target {np.round(tcp_to, 4)} err {np.linalg.norm(p2 - tcp_to):.4f}")
        return code

    def move_tcp(self, tcp_to, quat, **kw):''')
s=s.replace('''def yaw_down(yaw):''','''def slerp(qa, qb, t):
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    d = np.dot(qa, qb)
    if d < 0: qb, d = -qb, -d
    if d > 0.9995:
        r = qa + t * (qb - qa); return tuple(r / np.linalg.norm(r))
    th = np.arccos(d)
    return tuple((np.sin((1 - t) * th) * qa + np.sin(t * th) * qb) / np.sin(th))


def yaw_down(yaw):''')
open('ctl.py','w').write(s)
EOF
timeout 300 python3 -c "
from ctl import *
c=Ctl()
pos,quat=c.tcp_world(); print('tcp',np.round(pos,4),np.round(quat,3),'q',np.round(c.arm_q(),3))
# dry-run: path from current to above yw mug at 0.70 straight-down, then descent
qs,frac,code=c.cart_path([(-0.223,0.067,0.70)],[DOWN]); print('to pre: n',len(qs),'frac',frac,'code',code, 'max dq',np.abs(np.diff(np.array(qs),axis=0)).max() if len(qs)>1 else None)
if qs:
    q_pre=qs[-1]; print('q_pre',np.round(q_pre,3))
    qs2,frac2,code2=c.cart_path([(-0.223,0.067,0.512)],[DOWN],start_q=q_pre); print('descent: n',len(qs2),'frac',frac2,'code',code2,'max dq',np.abs(np.diff(np.array(qs2),axis=0)).max() if len(qs2)>1 else None)
    if qs2: print('q_grasp',np.round(qs2[-1],3))
    lim=np.array(FJT['limits_rad']); 
    if qs2:
        Q=np.array(qs2); marg=np.minimum(Q-lim[:,0],lim[:,1]-Q); print('min margin along descent',marg.min(), 'joint',marg.min(0).argmin()+1)
c.close()
"

# openrua op 37
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
c.gripper(0.04)
c.move_cart((-0.006,0.327,0.70), DOWN, speed=0.08, quat_from=c.tcp_world()[1], n_rot=4)
c.close()
" && timeout 120 python3 scene.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
def cloud(cam):
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); K=np.load(f'{cam}_K.npy')
    rgb=cv2.imread(f'{cam}_rgb.png')
    H,W=d.shape
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    pc=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)
    return (pc@T.T)[...,:3], rgb
pw,rgb=cloud('birdview')
X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
box=(X>-0.32)&(X<-0.12)&(Y>-0.10)&(Y<0.12)&np.isfinite(Z)&(Z<0.7)&(Z>0.43)
floor=box&(Z>0.435)&(Z<0.47)
print('yw floor cen (%.3f,%.3f) n=%d ext x %.3f y %.3f'%(X[floor].mean(),Y[floor].mean(),floor.sum(),X[floor].max()-X[floor].min(),Y[floor].max()-Y[floor].min()))
print('yw xrange [%.3f,%.3f] yrange [%.3f,%.3f] zmax %.3f'%(X[box].min(),X[box].max(),Y[box].min(),Y[box].max(),Z[box].max()))
rim=box&(Z>0.53); xs,ys=X[rim],Y[rim]
A=np.c_[2*xs,2*ys,np.ones_like(xs)]; b=xs**2+ys**2
cc=np.linalg.lstsq(A,b,rcond=None)[0]; print('rim circle (%.3f,%.3f) r %.3f n=%d'%(cc[0],cc[1],np.sqrt(cc[2]+cc[0]**2+cc[1]**2),rim.sum()))
# white mug check
box=(X>-0.1)&(X<0.12)&(Y>-0.42)&(Y<-0.2)&np.isfinite(Z)&(Z>0.46)&(Z<0.62)
print('white mug xrange [%.3f,%.3f] yrange [%.3f,%.3f] zmax %.3f'%(X[box].min(),X[box].max(),Y[box].min(),Y[box].max(),Z[box].max()))
fl=box&(Z<0.49); print('white floor cen (%.3f,%.3f) z %.3f n=%d'%(X[fl].mean(),Y[fl].mean(),Z[fl].mean(),fl.sum()))
"

# openrua op 38
mkdir -p "$(dirname /workspace/pick_place_cart.py)"
cat > /workspace/pick_place_cart.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Rim-grasp a mug and place it on a plate, all moves via MoveIt Cartesian
paths (joint-continuous). Args as pick_place.py.

Usage: python3 pick_place_cart.py <mug_x> <mug_y> <rim_z> <side> <plate_x> <plate_y>
"""
import sys
import numpy as np
from ctl import Ctl, DOWN

WALL_R = 0.041
GRASP_BELOW_RIM = 0.03
TABLE_Z = 0.422
PLATE_TOP = 0.456
CARRY_Z = 0.70
OPEN, CLOSED = 0.04, 0.0


def main():
    mx, my, rim_z, side, px, py = map(float, sys.argv[1:7])
    mug_h = rim_z - TABLE_Z
    grasp = np.array([mx, my + side * WALL_R, rim_z - GRASP_BELOW_RIM])
    pre = grasp.copy(); pre[2] = CARRY_Z
    place = np.array([px, py + side * WALL_R, PLATE_TOP + (mug_h - GRASP_BELOW_RIM) + 0.008])
    place_pre = place.copy(); place_pre[2] = CARRY_Z
    print("grasp", grasp, "place", place)

    c = Ctl()

    def go(target, speed, tol=0.008):
        c.move_cart(target, DOWN, speed=speed)
        pos, _ = c.tcp_world()
        err = np.linalg.norm(pos - target)
        if err > tol:
            print(f"  off by {err:.4f}, correcting"); c.move_cart(target, DOWN, speed=0.03)
            pos, _ = c.tcp_world(); err = np.linalg.norm(pos - target)
        return err

    print("[0] open gripper"); c.gripper(OPEN)
    print("[1] to pre-grasp", pre); go(pre, 0.1)
    w0 = c.wrench(); print("  wrench", w0)
    print("[2] descend to just above rim");
    above = grasp.copy(); above[2] = rim_z + 0.03
    err = go(above, 0.06)
    if err > 0.01:
        print("ABORT: cannot reach above-rim pose accurately"); c.close(); sys.exit(4)
    print("[2b] descend to grasp", grasp); err = go(grasp, 0.04)
    w1 = c.wrench(); print("  wrench", w1, "delta", None if w0 is None else np.round(w1 - w0, 2))
    print("[3] close"); f = c.gripper(CLOSED)
    gap = f[0] - f[1]
    print(f"  finger gap {gap:.4f} (0 => closed on air)")
    if gap < 0.002:
        print("GRASP FAILED: nothing held"); c.gripper(OPEN); go(pre, 0.06); c.close(); sys.exit(2)
    print("[4] lift", pre); go(pre, 0.06)
    f = c.fingers(); print(f"  gap after lift {f[0]-f[1]:.4f}")
    if f[0] - f[1] < 0.002:
        print("LOST OBJECT during lift"); c.close(); sys.exit(3)
    print("[5] transport", place_pre); go(place_pre, 0.08)
    f = c.fingers(); print(f"  gap after transport {f[0]-f[1]:.4f}")
    print("[6] lower", place); go(place, 0.05)
    w2 = c.wrench(); print("  wrench", w2)
    print("[7] release"); c.gripper(OPEN)
    print("[8] retreat", place_pre); go(place_pre, 0.08)
    print("DONE")
    c.close()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 39
nohup python3 -u pick_place_cart.py -0.223 0.019 0.542 1 -0.006 0.286 > yw3.log 2>&1 &
sleep 75; cat /workspace/yw3.log

# openrua op 40
sleep 40; tail -n +32 /workspace/yw3.log

# openrua op 41
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
c.move_cart((-0.15,0.0,0.75), DOWN, speed=0.06)
c.close()
" && timeout 120 python3 scene.py birdview >/dev/null && timeout 120 python3 tools/perception/cam_snap.py agentview >/dev/null && python3 -c "
import numpy as np, cv2
def cloud(cam):
    d=np.load(f'{cam}_depth.npy'); T=np.load(f'{cam}_T.npy'); K=np.load(f'{cam}_K.npy')
    rgb=cv2.imread(f'{cam}_rgb.png')
    H,W=d.shape
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    pc=np.stack([(u-K[0,2])*d/K[0,0],(v-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)
    return (pc@T.T)[...,:3], rgb
pw,rgb=cloud('birdview')
X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
for name,(x0,x1,y0,y1) in {'LEFT plate region':(-0.12,0.15,-0.45,-0.18),'RIGHT plate region':(-0.15,0.12,0.18,0.45)}.items():
    box=(X>x0)&(X<x1)&(Y>y0)&(Y<y1)&np.isfinite(Z)
    plate=box&(Z>0.435)&(Z<0.462)
    mug=box&(Z>0.462)&(Z<0.7)
    fl=mug&(Z<0.49)
    rim=mug&(Z>0.53)
    xs,ys=X[rim],Y[rim]; A=np.c_[2*xs,2*ys,np.ones_like(xs)]; b=xs**2+ys**2
    cc=np.linalg.lstsq(A,b,rcond=None)[0]
    print(name)
    print('  plate cen (%.3f,%.3f) n=%d'%(X[plate].mean(),Y[plate].mean(),plate.sum()))
    print('  mug rim circle cen (%.3f,%.3f) r %.3f, rim z %.3f, interior floor z %.3f (was 0.451 on table)'%(cc[0],cc[1],np.sqrt(cc[2]+cc[0]**2+cc[1]**2),Z[rim].mean(),Z[fl].mean()))
    print('  mug color rgb', rgb[rim].mean(0)[::-1].astype(int))
"

# openrua op 42
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Hard-won facts about the Franka Panda sim workstation in /workspace (MoveIt frame, IK discontinuities, controller -5 errors, rim grasp for mugs)
metadata:
  type: project
---

Facts learned driving the Panda workstation (/workspace, machine.yaml) on 2026-09-12:

- MoveIt's model frame on this machine is `world` (panda_link0 sits at world (-0.51, 0, 0.42)); `/compute_fk` returns world-frame poses. docs/30-action.md's note that "the planner frame is the arm base" did NOT hold here — verify with FK vs `tf2_echo panda_link0 panda_hand` before applying an offset.
- `/compute_ik` with a seed still flips branches between nearby Cartesian waypoints (esp. targets within ~0.3 m of the base, joint4 near its -3.07 limit). Chaining per-waypoint IK into one trajectory makes the controller abort with error_code -5 and the arm stops mid-path. Use `/compute_cartesian_path` (GetCartesianPath, link panda_hand, header frame `world`, jump_threshold ~3) — joint-continuous, tracked to <1 mm.
- Controller also returns -5 when joint speed is high (~>1 rad/s); fix by re-sending the same target slowly and appending a 1 s hold point.
- Mugs (~9 cm outer dia) exceed the 7.7 cm max finger gap: grasp the rim (one finger inside, one outside, TCP on the wall at r≈0.041, 3 cm below rim, away from the handle). Closed finger gap ~0.010 = holding.
- Scene starts unsettled (~6 cm above supports); one gripper command advances the sim so objects drop. Table z=0.422, plates 2.2 cm thick.
- camera "left/right" in task text = agentview camera convention (image right = world +y).

**Why:** each of these cost a failed grasp or a wrong offset before being discovered.
**How to apply:** start from /workspace/ctl.py (Ctl.move_cart) and pick_place_cart.py if they still exist.
OPENRUA_EOF

# openrua op 43
cat /claude-config/projects/-workspace/memory/MEMORY.md 2>/dev/null; echo "- [Panda workstation quirks](panda-workstation-quirks.md) — MoveIt frame is world, use compute_cartesian_path not chained IK, rim-grasp mugs" >> /claude-config/projects/-workspace/memory/MEMORY.md
