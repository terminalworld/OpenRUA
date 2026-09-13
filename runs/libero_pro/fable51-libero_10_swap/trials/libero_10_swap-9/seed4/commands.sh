#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---ACTIONS--- && ros2 action list && echo ---SERVICES--- && ros2 service list | head -60 && echo ---NODES--- && ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ----; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80

# openrua op 6
for p in "birdview 330 297" "birdview 425 297" "birdview 250 270" "agentview 340 265" "agentview 160 250"; do echo -n "$p => "; timeout 60 python3 tools/perception/px2world.py $p 2>&1 | grep -v XMLPARSER; done

# openrua op 7
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a camera's depth frame as a world-frame point cloud (HxWx3 .npy)
plus the color image. Usage: python3 cloud.py <camera>
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2


def grab(node, topic, msg_type, timeout=30.0):
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
    node = rclpy.create_node("cloud")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(color_msg, "bgr8")
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1)
    world = pc @ R.T + T
    np.save(f"{cam}_world.npy", world)
    cv2.imwrite(f"{cam}.png", color)
    print(f"{cam}: cam pos {T}, saved {cam}_world.npy shape {world.shape}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
for c in birdview agentview frontview sideview; do timeout 120 python3 cloud.py $c 2>&1 | grep -v XMLPARSER; done

# openrua op 9
python3 -c "
import numpy as np, cv2
w = np.load('birdview_world.npy'); img = cv2.imread('birdview.png')
z = w[...,2]
# table height: mode of z in table region
tbl = z[350:450, 200:450]
print('table z median', np.median(tbl))
# mug (yellow/white) around px (330,297)
patch = w[285:310, 315:345]
print('mug patch z range', np.nanmin(patch[...,2]), np.nanmax(patch[...,2]))
# find mug rim: highest z within region
reg = w[270:330, 300:360]
zz = reg[...,2]
mask = zz > 0.97
pts = reg[mask]
print('mug pts count', mask.sum(), 'x range', pts[:,0].min(), pts[:,0].max(), 'y range', pts[:,1].min(), pts[:,1].max(), 'z max', zz.max())
# body only (exclude handle): use z > 0.99 (rim)
rim = reg[zz>1.0]
print('rim: x', rim[:,0].min(), rim[:,0].max(), 'y', rim[:,1].min(), rim[:,1].max(), 'z', rim[:,2].min(), rim[:,2].max(), 'center', rim[:,0].mean(), rim[:,1].mean())
"


# openrua op 10
python3 -c "
import numpy as np, cv2
w = np.load('birdview_world.npy'); img = cv2.imread('birdview.png')
reg = w[260:340, 290:370]; zz=reg[...,2]
hist, edges = np.histogram(zz[zz>0.91], bins=20)
for h,e in zip(hist,edges): print(f'{e:.3f} {h}')
# per-height x/y extents
for lo in [1.06,1.05,1.04,1.02,1.0,0.98,0.95]:
    m = zz>lo
    p = reg[m]
    print(lo, m.sum(), 'x[%.3f,%.3f] y[%.3f,%.3f] cx=%.3f cy=%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,0].mean(),p[:,1].mean()))
"


# openrua op 11
python3 -c "
import numpy as np, cv2
for cam, (r0,r1,c0,c1) in {'frontview':(280,380,290,380), 'agentview':(200,310,280,400), 'sideview':(230,300,255,320)}.items():
    w = np.load(cam+'_world.npy')
    reg = w[r0:r1, c0:c1]
    # mug points: those with x between -0.12 and 0.08, y between -0.08 and 0.12, z>0.9
    m = (reg[...,0]>-0.15)&(reg[...,0]<0.1)&(reg[...,1]>-0.1)&(reg[...,1]<0.13)&(reg[...,2]>0.85)&(reg[...,2]<1.2)
    p = reg[m]
    print(cam, m.sum(), 'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
    hist, edges = np.histogram(p[:,2], bins=12)
    print(' z hist', [(round(e,3),h) for h,e in zip(hist,edges)])
"


# openrua op 12
timeout 300 python3 tools/action/gripper_cmd.py 0.04 2>&1 | grep -v XMLPARSER; ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 13
timeout 120 python3 cloud.py agentview 2>&1 | grep -v XMLPARSER; timeout 120 python3 cloud.py birdview 2>&1 | grep -v XMLPARSER; python3 -c "
import numpy as np
w = np.load('agentview_world.npy')
reg = w[200:310, 280:400]
m = (reg[...,0]>-0.15)&(reg[...,0]<0.1)&(reg[...,1]>-0.1)&(reg[...,1]<0.13)&(reg[...,2]>0.85)&(reg[...,2]<1.2)
p = reg[m]
hist, edges = np.histogram(p[:,2], bins=12)
print(' z hist', [(round(e,3),h) for h,e in zip(hist,edges)])
"

# openrua op 14
python3 -c "
import numpy as np, cv2
w = np.load('birdview_world.npy'); img=cv2.imread('birdview.png')
z=w[...,2]
# mug region: search around x in [-0.1,0.05], y in [-0.06,0.1]
m = (w[...,0]>-0.12)&(w[...,0]<0.08)&(w[...,1]>-0.08)&(w[...,1]<0.12)&(z>0.93)
p=w[m]
print('mug pts', m.sum(), 'z max', p[:,2].max())
hist, edges = np.histogram(p[:,2], bins=10); print([(round(e,3),h) for h,e in zip(hist,edges)])
rim = p[p[:,2]>0.99]
print('rim x[%.3f,%.3f] y[%.3f,%.3f] center (%.4f,%.4f)'%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max(),rim[:,0].mean(),rim[:,1].mean()))
# yellow handle pixels in birdview: HSV
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
ym=(hsv[...,0]>18)&(hsv[...,0]<38)&(hsv[...,1]>100)&(hsv[...,2]>100)
ys,xs=np.nonzero(ym); print('yellow px', len(xs), 'u range',xs.min(),xs.max(),'v range',ys.min(),ys.max())
hp = w[ym]; print('handle world x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(hp[:,0].min(),hp[:,0].max(),hp[:,1].min(),hp[:,1].max(),hp[:,2].min(),hp[:,2].max()))
# rim ring: fit circle to rim points excluding handle side
"


# openrua op 15
python3 -c "
import numpy as np, cv2
w = np.load('birdview_world.npy'); img=cv2.imread('birdview.png')
z=w[...,2]
m = (w[...,0]>-0.12)&(w[...,0]<0.08)&(w[...,1]>-0.08)&(w[...,1]<0.12)&(z>0.995)
p=w[m][:,:2]
# algebraic circle fit
A=np.c_[2*p[:,0],2*p[:,1],np.ones(len(p))]; b=(p**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print('circle fit: center (%.4f,%.4f) r=%.4f'%(cx,cy,r))
res = np.abs(np.hypot(p[:,0]-cx,p[:,1]-cy)-r); print('resid median',np.median(res),'max',res.max())
# refit excluding outliers (handle)
k=res<0.008; p2=p[k]
A=np.c_[2*p2[:,0],2*p2[:,1],np.ones(len(p2))]; b=(p2**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print('refit: center (%.4f,%.4f) r=%.4f n=%d'%(cx,cy,r,len(p2)))
# pixel of center
ys,xs=np.nonzero(m); print('rim pixel bbox u',xs.min(),xs.max(),'v',ys.min(),ys.max())
# inner floor of mug
inner=(w[...,0]>cx-0.03)&(w[...,0]<cx+0.03)&(w[...,1]>cy-0.03)&(w[...,1]<cy+0.03)&(z<0.99)&(z>0.9)
print('inner floor z', np.median(w[inner][:,2]) if inner.sum() else None, inner.sum())
"


# openrua op 16
python3 -c "
import numpy as np
w = np.load('agentview_world.npy')
cx,cy=-0.0177,0.0365
m = (w[...,0]>cx-0.08)&(w[...,0]<cx+0.08)&(w[...,1]>cy-0.08)&(w[...,1]<cy+0.08)&(w[...,2]>0.905)&(w[...,2]<1.01)
p=w[m]
for lo in np.arange(0.91,1.0,0.01):
    s=p[(p[:,2]>=lo)&(p[:,2]<lo+0.01)]
    if len(s)==0: continue
    # radius from center estimate
    r=np.hypot(s[:,0]-cx,s[:,1]-cy)
    print('z %.2f n=%4d y[%.3f,%.3f] width=%.3f  r med=%.3f r90=%.3f'%(lo,len(s),s[:,1].min(),s[:,1].max(),s[:,1].max()-s[:,1].min(),np.median(r),np.percentile(r,90)))
"


# openrua op 17
python3 -c "
import numpy as np
w = np.load('birdview_world.npy'); z=w[...,2]
# microwave region: y<-0.08, x in [-0.5,0.4]
m=(w[...,1]<-0.05)&(w[...,1]>-0.7)&(w[...,0]>-0.6)&(w[...,0]<0.4)&(z>0.95)
p=w[m]
hist,edges=np.histogram(p[:,2],bins=15); print('z hist',[(round(e,3),h) for h,e in zip(hist,edges)])
top=p[p[:,2]>np.percentile(p[:,2],50)]
print('top z med %.3f'%np.median(top[:,2]))
body=p[(p[:,2]>1.15)]
print('body(z>1.15) x[%.3f,%.3f] y[%.3f,%.3f] n=%d'%(body[:,0].min(),body[:,0].max(),body[:,1].min(),body[:,1].max(),len(body)))
# door: thinner, maybe different height
for lo,hi in [(0.95,1.05),(1.05,1.15),(1.15,1.2),(1.2,1.3)]:
    s=p[(p[:,2]>=lo)&(p[:,2]<hi)]
    if len(s): print('z[%.2f,%.2f) n=%d x[%.3f,%.3f] y[%.3f,%.3f]'%(lo,hi,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
"


# openrua op 18
python3 -c "
import numpy as np
w = np.load('birdview_world.npy'); z=w[...,2]
m=(w[...,1]<-0.1)&(w[...,1]>-0.7)&(w[...,0]>-0.6)&(w[...,0]<0.3)&(z>1.0)&(z<1.2)
p=w[m]
print('z med %.3f  p95 %.3f p5 %.3f'%(np.median(p[:,2]),np.percentile(p[:,2],95),np.percentile(p[:,2],5)))
# occupancy grid 2cm
xs=np.arange(-0.6,0.3,0.02); ys=np.arange(-0.7,-0.1,0.02)
H,_,_=np.histogram2d(p[:,0],p[:,1],bins=[xs,ys])
print('     y:'+''.join('%d'%(abs(int(round(y*100)))//10%10) for y in ys[:-1]))
print('       '+''.join('%d'%(abs(int(round(y*100)))%10) for y in ys[:-1]))
for i,x in enumerate(xs[:-1]):
    print('x=%+.2f '%x+''.join('#' if H[i,j]>0 else '.' for j in range(len(ys)-1)))
"


# openrua op 19
ls -la /workspace/.ros/log/; grep -il "microwave\|mug\|robocasa\|xml" /workspace/.ros/log/* 2>/dev/null; timeout 10 ros2 topic echo /pipeline_state --once 2>/dev/null | head -20; ros2 topic info /pipeline_state

# openrua op 20
for c in paperview galleryview; do timeout 60 python3 tools/perception/cam_snap.py $c 2>&1 | grep -v XMLPARSER; done

# openrua op 21
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot helpers: joint state, FK, IK, trajectory, gripper, TF, camera.
World frame = panda_link0 + (-0.66, 0, 0.912).
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image, CameraInfo
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
LIMITS = np.array(FJT["limits_rad"])
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])
TCP = M["hand"]["tcp_offset_m"]


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)

    def _on_js(self, msg):
        self.js = msg

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self.js = None
            end = time.time() + 20
            while self.js is None and time.time() < end:
                self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk_pose(self, q=None, link="panda_hand"):
        """Return (pos_world, quat_xyzw) of link for joints q (default current)."""
        if q is None:
            q = self.q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp_pose(self, q=None):
        pos, quat = self.fk_pose(q)
        R = Rot.from_quat(quat).as_matrix()
        return pos + TCP * R[:, 2], quat

    # ---------- IK ----------
    def solve_ik(self, pos_world, quat_xyzw, seed=None, at_tcp=True, tries=1):
        """IK for hand (or TCP) pose in world; returns joint array or None."""
        pos = np.array(pos_world, float)
        R = Rot.from_quat(quat_xyzw).as_matrix()
        if at_tcp:
            pos = pos - TCP * R[:, 2]
        pos_base = pos - BASE_IN_WORLD
        if seed is None:
            seed = self.q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos_base)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_xyzw)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(x) for x in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.array([sol[j] for j in ARM])
            seed = np.clip(seed + np.random.uniform(-0.3, 0.3, 7), LIMITS[:, 0], LIMITS[:, 1])
        return None

    # ---------- acting ----------
    def move_joints(self, q_target, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if waypoints:
            n = len(waypoints) + 1
            for i, wp in enumerate(waypoints):
                t = seconds * (i + 1) / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in wp])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q_target])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        handle = send.result()
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qn = self.q()
        err = np.abs(qn - np.array(q_target)).max()
        return code, err

    def move_tcp(self, pos_world, quat_xyzw, seconds=3.0, seed=None, tries=5):
        q = self.solve_ik(pos_world, quat_xyzw, seed=seed, tries=tries)
        if q is None:
            return None
        code, err = self.move_joints(q, seconds)
        return code, err, q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        return r.reached_goal, r.stalled, self.fingers()

    # ---------- camera ----------
    def snap(self, cam, out=None):
        from cv_bridge import CvBridge
        import cv2
        got = {}
        topic = f"/{cam}/color/image_raw"
        sub = self.node.create_subscription(Image, topic, lambda m: got.setdefault("m", m), 1)
        end = time.time() + 30
        while "m" not in got and time.time() < end:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        img = CvBridge().imgmsg_to_cv2(got["m"], "bgr8")
        out = out or f"{cam}.png"
        cv2.imwrite(out, img)
        return out


def quat_down(yaw_deg=0.0):
    """Quaternion (xyzw) for hand pointing straight down (z axis = -world z),
    with the finger-closing axis (hand y) rotated by yaw about world z.
    At yaw=0, hand x = world x, hand y = -world y."""
    return Rot.from_euler("xyz", [180, 0, yaw_deg], degrees=True).as_quat()


def quat_from_axes(z_axis, x_axis):
    z = np.array(z_axis, float); z /= np.linalg.norm(z)
    x = np.array(x_axis, float); x -= x.dot(z) * z; x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return Rot.from_matrix(np.c_[x, y, z]).as_quat()
OPENRUA_EOF

# openrua op 22
timeout 120 python3 -c "
from rob import *
r=Robot()
print('q', r.q())
print('fingers', r.fingers())
pos,quat=r.fk_pose(); print('hand world', pos, quat)
print('tcp', r.tcp_pose()[0])
print('R', Rot.from_quat(quat).as_matrix().round(3))
" 2>&1 | grep -v XMLPARSER

# openrua op 23
timeout 30 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A1 "Translation" | head -2; timeout 30 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A1 "Translation" | head -2

# openrua op 24
sed -i 's/pos = np.array(\[p.position.x, p.position.y, p.position.z\]) + BASE_IN_WORLD/pos = np.array([p.position.x, p.position.y, p.position.z])  # FK is already in world/' rob.py && sed -i 's/        pos_base = pos - BASE_IN_WORLD/        pos_base = pos  # IK model frame == world (verified by FK roundtrip)/' rob.py && grep -n "pos_base = \|already in world" rob.py
timeout 120 python3 -c "
from rob import *
r=Robot()
q0=r.q(); pos,quat=r.fk_pose(); print('hand world', pos.round(3), quat.round(3))
q=r.solve_ik(pos,quat,seed=q0,at_tcp=False)
print('IK sol', None if q is None else q.round(3)); print('cur   ', q0.round(3))
if q is not None: print('fk of sol', r.fk_pose(q)[0].round(3))
" 2>&1 | grep -v XMLPARSER

# openrua op 25
timeout 120 python3 -c "
from rob import *
r=Robot()
q0=r.q(); pos,quat=r.fk_pose()
q=np.array([ 0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009])
p2,q2=r.fk_pose(q); print('sol pose', p2.round(3), q2.round(3)); print('cur pose', pos.round(3), quat.round(3))
print(Rot.from_quat(q2).as_matrix().round(2)); print(Rot.from_quat(quat).as_matrix().round(2))
" 2>&1 | grep -v XMLPARSER

# openrua op 26
sed -i 's/            req.ik_request.group_name = M\["planning"\]\["group"\]/            req.ik_request.group_name = M["planning"]["group"]\n            req.ik_request.ik_link_name = "panda_hand"/' rob.py
timeout 120 python3 -c "
from rob import *
r=Robot()
q0=r.q(); pos,quat=r.fk_pose()
q=r.solve_ik(pos,quat,seed=q0,at_tcp=False)
print('IK sol', None if q is None else q.round(3))
if q is not None:
    p2,q2=r.fk_pose(q); print('sol pose', p2.round(3), q2.round(3)); print('cur pose', pos.round(3), quat.round(3))
" 2>&1 | grep -v XMLPARSER

# openrua op 27
timeout 30 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A2 "Translation" | head -3; timeout 30 ros2 run tf2_ros tf2_echo world robot0_robotview_optical_frame 2>&1 | grep -A2 "Translation" | head -3

# openrua op 28
timeout 600 python3 -c "
from rob import *
r=Robot()
# View 1: look at the -y face of the microwave from -y, above, angled down
quat = quat_from_axes(z_axis=[0.0,0.7,-0.7], x_axis=[0,0,-1])
res = r.move_tcp([-0.15,-0.55,1.15], quat, seconds=4, tries=8)
print('move', None if res is None else (res[0], round(res[1],4), res[2].round(3)))
print('hand', r.fk_pose()[0].round(3))
print(r.snap('robot0_eye_in_hand','eih_view1.png'))
" 2>&1 | grep -v XMLPARSER

# openrua op 29
timeout 900 python3 -c "
from rob import *
r=Robot()
q0=r.q()
cands = [
 ([-0.15,-0.55,1.15],[0,0.7,-0.7],[0,0,-1]),
 ([-0.2,-0.5,1.2],[0,0.7,-0.7],[0,0,-1]),
 ([-0.2,-0.5,1.2],[0,0.7,-0.7],[1,0,0]),
 ([-0.25,-0.45,1.25],[0,0.6,-0.8],[1,0,0]),
 ([-0.3,-0.4,1.3],[0.3,0.5,-0.8],[1,0,0]),
 ([-0.2,-0.5,1.05],[0,1,0],[0,0,-1]),
 ([-0.2,-0.5,1.05],[0,1,0],[1,0,0]),
 ([-0.2,-0.5,1.05],[0,1,0],[-1,0,0]),
]
for pos,z,x in cands:
    q=r.solve_ik(pos, quat_from_axes(z,x), seed=q0, tries=6)
    print(pos,z,x,'->', None if q is None else q.round(2))
" 2>&1 | grep -v XMLPARSER

# openrua op 30
timeout 900 python3 -c "
from rob import *
r=Robot()
q0=r.q()
np.random.seed(0)
for pos,z,x in [([-0.2,-0.5,1.05],[0,1,0],[0,0,-1]), ([-0.2,-0.5,1.05],[0,1,0],[0,0,1]), ([-0.15,-0.45,1.0],[0,1,0],[0,0,-1]), ([-0.15,-0.45,1.0],[0,1,0],[0,0,1])]:
    for seed in [q0, np.array([-0.8,0.5,0,-1.8,0,2.3,0.8]), np.array([-1.2,0.8,0.3,-1.5,-0.5,2.0,0.5]), np.array([-0.5,-0.5,-1.0,-2.0,1.5,2.5,-1.0])]:
        q=r.solve_ik(pos, quat_from_axes(z,x), seed=seed, tries=8)
        if q is not None: print(pos,z,x,'->', q.round(2)); break
    else: print(pos,z,x,'-> None')
" 2>&1 | grep -v XMLPARSER

# openrua op 31
python3 -c "
import cv2
img=cv2.imread('birdview.png'); crop=img[150:350,120:320]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
img=cv2.imread('agentview.png'); crop=img[100:380,0:280]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"

# openrua op 32
timeout 60 python3 -c "
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0]); print(len(got[0]))
" 2>&1 | grep -v XMLPARSER; grep -n -A12 'link name="panda_hand"' robot.urdf | head -40; grep -n -B2 -A10 'link name="panda_leftfinger"' robot.urdf | head -40

# openrua op 33
python3 -c "
import numpy as np, struct
def stl_bbox(path):
    d=open(path,'rb').read()
    n=struct.unpack_from('<I',d,80)[0]
    arr=np.frombuffer(d,dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]),count=n,offset=84)
    v=arr['v'].reshape(-1,3); return v.min(0), v.max(0)
for f in ['hand','finger']:
    lo,hi=stl_bbox(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'); print(f, lo.round(4), hi.round(4))
"

# openrua op 34
find / -iname "hand*.stl" -o -iname "hand*.dae" -o -iname "finger*.stl" 2>/dev/null | head; find / -path /proc -prune -o -iname "*.xml" -print 2>/dev/null | grep -i -E "microwave|mug|robocasa|kitchen" | head

# openrua op 35
timeout 600 python3 -c "
from rob import *
r=Robot()
quat = quat_from_axes(z_axis=[0.3,0.5,-0.8], x_axis=[1,0,0])
res = r.move_tcp([-0.3,-0.4,1.3], quat, seconds=4, tries=8)
print('move', None if res is None else (res[0], round(res[1],4), res[2].round(3)))
print('hand', r.fk_pose()[0].round(3))
print(r.snap('robot0_eye_in_hand','eih_view1.png'))
" 2>&1 | grep -v XMLPARSER

# openrua op 36
timeout 600 python3 -c "
from rob import *
r=Robot()
q=np.array([-0.197,  0.162, -0.905, -1.801,  0.644,  1.517, -0.337])
for i in range(3):
    code,err=r.move_joints(q, 4); print('try',i,'code',code,'err',round(err,4))
    if err<0.02: break
print('hand', r.fk_pose()[0].round(3), r.q().round(3))
" 2>&1 | grep -v XMLPARSER

# openrua op 37
timeout 180 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v XMLPARSER; python3 -c "
import numpy as np
w=np.load('robot0_eye_in_hand_world.npy'); z=w[...,2]
# points inside microwave footprint: x in [-0.27,0.1], y in [-0.36,-0.12]
m=(w[...,0]>-0.27)&(w[...,0]<0.1)&(w[...,1]>-0.36)&(w[...,1]<-0.12)&(z>0.85)&(z<1.2)
p=w[m]; print('n',len(p))
hist,edges=np.histogram(p[:,2],bins=16); print([(round(e,3),h) for h,e in zip(hist,edges)])
# points behind the front plane y>-0.34 (inside cavity)
inside=p[(p[:,1]>-0.33)]
print('inside n',len(inside))
if len(inside):
    hist,edges=np.histogram(inside[:,2],bins=12); print('inside z',[(round(e,3),h) for h,e in zip(hist,edges)])
    hist,edges=np.histogram(inside[:,0],bins=12); print('inside x',[(round(e,3),h) for h,e in zip(hist,edges)])
    hist,edges=np.histogram(inside[:,1],bins=12); print('inside y',[(round(e,3),h) for h,e in zip(hist,edges)])
"

# openrua op 38
python3 -c "
import numpy as np
w=np.load('robot0_eye_in_hand_world.npy'); z=w[...,2]
m=(w[...,0]>-0.3)&(w[...,0]<0.12)&(w[...,1]>-0.4)&(w[...,1]<-0.1)
p=w[m]
fl=p[(p[:,2]>0.925)&(p[:,2]<0.955)]
print('floor cluster n',len(fl),'z med %.4f'%np.median(fl[:,2]))
print(' x[%.3f,%.3f] y[%.3f,%.3f]'%(fl[:,0].min(),fl[:,0].max(),fl[:,1].min(),fl[:,1].max()))
# 1cm grid occupancy of floor cluster
xs=np.arange(-0.3,0.12,0.02); ys=np.arange(-0.4,-0.1,0.02)
H,_,_=np.histogram2d(fl[:,0],fl[:,1],bins=[xs,ys])
print('      y:'+''.join('%d'%(abs(int(round(y*100)))//10%10) for y in ys[:-1])); print('        '+''.join('%d'%(abs(int(round(y*100)))%10) for y in ys[:-1]))
for i,x in enumerate(xs[:-1]): print('x=%+.2f '%x+''.join('#' if H[i,j]>3 else '.' for j in range(len(ys)-1)))
# wall points between floor and ceiling
mid=p[(p[:,2]>0.96)&(p[:,2]<1.07)]
print('mid n',len(mid))
H,_,_=np.histogram2d(mid[:,0],mid[:,1],bins=[xs,ys])
for i,x in enumerate(xs[:-1]): print('x=%+.2f '%x+''.join('#' if H[i,j]>3 else '.' for j in range(len(ys)-1)))
"

# openrua op 39
python3 -c "
import numpy as np
w=np.load('birdview_world.npy'); z=w[...,2]
# hand was at world (-0.203,0,1.27) pointing down; fingers at ~1.17-1.21
for lo,hi in [(1.15,1.20),(1.20,1.25),(1.25,1.30),(1.30,1.35),(1.35,1.40),(1.40,1.5)]:
    m=(z>lo)&(z<hi)&(w[...,0]>-0.45)&(w[...,0]<0.05)&(np.abs(w[...,1])<0.3)
    p=w[m]
    if len(p): print('z[%.2f,%.2f) n=%d x[%.3f,%.3f] y[%.3f,%.3f]'%(lo,hi,len(p),p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()))
"

# openrua op 40
python3 -c "
import numpy as np, cv2
w=np.load('agentview_world.npy'); img=cv2.imread('agentview.png')
cx,cy=-0.0177,0.0365
m=(w[...,0]>cx-0.03)&(w[...,0]<cx+0.03)&(w[...,1]<cy-0.046)&(w[...,1]>cy-0.11)&(w[...,2]>0.905)&(w[...,2]<1.02)
p=w[m]; print('handle pts',len(p))
for lo in np.arange(0.91,1.01,0.01):
    s=p[(p[:,2]>=lo)&(p[:,2]<lo+0.01)]
    if len(s): print('z %.2f n=%3d x[%.3f,%.3f] y[%.3f,%.3f]'%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
ys,xs=np.nonzero(m); print('px u',xs.min(),xs.max(),'v',ys.min(),ys.max())
crop=img[ys.min()-20:ys.max()+20, xs.min()-30:xs.max()+30]; cv2.imwrite('handle_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 41
mkdir -p "$(dirname /workspace/ik_probe.py)"
cat > /workspace/ik_probe.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
import numpy as np, sys
from rob import Robot, quat_from_axes
r = Robot("probe")
q0 = r.q()
print("q0", np.round(q0, 3))
mc = np.array([-0.0177, 0.0365])
res = {}
for th in [40, 50, 60, 70]:
    t = np.radians(th)
    z = [0, np.sin(t), -np.cos(t)]      # hand z tilted from -world z toward +y (hand leans toward -y)
    for sx in [1, -1]:
        x = [0, sx*np.cos(t), sx*np.sin(t)]  # hand x in y-z plane -> hand y = +-world x
        quat = quat_from_axes(z, x)
        for name, tip in [("pick", (mc[0], mc[1]-0.064, 1.03)),
                          ("place", (-0.15, -0.31, 1.03)),
                          ("place2", (-0.13, -0.29, 1.02))]:
            q = r.solve_ik(tip, quat, seed=q0, tries=8)
            ok = q is not None
            print(f"th={th} sx={sx} {name}: {'OK' if ok else '--'}", np.round(q,2) if ok else "")
            sys.stdout.flush()
OPENRUA_EOF

# openrua op 42
timeout 600 python3 -u ik_probe.py 2>&1 | tail -40

# openrua op 43
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3)
P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.23)&(P[:,0]<-0.02)&(P[:,1]>-0.34)&(P[:,1]<-0.15)
Q=P[m]
print(len(Q))
for lo in np.arange(0.90,1.15,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'y',np.round(s[:,1].min(),3),np.round(s[:,1].max(),3),'x',np.round(s[:,0].min(),3),np.round(s[:,0].max(),3))
"


# openrua op 44
mkdir -p "$(dirname /workspace/ik_probe2.py)"
cat > /workspace/ik_probe2.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
import numpy as np, sys
from rob import Robot, quat_from_axes
r = Robot("probe2")
q0 = r.q()
gx, gy, gz = -0.0135, -0.030, 0.980
for th in [25, 30, 35]:
    t = np.radians(th)
    z = [0, np.sin(t), -np.cos(t)]
    for sx in [1, -1]:
        x = [0, sx*np.cos(t), sx*np.sin(t)]
        quat = quat_from_axes(z, x)
        seed = q0
        for name, tip in [("pre", (gx, gy, 1.10)), ("grasp", (gx, gy, gz)), ("lift", (gx, gy, 1.25)),
                          ("carry", (-0.13, -0.45, 1.25)), ("down", (-0.13, -0.45, 1.04)),
                          ("in", (-0.13, -0.338, 1.04)), ("rel", (-0.13, -0.338, 1.03))]:
            q = r.solve_ik(tip, quat, seed=seed, tries=8)
            if q is None:
                print(f"th={th} sx={sx} {name}: --"); continue
            seed = q
            p7, _ = r.fk_pose(q, "panda_link7")
            p6, _ = r.fk_pose(q, "panda_link6")
            ph, _ = r.fk_pose(q, "panda_hand")
            print(f"th={th} sx={sx} {name}: q={np.round(q,2)} hand={np.round(ph,3)} l7={np.round(p7,3)} l6={np.round(p6,3)}")
            sys.stdout.flush()
OPENRUA_EOF

# openrua op 45
timeout 900 python3 -u ik_probe2.py 2>&1 | tail -50

# openrua op 46
mkdir -p "$(dirname /workspace/step.py)"
cat > /workspace/step.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""step.py <cmd> [args]  -- one-shot robot actions built on rob.py
  tcp x y z th [sx] [sec]  : move TCP to world pos with hand tilted th deg toward -y (hand y = sx*world x)
  down                      : hand pointing straight down: tcp x y z (yaw 0 -> hand x = world x, closing along y)
  grip open|close
  state                     : print q, tcp pose, fingers
  snap cam [out]
"""
import sys, json
import numpy as np
from rob import Robot, quat_from_axes, quat_down, TCP
from scipy.spatial.transform import Rotation as Rot


def tilt_quat(th, sx=1):
    t = np.radians(th)
    z = [0, np.sin(t), -np.cos(t)]
    x = [0, sx * np.cos(t), sx * np.sin(t)]
    return quat_from_axes(z, x)


def main():
    a = sys.argv[1:]
    r = Robot("step")
    cmd = a[0]
    if cmd == "state":
        q = r.q(); p, qu = r.tcp_pose(q)
        print("q", np.round(q, 4).tolist()); print("tcp", np.round(p, 4).tolist(), "quat", np.round(qu, 3).tolist())
        print("fingers", r.fingers())
    elif cmd == "grip":
        w = 0.08 if a[1] == "open" else 0.0
        print("grip", r.gripper(w))
    elif cmd == "snap":
        print(r.snap(a[1], a[2] if len(a) > 2 else None))
    elif cmd in ("tcp", "vert"):
        x, y, z = map(float, a[1:4])
        if cmd == "tcp":
            th = float(a[4]); sx = int(a[5]) if len(a) > 5 else 1
            sec = float(a[6]) if len(a) > 6 else 4.0
            quat = tilt_quat(th, sx)
        else:
            # vertical: hand z = -world z, hand y = world x  (closing along x)
            quat = quat_from_axes([0, 0, -1], [0, 1, 0]) if (len(a) <= 4 or a[4] == "x") else quat_from_axes([0, 0, -1], [1, 0, 0])
            sec = float(a[5]) if len(a) > 5 else 4.0
        seed = r.q()
        q = r.solve_ik([x, y, z], quat, seed=seed, tries=10)
        if q is None:
            print("IK FAIL"); return
        dq = np.abs(q - seed).max()
        print("q_target", np.round(q, 3).tolist(), "max dq", round(dq, 3))
        for attempt in range(3):
            code, err = r.move_joints(q, seconds=sec)
            print("move code", code, "err", round(err, 4))
            if err < 0.01:
                break
        p, qu = r.tcp_pose()
        print("tcp now", np.round(p, 4).tolist(), "quat", np.round(qu, 3).tolist())
        print("fingers", r.fingers())
    elif cmd == "joints":
        q = np.array(json.loads(a[1])); sec = float(a[2]) if len(a) > 2 else 4.0
        for attempt in range(3):
            code, err = r.move_joints(q, seconds=sec)
            print("move code", code, "err", round(err, 4))
            if err < 0.01:
                break
        p, qu = r.tcp_pose(); print("tcp now", np.round(p, 4).tolist())


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 47
timeout 300 python3 -u step.py vert -0.0135 -0.03 1.25 x 5 2>&1 | tail

# openrua op 48
timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_pre.png 2>&1 | tail -2

# openrua op 49
timeout 300 python3 -u step.py vert -0.0135 -0.03 1.05 x 3 2>&1 | tail -4 && timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_pre2.png | tail -1

# openrua op 50
timeout 120 python3 -u cloud.py robot0_eye_in_hand 2>&1 | tail -1 && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# handle region: near x -0.0135, y in [-0.06,-0.012], z>0.93
m=(P[:,0]>-0.06)&(P[:,0]<0.04)&(P[:,1]>-0.07)&(P[:,1]<-0.012)&(P[:,2]>0.93)&(P[:,2]<1.02)
Q=P[m]; print(len(Q))
for lo in np.arange(0.93,1.02,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[5,50,95]),3),'y',np.round(np.percentile(s[:,1],[5,50,95]),3))
# rim
m=(P[:,2]>0.99)&(P[:,2]<1.01)&(np.hypot(P[:,0]+0.0177,P[:,1]-0.0365)<0.07)
R=P[m]; print('rim n',len(R),'center',np.round(R[:,:2].mean(0),4),'z',np.round(R[:,2].mean(),3))
"


# openrua op 51
timeout 300 python3 -u step.py vert -0.015 -0.030 0.963 x 3 2>&1 | tail -3 && timeout 120 python3 -u step.py grip close | tail -1

# openrua op 52
timeout 300 python3 -u step.py vert -0.015 -0.030 1.10 x 3 2>&1 | tail -2 && timeout 120 python3 -u step.py snap agentview agent_lift.png | tail -1

# openrua op 53
timeout 300 python3 -u step.py vert -0.02 0.15 1.10 x 4 2>&1 | tail -2 && timeout 300 python3 -u step.py vert -0.02 0.15 0.975 x 3 2>&1 | tail -2 && timeout 120 python3 -u step.py grip open | tail -1

# openrua op 54
timeout 300 python3 -u step.py vert -0.02 0.15 1.25 x 3 2>&1 | tail -1 && timeout 120 python3 -u cloud.py agentview | tail -1 && python3 -c "
import numpy as np
P=np.load('agentview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.985)&(P[:,2]<1.01)&(np.hypot(P[:,0]+0.02,P[:,1]-0.15)<0.08)
R=P[m]; print('rim n',len(R),'center',np.round(R[:,:2].mean(0),4),'z',np.round(R[:,2].mean(),3),'x range',np.round(R[:,0].min(),3),np.round(R[:,0].max(),3),'y range',np.round(R[:,1].min(),3),np.round(R[:,1].max(),3))
cx,cy=R[:,:2].mean(0)
m=(P[:,2]>0.92)&(P[:,2]<0.995)&(P[:,1]<cy-0.05)&(P[:,1]>cy-0.11)&(np.abs(P[:,0]-cx)<0.04)
H=P[m]; print('handle n',len(H))
for lo in np.arange(0.92,1.0,0.01):
    s=H[(H[:,2]>=lo)&(H[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[5,50,95]),3),'y',np.round(np.percentile(s[:,1],[5,50,95]),3))
"


# openrua op 55
timeout 120 python3 -u cloud.py birdview | tail -1 && python3 -c "
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.91)&(P[:,2]<1.05)&(np.hypot(P[:,0]+0.02,P[:,1]-0.18)<0.12)
Q=P[m]; print('n',len(Q),'zmax',Q[:,2].max())
for lo in np.arange(0.91,1.02,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[2,50,98]),3),'y',np.round(np.percentile(s[:,1],[2,50,98]),3))
R=Q[Q[:,2]>0.99]
print('top ring center',np.round(R[:,:2].mean(0),4),'x range',np.round([R[:,0].min(),R[:,0].max()],3),'y range',np.round([R[:,1].min(),R[:,1].max()],3))
"


# openrua op 56
python3 -c "
import cv2
im=cv2.imread('birdview.png'); h,w=im.shape[:2]
crop=im[h//2-140:h//2+100, w//2-160:w//2+160]; cv2.imwrite('bird_crop2.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop2.png', cv2.resize(im[120:360,160:480],None,fx=2,fy=2))
"

# openrua op 57
timeout 300 python3 -u step.py joints "[-0.197,0.161,-0.904,-1.802,0.643,1.519,-0.335]" 5 2>&1 | tail -2 && timeout 120 python3 -u cloud.py birdview | tail -1 && python3 -c "
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.91)&(P[:,2]<1.05)&(np.hypot(P[:,0]+0.02,P[:,1]-0.18)<0.15)
Q=P[m]; print('n',len(Q),'zmax',Q[:,2].max())
for lo in np.arange(0.91,1.02,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[2,50,98]),3),'y',np.round(np.percentile(s[:,1],[2,50,98]),3))
R=Q[Q[:,2]>0.99]
print('top ring center',np.round(R[:,:2].mean(0),4),'x range',np.round([R[:,0].min(),R[:,0].max()],3),'y range',np.round([R[:,1].min(),R[:,1].max()],3))
"


# openrua op 58
python3 -c "
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.985)&(P[:,2]<1.02)&(P[:,1]>0.13)&(P[:,1]<0.29)&(P[:,0]>-0.1)&(P[:,0]<0.06)
R=P[m]; print(len(R))
x,y=R[:,0],R[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print('fit center',round(cx,4),round(cy,4),'r',round(r,4))
res=np.hypot(x-cx,y-cy)-r; print('resid p95',np.round(np.percentile(np.abs(res),95),4))
# handle: z in 0.93..0.985, y<cy-0.05
m=(P[:,2]>0.93)&(P[:,2]<0.985)&(P[:,1]<cy-0.05)&(P[:,1]>cy-0.11)&(np.abs(P[:,0]-cx)<0.05)
H=P[m]; print('handle n',len(H),'x',np.round(np.percentile(H[:,0],[5,50,95]),3),'y',np.round(np.percentile(H[:,1],[5,50,95]),3), 'zmax',H[:,2].max())
"


# openrua op 59
timeout 300 python3 -u step.py tcp -0.021 0.146 1.10 70 1 5 2>&1 | tail -4 && timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_pre3.png | tail -1

# openrua op 60
timeout 300 python3 -u step.py tcp -0.021 0.146 0.970 70 1 4 2>&1 | tail -3 && timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_grasp.png | tail -1

# openrua op 61
timeout 300 python3 -u step.py tcp -0.021 0.146 0.970 70 1 3 2>&1 | tail -3

# openrua op 62
timeout 120 python3 -c "
import numpy as np
from rob import Robot, LIMITS
r=Robot('chk'); q=r.q(); print('q',np.round(q,3)); print('dist to limits', np.round(np.minimum(q-LIMITS[:,0], LIMITS[:,1]-q),3))
for l in ['panda_link4','panda_link5','panda_link6','panda_link7','panda_hand']:
    print(l, np.round(r.fk_pose(q,l)[0],3))
" 2>&1 | tail -8

# openrua op 63
timeout 300 python3 -u step.py tcp -0.021 0.146 0.995 70 1 3 2>&1 | tail -3

# openrua op 64
timeout 300 python3 -u step.py tcp -0.021 0.137 0.964 70 1 3 2>&1 | tail -3

# openrua op 65
timeout 300 python3 -u step.py tcp -0.021 0.146 1.10 70 1 3 2>&1 | tail -2 && timeout 120 python3 -u cloud.py agentview | tail -1 && python3 -c "
import numpy as np
P=np.load('agentview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.99)&(P[:,2]<1.02)&(P[:,1]>0.10)&(P[:,1]<0.29)&(P[:,0]>-0.1)&(P[:,0]<0.06)
R=P[m]; x,y=R[:,0],R[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print('rim n',len(R),'fit center',round(cx,4),round(cy,4),'r',round(r,4),'y range',np.round([y.min(),y.max()],3),'x range',np.round([x.min(),x.max()],3))
m=(P[:,2]>0.92)&(P[:,2]<0.99)&(P[:,1]<cy-0.05)&(P[:,1]>cy-0.12)&(np.abs(P[:,0]-cx)<0.05)
H=P[m]; print('handle n',len(H))
for lo in np.arange(0.92,0.99,0.01):
    s=H[(H[:,2]>=lo)&(H[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[5,50,95]),3),'y',np.round(np.percentile(s[:,1],[5,50,95]),3))
"

# openrua op 66
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/step.py", "old_string": "        p, qu = r.tcp_pose()\\n        print(\\"tcp now\\", np.round(p, 4).tolist(), \\"quat\\", np.round(qu, 3).tolist())\\n        print(\\"fingers\\", r.fingers())\\n    elif cmd == \\"joints\\":", "new_string": "        p, qu = r.tcp_pose()\\n        print(\\"tcp now\\", np.round(p, 4).tolist(), \\"quat\\", np.round(qu, 3).tolist())\\n        # closed-loop correction of steady-state offset\\n        tgt = np.array([x, y, z])\\n        for it in range(3):\\n            off = p - tgt\\n            if np.linalg.norm(off) < 0.003:\\n                break\\n            q = r.solve_ik(tgt - off, quat, seed=r.q(), tries=10)\\n            if q is None:\\n                print(\\"IK FAIL (corr)\\"); break\\n            code, err = r.move_joints(q, seconds=2.0)\\n            p, qu = r.tcp_pose()\\n            print(f\\"corr{it} off={np.round(off,4).tolist()} -> tcp {np.round(p,4).tolist()} code {code} err {round(err,4)}\\")\\n        print(\\"fingers\\", r.fingers())\\n    elif cmd == \\"joints\\":", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 67
timeout 400 python3 -u step.py tcp -0.016 0.138 1.02 70 1 3 2>&1 | tail -5

# openrua op 68
timeout 120 python3 -c "
import numpy as np
from rob import Robot, LIMITS
r=Robot('chk'); q=r.q(); print('q',np.round(q,3)); print('dist to limits', np.round(np.minimum(q-LIMITS[:,0], LIMITS[:,1]-q),3))
for l in ['panda_link3','panda_link4','panda_link5','panda_link6','panda_link7','panda_hand']:
    print(l, np.round(r.fk_pose(q,l)[0],3))
r.snap('agentview','agent_stuck.png'); r.snap('robot0_eye_in_hand','eih_stuck.png')
" 2>&1 | tail -9

# openrua op 69
timeout 300 python3 -u step.py tcp -0.016 0.138 1.10 70 1 3 2>&1 | grep -E "tcp now|corr|fingers" | tail -3 && timeout 400 python3 -u step.py vert -0.016 0.14 1.20 x 5 2>&1 | grep -E "tcp now|corr|fingers" | tail -3

# openrua op 70
timeout 300 python3 -u step.py vert -0.016 0.14 0.963 x 4 2>&1 | grep -E "tcp now|corr|fingers" | tail -3 && timeout 120 python3 -u step.py grip close | tail -1

# openrua op 71
timeout 300 python3 -u step.py vert -0.016 0.14 1.10 x 3 2>&1 | grep -E "tcp now|fingers" | tail -2 && timeout 300 python3 -u step.py vert -0.076 0.1935 1.10 x 3 2>&1 | grep -E "tcp now|fingers" | tail -2 && timeout 300 python3 -u step.py vert -0.076 0.1935 0.975 x 3 2>&1 | grep -E "tcp now|fingers" | tail -2 && timeout 120 python3 -u step.py grip open | tail -1 && timeout 300 python3 -u step.py vert -0.076 0.1935 1.25 x 3 2>&1 | grep -E "tcp now" | tail -1

# openrua op 72
timeout 300 python3 -u step.py joints "[-0.197,0.161,-0.904,-1.802,0.643,1.519,-0.335]" 5 2>&1 | tail -1 && timeout 120 python3 -u cloud.py agentview | tail -1 && python3 -c "
import numpy as np
P=np.load('agentview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.99)&(P[:,2]<1.02)&(P[:,1]>0.16)&(P[:,1]<0.33)&(P[:,0]>-0.16)&(P[:,0]<0.0)
R=P[m]; x,y=R[:,0],R[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print('rim n',len(R),'fit center',round(cx,4),round(cy,4),'r',round(r,4),'y range',np.round([y.min(),y.max()],3),'x range',np.round([x.min(),x.max()],3))
m=(P[:,2]>0.92)&(P[:,2]<0.99)&(P[:,1]<cy-0.05)&(P[:,1]>cy-0.12)&(np.abs(P[:,0]-cx)<0.05)
H=P[m]; print('handle n',len(H))
for lo in np.arange(0.92,0.99,0.01):
    s=H[(H[:,2]>=lo)&(H[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[5,50,95]),3),'y',np.round(np.percentile(s[:,1],[5,50,95]),3))
"

# openrua op 73
timeout 400 python3 -u step.py tcp -0.081 0.180 1.10 70 1 5 2>&1 | grep -E "tcp now|corr|fingers|FAIL" | tail -4 && timeout 400 python3 -u step.py tcp -0.081 0.180 1.00 70 1 3 2>&1 | grep -E "tcp now|corr|fingers|FAIL" | tail -4

# openrua op 74
timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_pre4.png | tail -1

# openrua op 75
timeout 400 python3 -u step.py tcp -0.081 0.180 0.962 70 1 3 2>&1 | grep -E "tcp now|corr|fingers|FAIL" | tail -4 && timeout 120 python3 -u step.py grip close | tail -1

# openrua op 76
timeout 400 python3 -u step.py tcp -0.081 0.180 1.08 70 1 3 2>&1 | grep -E "tcp now|corr|fingers|FAIL" | tail -3 && timeout 120 python3 -u step.py snap agentview agent_lift2.png | tail -1 && timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_lift2.png | tail -1

# openrua op 77
timeout 400 python3 -u step.py tcp -0.081 0.180 0.968 70 1 3 2>&1 | grep -E "tcp now|corr|fingers|FAIL" | tail -3 && timeout 120 python3 -u step.py grip open | tail -1 && timeout 400 python3 -u step.py tcp -0.081 0.180 1.08 70 1 3 2>&1 | grep -E "tcp now" | tail -1

# openrua op 78
timeout 400 python3 -u step.py tcp -0.081 0.175 1.10 40 1 4 2>&1 | grep -E "tcp now|corr|fingers|FAIL" | tail -3 && timeout 400 python3 -u step.py tcp -0.081 0.175 1.00 40 1 3 2>&1 | grep -E "tcp now|corr|FAIL" | tail -2 && timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_pre5.png | tail -1

# openrua op 79
timeout 400 python3 -u step.py tcp -0.081 0.175 0.955 40 1 3 2>&1 | grep -E "tcp now|corr|FAIL" | tail -2 && timeout 120 python3 -u step.py grip close | tail -1 && timeout 400 python3 -u step.py tcp -0.081 0.175 1.10 40 1 3 2>&1 | grep -E "tcp now|fingers" | tail -2 && timeout 120 python3 -u step.py snap agentview agent_lift3.png | tail -1

# openrua op 80
timeout 400 python3 -u step.py tcp -0.081 0.175 1.25 40 1 3 2>&1 | grep -E "tcp now|corr|FAIL" | tail -2 && timeout 400 python3 -u step.py tcp -0.162 -0.20 1.25 40 1 5 2>&1 | grep -E "tcp now|corr|FAIL|fingers" | tail -3

# openrua op 81
timeout 400 python3 -u step.py tcp -0.162 -0.46 1.25 40 1 4 2>&1 | grep -E "tcp now|corr|FAIL|fingers" | tail -3 && timeout 120 python3 -c "
import numpy as np
from rob import Robot
r=Robot('chk'); q=r.q(); print('q',np.round(q,3))
for l in ['panda_link4','panda_link6','panda_link7','panda_hand']:
    print(l, np.round(r.fk_pose(q,l)[0],3))
" 2>&1 | tail -5

# openrua op 82
timeout 600 python3 -c "
import numpy as np
from rob import Robot
from step import tilt_quat
r=Robot('probe3'); q0=r.q(); quat=tilt_quat(40,1)
for y in [-0.40,-0.44,-0.46]:
  for z in [1.25,1.18,1.12,1.06,1.01]:
    q=r.solve_ik([-0.162,y,z],quat,seed=q0,tries=6)
    if q is None: print(y,z,'--'); continue
    l7=r.fk_pose(q,'panda_link7')[0]; print(y,z,'OK l7',np.round(l7,3),'q',np.round(q,2))
" 2>&1 | tail -16

# openrua op 83
mkdir -p "$(dirname /workspace/pathchk.py)"
cat > /workspace/pathchk.py <<'OPENRUA_EOF'
import numpy as np
from rob import Robot
from scipy.spatial.transform import Rotation as Rot

MW = dict(x=(-0.27, 0.10), y=(-0.36, -0.12), z=(0.85, 1.115))  # microwave body bbox (world)


def in_box(p, b):
    return b["x"][0] < p[0] < b["x"][1] and b["y"][0] < p[1] < b["y"][1] and b["z"][0] < p[2] < b["z"][1]


def check_path(r, q_from, q_to, n=15, mug=True, verbose=False):
    """Sample joint interpolation; report min clearance-ish info. mug: mug hangs at TCP + (0, +0.075, -0.055..+0.045)."""
    bad = []
    for i in range(n + 1):
        q = q_from + (q_to - q_from) * i / n
        pos, quat = r.fk_pose(q, "panda_hand")
        R = Rot.from_quat(quat).as_matrix()
        tcp = pos + 0.1034 * R[:, 2]
        pts = {"hand": pos, "tcp": tcp}
        pts["l7"] = r.fk_pose(q, "panda_link7")[0]
        pts["l6"] = r.fk_pose(q, "panda_link6")[0]
        if mug:
            c = tcp + np.array([0.002, 0.075, 0])
            pts["mug_bot"] = c + np.array([0, 0, -0.055])
            pts["mug_top"] = c + np.array([0, 0, 0.045])
            pts["mug_edge_y"] = c + np.array([0, 0.048, -0.055])
            pts["mug_edge_ym"] = c + np.array([0, -0.048, -0.055])
        hits = [k for k, p in pts.items() if in_box(p, MW)]
        if verbose or hits:
            print(i, "tcp", np.round(tcp, 3), "l7", np.round(pts["l7"], 3), "HITS" if hits else "", hits)
        bad += hits
    return bad
OPENRUA_EOF

# openrua op 84
timeout 600 python3 -c "
import numpy as np
from rob import Robot
from step import tilt_quat
from pathchk import check_path
r=Robot('probe4'); q0=r.q(); quat=tilt_quat(40,1)
wps=[(-0.162,-0.40,1.25),(-0.162,-0.44,1.18),(-0.162,-0.46,1.10),(-0.162,-0.46,1.01)]
qs=[q0]
for wp in wps:
    q=r.solve_ik(wp,quat,seed=qs[-1],tries=8); print(wp, None if q is None else np.round(q,2))
    qs.append(q)
for a,b,wp in zip(qs[:-1],qs[1:],wps):
    print('--- to',wp); bad=check_path(r,a,b); print('bad' if bad else 'clear')
" 2>&1 | tail -30

# openrua op 85
sed -i 's/y=(-0.36, -0.12)/y=(-0.325, -0.12)/' pathchk.py && timeout 600 python3 -c "
import numpy as np
from rob import Robot
from step import tilt_quat
from pathchk import check_path
r=Robot('probe4'); q0=r.q(); quat=tilt_quat(40,1)
wps=[(-0.162,-0.40,1.25),(-0.162,-0.44,1.18),(-0.162,-0.46,1.10),(-0.162,-0.46,1.01)]
qs=[q0]
for wp in wps:
    q=r.solve_ik(wp,quat,seed=qs[-1],tries=8); print(wp, None if q is None else np.round(q,2))
    qs.append(q)
for a,b,wp in zip(qs[:-1],qs[1:],wps):
    bad=check_path(r,a,b); print('--- to',wp,'bad' if bad else 'clear', 'maxdq',np.round(np.abs(b-a).max(),2))
" 2>&1 | tail -12

# openrua op 86
for wp in "-0.40 1.25 4" "-0.46 1.18 3" "-0.46 1.01 3"; do set -- $wp; timeout 400 python3 -u step.py tcp -0.162 $1 $2 40 1 $3 2>&1 | grep -E "tcp now|corr|FAIL" | tail -2; done; timeout 120 python3 -u step.py state | tail -1

# openrua op 87
timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_front.png | tail -1; timeout 120 python3 -u step.py snap agentview agent_front.png | tail -1

# openrua op 88
timeout 120 python3 -u cloud.py robot0_eye_in_hand | tail -1 && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# mug top rim: yellow mug near (-0.16,-0.385)
m=(np.hypot(P[:,0]+0.16,P[:,1]+0.385)<0.06)&(P[:,2]>1.0)
R=P[m]; print('mug rim n',len(R),'z',np.round(np.percentile(R[:,2],[5,50,95]),3),'x',np.round([R[:,0].min(),R[:,0].max()],3),'y',np.round([R[:,1].min(),R[:,1].max()],3))
# opening top edge: y in [-0.33,-0.31], x in [-0.23,-0.02]
m=(P[:,1]>-0.335)&(P[:,1]<-0.305)&(P[:,0]>-0.23)&(P[:,0]<-0.03)&(P[:,2]>1.0)
E=P[m]; print('front face pts n',len(E))
for lo in np.arange(1.0,1.13,0.01):
    s=E[(E[:,2]>=lo)&(E[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'y',np.round(np.percentile(s[:,1],[5,50,95]),3))
# cavity floor
m=(P[:,1]>-0.32)&(P[:,1]<-0.15)&(P[:,0]>-0.22)&(P[:,0]<-0.03)&(P[:,2]<0.97)&(P[:,2]>0.93)
F=P[m]; print('floor n',len(F),'z',np.round(np.percentile(F[:,2],[5,50,95]),3))
"

# openrua op 89
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.335)&(P[:,1]<-0.305)&(P[:,0]>-0.23)&(P[:,0]<-0.03)&(P[:,2]>1.0)&(P[:,2]<1.075)
E=P[m]; print(np.histogram(E[:,0],bins=10,range=(-0.23,-0.03)))
# points deeper inside the cavity above z 1.0
m=(P[:,1]>-0.30)&(P[:,1]<-0.15)&(P[:,0]>-0.22)&(P[:,0]<-0.04)&(P[:,2]>1.0)
D=P[m]; print('deep n',len(D)); 
if len(D): print('z',np.round(np.percentile(D[:,2],[5,50,95]),3),'y',np.round(np.percentile(D[:,1],[5,50,95]),3))
"

# openrua op 90
timeout 400 python3 -u step.py tcp -0.162 -0.46 1.025 40 1 2 2>&1 | grep -E "tcp now|corr|FAIL" | tail -1 && timeout 400 python3 -u step.py tcp -0.162 -0.333 1.025 40 1 4 2>&1 | grep -E "tcp now|corr|FAIL|fingers" | tail -3

# openrua op 91
timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_ins.png | tail -1; timeout 120 python3 -u step.py snap agentview agent_ins.png | tail -1

# openrua op 92
timeout 400 python3 -u step.py tcp -0.16 -0.344 1.015 40 1 2 2>&1 | grep -E "tcp now|corr|FAIL|fingers" | tail -3

# openrua op 93
timeout 120 python3 -u step.py grip open | tail -1 && timeout 400 python3 -u step.py tcp -0.16 -0.45 1.03 40 1 3 2>&1 | grep -E "tcp now|corr|FAIL" | tail -1 && timeout 400 python3 -u step.py tcp -0.16 -0.45 1.20 40 1 3 2>&1 | grep -E "tcp now|corr|FAIL" | tail -1 && timeout 120 python3 -u step.py snap agentview agent_placed.png | tail -1

# openrua op 94
timeout 120 python3 -u cloud.py robot0_eye_in_hand | tail -1 && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.23)&(P[:,0]<-0.03)&(P[:,1]>-0.40)&(P[:,1]<-0.16)&(P[:,2]>0.95)&(P[:,2]<1.07)
Q=P[m]; print('n',len(Q))
for lo in np.arange(0.95,1.07,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[2,50,98]),3),'y',np.round(np.percentile(s[:,1],[2,50,98]),3))
R=Q[Q[:,2]>1.035]
x,y=R[:,0],R[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print('rim fit',round(cx,4),round(cy,4),'r',round(r,4),'zmax',round(R[:,2].max(),3))
"

# openrua op 95
python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); cv2.imwrite('eih_placed.png', im)
"; echo ok

# openrua op 96
timeout 400 python3 -u step.py tcp -0.15 -0.50 1.08 40 1 3 2>&1 | grep -E "tcp now|corr|FAIL" | tail -1 && timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_low.png | tail -1

# openrua op 97
timeout 400 python3 -u step.py tcp -0.15 -0.47 1.08 40 1 3 2>&1 | grep -E "tcp now|corr|FAIL" | tail -1 && timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_low.png | tail -1

# openrua op 98
timeout 120 python3 -u cloud.py robot0_eye_in_hand | tail -1 && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# exclude side wall x<-0.22, and door
m=(P[:,0]>-0.22)&(P[:,0]<-0.05)&(P[:,1]>-0.45)&(P[:,1]<-0.20)&(P[:,2]>0.90)&(P[:,2]<1.10)
Q=P[m]; print('n',len(Q))
for lo in np.arange(0.90,1.10,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[2,50,98]),3),'y',np.round(np.percentile(s[:,1],[2,50,98]),3))
"

# openrua op 99
timeout 120 python3 -u step.py grip close | tail -1 && timeout 400 python3 -u step.py tcp -0.128 -0.415 1.10 40 -1 4 2>&1 | grep -E "tcp now|corr|FAIL" | tail -1 && timeout 400 python3 -u step.py tcp -0.128 -0.415 0.99 40 -1 3 2>&1 | grep -E "tcp now|corr|FAIL" | tail -1

# openrua op 100
timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_push0.png | tail -1

# openrua op 101
timeout 400 python3 -u step.py tcp -0.128 -0.321 0.99 40 -1 5 2>&1 | grep -E "tcp now|corr|FAIL" | tail -3

# openrua op 102
timeout 120 python3 -u step.py snap robot0_eye_in_hand eih_push1.png | tail -1; timeout 120 python3 -u step.py snap agentview agent_push1.png | tail -1

# openrua op 103
for c in frontview paperview galleryview robot0_robotview; do timeout 120 python3 -u step.py snap $c ${c}_now.png | tail -1; done; python3 -c "
import cv2, numpy as np
ims=[cv2.resize(cv2.imread(f'{c}_now.png'),(480,360)) for c in ['frontview','paperview','galleryview','robot0_robotview']]
top=np.hstack(ims[:2]); bot=np.hstack(ims[2:]); cv2.imwrite('grid_now.png', np.vstack([top,bot]))"

# openrua op 104
timeout 400 python3 -u step.py tcp -0.128 -0.42 1.0 40 -1 4 2>&1 | grep -E "tcp now|corr|FAIL" | tail -1 && timeout 400 python3 -u step.py tcp -0.15 -0.47 1.10 40 1 4 2>&1 | grep -E "tcp now|corr|FAIL" | tail -1 && timeout 120 python3 -u cloud.py robot0_eye_in_hand | tail -1 && cp robot0_eye_in_hand.png eih_after_push.png

# openrua op 105
timeout 120 python3 -u step.py state | tail -3; python3 -c "
from step import tilt_quat; import numpy as np; print(np.round(tilt_quat(40,1),3), np.round(tilt_quat(40,-1),3))"

# openrua op 106
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "            if res is not None and res.error_code.val == 1:\\n                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))\\n                return np.array([sol[j] for j in ARM])", "new_string": "            if res is not None and res.error_code.val == 1:\\n                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))\\n                qs = np.array([sol[j] for j in ARM])\\n                # verify: some IK plugins return approximate solutions\\n                fp, fq = self.fk_pose(qs)\\n                dpos = np.linalg.norm(fp - pos_base)\\n                dang = (Rot.from_quat(fq) * Rot.from_quat(quat_xyzw).inv()).magnitude()\\n                if dpos < 0.003 and dang < np.radians(2):\\n                    return qs\\n                self.node.get_logger().warn(f\\"IK approx rejected dpos={dpos:.4f} dang={np.degrees(dang):.1f}\\")", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 107
timeout 400 python3 -u step.py tcp -0.15 -0.42 1.30 40 1 4 2>&1 | grep -E "tcp now|corr|FAIL|approx" | tail -2 && timeout 120 python3 -u cloud.py robot0_eye_in_hand | tail -1 && cp robot0_eye_in_hand.png eih_survey.png && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.25)&(P[:,0]<0.0)&(P[:,1]>-0.60)&(P[:,1]<-0.33)&(P[:,2]>0.905)&(P[:,2]<1.10)
Q=P[m]; print('n',len(Q))
for lo in np.arange(0.90,1.10,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[2,50,98]),3),'y',np.round(np.percentile(s[:,1],[2,50,98]),3))
"

# openrua op 108
timeout 300 python3 -u step.py joints "[-0.197,0.161,-0.904,-1.802,0.643,1.519,-0.335]" 5 2>&1 | tail -1 && timeout 120 python3 -u cloud.py robot0_eye_in_hand | tail -1 && cp robot0_eye_in_hand.png eih_survey.png && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.25)&(P[:,0]<0.0)&(P[:,1]>-0.65)&(P[:,1]<-0.33)&(P[:,2]>0.905)&(P[:,2]<1.10)
Q=P[m]; print('n',len(Q))
for lo in np.arange(0.90,1.10,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[2,50,98]),3),'y',np.round(np.percentile(s[:,1],[2,50,98]),3))
"

# openrua op 109
python3 -c "
import numpy as np, cv2
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.40)&(P[:,0]<-0.05)&(P[:,1]>-0.65)&(P[:,1]<-0.36)&(P[:,2]>0.905)&(P[:,2]<1.3)
Q=P[m]; print('n',len(Q))
# height map 5mm cells
res=0.005; x0,y0=-0.40,-0.65
W=int(0.35/res); H=int(0.29/res)
hm=np.zeros((H,W)); 
ix=((Q[:,0]-x0)/res).astype(int); iy=((Q[:,1]-y0)/res).astype(int)
np.maximum.at(hm,(iy,ix),Q[:,2])
# print top rows
for lo in np.arange(1.0,1.12,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[2,50,98]),3),'y',np.round(np.percentile(s[:,1],[2,50,98]),3))
img=np.clip((hm-0.9)/0.25,0,1); img[hm==0]=0
img=cv2.resize((img*255).astype(np.uint8),None,fx=6,fy=6,interpolation=cv2.INTER_NEAREST)
cv2.imwrite('hm.png',cv2.applyColorMap(img,cv2.COLORMAP_JET))
print('image: x from -0.40 (left) to -0.05 (right), y from -0.65 (top row) to -0.36 (bottom)')
"

# openrua op 110
python3 -c "
import numpy as np, cv2
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.40)&(P[:,0]<-0.05)&(P[:,1]>-0.65)&(P[:,1]<-0.36)&(P[:,2]>0.905)&(P[:,2]<1.3)
Q=P[m]; print('n',len(Q))
res=0.005; x0,y0=-0.40,-0.65
W=int(round(0.35/res))+1; H=int(round(0.29/res))+1
hm=np.zeros((H,W))
ix=np.clip(((Q[:,0]-x0)/res).astype(int),0,W-1); iy=np.clip(((Q[:,1]-y0)/res).astype(int),0,H-1)
np.maximum.at(hm,(iy,ix),Q[:,2])
for lo in np.arange(0.91,1.12,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[2,50,98]),3),'y',np.round(np.percentile(s[:,1],[2,50,98]),3))
img=np.clip((hm-0.9)/0.2,0,1); img[hm==0]=0
img=cv2.resize((img*255).astype(np.uint8),None,fx=8,fy=8,interpolation=cv2.INTER_NEAREST)
cv2.imwrite('hm.png',cv2.applyColorMap(img,cv2.COLORMAP_JET))
"

# openrua op 111
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# handle region: right of body
m=(P[:,0]>-0.17)&(P[:,0]<-0.10)&(P[:,1]>-0.55)&(P[:,1]<-0.36)&(P[:,2]>0.903)&(P[:,2]<0.94)
Q=P[m]; print('handle n',len(Q))
for lo in np.arange(-0.50,-0.40,0.005):
    s=Q[(Q[:,1]>=lo)&(Q[:,1]<lo+0.005)]
    if len(s): print('y',round(lo,3),len(s),'x',np.round(np.percentile(s[:,0],[2,98]),3),'z',np.round(np.percentile(s[:,2],[2,98]),3))
# body rim end: points near x=-0.208 axis, z 0.96-1.0, min y
m2=(P[:,0]>-0.26)&(P[:,0]<-0.16)&(P[:,1]>-0.56)&(P[:,1]<-0.38)&(P[:,2]>0.93)&(P[:,2]<1.01)
B=P[m2]
for lo in np.arange(-0.53,-0.39,0.005):
    s=B[(B[:,1]>=lo)&(B[:,1]<lo+0.005)]
    if len(s): print('body y',round(lo,3),len(s),'x',np.round(np.percentile(s[:,0],[2,98]),3),'zmax',round(s[:,2].max(),3))
"

# openrua op 112
cat > probe3.py <<'EOF'
import numpy as np, sys
from rob import Robot, quat_from_axes
r = Robot("probe3")
q0 = r.q()
tip = np.array([-0.156, -0.494, 0.948])
for th in [25, 30, 35, 40]:
    t = np.radians(th)
    for yaw in [0, -10, -20]:   # yaw: rotate pointing dir about z (negative -> toward -x)
        yw = np.radians(yaw)
        d = np.array([-np.sin(yw), np.cos(yw), 0])   # horizontal pointing dir
        z = d*np.sin(t) + np.array([0,0,-np.cos(t)])
        x = d*np.cos(t) + np.array([0,0,np.sin(t)])
        quat = quat_from_axes(z, x)
        seed = q0
        out = []
        for name, p in [("pre", tip - 0.06*z), ("grasp", tip), ("lift", tip + [0,0,0.17])]:
            q = r.solve_ik(p, quat, seed=seed, tries=6)
            if q is None:
                out.append(f"{name}:--"); continue
            seed = q
            l7 = r.fk_pose(q, "panda_link7")[0]
            out.append(f"{name}:OK l7={np.round(l7,3)} q={np.round(q,2)}")
        print(f"th={th} yaw={yaw} | " + " | ".join(out)); sys.stdout.flush()
EOF
python3 probe3.py 2>&1 | grep -v "IK approx"

# openrua op 113
python3 - <<'EOF'
import re
s=open('step.py').read()
s=s.replace('''def main():''','''def dir_quat(th, yaw, sx=1):
    """hand pointing along horizontal dir d (yaw deg about z from +y, negative -> toward -x),
    pitched th deg from vertical; hand y (closing) = horizontal perpendicular to d."""
    t = np.radians(th); yw = np.radians(yaw)
    d = np.array([-np.sin(yw), np.cos(yw), 0])
    z = d*np.sin(t) + np.array([0, 0, -np.cos(t)])
    x = sx*(d*np.cos(t) + np.array([0, 0, np.sin(t)]))
    return quat_from_axes(z, x)


def main():''')
s=s.replace('''    elif cmd in ("tcp", "vert"):
        x, y, z = map(float, a[1:4])
        if cmd == "tcp":''','''    elif cmd in ("tcp", "vert", "pose"):
        x, y, z = map(float, a[1:4])
        if cmd == "pose":
            th = float(a[4]); yaw = float(a[5]); sx = int(a[6]) if len(a) > 6 else 1
            sec = float(a[7]) if len(a) > 7 else 4.0
            quat = dir_quat(th, yaw, sx)
        elif cmd == "tcp":''')
open('step.py','w').write(s)
EOF
grep -n "pose" step.py | head

# openrua op 114
cat > chk3.py <<'EOF'
import numpy as np
from rob import Robot
from step import dir_quat
from pathchk import check_path, in_box, MW
r = Robot("chk3")
q0 = r.q()
quat = dir_quat(30, -10)
tip = np.array([-0.156, -0.494, 0.948])
z = np.array([0, np.sin(np.radians(30)), -np.cos(np.radians(30))])
qpre = r.solve_ik(tip - 0.06*z, quat, seed=q0, tries=8)
qg = r.solve_ik(tip, quat, seed=qpre, tries=8)
print("qpre", np.round(qpre,3).tolist()); print("qg", np.round(qg,3).tolist())
for name,q in [("cur",q0),("pre",qpre),("grasp",qg)]:
    for l in ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]:
        p=r.fk_pose(q,l)[0]; print(name,l,np.round(p,3), "IN MW" if in_box(p,MW) else "")
print("path cur->pre hits:", check_path(r, q0, qpre, n=20, mug=False, verbose=False))
np.save("qpre.npy", qpre); np.save("qg.npy", qg)
EOF
python3 chk3.py 2>&1 | grep -v "IK approx"

# openrua op 115
python3 step.py grip open 2>&1 | grep -v approx; python3 step.py pose -0.156 -0.524 1.0 30 -10 1 5 2>&1 | grep -v approx

# openrua op 116
python3 step.py snap robot0_eye_in_hand eih_rim0.png 2>&1 | grep -v approx

# openrua op 117
python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.30)&(P[:,0]<-0.10)&(P[:,1]>-0.56)&(P[:,1]<-0.38)&(P[:,2]>0.905)&(P[:,2]<1.02)
B=P[m]
# rim end: points with y<-0.50
R=B[B[:,1]<-0.503]
print('rim pts',len(R),'y',np.round(np.percentile(R[:,1],[2,50,98]),3))
# fit circle in x-z on rim points
from scipy.optimize import least_squares
def res(p): return np.hypot(R[:,0]-p[0],R[:,2]-p[1])-p[2]
s=least_squares(res,[-0.206,0.95,0.05]); print('circle x,z,r',np.round(s.x,4))
for lo in np.arange(0.90,1.01,0.01):
    q=R[(R[:,2]>=lo)&(R[:,2]<lo+0.01)]
    if len(q): print(round(lo,2),len(q),'x',np.round(np.percentile(q[:,0],[2,98]),3))
# handle bar nearest rim
H=B[(B[:,0]>-0.16)&(B[:,2]<0.94)]
print('handle y range',np.round(np.percentile(H[:,1],[1,99]),3) if len(H) else None)
"

# openrua op 118
python3 -c "
import numpy as np
from scipy.optimize import least_squares
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.30)&(P[:,0]<-0.10)&(P[:,1]>-0.516)&(P[:,1]<-0.50)&(P[:,2]>0.905)&(P[:,2]<0.997)
R=P[m]; print('n',len(R))
def res(p): return np.hypot(R[:,0]-p[0],R[:,2]-p[1])-p[2]
s=least_squares(res,[-0.206,0.95,0.05]); print('circle x,z,r',np.round(s.x,4), 'rms',np.round(np.sqrt(np.mean(res(s.x)**2)),4))
for lo in np.arange(0.90,1.0,0.01):
    q=R[(R[:,2]>=lo)&(R[:,2]<lo+0.01)]
    if len(q): print(round(lo,2),len(q),'x',np.round(np.percentile(q[:,0],[2,98]),3),'y',np.round(np.percentile(q[:,1],[2,98]),3))
# also body silhouette further in
m=(P[:,0]>-0.30)&(P[:,0]<-0.10)&(P[:,1]>-0.50)&(P[:,1]<-0.48)&(P[:,2]>0.905)&(P[:,2]<1.01)
Q=P[m]; print('body y-0.49: x',np.round(np.percentile(Q[:,0],[1,99]),3),'zmax',Q[:,2].max().round(3))
"

# openrua op 119
python3 -c "
import numpy as np
from scipy.optimize import least_squares
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.30)&(P[:,0]<-0.10)&(P[:,1]>-0.516)&(P[:,1]<-0.50)&(P[:,2]>0.93)&(P[:,2]<0.997)
R=P[m]; print('n',len(R))
def res(p): return np.hypot(R[:,0]-p[0],R[:,2]-p[1])-p[2]
s=least_squares(res,[-0.206,0.95,0.05]); print('circle x,z,r',np.round(s.x,4), 'rms',np.round(np.sqrt(np.mean(res(s.x)**2)),4))
"

# openrua op 120
python3 step.py pose -0.155 -0.494 0.957 30 -10 1 4 2>&1 | grep -v approx

# openrua op 121
python3 step.py grip close 2>&1 | grep -v approx; python3 step.py snap robot0_eye_in_hand eih_rim1.png 2>&1 | grep -v approx

# openrua op 122
python3 step.py pose -0.155 -0.494 1.05 30 -10 1 5 2>&1 | grep -v approx; python3 step.py pose -0.155 -0.494 1.14 30 -10 1 4 2>&1 | grep -v approx

# openrua op 123
python3 step.py snap robot0_eye_in_hand eih_rim2.png 2>&1 | grep -v approx

# openrua op 124
python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.35)&(P[:,0]<-0.05)&(P[:,1]>-0.65)&(P[:,1]<-0.37)&(P[:,2]>0.905)&(P[:,2]<1.10)
Q=P[m]; print('n',len(Q))
for lo in np.arange(0.91,1.10,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
    if len(s): print(round(lo,2),len(s),'x',np.round(np.percentile(s[:,0],[2,50,98]),3),'y',np.round(np.percentile(s[:,1],[2,50,98]),3))
"

# openrua op 125
python3 step.py grip open 2>&1 | grep -v approx; python3 step.py pose -0.171 -0.523 1.003 40 -20 1 4 2>&1 | grep -v approx

# openrua op 126
python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1; python3 -c "
import numpy as np
from scipy.optimize import least_squares
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# rim end plane: points of body with min y
m=(P[:,0]>-0.30)&(P[:,0]<-0.16)&(P[:,1]>-0.53)&(P[:,1]<-0.40)&(P[:,2]>0.93)&(P[:,2]<1.01)
B=P[m]; print('rim end y min pct', np.round(np.percentile(B[:,1],[0.5,1,2]),4))
m=(P[:,0]>-0.30)&(P[:,0]<-0.10)&(P[:,1]>-0.516)&(P[:,1]<-0.50)&(P[:,2]>0.93)&(P[:,2]<0.997)
R=P[m]
def res(p): return np.hypot(R[:,0]-p[0],R[:,2]-p[1])-p[2]
s=least_squares(res,[-0.206,0.95,0.045]); print('circle x,z,r',np.round(s.x,4),'n',len(R))
# +x side of body: max x at y in [-0.50,-0.48]
m=(P[:,0]>-0.25)&(P[:,0]<-0.10)&(P[:,1]>-0.50)&(P[:,1]<-0.485)&(P[:,2]>0.94)&(P[:,2]<0.975)
Q=P[m]; print('+x side x max', np.round(np.percentile(Q[:,0],[98,99.5]),4) if len(Q) else None)
"

# openrua op 127
python3 step.py pose -0.162 -0.486 0.957 40 -20 1 4 2>&1 | grep -v approx

# openrua op 128
python3 step.py grip close 2>&1 | grep -v approx; python3 step.py state 2>&1 | grep -v approx

# openrua op 129
for z in 0.985 1.01 1.04; do python3 step.py pose -0.162 -0.486 $z 40 -20 1 3 2>&1 | grep -E "tcp now|fingers"; done

# openrua op 130
ls; grep -n -i -E "effort|force|friction|mass" machine.yaml *.md 2>/dev/null | head -30

# openrua op 131
sed -n 40,70p machine.yaml; grep -n -i -E "force|wrench|tactile|contact" docs/20-perception.md | head

# openrua op 132
cat > rimgrasp.py <<'EOF'
import numpy as np, sys, time
from rob import Robot, quat_from_axes
from scipy.spatial.transform import Rotation as Rot

def rim_quat(th, yaw, rho):
    t, yw, rh = np.radians(th), np.radians(yaw), np.radians(rho)
    d = np.array([-np.sin(yw), np.cos(yw), 0])
    z = d*np.sin(t) + np.array([0, 0, -np.cos(t)])
    x0 = d*np.cos(t) + np.array([0, 0, np.sin(t)])
    y0 = np.cross(z, x0)
    y = np.cos(rh)*y0 + np.sin(rh)*x0      # closing axis rotated about z by rho
    x = np.cross(y, z)
    return quat_from_axes(z, x), z, y

if __name__ == "__main__":
    mode = sys.argv[1]
    th, yaw, rho = 40, -20, 15
    quat, z, c = rim_quat(th, yaw, rho)
    print("z", np.round(z,3), "closing", np.round(c,3))
    tip = np.array([float(v) for v in sys.argv[2:5]])
    r = Robot("rim")
    q0 = r.q()
    targets = {"pre": tip - 0.06*z, "grasp": tip, "lift": tip + [0,0,0.02], "lift2": tip + [0,0,0.17]}
    qs = {}
    seed = q0
    for k in ["pre", "grasp", "lift", "lift2"]:
        q = r.solve_ik(targets[k], quat, seed=seed, tries=8)
        print(k, np.round(targets[k],3), "OK" if q is not None else "FAIL")
        if q is None: sys.exit(1)
        qs[k] = q; seed = q
    if mode == "check": sys.exit(0)
    def go(k, sec):
        for _ in range(3):
            code, err = r.move_joints(qs[k], seconds=sec)
            if err < 0.01: break
        p, _ = r.tcp_pose()
        print(k, "code", code, "err", round(err,4), "tcp", np.round(p,4).tolist(), "fingers", np.round(r.fingers(),4))
        return p
    go("pre", 4)
    p = go("grasp", 4)
    # correct offset once
    off = p - tip
    if np.linalg.norm(off) > 0.003:
        q = r.solve_ik(tip - off, quat, seed=r.q(), tries=8)
        if q is not None:
            qs["grasp2"] = q; go("grasp2", 2)
    print("close", r.gripper(0.0))
    go("lift", 3)
    f = r.fingers()
    if f[0] < 0.0015:
        print("SLIPPED"); sys.exit(2)
    go("lift2", 5)
    print("final fingers", r.fingers())
EOF
python3 rimgrasp.py check -0.166 -0.485 0.966 2>&1 | grep -v approx

# openrua op 133
python3 step.py grip open 2>&1 | grep -v approx; python3 rimgrasp.py run -0.165 -0.485 0.966 2>&1 | grep -v approx

# openrua op 134
grep -n -A12 'link name="panda_leftfinger"' robot.urdf | head -40; grep -n -B2 -A8 'joint name="panda_finger_joint1"' robot.urdf | head -30

# openrua op 135
python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.19)&(P[:,0]<-0.10)&(P[:,1]>-0.52)&(P[:,1]<-0.38)&(P[:,2]>0.903)&(P[:,2]<0.945)
H=P[m]; print('handle n',len(H))
for lo in np.arange(-0.49,-0.40,0.005):
    s=H[(H[:,1]>=lo)&(H[:,1]<lo+0.005)]
    if len(s): print('y',round(lo,3),len(s),'x',np.round(np.percentile(s[:,0],[1,50,99]),3),'z',np.round(np.percentile(s[:,2],[1,99]),3))
# body +x surface near y=-0.45 at various z
m=(P[:,0]>-0.20)&(P[:,0]<-0.145)&(P[:,1]>-0.46)&(P[:,1]<-0.44)&(P[:,2]>0.905)&(P[:,2]<1.0)
B=P[m]
for lo in np.arange(0.90,1.0,0.01):
    s=B[(B[:,2]>=lo)&(B[:,2]<lo+0.01)]
    if len(s): print('body z',round(lo,2),len(s),'xmax',np.round(np.percentile(s[:,0],[99]),4))
"

# openrua op 136
python3 - <<'EOF'
s=open('step.py').read()
s=s.replace('''            quat = quat_from_axes([0, 0, -1], [0, 1, 0]) if (len(a) <= 4 or a[4] == "x") else quat_from_axes([0, 0, -1], [1, 0, 0])''',
'''            ax = a[4] if len(a) > 4 else "x"
            xdir = {"x": [0, 1, 0], "xm": [0, -1, 0], "y": [1, 0, 0], "ym": [-1, 0, 0]}[ax]  # x/xm: close along world x
            quat = quat_from_axes([0, 0, -1], xdir)''')
open('step.py','w').write(s)
EOF
python3 step.py grip open 2>&1 | grep -v approx; python3 step.py vert -0.100 -0.447 0.99 xm 4 2>&1 | grep -v approx

# openrua op 137
python3 cloud.py eye_in_hand >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.50)&(P[:,1]<-0.38)&(P[:,2]>0.903)&(P[:,2]<1.0)&(P[:,0]>-0.30)&(P[:,0]<-0.05)
Q=P[m]
for y0 in np.arange(-0.49,-0.39,0.01):
    s=Q[(Q[:,1]>y0)&(Q[:,1]<y0+0.01)]
    if len(s)==0: continue
    # bar: points with x>-0.15
    b=s[s[:,0]>-0.15]; body=s[s[:,0]<=-0.15]
    print(f"y {y0:.2f}: body xmax {body[:,0].max() if len(body) else None:.4f} (z at xmax {body[np.argmax(body[:,0]),2] if len(body) else 0:.3f}) | bar n={len(b)} x {b[:,0].min() if len(b) else 0:.4f}..{b[:,0].max() if len(b) else 0:.4f} z {b[:,2].min() if len(b) else 0:.3f}..{b[:,2].max() if len(b) else 0:.3f}")
EOF

# openrua op 138
ls *.npy; python3 cloud.py eye_in_hand 2>&1 | tail -3; ls *.npy

# openrua op 139
python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.50)&(P[:,1]<-0.38)&(P[:,2]>0.903)&(P[:,2]<1.0)&(P[:,0]>-0.30)&(P[:,0]<-0.05)
Q=P[m]
for y0 in np.arange(-0.49,-0.39,0.01):
    s=Q[(Q[:,1]>y0)&(Q[:,1]<y0+0.01)]
    if len(s)==0: continue
    b=s[s[:,0]>-0.15]; body=s[s[:,0]<=-0.15]
    bs = f"body xmax {body[:,0].max():.4f} z@ {body[np.argmax(body[:,0]),2]:.3f}" if len(body) else "body -"
    bb = f"bar n={len(b)} x {b[:,0].min():.4f}..{b[:,0].max():.4f} z {b[:,2].min():.3f}..{b[:,2].max():.3f}" if len(b) else "bar -"
    print(f"y {y0:.2f}: {bs} | {bb}")
EOF

# openrua op 140
for c in birdview frontview sideview; do python3 cloud.py $c >/dev/null 2>&1; done; python3 - <<'EOF'
import numpy as np
for cam in ['birdview','frontview','sideview']:
    P=np.load(f'{cam}_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,1]>-0.50)&(P[:,1]<-0.38)&(P[:,2]>0.903)&(P[:,2]<0.985)&(P[:,0]>-0.30)&(P[:,0]<-0.05)
    Q=P[m]; print(cam, len(Q))
    for y0 in np.arange(-0.49,-0.39,0.01):
        s=Q[(Q[:,1]>y0)&(Q[:,1]<y0+0.01)]
        if len(s)==0: continue
        b=s[(s[:,0]>-0.15)&(s[:,2]<0.94)]; body=s[s[:,0]<=-0.15]
        bs = f"body xmax {body[:,0].max():.4f} z@ {body[np.argmax(body[:,0]),2]:.3f}" if len(body) else "body -"
        bb = f"bar n={len(b)} x {b[:,0].min():.4f}..{b[:,0].max():.4f} z {b[:,2].min():.3f}..{b[:,2].max():.3f}" if len(b) else "bar -"
        print(f"  y {y0:.2f}: {bs} | {bb}")
EOF

# openrua op 141
python3 cloud.py agentview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
for cam in ['agentview','frontview']:
    P=np.load(f'{cam}_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,1]>-0.53)&(P[:,1]<-0.38)&(P[:,2]>0.903)&(P[:,2]<1.05)&(P[:,0]>-0.30)&(P[:,0]<-0.0)
    Q=P[m]; print(cam, len(Q))
    for y0 in np.arange(-0.53,-0.38,0.01):
        s=Q[(Q[:,1]>y0)&(Q[:,1]<y0+0.01)]
        if len(s)==0: continue
        print(f"  y {y0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}")
EOF

# openrua op 142
python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.56)&(P[:,1]<-0.36)&(P[:,2]>0.903)&(P[:,2]<0.96)&(P[:,0]>-0.35)&(P[:,0]<-0.0)
Q=P[m]; print(len(Q))
for y0 in np.arange(-0.56,-0.36,0.01):
    s=Q[(Q[:,1]>y0)&(Q[:,1]<y0+0.01)]
    if len(s)==0: continue
    h,e=np.histogram(s[:,0],bins=np.arange(-0.35,0.0,0.01))
    print(f"y {y0:.2f}", ' '.join(f"{int(v):3d}" if v else '  .' for v in h))
print('x bins', np.round(np.arange(-0.35,0.0,0.01),2).tolist())
EOF

# openrua op 143
python3 step.py vert -0.15 -0.42 1.22 xm 4 2>&1 | grep -v approx | tail -2; python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.56)&(P[:,1]<-0.36)&(P[:,2]>0.903)&(P[:,2]<1.05)&(P[:,0]>-0.35)&(P[:,0]<-0.0)
Q=P[m]; print(len(Q))
for y0 in np.arange(-0.56,-0.36,0.01):
    s=Q[(Q[:,1]>y0)&(Q[:,1]<y0+0.01)]
    if len(s)==0: continue
    h,e=np.histogram(s[:,0],bins=np.arange(-0.35,0.0,0.01))
    zmax=[s[(s[:,0]>e[i])&(s[:,0]<e[i+1]),2].max() if h[i] else 0 for i in range(len(h))]
    print(f"y {y0:.2f}", ' '.join(f"{int((z-0.9)*100):3d}" if z else '  .' for z in zmax))
print('x bins', np.round(np.arange(-0.35,0.0,0.01),2).tolist())
EOF

# openrua op 144
python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.50)&(P[:,1]<-0.25)&(P[:,2]>0.903)&(P[:,2]<1.10)&(P[:,0]>-0.25)&(P[:,0]<0.05)
Q=P[m]; print(len(Q))
for y0 in np.arange(-0.50,-0.25,0.01):
    s=Q[(Q[:,1]>y0)&(Q[:,1]<y0+0.01)]
    if len(s)==0: continue
    h,e=np.histogram(s[:,0],bins=np.arange(-0.25,0.05,0.01))
    zmax=[s[(s[:,0]>e[i])&(s[:,0]<e[i+1]),2].max() if h[i] else 0 for i in range(len(h))]
    print(f"y {y0:.2f}", ' '.join(f"{int((z-0.9)*100):3d}" if z else '  .' for z in zmax))
print('x bins', np.round(np.arange(-0.25,0.05,0.01),2).tolist())
EOF

# openrua op 145
python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.50)&(P[:,1]<-0.30)&(P[:,2]>0.903)&(P[:,2]<1.10)&(P[:,0]>-0.20)&(P[:,0]<0.0)
Q=P[m]
for z0 in np.arange(0.90,1.04,0.01):
    s=Q[(Q[:,2]>z0)&(Q[:,2]<z0+0.01)]
    if len(s)<5: continue
    print(f"z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
# handle candidates: top points
top=Q[Q[:,2]>1.005]
print("top>1.005:", len(top), np.round(top.mean(0),3), np.round(top.min(0),3), np.round(top.max(0),3))
EOF

# openrua op 146
for c in birdview agentview; do python3 cloud.py $c >/dev/null 2>&1; done; python3 - <<'EOF'
import numpy as np
for cam in ['birdview','agentview']:
    P=np.load(f'{cam}_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,1]>-0.50)&(P[:,1]<-0.30)&(P[:,2]>0.903)&(P[:,2]<1.10)&(P[:,0]>-0.25)&(P[:,0]<0.0)
    Q=P[m]; print(cam,len(Q))
    for z0 in np.arange(0.90,1.06,0.01):
        s=Q[(Q[:,2]>z0)&(Q[:,2]<z0+0.01)]
        if len(s)<5: continue
        print(f"  z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 147
python3 step.py snap birdview bv_now.png >/dev/null 2>&1; python3 step.py snap agentview av_now.png >/dev/null 2>&1; echo ok

# openrua op 148
python3 step.py vert -0.09 -0.38 1.40 xm 4 2>&1 | grep -v approx | tail -2; python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.50)&(P[:,1]<-0.28)&(P[:,2]>0.903)&(P[:,2]<1.10)&(P[:,0]>-0.25)&(P[:,0]<0.05)
Q=P[m]; print(len(Q))
for y0 in np.arange(-0.50,-0.28,0.01):
    s=Q[(Q[:,1]>y0)&(Q[:,1]<y0+0.01)]
    if len(s)==0: continue
    h,e=np.histogram(s[:,0],bins=np.arange(-0.25,0.05,0.01))
    zmax=[s[(s[:,0]>e[i])&(s[:,0]<e[i+1]),2].max() if h[i] else 0 for i in range(len(h))]
    print(f"y {y0:.2f}", ' '.join(f"{(z-0.9)*100:4.1f}" if z else '   .' for z in zmax))
print('x bins', np.round(np.arange(-0.25,0.05,0.01),2).tolist())
EOF

# openrua op 149
python3 step.py pose -0.09 -0.58 1.05 70 0 1 4 2>&1 | grep -v approx | tail -2; python3 step.py snap robot0_eye_in_hand eih_side.png >/dev/null 2>&1; python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.50)&(P[:,1]<-0.30)&(P[:,2]>0.903)&(P[:,2]<1.10)&(P[:,0]>-0.25)&(P[:,0]<0.05)
Q=P[m]; print(len(Q))
# x-z map: for each z bin, x extents and y range
for z0 in np.arange(0.90,1.04,0.01):
    s=Q[(Q[:,2]>z0)&(Q[:,2]<z0+0.01)]
    if len(s)<5: continue
    h,e=np.histogram(s[:,0],bins=np.arange(-0.25,0.05,0.01))
    print(f"z {z0:.2f}", ' '.join(f"{int(v):4d}" if v else '   .' for v in h), f" y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 150
python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.46)&(P[:,1]<-0.31)&(P[:,2]>0.903)&(P[:,2]<1.06)&(P[:,0]>-0.16)&(P[:,0]<-0.02)
Q=P[m]
ys=np.arange(-0.46,-0.31,0.005)
print('      y:', ' '.join(f"{y:5.3f}"[1:] for y in ys))
for x0 in np.arange(-0.15,-0.02,0.01):
    s=Q[(Q[:,0]>x0)&(Q[:,0]<x0+0.01)]
    row=[]
    for y0 in ys:
        t=s[(s[:,1]>y0)&(s[:,1]<y0+0.005)]
        row.append(f"{(t[:,2].max()-0.9)*100:5.1f}" if len(t) else '    .')
    print(f"x {x0:.2f}:", ' '.join(row))
EOF

# openrua op 151
python3 step.py vert -0.20 0.05 1.30 xm 4 2>&1 | grep -v approx | tail -1; python3 step.py snap birdview bv2.png >/dev/null 2>&1; python3 step.py snap agentview av2.png >/dev/null 2>&1; python3 step.py snap sideview sv2.png >/dev/null 2>&1; echo done

# openrua op 152
python3 -c "
import cv2; im=cv2.imread('bv2.png'); c=im[230:310,170:250]; cv2.imwrite('bv2_crop.png', cv2.resize(c,None,fx=6,fy=6,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('av2.png'); cv2.imwrite('av2s.png', im)
"

# openrua op 153
python3 step.py pose -0.09 -0.50 1.06 55 0 1 4 2>&1 | grep -v approx | tail -2; python3 step.py snap robot0_eye_in_hand eih_low.png >/dev/null 2>&1; echo ok

# openrua op 154
python3 - <<'EOF' 2>&1 | grep -v approx
import numpy as np
from rob import Robot
from step import dir_quat
r=Robot("pp"); q0=r.q()
for th in [30,40,50]:
  for y in [-0.46,-0.50]:
    for z in [1.06,1.12]:
      for sx in [1,-1]:
        q=r.solve_ik([-0.09,y,z],dir_quat(th,0,sx),seed=q0,tries=6)
        print(th,y,z,sx,'OK' if q is not None else '--')
EOF

# openrua op 155
python3 - <<'EOF' 2>&1 | grep -v approx
import numpy as np
from rob import Robot
from step import dir_quat
r=Robot("pp"); q0=np.load('qpre.npy')
for th in [30,45,60]:
  for y in [-0.46,-0.52]:
    for z in [1.02,1.10]:
      for sx in [1,-1]:
        q=r.solve_ik([-0.09,y,z],dir_quat(th,0,sx),seed=q0,tries=6)
        print(th,y,z,sx,'OK' if q is not None else '--', np.round(q,2) if q is not None else '')
EOF

# openrua op 156
python3 - <<'EOF' 2>&1 | tail -5
import numpy as np
from rob import Robot
from step import dir_quat
r=Robot("pp"); q0=np.load('qpre.npy')
p,qu=r.tcp_pose(q0); print('qpre tcp',np.round(p,3),np.round(qu,3))
print('roundtrip', r.solve_ik(p,qu,seed=q0,tries=3) is not None)
print('dirquat30', np.round(dir_quat(30,-10,1),3))
print('same pos, dirquat(30,-10)', r.solve_ik(p,dir_quat(30,-10,1),seed=q0,tries=3) is not None)
print('cur q', np.round(r.q(),2))
EOF

# openrua op 157
python3 - <<'EOF' 2>&1 | grep -v approx
import numpy as np
from rob import Robot
from step import dir_quat
r=Robot("pp"); q0=np.load('qpre.npy')
for th in [30,45]:
  for yaw in [-10,-25,-40]:
    for y,z in [(-0.50,1.02),(-0.50,1.10),(-0.54,1.06)]:
      for sx in [1,-1]:
        q=r.solve_ik([-0.12,y,z],dir_quat(th,yaw,sx),seed=q0,tries=4)
        if q is not None: print(th,yaw,y,z,sx,'OK')
EOF

# openrua op 158
python3 step.py pose -0.12 -0.50 1.06 45 -25 1 5 2>&1 | grep -v approx | tail -2; python3 step.py snap robot0_eye_in_hand eih_low.png >/dev/null 2>&1; python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1; echo ok

# openrua op 159
python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.50)&(P[:,1]<-0.30)&(P[:,2]>0.903)&(P[:,2]<1.06)&(P[:,0]>-0.20)&(P[:,0]<0.02)
Q=P[m]; print(len(Q))
ys=np.arange(-0.50,-0.30,0.01)
print('       y:', ' '.join(f"{y:5.2f}"[1:] for y in ys))
for x0 in np.arange(-0.19,0.02,0.01):
    s=Q[(Q[:,0]>x0)&(Q[:,0]<x0+0.01)]
    row=[]
    for y0 in ys:
        t=s[(s[:,1]>y0)&(s[:,1]<y0+0.01)]
        row.append(f"{(t[:,2].max()-0.9)*100:5.1f}" if len(t) else '    .')
    print(f"x {x0:.2f}:", ' '.join(row))
EOF

# openrua op 160
python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
for xs in [(-0.10,-0.08),(-0.13,-0.115),(-0.06,-0.045)]:
    m=(P[:,0]>xs[0])&(P[:,0]<xs[1])&(P[:,1]>-0.50)&(P[:,1]<-0.30)&(P[:,2]>0.903)&(P[:,2]<1.06)
    Q=P[m]; print('x slab',xs,len(Q))
    for z0 in np.arange(0.90,1.03,0.01):
        s=Q[(Q[:,2]>z0)&(Q[:,2]<z0+0.01)]
        if len(s)<3: continue
        print(f"  z {z0:.2f}: y {s[:,1].min():.3f}..{s[:,1].max():.3f} n={len(s)}")
EOF

# openrua op 161
cat > /workspace/bargrasp.py <<'EOF'
#!/usr/bin/env python3
"""Vertical pinch of a horizontal bar lying along direction (bx,by): closing axis n = perp."""
import sys, numpy as np
from rob import Robot, quat_from_axes
r = Robot("bar")
cx, cy, bx, by, z, off = map(float, sys.argv[1:7])   # bar center, bar dir, tcp z, tcp offset along n (neg = toward -n)
mode = sys.argv[7] if len(sys.argv) > 7 else "run"
b = np.array([bx, by, 0]); b /= np.linalg.norm(b)
n = np.array([-b[1], b[0], 0])
if n[1] < 0: n = -n            # n points +y-ish
quat = quat_from_axes([0, 0, -1], b)   # hand x = bar dir, hand y (closing) = +-n
tcp = np.array([cx, cy, 0]) + off * n
def go(p, sec=4.0, tries=3):
    q = r.solve_ik(p, quat, seed=r.q(), tries=10)
    if q is None: print("IK FAIL", p); sys.exit(1)
    for a in range(tries):
        code, err = r.move_joints(q, sec)
        if err < 0.01: break
    pn, _ = r.tcp_pose(); print("at", np.round(pn, 4).tolist(), "code", code, "err", round(err, 4), "fingers", np.round(r.fingers(), 4).tolist())
    return pn
print("n", np.round(n, 3), "tcp xy", np.round(tcp[:2], 4))
if mode == "check":
    for zz in [1.03, z]:
        q = r.solve_ik([tcp[0], tcp[1], zz], quat, seed=r.q(), tries=10); print(zz, q is not None)
    sys.exit()
go([tcp[0], tcp[1], 1.03])
p = go([tcp[0], tcp[1], z], 5.0)
if abs(p[2] - z) > 0.006 or np.linalg.norm(p[:2] - tcp[:2]) > 0.004:
    p = go([tcp[0], tcp[1], z] - (p - [tcp[0], tcp[1], z]), 3.0)
if mode == "down": sys.exit()
print("close", r.gripper(0.0))
go([tcp[0], tcp[1], z + 0.04], 3.0)
print("fingers after lift", np.round(r.fingers(), 4).tolist())
EOF
python3 step.py grip open 2>&1 | grep -v approx; python3 bargrasp.py -0.09 -0.42 0.98 -0.17 0.91 -0.035 check 2>&1 | grep -v approx

# openrua op 162
python3 bargrasp.py -0.09 -0.42 0.98 -0.17 0.91 -0.035 down 2>&1 | grep -v approx

# openrua op 163
python3 step.py state 2>&1 | grep -v approx; python3 - <<'EOF' 2>&1 | grep -v approx
import numpy as np
from rob import Robot, quat_from_axes, LIMITS
r=Robot("x")
b=np.array([0.98,-0.17,0]); b/=np.linalg.norm(b)
quat=quat_from_axes([0,0,-1],b)
q=r.solve_ik([-0.096,-0.4545,0.91],quat,seed=r.q(),tries=10)
print("q_target",np.round(q,3)); print("limits lo",LIMITS[:,0]); print("limits hi",LIMITS[:,1])
print("cur",np.round(r.q(),3))
EOF

# openrua op 164
timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | head -20; python3 - <<'EOF' 2>&1 | grep -v approx
import numpy as np
from rob import Robot
r=Robot("x"); q=r.q()
for l in ["panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]:
    print(l, np.round(r.fk_pose(q,l)[0],3))
EOF

# openrua op 165
python3 step.py snap robot0_eye_in_hand eih_now.png >/dev/null 2>&1; python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.60)&(P[:,1]<-0.30)&(P[:,2]>0.903)&(P[:,2]<1.06)&(P[:,0]>-0.25)&(P[:,0]<0.05)
Q=P[m]; print(len(Q))
ys=np.arange(-0.60,-0.30,0.01)
print('       y:', ' '.join(f"{y:5.2f}"[1:] for y in ys))
for x0 in np.arange(-0.24,0.04,0.01):
    s=Q[(Q[:,0]>x0)&(Q[:,0]<x0+0.01)]
    if len(s)==0: continue
    row=[]
    for y0 in ys:
        t=s[(s[:,1]>y0)&(s[:,1]<y0+0.01)]
        row.append(f"{(t[:,2].max()-0.9)*100:5.1f}" if len(t) else '    .')
    print(f"x {x0:.2f}:", ' '.join(row))
EOF

# openrua op 166
python3 - <<'EOF' 2>&1 | grep -v approx
import numpy as np
from rob import Robot, quat_from_axes
r=Robot("x")
b=np.array([0.98,-0.17,0]); b/=np.linalg.norm(b)
quat=quat_from_axes([0,0,-1],b)
tgt=np.array([-0.096,-0.4545,0.96])
for i in range(3):
    q=r.solve_ik(tgt,quat,seed=r.q(),tries=10)
    code,err=r.move_joints(q,6.0)
    p,_=r.tcp_pose(); print(i,"code",code,"err",round(err,4),"tcp",np.round(p,4),"q",np.round(r.q(),3))
    if err<0.01: break
EOF

# openrua op 167
python3 - <<'EOF'
s=open('/workspace/bargrasp.py').read()
s=s.replace('''def go(p, sec=4.0, tries=3):
    q = r.solve_ik(p, quat, seed=r.q(), tries=10)
    if q is None: print("IK FAIL", p); sys.exit(1)
    for a in range(tries):
        code, err = r.move_joints(q, sec)
        if err < 0.01: break
    pn, _ = r.tcp_pose(); print("at", np.round(pn, 4).tolist(), "code", code, "err", round(err, 4), "fingers", np.round(r.fingers(), 4).tolist())
    return pn''','''def go(p, sec=4.0, tries=3):
    p = np.array(p, float); cmd = p.copy()
    for it in range(5):
        q = r.solve_ik(cmd, quat, seed=r.q(), tries=10)
        if q is None: print("IK FAIL", cmd); sys.exit(1)
        for a in range(tries):
            code, err = r.move_joints(q, sec if it == 0 else 3.0)
            if err < 0.03: break
        pn, _ = r.tcp_pose(); off = pn - p
        print(f"  it{it} at {np.round(pn,4).tolist()} code {code} err {round(err,4)} off {np.round(off,4).tolist()}")
        if np.linalg.norm(off) < 0.003: break
        cmd = cmd - off
    print("at", np.round(pn, 4).tolist(), "fingers", np.round(r.fingers(), 4).tolist())
    return pn''')
open('/workspace/bargrasp.py','w').write(s)
EOF
python3 bargrasp.py -0.09 -0.42 0.98 -0.17 0.91 -0.035 down 2>&1 | grep -v approx

# openrua op 168
python3 step.py state 2>&1 | grep -v approx; python3 step.py vert -0.10 -0.50 1.10 y 4 2>&1 | grep -v approx | tail -2

# openrua op 169
cat > /workspace/hmap.py <<'EOF'
import sys, numpy as np
cam = sys.argv[1]; x0,x1,y0,y1 = map(float, sys.argv[2:6]); step = float(sys.argv[6]) if len(sys.argv)>6 else 0.01
P=np.load(f'{cam}_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.903)&(P[:,2]<1.10)&(P[:,0]>x0)&(P[:,0]<x1)
Q=P[m]; print(len(Q))
ys=np.arange(y0,y1,step)
print('       y:', ' '.join(f"{abs(y):5.3f}"[1:] for y in ys))
for xa in np.arange(x0,x1,step):
    s=Q[(Q[:,0]>xa)&(Q[:,0]<xa+step)]
    if len(s)==0: continue
    row=[]
    for ya in ys:
        t=s[(s[:,1]>ya)&(s[:,1]<ya+step)]
        row.append(f"{(t[:,2].max()-0.9)*100:5.1f}" if len(t) else '    .')
    print(f"x {xa:.3f}:", ' '.join(row))
EOF
python3 step.py vert -0.09 -0.38 1.40 xm 4 2>&1 | grep -v approx | tail -1; python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1; python3 hmap.py robot0_eye_in_hand -0.20 0.0 -0.50 -0.30

# openrua op 170
python3 - <<'EOF' 2>&1 | grep -v approx
import numpy as np
from rob import Robot
from step import dir_quat
r=Robot("pp"); q0=r.q()
for th in [30,40,50]:
  for yaw in [90,70,50]:
    for tcp in [(-0.02,-0.375,1.00),(0.0,-0.375,1.01)]:
      for sx in [1,-1]:
        q=r.solve_ik(tcp,dir_quat(th,yaw,sx),seed=q0,tries=5)
        if q is not None:
            p7=r.fk_pose(q,'panda_link7')[0]; ph=r.fk_pose(q,'panda_hand')[0]
            print(th,yaw,tcp,sx,'OK l7',np.round(p7,3),'hand',np.round(ph,3))
EOF

# openrua op 171
python3 - <<'EOF' 2>&1 | grep -v approx | tail -20
import numpy as np
from rob import Robot
from step import dir_quat
r=Robot("pp"); q0=r.q()
n=0
for th in [30,40,50]:
  for yaw in [90,70,50]:
    for tcp in [(-0.02,-0.375,1.00),(0.0,-0.375,1.01)]:
      for sx in [1,-1]:
        q=r.solve_ik(tcp,dir_quat(th,yaw,sx),seed=q0,tries=5)
        if q is not None:
            n+=1
            p7=r.fk_pose(q,'panda_link7')[0]; ph=r.fk_pose(q,'panda_hand')[0]
            print(th,yaw,tcp,sx,'OK l7',np.round(p7,3),'hand',np.round(ph,3))
print("found",n)
EOF

# openrua op 172
python3 - <<'EOF' 2>&1 | grep -v approx | tail -30
import numpy as np
from rob import Robot
from step import dir_quat
r=Robot("pp"); q0=r.q()
n=0
for th in [30,40,50]:
  for yaw in [90,70]:
    for x in [-0.10,-0.14,-0.18]:
      for sx in [1,-1]:
        tcp=(x,-0.375,1.00)
        q=r.solve_ik(tcp,dir_quat(th,yaw,sx),seed=q0,tries=5)
        if q is not None:
            n+=1
            p7=r.fk_pose(q,'panda_link7')[0]; ph=r.fk_pose(q,'panda_hand')[0]
            print(th,yaw,tcp,sx,'OK l7',np.round(p7,3),'hand',np.round(ph,3))
print("found",n)
EOF

# openrua op 173
python3 hmap.py robot0_eye_in_hand -0.14 -0.02 -0.47 -0.38 0.005 | cut -c1-200

# openrua op 174
python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,1]>-0.385)&(P[:,1]<-0.365)&(P[:,2]>0.903)&(P[:,2]<1.06)&(P[:,0]>-0.20)&(P[:,0]<0.02)
Q=P[m]
zs=np.arange(1.04,0.90,-0.01)
xs=np.arange(-0.20,0.02,0.005)
print('x:', ' '.join(f"{x*100:4.0f}" for x in xs))
for z0 in zs:
    row=''
    for x0 in xs:
        t=Q[(Q[:,0]>x0)&(Q[:,0]<x0+0.005)&(Q[:,2]>z0)&(Q[:,2]<z0+0.01)]
        row+=f"{len(t):4d}" if len(t) else '   .'
    print(f"z {z0:.2f}", row)
EOF

# openrua op 175
python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
for xc in [-0.12,-0.09,-0.06,-0.02,0.03]:
    m=(P[:,0]>xc-0.005)&(P[:,0]<xc+0.005)&(P[:,2]>0.903)&(P[:,2]<1.06)&(P[:,1]>-0.50)&(P[:,1]<-0.30)
    Q=P[m]
    ys=np.arange(-0.50,-0.30,0.005)
    print(f"=== x={xc}   y:", ' '.join(f"{y*100:4.0f}" for y in ys))
    for z0 in np.arange(1.03,0.90,-0.01):
        row=''
        for y0 in ys:
            t=Q[(Q[:,1]>y0)&(Q[:,1]<y0+0.005)&(Q[:,2]>z0)&(Q[:,2]<z0+0.01)]
            row+=f"{len(t):4d}" if len(t) else '   .'
        print(f"z {z0:.2f}", row)
EOF

# openrua op 176
python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# for each x column, find the most -y point of the microwave face region (z 0.95..1.10, y>-0.40)
for xc in np.arange(-0.26,0.12,0.02):
    m=(P[:,0]>xc-0.01)&(P[:,0]<xc+0.01)&(P[:,2]>0.95)&(P[:,2]<1.10)&(P[:,1]>-0.40)&(P[:,1]<-0.25)
    Q=P[m]
    if len(Q)==0: print(f"x {xc:.2f}: none"); continue
    ys=np.percentile(Q[:,1],[1,5,50]); zs=np.percentile(Q[:,2],[1,50,99])
    print(f"x {xc:.2f}: n={len(Q)} y1%={ys[0]:.3f} y5%={ys[1]:.3f} ymed={ys[2]:.3f}  z1%={zs[0]:.3f} zmed={zs[1]:.3f} z99={zs[2]:.3f}")
EOF

# openrua op 177
python3 step.py grip close && python3 step.py vert -0.012 -0.39 1.10 xm 4 && python3 step.py vert -0.012 -0.39 1.005 xm 4

# openrua op 178
python3 step.py vert -0.05 -0.39 1.005 xm 4 && python3 step.py vert -0.09 -0.39 1.005 xm 4

# openrua op 179
python3 step.py vert -0.09 -0.39 1.30 xm 4 >/dev/null && python3 step.py vert -0.15 -0.40 1.40 xm 4 >/dev/null && python3 cloud.py robot0_eye_in_hand && python3 hmap.py robot0_eye_in_hand -0.32 0.02 -0.52 -0.30 0.01

# openrua op 180
python3 step.py snap robot0_eye_in_hand eih_push1.png && python3 step.py snap birdview bv_push1.png

# openrua op 181
python3 -c "
import cv2
im=cv2.imread('bv_push1.png'); print(im.shape)
# crop around mug: world x -0.25..0.05 -> img_y 17..105 ; y -0.55..-0.25 -> img_x 160..248
c=im[0:140,120:300]; c=cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC); cv2.imwrite('bv_push1_crop.png',c)"

# openrua op 182
cat > slice.py <<'EOF'
#!/usr/bin/env python3
"""slice.py <cam> x|y c0 [c1 ...] : print z-vs-(other axis) occupancy slices of <cam>_world.npy at coordinate(s) c (half-width 5mm)"""
import sys, numpy as np
cam, ax = sys.argv[1], sys.argv[2]
P = np.load(f'{cam}_world.npy').reshape(-1, 3); P = P[np.isfinite(P).all(1)]
i = 0 if ax == 'x' else 1; j = 1 - i
lo, hi = (-0.52, -0.28) if j == 1 else (-0.30, 0.04)
for c in map(float, sys.argv[3:]):
    m = (np.abs(P[:, i] - c) < 0.005) & (P[:, 2] > 0.903) & (P[:, 2] < 1.06) & (P[:, j] > lo) & (P[:, j] < hi)
    Q = P[m]; cs = np.arange(lo, hi, 0.005)
    print(f"=== {ax}={c:.3f}   {'y' if j else 'x'}:", ' '.join(f"{v*100:4.0f}" for v in cs))
    for z0 in np.arange(1.03, 0.90, -0.01):
        row = ''
        for c0 in cs:
            t = Q[(Q[:, j] > c0) & (Q[:, j] < c0 + 0.005) & (Q[:, 2] > z0) & (Q[:, 2] < z0 + 0.01)]
            row += f"{len(t):4d}" if len(t) else '   .'
        print(f"z {z0:.2f}", row)
EOF
python3 slice.py robot0_eye_in_hand x -0.14 -0.10 -0.07 -0.05 -0.03

# openrua op 183
python3 step.py vert -0.05 -0.39 1.10 xm 4 | tail -2 && python3 step.py vert -0.05 -0.39 0.99 xm 4 | tail -2 && python3 step.py vert -0.08 -0.39 0.99 xm 3 | tail -3

# openrua op 184
python3 step.py vert -0.11 -0.39 0.99 xm 3 | grep -E "move code|corr|tcp now" && python3 step.py vert -0.14 -0.39 0.99 xm 3 | grep -E "move code|corr|tcp now"

# openrua op 185
python3 step.py vert -0.14 -0.39 1.30 xm 4 >/dev/null && python3 step.py vert -0.18 -0.40 1.40 xm 4 >/dev/null && python3 cloud.py robot0_eye_in_hand && python3 hmap.py robot0_eye_in_hand -0.32 0.0 -0.52 -0.30 0.01

# openrua op 186
python3 hmap.py robot0_eye_in_hand -0.17 -0.09 -0.48 -0.40 0.005 && python3 slice.py robot0_eye_in_hand x -0.13 -0.12

# openrua op 187
cat > topbar.py <<'EOF'
#!/usr/bin/env python3
"""topbar.py: vertical pinch of the mug's top handle bar, lift, rotate/translate, set down.
usage: topbar.py cx cy az_deg  ncx ncy naz_deg [mode]   (mode: check|grasp|run)
cx,cy: mug axis; az: handle azimuth (deg, from +x). new center/azimuth for set-down."""
import sys, numpy as np
from rob import Robot, quat_from_axes

cx, cy, az, ncx, ncy, naz = map(float, sys.argv[1:7])
mode = sys.argv[7] if len(sys.argv) > 7 else "check"
R_BAR = 0.062          # top bar centre radius
Z_GRASP = 0.968
OFF = 10.0             # closing axis rotated this many deg from perpendicular (hand body away from the face)

def hand_quat(az_deg):
    u = np.array([np.cos(np.radians(az_deg)), np.sin(np.radians(az_deg)), 0])      # bar direction (outward)
    xh = np.array([np.cos(np.radians(az_deg - OFF)), np.sin(np.radians(az_deg - OFF)), 0])
    return quat_from_axes([0, 0, -1], xh), u

r = Robot("topbar")
q_now = r.q()
quat, u = hand_quat(az)
tcp = np.array([cx, cy, 0]) + R_BAR * u
nquat, nu = hand_quat(naz)
ntcp = np.array([ncx, ncy, 0]) + R_BAR * nu
print("grasp tcp", np.round(tcp, 4), "new tcp", np.round(ntcp, 4))

def go(p, qu, sec=3.0, tol=0.003, iters=4):
    tgt = np.array(p, float); q = None
    for it in range(iters):
        cur, _ = r.tcp_pose()
        if it == 0:
            cmd = tgt.copy()
        else:
            off = cur - tgt
            if np.linalg.norm(off) < tol:
                break
            cmd = cmd - off
        q = r.solve_ik(cmd, qu, seed=r.q(), tries=10)
        if q is None:
            print("  IK FAIL at", np.round(cmd, 4)); return False
        code, err = r.move_joints(q, seconds=sec if it == 0 else 2.0)
        cur, _ = r.tcp_pose()
        print(f"  it{it} code {code} err {err:.4f} tcp {np.round(cur,4)} off {np.round(cur-tgt,4)}")
    return True

poses = [("pre", [tcp[0], tcp[1], 1.10], quat), ("grasp", [tcp[0], tcp[1], Z_GRASP], quat),
         ("lift", [tcp[0], tcp[1], Z_GRASP + 0.04], quat), ("rot", [ntcp[0], ntcp[1], Z_GRASP + 0.04], nquat),
         ("down", [ntcp[0], ntcp[1], Z_GRASP + 0.002], nquat), ("up", [ntcp[0], ntcp[1], 1.10], nquat)]
if mode == "check":
    seed = q_now
    for name, p, qu in poses:
        q = r.solve_ik(p, qu, seed=seed, tries=10)
        print(name, np.round(p, 3), "OK" if q is not None else "IK FAIL", np.round(q, 2) if q is not None else "")
        if q is not None: seed = q
    sys.exit()

print("open", r.gripper(0.08))
for name, p, qu in poses:
    print(name, np.round(p, 4))
    ok = go(p, qu, sec=4.0 if name in ("pre", "up") else 3.0)
    if not ok: sys.exit(1)
    if name == "grasp":
        if mode == "grasp":
            sys.exit()
        print("close", r.gripper(0.0))
        f = r.fingers()
        if abs(f[0]) < 0.006:
            print("GRASP MISSED (fingers %.4f), opening" % f[0]); r.gripper(0.08); sys.exit(1)
    if name == "lift":
        print("fingers after lift", r.fingers())
    if name == "down":
        print("open", r.gripper(0.08))
print("done", r.fingers())
EOF
python3 topbar.py -0.183 -0.385 -40  -0.21 -0.382 -90 check

# openrua op 188
python3 topbar.py -0.183 -0.385 -40  -0.21 -0.382 -90 run 2>&1 | grep -v "^\[WARN"

# openrua op 189
python3 step.py vert -0.21 -0.45 1.40 xm 4 >/dev/null && python3 cloud.py robot0_eye_in_hand && python3 hmap.py robot0_eye_in_hand -0.30 -0.10 -0.52 -0.30 0.01

# openrua op 190
python3 slice.py robot0_eye_in_hand x -0.20 -0.21 2>&1 | head -80

# openrua op 191
cat > /workspace/probe4.py <<'EOF'
import numpy as np, sys
from rob import Robot
from step import tilt_quat
r = Robot("probe4")
q0 = r.q()
for th in [30, 35, 40]:
    quat = tilt_quat(th, -1)
    seed = q0
    for name, tip in [("above", (-0.203, -0.480, 1.02)), ("grasp", (-0.203, -0.480, 0.950)),
                      ("lift", (-0.203, -0.480, 1.20)), ("carry", (-0.162, -0.475, 1.20)),
                      ("down", (-0.162, -0.475, 1.005)), ("in", (-0.162, -0.311, 1.005)), ("rel", (-0.162, -0.311, 0.994))]:
        q = r.solve_ik(tip, quat, seed=seed, tries=8)
        if q is None:
            print(f"th={th} {name}: --"); sys.stdout.flush(); continue
        seed = q
        p7, _ = r.fk_pose(q, "panda_link7")
        print(f"th={th} {name}: q={np.round(q,2)} l7={np.round(p7,3)}"); sys.stdout.flush()
EOF
python3 probe4.py 2>&1 | grep -v WARN

# openrua op 192
sed -n 1,60p /workspace/topbar.py

# openrua op 193
cat > /workspace/outerbar.py <<'EOF'
#!/usr/bin/env python3
"""outerbar.py: tilted (th deg, fingers pointing +y-down) pinch of the handle's outer bar.
usage: outerbar.py grasp gx gy gz        -> open, above, grasp, close, lift 4cm
       outerbar.py carry                  -> lift, carry, down, in, rel(open), retreat
       outerbar.py go x y z [sec]         -> single move with current quat"""
import sys, numpy as np
from rob import Robot
from step import tilt_quat

TH = 30
quat = tilt_quat(TH, -1)
r = Robot("outerbar")

def go(p, sec=3.0, tol=0.003, iters=4):
    tgt = np.array(p, float); cmd = tgt.copy()
    for it in range(iters):
        if it > 0:
            cur, _ = r.tcp_pose(); off = cur - tgt
            if np.linalg.norm(off) < tol: break
            cmd = cmd - off
        q = r.solve_ik(cmd, quat, seed=r.q(), tries=10)
        if q is None:
            print("  IK FAIL at", np.round(cmd, 4)); return False
        code, err = r.move_joints(q, seconds=sec if it == 0 else 2.0)
        cur, _ = r.tcp_pose()
        print(f"  it{it} code {code} err {err:.4f} tcp {np.round(cur,4)} off {np.round(cur-tgt,4)} fingers {np.round(r.fingers(),4)}")
        sys.stdout.flush()
    return True

def line(p0, p1, step=0.02, sec_per=1.0):
    """straight-line cartesian move in small steps"""
    p0 = np.array(p0, float); p1 = np.array(p1, float)
    n = max(1, int(np.ceil(np.linalg.norm(p1 - p0) / step)))
    for i in range(1, n + 1):
        p = p0 + (p1 - p0) * i / n
        q = r.solve_ik(p, quat, seed=r.q(), tries=10)
        if q is None:
            print("  IK FAIL at", np.round(p, 4)); return False
        code, err = r.move_joints(q, seconds=sec_per)
        if code != 0 and err > 0.02:
            cur, _ = r.tcp_pose(); print(f"  step {i}/{n} code {code} err {err:.4f} tcp {np.round(cur,4)}"); return False
    cur, _ = r.tcp_pose(); print(f"  line end tcp {np.round(cur,4)} fingers {np.round(r.fingers(),4)}")
    return True

mode = sys.argv[1]
if mode == "grasp":
    gx, gy, gz = map(float, sys.argv[2:5])
    print("open", r.gripper(0.08))
    print("above"); go([gx, gy, 1.02], sec=4.0)
    print("grasp"); go([gx, gy, gz], sec=3.0)
    print("close", r.gripper(0.0))
    f = r.fingers()
    if min(f) < 0.006:
        print("BAD GRASP, opening"); print(r.gripper(0.08)); sys.exit(1)
    print("lift"); go([gx, gy, gz + 0.04], sec=2.0)
    print("fingers", r.fingers())
elif mode == "go":
    x, y, z = map(float, sys.argv[2:5]); sec = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
    go([x, y, z], sec=sec)
elif mode == "line":
    x, y, z = map(float, sys.argv[2:5])
    cur, _ = r.tcp_pose(); line(cur, [x, y, z])
EOF
python3 outerbar.py grasp -0.203 -0.480 0.950 2>&1 | grep -v WARN

# openrua op 194
sed -i 's/if min(f) < 0.006:/if min(abs(f[0]), abs(f[1])) < 0.006:/' outerbar.py && python3 outerbar.py grasp -0.203 -0.480 0.950 2>&1 | grep -v WARN

# openrua op 195
python3 cloud.py frontview >/dev/null && python3 hmap.py frontview -0.30 -0.10 -0.52 -0.30 0.01

# openrua op 196
cat >> /workspace/outerbar.py <<'EOF'
elif mode == "carry":
    print("lift"); go([-0.203, -0.480, 1.20], sec=4.0)
    print("carry"); go([-0.162, -0.475, 1.20], sec=3.0)
    print("down"); go([-0.162, -0.475, 1.005], sec=4.0)
    print("in"); ok = line([-0.162, -0.475, 1.005], [-0.162, -0.335, 1.005], step=0.02, sec_per=1.0)
    if not ok: sys.exit(1)
    go([-0.162, -0.335, 1.005], sec=1.0)
    print("fingers", r.fingers())
EOF
python3 outerbar.py carry 2>&1 | grep -v WARN

# openrua op 197
python3 step.py state 2>&1 | grep -v WARN; python3 cloud.py frontview >/dev/null && python3 hmap.py frontview -0.30 -0.06 -0.52 -0.30 0.01 && python3 slice.py frontview x -0.16 -0.14 2>&1 | head -40

# openrua op 198
for c in sideview agentview; do python3 cloud.py $c >/dev/null && echo "== $c" && python3 hmap.py $c -0.30 -0.06 -0.52 -0.30 0.01; done

# openrua op 199
python3 step.py snap robot0_eye_in_hand eih_ins1.png 2>&1 | grep -v WARN; python3 step.py snap frontview fv_ins1.png 2>&1 | grep -v WARN

# openrua op 200
python3 step.py snap sideview sv_ins1.png 2>&1 | grep -v WARN; python3 step.py snap agentview av_ins1.png 2>&1 | grep -v WARN

# openrua op 201
python3 outerbar.py line -0.162 -0.475 1.03 2>&1 | grep -v WARN; python3 outerbar.py go -0.162 -0.475 1.10 3 2>&1 | grep -v WARN

# openrua op 202
python3 cloud.py robot0_eye_in_hand && python3 slice.py robot0_eye_in_hand x -0.10 -0.16 -0.06 2>&1 | head -50

# openrua op 203
python3 cloud.py frontview >/dev/null && python3 hmap.py frontview -0.26 -0.08 -0.52 -0.30 0.01 && python3 slice.py frontview x -0.16 2>&1 | head -3; python3 - <<'EOF'
import numpy as np
P=np.load('/workspace/frontview_world.npy').reshape(-1,3)
m=(P[:,0]>-0.24)&(P[:,0]<-0.08)&(P[:,1]>-0.50)&(P[:,1]<-0.30)&(P[:,2]>1.0)&(P[:,2]<1.25)
Q=P[m]
for y0 in np.arange(-0.50,-0.30,0.01):
    s=Q[(Q[:,1]>=y0)&(Q[:,1]<y0+0.01)]
    if len(s): print(f"y {y0:.2f}: n={len(s)} zmin={s[:,2].min():.3f} zmax={s[:,2].max():.3f} xrange=({s[:,0].min():.3f},{s[:,0].max():.3f})")
EOF

# openrua op 204
python3 outerbar.py go -0.203 -0.480 1.03 3 2>&1 | grep -v WARN; for z in 1.00 0.99 0.98 0.97 0.96; do python3 outerbar.py go -0.203 -0.480 $z 1.5 2>&1 | grep -v WARN | head -1; done

# openrua op 205
python3 step.py grip open 2>&1 | grep -v WARN; python3 outerbar.py go -0.203 -0.50 1.08 2 2>&1 | grep -v WARN; python3 step.py vert -0.21 -0.45 1.40 xm 4 2>&1 | grep -v WARN | tail -2; python3 cloud.py robot0_eye_in_hand >/dev/null && python3 hmap.py robot0_eye_in_hand -0.30 -0.10 -0.54 -0.30 0.01

# openrua op 206
python3 slice.py robot0_eye_in_hand y -0.40 -0.42 2>&1 | head -32; python3 slice.py robot0_eye_in_hand x -0.16 2>&1 | head -16

# openrua op 207
python3 step.py vert -0.163 -0.415 1.06 xm 4 2>&1 | grep -v WARN | tail -2 && python3 step.py vert -0.163 -0.415 1.005 xm 2 2>&1 | grep -v WARN | tail -2 && python3 step.py grip close 2>&1 | grep -v WARN

# openrua op 208
python3 step.py vert -0.163 -0.415 1.04 xm 2 2>&1 | grep -v WARN | tail -2; python3 cloud.py frontview >/dev/null && python3 hmap.py frontview -0.26 -0.08 -0.50 -0.30 0.01

# openrua op 209
python3 step.py vert -0.163 -0.415 1.09 xm 2 2>&1 | grep -v WARN | tail -1; python3 step.py vert -0.09 -0.43 1.09 ym 5 2>&1 | grep -v WARN | tail -3

# openrua op 210
python3 cloud.py frontview >/dev/null && python3 hmap.py frontview -0.24 0.0 -0.52 -0.34 0.01

# openrua op 211
python3 step.py snap frontview fv_hang.png 2>&1 | grep -v WARN; python3 step.py snap agentview av_hang.png 2>&1 | grep -v WARN

# openrua op 212
for z in 1.03 1.02 1.012; do python3 step.py vert -0.09 -0.43 $z ym 2 2>&1 | grep -v WARN | grep -E "move code|fingers" | tr '\n' ' '; echo; done

# openrua op 213
for z in 1.004 0.996; do python3 step.py vert -0.09 -0.43 $z ym 2 2>&1 | grep -v WARN | grep -E "move code|fingers" | tr '\n' ' '; echo; done

# openrua op 214
python3 step.py grip open 2>&1 | grep -v WARN; python3 step.py vert -0.09 -0.43 1.10 ym 2 2>&1 | grep -v WARN | tail -1; python3 step.py vert -0.12 -0.45 1.40 xm 4 2>&1 | grep -v WARN | tail -1; python3 cloud.py robot0_eye_in_hand >/dev/null && python3 hmap.py robot0_eye_in_hand -0.26 0.04 -0.52 -0.34 0.01

# openrua op 215
python3 step.py grip close 2>&1 | grep -v WARN; python3 step.py vert -0.19 -0.44 1.06 ym 4 2>&1 | grep -v WARN | tail -1; python3 step.py vert -0.19 -0.44 0.935 ym 2 2>&1 | grep -v WARN | tail -1; for x in -0.16 -0.13; do python3 step.py vert $x -0.44 0.935 ym 2 2>&1 | grep -v WARN | grep -E "move code|corr|tcp now" | tr '\n' ' '; echo; done

# openrua op 216
python3 step.py vert -0.13 -0.44 1.10 ym 2 2>&1 | grep -v WARN | tail -1; python3 step.py vert -0.12 -0.45 1.40 xm 4 2>&1 | grep -v WARN | tail -1; python3 cloud.py robot0_eye_in_hand >/dev/null && python3 hmap.py robot0_eye_in_hand -0.22 0.04 -0.52 -0.34 0.01

# openrua op 217
python3 step.py state 2>&1 | grep -v WARN; python3 cloud.py robot0_eye_in_hand && python3 hmap.py robot0_eye_in_hand -0.22 0.04 -0.52 -0.34 0.01

# openrua op 218
python3 - <<'EOF' 2>&1 | grep -v WARN
import numpy as np
from rob import Robot, quat_from_axes
r = Robot("fix")
print("q now", np.round(r.q(),3))
seed = np.array([-1.36, 1.02, 0.7, -1.27, -0.24, 1.73, 1.62])
quat = quat_from_axes([0,0,-1],[0,-1,0])
q = r.solve_ik([-0.12,-0.45,1.40], quat, seed=seed, tries=10)
print("q target", np.round(q,3))
if q is not None:
    code, err = r.move_joints(q, seconds=8.0)
    print("code", code, "err", round(err,4), "tcp", np.round(r.tcp_pose()[0],4))
EOF

# openrua op 219
python3 - <<'EOF' 2>&1 | grep -v WARN
import numpy as np
from rob import Robot, quat_from_axes
r = Robot("fix2")
q0 = r.q(); print("q now", np.round(q0,3), "tcp", np.round(r.tcp_pose()[0],4))
tgt = np.array([-1.012, 0.762, 0.8, -0.652, -0.543, 1.284, 1.815])
for i in range(4):
    code, err = r.move_joints(tgt, seconds=6.0)
    q = r.q()
    print(i, "code", code, "err", round(err,4), "q", np.round(q,3), "tcp", np.round(r.tcp_pose()[0],4))
    if err < 0.01: break
EOF

# openrua op 220
python3 cloud.py robot0_eye_in_hand >/dev/null && python3 hmap.py robot0_eye_in_hand -0.30 0.04 -0.56 -0.30 0.01

# openrua op 221
sleep 2; python3 cloud.py robot0_eye_in_hand >/dev/null && python3 hmap.py robot0_eye_in_hand -0.30 0.04 -0.56 -0.30 0.01

# openrua op 222
python3 step.py vert 0.01 -0.435 1.06 xm 5 2>&1 | grep -v WARN | grep -E "move code|tcp now"; python3 step.py vert 0.01 -0.435 0.985 xm 2 2>&1 | grep -v WARN | grep -E "move code|tcp now"; for x in -0.02 -0.05 -0.08 -0.10; do python3 step.py vert $x -0.435 0.985 xm 2 2>&1 | grep -v WARN | grep -E "move code|tcp now" | tr '\n' ' '; echo; done

# openrua op 223
python3 step.py vert -0.10 -0.435 1.10 xm 2 2>&1 | grep -v WARN | grep "tcp now"; python3 step.py vert -0.12 -0.45 1.40 xm 6 2>&1 | grep -v WARN | grep -E "move code|tcp now"; python3 cloud.py robot0_eye_in_hand >/dev/null && python3 hmap.py robot0_eye_in_hand -0.30 0.02 -0.54 -0.32 0.01

# openrua op 224
python3 topbar.py -0.16 -0.445 193 -0.203 -0.395 -90 check 2>&1 | grep -v WARN

# openrua op 225
cat > /workspace/ikbest.py <<'EOF'
import numpy as np
from rob import LIMITS

def ik_best(r, pos, quat, ref, n=12, extra_seeds=()):
    """collect IK solutions from several seeds, return the one closest (weighted L-inf/L2) to ref."""
    sols = []
    seeds = [np.array(ref)] + [np.array(s) for s in extra_seeds]
    rng = np.random.default_rng(0)
    for i in range(n):
        seeds.append(np.clip(np.array(ref) + rng.uniform(-0.6, 0.6, 7), LIMITS[:, 0], LIMITS[:, 1]))
    for s in seeds:
        q = r.solve_ik(pos, quat, seed=s, tries=1)
        if q is not None:
            sols.append(q)
    if not sols:
        return None, None
    ref = np.array(ref)
    d = [np.linalg.norm(q - ref) + 2 * np.abs(q - ref).max() for q in sols]
    i = int(np.argmin(d))
    return sols[i], d[i]
EOF
cat > /workspace/topbar2.py <<'EOF'
#!/usr/bin/env python3
"""topbar2.py cx cy az ncx ncy naz [check|run] -- like topbar.py but with configuration-consistent IK."""
import sys, numpy as np
from rob import Robot, quat_from_axes
from ikbest import ik_best

cx, cy, az, ncx, ncy, naz = map(float, sys.argv[1:7])
mode = sys.argv[7] if len(sys.argv) > 7 else "check"
R_BAR = 0.062; Z_GRASP = 0.968; OFF = 10.0

def hand_quat(az_deg):
    u = np.array([np.cos(np.radians(az_deg)), np.sin(np.radians(az_deg)), 0])
    xh = np.array([np.cos(np.radians(az_deg - OFF)), np.sin(np.radians(az_deg - OFF)), 0])
    return quat_from_axes([0, 0, -1], xh), u

r = Robot("topbar2")
quat, u = hand_quat(az); tcp = np.array([cx, cy, 0]) + R_BAR * u
nquat, nu = hand_quat(naz); ntcp = np.array([ncx, ncy, 0]) + R_BAR * nu
poses = [("pre", [tcp[0], tcp[1], 1.10], quat), ("grasp", [tcp[0], tcp[1], Z_GRASP], quat),
         ("lift", [tcp[0], tcp[1], Z_GRASP + 0.04], quat), ("rot", [ntcp[0], ntcp[1], Z_GRASP + 0.04], nquat),
         ("down", [ntcp[0], ntcp[1], Z_GRASP + 0.002], nquat), ("up", [ntcp[0], ntcp[1], 1.10], nquat)]
ref = r.q()
plan = []
for name, p, qu in poses:
    q, d = ik_best(r, p, qu, ref)
    print(name, np.round(p, 3), "IK FAIL" if q is None else f"d={d:.2f} q={np.round(q,2)}")
    if q is None: sys.exit(1)
    plan.append((name, p, qu, q)); ref = q
if mode == "check": sys.exit()

def go(p, qu, qseed, sec):
    tgt = np.array(p, float); cmd = tgt.copy(); q = qseed
    for it in range(4):
        if it > 0:
            cur, _ = r.tcp_pose(); off = cur - tgt
            if np.linalg.norm(off) < 0.003: break
            cmd = cmd - off
            q, _ = ik_best(r, cmd, qu, r.q(), n=4)
            if q is None: print("  IK FAIL corr"); return False
        code, err = r.move_joints(q, seconds=sec if it == 0 else 2.0)
        cur, _ = r.tcp_pose()
        print(f"  it{it} code {code} err {err:.4f} tcp {np.round(cur,4)} off {np.round(cur-tgt,4)} fingers {np.round(r.fingers(),4)}"); sys.stdout.flush()
    return True

print("open", r.gripper(0.08))
for name, p, qu, q in plan:
    print(name, np.round(p, 4))
    dq = np.abs(q - r.q()).max(); sec = max(2.0, min(8.0, dq * 3.0))
    go(p, qu, q, sec)
    if name == "grasp":
        print("close", r.gripper(0.0)); f = r.fingers()
        if min(abs(f[0]), abs(f[1])) < 0.006:
            print("BAD GRASP"); print(r.gripper(0.08)); sys.exit(1)
    if name == "down":
        print("open", r.gripper(0.08))
print("done", r.fingers())
EOF
python3 topbar2.py -0.16 -0.445 193 -0.203 -0.395 -90 check 2>&1 | grep -v WARN

# openrua op 226
python3 - <<'EOF' 2>&1 | grep -v WARN
import re
s = open('topbar2.py').read()
s = s.replace('''def hand_quat(az_deg):
    u = np.array([np.cos(np.radians(az_deg)), np.sin(np.radians(az_deg)), 0])
    xh = np.array([np.cos(np.radians(az_deg - OFF)), np.sin(np.radians(az_deg - OFF)), 0])
    return quat_from_axes([0, 0, -1], xh), u''','''FLIP1 = float(sys.argv[8]) if len(sys.argv) > 8 else 0.0
FLIP2 = float(sys.argv[9]) if len(sys.argv) > 9 else 0.0
def hand_quat(az_deg, flip=0.0):
    u = np.array([np.cos(np.radians(az_deg)), np.sin(np.radians(az_deg)), 0])
    a = np.radians(az_deg - OFF + flip)
    xh = np.array([np.cos(a), np.sin(a), 0])
    return quat_from_axes([0, 0, -1], xh), u''')
s = s.replace('quat, u = hand_quat(az);', 'quat, u = hand_quat(az, FLIP1);')
s = s.replace('nquat, nu = hand_quat(naz);', 'nquat, nu = hand_quat(naz, FLIP2);')
open('topbar2.py','w').write(s)
EOF
for f in "0 180" "180 0" "180 180"; do echo "== flips $f"; python3 topbar2.py -0.16 -0.445 193 -0.203 -0.395 -90 check $f 2>&1 | grep -v WARN; done
