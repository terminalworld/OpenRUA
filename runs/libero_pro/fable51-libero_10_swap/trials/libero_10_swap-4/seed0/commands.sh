#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la .ros tools

# openrua op 2
ls -la tools/action tools/perception; cat tools/perception/*.py tools/action/*.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ---; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -12; echo ---; timeout 20 ros2 topic echo /agentview/color/camera_info --once | head -20

# openrua op 6
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -A3 "At time" | head -5; echo ---; timeout 30 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -3; echo ---; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | sort -u

# openrua op 7
mkdir -p "$(dirname /workspace/px.py)"
cat > /workspace/px.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Batch pixel->world for one camera. Usage: px.py <cam> u,v [u,v ...]
Also saves <cam>_depth.npy for offline use."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def main():
    cam = sys.argv[1]
    pts = [tuple(int(x) for x in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("px")
    buf = Buffer(); TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
    np.save(f"{cam}_depth.npy", depth)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    np.save(f"{cam}_T.npy", np.vstack([np.hstack([R, tr[:, None]]), [0, 0, 0, 1]]))
    np.save(f"{cam}_K.npy", np.array(info.k).reshape(3, 3))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    for (u, v) in pts:
        d = float(depth[v, u])
        p = np.array([(u - cx) * d / fx, (v - cy) * d / fy, d])
        wp = R @ p + tr
        print(f"px({u},{v}) depth={d:.3f} world=({wp[0]:.3f}, {wp[1]:.3f}, {wp[2]:.3f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 90 python3 px.py agentview 85,290 545,305 210,240 335,200 410,240 320,400; echo ---; timeout 90 python3 px.py birdview 245,283 388,286 283,262 322,235 320,200

# openrua op 9
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); K=np.load('birdview_K.npy')
h,w=d.shape; vs,us=np.mgrid[0:h,0:w]
p=np.stack([(us-K[0,2])*d/K[0,0],(vs-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)
W=(T@p.reshape(-1,4).T).T[:,:3].reshape(h,w,3)
np.save('birdview_W.npy',W)
z=W[...,2]
table=(np.abs(z-0.485)<0.01)
print('table z median', np.median(z[table]))
# elevated
el=(z>0.50)&(z<0.65)&np.isfinite(z)
img=cv2.imread('birdview.png')
mask=el.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
  if stats[i,4]<15: continue
  m=lab==i
  pts=W[m]
  col=img[m].mean(0)
  print(i,'px',stats[i,4],'centroid px',cent[i].round(0),'world xy',pts[:,:2].mean(0).round(3),'zmax',pts[:,2].max().round(3),'bgr',col.round(0))
"


# openrua op 10
python3 -c "
import numpy as np, cv2
W=np.load('birdview_W.npy'); z=W[...,2]
img=cv2.imread('birdview.png')
el=(z>0.50)&(z<0.65)&np.isfinite(z)
n,lab,stats,cent=cv2.connectedComponentsWithStats(el.astype(np.uint8))
for i in range(1,n):
  if stats[i,4]<15: continue
  pts=W[lab==i]
  print(i,'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
# plates: bright pixels at table height
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
bright=(hsv[...,2]>150)&(np.abs(z-0.487)<0.015)
n,lab,stats,cent=cv2.connectedComponentsWithStats(bright.astype(np.uint8))
for i in range(1,n):
  if stats[i,4]<50: continue
  pts=W[lab==i]
  print('plate?',i,stats[i,4],'center',pts[:,:2].mean(0).round(3),'x[%.3f,%.3f] y[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max()))
"


# openrua op 11
python3 -c "
import cv2
img=cv2.imread('birdview.png'); crop=img[200:320,200:440]; cv2.imwrite('bird_crop.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
img=cv2.imread('agentview.png'); crop=img[140:300,150:480]; cv2.imwrite('agent_crop.png',cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 12
python3 -c "
import numpy as np
d=np.load('agentview_depth.npy'); T=np.load('agentview_T.npy'); K=np.load('agentview_K.npy')
h,w=d.shape; vs,us=np.mgrid[0:h,0:w]
p=np.stack([(us-K[0,2])*d/K[0,0],(vs-K[1,2])*d/K[1,1],d,np.ones_like(d)],-1)
W=(T@p.reshape(-1,4).T).T[:,:3].reshape(h,w,3); np.save('agentview_W.npy',W)
def ext(name,xr,yr):
  for zlo in [0.50,0.52,0.54,0.56,0.58]:
    m=(W[...,0]>xr[0])&(W[...,0]<xr[1])&(W[...,1]>yr[0])&(W[...,1]<yr[1])&(W[...,2]>zlo)&(W[...,2]<zlo+0.02)
    if m.sum()==0: print(name,zlo,'none'); continue
    pts=W[m]; print(name,'z%.2f'%zlo,'n',m.sum(),'y[%.3f,%.3f] w=%.3f'%(pts[:,1].min(),pts[:,1].max(),pts[:,1].max()-pts[:,1].min()),'xfront=%.3f'%pts[:,0].max())
ext('white',(-0.2,0.0),(-0.25,-0.05))
ext('yellow',(-0.3,-0.15),(-0.1,0.1))
ext('red',(-0.2,0.0),(0.05,0.25))
# birdview strict
Wb=np.load('birdview_W.npy')
for name,xr,yr in [('white',(-0.2,0.0),(-0.25,-0.05)),('yellow',(-0.3,-0.15),(-0.1,0.03))]:
  m=(Wb[...,0]>xr[0])&(Wb[...,0]<xr[1])&(Wb[...,1]>yr[0])&(Wb[...,1]<yr[1])&(Wb[...,2]>0.56)
  pts=Wb[m]; print('bird',name,m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max()), 'center',pts[:,:2].mean(0).round(3))
"


# openrua op 13
python3 -c "
import numpy as np, cv2
Wb=np.load('birdview_W.npy'); img=cv2.imread('birdview.png')
m=(Wb[...,2]>0.545)&(Wb[...,2]<0.63)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m.astype(np.uint8))
for i in range(1,n):
  if stats[i,4]<10: continue
  pts=Wb[lab==i]; col=img[lab==i].mean(0)
  print(i,stats[i,4],'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()),'c',pts[:,:2].mean(0).round(3),'bgr',col.round(0))
"


# openrua op 14
python3 -c "
import numpy as np, cv2
W=np.load('agentview_W.npy'); img=cv2.imread('agentview.png'); hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
z=W[...,2]
# yellow mug region: pixels x in [-0.3,-0.15], z>0.5
def region(xr,yr):
  return (W[...,0]>xr[0])&(W[...,0]<xr[1])&(W[...,1]>yr[0])&(W[...,1]<yr[1])&(z>0.495)&(z<0.62)
ym=region((-0.30,-0.15),(-0.10,0.10))
# split yellow (hue ~20-35, sat>80) vs white
yel=ym&(hsv[...,1]>80)&(hsv[...,0]>15)&(hsv[...,0]<40)
whi=ym&~yel
for nm,mm in [('yellow part',yel),('white part',whi),('all yellowmug',ym)]:
  pts=W[mm]; us=np.where(mm)[1]
  print(nm,mm.sum(),'u[%d,%d]'%(us.min(),us.max()),'y[%.3f,%.3f] z[%.3f,%.3f] xmin=%.3f'%(pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max(),pts[:,0].min()))
# per-row y-extent for the yellow mug body at mid height
for zlo in [0.50,0.53,0.56]:
  mm=ym&(z>zlo)&(z<zlo+0.015)
  pts=W[mm]; ys=np.sort(pts[:,1]); print('yellow z%.2f'%zlo, 'y pct', np.percentile(ys,[2,10,50,90,98]).round(3))
wm=region((-0.2,0.0),(-0.25,-0.05))
for zlo in [0.50,0.53,0.56]:
  mm=wm&(z>zlo)&(z<zlo+0.015)
  pts=W[mm]; ys=np.sort(pts[:,1]); print('white z%.2f'%zlo, 'y pct', np.percentile(ys,[2,10,50,90,98]).round(3),'xmin %.3f'%pts[:,0].min())
"


# openrua op 15
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 16
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable controller: IK -> trajectory, gripper, joint/FK readers.
World<->base: base = world + (0.51, 0, -0.42) (from TF world->panda_link0)."""
import sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE_OFF = np.array([-0.51, 0.0, 0.42])  # world position of panda_link0
TCP = 0.1034
# top-down grasp, fingers opening along world X
Q_FX = (0.7071068, 0.7071068, 0.0, 0.0)
# top-down grasp, fingers opening along world Y
Q_FY = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._js_cb, 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand,
                                 "/franka_gripper/gripper_action")
        self.twist_pub = self.node.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"
        assert self.fjt.wait_for_server(10), "no FJT"
        assert self.grip.wait_for_server(10), "no gripper"
        self.wait_js()

    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self.js = {}
        end = time.time() + 20
        while not self.js and time.time() < end:
            self.spin(0.2)
        return dict(self.js)

    def arm_q(self):
        js = self.wait_js()
        return [js[j] for j in ARM]

    def fingers(self):
        js = self.wait_js()
        return js["panda_finger_joint1"], js["panda_finger_joint2"]

    def _call(self, cli, req, timeout=60):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def fk_hand(self, q=None):
        """Hand pose in WORLD frame (position, quat xyzw)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(q if q is not None else self.arm_q())
        res = self._call(self.fk, req)
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_OFF
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_world(self, q=None):
        pos, quat = self.fk_hand(q)
        R = quat_to_R(*quat)
        return pos + TCP * R[:, 2], quat

    def solve_ik(self, pos_world, quat, at_tcp=True, seed=None):
        pos = np.array(pos_world, float)
        R = quat_to_R(*quat)
        if at_tcp:
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE_OFF
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(seed if seed is not None else self.arm_q())
        req.ik_request.timeout = Duration(sec=2)
        req.ik_request.avoid_collisions = False
        res = self._call(self.ik, req)
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def traj(self, points, secs):
        """points: list of 7-vectors; secs: list of time_from_start."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        for q, t in zip(points, secs):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        q = np.array(self.arm_q())
        err = np.abs(q - np.array(points[-1])).max()
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, secs=3.0, seed=None, via=None):
        q = self.solve_ik(pos, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED for {np.round(pos,3)}")
            return None
        pts, ts = [], []
        if via is not None:
            pts.append(via); ts.append(secs * 0.5)
        pts.append(q); ts.append(secs)
        self.traj(pts, ts)
        tcp, _ = self.tcp_world()
        print(f"  tcp now {np.round(tcp,3)} (target {np.round(pos,3)})")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        f = self.fingers()
        print(f"  gripper -> fingers {f[0]:.4f} {f[1]:.4f}")
        return f

    def servo(self, v, n, dt=0.05):
        """Stream a base-frame linear velocity v (m/s) for n ticks."""
        msg = TwistStamped()
        msg.header.frame_id = "panda_link0"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(dt)

    def snap(self, cam, out=None):
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
        return img

    def depth_cloud(self, cam):
        """Return (H,W,3) world points + color image for a camera (needs TF)."""
        from tf2_ros import Buffer, TransformListener
        buf = Buffer(); TransformListener(buf, self.node)
        got = {}
        s1 = self.node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
        s2 = self.node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("k", m), 1)
        s3 = self.node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
        end = time.time() + 30
        while len(got) < 3 and time.time() < end:
            self.spin(0.2)
        for s in (s1, s2, s3):
            self.node.destroy_subscription(s)
        frame = f"{cam}_optical_frame"
        end = time.time() + 10
        while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
            self.spin(0.2)
        t = buf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float32)
        K = np.array(got["k"].k).reshape(3, 3)
        h, w = d.shape
        vs, us = np.mgrid[0:h, 0:w]
        p = np.stack([(us - K[0, 2]) * d / K[0, 0], (vs - K[1, 2]) * d / K[1, 1], d], -1)
        W = p.reshape(-1, 3) @ R.T + tr
        img = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
        return W.reshape(h, w, 3), img
OPENRUA_EOF

# openrua op 17
timeout 120 python3 -c "
from ctl import *
c=Ctl()
print('q',np.round(c.arm_q(),3))
pos,quat=c.fk_hand(); print('hand world',np.round(pos,3),'quat',np.round(quat,3))
R=quat_to_R(*quat); print('hand axes (cols x,y,z) in world:\n',np.round(R,2))
tcp,_=c.tcp_world(); print('tcp',np.round(tcp,3))
# test IK above white mug
for nm,q in [('FX',Q_FX),('FY',Q_FY)]:
  s=c.solve_ik([-0.09+0.047,-0.145,0.70],q); print(nm,'IK',None if s is None else np.round(s,3))
"


# openrua op 18
timeout 30 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A2 "At time" | head -3; timeout 20 ros2 topic echo /tf --once 2>&1 | grep -B1 -A8 "child_frame_id: panda_link0" | head -20

# openrua op 19
timeout 120 python3 -c "
import ctl
from ctl import *
c=Ctl()
q0=c.arm_q()
# try IK for current hand pose expressed in world (no BASE_OFF) and in base
ctl.BASE_OFF=np.zeros(3)
s=c.solve_ik([-0.053,0,0.778],(1,0,-0.028,0),at_tcp=False); print('world-frame IK',None if s is None else np.round(s,3))
ctl.BASE_OFF=np.array([-0.51,0,0.42])
s=c.solve_ik([-0.053,0,0.778],(1,0,-0.028,0),at_tcp=False); print('base-frame IK',None if s is None else np.round(s,3))
print('q0',np.round(q0,3))
"


# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat", "new_string": "        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
timeout 120 python3 -c "
import ctl
from ctl import *
c=Ctl()
q0=c.arm_q()
ctl.BASE_OFF=np.zeros(3)
s=c.solve_ik([-0.053,0,0.778],(1,0,-0.028,0),at_tcp=False); print('world-frame IK',None if s is None else np.round(s,3))
ctl.BASE_OFF=np.array([-0.51,0,0.42])
s=c.solve_ik([-0.053,0,0.778],(1,0,-0.028,0),at_tcp=False); print('base-frame IK',None if s is None else np.round(s,3))
print('q0',np.round(q0,3))
"


# openrua op 22
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "BASE_OFF = np.array([-0.51, 0.0, 0.42])  # world position of panda_link0", "new_string": "# Verified: /compute_fk and /compute_ik poses are already in WORLD coords\\n# (FK of the current state matched TF world->panda_hand), so no offset.\\nBASE_OFF = np.zeros(3)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 23
timeout 200 python3 -c "
from ctl import *
c=Ctl()
q0=c.arm_q()
for nm,qq in [('FX',Q_FX),('FY',Q_FY)]:
  for z in [0.75,0.70]:
    s=c.solve_ik([-0.09+0.047,-0.145,z],qq); print(nm,z,'IK',None if s is None else np.round(s,3))
    if s is not None:
      p,qt=c.tcp_world(s); print('   fk tcp',np.round(p,3),'quat',np.round(qt,3))
"


# openrua op 24
python3 - <<'EOF'
import re
s=open('ctl.py').read()
s=s.replace("""# top-down grasp, fingers opening along world X
Q_FX = (0.7071068, 0.7071068, 0.0, 0.0)
# top-down grasp, fingers opening along world Y
Q_FY = (1.0, 0.0, 0.0, 0.0)""","""# IK tip link is panda_link8 (= panda_hand rotated +45deg about Z), so
# these are link8 orientations. Both are top-down (hand Z = world -Z):
# fingers opening along world X
Q_FX = (0.9238795, 0.3826834, 0.0, 0.0)
# fingers opening along world Y
Q_FY = (0.9238795, -0.3826834, 0.0, 0.0)""")
open('ctl.py','w').write(s)
EOF
timeout 200 python3 -c "
from ctl import *
c=Ctl()
for nm,qq in [('FX',Q_FX),('FY',Q_FY)]:
    s=c.solve_ik([-0.043,-0.145,0.72],qq); print(nm,'IK',None if s is None else np.round(s,3))
    if s is not None:
      p,qt=c.fk_hand(s); print('   hand axes\n',np.round(quat_to_R(*qt),2))
"


# openrua op 25
timeout 300 python3 -c "
from ctl import *
c=Ctl()
c.move_tcp([-0.09,-0.145,0.76],Q_FX,3.0)
c.snap('robot0_eye_in_hand','eih1.png')
c.snap('agentview','agent1.png')
" && timeout 30 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A2 "At time" | head -3

# openrua op 26
timeout 60 python3 -c "
from ctl import *
c=Ctl(); print(np.round(c.arm_q(),3)); p,q=c.tcp_world(); print('tcp',np.round(p,3))"

# openrua op 27
timeout 120 python3 -c "
from ctl import *
c=Ctl()
W,img=c.depth_cloud('robot0_eye_in_hand')
np.save('eih_W.npy',W)
z=W[...,2]
m=(z>0.55)&(z<0.63)&(W[...,0]>-0.2)&(W[...,0]<0.0)&(W[...,1]>-0.25)&(W[...,1]<-0.05)
pts=W[m]; print('n',m.sum())
print('x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
# rim only: top 1.5cm
top=pts[pts[:,2]>pts[:,2].max()-0.012]
print('rim n',len(top),'x[%.3f,%.3f] y[%.3f,%.3f]'%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max()))
# circle fit (algebraic) on rim xy
x,y=top[:,0],top[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
cx,cy,cc=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(cc+cx*cx+cy*cy)
print('circle center (%.3f,%.3f) r=%.3f'%(cx,cy,r))
# table height near
t=(np.abs(W[...,0]+0.09)<0.1)&(np.abs(W[...,1]+0.3)<0.05)&(z<0.5)&(z>0.4); print('table z',np.median(W[t][:,2]))
"


# openrua op 28
timeout 120 python3 -c "
import numpy as np
W=np.load('eih_W.npy'); z=W[...,2]
print('finite',np.isfinite(z).mean(), 'z range',np.nanmin(z),np.nanmax(z))
for (u,v) in [(330,285),(330,150),(460,270),(50,80)]:
  print((u,v),np.round(W[v,u],3))
"


# openrua op 29
timeout 120 python3 -c "
from ctl import *
import numpy as np, cv2
rclpy.init(); node=rclpy.create_node('t')
got={}
s1=node.create_subscription(Image,'/robot0_eye_in_hand/depth/image_raw',lambda m: got.setdefault('d',m),1)
s2=node.create_subscription(CameraInfo,'/robot0_eye_in_hand/color/camera_info',lambda m: got.setdefault('k',m),1)
while len(got)<2: rclpy.spin_once(node,timeout_sec=0.2)
d=CvBridge().imgmsg_to_cv2(got['d'],'passthrough'); print(d.shape,d.dtype,got['d'].encoding, 'K',np.array(got['k'].k).reshape(3,3)[0], got['k'].width,got['k'].height)
print('depth at mug px',d[285,330],'plate',d[270,460],'table',d[150,330],'fingers',d[400,170])
np.save('eih_d.npy',d)
vis=((d-d.min())/(d.max()-d.min())*255).astype(np.uint8); cv2.imwrite('eih_depth.png',vis)
"


# openrua op 30
python3 -c "
import numpy as np
from ctl import quat_to_R
R=quat_to_R(0.819,-0.573,0.0,-0.001); t=np.array([-0.043,-0.128,0.864])
print('cam axes\n',np.round(R,2))
def ray(u,v,zp):
  d=R@np.array([(u-320)/312.77,(v-240)/312.77,1.0]); s=(zp-t[2])/d[2]; return t+s*d
print('white mug rim center px(330,287) @z=0.595 ->',np.round(ray(330,287,0.595),3))
print('left plate px(460,270) @z=0.49 ->',np.round(ray(460,270,0.49),3))
print('right plate px(45,85) @z=0.49 ->',np.round(ray(45,85,0.49),3))
print('yellow mug rim px(110,335) @z=0.584 ->',np.round(ray(110,335,0.584),3))
"


# openrua op 31
timeout 120 python3 -c "
from ctl import *
c=Ctl()
W,img=c.depth_cloud('birdview'); cv2.imwrite('bird2.png',img); np.save('bird2_W.npy',W)
z=W[...,2]
m=(z>0.545)&(z<0.63)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m.astype(np.uint8))
for i in range(1,n):
  if stats[i,4]<10: continue
  pts=W[lab==i]; col=img[lab==i].mean(0)
  x,y=pts[:,0],pts[:,1]
  A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
  cx,cy,cc=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(cc+cx*cx+cy*cy)
  print(i,stats[i,4],'x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(x.min(),x.max(),y.min(),y.max(),pts[:,2].max()),'bbox-center (%.3f,%.3f)'%((x.min()+x.max())/2,(y.min()+y.max())/2),'fit c(%.3f,%.3f) r=%.3f'%(cx,cy,r),'bgr',col.round(0))
"


# openrua op 32
timeout 120 python3 -c "
from ctl import *
c=Ctl()
W,img=c.depth_cloud('agentview'); cv2.imwrite('agent2.png',img); np.save('agent2_W.npy',W)
z=W[...,2]
# table height: region in front, no objects
t=(W[...,0]>0.1)&(W[...,0]<0.3)&(np.abs(W[...,1])<0.1)&np.isfinite(z); print('table z median',np.median(z[t]))
Wb=np.load('bird2_W.npy'); zb=Wb[...,2]
t=(Wb[...,0]>0.1)&(Wb[...,0]<0.3)&(np.abs(Wb[...,1])<0.1); print('bird table z',np.median(zb[t]))
# plates in bird2
hsv=cv2.cvtColor(cv2.imread('bird2.png'),cv2.COLOR_BGR2HSV)
tz=np.median(zb[t])
bright=(hsv[...,2]>150)&(np.abs(zb-tz)<0.02)
n,lab,stats,cent=cv2.connectedComponentsWithStats(bright.astype(np.uint8))
for i in range(1,n):
  if stats[i,4]<200: continue
  pts=Wb[lab==i]; print('plate',stats[i,4],'center',pts[:,:2].mean(0).round(3),'zmed %.3f'%np.median(pts[:,2]),'x[%.3f,%.3f] y[%.3f,%.3f]'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max()))
# mugs in agentview
for nm,xr,yr in [('white',(-0.2,0.0),(-0.25,-0.05)),('yellow',(-0.3,-0.12),(-0.1,0.12)),('red',(-0.2,0.0),(0.03,0.25))]:
  m=(W[...,0]>xr[0])&(W[...,0]<xr[1])&(W[...,1]>yr[0])&(W[...,1]<yr[1])&(z>tz+0.01)&(z<tz+0.2)
  pts=W[m]; print(nm,m.sum(),'zmax %.3f'%pts[:,2].max(),'xfront %.3f'%pts[:,0].max(),'y[%.3f,%.3f]'%(pts[:,1].min(),pts[:,1].max()))
"


# openrua op 33
timeout 60 python3 -c "
import numpy as np, cv2
W=np.load('agent2_W.npy'); z=W[...,2]; tz=0.425
Wb=np.load('bird2_W.npy'); zb=Wb[...,2]
def fit(pts):
  x,y=pts[:,0],pts[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
  cx,cy,cc=np.linalg.lstsq(A,b,rcond=None)[0]; return cx,cy,np.sqrt(cc+cx*cx+cy*cy)
for nm,xr,yr in [('white',(-0.2,0.02),(-0.25,-0.05)),('yellow',(-0.3,-0.12),(-0.1,0.12)),('red',(-0.2,0.02),(0.03,0.25))]:
  m=(W[...,0]>xr[0])&(W[...,0]<xr[1])&(W[...,1]>yr[0])&(W[...,1]<yr[1])&(z>tz+0.04)&(z<tz+0.2)
  pts=W[m]; zmax=pts[:,2].max()
  rim=pts[pts[:,2]>zmax-0.012]
  print(nm,'zmax %.3f'%zmax,'rim n',len(rim),'x[%.3f,%.3f] y[%.3f,%.3f]'%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max()),'fit c(%.3f,%.3f) r=%.3f'%fit(rim))
  mb=(Wb[...,0]>xr[0])&(Wb[...,0]<xr[1])&(Wb[...,1]>yr[0])&(Wb[...,1]<yr[1])&(zb>tz+0.04)&(zb<tz+0.2)
  if mb.sum()>10:
    pts=Wb[mb]; zmax=pts[:,2].max(); rim=pts[pts[:,2]>zmax-0.012]
    print('   bird zmax %.3f'%zmax,'rim n',len(rim),'x[%.3f,%.3f] y[%.3f,%.3f]'%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max()),'fit c(%.3f,%.3f) r=%.3f'%fit(rim))
"


# openrua op 34
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
cx,cy,rz=-0.087,-0.155,0.550
hx,hy=cx+0.045,cy
print('pregrasp'); c.move_tcp([hx,hy,0.66],Q_FX,3.0)
print('descend'); c.move_tcp([hx,hy,rz-0.025],Q_FX,2.5)
c.snap('agentview','w_descend.png')
print('close'); f=c.gripper(0.0)
print('lift'); c.move_tcp([hx,hy,0.70],Q_FX,2.5)
print('fingers after lift',c.fingers())
c.snap('agentview','w_lift.png')
" 2>&1 | tee w_pick.log

# openrua op 35
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
px,py,pz=-0.02,-0.325,0.444
hx,hy=px+0.045,py
print('over plate'); c.move_tcp([hx,hy,0.70],Q_FX,3.0)
print('fingers',c.fingers())
print('lower'); c.move_tcp([hx,hy,pz+0.10+0.008],Q_FX,2.5)
print('open'); c.gripper(0.04)
print('retreat'); c.move_tcp([hx,hy,0.70],Q_FX,2.5)
c.snap('agentview','w_placed.png')
" 2>&1 | tee w_place.log

# openrua op 36
timeout 900 python3 -u -c "
from ctl import *
c=Ctl()
cx,cy,rz=-0.216,0.033,0.543
hx,hy=cx+0.045,cy
print('pregrasp'); c.move_tcp([hx,hy,0.66],Q_FX,3.0)
print('descend'); c.move_tcp([hx,hy,rz-0.025],Q_FX,2.5)
print('close'); c.gripper(0.0)
print('lift'); c.move_tcp([hx,hy,0.70],Q_FX,2.5)
print('fingers after lift',c.fingers())
c.snap('agentview','y_lift.png')
" 2>&1 | tee y_pick.log

# openrua op 37
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "        code = res.result().result.error_code\\n        q = np.array(self.arm_q())\\n        err = np.abs(q - np.array(points[-1])).max()\\n        print(f\\"  traj done code={code} max_joint_err={err:.4f}\\")\\n        return code, err", "new_string": "        code = res.result().result.error_code\\n        # controller may report before the arm has converged: poll until\\n        # the joint state stops changing, then compare with the target\\n        prev = np.array(self.arm_q())\\n        for _ in range(40):\\n            q = np.array(self.arm_q())\\n            if np.abs(q - prev).max() < 1e-4:\\n                break\\n            prev = q\\n        err = np.abs(q - np.array(points[-1])).max()\\n        print(f\\"  traj done code={code} max_joint_err={err:.4f}\\")\\n        if err > 0.02:\\n            print(\\"  resending goal to converge\\")\\n            send = self.fjt.send_goal_async(goal)\\n            rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)\\n            res = send.result().get_result_async()\\n            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)\\n            q = np.array(self.arm_q())\\n            err = np.abs(q - np.array(points[-1])).max()\\n            print(f\\"  retry code={res.result().result.error_code} max_joint_err={err:.4f}\\")\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 38
timeout 900 python3 -u -c "
from ctl import *
c=Ctl()
px,py,pz=0.007,0.278,0.444
hx,hy=px+0.03,py
print('over plate'); c.move_tcp([hx,hy,0.70],Q_FX,3.0)
print('fingers',c.fingers())
print('lower'); c.move_tcp([hx,hy,0.542],Q_FX,2.5)
print('open'); c.gripper(0.04)
print('retreat'); c.move_tcp([hx,hy,0.70],Q_FX,2.5)
c.snap('agentview','y_placed.png')
" 2>&1 | tee y_place.log

# openrua op 39
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
print('retreat up/back'); c.move_tcp([-0.15,0.0,0.80],Q_FX,3.0)
W,img=c.depth_cloud('birdview'); cv2.imwrite('bird_final.png',img)
c.snap('agentview','agent_final.png')
z=W[...,2]; tz=np.median(z[(W[...,0]>0.1)&(W[...,0]<0.3)&(np.abs(W[...,1])<0.1)]); print('table z %.3f'%tz)
def fit(pts):
  x,y=pts[:,0],pts[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
  cx,cy,cc=np.linalg.lstsq(A,b,rcond=None)[0]; return cx,cy,np.sqrt(cc+cx*cx+cy*cy)
for nm,xr,yr,plate in [('white',(-0.15,0.15),(-0.45,-0.2),(-0.02,-0.325)),('yellow',(-0.15,0.15),(0.15,0.45),(0.007,0.278)),('red',(-0.2,0.05),(0.03,0.2),None)]:
  m=(W[...,0]>xr[0])&(W[...,0]<xr[1])&(W[...,1]>yr[0])&(W[...,1]<yr[1])&(z>tz+0.05)&(z<tz+0.25)
  pts=W[m]; zmax=pts[:,2].max(); rim=pts[pts[:,2]>zmax-0.012]; cx,cy,r=fit(rim)
  s=f'{nm}: rim z={zmax:.3f} (height above table {zmax-tz:.3f}) center=({cx:.3f},{cy:.3f}) r={r:.3f}'
  if plate: s+=f'  plate center={plate} offset={np.hypot(cx-plate[0],cy-plate[1]):.3f} m'
  print(s)
"


# openrua op 40
timeout 120 python3 -c "
from ctl import *
c=Ctl()
W,img=c.depth_cloud('birdview')
z=W[...,2]; tz=0.425
def fit(pts):
  x,y=pts[:,0],pts[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
  cx,cy,cc=np.linalg.lstsq(A,b,rcond=None)[0]; return cx,cy,np.sqrt(cc+cx*cx+cy*cy)
m=(W[...,0]>-0.15)&(W[...,0]<0.15)&(W[...,1]>0.19)&(W[...,1]<0.45)&(z>tz+0.05)&(z<tz+0.25)
pts=W[m]; zmax=pts[:,2].max(); rim=pts[pts[:,2]>zmax-0.012]; cx,cy,r=fit(rim)
print(f'yellow: n={m.sum()} rim z={zmax:.3f} (height {zmax-tz:.3f}) center=({cx:.3f},{cy:.3f}) r={r:.3f} offset from plate (0.007,0.278)={np.hypot(cx-0.007,cy-0.278):.3f}')
# plate visible ring check: plate pixels at table+~2cm around yellow
"

