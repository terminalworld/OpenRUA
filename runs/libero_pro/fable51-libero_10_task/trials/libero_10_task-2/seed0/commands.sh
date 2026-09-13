#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/ik_move.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
timeout 30 ros2 topic list; echo ---ACTIONS; timeout 30 ros2 action list; echo ---SERVICES; timeout 30 ros2 service list | grep -v -E "parameter|describe_|get_type" ; echo ---NODES; timeout 30 ros2 node list

# openrua op 3
timeout 20 ros2 topic echo /joint_states --once; timeout 10 ros2 topic echo /pipeline_state --once; timeout 10 ros2 topic info /pipeline_state

# openrua op 4
for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 30 ros2 node info /robot_bridge; timeout 10 ros2 topic echo /tf_static --once 2>/dev/null | head -80

# openrua op 6
timeout 30 ros2 topic echo /tf --once 2>/dev/null | head -120; timeout 20 ros2 param list /robot_bridge

# openrua op 7
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+info from a camera, save world-frame point cloud as npz.

Usage: python3 cloud.py <camera>   -> <camera>_cloud.npz (xyz HxWx3, bgr HxWx3)
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    import time
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
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
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    t0 = time.time()
    while time.time() - t0 < 10 and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    xyz = pc @ R.T + p
    np.savez(f"{cam}_cloud.npz", xyz=xyz, bgr=color, depth=depth)
    print(f"{cam}_cloud.npz", xyz.shape)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 90 python3 cloud.py birdview && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]
print('z range', np.nanmin(z), np.nanmax(z))
# table height: most common z
h,e=np.histogram(z[np.isfinite(z)], bins=200)
print('top bins', [(round(e[i],3), h[i]) for i in np.argsort(h)[-8:]])
"

# openrua op 9
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
table=(np.abs(z-0.896)<0.01)
xs=x[table]; ys=y[table]
print('table x', xs.min(), xs.max(), 'y', ys.min(), ys.max())
# objects: above table and within table extent, excluding robot region x< -0.35
obj=(z>0.905)&(z<1.3)&(x>-0.3)&(x<xs.max())&(np.abs(y)<ys.max())
mask=obj.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    print(i,'px',stats[i,4],'centroid uv',cent[i].round(1),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(x[m].min(),x[m].max(),y[m].min(),y[m].max(),z[m].min(),z[m].max()), 'mean bgr', bgr[m].mean(0).round())
"


# openrua op 10
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.905)&(x>-0.30)&(x<-0.15)&(y>0.13)&(y<0.27)
print('knob pts',m.sum())
for lo,hi in [(0.905,0.93),(0.93,0.945),(0.945,0.955),(0.955,0.97)]:
    mm=m&(z>=lo)&(z<hi)
    if mm.sum(): print(lo,hi,mm.sum(),'x[%.3f %.3f] y[%.3f %.3f]'%(x[mm].min(),x[mm].max(),y[mm].min(),y[mm].max()))
# print a small ascii height map
vs,us=np.where(m)
sub=z[vs.min():vs.max()+1, us.min():us.max()+1]
np.set_printoptions(linewidth=250, precision=3)
print(np.round((sub-0.896)*100,1))
# stove
ms=(z>0.92)&(z<0.94)&(x>-0.17)&(x<0.05)&(y>0.09)&(y<0.31)
print('stove center', x[ms].mean(), y[ms].mean(), 'z', z[ms].mean())
"


# openrua op 11
timeout 20 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once

# openrua op 12
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot helper: joint state, FK, IK, trajectory, gripper.

World frame = panda_link0 frame + BASE offset (from /tf: world->panda_link0).
All poses here are in WORLD unless noted.
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
LIMITS = np.array(FJT["limits_rad"])
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation
TCP = M["hand"]["tcp_offset_m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    # returns x,y,z,w
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def topdown_quat(yaw):
    """Hand pointing down (hand z = -world z), hand x rotated by yaw about world z
    from +world x. yaw=0 -> fingers close along world y."""
    c, s = np.cos(yaw), np.sin(yaw)
    hx = np.array([c, s, 0.0])
    hz = np.array([0.0, 0.0, -1.0])
    hy = np.cross(hz, hx)
    R = np.stack([hx, hy, hz], axis=1)
    return R_quat(R)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            t0 = time.time()
            while self._js is None and time.time() - t0 < 10:
                self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def _call(self, cli, req, timeout=60):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def fk_pose(self, q=None, link="panda_hand"):
        """Returns (pos_world, quat xyzw) of link for arm config q."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        res = self._call(self.fk, req)
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik_q(self, pos_world, quat, seed=None, at_tcp=False):
        pos = np.array(pos_world, dtype=float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        if seed is None:
            seed = self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        res = self._call(self.ik, req)
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    def move_q(self, q, seconds=3.0, tol=0.02, retries=2):
        q = np.array(q, dtype=float)
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            gh = send.result()
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
            code = rf.result().result.error_code
            cur = self.arm_q()
            err = np.abs(cur - q).max()
            print(f"  move_q: code={code} max_err={err:.4f}")
            if err < tol:
                return True
        return err < tol

    def move_pose(self, pos_world, quat, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik_q(pos_world, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            print("  IK failed for", pos_world, quat)
            return None
        ok = self.move_q(q, seconds)
        return q if ok else None

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def servo(self, lin, ang=(0, 0, 0), ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            self.tw.publish(msg)
            self.spin(0.05)
OPENRUA_EOF

# openrua op 13
timeout 120 python3 -c "
from rob import *
r=Robot()
q=r.arm_q(); print('q',q.round(3))
pos,quat=r.fk_pose(q); print('hand world',pos.round(4),'quat',quat.round(4))
print('R',quat_R(*quat).round(3))
print('topdown yaw0 quat', topdown_quat(0).round(4))
# test IK roundtrip at current pose
q2=r.ik_q(pos,quat); print('ik roundtrip', None if q2 is None else q2.round(3))
"

# openrua op 14
timeout 120 python3 -c "
from rob import *
r=Robot()
q=r.arm_q()
for link in ['panda_link0','panda_link1','panda_link2','panda_link4','panda_link7','panda_link8','panda_hand']:
    req = GetPositionFK.Request(); req.fk_link_names=[link]
    req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=[float(v) for v in q]
    res=r._call(r.fk,req); p=res.pose_stamped[0]
    print(link, p.header.frame_id, round(p.pose.position.x,3), round(p.pose.position.y,3), round(p.pose.position.z,3))
"; timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -12

# openrua op 15
timeout 120 python3 -c "
from rob import *
import rob
rob.BASE[:]=0  # FK already in world; test IK frames
r=Robot()
q=r.arm_q()
pos,quat=r.fk_pose(q); print('hand world',pos.round(4),quat.round(4))
for label,p in [('world',pos),('base-rel',pos-np.array([-0.66,0,0.912]))]:
    q2=r.ik_q(p,quat)
    if q2 is None: print(label,'IK failed'); continue
    p2,quat2=r.fk_pose(q2)
    print(label,'q2',q2.round(3),'-> fk',p2.round(4),quat2.round(4))
"

# openrua op 16
timeout 120 python3 -c "
from rob import *
import rob
rob.BASE[:]=0
r=Robot()
q=r.arm_q()
pos,quat8=r.fk_pose(q,link='panda_link8'); print('link8',pos.round(4),quat8.round(4))
posh,quath=r.fk_pose(q,link='panda_hand')
# hand->link8: R8 = Rh * Rz(+45deg)
c,s=np.cos(np.pi/4),np.sin(np.pi/4)
Rz=np.array([[c,-s,0],[s,c,0],[0,0,1]])
print('computed link8 quat from hand', R_quat(quat_R(*quath)@Rz).round(4))
q2=r.ik_q(pos,quat8); print('q2',q2.round(3))
p2,qq=r.fk_pose(q2,link='panda_hand'); print('hand fk of sol',p2.round(4),qq.round(4))
"

# openrua op 17
python3 - <<'EOF'
import re
s=open('rob.py').read()
s=s.replace('BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation',
 'BASE = np.array([0.0, 0.0, 0.0])  # FK/IK already answer in WORLD (model includes world->link0)\n_c, _s = np.cos(np.pi / 4), np.sin(np.pi / 4)\nRZ45 = np.array([[_c, -_s, 0], [_s, _c, 0], [0, 0, 1]])  # hand -> link8: R8 = Rh @ RZ45')
s=s.replace('''        pos = pos - BASE
        req = GetPositionIK.Request()''','''        pos = pos - BASE
        # IK tip link is panda_link8 (hand rotated -45deg about z): convert
        quat = R_quat(quat_R(*quat) @ RZ45)
        req = GetPositionIK.Request()''')
open('rob.py','w').write(s)
EOF
timeout 120 python3 -c "
from rob import *
r=Robot()
q=r.arm_q()
pos,quat=r.fk_pose(q); print('hand',pos.round(4),quat.round(4))
q2=r.ik_q(pos,quat); print('roundtrip q2',q2.round(3))
# test target above knob: knob tab center (-0.226,0.201), hand pointing down, fingers along y, hand 15cm above knob top
qk=r.ik_q([-0.226,0.201,0.96+0.25], topdown_quat(0)); print('above-knob q', None if qk is None else qk.round(3))
if qk is not None:
    p,qq=r.fk_pose(qk); print(' fk', p.round(4), qq.round(4))
"

# openrua op 18
timeout 900 python3 -u -c "
from rob import *
r=Robot()
r.gripper(0.04)
q=r.move_pose([-0.226,0.201,1.21], topdown_quat(0), seconds=3)
print('q now', r.arm_q().round(3))
p,qq=r.fk_pose(); print('hand', p.round(4), qq.round(4))
" 2>&1 | tail -8

# openrua op 19
timeout 90 python3 cloud.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
# knob tab
m=(z>0.945)&(z<0.98)&(x>-0.30)&(x<-0.15)&(y>0.13)&(y<0.27)
print('tab pts',m.sum(),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f] center(%.3f,%.3f)'%(x[m].min(),x[m].max(),y[m].min(),y[m].max(),z[m].min(),z[m].max(),x[m].mean(),y[m].mean()))
# knob body
m=(z>0.905)&(z<0.945)&(x>-0.30)&(x<-0.14)&(y>0.12)&(y<0.28)
print('body pts',m.sum(),'x[%.3f %.3f] y[%.3f %.3f] center(%.3f,%.3f)'%(x[m].min(),x[m].max(),y[m].min(),y[m].max(),x[m].mean(),y[m].mean()))
# pan
m=(z>0.905)&(z<1.1)&(x>-0.2)&(x<0.1)&(y>-0.4)&(y<0.05)
print('pan pts',m.sum(),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(x[m].min(),x[m].max(),y[m].min(),y[m].max(),z[m].min(),z[m].max()))
"

# openrua op 20
timeout 90 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
ok=np.isfinite(z)&(z>0.5)
print('z range',z[ok].min(),z[ok].max())
h,e=np.histogram(z[ok],bins=100); print('table z guess', e[np.argmax(h)])
m=ok&(x>-0.30)&(x<-0.15)&(y>0.13)&(y<0.27)&(z>0.905)
for lo,hi in [(0.905,0.92),(0.92,0.93),(0.93,0.94),(0.94,0.95),(0.95,0.96),(0.96,0.97),(0.97,1.0)]:
    mm=m&(z>=lo)&(z<hi)
    if mm.sum(): print(lo,hi,mm.sum(),'x[%.3f %.3f] y[%.3f %.3f]'%(x[mm].min(),x[mm].max(),y[mm].min(),y[mm].max()))
"

# openrua op 21
timeout 900 python3 -u -c "
from rob import *
r=Robot()
for z in [1.10, 1.038]:
    q=r.move_pose([-0.205,0.200,z], topdown_quat(0), seconds=2)
    p,qq=r.fk_pose(); print('hand', p.round(4), qq.round(4))
print('fingers', r.fingers())
" 2>&1 | tail -8; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 22
timeout 600 python3 -u -c "
from rob import *
r=Robot()
r.gripper(0.0)
" 2>&1 | tail -3; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 23
timeout 900 python3 -u -c "
from rob import *
r=Robot()
q=r.arm_q(); print('q before', q.round(3))
q2=q.copy(); q2[6]-=1.4
ok=r.move_q(q2, seconds=4, retries=0)
print('q after', r.arm_q().round(3), 'fingers', r.fingers())
" 2>&1 | tail -4; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 24
timeout 900 python3 -u -c "
from rob import *
r=Robot()
r.gripper(0.04)
p,qq=r.fk_pose(); print('hand', p.round(4))
q=r.move_pose([p[0],p[1],1.20], qq, seconds=2)
print('hand after', r.fk_pose()[0].round(4))
" 2>&1 | tail -4; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 25
timeout 90 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.905)&(z<1.1)&(x>-0.2)&(x<0.1)&(y>-0.4)&(y<0.06)
print('pan+pot pts',m.sum())
np.set_printoptions(linewidth=250)
for lo,hi in [(0.905,0.92),(0.92,0.94),(0.94,0.96),(0.96,0.98),(0.98,1.0),(1.0,1.02),(1.02,1.04),(1.04,1.1)]:
    mm=m&(z>=lo)&(z<hi)
    if mm.sum(): print(lo,hi,mm.sum(),'x[%.3f %.3f] y[%.3f %.3f]'%(x[mm].min(),x[mm].max(),y[mm].min(),y[mm].max()))
# rim: highest ring of pan body (exclude moka pot region y>-0.09 & x>0)
body=m&(y<-0.09)
zz=z[body]; print('body z max', zz.max())
rim=body&(z>zz.max()-0.012)
print('rim pts',rim.sum(),'x[%.3f %.3f] y[%.3f %.3f]'%(x[rim].min(),x[rim].max(),y[rim].min(),y[rim].max()))
# circle fit on rim points
X=x[rim];Y=y[rim]
A=np.c_[2*X,2*Y,np.ones_like(X)]; b=X**2+Y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; R=np.sqrt(c[2]+cx**2+cy**2)
print('rim circle center (%.3f,%.3f) R=%.3f'%(cx,cy,R))
# handle: points at pan height on +y side beyond the rim
hd=m&(y>cy+R-0.01)&(y<0.06)&(x<0.0)
print('handle pts',hd.sum(),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(x[hd].min(),x[hd].max(),y[hd].min(),y[hd].max(),z[hd].min(),z[hd].max()))
"

# openrua op 26
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.91)&(z<0.97)&(x>-0.12)&(x<0.0)&(y>-0.13)&(y<0.05)
for lo in np.arange(-0.13,0.04,0.01):
    mm=m&(y>=lo)&(y<lo+0.01)
    if mm.sum(): print('y %.2f n=%3d x[%.3f %.3f] xc=%.3f z[%.3f %.3f]'%(lo,mm.sum(),x[mm].min(),x[mm].max(),x[mm].mean(),z[mm].min(),z[mm].max()))
"

# openrua op 27
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
cx,cy=-0.060,-0.219
rr=np.hypot(x-cx,y-cy)
m=(z>0.903)&(z<0.95)&(rr<0.13)&(y<-0.10)
for lo,hi in [(0.903,0.915),(0.915,0.925),(0.925,0.935),(0.935,0.945),(0.945,0.95)]:
    mm=m&(z>=lo)&(z<hi)
    if mm.sum(): print(lo,hi,mm.sum(),'r[%.3f %.3f]'%(rr[mm].min(),rr[mm].max()))
# stove plate and coil
ms=(z>0.915)&(z<0.945)&(x>-0.17)&(x<0.05)&(y>0.09)&(y<0.31)
print('stove x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(x[ms].min(),x[ms].max(),y[ms].min(),y[ms].max(),z[ms].min(),z[ms].max()))
bgr=d['bgr']; red=(bgr[...,2]>150)&(bgr[...,1]<100)&ms
print('red coil pts',red.sum(),'center (%.3f,%.3f) x[%.3f %.3f] y[%.3f %.3f]'%(x[red].mean(),y[red].mean(),x[red].min(),x[red].max(),y[red].min(),y[red].max()))
"

# openrua op 28
timeout 300 python3 -c "
from rob import *
r=Robot()
seed=r.arm_q()
for name,pos in [('grasp',[-0.060,-0.045,1.038]),('pregrasp',[-0.060,-0.045,1.15]),('place',[-0.050,0.374,1.07]),('preplace',[-0.050,0.374,1.15])]:
    for yaw in [np.pi/2,-np.pi/2]:
        q=r.ik_q(pos, topdown_quat(yaw), seed=seed)
        if q is None: print(name, round(yaw,2), 'IK FAIL'); continue
        p,qq=r.fk_pose(q)
        print(name, round(yaw,2), q.round(2), 'fk', p.round(3), 'inlim', bool(np.all((q>LIMITS[:,0])&(q<LIMITS[:,1]))))
"

# openrua op 29
timeout 1200 python3 -u -c "
from rob import *
r=Robot()
Q=topdown_quat(np.pi/2)
r.gripper(0.04)
assert r.move_pose([-0.060,-0.045,1.15], Q, seconds=3) is not None
assert r.move_pose([-0.060,-0.045,1.038], Q, seconds=2) is not None
print('hand', r.fk_pose()[0].round(4))
" 2>&1 | tail -5; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 30
timeout 120 python3 -c "
from rob import *
r=Robot()
q=r.arm_q(); print('q', q.round(3))
p,qq=r.fk_pose(); print('hand', p.round(4), qq.round(3))
Q=topdown_quat(np.pi/2)
qt=r.ik_q([-0.060,-0.045,1.15], Q, seed=q); print('target', qt.round(3)); print('diff', (qt-q).round(3))
"

# openrua op 31
timeout 1200 python3 -u -c "
from rob import *
r=Robot()
p,qq=r.fk_pose()
Q0=topdown_quat(0)
r.move_pose([p[0],p[1],1.22], Q0, seconds=2)
print('hand', r.fk_pose()[0].round(4))
" 2>&1 | grep -v XMLPARSER | tail -3; timeout 90 python3 cloud.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview; python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.905)&(z<0.95)&(x>-0.2)&(x<0.1)&(y>-0.4)&(y<-0.09)
body=m
zz=z[body]; rim=body&(z>zz.max()-0.012)
X=x[rim];Y=y[rim]
A=np.c_[2*X,2*Y,np.ones_like(X)]; b=X**2+Y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; R=np.sqrt(c[2]+cx**2+cy**2)
print('rim circle center (%.3f,%.3f) R=%.3f ztop=%.3f'%(cx,cy,R,zz.max()))
mp=(z>0.95)&(z<1.1)&(x>-0.05)&(x<0.15)&(y>-0.15)&(y<0.1)
print('moka x[%.3f %.3f] y[%.3f %.3f] ztop %.3f'%(x[mp].min(),x[mp].max(),y[mp].min(),y[mp].max(),z[mp].max()))
"

# openrua op 32
timeout 1200 python3 -u -c "
from rob import *
r=Robot()
Q0=topdown_quat(0)
r.gripper(0.04)
assert r.move_pose([-0.061,-0.316,1.15], Q0, seconds=3) is not None
assert r.move_pose([-0.061,-0.316,1.027], Q0, seconds=2) is not None
print('hand', r.fk_pose()[0].round(4))
f=r.gripper(0.0)
print('gap', f[0]-f[1])
" 2>&1 | grep -v XMLPARSER | tail -6; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 33
timeout 1200 python3 -u -c "
from rob import *
r=Robot()
Q0=topdown_quat(0)
r.move_pose([-0.061,-0.316,1.09], Q0, seconds=2)
print('hand', r.fk_pose()[0].round(4), 'fingers', r.fingers())
" 2>&1 | grep -v XMLPARSER | tail -3; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 34
timeout 90 python3 cloud.py birdview >/dev/null; python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
# pan region: exclude hand (hand is at z>1.0 near y=-0.316)
m=(z>0.905)&(z<1.05)&(x>-0.25)&(x<0.1)&(y>-0.45)&(y<-0.09)
for lo,hi in [(0.905,0.93),(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.01),(1.01,1.05)]:
    mm=m&(z>=lo)&(z<hi)
    if mm.sum(): print(lo,hi,mm.sum(),'x[%.3f %.3f] y[%.3f %.3f]'%(x[mm].min(),x[mm].max(),y[mm].min(),y[mm].max()))
"

# openrua op 35
timeout 1200 python3 -u -c "
from rob import *
r=Robot()
Q0=topdown_quat(0)
r.move_pose([-0.061,-0.316,1.027], Q0, seconds=2)
print('fingers', r.fingers())
r.move_pose([-0.061,-0.466,1.027], Q0, seconds=3)
print('hand', r.fk_pose()[0].round(4), 'fingers', r.fingers())
r.gripper(0.04)
r.move_pose([-0.061,-0.466,1.18], Q0, seconds=2)
print('hand', r.fk_pose()[0].round(4))
" 2>&1 | grep -v XMLPARSER | tail -8; timeout 90 python3 cloud.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 36
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.905)&(z<0.99)&(x>-0.25)&(x<0.1)&(y>-0.58)&(y<-0.09)
zz=z[m]; print('pan zmax',zz.max())
rim=m&(z>zz.max()-0.012)&(z<0.96)
X=x[rim];Y=y[rim]
A=np.c_[2*X,2*Y,np.ones_like(X)]; b=X**2+Y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; R=np.sqrt(c[2]+cx**2+cy**2)
print('rim circle center (%.3f,%.3f) R=%.3f'%(cx,cy,R))
# handle profile: points outside the circle
rr=np.hypot(x-cx,y-cy)
h=m&(rr>R+0.005)
print('handle pts',h.sum(),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(x[h].min(),x[h].max(),y[h].min(),y[h].max(),z[h].min(),z[h].max()))
for lo in np.arange(-0.40,-0.05,0.01):
    mm=h&(y>=lo)&(y<lo+0.01)
    if mm.sum()>3: print('y %.2f n=%3d x[%.3f %.3f] xc=%.3f z[%.3f %.3f]'%(lo,mm.sum(),x[mm].min(),x[mm].max(),x[mm].mean(),z[mm].min(),z[mm].max()))
"; 

# openrua op 37
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.925)&(z<0.95)&(x>-0.25)&(x<0.1)&(y>-0.58)&(y<-0.28)
X=x[m];Y=y[m]
A=np.c_[2*X,2*Y,np.ones_like(X)]; b=X**2+Y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; R=np.sqrt(c[2]+cx**2+cy**2)
print('rim circle center (%.3f,%.3f) R=%.3f n=%d'%(cx,cy,R,m.sum()))
mp=(z>0.95)&(z<1.1)&(x>-0.05)&(x<0.2)&(y>-0.2)&(y<0.15)
print('moka x[%.3f %.3f] y[%.3f %.3f] ztop %.3f'%(x[mp].min(),x[mp].max(),y[mp].min(),y[mp].max(),z[mp].max()))
"

# openrua op 38
timeout 1200 python3 -u -c "
from rob import *
r=Robot()
Q=topdown_quat(np.pi/2)
r.gripper(0.04)
assert r.move_pose([-0.056,-0.19,1.16], Q, seconds=3) is not None
assert r.move_pose([-0.056,-0.19,1.035], Q, seconds=2) is not None
print('hand', r.fk_pose()[0].round(4))
f=r.gripper(0.0); print('gap', f[0]-f[1])
" 2>&1 | grep -v XMLPARSER | tail -6; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 39
timeout 1200 python3 -u -c "
from rob import *
r=Robot()
Q=topdown_quat(np.pi/2)
r.move_pose([-0.056,-0.19,1.10], Q, seconds=2)
print('hand', r.fk_pose()[0].round(4), 'fingers', r.fingers())
" 2>&1 | grep -v XMLPARSER | tail -2; timeout 90 python3 cloud.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview; python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.905)&(z<1.08)&(x>-0.25)&(x<0.1)&(y>-0.58)&(y<-0.25)
for lo,hi in [(0.905,0.93),(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.01),(1.01,1.03),(1.03,1.08)]:
    mm=m&(z>=lo)&(z<hi)
    if mm.sum(): print(lo,hi,mm.sum(),'x[%.3f %.3f] y[%.3f %.3f]'%(x[mm].min(),x[mm].max(),y[mm].min(),y[mm].max()))
"

# openrua op 40
timeout 1500 python3 -u -c "
from rob import *
r=Robot()
Q=topdown_quat(np.pi/2)
q0=r.arm_q()
qa=r.ik_q([-0.056,-0.19,1.25], Q, seed=q0); print('qa', None if qa is None else qa.round(2))
qb=r.ik_q([-0.051,0.374,1.25], Q, seed=qa); print('qb', None if qb is None else qb.round(2))
qc=r.ik_q([-0.051,0.374,1.075], Q, seed=qb); print('qc', None if qc is None else qc.round(2))
assert qa is not None and qb is not None and qc is not None
assert r.move_q(qa, 2.5); print('hand', r.fk_pose()[0].round(4), 'fingers', r.fingers())
assert r.move_q(qb, 4.0); print('hand', r.fk_pose()[0].round(4), 'fingers', r.fingers())
" 2>&1 | grep -v XMLPARSER | tail -8; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 41
timeout 90 python3 cloud.py birdview >/dev/null; python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>1.10)&(z<1.20)&(x>-0.25)&(x<0.15)&(y>0.0)&(y<0.35)
print('pan pts',m.sum(),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(x[m].min(),x[m].max(),y[m].min(),y[m].max(),z[m].min(),z[m].max()))
zz=z[m]; rim=m&(z>zz.max()-0.015)
X=x[rim];Y=y[rim]
A=np.c_[2*X,2*Y,np.ones_like(X)]; b=X**2+Y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; R=np.sqrt(c[2]+cx**2+cy**2)
print('rim circle center (%.3f,%.3f) R=%.3f n=%d'%(cx,cy,R,rim.sum()))
"

# openrua op 42
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>1.09)&(z<1.20)&(x>-0.25)&(x<0.15)&(y>0.0)&(y<0.36)
for lo,hi in [(1.09,1.105),(1.105,1.115),(1.115,1.125),(1.125,1.135),(1.135,1.145),(1.145,1.16)]:
    mm=m&(z>=lo)&(z<hi)
    if mm.sum(): print(lo,hi,mm.sum(),'x[%.3f %.3f] y[%.3f %.3f] xc=%.3f yc=%.3f'%(x[mm].min(),x[mm].max(),y[mm].min(),y[mm].max(),(x[mm].min()+x[mm].max())/2,(y[mm].min()+y[mm].max())/2))
"

# openrua op 43
timeout 1500 python3 -u -c "
from rob import *
r=Robot()
Q=topdown_quat(np.pi/2)
qc=r.ik_q([-0.051,0.374,1.09], Q); assert qc is not None
assert r.move_q(qc, 3.0); print('hand', r.fk_pose()[0].round(4), 'fingers', r.fingers())
r.gripper(0.04)
qd=r.ik_q([-0.051,0.374,1.22], Q); assert r.move_q(qd, 2.5); print('hand', r.fk_pose()[0].round(4))
" 2>&1 | grep -v XMLPARSER | tail -6; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 90 python3 cloud.py birdview >/dev/null

# openrua op 44
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
m=(z>0.935)&(z<1.05)&(x>-0.25)&(x<0.15)&(y>0.05)&(y<0.45)
print('pan pts',m.sum(),'z[%.3f %.3f]'%(z[m].min(),z[m].max()))
for lo,hi in [(0.935,0.95),(0.95,0.96),(0.96,0.97),(0.97,0.98),(0.98,1.0),(1.0,1.05)]:
    mm=m&(z>=lo)&(z<hi)
    if mm.sum(): print(lo,hi,mm.sum(),'x[%.3f %.3f] y[%.3f %.3f] xc=%.3f yc=%.3f'%(x[mm].min(),x[mm].max(),y[mm].min(),y[mm].max(),(x[mm].min()+x[mm].max())/2,(y[mm].min()+y[mm].max())/2))
# rim circle
zz=z[m]; rim=m&(z>zz.max()-0.008)&(y<0.31)
X=x[rim];Y=y[rim]
A=np.c_[2*X,2*Y,np.ones_like(X)]; b=X**2+Y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; R=np.sqrt(c[2]+cx**2+cy**2)
print('rim circle center (%.3f,%.3f) R=%.3f  stove plate center (-0.064,0.200)'%(cx,cy,R))
"; timeout 60 python3 tools/perception/cam_snap.py frontview; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 45
timeout 1200 python3 -u -c "
from rob import *
r=Robot()
q=r.ik_q([-0.25,0.05,1.32], topdown_quat(0)); assert q is not None
r.move_q(q, 3.0); print('hand', r.fk_pose()[0].round(4))
" 2>&1 | grep -v XMLPARSER | tail -2; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py frontview; timeout 90 python3 cloud.py birdview > /dev/null

# openrua op 46
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
# pan: exclude knob region x<-0.16
m=(z>0.955)&(z<0.975)&(x>-0.16)&(x<0.1)&(y>0.05)&(y<0.32)
X=x[m];Y=y[m]
A=np.c_[2*X,2*Y,np.ones_like(X)]; b=X**2+Y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; R=np.sqrt(c[2]+cx**2+cy**2)
print('pan rim: center (%.3f,%.3f) R=%.3f rim z %.3f..%.3f (stove top 0.930 -> pan bottom on stove)'%(cx,cy,R,z[m].min(),z[m].max()))
print('stove plate center (-0.064,0.200), x[-0.157,0.029] y[0.107,0.294]; pan offset from plate center: (%.3f,%.3f)'%(cx+0.064,cy-0.200))
# knob tab orientation
k=(z>0.945)&(z<0.98)&(x>-0.30)&(x<-0.16)&(y>0.13)&(y<0.27)
X=x[k]-x[k].mean(); Y=y[k]-y[k].mean()
u,s,vt=np.linalg.svd(np.c_[X,Y],full_matrices=False)
ang=np.degrees(np.arctan2(vt[0,1],vt[0,0]))
print('knob tab: n=%d, principal axis angle from +x = %.1f deg (was 0 deg before turning)'%(k.sum(),ang))
"
