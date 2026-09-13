#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools /workspace/.ros

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -v -E "parameter|describe_|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 5
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview sideview birdview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 30 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | sort -u

# openrua op 7
timeout 60 python3 tools/perception/px2world.py agentview 357 205; timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 235 240; timeout 60 python3 tools/perception/px2world.py birdview 300 300

# openrua op 8
mkdir -p "$(dirname /workspace/pc.py)"
cat > /workspace/pc.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera; save world-frame point cloud.

Usage: python3 pc.py <camera>   -> <camera>_xyz.npy (H,W,3 world), <camera>.png
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
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
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
    node = rclpy.create_node("pc_" + cam)
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pts = np.stack([X, Y, depth], -1) @ R.T + p0
    pts[~np.isfinite(depth) | (depth <= 0)] = np.nan
    np.save(f"{cam}_xyz.npy", pts)
    cv2.imwrite(f"{cam}.png", color)
    print(cam, "cam pose", p0, "saved", pts.shape)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 9
for c in birdview agentview robot0_eye_in_hand sideview frontview; do timeout 90 python3 pc.py $c; done

# openrua op 10
python3 -c "
import numpy as np, cv2
P=np.load('birdview_xyz.npy'); img=cv2.imread('birdview.png')
z=P[...,2]
print('z percentiles', np.nanpercentile(z,[1,5,50,90,99]))
# table height: mode
h,edges=np.histogram(z[np.isfinite(z)],bins=200)
top=np.argsort(h)[-6:]
for i in top: print('z bin',edges[i],h[i])
# birdview: u->world y, v->world x. Height map in region of the table
# cabinet region: left of image
for name,(u0,u1,v0,v1) in {'cabinet':(190,300,220,320),'bowl':(290,340,230,290),'shelf':(370,450,255,345)}.items():
    sub=P[v0:v1,u0:u1]
    print(name,'x',np.nanmin(sub[...,0]),np.nanmax(sub[...,0]),'y',np.nanmin(sub[...,1]),np.nanmax(sub[...,1]),'z',np.nanmin(sub[...,2]),np.nanmax(sub[...,2]))
"


# openrua op 11
python3 -c "
import numpy as np, cv2
res=0.005
x0,x1,y0,y1=-0.6,0.5,-0.7,0.7
W=int((y1-y0)/res); H=int((x1-x0)/res)
hm=np.full((H,W),np.nan); col=np.zeros((H,W,3),np.uint8)
for cam in ['birdview','agentview','sideview','frontview','robot0_eye_in_hand']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); img=cv2.imread(cam+'.png').reshape(-1,3)
    ok=np.isfinite(P[:,0])&(P[:,2]<1.5)
    P=P[ok]; img=img[ok]
    i=((P[:,0]-x0)/res).astype(int); j=((P[:,1]-y0)/res).astype(int)
    m=(i>=0)&(i<H)&(j>=0)&(j<W)
    for ii,jj,z,c in zip(i[m],j[m],P[m,2],img[m]):
        if np.isnan(hm[ii,jj]) or z>hm[ii,jj]: hm[ii,jj]=z; col[ii,jj]=c
np.save('heightmap.npy',hm)
vis=np.nan_to_num((hm-0.85)/0.5,nan=0).clip(0,1)
vis=(cv2.applyColorMap((vis*255).astype(np.uint8),cv2.COLORMAP_JET))
vis[np.isnan(hm)]=0
# grid lines every 0.1m
for xv in np.arange(-0.6,0.51,0.1):
    i=int((xv-x0)/res); cv2.line(vis,(0,i),(W-1,i),(255,255,255),1); cv2.putText(vis,f'{xv:.1f}',(2,i-2),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
for yv in np.arange(-0.7,0.71,0.1):
    j=int((yv-y0)/res); cv2.line(vis,(j,0),(j,H-1),(255,255,255),1); cv2.putText(vis,f'{yv:.1f}',(j+2,12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
cv2.imwrite('heightmap.png',cv2.resize(vis,(W*3,H*3),interpolation=cv2.INTER_NEAREST))
cv2.imwrite('topcolor.png',cv2.resize(col,(W*3,H*3),interpolation=cv2.INTER_NEAREST))
print('ok',H,W)
"


# openrua op 12
python3 -c "
import numpy as np
hm=np.load('heightmap.npy'); res=0.005; x0,y0=-0.6,-0.7
def q(xa,xb,ya,yb):
    sub=hm[int((xa-x0)/res):int((xb-x0)/res), int((ya-y0)/res):int((yb-y0)/res)]
    return np.nanpercentile(sub,[5,50,95]) if np.isfinite(sub).any() else None
print('table', q(0.3,0.4,0.0,0.3))
print('cabinet top', q(-0.15,0.0,-0.45,-0.32))
print('drawer interior', q(-0.15,0.0,-0.27,-0.12))
# scan along y at x=-0.08 to see the drawer profile
i=int((-0.08-x0)/res)
for j in range(int((-0.5-y0)/res), int((0.15-y0)/res),2):
    print(f'y={y0+j*res:.3f} z={hm[i,j]:.3f}')
"


# openrua op 13
python3 -c "
import numpy as np
hm=np.load('heightmap.npy'); res=0.005; x0,y0=-0.6,-0.7
# scan along x at y=-0.15 (drawer open region)
j=int((-0.15-y0)/res)
print('--- x scan at y=-0.15')
for i in range(int((-0.35-x0)/res), int((0.15-x0)/res),2):
    print(f'x={x0+i*res:.3f} z={hm[i,j]:.3f}', end=' | ')
print()
j=int((-0.35-y0)/res)
print('--- x scan at y=-0.35 (cabinet)')
for i in range(int((-0.35-x0)/res), int((0.15-x0)/res),2):
    print(f'x={x0+i*res:.3f} z={hm[i,j]:.3f}', end=' | ')
print()
# bowl: use eye-in-hand cloud, points with z between 0.91 and 1.0 near x -0.17 y 0.08
P=np.load('robot0_eye_in_hand_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
m=(P[:,2]>0.907)&(P[:,2]<1.05)&(P[:,0]>-0.3)&(P[:,0]<0.0)&(P[:,1]>-0.05)&(P[:,1]<0.25)
B=P[m]; print('bowl pts',len(B)); print('x',B[:,0].min(),B[:,0].max(),'y',B[:,1].min(),B[:,1].max(),'z',B[:,2].min(),B[:,2].max())
print('bowl centroid', B.mean(0))
# top rim points
r=B[B[:,2]>B[:,2].max()-0.01]; print('rim x',r[:,0].min(),r[:,0].max(),'y',r[:,1].min(),r[:,1].max(),'center',r[:,:2].mean(0))
"


# openrua op 14
python3 -c "
import numpy as np, cv2
P=np.load('robot0_eye_in_hand_xyz.npy'); img=cv2.imread('robot0_eye_in_hand.png')
sub=P[170:315,165:305].reshape(-1,3); sub=sub[np.isfinite(sub[:,0])]
b=sub[sub[:,2]>0.907]
print('bowl pts',len(b),'x',b[:,0].min(),b[:,0].max(),'y',b[:,1].min(),b[:,1].max(),'z',b[:,2].min(),b[:,2].max())
rim=b[b[:,2]>b[:,2].max()-0.008]
print('rim x',rim[:,0].min(),rim[:,0].max(),'y',rim[:,1].min(),rim[:,1].max(),'center',rim[:,:2].mean(0), 'n',len(rim))
cx,cy=rim[:,:2].mean(0); r=np.hypot(rim[:,0]-cx,rim[:,1]-cy); print('rim radius pct',np.percentile(r,[5,50,95]))
# also from birdview: region
Pb=np.load('birdview_xyz.npy')
# birdview u->y, v->x ; find pixels with z in (0.907,1.0) and x in [-0.25,-0.05], y in[-0.1,0.15]
m=(Pb[...,2]>0.907)&(Pb[...,2]<1.0)&(Pb[...,0]>-0.3)&(Pb[...,0]<0.0)&(Pb[...,1]>-0.1)&(Pb[...,1]<0.15)
bb=Pb[m]; print('birdview bowl-ish pts',len(bb)); 
if len(bb): print('x',bb[:,0].min(),bb[:,0].max(),'y',bb[:,1].min(),bb[:,1].max(),'zmax',bb[:,2].max())
"


# openrua op 15
python3 -c "
import numpy as np, cv2
for cam,(u0,u1,v0,v1) in {'sideview':(325,395,255,305),'agentview':(320,395,180,235),'frontview':(320,380,290,335)}.items():
    P=np.load(cam+'_xyz.npy')[v0:v1,u0:u1].reshape(-1,3); P=P[np.isfinite(P[:,0])]
    b=P[(P[:,2]>0.905)&(P[:,2]<1.1)&(P[:,0]>-0.3)&(P[:,0]<0.05)&(P[:,1]>-0.1)&(P[:,1]<0.2)]
    print(cam,len(b),'x',b[:,0].min(),b[:,0].max(),'y',b[:,1].min(),b[:,1].max(),'z',b[:,2].min(),b[:,2].max())
    top=b[b[:,2]>b[:,2].max()-0.01]; print('  top center',top[:,:2].mean(0))
"


# openrua op 16
python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side.png', cv2.resize(im[180:330,180:420],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[120:340,20:400],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"

# openrua op 17
python3 -c "
import numpy as np
for cam in ['birdview','agentview','sideview','frontview','robot0_eye_in_hand']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    cab=P[(P[:,0]>-0.2)&(P[:,0]<0.0)&(P[:,1]>-0.42)&(P[:,1]<-0.25)&(P[:,2]>1.05)]
    tab=P[(P[:,0]>0.2)&(P[:,0]<0.4)&(P[:,1]>-0.2)&(P[:,1]<0.2)&(P[:,2]>0.8)&(P[:,2]<1.0)]
    bowl=P[(P[:,0]>-0.23)&(P[:,0]<-0.11)&(P[:,1]>0.0)&(P[:,1]<0.13)&(P[:,2]>0.905)&(P[:,2]<1.1)]
    print(cam,'cab top',np.percentile(cab[:,2],[50,99]) if len(cab) else None,'table',np.median(tab[:,2]) if len(tab) else None,'bowl z',np.percentile(bowl[:,2],[1,50,99]) if len(bowl) else None)
"


# openrua op 18
timeout 20 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once

# openrua op 19
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Controller helpers for the Panda: FK/IK (MoveIt), trajectories, gripper,
servo bursts, joint-state and camera reads. World<->base conversion built in
(base panda_link0 at world (-0.66, 0, 0.912), identity rotation).
"""
import sys
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE_W = np.array([-0.66, 0.0, 0.912])
TCP_OFF = 0.1034


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    m = R
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(m))
    if i == 0:
        s = np.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s])
    s = np.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s])


def down_quat(yaw):
    """Hand z pointing straight down (-world z); yaw = rotation of the hand
    x-axis about world z. Fingers close along the hand y-axis."""
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])  # x=(c,s,0), y=(s,-c,0), z=(0,0,-1)
    return R_quat(R)


class Ctl:
    def __init__(self, name="ctl"):
        rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._wr_cb, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.wait_js()

    def _js_cb(self, m):
        self.js = m

    def _wr_cb(self, m):
        self.wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self.js = None
        end = time.time() + 15
        while self.js is None and time.time() < end:
            self.spin(0.2)
        return self.joints()

    def joints(self):
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def wrench(self):
        self.wr = None
        end = time.time() + 5
        while self.wr is None and time.time() < end:
            self.spin(0.2)
        f = self.wr.wrench.force
        return np.array([f.x, f.y, f.z])

    # ---------- kinematics ----------
    def fk_world(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, self.arm_q() if q is None else q))
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if r is None else r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def ik_world(self, pos_w, quat, seed=None, at_tcp=False, timeout=20.0):
        pos_w = np.array(pos_w, float)
        if at_tcp:
            pos_w = pos_w - TCP_OFF * quat_R(quat)[:, 2]
        p_b = pos_w - BASE_W
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, p_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(map(float, self.arm_q() if seed is None else seed))
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            raise RuntimeError("IK no answer")
        if r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    # ---------- motion ----------
    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        wps = (waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=list(map(float, qq)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        self.wait_js()
        err = np.abs(self.arm_q() - np.array(q)).max()
        print(f"move_q: error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_pose(self, pos_w, quat, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik_world(pos_w, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            print(f"IK FAILED for {pos_w} {quat}", flush=True)
            return None
        code, err = self.move_q(q, seconds)
        pos, qq = self.fk_world()
        print(f"  hand now at {pos.round(4)} quat {qq.round(3)}", flush=True)
        return q

    def gripper(self, width, effort=30.0):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(effort)
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res_fut = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res_fut, timeout_sec=120)
        r = res_fut.result().result
        self.wait_js()
        print(f"gripper: reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}", flush=True)
        return r

    def servo(self, v_lin, n=20, frame="panda_link0"):
        msg = TwistStamped()
        msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v_lin)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)
        self.wait_js()

    def snap(self, cam, out=None):
        from cv_bridge import CvBridge
        import cv2
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("m", m), 1)
        end = time.time() + 20
        while "m" not in got and time.time() < end:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        img = CvBridge().imgmsg_to_cv2(got["m"], "bgr8")
        cv2.imwrite(out or f"{cam}.png", img)
        return img

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
OPENRUA_EOF

# openrua op 20
timeout 120 python3 -c "
from ctl import *
c=Ctl('probe')
print('q',c.arm_q().round(4)); print('fingers',c.fingers())
pos,q=c.fk_world(); print('hand world',pos.round(4),'quat',q.round(4)); print('R',quat_R(q).round(3))
pos8,_=c.fk_world(link='panda_link8'); print('link8',pos8.round(4))
print('down_quat(0)',down_quat(0).round(4),'down_quat(pi/4)',down_quat(np.pi/4).round(4))
c.close()
"

# openrua op 21
timeout 200 python3 -c "
from ctl import *
c=Ctl('probe')
q0=c.arm_q()
pos,q=c.fk_world()
raw=pos-BASE_W
print('raw fk',raw.round(4))
# hypothesis A: raw fk is base-frame -> ik_world with pos=raw+BASE_W
sA=c.ik_world(raw+BASE_W,q)
print('A (raw as base):',None if sA is None else sA.round(3))
# hypothesis B: raw fk is world-frame -> ik pos in base = raw - BASE_W -> ik_world with pos=raw
sB=c.ik_world(raw,q)
print('B (raw as world):',None if sB is None else sB.round(3))
print('current',q0.round(3))
c.close()
"

# openrua op 22
timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A1 Translation | head -3; timeout 15 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -E "Translation|Invalid|unconnected" | head -3; timeout 15 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 Translation | head -4

# openrua op 23
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "BASE_W = np.array([-0.66, 0.0, 0.912])", "new_string": "# FK/IK on this machine already answer in world coordinates (verified:\\n# FK hand pose == TF world->panda_hand), so no base offset is applied.\\nBASE_W = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 24
timeout 300 python3 -c "
from ctl import *
c=Ctl('wake')
c.gripper(0.04)
c.gripper(0.04)
c.close()
" && for cam in birdview agentview sideview; do timeout 60 python3 pc.py $cam; done && python3 -c "
import numpy as np
for cam in ['birdview','agentview','sideview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    bowl=P[(P[:,0]>-0.25)&(P[:,0]<-0.09)&(P[:,1]>-0.02)&(P[:,1]<0.15)&(P[:,2]>0.905)&(P[:,2]<1.1)]
    print(cam,'bowl z',np.percentile(bowl[:,2],[1,50,99]).round(4) if len(bowl) else None,'x',bowl[:,0].min().round(3),bowl[:,0].max().round(3),'y',bowl[:,1].min().round(3),bowl[:,1].max().round(3))
"

# openrua op 25
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
# handle region: y in [-0.09,-0.05], z between 0.93 and 1.0
h=P[(P[:,1]>-0.095)&(P[:,1]<-0.04)&(P[:,2]>0.93)&(P[:,2]<1.0)&(P[:,0]>-0.3)&(P[:,0]<0.1)]
print('handle pts',len(h),'x',h[:,0].min().round(3),h[:,0].max().round(3),'y',h[:,1].min().round(3),h[:,1].max().round(3),'z',h[:,2].min().round(3),h[:,2].max().round(3))
import collections
xs=np.round(h[:,0],2); print(sorted(collections.Counter(xs).items()))
# drawer front wall top (z~0.985) y in [-0.12,-0.09]
w=P[(P[:,1]>-0.13)&(P[:,1]<-0.085)&(P[:,2]>0.97)&(P[:,2]<1.0)&(P[:,0]>-0.3)&(P[:,0]<0.1)]
print('front wall x',w[:,0].min().round(3),w[:,0].max().round(3),'y',w[:,1].min().round(3),w[:,1].max().round(3))
# bottle: tall thing near x 0.07 y -0.2
b=P[(P[:,0]>0.0)&(P[:,0]<0.2)&(P[:,1]>-0.3)&(P[:,1]<-0.1)&(P[:,2]>0.91)]
print('bottle x',b[:,0].min().round(3),b[:,0].max().round(3),'y',b[:,1].min().round(3),b[:,1].max().round(3),'ztop',b[:,2].max().round(3))
# side walls of drawer: z~0.985 with y in[-0.22,-0.12]
s=P[(P[:,1]>-0.22)&(P[:,1]<-0.12)&(P[:,2]>0.97)&(P[:,2]<1.0)&(P[:,0]>-0.3)&(P[:,0]<0.1)]
print('side walls x values', sorted(collections.Counter(np.round(s[:,0],3)).items()))
"


# openrua op 26
cat >> /workspace/ctl.py <<'EOF'


def move_line(c, p_from, p_to, quat, seconds, n=6, at_tcp=True, seed=None):
    """Straight-ish Cartesian move: IK for n waypoints, one trajectory."""
    p_from, p_to = np.array(p_from, float), np.array(p_to, float)
    wps = []
    q = c.arm_q() if seed is None else np.array(seed)
    for i in range(1, n + 1):
        p = p_from + (p_to - p_from) * i / n
        sol = c.ik_world(p, quat, seed=q, at_tcp=at_tcp)
        if sol is None:
            print(f"move_line: IK failed at waypoint {i} {p}", flush=True)
            return None
        # guard against branch flips
        if np.abs(sol - q).max() > 1.0:
            print(f"move_line: branch jump at waypoint {i}: {np.abs(sol-q).round(2)}", flush=True)
            return None
        q = sol
        wps.append((sol, seconds * i / n))
    last = wps.pop()
    code, err = c.move_q(last[0], last[1], waypoints=wps)
    pos, qq = c.fk_world()
    tcp = pos + TCP_OFF * quat_R(qq)[:, 2]
    print(f"  tcp now {tcp.round(4)}", flush=True)
    return last[0]


def tcp_pose(c):
    pos, qq = c.fk_world()
    return pos + TCP_OFF * quat_R(qq)[:, 2], qq
EOF
timeout 300 python3 -c "
from ctl import *
c=Ctl('step1')
print('tcp now', tcp_pose(c)[0].round(4))
Q=down_quat(np.pi/2)
# test IK feasibility for key poses
for name,p in {'above_drawer':(-0.106,-0.15,1.10),'in_drawer':(-0.106,-0.15,0.95),'pulled':(-0.106,-0.03,0.95),'above_bowl':(-0.116,0.067,1.08),'at_bowl':(-0.116,0.067,0.928)}.items():
    s=c.ik_world(p,Q,at_tcp=True)
    print(name, None if s is None else s.round(3))
c.close()
"

# openrua op 27
mkdir -p "$(dirname /workspace/drawer_state.py)"
cat > /workspace/drawer_state.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Report drawer front-wall position and cavity extent from a fresh birdview
depth frame. Prints y of the front wall (outer/inner faces) at x=-0.18."""
import subprocess, sys
import numpy as np
subprocess.run([sys.executable, "pc.py", "birdview"], check=True, capture_output=True)
P = np.load("birdview_xyz.npy").reshape(-1, 3)
P = P[np.isfinite(P[:, 0])]
for x in (-0.18, -0.03):
    strip = P[(np.abs(P[:, 0] - x) < 0.008) & (P[:, 1] > -0.5) & (P[:, 1] < 0.2)]
    wall = strip[(strip[:, 2] > 0.97) & (strip[:, 2] < 1.0)]
    floor = strip[(strip[:, 2] > 0.915) & (strip[:, 2] < 0.935)]
    print(f"x={x}: front wall y in [{wall[:,1].min():.3f}, {wall[:,1].max():.3f}] "
          f"drawer floor y in [{floor[:,1].min():.3f}, {floor[:,1].max():.3f}]"
          if len(wall) and len(floor) else f"x={x}: wall pts {len(wall)} floor pts {len(floor)}")
# anything in the cavity above the floor?
cav = P[(P[:, 0] > -0.2) & (P[:, 0] < -0.01) & (P[:, 1] > -0.225) & (P[:, 1] < -0.0) & (P[:, 2] > 0.935) & (P[:, 2] < 0.975)]
print(f"objects in cavity (0.935<z<0.975): {len(cav)} pts",
      (f"x[{cav[:,0].min():.3f},{cav[:,0].max():.3f}] y[{cav[:,1].min():.3f},{cav[:,1].max():.3f}] zmax {cav[:,2].max():.3f}" if len(cav) else ""))
OPENRUA_EOF

# openrua op 28
timeout 120 python3 drawer_state.py

# openrua op 29
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy')
# birdview: u->y, v->x. find column/row mapping
X=P[...,0]; Y=P[...,1]; Z=P[...,2]
# choose row where x ~ -0.18
row=np.nanargmin(np.abs(np.nanmedian(X,axis=1)+0.18)); print('row',row,'x',np.nanmedian(X[row]))
for u in range(0,640,3):
    if -0.3<Y[row,u]<0.0: print(f'y={Y[row,u]:.3f} z={Z[row,u]:.3f}',end=' | ')
"


# openrua op 30
mkdir -p "$(dirname /workspace/drawer_state.py)"
cat > /workspace/drawer_state.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Report drawer front-wall y position from fresh camera frames (agentview,
sideview, birdview). Wall = points with 0.965<z<1.0 inside the drawer's x span."""
import subprocess, sys
import numpy as np
cams = sys.argv[1:] or ["agentview", "sideview", "birdview"]
for cam in cams:
    subprocess.run([sys.executable, "pc.py", cam], check=True, capture_output=True)
    P = np.load(f"{cam}_xyz.npy").reshape(-1, 3)
    P = P[np.isfinite(P[:, 0])]
    d = P[(P[:, 0] > -0.2) & (P[:, 0] < -0.02) & (P[:, 1] > -0.24) & (P[:, 1] < 0.3)]
    wall = d[(d[:, 2] > 0.965) & (d[:, 2] < 1.0)]
    floor = d[(d[:, 2] > 0.918) & (d[:, 2] < 0.932)]
    handle = d[(d[:, 2] > 0.94) & (d[:, 2] < 0.962)]
    def rng(a, i):
        return f"[{a[:,i].min():.3f},{a[:,i].max():.3f}]" if len(a) else "-"
    print(f"{cam}: wall-top(z.965-1.0) y{rng(wall,1)} n={len(wall)} | floor y{rng(floor,1)} n={len(floor)} | handle-ish y{rng(handle,1)} x{rng(handle,0)} n={len(handle)}")
OPENRUA_EOF

# openrua op 31
timeout 200 python3 drawer_state.py

# openrua op 32
timeout 900 python3 -u -c "
from ctl import *
c=Ctl('step1')
Q=down_quat(np.pi/2)
c.gripper(0.0)
print('wrench before', c.wrench().round(2))
q=c.move_pose((-0.106,-0.15,1.10),Q,seconds=4,at_tcp=True)
print('tcp', tcp_pose(c)[0].round(4))
q=move_line(c,(-0.106,-0.15,1.10),(-0.106,-0.15,0.955),Q,seconds=3,n=4)
print('wrench in cavity', c.wrench().round(2))
c.close()
" 2>&1 | grep -v Warning

# openrua op 33
timeout 100 python3 -u -c "
from ctl import *
c=Ctl('chk')
print('q',c.arm_q().round(3)); print('target was', np.array([-0.019,0.265,-0.251,-2.117,0.094,2.372,-1.9]))
c.close()
" 2>&1 | grep -v Warning

# openrua op 34
timeout 300 python3 -u -c "
from ctl import *
c=Ctl('j7')
q=c.arm_q(); q[6]=-1.0
c.move_q(q,3.0); print(c.arm_q().round(3))
q[6]=-1.9
c.move_q(q,3.0); print(c.arm_q().round(3))
print(tcp_pose(c))
c.close()
" 2>&1 | grep -v Warning

# openrua op 35
timeout 100 python3 -u -c "
from ctl import *
c=Ctl('fkchk')
q=c.arm_q()
for j7 in [-1.9,-1.11,0.041,0.785]:
    qq=q.copy(); qq[6]=j7
    pos,qu=c.fk_world(qq); print(j7, pos.round(4), qu.round(4), 'yaw=',(2*np.arctan2(qu[1],qu[0])).round(3))
print('js j7 now', c.arm_q()[6])
c.close()
" 2>&1 | grep -v Warning

# openrua op 36
timeout 100 python3 -u -c "
from ctl import *
c=Ctl('ikchk')
for yaw in [0, np.pi/2, np.pi/4]:
    Q=down_quat(yaw)
    s=c.ik_world((-0.106,-0.15,1.10),Q,at_tcp=True)
    pos,qh=c.fk_world(s); pos8,q8=c.fk_world(s,link='panda_link8')
    print('req yaw',round(yaw,3),'-> hand yaw',(2*np.arctan2(qh[1],qh[0])).round(3),'link8 yaw',(2*np.arctan2(q8[1],q8[0])).round(3), 'j7',s[6].round(3))
c.close()
" 2>&1 | grep -v Warning

# openrua op 37
python3 - <<'EOF'
import re
s=open('ctl.py').read()
s=s.replace('''        pos_w = np.array(pos_w, float)
        if at_tcp:
            pos_w = pos_w - TCP_OFF * quat_R(quat)[:, 2]
        p_b = pos_w - BASE_W''','''        pos_w = np.array(pos_w, float)
        if at_tcp:
            pos_w = pos_w - TCP_OFF * quat_R(quat)[:, 2]
        p_b = pos_w - BASE_W
        # machine fact: the IK tip link is panda_link8, which is rotated
        # -45deg about the hand z-axis relative to panda_hand (FK/camera
        # frame). Convert the requested HAND orientation to link8.
        quat = hand_to_link8(quat)''')
s=s.replace('''class Ctl:''','''def rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def hand_to_link8(q_hand):
    return R_quat(quat_R(q_hand) @ rotz(np.pi / 4))


class Ctl:''')
# move_q: retry on tolerance violation with lag
s=s.replace('''        code = res.result().result.error_code
        self.wait_js()
        err = np.abs(self.arm_q() - np.array(q)).max()
        print(f"move_q: error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err''','''        code = res.result().result.error_code
        self.wait_js()
        err = np.abs(self.arm_q() - np.array(q)).max()
        print(f"move_q: error_code={code} max_joint_err={err:.4f}", flush=True)
        if err > 0.02 and retries > 0:
            # controller lag (joint 7 is slow on this machine): resend
            return self.move_q(q, max(seconds, 3.0), retries=retries - 1)
        return code, err''')
s=s.replace('def move_q(self, q, seconds=3.0, waypoints=None):','def move_q(self, q, seconds=3.0, waypoints=None, retries=3):')
# duration budget for slow joint 7 (~0.19 rad/s observed)
s=s.replace('''        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        wps = (waypoints or []) + [(q, seconds)]''','''        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        dj7 = abs(self.arm_q()[6] - q[6])
        need = dj7 / 0.15
        if need > seconds and not waypoints:
            print(f"move_q: stretching duration {seconds}->{need:.1f}s for joint7 travel {dj7:.2f}", flush=True)
            seconds = need
        wps = (waypoints or []) + [(q, seconds)]''')
open('ctl.py','w').write(s)
EOF
timeout 100 python3 -u -c "
from ctl import *
c=Ctl('ikchk')
for yaw in [0, np.pi/2]:
    s=c.ik_world((-0.106,-0.15,1.10),down_quat(yaw),at_tcp=True)
    pos,qh=c.fk_world(s); print('req hand yaw',round(yaw,3),'-> FK hand yaw',(2*np.arctan2(qh[1],qh[0])).round(3),'j7',s[6].round(3),'pos',pos.round(4))
print('current q', c.arm_q().round(3))
c.close()
" 2>&1 | grep -v Warning

# openrua op 38
timeout 900 python3 -u -c "
from ctl import *
c=Ctl('step1b')
Q=down_quat(np.pi/2)
print('fingers',c.fingers(),'tcp',tcp_pose(c)[0].round(4))
q=move_line(c,(-0.106,-0.15,1.10),(-0.106,-0.15,0.955),Q,seconds=3,n=4)
print('wrench in cavity', c.wrench().round(2))
c.close()
" 2>&1 | grep -v Warning

# openrua op 39
timeout 100 python3 tools/perception/cam_snap.py agentview && timeout 100 python3 tools/perception/cam_snap.py sideview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[100:340,20:420],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side.png', cv2.resize(im[150:350,200:480],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))"

# openrua op 40
mkdir -p "$(dirname /workspace/drawer_state.py)"
cat > /workspace/drawer_state.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Report the drawer front-face y position from fresh camera frames.
Front face = max-y of points with x in [-0.195,-0.155] (beside the handle),
z in [0.92,0.98] (the drawer front, below the hand body)."""
import subprocess, sys
import numpy as np
cams = sys.argv[1:] or ["sideview", "agentview"]
for cam in cams:
    subprocess.run([sys.executable, "pc.py", cam], check=True, capture_output=True)
    P = np.load(f"{cam}_xyz.npy").reshape(-1, 3)
    P = P[np.isfinite(P[:, 0])]
    for xr in [(-0.195, -0.155), (-0.06, -0.02)]:
        f = P[(P[:, 0] > xr[0]) & (P[:, 0] < xr[1]) & (P[:, 2] > 0.92) & (P[:, 2] < 0.98) & (P[:, 1] > -0.3) & (P[:, 1] < 0.4)]
        if len(f):
            ys = np.sort(f[:, 1])
            print(f"{cam} x{xr}: front-face y ~ {np.percentile(ys, 98):.3f} (max {ys.max():.3f}), n={len(f)}")
        else:
            print(f"{cam} x{xr}: no points")
    h = P[(P[:, 0] > -0.14) & (P[:, 0] < -0.06) & (P[:, 2] > 0.93) & (P[:, 2] < 0.975) & (P[:, 1] > -0.3) & (P[:, 1] < 0.4)]
    if len(h):
        print(f"{cam} handle-bar region: max y {np.percentile(h[:,1],98):.3f}")
OPENRUA_EOF

# openrua op 41
timeout 200 python3 drawer_state.py

# openrua op 42
sed -i 's/(P\[:, 1\] < 0.4)/(P[:, 1] < 0.005)/g' drawer_state.py && timeout 200 python3 drawer_state.py

# openrua op 43
timeout 1500 python3 -u -c "
from ctl import *
import subprocess
c=Ctl('pull')
Q=down_quat(np.pi/2)
y=-0.15
for ynew in [-0.12,-0.09,-0.06,-0.03]:
    q=move_line(c,(-0.106,y,0.955),(-0.106,ynew,0.955),Q,seconds=2.5,n=3)
    if q is None: break
    y=ynew
    print('wrench', c.wrench().round(2))
    out=subprocess.run(['python3','drawer_state.py','sideview'],capture_output=True,text=True).stdout
    print(out.strip())
c.close()
" 2>&1 | grep -v Warning

# openrua op 44
timeout 600 python3 -u -c "
from ctl import *
c=Ctl('release')
Q=down_quat(np.pi/2)
t=tcp_pose(c)[0]
move_line(c,t,(-0.106,-0.13,0.955),Q,seconds=2,n=2)
print('wrench', c.wrench().round(2))
move_line(c,(-0.106,-0.13,0.955),(-0.106,-0.13,1.12),Q,seconds=3,n=3)
c.close()
" 2>&1 | grep -v Warning && timeout 200 python3 drawer_state.py

# openrua op 45
timeout 600 python3 -u -c "
from ctl import *
c=Ctl('pregrasp')
Q=down_quat(np.pi/2)
c.move_pose((-0.116,0.067,1.12),Q,seconds=4,at_tcp=True)
print('tcp', tcp_pose(c)[0].round(4))
c.gripper(0.04)
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER" && timeout 100 python3 pc.py birdview && python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy'); X=P[...,0]; Y=P[...,1]; Z=P[...,2]
for xt in [-0.18,-0.106,-0.03]:
    row=np.nanargmin(np.abs(np.nanmedian(X,axis=1)-xt)); print('--- x',np.nanmedian(X[row]).round(3))
    s=''
    for u in range(0,640,2):
        if -0.26<Y[row,u]<0.0: s+=f'{Y[row,u]:.3f}:{Z[row,u]:.3f} '
    print(s)
"

# openrua op 46
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
near=P[(np.abs(P[:,0]+0.116)<0.2)&(np.abs(P[:,1]-0.067)<0.2)]
for lo,hi in [(1.10,1.16),(1.16,1.23),(1.23,1.30),(1.30,1.40),(1.40,1.6)]:
    s=near[(near[:,2]>lo)&(near[:,2]<hi)]
    if len(s): print(f'z {lo}-{hi}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
"


# openrua op 47
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('urdf')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<15: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0] if got else '')
print(len(got[0]) if got else 'none')
" 2>&1 | grep -v -E "Warning|XMLPARSER"; grep -n -A12 '<link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A8 'panda_finger_joint1' robot.urdf | head -40

# openrua op 48
python3 -c "
import numpy as np, struct
def stl_bbox(path):
    d=open(path,'rb').read()
    if d[:5]==b'solid' and b'facet' in d[:300]:
        import re
        v=np.array(re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',d),float)
    else:
        n=struct.unpack_from('<I',d,80)[0]
        arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0),v.max(0)
for f in ['hand','finger']:
    print(f, stl_bbox(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'))
"

# openrua op 49
find / -iname "hand*.stl" -o -iname "hand*.dae" -o -iname "finger*.stl" 2>/dev/null | head; find / -path /proc -prune -o -iname "*panda*" -type d -print 2>/dev/null | head

# openrua op 50
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
# bottle: x>0.03, y in [-0.3,-0.05], z>0.91
b=P[(P[:,0]>0.03)&(P[:,0]<0.2)&(P[:,1]>-0.35)&(P[:,1]<-0.02)&(P[:,2]>0.91)]
print('bottle n',len(b),'x',b[:,0].min().round(3),b[:,0].max().round(3),'y',b[:,1].min().round(3),b[:,1].max().round(3),'z',b[:,2].min().round(3),b[:,2].max().round(3))
body=b[b[:,2]<1.03]; print(' body (z<1.03) x',body[:,0].min().round(3),body[:,0].max().round(3),'y',body[:,1].min().round(3),body[:,1].max().round(3), 'center',body[:,:2].mean(0).round(3))
# middle handle: y in [-0.23,-0.19], z>1.0, x in [-0.25,0.05]
h=P[(P[:,0]>-0.25)&(P[:,0]<0.05)&(P[:,1]>-0.228)&(P[:,1]<-0.15)&(P[:,2]>1.0)]
print('mid handle n',len(h),'x',h[:,0].min().round(3),h[:,0].max().round(3),'y',h[:,1].min().round(3),h[:,1].max().round(3),'z',h[:,2].min().round(3),h[:,2].max().round(3))
# shelf: x in [-0.2,0.2], y in [0.15,0.5], z>0.91
s=P[(P[:,0]>-0.25)&(P[:,0]<0.25)&(P[:,1]>0.14)&(P[:,1]<0.5)&(P[:,2]>0.91)]
print('shelf n',len(s),'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3),'z',s[:,2].min().round(3),s[:,2].max().round(3))
import collections
print(' shelf z histogram', sorted(collections.Counter(np.round(s[:,2],2)).items()))
lo=s[s[:,2]<1.1]; print(' lower tier x',lo[:,0].min().round(3),lo[:,0].max().round(3),'y',lo[:,1].min().round(3),lo[:,1].max().round(3))
# side view for shelf lower tier
S=np.load('sideview_xyz.npy').reshape(-1,3); S=S[np.isfinite(S[:,0])]
s2=S[(S[:,0]>-0.25)&(S[:,0]<0.25)&(S[:,1]>0.14)&(S[:,1]<0.5)&(S[:,2]>0.91)&(S[:,2]<1.15)]
print(' sideview shelf lower z hist', sorted(collections.Counter(np.round(s2[:,2],2)).items()))
"


# openrua op 51
timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[100:360,0:420],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))"

# openrua op 52
python3 -c "
import cv2, numpy as np
im=cv2.imread('birdview.png'); P=np.load('birdview_xyz.npy')
# annotate: find pixel for world points
def px(x,y):
    d=np.hypot(P[...,0]-x,P[...,1]-y); i=np.nanargmin(d); return (i%640, i//640)
for (x,y) in [(-0.105,-0.16),(0.08,-0.19),(-0.171,0.067)]:
    u,v=px(x,y); cv2.circle(im,(u,v),4,(0,0,255),1); print((x,y),'->',(u,v))
cv2.imwrite('crop_bird.png', cv2.resize(im[180:340,150:400],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))"

# openrua op 53
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy')
sub=P[285:320,285:320].reshape(-1,3); sub=sub[np.isfinite(sub[:,0])]
b=sub[sub[:,2]>0.95]
print('bottle n',len(b),'x',b[:,0].min().round(3),b[:,0].max().round(3),'y',b[:,1].min().round(3),b[:,1].max().round(3),'z',b[:,2].min().round(3),b[:,2].max().round(3))
top=b[b[:,2]>1.05]; print('top center', top[:,:2].mean(0).round(3), 'n',len(top))
body=b[(b[:,2]>0.95)&(b[:,2]<1.03)]; print('body x',body[:,0].min().round(3),body[:,0].max().round(3),'y',body[:,1].min().round(3),body[:,1].max().round(3))
"


# openrua op 54
timeout 60 python3 pc.py frontview >/dev/null && python3 -c "
import cv2, numpy as np
im=cv2.imread('frontview.png'); cv2.imwrite('crop_front.png', cv2.resize(im[230:420,60:420],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
P=np.load('frontview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
b=P[(P[:,0]>-0.02)&(P[:,0]<0.2)&(P[:,1]>-0.2)&(P[:,1]<0.0)&(P[:,2]>0.99)&(P[:,2]<1.3)]
print('bottle-ish n',len(b),'x',b[:,0].min().round(3),b[:,0].max().round(3),'y',b[:,1].min().round(3),b[:,1].max().round(3),'z',b[:,2].min().round(3),b[:,2].max().round(3))
import collections; print(sorted(collections.Counter(np.round(b[:,2],2)).items()))
"

# openrua op 55
python3 -c "
import numpy as np, collections
for cam in ['birdview','frontview','agentview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    h=P[(P[:,0]>-0.25)&(P[:,0]<0.05)&(P[:,1]>-0.232)&(P[:,1]<-0.19)&(P[:,2]>1.0)&(P[:,2]<1.12)]
    print(cam,'mid handle n',len(h))
    if len(h):
        print('  z hist',sorted(collections.Counter(np.round(h[:,2],2)).items()))
        print('  y hist',sorted(collections.Counter(np.round(h[:,1],2)).items()))
        print('  x range',h[:,0].min().round(3),h[:,0].max().round(3))
    # upper handle too
    h2=P[(P[:,0]>-0.25)&(P[:,0]<0.05)&(P[:,1]>-0.232)&(P[:,1]<-0.19)&(P[:,2]>1.05)&(P[:,2]<1.2)]
    if len(h2): print('  upper z hist',sorted(collections.Counter(np.round(h2[:,2],2)).items()))
"


# openrua op 56
python3 -c "
import numpy as np, collections
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
h=P[(P[:,0]>-0.25)&(P[:,0]<0.05)&(P[:,1]>-0.218)&(P[:,1]<-0.19)&(P[:,2]>0.99)&(P[:,2]<1.13)]
print('bar pts n',len(h))
print('  z hist',sorted(collections.Counter(np.round(h[:,2],2)).items()))
print('  x hist',sorted(collections.Counter(np.round(h[:,0],2)).items()))
lo=h[h[:,2]<1.06]; print('  low part (z<1.06): x',lo[:,0].min().round(3),lo[:,0].max().round(3),'y',lo[:,1].min().round(3),lo[:,1].max().round(3))
hi=h[h[:,2]>1.08]; print('  high part (z>1.08): x',hi[:,0].min().round(3),hi[:,0].max().round(3),'y',hi[:,1].min().round(3),hi[:,1].max().round(3))
"


# openrua op 57
timeout 60 python3 pc.py sideview >/dev/null; python3 -c "
import numpy as np, collections
P=np.load('sideview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
h=P[(P[:,0]>-0.15)&(P[:,0]<-0.05)&(P[:,1]>-0.226)&(P[:,1]<-0.10)&(P[:,2]>0.9)&(P[:,2]<1.13)]
print('protruding pts n',len(h))
zs=np.round(h[:,2]/0.005)*0.005
print(sorted(collections.Counter(np.round(zs,3)).items()))
for z0 in [1.02,1.09]:
    s=h[np.abs(h[:,2]-z0)<0.012]; print(z0,'y range',s[:,1].min().round(3),s[:,1].max().round(3),'x',s[:,0].min().round(3),s[:,0].max().round(3))
"


# openrua op 58
python3 -c "
import numpy as np, collections
for cam in ['agentview','frontview','birdview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    h=P[(P[:,0]>-0.15)&(P[:,0]<-0.05)&(P[:,1]>-0.226)&(P[:,1]<-0.12)&(P[:,2]>0.99)&(P[:,2]<1.13)]
    zs=np.round(h[:,2]/0.005)*0.005
    print(cam, sorted(collections.Counter(np.round(zs,3)).items()))
    for z0 in [1.02,1.09]:
        s=h[np.abs(h[:,2]-z0)<0.015]
        if len(s): print('  z~',z0,'y range',s[:,1].min().round(3),s[:,1].max().round(3),'x',s[:,0].min().round(3),s[:,0].max().round(3))
"


# openrua op 59
cat >> /workspace/ctl.py <<'EOF'


def rotx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def tilt_quat(yaw, alpha):
    """down_quat(yaw) then tilted about the WORLD x-axis so the hand z-axis
    (finger direction) points down and toward -y by angle alpha; the wrist
    leans toward +y."""
    return R_quat(rotx(-alpha) @ quat_R(down_quat(yaw)))
EOF
timeout 300 python3 -u -c "
from ctl import *
c=Ctl('iktest')
tests={
 'place_pre_a10': ((-0.055,-0.161,1.06), tilt_quat(np.pi/2, np.radians(10))),
 'place_a10': ((-0.055,-0.161,0.965), tilt_quat(np.pi/2, np.radians(10))),
 'push_start_a90': ((-0.20,-0.03,0.96), tilt_quat(np.pi/2, np.radians(90))),
 'push_end_a90': ((-0.20,-0.222,0.96), tilt_quat(np.pi/2, np.radians(90))),
 'push_start_a75': ((-0.20,-0.03,0.955), tilt_quat(np.pi/2, np.radians(75))),
 'push_end_a75': ((-0.20,-0.222,0.955), tilt_quat(np.pi/2, np.radians(75))),
 'push_end_a90_x-0.15': ((-0.15,-0.222,0.96), tilt_quat(np.pi/2, np.radians(90))),
}
for k,(p,q) in tests.items():
    R=quat_R(q); 
    s=c.ik_world(p,q,at_tcp=True)
    print(k,'zhand',R[:,2].round(2),'yhand',R[:,1].round(2),'->',None if s is None else s.round(2))
    if s is not None:
        pos,qh=c.fk_world(s); print('   fk tcp',(pos+TCP_OFF*quat_R(qh)[:,2]).round(3),'fk zhand',quat_R(qh)[:,2].round(2))
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER"

# openrua op 60
timeout 900 python3 -u -c "
from ctl import *
c=Ctl('pick')
Q=down_quat(np.pi/2)
print('fingers',c.fingers())
t=tcp_pose(c)[0]; print('tcp',t.round(4))
move_line(c,t,(-0.116,0.067,1.03),Q,seconds=3,n=3)
move_line(c,(-0.116,0.067,1.03),(-0.116,0.067,0.927),Q,seconds=3,n=4)
print('wrench',c.wrench().round(2))
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[100:330,200:500],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))"

# openrua op 61
timeout 600 python3 -u -c "
from ctl import *
c=Ctl('grasp')
print('wrench before',c.wrench().round(2))
c.gripper(0.0)
print('fingers',c.fingers()); print('wrench after',c.wrench().round(2))
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER"

# openrua op 62
timeout 900 python3 -u -c "
from ctl import *
c=Ctl('lift')
Q=down_quat(np.pi/2)
t=tcp_pose(c)[0]
move_line(c,t,(t[0],t[1],1.06),Q,seconds=3,n=3)
print('fingers',c.fingers()); print('wrench',c.wrench().round(2))
print('tcp',tcp_pose(c)[0].round(4))
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER" && for cam in agentview sideview frontview; do timeout 60 python3 pc.py $cam; done && python3 -c "
import numpy as np
for cam in ['agentview','sideview','frontview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    # bowl: points in region around expected held bowl (x -0.25..-0.05, y -0.02..0.16, z 0.98..1.06) excluding hand (z>1.06)
    b=P[(P[:,0]>-0.3)&(P[:,0]<-0.05)&(P[:,1]>-0.05)&(P[:,1]<0.2)&(P[:,2]>0.97)&(P[:,2]<1.05)]
    if len(b):
        print(cam,'n',len(b),'x',b[:,0].min().round(3),b[:,0].max().round(3),'y',b[:,1].min().round(3),b[:,1].max().round(3),'z',b[:,2].min().round(3),b[:,2].max().round(3))
    # table check: anything left at bowl's old spot
    t=P[(P[:,0]>-0.25)&(P[:,0]<-0.1)&(P[:,1]>0.0)&(P[:,1]<0.13)&(P[:,2]>0.915)&(P[:,2]<0.97)]
    print(cam,'left on table n',len(t))
"

# openrua op 63
python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side.png', cv2.resize(im[80:330,180:520],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('frontview.png'); cv2.imwrite('crop_front.png', cv2.resize(im[80:330,150:490],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))"

# openrua op 64
python3 -c "
import numpy as np
for cam in ['agentview','sideview','frontview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    b=P[(P[:,0]>-0.3)&(P[:,0]<-0.06)&(P[:,1]>-0.05)&(P[:,1]<0.2)&(P[:,2]>0.97)&(P[:,2]<1.10)]
    print(cam,'n',len(b),'x',b[:,0].min().round(3),b[:,0].max().round(3),'y',b[:,1].min().round(3),b[:,1].max().round(3),'z',b[:,2].min().round(3),b[:,2].max().round(3))
    for z0 in np.arange(0.99,1.10,0.01):
        s=b[(b[:,2]>=z0)&(b[:,2]<z0+0.01)]
        if len(s): print(f'  z {z0:.2f}: n {len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
"

# openrua op 65
timeout 900 python3 -u -c "
from ctl import *
c=Ctl('place')
Q=down_quat(np.pi/2)
t=tcp_pose(c)[0]
move_line(c,t,(t[0],t[1],1.10),Q,seconds=2,n=2)
move_line(c,(t[0],t[1],1.10),(-0.054,-0.161,1.10),Q,seconds=4,n=5)
print('fingers',c.fingers(),'wrench',c.wrench().round(2))
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER" && for cam in agentview sideview; do timeout 60 python3 pc.py $cam; done && python3 -c "
import numpy as np, cv2
for cam in ['agentview','sideview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    b=P[(P[:,0]>-0.3)&(P[:,0]<0.0)&(P[:,1]>-0.25)&(P[:,1]<-0.05)&(P[:,2]>1.0)&(P[:,2]<1.13)]
    for z0 in np.arange(1.0,1.13,0.01):
        s=b[(b[:,2]>=z0)&(b[:,2]<z0+0.01)]
        if len(s): print(f'{cam} z {z0:.2f}: n {len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side.png', cv2.resize(im[80:330,180:520],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"

# openrua op 66
python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[120:340,180:480],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))"

# openrua op 67
timeout 900 python3 -u -c "
from ctl import *
c=Ctl('place2')
Qt=tilt_quat(np.pi/2,np.radians(10))
t=tcp_pose(c)[0]
move_line(c,t,(-0.054,-0.167,1.10),Qt,seconds=3,n=2)
print('fingers',c.fingers(),'wrench',c.wrench().round(2))
for z in [1.06,1.03,1.00]:
    t=tcp_pose(c)[0]
    move_line(c,t,(-0.054,-0.167,z),Qt,seconds=2,n=2)
    print('z',z,'fingers',c.fingers(),'wrench',c.wrench().round(2))
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER"

# openrua op 68
for cam in agentview sideview; do timeout 60 python3 pc.py $cam; done; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side.png', cv2.resize(im[100:330,180:520],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[120:340,180:480],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))"

# openrua op 69
python3 -c "
import numpy as np
for cam in ['agentview','sideview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    cab=P[(P[:,0]>-0.2)&(P[:,0]<0.0)&(P[:,1]>-0.45)&(P[:,1]<-0.235)]
    print(cam,'cabinet region z max',cab[:,2].max().round(3),'y range',cab[:,1].min().round(3),cab[:,1].max().round(3))
    for z0 in np.arange(0.92,1.12,0.01):
        s=P[(P[:,0]>-0.25)&(P[:,0]<0.05)&(P[:,1]>-0.235)&(P[:,1]<-0.07)&(P[:,2]>=z0)&(P[:,2]<z0+0.01)]
        if len(s): print(f'  z {z0:.2f}: n {len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
"

# openrua op 70
timeout 900 python3 -u -c "
from ctl import *
c=Ctl('release')
Qt=tilt_quat(np.pi/2,np.radians(10))
c.gripper(0.04)
print('wrench',c.wrench().round(2))
t=tcp_pose(c)[0]
move_line(c,t,(t[0],t[1],1.12),Qt,seconds=3,n=2)
print('wrench',c.wrench().round(2))
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER" && for cam in agentview sideview; do timeout 60 python3 pc.py $cam; done && python3 -c "
import numpy as np, cv2
for cam in ['agentview','sideview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    for z0 in np.arange(0.92,1.06,0.01):
        s=P[(P[:,0]>-0.25)&(P[:,0]<0.05)&(P[:,1]>-0.228)&(P[:,1]<-0.095)&(P[:,2]>=z0)&(P[:,2]<z0+0.01)]
        if len(s): print(f'{cam} z {z0:.2f}: n {len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[120:340,180:480],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"

# openrua op 71
python3 -c "
import numpy as np
for cam in ['agentview','sideview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    s=P[(P[:,0]>-0.19)&(P[:,0]<-0.02)&(P[:,1]>-0.22)&(P[:,1]<-0.10)&(P[:,2]>0.93)&(P[:,2]<0.99)]
    print(cam,'interior pts n',len(s),'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3),'z',s[:,2].min().round(3),s[:,2].max().round(3))
    top=s[s[:,2]>0.96]
    if len(top): print('  rim pts x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3),'center',top[:,:2].mean(0).round(3))
"

# openrua op 72
python3 -c "
import numpy as np
for cam in ['agentview','sideview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    h=P[(P[:,0]>-0.2)&(P[:,0]<0.03)&(P[:,1]>-0.068)&(P[:,1]<0.0)&(P[:,2]>0.915)&(P[:,2]<1.0)]
    print(cam,'handle n',len(h),'x',h[:,0].min().round(3),h[:,0].max().round(3),'y',h[:,1].min().round(3),h[:,1].max().round(3),'z',h[:,2].min().round(3),h[:,2].max().round(3))
    for x0 in np.arange(-0.2,0.03,0.01):
        s=h[(h[:,0]>=x0)&(h[:,0]<x0+0.01)]
        if len(s): print(f'  x {x0:.2f}: n {len(s)} y max {s[:,1].max():.3f} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
    # table surface
    t=P[(P[:,0]>-0.3)&(P[:,0]<-0.25)&(P[:,1]>-0.1)&(P[:,1]<0.1)]
    print(cam,'table z',np.median(t[:,2]).round(3))
"

# openrua op 73
python3 -c "
import numpy as np
P=np.load('agentview_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
s=P[(P[:,0]>-0.25)&(P[:,0]<0.2)&(P[:,1]>0.12)&(P[:,1]<0.6)&(P[:,2]>0.905)]
print('shelf n',len(s),'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
for z0 in np.arange(0.9,1.5,0.05):
    q=s[(s[:,2]>=z0)&(s[:,2]<z0+0.05)]
    if len(q): print(f'  z {z0:.2f}: n {len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]')
"

# openrua op 74
timeout 600 python3 -u -c "
from ctl import *
c=Ctl('chk')
q0=c.arm_q()
for a in [90,80]:
  Qt=tilt_quat(np.pi/2,np.radians(a))
  for name,p in [('start',(-0.19,-0.02,0.955)),('end',(-0.19,-0.215,0.955))]:
    q=c.ik_world(np.array(p),Qt,at_tcp=True,seed=q0)
    print(a,name,'q',None if q is None else q.round(2))
    if q is None: continue
    for l in ['panda_link3','panda_link4','panda_link5','panda_link6','panda_link7','panda_link8','panda_hand']:
        pos,_=c.fk_world(q,link=l); print('   ',l,pos.round(3))
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER"

# openrua op 75
python3 -c "
import numpy as np
for cam in ['agentview','sideview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    s=P[(P[:,0]>-0.62)&(P[:,0]<-0.25)&(P[:,1]>0.05)&(P[:,1]<0.45)&(P[:,2]>0.93)&(P[:,2]<1.5)]
    print(cam,'n',len(s))
    for z0 in np.arange(0.9,1.5,0.05):
        q=s[(s[:,2]>=z0)&(s[:,2]<z0+0.05)]
        if len(q): print(f'  z {z0:.2f}: n {len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]')
"

# openrua op 76
timeout 600 python3 -u -c "
from ctl import *
c=Ctl('chk2')
q0=c.arm_q(); print('current q',q0.round(2))
Qt=tilt_quat(np.pi/2,np.radians(80))
sols=[]
for j1 in [-0.6,-0.3,0.0,0.3,0.6,1.0]:
  for j3 in [-1.0,0.0,1.0]:
    seed=np.array([j1,0.5,j3,-2.0,0.0,2.5,0.5])
    q=c.ik_world(np.array((-0.195,-0.03,1.06)),Qt,at_tcp=True,seed=seed,timeout=5)
    if q is not None and not any(np.abs(q-s).max()<0.05 for s in sols):
        sols.append(q); print('sol',q.round(2))
        for l in ['panda_link4','panda_link6','panda_link7']:
            pos,_=c.fk_world(q,link=l); print('   ',l,pos.round(3))
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER"

# openrua op 77
timeout 900 python3 -u -c "
from ctl import *
c=Ctl('prepush')
c.gripper(0.0)
Q10=tilt_quat(np.pi/2,np.radians(10)); Qt=tilt_quat(np.pi/2,np.radians(80))
t=tcp_pose(c)[0]
move_line(c,t,(t[0],-0.10,1.20),Q10,seconds=3,n=3)
seed=np.array([-0.04,0.53,0.42,-2.2,-1.29,1.41,0.42])
q=c.ik_world(np.array((-0.195,-0.08,1.20)),Qt,at_tcp=True,seed=seed)
print('target q',q.round(2),'current',c.arm_q().round(2))
c.move_q(q,seconds=6)
print('tcp',tcp_pose(c)[0].round(3),'wrench',c.wrench().round(2))
t=tcp_pose(c)[0]
move_line(c,t,(-0.195,-0.03,0.955),Qt,seconds=5,n=5,seed=c.arm_q())
print('tcp',tcp_pose(c)[0].round(3),'wrench',c.wrench().round(2),'q',c.arm_q().round(2))
for l in ['panda_link4','panda_link6','panda_link7','panda_hand']:
    pos,_=c.fk_world(link=l); print('   ',l,pos.round(3))
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER"

# openrua op 78
timeout 60 python3 pc.py sideview >/dev/null && timeout 60 python3 pc.py agentview >/dev/null && python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side.png', cv2.resize(im[100:330,180:520],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[120:340,180:480],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))"

# openrua op 79
timeout 900 python3 -u -c "
from ctl import *
c=Ctl('push')
Qt=tilt_quat(np.pi/2,np.radians(80))
w0=c.wrench()
for y in [-0.09,-0.13,-0.17,-0.20]:
    t=tcp_pose(c)[0]
    move_line(c,t,(-0.195,y,0.955),Qt,seconds=3,n=3,seed=c.arm_q())
    w=c.wrench(); print('y',y,'tcp',tcp_pose(c)[0].round(3),'dwrench',(w-w0).round(2))
    if np.abs(w-w0).max()>25: print('high force, stop'); break
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER"; timeout 120 python3 drawer_state.py sideview agentview

# openrua op 80
timeout 90 python3 pc.py agentview; python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side.png', cv2.resize(im[100:330,180:520],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[120:340,180:480],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))"

# openrua op 81
python3 -c "
import numpy as np
for cam in ['agentview','sideview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    for xr in [(-0.05,-0.01),(-0.13,-0.08),(0.0,0.02)]:
        f=P[(P[:,0]>xr[0])&(P[:,0]<xr[1])&(P[:,2]>0.92)&(P[:,2]<0.985)&(P[:,1]>-0.3)&(P[:,1]<-0.0)]
        if len(f):
            ys=np.sort(f[:,1]); print(cam,xr,'front y 98pct',np.percentile(ys,98).round(3),'max',ys.max().round(3),'n',len(f))
    # middle drawer front for reference
    f=P[(P[:,0]>-0.05)&(P[:,0]<-0.01)&(P[:,2]>1.04)&(P[:,2]<1.10)&(P[:,1]>-0.3)&(P[:,1]<0.0)]
    if len(f): print(cam,'middle drawer front y max',f[:,1].max().round(3))
"

# openrua op 82
timeout 900 python3 -u -c "
from ctl import *
c=Ctl('push2')
Qt=tilt_quat(np.pi/2,np.radians(80))
w0=c.wrench()
for y in [-0.215,-0.224,-0.230]:
    t=tcp_pose(c)[0]
    move_line(c,t,(-0.195,y,0.955),Qt,seconds=2,n=2,seed=c.arm_q())
    w=c.wrench(); print('y',y,'tcp',tcp_pose(c)[0].round(3),'dwrench',(w-w0).round(2))
    if np.abs(w-w0).max()>8: print('contact force, stop'); break
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER"; timeout 90 python3 pc.py agentview >/dev/null; timeout 90 python3 pc.py sideview >/dev/null; python3 -c "
import numpy as np
for cam in ['agentview','sideview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    f=P[(P[:,0]>-0.05)&(P[:,0]<-0.01)&(P[:,2]>0.92)&(P[:,2]<0.985)&(P[:,1]>-0.3)&(P[:,1]<-0.0)]
    print(cam,'bottom drawer front y',np.percentile(f[:,1],98).round(3),'n',len(f))
    f=P[(P[:,0]>-0.05)&(P[:,0]<-0.01)&(P[:,2]>1.04)&(P[:,2]<1.10)&(P[:,1]>-0.3)&(P[:,1]<0.0)]
    print(cam,'middle drawer front y',f[:,1].max().round(3))
"

# openrua op 83
timeout 900 python3 -u -c "
from ctl import *
c=Ctl('retract')
Qt=tilt_quat(np.pi/2,np.radians(80))
t=tcp_pose(c)[0]
move_line(c,t,(-0.195,-0.10,0.98),Qt,seconds=3,n=3,seed=c.arm_q())
t=tcp_pose(c)[0]
move_line(c,t,(-0.195,-0.10,1.20),Qt,seconds=3,n=3,seed=c.arm_q())
print('tcp',tcp_pose(c)[0].round(3),'wrench',c.wrench().round(2))
c.close()
" 2>&1 | grep -v -E "Warning|XMLPARSER"; for cam in agentview sideview birdview frontview; do timeout 90 python3 pc.py $cam >/dev/null; done; python3 -c "
import numpy as np, cv2
for cam in ['agentview','sideview','birdview','frontview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    f=P[(P[:,0]>-0.05)&(P[:,0]<-0.01)&(P[:,2]>0.92)&(P[:,2]<0.985)&(P[:,1]>-0.3)&(P[:,1]<-0.0)]
    if len(f): print(cam,'bottom drawer front y',np.percentile(f[:,1],98).round(3),'n',len(f))
    # any bowl-like object outside the cabinet on the table? (z 0.93-1.0, in front of cabinet y>-0.23)
    o=P[(P[:,0]>-0.4)&(P[:,0]<0.3)&(P[:,1]>-0.23)&(P[:,1]<0.17)&(P[:,2]>0.93)&(P[:,2]<1.0)]
    if len(o): print(cam,'objects on table region n',len(o),'x',o[:,0].min().round(3),o[:,0].max().round(3),'y',o[:,1].min().round(3),o[:,1].max().round(3))
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side.png', cv2.resize(im[100:330,180:520],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[100:340,150:480],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"

# openrua op 84
for cam in agentview sideview birdview; do for i in 1 2 3; do timeout 90 python3 pc.py $cam >/dev/null 2>&1 && break; done; done; python3 -c "
import numpy as np, cv2
for cam in ['agentview','sideview','birdview','frontview']:
    P=np.load(cam+'_xyz.npy').reshape(-1,3); P=P[np.isfinite(P[:,0])]
    f=P[(P[:,0]>-0.05)&(P[:,0]<-0.01)&(P[:,2]>0.92)&(P[:,2]<0.985)&(P[:,1]>-0.3)&(P[:,1]<-0.0)]
    if len(f): print(cam,'bottom drawer front y',np.percentile(f[:,1],98).round(3),'n',len(f))
    h=P[(P[:,0]>-0.13)&(P[:,0]<-0.08)&(P[:,2]>0.94)&(P[:,2]<0.96)&(P[:,1]>-0.3)&(P[:,1]<-0.0)]
    if len(h): print(cam,'bottom handle front y',h[:,1].max().round(3))
    # bowl-height objects on the open table in front of cabinet (exclude cabinet y<-0.2 and bottle x>-0.02)
    o=P[(P[:,0]>-0.4)&(P[:,0]<-0.03)&(P[:,1]>-0.19)&(P[:,1]<0.17)&(P[:,2]>0.93)&(P[:,2]<1.0)]
    print(cam,'table-region pts (bowl height, excl. cabinet/bottle) n',len(o))
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side.png', cv2.resize(im[100:330,180:520],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[100:340,150:480],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('birdview.png'); cv2.imwrite('crop_bird.png', cv2.resize(im[150:400,200:450],None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"
