#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -v parameter | head -40; echo ---; ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[140:260,180:400]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('robot0_eye_in_hand.png'); cv2.imwrite('eih_crop.png', cv2.resize(im[250:420,180:640],None,fx=2,fy=2))
"

# openrua op 6
ros2 topic echo /birdview/color/camera_info --once | head -30; ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|child" | head -60

# openrua op 7
cat > /workspace/tfdump.py <<'EOF'
import rclpy, yaml
from tf2_ros import Buffer, TransformListener
rclpy.init(); node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
import time
for _ in range(20): rclpy.spin_once(node, timeout_sec=0.2)
print(buf.all_frames_as_yaml())
for f in ["agentview_optical_frame","birdview_optical_frame","frontview_optical_frame","sideview_optical_frame","robot0_eye_in_hand_optical_frame","robot0_robotview_optical_frame","panda_link0","panda_hand"]:
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f, [round(v,4) for v in (tr.x,tr.y,tr.z)], [round(v,4) for v in (q.x,q.y,q.z,q.w)])
    except Exception as e:
        print(f, "ERR", e)
EOF
python3 /workspace/tfdump.py

# openrua op 8
timeout 20 ros2 topic echo /tf 2>&1 | grep -E "frame_id|child_frame_id" | sort | uniq -c | head; echo ---; timeout 10 ros2 topic echo /tf_static --qos-durability transient_local 2>&1 | grep -E "frame_id|child_frame_id" | sort | uniq -c

# openrua op 9
timeout 20 ros2 topic echo /tf 2>&1 | grep -E "child_frame_id" | sort | uniq -c

# openrua op 10
cat > /workspace/tfdump.py <<'EOF'
import rclpy, sys
from tf2_ros import Buffer, TransformListener
rclpy.init(); node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
frames = sys.argv[1:] or ["agentview_optical_frame","birdview_optical_frame","frontview_optical_frame","sideview_optical_frame","robot0_eye_in_hand_optical_frame","robot0_robotview_optical_frame","panda_link0","panda_hand","panda_leftfinger","panda_rightfinger"]
import time
t0=time.time()
while time.time()-t0 < 15 and not all(buf.can_transform("world", f, rclpy.time.Time()) for f in frames):
    rclpy.spin_once(node, timeout_sec=0.2)
for f in frames:
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f, [round(v,4) for v in (tr.x,tr.y,tr.z)], [round(v,4) for v in (q.x,q.y,q.z,q.w)])
    except Exception as e:
        print(f, "ERR", e)
EOF
python3 /workspace/tfdump.py

# openrua op 11
cat > /workspace/cloud.py <<'EOF'
"""Project a camera's depth frame to world points. Usage: cloud.py <cam> -> saves <cam>_cloud.npy (H,W,3) world xyz + <cam>_rgb from png"""
import sys, numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(node, topic, T):
    got={}
    sub=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub); return got["m"]

def main():
    cam=sys.argv[1]
    rclpy.init(); node=rclpy.create_node("cloud"); buf=Buffer(); TransformListener(buf,node)
    depth=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/depth/image_raw",Image),"passthrough").astype(np.float64)
    color=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/color/image_raw",Image),"bgr8")
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    frame=f"{cam}_optical_frame"
    t0=time.time()
    while not buf.can_transform("world",frame,rclpy.time.Time()) and time.time()-t0<15: rclpy.spin_once(node,timeout_sec=0.2)
    t=buf.lookup_transform("world",frame,rclpy.time.Time()); q=t.transform.rotation; tr=t.transform.translation
    R=quat_R(q.x,q.y,q.z,q.w); p=np.array([tr.x,tr.y,tr.z])
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    H,W=depth.shape; v,u=np.mgrid[0:H,0:W]
    pc=np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth],-1)
    pw=pc@R.T+p
    np.save(f"{cam}_cloud.npy",pw); np.save(f"{cam}_rgb.npy",color)
    print(cam, "cam pos",p, "depth range",np.nanmin(depth),np.nanmax(depth))
    rclpy.shutdown()
main()
EOF
for c in birdview agentview frontview sideview robot0_eye_in_hand; do timeout 120 python3 /workspace/cloud.py $c; done

# openrua op 12
cat > /workspace/hmap.py <<'EOF'
import numpy as np, cv2, sys
cams=sys.argv[1].split(",")
res=0.005
x0,x1,y0,y1=-0.6,0.5,-0.5,0.5
nx,ny=int((x1-x0)/res),int((y1-y0)/res)
hm=np.full((ny,nx),np.nan)
for cam in cams:
    pw=np.load(f"{cam}_cloud.npy").reshape(-1,3)
    ok=np.isfinite(pw).all(1); pw=pw[ok]
    ix=((pw[:,0]-x0)/res).astype(int); iy=((pw[:,1]-y0)/res).astype(int)
    m=(ix>=0)&(ix<nx)&(iy>=0)&(iy<ny); ix,iy,z=ix[m],iy[m],pw[m,2]
    # max height per cell
    order=np.argsort(z); 
    cur=hm[iy[order],ix[order]]
    hm[iy[order],ix[order]]=np.where(np.isnan(cur),z[order],np.maximum(cur,z[order]))
np.save("hmap.npy",hm)
# table height estimate
tab=np.nanmedian(hm); print("table z median",tab)
vals=hm[np.isfinite(hm)]
print("percentiles",np.percentile(vals,[1,5,50,90,99,100]))
# visualize: height above table, color scaled 0..0.3m
rel=np.clip((hm-tab)/0.3,0,1); rel[np.isnan(hm)]=0
img=(rel*255).astype(np.uint8); img=cv2.applyColorMap(img,cv2.COLORMAP_JET)
img=cv2.resize(img,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
# grid lines every 0.1 m
for xv in np.arange(x0,x1+1e-6,0.1):
    px=int((xv-x0)/res*4); cv2.line(img,(px,0),(px,img.shape[0]),(255,255,255),1); cv2.putText(img,f"{xv:.1f}",(px+2,12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
for yv in np.arange(y0,y1+1e-6,0.1):
    py=int((yv-y0)/res*4); cv2.line(img,(0,py),(img.shape[1],py),(255,255,255),1); cv2.putText(img,f"{yv:.1f}",(2,py-2),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
cv2.imwrite("hmap.png",img); print("hmap.png", img.shape)
EOF
python3 /workspace/hmap.py birdview,agentview,frontview,sideview

# openrua op 13
ros2 node info /robot_bridge; ros2 param list /robot_bridge 2>/dev/null | head -30; ros2 topic echo /pipeline_state --once 2>/dev/null | head -20

# openrua op 14
sleep 20; cat /tmp/claude-1000/-workspace/40dc7992-66e2-4e73-a988-ea93bc433e8d/tasks/b821n012m.output

# openrua op 15
python3 -c "
import numpy as np
hm=np.load('hmap.npy'); res=0.005; x0,y0=-0.6,-0.5
tab=0.8823
def cell(x,y): return hm[int((y-y0)/res), int((x-x0)/res)]
print('x:   ', ' '.join(f'{x:6.2f}' for x in np.arange(-0.52,-0.33,0.01)))
for y in np.arange(-0.38,0.09,0.01):
    print(f'y={y:5.2f}', ' '.join(f'{(cell(x,y)-tab)*100:6.1f}' for x in np.arange(-0.52,-0.33,0.01)))
"


# openrua op 16
python3 -c "
import numpy as np
hm=np.load('hmap.npy'); res=0.005; x0,y0=-0.6,-0.5
tab=0.8823
def cell(x,y): return hm[int((y-y0)/res), int((x-x0)/res)]
xs=np.arange(-0.25,-0.08,0.01)
print('x:   ', ' '.join(f'{x:6.2f}' for x in xs))
for y in np.arange(-0.06,0.06,0.005):
    print(f'y={y:6.3f}', ' '.join(f'{(cell(x,y)-tab)*100:6.1f}' for x in xs))
"


# openrua op 17
python3 -c "
import numpy as np
hm=np.load('hmap.npy'); res=0.005; x0,y0=-0.6,-0.5; tab=0.8823
iy,ix=np.where((hm-tab>0.15)&(hm-tab<0.25))
xs=x0+ix*res+res/2; ys=y0+iy*res+res/2
m=(xs>-0.3)&(xs<0.0)&(ys>-0.1)&(ys<0.1)
xs,ys=xs[m],ys[m]
print('n',len(xs),'x',xs.min(),xs.max(),'y',ys.min(),ys.max(),'center',xs.mean(),ys.mean())
P=np.stack([xs,ys],1)-[xs.mean(),ys.mean()]
w,v=np.linalg.eigh(P.T@P); print('axes',v[:,1],'yaw deg',np.degrees(np.arctan2(v[1,1],v[0,1])))
proj=P@v; print('extent along major',proj[:,1].min(),proj[:,1].max(),'minor',proj[:,0].min(),proj[:,0].max())
print('top z', np.nanmean(hm[iy[m],ix[m]]))
"


# openrua op 18
ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; ros2 interface show control_msgs/action/GripperCommand

# openrua op 19
mkdir -p "$(dirname /workspace/lib.py)"
cat > /workspace/lib.py <<'OPENRUA_EOF'
"""Reusable robot helpers for this Panda workstation (built once per process)."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])  # panda_link0 in world (TF, identity rotation)
TCP_OFF = M["hand"]["tcp_offset_m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    """Rotation matrix -> (x, y, z, w)."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("agent_lib")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.fk_cli.wait_for_service(10); self.ik_cli.wait_for_service(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 20:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in ARM]

    def fingers(self):
        js = self.joints()
        return js["panda_finger_joint1"], js["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """Returns (pos_world, R_world) of link for arm config q (default: current)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = quat_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk(q)
        return pos + TCP_OFF * R[:, 2], R

    # ---------- planning ----------
    def ik(self, pos_world, R, seed=None, at_tcp=True, tries=1):
        """IK for hand pose in world; pos may be the TCP point if at_tcp."""
        pos = np.asarray(pos_world, float)
        if at_tcp:
            pos = pos - TCP_OFF * R[:, 2]
        pos_base = pos - BASE_IN_WORLD
        qx, qy, qz, qw = R_quat(R)
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, (qx, qy, qz, qw))
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=0, nanosec=200_000_000)
        for _ in range(tries):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    # ---------- acting ----------
    def move(self, q, seconds=3.0, waypoints=None):
        """Send one trajectory (optionally through waypoints [(q, t), ...]) and wait."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if waypoints:
            for wq, wt in waypoints:
                pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
                pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q))
        print(f"move: error_code={code} max joint err={err.max():.4f}")
        return code

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        print(f"gripper({width}): pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}")
        return r

    def servo(self, v, n=20, dt=0.05):
        """Stream n twist messages with linear velocity v (m/s, base frame)."""
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            self.twist_pub.publish(msg)
            self.spin(dt)

    def move_tcp(self, pos, R, seconds=3.0, seed=None):
        q = self.ik(pos, R, seed=seed)
        if q is None:
            print("IK failed for", pos)
            return None
        self.move(q, seconds)
        p, _ = self.tcp()
        print(f"tcp now {np.round(p, 4)} target {np.round(pos, 4)} err {np.linalg.norm(p - pos):.4f}")
        return q


# Standard top-down grasp orientation: hand z pointing down (-z world),
# hand x axis along world direction given by yaw (fingers close along hand y).
def down_R(yaw=0.0):
    c, s = np.cos(yaw), np.sin(yaw)
    # columns: x_hand, y_hand, z_hand in world
    x = np.array([c, s, 0.0])
    z = np.array([0.0, 0.0, -1.0])
    y = np.cross(z, x)
    return np.stack([x, y, z], 1)
OPENRUA_EOF

# openrua op 20
timeout 120 python3 -c "
from lib import *
r=Robot()
q=r.arm_q(); print('q',np.round(q,3))
p,R=r.fk(); print('hand',np.round(p,4)); print(np.round(R,3))
t,_=r.tcp(); print('tcp',np.round(t,4))
print('fingers',r.fingers())
# check where hand frame axes go: fingers close along hand y?
for link in ['panda_leftfinger','panda_rightfinger']:
    print(link, np.round(r.fk(link=link)[0],4))
"

# openrua op 21
timeout 120 python3 -c "
from lib import *
import lib
r=Robot()
req = GetPositionFK.Request(); req.fk_link_names=['panda_hand']
req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=r.arm_q()
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
print('fk frame:', fut.result().pose_stamped[0].header.frame_id, fut.result().pose_stamped[0].pose.position)
# IK round trip with base offset (as lib does) vs without
lib.BASE_IN_WORLD=np.zeros(3)
p,R=r.fk(); print('hand (no offset)',np.round(p,4))
q=r.ik(p,R,at_tcp=False); print('ik no-offset ->', None if q is None else np.round(q,3))
lib.BASE_IN_WORLD=np.array([-0.75,0,0.912])
q=r.ik(p,R,at_tcp=False); print('ik with-offset ->', None if q is None else np.round(q,3))
"

# openrua op 22
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/lib.py", "old_string": "BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])  # panda_link0 in world (TF, identity rotation)", "new_string": "# Verified: /compute_fk and /compute_ik on this machine already work in the\\n# `world` frame (FK header says world and matches TF world->panda_hand), so no\\n# base offset is needed.\\nBASE_IN_WORLD = np.zeros(3)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 23
timeout 300 python3 -c "
from lib import *
r=Robot()
R=down_R(0.0)
q=r.ik([-0.30,0.30,1.30],R,at_tcp=False)
print('ik',None if q is None else np.round(q,3))
if q is not None:
    r.move(q,3.0)
    p,_=r.fk(); print('hand',np.round(p,4))
" && timeout 120 python3 tools/perception/cam_snap.py birdview && timeout 120 python3 /workspace/cloud.py birdview

# openrua op 24
python3 /workspace/hmap.py birdview && python3 -c "
import cv2; im=cv2.imread('birdview.png'); cv2.imwrite('bird_crop.png', cv2.resize(im[140:260,180:400],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 25
python3 -c "
import numpy as np
hm=np.load('hmap.npy'); res=0.005; x0,y0=-0.6,-0.5
tab=0.8804
def cell(x,y): return hm[int(round((y-y0)/res)), int(round((x-x0)/res))]
xs=np.arange(-0.52,-0.31,0.005)
print('x:    ', ' '.join(f'{x:5.3f}'[-4:] for x in xs))
for y in np.arange(-0.38,0.10,0.01):
    print(f'y={y:5.2f}', ' '.join(f'{(cell(x,y)-tab)*100:4.0f}' for x in xs))
"


# openrua op 26
python3 -c "
import numpy as np
pw=np.load('birdview_cloud.npy').reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
tab=0.8804
# back compartment floor
m=(pw[:,0]>-0.45)&(pw[:,0]<-0.41)&(pw[:,1]>-0.25)&(pw[:,1]<-0.05)&(np.abs(pw[:,2]-tab-0.02)<0.008)
f=pw[m]; print('back floor y range',f[:,1].min(),f[:,1].max(),'x range',f[:,0].min(),f[:,0].max(),'z',f[:,2].mean())
m=(pw[:,0]>-0.47)&(pw[:,0]<-0.33)&(pw[:,1]>-0.20)&(pw[:,1]<-0.09)&(np.abs(pw[:,2]-tab-0.02)<0.008)
f=pw[m]; xs=np.sort(f[:,0]); print('floor x values (unique-ish):', np.round(np.unique(np.round(xs,3)),3))
# front compartment floor
m=(pw[:,0]>-0.395)&(pw[:,0]<-0.335)&(pw[:,1]>-0.25)&(pw[:,1]<-0.05)&(np.abs(pw[:,2]-tab-0.02)<0.008)
f=pw[m]; print('front floor y range',f[:,1].min(),f[:,1].max(),'x range',f[:,0].min(),f[:,0].max())
# book top
m=(pw[:,2]>1.05)&(pw[:,0]>-0.3)&(pw[:,0]<0.0)&(np.abs(pw[:,1])<0.1)
b=pw[m]; c=b[:,:2].mean(0); P=b[:,:2]-c; w,v=np.linalg.eigh(P.T@P); u=v[:,1]; pr=P@u; pm=P@v[:,0]
print('book top n',len(b),'center',c,'z mean',b[:,2].mean(),'axis',u,'yaw',np.degrees(np.arctan2(u[1],u[0])),'len',pr.max()-pr.min(),'thick',pm.max()-pm.min())
# wall tops at y=-0.21 and y=-0.085 in the x-range of back compartment
for lo,hi in [(-0.23,-0.20),(-0.09,-0.07)]:
    m=(pw[:,0]>-0.455)&(pw[:,0]<-0.41)&(pw[:,1]>lo)&(pw[:,1]<hi)&(pw[:,2]>tab+0.05)
    w_=pw[m]; print('wall y in',lo,hi,': y range',w_[:,1].min(),w_[:,1].max(),'z max',w_[:,2].max())
"


# openrua op 27
python3 -c "
import numpy as np
pw=np.load('birdview_cloud.npy').reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
tab=0.8804
m=(pw[:,0]>-0.45)&(pw[:,0]<-0.41)&(pw[:,1]>-0.25)&(pw[:,1]<-0.05)
s=pw[m]
for y in np.arange(-0.23,-0.06,0.0025):
    mm=(s[:,1]>=y)&(s[:,1]<y+0.0025)
    if mm.sum(): print(f'y {y:.4f} n {mm.sum():3d} zmax-tab {100*(s[mm,2].max()-tab):5.1f} zmin-tab {100*(s[mm,2].min()-tab):5.1f}')
print('--- x profile at y in [-0.18,-0.12]')
m=(pw[:,1]>-0.18)&(pw[:,1]<-0.12)&(pw[:,0]>-0.48)&(pw[:,0]<-0.32); s=pw[m]
for x in np.arange(-0.48,-0.32,0.0025):
    mm=(s[:,0]>=x)&(s[:,0]<x+0.0025)
    if mm.sum(): print(f'x {x:.4f} n {mm.sum():3d} zmax-tab {100*(s[mm,2].max()-tab):5.1f} zmin-tab {100*(s[mm,2].min()-tab):5.1f}')
"


# openrua op 28
python3 -c "
import numpy as np
pw=np.load('birdview_cloud.npy').reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
m=(pw[:,2]>1.05)&(pw[:,0]>-0.3)&(pw[:,0]<0.0)&(np.abs(pw[:,1])<0.1)
b=pw[m]; c=b[:,:2].mean(0); P=b[:,:2]-c; w,v=np.linalg.eigh(P.T@P); u=v[:,1]; pr=P@u; pm=P@v[:,0]
print('book top n',len(b),'center',c,'z mean',b[:,2].mean(),'z max',b[:,2].max(),'axis',u,'yaw',np.degrees(np.arctan2(u[1],u[0])),'len',pr.max()-pr.min(),'thick',pm.max()-pm.min())
# book side faces visible from the birdview (camera at -0.2,0,3): points with z between 0.9 and 1.05 near the book
m=(pw[:,2]>0.95)&(pw[:,2]<1.05)&(pw[:,0]>-0.3)&(pw[:,0]<0.0)&(np.abs(pw[:,1])<0.1)
s=pw[m]; print('side pts',len(s)); 
if len(s): print('side x range',s[:,0].min(),s[:,0].max(),'y range',s[:,1].min(),s[:,1].max())
"


# openrua op 29
python3 -c "
import numpy as np
pw=np.load('birdview_cloud.npy').reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
m=(pw[:,2]>0.95)&(pw[:,2]<1.3)&(pw[:,0]>-0.3)&(pw[:,0]<0.3)&(np.abs(pw[:,1])<0.3)
s=pw[m]; print(len(s)); 
if len(s): print('x',s[:,0].min(),s[:,0].max(),'y',s[:,1].min(),s[:,1].max(),'z',s[:,2].min(),s[:,2].max())
m=(pw[:,2]>0.885)&(pw[:,2]<0.95)&(pw[:,0]>-0.3)&(pw[:,0]<0.1)&(np.abs(pw[:,1])<0.1)
s=pw[m]; print(len(s));
if len(s): print('x',s[:,0].min(),s[:,0].max(),'y',s[:,1].min(),s[:,1].max(),'z',s[:,2].min(),s[:,2].max())
"; python3 -c "
import cv2; im=cv2.imread('birdview.png'); cv2.imwrite('bird_crop2.png', cv2.resize(im[200:330,240:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 30
python3 -c "
import numpy as np
pw=np.load('birdview_cloud.npy')
print(pw.shape)
for v in range(262,282,2):
    print(v, ' '.join(f'{pw[v,u,2]:.3f}' for u in range(300,326,2)))
"

# openrua op 31
for c in agentview frontview sideview; do timeout 120 python3 cloud.py $c; done; python3 -c "
import numpy as np
for cam in ['birdview','agentview','frontview','sideview']:
    pw=np.load(f'{cam}_cloud.npy').reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
    m=(pw[:,2]>0.95)&(pw[:,2]<1.3)&(pw[:,0]>-0.25)&(pw[:,0]<0.0)&(np.abs(pw[:,1])<0.08)
    b=pw[m]; print(cam,'book pts',len(b), end=' ')
    if len(b): print('x',b[:,0].min().round(3),b[:,0].max().round(3),'y',b[:,1].min().round(3),b[:,1].max().round(3),'z',b[:,2].min().round(3),b[:,2].max().round(3))
    else: print()
"

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py sideview; python3 -c "
import cv2; im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[150:400,100:480],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))"

# openrua op 33
python3 -c "
import numpy as np
allp=[]
for cam in ['birdview','agentview','frontview','sideview']:
    pw=np.load(f'{cam}_cloud.npy').reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
    m=(pw[:,2]>0.89)&(pw[:,2]<1.3)&(pw[:,0]>-0.20)&(pw[:,0]<0.0)&(pw[:,1]>-0.08)&(pw[:,1]<0.03)
    allp.append(pw[m])
b=np.concatenate(allp); print('n',len(b),'x',b[:,0].min(),b[:,0].max(),'y',b[:,1].min(),b[:,1].max(),'z',b[:,2].min(),b[:,2].max())
top=b[b[:,2]>1.012]; c=top[:,:2].mean(0); P=top[:,:2]-c; w,v=np.linalg.eigh(P.T@P); u=v[:,1]; pr=P@u; pm=P@v[:,0]
print('top n',len(top),'center',c,'z',top[:,2].mean(),'yaw',np.degrees(np.arctan2(u[1],u[0])),'len',pr.max()-pr.min(),'thick',pm.max()-pm.min())
# footprint from all points projected on axes
P=b[:,:2]-c; pr=P@u; pm=P@v[:,0]; print('all: along',pr.min(),pr.max(),'across',pm.min(),pm.max())
# percentiles to reject outliers
print('along pct',np.percentile(pr,[0.5,99.5]),'across pct',np.percentile(pm,[0.5,99.5]))
"


# openrua op 34
timeout 300 python3 -c "
from lib import *
r=Robot()
q0=r.arm_q()
yaw_b=np.radians(-8.7)
bc=np.array([-0.096,-0.021])
poses={'above_book':([bc[0],bc[1],1.20],down_R(yaw_b)),
       'grasp':([bc[0],bc[1],0.99],down_R(yaw_b)),
       'lift':([bc[0],bc[1],1.30],down_R(yaw_b)),
       'over_comp_p90':([-0.4315,-0.1465,1.30],down_R(np.pi/2)),
       'over_comp_m90':([-0.4315,-0.1465,1.30],down_R(-np.pi/2)),
       'insert_p90':([-0.4315,-0.1465,1.065],down_R(np.pi/2)),
       'insert_m90':([-0.4315,-0.1465,1.065],down_R(-np.pi/2))}
seed=q0
for k,(p,R) in poses.items():
    q=r.ik(p,R,seed=seed,tries=3)
    print(k, None if q is None else np.round(q,3))
    if q is not None:
        t,_=r.tcp(q); print('   fk tcp',np.round(t,4))
        if k in ('above_book','grasp','lift'): seed=q
"

# openrua op 35
mkdir -p "$(dirname /workspace/pick.py)"
cat > /workspace/pick.py <<'OPENRUA_EOF'
"""Stage A: pick up the book (top-down grasp across its thickness) and lift."""
from lib import *

r = Robot()
yaw_b = np.radians(-8.7)
bc = np.array([-0.096, -0.021])
Rg = down_R(yaw_b)

print("fingers before:", r.fingers())
r.gripper(GRIP["open_m"])

q = r.move_tcp([bc[0], bc[1], 1.20], Rg, seconds=4.0)
assert q is not None
# descend in two steps, seeded for branch continuity
q = r.move_tcp([bc[0], bc[1], 1.08], Rg, seconds=2.0, seed=q)
q = r.move_tcp([bc[0], bc[1], 0.99], Rg, seconds=2.0, seed=q)
assert q is not None

r.gripper(GRIP["closed_m"])
f = r.fingers()
print("fingers after close:", f, "gap ~", f[0] * 2 if f[0] > 0 else f[0] - f[1])

q = r.move_tcp([bc[0], bc[1], 1.30], Rg, seconds=3.0, seed=q)
print("fingers after lift:", r.fingers())
print("arm q:", np.round(r.arm_q(), 4))
OPENRUA_EOF

# openrua op 36
timeout 900 python3 -u pick.py 2>&1 | tee pick.log

# openrua op 37
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 38
timeout 300 python3 -c "
from lib import *
r=Robot()
q0=np.array(r.arm_q()); print('q0',np.round(q0,3))
R90=down_R(np.pi/2)
q1=r.ik([-0.4315,-0.1465,1.30],R90,seed=list(q0),tries=3); print('q1',np.round(q1,3))
for s in np.linspace(0,1,11):
    q=q0*(1-s)+np.array(q1)*s; t,R=r.tcp(list(q)); print(f's={s:.1f} tcp {np.round(t,3)} zaxis {np.round(R[:,2],2)}')
np.save('q1.npy',np.array(q1))
"

# openrua op 39
timeout 120 python3 -c "
from lib import *
r=Robot()
q=r.arm_q(); t,_=r.tcp(q); print('tcp now',np.round(t,4),'q',np.round(q,4))
print('fingers',r.fingers())
"

# openrua op 40
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/lib.py", "old_string": "    def move_tcp(self, pos, R, seconds=3.0, seed=None):\\n        q = self.ik(pos, R, seed=seed)\\n        if q is None:\\n            print(\\"IK failed for\\", pos)\\n            return None\\n        self.move(q, seconds)\\n        p, _ = self.tcp()\\n        print(f\\"tcp now {np.round(p, 4)} target {np.round(pos, 4)} err {np.linalg.norm(p - pos):.4f}\\")\\n        return q", "new_string": "    def move_tcp(self, pos, R, seconds=3.0, seed=None, tol=0.003, refine=2):\\n        \\"\\"\\"IK -> trajectory; then re-command with the measured offset folded in\\n        (the controller shows steady-state error under load) until within tol.\\"\\"\\"\\n        pos = np.asarray(pos, float)\\n        cmd = pos.copy()\\n        q = None\\n        for i in range(1 + refine):\\n            q = self.ik(cmd, R, seed=seed, tries=3)\\n            if q is None:\\n                print(\\"IK failed for\\", cmd)\\n                return None\\n            self.move(q, seconds if i == 0 else max(1.0, seconds / 2))\\n            p, _ = self.tcp()\\n            err = pos - p\\n            print(f\\"tcp now {np.round(p, 4)} target {np.round(pos, 4)} err {np.linalg.norm(err):.4f}\\")\\n            if np.linalg.norm(err) < tol:\\n                break\\n            cmd = cmd + err\\n            seed = q\\n        return q", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 41
mkdir -p "$(dirname /workspace/place1.py)"
cat > /workspace/place1.py <<'OPENRUA_EOF'
"""Stage B: carry the book over the back compartment (yaw 90 deg so its long
axis runs along world y) and lower it until its bottom is just above the
wall tops. Stops there for a visual check."""
from lib import *

r = Robot()
R90 = down_R(np.pi / 2)
cx, cy = -0.4315, -0.1465          # back compartment interior centre
q1 = list(np.load("q1.npy"))

print("fingers:", r.fingers())
r.move(q1, 4.0)
q = r.move_tcp([cx, cy, 1.30], R90, seconds=2.0, seed=q1)
assert q is not None
prev = np.array(q)
for z in (1.27, 1.23):
    qn = r.ik([cx, cy, z], R90, seed=list(prev), tries=3)
    assert qn is not None, f"IK failed at z={z}"
    d = np.abs(np.array(qn) - prev).max()
    print(f"z={z}: max joint delta {d:.3f}")
    assert d < 0.6, "joint jump too large; aborting before motion"
    q = r.move_tcp([cx, cy, z], R90, seconds=2.0, seed=list(prev))
    prev = np.array(q)
np.save("q_prev.npy", prev)
print("fingers:", r.fingers())
OPENRUA_EOF

# openrua op 42
timeout 1200 python3 -u place1.py 2>&1 | tee place1.log && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 43
for c in agentview frontview sideview; do timeout 120 python3 cloud.py $c; done; python3 -c "
import numpy as np
for cam in ['agentview','frontview','sideview']:
    pw=np.load(f'{cam}_cloud.npy').reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
    m=(pw[:,2]>1.07)&(pw[:,2]<1.19)&(pw[:,0]>-0.50)&(pw[:,0]<-0.33)&(pw[:,1]>-0.30)&(pw[:,1]<0.0)
    b=pw[m]; print(cam,'n',len(b), end=' ')
    if len(b): print('x',np.percentile(b[:,0],[1,50,99]).round(4),'y',np.percentile(b[:,1],[0.5,50,99.5]).round(4),'z',b[:,2].min().round(3),b[:,2].max().round(3))
    else: print()
"

# openrua op 44
timeout 60 python3 tools/perception/cam_snap.py sideview && python3 -c "
import cv2; im=cv2.imread('sideview.png'); cv2.imwrite('side_crop.png', cv2.resize(im[100:340,330:560],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[0:200,150:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 45
python3 -c "
import numpy as np
for cam in ['sideview','agentview','frontview']:
    pw=np.load(f'{cam}_cloud.npy').reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
    m=(pw[:,0]>-0.49)&(pw[:,0]<-0.37)&(pw[:,1]>-0.22)&(pw[:,1]<-0.09)&(pw[:,2]>0.95)&(pw[:,2]<1.35)
    b=pw[m]
    print(cam, len(b))
    for z in np.arange(0.96,1.34,0.02):
        mm=(b[:,2]>=z)&(b[:,2]<z+0.02)
        if mm.sum(): print(f'  z {z:.2f} n {mm.sum():4d} x [{b[mm,0].min():.3f},{b[mm,0].max():.3f}] y [{b[mm,1].min():.3f},{b[mm,1].max():.3f}]')
"

# openrua op 46
python3 -c "
import numpy as np
pts=[]
for cam in ['sideview','agentview','frontview']:
    pw=np.load(f'{cam}_cloud.npy').reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
    m=(pw[:,0]>-0.50)&(pw[:,0]<-0.35)&(pw[:,1]>-0.25)&(pw[:,1]<-0.05)&(pw[:,2]>1.10)&(pw[:,2]<1.25)
    pts.append(pw[m])
b=np.concatenate(pts)
res=0.005; x0,y0=-0.50,-0.25
for zlo in [1.12,1.16,1.20,1.23]:
    s=b[(b[:,2]>=zlo)&(b[:,2]<zlo+0.02)]
    g=np.zeros((int(0.20/res),int(0.15/res)),int)
    for x,y,z in s: g[int((y-y0)/res), int((x-x0)/res)]+=1
    print(f'z slice {zlo:.2f}-{zlo+0.02:.2f}  (rows y from -0.25, cols x from -0.50, 5mm)')
    for i,row in enumerate(g):
        print(f'{y0+i*res:6.3f} '+''.join('#' if v>=3 else ('.' if v>0 else ' ') for v in row))
" | head -100

# openrua op 47
timeout 120 python3 -c "
from lib import *
r=Robot()
q=r.arm_q(); t,R=r.tcp(q); print('tcp',np.round(t,4)); print('R hand->world\n',np.round(R,3))
print('hand x axis yaw deg', np.degrees(np.arctan2(R[1,0],R[0,0])))
print('q',np.round(q,3)); print('target q was',np.round(np.load('q_prev.npy'),3))
print('fingers',r.fingers())
"

# openrua op 48
timeout 300 python3 -c "
from lib import *
r=Robot()
q0=r.arm_q()
for yaw in [0,-8.7,45,90,135,180]:
    R=down_R(np.radians(yaw))
    q=r.ik([-0.3,-0.1,1.25],R,seed=q0,tries=2)
    if q is None: print(yaw,'IK none'); continue
    t,Ra=r.tcp(q)
    print(f'req yaw {yaw:6.1f} -> fk yaw {np.degrees(np.arctan2(Ra[1,0],Ra[0,0])):7.1f}  z-axis {np.round(Ra[:,2],3)}  q7 {q[6]:.3f}')
print(R_quat(down_R(np.pi/2)))
"

# openrua op 49
timeout 120 python3 -c "
from lib import *
r=Robot()
q=r.arm_q()
for link in ['panda_link8','panda_hand']:
    p,R=r.fk(q,link=link); print(link,np.round(p,4),'yaw',np.degrees(np.arctan2(R[1,0],R[0,0])))
"

# openrua op 50
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/lib.py", "old_string": "        pos_base = pos - BASE_IN_WORLD\\n        qx, qy, qz, qw = R_quat(R)", "new_string": "        pos_base = pos - BASE_IN_WORLD\\n        # /compute_ik solves for the group tip panda_link8; panda_hand shares its\\n        # origin but is yawed +45 deg about z (verified with /compute_fk), so\\n        # convert the desired HAND rotation into the link8 rotation.\\n        c, s = np.cos(-np.pi / 4), np.sin(-np.pi / 4)\\n        R8 = np.asarray(R) @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])\\n        qx, qy, qz, qw = R_quat(R8)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 51
timeout 300 python3 -c "
from lib import *
r=Robot()
q0=r.arm_q()
for yaw in [0,90,-90]:
    R=down_R(np.radians(yaw))
    q=r.ik([-0.3,-0.1,1.25],R,seed=q0,tries=2)
    if q is None: print(yaw,'IK none'); continue
    t,Ra=r.tcp(q)
    print(f'req yaw {yaw:6.1f} -> fk hand yaw {np.degrees(np.arctan2(Ra[1,0],Ra[0,0])):7.1f}  tcp {np.round(t,3)} q {np.round(q,2)}')
"

# openrua op 52
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/lib.py", "old_string": "        # origin but is yawed +45 deg about z (verified with /compute_fk), so\\n        # convert the desired HAND rotation into the link8 rotation.\\n        c, s = np.cos(-np.pi / 4), np.sin(-np.pi / 4)", "new_string": "        # origin but is rotated -45 deg about the local z (URDF; verified with\\n        # /compute_fk), so convert the desired HAND rotation into link8\'s.\\n        c, s = np.cos(np.pi / 4), np.sin(np.pi / 4)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 53
timeout 300 python3 -c "
from lib import *
r=Robot()
q0=r.arm_q()
for yaw in [0,90,-90,-8.7]:
    R=down_R(np.radians(yaw))
    q=r.ik([-0.3,-0.1,1.25],R,seed=q0,tries=2)
    if q is None: print(yaw,'IK none'); continue
    t,Ra=r.tcp(q)
    print(f'req yaw {yaw:6.1f} -> fk hand yaw {np.degrees(np.arctan2(Ra[1,0],Ra[0,0])):7.1f}  tcp {np.round(t,3)} q {np.round(q,2)}')
"

# openrua op 54
timeout 600 python3 -c "
from lib import *
r=Robot()
q0=r.arm_q()
R90=down_R(np.pi/2)
q=r.move_tcp([-0.4315,-0.1465,1.25],R90,seconds=3.0,seed=q0)
t,Ra=r.tcp(); print('hand yaw',np.degrees(np.arctan2(Ra[1,0],Ra[0,0])),'fingers',r.fingers())
np.save('q_prev.npy',np.array(r.arm_q()))
" && for c in agentview frontview sideview; do timeout 120 python3 cloud.py $c; done

# openrua op 55
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/lib.py", "old_string": "    def move(self, q, seconds=3.0, waypoints=None):\\n        \\"\\"\\"Send one trajectory (optionally through waypoints [(q, t), ...]) and wait.\\"\\"\\"\\n        goal = FollowJointTrajectory.Goal()", "new_string": "    def move(self, q, seconds=3.0, waypoints=None, resend=2, jtol=0.02):\\n        \\"\\"\\"Send one trajectory (optionally through waypoints [(q, t), ...]) and\\n        wait; re-send the final point while the joints lag it by > jtol rad.\\"\\"\\"\\n        code = self._move_once(q, seconds, waypoints)\\n        for _ in range(resend):\\n            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()\\n            if err <= jtol:\\n                break\\n            code = self._move_once(q, max(1.5, seconds / 2), None)\\n        return code\\n\\n    def _move_once(self, q, seconds, waypoints):\\n        goal = FollowJointTrajectory.Goal()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 56
timeout 600 python3 -c "
from lib import *
r=Robot()
q0=r.arm_q()
R90=down_R(np.pi/2)
q=r.move_tcp([-0.4315,-0.1465,1.25],R90,seconds=2.0,seed=q0)
t,Ra=r.tcp(); print('hand yaw',np.degrees(np.arctan2(Ra[1,0],Ra[0,0])),'fingers',r.fingers())
np.save('q_prev.npy',np.array(r.arm_q()))
" && for c in agentview frontview sideview; do timeout 120 python3 cloud.py $c; done

# openrua op 57
python3 -c "
import numpy as np
pts=[]
for cam in ['sideview','agentview','frontview']:
    pw=np.load(f'{cam}_cloud.npy').reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
    m=(pw[:,0]>-0.52)&(pw[:,0]<-0.33)&(pw[:,1]>-0.27)&(pw[:,1]<-0.03)&(pw[:,2]>1.10)&(pw[:,2]<1.40)
    s=pw[m]; pts.append(s)
    for z in np.arange(1.10,1.40,0.02):
        mm=(s[:,2]>=z)&(s[:,2]<z+0.02)
        if mm.sum(): print(f'{cam} z {z:.2f} n {mm.sum():4d} x [{s[mm,0].min():.3f},{s[mm,0].max():.3f}] y [{s[mm,1].min():.3f},{s[mm,1].max():.3f}]')
b=np.concatenate(pts)
res=0.005; x0,y0=-0.52,-0.27
s=b[(b[:,2]>=1.15)&(b[:,2]<1.24)]
g=np.zeros((int(0.24/res),int(0.19/res)),int)
for x,y,z in s: g[int((y-y0)/res), int((x-x0)/res)]+=1
print('slice 1.15-1.24 top-down (rows y, cols x from -0.52)')
print('       '+''.join(str(int(round((x0+i*res)*100))%10) for i in range(g.shape[1])))
for i,row in enumerate(g):
    print(f'{y0+i*res:6.3f} '+''.join('#' if v>=3 else ('.' if v>0 else ' ') for v in row))
"

# openrua op 58
mkdir -p "$(dirname /workspace/place2.py)"
cat > /workspace/place2.py <<'OPENRUA_EOF'
"""Stage C: lower the (now y-aligned) book into the back compartment until the
fingertips are just above the wall tops, release, retreat."""
from lib import *

r = Robot()
R90 = down_R(np.pi / 2)
cx, cy = -0.4315, -0.1465
prev = np.array(r.arm_q())
t, Ra = r.tcp(list(prev))
print("start tcp", np.round(t, 4), "yaw", np.degrees(np.arctan2(Ra[1, 0], Ra[0, 0])), "fingers", r.fingers())

for z in (1.18, 1.12, 1.07):
    qn = r.ik([cx, cy, z], R90, seed=list(prev), tries=3)
    assert qn is not None, f"IK failed at z={z}"
    d = np.abs(np.array(qn) - prev).max()
    print(f"z={z}: max joint delta {d:.3f}")
    assert d < 0.6, "joint jump too large; aborting before motion"
    q = r.move_tcp([cx, cy, z], R90, seconds=2.0, seed=list(prev))
    prev = np.array(q)
    t, Ra = r.tcp()
    print("   yaw", np.degrees(np.arctan2(Ra[1, 0], Ra[0, 0])), "fingers", r.fingers())

r.gripper(GRIP["open_m"])
print("fingers after open:", r.fingers())
q = r.move_tcp([cx, cy, 1.30], R90, seconds=3.0, seed=list(prev))
print("retreated; fingers", r.fingers())
OPENRUA_EOF

# openrua op 59
timeout 1500 python3 -u place2.py 2>&1 | tee place2.log && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 60
for c in agentview frontview birdview; do timeout 120 python3 cloud.py $c; done; python3 -c "
import numpy as np
for cam in ['agentview','frontview','birdview']:
    pw=np.load(f'{cam}_cloud.npy').reshape(-1,3); pw=pw[np.isfinite(pw).all(1)]
    # anything above the compartment floor+3cm inside the back-compartment footprint (x within slot incl. walls)
    m=(pw[:,0]>-0.458)&(pw[:,0]<-0.405)&(pw[:,1]>-0.209)&(pw[:,1]<-0.084)&(pw[:,2]>0.93)&(pw[:,2]<1.10)
    b=pw[m]; print(cam,'n',len(b), end=' ')
    if len(b): print('x',np.percentile(b[:,0],[1,50,99]).round(4),'y',np.percentile(b[:,1],[1,50,99]).round(4),'z',np.percentile(b[:,2],[1,50,99]).round(4))
    else: print()
    # sanity: nothing book-like left on the table where the book was
    m=(pw[:,0]>-0.25)&(pw[:,0]<0.0)&(pw[:,1]>-0.1)&(pw[:,1]<0.05)&(pw[:,2]>0.90)
    print('   points left at old book spot:',m.sum())
"

# openrua op 61
timeout 60 python3 tools/perception/cam_snap.py birdview; python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(im[140:290,100:380],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('birdview.png'); cv2.imwrite('bird_crop.png', cv2.resize(im[140:230,190:330],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 62
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Verified quirks of the /workspace Panda sim (IK tip link yaw offset, world-frame FK/IK, controller lag, unsettled objects at start) that the shipped docs get wrong or omit
metadata:
  type: project
---

Facts verified on 2026-09-12 on the Franka Panda ROS 2 workstation in /workspace:

- `/compute_ik` solves for **panda_link8**, not panda_hand. panda_hand shares the
  origin but is rotated -45° about the local z, so a requested "hand" yaw comes out
  +45° off. Convert: R_link8 = R_hand @ Rz(+45°) before sending (done in
  /workspace/lib.py `Robot.ik`).
- `/compute_fk` returns poses in the `world` frame (header says world; matches TF
  world->panda_hand). No base offset needed for FK or IK despite docs saying
  "planner frame is the arm base". world->panda_link0 = (-0.75, 0, 0.912).
- FollowJointTrajectory often returns error_code -5 with the joints lagging the goal
  (up to ~0.1–1 rad); re-sending the same final point converges. Always verify with
  FK of the actual joints and re-command with the offset folded in.
- Objects may start floating a few cm above the table and settle once the sim
  clock advances (first motion). Re-measure object heights after the first move.
- LIBERO study scene: caddy at x∈[-0.47,-0.33], y∈[-0.36,0.07]; naming is from the
  agentview camera (+x, facing the robot): left=-y big, right=+y tall box,
  front=+x small, back=-x small (x∈[-0.458,-0.405], y∈[-0.209,-0.084]). Table z=0.880.

**Why:** these cost several turns to discover and the docs point the other way.
**How to apply:** reuse /workspace/lib.py if it still exists; otherwise apply the
link8 yaw conversion and world-frame assumption before the first IK move.
OPENRUA_EOF

# openrua op 63
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda workstation quirks](panda-workstation-quirks.md) — IK targets panda_link8 (+45° yaw), FK/IK in world frame, resend lagging trajectories, objects settle after first motion, caddy compartment layout" >> MEMORY.md && cat MEMORY.md
