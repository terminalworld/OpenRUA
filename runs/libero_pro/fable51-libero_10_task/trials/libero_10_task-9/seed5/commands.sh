#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---ACTIONS; timeout 20 ros2 action list; echo ---SERVICES; timeout 20 ros2 service list | grep -v -E 'parameter|describe|get_type' | head -40; echo ---NODES; timeout 20 ros2 node list

# openrua op 4
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E 'frame_id|child_frame_id' | sort | uniq -c | head -60; echo ---; timeout 20 ros2 topic echo /tf --once 2>&1 | grep -E 'frame_id' | head -40

# openrua op 6
timeout 30 ros2 topic echo /tf --once 2>&1 | head -60

# openrua op 7
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png && timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A12 '^k:' ; timeout 20 ros2 topic echo /agentview/color/camera_info --once | grep -A12 '^k:'; timeout 20 ros2 topic echo /tf --once | grep -B2 -A14 agentview_optical

# openrua op 8
mkdir -p "$(dirname /workspace/geo.py)"
cat > /workspace/geo.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Geometry helpers: camera TFs, pixel->world, point clouds from saved depth.

Usage:
  python3 geo.py tf                      # dump world->camera TFs to cams.json
  python3 geo.py px <cam> <u> <v> [...]  # world xyz for pixels (uses <cam>_depth.npy)
  python3 geo.py cloud <cam>             # print table-height stats + object blobs
"""
import json
import sys

import numpy as np

K = dict(fx=579.4112549695428, fy=579.4112549695428, cx=320.0, cy=240.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def fetch_tf():
    import rclpy
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    node = rclpy.create_node("geo_tf")
    got = {}
    node.create_subscription(TFMessage, "/tf", lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.5)
    out = {}
    for t in got["m"].transforms:
        tr, q = t.transform.translation, t.transform.rotation
        out[t.child_frame_id] = dict(t=[tr.x, tr.y, tr.z], q=[q.x, q.y, q.z, q.w])
    rclpy.shutdown()
    json.dump(out, open("cams.json", "w"), indent=1)
    print(json.dumps(out, indent=1))


def cam_T(cam):
    c = json.load(open("cams.json"))[f"{cam}_optical_frame"]
    T = np.eye(4)
    T[:3, :3] = quat_R(*c["q"])
    T[:3, 3] = c["t"]
    return T


def px_to_world(cam, depth, u, v):
    z = depth[v, u]
    p = np.array([(u - K["cx"]) * z / K["fx"], (v - K["cy"]) * z / K["fy"], z, 1.0])
    return (cam_T(cam) @ p)[:3]


def cloud(cam, depth):
    h, w = depth.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    z = depth
    pts = np.stack([(uu - K["cx"]) * z / K["fx"], (vv - K["cy"]) * z / K["fy"], z,
                    np.ones_like(z)], -1).reshape(-1, 4)
    return (cam_T(cam) @ pts.T).T[:, :3].reshape(h, w, 3)


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "tf":
        fetch_tf()
    elif cmd == "px":
        cam = sys.argv[2]
        depth = np.load(f"{cam}_depth.npy")
        args = list(map(int, sys.argv[3:]))
        for u, v in zip(args[::2], args[1::2]):
            print(u, v, np.round(px_to_world(cam, depth, u, v), 4))
    elif cmd == "cloud":
        cam = sys.argv[2]
        depth = np.load(f"{cam}_depth.npy")
        P = cloud(cam, depth)
        np.save(f"{cam}_cloud.npy", P)
        zs = P[..., 2]
        print("z percentiles", np.percentile(zs[np.isfinite(zs)], [1, 5, 25, 50, 75, 95, 99]))
OPENRUA_EOF

# openrua op 9
timeout 60 python3 geo.py tf > /dev/null && timeout 60 python3 geo.py cloud birdview && python3 geo.py px birdview 237 270 320 300 430 300 395 300 350 245 300 245 395 245 200 400 470 250 465 355

# openrua op 10
cat cams.json | head -30

# openrua op 11
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/geo.py", "old_string": "    got = {}\\n    node.create_subscription(TFMessage, \\"/tf\\", lambda m: got.setdefault(\\"m\\", m), 1)\\n    while \\"m\\" not in got:\\n        rclpy.spin_once(node, timeout_sec=0.5)\\n    out = {}\\n    for t in got[\\"m\\"].transforms:\\n        tr, q = t.transform.translation, t.transform.rotation\\n        out[t.child_frame_id] = dict(t=[tr.x, tr.y, tr.z], q=[q.x, q.y, q.z, q.w])", "new_string": "    out = {}\\n\\n    def cb(m):\\n        for t in m.transforms:\\n            tr, q = t.transform.translation, t.transform.rotation\\n            out[f\\"{t.header.frame_id}->{t.child_frame_id}\\"] = dict(\\n                t=[tr.x, tr.y, tr.z], q=[q.x, q.y, q.z, q.w])\\n            if t.header.frame_id == \\"world\\":\\n                out[t.child_frame_id] = out[f\\"{t.header.frame_id}->{t.child_frame_id}\\"]\\n\\n    node.create_subscription(TFMessage, \\"/tf\\", cb, 50)\\n    for _ in range(40):\\n        rclpy.spin_once(node, timeout_sec=0.5)\\n        if \\"birdview_optical_frame\\" in out and \\"panda_link0\\" in out:\\n            break", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 12
timeout 60 python3 geo.py tf | grep -c optical && timeout 60 python3 geo.py cloud birdview && python3 geo.py px birdview 237 270 320 300 430 300 395 300 350 245 300 245 395 245 200 400 470 250 465 355

# openrua op 13
python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy'); z=P[...,2]
# microwave top
m=(z>1.09)&(z<1.12)
ys,xs=np.where(m); print('microwave top: n',m.sum(),'x',P[m][:,0].min(),P[m][:,0].max(),'y',P[m][:,1].min(),P[m][:,1].max(),'px u',xs.min(),xs.max(),'v',ys.min(),ys.max())
# anything between table and microwave top, excluding arm region (v<250 & u 250-380 is arm)
m2=(z>0.93)&(z<1.08)
ys,xs=np.where(m2)
print('mid heights count',m2.sum())
# histogram of z in mid range
h,e=np.histogram(z[m2],bins=15); print(list(zip(np.round(e[:-1],3),h)))
# mugs: z 0.95-1.0
for name,(u0,u1,v0,v1) in {'white':(200,280,240,300),'yellow':(290,350,270,330)}.items():
    sub=P[v0:v1,u0:u1]; zz=sub[...,2]; mm=(zz>0.93)&(zz<1.02)
    pts=sub[mm]; print(name,'n',mm.sum(),'center',pts[:,:2].mean(0),'x range',pts[:,0].min(),pts[:,0].max(),'y range',pts[:,1].min(),pts[:,1].max(),'zmax',zz[mm].max())
# door: thin line near v=245, u 300-395
sub=P[235:255,280:400]; zz=sub[...,2]; mm=(zz>0.93)&(zz<1.2)
pts=sub[mm]; print('door region pts',mm.sum()); 
if mm.sum(): print('x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z',pts[:,2].min(),pts[:,2].max())
"


# openrua op 14
python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy')
print('row v=300:'); 
for u in range(380,481,5): print(u, np.round(P[300,u],3))
print('col u=430:')
for v in range(230,371,5): print(v, np.round(P[v,430],3))
print('door row v=245..250 along u:')
for u in range(290,400,10): print(u, np.round(P[245,u],3), np.round(P[250,u],3))
"


# openrua op 15
for c in galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[140:420,280:640]; cv2.imwrite('crop_agent_mw.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('frontview.png'); crop=im[250:450,300:640]; cv2.imwrite('crop_front_mw.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"; python3 -c "
import json; c=json.load(open('cams.json'))
for k in ['sideview_optical_frame','galleryview_optical_frame','paperview_optical_frame','robot0_robotview_optical_frame']: print(k, c[k])"

# openrua op 16
timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw frontview_depth.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw sideview_depth.png >/dev/null; python3 geo.py cloud frontview >/dev/null; python3 geo.py cloud sideview > /dev/null; python3 -c "
import numpy as np
P=np.load('frontview_cloud.npy')
# white mug region in frontview: u 160-220, v 280-345
sub=P[270:350,150:230]; 
x,y,z=sub[...,0],sub[...,1],sub[...,2]
m=(x>-0.2)&(x<0.0)&(y<-0.15)&(y>-0.4)&(z>0.9)
pts=sub[m]
print('n',m.sum(),'z range',pts[:,2].min(),pts[:,2].max(), 'x range', pts[:,0].min(), pts[:,0].max())
for z0 in np.arange(0.905,1.03,0.01):
    s=pts[(pts[:,2]>z0)&(pts[:,2]<z0+0.01)]
    if len(s): print(round(z0,3),'n',len(s),'y',round(s[:,1].min(),3),round(s[:,1].max(),3),'width',round(s[:,1].max()-s[:,1].min(),3),'xmin',round(s[:,0].min(),3))
"


# openrua op 17
python3 -c "
import numpy as np
for cam in ['frontview','sideview','agentview']:
    try: P=np.load(cam+'_cloud.npy')
    except Exception as e: print(cam,e); continue
    z=P[...,2]; f=np.isfinite(z)&(z>0.5)&(z<1.2)
    h,e=np.histogram(z[f],bins=30); i=h.argmax(); print(cam,'dominant z',e[i],e[i+1],'count',h[i])
    print('  sample pts', np.round(P[400,100],3), np.round(P[450,320],3), np.round(P[300,100],3))
"; python3 geo.py cloud agentview

# openrua op 18
python3 -c "
import numpy as np
P=np.load('frontview_cloud.npy'); D=np.load('frontview_depth.npy')
print(D.shape)
for (u,v) in [(190,300),(190,320),(190,340),(185,290),(190,285),(190,350),(150,320),(230,320),(330,330),(320,300)]:
    print(u,v,'d',round(float(D[v,u]),3),np.round(P[v,u],3))
"


# openrua op 19
python3 -c "
import numpy as np
B=np.load('birdview_cloud.npy'); D=np.load('birdview_depth.npy')
print('birdview mug column v=270, u 220..260')
for u in range(220,262,4): print(u,'d',round(float(D[270,u]),3),np.round(B[270,u],3))
S=np.load('sideview_cloud.npy'); Ds=np.load('sideview_depth.npy')
print('sideview: white mug approx (325,225)')
for (u,v) in [(325,215),(325,225),(325,235),(325,245),(310,230),(340,230),(325,205)]:
    print(u,v,'d',round(float(Ds[v,u]),3),np.round(S[v,u],3))
"


# openrua op 20
python3 -c "
import numpy as np
A=np.load('agentview_cloud.npy')
x,y,z=A[...,0],A[...,1],A[...,2]
m=(y>0.285)&(y<0.47)&(x>-0.2)&(x<0.12)&np.isfinite(z)
pts=A[m]; print('cavity-ish pts',m.sum())
h,e=np.histogram(pts[:,2],bins=np.arange(0.85,1.15,0.01)); 
for a,b in zip(e[:-1],h): print(round(a,2),b)
print('x hist inside cavity band z 0.95-1.05:')
s=pts[(pts[:,2]>0.95)&(pts[:,2]<1.05)]
h,e=np.histogram(s[:,0],bins=np.arange(-0.2,0.13,0.01)); 
for a,b in zip(e[:-1],h): print(round(a,2),b)
print('y hist:'); h,e=np.histogram(s[:,1],bins=np.arange(0.28,0.48,0.01))
for a,b in zip(e[:-1],h): print(round(a,2),b)
B=np.load('birdview_cloud.npy'); 
sub=B[240:300,200:280]; zz=sub[...,2]; print('white mug rim zmax', np.nanmax(zz[zz<1.2]))
sub=B[270:330,290:350]; zz=sub[...,2]; print('yellow mug rim zmax', np.nanmax(zz[zz<1.2]))
"


# openrua op 21
python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[260:360,140:240]; cv2.imwrite('crop_front_mug.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('sideview.png'); crop=im[190:290,240:400]; cv2.imwrite('crop_side_mug.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 22
python3 -c "
import json; c=json.load(open('cams.json'))
for k,v in c.items():
    if 'panda' in k or 'eye' in k: print(k, [round(a,4) for a in v['t']], [round(a,4) for a in v['q']])
"

# openrua op 23
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Robot helper library + CLI (Panda via MoveIt IK/FK + FJT + gripper).

CLI:
  python3 rob.py js                 # joint state dict
  python3 rob.py fk                 # hand pose in world (xyz, quat, rpy) + tcp
  python3 rob.py ik x y z qx qy qz qw   # print IK solution (world frame pose of hand)
  python3 rob.py go x y z qx qy qz qw [secs] [--tcp]   # IK + move
  python3 rob.py joints j1,..,j7 [secs]
  python3 rob.py grip open|close
  python3 rob.py wrench
"""
import math
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE_T = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (identity rotation)
TCP_OFF = 0.1034
FJT = "/panda_arm_controller/follow_joint_trajectory"
GRIP = "/franka_gripper/gripper_action"
LIMITS = [(-2.9, 2.9), (-1.76, 1.76), (-2.9, 2.9), (-3.07, -0.07), (-2.9, 2.9),
          (-0.02, 3.75), (-2.9, 2.9)]


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()  # x y z w


def rpy_quat(r, p, y):
    from scipy.spatial.transform import Rotation
    return Rotation.from_euler("xyz", [r, p, y]).as_quat()


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT)
        self.grip = ActionClient(self.node, GripperCommand, GRIP)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr.pop("m", None)
        for _ in range(50):
            self.spin(0.2)
            if "m" in self._wr:
                break
        if "m" not in self._wr:
            return None
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z,
                         w.torque.x, w.torque.y, w.torque.z])

    def fk(self, q=None, link="panda_hand"):
        """Hand pose in WORLD frame: (pos[3], quat[4])."""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_T
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        return pos + TCP_OFF * quat_R(quat)[:, 2], quat

    # ---------- IK ----------
    def ik(self, pos_w, quat, seed=None, tcp=False, timeout=1.0, attempts=1):
        """IK for hand at world pose. If tcp, pos_w is the fingertip point."""
        pos_w = np.asarray(pos_w, float)
        quat = np.asarray(quat, float)
        if tcp:
            pos_w = pos_w - TCP_OFF * quat_R(quat)[:, 2]
        pos_b = pos_w - BASE_T
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(
            map(float, seed if seed is not None else self.arm_q()))
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout),
                                          nanosec=int((timeout % 1) * 1e9))
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in ARM]

    # ---------- motion ----------
    def move_joints(self, q, secs=3.0, via=None):
        """Send trajectory to q (list of 7) over secs; via = list of
        (q, t) intermediate points. Returns error_code."""
        for i, (v, (lo, hi)) in enumerate(zip(q, LIMITS)):
            if not (lo <= v <= hi):
                raise ValueError(f"joint{i+1}={v:.3f} outside [{lo},{hi}]")
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        for vq, vt in (via or []):
            pt = JointTrajectoryPoint(positions=list(map(float, vq)))
            pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=list(map(float, q)))
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        if not gh.accepted:
            raise RuntimeError("FJT goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"[move] code={code} max_joint_err={err:.4f}", flush=True)
        return code

    def move_pose(self, pos_w, quat, secs=3.0, tcp=False, seed=None):
        q = self.ik(pos_w, quat, seed=seed, tcp=tcp)
        if q is None:
            print(f"[move_pose] IK FAILED for {np.round(pos_w,3)}", flush=True)
            return None
        return self.move_joints(q, secs)

    def gripper(self, width):
        """width = per-finger position (0.04 open, 0.0 closed)."""
        self.grip.wait_for_server(timeout_sec=10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        send = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"[grip] reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}", flush=True)
        return f


def main():
    a = sys.argv[1:]
    r = Robot()
    cmd = a[0]
    if cmd == "js":
        print(r.joints())
    elif cmd == "fk":
        pos, q = r.fk()
        from scipy.spatial.transform import Rotation
        print("hand pos", np.round(pos, 4), "quat", np.round(q, 4),
              "rpy", np.round(Rotation.from_quat(q).as_euler("xyz"), 3))
        print("tcp", np.round(r.tcp()[0], 4))
        print("hand R (cols = hand x,y,z in world):\n", np.round(quat_R(q), 3))
    elif cmd == "ik":
        v = list(map(float, a[1:8]))
        print(r.ik(v[:3], v[3:], tcp="--tcp" in a))
    elif cmd == "go":
        v = list(map(float, [x for x in a[1:] if not x.startswith("--")]))
        secs = v[7] if len(v) > 7 else 3.0
        r.move_pose(v[:3], v[3:7], secs, tcp="--tcp" in a)
        print("tcp now", np.round(r.tcp()[0], 4))
    elif cmd == "joints":
        q = list(map(float, a[1].split(",")))
        secs = float(a[2]) if len(a) > 2 else 3.0
        r.move_joints(q, secs)
    elif cmd == "grip":
        r.gripper(0.04 if a[1] == "open" else 0.0)
    elif cmd == "wrench":
        print(r.wrench())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 24
timeout 120 python3 rob.py fk && timeout 60 python3 rob.py wrench

# openrua op 25
timeout 120 python3 -c "
from rob import *
r=Robot()
r.__class__.fk_raw = lambda self,q=None: None
import numpy as np
q0=r.arm_q(); print('q0',np.round(q0,3))
# raw FK without BASE_T
pos,quat=r.fk(); raw=pos-BASE_T; print('raw fk', np.round(raw,4), np.round(quat,4))
# test IK in raw (as-returned) coordinates: pass pos_w such that pos_b = raw -> pos_w = raw + BASE_T = pos
s1=r.ik(pos, quat); print('IK with pos_b=raw:', None if s1 is None else np.round(s1,3))
# test IK where pos_b = raw - BASE_T (i.e. treat raw as world)
s2=r.ik(raw, quat); print('IK with pos_b=raw-BASE:', None if s2 is None else np.round(s2,3))
"


# openrua op 26
sed -i 's|^BASE_T = np.array(\[-0.66, 0.0, 0.912\]).*|BASE_T = np.array([0.0, 0.0, 0.0])  # MoveIt FK/IK already report in WORLD (verified: fk == camera TF)|' rob.py && grep -n BASE_T rob.py | head -3

# openrua op 27
python3 -c "
import numpy as np, json
from rob import quat_R
c=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame']
Rc=quat_R(c['q']); tc=np.array(c['t'])
Rh=quat_R([0.9996,0,-0.0284,0]); th=np.array([-0.203,0,1.2696])
Rrel=Rh.T@Rc; trel=Rh.T@(tc-th)
print('cam in hand: t',np.round(trel,4)); print(np.round(Rrel,3))
print('cam optical z (view dir) in world', np.round(Rc[:,2],3), 'cam up(-y) in world', np.round(-Rc[:,1],3))
"


# openrua op 28
timeout 300 python3 -c "
from rob import *
r=Robot()
R=np.array([[0,1,0],[0,0,1],[1,0,0]],float)  # cols: x=(0,0,1), y=(1,0,0), z=(0,1,0)
q=R_quat(R); print('quat',np.round(q,4))
s=r.ik([-0.05,-0.12,1.15], q); print('via ik', None if s is None else np.round(s,3))
s2=r.ik([-0.03,0.0,0.98], q, seed=s); print('look ik', None if s2 is None else np.round(s2,3))
"


# openrua op 29
timeout 300 python3 -c "
from rob import *
r=Robot()
R=np.array([[0,1,0],[0,0,1],[1,0,0]],float)
q=R_quat(R)
seeds=[[0,0.3,0,-2.0,0,2.3,0.78],[0,0.3,0,-2.0,-1.57,1.57,0.0],[0.3,0.5,-0.3,-1.8,1.57,1.57,0.78],[0,-0.16,0,-2.44,0,2.22,0.785],[0.5,0.4,-0.5,-1.9,-1.5,1.6,1.5]]
for sd in seeds:
    s=r.ik([-0.03,0.0,0.98], q, seed=sd, timeout=0.5); print(np.round(sd,2),'->', None if s is None else np.round(s,3))
"


# openrua op 30
timeout 600 python3 -u -c "
from rob import *
r=Robot()
R=np.array([[0,1,0],[0,0,1],[1,0,0]],float); q=R_quat(R)
sd=[0.3,0.5,-0.3,-1.8,1.57,1.57,0.78]
s1=r.ik([-0.05,-0.12,1.15], q, seed=sd); print('via', np.round(s1,3))
s2=r.ik([-0.03,0.0,0.98], q, seed=s1); print('look', np.round(s2,3))
r.move_joints(s1, 4.0)
print('hand', np.round(r.fk()[0],3))
r.move_joints(s2, 3.0)
print('hand', np.round(r.fk()[0],3), 'tcp', np.round(r.tcp()[0],3))
" 2>&1 | tail -8 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand look1.png

# openrua op 31
timeout 60 python3 rob.py js

# openrua op 32
timeout 600 python3 -u -c "
from rob import *
r=Robot()
s2=[0.03,1.072,-0.273,-1.52,1.612,1.314,-2.583]
r.move_joints(s2, 5.0)
print(np.round(r.arm_q(),3))
print('hand', np.round(r.fk()[0],3), 'tcp', np.round(r.tcp()[0],3))
" 2>&1 | tail -4

# openrua op 33
timeout 600 python3 -u -c "
from rob import *
r=Robot()
s2=[0.03,1.072,-0.273,-1.52,1.612,1.314,-2.583]
for secs in (4.0, 4.0):
    r.move_joints(s2, secs); print(np.round(r.arm_q(),3))
print('hand', np.round(r.fk()[0],3), 'tcp', np.round(r.tcp()[0],3))
" 2>&1 | tail -6

# openrua op 34
timeout 600 python3 -u -c "
from rob import *
r=Robot()
s2=[0.03,1.072,-0.273,-1.52,1.612,1.314,-2.583]
r.move_joints(s2, 3.0); print(np.round(r.arm_q(),3))
print('hand', np.round(r.fk()[0],3), 'tcp', np.round(r.tcp()[0],3))
" 2>&1 | tail -3; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand look1.png

# openrua op 35
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null; timeout 60 python3 geo.py tf >/dev/null; cp eih_depth.npy robot0_eye_in_hand_depth.npy; timeout 60 python3 geo.py cloud robot0_eye_in_hand; python3 -c "
import numpy as np, json
c=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame']; print('cam', np.round(c['t'],3))
P=np.load('robot0_eye_in_hand_cloud.npy'); x,y,z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(z)&(y>0.27)&(y<0.6)&(x>-0.3)&(x<0.3)
pts=P[m]; print('n',m.sum())
print('z hist'); h,e=np.histogram(pts[:,2],bins=np.arange(0.88,1.14,0.01)); [print(round(a,2),b) for a,b in zip(e[:-1],h) if b>0]
print('y hist'); h,e=np.histogram(pts[:,1],bins=np.arange(0.27,0.6,0.01)); [print(round(a,2),b) for a,b in zip(e[:-1],h) if b>0]
print('x hist'); h,e=np.histogram(pts[:,0],bins=np.arange(-0.3,0.3,0.01)); [print(round(a,2),b) for a,b in zip(e[:-1],h) if b>0]
"


# openrua op 36
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null; python3 geo.py cloud birdview >/dev/null; python3 -c "
import numpy as np
B=np.load('birdview_cloud.npy')
for name,(u0,u1,v0,v1) in {'white':(200,280,240,300),'yellow':(290,350,270,330)}.items():
    sub=B[v0:v1,u0:u1]; zz=sub[...,2]; m=(zz>0.92)&(zz<1.2); pts=sub[m]
    print(name,'zmax',round(float(zz[m].max()),4),'center xy',np.round(pts[:,:2].mean(0),3),'x',round(pts[:,0].min(),3),round(pts[:,0].max(),3),'y',round(pts[:,1].min(),3),round(pts[:,1].max(),3))
    h,e=np.histogram(pts[:,2],bins=np.arange(0.92,1.1,0.01)); print('  ',[(round(a,2),b) for a,b in zip(e[:-1],h) if b>0])
"


# openrua op 37
python3 -c "
import numpy as np
B=np.load('birdview_cloud.npy')
sub=B[250:292,222:262]; zz=sub[...,2]; m=(zz>0.92)&(zz<1.2); pts=sub[m]
print('white zmax',round(float(zz[m].max()),4))
h,e=np.histogram(pts[:,2],bins=np.arange(0.92,1.12,0.01)); print([(round(a,2),b) for a,b in zip(e[:-1],h) if b>0])
top=pts[pts[:,2]>1.0]; print('top pts x',round(top[:,0].min(),3),round(top[:,0].max(),3),'y',round(top[:,1].min(),3),round(top[:,1].max(),3))
for u in range(222,262,3): print(u, np.round(B[270,u],3))
"


# openrua op 38
python3 -c "
import numpy as np
B=np.load('birdview_cloud.npy')
for (u,v) in [(430,300),(450,330),(410,260),(370,247),(350,247),(390,320)]: print(u,v,np.round(B[v,u],3))
P=np.load('robot0_eye_in_hand_cloud.npy'); x,y,z=P[...,0],P[...,1],P[...,2]
# floor of cavity: points with y in 0.3-0.42 and z<0.97
m=np.isfinite(z)&(y>0.30)&(y<0.42)&(x>-0.11)&(x<-0.01)
pts=P[m]; print('interior pts',m.sum()); 
h,e=np.histogram(pts[:,2],bins=np.arange(0.9,1.12,0.005)); print([(round(a,3),b) for a,b in zip(e[:-1],h) if b>0])
# side walls: points with z in 0.97-1.03, y 0.3-0.42
m=np.isfinite(z)&(y>0.30)&(y<0.42)&(z>0.97)&(z<1.03)
pts=P[m]; h,e=np.histogram(pts[:,0],bins=np.arange(-0.25,0.1,0.005)); print('x walls',[(round(a,3),b) for a,b in zip(e[:-1],h) if b>0])
# back wall
m=np.isfinite(z)&(x>-0.11)&(x<-0.01)&(z>0.97)&(z<1.03)
pts=P[m]; h,e=np.histogram(pts[:,1],bins=np.arange(0.2,0.6,0.005)); print('y walls',[(round(a,3),b) for a,b in zip(e[:-1],h) if b>0])
"


# openrua op 39
timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -E -A9 '^k:|height|width' | head -20

# openrua op 40
python3 - <<'EOF'
import re
s=open('geo.py').read()
s=s.replace('K = dict(fx=579.4112549695428, fy=579.4112549695428, cx=320.0, cy=240.0)',
'''K_DEFAULT = dict(fx=579.4112549695428, fy=579.4112549695428, cx=320.0, cy=240.0)
K_BY_CAM = {"robot0_eye_in_hand": dict(fx=312.77408948188935, fy=312.77408948188935, cx=320.0, cy=240.0)}


def K_for(cam):
    return K_BY_CAM.get(cam, K_DEFAULT)''')
s=s.replace('''def px_to_world(cam, depth, u, v):
    z = depth[v, u]''','''def px_to_world(cam, depth, u, v):
    K = K_for(cam)
    z = depth[v, u]''')
s=s.replace('''def cloud(cam, depth):
    h, w = depth.shape''','''def cloud(cam, depth):
    K = K_for(cam)
    h, w = depth.shape''')
open('geo.py','w').write(s)
EOF
python3 geo.py cloud robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); x,y,z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(z)&(y>0.30)&(y<0.42)&(x>-0.15)&(x<0.05)
pts=P[m]; h,e=np.histogram(pts[:,2],bins=np.arange(0.9,1.12,0.005)); print('z floor/ceil',[(round(a,3),b) for a,b in zip(e[:-1],h) if b>500])
m=np.isfinite(z)&(y>0.30)&(y<0.42)&(z>0.97)&(z<1.03)
pts=P[m]; h,e=np.histogram(pts[:,0],bins=np.arange(-0.3,0.15,0.005)); print('x walls',[(round(a,3),b) for a,b in zip(e[:-1],h) if b>500])
m=np.isfinite(z)&(x>-0.15)&(x<0.05)&(z>0.97)&(z<1.03)
pts=P[m]; h,e=np.histogram(pts[:,1],bins=np.arange(0.2,0.6,0.005)); print('y walls',[(round(a,3),b) for a,b in zip(e[:-1],h) if b>500])
"


# openrua op 41
python3 -c "
import numpy as np
B=np.load('birdview_cloud.npy')
sub=B[245:295,215:275]; x,y,z=sub[...,0],sub[...,1],sub[...,2]
m=(z>0.905)&(z<1.1)&(x>-0.16)&(x<-0.04)
pts=sub[m]
# rows by y bins
for y0 in np.arange(-0.34,-0.18,0.01):
    s=pts[(pts[:,1]>=y0)&(pts[:,1]<y0+0.01)]
    if len(s): print(round(y0,3),'n',len(s),'x',round(s[:,0].min(),3),round(s[:,0].max(),3),'zmax',round(s[:,2].max(),3),'zmin',round(s[:,2].min(),3))
"


# openrua op 42
cat >> rob.py <<'EOF'


def links_fk(r, q, links=("panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7", "panda_hand")):
    """World positions of several links for a joint vector q."""
    r.fk_cli.wait_for_service(timeout_sec=10)
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = list(links)
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = list(map(float, q))
    fut = r.fk_cli.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    out = {}
    for n, ps in zip(res.fk_link_names, res.pose_stamped):
        p = ps.pose.position
        out[n] = np.array([p.x, p.y, p.z])
    return out


def hand_quat(z_dir, x_dir):
    """Quaternion for hand with approach axis z_dir and hand-x along x_dir (world)."""
    z = np.asarray(z_dir, float); z /= np.linalg.norm(z)
    x = np.asarray(x_dir, float); x = x - z * (x @ z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return R_quat(np.stack([x, y, z], 1))
EOF
timeout 300 python3 -c "
from rob import *
r=Robot()
# grasp pose: approach -y, fingers close along x => hand y = x_world (or -x); hand x = +z world
q=hand_quat([0,-1,0],[0,0,1]); print('quat',np.round(q,4), 'R cols', np.round(quat_R(q),2).tolist())
tcp=np.array([-0.105,-0.215,0.955])
seeds=[[0.3,0.5,-0.3,-1.8,1.57,1.57,0.78],[-0.3,0.5,0.3,-1.8,-1.57,1.57,0.78],[0,0.3,0,-2.0,0,2.3,0.78],[0.5,0.3,-0.5,-2.0,-1.57,1.6,-0.7],[-0.5,0.3,0.5,-2.0,1.57,1.6,2.3]]
for sd in seeds:
    s=r.ik(tcp,q,seed=sd,tcp=True,timeout=0.5)
    if s is None: print(np.round(sd,2),'-> None'); continue
    L=links_fk(r,s); print(np.round(s,3)); [print('   ',k,np.round(v,3)) for k,v in L.items()]
"


# openrua op 43
timeout 300 python3 -c "
from rob import *
r=Robot()
q=hand_quat([0,-1,0],[0,0,-1]); print('quat',np.round(q,4))
tcp=np.array([-0.105,-0.215,0.955])
seeds=[[0.3,0.5,-0.3,-1.8,1.57,1.57,-0.7],[-0.3,0.5,0.3,-1.8,-1.57,1.57,-0.7],[0,0.3,0,-2.0,0,2.3,-0.7],[0.5,0.3,-0.5,-2.0,-1.57,1.6,-0.7],[-0.5,0.3,0.5,-2.0,1.57,1.6,-0.7],[0,0.3,0,-2.0,0,2.3,2.3]]
for sd in seeds:
    s=r.ik(tcp,q,seed=sd,tcp=True,timeout=0.5)
    if s is None: print(np.round(sd,2),'-> None'); continue
    L=links_fk(r,s); print(np.round(s,3)); [print('   ',k,np.round(v,3)) for k,v in L.items() if k in ('panda_link4','panda_link5','panda_link7')]
"


# openrua op 44
timeout 600 python3 -c "
from rob import *
r=Robot()
rng=np.random.default_rng(0)
q=hand_quat([0,-1,0],[0,0,1])
tcp=np.array([-0.105,-0.215,0.955])
sols=[]
for i in range(40):
    sd=[rng.uniform(lo,hi) for lo,hi in LIMITS]
    s=r.ik(tcp,q,seed=sd,tcp=True,timeout=0.3)
    if s is None: continue
    L=links_fk(r,s,('panda_link4','panda_link5','panda_link7'))
    if not any(np.allclose(s,x[0],atol=0.05) for x in sols):
        sols.append((s,L))
for s,L in sols:
    print(np.round(s,3),'link5',np.round(L['panda_link5'],3),'link4',np.round(L['panda_link4'],3))
"


# openrua op 45
python3 -c "
import numpy as np
B=np.load('birdview_cloud.npy'); x,y,z=B[...,0],B[...,1],B[...,2]
# yellow mug lying: search around image (290-340, 310-360)
sub=B[305:365,285:345]; zz=sub[...,2]; m=(zz>0.905)&(zz<1.05); pts=sub[m]
print('n',m.sum(),'x',round(pts[:,0].min(),3),round(pts[:,0].max(),3),'y',round(pts[:,1].min(),3),round(pts[:,1].max(),3),'zmax',round(pts[:,2].max(),3),'center',np.round(pts.mean(0),3))
"


# openrua op 46
timeout 600 python3 -u -c "
from rob import *
r=Robot()
q=r.arm_q(); print(np.round(q,3))
t=list(q); t[6]=q[6]+1.0
r.move_joints(t, 6.0); print(np.round(r.arm_q(),3))
" 2>&1 | tail -3

# openrua op 47
timeout 600 python3 -u -c "
from rob import *
r=Robot()
q=r.arm_q()
t=list(q); t[6]=q[6]-1.0
r.move_joints(t, 3.0); print(np.round(r.arm_q(),3))
t2=list(q); t2[6]=q[6]-0.5
r.move_joints(t2, 3.0); print(np.round(r.arm_q(),3))
" 2>&1 | tail -4

# openrua op 48
python3 - <<'EOF'
s=open('rob.py').read()
s=s.replace('''        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()''','''        cur = np.array(self.arm_q())
        dq = np.abs(np.array(q) - cur)
        need = max(dq[6] / 0.15, dq.max() / 1.0) + 0.5  # j7 is capped ~0.18 rad/s
        if secs < need and not via:
            print(f"[move] stretching duration {secs:.1f}->{need:.1f}s", flush=True)
            secs = need
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()''')
open('rob.py','w').write(s)
EOF
timeout 600 python3 -c "
from rob import *
r=Robot()
a=np.deg2rad(30); d=np.array([-np.sin(a),-np.cos(a),0])
B=np.array([-0.104,-0.205,0.955])
pre=B-0.06*d; grasp=B+0.01*d; lift=grasp+[0,0,0.10]
print('pre',np.round(pre,3),'grasp',np.round(grasp,3))
qh=hand_quat(d,[0,0,1])
seed=r.arm_q()
for name,p in [('pre',pre),('grasp',grasp),('lift',lift)]:
    s=r.ik(p,qh,seed=seed,tcp=True,timeout=0.5)
    if s is None: print(name,'IK None'); continue
    L=links_fk(r,s,('panda_link4','panda_link5','panda_link7')); print(name,np.round(s,3),'link5',np.round(L['panda_link5'],3),'link7',np.round(L['panda_link7'],3)); seed=s
"


# openrua op 49
cat >> rob.py <<'EOF'


NOMINAL = np.array([0.0, 0.4, 0.0, -1.9, 0.0, 2.3, 0.8])


def best_ik(r, pos, quat, tcp=True, n=30, seed=None, prefer=None, avoid=None, rng=None):
    """Try IK from several seeds; return the solution closest to `prefer`
    (joint vector) / NOMINAL, with joint-limit margin, optionally
    rejecting solutions where a link enters an `avoid(links)->bool` test."""
    rng = rng or np.random.default_rng(1)
    seeds = [seed] if seed is not None else []
    seeds += [r.arm_q(), list(NOMINAL)]
    seeds += [[rng.uniform(lo, hi) for lo, hi in LIMITS] for _ in range(n)]
    best, best_c = None, 1e9
    ref = np.array(prefer if prefer is not None else NOMINAL)
    for sd in seeds:
        s = r.ik(pos, quat, seed=sd, tcp=tcp, timeout=0.3)
        if s is None:
            continue
        if any(v < lo + 0.08 or v > hi - 0.08 for v, (lo, hi) in zip(s, LIMITS)):
            continue
        if avoid is not None:
            L = links_fk(r, s)
            if avoid(L):
                continue
        c = np.linalg.norm((np.array(s) - ref) * np.array([1, 1, 1, 1, 0.5, 1, 0.3]))
        if c < best_c:
            best, best_c = s, c
    return best
EOF
timeout 900 python3 -c "
from rob import *
r=Robot()
a=np.deg2rad(30); d=np.array([-np.sin(a),-np.cos(a),0])
B=np.array([-0.104,-0.205,0.955])
pre=B-0.06*d; grasp=B+0.01*d; lift=grasp+[0,0,0.10]
qh=hand_quat(d,[0,0,1])
def door_hit(L):
    # door: x in [-0.2,-0.14], y in [0.0,0.29], z<1.12 ; keep link5/6 centre >6cm from that box
    for k in ('panda_link5','panda_link6','panda_link7','panda_hand'):
        p=L[k]; dx=max(-0.2-p[0],0,p[0]+0.14); dy=max(0.0-p[1],0,p[1]-0.29)
        if p[2]<1.15 and np.hypot(dx,dy)<0.06: return True
    return False
prev=None
for name,p in [('pre',pre),('grasp',grasp),('lift',lift)]:
    s=best_ik(r,p,qh,seed=prev,prefer=prev,avoid=door_hit)
    if s is None: print(name,'None'); continue
    L=links_fk(r,s,('panda_link4','panda_link5','panda_link7')); print(name,np.round(s,3),'link5',np.round(L['panda_link5'],3)); prev=s
"


# openrua op 50
timeout 900 python3 -c "
from rob import *
r=Robot()
a=np.deg2rad(30); d=np.array([-np.sin(a),-np.cos(a),0])
B=np.array([-0.104,-0.205,0.955])
pre=B-0.06*d; grasp=B+0.01*d; lift=grasp+[0,0,0.10]
qh=hand_quat(d,[0,0,-1])
def door_hit(L):
    for k in ('panda_link5','panda_link6','panda_link7','panda_hand'):
        p=L[k]; dx=max(-0.2-p[0],0,p[0]+0.14); dy=max(0.0-p[1],0,p[1]-0.29)
        if p[2]<1.15 and np.hypot(dx,dy)<0.06: return True
    return False
cur=r.arm_q(); print('cur',np.round(cur,3))
# retract: same orientation as now, hand up 0.2
pos,q=r.fk(); up=r.ik(pos+[0,0,0.2],q,seed=cur); print('up',np.round(up,3))
prev=up
for name,p in [('pre_hi',pre+[0,0,0.12]),('pre',pre),('grasp',grasp),('lift',lift)]:
    s=best_ik(r,p,qh,seed=prev,prefer=prev,avoid=door_hit)
    if s is None: print(name,'None'); continue
    L=links_fk(r,s,('panda_link4','panda_link5','panda_link7')); print(name,np.round(s,3),'link5',np.round(L['panda_link5'],3)); prev=s
"


# openrua op 51
timeout 900 python3 -u -c "
from rob import *
r=Robot()
up=[0.089,0.637,-0.339,-1.747,1.577,1.293,-2.671]
pre_hi=[0.3,1.018,-0.303,-1.359,-0.925,1.274,-0.774]
pre=[0.256,1.314,-0.275,-1.12,-0.868,1.29,-0.712]
for name,q in [('up',up),('pre_hi',pre_hi),('pre',pre)]:
    c=r.move_joints(q,3.0)
    if c!=0: c=r.move_joints(q,3.0)
    print(name,'tcp',np.round(r.tcp()[0],3))
print('fingers',r.fingers())
" 2>&1 | grep -v '^\[move\] stretch' ; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview > /dev/null

# openrua op 52
timeout 900 python3 -c "
from rob import *
r=Robot()
a=np.deg2rad(30); b=np.deg2rad(25)
d=np.array([-np.sin(a)*np.cos(b),-np.cos(a)*np.cos(b),-np.sin(b)])
B=np.array([-0.104,-0.205,0.958])
pre=B-0.06*d; grasp=B+0.01*d; lift=grasp+[0,0,0.10]
print('pre',np.round(pre,3),'grasp',np.round(grasp,3))
qh=hand_quat(d,[0,0,-1]); print('quat',np.round(qh,4))
def door_hit(L):
    for k in ('panda_link5','panda_link6','panda_link7','panda_hand'):
        p=L[k]; dx=max(-0.2-p[0],0,p[0]+0.14); dy=max(0.0-p[1],0,p[1]-0.29)
        if p[2]<1.15 and np.hypot(dx,dy)<0.06: return True
    return False
prev=r.arm_q(); print('cur',np.round(prev,3))
for name,p in [('pre',pre),('grasp',grasp),('lift',lift)]:
    s=best_ik(r,p,qh,seed=prev,prefer=prev,avoid=door_hit)
    if s is None: print(name,'None'); continue
    L=links_fk(r,s,('panda_link4','panda_link5','panda_link6','panda_link7')); print(name,np.round(s,3),'link5',np.round(L['panda_link5'],3),'link7',np.round(L['panda_link7'],3)); prev=s
"


# openrua op 53
timeout 900 python3 -u -c "
from rob import *
r=Robot()
pre=[0.135,0.932,-0.162,-1.497,-0.787,1.644,-0.658]
grasp=[0.07,0.923,-0.192,-1.615,-0.843,1.774,-0.619]
for name,q in [('pre',pre),('grasp',grasp)]:
    c=r.move_joints(q,3.0)
    if c!=0: c=r.move_joints(q,3.0)
    print(name,'tcp',np.round(r.tcp()[0],3))
print('fingers',r.fingers())
" 2>&1 | grep -v stretch; timeout 60 python3 tools/perception/cam_snap.py agentview > /dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 54
timeout 300 python3 -u -c "
from rob import *
r=Robot()
r.gripper(0.0)
print('tcp',np.round(r.tcp()[0],3))
" 2>&1 | tail -2; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview > /dev/null

# openrua op 55
timeout 600 python3 -u -c "
from rob import *
r=Robot()
lift=[0.102,0.712,-0.218,-1.71,-0.849,1.699,-0.709]
c=r.move_joints(lift,3.0)
print('tcp',np.round(r.tcp()[0],3),'fingers',np.round(r.fingers(),4))
" 2>&1 | grep -v stretch; timeout 60 python3 tools/perception/cam_snap.py agentview > /dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview > /dev/null

# openrua op 56
cat >> rob.py <<'EOF'


def tilt_dir(yaw_deg, tilt_deg):
    """Approach direction pointing at horizontal angle yaw (deg, from +x)
    and tilt_deg below horizontal."""
    a, b = np.deg2rad(yaw_deg), np.deg2rad(tilt_deg)
    return np.array([np.cos(a) * np.cos(b), np.sin(a) * np.cos(b), -np.sin(b)])


def clear_of_door(L, margin=0.06):
    """False if wrist/hand links come near the open door / microwave box or table."""
    boxes = [(-0.20, -0.14, 0.0, 0.30, 0.0, 1.12),   # open door
             (-0.18, 0.17, 0.27, 0.48, 0.0, 1.12)]   # microwave body
    for k in ('panda_link5', 'panda_link6', 'panda_link7', 'panda_hand'):
        p = L[k]
        if p[2] < 0.9 + margin:
            return False
        for (x0, x1, y0, y1, z0, z1) in boxes:
            dx = max(x0 - p[0], 0, p[0] - x1); dy = max(y0 - p[1], 0, p[1] - y1)
            dz = max(z0 - p[2], 0, p[2] - z1)
            if np.sqrt(dx * dx + dy * dy + dz * dz) < margin:
                return False
    return True
EOF
timeout 900 python3 -c "
from rob import *
r=Robot()
prev=r.arm_q(); print('cur',np.round(prev,3),'tcp',np.round(r.tcp()[0],3))
tcp0=np.array([-0.11,-0.28,1.06])
plan=[]
for yaw in [-120,-90,-60,-30,0,30,60,90]:
    d=tilt_dir(yaw,25); qh=hand_quat(d,[0,0,-1])
    s=best_ik(r,tcp0,qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print(yaw,'None'); continue
    L=links_fk(r,s,('panda_link5','panda_link7')); print(yaw,np.round(s,3),'l7',np.round(L['panda_link7'],3),'dj7',round(s[6]-prev[6],2)); plan.append((yaw,s)); prev=s
import json; json.dump([(y,list(map(float,s))) for y,s in plan],open('yaw_plan.json','w'))
"


# openrua op 57
timeout 900 python3 -c "
from rob import *
r=Robot()
import json
plan=json.load(open('yaw_plan.json'))
prev=np.array(plan[2][1])  # yaw -60 solution
tcp0=np.array([-0.11,-0.28,1.06])
rng=np.random.default_rng(1)
for yaw in [-50,-40,-30,-20,-10,0]:
  for tilt in [25,15,35]:
    d=tilt_dir(yaw,tilt)
    for xdir in ([0,0,-1],[0,0,1]):
        qh=hand_quat(d,xdir)
        s=best_ik(r,tcp0,qh,seed=prev,prefer=prev,n=40,rng=rng,avoid=lambda L: not clear_of_door(L))
        if s is None: print(yaw,tilt,xdir,'None'); continue
        dq=np.abs(np.array(s)-prev).max()
        print(yaw,tilt,xdir,np.round(s,2),'maxdq',round(dq,2))
"


# openrua op 58
timeout 1200 python3 -c "
from rob import *
r=Robot()
import json
plan=json.load(open('yaw_plan.json'))
rng=np.random.default_rng(2)
def chain(tcp, yaws, start, tilt=25):
    prev=np.array(start); out=[]
    for yaw in yaws:
        d=tilt_dir(yaw,tilt); best=None
        for xdir in ([0,0,-1],[0,0,1]):
            s=best_ik(r,tcp,hand_quat(d,xdir),seed=prev,prefer=prev,n=40,rng=rng,avoid=lambda L: not clear_of_door(L))
            if s is None: continue
            dq=np.abs(np.array(s)-prev); dq[6]=min(dq[6], abs(dq[6]-2*np.pi))
            if best is None or dq.max()<best[0]: best=(dq.max(),s,xdir)
        if best is None: print(yaw,'None'); return out
        print(yaw,np.round(best[1],2),'maxdq',round(best[0],2),best[2]); out.append((yaw,best[1])); prev=np.array(best[1])
    return out
print('--- via -x at tcp0')
chain(np.array([-0.11,-0.28,1.06]), [-150,180,150,120,90], plan[0][1])
print('--- via +x at higher/closer tcp')
chain(np.array([-0.2,-0.28,1.15]), [-120,-90,-60,-30,0,30,60,90], plan[0][1])
"


# openrua op 59
timeout 1500 python3 -c "
from rob import *
r=Robot()
d=np.array([-0.453,-0.785,-0.423]); qh=hand_quat(d,[0,0,-1])
dh=np.array([-0.5,-0.866,0.0])
prev=r.arm_q()
def goto(p, secs=3.0):
    global prev
    s=best_ik(r,np.array(p),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('no ik',p); return None
    if np.abs(np.array(s)-prev).max()>0.8: print('big jump',np.round(np.array(s)-prev,2))
    c=r.move_joints(s,secs); prev=np.array(r.arm_q())
    print(' tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4)); return c
tcp0=np.array([-0.18,-0.34,1.06])
goto(tcp0,4)
goto([-0.18,-0.34,1.0015],3)
P=tcp0[:2]-0.06*dh[:2]
for th in [75,90,105,120,135,149.4]:
    t=np.deg2rad(th)
    p=np.array([P[0],P[1],0.9])+0.118*(np.cos(t)*dh+np.sin(t)*np.array([0,0,1.0]))
    print('theta',th,'target',np.round(p,4)); goto(p,3)
    print(' wrench',np.round(r.wrench(),2))
" 2>&1 | grep -v '^\[INFO\]'
python3 tools/perception/cam_snap.py frontview && python3 tools/perception/cam_snap.py agentview

# openrua op 60
python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png >/dev/null && python3 geo.py cloud agentview >/dev/null && timeout 120 python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy').reshape(-1,3)
m=np.isfinite(P).all(1)&(P[:,0]>-0.25)&(P[:,0]<0.0)&(P[:,1]>-0.42)&(P[:,1]<-0.12)&(P[:,2]>0.905)&(P[:,2]<1.02)
Q=P[m]; print(len(Q))
# exclude gripper: points near the hand? just print z-slices
for z0 in np.arange(0.905,1.02,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(round(z0,3),len(s),'x',np.round([s[:,0].min(),s[:,0].max()],3),'y',np.round([s[:,1].min(),s[:,1].max()],3))
"

# openrua op 61
timeout 120 python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy').reshape(-1,3)
m=np.isfinite(P).all(1)&(P[:,0]>-0.25)&(P[:,0]<-0.085)&(P[:,1]>-0.42)&(P[:,1]<-0.12)&(P[:,2]>0.905)&(P[:,2]<1.05)
Q=P[m]
for z0 in np.arange(0.905,1.05,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(round(z0,3),len(s),'x',np.round([s[:,0].min(),s[:,0].max(),s[:,0].mean()],3),'y',np.round([s[:,1].min(),s[:,1].max(),s[:,1].mean()],3))
"

# openrua op 62
cat > measure_mug.py <<'EOF'
import subprocess, numpy as np, sys
def measure(cam='agentview', box=(-0.25,-0.085,-0.42,-0.12), quiet=False):
    subprocess.run(['python3','tools/perception/cam_snap.py',f'/{cam}/depth/image_raw',f'{cam}_depth.png'],check=True,capture_output=True)
    subprocess.run(['python3','geo.py','cloud',cam],check=True,capture_output=True)
    P=np.load(f'{cam}_cloud.npy').reshape(-1,3)
    x0,x1,y0,y1=box
    m=np.isfinite(P).all(1)&(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.905)&(P[:,2]<1.06)
    Q=P[m]
    if not quiet:
        for z0 in np.arange(0.905,1.06,0.02):
            s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
            if len(s): print(round(z0,3),len(s),'x',np.round([s[:,0].min(),s[:,0].max(),s[:,0].mean()],3),'y',np.round([s[:,1].min(),s[:,1].max(),s[:,1].mean()],3))
    return Q
if __name__=='__main__':
    measure(*(sys.argv[1:2] or ['agentview']))
EOF
timeout 900 python3 -c "
from rob import *
from measure_mug import measure
r=Robot()
d=np.array([-0.453,-0.785,-0.423]); qh=hand_quat(d,[0,0,-1])
back=np.array([0.5,0.866,0.0])
prev=r.arm_q()
for k,(ds,dz) in enumerate([(0.02,-0.005),(0.02,-0.005)]):
    p=r.tcp()[0]+ds*back+np.array([0,0,dz])
    s=best_ik(r,p,qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    r.move_joints(s,2.5); prev=np.array(r.arm_q())
    print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench(),2))
    measure()
" 2>&1 | grep -v INFO

# openrua op 63
timeout 900 python3 -c "
from rob import *
from measure_mug import measure
r=Robot()
d=np.array([-0.453,-0.785,-0.423]); qh=hand_quat(d,[0,0,-1])
back=np.array([0.5,0.866,0.0])
prev=r.arm_q()
for k,(ds,dz) in enumerate([(0.004,-0.012),(0.004,-0.012)]):
    p=r.tcp()[0]+ds*back+np.array([0,0,dz])
    s=best_ik(r,p,qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    r.move_joints(s,2.5); prev=np.array(r.arm_q())
    print('tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench(),2))
    measure()
" 2>&1 | grep -v INFO

# openrua op 64
python3 tools/perception/cam_snap.py frontview >/dev/null && python3 tools/perception/cam_snap.py sideview >/dev/null && python3 -c "
import cv2
im=cv2.imread('frontview.png'); cv2.imwrite('crop_f.png', cv2.resize(im[250:420,120:330],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 65
timeout 120 python3 -c "
import numpy as np
P=np.load('agentview_cloud.npy').reshape(-1,3)
m=np.isfinite(P).all(1)&(P[:,0]>-0.17)&(P[:,0]<-0.10)&(P[:,1]>-0.32)&(P[:,1]<-0.10)&(P[:,2]>0.90)&(P[:,2]<1.06)
Q=P[m]
ys=np.arange(-0.32,-0.10,0.01); zs=np.arange(1.06,0.90,-0.01)
print('      y: '+' '.join(f'{y*100:4.0f}' for y in ys))
for z in zs:
    row=[]
    for y in ys:
        n=((Q[:,1]>=y)&(Q[:,1]<y+0.01)&(Q[:,2]>=z-0.01)&(Q[:,2]<z)).sum()
        row.append('   .' if n==0 else f'{min(n,999):4d}')
    print(f'z={z:.2f} '+' '.join(row))
"

# openrua op 66
timeout 900 python3 -c "
from rob import *
from measure_mug import measure
r=Robot()
d=np.array([-0.453,-0.785,-0.423]); qh=hand_quat(d,[0,0,-1])
prev=r.arm_q()
for dx in [-0.02,-0.015]:
    p=r.tcp()[0]+np.array([dx,0,0])
    s=best_ik(r,p,qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    r.move_joints(s,2.5); prev=np.array(r.arm_q())
    print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench(),2))
    Q=measure(quiet=True)
    for z0 in [0.905,0.925,0.945,1.005,1.025,1.045]:
        s_=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
        if len(s_): print(' ',z0,len(s_),'x',np.round([s_[:,0].min(),s_[:,0].max()],3),'y',np.round([s_[:,1].min(),s_[:,1].max()],3))
" 2>&1 | grep -v INFO

# openrua op 67
timeout 900 python3 -c "
from rob import *
r=Robot()
r.gripper(0.08)
d=np.array([-0.453,-0.785,-0.423]); qh=hand_quat(d,[0,0,-1])
prev=r.arm_q()
p=r.tcp()[0]+np.array([0.02,0.035,0.10])   # retreat back (toward hand) and up
s=best_ik(r,p,qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
r.move_joints(s,3); print('tcp',np.round(r.tcp()[0],4))
" 2>&1 | grep -v INFO
python3 tools/perception/cam_snap.py frontview >/dev/null; python3 tools/perception/cam_snap.py birdview >/dev/null; python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; echo ok

# openrua op 68
python3 geo.py cloud birdview >/dev/null && timeout 120 python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3)
m=np.isfinite(P).all(1)&(P[:,0]>-0.3)&(P[:,0]<0.0)&(P[:,1]>-0.4)&(P[:,1]<-0.12)&(P[:,2]>0.92)&(P[:,2]<1.03)
Q=P[m]; print(len(Q))
for z0 in np.arange(0.92,1.03,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(round(z0,3),len(s),'x',np.round([s[:,0].min(),s[:,0].max(),s[:,0].mean()],3),'y',np.round([s[:,1].min(),s[:,1].max(),s[:,1].mean()],3))
rim=Q[Q[:,2]>1.0]
# fit circle to rim xy
A=np.c_[2*rim[:,0],2*rim[:,1],np.ones(len(rim))]; b=(rim[:,:2]**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center',np.round(c[:2],4),'R',round(R,4),'rim z',np.round(rim[:,2].mean(),4))
# yellow mug extents
m2=np.isfinite(P).all(1)&(P[:,0]>0.0)&(P[:,0]<0.35)&(P[:,1]>-0.2)&(P[:,1]<0.2)&(P[:,2]>0.915)&(P[:,2]<1.1)
Y=P[m2]; print('yellow: x',np.round([Y[:,0].min(),Y[:,0].max()],3),'y',np.round([Y[:,1].min(),Y[:,1].max()],3),'z',np.round([Y[:,2].min(),Y[:,2].max()],3))
"

# openrua op 69
timeout 900 python3 -c "
from rob import *
r=Robot()
qh=hand_quat([0,0,-1],[1,0,0])
prev=r.arm_q()
s=best_ik(r,np.array([-0.25,-0.30,1.25]),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
print(np.round(s,3)); r.move_joints(s,4); print('tcp',np.round(r.tcp()[0],4))
" 2>&1 | grep -v INFO
python3 tools/perception/cam_snap.py birdview >/dev/null; python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; python3 geo.py cloud birdview >/dev/null
timeout 120 python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3)
m=np.isfinite(P).all(1)&(P[:,0]>-0.3)&(P[:,0]<0.0)&(P[:,1]>-0.4)&(P[:,1]<-0.12)&(P[:,2]>0.92)&(P[:,2]<1.05)
Q=P[m]
for z0 in np.arange(0.92,1.05,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(round(z0,3),len(s),'x',np.round([s[:,0].min(),s[:,0].max(),s[:,0].mean()],3),'y',np.round([s[:,1].min(),s[:,1].max(),s[:,1].mean()],3))
rim=Q[Q[:,2]>0.995]
A=np.c_[2*rim[:,0],2*rim[:,1],np.ones(len(rim))]; b=(rim[:,:2]**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center',np.round(c[:2],4),'R',round(R,4),'rim z',np.round(rim[:,2].max(),4))
m2=np.isfinite(P).all(1)&(P[:,0]>0.0)&(P[:,0]<0.4)&(P[:,1]>-0.25)&(P[:,1]<0.2)&(P[:,2]>0.915)&(P[:,2]<1.1)
Y=P[m2]; print('yellow: x',np.round([Y[:,0].min(),Y[:,0].max()],3),'y',np.round([Y[:,1].min(),Y[:,1].max()],3),'z',np.round([Y[:,2].min(),Y[:,2].max()],3))
"

# openrua op 70
timeout 900 python3 -c "
from rob import *
r=Robot()
s=[-0.207,-0.136,-0.41,-2.186,-0.061,2.06,-0.585]
r.move_joints(s,4); print('tcp',np.round(r.tcp()[0],4)); print('js',np.round(r.arm_q(),3))
" 2>&1 | grep -v INFO
sleep 1; python3 tools/perception/cam_snap.py birdview >/dev/null; python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; python3 geo.py cloud birdview >/dev/null
timeout 120 python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3)
m=np.isfinite(P).all(1)&(P[:,0]>-0.3)&(P[:,0]<0.0)&(P[:,1]>-0.4)&(P[:,1]<-0.12)&(P[:,2]>0.92)&(P[:,2]<1.05)
Q=P[m]
for z0 in np.arange(0.92,1.05,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(round(z0,3),len(s),'x',np.round([s[:,0].min(),s[:,0].max(),s[:,0].mean()],3),'y',np.round([s[:,1].min(),s[:,1].max(),s[:,1].mean()],3))
rim=Q[Q[:,2]>0.995]
A=np.c_[2*rim[:,0],2*rim[:,1],np.ones(len(rim))]; b=(rim[:,:2]**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center',np.round(c[:2],4),'R',round(R,4),'rim z',np.round(rim[:,2].max(),4))
"

# openrua op 71
ls -la --time-style=full-iso birdview_depth.npy birdview_cloud.npy birdview.png; date; python3 -c "
import numpy as np; d=np.load('birdview_depth.npy'); print(d.shape, d.dtype, np.nanmin(d), np.nanmax(d))"

# openrua op 72
timeout 120 python3 -c "
import numpy as np, cv2
P=np.load('birdview_cloud.npy'); im=cv2.imread('birdview.png')
# horizontal scan through row 268, vertical scan through col 252
for v in [262,268,274]:
    print('row',v, [(u, tuple(np.round(P[v,u],3))) for u in range(228,282,6)])
print()
for u in [246,252,258]:
    print('col',u, [(v, tuple(np.round(P[v,u],3))) for v in range(244,296,6)])
# whiteness mask around mug to find rim ellipse
g=im[230:310,215:300].astype(int); wm=(g.min(2)>170)
ys,xs=np.nonzero(wm); print('white px bbox u',xs.min()+215,xs.max()+215,'v',ys.min()+230,ys.max()+230)
"

# openrua op 73
python3 -c "
import cv2, json, numpy as np
im=cv2.imread('birdview.png'); cv2.imwrite('crop_b.png', cv2.resize(im[235:305,215:300],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
c=json.load(open('cams.json'))['birdview_optical_frame']; print('birdview cam pos',c['t'],'q',c['q'])
"

# openrua op 74
timeout 120 python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3)
m=np.isfinite(P).all(1)&(P[:,0]>-0.17)&(P[:,0]<-0.02)&(P[:,1]>-0.215)&(P[:,1]<-0.14)&(P[:,2]>0.915)&(P[:,2]<1.05)
H=P[m]; print('handle-region pts',len(H))
for y0 in np.arange(-0.215,-0.14,0.005):
    s=H[(H[:,1]>=y0)&(H[:,1]<y0+0.005)]
    if len(s): print(round(y0,3),len(s),'x',np.round([s[:,0].min(),s[:,0].max()],3),'z',np.round([s[:,2].min(),s[:,2].max(),s[:,2].mean()],3))
print('--- body along x (y in -0.30..-0.21)')
m=np.isfinite(P).all(1)&(P[:,0]>-0.2)&(P[:,0]<0.0)&(P[:,1]>-0.30)&(P[:,1]<-0.21)&(P[:,2]>0.915)
B=P[m]
for x0 in np.arange(-0.18,-0.01,0.01):
    s=B[(B[:,0]>=x0)&(B[:,0]<x0+0.01)]
    if len(s): print(round(x0,3),len(s),'y',np.round([s[:,1].min(),s[:,1].max()],3),'zmax',np.round(s[:,2].max(),3))
"

# openrua op 75
timeout 900 python3 -c "
from rob import *
r=Robot()
qh=hand_quat([0,0,-1],[1,0,0])   # pads close along y
prev=r.arm_q()
def go(p,secs=3):
    global prev
    s=best_ik(r,np.array(p),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('noik',p); return
    c=r.move_joints(s,secs); prev=np.array(r.arm_q()); print(' tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2)); return c
go([-0.097,-0.165,1.10],4)
go([-0.097,-0.165,0.995],3)
r.gripper(0.0)
go([-0.097,-0.165,1.04],3)
" 2>&1 | grep -v INFO
python3 tools/perception/cam_snap.py frontview >/dev/null; python3 tools/perception/cam_snap.py agentview >/dev/null; echo ok

# openrua op 76
python3 tools/perception/cam_snap.py birdview >/dev/null; python3 -c "
import cv2
im=cv2.imread('birdview.png'); cv2.imwrite('crop_b.png', cv2.resize(im[225:305,205:300],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 77
timeout 600 python3 -c "
from rob import *
r=Robot()
r.gripper(0.08)
qh=hand_quat([0,0,-1],[1,0,0])
s=best_ik(r,np.array([-0.097,-0.19,1.15]),qh,seed=r.arm_q(),prefer=r.arm_q(),avoid=lambda L: not clear_of_door(L))
r.move_joints(s,3); print('tcp',np.round(r.tcp()[0],4)); print(np.round(r.arm_q(),3))
" 2>&1 | grep -v INFO
python3 geo.py tf >/dev/null; python3 -c "
import json; c=json.load(open('cams.json')); print([k for k in c if 'eye' in k]); print(c.get('robot0_eye_in_hand_optical_frame'))"
python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; python3 geo.py cloud robot0_eye_in_hand

# openrua op 78
timeout 120 python3 -c "
import numpy as np, cv2
P=np.load('robot0_eye_in_hand_cloud.npy')
Z=P[...,2]
m=np.isfinite(Z)&(Z>0.915)&(Z<1.05)
pts=P[m]; print('obj pts',len(pts),'x',np.round([pts[:,0].min(),pts[:,0].max()],3),'y',np.round([pts[:,1].min(),pts[:,1].max()],3))
# handle region: y > -0.205
H=pts[pts[:,1]>-0.205]
for y0 in np.arange(-0.205,-0.15,0.005):
    s=H[(H[:,1]>=y0)&(H[:,1]<y0+0.005)]
    if len(s): print(round(y0,3),len(s),'x',np.round([s[:,0].min(),s[:,0].max()],3),'z',np.round([s[:,2].min(),s[:,2].max()],3))
im=cv2.imread('robot0_eye_in_hand.png'); cv2.imwrite('crop_e.png',im)
"

# openrua op 79
timeout 120 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); Z=P[...,2]
m=np.isfinite(Z)&(Z>0.915)&(Z<1.05)&(P[...,0]>-0.2)&(P[...,0]<0.0)&(P[...,1]>-0.32)&(P[...,1]<-0.15)
pts=P[m]
body=pts[pts[:,2]>0.93]
print('body+handle centroid',np.round(body.mean(0),3))
# highest ridge of body -> axis line
hi=pts[pts[:,2]>0.985]
print('ridge pts',len(hi),'x',np.round([hi[:,0].min(),hi[:,0].max()],3),'y',np.round([hi[:,1].min(),hi[:,1].max()],3))
# grid occupancy top-down, 1cm cells, showing max z
xs=np.arange(-0.19,-0.01,0.01); ys=np.arange(-0.32,-0.15,0.01)
print('     y:'+''.join(f'{y*100:4.0f}' for y in ys))
for x in xs[::-1]:
    row=''
    for y in ys:
        s=pts[(pts[:,0]>=x)&(pts[:,0]<x+0.01)&(pts[:,1]>=y)&(pts[:,1]<y+0.01)]
        row+= '   .' if len(s)<3 else f'{(s[:,2].max()-0.9)*100:4.0f}'
    print(f'x={x*100:4.0f}'+row)
"

# openrua op 80
timeout 1200 python3 -c "
from rob import *
r=Robot()
r.gripper(0.0)
qh=hand_quat([0,0,-1],[0,1,0])   # vertical hand, finger axis along y
prev=r.arm_q()
def go(p,secs=3):
    global prev
    s=best_ik(r,np.array(p),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('noik',p); return None
    c=r.move_joints(s,secs); prev=np.array(r.arm_q()); print(' tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2)); return c
piv=np.array([-0.14,0.90]); R=0.154; y=-0.245
def tcp_at(phi):
    e=piv+R*np.array([np.cos(np.deg2rad(phi)),np.sin(np.deg2rad(phi))])
    return [e[0]+0.012, y, e[1]+0.005]
p0=tcp_at(35.8); go([p0[0],y,1.06],4); go(p0,3)
for phi in [42,48,54,60,66]:
    print('phi',phi); go(tcp_at(phi),2.5)
go([r.tcp()[0][0], y, 1.10],3)
" 2>&1 | grep -v INFO
python3 tools/perception/cam_snap.py frontview >/dev/null; python3 tools/perception/cam_snap.py agentview >/dev/null; echo ok

# openrua op 81
timeout 1200 python3 -c "
from rob import *
r=Robot()
qh=hand_quat([0,0,-1],[0,1,0])
prev=r.arm_q()
def go(p,secs=3):
    global prev
    s=best_ik(r,np.array(p),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('noik',p); return None
    c=r.move_joints(s,secs); prev=np.array(r.arm_q()); print(' tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2)); return c
piv=np.array([-0.14,0.90]); R=0.154; y=-0.245
def tcp_at(phi):
    e=piv+R*np.array([np.cos(np.deg2rad(phi)),np.sin(np.deg2rad(phi))])
    return [e[0]+0.014, y, e[1]-0.012]
p0=tcp_at(34); go([p0[0]+0.01,y,1.06],3); go([p0[0]+0.01,y,p0[2]],3); go(p0,2)
for phi in [40,46,52,58,64,70]:
    print('phi',phi); go(tcp_at(phi),2.5)
go([r.tcp()[0][0], y, 1.10],3)
" 2>&1 | grep -v INFO
python3 tools/perception/cam_snap.py frontview >/dev/null; echo ok

# openrua op 82
python3 geo.py tf >/dev/null; python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; python3 geo.py cloud robot0_eye_in_hand >/dev/null
timeout 120 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_cloud.npy'); Z=P[...,2]
m=np.isfinite(Z)&(Z>0.915)&(Z<1.06)&(P[...,0]>-0.2)&(P[...,0]<0.05)&(P[...,1]>-0.32)&(P[...,1]<-0.15)
pts=P[m]
xs=np.arange(-0.19,0.03,0.01); ys=np.arange(-0.32,-0.15,0.01)
print('     y:'+''.join(f'{y*100:4.0f}' for y in ys))
for x in xs[::-1]:
    row=''
    for y in ys:
        s=pts[(pts[:,0]>=x)&(pts[:,0]<x+0.01)&(pts[:,1]>=y)&(pts[:,1]<y+0.01)]
        row+= '   .' if len(s)<3 else f'{(s[:,2].max()-0.9)*100:4.0f}'
    print(f'x={x*100:4.0f}'+row)
"

# openrua op 83
timeout 1200 python3 -c "
from rob import *
r=Robot()
qh=hand_quat([0,0,-1],[0,1,0])
prev=r.arm_q()
def go(p,secs=3):
    global prev
    s=best_ik(r,np.array(p),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('noik',p); return None
    c=r.move_joints(s,secs); prev=np.array(r.arm_q()); print(' tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2)); return c
piv=np.array([-0.14,0.90]); R=0.154; y=-0.245
def edge(phi): return piv+R*np.array([np.cos(np.deg2rad(phi)),np.sin(np.deg2rad(phi))])
def tcp_at(phi, dx=0.003, dz=-0.015):
    e=edge(phi); return [e[0]+dx, y, e[1]+dz]
e0=edge(35.8)
go([e0[0]+0.035,y,1.06],3); go([e0[0]+0.035,y,e0[1]-0.015],3); go(tcp_at(35.8),2)
for phi in [40,45,50,55,60,65,70]:
    print('phi',phi); go(tcp_at(phi),2.5)
go([r.tcp()[0][0], y, 1.10],3)
" 2>&1 | grep -v INFO
python3 tools/perception/cam_snap.py frontview >/dev/null; echo ok

# openrua op 84
python3 tools/perception/cam_snap.py birdview birdview.png && python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
d=np.load('birdview_depth.npy'); P=cloud('birdview',d)
np.save('birdview_cloud.npy',P)
m=(P[...,0]>-0.3)&(P[...,0]<0.05)&(P[...,1]>-0.42)&(P[...,1]<-0.1)&(P[...,2]>0.905)&(P[...,2]<1.05)
pts=P[m]
print(len(pts))
for z0 in np.arange(0.90,1.02,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s): print(round(z0,2), len(s), 'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
"


# openrua op 85
python3 -c "
import cv2
im=cv2.imread('birdview.png'); h,w=im.shape[:2]
# birdview cam at (-0.2,0,3.0) looking down; crop around the mug region
c=im[150:400, 150:450]; cv2.imwrite('crop_b.png', cv2.resize(c,None,fx=2,fy=2))
"

# openrua op 86
grep -n "^def \|^class \|^    def \|^NOMINAL" rob.py | head -50

# openrua op 87
cat > park.py <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('park')
prev=np.array(r.arm_q()); print('start tcp',np.round(r.tcp(),4),'q',np.round(prev,3))
qh=hand_quat([0,0,-1],[0,1,0])
# retreat to a pose far from mug region so birdview sees the mug: over -x side, high
for p in [(-0.10,-0.25,1.25),(-0.35,-0.05,1.30)]:
    s=best_ik(r,np.array(p),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('no ik',p); break
    r.move_joints(s,3.0); prev=np.array(r.arm_q())
    print('tcp',np.round(r.tcp(),4))
EOF
python3 park.py 2>&1 | tail -5 && python3 tools/perception/cam_snap.py birdview birdview.png && python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 -c "
import numpy as np, sys, cv2
sys.path.insert(0,'.')
from geo import cloud
d=np.load('birdview_depth.npy'); P=cloud('birdview',d); np.save('birdview_cloud.npy',P)
m=(P[...,0]>-0.3)&(P[...,0]<0.05)&(P[...,1]>-0.42)&(P[...,1]<-0.1)&(P[...,2]>0.905)&(P[...,2]<1.05)
pts=P[m]; print(len(pts))
for z0 in np.arange(0.90,1.02,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s): print(round(z0,2), len(s), 'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
im=cv2.imread('birdview.png'); c=im[150:400,150:450]; cv2.imwrite('crop_b.png',cv2.resize(c,None,fx=2,fy=2))
"

# openrua op 88
python3 park.py 2>&1 | head -30; sed -n 100,130p rob.py

# openrua op 89
sed -i 's/np.round(r.tcp(),4)/np.round(r.tcp()[0],4)/g' park.py && python3 park.py 2>&1 | tail -4 && python3 tools/perception/cam_snap.py birdview birdview.png && python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 -c "
import numpy as np, sys, cv2
sys.path.insert(0,'.')
from geo import cloud
d=np.load('birdview_depth.npy'); P=cloud('birdview',d); np.save('birdview_cloud.npy',P)
m=(P[...,0]>-0.3)&(P[...,0]<0.05)&(P[...,1]>-0.42)&(P[...,1]<-0.1)&(P[...,2]>0.905)&(P[...,2]<1.05)
pts=P[m]; print(len(pts))
for z0 in np.arange(0.90,1.02,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s): print(round(z0,2), len(s), 'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
im=cv2.imread('birdview.png'); c=im[150:400,150:450]; cv2.imwrite('crop_b.png',cv2.resize(c,None,fx=2,fy=2))
"

# openrua op 90
python3 -c "
import numpy as np
P=np.load('birdview_cloud.npy')
# body-only region x in [-0.145,-0.105]
m=(P[...,0]>-0.145)&(P[...,0]<-0.105)&(P[...,1]>-0.42)&(P[...,1]<-0.1)&(P[...,2]>0.905)
pts=P[m]
for z0 in np.arange(0.90,1.02,0.005):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.005)]
    if len(s): print(round(z0,3), len(s), 'y',s[:,1].min().round(3),s[:,1].max().round(3))
# mouth end: x max at various z near mouth
m2=(P[...,0]>-0.08)&(P[...,0]<0.0)&(P[...,1]>-0.32)&(P[...,1]<-0.20)&(P[...,2]>0.905)
q=P[m2]
print('mouth region top z',q[:,2].max().round(3),'xmax',q[:,0].max().round(3))
for z0 in [0.93,0.95,0.97,0.98,0.99]:
    s=q[(q[:,2]>=z0)&(q[:,2]<z0+0.01)]
    if len(s): print(z0,'xmax',s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
# table extents at z~0.9
t=P[(np.abs(P[...,2]-0.90)<0.01)]
print('table x',t[:,0].min().round(2),t[:,0].max().round(2),'y',t[:,1].min().round(2),t[:,1].max().round(2))
"


# openrua op 91
python3 tools/perception/cam_snap.py /frontview/depth/image_raw frontview_depth.png >/dev/null && python3 -c "
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
d=np.load('frontview_depth.npy'); P=cloud('frontview',d); np.save('frontview_cloud.npy',P)
m=(P[...,0]>-0.20)&(P[...,0]<0.02)&(P[...,1]>-0.36)&(P[...,1]<-0.14)&(P[...,2]>0.903)&(P[...,2]<1.05)
pts=P[m]; print(len(pts))
print('x range',pts[:,0].min().round(3),pts[:,0].max().round(3))
# rim: points with x > -0.06 (near mouth)
for z0 in np.arange(0.90,1.01,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s): print(round(z0,2), len(s), 'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
"


# openrua op 92
sed -n 255,335p rob.py

# openrua op 93
cat > slide.py <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('slide')
prev=np.array(r.arm_q()); print('start tcp',np.round(r.tcp()[0],4))
qh=hand_quat([0,0,-1],[0,1,0])
r.gripper(0.0)
def go(p,secs=2.5):
    global prev
    s=best_ik(r,np.array(p),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('no ik',p); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2))
y=-0.28; z=0.973
go((0.0,y,1.08)); go((0.0,y,z))
for x in np.arange(-0.03,-0.19,-0.03):
    go((x,y,z),2.0)
go((-0.18,y,1.1)); go((-0.10,-0.05,1.30),3.0)
EOF
python3 slide.py 2>&1 | grep -v "^\[move\] code=0"

# openrua op 94
python3 tools/perception/cam_snap.py birdview birdview.png >/dev/null && python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null && python3 -c "
import numpy as np, sys, cv2
sys.path.insert(0,'.')
from geo import cloud
d=np.load('birdview_depth.npy'); P=cloud('birdview',d); np.save('birdview_cloud.npy',P)
m=(P[...,0]>-0.45)&(P[...,0]<0.05)&(P[...,1]>-0.45)&(P[...,1]<-0.1)&(P[...,2]>0.905)&(P[...,2]<1.1)
pts=P[m]; print(len(pts))
for z0 in np.arange(0.90,1.06,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s): print(round(z0,2), len(s), 'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
im=cv2.imread('birdview.png'); c=im[150:400,100:400]; cv2.imwrite('crop_b.png',cv2.resize(c,None,fx=2,fy=2))
"

# openrua op 95
python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[280:420,120:320]; cv2.imwrite('crop_f.png',cv2.resize(c,None,fx=3,fy=3))
"

# openrua op 96
cat > look.py <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('look')
prev=np.array(r.arm_q())
x,y,z=map(float,sys.argv[1:4])
qh=hand_quat([0,0,-1],[1,0,0])
s=best_ik(r,np.array([x,y,z]),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
r.move_joints(s,3.0); print('tcp',np.round(r.tcp()[0],4))
EOF
python3 look.py -0.12 -0.25 1.32 2>&1 | tail -1 && python3 geo.py tf >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && mv eih_depth.npy robot0_eye_in_hand_depth.npy && python3 -c "
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
d=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',d); np.save('eih_cloud.npy',P)
m=(P[...,0]>-0.45)&(P[...,0]<0.05)&(P[...,1]>-0.45)&(P[...,1]<-0.1)&(P[...,2]>0.905)&(P[...,2]<1.1)
pts=P[m]; print(len(pts))
for z0 in np.arange(0.90,1.06,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s): print(round(z0,2), len(s), 'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
"

# openrua op 97
python3 -c "
import numpy as np
P=np.load('eih_cloud.npy')
m=(P[...,0]>-0.45)&(P[...,0]<0.05)&(P[...,1]>-0.45)&(P[...,1]<-0.1)&(P[...,2]>0.92)&(P[...,2]<1.1)
pts=P[m]
# occupancy grid of top heights, 1cm cells
xs=np.arange(-0.20,-0.04,0.01); ys=np.arange(-0.34,-0.17,0.01)
print('     '+' '.join(f'{y:+.2f}'[1:] for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=pts[(np.abs(pts[:,0]-x)<0.005)&(np.abs(pts[:,1]-y)<0.005)]
        row.append(f'{(s[:,2].max()-0.9)*100:4.1f}' if len(s) else '  . ')
    print(f'{x:+.2f}',' '.join(row))
"


# openrua op 98
python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); print(im.shape)
c=im[140:340,200:440]; cv2.imwrite('crop_e.png',cv2.resize(c,None,fx=3,fy=3))
"

# openrua op 99
python3 look.py -0.09 -0.24 1.17 2>&1 | tail -1 && python3 geo.py tf >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && mv eih_depth.npy robot0_eye_in_hand_depth.npy && python3 -c "
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
d=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',d); np.save('eih_cloud.npy',P)
m=(P[...,0]>-0.25)&(P[...,0]<0.0)&(P[...,1]>-0.40)&(P[...,1]<-0.1)&(P[...,2]>0.92)&(P[...,2]<1.1)
pts=P[m]
xs=np.arange(-0.19,-0.04,0.005); ys=np.arange(-0.33,-0.17,0.005)
print('      '+' '.join(f'{-y*100:3.0f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=pts[(np.abs(pts[:,0]-x)<0.0025)&(np.abs(pts[:,1]-y)<0.0025)]
        row.append(f'{(s[:,2].max()-0.9)*100:3.0f}' if len(s) else '  .')
    print(f'{x:+.3f}',' '.join(row))
"

# openrua op 100
cat > look2.py <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('look2')
prev=np.array(r.arm_q())
x,y,z=map(float,sys.argv[1:4]); d=np.array(list(map(float,sys.argv[4:7])))
xd=np.array(list(map(float,sys.argv[7:10]))) if len(sys.argv)>9 else np.array([1,0,0])
qh=hand_quat(d,xd)
s=best_ik(r,np.array([x,y,z]),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
if s is None: print('no ik'); sys.exit(1)
r.move_joints(s,3.0); print('tcp',np.round(r.tcp()[0],4),'q',np.round(r.arm_q(),2))
EOF
python3 look2.py -0.13 -0.55 1.02 0 0.94 -0.34 0 0.34 0.94 2>&1 | tail -1 && python3 geo.py tf >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && mv eih_depth.npy robot0_eye_in_hand_depth.npy && python3 -c "
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
d=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',d); np.save('eih_cloud.npy',P)
m=(P[...,0]>-0.25)&(P[...,0]<0.0)&(P[...,1]>-0.40)&(P[...,1]<-0.1)&(P[...,2]>0.905)&(P[...,2]<1.1)
pts=P[m]; print(len(pts),'ymin',pts[:,1].min().round(3))
# the mouth: points with the smallest y; per z slice show y-min and x-extent of points within 5mm of ymin
for z0 in np.arange(0.90,1.0,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s):
        ym=s[:,1].min(); f=s[s[:,1]<ym+0.006]
        print(round(z0,2),len(s),'ymin',ym.round(3),'front x',f[:,0].min().round(3),f[:,0].max().round(3),'| all x',s[:,0].min().round(3),s[:,0].max().round(3))
"

# openrua op 101
python3 look2.py -0.13 -0.47 1.05 0 0.866 -0.5 0 0.5 0.866 2>&1 | tail -1

# openrua op 102
python3 look2.py -0.13 -0.42 1.10 0 0.766 -0.643 0 0.643 0.766 2>&1 | tail -1 || python3 look2.py -0.13 -0.40 1.12 0 0.643 -0.766 0 0.766 0.643 2>&1 | tail -1

# openrua op 103
python3 geo.py tf >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && mv eih_depth.npy robot0_eye_in_hand_depth.npy && python3 -c "
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
d=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',d); np.save('eih_cloud.npy',P)
m=(P[...,0]>-0.25)&(P[...,0]<0.0)&(P[...,1]>-0.40)&(P[...,1]<-0.1)&(P[...,2]>0.905)&(P[...,2]<1.1)
pts=P[m]; print(len(pts),'ymin',pts[:,1].min().round(3))
for z0 in np.arange(0.90,1.0,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s):
        ym=s[:,1].min(); f=s[s[:,1]<ym+0.006]
        print(round(z0,2),len(s),'ymin',ym.round(3),'front x',f[:,0].min().round(3),f[:,0].max().round(3),'| all x',s[:,0].min().round(3),s[:,0].max().round(3))
# the front face (mouth ring): points with y < ymin+0.012 -> fit circle in x-z
f=pts[pts[:,1]<pts[:,1].min()+0.012]
print('ring pts',len(f),'x',f[:,0].min().round(3),f[:,0].max().round(3),'z',f[:,2].min().round(3),f[:,2].max().round(3))
"

# openrua op 104
python3 -c "
import numpy as np
P=np.load('eih_cloud.npy')
m=(P[...,0]>-0.25)&(P[...,0]<0.0)&(P[...,1]>-0.40)&(P[...,1]<-0.1)&(P[...,2]>0.905)&(P[...,2]<1.1)
pts=P[m]
xs=np.arange(-0.19,-0.05,0.01); zs=np.arange(0.905,1.0,0.01)
print('z\\\\x '+' '.join(f'{x:+.2f}' for x in xs))
for z in zs:
    row=[]
    for x in xs:
        s=pts[(np.abs(pts[:,0]-x)<0.005)&(np.abs(pts[:,2]-z)<0.005)]
        row.append(f'{-s[:,1].min()*100:5.1f}' if len(s) else '   . ')
    print(f'{z:.3f}',' '.join(row))
print('(values = -y of the nearest surface point, cm)')
"

# openrua op 105
cat > pinch.py <<'EOF'
import numpy as np, sys, json
sys.path.insert(0,'.')
from rob import *
r=Robot('pinch')
prev=np.array(r.arm_q())
p=np.array([-0.149,-0.314,0.947])          # ring point, -x side
n_in=np.array([-0.31,0.95,0.0])            # into the mouth
yhat=np.array([0.95,0.31,0.0])             # closing axis (radial)
b=np.deg2rad(35)
d=np.cos(b)*n_in+np.sin(b)*np.array([0,0,-1.0])
qh=hand_quat(d, np.cross(yhat,d))
tcp=p+0.018*n_in
def go(pt,secs=2.5):
    global prev
    s=best_ik(r,np.array(pt),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2))
r.gripper(0.08)
go(tcp-0.12*d,4.0); go(tcp-0.06*d); go(tcp-0.03*d,2.0); go(tcp,2.0)
r.gripper(0.0)
print('fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
go(tcp+np.array([0,0,0.05]),2.0)
go(tcp+np.array([0,0,0.12]),2.5)
print('fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
json.dump(dict(q=list(map(float,r.arm_q())),d=d.tolist(),yhat=yhat.tolist()),open('pinch_state.json','w'))
EOF
python3 pinch.py 2>&1 | grep -v "code=0"

# openrua op 106
cat > ikcheck.py <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('ikc')
prev=np.array(r.arm_q())
p=np.array([-0.149,-0.314,0.947]); n_in=np.array([-0.31,0.95,0.0]); yhat=np.array([0.95,0.31,0.0])
for beta in [35,45,55]:
    b=np.deg2rad(beta); d=np.cos(b)*n_in+np.sin(b)*np.array([0,0,-1.0]); qh=hand_quat(d,np.cross(yhat,d)); tcp=p+0.018*n_in
    for k in [0.08,0.05,0.0]:
        s=best_ik(r,tcp-k*d,qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
        print(beta,k,None if s is None else np.round(s,2))
EOF
python3 ikcheck.py 2>&1 | grep -v code=

# openrua op 107
sed -i 's/qh=hand_quat(d,np.cross(yhat,d))/qh=hand_quat(d,-np.cross(yhat,d))/; s/for beta in \[35,45,55\]/for beta in [45]/' ikcheck.py && python3 ikcheck.py 2>&1 | grep -v code=

# openrua op 108
sed -i 's/b=np.deg2rad(35)/b=np.deg2rad(45)/; s/qh=hand_quat(d, np.cross(yhat,d))/qh=hand_quat(d, -np.cross(yhat,d))/; s/go(tcp-0.12\*d,4.0); go(tcp-0.06\*d)/go(tcp-0.08*d,4.0); go(tcp-0.05*d)/' pinch.py && grep -n "deg2rad\|hand_quat\|go(tcp-" pinch.py && python3 pinch.py 2>&1 | grep -v "code=0"

# openrua op 109
python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null && python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[250:430,100:340]; cv2.imwrite('crop_f.png',cv2.resize(c,None,fx=3,fy=3))
"

# openrua op 110
python3 look.py -0.12 -0.25 1.22 2>&1 | tail -1 && python3 geo.py tf >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && mv eih_depth.npy robot0_eye_in_hand_depth.npy && python3 -c "
import numpy as np, sys, cv2
sys.path.insert(0,'.')
from geo import cloud
d=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',d); np.save('eih_cloud.npy',P)
im=cv2.imread('robot0_eye_in_hand.png')
# overlay grid: pixels whose world x or y is within 0.7mm of a 2cm multiple, at table height
X,Y,Z=P[...,0],P[...,1],P[...,2]
gx=(np.abs((X+1)%0.02-0.0)<0.0007)|(np.abs((X+1)%0.02-0.02)<0.0007)
gy=(np.abs((Y+1)%0.02-0.0)<0.0007)|(np.abs((Y+1)%0.02-0.02)<0.0007)
tab=Z<0.905
im[gx&tab]=(0,0,255); im[gy&tab]=(255,0,0)
# label a few points
for xv in [-0.20,-0.16,-0.12,-0.08,-0.04]:
  for yv in [-0.34,-0.30,-0.26,-0.22,-0.18]:
    m=(np.abs(X-xv)<0.002)&(np.abs(Y-yv)<0.002)&tab
    if m.any():
        v,u=np.argwhere(m)[0]; cv2.putText(im,f'{xv:+.2f},{yv:+.2f}',(u-20,v),cv2.FONT_HERSHEY_PLAIN,0.6,(0,255,0),1)
cv2.imwrite('eih_grid.png',im)
print(im.shape)
"

# openrua op 111
cat > ortho.py <<'EOF'
import numpy as np, cv2, sys
P=np.load(sys.argv[1]); im=cv2.imread(sys.argv[2]); out=sys.argv[3]
x0,x1,y0,y1=map(float,sys.argv[4:8]); res=0.002
W=int((x1-x0)/res); H=int((y1-y0)/res)
img=np.zeros((H,W,3),np.uint8); zb=np.full((H,W),-1.0)
X,Y,Z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(Z)&(X>=x0)&(X<x1)&(Y>=y0)&(Y<y1)
us=((X[m]-x0)/res).astype(int); vs=((Y[m]-y0)/res).astype(int)
cols=im[m]; zs=Z[m]
order=np.argsort(zs)   # paint highest last
for u,v,c,z in zip(us[order],vs[order],cols[order],zs[order]):
    img[v,u]=c; zb[v,u]=z
# image row = y (increasing downward -> flip so +y is up), col = x
img=img[::-1]; 
# grid every 2cm
for xv in np.arange(np.ceil(x0/0.02)*0.02,x1,0.02):
    u=int((xv-x0)/res); cv2.line(img,(u,0),(u,H-1),(0,0,255) if abs(xv%0.1)<1e-6 or abs(xv%0.1-0.1)<1e-6 else (0,0,120),1)
    cv2.putText(img,f'{xv:.2f}',(u+1,H-3),cv2.FONT_HERSHEY_PLAIN,0.7,(0,255,255),1)
for yv in np.arange(np.ceil(y0/0.02)*0.02,y1,0.02):
    v=H-1-int((yv-y0)/res); cv2.line(img,(0,v),(W-1,v),(255,0,0) if abs(yv%0.1)<1e-6 or abs(yv%0.1-0.1)<1e-6 else (120,0,0),1)
    cv2.putText(img,f'{yv:.2f}',(2,v-2),cv2.FONT_HERSHEY_PLAIN,0.7,(0,255,255),1)
img=cv2.resize(img,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)
cv2.imwrite(out,img); print(out,img.shape)
EOF
python3 ortho.py eih_cloud.npy robot0_eye_in_hand.png ortho.png -0.22 -0.02 -0.36 -0.14

# openrua op 112
sed -i 's/us=((X\[m\]-x0)\/res).astype(int); vs=((Y\[m\]-y0)\/res).astype(int)/us=np.clip(((X[m]-x0)\/res).astype(int),0,W-1); vs=np.clip(((Y[m]-y0)\/res).astype(int),0,H-1)/' ortho.py && python3 ortho.py eih_cloud.npy robot0_eye_in_hand.png ortho.png -0.22 -0.02 -0.36 -0.14

# openrua op 113
sed -i 's/^img=img\[::-1\]; $/img=np.ascontiguousarray(img[::-1])/' ortho.py && python3 ortho.py eih_cloud.npy robot0_eye_in_hand.png ortho.png -0.22 -0.02 -0.36 -0.14

# openrua op 114
sed -i 's/res=0.002/res=0.001/; s/cv2.FONT_HERSHEY_PLAIN,0.7/cv2.FONT_HERSHEY_PLAIN,0.5/g' ortho.py && sed -i 's/img=cv2.resize(img,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)/img=cv2.resize(img,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)/' ortho.py && python3 ortho.py eih_cloud.npy robot0_eye_in_hand.png ortho.png -0.22 -0.02 -0.36 -0.14

# openrua op 115
python3 look.py -0.12 -0.26 1.13 2>&1 | tail -1 && python3 geo.py tf >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && mv eih_depth.npy robot0_eye_in_hand_depth.npy && python3 -c "
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
d=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',d); np.save('eih_cloud.npy',P)
" && python3 ortho.py eih_cloud.npy robot0_eye_in_hand.png ortho.png -0.22 -0.04 -0.36 -0.16

# openrua op 116
python3 -c "
import json, numpy as np, sys
sys.path.insert(0,'.')
from geo import quat_R
c=json.load(open('cams.json'))
for k in ['frontview_optical_frame','sideview_optical_frame','agentview_optical_frame','birdview_optical_frame']:
    if k in c:
        R=quat_R(*c[k]['q']); print(k,'pos',np.round(c[k]['t'],3),'look(z)',np.round(R[:,2],3),'right(x)',np.round(R[:,0],3))
"; python3 tools/perception/cam_snap.py sideview sideview.png

# openrua op 117
cat > prepose.py <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('pre')
prev=np.array(r.arm_q())
p=np.array([-0.149,-0.324,0.947]); n_in=np.array([-0.34,0.94,0.0]); yhat=np.array([0.94,0.34,0.0])
b=np.deg2rad(45); d=np.cos(b)*n_in+np.sin(b)*np.array([0,0,-1.0]); qh=hand_quat(d,-np.cross(yhat,d))
tcp=p+0.018*n_in
k=float(sys.argv[1]) if len(sys.argv)>1 else 0.06
r.gripper(0.08)
s=best_ik(r,tcp-k*d,qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
r.move_joints(s,4.0); print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2))
EOF
python3 prepose.py 0.06 2>&1 | grep -v code= && python3 geo.py tf >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png

# openrua op 118
python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && mv eih_depth.npy robot0_eye_in_hand_depth.npy && python3 -c "
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
d=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',d); np.save('eih_cloud.npy',P)
X,Y,Z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(Z)&(X>-0.25)&(X<-0.03)&(Y>-0.40)&(Y<-0.15)&(Z>0.905)&(Z<1.05)
pts=P[m]; print(len(pts))
# front-most y per (x,z) cell, 5mm
xs=np.arange(-0.20,-0.04,0.01); zs=np.arange(0.905,1.005,0.01)
print('z\\\\x  '+' '.join(f'{x:+.2f}' for x in xs))
for z in zs:
    row=[]
    for x in xs:
        s=pts[(np.abs(pts[:,0]-x)<0.005)&(np.abs(pts[:,2]-z)<0.005)]
        row.append(f'{-s[:,1].min()*100:5.1f}' if len(s) else '   . ')
    print(f'{z:.3f}',' '.join(row))
# ring top: highest points
top=pts[pts[:,2]>0.985]; print('top z',top[:,2].max().round(4),'x',top[:,0].min().round(3),top[:,0].max().round(3),'y',top[:,1].min().round(3),top[:,1].max().round(3))
"

# openrua op 119
cat > pinch2.py <<'EOF'
import numpy as np, sys, json, subprocess
sys.path.insert(0,'.')
from rob import *
r=Robot('pinch2')
prev=np.array(r.arm_q())
c=np.array([-0.105,-0.298,0.947]); R=0.0465
t=np.array([0.866,0.5,0.0]); n_in=np.array([-0.5,0.866,0.0])
p=c-R*t
b=np.deg2rad(45); d=np.cos(b)*n_in-np.sin(b)*np.array([0,0,1.0])
qh=hand_quat(d,-np.cross(t,d))
tcp=p+0.018*n_in
mode=sys.argv[1]
def go(pt,secs=2.5):
    global prev
    s=best_ik(r,np.array(pt),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(w,2)); return w
if mode=='pre':
    r.gripper(0.08); go(tcp-0.06*d,4.0)
elif mode=='approach':
    w0=r.wrench()[:3]
    for k in [0.045,0.03,0.02,0.01,0.0]:
        w=go(tcp-k*d,1.5)
        if np.linalg.norm(w-w0)>2.0: print('CONTACT at k',k); break
elif mode=='close':
    r.gripper(0.0); print('fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
elif mode=='lift':
    cur=r.tcp()[0]
    go(cur+np.array([0,0,0.04]),2.0); go(cur+np.array([0,0,0.12]),2.5)
    print('fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
EOF
python3 pinch2.py pre 2>&1 | grep -v code= && python3 geo.py tf >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png

# openrua op 120
cat >> pinch2.py <<'EOF'
if mode=='shift':
    dy=float(sys.argv[2])           # metres along hand-y (image right)
    yh=-t
    cur=r.tcp()[0]; go(cur+dy*yh,2.0)
EOF
python3 pinch2.py shift 0.014 2>&1 | grep -v code= && python3 geo.py tf >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); cv2.line(im,(320,0),(320,479),(0,255,0),1); cv2.imwrite('eih_mark.png',im)"

# openrua op 121
cat > approach2.py <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('appr')
prev=np.array(r.arm_q())
c=np.array([-0.105,-0.298,0.947]); t=np.array([0.866,0.5,0.0]); n_in=np.array([-0.5,0.866,0.0])
b=np.deg2rad(45); d=np.cos(b)*n_in-np.sin(b)*np.array([0,0,1.0]); qh=hand_quat(d,-np.cross(t,d))
start=r.tcp()[0]; w0=r.wrench()[:3]
total=float(sys.argv[1])
def go(pt,secs):
    global prev
    s=best_ik(r,np.array(pt),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(w,2),'dw',np.round(np.linalg.norm(w-w0),2)); return w
for k in np.arange(0.01,total+1e-6,0.01):
    w=go(start+k*d,1.2)
    if np.linalg.norm(w-w0)>2.5: print('CONTACT at k',round(k,3)); break
EOF
python3 approach2.py 0.078 2>&1 | grep -v code=

# openrua op 122
python3 geo.py tf >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); cv2.line(im,(320,0),(320,479),(0,255,0),1); cv2.imwrite('eih_mark.png',im)"

# openrua op 123
sed -i 's/if np.linalg.norm(w-w0)>2.5: print/if np.linalg.norm(w-w0)>float(sys.argv[2]): print/' approach2.py && python3 approach2.py 0.02 9.0 2>&1 | grep -v code= && python3 geo.py tf >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); cv2.line(im,(320,0),(320,479),(0,255,0),1); cv2.imwrite('eih_mark.png',im)"

# openrua op 124
python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && mv eih_depth.npy robot0_eye_in_hand_depth.npy && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_depth.npy'); fx=312.774; cx=320; cy=240
print('depth stats', np.nanmin(d), np.nanmax(d))
# print coarse depth grid (every 40px cols, 40px rows)
for v in range(20,480,40):
    print(f'{v:3d}',' '.join(f'{d[v,u]:5.3f}' if np.isfinite(d[v,u]) else '  nan' for u in range(20,640,40)))
"

# openrua op 125
python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_depth.npy'); fx=312.774; cx=320; cy=240
H,W=d.shape; uu,vv=np.meshgrid(np.arange(W),np.arange(H))
X=(uu-cx)*d/fx; Y=(vv-cy)*d/fx; Z=d
hy=X; hx=0.041-Y; hz=0.012+Z      # hand-frame coords (y=closing axis, x=hand x, z=approach)
m=np.isfinite(d)&(d<0.3)&(vv<410)
# mug points: exclude table plane. Table in hand frame: world z=0.90. Need world; use TCP pose: tcp=(-0.1663,-0.3173,0.9556), d=(-0.354,0.612,-0.707), xhat=(0.354,-0.612,-0.707), yhat=-t=(-0.866,-0.5,0)
tcp=np.array([-0.1663,-0.3173,0.9556]); D=np.array([-0.354,0.612,-0.707]); XH=np.array([0.354,-0.612,-0.707]); YH=np.array([-0.866,-0.5,0])
wz=tcp[2]+(hz-0.1034)*D[2]+hx*XH[2]+hy*YH[2]
wx=tcp[0]+(hz-0.1034)*D[0]+hx*XH[0]+hy*YH[0]
wy=tcp[1]+(hz-0.1034)*D[1]+hx*XH[1]+hy*YH[1]
obj=m&(wz>0.907)
print('object pts',obj.sum())
# ring: for hand-z slices, show hand-y extents; also wz range
for z0 in np.arange(0.06,0.20,0.01):
    s=obj&(hz>=z0)&(hz<z0+0.01)
    if s.sum(): print(f'hz {z0:.2f} n={s.sum():5d} hy[{hy[s].min():+.3f},{hy[s].max():+.3f}] hx[{hx[s].min():+.3f},{hx[s].max():+.3f}] wz[{wz[s].min():.3f},{wz[s].max():.3f}] wx[{wx[s].min():.3f},{wx[s].max():.3f}] wy[{wy[s].min():.3f},{wy[s].max():.3f}]')
# fingers check: pixels with d<0.1 near bottom corners
f=m&(vv>330)&(vv<405)&(d<0.1)
print('finger px hand coords: hy',hy[f].min().round(3),hy[f].max().round(3),'hx',hx[f].min().round(3),hx[f].max().round(3),'hz',hz[f].min().round(3),hz[f].max().round(3))
"

# openrua op 126
cat > nudge.py <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('nudge')
prev=np.array(r.arm_q())
t=np.array([0.866,0.5,0.0]); n_in=np.array([-0.5,0.866,0.0])
b=np.deg2rad(45); d=np.cos(b)*n_in-np.sin(b)*np.array([0,0,1.0]); qh=hand_quat(d,-np.cross(t,d))
xh=-np.cross(t,d); yh=np.cross(d,xh)
print('xhat',np.round(xh,3),'yhat',np.round(yh,3))
dd,dx,dy=map(float,sys.argv[1:4])
cur=r.tcp()[0]
tgt=cur+dd*d+dx*xh+dy*yh
s=best_ik(r,tgt,qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
r.move_joints(s,2.5); print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2))
EOF
python3 nudge.py -0.03 0.025 0 2>&1 | grep -v code= && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); cv2.line(im,(320,0),(320,479),(0,255,0),1); cv2.imwrite('eih_mark.png',im)"

# openrua op 127
cp eih_mark.png eih_prev.png; python3 nudge.py 0 0.03 0 2>&1 | grep -v code= && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); cv2.line(im,(320,0),(320,479),(0,255,0),1); cv2.imwrite('eih_mark.png',im)"

# openrua op 128
python3 nudge.py 0 0.02 0 2>&1 | grep -v code= && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); cv2.line(im,(320,0),(320,479),(0,255,0),1); cv2.imwrite('eih_mark.png',im)"

# openrua op 129
python3 -c "
import numpy as np
D=np.load('robot0_eye_in_hand_depth.npy')
for v in [5,40,80,120,160,200,240,280,320,350,380]:
    print(v, np.round(D[v,[380,450,520,600]],3))
"

# openrua op 130
python3 -c "
import numpy as np
D=np.load('robot0_eye_in_hand_depth.npy'); f=312.774
t=np.array([0.866,0.5,0.0]); n_in=np.array([-0.5,0.866,0.0])
d=0.707*n_in-0.707*np.array([0,0,1.0]); xh=-np.cross(t,d); yh=np.cross(d,xh)
vs,us=np.mgrid[0:400:20,360:640:20]; vs=vs.ravel(); us=us.ravel()
Z=D[vs,us]; m=np.isfinite(Z)&(Z<1)
best=None
for phi in np.deg2rad(np.arange(-90,91,2)):
    for h in np.arange(0.06,0.14,0.002):
        right=np.cos(phi)*yh+np.sin(phi)*xh
        down=np.cos(phi)*(-xh)+np.sin(phi)*yh
        rays=d[None,:]+((us[m]-320)/f)[:,None]*right+((vs[m]-240)/f)[:,None]*down
        # depth along optical axis: point = cam + Z*rays (rays with unit optical component)
        z_pt = -h + Z[m]*rays[:,2]
        err=np.sqrt(np.mean(z_pt**2))
        if best is None or err<best[0]: best=(err,np.rad2deg(phi),h)
print(best)
"

# openrua op 131
python3 -c "
import numpy as np
D=np.load('robot0_eye_in_hand_depth.npy'); f=312.774
vs,us=np.mgrid[0:400:10,360:640:10]; vs=vs.ravel(); us=us.ravel()
Z=D[vs,us]; m=np.isfinite(Z)&(Z<1)
P=np.stack([(us[m]-320)*Z[m]/f,(vs[m]-240)*Z[m]/f,Z[m]],1)
c=P.mean(0); U,S,Vt=np.linalg.svd(P-c); n=Vt[2]; 
if n[2]>0: n=-n
print('table normal in cam frame',np.round(n,3),'dist',np.round(-n@c,4),'resid',S[2]/np.sqrt(len(P)))
t=np.array([0.866,0.5,0.0]); n_in=np.array([-0.5,0.866,0.0])
d=0.707*n_in-0.707*np.array([0,0,1.0]); xh=-np.cross(t,d); yh=np.cross(d,xh)
# expected: world z in cam frame under model cam_x=yh, cam_y=-xh, cam_z=d
z=np.array([0,0,1.0]); print('expected', np.round([z@yh, z@(-xh), z@d],3))
"

# openrua op 132
python3 geo.py tf >/dev/null 2>&1; python3 -c "
import json,numpy as np
from geo import quat_R
c=json.load(open('cams.json'))
for k in ['panda_hand','robot0_eye_in_hand_optical_frame','panda_link8']:
    if k in c:
        R=quat_R(*c[k]['q']); print(k,'t',np.round(c[k]['t'],4)); print(np.round(R,3))
print([k for k in c if 'eye' in k or 'hand' in k])
"

# openrua op 133
python3 -c "
import numpy as np,sys
sys.path.insert(0,'.')
from rob import *
from geo import quat_R
r=Robot('chk')
p,q=r.tcp(); print('tcp',np.round(p,4),'quat',np.round(q,4))
print(np.round(quat_R(*q),3))
t=np.array([0.866,0.5,0.0]); n_in=np.array([-0.5,0.866,0.0])
d=0.707*n_in-0.707*np.array([0,0,1.0]); qh=hand_quat(d,-np.cross(t,d)); print('qh',np.round(qh,4)); print(np.round(quat_R(*qh),3))
" 2>&1 | grep -v code=

# openrua op 134
grep -n "link8\|panda_hand\|ik_link\|fk_link\|tcp\b\|TCP" rob.py | head -40

# openrua op 135
sed -i 's/        req.ik_request.group_name = "panda_arm"/        req.ik_request.group_name = "panda_arm"\n        req.ik_request.ik_link_name = "panda_hand"/' rob.py && python3 -c "
import numpy as np,sys
sys.path.insert(0,'.')
from rob import *
from geo import quat_R
r=Robot('chk')
p,q=r.tcp()
s=r.ik(p,q,tcp=True); print('ik for current pose',s and np.round(s,3)); print('cur',np.round(r.arm_q(),3))
if s: print('fk of sol',np.round(r.tcp(s)[0],4),np.round(r.tcp(s)[1],4))
" 2>&1 | grep -v code=

# openrua op 136
python3 -c "
import numpy as np,sys
sys.path.insert(0,'.')
from rob import *
r=Robot('ret')
p,q=r.tcp(); d=quat_R(q)[:,2]
s=best_ik(r,p-0.05*d,q,seed=r.arm_q(),prefer=r.arm_q())
r.move_joints(s,2.5); print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2))
" 2>&1 | grep -v code= && python3 pinch2.py pre 2>&1 | grep -v code=

# openrua op 137
python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); cv2.line(im,(320,0),(320,479),(0,255,0),1)
for x in (164,476): cv2.line(im,(x,0),(x,479),(255,0,0),1)
cv2.imwrite('eih_mark.png',im)"

# openrua op 138
python3 -c "
import numpy as np, json
from geo import quat_R
D=np.load('robot0_eye_in_hand_depth.npy'); f=312.774
c=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame']
R=quat_R(*c['q']); T=np.array(c['t'])
vs,us=np.mgrid[0:400,0:640]
Z=D; X=(us-320)*Z/f; Y=(vs-240)*Z/f
P=np.stack([X,Y,Z],-1)@R.T+T   # world
wz=P[...,2]
obj=(wz>0.905)&(Z<0.4)
hx=0.05-Y; hy=X; hz=Z
print('object px',obj.sum())
# rim = points closest along hz per column band; show slices in hy
for lo,hi in [(-0.06,-0.04),(-0.04,-0.02),(-0.02,0.0),(0.0,0.01),(0.01,0.02),(0.02,0.04)]:
    m=obj&(hy>=lo)&(hy<hi)
    if m.sum()==0: print(lo,hi,'none'); continue
    zmin=hz[m].min()
    mm=m&(hz<zmin+0.01)
    print(f'hy[{lo},{hi}] n={m.sum()} hz_min={zmin:.3f} front hx range {hx[mm].min():.3f}..{hx[mm].max():.3f}  wz {wz[mm].min():.3f}..{wz[mm].max():.3f}')
# right edge of mouth: object points with max hy
m=obj; i=np.argmax(hy[m]); print('max hy',hy[m].max(), 'at hx',hx[m][i],'hz',hz[m][i],'world',P[m][i])
"

# openrua op 139
cat > rim.py <<'EOF'
import numpy as np, json
from geo import quat_R
D=np.load('robot0_eye_in_hand_depth.npy'); f=312.774
c=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame']
R=quat_R(*c['q']); T=np.array(c['t'])
vs,us=np.mgrid[0:480,0:640]
Z=D; X=(us-320)*Z/f; Y=(vs-240)*Z/f
P=np.stack([X,Y,Z],-1)@R.T+T   # world
wz=P[...,2]
obj=(wz>0.905)&(Z<0.4)&(vs<400)
hx=0.05-Y; hy=X; hz=Z
print('object px',obj.sum())
for lo,hi in [(-0.08,-0.06),(-0.06,-0.04),(-0.04,-0.02),(-0.02,0.0),(0.0,0.01),(0.01,0.02),(0.02,0.04)]:
    m=obj&(hy>=lo)&(hy<hi)
    if m.sum()==0: print(lo,hi,'none'); continue
    zmin=hz[m].min()
    mm=m&(hz<zmin+0.008)
    print(f'hy[{lo},{hi}] n={m.sum()} hz_min={zmin:.3f} front hx {hx[mm].min():.3f}..{hx[mm].max():.3f}  wz {wz[mm].min():.3f}..{wz[mm].max():.3f} wxy {P[mm][:,0].mean():.3f},{P[mm][:,1].mean():.3f}')
m=obj; i=np.argmax(hy[m]); print('max hy',round(hy[m].max(),4), 'at hx',round(hx[m][i],4),'hz',round(hz[m][i],4),'world',np.round(P[m][i],3))
EOF
python3 rim.py

# openrua op 140
python3 - <<'EOF'
import numpy as np, json
from geo import quat_R
D=np.load('robot0_eye_in_hand_depth.npy'); f=312.774
c=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame']
R=quat_R(*c['q']); T=np.array(c['t'])
vs,us=np.mgrid[0:480,0:640]
Z=D; X=(us-320)*Z/f; Y=(vs-240)*Z/f
P=np.stack([X,Y,Z],-1)@R.T+T
np.save('eih_cloud.npy',P)
obj=(P[...,2]>0.905)&(Z<0.5)&(Z>0.1)&(vs<400)
Q=P[obj]
print('n',len(Q)); print('x',np.percentile(Q[:,0],[0,2,50,98,100]).round(3)); print('y',np.percentile(Q[:,1],[0,2,50,98,100]).round(3)); print('z',np.percentile(Q[:,2],[0,50,98,100]).round(3))
EOF
python3 ortho.py eih_cloud.npy robot0_eye_in_hand.png ortho.png -0.25 -0.02 -0.45 -0.15 && echo ok

# openrua op 141
python3 tools/perception/cam_snap.py birdview birdview.png >/dev/null && python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 geo.py cloud birdview >/dev/null && python3 ortho.py birdview_cloud.npy birdview.png ortho_b.png -0.28 0.0 -0.45 -0.10 && echo ok

# openrua op 142
python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null && python3 tools/perception/cam_snap.py /frontview/depth/image_raw frontview_depth.png >/dev/null && python3 geo.py cloud frontview >/dev/null && python3 ortho.py frontview_cloud.npy frontview.png ortho_f.png -0.28 0.0 -0.45 -0.10 && python3 - <<'EOF'
import numpy as np
P=np.load('frontview_cloud.npy'); z=P[...,2]
m=(z>0.905)&(P[...,0]>-0.28)&(P[...,0]<0.0)&(P[...,1]>-0.45)&(P[...,1]<-0.1)&(z<1.05)
Q=P[m]; print('n',len(Q))
for zl in [0.91,0.93,0.95,0.97,0.99]:
    mm=(Q[:,2]>=zl)&(Q[:,2]<zl+0.02)
    if mm.sum(): print(zl, 'x',Q[mm,0].min().round(3),Q[mm,0].max().round(3),'y',Q[mm,1].min().round(3),Q[mm,1].max().round(3), mm.sum())
EOF

# openrua op 143
python3 park.py 2>&1 | grep -v code= | tail -3; python3 tools/perception/cam_snap.py birdview birdview.png >/dev/null && python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 geo.py cloud birdview >/dev/null && python3 ortho.py birdview_cloud.npy birdview.png ortho_b.png -0.28 0.0 -0.45 -0.10 && echo ok

# openrua op 144
cat > gridmap.py <<'EOF'
import numpy as np, sys
P=np.load(sys.argv[1]); x0,x1,y0,y1=map(float,sys.argv[2:6]); zmin=float(sys.argv[6]) if len(sys.argv)>6 else 0.905
cs=0.01
m=np.isfinite(P[...,2])&(P[...,2]>zmin)&(P[...,0]>=x0)&(P[...,0]<x1)&(P[...,1]>=y0)&(P[...,1]<y1)
Q=P[m]
nx=int(round((x1-x0)/cs)); ny=int(round((y1-y0)/cs))
G=np.full((ny,nx),np.nan)
ix=((Q[:,0]-x0)/cs).astype(int); iy=((Q[:,1]-y0)/cs).astype(int)
for a,b,z in zip(ix,iy,Q[:,2]):
    if np.isnan(G[b,a]) or z>G[b,a]: G[b,a]=z
print('      x:'+''.join(f'{x0+cs*i+cs/2:6.2f}' for i in range(nx)))
for b in range(ny-1,-1,-1):
    row=''.join('   .  ' if np.isnan(G[b,a]) else f'{(G[b,a]-0.90)*100:5.1f} ' for a in range(nx))
    print(f'y={y0+cs*b+cs/2:6.2f} '+row)
EOF
python3 gridmap.py birdview_cloud.npy -0.24 -0.04 -0.36 -0.16

# openrua op 145
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_cloud.npy')
m=np.isfinite(P[...,2])&(P[...,2]>0.905)&(P[...,0]>-0.24)&(P[...,0]<-0.04)&(P[...,1]>-0.36)&(P[...,1]<-0.16)
Q=P[m]
# PCA of the footprint of tall points
for a_deg in range(-50,10,5):
    a=np.array([np.sin(np.deg2rad(-a_deg)),-np.cos(np.deg2rad(-a_deg)),0])  # mouth direction ~ -y rotated
    a=np.array([np.cos(np.deg2rad(a_deg))*0+0,0,0])
import itertools
best=None
for th in np.deg2rad(np.arange(-60,61,2)):
    a=np.array([np.sin(th),-np.cos(th),0.0]); t=np.array([np.cos(th),np.sin(th),0.0])
    pa=Q@a; pt=Q@t
    # extent along t should be minimal (width of mug 0.093)
    w=np.percentile(pt,99)-np.percentile(pt,1); L=np.percentile(pa,99)-np.percentile(pa,1)
    if best is None or w<best[0]: best=(w,np.rad2deg(th),L)
print('best yaw (deg from -y toward +x)',best)
th=np.deg2rad(best[1]); a=np.array([np.sin(th),-np.cos(th),0.0]); t=np.array([np.cos(th),np.sin(th),0.0])
pa=Q@a; pt=Q@t
front=pa>np.percentile(pa,97)
F=Q[front]; print('rim front pts',len(F),'a-range',pa[front].min().round(3),pa[front].max().round(3))
print('rim centre est (mean of front pts)',F.mean(0).round(3),'t-range',(F@t).min().round(3),(F@t).max().round(3),'z',F[:,2].min().round(3),F[:,2].max().round(3))
print('t extent all',np.percentile(pt,[1,50,99]).round(3))
EOF

# openrua op 146
cat pinch2.py; echo ----; cat approach2.py

# openrua op 147
cat > mugmodel.py <<'EOF'
import numpy as np
c=np.array([-0.121,-0.299,0.9465]); R=0.0465
th=np.deg2rad(34.0)
t=np.array([np.cos(th),np.sin(th),0.0]); n_in=np.array([-np.sin(th),np.cos(th),0.0])   # n_in = into the mug (mouth->base)
p=c-R*t                     # -x side wall point
b=np.deg2rad(45); d=np.cos(b)*n_in-np.sin(b)*np.array([0,0,1.0])
EOF
sed -i "s/^c=np.array(\[-0.105,-0.298,0.947\]); R=0.0465$/from mugmodel import c,R,t,n_in,p,d/; s/^t=np.array(\[0.866,0.5,0.0\]); n_in=np.array(\[-0.5,0.866,0.0\])$//; s/^p=c-R\*t$//; s/^b=np.deg2rad(45); d=np.cos(b)\*n_in-np.sin(b)\*np.array(\[0,0,1.0\])$//" pinch2.py
sed -i "s/^c=np.array(\[-0.105,-0.298,0.947\]); t=np.array(\[0.866,0.5,0.0\]); n_in=np.array(\[-0.5,0.866,0.0\])$/from mugmodel import c,t,n_in,d/; s/^b=np.deg2rad(45); d=np.cos(b)\*n_in-np.sin(b)\*np.array(\[0,0,1.0\]); qh=hand_quat(d,-np.cross(t,d))$/qh=hand_quat(d,-np.cross(t,d))/" approach2.py
head -12 pinch2.py; echo ---; head -8 approach2.py
python3 -c "
from mugmodel import *; import numpy as np
print('p',p.round(4),'d',d.round(3),'tcp',(p+0.018*n_in).round(4))"

# openrua op 148
python3 pinch2.py pre 2>&1 | grep -v "code=1" ; python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); cv2.line(im,(320,0),(320,479),(0,255,0),1)
cv2.imwrite('eih_mark.png',im)"

# openrua op 149
python3 -c "
import numpy as np
D=np.load('robot0_eye_in_hand_depth.npy')
for v in [280,300,320,340,360]:
    print(v, [(u,round(float(D[v,u]),3)) for u in range(270,341,10)])
"

# openrua op 150
python3 pinch2.py shift -0.013 2>&1 | grep tcp && python3 approach2.py 0.06 3.0 2>&1 | grep -v "code="

# openrua op 151
python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); cv2.line(im,(320,0),(320,479),(0,255,0),1)
cv2.imwrite('eih_mark.png',im)"

# openrua op 152
python3 pinch2.py close 2>&1 | grep -v "code="

# openrua op 153
python3 - <<'EOF' 2>&1 | grep -v "code="
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
from mugmodel import *
r=Robot('lift1'); prev=np.array(r.arm_q())
qh=hand_quat(d,-np.cross(t,d))
cur=r.tcp()[0]
s=best_ik(r,cur+np.array([0,0,0.04]),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
r.move_joints(s,2.0)
print('tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
EOF
python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null; python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null

# openrua op 154
python3 tools/perception/cam_snap.py /frontview/depth/image_raw frontview_depth.png >/dev/null && python3 geo.py cloud frontview >/dev/null && python3 gridmap.py frontview_cloud.npy -0.26 -0.04 -0.42 -0.20 0.905

# openrua op 155
python3 - <<'EOF' 2>&1 | grep -v "code="
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
from mugmodel import *
r=Robot('lift2'); prev=np.array(r.arm_q())
qh=hand_quat(d,-np.cross(t,d))
cur=r.tcp()[0]
s=best_ik(r,np.array([cur[0],cur[1],1.10]),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
r.move_joints(s,3.0)
print('tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
EOF
python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null; python3 tools/perception/cam_snap.py /frontview/depth/image_raw frontview_depth.png >/dev/null && python3 geo.py cloud frontview >/dev/null; python3 tools/perception/cam_snap.py sideview sideview.png >/dev/null

# openrua op 156
python3 - <<'EOF'
import numpy as np
P=np.load('frontview_cloud.npy')
m=np.isfinite(P[...,2])&(P[...,2]>0.91)&(P[...,2]<1.25)&(P[...,0]>-0.30)&(P[...,0]<0.0)&(P[...,1]>-0.45)&(P[...,1]<-0.15)
Q=P[m]; print(len(Q))
# slice by z
for zl in np.arange(0.92,1.20,0.02):
    mm=(Q[:,2]>=zl)&(Q[:,2]<zl+0.02)
    if mm.sum()>5: print(f'z {zl:.2f}: n={mm.sum():4d} x {Q[mm,0].min():.3f}..{Q[mm,0].max():.3f}  y {Q[mm,1].min():.3f}..{Q[mm,1].max():.3f}')
EOF

# openrua op 157
python3 -c "
import numpy as np
P=np.load('frontview_cloud.npy')
for v in range(240,320,10):
    print(v, ' '.join(f'({u}:{P[v,u,0]:.2f},{P[v,u,1]:.2f},{P[v,u,2]:.2f})' for u in range(160,241,20)))
"

# openrua op 158
python3 - <<'EOF'
import numpy as np
P=np.load('frontview_cloud.npy')
m=np.isfinite(P[...,2])&(P[...,2]>0.95)&(P[...,2]<1.30)&(P[...,0]>-0.30)&(P[...,0]<0.05)&(P[...,1]>-0.50)&(P[...,1]<-0.10)
Q=P[m]
for xl in np.arange(-0.20,0.0,0.02):
    mm=(Q[:,0]>=xl)&(Q[:,0]<xl+0.02)
    if mm.sum()>5: print(f'x {xl:.2f}: n={mm.sum():4d} y {Q[mm,1].min():.3f}..{Q[mm,1].max():.3f}  z {Q[mm,2].min():.3f}..{Q[mm,2].max():.3f}')
EOF

# openrua op 159
python3 - <<'EOF'
import numpy as np
P=np.load('frontview_cloud.npy')
m=np.isfinite(P[...,2])&(P[...,2]>1.04)&(P[...,2]<1.17)&(P[...,0]>-0.19)&(P[...,0]<-0.05)&(P[...,1]>-0.36)&(P[...,1]<-0.17)
Q=P[m]; print(len(Q)); c=Q.mean(0); U,S,Vt=np.linalg.svd(Q-c,full_matrices=False)
print('centre',c.round(3)); print('axes',Vt.round(3)); print('sv',(S/np.sqrt(len(Q))).round(4))
a=Vt[0]; pa=(Q-c)@a; print('axis extent',pa.min().round(3),pa.max().round(3))
# end points
print('end1',(c+pa.min()*a).round(3),'end2',(c+pa.max()*a).round(3))
tcp=np.array([-0.1586,-0.3051,1.099]); print('tcp rel centre along axis',((tcp-c)@a).round(3))
EOF

# openrua op 160
python3 tools/perception/cam_snap.py /sideview/depth/image_raw sideview_depth.png >/dev/null && python3 geo.py cloud sideview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load('sideview_cloud.npy')
m=np.isfinite(P[...,2])&(P[...,2]>1.0)&(P[...,2]<1.30)&(P[...,0]>-0.25)&(P[...,0]<0.0)&(P[...,1]>-0.45)&(P[...,1]<-0.15)
Q=P[m]; print(len(Q))
for zl in np.arange(1.0,1.30,0.02):
    mm=(Q[:,2]>=zl)&(Q[:,2]<zl+0.02)
    if mm.sum()>5: print(f'z {zl:.2f}: n={mm.sum():4d} x {Q[mm,0].min():.3f}..{Q[mm,0].max():.3f}  y {Q[mm,1].min():.3f}..{Q[mm,1].max():.3f}')
print('---by y')
for yl in np.arange(-0.40,-0.15,0.02):
    mm=(Q[:,1]>=yl)&(Q[:,1]<yl+0.02)
    if mm.sum()>5: print(f'y {yl:.2f}: n={mm.sum():4d} x {Q[mm,0].min():.3f}..{Q[mm,0].max():.3f}  z {Q[mm,2].min():.3f}..{Q[mm,2].max():.3f}')
EOF

# openrua op 161
python3 tools/perception/cam_snap.py birdview birdview.png >/dev/null && python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 geo.py cloud birdview >/dev/null && python3 gridmap.py birdview_cloud.npy -0.26 -0.02 -0.46 -0.16 1.0

# openrua op 162
cat > carry.py <<'EOF'
import numpy as np, sys, json
sys.path.insert(0,'.')
from rob import *
r=Robot('carry'); prev=np.array(r.arm_q())
d2=np.array([-0.5,0.7071,-0.5]); x2=np.array([-0.5,-0.7071,-0.5])
d2/=np.linalg.norm(d2); x2/=np.linalg.norm(x2)
qh2=hand_quat(d2,x2)
print('y_hand',np.round(np.cross(d2,x2),3))
W=[(-0.10,-0.20,1.15),(-0.043,0.05,1.15),(-0.043,0.10,1.023),(-0.043,0.20,1.023),(-0.043,0.30,1.023)]
sols=[]
for w in W:
    s=best_ik(r,np.array(w),qh2,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    print(w, None if s is None else np.round(s,2))
    if s is not None: prev=np.array(s)
    sols.append(s)
json.dump([None if s is None else list(map(float,s)) for s in sols],open('carry_sols.json','w'))
if len(sys.argv)>1 and sys.argv[1]=='go':
    n=int(sys.argv[2]) if len(sys.argv)>2 else len(W)
    for i,s in enumerate(sols[:n]):
        r.move_joints(s,4.0)
        print('W',i,'tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
EOF
python3 carry.py 2>&1 | grep -v code=

# openrua op 163
python3 - <<'EOF' 2>&1 | grep -v code=
import numpy as np, sys, json
sys.path.insert(0,'.')
from rob import *
r=Robot('ikt')
sols=json.load(open('carry_sols.json')); seed=np.array(sols[3])
d2=np.array([-0.5,0.7071,-0.5]); x2=np.array([-0.5,-0.7071,-0.5]); d2/=np.linalg.norm(d2); x2/=np.linalg.norm(x2); qh2=hand_quat(d2,x2)
for y in [0.24,0.27,0.30]:
    s=best_ik(r,np.array([-0.043,y,1.023]),qh2,seed=seed,prefer=seed,n=60)
    print(y,'no-avoid', None if s is None else np.round(s,2))
    if s is not None:
        L=links_fk(r,s); print('  clear_of_door',clear_of_door(L)); 
        for k,v in L.items(): print('  ',k,np.round(v,3))
    s2=best_ik(r,np.array([-0.043,y,1.023]),qh2,seed=seed,prefer=seed,n=60,avoid=lambda L: not clear_of_door(L))
    print(y,'avoid', None if s2 is None else np.round(s2,2))
EOF

# openrua op 164
grep -n "def clear_of_door" -A 25 rob.py

# openrua op 165
cat > carry2.py <<'EOF'
import numpy as np, sys, json
sys.path.insert(0,'.')
from rob import *
r=Robot('carry2'); prev=np.array(r.arm_q())
d2=np.array([-0.5,0.7071,-0.5]); x2=np.array([-0.5,-0.7071,-0.5]); d2/=np.linalg.norm(d2); x2/=np.linalg.norm(x2); qh2=hand_quat(d2,x2)
def door_only(L):
    for k in ('panda_link5','panda_link6','panda_link7','panda_hand'):
        p=L[k]
        if p[2]<0.95: return False
        dx=max(-0.20-p[0],0,p[0]+0.14); dy=max(0.0-p[1],0,p[1]-0.30); dz=max(0,p[2]-1.12)
        if np.sqrt(dx*dx+dy*dy+dz*dz)<0.05: return False
    return True
def go(pt,secs,avoid):
    global prev
    s=best_ik(r,np.array(pt),qh2,seed=prev,prefer=prev,avoid=avoid)
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4),'wrench',np.round(w,2)); return w
stage=sys.argv[1]
if stage=='A':
    for w in [(-0.10,-0.20,1.15),(-0.043,0.05,1.15),(-0.043,0.10,1.023),(-0.043,0.20,1.023)]:
        go(w,4.0,lambda L: not clear_of_door(L))
elif stage=='B':
    w0=r.wrench()[:3]
    for y in [0.23,0.26,0.28,0.30]:
        w=go((-0.043,y,1.023),2.0,lambda L: not door_only(L))
        print('dw',np.round(np.linalg.norm(w-w0),2))
        if np.linalg.norm(w-w0)>2.5: print('CONTACT'); break
EOF
python3 carry2.py A 2>&1 | grep -v code=

# openrua op 166
python3 tools/perception/cam_snap.py frontview frontview.png >/dev/null; python3 tools/perception/cam_snap.py agentview agentview.png >/dev/null; echo done

# openrua op 167
python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 geo.py cloud birdview >/dev/null && python3 gridmap.py birdview_cloud.npy -0.22 0.12 0.04 0.32 0.93

# openrua op 168
cat > carry3.py <<'EOF'
import numpy as np, sys, json
sys.path.insert(0,'.')
from rob import *
r=Robot('carry3'); prev=np.array(r.arm_q())
d2=np.array([-0.5,0.7071,-0.5]); x2=np.array([-0.5,-0.7071,-0.5]); d2/=np.linalg.norm(d2); x2/=np.linalg.norm(x2); qh2=hand_quat(d2,x2)
def door_only(L):
    for k in ('panda_link5','panda_link6','panda_link7','panda_hand'):
        p=L[k]
        if p[2]<0.95: return False
        dx=max(-0.22-p[0],0,p[0]+0.16); dy=max(0.0-p[1],0,p[1]-0.30); dz=max(0,p[2]-1.12)
        if np.sqrt(dx*dx+dy*dy+dz*dz)<0.05: return False
    return True
def go(pt,secs):
    global prev
    s=best_ik(r,np.array(pt),qh2,seed=prev,prefer=prev,avoid=lambda L: not door_only(L))
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4),'wrench',np.round(w,2)); return w
stage=sys.argv[1]
X=-0.073
if stage=='shift':
    go((X,0.20,1.0226),2.0)
elif stage=='insert':
    w0=r.wrench()[:3]
    for y in [0.23,0.26,0.28,0.30]:
        w=go((X,y,1.0226),2.0)
        print('dw',np.round(np.linalg.norm(w-w0),2))
        if np.linalg.norm(w-w0)>2.5: print('CONTACT'); break
elif stage=='release':
    r.gripper(0.08); print('fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
    cur=r.tcp()[0]
    go((cur[0],0.22,cur[2]),2.5); go((cur[0],0.12,cur[2]+0.05),2.5)
EOF
python3 carry3.py shift 2>&1 | grep -v code= && python3 carry3.py insert 2>&1 | grep -v code=

# openrua op 169
cat >> carry3.py <<'EOF'
elif stage=='retry':
    cur=r.tcp()[0]
    go((X,0.22,cur[2]),2.0); go((X,0.22,1.038),2.0)
    w0=r.wrench()[:3]
    for y in [0.25,0.27,0.29,0.31]:
        w=go((X,y,1.038),2.0)
        print('dw',np.round(np.linalg.norm(w-w0),2))
        if np.linalg.norm(w-w0)>2.5: print('CONTACT'); break
EOF
python3 carry3.py retry 2>&1 | grep -v code=

# openrua op 170
python3 geo.py tf >/dev/null 2>&1; python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; echo ok

# openrua op 171
python3 - <<'EOF'
import numpy as np, json
from geo import quat_R
D=np.load('robot0_eye_in_hand_depth.npy'); f=312.774
c=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame']
R=quat_R(*c['q']); T=np.array(c['t'])
vs,us=np.mgrid[0:480,0:640]
Z=D; X=(us-320)*Z/f; Y=(vs-240)*Z/f
P=(np.stack([X,Y,Z],-1)@R.T+T)
ok=np.isfinite(Z)&(Z<1.0)&(vs<400)
Q=P[ok]; print(len(Q))
print('cam',T.round(3))
# structure: histogram of y for points with z in cavity range and x in cavity range
m=(Q[:,2]>0.93)&(Q[:,2]<1.12)&(Q[:,0]>-0.16)&(Q[:,0]<0.1)
for yl in np.arange(0.24,0.50,0.02):
    mm=m&(Q[:,1]>=yl)&(Q[:,1]<yl+0.02)
    if mm.sum()>20: print(f'y {yl:.2f}: n={mm.sum():5d} x {Q[mm,0].min():.3f}..{Q[mm,0].max():.3f} z {Q[mm,2].min():.3f}..{Q[mm,2].max():.3f}')
print('--- by z (y 0.29..0.44)')
m2=(Q[:,1]>0.29)&(Q[:,1]<0.44)&(Q[:,0]>-0.16)&(Q[:,0]<0.1)
for zl in np.arange(0.92,1.12,0.01):
    mm=m2&(Q[:,2]>=zl)&(Q[:,2]<zl+0.01)
    if mm.sum()>20: print(f'z {zl:.2f}: n={mm.sum():5d} x {Q[mm,0].min():.3f}..{Q[mm,0].max():.3f} y {Q[mm,1].min():.3f}..{Q[mm,1].max():.3f}')
EOF

# openrua op 172
cat >> carry3.py <<'EOF'
elif stage=='probe':
    X2=float(sys.argv[2]); Z2=float(sys.argv[3])
    cur=r.tcp()[0]
    go((cur[0],0.265,cur[2]),1.5); go((X2,0.265,Z2),1.5)
    w0=r.wrench()[:3]
    for y in [0.285,0.30,0.315]:
        w=go((X2,y,Z2),1.5)
        print('dw',np.round(np.linalg.norm(w-w0),2))
        if np.linalg.norm(w-w0)>2.5: print('CONTACT'); break
EOF
python3 carry3.py probe -0.088 1.0375 2>&1 | grep -v code=

# openrua op 173
python3 - <<'EOF'
import numpy as np, json
from geo import quat_R
D=np.load('robot0_eye_in_hand_depth.npy'); f=312.774
c=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame']
R=quat_R(*c['q']); T=np.array(c['t'])
vs,us=np.mgrid[0:480,0:640]
Z=D; X=(us-320)*Z/f; Y=(vs-240)*Z/f
P=(np.stack([X,Y,Z],-1)@R.T+T)
ok=np.isfinite(Z)&(Z<1.0)&(vs<400)
Q=P[ok]
m=(Q[:,0]>-0.13)&(Q[:,0]<0.1)&(Q[:,2]>0.95)&(Q[:,2]<1.08)&(Q[:,1]>0.20)
for yl in np.arange(0.20,0.46,0.01):
    mm=m&(Q[:,1]>=yl)&(Q[:,1]<yl+0.01)
    if mm.sum()>10: print(f'y {yl:.2f}: n={mm.sum():5d} x {Q[mm,0].min():.3f}..{Q[mm,0].max():.3f} z {Q[mm,2].min():.3f}..{Q[mm,2].max():.3f}')
EOF

# openrua op 174
python3 - <<'EOF'
import numpy as np, json
from geo import quat_R
D=np.load('robot0_eye_in_hand_depth.npy'); f=312.774
c=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame']
R=quat_R(*c['q']); T=np.array(c['t'])
vs,us=np.mgrid[0:480,0:640]
Z=D; X=(us-320)*Z/f; Y=(vs-240)*Z/f
P=(np.stack([X,Y,Z],-1)@R.T+T)
ok=np.isfinite(Z)&(Z<1.0)&(vs<400)
Q=P[ok]
# frame: points with x in -0.13..-0.10 (left of mug) and y 0.26..0.31: list z distribution
m=(Q[:,0]>-0.135)&(Q[:,0]<-0.10)&(Q[:,1]>0.255)&(Q[:,1]<0.31)
for zl in np.arange(0.93,1.12,0.01):
    mm=m&(Q[:,2]>=zl)&(Q[:,2]<zl+0.01)
    if mm.sum()>5: print(f'z {zl:.2f}: n={mm.sum():5d} y {Q[mm,1].min():.3f}..{Q[mm,1].max():.3f}')
print('--- face plane points y 0.26..0.285, all x')
m=(Q[:,1]>0.26)&(Q[:,1]<0.285)&(Q[:,0]>-0.2)&(Q[:,0]<0.1)
for zl in np.arange(0.90,1.12,0.01):
    mm=m&(Q[:,2]>=zl)&(Q[:,2]<zl+0.01)
    if mm.sum()>5: print(f'z {zl:.2f}: n={mm.sum():5d} x {Q[mm,0].min():.3f}..{Q[mm,0].max():.3f}')
EOF

# openrua op 175
python3 - <<'EOF'
import numpy as np, json, cv2
from geo import quat_R
D=np.load('robot0_eye_in_hand_depth.npy'); f=312.774
im=cv2.imread('robot0_eye_in_hand.png')
g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
white=(g>150)
vs,us=np.mgrid[0:480,0:640]
white&=(vs<400)
print('white px',white.sum())
Z=D
X=(us-320)*Z/f; Y=(vs-240)*Z/f
hx=0.05-Y; hy=X; hz=Z
w=white&np.isfinite(Z)&(Z<0.4)
print('mug white pts: hy range',hy[w].min().round(3),hy[w].max().round(3),' hx',hx[w].min().round(3),hx[w].max().round(3),' hz',hz[w].min().round(3),hz[w].max().round(3))
# rim (nearest along hz among white) 
c=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame']; R=quat_R(*c['q']); T=np.array(c['t'])
P=(np.stack([X,Y,Z],-1)@R.T+T)
print('world white: x',P[w][:,0].min().round(3),P[w][:,0].max().round(3),'y',P[w][:,1].min().round(3),P[w][:,1].max().round(3),'z',P[w][:,2].min().round(3),P[w][:,2].max().round(3))
# lowest white point in world
i=np.argmin(P[w][:,2]); print('lowest white',P[w][i].round(3))
# frontmost (max y) 
i=np.argmax(P[w][:,1]); print('max-y white',P[w][i].round(3))
i=np.argmin(P[w][:,1]); print('min-y white',P[w][i].round(3))
EOF

# openrua op 176
python3 - <<'EOF'
import numpy as np, json, cv2
from geo import quat_R
D=np.load('robot0_eye_in_hand_depth.npy'); f=312.774
im=cv2.imread('robot0_eye_in_hand.png'); g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
vs,us=np.mgrid[0:480,0:640]
Z=D; X=(us-320)*Z/f; Y=(vs-240)*Z/f
c=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame']; R=quat_R(*c['q']); T=np.array(c['t'])
P=(np.stack([X,Y,Z],-1)@R.T+T)
w=(g>120)&(vs<400)&np.isfinite(Z)&(P[...,0]>-0.135)&(P[...,0]<0.03)&(P[...,1]>0.2)&(P[...,1]<0.42)&(P[...,2]>0.94)&(P[...,2]<1.08)
print('n',w.sum())
Q=P[w]
for yl in np.arange(0.22,0.40,0.01):
    mm=(Q[:,1]>=yl)&(Q[:,1]<yl+0.01)
    if mm.sum()>10: print(f'y {yl:.2f}: n={mm.sum():5d} x {Q[mm,0].min():.3f}..{Q[mm,0].max():.3f} z {Q[mm,2].min():.3f}..{Q[mm,2].max():.3f}')
# save mask overlay
ov=im.copy(); ov[w]=(0,255,0); cv2.imwrite('eih_mask.png',ov)
EOF

# openrua op 177
python3 carry3.py probe -0.108 1.0375 2>&1 | grep -v code=

# openrua op 178
python3 carry3.py probe -0.095 1.052 2>&1 | grep -v code=

# openrua op 179
python3 geo.py tf >/dev/null 2>&1; python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null; python3 - <<'EOF'
import numpy as np, json
from geo import quat_R
D=np.load('robot0_eye_in_hand_depth.npy'); f=312.774
c=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame']; R=quat_R(*c['q']); T=np.array(c['t'])
vs,us=np.mgrid[0:480,0:640]
Z=D; X=(us-320)*Z/f; Y=(vs-240)*Z/f
P=(np.stack([X,Y,Z],-1)@R.T+T)
ok=np.isfinite(Z)&(Z<1.0)&(vs<400)
Q=P[ok]
m=(Q[:,0]>-0.13)&(Q[:,0]<0.07)&(Q[:,2]>0.94)&(Q[:,2]<1.09)
for yl in np.arange(0.30,0.48,0.01):
    mm=m&(Q[:,1]>=yl)&(Q[:,1]<yl+0.01)
    if mm.sum()>10: print(f'y {yl:.2f}: n={mm.sum():5d} x {Q[mm,0].min():.3f}..{Q[mm,0].max():.3f} z {Q[mm,2].min():.3f}..{Q[mm,2].max():.3f}')
# depth image stats: where does the camera see far?
print('cam',T.round(3),'axis',R[:,2].round(3))
EOF

# openrua op 180
timeout 60 ros2 param get /move_group robot_description 2>/dev/null | grep -n -A 12 '<link name="panda_hand"' | head -60

# openrua op 181
python3 - <<'EOF'
import numpy as np,struct
def load(fn):
    b=open(fn,'rb').read()
    if b[:5]==b'solid':
        v=[list(map(float,l.split()[1:])) for l in b.decode().splitlines() if l.strip().startswith('vertex')]
        return np.array(v)
    n=struct.unpack('<I',b[80:84])[0]
    a=np.frombuffer(b[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    return a['v'].reshape(-1,3).astype(float)
for f in ['hand','finger']:
    v=load(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl')
    print(f,'min',v.min(0).round(4),'max',v.max(0).round(4))
EOF

# openrua op 182
find / -iname 'hand*.stl' -o -iname 'finger*.stl' -o -iname 'hand*.obj' 2>/dev/null | head; ls /opt/ros/jazzy/share | grep -i panda

# openrua op 183
sed -n 1,40p carry3.py && cat cams.json | python3 -c "import json,sys; d=json.load(sys.stdin); [print(k,[round(v,3) for v in d[k]['t']]) for k in d if k.endswith('optical_frame')]"

# openrua op 184
cat > rel.py <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('rel'); prev=np.array(r.arm_q())
d2=np.array([-0.5,0.7071,-0.5]); x2=np.array([-0.5,-0.7071,-0.5]); d2/=np.linalg.norm(d2); x2/=np.linalg.norm(x2); qh2=hand_quat(d2,x2)
def door_only(L):
    for k in ('panda_link5','panda_link6','panda_link7','panda_hand'):
        p=L[k]
        if p[2]<0.95: return False
        dx=max(-0.22-p[0],0,p[0]+0.16); dy=max(0.0-p[1],0,p[1]-0.30); dz=max(0,p[2]-1.12)
        if np.sqrt(dx*dx+dy*dy+dz*dz)<0.05: return False
    return True
def go(pt,secs):
    global prev
    s=best_ik(r,np.array(pt),qh2,seed=prev,prefer=prev,avoid=lambda L: not door_only(L))
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4),'wrench',np.round(w,2)); return w
cur=r.tcp()[0]
go((cur[0],0.276,cur[2]),1.5)
go((cur[0],0.276,1.030),2.0)
r.gripper(0.08); print('OPEN fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
go((cur[0],0.22,1.030),2.0)
go((cur[0],0.12,1.08),2.5)
EOF
python3 rel.py

# openrua op 185
cat > push.py <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('push'); prev=np.array(r.arm_q())
pitch=np.deg2rad(float(sys.argv[2])) if len(sys.argv)>2 else 0.0   # down-tilt of hand z
z3=np.array([0,np.cos(pitch),-np.sin(pitch)]); y3=np.array([1.0,0,0]); x3=np.cross(y3,z3)
qh3=hand_quat(z3,x3)
def door_only(L):
    for k in ('panda_link5','panda_link6','panda_link7','panda_hand'):
        p=L[k]
        if p[2]<0.95: return False
        dx=max(-0.22-p[0],0,p[0]+0.16); dy=max(0.0-p[1],0,p[1]-0.30); dz=max(0,p[2]-1.12)
        if np.sqrt(dx*dx+dy*dy+dz*dz)<0.05: return False
    return True
def go(pt,secs):
    global prev
    s=best_ik(r,np.array(pt),qh3,seed=prev,prefer=prev,avoid=lambda L: not door_only(L))
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4),'wrench',np.round(w,2)); return w
mode=sys.argv[1]
if mode=='look':
    # via point above, then horizontal hand at entrance height
    go((-0.055,0.05,1.10),3.0)
    go((-0.055,0.10,1.00),2.5)
elif mode=='to':
    x,y,z=map(float,sys.argv[3:6]); go((x,y,z),2.5)
elif mode=='steps':
    z=float(sys.argv[3]); x=float(sys.argv[4]); ys=list(map(float,sys.argv[5:]))
    w0=r.wrench()[:3]
    for y in ys:
        w=go((x,y,z),1.5); dw=np.linalg.norm(w-w0); print('dw',np.round(dw,2))
        if dw>3.0: print('CONTACT'); break
EOF
python3 push.py look 0

# openrua op 186
python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; ls -la robot0_eye_in_hand*; python3 geo.py tf >/dev/null; python3 -c "
import json;d=json.load(open('cams.json'))['robot0_eye_in_hand_optical_frame'];print(d)"

# openrua op 187
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
print('depth stats', np.nanmin(D), np.nanmax(D), D.dtype, D.shape)
# region of the mug in the image
sub=P[230:350,255:400]
X,Y,Z=sub[...,0],sub[...,1],sub[...,2]
# rim = points with y between 0.24 and 0.30 (mouth plane) and z>0.93
for lo,hi in [(0.20,0.24),(0.24,0.26),(0.26,0.28),(0.28,0.30),(0.30,0.34),(0.34,0.40),(0.40,0.46)]:
    m=(Y>lo)&(Y<hi)&(Z>0.93)&(Z<1.09)
    if m.sum(): print(f'y {lo}-{hi}: n={m.sum()} x[{X[m].min():.3f},{X[m].max():.3f}] z[{Z[m].min():.3f},{Z[m].max():.3f}]')
# cavity floor / walls from whole image
allm=(P[...,1]>0.28)&(P[...,1]<0.5)
Pa=P[allm]; print('cavity pts z min',Pa[:,2].min().round(3),'y max',Pa[:,1].max().round(3))
m=(P[...,2]<0.95)&(P[...,1]>0.28); print('floor y range',P[m][:,1].min().round(3),P[m][:,1].max().round(3), 'floor z', np.median(P[m][:,2]).round(3))
m=(P[...,1]>0.42); print('back wall pts', m.sum(), 'y median', np.median(P[m][:,1]).round(3) if m.sum() else None)
EOF

# openrua op 188
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
np.set_printoptions(linewidth=250)
Y=P[...,1]
print('y (cm) rows 220..360 step 10, cols 240..420 step 10')
for v in range(220,361,10):
    print(v, ' '.join(f'{int(round(Y[v,u]*100)):3d}' if Y[v,u]<1 else '  .' for u in range(240,421,10)))
print('z (cm above table) same grid')
Z=P[...,2]
for v in range(220,361,10):
    print(v, ' '.join(f'{int(round((Z[v,u]-0.9)*100)):3d}' if Y[v,u]<1 else '  .' for u in range(240,421,10)))
print('x (cm) same grid')
X=P[...,0]
for v in range(220,361,10):
    print(v, ' '.join(f'{int(round(X[v,u]*100)):3d}' if Y[v,u]<1 else '  .' for u in range(240,421,10)))
EOF

# openrua op 189
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
sub=P[225:345,255:400].reshape(-1,3)
m=(sub[:,1]<0.30)&(sub[:,1]>0.20)&(sub[:,2]>0.945)&(sub[:,2]<1.07)
Q=sub[m]; print('rim pts',len(Q))
# fit circle in x-z
A=np.c_[2*Q[:,0],2*Q[:,2],np.ones(len(Q))]; b=Q[:,0]**2+Q[:,2]**2
cx,cz,c0=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c0+cx*cx+cz*cz)
print('circle centre x %.4f z %.4f R %.4f'%(cx,cz,R))
# plane fit for rim y as function of x,z
A2=np.c_[Q[:,0]-cx,Q[:,2]-cz,np.ones(len(Q))]; a,bz,y0=np.linalg.lstsq(A2,Q[:,1],rcond=None)[0]
print('rim plane: y = %.4f + %.3f*(x-cx) + %.3f*(z-cz)'%(y0,a,bz), 'yaw deg',np.degrees(np.arctan(a)),'pitch deg',np.degrees(np.arctan(bz)))
# handle: points at x>0.0 and y in 0.28..0.40
m2=(P[...,0]>0.0)&(P[...,0]<0.06)&(P[...,1]>0.27)&(P[...,1]<0.42)&(P[...,2]>0.95)&(P[...,2]<1.06)
H=P[m2]; print('handle-ish pts',len(H)); 
if len(H): print(' x',H[:,0].min().round(3),H[:,0].max().round(3),'y',H[:,1].min().round(3),H[:,1].max().round(3),'z',H[:,2].min().round(3),H[:,2].max().round(3))
EOF

# openrua op 190
python3 push.py to 0 -0.053 0.18 1.018 && python3 push.py steps 0 1.018 -0.053 0.22 0.25 0.27 0.29 0.31

# openrua op 191
python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; python3 geo.py tf >/dev/null; python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
Y=P[...,1]; Z=P[...,2]; X=P[...,0]
m=(Y>0.20)&(Y<0.42)&(Z>0.945)&(Z<1.07)&(X>-0.13)&(X<0.06)
Q=P[m]
# rim = nearest points: histogram of y
h,e=np.histogram(Q[:,1],bins=np.arange(0.20,0.43,0.01)); print(list(zip(np.round(e[:-1],2),h)))
mm=Q[:,1]<np.percentile(Q[:,1],5); print('nearest 5%: y',Q[mm][:,1].mean().round(3),'x',Q[mm][:,0].min().round(3),Q[mm][:,0].max().round(3),'z',Q[mm][:,2].min().round(3),Q[mm][:,2].max().round(3))
EOF

# openrua op 192
python3 push.py steps 0 1.018 -0.053 0.312 && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; python3 geo.py tf >/dev/null; python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
Y=P[...,1]; Z=P[...,2]; X=P[...,0]
m=(Y>0.27)&(Y<0.42)&(Z>0.945)&(Z<1.07)&(X>-0.13)&(X<0.06)
Q=P[m]
h,e=np.histogram(Q[:,1],bins=np.arange(0.27,0.43,0.01)); print(list(zip(np.round(e[:-1],2),h)))
mm=Q[:,1]<np.percentile(Q[:,1],3); print('nearest 3%: y',Q[mm][:,1].mean().round(3),'x',Q[mm][:,0].min().round(3),Q[mm][:,0].max().round(3),'z',Q[mm][:,2].min().round(3),Q[mm][:,2].max().round(3))
EOF

# openrua op 193
python3 push.py to 0 -0.053 0.16 1.018 >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; python3 geo.py tf >/dev/null; python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
Y=P[...,1]; Z=P[...,2]; X=P[...,0]
m=(Y>0.25)&(Y<0.42)&(Z>0.945)&(Z<1.07)&(X>-0.13)&(X<0.06)
Q=P[m]
h,e=np.histogram(Q[:,1],bins=np.arange(0.25,0.43,0.01)); print(list(zip(np.round(e[:-1],2),h)))
rim=Q[(Q[:,1]<0.33)]
A=np.c_[2*rim[:,0],2*rim[:,2],np.ones(len(rim))]; b=rim[:,0]**2+rim[:,2]**2
cx,cz,c0=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c0+cx*cx+cz*cz)
print('rim circle centre x %.4f z %.4f R %.4f n %d'%(cx,cz,R,len(rim)))
A2=np.c_[rim[:,0]-cx,rim[:,2]-cz,np.ones(len(rim))]; a,bz,y0=np.linalg.lstsq(A2,rim[:,1],rcond=None)[0]
print('rim plane y0 %.4f yaw %.1f pitch %.1f'%(y0,np.degrees(np.arctan(a)),np.degrees(np.arctan(bz))))
print('rim y min/max',rim[:,1].min().round(3),rim[:,1].max().round(3))
EOF

# openrua op 194
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
Y=P[...,1]; Z=P[...,2]; X=P[...,0]
m=(Y>0.25)&(Y<0.285)&(Z>0.945)&(Z<1.07)&(X>-0.13)&(X<0.06)
vv,uu=np.nonzero(m)
print('n',m.sum(),'rows',vv.min(),vv.max(),'cols',uu.min(),uu.max())
Q=P[m]; print('x',Q[:,0].min().round(3),Q[:,0].max().round(3),'z',Q[:,2].min().round(3),Q[:,2].max().round(3))
for lo,hi in [(0.25,0.26),(0.26,0.27),(0.27,0.28),(0.28,0.285)]:
    k=(Q[:,1]>=lo)&(Q[:,1]<hi)
    if k.sum(): print(lo,hi,k.sum(),'x',Q[k][:,0].min().round(3),Q[k][:,0].max().round(3),'z',Q[k][:,2].min().round(3),Q[k][:,2].max().round(3))
EOF

# openrua op 195
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
Y=P[...,1]
print('y (cm) rows 250..395 step 5, cols 255..395 step 5')
for v in range(250,396,5):
    print(v, ' '.join(f'{int(round(Y[v,u]*100)):3d}' if Y[v,u]<1 else '  .' for u in range(255,396,5)))
EOF

# openrua op 196
python3 push.py to 0 -0.047 0.22 1.018 && python3 push.py steps 0 1.018 -0.047 0.26 0.28 0.295 0.308

# openrua op 197
python3 push.py to 0 -0.047 0.22 1.011 && python3 push.py steps 0 1.011 -0.047 0.26 0.28 0.295 0.308

# openrua op 198
python3 push.py to 0 -0.047 0.16 1.011 >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; python3 geo.py tf >/dev/null; python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
Y=P[...,1]
print('y (cm) rows 250..395 step 5, cols 255..395 step 5')
for v in range(250,396,5):
    print(v, ' '.join(f'{int(round(Y[v,u]*100)):3d}' if Y[v,u]<1 else '  .' for u in range(255,396,5)))
Z=P[...,2]; X=P[...,0]
m=(Y>0.25)&(Y<0.33)&(Z>0.945)&(Z<1.07)&(X>-0.13)&(X<0.06)
rim=P[m]
A=np.c_[2*rim[:,0],2*rim[:,2],np.ones(len(rim))]; b=rim[:,0]**2+rim[:,2]**2
cx,cz,c0=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c0+cx*cx+cz*cz)
print('rim circle centre x %.4f z %.4f R %.4f n %d'%(cx,cz,R,len(rim)))
A2=np.c_[rim[:,0]-cx,rim[:,2]-cz,np.ones(len(rim))]; a,bz,y0=np.linalg.lstsq(A2,rim[:,1],rcond=None)[0]
print('rim plane y0 %.4f yaw %.1f pitch %.1f'%(y0,np.degrees(np.arctan(a)),np.degrees(np.arctan(bz))))
EOF

# openrua op 199
python3 - <<'EOF'
import sys; sys.path.insert(0,'.')
from rob import *
r=Robot('grip'); r.gripper(0.0); print(r.fingers())
EOF
python3 push.py to 0 -0.0895 0.22 1.006 && python3 push.py steps 0 1.006 -0.0895 0.26 0.275 0.29 0.305

# openrua op 200
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('dbg'); prev=np.array(r.arm_q())
z3=np.array([0,1.0,0]); y3=np.array([1.0,0,0]); x3=np.cross(y3,z3); qh3=hand_quat(z3,x3)
for pt in [(-0.0895,0.22,1.006),(-0.08,0.22,1.006),(-0.0895,0.16,1.006),(-0.0895,0.22,1.03)]:
    s=best_ik(r,np.array(pt),qh3,seed=prev,prefer=prev,n=60)
    print(pt,'plain ik', None if s is None else np.round(s,3))
    if s is not None:
        L=links_fk(r,s); print({k:np.round(v,3) for k,v in L.items() if k in ('panda_link6','panda_link7','panda_hand')})
EOF

# openrua op 201
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('dbg'); prev=np.array(r.arm_q())
z3=np.array([0,1.0,0]); y3=np.array([1.0,0,0]); x3=np.cross(y3,z3); qh3=hand_quat(z3,x3)
def door_only(L):
    for k in ('panda_link5','panda_link6','panda_link7','panda_hand'):
        p=L[k]
        if p[2]<0.95: return False
        dx=max(-0.22-p[0],0,p[0]+0.16); dy=max(0.0-p[1],0,p[1]-0.30); dz=max(0,p[2]-1.12)
        if np.sqrt(dx*dx+dy*dy+dz*dz)<0.05: return False
    return True
rng=np.random.default_rng(1)
for pt in [(-0.08,0.22,1.026),(-0.0895,0.22,1.006)]:
    s=best_ik(r,np.array(pt),qh3,seed=prev,prefer=prev,avoid=lambda L: not door_only(L),n=300,rng=rng)
    print(pt, None if s is None else np.round(s,3))
    if s is not None:
        L=links_fk(r,s); print({k:np.round(v,3) for k,v in L.items() if k in ('panda_link5','panda_link6','panda_link7','panda_hand')}); print('joint dist from prev',np.round(np.abs(s-prev).max(),2))
EOF

# openrua op 202
python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 geo.py cloud birdview >/dev/null && python3 gridmap.py birdview_cloud.npy -0.26 -0.10 -0.06 0.32 0.95 2>&1 | head -60

# openrua op 203
python3 - <<'EOF'
import re
s=open('push.py').read()
s=s.replace("""        dx=max(-0.22-p[0],0,p[0]+0.16); dy=max(0.0-p[1],0,p[1]-0.30); dz=max(0,p[2]-1.12)
        if np.sqrt(dx*dx+dy*dy+dz*dz)<0.05: return False""",
"""        # open door: strip ~3cm thick from hinge (-0.165,0.285) to free edge (-0.25,0.02)
        yc=min(max(p[1],0.0),0.30); xd=-0.165-0.32*(0.285-yc)+0.015
        dx=max(0.0,p[0]-xd) if p[0]>xd else 0.0
        if p[0]<xd: dx=0.0   # behind the door face: definitely bad
        dy=max(0.0-p[1],0,p[1]-0.30); dz=max(0,p[2]-1.12)
        if p[0]<xd+0.05 and dy<0.05 and dz<0.05: return False""")
open('push.py','w').write(s)
EOF
python3 push.py to 0 -0.0895 0.22 1.006 && python3 push.py steps 0 1.006 -0.0895 0.26 0.275 0.29 0.305

# openrua op 204
sed -i 's/if dw>3.0/if dw>float(__import__("os").environ.get("DWMAX","3.0"))/' push.py && DWMAX=9 python3 push.py steps 0 1.006 -0.0895 0.29 0.30 0.307

# openrua op 205
python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; python3 geo.py tf >/dev/null

# openrua op 206
python3 push.py to 0 -0.0895 0.22 1.006 >/dev/null; python3 push.py to 0 -0.05 0.14 1.01 >/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; python3 geo.py tf >/dev/null; python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
Y=P[...,1]; X=P[...,0]; Z=P[...,2]
print('y (cm) rows 230..400 step 5, cols 230..430 step 5')
for v in range(230,401,5):
    print(v, ' '.join(f'{int(round(Y[v,u]*100)):3d}' if Y[v,u]<1 else '  .' for u in range(230,431,5)))
print('x (cm) same')
for v in range(230,401,10):
    print(v, ' '.join(f'{int(round(X[v,u]*100)):3d}' if Y[v,u]<1 else '  .' for u in range(230,431,5)))
EOF

# openrua op 207
python3 push.py to 0 -0.045 0.22 1.0 && DWMAX=5 python3 push.py steps 0 1.0 -0.045 0.255 0.265 0.275 0.285 0.295 0.305

# openrua op 208
cat > look.py <<'EOF'
import numpy as np, sys, subprocess
sys.path.insert(0,'.')
subprocess.run(['python3','push.py','to','0',sys.argv[1] if len(sys.argv)>1 else '-0.05','0.14','1.01'],check=True,stdout=subprocess.DEVNULL)
subprocess.run(['python3','tools/perception/cam_snap.py','robot0_eye_in_hand','robot0_eye_in_hand.png'],stdout=subprocess.DEVNULL)
subprocess.run(['python3','tools/perception/cam_snap.py','/robot0_eye_in_hand/depth/image_raw','robot0_eye_in_hand_depth.png'],stdout=subprocess.DEVNULL)
subprocess.run(['python3','geo.py','tf'],stdout=subprocess.DEVNULL)
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
Y=P[...,1]; X=P[...,0]; Z=P[...,2]
print('y (cm) rows 230..360 step 5, cols 230..430 step 5')
for v in range(230,361,5):
    print(v, ' '.join(f'{int(round(Y[v,u]*100)):3d}' if Y[v,u]<1 else '  .' for u in range(230,431,5)))
print('x (cm) rows step 10')
for v in range(230,361,10):
    print(v, ' '.join(f'{int(round(X[v,u]*100)):3d}' if Y[v,u]<1 else '  .' for u in range(230,431,5)))
# mug points: inside cavity region, exclude walls (y<0.43), floor (z>0.95), ceiling
m=(Y>0.2)&(Y<0.43)&(Z>0.95)&(Z<1.075)&(X>-0.137)&(X<0.064)
Q=P[m]; print('mug pts',len(Q),'y min %.3f'%Q[:,1].min(),'x range %.3f %.3f'%(Q[:,0].min(),Q[:,0].max()))
near=Q[Q[:,1]<Q[:,1].min()+0.01]; print('nearest 1cm band: n',len(near),'x %.3f..%.3f z %.3f..%.3f'%(near[:,0].min(),near[:,0].max(),near[:,2].min(),near[:,2].max()))
EOF
python3 look.py -0.05

# openrua op 209
python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 geo.py cloud birdview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3)
m=(P[:,2]>1.09)&(P[:,2]<1.13)&(P[:,0]<-0.12)&(P[:,0]>-0.32)&(P[:,1]>-0.08)&(P[:,1]<0.34)
Q=P[m]; print('door-top pts',len(Q))
# fit a line to the door top points (x as fn of y)
A=np.c_[Q[:,1],np.ones(len(Q))]; a,b=np.linalg.lstsq(A,Q[:,0],rcond=None)[0]
print('x = %.4f*y + %.4f'%(a,b))
res=Q[:,0]-(a*Q[:,1]+b); print('residual std %.4f, min %.4f max %.4f'%(res.std(),res.min(),res.max()))
print('y range %.3f..%.3f'%(Q[:,1].min(),Q[:,1].max()))
# thickness: perpendicular spread
n=np.array([1,-a])/np.hypot(1,a); d=Q[:,:2]@n; print('perp spread %.4f..%.4f'%(d.min()-d.mean(),d.max()-d.mean()))
# microwave body top near the door hinge
m2=(P[:,2]>1.09)&(P[:,2]<1.13)&(P[:,0]>-0.20)&(P[:,0]<0.10)&(P[:,1]>0.20)&(P[:,1]<0.50)
B=P[m2]; print('body top: x %.3f..%.3f y %.3f..%.3f'%(B[:,0].min(),B[:,0].max(),B[:,1].min(),B[:,1].max()))
# body front edge at various x
for x0 in [-0.16,-0.10,0.0,0.05]:
    k=np.abs(B[:,0]-x0)<0.005; print(' x',x0,'front y min %.3f'%B[k][:,1].min())
EOF

# openrua op 210
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3)
m=(P[:,2]>1.100)&(P[:,2]<1.115)&(P[:,0]<-0.10)&(P[:,0]>-0.32)&(P[:,1]>-0.08)&(P[:,1]<0.34)
Q=P[m]; print('pts at door-top height',len(Q))
xs=np.arange(-0.30,-0.10,0.01); ys=np.arange(0.33,-0.08,-0.01)
print('     '+' '.join(f'{x*100:3.0f}' for x in xs))
for y in ys:
    row=''
    for x in xs:
        k=(np.abs(Q[:,0]-x)<0.005)&(np.abs(Q[:,1]-y)<0.005)
        row+='  # ' if k.sum()>3 else '  . '
    print(f'{y*100:4.0f} '+row)
EOF

# openrua op 211
cat > door.py <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('door'); prev=np.array(r.arm_q())
H=np.array([-0.18,0.265]); RHO=0.16; OFF=0.022; ALPHA=np.deg2rad(35); ZT=1.045
def pose(th_deg, extra=0.0):
    th=np.deg2rad(th_deg); d=np.array([np.cos(th),np.sin(th),0.0]); n_out=np.array([np.sin(th),-np.cos(th),0.0])
    tcp=np.array([*(H+RHO*d[:2]+(OFF+extra)*n_out[:2]),ZT])
    z=-np.cos(ALPHA)*np.array([0,0,1.0])-np.sin(ALPHA)*n_out; x=np.cross(d,z)
    return tcp,hand_quat(z,x)
def go(tcp,q,secs,n=40):
    global prev
    s=best_ik(r,tcp,q,seed=prev,prefer=prev,n=n,avoid=lambda L: any(L[k][2]<1.0 for k in ('panda_link6','panda_link7','panda_hand')))
    if s is None: print('no ik',np.round(tcp,3)); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(w,2)); return w
mode=sys.argv[1]
if mode=='start':
    th0=float(sys.argv[2])
    tcp,q=pose(th0,0.06)
    # via point high above
    go(np.array([-0.15,0.05,1.30]),q,3.0)
    go(tcp,q,3.0)
    tcp,q=pose(th0,0.0); go(tcp,q,2.0)
elif mode=='sweep':
    th0=float(sys.argv[2]); th1=float(sys.argv[3]); step=float(sys.argv[4])
    wprev=r.wrench()[:3]
    for th in np.arange(th0+step,th1+1e-6,step):
        tcp,q=pose(th); w=go(tcp,q,1.2)
        dw=np.linalg.norm(w-wprev); wprev=w
        if dw>float(sys.argv[5]) if len(sys.argv)>5 else dw>8: print('FORCE JUMP at',th,dw); break
elif mode=='retreat':
    th=float(sys.argv[2]); tcp,q=pose(th,0.06); go(tcp,q,2.0); tcp[2]=1.25; go(tcp,q,2.5)
EOF
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
exec(open('door.py').read().split("mode=sys.argv[1]")[0].replace("r=Robot('door'); prev=np.array(r.arm_q())","r=Robot('doorchk'); prev=np.array(r.arm_q())"))
for th in [-119,-90,-60,-30,0]:
    tcp,q=pose(th); s=best_ik(r,tcp,q,seed=prev,prefer=prev,n=40)
    print(th,np.round(tcp,3),'ik ok' if s is not None else 'NO IK')
EOF

# openrua op 212
python3 door.py start -119

# openrua op 213
python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null; python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 geo.py cloud birdview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3)
m=(P[:,2]>1.100)&(P[:,2]<1.115)&(P[:,0]<-0.10)&(P[:,0]>-0.36)&(P[:,1]>-0.02)&(P[:,1]<0.30)
Q=P[m]
xs=np.arange(-0.34,-0.10,0.01); ys=np.arange(0.29,-0.02,-0.01)
print('     '+' '.join(f'{x*100:3.0f}' for x in xs))
for y in ys:
    row=''
    for x in xs:
        k=(np.abs(Q[:,0]-x)<0.005)&(np.abs(Q[:,1]-y)<0.005)
        row+='  # ' if k.sum()>3 else '  . '
    print(f'{y*100:4.0f} '+row)
EOF

# openrua op 214
python3 door.py sweep -119 0 4 8 2>&1 | tail -40

# openrua op 215
python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 geo.py cloud birdview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load('birdview_cloud.npy').reshape(-1,3)
m=(P[:,2]>1.100)&(P[:,2]<1.115)&(P[:,0]<0.12)&(P[:,0]>-0.36)&(P[:,1]>0.15)&(P[:,1]<0.32)
Q=P[m]
xs=np.arange(-0.30,0.12,0.01); ys=np.arange(0.31,0.15,-0.01)
print('     '+' '.join(f'{x*100:3.0f}' for x in xs))
for y in ys:
    row=''
    for x in xs:
        k=(np.abs(Q[:,0]-x)<0.005)&(np.abs(Q[:,1]-y)<0.005)
        row+='  # ' if k.sum()>3 else '  . '
    print(f'{y*100:4.0f} '+row)
# any door-height points elsewhere (open door remnants)?
m2=(P[:,2]>1.100)&(P[:,2]<1.115)&(P[:,0]<-0.19)&(P[:,1]<0.25)&(P[:,1]>-0.1)
print('door-height pts left of body, y<0.25:',m2.sum())
EOF

# openrua op 216
python3 gridmap.py birdview_cloud.npy -0.22 0.10 0.12 0.32 0.95 2>&1 | head -30

# openrua op 217
python3 door.py retreat -3 && python3 push.py to 0 -0.06 -0.12 1.06 && python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; python3 geo.py tf > /dev/null

# openrua op 218
python3 - <<'EOF'
import numpy as np, sys, time
sys.path.insert(0,'.')
from rob import *
r=Robot('chk')
q=np.array(r.arm_q()); print('q',np.round(q,3)); time.sleep(0.5); print('q again',np.round(r.arm_q(),3))
print('tcp',np.round(r.tcp()[0],4),np.round(r.tcp()[1],3)); print('wrench',np.round(r.wrench(),2))
L=links_fk(r,q); print({k:np.round(v,3) for k,v in L.items()})
EOF

# openrua op 219
python3 - <<'EOF'
import numpy as np, sys, time
sys.path.insert(0,'.')
from rob import *
r=Robot('chk')
q=np.array(r.arm_q()); print('q',np.round(q,3)); time.sleep(0.5); print('q again',np.round(r.arm_q(),3))
print('tcp',np.round(r.tcp()[0],4),np.round(r.tcp()[1],3)); print('wrench',np.round(r.wrench(),2))
L=links_fk(r,q); print({k:np.round(v,3) for k,v in L.items()})
EOF

# openrua op 220
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('mv'); prev=np.array(r.arm_q())
z3=np.array([0,1.0,0]); y3=np.array([1.0,0,0]); x3=np.cross(y3,z3); qh3=hand_quat(z3,x3)
# first a vertical hand at a neutral spot, then the look pose
qv=hand_quat(np.array([0,0,-1.0]),np.array([1.0,0,0]))
for pt,q in [((-0.15,-0.05,1.40),qv),((-0.06,-0.12,1.20),qh3),((-0.06,-0.12,1.06),qh3)]:
    s=best_ik(r,np.array(pt),q,seed=prev,prefer=prev,n=60)
    if s is None: print('no ik',pt); break
    print('target',pt,'joint dist',np.round(np.abs(s-prev).max(),2))
    r.move_joints(s,6.0); prev=np.array(r.arm_q()); print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2))
EOF

# openrua op 221
python3 tools/perception/cam_snap.py robot0_eye_in_hand robot0_eye_in_hand.png >/dev/null && python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw robot0_eye_in_hand_depth.png >/dev/null; python3 geo.py tf > /dev/null; echo ok

# openrua op 222
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'.')
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
X,Y,Z=P[...,0],P[...,1],P[...,2]
# front surfaces of the microwave: points with z in 0.95..1.10, x in -0.2..0.1, y<0.5
m=(Z>0.93)&(Z<1.10)&(X>-0.22)&(X<0.12)&(Y<0.6)&(Y>0.1)
Q=P[m]
for x0 in np.arange(-0.20,0.11,0.02):
    k=np.abs(Q[:,0]-x0)<0.005
    if k.sum(): 
        ys=Q[k][:,1]; print('x %+.2f  front y min %.3f  median %.3f  n %d'%(x0,ys.min(),np.median(ys),k.sum()))
# door vs hinge side: y of frontmost surface at z ~1.0 for hinge post (x~-0.19..-0.16) and door (x -0.14..0.03)
EOF

# openrua op 223
python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && python3 geo.py cloud birdview >/dev/null && python3 gridmap.py birdview_cloud.npy -0.30 0.12 0.20 0.34 0.95 2>&1 | head -12; python3 tools/perception/cam_snap.py birdview birdview.png >/dev/null

# openrua op 224
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-microwave-insights.md)"
cat > /claude-config/projects/-workspace/memory/panda-microwave-insights.md <<'OPENRUA_EOF'
---
name: panda-microwave-insights
description: Hard-won geometry facts for the Panda sim workspace (hand size vs microwave opening, camera/IK conventions, what worked for mug-in-microwave)
metadata:
  type: project
---

Learned while completing "put the white mug in the microwave and close it" (2026-09-13):

- rob.py `ik()` must set `ik_link_name="panda_hand"` (MoveIt default tip is panda_link8, rolled 45°). Already patched in /workspace/rob.py; if rob.py is regenerated, re-apply.
- The Franka hand body is ~0.20 m along the finger axis; it cannot pass through the microwave opening (0.203 x 0.137). Only the fingers (≈4.5 cm) reach inside: with a horizontal hand (z=+y) the TCP stops at y≈0.307 against the face plane (0.271). Plan cavity work as "insert as far as body allows, release, then push with fingertips".
- Pushing a mug lying on its side with closed fingertips at one rim point yaws it hard (it pivoted 50° in one push); a straddle push with open fingers only contacts if the rim passes within ~2 mm of the pad's inner face. Small steps + re-measure with eye-in-hand depth (`look.py`) each time.
- Door: hinge ≈(-0.18,0.265), length 0.25, closes with front face at y=0.249; arc push with 35°-tilted hand (door.py) from open angle to -3° worked; force jump ~8 N marks the closed stop.
- Paused-clock sim: the birdview/eye-in-hand only update after a command; always re-snap and refresh cams.json (`python3 geo.py tf`) after moving.

**Why:** These took many probes to discover and are not in the docs or code.
**How to apply:** Reuse push.py / door.py / look.py in /workspace; check the hand-body clearance before planning any insertion into an enclosure.
OPENRUA_EOF

# openrua op 225
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index)") && grep -q panda-microwave-insights MEMORY.md 2>/dev/null || echo "- [Panda microwave insights](panda-microwave-insights.md) — hand body 0.2 m can't enter cavity; IK link fix; door arc params; push tactics" >> MEMORY.md; cat MEMORY.md
