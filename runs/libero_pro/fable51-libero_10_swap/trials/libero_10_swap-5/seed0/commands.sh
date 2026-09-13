#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[130:300,180:420]; cv2.imwrite('bird_crop.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); crop=im[140:400,90:380]; cv2.imwrite('agent_crop.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png; timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agent_depth.png; timeout 20 ros2 topic echo /birdview/color/camera_info --once | head -30

# openrua op 7
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Build a world-frame point cloud (H,W,3) for a camera and save as <cam>_xyz.npy.
Usage: python3 cloud.py <cam>
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, typ, timeout=20.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    import time
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
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
    node = rclpy.create_node("cloud")
    buf = Buffer()
    TransformListener(buf, node)
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    import time
    t0 = time.time()
    while time.time() - t0 < 10 and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1) @ R.T + T
    np.save(f"{cam}_xyz.npy", pc)
    print("cam pos", T, "R", R.round(3).tolist())
    print("saved", f"{cam}_xyz.npy", pc.shape)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 90 python3 cloud.py birdview && timeout 90 python3 cloud.py agentview

# openrua op 9
python3 -c "
import numpy as np
pc=np.load('birdview_xyz.npy')
z=pc[...,2]
print('table z mode:', np.round(np.median(z[250:300,400:600]),3))
# region of caddy in birdview px: x 210-345, y 155-210
sub=pc[150:215,205:350]
zz=sub[...,2]
print('caddy z max', zz.max().round(3))
# print a coarse height map (every 3 px)
np.set_printoptions(linewidth=250)
hm=(zz[::3,::3]*100).round(0).astype(int)
for r in hm: print(' '.join(f'{v:3d}' for v in r))
"


# openrua op 10
python3 -c "
import numpy as np
np.set_printoptions(linewidth=250)
for cam in ['birdview','agentview']:
    pc=np.load(f'{cam}_xyz.npy').reshape(-1,3)
    pc=pc[np.isfinite(pc).all(1)]
    # grid over x in [-0.45,0.45], y in [-0.5,0.5] at 2cm
    res=0.02
    xs=np.arange(-0.45,0.45,res); ys=np.arange(-0.55,0.55,res)
    hm=np.full((len(xs),len(ys)),np.nan)
    ix=((pc[:,0]-xs[0])/res).astype(int); iy=((pc[:,1]-ys[0])/res).astype(int)
    ok=(ix>=0)&(ix<len(xs))&(iy>=0)&(iy<len(ys))
    for a,b,z in zip(ix[ok],iy[ok],pc[ok,2]):
        if np.isnan(hm[a,b]) or z>hm[a,b]: hm[a,b]=z
    print(cam, 'rows=x from',xs[0],'cols=y from',ys[0])
    print('     '+' '.join(f'{int(round(y*100)):3d}' for y in ys))
    for x,r in zip(xs,hm):
        print(f'{int(round(x*100)):4d} '+' '.join('  .' if np.isnan(v) else f'{int(round((v-0.88)*100)):3d}' for v in r))
"


# openrua op 11
python3 -c "
import numpy as np
pc=np.load('birdview_xyz.npy')
# profile along x at y≈-0.17 (middle section) and y≈-0.30 (left compartment)
for ytarget in [-0.30,-0.17,-0.05]:
    pts=pc[(np.abs(pc[...,1]-ytarget)<0.004)]
    pts=pts[np.argsort(pts[:,0])]
    sel=pts[(pts[:,0]>-0.52)&(pts[:,0]<-0.25)]
    print('y=',ytarget)
    print(' '.join(f'{x*100:.1f}:{(z-0.88)*100:.0f}' for x,y,z in sel[::2]))
print()
# profile along y at x≈-0.42 and x=-0.37
for xtarget in [-0.43,-0.38,-0.345]:
    pts=pc[(np.abs(pc[...,0]-xtarget)<0.004)]
    pts=pts[np.argsort(pts[:,1])]
    sel=pts[(pts[:,1]>-0.42)&(pts[:,1]<0.12)]
    print('x=',xtarget)
    print(' '.join(f'{y*100:.1f}:{(z-0.88)*100:.0f}' for x,y,z in sel[::2]))
"


# openrua op 12
python3 -c "
import numpy as np
pc=np.load('birdview_xyz.npy')
m=(pc[...,2]>0.95)&(pc[...,0]>-0.30)&(pc[...,0]<-0.10)&(pc[...,1]>0.10)&(pc[...,1]<0.30)
pts=pc[m]
print('n',len(pts),'zmax',pts[:,2].max(), 'z quantiles',np.quantile(pts[:,2],[0.1,0.5,0.9]).round(3))
xy=pts[:,:2]
c=xy.mean(0); print('centroid',c.round(4))
u,s,vt=np.linalg.svd(xy-c,full_matrices=False)
print('axes',vt.round(3),'sing',s.round(3))
proj=(xy-c)@vt.T
print('extent along major',proj[:,0].min().round(3),proj[:,0].max().round(3),'minor',proj[:,1].min().round(3),proj[:,1].max().round(3))
ang=np.degrees(np.arctan2(vt[0,1],vt[0,0])); print('major axis angle deg from +x',ang)
# top-surface points only (z>1.03)
top=pts[pts[:,2]>1.03]; print('top n',len(top),'top z',top[:,2].mean().round(4))
proj=(top[:,:2]-c)@vt.T
print('top extent major',proj[:,0].min().round(3),proj[:,0].max().round(3),'minor',proj[:,1].min().round(3),proj[:,1].max().round(3))
"


# openrua op 13
python3 -c "
import numpy as np
pc=np.load('birdview_xyz.npy')
m=(pc[...,2]>0.95)&(pc[...,0]>-0.22)&(pc[...,0]<-0.0)&(pc[...,1]>-0.08)&(pc[...,1]<0.08)
pts=pc[m]
print('n',len(pts),'zmax',pts[:,2].max().round(4), 'z quantiles',np.quantile(pts[:,2],[0.1,0.5,0.9]).round(3))
top=pts[pts[:,2]>1.06]
xy=top[:,:2]
c=xy.mean(0); print('top centroid',c.round(4),'top z mean',top[:,2].mean().round(4))
u,s,vt=np.linalg.svd(xy-c,full_matrices=False)
proj=(xy-c)@vt.T
print('major axis',vt[0].round(3),'angle deg from +x',np.degrees(np.arctan2(vt[0,1],vt[0,0])).round(1))
print('extent major',proj[:,0].min().round(3),proj[:,0].max().round(3),'minor',proj[:,1].min().round(3),proj[:,1].max().round(3))
print('x range',xy[:,0].min().round(3),xy[:,0].max().round(3),'y range',xy[:,1].min().round(3),xy[:,1].max().round(3))
"


# openrua op 14
timeout 15 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ----; timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -12; echo ----; timeout 10 ros2 topic echo /tf_static --once 2>&1 | head -60

# openrua op 15
timeout 15 ros2 service list | grep -iE "ik|fk|state|scene" 

# openrua op 16
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable arm controller: IK -> trajectory, FK check, gripper, joint state.
World<->base offset from TF (world -> panda_link0 = (-0.75, 0, 0.912)).
"""
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
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = M["hand"]["tcp_offset_m"]
BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])


def topdown_quat(theta_deg):
    """Hand z down; hand x axis at theta (deg) from world +x. Returns xyzw."""
    th = np.radians(theta_deg)
    xh = np.array([np.cos(th), np.sin(th), 0.0])
    zh = np.array([0.0, 0.0, -1.0])
    yh = np.cross(zh, xh)
    R = np.column_stack([xh, yh, zh])
    return Rot.from_matrix(R).as_quat()  # xyzw


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)

    def spin(self, fut, timeout):
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 20:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def _seed(self, q):
        js = JointState()
        js.name = list(JOINTS)
        js.position = [float(v) for v in q]
        return js

    def fk_hand(self, q=None):
        """Hand pose in WORLD: (xyz, quat xyzw)."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        res = self.spin(self.fk.call_async(req), 30)
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return xyz, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def tcp_world(self, q=None):
        xyz, quat = self.fk_hand(q)
        R = Rot.from_quat(quat).as_matrix()
        return xyz + TCP * R[:, 2], quat

    def solve_ik(self, tcp_world_xyz, quat_xyzw, seed=None, tries=5):
        """IK for a TCP target in world frame. Returns joint list or None."""
        R = Rot.from_quat(quat_xyzw).as_matrix()
        hand_world = np.asarray(tcp_world_xyz) - TCP * R[:, 2]
        hand_base = hand_world - BASE_IN_WORLD
        seed = self.arm_q() if seed is None else seed
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, hand_base)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_xyzw)
            s = np.array(seed, dtype=float)
            if k > 0:
                s = s + np.random.uniform(-0.3, 0.3, size=7)
                s = np.clip(s, [l[0] for l in LIMITS], [l[1] for l in LIMITS])
            req.ik_request.robot_state.joint_state = self._seed(s)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            res = self.spin(self.ik.call_async(req), 60)
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                q = [sol[j] for j in JOINTS]
                # sanity: FK back to the target
                got, _ = self.tcp_world(q)
                err = np.linalg.norm(got - np.asarray(tcp_world_xyz))
                if err < 0.005:
                    return q
                print(f"  ik try {k}: FK mismatch {err:.4f}")
            else:
                print(f"  ik try {k}: fail code={None if res is None else res.error_code.val}")
        return None

    def move_q(self, q, seconds=3.0, tol=0.02):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        gh = self.spin(self.fjt.send_goal_async(goal), 60)
        res = self.spin(gh.get_result_async(), 600)
        code = res.result.error_code if res else None
        now = np.array(self.arm_q())
        err = np.abs(now - np.array(q)).max()
        print(f"  move: code={code} max_joint_err={err:.4f}")
        return err < tol

    def move_tcp(self, xyz, quat, seconds=3.0, seed=None):
        q = self.solve_ik(xyz, quat, seed=seed)
        if q is None:
            print("  IK FAILED for", np.round(xyz, 3))
            return False
        ok = self.move_q(q, seconds)
        got, _ = self.tcp_world()
        print(f"  tcp now {np.round(got, 4)} target {np.round(xyz, 4)} err {np.linalg.norm(got - xyz):.4f}")
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        gh = self.spin(self.grip.send_goal_async(goal), 60)
        res = self.spin(gh.get_result_async(), 300)
        f = self.fingers()
        print(f"  gripper -> {width}: reached={res.result.reached_goal} stalled={res.result.stalled} fingers={np.round(f, 4)}")
        return f
OPENRUA_EOF

# openrua op 17
timeout 120 python3 -c "
from ctl import *
c=Ctl()
print('arm q', np.round(c.arm_q(),4))
print('fingers', c.fingers())
xyz,q=c.fk_hand(); print('hand world', xyz.round(4), q.round(4))
t,_=c.tcp_world(); print('tcp world', t.round(4))
print('topdown quat 0', topdown_quat(0).round(4), ' -11.4:', topdown_quat(-11.4).round(4))
"

# openrua op 18
timeout 120 python3 -c "
from ctl import *
c=Ctl()
q0=np.array(c.arm_q())
res=c.spin(c.fk.call_async((lambda r:(setattr(r,'fk_link_names',['panda_hand']), setattr(r.robot_state,'joint_state',c._seed(q0)), r)[2])(GetPositionFK.Request())),30)
print('fk frame', res.pose_stamped[0].header.frame_id, res.pose_stamped[0].pose.position)
hand=np.array([-0.293,0,1.27])
for name,pos in [('world',hand),('base',hand-BASE_IN_WORLD)]:
    req=GetPositionIK.Request(); req.ik_request.group_name='panda_arm'; req.ik_request.pose_stamped.header.frame_id=''
    p=req.ik_request.pose_stamped.pose; p.position.x,p.position.y,p.position.z=map(float,pos); p.orientation.x=1.0
    req.ik_request.robot_state.joint_state=c._seed(q0); req.ik_request.timeout.sec=2
    r=c.spin(c.ik.call_async(req),60)
    if r.error_code.val==1:
        sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position)); q=np.array([sol[j] for j in JOINTS])
        print(name,'ok, diff from current', np.abs(q-q0).max().round(4))
    else: print(name,'fail',r.error_code.val)
"

# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])", "new_string": "# Verified empirically: /compute_fk and /compute_ik on this machine speak\\n# WORLD coordinates (FK header frame_id == \\"world\\", values match world),\\n# so no base offset is applied.\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
BOOK=np.array([-0.1064,-0.0104]); TOP=1.072; TH=-11.4
q=topdown_quat(TH)
print('open gripper'); c.gripper(GRIP['open_m'])
print('move above book'); print(c.move_tcp(np.array([BOOK[0],BOOK[1],1.22]), q, 4.0))
print('q now', np.round(c.arm_q(),3))
" 2>&1 | tail -20

# openrua op 21
timeout 120 python3 -c "
from ctl import *
c=Ctl()
xyz,q=c.fk_hand(); print('hand', xyz.round(4), q.round(4)); print('target q', topdown_quat(-11.4).round(4))
R=Rot.from_quat(q).as_matrix(); print('hand x axis', R[:,0].round(3), 'hand z axis', R[:,2].round(3))
"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 22
timeout 300 python3 -u -c "
from ctl import *
c=Ctl()
xyz=np.array([-0.1064,-0.0104,1.22])
for th in [-11.4, 0, 45, 90]:
    qt=topdown_quat(th)
    for k in range(3):
        q=c.solve_ik(xyz,qt)
        if q is None: print(th,'none'); continue
        _,qq=c.fk_hand(q); R=Rot.from_quat(qq).as_matrix(); ang=np.degrees(np.arctan2(R[1,0],R[0,0]))
        print(f'theta {th}: got hand-x angle {ang:.1f}, q7={q[6]:.3f} q1={q[0]:.3f} q3={q[2]:.3f} q5={q[4]:.3f}')
" 2>&1 | tail -20

# openrua op 23
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "            req.ik_request.pose_stamped.header.frame_id = \\"\\"\\n            p = req.ik_request.pose_stamped.pose", "new_string": "            req.ik_request.pose_stamped.header.frame_id = \\"\\"\\n            # group tip is panda_link8 (45 deg off panda_hand); ask for the hand\\n            req.ik_request.ik_link_name = \\"panda_hand\\"\\n            p = req.ik_request.pose_stamped.pose", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 24
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "                got, _ = self.tcp_world(q)\\n                err = np.linalg.norm(got - np.asarray(tcp_world_xyz))\\n                if err < 0.005:\\n                    return q\\n                print(f\\"  ik try {k}: FK mismatch {err:.4f}\\")", "new_string": "                got, gq = self.tcp_world(q)\\n                err = np.linalg.norm(got - np.asarray(tcp_world_xyz))\\n                ang = (Rot.from_quat(gq) * Rot.from_quat(quat_xyzw).inv()).magnitude()\\n                if err < 0.005 and ang < 0.02:\\n                    return q\\n                print(f\\"  ik try {k}: FK mismatch pos {err:.4f} ang {np.degrees(ang):.1f}deg\\")", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 25
timeout 300 python3 -u -c "
from ctl import *
c=Ctl()
xyz=np.array([-0.1064,-0.0104,1.22])
for th in [-11.4, 90]:
    q=c.solve_ik(xyz,topdown_quat(th))
    if q is None: print(th,'none'); continue
    _,qq=c.fk_hand(q); R=Rot.from_quat(qq).as_matrix(); ang=np.degrees(np.arctan2(R[1,0],R[0,0]))
    print(f'theta {th}: got hand-x angle {ang:.1f}, q={np.round(q,3)}')
" 2>&1 | tail -20

# openrua op 26
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
BOOK=np.array([-0.1064,-0.0104]); TH=-11.4
print(c.move_tcp(np.array([BOOK[0],BOOK[1],1.22]), topdown_quat(TH), 3.0))
" 2>&1 | tail -5; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 27
timeout 120 python3 -u -c "
from ctl import *
c=Ctl()
q=np.array(c.arm_q()); print('q now', np.round(q,3))
t,qq=c.tcp_world(); R=Rot.from_quat(qq).as_matrix(); print('tcp', t.round(4), 'hand-x angle', np.degrees(np.arctan2(R[1,0],R[0,0])).round(1), 'z axis', R[:,2].round(3))
" 2>&1 | tail -3

# openrua op 28
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "        ok = self.move_q(q, seconds)\\n        got, _ = self.tcp_world()", "new_string": "        ok = self.move_q(q, seconds)\\n        for _ in range(2):\\n            if ok:\\n                break\\n            print(\\"  resending goal (controller lag)\\")\\n            ok = self.move_q(q, max(2.0, seconds / 2))\\n        got, _ = self.tcp_world()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 29
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
BOOK=np.array([-0.1064,-0.0104]); TH=-11.4
print(c.move_tcp(np.array([BOOK[0],BOOK[1],1.22]), topdown_quat(TH), 3.0))
t,qq=c.tcp_world(); R=Rot.from_quat(qq).as_matrix(); print('hand-x angle', np.degrees(np.arctan2(R[1,0],R[0,0])).round(1))
" 2>&1 | tail -8; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 30
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
BOOK=np.array([-0.1064,-0.0104]); TH=-11.4
print('descend'); print(c.move_tcp(np.array([BOOK[0],BOOK[1],1.047]), topdown_quat(TH), 3.0))
print('close'); f=c.gripper(GRIP['closed_m'])
" 2>&1 | tail -8; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 31
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
BOOK=np.array([-0.1064,-0.0104]); TH=-11.4
print('lift'); print(c.move_tcp(np.array([BOOK[0],BOOK[1],1.30]), topdown_quat(TH), 3.0))
print('fingers', c.fingers())
" 2>&1 | tail -5; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 32
timeout 90 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np
pc=np.load('birdview_xyz.npy')
m=(pc[...,2]>0.95)&(pc[...,2]<1.2)&(pc[...,0]>-0.25)&(pc[...,0]<0.0)&(pc[...,1]>-0.10)&(pc[...,1]<0.10)
pts=pc[m]
print('n',len(pts),'zmax',pts[:,2].max().round(4))
for lo,hi in [(1.06,1.2),(1.0,1.06),(0.95,1.0)]:
    s=pts[(pts[:,2]>lo)&(pts[:,2]<=hi)]
    if len(s)==0: print(lo,hi,'none'); continue
    xy=s[:,:2]; c=xy.mean(0); u,sv,vt=np.linalg.svd(xy-c,full_matrices=False); proj=(xy-c)@vt.T
    print(f'z in ({lo},{hi}] n={len(s)} centroid {c.round(4)} major dir {vt[0].round(3)} ang {np.degrees(np.arctan2(vt[0,1],vt[0,0])):.1f} ext major [{proj[:,0].min():.3f},{proj[:,0].max():.3f}] minor [{proj[:,1].min():.3f},{proj[:,1].max():.3f}]')
"

# openrua op 33
python3 -c "
import numpy as np
pc=np.load('birdview_xyz.npy')
m=(pc[...,2]>0.95)&(pc[...,2]<1.2)
pts=pc[m]
print('n',len(pts)); 
import collections
# cluster coarse by xy 5cm cells
cells=collections.Counter(zip(np.floor(pts[:,0]/0.05).astype(int),np.floor(pts[:,1]/0.05).astype(int)))
for k,v in sorted(cells.items()): print(k,v, 'zmax', pts[(np.floor(pts[:,0]/0.05)==k[0])&(np.floor(pts[:,1]/0.05)==k[1])][:,2].max().round(3))
"

# openrua op 34
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
print(c.move_tcp(np.array([-0.30,-0.28,1.30]), topdown_quat(0), 3.0))
" 2>&1 | tail -3; timeout 90 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np
pc=np.load('birdview_xyz.npy')
m=(pc[...,2]>0.95)&(pc[...,2]<1.2)&(pc[...,0]>-0.30)&(pc[...,0]<0.05)&(pc[...,1]>-0.12)&(pc[...,1]<0.12)
pts=pc[m]
print('n',len(pts),'zmax',pts[:,2].max().round(4))
for lo,hi in [(1.06,1.2),(1.0,1.06),(0.95,1.0)]:
    s=pts[(pts[:,2]>lo)&(pts[:,2]<=hi)]
    if len(s)==0: print(lo,hi,'none'); continue
    xy=s[:,:2]; c=xy.mean(0); u,sv,vt=np.linalg.svd(xy-c,full_matrices=False); proj=(xy-c)@vt.T
    print(f'z in ({lo},{hi}] n={len(s)} centroid {c.round(4)} major dir {vt[0].round(3)} ang {np.degrees(np.arctan2(vt[0,1],vt[0,0])):.1f} ext major [{proj[:,0].min():.3f},{proj[:,0].max():.3f}] minor [{proj[:,1].min():.3f},{proj[:,1].max():.3f}]')
"

# openrua op 35
python3 -c "
import numpy as np
pc=np.load('birdview_xyz.npy')
print('table z', np.median(pc[250:300,400:600,2]).round(4))
for ytarget in [-0.30,-0.17]:
    pts=pc[(np.abs(pc[...,1]-ytarget)<0.004)]
    pts=pts[np.argsort(pts[:,0])]
    sel=pts[(pts[:,0]>-0.50)&(pts[:,0]<-0.30)]
    print('y=',ytarget)
    print(' '.join(f'{x*100:.1f}:{(z-0.88)*100:.1f}' for x,y,z in sel[::2]))
for xtarget in [-0.43,-0.38]:
    pts=pc[(np.abs(pc[...,0]-xtarget)<0.004)]
    pts=pts[np.argsort(pts[:,1])]
    sel=pts[(pts[:,1]>-0.40)&(pts[:,1]<0.10)]
    print('x=',xtarget)
    print(' '.join(f'{y*100:.1f}:{(z-0.88)*100:.1f}' for x,y,z in sel[::2]))
"


# openrua op 36
timeout 60 python3 -c "
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<15: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0]); print(len(got[0]))
"; grep -n -A12 "panda_finger_joint1\|link name=\"panda_leftfinger\"\|link name=\"panda_hand\"" robot.urdf | head -80

# openrua op 37
timeout 900 python3 -u -c "
from ctl import *
c=Ctl()
BOOK=np.array([-0.1053,-0.010]); TH=-10.4; TOP=1.021
qg=topdown_quat(TH)
print('open'); c.gripper(GRIP['open_m'])
print('above'); print(c.move_tcp(np.array([BOOK[0],BOOK[1],1.15]), qg, 4.0))
print('descend'); print(c.move_tcp(np.array([BOOK[0],BOOK[1],TOP-0.02]), qg, 3.0))
print('close'); c.gripper(GRIP['closed_m'])
" 2>&1 | grep -v '^\s*$' | tail -12; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 38
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
BOOK=np.array([-0.1053,-0.010]); TH=-10.4
print('lift'); print(c.move_tcp(np.array([BOOK[0],BOOK[1],1.30]), topdown_quat(TH), 3.0))
print('fingers', np.round(c.fingers(),4))
" 2>&1 | grep -v '^\s*$' | tail -5; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 39
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
SLOT=np.array([-0.437,-0.1485])
print('to slot'); print(c.move_tcp(np.array([SLOT[0],SLOT[1],1.30]), topdown_quat(90), 5.0))
t,qq=c.tcp_world(); R=Rot.from_quat(qq).as_matrix(); print('hand-x angle', np.degrees(np.arctan2(R[1,0],R[0,0])).round(1))
print('fingers', np.round(c.fingers(),4))
" 2>&1 | grep -v '^\s*$' | tail -6; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 40
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
SLOT=np.array([-0.437,-0.1485])
q=c.solve_ik(np.array([SLOT[0],SLOT[1],1.30]), topdown_quat(90))
print('q target', np.round(q,3), 'now', np.round(c.arm_q(),3))
for i in range(4):
    if c.move_q(q, 3.0): break
t,qq=c.tcp_world(); R=Rot.from_quat(qq).as_matrix(); print('tcp',t.round(4),'hand-x angle', np.degrees(np.arctan2(R[1,0],R[0,0])).round(1))
print('fingers', np.round(c.fingers(),4))
" 2>&1 | grep -v '^\s*$' | tail -8; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 41
timeout 600 python3 -u -c "
from ctl import *
from geometry_msgs.msg import WrenchStamped
c=Ctl()
w={}
c.node.create_subscription(WrenchStamped,'/franka_robot_state_broadcaster/external_wrench',lambda m: w.__setitem__('m',m),1)
def wrench():
    w.pop('m',None); import time; t=time.time()
    while 'm' not in w and time.time()-t<10: rclpy.spin_once(c.node,timeout_sec=0.2)
    f=w['m'].wrench.force; return np.round([f.x,f.y,f.z],2)
SLOT=np.array([-0.437,-0.1485])
print('wrench before', wrench())
for z in [1.15, 1.10, 1.075]:
    q=c.solve_ik(np.array([SLOT[0],SLOT[1],z]), topdown_quat(90))
    for i in range(4):
        if c.move_q(q, 2.5): break
    t,qq=c.tcp_world(); print('z',z,'tcp',t.round(4),'fingers',np.round(c.fingers(),4),'wrench',wrench())
" 2>&1 | grep -v '^\s*$' | tail -12; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 42
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
SLOT=np.array([-0.437,-0.1485])
print('release'); c.gripper(GRIP['open_m'])
q=c.solve_ik(np.array([SLOT[0],SLOT[1],1.30]), topdown_quat(90))
for i in range(4):
    if c.move_q(q, 3.0): break
print('tcp', c.tcp_world()[0].round(4))
" 2>&1 | grep -v '^\s*$' | tail -6; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 43
timeout 600 python3 -u -c "
from ctl import *
c=Ctl()
q=c.solve_ik(np.array([-0.15,0.30,1.35]), topdown_quat(0))
for i in range(4):
    if c.move_q(q, 4.0): break
print('tcp', c.tcp_world()[0].round(4), 'fingers', np.round(c.fingers(),4))
" 2>&1 | grep -v '^\s*$' | tail -3; timeout 90 python3 cloud.py birdview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview; python3 -c "
import numpy as np
pc=np.load('birdview_xyz.npy')
# book search: anything above the caddy floor inside the middle-back slot region and around
m=(pc[...,2]>0.93)&(pc[...,0]>-0.50)&(pc[...,0]<-0.30)&(pc[...,1]>-0.40)&(pc[...,1]<0.10)
pts=pc[m]
# points higher than the 11cm divider but not walls: look at z>1.0 in the middle section y range
s=pts[(pts[:,1]>-0.20)&(pts[:,1]<-0.09)&(pts[:,0]>-0.46)&(pts[:,0]<-0.34)]
print('middle-section pts above 0.93:',len(s),'zmax',s[:,2].max().round(4))
top=s[s[:,2]>s[:,2].max()-0.012]
print('book top: n',len(top),'z mean',top[:,2].mean().round(4),'x range',top[:,0].min().round(3),top[:,0].max().round(3),'y range',top[:,1].min().round(3),top[:,1].max().round(3))
# is the old book spot empty?
old=pc[(pc[...,2]>0.95)&(pc[...,0]>-0.20)&(pc[...,0]<0.0)&(pc[...,1]>-0.08)&(pc[...,1]<0.06)]
print('points above table at old book spot:',len(old))
"

# openrua op 44
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[130:300,180:420]; cv2.imwrite('bird_crop2.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))"
