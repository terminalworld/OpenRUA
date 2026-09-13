#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 node list

# openrua op 4
timeout 20 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; timeout 30 ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -8; timeout 20 ros2 topic echo /agentview/color/camera_info --once | head -20

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+info from a camera, save world-frame point cloud + image.

Usage: python3 scene.py <camera>
Writes <camera>_pts.npy (H,W,3 world xyz, nan where invalid) and <camera>.png.
"""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


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


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene_" + cam)
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    color = grab(node, f"/{cam}/color/image_raw", Image)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    img = br.imgmsg_to_cv2(color, "bgr8")
    d = br.imgmsg_to_cv2(depth, "passthrough").astype(np.float64)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * d / fx
    Y = (v - cy) * d / fy
    pc = np.stack([X, Y, d], -1)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    pw = pc @ R.T + T
    bad = ~np.isfinite(d) | (d <= 0)
    pw[bad] = np.nan
    np.save(f"{cam}_pts.npy", pw)
    cv2.imwrite(f"{cam}.png", img)
    print(cam, "cam pos", T, "saved")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 scene.py birdview && timeout 120 python3 scene.py agentview && python3 -c "
import numpy as np
p=np.load('birdview_pts.npy')
z=p[...,2]
print('z range', np.nanmin(z), np.nanmax(z))
# table height: most common z
h,e=np.histogram(z[np.isfinite(z)],bins=100); print(list(zip(e[:-1].round(3),h))[:] )
"

# openrua op 8
python3 -c "
import numpy as np, cv2
p=np.load('birdview_pts.npy'); img=cv2.imread('birdview.png')
z=p[...,2]
mask=(np.isfinite(z)&(z>0.428)&(z<0.7)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    xyz=p[m]; col=img[m].mean(0)
    print(i,'px',cent[i].round(0),'area',stats[i,4],'x %.3f..%.3f y %.3f..%.3f zmax %.3f'%(np.nanmin(xyz[:,0]),np.nanmax(xyz[:,0]),np.nanmin(xyz[:,1]),np.nanmax(xyz[:,1]),np.nanmax(xyz[:,2])),'bgr',col.round(0))
"


# openrua op 9
cd /workspace/.ros/log && grep -il "pudding\|plate\|region\|bddl" * ; grep -ih "pudding\|region\|goal" * | head -30

# openrua op 10
cd /workspace && python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); img=cv2.imread('agentview.png')
z=p[...,2]
mask=(np.isfinite(z)&(z>0.432)&(z<0.7)&(p[...,0]>-0.4)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    xyz=p[m]; col=img[m].mean(0)
    print(i,'px',cent[i].round(0),'area',stats[i,4],'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(np.nanmin(xyz[:,0]),np.nanmax(xyz[:,0]),np.nanmin(xyz[:,1]),np.nanmax(xyz[:,1]),np.nanmin(xyz[:,2]),np.nanmax(xyz[:,2])),'bgr',col.round(0))
# plate: from agentview, points with z between 0.42 and 0.44 near x 0.14
m=(np.isfinite(z)&(z>0.421)&(z<0.46)&(p[...,0]>0.0)&(p[...,0]<0.3)&(abs(p[...,1])<0.15))
xyz=p[m]; print('plate-ish', xyz.shape, xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(), np.percentile(xyz[:,2],[5,50,95]))
"


# openrua op 11
timeout 30 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A3 "Translation" | head -4

# openrua op 12
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854 2.0 && timeout 120 python3 scene.py birdview && timeout 120 python3 scene.py agentview && python3 -c "
import numpy as np, cv2
for cam in ['birdview','agentview']:
    p=np.load(cam+'_pts.npy'); img=cv2.imread(cam+'.png')
    z=p[...,2]
    tbl=z[np.isfinite(z)&(p[...,0]>0.25)&(p[...,0]<0.4)&(abs(p[...,1])<0.1)]
    print(cam,'table z', np.nanmedian(tbl))
    mask=(np.isfinite(z)&(z>0.432)&(z<0.7)&(p[...,0]>-0.4)).astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
    for i in range(1,n):
        if stats[i,4]<30: continue
        m=lab==i
        xyz=p[m]; col=img[m].mean(0)
        print(i,'px',cent[i].round(0),'area',stats[i,4],'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(np.nanmin(xyz[:,0]),np.nanmax(xyz[:,0]),np.nanmin(xyz[:,1]),np.nanmax(xyz[:,1]),np.nanmin(xyz[:,2]),np.nanmax(xyz[:,2])),'bgr',col.round(0))
"

# openrua op 13
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable arm helpers: joint state, IK (world-frame pose), trajectory, gripper.

World -> planning frame (panda_link0) offset comes from TF at import time.
"""
import math
import sys
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
from tf2_ros import Buffer, TransformListener

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # panda_link0 in world (TF)

# quaternion: hand pointing down, fingers open along world y
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)
# hand pointing down, fingers open along world x (rotated 90 deg about z)
Q_DOWN_X = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self, name="arm_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(timeout_sec=20)
        self.grip.wait_for_server(timeout_sec=20)
        self.ik.wait_for_service(timeout_sec=20)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    def spin(self, t=0.2):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self._js = {}
        while not self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in JOINTS]

    def finger_gap(self):
        js = self.joints()
        return js["panda_finger_joint1"], js["panda_finger_joint2"]

    def hand_pose_world(self):
        """TF panda_link0->panda_hand, shifted into world."""
        self.spin(0.3)
        t = self.tfbuf.lookup_transform("panda_link0", "panda_hand", rclpy.time.Time())
        tr = t.transform.translation
        q = t.transform.rotation
        p = np.array([tr.x, tr.y, tr.z]) + BASE_IN_WORLD
        return p, (q.x, q.y, q.z, q.w)

    def tcp_world(self):
        p, q = self.hand_pose_world()
        R = quat_R(*q)
        return p + TCP * R[:, 2], q

    def ik_world(self, xyz_tcp, quat, seed=None, at_tcp=True, tries=3):
        """IK for a world-frame TCP pose. Returns joint list or None."""
        xyz = np.array(xyz_tcp, dtype=float)
        if at_tcp:
            R = quat_R(*quat)
            xyz = xyz - TCP * R[:, 2]
        xyz_base = xyz - BASE_IN_WORLD
        seed = seed if seed is not None else self.arm_q()
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, xyz_base)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            s = JointState()
            s.name = list(JOINTS)
            s.position = [float(v) for v in seed]
            req.ik_request.robot_state.joint_state = s
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            # perturb seed and retry
            seed = [v + np.random.uniform(-0.3, 0.3) for v in seed]
        return None

    def move_joints(self, q, secs=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                t = secs * (i + 1) / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=120)
        gh = send.result()
        if gh is None or not gh.accepted:
            print("FJT goal rejected", flush=True)
            return None
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=900)
        if res.result() is None:
            print("FJT no result (timeout)", flush=True)
            return None
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"FJT code={code} max_joint_err={err:.4f}", flush=True)
        return code

    def move_tcp(self, xyz, quat, secs=3.0, seed=None):
        q = self.ik_world(xyz, quat, seed=seed)
        if q is None:
            print(f"IK FAILED for {xyz}", flush=True)
            return None
        code = self.move_joints(q, secs)
        p, _ = self.tcp_world()
        print(f"tcp now {p.round(4)} target {np.array(xyz).round(4)}", flush=True)
        return q

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result if res.result() else None
        g = self.finger_gap()
        print(f"gripper -> {width}: reached={getattr(r,'reached_goal',None)} "
              f"stalled={getattr(r,'stalled',None)} fingers={g}", flush=True)
        return g

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])
OPENRUA_EOF

# openrua op 14
timeout 300 python3 -u -c "
from arm import *
a=Arm('test')
p,q=a.tcp_world(); print('tcp world',p.round(4),np.round(q,3))
print('fingers',a.finger_gap())
seed=a.arm_q(); print('q',np.round(seed,3))
targets={
 'above_mug':((-0.110,-0.199,0.65),Q_DOWN_Y),
 'grasp_mug':((-0.110,-0.199,0.525),Q_DOWN_Y),
 'above_plate':((0.145,0.002,0.65),Q_DOWN_Y),
 'place_mug':((0.145,0.002,0.55),Q_DOWN_Y),
 'above_pud':((-0.181,-0.024,0.60),Q_DOWN_Y),
 'grasp_pud':((-0.181,-0.024,0.44),Q_DOWN_Y),
 'above_right':((0.145,0.19,0.60),Q_DOWN_Y),
 'place_pud':((0.145,0.19,0.45),Q_DOWN_Y),
}
for k,(xyz,qq) in targets.items():
    sol=a.ik_world(xyz,qq,seed=seed)
    print(k, None if sol is None else np.round(sol,3))
"


# openrua op 15
timeout 300 python3 -u -c "
from arm import *
a=Arm('test2')
p,q=a.hand_pose_world(); print('hand world',p.round(4),np.round(q,3))
seed=a.arm_q()
req = GetPositionIK.Request()
req.ik_request.group_name = 'panda_arm'
req.ik_request.pose_stamped.header.frame_id = ''
pp = req.ik_request.pose_stamped.pose
xyz_base=p-BASE_IN_WORLD
pp.position.x, pp.position.y, pp.position.z = map(float, xyz_base)
pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q)
s = JointState(); s.name=list(JOINTS); s.position=[float(v) for v in seed]
req.ik_request.robot_state.joint_state = s
for variant in ['plain','ik_link','timeout']:
    if variant=='ik_link': req.ik_request.ik_link_name='panda_hand'
    if variant=='timeout': req.ik_request.timeout.sec=1
    fut=a.ik.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=60)
    r=fut.result(); print(variant, r.error_code.val if r else None, np.round(r.solution.joint_state.position,3) if r else '')
"


# openrua op 16
timeout 300 python3 -u -c "
from arm import *
a=Arm('test3')
seed=a.arm_q()
a.fk.wait_for_service(timeout_sec=20)
req=GetPositionFK.Request()
req.header.frame_id=''
req.fk_link_names=['panda_hand','panda_link8','panda_link0']
s=JointState(); s.name=list(JOINTS); s.position=[float(v) for v in seed]
req.robot_state.joint_state=s
fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=60)
r=fut.result(); print('code',r.error_code.val)
for n,ps in zip(r.fk_link_names,r.pose_stamped):
    p=ps.pose.position; q=ps.pose.orientation
    print(n, ps.header.frame_id, round(p.x,4),round(p.y,4),round(p.z,4),'|',round(q.x,3),round(q.y,3),round(q.z,3),round(q.w,3))
"


# openrua op 17
sed -i 's/        xyz_base = xyz - BASE_IN_WORLD/        xyz_base = xyz  # planner model frame == world on this machine (verified via FK)/' arm.py && timeout 300 python3 -u -c "
from arm import *
a=Arm('test4')
seed=a.arm_q()
targets={
 'current_hand':((-0.053,0,0.7776),(1,0,-0.028,0)),
 'above_mug':((-0.110,-0.199,0.65),Q_DOWN_Y),
 'grasp_mug':((-0.110,-0.199,0.525),Q_DOWN_Y),
 'above_plate':((0.145,0.002,0.65),Q_DOWN_Y),
 'place_mug':((0.145,0.002,0.55),Q_DOWN_Y),
 'above_pud':((-0.181,-0.024,0.60),Q_DOWN_Y),
 'grasp_pud':((-0.181,-0.024,0.44),Q_DOWN_Y),
 'above_right':((0.145,0.19,0.60),Q_DOWN_Y),
 'place_pud':((0.145,0.19,0.45),Q_DOWN_Y),
}
for k,(xyz,qq) in targets.items():
    sol=a.ik_world(xyz,qq,seed=seed,at_tcp=(k!='current_hand'))
    print(k, None if sol is None else np.round(sol,3))
"


# openrua op 18
cat >> arm.py <<'EOF'

    def move_line(self, xyz_to, quat, secs=3.0, steps=4, seed=None):
        """Straight-ish TCP line: IK at intermediate points, one trajectory."""
        p0, _ = self.tcp_world()
        p1 = np.array(xyz_to, dtype=float)
        seed = seed if seed is not None else self.arm_q()
        via = []
        for i in range(1, steps + 1):
            pt = p0 + (p1 - p0) * i / steps
            q = self.ik_world(pt, quat, seed=seed)
            if q is None:
                print(f"IK FAILED on line at {pt}", flush=True)
                return None
            via.append(q)
            seed = q
        code = self.move_joints(via[-1], secs, via=via[:-1])
        p, _ = self.tcp_world()
        print(f"tcp now {p.round(4)} target {p1.round(4)}", flush=True)
        return via[-1]
EOF
cat > stage1.py <<'EOF'
from arm import *
a = Arm('stage1')
print('start tcp', a.tcp_world()[0].round(4), 'fingers', a.finger_gap(), flush=True)
MUG = (-0.110, -0.199)          # TCP over the mug's -y wall (opposite the handle)
PLATE = (0.145, 0.002)
# 1. above the mug wall
assert a.move_tcp((MUG[0], MUG[1], 0.66), Q_DOWN_Y, secs=4) is not None
# 2. descend straight to grasp height (rim at 0.55)
assert a.move_line((MUG[0], MUG[1], 0.52), Q_DOWN_Y, secs=3) is not None
# 3. close
g = a.close()
# 4. lift
assert a.move_line((MUG[0], MUG[1], 0.68), Q_DOWN_Y, secs=3) is not None
print('after lift fingers', a.finger_gap(), flush=True)
EOF
nohup python3 -u stage1.py > stage1.log 2>&1 &
echo started

# openrua op 19
sleep 60; cat /workspace/stage1.log

# openrua op 20
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 21
cat > stage2.py <<'EOF'
from arm import *
a = Arm('stage2')
print('start tcp', a.tcp_world()[0].round(4), 'fingers', a.finger_gap(), flush=True)
PLATE = (0.145, 0.002)
# 1. carry above the plate (mug bottom ~0.095 below TCP; keep it high)
assert a.move_tcp((PLATE[0], PLATE[1], 0.68), Q_DOWN_Y, secs=5) is not None
print('fingers', a.finger_gap(), flush=True)
# 2. lower straight until the mug bottom is just above the plate (~0.44)
assert a.move_line((PLATE[0], PLATE[1], 0.545), Q_DOWN_Y, secs=3) is not None
# 3. release
a.open()
# 4. retreat straight up
assert a.move_line((PLATE[0], PLATE[1], 0.70), Q_DOWN_Y, secs=3) is not None
print('done', flush=True)
EOF
nohup python3 -u stage2.py > stage2.log 2>&1 &
sleep 75; cat stage2.log

# openrua op 22
timeout 120 python3 scene.py agentview && timeout 120 python3 scene.py birdview && python3 -c "
import numpy as np, cv2
for cam in ['birdview','agentview']:
    p=np.load(cam+'_pts.npy'); img=cv2.imread(cam+'.png')
    z=p[...,2]
    mask=(np.isfinite(z)&(z>0.432)&(z<0.7)&(p[...,0]>-0.4)).astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
    for i in range(1,n):
        if stats[i,4]<30: continue
        m=lab==i
        xyz=p[m]; col=img[m].mean(0)
        print(cam,i,'px',cent[i].round(0),'area',stats[i,4],'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(np.nanmin(xyz[:,0]),np.nanmax(xyz[:,0]),np.nanmin(xyz[:,1]),np.nanmax(xyz[:,1]),np.nanmin(xyz[:,2]),np.nanmax(xyz[:,2])),'bgr',col.round(0))
"

# openrua op 23
python3 -c "
import cv2
img=cv2.imread('birdview.png'); crop=img[230:380,230:420]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
img=cv2.imread('agentview.png'); crop=img[270:460,230:440]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2,fy=2))
"

# openrua op 24
python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); img=cv2.imread('agentview.png')
z=p[...,2]
m=np.isfinite(z)&(p[...,0]>0.03)&(p[...,0]<0.26)&(p[...,1]>-0.12)&(p[...,1]<0.16)&(z>0.46)
xyz=p[m]; col=img[m]
bright=col.mean(1)>90
xyz=xyz[bright]
print('mug pts',len(xyz),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].min(),xyz[:,2].max()))
# top rim: highest 1.5cm of points
top=xyz[xyz[:,2]>xyz[:,2].max()-0.02]
print('rim pts',len(top),'center',top.mean(0).round(3),'x %.3f..%.3f y %.3f..%.3f'%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max()))
# slices by z
for zl in np.arange(0.46,0.58,0.02):
    s=xyz[(xyz[:,2]>=zl)&(xyz[:,2]<zl+0.02)]
    if len(s): print('z %.2f n %4d x %.3f..%.3f y %.3f..%.3f'%(zl,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
"

# openrua op 25
cat > stage3.py <<'EOF'
from arm import *
a = Arm('stage3')
print('start tcp', a.tcp_world()[0].round(4), 'fingers', a.finger_gap(), flush=True)
G = (0.137, -0.022)   # -y wall of the tilted mug, at fingertip height 0.52
assert a.move_tcp((G[0], G[1], 0.68), Q_DOWN_Y, secs=4) is not None
assert a.move_line((G[0], G[1], 0.52), Q_DOWN_Y, secs=3) is not None
a.close()
assert a.move_line((G[0], G[1], 0.72), Q_DOWN_Y, secs=3) is not None
print('after lift fingers', a.finger_gap(), flush=True)
# level the mug: hand = R_x(165 deg)
Q_LEVEL = (math.sin(math.radians(82.5)), 0.0, 0.0, math.cos(math.radians(82.5)))
assert a.move_tcp((G[0], G[1], 0.72), Q_LEVEL, secs=3) is not None
print('leveled; fingers', a.finger_gap(), 'tcp', a.tcp_world()[0].round(4), flush=True)
EOF
nohup python3 -u stage3.py > stage3.log 2>&1 &
sleep 90; cat stage3.log

# openrua op 26
timeout 120 python3 scene.py agentview && python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); img=cv2.imread('agentview.png')
z=p[...,2]
# mug in the air: z between 0.56 and 0.72 near x 0.14
m=np.isfinite(z)&(p[...,0]>0.03)&(p[...,0]<0.26)&(p[...,1]>-0.15)&(p[...,1]<0.15)&(z>0.50)&(z<0.715)
xyz=p[m]; col=img[m]
bright=col.mean(1)>90; xyz=xyz[bright]
print('pts',len(xyz),'z %.3f..%.3f'%(xyz[:,2].min(),xyz[:,2].max()))
for zl in np.arange(0.56,0.72,0.02):
    s=xyz[(xyz[:,2]>=zl)&(xyz[:,2]<zl+0.02)]
    if len(s): print('z %.2f n %4d x %.3f..%.3f y %.3f..%.3f'%(zl,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
"

# openrua op 27
python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); img=cv2.imread('agentview.png')
z=p[...,2]
m=np.isfinite(z)&(p[...,0]>0.0)&(p[...,0]<0.30)&(p[...,1]>-0.15)&(p[...,1]<0.15)&(z>0.50)&(z<0.75)
xyz=p[m]; col=img[m]
print('pts',len(xyz),'z %.3f..%.3f'%(xyz[:,2].min(),xyz[:,2].max()))
for zl in np.arange(0.50,0.75,0.02):
    s=xyz[(xyz[:,2]>=zl)&(xyz[:,2]<zl+0.02)]
    if len(s): print('z %.2f n %4d x %.3f..%.3f y %.3f..%.3f  ymid %.3f'%(zl,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max(),(s[:,1].min()+s[:,1].max())/2))
"

# openrua op 28
cat > stage4.py <<'EOF'
from arm import *
a = Arm('stage4')
p, q = a.tcp_world()
print('start tcp', p.round(4), np.round(q,3), 'fingers', a.finger_gap(), flush=True)
# set the mug down on the free table spot near its original location
T = (-0.096, -0.196)
assert a.move_tcp((T[0], T[1], 0.72), q, secs=5) is not None
assert a.move_line((T[0], T[1], 0.55), q, secs=3) is not None
a.open()
assert a.move_line((T[0], T[1], 0.72), Q_DOWN_Y, secs=3) is not None
print('done', flush=True)
EOF
nohup python3 -u stage4.py > stage4.log 2>&1 &
sleep 70; cat stage4.log

# openrua op 29
timeout 200 python3 -u -c "
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm('t5')
seed=a.arm_q()
req=GetPositionFK.Request(); req.header.frame_id=''
req.fk_link_names=['panda_hand','panda_link8']
s=JointState(); s.name=list(JOINTS); s.position=[float(v) for v in seed]; req.robot_state.joint_state=s
fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=60); r=fut.result()
qs={n:[ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w] for n,ps in zip(r.fk_link_names,r.pose_stamped)}
print(qs)
Rh=Rot.from_quat(qs['panda_hand']); R8=Rot.from_quat(qs['panda_link8'])
rel=R8.inv()*Rh   # hand = link8 * rel
print('link8->hand rel rotvec deg', np.degrees(rel.as_rotvec()).round(2))
# test IK with ik_link_name=panda_hand at current pose
p=a.hand_pose_world()[0]
req2=GetPositionIK.Request(); req2.ik_request.group_name='panda_arm'; req2.ik_request.ik_link_name='panda_hand'
pp=req2.ik_request.pose_stamped.pose; pp.position.x,pp.position.y,pp.position.z=map(float,p)
q=qs['panda_hand']; pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=map(float,q)
req2.ik_request.robot_state.joint_state=s
fut=a.ik.call_async(req2); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=60); r=fut.result()
print('ik with link name panda_hand:', r.error_code.val, np.round(r.solution.joint_state.position,3), 'seed', np.round(seed,3))
"


# openrua op 30
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "            req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "            req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n            # the group\'s default tip is panda_link8 (45 deg off panda_hand\\n            # about z) -> solve for the hand link explicitly\\n            req.ik_request.ik_link_name = \\"panda_hand\\"\\n            req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 31
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-moveit-ik-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-moveit-ik-quirks.md <<'OPENRUA_EOF'
---
name: panda-moveit-ik-quirks
description: Verified MoveIt IK facts on the Franka Panda ROS 2 workstation (/workspace) that contradict the generic docs
metadata:
  type: project
---

On the Panda workstation in /workspace (verified 2026-09-12 via /compute_fk):
- The planner's model frame is **world**, not panda_link0 (panda_link0 sits at world (-0.51, 0, 0.42)). Send IK poses in world coords with empty frame_id; docs/30-action.md "planning facts" claim base frame, which is wrong here.
- `/compute_ik` for group `panda_arm` solves for **panda_link8** by default, which is rotated 45° about z from `panda_hand`. Set `ik_request.ik_link_name = "panda_hand"` (works) or grasps end up with fingers 45° off axis.
- Objects float ~6 cm above supports at session start; run one short trajectory to let physics settle before measuring.

**Why:** cost a failed mug placement before it was noticed.
**How to apply:** use /workspace/arm.py (Arm.ik_world) which already applies both fixes; re-verify with FK if the machine changes.
OPENRUA_EOF

# openrua op 32
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-moveit-ik-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-moveit-ik-quirks.md <<'OPENRUA_EOF'
---
name: panda-moveit-ik-quirks
description: Verified MoveIt IK facts on the Franka Panda ROS 2 workstation (/workspace) that contradict the generic docs
metadata:
  type: project
---

On the Panda workstation in /workspace (verified 2026-09-12 via /compute_fk):
- The planner's model frame is **world**, not panda_link0 (panda_link0 sits at world (-0.51, 0, 0.42)). Send IK poses in world coords with empty frame_id; docs/30-action.md "planning facts" claim base frame, which is wrong here.
- `/compute_ik` for group `panda_arm` solves for **panda_link8** by default, which is rotated 45° about z from `panda_hand`. Set `ik_request.ik_link_name = "panda_hand"` (works) or grasps end up with fingers 45° off axis.
- Objects float ~6 cm above supports at session start; run one short trajectory to let physics settle before measuring.
- Camera convention: agentview/frontview/eye-in-hand all have image-right = world +y.

**Why:** cost a failed mug placement before it was noticed.
**How to apply:** use /workspace/arm.py (Arm.ik_world) which already applies both fixes; re-verify with FK if the machine changes.
OPENRUA_EOF

# openrua op 33
ls /claude-config/projects/-workspace/memory/; cat /claude-config/projects/-workspace/memory/MEMORY.md 2>/dev/null || echo "(no MEMORY.md)"

# openrua op 34
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda MoveIt IK quirks](panda-moveit-ik-quirks.md) — planner frame is world, IK tip is link8 (use ik_link_name=panda_hand), objects float until first motion
OPENRUA_EOF

# openrua op 35
timeout 120 python3 scene.py agentview && python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); img=cv2.imread('agentview.png')
z=p[...,2]
mask=(np.isfinite(z)&(z>0.432)&(z<0.62)&(p[...,0]>-0.4)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    xyz=p[m]; col=img[m].mean(0)
    print(i,'px',cent[i].round(0),'area',stats[i,4],'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(np.nanmin(xyz[:,0]),np.nanmax(xyz[:,0]),np.nanmin(xyz[:,1]),np.nanmax(xyz[:,1]),np.nanmin(xyz[:,2]),np.nanmax(xyz[:,2])),'bgr',col.round(0))
# white mug slices
m=np.isfinite(z)&(p[...,0]>-0.25)&(p[...,0]<0.0)&(p[...,1]>-0.30)&(p[...,1]<-0.08)&(z>0.44)&(z<0.62)
xyz=p[m]
for zl in np.arange(0.44,0.58,0.02):
    s=xyz[(xyz[:,2]>=zl)&(xyz[:,2]<zl+0.02)]
    if len(s): print('z %.2f n %4d x %.3f..%.3f y %.3f..%.3f'%(zl,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
"

# openrua op 36
python3 -c "
import cv2
img=cv2.imread('agentview.png'); crop=img[150:330,80:300]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2,fy=2))
"

# openrua op 37
cat > stage5.py <<'EOF'
from arm import *
a = Arm('stage5')
print('start tcp', a.tcp_world()[0].round(4), 'fingers', a.finger_gap(), flush=True)
G = (-0.129, -0.261)          # -y wall of the upright mug
PLATE = (0.138, 0.020)
OFF = 0.047                   # mug centre is +y of the grasped wall
assert a.move_tcp((G[0], G[1], 0.68), Q_DOWN_Y, secs=4) is not None
p, q = a.tcp_world(); print('pre-grasp hand quat', np.round(q, 3), flush=True)
assert a.move_line((G[0], G[1], 0.52), Q_DOWN_Y, secs=3) is not None
a.close()
assert a.move_line((G[0], G[1], 0.70), Q_DOWN_Y, secs=3) is not None
print('after lift fingers', a.finger_gap(), flush=True)
assert a.move_tcp((PLATE[0], PLATE[1] - OFF, 0.70), Q_DOWN_Y, secs=5) is not None
assert a.move_line((PLATE[0], PLATE[1] - OFF, 0.545), Q_DOWN_Y, secs=3) is not None
a.open()
assert a.move_line((PLATE[0], PLATE[1] - OFF, 0.72), Q_DOWN_Y, secs=3) is not None
print('done', flush=True)
EOF
nohup python3 -u stage5.py > stage5.log 2>&1 &
sleep 120; cat stage5.log

# openrua op 38
timeout 120 python3 scene.py agentview && python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); img=cv2.imread('agentview.png')
z=p[...,2]
# plate
m=np.isfinite(z)&(p[...,0]>0.0)&(p[...,0]<0.3)&(abs(p[...,1])<0.2)&(z>0.44)&(z<0.46)
xyz=p[m]; c=img[m]; light=c.mean(1)>120; xyz=xyz[light]
print('plate-ish x %.3f..%.3f y %.3f..%.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max()))
# mug on plate slices
m=np.isfinite(z)&(p[...,0]>0.0)&(p[...,0]<0.3)&(abs(p[...,1])<0.2)&(z>0.46)&(z<0.62)
xyz=p[m]
for zl in np.arange(0.46,0.60,0.02):
    s=xyz[(xyz[:,2]>=zl)&(xyz[:,2]<zl+0.02)]
    if len(s): print('z %.2f n %4d x %.3f..%.3f y %.3f..%.3f'%(zl,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
crop=img[280:470,200:460]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2,fy=2))
"

# openrua op 39
cat > stage6.py <<'EOF'
from arm import *
a = Arm('stage6')
print('start tcp', a.tcp_world()[0].round(4), 'fingers', a.finger_gap(), flush=True)
PUD = (-0.181, -0.024)        # box centre; short axis along y
DST = (0.147, 0.18)           # right (+y) of the plate (plate centre y=0.02, r=0.068)
assert a.move_tcp((PUD[0], PUD[1], 0.60), Q_DOWN_Y, secs=4) is not None
assert a.move_line((PUD[0], PUD[1], 0.44), Q_DOWN_Y, secs=3) is not None
a.close()
assert a.move_line((PUD[0], PUD[1], 0.60), Q_DOWN_Y, secs=3) is not None
print('after lift fingers', a.finger_gap(), flush=True)
assert a.move_tcp((DST[0], DST[1], 0.60), Q_DOWN_Y, secs=5) is not None
assert a.move_line((DST[0], DST[1], 0.447), Q_DOWN_Y, secs=3) is not None
a.open()
assert a.move_line((DST[0], DST[1], 0.62), Q_DOWN_Y, secs=3) is not None
print('done', flush=True)
EOF
nohup python3 -u stage6.py > stage6.log 2>&1 &
sleep 120; cat stage6.log

# openrua op 40
timeout 120 python3 scene.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview && python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); img=cv2.imread('agentview.png')
z=p[...,2]
mask=(np.isfinite(z)&(z>0.432)&(z<0.62)&(p[...,0]>-0.4)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    xyz=p[m]; col=img[m].mean(0)
    print(i,'px',cent[i].round(0),'area',stats[i,4],'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(np.nanmin(xyz[:,0]),np.nanmax(xyz[:,0]),np.nanmin(xyz[:,1]),np.nanmax(xyz[:,1]),np.nanmin(xyz[:,2]),np.nanmax(xyz[:,2])),'bgr',col.round(0))
"

# openrua op 41
timeout 200 python3 -u -c "
from arm import *
a=Arm('t6')
q=a.arm_q(); print('current q', np.round(q,3))
print('tcp', a.tcp_world()[0].round(4), np.round(a.tcp_world()[1],3))
for name,xyz in [('above_pud',(-0.176,-0.037,0.60)),('grasp_pud',(-0.176,-0.037,0.44)),('above_dst',(0.147,0.18,0.60))]:
    for t in range(3):
        sol=a.ik_world(xyz,Q_DOWN_Y,seed=q,tries=1)
        print(name, None if sol is None else np.round(sol,3), 'maxdelta', None if sol is None else np.abs(np.array(sol)-np.array(q)).max().round(3))
"


# openrua op 42
python3 - <<'EOF'
src = open('arm.py').read()
src = src.replace('''    def move_joints(self, q, secs=3.0, via=None):
        goal = FollowJointTrajectory.Goal()''', '''    def ik_best(self, xyz_tcp, quat, n=6):
        """Several IK calls (current + home + perturbed seeds); pick the
        solution nearest the current joints."""
        cur = np.array(self.arm_q())
        seeds = [cur, np.array(HOME)] + [cur + np.random.uniform(-0.4, 0.4, 7) for _ in range(n - 2)]
        best = None
        for s in seeds:
            sol = self.ik_world(xyz_tcp, quat, seed=list(s), tries=1)
            if sol is None:
                continue
            d = np.abs(np.array(sol) - cur).max()
            if best is None or d < best[0]:
                best = (d, sol)
        return None if best is None else best[1]

    def move_joints(self, q, secs=3.0, via=None, _retry=True):
        delta = np.abs(np.array(q) - np.array(self.arm_q())).max()
        secs = max(secs, float(delta) / 0.4)   # <= 0.4 rad/s on the fastest joint
        goal = FollowJointTrajectory.Goal()''')
src = src.replace('''        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"FJT code={code} max_joint_err={err:.4f}", flush=True)
        return code''', '''        code = res.result().result.error_code
        self.spin(0.3)
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"FJT code={code} max_joint_err={err:.4f} (secs={secs:.1f})", flush=True)
        if err > 0.03 and _retry:
            print("  -> re-sending goal", flush=True)
            return self.move_joints(q, secs=max(3.0, secs), _retry=False)
        return code''')
src = src.replace('''    def move_tcp(self, xyz, quat, secs=3.0, seed=None):
        q = self.ik_world(xyz, quat, seed=seed)''', '''    def move_tcp(self, xyz, quat, secs=3.0, seed=None):
        q = self.ik_best(xyz, quat) if seed is None else self.ik_world(xyz, quat, seed=seed)''')
src = src.replace('''BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # panda_link0 in world (TF)''', '''BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # panda_link0 in world (TF)
HOME = [0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854]  # initial config''')
open('arm.py','w').write(src)
EOF
grep -n "ik_best\|HOME\|re-sending\|0.4 rad" arm.py | head; cat > stage7.py <<'EOF'
from arm import *
a = Arm('stage7')
print('start tcp', a.tcp_world()[0].round(4), 'q', np.round(a.arm_q(),3), flush=True)
# lift straight up first, then return to the nominal home configuration
assert a.move_line((0.147, 0.18, 0.78), Q_DOWN_Y, secs=3) is not None
a.move_joints(HOME, secs=6)
print('home tcp', a.tcp_world()[0].round(4), 'q', np.round(a.arm_q(),3), flush=True)
EOF
nohup python3 -u stage7.py > stage7.log 2>&1 &
sleep 60; cat stage7.log

# openrua op 43
timeout 120 python3 scene.py agentview && python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); img=cv2.imread('agentview.png')
z=p[...,2]
m=np.isfinite(z)&(p[...,0]>-0.30)&(p[...,0]<-0.08)&(p[...,1]>-0.12)&(p[...,1]<0.03)&(z>0.435)&(z<0.50)
xyz=p[m]
print('pud n',len(xyz),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].min(),xyz[:,2].max()))
top=xyz[xyz[:,2]>0.455]
print('top n',len(top),'x %.3f..%.3f y %.3f..%.3f'%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max()), 'centre', top[:,:2].mean(0).round(3))
"

# openrua op 44
sed -i 's/^PUD = .*/PUD = (-0.176, -0.037)        # box centre; short axis along y/' stage6.py && nohup python3 -u stage6.py > stage6b.log 2>&1 &
sleep 150; cat /workspace/stage6b.log

# openrua op 45
timeout 120 python3 scene.py agentview && timeout 60 python3 tools/perception/cam_snap.py frontview && python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); img=cv2.imread('agentview.png')
z=p[...,2]
mask=(np.isfinite(z)&(z>0.432)&(z<0.62)&(p[...,0]>-0.4)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    xyz=p[m]; col=img[m].mean(0)
    print(i,'px',cent[i].round(0),'area',stats[i,4],'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(np.nanmin(xyz[:,0]),np.nanmax(xyz[:,0]),np.nanmin(xyz[:,1]),np.nanmax(xyz[:,1]),np.nanmin(xyz[:,2]),np.nanmax(xyz[:,2])),'bgr',col.round(0))
"

# openrua op 46
python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); img=cv2.imread('agentview.png').astype(int)
z=p[...,2]
b,g,r=img[...,0],img[...,1],img[...,2]
base=np.isfinite(z)&(z>0.432)&(z<0.62)&(p[...,0]>-0.4)
red=base&(r>110)&(r>g+40)
white=base&(b>120)&(g>120)&(r>120)&(abs(r-b)<30)
dark=base&(r<90)&(g<70)&(b<80)
for name,m in [('red',red),('white',white),('dark',dark)]:
    xyz=p[m]
    print(name,len(xyz),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(np.percentile(xyz[:,0],1),np.percentile(xyz[:,0],99),np.percentile(xyz[:,1],1),np.percentile(xyz[:,1],99),np.percentile(xyz[:,2],1),np.percentile(xyz[:,2],99)))
crop=img[260:460,250:540].astype(np.uint8); cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2,fy=2))
"

# openrua op 47
cat > gohome.py <<'EOF'
from arm import *
a = Arm('gohome')
p,_ = a.tcp_world()
assert a.move_line((p[0], p[1], 0.78), Q_DOWN_Y, secs=3) is not None
a.move_joints(HOME, secs=6)
print('home tcp', a.tcp_world()[0].round(4), flush=True)
EOF
timeout 600 python3 -u gohome.py && timeout 120 python3 scene.py birdview && timeout 120 python3 scene.py agentview && python3 -c "
import numpy as np, cv2
for cam in ['birdview','agentview']:
    p=np.load(cam+'_pts.npy'); img=cv2.imread(cam+'.png').astype(int)
    z=p[...,2]; b,g,r=img[...,0],img[...,1],img[...,2]
    base=np.isfinite(z)&(z>0.432)&(z<0.62)&(p[...,0]>-0.1)&(p[...,0]<0.35)
    red=base&(r>100)&(r>g+35)&(r>b+35)
    xyz=p[red]
    print(cam,'red',len(xyz),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(np.percentile(xyz[:,0],2),np.percentile(xyz[:,0],98),np.percentile(xyz[:,1],2),np.percentile(xyz[:,1],98),np.percentile(xyz[:,2],2),np.percentile(xyz[:,2],98)))
    # pudding: dark, low
    dark=base&(r<90)&(g<70)&(b<80)&(z<0.475)&(p[...,1]>0.10)
    xyz=p[dark]
    if len(xyz): print(cam,'pud',len(xyz),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].min(),xyz[:,2].max()))
img=cv2.imread('birdview.png'); crop=img[280:360,290:400]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
"

# openrua op 48
python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); img=cv2.imread('agentview.png').astype(int)
z=p[...,2]; b,g,r=img[...,0],img[...,1],img[...,2]
base=np.isfinite(z)&(z>0.462)&(z<0.60)&(p[...,0]>-0.1)&(p[...,0]<0.35)&(p[...,1]>0.0)
red=base&(r>100)&(r>g+35)&(r>b+35)
xyz=p[red]
print('red n',len(xyz))
c=xyz[:,:2].mean(0); u,s,vt=np.linalg.svd(xyz[:,:2]-c,full_matrices=False)
ax=vt[0]; print('centre',c.round(3),'axis',ax.round(3),'angle deg',np.degrees(np.arctan2(ax[1],ax[0])).round(1),'sv',s.round(2))
proj=(xyz[:,:2]-c)@ax; print('extent along axis %.3f..%.3f'%(proj.min(),proj.max()))
perp=np.array([-ax[1],ax[0]]); pp=(xyz[:,:2]-c)@perp; print('extent perp %.3f..%.3f'%(pp.min(),pp.max()))
print('z %.3f..%.3f'%(xyz[:,2].min(),xyz[:,2].max()))
# also all points (any colour) in the mug region above 0.462 to catch the white interior/rim
m=np.isfinite(z)&(z>0.462)&(z<0.60)&(p[...,0]>0.0)&(p[...,0]<0.3)&(p[...,1]>0.06)&(p[...,1]<0.30)
xyz=p[m]; print('all n',len(xyz),'x %.3f..%.3f y %.3f..%.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max()))
proj=(xyz[:,:2]-c)@ax; print('all extent along axis %.3f..%.3f'%(proj.min(),proj.max()))
# pudding
m=np.isfinite(z)&(z>0.44)&(z<0.475)&(p[...,0]>0.0)&(p[...,0]<0.3)&(p[...,1]>0.13)&(p[...,1]<0.30)&(r<120)
xyz=p[m]; print('pud n',len(xyz),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].min(),xyz[:,2].max()))
"

# openrua op 49
cat > hover.py <<'EOF'
from arm import *
import sys
a = Arm('hover')
x, y, z = map(float, sys.argv[1:4])
assert a.move_tcp((x, y, z), Q_DOWN_Y, secs=4) is not None
print('tcp', a.tcp_world()[0].round(4), np.round(a.tcp_world()[1],3), flush=True)
EOF
timeout 600 python3 -u hover.py 0.10 0.15 0.85 && timeout 120 python3 scene.py robot0_eye_in_hand && python3 -c "
import numpy as np, cv2
p=np.load('robot0_eye_in_hand_pts.npy'); img=cv2.imread('robot0_eye_in_hand.png')
z=p[...,2]
print('z range', np.nanmin(z), np.nanmax(z))
mask=(np.isfinite(z)&(z>0.432)&(z<0.65)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<100: continue
    m=lab==i
    xyz=p[m]; col=img[m].mean(0)
    print(i,'px',cent[i].round(0),'area',stats[i,4],'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(np.nanmin(xyz[:,0]),np.nanmax(xyz[:,0]),np.nanmin(xyz[:,1]),np.nanmax(xyz[:,1]),np.nanmin(xyz[:,2]),np.nanmax(xyz[:,2])),'bgr',col.round(0))
"

# openrua op 50
python3 -c "
import numpy as np, cv2
p=np.load('robot0_eye_in_hand_pts.npy'); img=cv2.imread('robot0_eye_in_hand.png').astype(int)
z=p[...,2]; b,g,r=img[...,0],img[...,1],img[...,2]
mask=(np.isfinite(z)&(z>0.459)&(z<0.473)&(p[...,1]>0.05)&(p[...,0]<0.3)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<40: continue
    m=lab==i; xyz=p[m]; col=img[m].mean(0)
    c=xyz[:,:2].mean(0); u,s,vt=np.linalg.svd(xyz[:,:2]-c,full_matrices=False); ax=vt[0]
    pr=(xyz[:,:2]-c)@ax; pp=(xyz[:,:2]-c)@np.array([-ax[1],ax[0]])
    print(i,'area',stats[i,4],'centre',c.round(3),'axis deg',np.degrees(np.arctan2(ax[1],ax[0])).round(1),'len %.3f wid %.3f'%(pr.max()-pr.min(),pp.max()-pp.min()),'bgr',col.round(0))
# red mug: red points z>0.475
red=np.isfinite(z)&(z>0.475)&(z<0.60)&(r>90)&(r>g+30)&(r>b+30)&(p[...,1]>0.03)
xyz=p[red]; c=xyz[:,:2].mean(0); u,s,vt=np.linalg.svd(xyz[:,:2]-c,full_matrices=False); ax=vt[0]
pr=(xyz[:,:2]-c)@ax
print('red mug n',len(xyz),'centre',c.round(3),'axis',ax.round(3),'deg',np.degrees(np.arctan2(ax[1],ax[0])).round(1),'along %.3f..%.3f'%(pr.min(),pr.max()),'zmax %.3f'%xyz[:,2].max())
# white-ish points (mug interior / rim) near the red mug region
wh=np.isfinite(z)&(z>0.44)&(z<0.60)&(r>150)&(g>150)&(b>150)&(p[...,1]>0.10)&(p[...,0]<0.25)
xyz=p[wh]; print('white near mug n',len(xyz),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].min(),xyz[:,2].max()))
"

# openrua op 51
python3 -c "
import numpy as np
a=np.array([0.576,0.817]); a/=np.linalg.norm(a); n=np.array([-a[1],a[0]]); c=np.array([0.04,0.14]); zc=0.4755
for cam in ['agentview','robot0_eye_in_hand','birdview']:
    p=np.load(cam+'_pts.npy'); z=p[...,2]
    m=np.isfinite(z)&(z>0.44)&(z<0.60)
    xyz=p[m]
    d=xyz[:,:2]-c; along=d@a; perp=d@n
    rad=np.sqrt(perp**2+(xyz[:,2]-zc)**2)
    sel=(np.abs(perp)<0.056)&(along>-0.15)&(along<0.15)&(rad<0.056)
    s=xyz[sel]; al=along[sel]; rd=rad[sel]; pe=perp[sel]
    wall=(rd>0.038)
    print(cam,'n',sel.sum(),'along all %.3f..%.3f'%(al.min(),al.max()),'wall along %.3f..%.3f'%(al[wall].min(),al[wall].max()), 'perp %.3f..%.3f'%(pe.min(),pe.max()),'zmax %.3f'%s[:,2].max())
    # histogram of along for wall points near the +a end
    h,e=np.histogram(al[wall],bins=np.arange(-0.10,0.14,0.01)); print('  ',list(zip(e[:-1].round(2),h)))
"

# openrua op 52
python3 -c "
import numpy as np
a=np.array([0.576,0.817]); a/=np.linalg.norm(a); n=np.array([-a[1],a[0]]); c=np.array([0.04,0.14]); zc=0.4755
p=np.load('agentview_pts.npy'); z=p[...,2]
m=np.isfinite(z)&(z>0.44)&(z<0.60); xyz=p[m]
d=xyz[:,:2]-c; along=d@a; perp=d@n; dz=xyz[:,2]-zc; rad=np.sqrt(perp**2+dz**2)
for lo in np.arange(-0.08,0.13,0.01):
    s=(along>=lo)&(along<lo+0.01)&(rad<0.09)&(np.abs(perp)<0.09)
    if s.sum()==0: continue
    print('along %.2f n %4d perp %.3f..%.3f  dz %.3f..%.3f rad %.3f..%.3f'%(lo,s.sum(),perp[s].min(),perp[s].max(),dz[s].min(),dz[s].max(),rad[s].min(),rad[s].max()))
"

# openrua op 53
cat >> arm.py <<'EOF'


def R_quat(R):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def rot_axis(axis, ang):
    """Rodrigues rotation matrix about unit axis by ang (rad)."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def _move_path(self, poses, secs=4.0, seed=None):
    """Sequential IK through a list of (xyz_tcp, quat); one trajectory."""
    seed = seed if seed is not None else self.arm_q()
    via = []
    for xyz, quat in poses:
        q = self.ik_world(xyz, quat, seed=seed)
        if q is None:
            print(f"IK FAILED on path at {np.round(xyz, 3)}", flush=True)
            return None
        if np.abs(np.array(q) - np.array(seed)).max() > 1.2:
            print(f"IK branch jump on path at {np.round(xyz, 3)}", flush=True)
            return None
        via.append(q)
        seed = q
    self.move_joints(via[-1], secs, via=via[:-1])
    p, qq = self.tcp_world()
    print(f"tcp now {p.round(4)} quat {np.round(qq, 3)}", flush=True)
    return via[-1]


Arm.move_path = _move_path
EOF
python3 -c "import arm; print('ok')"

# openrua op 54
timeout 60 ros2 param get /move_group robot_description 2>/dev/null > /tmp/rd.txt; grep -n -i -A12 'link name="panda_hand"' /tmp/rd.txt | head -60; grep -o 'package://[^"]*hand[^"]*' /tmp/rd.txt | sort -u

# openrua op 55
python3 - <<'EOF'
import numpy as np, struct
def stl_bbox(fn):
    d=open(fn,'rb').read()
    if d[:5]==b'solid' and b'facet' in d[:300]:
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)',d)])
    else:
        n=struct.unpack_from('<I',d,80)[0]
        arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0),v.max(0)
for f in ['hand','finger']:
    print(f, stl_bbox(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'))
EOF

# openrua op 56
find / -iname 'hand*.stl' -o -iname 'hand*.obj' -o -iname 'finger*.stl' -o -iname 'finger*.obj' 2>/dev/null | grep -v proc | head

# openrua op 57
python3 -c "
import numpy as np
for cam in ['agentview','birdview']:
    p=np.load(cam+'_pts.npy'); z=p[...,2]
    for lo,hi in [(0.85,0.90),(0.90,0.95),(0.95,0.98),(0.98,1.01),(1.01,1.05),(1.05,1.1)]:
        m=np.isfinite(z)&(z>lo)&(z<hi)&(np.abs(p[...,0]-0.1)<0.2)&(np.abs(p[...,1]-0.15)<0.2)
        if m.sum(): print(cam,lo,hi,m.sum(),'x %.3f..%.3f y %.3f..%.3f'%(p[m,0].min(),p[m,0].max(),p[m,1].min(),p[m,1].max()))
"

# openrua op 58
python3 scene.py agentview && python3 scene.py birdview && python3 -c "
import numpy as np
for cam in ['agentview','birdview']:
    p=np.load(cam+'_pts.npy'); z=p[...,2]
    for lo in np.arange(0.80,1.12,0.02):
        m=np.isfinite(z)&(z>lo)&(z<lo+0.02)&(np.abs(p[...,0]-0.1)<0.2)&(np.abs(p[...,1]-0.15)<0.2)
        if m.sum(): print(cam,'%.2f'%lo,m.sum(),'x %.3f..%.3f y %.3f..%.3f'%(p[m,0].min(),p[m,0].max(),p[m,1].min(),p[m,1].max()))
"

# openrua op 59
mkdir -p "$(dirname /workspace/stage8.py)"
cat > /workspace/stage8.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Right the knocked-over red mug.

Lying-cylinder geometry (from point clouds): axis dir a, centre c, axis
height ZC, outer radius R0, mouth plane at +MOUTH along a from c.
Plan: pitched head-on rim pinch at the top of the mouth (one finger inside,
one outside), lift, rotate 90 deg about n so the axis points up, place.

Usage: python3 stage8.py plan|A|B|C
"""
import math
import sys

import numpy as np

from arm import Arm, HOME, R_quat, rot_axis

a = np.array([0.576, 0.817, 0.0]); a /= np.linalg.norm(a)
n = np.array([-a[1], a[0], 0.0])
Z = np.array([0.0, 0.0, 1.0])
c = np.array([0.04, 0.14, 0.0])
ZC, R0, WALL = 0.4755, 0.05, 0.005
MOUTH = 0.068
BETA = math.radians(35.0)
TABLE = 0.4255
MUG_H = 0.148

# grasp point: 2 cm inside mouth, slightly to +n, wall mid-thickness
PERP = 0.01
zg = ZC + math.sqrt((R0 - WALL / 2) ** 2 - PERP ** 2)
G = c + (MOUTH - 0.02) * a + PERP * n + np.array([0, 0, zg])

d1 = -math.cos(BETA) * a - math.sin(BETA) * Z          # approach: into mouth, down
f1 = -math.sin(BETA) * a + math.cos(BETA) * Z          # finger axis: up/back


def R_from(d, f):
    x = np.cross(f, d)
    return np.column_stack([x, f, d])


def rotated(R, ang):
    return rot_axis(n, ang) @ R


R1 = R_from(d1, f1)
R1b = R_from(d1, -f1)
PRE = G - 0.07 * d1
LIFT = G + np.array([0, 0, 0.20])
TARGET_AXIS = np.array([0.0, 0.14])
STEPS = 6


def place_tcp(R):
    # after rotation, grasp point is at axis - R0*(rotated +Z)
    rad = R @ np.linalg.inv(R1) @ Z  # world dir of original +Z after rotation
    xy = TARGET_AXIS - (R0 - WALL / 2) * rad[:2]
    return np.array([xy[0], xy[1], TABLE + MUG_H - 0.02 + 0.006])


def main():
    mode = sys.argv[1]
    arm = Arm("stage8")
    print("G", G.round(4), "PRE", PRE.round(4), flush=True)
    if mode == "plan":
        cur = np.array(arm.arm_q())
        for name, R in [("R1", R1), ("R1b", R1b)]:
            q = arm.ik_best(PRE, R_quat(R))
            print(name, "pre-grasp IK", None if q is None else np.round(q, 3),
                  None if q is None else round(float(np.abs(np.array(q) - cur).max()), 3))
            if q is None:
                continue
            seed = q
            chain = [(G, R_quat(R)), (LIFT, R_quat(R))]
            for k in range(1, STEPS + 1):
                chain.append((LIFT, R_quat(rotated(R, -k * math.pi / 2 / STEPS))))
            Rf = rotated(R, -math.pi / 2)
            P = place_tcp(Rf)
            chain.append((np.array([P[0], P[1], LIFT[2]]), R_quat(Rf)))
            chain.append((P, R_quat(Rf)))
            ok = True
            for xyz, quat in chain:
                s = arm.ik_world(xyz, quat, seed=seed, tries=2)
                if s is None:
                    print("   IK fail at", np.round(xyz, 3)); ok = False; break
                print("   step dq=%.3f" % np.abs(np.array(s) - np.array(seed)).max(), np.round(s, 2))
                seed = s
            print(name, "chain ok" if ok else "chain FAILED", "place", P.round(4))
        return

    R = R1 if len(sys.argv) < 3 else (R1b if sys.argv[2] == "b" else R1)
    if mode == "A":
        arm.open()
        arm.move_tcp(PRE, R_quat(R), secs=6)
        # straight approach along d1 to the grasp point
        arm.move_path([(PRE + (G - PRE) * i / 4, R_quat(R)) for i in range(1, 5)], secs=6)
        p, q = arm.tcp_world(); print("at grasp: tcp", p.round(4), "quat", np.round(q, 3), flush=True)
        arm.close()
        print("fingers", arm.finger_gap(), flush=True)
    elif mode == "B":
        arm.move_path([(G + (LIFT - G) * i / 4, R_quat(R)) for i in range(1, 5)], secs=6)
        poses = [(LIFT, R_quat(rotated(R, -k * math.pi / 2 / STEPS))) for k in range(1, STEPS + 1)]
        arm.move_path(poses, secs=10)
        print("fingers", arm.finger_gap(), flush=True)
    elif mode == "C":
        Rf = rotated(R, -math.pi / 2)
        P = place_tcp(Rf)
        p0, _ = arm.tcp_world()
        P1 = np.array([P[0], P[1], p0[2]])
        arm.move_path([(p0 + (P1 - p0) * i / 4, R_quat(Rf)) for i in range(1, 5)], secs=6)
        arm.move_path([(P1 + (P - P1) * i / 5, R_quat(Rf)) for i in range(1, 6)], secs=6)
        p, q = arm.tcp_world(); print("at place: tcp", p.round(4), "quat", np.round(q, 3), flush=True)
        arm.open()
        up = P + np.array([0, 0, 0.14])
        arm.move_path([(P + (up - P) * i / 3, R_quat(Rf)) for i in range(1, 4)], secs=5)
    elif mode == "home":
        arm.move_joints(HOME, secs=8)
    print("done", flush=True)


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 60
timeout 600 python3 -u stage8.py plan 2>&1 | grep -v '^\[' | tail -40

# openrua op 61
cat > probe8.py <<'EOF'
import math, sys, numpy as np
from arm import Arm, HOME, R_quat
from stage8 import a, n, Z, G, R_from
arm = Arm("probe8")
cur = np.array(arm.arm_q())
for beta in [35, 45, 55]:
    for gam in [0, 20, 35, 50, 70]:
        b, g = math.radians(beta), math.radians(gam)
        h = math.cos(g) * (-a) + math.sin(g) * (-n)   # horizontal approach dir, yawed toward -n (pointing -y-ish)
        d = math.cos(b) * h - math.sin(b) * Z
        f = (Z - (Z @ d) * d); f /= np.linalg.norm(f)
        best = None
        for fs in (f, -f):
            R = R_from(d, fs)
            for seed in (cur, np.array(HOME)):
                q = arm.ik_world(G, R_quat(R), seed=list(seed), tries=1)
                if q is not None:
                    dq = np.abs(np.array(q) - cur).max()
                    if best is None or dq < best[0]:
                        best = (dq, np.round(q, 2), 'f+' if fs is f else 'f-')
        print(f"beta={beta} gam={gam} ->", best, flush=True)
EOF
timeout 900 python3 -u probe8.py 2>&1 | grep -v '^\['

# openrua op 62
cat > probe8b.py <<'EOF'
import math, sys, numpy as np
from arm import Arm, HOME, R_quat
from stage8 import a, n, Z, G, R_from, rotated, place_tcp, STEPS, d1, LIFT, R1
import stage8
arm = Arm("probe8b")
cur = np.array(arm.arm_q())
R = R1
qG = arm.ik_world(G, R_quat(R), seed=list(cur), tries=2)
print("G", np.round(qG, 2))
for name, P in [("pre-0.04d", G - 0.04 * d1), ("pre-0.07d", G - 0.07 * d1), ("pre-up6-0.02d", G + np.array([0, 0, 0.06]) - 0.02 * d1), ("pre-up8", G + np.array([0, 0, 0.08]))]:
    q = arm.ik_world(P, R_quat(R), seed=qG, tries=2)
    print(name, P.round(3), None if q is None else (np.round(q, 2), round(float(np.abs(np.array(q) - qG).max()), 3)))
seed = qG
chain = [("lift", LIFT, R_quat(R))]
for k in range(1, STEPS + 1):
    chain.append((f"rot{k}", LIFT, R_quat(rotated(R, -k * math.pi / 2 / STEPS))))
Rf = rotated(R, -math.pi / 2)
P = place_tcp(Rf)
chain.append(("over", np.array([P[0], P[1], LIFT[2]]), R_quat(Rf)))
chain.append(("place", P, R_quat(Rf)))
for name, xyz, quat in chain:
    s = arm.ik_world(xyz, quat, seed=seed, tries=2)
    if s is None:
        print(name, "IK FAIL", np.round(xyz, 3)); break
    print(name, np.round(xyz, 3), "dq=%.3f" % np.abs(np.array(s) - np.array(seed)).max(), np.round(s, 2))
    seed = s
EOF
timeout 900 python3 -u probe8b.py 2>&1 | grep -v '^\['

# openrua op 63
python3 - <<'EOF'
import re
s=open('stage8.py').read()
s=s.replace("PRE = G - 0.07 * d1","PRE = G + np.array([0, 0, 0.06]) - 0.02 * d1")
s=s.replace("xy = TARGET_AXIS - (R0 - WALL / 2) * rad[:2]","xy = TARGET_AXIS + (R0 - WALL / 2) * rad[:2]")
open('stage8.py','w').write(s)
EOF
python3 -c "
import math,numpy as np
from stage8 import *
Rf=rotated(R1,-math.pi/2)
print('d1',d1.round(3),'f1',f1.round(3)); print('d\'',Rf[:,2].round(3),'f\'',Rf[:,1].round(3)); print('a->',(rot_axis(n,-math.pi/2)@a).round(3)); print('place',place_tcp(Rf).round(4))"

# openrua op 64
python3 - <<'EOF'
s=open('stage8.py').read()
s=s.replace("""        arm.move_tcp(PRE, R_quat(R), secs=6)
""","""        HI = PRE + np.array([0, 0, 0.16])
        arm.move_tcp(HI, R_quat(R), secs=8)
        arm.move_path([(HI + (PRE - HI) * i / 3, R_quat(R)) for i in range(1, 4)], secs=5)
""")
open('stage8.py','w').write(s)
EOF
nohup python3 -u stage8.py A > stage8A.log 2>&1 &
sleep 60; grep -v '^\[' stage8A.log

# openrua op 65
python3 scene.py agentview >/dev/null && python3 scene.py robot0_eye_in_hand >/dev/null; python3 -c "
import numpy as np
a=np.array([0.576,0.817]); a/=np.linalg.norm(a); n=np.array([-a[1],a[0]]); c=np.array([0.04,0.14]); zc=0.4755
p=np.load('agentview_pts.npy'); z=p[...,2]
m=np.isfinite(z)&(z>0.44)&(z<0.60); xyz=p[m]
d=xyz[:,:2]-c; along=d@a; perp=d@n; dz=xyz[:,2]-zc
for lo in np.arange(-0.09,0.08,0.01):
    s=(along>=lo)&(along<lo+0.01)&(np.abs(perp)<0.06)&(np.abs(dz)<0.06)
    if s.sum(): print('along %.2f n %4d perp %.3f..%.3f dz %.3f..%.3f'%(lo,s.sum(),perp[s].min(),perp[s].max(),dz[s].min(),dz[s].max()))
"

# openrua op 66
nohup python3 -u stage8.py B > stage8B.log 2>&1 & sleep 75; grep -v '^\[' /workspace/stage8B.log

# openrua op 67
python3 scene.py agentview >/dev/null && python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); z=p[...,2]
img=cv2.imread('agentview.png')
# red mask
b,g,r=[img[...,i].astype(int) for i in range(3)]
red=(r>120)&(g<80)&(b<80)
m=np.isfinite(z)&(z>0.50)&(z<0.80)&red
xyz=p[m]; print('red pts',m.sum())
for lo in np.arange(0.50,0.80,0.02):
    s=(xyz[:,2]>=lo)&(xyz[:,2]<lo+0.02)
    if s.sum(): print('z %.2f n %4d x %.3f..%.3f y %.3f..%.3f'%(lo,s.sum(),xyz[s,0].min(),xyz[s,0].max(),xyz[s,1].min(),xyz[s,1].max()))
# PCA axis of red points
q=xyz-xyz.mean(0); w,v=np.linalg.eigh(q.T@q); print('axis',v[:,2].round(3),'centre',xyz.mean(0).round(3))
"

# openrua op 68
python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); z=p[...,2]
img=cv2.imread('agentview.png')
b,g,r=[img[...,i].astype(int) for i in range(3)]
red=(r>80)&(r>1.6*g)&(r>1.6*b)
m=np.isfinite(z)&(z>0.50)&(z<0.80)&red&(p[...,1]>0.05)
xyz=p[m]; print('red pts',m.sum())
for lo in np.arange(0.56,0.76,0.02):
    s=(xyz[:,2]>=lo)&(xyz[:,2]<lo+0.02)
    if s.sum(): print('z %.2f n %4d x %.3f..%.3f y %.3f..%.3f'%(lo,s.sum(),xyz[s,0].min(),xyz[s,0].max(),xyz[s,1].min(),xyz[s,1].max()))
q=xyz-xyz.mean(0); w,v=np.linalg.eigh(q.T@q); ax=v[:,2]; ax*=np.sign(ax[2]); print('axis',ax.round(3),'centre',xyz.mean(0).round(3))
# fit circle centre per slice (visible half only -> use x-min? ) use mean of extreme y as axis y
"

# openrua op 69
python3 scene.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
p=np.load('birdview_pts.npy'); z=p[...,2]
img=cv2.imread('birdview.png')
b,g,r=[img[...,i].astype(int) for i in range(3)]
red=(r>80)&(r>1.6*g)&(r>1.6*b)
m=np.isfinite(z)&(z>0.55)&(z<0.80)&red&(p[...,1]>0.05)
xyz=p[m]; print('red pts',m.sum())
for lo in np.arange(0.56,0.78,0.02):
    s=(xyz[:,2]>=lo)&(xyz[:,2]<lo+0.02)
    if s.sum(): print('z %.2f n %4d x %.3f..%.3f y %.3f..%.3f'%(lo,s.sum(),xyz[s,0].min(),xyz[s,0].max(),xyz[s,1].min(),xyz[s,1].max()))
# circle fit to rim points (z>0.70)
s=xyz[:,2]>0.705; P=xyz[s,:2]
A=np.c_[2*P,np.ones(len(P))]; bb=(P**2).sum(1); sol=np.linalg.lstsq(A,bb,rcond=None)[0]; cx,cy=sol[:2]; rr=np.sqrt(sol[2]+cx**2+cy**2)
print('rim circle centre',round(cx,4),round(cy,4),'r',round(rr,4))
# also all non-red points high up (interior of mug seen from top - white interior)
m2=np.isfinite(z)&(z>0.55)&(z<0.72)&(np.abs(p[...,0]-cx)<0.06)&(np.abs(p[...,1]-cy)<0.06)
print('interior/bottom pts',m2.sum(),'zmin',p[m2,2].min() if m2.sum() else None)
"

# openrua op 70
python3 -c "
import numpy as np, cv2
p=np.load('birdview_pts.npy'); z=p[...,2]
m=np.isfinite(z)&(z>0.595)&(z<0.625)&(np.abs(p[...,0]-0.1)<0.12)&(np.abs(p[...,1]-0.22)<0.12)
P=p[m]; print(m.sum(), 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max(),P[:,2].min(),P[:,2].max()))
print('centre',P[:,:2].mean(0).round(4))
img=cv2.imread('birdview.png'); ys,xs=np.where(m); print('px rows',ys.min(),ys.max(),'cols',xs.min(),xs.max())
cv2.imwrite('bird_crop.png', cv2.resize(img[max(0,ys.min()-80):ys.max()+80, max(0,xs.min()-80):xs.max()+80],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 71
python3 - <<'EOF'
s=open('stage8.py').read()
s=s.replace("""        Rf = rotated(R, -math.pi / 2)
        P = place_tcp(Rf)
        p0, _ = arm.tcp_world()""","""        Rf = rotated(R, -math.pi / 2)
        # measured while hanging: axis = TCP + (0.028, 0.043); bottom = TCP z - 0.125
        P = np.array([TARGET_AXIS[0] - 0.028, TARGET_AXIS[1] - 0.043, TABLE + 0.125 + 0.007])
        p0, _ = arm.tcp_world()""")
open('stage8.py','w').write(s)
EOF
nohup python3 -u stage8.py C > stage8C.log 2>&1 & sleep 70; grep -v '^\[' /workspace/stage8C.log

# openrua op 72
sleep 20; grep -v '^\[' /workspace/stage8C.log | tail -4; python3 scene.py agentview >/dev/null; python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); z=p[...,2]
img=cv2.imread('agentview.png')
b,g,r=[img[...,i].astype(int) for i in range(3)]
red=(r>80)&(r>1.6*g)&(r>1.6*b)
m=np.isfinite(z)&(z>0.43)&(z<0.65)&red&(p[...,1]>0.05)&(p[...,0]<0.12)
xyz=p[m]; print('red pts',m.sum())
for lo in np.arange(0.43,0.60,0.02):
    s=(xyz[:,2]>=lo)&(xyz[:,2]<lo+0.02)
    if s.sum(): print('z %.2f n %4d x %.3f..%.3f y %.3f..%.3f'%(lo,s.sum(),xyz[s,0].min(),xyz[s,0].max(),xyz[s,1].min(),xyz[s,1].max()))
q=xyz-xyz.mean(0); w,v=np.linalg.eigh(q.T@q); ax=v[:,2]; ax*=np.sign(ax[2]); print('axis',ax.round(3),'top z',xyz[:,2].max().round(4))
"

# openrua op 73
python3 scene.py birdview >/dev/null; python3 -c "
import numpy as np
for cam in ['agentview','birdview']:
    p=np.load(cam+'_pts.npy'); z=p[...,2]
    m=np.isfinite(z)&(z>0.445)&(z<0.60)&(p[...,0]>-0.12)&(p[...,0]<0.085)&(p[...,1]>-0.02)&(p[...,1]<0.22)
    xyz=p[m]; print(cam,'pts',m.sum())
    for lo in np.arange(0.44,0.58,0.02):
        s=(xyz[:,2]>=lo)&(xyz[:,2]<lo+0.02)
        if s.sum(): print('  z %.2f n %4d x %.3f..%.3f y %.3f..%.3f'%(lo,s.sum(),xyz[s,0].min(),xyz[s,0].max(),xyz[s,1].min(),xyz[s,1].max()))
    hi=xyz[xyz[:,2]>0.50]; q=hi[:,:2]-hi[:,:2].mean(0); w,v=np.linalg.eigh(q.T@q); print('  top-ridge axis',v[:,1].round(3),'centre',hi.mean(0).round(3),'zmax',xyz[:,2].max().round(4))
"

# openrua op 74
python3 -c "
import cv2
img=cv2.imread('agentview.png'); crop=img[240:350,280:440]; cv2.imwrite('crop_red.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
b=cv2.imread('birdview.png'); import numpy as np; p=np.load('birdview_pts.npy'); m=np.isfinite(p[...,2])&(p[...,2]>0.5)&(np.abs(p[...,0])<0.06)&(p[...,1]>-0.03)&(p[...,1]<0.13); ys,xs=np.where(m); print(ys.min(),ys.max(),xs.min(),xs.max())
cv2.imwrite('crop_bird.png',cv2.resize(b[ys.min()-60:ys.max()+60,xs.min()-60:xs.max()+60],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 75
python3 -c "
import numpy as np
p=np.load('agentview_pts.npy'); z=p[...,2]
m=np.isfinite(z)&(z>0.428)&(z<0.447)&(np.abs(p[...,0]-0.147)<0.16)&(np.abs(p[...,1]-0.02)<0.16)
P=p[m]; r=np.hypot(P[:,0]-0.147,P[:,1]-0.02)
print('plate-ish pts',m.sum(),'r percentiles',np.percentile(r,[50,90,95,98,99.5]).round(3))
ang=np.degrees(np.arctan2(P[:,1]-0.02,P[:,0]-0.147))
for lo in range(-180,180,45):
    s=(ang>=lo)&(ang<lo+45)
    if s.sum(): print('  sector %4d..%4d n %4d rmax %.3f'%(lo,lo+45,s.sum(),r[s].max()))
# red mug side near plate
m2=np.isfinite(z)&(z>0.45)&(z<0.53)&(p[...,0]>-0.06)&(p[...,0]<0.06)&(p[...,1]>-0.05)&(p[...,1]<0.15)
Q=p[m2]; print('mug pts',m2.sum(),'x %.3f..%.3f y %.3f..%.3f'%(Q[:,0].min(),Q[:,0].max(),Q[:,1].min(),Q[:,1].max()))
s=Q[:,2]<0.49; print(' low band x max %.3f'%Q[s,0].max(), ' xmax at z:', Q[np.argmax(Q[:,0]),2].round(3))
"

# openrua op 76
python3 - <<'EOF'
s=open('stage8.py').read()
s=s.replace("a = np.array([0.576, 0.817, 0.0]); a /= np.linalg.norm(a)","a = np.array([0.0, 1.0, 0.0]); a /= np.linalg.norm(a)   # attempt 2: mug lies along +y, mouth at +y")
s=s.replace("c = np.array([0.04, 0.14, 0.0])","c = np.array([-0.02, 0.05, 0.0])")
s=s.replace("MOUTH = 0.068","MOUTH = 0.074")
s=s.replace("TARGET_AXIS = np.array([0.0, 0.14])","TARGET_AXIS = np.array([-0.02, 0.16])")
# phase C: parametrised hanging offset + slow retreat
s=s.replace("""        # measured while hanging: axis = TCP + (0.028, 0.043); bottom = TCP z - 0.125
        P = np.array([TARGET_AXIS[0] - 0.028, TARGET_AXIS[1] - 0.043, TABLE + 0.125 + 0.007])""",
"""        # measured while hanging (argv): axis = TCP + (ox, oy); bottom = TCP z - dz
        ox, oy, dz = map(float, sys.argv[3:6])
        P = np.array([TARGET_AXIS[0] - ox, TARGET_AXIS[1] - oy, TABLE + dz + 0.001])""")
s=s.replace("""        arm.move_path([(P1 + (P - P1) * i / 5, R_quat(Rf)) for i in range(1, 6)], secs=6)""",
"""        arm.move_path([(P1 + (P - P1) * i / 6, R_quat(Rf)) for i in range(1, 7)], secs=10)""")
s=s.replace("""        arm.open()
        up = P + np.array([0, 0, 0.14])
        arm.move_path([(P + (up - P) * i / 3, R_quat(Rf)) for i in range(1, 4)], secs=5)""",
"""        arm.open()
        arm.spin(1.0)
        print("fingers", arm.finger_gap(), flush=True)
        p2, _ = arm.tcp_world()
        up = p2 + np.array([0, 0, 0.15])
        arm.move_path([(p2 + (up - p2) * i / 6, R_quat(Rf)) for i in range(1, 7)], secs=14)""")
open('stage8.py','w').write(s)
EOF
sed -i 's/R = R1 if len(sys.argv) < 3 else (R1b if sys.argv\[2\] == "b" else R1)/R = R1b if (len(sys.argv) > 2 and sys.argv[2] == "b") else R1/' stage8.py
python3 -c "
import math,numpy as np
from stage8 import *
Rf=rotated(R1,-math.pi/2)
print('G',G.round(4),'PRE',PRE.round(4),'LIFT',LIFT.round(4)); print('d1',d1.round(3),'f1',f1.round(3)); print('d\'',Rf[:,2].round(3),'f\'',Rf[:,1].round(3)); print('a->',(rot_axis(n,-math.pi/2)@a).round(3))"

# openrua op 77
cat > probe8c.py <<'EOF'
import math, numpy as np
from arm import Arm, HOME, R_quat
from stage8 import G, PRE, LIFT, R1, rotated, STEPS, TARGET_AXIS, TABLE
arm = Arm("probe8c")
cur = np.array(arm.arm_q())
R = R1
HI = PRE + np.array([0, 0, 0.16])
qHI = arm.ik_best(HI, R_quat(R)); print("HI", None if qHI is None else (np.round(qHI, 2), round(float(np.abs(qHI - cur).max()), 2)))
seed = qHI
Rf = rotated(R, -math.pi / 2)
P = np.array([TARGET_AXIS[0], TARGET_AXIS[1] - 0.051, TABLE + 0.123])
chain = [("pre", PRE, R_quat(R)), ("G", G, R_quat(R)), ("lift", LIFT, R_quat(R))]
chain += [(f"rot{k}", LIFT, R_quat(rotated(R, -k * math.pi / 2 / STEPS))) for k in range(1, STEPS + 1)]
chain += [("over", np.array([P[0], P[1], LIFT[2]]), R_quat(Rf)), ("place", P, R_quat(Rf)), ("up", P + [0, 0, 0.15], R_quat(Rf))]
for name, xyz, quat in chain:
    s = arm.ik_world(xyz, quat, seed=seed, tries=2)
    if s is None:
        print(name, "IK FAIL", np.round(xyz, 3)); break
    print(name, np.round(xyz, 3), "dq=%.3f" % np.abs(np.array(s) - np.array(seed)).max(), np.round(s, 2))
    seed = s
EOF
timeout 600 python3 -u probe8c.py 2>&1 | grep -v '^\['

# openrua op 78
sed -i 's/HI = PRE + np.array(\[0, 0, 0.16\])/HI = PRE + np.array([0, 0, 0.22])/' stage8.py && nohup python3 -u stage8.py A > stage8A2.log 2>&1 & sleep 75; grep -v '^\[' /workspace/stage8A2.log

# openrua op 79
python3 scene.py agentview >/dev/null; python3 -c "
import numpy as np
p=np.load('agentview_pts.npy'); z=p[...,2]
m=np.isfinite(z)&(z>0.45)&(z<0.535)&(p[...,0]>-0.06)&(p[...,0]<0.06)&(p[...,1]>-0.05)&(p[...,1]<0.16)
Q=p[m]; print('mug pts',m.sum(),'x %.3f..%.3f y %.3f..%.3f zmax %.4f'%(Q[:,0].min(),Q[:,0].max(),Q[:,1].min(),Q[:,1].max(),Q[:,2].max()))
"

# openrua op 80
nohup python3 -u stage8.py B > stage8B2.log 2>&1 & sleep 80; grep -v '^\[' /workspace/stage8B2.log

# openrua op 81
python3 scene.py agentview >/dev/null; python3 scene.py birdview >/dev/null; python3 -c "
import numpy as np
p=np.load('birdview_pts.npy'); z=p[...,2]
m=np.isfinite(z)&(z>0.56)&(z<0.66)&(np.abs(p[...,0]+0.03)<0.12)&(np.abs(p[...,1]-0.15)<0.12)
P=p[m]; print(m.sum(), 'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(P[:,0].min(),P[:,0].max(),P[:,1].min(),P[:,1].max(),P[:,2].min(),P[:,2].max()) if m.sum() else '')
if m.sum(): print('centre',P[:,:2].mean(0).round(4))
"

# openrua op 82
python3 -c "
import numpy as np
p=np.load('agentview_pts.npy'); z=p[...,2]
m=np.isfinite(z)&(z>0.45)&(z<0.60)&(p[...,0]>-0.08)&(p[...,0]<0.06)&(p[...,1]>-0.06)&(p[...,1]<0.2)
Q=p[m]; print('mug pts',m.sum(),'x %.3f..%.3f y %.3f..%.3f zmax %.4f'%(Q[:,0].min(),Q[:,0].max(),Q[:,1].min(),Q[:,1].max(),Q[:,2].max()))
for lo in np.arange(0.45,0.58,0.02):
    s=(Q[:,2]>=lo)&(Q[:,2]<lo+0.02)
    if s.sum(): print('  z %.2f n %4d x %.3f..%.3f y %.3f..%.3f'%(lo,s.sum(),Q[s,0].min(),Q[s,0].max(),Q[s,1].min(),Q[s,1].max()))
"

# openrua op 83
python3 - <<'EOF'
s=open('stage8.py').read()
s=s.replace("BETA = math.radians(35.0)","BETA = math.radians(0.0)   # attempt 3: horizontal hand, vertical fingers")
s=s.replace("PERP = 0.01\nzg = ZC + math.sqrt((R0 - WALL / 2) ** 2 - PERP ** 2)","PERP = 0.0\nzg = ZC + R0 + 0.0065   # TCP a bit above wall mid so hand shell clears table")
s=s.replace("MOUTH = 0.074","MOUTH = 0.072")
s=s.replace("G = c + (MOUTH - 0.02) * a + PERP * n + np.array([0, 0, zg])","G = c + (MOUTH - 0.015) * a + PERP * n + np.array([0, 0, zg])")
s=s.replace("PRE = G + np.array([0, 0, 0.06]) - 0.02 * d1","PRE = G - 0.07 * d1   # straight back along the approach axis")
s=s.replace("HI = PRE + np.array([0, 0, 0.22])","HI = PRE + np.array([0, 0, 0.27])")
open('stage8.py','w').write(s)
EOF
python3 -c "
import math,numpy as np
from stage8 import *
Rf=rotated(R1,-math.pi/2)
print('G',G.round(4),'PRE',PRE.round(4),'LIFT',LIFT.round(4)); print('R1 quat',np.round(R_quat(R1),4),'Rf quat',np.round(R_quat(Rf),4)); print('a->',(rot_axis(n,-math.pi/2)@a).round(3))"

# openrua op 84
cat > probe8d.py <<'EOF'
import math, numpy as np
from arm import Arm, HOME, R_quat
from stage8 import G, PRE, LIFT, R1, rotated, STEPS, TARGET_AXIS, TABLE
arm = Arm("probe8d")
cur = np.array(arm.arm_q())
R = R1
HI = PRE + np.array([0, 0, 0.27])
qHI = arm.ik_best(HI, R_quat(R)); print("HI", None if qHI is None else (np.round(qHI, 2), round(float(np.abs(qHI - cur).max()), 2)))
seed = qHI
Rf = rotated(R, -math.pi / 2)
P = np.array([TARGET_AXIS[0], TARGET_AXIS[1] - 0.05, TABLE + 0.133])
chain = [("pre", PRE, R_quat(R)), ("G", G, R_quat(R)), ("lift", LIFT, R_quat(R))]
chain += [(f"rot{k}", LIFT, R_quat(rotated(R, -k * math.pi / 2 / STEPS))) for k in range(1, STEPS + 1)]
chain += [("over", np.array([P[0], P[1], LIFT[2]]), R_quat(Rf)), ("place", P, R_quat(Rf)), ("up", P + [0, 0, 0.15], R_quat(Rf))]
for name, xyz, quat in chain:
    s = arm.ik_world(xyz, quat, seed=seed, tries=2)
    if s is None:
        print(name, "IK FAIL", np.round(xyz, 3)); break
    print(name, np.round(xyz, 3), "dq=%.3f" % np.abs(np.array(s) - np.array(seed)).max(), np.round(s, 2))
    seed = s
EOF
timeout 600 python3 -u probe8d.py 2>&1 | grep -v '^\['

# openrua op 85
python3 - <<'EOF'
s=open('stage8.py').read()
s=s.replace("""        arm.move_path([(HI + (PRE - HI) * i / 3, R_quat(R)) for i in range(1, 4)], secs=5)
        # straight approach along d1 to the grasp point
        arm.move_path([(PRE + (G - PRE) * i / 4, R_quat(R)) for i in range(1, 5)], secs=6)""",
"""        arm.move_path([(HI + (PRE - HI) * i / 4, R_quat(R)) for i in range(1, 5)], secs=7)
        p, q = arm.tcp_world(); print("at pre: tcp", p.round(4), "quat", np.round(q, 3), flush=True)
        # straight approach along d1 to the grasp point
        arm.move_path([(PRE + (G - PRE) * i / 4, R_quat(R)) for i in range(1, 5)], secs=6)""")
open('stage8.py','w').write(s)
EOF
nohup python3 -u stage8.py A > stage8A3.log 2>&1 & sleep 80; grep -v '^\[' /workspace/stage8A3.log

# openrua op 86
python3 - <<'EOF'
s=open('arm.py').read()
s=s.replace("""        if err > 0.03 and _retry:
            print("  -> re-sending goal", flush=True)
            return self.move_joints(q, secs=max(3.0, secs), _retry=False)
        return code""","""        if err > 0.01 and _retry:
            print("  -> re-sending goal", flush=True)
            return self.move_joints(q, secs=max(3.0, secs), _retry=(_retry - 1) if isinstance(_retry, int) and not isinstance(_retry, bool) else 1 if _retry is True else False)
        return code""")
open('arm.py','w').write(s)
EOF
python3 - <<'EOF'
# simpler: rewrite retry logic cleanly
s=open('/workspace/arm.py').read()
s=s.replace("""            return self.move_joints(q, secs=max(3.0, secs), _retry=(_retry - 1) if isinstance(_retry, int) and not isinstance(_retry, bool) else 1 if _retry is True else False)""",
"""            nxt = 2 if _retry is True else _retry - 1   # up to 3 sends total
            return self.move_joints(q, secs=max(3.0, secs), _retry=nxt if nxt > 0 else False)""")
open('/workspace/arm.py','w').write(s)
EOF
cat > regrasp.py <<'EOF'
import numpy as np
from arm import Arm, R_quat
from stage8 import G, R1
arm = Arm("regrasp")
arm.open()
p, _ = arm.tcp_world()
arm.move_path([(p + (G - p) * i / 2, R_quat(R1)) for i in range(1, 3)], secs=4)
p, q = arm.tcp_world(); print("at grasp: tcp", p.round(4), "quat", np.round(q, 3), flush=True)
arm.close(); print("fingers", arm.finger_gap(), flush=True)
EOF
nohup python3 -u regrasp.py > regrasp.log 2>&1 & sleep 45; grep -v '^\[' /workspace/regrasp.log

# openrua op 87
sleep 15; grep -v '^\[' /workspace/regrasp.log | tail -3; python3 scene.py agentview >/dev/null; python3 -c "
import numpy as np
p=np.load('agentview_pts.npy'); z=p[...,2]
m=np.isfinite(z)&(z>0.45)&(z<0.60)&(p[...,0]>-0.08)&(p[...,0]<0.06)&(p[...,1]>-0.06)&(p[...,1]<0.16)
Q=p[m]; print('mug pts',m.sum(),'x %.3f..%.3f y %.3f..%.3f zmax %.4f'%(Q[:,0].min(),Q[:,0].max(),Q[:,1].min(),Q[:,1].max(),Q[:,2].max()))
# hand shell lowest points near y 0.15-0.3
m2=np.isfinite(z)&(z>0.40)&(z<0.50)&(p[...,0]>-0.08)&(p[...,0]<0.04)&(p[...,1]>0.13)&(p[...,1]<0.32)
Q2=p[m2]; print('hand-low pts',m2.sum(), 'zmin %.4f'%Q2[:,2].min() if m2.sum() else '')
"

# openrua op 88
cat > regrasp2.py <<'EOF'
import numpy as np
from arm import Arm, R_quat
from stage8 import G, R1
arm = Arm("regrasp2")
arm.open()
p, _ = arm.tcp_world()
G2 = np.array([G[0], G[1], 0.548])
up = np.array([p[0], p[1], 0.548])
arm.move_path([(up, R_quat(R1))], secs=3)
p, _ = arm.tcp_world()
arm.move_path([(p + (G2 - p) * i / 2, R_quat(R1)) for i in range(1, 3)], secs=4)
p, q = arm.tcp_world(); print("at grasp: tcp", p.round(4), "quat", np.round(q, 3), flush=True)
arm.close(); print("fingers", arm.finger_gap(), flush=True)
EOF
nohup python3 -u regrasp2.py > regrasp2.log 2>&1 & sleep 40; grep -v '^\[' /workspace/regrasp2.log

# openrua op 89
python3 scene.py agentview >/dev/null; python3 -c "
import numpy as np
p=np.load('agentview_pts.npy'); z=p[...,2]
m=np.isfinite(z)&(z>0.45)&(z<0.56)&(p[...,0]>-0.06)&(p[...,0]<0.035)&(p[...,1]>-0.06)&(p[...,1]<0.10)
Q=p[m]
for lo in np.arange(-0.03,0.10,0.02):
    s=(Q[:,1]>=lo)&(Q[:,1]<lo+0.02)
    if s.sum(): print('  y %.2f n %4d x %.3f..%.3f zmax %.4f'%(lo,s.sum(),Q[s,0].min(),Q[s,0].max(),Q[s,2].max()))
"

# openrua op 90
python3 -c "
import cv2
img=cv2.imread('agentview.png'); crop=img[200:360,240:460]; cv2.imwrite('crop_red2.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 91
python3 -c "
import numpy as np
p=np.load('agentview_pts.npy')
for (u,v) in [(390,277),(397,277),(403,277),(410,270),(417,250),(420,260),(300,275),(330,262)]:
    print((u,v), np.round(p[v,u],4))
"

# openrua op 92
python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); z=p[...,2]
img=cv2.imread('agentview.png'); b,g,r=[img[...,i].astype(int) for i in range(3)]
red=(r>80)&(r>1.6*g)&(r>1.6*b)
m=np.isfinite(z)&(z>0.44)&(z<0.60)&red&(p[...,0]<0.06)&(p[...,0]>-0.1)&(p[...,1]>-0.1)&(p[...,1]<0.25)
Q=p[m]; print('red mug pts',m.sum(),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(Q[:,0].min(),Q[:,0].max(),Q[:,1].min(),Q[:,1].max(),Q[:,2].min(),Q[:,2].max()))
for lo in np.arange(-0.04,0.14,0.02):
    s=(Q[:,1]>=lo)&(Q[:,1]<lo+0.02)
    if s.sum(): print('  y %.2f n %4d x %.3f..%.3f z %.3f..%.3f'%(lo,s.sum(),Q[s,0].min(),Q[s,0].max(),Q[s,2].min(),Q[s,2].max()))
# white interior of red mug? light pixels near mouth
wh=(r>150)&(g>150)&(b>150)
m2=np.isfinite(z)&(z>0.44)&(z<0.56)&wh&(p[...,0]<0.06)&(p[...,0]>-0.1)&(p[...,1]>0.05)&(p[...,1]<0.2)
print('white pts near mouth',m2.sum(), (p[m2][:,1].min().round(3),p[m2][:,1].max().round(3)) if m2.sum() else '')
"

# openrua op 93
python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); z=p[...,2]
img=cv2.imread('agentview.png'); b,g,r=[img[...,i].astype(int) for i in range(3)]
red=(r>80)&(r>1.6*g)&(r>1.6*b)&np.isfinite(z)&(z>0.44)&(z<0.6)&(p[...,1]>-0.1)&(p[...,1]<0.3)&(p[...,0]<0.06)
ys,xs=np.where(red); i=np.argmax(xs); print('rightmost red px',xs[i],ys[i],'world',p[ys[i],xs[i]].round(4))
# fingers: grey pixels with z in 0.50-0.58 near y 0.09-0.16
grey=(abs(r-g)<15)&(abs(g-b)<15)&(r>60)&np.isfinite(z)&(z>0.50)&(z<0.60)&(p[...,1]>0.08)&(p[...,1]<0.2)&(p[...,0]<0.05)&(p[...,0]>-0.08)
ys2,xs2=np.where(grey); j=np.argmin(xs2); print('leftmost grey px',xs2[j],ys2[j],'world',p[ys2[j],xs2[j]].round(4), 'n',grey.sum())
print('grey y range', p[grey][:,1].min().round(3), p[grey][:,1].max().round(3), 'z', p[grey][:,2].min().round(3), p[grey][:,2].max().round(3))
"

# openrua op 94
python3 scene.py robot0_eye_in_hand >/dev/null; echo ok

# openrua op 95
python3 -c "
import numpy as np, cv2
p=np.load('agentview_pts.npy'); z=p[...,2]
img=cv2.imread('agentview.png'); b,g,r=[img[...,i].astype(int) for i in range(3)]
red=(r>80)&(r>1.6*g)&(r>1.6*b)&np.isfinite(z)&(z>0.44)&(z<0.60)&(p[...,1]>-0.1)&(p[...,1]<0.3)&(p[...,0]<0.06)&(p[...,0]>-0.1)
Q=p[red]
s=(Q[:,1]>-0.045)&(Q[:,1]<0.0)&(Q[:,2]<0.531)
P=Q[s][:,[0,2]]
A=np.c_[2*P,np.ones(len(P))]; bb=(P**2).sum(1); sol=np.linalg.lstsq(A,bb,rcond=None)[0]; cx,cz=sol[:2]; rr=np.sqrt(sol[2]+cx**2+cz**2)
print('circle fit (x,z) centre',round(cx,4),round(cz,4),'r',round(rr,4),'n',len(P))
# fixed radius 0.05 fit: minimize
best=None
for x0 in np.arange(-0.05,0.02,0.002):
    for z0 in np.arange(0.46,0.49,0.002):
        e=np.abs(np.hypot(P[:,0]-x0,P[:,1]-z0)-0.05).mean()
        if best is None or e<best[0]: best=(e,x0,z0)
print('r=0.05 fit',best)
print('mouth-end y min', Q[:,1].min().round(4), 'pts with y<-0.03:', (Q[:,1]<-0.03).sum())
"

# openrua op 96
python3 - <<'EOF'
s=open('stage8.py').read()
s=s.replace("a = np.array([0.0, 1.0, 0.0]); a /= np.linalg.norm(a)   # attempt 2: mug lies along +y, mouth at +y",
            "a = np.array([0.0, -1.0, 0.0]); a /= np.linalg.norm(a)   # attempt 4: mug lies along y, mouth faces -y")
s=s.replace("c = np.array([-0.02, 0.05, 0.0])","c = np.array([-0.022, 0.034, 0.0])")
s=s.replace("MOUTH = 0.072","MOUTH = 0.074")
s=s.replace("zg = ZC + R0 + 0.0065   # TCP a bit above wall mid so hand shell clears table","zg = 0.549   # hand shell (~0.113 below TCP axis) must clear the table")
s=s.replace("""        arm.open()
        HI = PRE + np.array([0, 0, 0.27])
        arm.move_tcp(HI, R_quat(R), secs=8)""","""        arm.open()
        p0, q0 = arm.tcp_world()
        arm.move_path([(p0 + np.array([0, 0, 0.26]), q0)], secs=5)   # straight up first
        HI = PRE + np.array([0, 0, 0.27])
        arm.move_tcp(HI, R_quat(R), secs=10)""")
open('stage8.py','w').write(s)
EOF
python3 -c "
import math,numpy as np
from stage8 import *
Rf=rotated(R1,-math.pi/2)
print('G',G.round(4),'PRE',PRE.round(4),'LIFT',LIFT.round(4)); print('R1 quat',np.round(R_quat(R1),4),'Rf quat',np.round(R_quat(Rf),4)); print('a->',(rot_axis(n,-math.pi/2)@a).round(3),'Z->',(rot_axis(n,-math.pi/2)@Z).round(3))"

# openrua op 97
sed -i 's/P = np.array(\[TARGET_AXIS\[0\], TARGET_AXIS\[1\] - 0.05, TABLE + 0.133\])/P = np.array([TARGET_AXIS[0], TARGET_AXIS[1] + 0.05, TABLE + 0.133])/' probe8d.py && timeout 600 python3 -u probe8d.py 2>&1 | grep -v '^\['

# openrua op 98
cat > probe8e.py <<'EOF'
import math, numpy as np
from arm import Arm, HOME, R_quat
from stage8 import G, PRE, LIFT, R1, R1b, rotated, STEPS, TARGET_AXIS, TABLE
arm = Arm("probe8e")
cur = np.array(arm.arm_q())
for name0, R in [("R1b", R1b), ("R1", R1)]:
    print("=== ", name0)
    qPRE = arm.ik_best(PRE, R_quat(R), n=8); 
    if qPRE is None: print("PRE fail"); continue
    print("PRE", np.round(qPRE, 2))
    HI = PRE + np.array([0, 0, 0.27])
    qHI = arm.ik_world(HI, R_quat(R), seed=qPRE, tries=2); print("HI", None if qHI is None else (np.round(qHI, 2), "dq from PRE %.2f" % np.abs(np.array(qHI) - np.array(qPRE)).max(), "dq from cur %.2f" % np.abs(np.array(qHI) - cur).max()))
    seed = qPRE
    Rf = rotated(R, -math.pi / 2)
    P = np.array([TARGET_AXIS[0], TARGET_AXIS[1] + 0.05, TABLE + 0.133])
    chain = [("G", G, R_quat(R)), ("lift", LIFT, R_quat(R))]
    chain += [(f"rot{k}", LIFT, R_quat(rotated(R, -k * math.pi / 2 / STEPS))) for k in range(1, STEPS + 1)]
    chain += [("over", np.array([P[0], P[1], LIFT[2]]), R_quat(Rf)), ("place", P, R_quat(Rf)), ("up", P + [0, 0, 0.15], R_quat(Rf))]
    for name, xyz, quat in chain:
        s = arm.ik_world(xyz, quat, seed=seed, tries=2)
        if s is None:
            print(name, "IK FAIL", np.round(xyz, 3)); break
        print(name, np.round(xyz, 3), "dq=%.3f" % np.abs(np.array(s) - np.array(seed)).max(), np.round(s, 2))
        seed = s
EOF
timeout 900 python3 -u probe8e.py 2>&1 | grep -v '^\['

# openrua op 99
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/stage8.py", "old_string": "        HI = PRE + np.array([0, 0, 0.27])\\n        arm.move_tcp(HI, R_quat(R), secs=10)", "new_string": "        HI = PRE + np.array([0, 0, 0.27])\\n        qPRE = arm.ik_best(PRE, R_quat(R), n=8)   # pick the branch at PRE, then HI in that branch\\n        qHI = arm.ik_world(HI, R_quat(R), seed=qPRE)\\n        if qHI is None:\\n            print(\\"IK FAILED for HI\\", flush=True); return\\n        arm.move_joints(qHI, secs=10)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 100
nohup python3 -u stage8.py A b > stage8A4.log 2>&1 &
sleep 100; cat /workspace/stage8A4.log | grep -v '^\['

# openrua op 101
python3 scene.py agentview 2>&1 | grep -v '^\[' ; python3 scene.py robot0_eye_in_hand 2>&1 | grep -v '^\['

# openrua op 102
cat > lifttest.py <<'EOF'
import numpy as np, sys
from arm import Arm
arm = Arm("lifttest")
dz = float(sys.argv[1])
p0, q0 = arm.tcp_world()
arm.move_path([(p0 + np.array([0, 0, dz * i / 3]), q0) for i in range(1, 4)], secs=4)
print("fingers", arm.finger_gap())
EOF
python3 -u lifttest.py 0.05 2>&1 | grep -v '^\[' && python3 scene.py agentview 2>&1 | grep -v '^\['

# openrua op 103
cat > redmug.py <<'EOF'
import numpy as np, cv2
img = cv2.imread("agentview.png"); pts = np.load("agentview_pts.npy")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
m = ((hsv[..., 0] < 8) | (hsv[..., 0] > 170)) & (hsv[..., 1] > 120) & (hsv[..., 2] > 60)
P = pts[m]; P = P[np.isfinite(P).all(1)]
P = P[(P[:, 2] > 0.43) & (P[:, 0] < 0.1)]   # exclude the plate stripes region roughly? keep x<0.1
print("n", len(P))
print("x range", P[:, 0].min().round(3), P[:, 0].max().round(3))
print("y range", P[:, 1].min().round(3), P[:, 1].max().round(3))
print("z range", P[:, 2].min().round(3), P[:, 2].max().round(3))
# body points (no handle): z<0.53; fit circle in xz for points with y in body band
B = P[(P[:, 2] < 0.53)]
from numpy.linalg import lstsq
A = np.c_[2 * B[:, 0], 2 * B[:, 2], np.ones(len(B))]; b = B[:, 0] ** 2 + B[:, 2] ** 2
s = lstsq(A, b, rcond=None)[0]; r = np.sqrt(s[2] + s[0] ** 2 + s[1] ** 2)
print("circle fit xz: cx %.4f cz %.4f r %.4f" % (s[0], s[1], r))
for ylo in np.arange(-0.08, 0.14, 0.02):
    S = P[(P[:, 1] >= ylo) & (P[:, 1] < ylo + 0.02)]
    if len(S): print("y %.2f..%.2f n %4d zmax %.3f xmin %.3f xmax %.3f" % (ylo, ylo + 0.02, len(S), S[:, 2].max(), S[:, 0].min(), S[:, 0].max()))
EOF
python3 redmug.py

# openrua op 104
cat >> redmug.py <<'EOF'
H = P[(P[:, 1] > -0.02) & (P[:, 1] < 0.04) & (P[:, 0] > 0.03)]
print("handle-ish: n", len(H), "x", H[:, 0].min().round(3), H[:, 0].max().round(3), "z", H[:, 2].min().round(3), H[:, 2].max().round(3))
for xlo in np.arange(-0.04, 0.10, 0.02):
    S = P[(P[:, 0] >= xlo) & (P[:, 0] < xlo + 0.02) & (P[:, 1] > -0.02) & (P[:, 1] < 0.04)]
    if len(S): print("x %.2f..%.2f n %4d z %.3f..%.3f" % (xlo, xlo + 0.02, len(S), S[:, 2].min(), S[:, 2].max()))
# mouth end: points y<-0.02
Mo = P[P[:, 1] < -0.025]
print("mouth end n", len(Mo), "x", Mo[:, 0].min().round(3), Mo[:, 0].max().round(3), "z", Mo[:, 2].min().round(3), Mo[:, 2].max().round(3), "ymin", Mo[:, 1].min().round(3))
EOF
python3 redmug.py | tail -12

# openrua op 105
sed -i 's/^zg = 0.549 .*/zg = 0.541   # hand shell (~0.113 below TCP axis) just clears the table; lower finger must enter below the inner rim top (0.521)/' stage8.py && grep -n "^zg" stage8.py && nohup python3 -u stage8.py A b > stage8A5.log 2>&1 &
sleep 95; grep -v '^\[' /workspace/stage8A5.log

# openrua op 106
python3 -u lifttest.py 0.05 2>&1 | grep -v '^\[' && python3 scene.py agentview 2>&1 | grep -v '^\[' && python3 redmug.py | head -5

# openrua op 107
python3 -u lifttest.py 0.05 2>&1 | grep -v '^\[' && python3 scene.py agentview 2>&1 | grep -v '^\[' && python3 redmug.py | grep -E "range|mouth end"

# openrua op 108
nohup python3 -u stage8.py B b > stage8B4.log 2>&1 &
sleep 60; grep -v '^\[' /workspace/stage8B4.log

# openrua op 109
python3 scene.py agentview 2>&1 | grep -v '^\[' && python3 scene.py birdview 2>&1 | grep -v '^\[' && python3 redmug.py | grep -E "range"

# openrua op 110
cat > hang.py <<'EOF'
import numpy as np, cv2
from numpy.linalg import lstsq
for cam in ["birdview", "agentview"]:
    img = cv2.imread(f"{cam}.png"); pts = np.load(f"{cam}_pts.npy")
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = ((hsv[..., 0] < 8) | (hsv[..., 0] > 170)) & (hsv[..., 1] > 120) & (hsv[..., 2] > 60)
    P = pts[m]; P = P[np.isfinite(P).all(1)]
    P = P[(P[:, 2] > 0.58) & (np.abs(P[:, 0] + 0.02) < 0.12) & (np.abs(P[:, 1] + 0.03) < 0.12)]
    print(cam, "n", len(P), "x", P[:, 0].min().round(3), P[:, 0].max().round(3), "y", P[:, 1].min().round(3), P[:, 1].max().round(3), "z", P[:, 2].min().round(3), P[:, 2].max().round(3))
    for zlo in np.arange(0.60, 0.76, 0.02):
        S = P[(P[:, 2] >= zlo) & (P[:, 2] < zlo + 0.02)]
        if len(S) > 20:
            A = np.c_[2 * S[:, 0], 2 * S[:, 1], np.ones(len(S))]; b = S[:, 0] ** 2 + S[:, 1] ** 2
            s = lstsq(A, b, rcond=None)[0]; r = np.sqrt(max(s[2] + s[0] ** 2 + s[1] ** 2, 0))
            print("  z %.2f n %4d x %.3f..%.3f y %.3f..%.3f  fit c(%.3f,%.3f) r %.3f" % (zlo, len(S), S[:, 0].min(), S[:, 0].max(), S[:, 1].min(), S[:, 1].max(), s[0], s[1], r))
EOF
python3 hang.py

# openrua op 111
cat > hang2.py <<'EOF'
import numpy as np, cv2
img = cv2.imread("agentview.png"); pts = np.load("agentview_pts.npy")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
m = ((hsv[..., 0] < 8) | (hsv[..., 0] > 170)) & (hsv[..., 1] > 120) & (hsv[..., 2] > 60)
P = pts[m]; P = P[np.isfinite(P).all(1)]
P = P[(P[:, 2] > 0.58) & (np.abs(P[:, 0] + 0.02) < 0.12) & (np.abs(P[:, 1] + 0.03) < 0.12)]
for zlo in np.arange(0.60, 0.76, 0.01):
    S = P[(P[:, 2] >= zlo) & (P[:, 2] < zlo + 0.01)]
    if len(S) > 20:
        thr = np.quantile(S[:, 0], 0.9)
        F = S[S[:, 0] >= thr]
        print("z %.2f n %4d front x %.3f  y_med %.3f (y span %.3f..%.3f)" % (zlo, len(S), F[:, 0].mean(), np.median(F[:, 1]), S[:, 1].min(), S[:, 1].max()))
EOF
python3 hang2.py

# openrua op 112
nohup python3 -u stage8.py C b -0.004 -0.019 0.132 > stage8C4.log 2>&1 &
sleep 75; grep -v '^\[' /workspace/stage8C4.log

# openrua op 113
sleep 45; grep -v '^\[' /workspace/stage8C4.log | tail -8

# openrua op 114
sleep 25; grep -v '^\[' /workspace/stage8C4.log | tail -3; python3 scene.py agentview 2>&1 | grep -v '^\['

# openrua op 115
python3 -u stage8.py home 2>&1 | grep -v '^\[' && python3 scene.py agentview 2>&1 | grep -v '^\[' && python3 - <<'EOF'
import numpy as np, cv2
img = cv2.imread("agentview.png"); pts = np.load("agentview_pts.npy")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
def stats(name, m):
    P = pts[m]; P = P[np.isfinite(P).all(1)]; P = P[P[:, 2] > 0.43]
    print(f"{name}: n={len(P)} x {P[:,0].min():.3f}..{P[:,0].max():.3f} y {P[:,1].min():.3f}..{P[:,1].max():.3f} z {P[:,2].min():.3f}..{P[:,2].max():.3f}")
    return P
red = ((hsv[..., 0] < 8) | (hsv[..., 0] > 170)) & (hsv[..., 1] > 120) & (hsv[..., 2] > 60)
R = stats("red mug", red & (pts[..., 0] < 0.08))
# white mug: low saturation, bright, above table
white = (hsv[..., 1] < 40) & (hsv[..., 2] > 150)
W = stats("white (mug+plate)", white & (pts[..., 2] > 0.44) & (np.abs(pts[..., 1]) < 0.12) & (pts[..., 0] > 0.05) & (pts[..., 0] < 0.3))
Wm = W[W[:, 2] > 0.47]
print("white mug body centre xy ~", Wm[:, 0].mean().round(3), Wm[:, 1].mean().round(3), "top z", Wm[:, 2].max().round(3))
pl = W[(W[:, 2] < 0.445)]
print("plate pts x %.3f..%.3f y %.3f..%.3f" % (pl[:, 0].min(), pl[:, 0].max(), pl[:, 1].min(), pl[:, 1].max()))
EOF

# openrua op 116
pgrep -af "stage8|lifttest|regrasp|probe8" || echo "no scripts running"
