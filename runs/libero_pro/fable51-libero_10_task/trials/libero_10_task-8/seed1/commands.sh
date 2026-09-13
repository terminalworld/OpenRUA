#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 node list

# openrua op 3
for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 4
timeout 60 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80

# openrua op 5
mkdir -p "$(dirname /workspace/scene3d.py)"
cat > /workspace/scene3d.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Project a whole camera depth frame into world coords and report
objects: usage scene3d.py <camera>  -> saves <camera>_world.npy (HxWx3)
and prints table-height stats."""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene3d")
    buf = Buffer(); TransformListener(buf, node)
    d = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    v, u = np.mgrid[0:d.height, 0:d.width]
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + tr
    np.save(f"{cam}_world.npy", P)
    print("cam pos", tr, "depth range", np.nanmin(depth), np.nanmax(depth))
    z = P[..., 2]
    print("z percentiles", np.nanpercentile(z, [5, 25, 50, 75, 95]))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 6
timeout 120 python3 scene3d.py agentview && timeout 120 python3 scene3d.py birdview

# openrua op 7
python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); z=P[...,2]
mask=(z>0.91)&(z<1.3)&(P[...,0]>-0.3)  # exclude robot area roughly
import scipy.ndimage as nd
lab,n=nd.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<15: continue
    pts=P[m]
    vs,us=np.where(m)
    print(i,m.sum(),'px(u,v)=',us.mean().round(),vs.mean().round(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'zmax',pts[:,2].max().round(3))
"


# openrua op 8
python3 -c "
import numpy as np
P=np.load('agentview_world.npy')
for (u,v) in [(470,275),(355,300),(205,290),(470,300),(355,330)]:
    print((u,v),P[v,u].round(3))
"


# openrua op 9
timeout 120 python3 scene3d.py sideview; timeout 120 python3 scene3d.py frontview; python3 -c "
import numpy as np
for cam in ['agentview','sideview','frontview']:
    P=np.load(cam+'_world.npy')
    m=(P[...,0]>-0.06)&(P[...,0]<0.12)&(P[...,1]>-0.06)&(P[...,1]<0.16)&(P[...,2]>0.905)&(P[...,2]<1.1)
    pts=P[m]
    print(cam,len(pts))
    for z0 in np.arange(0.91,1.06,0.01):
        s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
        if len(s)==0: continue
        print(f' z{z0:.2f} n{len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
"


# openrua op 10
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library: one node, FK/IK/trajectory/gripper/joint reads.
World<->base offset comes from TF world->panda_link0 (constant here).

CLI:
  robot.py fk                         current hand + tcp pose in world
  robot.py goto X Y Z QX QY QZ QW [sec] [--hand]   TCP (default) target in WORLD
  robot.py grip open|close
  robot.py js
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
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.660, 0.0, 0.912])  # tf world->panda_link0


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_state(self):
        j = self.joints()
        js = JointState()
        js.name = list(ARM)
        js.position = [j[n] for n in ARM]
        return js

    def fk_pose(self):
        """hand pose in world: (pos, quat xyzw)"""
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_state()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, q

    def solve_ik(self, pos_world, q, at_tcp=True, seed=None):
        pos = np.array(pos_world, float)
        if at_tcp:
            R = quat_R(*q)
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE_IN_WORLD
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = pos
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        req.ik_request.robot_state.joint_state = seed or self.arm_state()
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"IK failed code={code} for {pos_world}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, positions, seconds=3.0):
        self.traj.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        j = self.joints()
        err = max(abs(j[n] - p) for n, p in zip(ARM, positions))
        print(f"traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, pos_world, q, seconds=3.0, at_tcp=True):
        sol = self.solve_ik(pos_world, q, at_tcp)
        return self.move_joints(sol, seconds)

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        j = self.joints()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={j['panda_finger_joint1']:.4f},{j['panda_finger_joint2']:.4f}")
        return j["panda_finger_joint1"]


if __name__ == "__main__":
    r = Robot()
    cmd = sys.argv[1]
    if cmd == "fk":
        pos, q = r.fk_pose()
        R = quat_R(*q)
        print("hand world", pos.round(4), "quat", np.round(q, 4))
        print("tcp world", (pos + TCP * R[:, 2]).round(4))
        print("hand axes (cols x,y,z in world):\n", R.round(3))
    elif cmd == "goto":
        a = [x for x in sys.argv[2:] if not x.startswith("--")]
        v = list(map(float, a))
        sec = v[7] if len(v) > 7 else 3.0
        r.goto(v[:3], v[3:7], sec, at_tcp="--hand" not in sys.argv)
        pos, q = r.fk_pose()
        R = quat_R(*q)
        print("now tcp world", (pos + TCP * R[:, 2]).round(4))
    elif cmd == "grip":
        r.gripper(GRIP["open_m"] if sys.argv[2] == "open" else GRIP["closed_m"])
    elif cmd == "js":
        print(r.joints())
    rclpy.shutdown()
OPENRUA_EOF

# openrua op 11
timeout 120 python3 robot.py fk

# openrua op 12
timeout 120 python3 -c "
from robot import *
r=Robot()
req=GetPositionFK.Request(); req.fk_link_names=['panda_link0','panda_link8','panda_hand']
req.robot_state.joint_state=r.arm_state()
fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60)
res=fut.result()
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position, ps.pose.orientation)
print(res.error_code)
print(r.joints())
"

# openrua op 13
sed -i 's/pos = np.array(\[p.position.x, p.position.y, p.position.z\]) + BASE_IN_WORLD/pos = np.array([p.position.x, p.position.y, p.position.z])  # FK answers in world/' robot.py && timeout 200 python3 -c "
from robot import *
r=Robot()
pos,q=r.fk_pose(); print('hand world',pos,q)
cur=[r.joints()[n] for n in ARM]
for label,off in [('world-coords',np.zeros(3)),('base-coords',-BASE_IN_WORLD)]:
    try:
        # bypass BASE offset in solve_ik by pre-adding it
        sol=r.solve_ik(pos+off+BASE_IN_WORLD,q,at_tcp=False)
        print(label,'sol',np.round(sol,3),'diff from current',np.round(np.abs(np.array(sol)-cur).max(),3))
    except Exception as e: print(label,'ERR',e)
"

# openrua op 14
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "        pos = pos - BASE_IN_WORLD\\n        self.ik.wait_for_service(10)", "new_string": "        # verified: this machine\'s IK (empty frame_id) takes WORLD coords\\n        self.ik.wait_for_service(10)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 15
timeout 300 python3 robot.py grip open && timeout 600 python3 robot.py goto 0.028 0.058 1.16 0.7071068 0.7071068 0 0 4

# openrua op 16
cat >> robot.py <<'EOF'


def goto_converge(r, pos, q, seconds=3.0, at_tcp=True, tol=0.01, tries=4):
    sol = r.solve_ik(pos, q, at_tcp)
    for i in range(tries):
        code, err = r.move_joints(sol, seconds)
        if err < tol:
            break
    p, qq = r.fk_pose()
    tcp = p + TCP * quat_R(*qq)[:, 2]
    print("tcp world", tcp.round(4), "err", round(err, 4))
    return tcp, err
EOF
timeout 900 python3 -c "
from robot import *
r=Robot()
print(r.joints())
goto_converge(r,[0.028,0.058,1.16],(0.7071068,0.7071068,0,0),4)
"

# openrua op 17
python3 - <<'EOF'
src = open('robot.py').read()
main_idx = src.index('if __name__ == "__main__":')
tail_idx = src.index('\n\ndef goto_converge')
func = src[tail_idx:]
src = src[:main_idx].rstrip('\n') + '\n' + func.rstrip('\n') + '\n\n\n' + src[main_idx:tail_idx].rstrip('\n') + '\n'
src = src.replace("p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q",
                  "p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)")
open('robot.py','w').write(src)
EOF
python3 -c "import ast;ast.parse(open('/workspace/robot.py').read());print('ok')"

# openrua op 18
timeout 900 python3 -c "
from robot import *
r=Robot()
print(r.joints())
goto_converge(r,[0.028,0.058,1.16],(0.7071068,0.7071068,0,0),4)
"

# openrua op 19
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 20
timeout 120 python3 robot.py fk

# openrua op 21
python3 - <<'EOF'
src = open('/workspace/robot.py').read()
src = src.replace('''def quat_R(x, y, z, w):''', '''def quat_mul(a, b):
    """Hamilton product, quats as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + bx * 0 + ax * bw + ay * bz - az * by,
            aw * by + ay * bw + az * bx - ax * bz,
            aw * bz + az * bw + ax * by - ay * bx,
            aw * bw - ax * bx - ay * by - az * bz)


# the IK service solves for panda_link8; panda_hand is link8 rotated -45deg
# about z, so a desired HAND orientation must be converted to link8's.
Q_HAND_TO_LINK8 = (0.0, 0.0, 0.3826834323650898, 0.9238795325112867)


def quat_R(x, y, z, w):''')
src = src.replace('''        pos = np.array(pos_world, float)
        if at_tcp:''', '''        pos = np.array(pos_world, float)
        q = tuple(map(float, q))
        q8 = quat_mul(q, Q_HAND_TO_LINK8)  # ik target link is panda_link8
        if at_tcp:''')
src = src.replace("p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)",
                  "p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q8")
open('/workspace/robot.py','w').write(src)
EOF
python3 -c "
from robot import quat_mul
print(quat_mul((0.9996,0,-0.0284,0),(0,0,0.3827,0.9239)))"

# openrua op 22
timeout 900 python3 -c "
from robot import *
r=Robot()
goto_converge(r,[0.028,0.058,1.16],(0.7071068,0.7071068,0,0),3)
p,q=r.fk_pose(); print(quat_R(*q).round(3))
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 23
timeout 120 python3 scene3d.py robot0_eye_in_hand && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
print('knob px', P[305,316].round(4))
# pot top cluster: points with z>1.0 near the pot
m=(P[...,2]>1.0)&(P[...,2]<1.1)&(np.abs(P[...,0]-0.03)<0.08)&(np.abs(P[...,1]-0.06)<0.08)
pts=P[m]; print(len(pts), 'top rim (z>1.02) center', pts[pts[:,2]>1.02][:,:2].mean(0).round(4), 'x rng',pts[:,0].min().round(3),pts[:,0].max().round(3),'y rng',pts[:,1].min().round(3),pts[:,1].max().round(3))
k=pts[pts[:,2]>1.045]; print('knob', len(k), k[:,:2].mean(0).round(4), k[:,2].max().round(3))
"

# openrua op 24
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
for z0,z1 in [(1.02,1.045),(1.0,1.02),(0.98,1.0),(0.96,0.98),(0.94,0.96),(0.92,0.94),(0.905,0.92)]:
    m=(P[...,2]>z0)&(P[...,2]<=z1)&(np.abs(P[...,0]-0.03)<0.1)&(np.abs(P[...,1]-0.06)<0.05)
    pts=P[m]
    if len(pts)<5: print(z0,z1,len(pts)); continue
    print(f'z {z0}-{z1} n{len(pts)} x pct1/99 {np.percentile(pts[:,0],[1,99]).round(3)} y pct1/99 {np.percentile(pts[:,1],[1,99]).round(3)}')
"

# openrua op 25
cat >> robot.py <<'EOF'


def wrench(r):
    from geometry_msgs.msg import WrenchStamped
    port = next(s for s in M["sensors"] if s["kind"] == "wrench")["port"]
    got = {}
    sub = r.node.create_subscription(WrenchStamped, port, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        r.spin(0.2)
    r.node.destroy_subscription(sub)
    w = got["m"].wrench
    return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]).round(3)
EOF
timeout 900 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
print('wrench0',wrench(r))
goto_converge(r,[0.025,0.060,1.04],Q,2)
print('wrench1',wrench(r))
goto_converge(r,[0.025,0.060,0.965],Q,2)
print('wrench2',wrench(r))
" && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 26
timeout 900 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
goto_converge(r,[0.025,0.060,1.12],Q,2)
print('wrench',wrench(r))
print(r.joints())
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 scene3d.py robot0_eye_in_hand && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
m=(P[...,2]>1.02)&(P[...,2]<1.1)&(np.abs(P[...,0]-0.03)<0.1)&(np.abs(P[...,1]-0.06)<0.06)
pts=P[m]; print(len(pts),'rim x pct', np.percentile(pts[:,0],[1,50,99]).round(4),'y pct',np.percentile(pts[:,1],[1,50,99]).round(4),'zmax',pts[:,2].max().round(3))
k=pts[pts[:,2]>1.045]; print('knob',k[:,:2].mean(0).round(4))
"

# openrua op 27
timeout 1200 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
r.gripper(0.0)
goto_converge(r,[0.12,0.060,1.05],Q,2)
goto_converge(r,[0.12,0.060,0.97],Q,2)
w0=wrench(r); print('base wrench',w0)
for x in [0.10,0.085,0.075,0.068,0.062,0.056,0.05]:
    tcp,err=goto_converge(r,[x,0.060,0.97],Q,1.5,tries=2)
    w=wrench(r); print('x',x,'tcp',tcp.round(4),'dF',(w-w0)[:3])
    if abs((w-w0)[0])>2 or abs(tcp[0]-x)>0.008:
        print('CONTACT'); break
"


# openrua op 28
timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 scene3d.py agentview && timeout 120 python3 scene3d.py sideview && python3 -c "
import numpy as np
for cam in ['agentview','sideview']:
    P=np.load(cam+'_world.npy')
    # gripper region: x in 0.08..0.2, y -0.02..0.14, z 0.95..1.15
    m=(P[...,0]>0.08)&(P[...,0]<0.25)&(P[...,1]>-0.03)&(P[...,1]<0.15)&(P[...,2]>0.93)&(P[...,2]<1.3)
    pts=P[m]
    print(cam,len(pts))
    for z0 in np.arange(0.93,1.12,0.01):
        s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
        if len(s): print(f'  z{z0:.2f} n{len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
"

# openrua op 29
timeout 1200 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
sol=r.solve_ik([0.13,-0.05,0.97],Q)
print('ik sol',np.round(sol,3))
code,err=r.move_joints(sol,2.5)
code,err=r.move_joints(sol,2.5)
j=r.joints(); print('actual',np.round([j[n] for n in ARM],3))
print('per-joint err',np.round(np.array([j[n] for n in ARM])-sol,3))
p,q=r.fk_pose(); print('tcp',(p+TCP*quat_R(*q)[:,2]).round(4))
print('wrench',wrench(r))
"


# openrua op 30
cat >> robot.py <<'EOF'


def tcp_now(r):
    p, q = r.fk_pose()
    return p + TCP * quat_R(*q)[:, 2]


def goto_cl(r, target, q, seconds=2.0, tol=0.003, iters=5, at_tcp=True):
    """Closed-loop: command, read FK, offset the command by the residual.
    Compensates the controller's steady-state (gravity) sag."""
    target = np.array(target, float)
    cmd = target.copy()
    for i in range(iters):
        sol = r.solve_ik(cmd, q, at_tcp)
        for _ in range(3):
            code, err = r.move_joints(sol, seconds)
            if err < 0.01:
                break
        actual = tcp_now(r)
        e = target - actual
        print(f"  iter{i} actual {actual.round(4)} resid {np.linalg.norm(e)*1000:.1f}mm")
        if np.linalg.norm(e) < tol:
            break
        cmd = cmd + e
    return actual
EOF
timeout 1500 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
r.gripper(0.04)
goto_cl(r,[0.025,0.060,1.12],Q,2.5)
goto_cl(r,[0.025,0.060,1.04],Q,2)
print('wrench',wrench(r))
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 31
timeout 60 ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 32
timeout 1500 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
w0=wrench(r)
for z in [1.01, 0.985, 0.965]:
    a=goto_cl(r,[0.025,0.060,z],Q,2,iters=4)
    w=wrench(r); print('z',z,'actual',a.round(4),'dF',(w-w0)[:3])
    if abs((w-w0)[2])>8: print('CONTACT - stop'); break
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 33
timeout 120 python3 tools/perception/cam_snap.py agentview; timeout 120 python3 tools/perception/cam_snap.py sideview

# openrua op 34
timeout 120 python3 tools/perception/cam_snap.py birdview; timeout 120 python3 scene3d.py birdview >/dev/null; timeout 120 python3 scene3d.py sideview >/dev/null; timeout 120 python3 scene3d.py agentview > /dev/null; python3 -c "
import numpy as np
import scipy.ndimage as nd
P=np.load('birdview_world.npy'); z=P[...,2]
mask=(z>0.905)&(z<1.2)&(P[...,0]>-0.15)&(P[...,0]<0.2)&(P[...,1]>-0.12)&(P[...,1]<0.15)
lab,n=nd.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<15: continue
    pts=P[m]
    print(i,m.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
"


# openrua op 35
timeout 900 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
goto_cl(r,[0.025,0.060,1.20],Q,2.5,iters=2)
goto_cl(r,[-0.15,0.20,1.25],Q,3,iters=2)
" && for c in agentview birdview sideview frontview; do timeout 120 python3 tools/perception/cam_snap.py $c; timeout 120 python3 scene3d.py $c > /dev/null; done

# openrua op 36
python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); z=P[...,2]
m=(z>0.905)&(z<1.05)&(P[...,0]>-0.05)&(P[...,0]<0.15)&(P[...,1]>-0.12)&(P[...,1]<0.10)
pts=P[m]
print(len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'zmax',pts[:,2].max().round(3))
# profile along y: for each y bin, x extent and zmax
for y0 in np.arange(-0.12,0.10,0.01):
    s=pts[(pts[:,1]>=y0)&(pts[:,1]<y0+0.01)]
    if len(s): print(f' y{y0:.2f} n{len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] zmax {s[:,2].max():.3f}')
"


# openrua op 37
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[250:360,230:400]; cv2.imwrite('bird_crop.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); crop=im[240:380,200:400]; cv2.imwrite('agent_crop.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 38
python3 -c "
import numpy as np
P=np.load('frontview_world.npy')
m=(P[...,2]>0.903)&(P[...,2]<1.06)&(P[...,0]>-0.05)&(P[...,0]<0.10)&(P[...,1]>-0.125)&(P[...,1]<0.08)
pts=P[m]
print(len(pts))
for y0 in np.arange(-0.12,0.08,0.01):
    s=pts[(pts[:,1]>=y0)&(pts[:,1]<y0+0.01)]
    if len(s): print(f' y{y0:.2f} n{len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}] xmax_at_z {s[np.argmax(s[:,0]),2]:.3f}')
"


# openrua op 39
timeout 1500 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
w0=wrench(r)
goto_cl(r,[0.027,0.012,1.10],Q,3,iters=2)
for z in [1.02,0.975,0.935]:
    a=goto_cl(r,[0.027,0.012,z],Q,2,iters=3)
    w=wrench(r); print('z',z,'dF',(w-w0)[:3])
    if abs((w-w0)[2])>8: print('CONTACT'); break
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 40
timeout 600 python3 -c "
from robot import *
r=Robot()
r.gripper(0.0)
import time
for i in range(3):
    r.move_joints([r.joints()[n] for n in ARM],0.5)  # advance sim time to settle
    j=r.joints(); print('fingers',round(j['panda_finger_joint1'],4),round(j['panda_finger_joint2'],4))
print('wrench',wrench(r))
"

# openrua op 41
timeout 600 python3 -c "
from robot import *
r=Robot()
for i in range(4):
    r.move_joints([r.joints()[n] for n in ARM],1.0)
    j=r.joints(); print('fingers',round(j['panda_finger_joint1'],4),round(j['panda_finger_joint2'],4), 'wrench',wrench(r)[:3])
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 42
timeout 900 python3 -c "
from robot import *
r=Robot()
for i in range(3):
    r.move_joints([r.joints()[n] for n in ARM],3.0)
    j=r.joints(); print('fingers',round(j['panda_finger_joint1'],4),round(j['panda_finger_joint2'],4), 'wrench',wrench(r)[:3])
"

# openrua op 43
timeout 900 python3 -c "
from robot import *
r=Robot()
last=None
for i in range(6):
    r.move_joints([r.joints()[n] for n in ARM],3.0)
    j=r.joints(); f=round(j['panda_finger_joint1'],4); print('fingers',f,round(j['panda_finger_joint2'],4), 'wrench',wrench(r)[:3])
    if last is not None and abs(f-last)<0.0003: break
    last=f
"

# openrua op 44
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 45
timeout 900 python3 -c "
from robot import *
r=Robot()
last=None
for i in range(8):
    r.move_joints([r.joints()[n] for n in ARM],3.0)
    j=r.joints(); f=round(j['panda_finger_joint1'],4); f2=round(j['panda_finger_joint2'],4); print('fingers',f,f2,'gap',round(f-f2,4), 'wrench',wrench(r)[:3])
    if last is not None and abs((f-f2)-last)<0.0003: break
    last=f-f2
"

# openrua op 46
timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 47
timeout 900 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
goto_cl(r,[0.027,0.012,1.15],Q,2.5,iters=2)
goto_cl(r,[-0.15,0.20,1.25],Q,3,iters=2)
r.gripper(0.04)
" && for c in agentview birdview frontview; do timeout 120 python3 tools/perception/cam_snap.py $c; timeout 120 python3 scene3d.py $c > /dev/null; done

# openrua op 48
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[250:360,230:400]; cv2.imwrite('bird_crop.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); crop=im[240:380,200:400]; cv2.imwrite('agent_crop.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('frontview.png'); crop=im[300:430,150:400]; cv2.imwrite('front_crop.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 49
timeout 1500 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
goto_cl(r,[0.0325,-0.015,1.12],Q,3,iters=2)
goto_cl(r,[0.0325,-0.015,1.03],Q,2,iters=3)
print('wrench',wrench(r))
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 scene3d.py robot0_eye_in_hand && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
m=(P[...,2]>0.93)&(P[...,2]<1.02)&(np.abs(P[...,0]-0.03)<0.08)&(P[...,1]>-0.05)&(P[...,1]<0.05)
pts=P[m]
for y0 in np.arange(-0.05,0.05,0.01):
    s=pts[(pts[:,1]>=y0)&(pts[:,1]<y0+0.01)]
    if len(s)>5: print(f' y{y0:.2f} n{len(s):4d} x pct0.5/99.5 {np.percentile(s[:,0],[0.5,99.5]).round(4)} zmax {s[:,2].max():.3f}')
"

# openrua op 50
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
# finger points: z between 1.0 and 1.13, y within +-0.03 of -0.015
m=(P[...,2]>1.0)&(P[...,2]<1.13)&(np.abs(P[...,1]+0.015)<0.03)
pts=P[m]
print(len(pts))
import scipy.ndimage as nd
lab,n=nd.label(m)
for i in range(1,n+1):
    s=P[lab==i]
    if len(s)<50: continue
    print(i,len(s),'x',np.percentile(s[:,0],[1,50,99]).round(4),'y',np.percentile(s[:,1],[1,50,99]).round(4),'z',np.percentile(s[:,2],[1,50,99]).round(4))
"

# openrua op 51
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
for (u,v) in [(150,395),(170,385),(195,385),(440,385),(470,385),(495,395),(320,395),(320,340),(350,330),(350,250)]:
    print((u,v),P[v,u].round(4))
"

# openrua op 52
timeout 1500 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
w0=wrench(r)
goto_cl(r,[0.046,-0.015,1.03],Q,2,iters=3)
for z in [1.0,0.98,0.966]:
    a=goto_cl(r,[0.046,-0.015,z],Q,1.5,iters=3)
    w=wrench(r); print('z',z,'dF',(w-w0)[:3])
    if abs((w-w0)[2])>8: print('CONTACT'); break
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 53
timeout 1500 python3 -c "
from robot import *
import math
r=Robot()
Q=(0.7071068,0.7071068,0,0)
th=math.radians(12.6)
Qy=quat_mul((0,0,math.sin(th/2),math.cos(th/2)),Q)
print('Qy',np.round(Qy,4), 'finger axis', quat_R(*Qy)[:,1].round(3))
w0=wrench(r)
goto_cl(r,[0.0455,-0.015,1.03],Qy,2,iters=3)
for z in [1.0,0.985,0.972]:
    a=goto_cl(r,[0.0455,-0.015,z],Qy,1.5,iters=3)
    w=wrench(r); print('z',z,'dF',(w-w0)[:3])
    if abs((w-w0)[2])>8: print('CONTACT'); break
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 54
timeout 1500 python3 -c "
from robot import *
import math
r=Robot()
Q=(0.7071068,0.7071068,0,0)
th=math.radians(12.6)
Qy=quat_mul((0,0,math.sin(th/2),math.cos(th/2)),Q)
w0=wrench(r); print('base',w0[:3])
for z in [0.985,0.972]:
    a=goto_cl(r,[0.0455,-0.015,z],Qy,1.5,iters=3)
    w=wrench(r); print('z',z,'dF',(w-w0)[:3])
    if abs((w-w0)[2])>8: print('CONTACT'); break
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 55
timeout 1200 python3 -c "
from robot import *
r=Robot()
r.gripper(0.0)
last=None
hold=[r.joints()[n] for n in ARM]
for i in range(12):
    r.move_joints(hold,2.0)
    j=r.joints(); f=j['panda_finger_joint1']; f2=j['panda_finger_joint2']; print(f'fingers {f:.4f} {f2:.4f} gap {f-f2:.4f}', 'wrench',wrench(r)[:3])
    if last is not None and abs((f-f2)-last)<0.0003: break
    last=f-f2
"

# openrua op 56
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 tools/perception/cam_snap.py frontview && python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[280:430,150:420]; cv2.imwrite('front_crop.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 57
timeout 1200 python3 -c "
from robot import *
import math
r=Robot()
Q=(0.7071068,0.7071068,0,0)
th=math.radians(12.6)
Qy=quat_mul((0,0,math.sin(th/2),math.cos(th/2)),Q)
goto_cl(r,[0.0435,-0.013,1.01],Qy,2,iters=2)
j=r.joints(); print('fingers',round(j['panda_finger_joint1'],4),round(j['panda_finger_joint2'],4),'wrench',wrench(r)[:3])
" && timeout 120 python3 tools/perception/cam_snap.py frontview && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[250:430,150:420]; cv2.imwrite('front_crop.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 58
timeout 1500 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
goto_cl(r,[0.0,0.12,1.05],Q,2.5,iters=2)
w0=wrench(r); print('base',w0[:3])
for z in [0.95,0.935,0.925,0.918,0.912,0.906,0.900]:
    a=goto_cl(r,[0.0,0.12,z],Q,1.5,iters=2,tol=0.002)
    w=wrench(r); print('target z',z,'actual',a.round(4),'dF',(w-w0)[:3])
    if abs((w-w0)[2])>5: print('TABLE CONTACT at tcp z',a[2]); break
goto_cl(r,[0.0,0.12,1.05],Q,2,iters=1)
"

# openrua op 59
timeout 1800 python3 -c "
from robot import *
import math
r=Robot()
Q=(0.7071068,0.7071068,0,0)
th=math.radians(12.6)
Qy=quat_mul((0,0,math.sin(th/2),math.cos(th/2)),Q)
r.gripper(0.04)
goto_cl(r,[0.0455,-0.015,1.06],Qy,3,iters=2)
w0=wrench(r); print('base',w0[:3])
for z in [1.0,0.98,0.965,0.955,0.945]:
    a=goto_cl(r,[0.0455,-0.015,z],Qy,1.5,iters=3)
    w=wrench(r); print('z',z,'actual',a.round(4),'dF',(w-w0)[:3])
    if abs((w-w0)[2])>6: print('CONTACT'); break
j=r.joints(); print('fingers',round(j['panda_finger_joint1'],4),round(j['panda_finger_joint2'],4))
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 60
timeout 120 python3 scene3d.py robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
# pot points (z between 0.905 and 1.0) near TCP
m=(P[...,2]>0.905)&(P[...,2]<1.0)&(np.abs(P[...,1]+0.015)<0.045)&(P[...,0]>-0.03)&(P[...,0]<0.12)
pts=P[m]
for y0 in np.arange(-0.06,0.03,0.01):
    s=pts[(pts[:,1]>=y0)&(pts[:,1]<y0+0.01)]
    if len(s)>5: print(f' y{y0:.2f} n{len(s):4d} x[{s[:,0].min():.4f},{s[:,0].max():.4f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
# finger tip points: z 0.94..0.975, near x=0.0055 or 0.0855
for name,xc in [('finger2',0.0065),('finger1',0.0845)]:
    s=P[(np.abs(P[...,0]-xc)<0.012)&(np.abs(P[...,1]+0.015)<0.03)&(P[...,2]>0.94)&(P[...,2]<1.05)]
    if len(s): print(name,len(s),'x',np.percentile(s[:,0],[1,50,99]).round(4),'z',np.percentile(s[:,2],[1,50,99]).round(4))
"

# openrua op 61
timeout 1800 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
goto_cl(r,[0.0455,-0.015,1.06],Q,2,iters=1)
r.gripper(0.0)
goto_cl(r,[-0.045,-0.015,1.06],Q,2,iters=2)
goto_cl(r,[-0.045,-0.015,0.95],Q,2,iters=3)
j=r.joints(); print('fingers',round(j['panda_finger_joint1'],4))
w0=wrench(r); print('base',w0[:3])
for x in [-0.035,-0.025,-0.015,-0.005,0.0,0.005,0.01,0.015]:
    a=goto_cl(r,[x,-0.015,0.95],Q,1.5,iters=2,tol=0.002)
    w=wrench(r); print('x',x,'actual',a.round(4),'dF',(w-w0)[:3])
    if abs((w-w0)[0])>3 or abs(a[0]-x)>0.004: print('CONTACT at tcp x',a[0]); break
"

# openrua op 62
timeout 1800 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
goto_cl(r,[-0.045,-0.015,1.06],Q,2,iters=1)
r.gripper(0.0)
hold=[r.joints()[n] for n in ARM]
for i in range(30):
    r.move_joints(hold,2.0)
    f=r.joints()['panda_finger_joint1']
    if f<0.003: break
print('fingers closed at',round(f,4))
# top probe at bottom chamber center y=0.02, x center ~0.041 (from wrist cloud 0.0024..0.0735 -> 0.038; earlier 0.037)
goto_cl(r,[0.038,0.02,1.03],Q,2,iters=2)
w0=wrench(r); print('base',w0[:3])
for z in [1.0,0.99,0.985,0.98,0.975,0.97,0.965,0.96]:
    a=goto_cl(r,[0.038,0.02,z],Q,1.2,iters=2,tol=0.002)
    w=wrench(r); print('z',z,'actual',a.round(4),'dF',(w-w0)[:3])
    if abs((w-w0)[2])>3 or abs(a[2]-z)>0.004: print('TOP CONTACT at tcp z',a[2]); break
goto_cl(r,[0.038,0.02,1.03],Q,1.5,iters=1)
"

# openrua op 63
timeout 900 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
goto_cl(r,[-0.15,0.20,1.25],Q,3,iters=1)
" && for c in agentview frontview; do timeout 120 python3 tools/perception/cam_snap.py $c; done; python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[280:440,120:420]; cv2.imwrite('front_crop.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 64
timeout 120 python3 tools/perception/cam_snap.py birdview && timeout 120 python3 scene3d.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
im=cv2.imread('birdview.png'); crop=im[200:360,200:420]; cv2.imwrite('bird_crop.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
P=np.load('birdview_world.npy'); z=P[...,2]
m=(z>0.935)&(z<1.1)&(P[...,0]>-0.25)&(P[...,0]<0.15)&(P[...,1]>-0.12)&(P[...,1]<0.12)
import scipy.ndimage as nd
lab,n=nd.label(m)
for i in range(1,n+1):
    s=P[lab==i]
    if len(s)<15: continue
    print(i,len(s),'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3),'zmax',s[:,2].max().round(3))
"

# openrua op 65
timeout 900 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
goto_cl(r,[-0.16,-0.045,1.15],Q,3,iters=2)
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 scene3d.py robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
m=(P[...,2]>0.905)&(P[...,2]<1.05)&(P[...,0]>-0.28)&(P[...,0]<-0.05)&(P[...,1]>-0.15)&(P[...,1]<0.06)
pts=P[m]
print(len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'zmax',pts[:,2].max().round(3))
# PCA for axis
c=pts[:,:2].mean(0); u,s,vt=np.linalg.svd(pts[:,:2]-c,full_matrices=False); print('center',c.round(4),'axis',vt[0].round(3))
d=vt[0]; t=(pts[:,:2]-c)@d; n=(pts[:,:2]-c)@vt[1]
for t0 in np.arange(t.min(),t.max(),0.01):
    s=(t>=t0)&(t<t0+0.01)
    if s.sum()>20: print(f' t{t0:+.3f} n{s.sum():5d} width {np.percentile(n[s],99.5)-np.percentile(n[s],0.5):.4f} ncenter {np.mean(np.percentile(n[s],[0.5,99.5])):+.4f} zmax {pts[s,2].max():.3f}')
"

# openrua op 66
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
m=(P[...,2]>0.94)&(P[...,2]<1.05)&(P[...,0]>-0.28)&(P[...,0]<-0.05)&(P[...,1]>-0.15)&(P[...,1]<0.06)
pts=P[m]
print(len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'zmax',pts[:,2].max().round(3))
c=pts[:,:2].mean(0); u,s,vt=np.linalg.svd(pts[:,:2]-c,full_matrices=False); print('center',c.round(4),'axis',vt[0].round(3))
d=vt[0]; t=(pts[:,:2]-c)@d; n=(pts[:,:2]-c)@vt[1]
for t0 in np.arange(t.min(),t.max(),0.01):
    s=(t>=t0)&(t<t0+0.01)
    if s.sum()>20: print(f' t{t0:+.3f} n{s.sum():5d} width {np.percentile(n[s],99.5)-np.percentile(n[s],0.5):.4f} ncenter {np.mean(np.percentile(n[s],[0.5,99.5])):+.4f} zmax {pts[s,2].max():.3f}')
"

# openrua op 67
timeout 1800 python3 -c "
from robot import *
r=Robot()
Q=(0.7071068,0.7071068,0,0)
f=r.joints()['panda_finger_joint1']; print('finger',round(f,4))
if f>0.003:
    r.gripper(0.0); hold=[r.joints()[n] for n in ARM]
    for i in range(30):
        r.move_joints(hold,2.0); f=r.joints()['panda_finger_joint1']
        if f<0.003: break
    print('finger now',round(f,4))
goto_cl(r,[-0.1264,-0.0088,1.03],Q,2,iters=2)
w0=wrench(r); print('base',w0[:3])
for z in [1.0,0.99,0.985,0.98,0.975,0.97,0.965,0.96,0.955,0.95]:
    a=goto_cl(r,[-0.1264,-0.0088,z],Q,1.2,iters=2,tol=0.002)
    w=wrench(r); print('z',z,'actual',a.round(4),'dF',(w-w0)[:3])
    if abs((w-w0)[2])>3 or abs(a[2]-z)>0.004: print('TOP CONTACT at tcp z',a[2]); break
goto_cl(r,[-0.1264,-0.0088,1.03],Q,1.5,iters=1)
"

# openrua op 68
timeout 900 python3 -c "
from robot import *
r=Robot(); Q=(0.7071068,0.7071068,0,0)
goto_cl(r,[-0.16,-0.045,1.15],Q,2,iters=2)
" && python3 scene3d.py robot0_eye_in_hand && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3)
P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.915)&(P[:,2]<1.0)&(~((P[:,1]<-0.10)&(P[:,2]>0.925)&(P[:,0]<0.05)&(P[:,0]>-0.14)))
# exclude knob region
m&=~((P[:,0]<-0.17)&(P[:,1]<-0.16))
pot=P[m]
print('n',len(pot),'x',pot[:,0].min().round(3),pot[:,0].max().round(3),'y',pot[:,1].min().round(3),pot[:,1].max().round(3),'zmax',pot[:,2].max().round(3))
c=pot[:,:2].mean(0); u,s,vt=np.linalg.svd(pot[:,:2]-c,full_matrices=False)
d=vt[0]; n=vt[1]; print('center',c.round(4),'axis',d.round(3),'normal',n.round(3))
t=(pot[:,:2]-c)@d; nn=(pot[:,:2]-c)@n
for lo in np.arange(-0.09,0.09,0.01):
    s=(t>=lo)&(t<lo+0.01)
    if s.sum()>5: print(f't {lo:+.3f} n {nn[s].min():+.4f}..{nn[s].max():+.4f} w {nn[s].max()-nn[s].min():.4f} nc {(nn[s].max()+nn[s].min())/2:+.4f} zmax {pot[s,2].max():.3f} cnt {s.sum()}')
"

# openrua op 69
python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3)
P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.915)&(P[:,2]<0.99)
m&=~((P[:,0]>-0.15)&(P[:,0]<0.06)&(P[:,1]<-0.10))   # stove
m&=~((P[:,0]<-0.16)&(P[:,1]<-0.15))  # knob
m&=(P[:,1]<0.15)  # right pot
pot=P[m]
print('n',len(pot),'x',pot[:,0].min().round(3),pot[:,0].max().round(3),'y',pot[:,1].min().round(3),pot[:,1].max().round(3),'zmax',pot[:,2].max().round(3))
# 2D histogram to see the footprint
H,xe,ye=np.histogram2d(pot[:,0],pot[:,1],bins=[np.arange(-0.25,0.06,0.01),np.arange(-0.17,0.16,0.01)])
print('   y:',' '.join(f'{y*100:+3.0f}'[-3:] for y in ye[:-1]))
for i,x in enumerate(xe[:-1]): print(f'x{x:+.2f}',' '.join('###' if h>40 else (' ##' if h>10 else ('  .' if h>0 else '   ')) for h in H[i]))
"

# openrua op 70
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3)
P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.915)&(P[:,2]<0.99)
m&=~((P[:,0]>-0.15)&(P[:,0]<0.06)&(P[:,1]<-0.10))
m&=~((P[:,0]<-0.16)&(P[:,1]<-0.15)); m&=(P[:,1]<0.15)
m&=~((P[:,0]<-0.125)&(P[:,0]>-0.145)&(P[:,1]<-0.10))
pot=P[m]
c=pot[:,:2].mean(0); u,s,vt=np.linalg.svd(pot[:,:2]-c,full_matrices=False)
d=vt[0]; n=vt[1]
if d[0]<0: d=-d; n=-n
print('center',c.round(4),'axis',d.round(3),'normal',n.round(3))
t=(pot[:,:2]-c)@d; nn=(pot[:,:2]-c)@n
for lo in np.arange(-0.10,0.10,0.01):
    s=(t>=lo)&(t<lo+0.01)
    if s.sum()>5:
        hi=s&(pot[:,2]>0.96)
        print(f't {lo:+.3f} n {nn[s].min():+.4f}..{nn[s].max():+.4f} w {nn[s].max()-nn[s].min():.4f} nc {(nn[s].max()+nn[s].min())/2:+.4f} zmax {pot[s,2].max():.3f}  top(z>.96) nc {(nn[hi].max()+nn[hi].min())/2 if hi.sum() else 0:+.4f} w {(nn[hi].max()-nn[hi].min()) if hi.sum() else 0:.4f}')
"

# openrua op 71
mkdir -p "$(dirname /workspace/grasp.py)"
cat > /workspace/grasp.py <<'OPENRUA_EOF'
"""Grasp the lying moka pot across its bottom chamber (fingers perpendicular
to the pot axis), verify, lift.  Geometry from the wrist-camera scan."""
import math
import sys
import numpy as np
from robot import *

c = np.array([-0.1616, -0.0441])
d = np.array([0.741, 0.671])      # pot axis, +d = toward the base
n = np.array([-0.671, 0.741])     # finger opening direction
T_GRASP = float(sys.argv[1]) if len(sys.argv) > 1 else 0.045
Z_GRASP = float(sys.argv[2]) if len(sys.argv) > 2 else 0.916
NC = -0.0017

Q0 = (0.7071068, 0.7071068, 0, 0)
theta = math.atan2(-n[1], -n[0])           # hand x = -n
Qy = quat_mul((0, 0, math.sin(theta / 2), math.cos(theta / 2)), Q0)
print("theta deg", math.degrees(theta), "Qy", np.round(Qy, 4))
R = quat_R(*Qy)
print("hand x", R[:, 0].round(3), "hand z", R[:, 2].round(3))

g = c + T_GRASP * d + (NC + 0.00125) * n
print("grasp xy", g.round(4))

r = Robot()


def hold(k=1, sec=1.5):
    j = [r.joints()[q] for q in ARM]
    for _ in range(k):
        r.move_joints(j, sec)
    js = r.joints()
    return js["panda_finger_joint1"], js["panda_finger_joint2"]


# open gripper, confirm
r.gripper(GRIP["open_m"])
for i in range(40):
    f = hold()
    if min(f) > 0.039:
        break
print("fingers open", f)

goto_cl(r, [g[0], g[1], 1.03], Qy, 2.5, iters=3)
w0 = wrench(r)
print("baseline", w0[:3])
z = 1.03
for z in [1.0, 0.98, 0.965, 0.95, 0.94, 0.93, 0.922, Z_GRASP]:
    a = goto_cl(r, [g[0], g[1], z], Qy, 1.2, iters=2, tol=0.002)
    w = wrench(r)
    print("z", z, "actual", a.round(4), "dF", (w - w0)[:3])
    if np.abs((w - w0)[:3]).max() > 2.5:
        print("CONTACT during descent")
        break
print("at grasp height, fingers", hold())

r.gripper(GRIP["closed_m"])
prev = None
for i in range(60):
    f = hold()
    if prev is not None and abs(f[0] - prev[0]) < 0.0002 and abs(f[1] - prev[1]) < 0.0002:
        break
    prev = f
print("closed on", f, "gap", round(sum(f), 4))

w1 = wrench(r)
a = goto_cl(r, [g[0], g[1], Z_GRASP + 0.03], Qy, 1.5, iters=2)
f = hold()
w2 = wrench(r)
print("lifted 3cm: fingers", f, "dFz", (w2 - w1)[2], "dF vs base", (w2 - w0)[:3])
a = goto_cl(r, [g[0], g[1], 1.08], Qy, 2.0, iters=2)
f = hold()
print("lifted to 1.08: fingers", f, "wrench", wrench(r)[:3])
OPENRUA_EOF

# openrua op 72
timeout 1800 python3 -u grasp.py 2>&1 | grep -v '^traj done'

# openrua op 73
timeout 900 python3 -c "
from robot import *
r=Robot(); Q=(0.7071068,0.7071068,0,0)
goto_cl(r,[-0.16,-0.045,1.15],Q,2,iters=2)
" | grep -v 'traj done' && python3 scene3d.py robot0_eye_in_hand && python3 scene3d.py birdview && python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np
def pot(name):
    P=np.load(name).reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,2]>0.915)&(P[:,2]<0.99)
    m&=~((P[:,0]>-0.15)&(P[:,0]<0.06)&(P[:,1]<-0.10))
    m&=~((P[:,0]<-0.16)&(P[:,1]<-0.15)); m&=(P[:,1]<0.15)&(P[:,1]>-0.10)&(P[:,0]>-0.30)&(P[:,0]<0.0)
    return P[m]
for name in ['robot0_eye_in_hand_world.npy','birdview_world.npy']:
    p=pot(name)
    print(name,'n',len(p),'x',p[:,0].min().round(4),p[:,0].max().round(4),'y',p[:,1].min().round(4),p[:,1].max().round(4),'zmax',p[:,2].max().round(3),'mean',p[:,:2].mean(0).round(4))
    hi=p[p[:,2]>0.965]; print('   top(z>.965) mean',hi[:,:2].mean(0).round(4),'x',hi[:,0].min().round(3),hi[:,0].max().round(3),'y',hi[:,1].min().round(3),hi[:,1].max().round(3))
"

# openrua op 74
python3 tools/perception/cam_snap.py frontview >/dev/null; python3 tools/perception/cam_snap.py agentview >/dev/null; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.915)&(P[:,2]<1.1)&(P[:,0]>-0.30)&(P[:,0]<0.0)&(P[:,1]>-0.2)&(P[:,1]<0.15)
m&=~((P[:,0]>-0.15)&(P[:,0]<0.06)&(P[:,1]<-0.10)&(P[:,2]<0.94))
p=P[m]
for zlo in np.arange(0.92,1.1,0.02):
    s=(p[:,2]>=zlo)&(p[:,2]<zlo+0.02)
    if s.sum()>3: print(f'z {zlo:.2f} n {s.sum():5d} x {p[s,0].min():+.3f}..{p[s,0].max():+.3f} y {p[s,1].min():+.3f}..{p[s,1].max():+.3f} cx {p[s,0].mean():+.4f} cy {p[s,1].mean():+.4f}')
"

# openrua op 75
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.915)&(P[:,2]<1.12)
p=P[m]
print(len(p))
for zlo in np.arange(0.92,1.12,0.02):
    s=(p[:,2]>=zlo)&(p[:,2]<zlo+0.02)
    if s.sum()>3: print(f'z {zlo:.2f} n {s.sum():5d} x {p[s,0].min():+.3f}..{p[s,0].max():+.3f} y {p[s,1].min():+.3f}..{p[s,1].max():+.3f} cx {p[s,0].mean():+.4f} cy {p[s,1].mean():+.4f}')
"

# openrua op 76
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
p=P[P[:,2]>1.0]
print(len(p))
for zlo in np.arange(1.0,1.26,0.01):
    s=(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)
    if s.sum()>3: print(f'z {zlo:.2f} n {s.sum():5d} x {p[s,0].min():+.3f}..{p[s,0].max():+.3f} y {p[s,1].min():+.3f}..{p[s,1].max():+.3f} cx {p[s,0].mean():+.4f} cy {p[s,1].mean():+.4f}')
"; python3 -c "
from robot import *
r=Robot(); j=r.joints(); print('fingers',round(j['panda_finger_joint1'],4),round(j['panda_finger_joint2'],4)); print('wrench',wrench(r))"

# openrua op 77
python3 tools/perception/cam_snap.py sideview >/dev/null; python3 tools/perception/cam_snap.py frontview >/dev/null; python3 -c "
from PIL import Image
im=Image.open('agentview.png'); im.crop((150,60,350,220)).resize((600,480)).save('agent_crop.png')
"

# openrua op 78
python3 -c "
from PIL import Image
im=Image.open('frontview.png'); im.crop((220,200,360,300)).resize((700,500)).save('front_crop.png')
im=Image.open('sideview.png'); print(im.size); im.save('side.png')
"

# openrua op 79
timeout 1800 python3 -u -c "
from robot import *
import math
r=Robot()
Q0=(0.7071068,0.7071068,0,0)
def rotx(deg): a=math.radians(deg); return quat_mul((math.sin(a/2),0,0,math.cos(a/2)),Q0)
for deg in (-45,-90):
    Qr=rotx(deg); R=quat_R(*Qr); print('deg',deg,'Q',np.round(Qr,4),'hand y->', R[:,1].round(2),'hand z->',R[:,2].round(2))
    a=goto_cl(r,[-0.16,-0.045,1.15],Qr,3,iters=2)
    j=r.joints(); print('fingers',round(j['panda_finger_joint1'],4),round(j['panda_finger_joint2'],4))
" 2>&1 | grep -v 'traj done'

# openrua op 80
python3 tools/perception/cam_snap.py frontview >/dev/null; python3 tools/perception/cam_snap.py agentview >/dev/null; python3 -c "
from PIL import Image
Image.open('frontview.png').crop((200,150,400,320)).resize((700,595)).save('front_crop.png')
Image.open('agentview.png').crop((150,40,400,240)).resize((700,560)).save('agent_crop.png')"

# openrua op 81
timeout 1800 python3 -u -c "
from robot import *
r=Robot()
Q=(0.5,0.5,-0.5,0.5)
goto_cl(r,[-0.10,-0.13,1.10],Q,3,iters=2)
goto_cl(r,[-0.046,-0.215,1.05],Q,3,iters=2)
j=r.joints(); print('fingers',round(j['panda_finger_joint1'],4),round(j['panda_finger_joint2'],4))
w0=wrench(r); print('base',w0[:3])
for z in [1.02,1.0,0.99,0.98,0.975,0.97,0.965,0.96,0.955,0.95,0.945,0.94,0.935]:
    a=goto_cl(r,[-0.046,-0.215,z],Q,1.2,iters=2,tol=0.002)
    w=wrench(r); print('z',z,'actual',a.round(4),'dF',(w-w0)[:3])
    if (w-w0)[2]>1.0 or abs(a[2]-z)>0.004: print('PLACED CONTACT at tcp z',a[2]); break
" 2>&1 | grep -v 'traj done'

# openrua op 82
timeout 1800 python3 -u -c "
from robot import *
r=Robot()
Q=(0.5,0.5,-0.5,0.5)
r.gripper(GRIP['open_m'])
j=[r.joints()[n] for n in ARM]
for i in range(40):
    r.move_joints(j,1.5); js=r.joints(); f=(js['panda_finger_joint1'],js['panda_finger_joint2'])
    if min(f[0],-f[1])>0.039: break
print('fingers',f)
goto_cl(r,[-0.046,-0.215,1.0],Q,2,iters=1)
goto_cl(r,[-0.046,-0.215,1.10],Q,2,iters=1)
goto_cl(r,[-0.10,-0.05,1.20],Q,3,iters=1)
print('wrench',wrench(r)[:3])
" 2>&1 | grep -v 'traj done'; python3 tools/perception/cam_snap.py agentview >/dev/null; python3 tools/perception/cam_snap.py frontview >/dev/null; python3 scene3d.py birdview

# openrua op 83
python3 -c "
from PIL import Image
Image.open('frontview.png').crop((180,60,480,300)).resize((750,600)).save('front_crop.png')
Image.open('agentview.png').crop((220,60,420,200)).resize((800,560)).save('agent_crop.png')"

# openrua op 84
python3 -c "
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
p=P[(P[:,2]>0.95)&(P[:,2]<1.45)&(P[:,0]>-0.35)&(P[:,0]<0.1)]
for zlo in np.arange(0.95,1.45,0.02):
    s=(p[:,2]>=zlo)&(p[:,2]<zlo+0.02)
    if s.sum()>3: print(f'z {zlo:.2f} n {s.sum():5d} x {p[s,0].min():+.3f}..{p[s,0].max():+.3f} y {p[s,1].min():+.3f}..{p[s,1].max():+.3f}')
"; python3 -c "
from robot import *
r=Robot(); p,q=r.fk_pose(); print('hand',p.round(3),'tcp',tcp_now(r).round(3),np.round(q,3))"

# openrua op 85
python3 tools/perception/cam_snap.py sideview >/dev/null; python3 -c "
from robot import *
import numpy as np, rclpy
from tf2_ros import Buffer, TransformListener
r=Robot(); buf=Buffer(); TransformListener(buf,r.node)
for cam in ['sideview','frontview','agentview']:
    fr=cam+'_optical_frame'
    while not buf.can_transform('world',fr,rclpy.time.Time()): r.spin(0.2)
    t=buf.lookup_transform('world',fr,rclpy.time.Time()).transform
    q=t.rotation; R=quat_R(q.x,q.y,q.z,q.w)
    print(cam,'pos',np.round([t.translation.x,t.translation.y,t.translation.z],3),'look dir',R[:,2].round(2),'img right',R[:,0].round(2),'img down',R[:,1].round(2))
"

# openrua op 86
python3 scene3d.py frontview >/dev/null && python3 -c "
import numpy as np
P=np.load('frontview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
p=P[(P[:,2]>1.10)&(P[:,2]<1.36)&(P[:,0]>-0.25)&(P[:,0]<0.1)&(P[:,1]>-0.2)&(P[:,1]<0.0)]
for zlo in np.arange(1.10,1.36,0.01):
    s=(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)
    if s.sum()>3: print(f'z {zlo:.2f} n {s.sum():5d} x {p[s,0].min():+.3f}..{p[s,0].max():+.3f} y {p[s,1].min():+.3f}..{p[s,1].max():+.3f}  ymean {p[s,1].mean():+.3f}')
"

# openrua op 87
timeout 1800 python3 -u -c "
from robot import *
r=Robot()
Q=(0.5,0.5,-0.5,0.5)
X,Y=-0.061,-0.192
goto_cl(r,[X,Y,1.10],Q,3,iters=2)
goto_cl(r,[X,Y,1.02],Q,2,iters=2)
w0=wrench(r); print('base',w0[:3])
zc=None
for z in [1.01,1.0,0.995,0.99,0.985,0.98,0.975,0.97,0.965,0.96]:
    a=goto_cl(r,[X,Y,z],Q,1.2,iters=2,tol=0.002)
    w=wrench(r); print('z',z,'actual',a.round(4),'dF',(w-w0)[:3])
    if (w-w0)[2]>0.4 or abs(a[2]-z)>0.004: print('CONTACT at tcp z',a[2]); zc=z; break
if zc is not None:
    goto_cl(r,[X,Y,zc-0.004],Q,1.2,iters=2,tol=0.002); print('dF',(wrench(r)-w0)[:3])
    for y in [Y+0.02,Y+0.04,Y+0.07]:
        a=goto_cl(r,[X,y,zc-0.004],Q,1.5,iters=2); print('retreat y',y,'dF',(wrench(r)-w0)[:3])
    goto_cl(r,[X,Y+0.07,1.10],Q,2,iters=1)
    goto_cl(r,[-0.15,0.0,1.20],Q,3,iters=1)
    print('final wrench',wrench(r)[:3])
" 2>&1 | grep -v 'traj done'; python3 tools/perception/cam_snap.py agentview >/dev/null; python3 tools/perception/cam_snap.py frontview >/dev/null

# openrua op 88
python3 scene3d.py birdview >/dev/null && python3 scene3d.py frontview >/dev/null && python3 -c "
import numpy as np
for cam in ['birdview','frontview']:
    P=np.load(cam+'_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    p=P[(P[:,2]>0.94)&(P[:,2]<1.12)&(P[:,0]>-0.16)&(P[:,0]<0.07)&(P[:,1]>-0.32)&(P[:,1]<-0.09)]
    print(cam,'pot-on-stove points',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
    for zlo in np.arange(0.94,1.12,0.02):
        s=(p[:,2]>=zlo)&(p[:,2]<zlo+0.02)
        if s.sum()>3: print(f'   z {zlo:.2f} n {s.sum():5d} x {p[s,0].min():+.3f}..{p[s,0].max():+.3f} y {p[s,1].min():+.3f}..{p[s,1].max():+.3f} cx {p[s,0].mean():+.3f} cy {p[s,1].mean():+.3f}')
# anything left on the table where the pot used to lie?
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
q=P[(P[:,2]>0.915)&(P[:,2]<1.0)&(P[:,0]>-0.3)&(P[:,0]<0.1)&(P[:,1]>-0.1)&(P[:,1]<0.15)]
print('table region near old pot spot, points above table:',len(q))
"
