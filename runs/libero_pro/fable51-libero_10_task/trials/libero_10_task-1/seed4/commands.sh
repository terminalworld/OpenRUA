#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md

# openrua op 2
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo --- && ros2 action list && echo --- && ros2 service list | grep -v parameter && echo --- && ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[220:320,180:380]; cv2.imwrite('crop_cans.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im2=cv2.imread('robot0_eye_in_hand.png')
crop=im2[260:380,220:540]; cv2.imwrite('crop_cans2.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 20 ros2 topic echo /tf_static --once --qos-durability transient_local 2>&1 | grep -E "frame_id|child_frame_id" | head -40; echo ---; timeout 20 ros2 topic echo /birdview/color/camera_info --once 2>&1 | grep -A12 "^k:"

# openrua op 7
cat > tfdump.py <<'EOF'
import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('tfdump')
got={}
def cb(m):
    for t in m.transforms:
        got[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(TFMessage,'/tf_static',cb,qos)
n.create_subscription(TFMessage,'/tf',cb,10)
end=time.time()+5
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.2)
for k,v in sorted(got.items()):
    print(k, f"t=({v.translation.x:.3f},{v.translation.y:.3f},{v.translation.z:.3f}) q=({v.rotation.x:.3f},{v.rotation.y:.3f},{v.rotation.z:.3f},{v.rotation.w:.3f})")
EOF
python3 tfdump.py

# openrua op 8
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png && python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
# world z = 3.0 - depth (camera looks straight down)
z = 3.0 - d
print('table z around center', z[240,320], z[200,300])
# object mask: above table by > 1cm
tab = np.median(z[150:350,200:450])
print('table z', tab)
import cv2
mask = ((z > tab+0.01) & (z < tab+0.4)).astype(np.uint8)
n,lab,stats,cent = cv2.connectedComponentsWithStats(mask)
fx=579.4112549695428
for i in range(1,n):
    if stats[i,4]<5: continue
    u,v = cent[i]; 
    ys,xs = np.where(lab==i); zt = z[ys,xs].max()
    # world coords: optical frame q=(0.707,0.707,0,0): x_world = -0.2 + ? compute generally later
    print(i, 'px',(round(u),round(v)), 'area',stats[i,4], 'top z',round(zt,3))
"


# openrua op 9
for p in "agentview 215 247" "agentview 355 335" "agentview 350 345" "robot0_eye_in_hand 275 190" "robot0_eye_in_hand 275 170" "birdview 284 259" "birdview 284 231" "birdview 385 286" "birdview 385 260"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py $p; done

# openrua op 10
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); f=579.4112549695428
z=3.0-d
# basket region
for v in range(250,325,5):
    row=''
    for u in range(350,425,5):
        row+=f'{z[v,u]:.2f} '
    print(v,row)
print('cols u:',list(range(350,425,5)))
"


# openrua op 11
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small control library for this Panda: FK, IK, trajectory, gripper.
World<->base conversion uses machine.yaml/TF fact: panda_link0 at
world (-0.51, 0, 0.42), no rotation.
"""
import sys, time
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE = np.array([-0.51, 0.0, 0.42])
TCP = 0.1034


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("ctl")
        self._js = {}
        self.n.create_subscription(JointState, "/joint_states",
                                   lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.n.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.n, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.n, GripperCommand,
                                 "/franka_gripper/gripper_action")
        self.twist = self.n.create_publisher(TwistStamped,
                                             "/servo_node/delta_twist_cmds", 10)
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            rclpy.spin_once(self.n, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[a] for a in ARM]

    def finger_gap(self):
        j = self.joints()
        return abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"])

    def _seed(self):
        js = JointState()
        for a, p in zip(ARM, self.arm_q()):
            js.name.append(a); js.position.append(p)
        return js

    def hand_pose(self):
        """Hand frame pose in WORLD: (xyz, quat xyzw)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        r = fut.result()
        assert r is not None and r.error_code.val == 1, f"FK failed {r}"
        p = r.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z,
                      p.orientation.w])
        return xyz, q

    def tcp_pose(self):
        xyz, q = self.hand_pose()
        R = quat_R(*q)
        return xyz + TCP * R[:, 2], q

    def solve_ik(self, tcp_xyz_world, q, seed=None):
        """IK for hand so that TCP is at tcp_xyz_world with quaternion q."""
        R = quat_R(*q)
        hand = np.array(tcp_xyz_world) - TCP * R[:, 2] - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = seed or self._seed()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            log("IK failed", None if r is None else r.error_code.val)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[a] for a in ARM]

    def move_q(self, q, seconds=3.0, tol=0.02, retries=2):
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ARM
            pt = JointTrajectoryPoint(positions=[float(x) for x in q])
            pt.time_from_start = Duration(sec=int(seconds),
                                          nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            fut = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
            res = fut.result().get_result_async()
            rclpy.spin_until_future_complete(self.n, res, timeout_sec=600)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            log(f"move_q code={code} max_err={err:.4f}")
            if err < tol:
                return True
        return False

    def move_tcp(self, xyz, q, seconds=3.0):
        sol = self.solve_ik(xyz, q)
        if sol is None:
            return False
        ok = self.move_q(sol, seconds)
        p, _ = self.tcp_pose()
        log(f"tcp now {np.round(p, 4)} target {np.round(xyz, 4)} ok={ok}")
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res, timeout_sec=300)
        r = res.result().result
        log(f"gripper({width}) reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}")
        return r

    def servo(self, dx=0, dy=0, dz=0, ticks=20):
        msg = TwistStamped(); msg.header.frame_id = "panda_link0"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(dx), float(dy), float(dz)
        for _ in range(ticks):
            msg.header.stamp = self.n.get_clock().now().to_msg()
            self.twist.publish(msg); rclpy.spin_once(self.n, timeout_sec=0.05)
        p, _ = self.tcp_pose()
        log(f"servo done tcp {np.round(p, 4)}")

    def close(self):
        self.n.destroy_node(); rclpy.shutdown()


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


if __name__ == "__main__":
    c = Ctl()
    xyz, q = c.hand_pose()
    t, _ = c.tcp_pose()
    print("hand world", np.round(xyz, 4), "quat", np.round(q, 4))
    print("tcp world", np.round(t, 4))
    print("R", np.round(quat_R(*q), 3))
    print("joints", c.joints())
    c.close()
OPENRUA_EOF

# openrua op 12
timeout 120 python3 ctl.py

# openrua op 13
cat > tfchain.py <<'EOF'
import rclpy, time, numpy as np
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from ctl import quat_R
rclpy.init(); n=rclpy.create_node('tfc')
got={}
def cb(m):
    for t in m.transforms: got[(t.header.frame_id,t.child_frame_id)]=t.transform
n.create_subscription(TFMessage,'/tf_static',cb,QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(TFMessage,'/tf',cb,10)
end=time.time()+4
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.2)
chain=['world','panda_link0','panda_link1','panda_link2','panda_link3','panda_link4','panda_link5','panda_link6','panda_link7','panda_link8','panda_hand']
T=np.eye(4)
for a,b in zip(chain,chain[1:]):
    tr=got[(a,b)]; M=np.eye(4); M[:3,:3]=quat_R(tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w)
    M[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]; T=T@M
    print(b, np.round(T[:3,3],4))
print('hand R', np.round(T[:3,:3],3))
EOF
python3 tfchain.py

# openrua op 14
cat > iktest.py <<'EOF'
import numpy as np
from ctl import Ctl, quat_R, BASE, TCP
c=Ctl()
q0=np.array(c.arm_q())
# FK raw (no BASE added)
xyz,quat=c.hand_pose(); hand_raw = xyz-BASE
print('FK raw hand', np.round(hand_raw,4), 'quat', np.round(quat,4))
R=quat_R(*quat)
# case A: interpret raw as world -> tcp world
tcpA = hand_raw + TCP*R[:,2]
for label, tcp in [('raw-as-world', tcpA), ('raw-as-base(+BASE)', tcpA+BASE)]:
    sol=c.solve_ik(tcp, quat)
    if sol is None: print(label,'no IK'); continue
    print(label, 'max joint diff from current', np.round(np.abs(np.array(sol)-q0).max(),4))
c.close()
EOF
timeout 200 python3 iktest.py

# openrua op 15
cat > iktest2.py <<'EOF'
import numpy as np
from ctl import Ctl, quat_R, BASE, TCP
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
c=Ctl()
q0=np.array(c.arm_q())
xyz,quat=c.hand_pose(); hand_raw = xyz-BASE
tcp = hand_raw + TCP*quat_R(*quat)[:,2] + BASE   # so solve_ik passes hand_raw
sol=np.array(c.solve_ik(tcp, quat))
print('current ', np.round(q0,3)); print('solution', np.round(sol,3))
# FK of solution
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand']
js=JointState(); js.name=[f'panda_joint{i}' for i in range(1,8)]; js.position=[float(x) for x in sol]
req.robot_state.joint_state=js
import rclpy
fut=c.fk.call_async(req); rclpy.spin_until_future_complete(c.n,fut,timeout_sec=30)
p=fut.result().pose_stamped[0]
print('FK(sol) frame', p.header.frame_id, np.round([p.pose.position.x,p.pose.position.y,p.pose.position.z],4))
c.close()
EOF
timeout 200 python3 iktest2.py

# openrua op 16
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "BASE = np.array([-0.51, 0.0, 0.42])", "new_string": "# verified: /compute_fk and /compute_ik on this machine use WORLD coords\\n# (FK of the current state matched the TF chain world->panda_hand)\\nBASE = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 17
mkdir -p "$(dirname /workspace/run.py)"
cat > /workspace/run.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""CLI over ctl.Ctl: a sequence of commands separated by '--'.
  pose                     print tcp pose + finger gap
  open | close             gripper fully open / closed
  tcp X Y Z [SEC]          IK move so the TCP is at world XYZ, top-down grip
  servo DX DY DZ TICKS     cartesian servo burst (m/s in panda_link0)
"""
import sys
import numpy as np
from ctl import Ctl, log

TOPDOWN = np.array([1.0, 0.0, 0.0, 0.0])  # hand x->+x, fingers along world y


def main():
    argv = sys.argv[1:]
    cmds, cur = [], []
    for a in argv:
        if a == "--":
            cmds.append(cur); cur = []
        else:
            cur.append(a)
    if cur:
        cmds.append(cur)
    c = Ctl()
    ok_all = True
    for cmd in cmds:
        name, args = cmd[0], cmd[1:]
        log(">>", " ".join(cmd))
        if name == "pose":
            p, q = c.tcp_pose()
            log(f"tcp {np.round(p, 4)} quat {np.round(q, 4)} gap {c.finger_gap():.4f}")
        elif name == "open":
            c.gripper(0.04)
        elif name == "close":
            c.gripper(0.0)
        elif name == "tcp":
            xyz = [float(x) for x in args[:3]]
            sec = float(args[3]) if len(args) > 3 else 3.0
            ok = c.move_tcp(xyz, TOPDOWN, sec)
            ok_all &= ok
            if not ok:
                log("!! move failed, stopping sequence"); break
        elif name == "servo":
            dx, dy, dz, ticks = float(args[0]), float(args[1]), float(args[2]), int(args[3])
            c.servo(dx, dy, dz, ticks)
        else:
            log("unknown", name)
    c.close()
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 18
timeout 900 python3 -u run.py open -- tcp -0.12 -0.156 0.62 4 -- pose 2>&1 | tail -20

# openrua op 19
cat > fk2.py <<'EOF'
import numpy as np, rclpy
from ctl import Ctl
from moveit_msgs.srv import GetPositionFK
c=Ctl()
req=GetPositionFK.Request(); req.fk_link_names=['panda_link8','panda_hand']
req.robot_state.joint_state=c._seed()
fut=c.fk.call_async(req); rclpy.spin_until_future_complete(c.n,fut,timeout_sec=30)
for ps in fut.result().pose_stamped:
    o=ps.pose.orientation; print(ps.header.frame_id, np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],4), 'q', np.round([o.x,o.y,o.z,o.w],4))
print(fut.result().fk_link_names)
c.close()
EOF
timeout 100 python3 fk2.py

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "        R = quat_R(*q)\\n        hand = np.array(tcp_xyz_world) - TCP * R[:, 2] - BASE\\n        req = GetPositionIK.Request()", "new_string": "        R = quat_R(*q)\\n        hand = np.array(tcp_xyz_world) - TCP * R[:, 2] - BASE\\n        # verified: /compute_ik targets panda_link8, which is rotated -45deg\\n        # about z w.r.t. panda_hand (same origin). Convert hand quat -> link8.\\n        q = quat_mul(q, Q_HAND_TO_L8)\\n        req = GetPositionIK.Request()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "def quat_R(x, y, z, w):", "new_string": "def quat_mul(a, b):\\n    \\"\\"\\"Hamilton product, xyzw convention: result = a * b.\\"\\"\\"\\n    ax, ay, az, aw = a\\n    bx, by, bz, bw = b\\n    return np.array([\\n        aw * bx + ax * bw + ay * bz - az * by,\\n        aw * by - ax * bz + ay * bw + az * bx,\\n        aw * bz + ax * by - ay * bx + az * bw,\\n        aw * bw - ax * bx - ay * by - az * bz,\\n    ])\\n\\n\\n# TF panda_link8 -> panda_hand is (0,0,-0.383,0.924); inverse is +45deg about z\\nQ_HAND_TO_L8 = np.array([0.0, 0.0, 0.3826834, 0.9238795])\\n\\n\\ndef quat_R(x, y, z, w):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 22
timeout 900 python3 -u run.py tcp -0.12 -0.156 0.62 3 -- pose 2>&1 | tail -8

# openrua op 23
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png

# openrua op 24
timeout 900 python3 -u run.py tcp -0.12 -0.156 0.50 3 -- tcp -0.12 -0.156 0.452 2 -- close -- pose 2>&1 | grep -v "^$" | tail -12

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 26
timeout 60 python3 tools/perception/cam_snap.py frontview front2.png; timeout 60 python3 tools/perception/cam_snap.py sideview side2.png; python3 -c "
import cv2
im=cv2.imread('front2.png'); cv2.imwrite('front2c.png', cv2.resize(im[150:300,200:400],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('side2.png'); cv2.imwrite('side2c.png', cv2.resize(im[300:480,200:450],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 27
cat > diag.py <<'EOF'
import numpy as np, sys
from ctl import Ctl, ARM
import run
c=Ctl()
cur=np.array(c.arm_q()); print('current', np.round(cur,3))
sol=c.solve_ik([-0.12,-0.156,0.452], run.TOPDOWN); print('ik sol ', np.round(sol,3))
print('diff   ', np.round(np.array(sol)-cur,3))
lim=[[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]]
print('limits ', lim)
c.close()
EOF
timeout 100 python3 diag.py; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 28
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null; timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:" | tail -2; python3 -c "
import numpy as np
d=np.load('eih_depth.npy')
print('center can top', d[300,315], d[280,320], d[320,320])
print('fingers', d[390,150], d[390,480], d[380,170], d[380,470])
print('table far', d[100,320], d[200,320])
print('row 300 across:', [round(float(x),3) for x in d[300,100:560:20]])
print('row 380 across:', [round(float(x),3) for x in d[380,100:560:20]])
"

# openrua op 29
timeout 900 python3 -u run.py tcp -0.12 -0.156 0.60 3 -- tcp -0.12 -0.156 0.53 3 -- pose 2>&1 | tail -6 && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png

# openrua op 30
cat > center.py <<'EOF'
import numpy as np, cv2, sys
d=np.load('eih_depth.npy'); f=579.4112549695428
H,W=d.shape
# fingers: nearest surfaces (depth < 0.10) in lower part of image
fing = (d<0.105)
# can top: depth in a band above table and below ... exclude finger mask rows
tab = np.median(d[50:150,250:400]); print('table depth', tab)
ys,xs=np.where(fing); print('finger pixels', len(ys), 'rows', ys.min(), ys.max(), 'cols', xs.min(), xs.max())
# split fingers left/right
left = xs<320; right=~left
print('left finger cols', xs[left].min(), xs[left].max(), 'right finger cols', xs[right].min(), xs[right].max())
print('finger depth left/right', np.median(d[ys[left],xs[left]]), np.median(d[ys[right],xs[right]]))
mid_col = (xs[left].max()+xs[right].min())/2
print('finger inner faces:', xs[left].max(), xs[right].min(), 'mid col', mid_col)
# can top: depth between finger depth+0.005 and tab-0.03, not in finger region
can = (d>0.105)&(d<tab-0.03)
can[ys.min():,:]&= False  # ignore rows at/below finger tops? keep only rows above fingers
cy,cx=np.where(can); print('can pixels',len(cy),'depth med',np.median(d[cy,cx]), 'rows',cy.min(),cy.max(),'cols',cx.min(),cx.max())
print('can centroid col,row', cx.mean(), cy.mean(), ' bbox center', (cx.min()+cx.max())/2, (cy.min()+cy.max())/2)
Z=np.median(d[cy,cx])
diam=(cx.max()-cx.min())*Z/f; print('can diameter est (m)', diam, ' rows extent', (cy.max()-cy.min())*Z/f)
vis=cv2.imread('eih3.png'); vis[can]=(0,255,0); vis[fing]=(255,0,0); cv2.imwrite('center_vis.png',vis)
EOF
python3 center.py

# openrua op 31
cat > center.py <<'EOF'
import numpy as np, cv2
d=np.load('eih_depth.npy'); f=579.4112549695428
tab = np.median(d[50:150,250:400]); print('table depth', tab)
roi = np.zeros_like(d,bool); roi[300:400,:]=True
fing = (d<0.11)&roi
ys,xs=np.where(fing)
left = xs<320; right=~left
print('left finger cols', xs[left].min(), xs[left].max(), 'rows', ys[left].min(), ys[left].max(), 'depth', np.median(d[ys[left],xs[left]]))
print('right finger cols', xs[right].min(), xs[right].max(), 'rows', ys[right].min(), ys[right].max(), 'depth', np.median(d[ys[right],xs[right]]))
mid_col = (xs[left].max()+xs[right].min())/2
print('finger inner faces:', xs[left].max(), xs[right].min(), 'mid col', mid_col, 'inner gap px', xs[right].min()-xs[left].max())
can = (d>0.11)&(d<tab-0.03); can[400:,:]=False
n,lab,st,cen=cv2.connectedComponentsWithStats(can.astype(np.uint8))
i=1+np.argmax(st[1:,4]); can=(lab==i)
cy,cx=np.where(can); Z=np.median(d[cy,cx])
print('can pixels',len(cy),'depth med',Z, 'rows',cy.min(),cy.max(),'cols',cx.min(),cx.max())
print('can centroid col,row', cx.mean(), cy.mean(), ' bbox center', (cx.min()+cx.max())/2, (cy.min()+cy.max())/2)
print('can diameter est (m)', (cx.max()-cx.min())*Z/f, (cy.max()-cy.min())*Z/f)
fd=np.median(d[ys,xs]); print('finger gap est (m) at finger depth', (xs[right].min()-xs[left].max())*fd/f)
vis=cv2.imread('eih3.png'); vis[can]=(0,255,0); vis[fing]=(255,0,0); cv2.imwrite('center_vis.png',vis)
EOF
python3 center.py

# openrua op 32
timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -B1 -A10 "^k:"; timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -E "height|width"

# openrua op 33
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); z=3.0-d; f=579.4112549695428
sub=z[245:275,270:300]
m=sub>0.45
ys,xs=np.where(m); print('bbox rows',ys.min()+245,ys.max()+245,'cols',xs.min()+270,xs.max()+270, 'n',m.sum())
Z=np.median(d[245:275,270:300][m]); print('Z',Z,'diam est', (xs.max()-xs.min()+1)*Z/f, (ys.max()-ys.min()+1)*Z/f)
np.set_printoptions(linewidth=200, precision=2)
print(sub[5:25,5:25])
# same for tomato can? and OJ (component 11 at 259,290) and milk 295,299 for reference
for name,(u,v) in {'oj':(259,290),'milk':(295,299),'cream':(274,308)}.items():
    sub=z[v-15:v+15,u-15:u+15]; m=sub>0.44; ys,xs=np.where(m); Z=np.median((3.0-sub)[m])
    print(name,'extent m', (xs.max()-xs.min()+1)*Z/f, (ys.max()-ys.min()+1)*Z/f, 'top z', sub[m].max())
"


# openrua op 34
python3 -c "
import numpy as np
d=np.load('eih_depth.npy'); f=312.774
np.set_printoptions(linewidth=250, precision=3)
for r in range(360,480,8):
    print(r, d[r,60:640:20])
print('cols', list(range(60,640,20)))
"


# openrua op 35
cat > fitcan.py <<'EOF'
import numpy as np, sys
from scipy.optimize import least_squares
d=np.load(sys.argv[1] if len(sys.argv)>1 else 'eih_depth.npy'); f=312.774
tab=np.median(d[50:150,250:400])
can=(d>0.108)&(d<tab-0.03); can[400:,:]=False
rows=[];lefts=[];rights=[]
for r in range(200,365):
    cs=np.where(can[r])[0]
    if len(cs)>20 and cs.max()-cs.min()<400:
        rows.append(r); lefts.append(cs.min()); rights.append(cs.max()+1)
rows=np.array(rows); lefts=np.array(lefts); rights=np.array(rights)
c0=(lefts+rights)/2
def resid(p):
    u0,v0,R=p
    half=np.sqrt(np.clip(R**2-(rows-v0)**2,0,None))
    return np.concatenate([lefts-(u0-half), rights-(u0+half)])
p=least_squares(resid,[np.median(c0),330,95]).x
u0,v0,R=p
Z=np.median(d[can])
print(f'circle center px ({u0:.1f},{v0:.1f}) R={R:.1f}px -> diam {2*R*Z/f:.4f} m, Z={Z:.4f}')
print('rows used',rows.min(),rows.max(),'median col center',np.median(c0))
EOF
python3 fitcan.py && timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 312 348

# openrua op 36
sed -i 's/can\[400:,:\]=False/can[400:,:]=False; can[:,:150]=False; can[:,500:]=False/' fitcan.py && python3 fitcan.py

# openrua op 37
timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 314 348

# openrua op 38
timeout 900 python3 -u run.py tcp -0.1132 -0.1534 0.53 3 -- pose 2>&1 | tail -3 && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && python3 fitcan.py && timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 320 365

# openrua op 39
timeout 900 python3 -u run.py tcp -0.1118 -0.1538 0.515 3 2>&1 | tail -1 && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png && python3 -c "
import numpy as np
d=np.load('eih_depth.npy')
np.set_printoptions(linewidth=250, precision=3)
for r in range(340,410,6):
    print(r, d[r,140:520:10])
print('cols', list(range(140,520,10)))
"

# openrua op 40
timeout 900 python3 -u run.py tcp -0.1118 -0.1522 0.455 3 -- close -- pose 2>&1 | tail -5

# openrua op 41
timeout 900 python3 -u run.py tcp -0.1118 -0.1522 0.515 3 2>&1 | tail -1 && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && python3 -c "
import numpy as np
d=np.load('eih_depth.npy')
np.set_printoptions(linewidth=250, precision=3)
for r in range(370,404,6):
    print(r, d[r,180:470:10])
print('cols', list(range(180,470,10)))
"

# openrua op 42
timeout 900 python3 -u run.py tcp -0.1118 -0.1522 0.481 3 -- close -- pose 2>&1 | tail -5

# openrua op 43
timeout 900 python3 -u run.py tcp -0.1118 -0.1522 0.65 3 -- pose 2>&1 | tail -2 && timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png

# openrua op 44
timeout 900 python3 -u run.py tcp -0.112 -0.152 0.75 3 -- tcp -0.015 0.275 0.73 4 -- pose 2>&1 | tail -3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih5.png

# openrua op 45
timeout 900 python3 -u run.py open -- tcp -0.015 0.275 0.80 3 2>&1 | tail -2 && timeout 60 python3 tools/perception/cam_snap.py agentview agent4.png && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih6.png

# openrua op 46
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('eih_depth.npy'); f=312.774
tab=np.median(d[100:150,100:200]); print('table depth',tab)
sub=np.zeros_like(d,bool); sub[180:300,430:520]=True
m=(d<tab-0.012)&sub
ys,xs=np.where(m); Z=np.median(d[m])
print('butter px bbox cols',xs.min(),xs.max(),'rows',ys.min(),ys.max(),'centroid',xs.mean(),ys.mean(),'Z',Z,'height above table',tab-Z)
print('size m: cols',(xs.max()-xs.min()+1)*Z/f,'rows',(ys.max()-ys.min()+1)*Z/f)
# orientation via PCA
pts=np.stack([xs,ys],1).astype(float); pts-=pts.mean(0); w,v=np.linalg.eigh(pts.T@pts); print('major axis (col,row) dir',v[:,1], 'angle deg from col axis', np.degrees(np.arctan2(v[1,1],v[0,1])))
" && timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 475 242

# openrua op 47
timeout 900 python3 -u run.py tcp 0.0328 0.0534 0.60 4 -- tcp 0.0328 0.0534 0.475 3 -- pose 2>&1 | tail -2 && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih7.png && python3 -c "
import numpy as np
d=np.load('eih_depth.npy'); f=312.774
tab=np.median(d[50:150,250:400]); print('table depth',tab)
sub=np.zeros_like(d,bool); sub[150:405,150:500]=True
m=(d<tab-0.012)&(d>0.11)&sub
ys,xs=np.where(m); Z=np.median(d[m])
print('butter cols',xs.min(),xs.max(),'rows',ys.min(),ys.max(),'centroid',xs.mean(),ys.mean(),'Z',Z)
fm=(d<0.108)&sub; fy,fx=np.where(fm); l=fx<320
print('finger inner edges', fx[l].max(), fx[~l].min(), 'mid', (fx[l].max()+fx[~l].min())/2, 'finger rows', fy.min(), fy.max())
hand_row = 240+0.05*f/Z; print('hand axis row at butter depth', hand_row, ' butter centroid row', ys.mean(), ' -> dx world (m)', -(ys.mean()-hand_row)*Z/f, ' dy world (m)', -(xs.mean()-320)*Z/f)
"

# openrua op 48
timeout 900 python3 -u run.py tcp 0.0328 0.0495 0.433 3 -- close -- tcp 0.0328 0.0495 0.60 3 -- pose 2>&1 | tail -6 && timeout 60 python3 tools/perception/cam_snap.py agentview agent5.png

# openrua op 49
timeout 900 python3 -u run.py tcp 0.0328 0.0495 0.72 3 -- tcp -0.015 0.275 0.66 4 -- pose 2>&1 | tail -2 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih8.png

# openrua op 50
timeout 900 python3 -u run.py open -- tcp -0.015 0.275 0.85 3 2>&1 | tail -2 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih9.png && timeout 60 python3 tools/perception/cam_snap.py agentview agent6.png

# openrua op 51
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth2.png >/dev/null && python3 -c "
import numpy as np
d=np.load('bird_depth2.npy'); z=3.0-d; f=579.4112549695428
def px(x,y):  # world -> birdview pixel
    dd=3.0-0.5; return int(round(320+y*f/dd)), int(round(240+(x+0.2)*f/dd))
for name,(x,y) in {'old can spot':(-0.112,-0.153),'old butter spot':(0.033,0.05),'basket center':(-0.015,0.275)}.items():
    u,v=px(x,y); print(name, 'max z in 5px window', round(float(z[v-5:v+6,u-5:u+6].max()),3))
# basket interior profile
np.set_printoptions(linewidth=200, precision=2)
print(z[268:308:3, 363:409:3])
"

# openrua op 52
timeout 900 python3 -u run.py tcp -0.25 -0.05 0.80 4 2>&1 | tail -1 && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth2.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview agent7.png && timeout 60 python3 tools/perception/cam_snap.py birdview bird2.png && python3 -c "
import numpy as np
d=np.load('bird_depth2.npy'); z=3.0-d
np.set_printoptions(linewidth=200, precision=2)
print('basket region z (rows 268-308 step3, cols 363-409 step3):'); print(z[268:308:3, 363:409:3])
print('max inside interior', z[275:300,372:402].max(), ' rim ~0.63')
"

# openrua op 53
python3 -c "
import cv2
im=cv2.imread('agent7.png'); cv2.imwrite('agent7c.png', cv2.resize(im[200:300,450:620],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('bird2.png'); cv2.imwrite('bird2c.png', cv2.resize(im[260:315,355:415],None,fx=8,fy=8,interpolation=cv2.INTER_CUBIC))
"
