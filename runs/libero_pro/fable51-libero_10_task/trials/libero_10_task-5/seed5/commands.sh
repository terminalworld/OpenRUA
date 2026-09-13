#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md

# openrua op 2
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---ACTIONS--- && ros2 action list && echo ---SERVICES--- && ros2 service list | grep -v parameter && echo ---NODES--- && ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once && for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do python3 tools/perception/cam_snap.py $c; done

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[130:300,180:420]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); crop=im[140:360,100:480]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"

# openrua op 6
echo cup; python3 tools/perception/px2world.py birdview 325 263; echo cup_rim_left; python3 tools/perception/px2world.py birdview 312 263; echo back_comp; python3 tools/perception/px2world.py birdview 278 173; echo front_comp; python3 tools/perception/px2world.py birdview 278 193; echo left_comp; python3 tools/perception/px2world.py birdview 240 185; echo caddy_wall_top; python3 tools/perception/px2world.py birdview 259 185; echo table; python3 tools/perception/px2world.py birdview 400 350

# openrua op 7
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; timeout 20 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -12

# openrua op 8
python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png && python3 -c "
import numpy as np
d=np.load('bird_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
# row through cup center
row=d[263,290:370]; print('row263', np.round(row,3))
col=d[230:300,325]; print('col325', np.round(col,3))
# caddy scan rows
print('row173', np.round(d[173,200:320],3))
print('col278', np.round(d[140:230,278],3))
"

# openrua op 9
ros2 topic echo /birdview/color/camera_info --once | grep -A12 '^k:'; python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
# handle extent
print('row258', np.round(d[258,295:320],3))
print('row268', np.round(d[268,295:320],3))
# right compartment area rows around y 170-200, x 300-330
for y in (165,175,185,195,205):
    print(y, np.round(d[y,296:340],3))
"

# openrua op 10
python3 tools/action/ik_move.py 0.35 0.0 0.55 1 0 0 0 3 2>&1 | tail -3 && ros2 topic echo /joint_states --once | grep -A9 position: && python3 tools/perception/cam_snap.py birdview && python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png

# openrua op 11
python3 tools/action/fjt_send.py 0,-0.5,0,-2.2,0,1.8,0.785 3 && timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A1 Translation | head -2 && python3 tools/perception/cam_snap.py birdview && python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png

# openrua op 12
python3 -c "
import cv2, numpy as np
im=cv2.imread('birdview.png'); crop=im[130:300,180:420]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
d=np.load('bird_depth.npy')
for y in (150,160,165,170,175,180,185,190,195,200,205,210):
    print(y, ' '.join(f'{v:.2f}' for v in d[y,210:350:2]))
print('cols')
for x in (230,280,320,335):
    print(x, ' '.join(f'{v:.2f}' for v in d[150:220,x]))
"

# openrua op 13
python3 tools/action/fjt_send.py 0.9,-0.5,0,-2.2,0,1.8,0.785 3 && python3 tools/perception/cam_snap.py birdview && python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png && python3 -c "
import cv2, numpy as np
im=cv2.imread('birdview.png'); crop=im[130:300,180:420]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
d=np.load('bird_depth.npy')
for y in (150,155,160,165,170,175,180,185,190,195,200,205,210):
    print(y, ' '.join(f'{v:.2f}' for v in d[y,210:350:2]))
print('cols')
for x in (230,280,320,335):
    print(x, ' '.join(f'{v:.2f}' for v in d[150:220,x]))
"

# openrua op 14
ros2 topic echo /joint_states --once | grep -A9 position: | tr '\n' ' '; echo; python3 tools/perception/cam_snap.py /frontview/depth/image_raw front_depth.png >/dev/null; ros2 topic echo /frontview/color/camera_info --once | grep -A2 '^k:' | tail -1; python3 -c "
import numpy as np, cv2
d=np.load('front_depth.npy')
im=cv2.imread('frontview.png'); crop=im[280:360,280:380]; cv2.imwrite('front_cup.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
for y in range(290,352,4):
    print(y, ' '.join(f'{v:.2f}' for v in d[y,290:372:2]))
"

# openrua op 15
ros2 topic echo /frontview/color/camera_info --once | grep -A1 '^k:' ; python3 -c "
import numpy as np
d=np.load('front_depth.npy')
for y in range(346,366,2):
    row=d[y,300:370]; m=row<1.3; xs=np.where(m)[0]
    print(y, (300+xs.min(), 300+xs.max(), xs.max()-xs.min()+1) if xs.size else None, ' '.join(f'{v:.2f}' for v in row[::3]))
"

# openrua op 16
mkdir -p "$(dirname /workspace/rb.py)"
cat > /workspace/rb.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library for this session: IK moves, servo bursts, gripper,
sensing. Base frame = panda_link0 (world = base + (-0.75, 0, 0.912))."""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import TwistStamped, WrenchStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
W2B = np.array([0.75, 0.0, -0.912])  # world -> base translation


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rb")
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._js_cb, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._wr_cb, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _js_cb(self, m): self.js = m
    def _wr_cb(self, m): self.wr = m

    def spin(self, n=3):
        for _ in range(n): rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self.spin(); self.js = None
        while self.js is None: rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints(); return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self.wr = None
        while self.wr is None: rclpy.spin_once(self.node, timeout_sec=0.2)
        f = self.wr.wrench.force; t = self.wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def hand_pose(self):
        """hand pose in base frame via FK: (xyz, quat xyzw)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        q = self.arm_q()
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = q
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z,
                          p.orientation.w]))

    def solve_ik(self, xyz, quat, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        (p.orientation.x, p.orientation.y, p.orientation.z,
         p.orientation.w) = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = list(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name,
                       r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def traj(self, points, seconds):
        """points: list of joint vectors; seconds: list of cumulative times."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(points, seconds):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q = np.array(self.arm_q()); err = np.abs(q - np.array(points[-1])).max()
        print(f"traj code={code} max_joint_err={err:.4f}")
        return code, err

    def move_to(self, xyz, quat, seconds=3.0, via=None):
        """IK for the pose, trajectory there. via: optional list of
        intermediate (xyz) poses with same quat for a multi-point pass."""
        pts, ts, seed = [], [], self.arm_q()
        targets = (via or []) + [xyz]
        for i, t in enumerate(targets):
            q = self.solve_ik(t, quat, seed)
            if q is None:
                print(f"IK FAILED for {t}"); return False
            pts.append(q); seed = q
            ts.append(seconds * (i + 1) / len(targets))
        code, err = self.traj(pts, ts)
        return code == 0 or err < 0.02

    def servo(self, v, n, dt=0.05):
        """stream twist v=(vx,vy,vz) m/s in base frame for n ticks."""
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=dt)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={self.fingers()}")
        return r

    def close(self):
        self.node.destroy_node(); rclpy.shutdown()


def w2b(p):
    return np.array(p) + W2B


def b2w(p):
    return np.array(p) - W2B
OPENRUA_EOF

# openrua op 17
python3 -u -c "
from rb import *
r=Robot()
print('hand', r.hand_pose())
print('fingers', r.fingers())
ok=r.move_to([0.629,0.074,0.30],[1,0,0,0],4.0)
print('ok',ok); print('hand', r.hand_pose())
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 tools/perception/cam_snap.py birdview

# openrua op 18
python3 -u -c "
from rb import *
r=Robot()
p,q=r.hand_pose(); print('fk', p, q)
cur=r.arm_q(); print('cur', np.round(cur,3))
s=r.solve_ik(p,q); print('ik world-coords ->', None if s is None else np.round(s,3))
s=r.solve_ik(w2b(p),q); print('ik base-coords ->', None if s is None else np.round(s,3))
r.close()
" 2>&1 | grep -v Warning

# openrua op 19
python3 -u -c "
from rb import *
r=Robot()
ok=r.move_to([-0.121,0.074,1.212],[1,0,0,0],4.0)
print('ok',ok); print('hand', r.hand_pose())
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 tools/perception/cam_snap.py birdview

# openrua op 20
python3 - <<'EOF'
import re
s=open('rb.py').read()
s=s.replace("""        code, err = self.traj(pts, ts)
        return code == 0 or err < 0.02""","""        code, err = self.traj(pts, ts)
        tries = 0
        while err > 0.02 and tries < 3:  # controller lag: resend final point
            tries += 1
            code, err = self.traj([pts[-1]], [max(2.0, seconds / 2)])
        return err < 0.02""")
open('rb.py','w').write(s)
EOF
python3 -u -c "
from rb import *
r=Robot()
ok=r.move_to([-0.121,0.074,1.212],[1,0,0,0],4.0)
print('ok',ok); print('hand', r.hand_pose())
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 21
python3 -u -c "
from rb import *
from moveit_msgs.srv import GetPositionFK
r=Robot()
req = GetPositionFK.Request()
req.fk_link_names = ['panda_link8','panda_hand','panda_hand_tcp','panda_leftfinger']
req.robot_state.joint_state.name = list(ARM); req.robot_state.joint_state.position = r.arm_q()
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
res=fut.result(); print(res.error_code)
for ps in res.pose_stamped:
    p=ps.pose; print(ps.header.frame_id, np.round([p.position.x,p.position.y,p.position.z],4), np.round([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],4))
print(res.fk_link_names)
r.close()
" 2>&1 | grep -v Warning; ros2 param get /move_group robot_description_semantic 2>/dev/null | grep -i -A3 'group name' | head

# openrua op 22
cat >> rb.py <<'EOF'


def qmul(a, b):
    """quaternion product a⊗b, xyzw."""
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw*bx + bx*0 + ax*bw + (ay*bz - az*by),
                     aw*by + ay*bw + (az*bx - ax*bz),
                     aw*bz + az*bw + (ax*by - ay*bx),
                     aw*bw - (ax*bx + ay*by + az*bz)])


RZ45 = np.array([0, 0, np.sin(np.pi/8), np.cos(np.pi/8)])


def hand2link8(q_hand):
    """IK tip link is panda_link8 = hand ⊗ Rz(+45°)."""
    return qmul(np.array(q_hand, float), RZ45)


def q_down(yaw_deg=0.0):
    """hand pointing straight down, fingers closing along world y rotated
    by yaw about z (yaw=0: fingers along y; yaw=90: along x)."""
    h = np.radians(yaw_deg) / 2
    return qmul(np.array([0, 0, np.sin(h), np.cos(h)]), np.array([1., 0, 0, 0]))
EOF
python3 -c "
from rb import *
print(hand2link8([1,0,0,0]), q_down(0), q_down(90))
print('check hand from link8', qmul(hand2link8([1,0,0,0]), [0,0,-np.sin(np.pi/8),np.cos(np.pi/8)]))
"

# openrua op 23
python3 -u -c "
from rb import *
r=Robot()
ok=r.move_to([-0.121,0.074,1.212],hand2link8(q_down(0)),3.0)
print('ok',ok); print('hand', r.hand_pose())
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 24
python3 tools/perception/cam_snap.py birdview >/dev/null && python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[200:330,260:400]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 25
python3 -u -c "
from rb import *
r=Robot()
print('wrench before', r.wrench())
ok=r.move_to([-0.121,0.074,1.118],hand2link8(q_down(0)),3.0, via=[[-0.121,0.074,1.16]])
print('ok',ok); print('hand', r.hand_pose()); print('wrench after', r.wrench())
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py frontview >/dev/null && python3 tools/perception/cam_snap.py sideview >/dev/null && python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[200:380,240:420]; cv2.imwrite('front_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('sideview.png'); crop=im[150:330,250:430]; cv2.imwrite('side_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 26
echo rim; python3 tools/perception/px2world.py frontview 330 324; echo rim2; python3 tools/perception/px2world.py frontview 350 326; echo fingertip; python3 tools/perception/px2world.py frontview 328 316; echo fingertip2; python3 tools/perception/px2world.py frontview 380 316; echo cupbase; python3 tools/perception/px2world.py frontview 330 358

# openrua op 27
echo table_front; python3 tools/perception/px2world.py frontview 320 420; python3 tools/perception/px2world.py frontview 150 380; echo caddy_backwall_top_front; python3 tools/perception/px2world.py frontview 200 253; echo caddy_backwall_top_bird; python3 tools/perception/px2world.py birdview 240 161; echo table_bird_center; python3 tools/perception/px2world.py birdview 330 300; python3 tools/perception/px2world.py birdview 100 100

# openrua op 28
python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null; python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
print('row263', ' '.join(f'{v:.3f}' for v in d[263,300:330]))
print('row250', ' '.join(f'{v:.3f}' for v in d[250,300:330]))
print('col316', ' '.join(f'{v:.3f}' for v in d[240:290,316]))
"; echo side_rim; python3 tools/perception/cam_snap.py /sideview/depth/image_raw side_depth.png >/dev/null; python3 -c "
import numpy as np
d=np.load('side_depth.npy')
for y in range(272,292,2): print(y, ' '.join(f'{v:.2f}' for v in d[y,310:360:2]))
"

# openrua op 29
python3 -u -c "
from rb import *
r=Robot()
print('hand', r.hand_pose()[0], 'wrench', r.wrench()[0])
r.servo([0,0,-0.03], 20)
print('hand', r.hand_pose()[0], 'wrench', r.wrench()[0])
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py frontview >/dev/null && python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[200:380,240:420]; cv2.imwrite('front_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 30
python3 tools/perception/cam_snap.py /frontview/depth/image_raw front_depth.png >/dev/null; python3 -c "
import numpy as np
d=np.load('front_depth.npy')
print('col380', ' '.join(f'{v:.2f}' for v in d[300:340,380]))
print('col328', ' '.join(f'{v:.2f}' for v in d[300:340,328]))
print('col354', ' '.join(f'{v:.2f}' for v in d[310:340,354]))
print('col330', ' '.join(f'{v:.2f}' for v in d[310:340,330]))
"

# openrua op 31
echo rim_right_edge; python3 tools/perception/px2world.py frontview 354 324; python3 tools/perception/px2world.py frontview 354 325; echo fingertip_right; python3 tools/perception/px2world.py frontview 380 319; python3 tools/perception/px2world.py frontview 380 318

# openrua op 32
python3 -u -c "
from rb import *
r=Robot()
ok=r.move_to([-0.121,0.074,1.30],hand2link8(q_down(0)),2.5)
print('ok',ok)
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null && python3 tools/perception/cam_snap.py birdview > /dev/null && python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
sub=d[230:300,290:370]
mask=(sub<2.05)&(sub>1.9)
ys,xs=np.where(mask)
print('cup-ish px range u',xs.min()+290,xs.max()+290,'v',ys.min()+230,ys.max()+230)
print('row263', ' '.join(f'{v:.3f}' for v in d[263,300:350]))
print('col328', ' '.join(f'{v:.3f}' for v in d[240:290,328]))
# rim ring: depth between 1.98 and 2.02?
"

# openrua op 33
python3 -u -c "
from rb import *
r=Robot()
ok=r.move_to([-0.121,0.28,1.30],hand2link8(q_down(0)),2.5)
print('ok',ok)
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null && python3 tools/perception/cam_snap.py birdview > /dev/null && python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
sub=d[230:300,290:370]
mask=(sub<2.11)
ys,xs=np.where(mask)
print('cup px range u',xs.min()+290,xs.max()+290,'v',ys.min()+230,ys.max()+230)
for v in (255,260,263,266,270):
    print('row',v, ' '.join(f'{x:.3f}' for x in d[v,300:350]))
print('col328', ' '.join(f'{v:.3f}' for v in d[240:290,328]))
"

# openrua op 34
python3 tools/perception/px2world.py birdview 327 261; python3 tools/perception/px2world.py birdview 341 261; python3 tools/perception/px2world.py birdview 313 261; python3 tools/perception/px2world.py birdview 327 247; python3 tools/perception/px2world.py birdview 327 275

# openrua op 35
python3 -u -c "
from rb import *
r=Robot()
q=hand2link8(q_down(0))
ok=r.move_to([-0.127,0.070,1.20],q,3.0); print('ok',ok)
print('wrench', r.wrench()[0])
ok=r.move_to([-0.127,0.070,1.063],q,3.0, via=[[-0.127,0.070,1.12]]); print('ok',ok)
print('hand', r.hand_pose()[0], 'wrench', r.wrench()[0])
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py frontview >/dev/null && python3 tools/perception/cam_snap.py sideview >/dev/null && python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[200:380,240:420]; cv2.imwrite('front_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('sideview.png'); crop=im[150:330,250:430]; cv2.imwrite('side_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 36
python3 -u -c "
from rb import *
r=Robot()
print('wrench before', r.wrench()[0])
r.gripper(0.0)
print('fingers', r.fingers(), 'wrench', r.wrench()[0])
r.close()
" 2>&1 | grep -v Warning

# openrua op 37
python3 -u -c "
from rb import *
r=Robot()
q=hand2link8(q_down(0))
ok=r.move_to([-0.127,0.070,1.28],q,3.0, via=[[-0.127,0.070,1.15]]); print('ok',ok)
print('hand', r.hand_pose()[0], 'fingers', r.fingers(), 'wrench', r.wrench()[0])
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py frontview >/dev/null && python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[120:380,200:440]; cv2.imwrite('front_crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"

# openrua op 38
find / -iname "*caddy*" 2>/dev/null | head; find / -iname "*libero*" -maxdepth 4 2>/dev/null | head; ls /opt /home 2>/dev/null

# openrua op 39
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 1023}}" 2>/dev/null | head -c 3000; echo; ros2 topic echo /pipeline_state --once 2>/dev/null | head -20; ls -la /home/robot; cat .ros/log/*.log 2>/dev/null | grep -i -E "caddy|libero|bddl|task" | head

# openrua op 40
timeout 10 ros2 topic echo /pipeline_state --once 2>&1 | head -20; ls -la /home/robot; grep -il -E "caddy|libero" .ros/log/*.log 2>/dev/null | head

# openrua op 41
python3 -u -c "
from rb import *
r=Robot()
q=hand2link8(q_down(0))
ok=r.move_to([-0.425,-0.089,1.30],q,4.0, via=[[-0.28,0.0,1.30]]); print('ok',ok)
print('hand', r.hand_pose()[0], 'fingers', r.fingers(), 'wrench', r.wrench()[0])
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py birdview >/dev/null && python3 tools/perception/cam_snap.py agentview >/dev/null && python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[130:300,180:420]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 42
python3 -u -c "
from rb import *
r=Robot()
q=hand2link8(q_down(0))
base=r.wrench()[0]
for z in (1.25, 1.243, 1.237, 1.231, 1.225, 1.219):
    ok=r.move_to([-0.425,-0.089,z],q,1.5)
    p=r.hand_pose()[0]; w=r.wrench()[0]
    print(f'z={z} hand={np.round(p,4)} fingers={np.round(r.fingers(),4)} dW={np.round(w-base,3)}')
    if abs(w[2]-base[2])>0.8 or np.abs(w[:2]-base[:2]).max()>0.8:
        print('CONTACT'); break
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py frontview >/dev/null && python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[120:380,100:400]; cv2.imwrite('front_crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
"

# openrua op 43
python3 -u -c "
from rb import *
r=Robot()
r.gripper(0.04)
print('fingers', r.fingers(), 'wrench', r.wrench()[0])
q=hand2link8(q_down(0))
ok=r.move_to([-0.425,-0.089,1.33],q,2.0); print('ok',ok)
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py frontview >/dev/null && python3 tools/perception/cam_snap.py birdview >/dev/null && python3 tools/perception/cam_snap.py agentview >/dev/null && python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[120:380,100:400]; cv2.imwrite('front_crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('birdview.png'); crop=im[130:300,180:420]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 44
python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null && python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
for v in range(158,214,3):
    print(v, ' '.join(f'{x:.2f}' for x in d[v,250:300:2]))
"

# openrua op 45
python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[130:300,100:400]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 46
python3 -u -c "
from rb import *
r=Robot()
q=hand2link8(q_down(0))
ok=r.move_to([-0.35,0.15,1.40],q,3.0); print('ok',ok)
r.close()
" 2>&1 | grep -v Warning && python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null && python3 tools/perception/cam_snap.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('bird_depth.npy')
im=cv2.imread('birdview.png'); crop=im[150:210,250:300]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=8,fy=8,interpolation=cv2.INTER_NEAREST))
for v in range(160,202,2):
    print(v, ' '.join(f'{x:.2f}' for x in d[v,254:298:2]))
"

# openrua op 47
python3 tools/perception/cam_snap.py birdview birdview.png 2>&1 | grep -v Warning | tail -2; python3 -c "
from PIL import Image; im=Image.open('birdview.png'); im.crop((230,130,330,230)).resize((400,400),Image.NEAREST).save('bird_crop.png')"

# openrua op 48
for c in agentview sideview frontview; do python3 tools/perception/cam_snap.py $c $c.png 2>&1 | grep -v Warning | tail -1; done; python3 -c "
from PIL import Image
for c,box in [('agentview',(150,150,450,400)),('sideview',(100,100,540,380)),('frontview',(150,150,450,400))]:
    im=Image.open(c+'.png'); print(c, im.size); im.crop(box).resize(((box[2]-box[0])*2,(box[3]-box[1])*2),Image.NEAREST).save(c[:5]+'_crop.png')"

# openrua op 49
python3 tools/perception/cam_snap.py /birdview/depth bird_depth.png 2>&1 | grep -v Warning | tail -1; python3 -c "
import numpy as np
d=np.load('bird_depth.npy'); print(d.shape, d.dtype)
np.set_printoptions(linewidth=250, precision=2)
rows=range(160,204,2); cols=range(240,300,2)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows:
    print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
"

# openrua op 50
cat /tmp/claude-1000/-workspace/87eaab89-2e66-4243-adda-cb7f16f84c99/tasks/bkd44j7g9.output | head; ls -la /workspace/bird_depth.npy; grep -n "depth\|topic" /workspace/tools/perception/cam_snap.py | head -20

# openrua op 51
timeout 20 ros2 topic list 2>/dev/null | grep -i birdview; grep -o '"[^"]*birdview[^"]*depth[^"]*"' /claude-config/projects/-workspace/87eaab89-2e66-4243-adda-cb7f16f84c99.jsonl | sort | uniq -c | head

# openrua op 52
timeout 120 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png 2>&1 | grep -v Warning | tail -1; python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
rows=range(158,204,2); cols=range(240,300,2)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows:
    print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
"

# openrua op 53
timeout 300 python3 -u -c "
from rb import *
r=Robot()
print('q', r.arm_q()); print('fingers', r.fingers()); print('hand', r.hand_pose())
r.gripper(0.04)
ok=r.move_to([-0.347,-0.116,1.30], hand2link8(q_down(-45)), 4.0)
print('ok',ok, r.hand_pose())
r.close()" 2>&1 | grep -v Warning; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png 2>&1 | grep -v Warning | tail -1

# openrua op 54
timeout 300 python3 -u -c "
from rb import *
r=Robot()
f0,_=r.wrench(); print('f0',f0)
for z in [1.24,1.21,1.19]:
    ok=r.move_to([-0.347,-0.116,z], hand2link8(q_down(-45)), 2.0)
    f,_=r.wrench(); print('z',z,'ok',ok,'df',f-f0, 'hand', r.hand_pose()[0])
    if abs(f[2]-f0[2])>0.8: print('CONTACT'); break
r.close()" 2>&1 | grep -v Warning; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png 2>&1 | grep -v Warning | tail -1

# openrua op 55
python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
rows=range(184,204); cols=range(272,296)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows:
    print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
"

# openrua op 56
python3 -c "
from PIL import Image, ImageDraw
im=Image.open('birdview.png').crop((250,160,300,205)).resize((800,720),Image.NEAREST)
dr=ImageDraw.Draw(im)
for i,u in enumerate(range(250,300,5)):
    x=i*80; dr.line([(x,0),(x,720)],fill=(255,0,0),width=1); dr.text((x+2,2),str(u),fill=(255,0,0))
for j,v in enumerate(range(160,205,5)):
    y=j*80; dr.line([(0,y),(800,y)],fill=(0,255,0),width=1); dr.text((2,y+2),str(v),fill=(0,255,0))
im.save('bird_grid.png')"

# openrua op 57
cat >> rb.py <<'EOF'


def q_axis(axis, deg):
    """quaternion (xyzw) for rotation of deg about world axis."""
    ax = np.array(axis, float); ax /= np.linalg.norm(ax)
    h = np.radians(deg) / 2
    return np.concatenate([ax * np.sin(h), [np.cos(h)]])


def q2R(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
EOF
timeout 300 python3 -u -c "
from rb import *
r=Robot()
r.move_to([-0.347,-0.116,1.30], hand2link8(q_down(-45)), 2.0)
t=[0.707,0.707,0]; a=np.array([0.455,-0.455,0.766])
qh=qmul(q_axis(t,40), q_down(-45)); R=q2R(qh)
print('hand z axis', R[:,2], 'finger axis', R[:,1])
P1=np.array([-0.341,-0.114,1.10]); tcp=P1-0.02*a; hand=tcp+0.1034*a
appr=hand+0.06*a
print('hand target', hand, 'approach', appr)
ok=r.move_to(appr, hand2link8(qh), 4.0)
p,q=r.hand_pose(); print('ok',ok,'hand',p, 'Rz', q2R(q)[:,2], 'Ry', q2R(q)[:,1])
f,_=r.wrench(); print('f',f)
r.close()" 2>&1 | grep -v Warning; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png 2>&1 | grep -v Warning | tail -1

# openrua op 58
timeout 300 python3 -u -c "
from rb import *
r=Robot()
t=[0.707,0.707,0]; a=np.array([0.455,-0.455,0.766])
qh=qmul(q_axis(t,40), q_down(-45)); q8=hand2link8(qh)
appr=np.array([-0.275753,-0.179247,1.2098444])
f0,_=r.wrench(); print('f0',f0)
for s in [0.03,0.05,0.06]:
    ok=r.move_to(appr-s*a, q8, 2.0)
    f,_=r.wrench(); print('s',s,'ok',ok,'df',f-f0,'|df|',np.linalg.norm(f-f0),'hand',r.hand_pose()[0])
    if np.linalg.norm(f-f0)>0.8: print('CONTACT'); break
r.close()" 2>&1 | grep -v Warning; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png 2>&1 | grep -v Warning | tail -1

# openrua op 59
timeout 300 python3 -u -c "
from rb import *
r=Robot()
r.gripper(0.0)
f,_=r.wrench(); print('f',f)
r.close()" 2>&1 | grep -v Warning

# openrua op 60
timeout 300 python3 -u -c "
from rb import *
r=Robot()
a=np.array([0.455,-0.455,0.766])
t=[0.707,0.707,0]; qh=qmul(q_axis(t,40), q_down(-45)); q8=hand2link8(qh)
p,_=r.hand_pose()
ok=r.move_to(p+0.03*a, q8, 2.0)
f,_=r.wrench(); print('lifted ok',ok,'f',f,'fingers',r.fingers(),'hand',r.hand_pose()[0])
r.close()" 2>&1 | grep -v Warning; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png 2>&1 | grep -v Warning | tail -1; python3 -c "
from PIL import Image
im=Image.open('agentview.png'); im.crop((150,100,450,300)).resize((600,400),Image.NEAREST).save('agent_crop.png')"

# openrua op 61
timeout 120 python3 tools/perception/cam_snap.py birdview birdview.png 2>&1 | grep -v Warning | tail -1; timeout 120 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png 2>&1 | grep -v Warning | tail -1; python3 -c "
from PIL import Image, ImageDraw
im=Image.open('birdview.png').crop((240,150,310,215)).resize((840,780),Image.NEAREST)
dr=ImageDraw.Draw(im)
for i,u in enumerate(range(240,310,5)):
    x=i*60; dr.line([(x,0),(x,780)],fill=(255,0,0),width=1); dr.text((x+2,2),str(u),fill=(255,0,0))
for j,v in enumerate(range(150,215,5)):
    y=j*60; dr.line([(0,y),(840,y)],fill=(0,255,0),width=1); dr.text((2,y+2),str(v),fill=(0,255,0))
im.save('bird_grid.png')
import numpy as np
d=np.load('bird_depth.npy')
rows=range(160,206,2); cols=range(248,304,2)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows:
    print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
"

# openrua op 62
timeout 300 python3 -u -c "
from rb import *
r=Robot()
p,_=r.hand_pose()
r.move_to([p[0],p[1],1.40], hand2link8(q_down(-45)), 3.0)
r.move_to([-0.30,0.20,1.45], hand2link8(q_down(0)), 3.0)
r.gripper(0.04)
r.close()" 2>&1 | grep -v Warning; for c in birdview agentview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c $c.png 2>&1 | grep -v Warning | tail -1; done; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null 2>&1; python3 -c "
from PIL import Image, ImageDraw
im=Image.open('birdview.png').crop((240,150,310,215)).resize((840,780),Image.NEAREST)
dr=ImageDraw.Draw(im)
for i,u in enumerate(range(240,310,5)):
    x=i*60; dr.line([(x,0),(x,780)],fill=(255,0,0),width=1); dr.text((x+2,2),str(u),fill=(255,0,0))
for j,v in enumerate(range(150,215,5)):
    y=j*60; dr.line([(0,y),(840,y)],fill=(0,255,0),width=1); dr.text((2,y+2),str(v),fill=(0,255,0))
im.save('bird_grid.png')
Image.open('agentview.png').crop((150,100,450,300)).resize((600,400),Image.NEAREST).save('agent_crop.png')
Image.open('sideview.png').crop((380,180,600,340)).resize((660,480),Image.NEAREST).save('sidev_crop.png')
import numpy as np
d=np.load('bird_depth.npy')
rows=range(162,206,2); cols=range(248,304,2)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows:
    print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
"

# openrua op 63
timeout 300 python3 -u -c "
from rb import *
r=Robot()
r.gripper(0.0)
q8=hand2link8(q_down(0))
ok=r.move_to([-0.43,-0.131,1.30], q8, 3.0); print('above',ok)
f0,_=r.wrench()
ok=r.move_to([-0.43,-0.131,1.143], q8, 2.0); print('down',ok, r.wrench()[0]-f0)
for x in [-0.40,-0.37,-0.34,-0.31,-0.28]:
    ok=r.move_to([x,-0.131,1.143], q8, 1.5)
    f,_=r.wrench(); print('x',x,'ok',ok,'df',f-f0,'hand',r.hand_pose()[0])
r.close()" 2>&1 | grep -v Warning

# openrua op 64
timeout 300 python3 -u -c "
from rb import *
r=Robot()
r.move_to([-0.30,0.20,1.45], hand2link8(q_down(0)), 3.0)
r.close()" 2>&1 | grep -v Warning; for c in birdview agentview; do timeout 60 python3 tools/perception/cam_snap.py $c $c.png 2>&1 | grep -v Warning | tail -1; done; python3 -c "
from PIL import Image
Image.open('birdview.png').crop((200,140,340,260)).resize((700,600),Image.NEAREST).save('bird_crop.png')
Image.open('agentview.png').crop((100,100,500,350)).resize((800,500),Image.NEAREST).save('agent_crop.png')"

# openrua op 65
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null 2>&1; python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
rows=range(206,250,2); cols=range(248,296,2)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows:
    print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
"; python3 -c "
from PIL import Image, ImageDraw
im=Image.open('birdview.png').crop((245,205,300,250)).resize((880,720),Image.NEAREST)
dr=ImageDraw.Draw(im)
for i,u in enumerate(range(245,300,5)):
    x=i*80; dr.line([(x,0),(x,720)],fill=(255,0,0),width=1); dr.text((x+2,2),str(u),fill=(255,0,0))
for j,v in enumerate(range(205,250,5)):
    y=j*80; dr.line([(0,y),(880,y)],fill=(0,255,0),width=1); dr.text((2,y+2),str(v),fill=(0,255,0))
im.save('bird_grid.png')"

# openrua op 66
timeout 60 ros2 topic echo /robot_description --once --field data 2>/dev/null > urdf.xml; wc -c urdf.xml; grep -n -i "finger\|hand" urdf.xml | grep -i "mesh\|box\|cylinder\|origin" | head -30

# openrua op 67
cd /tmp && python3 -c "
import numpy as np, struct
def stl_bounds(p):
    data=open(p,'rb').read()
    n=struct.unpack('<I',data[80:84])[0]
    arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
    v=arr['v'].reshape(-1,3); return v.min(0), v.max(0)
for f in ['hand','finger']:
    print(f, stl_bounds(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'))
"

# openrua op 68
find / -iname "hand*.stl" -o -iname "finger*.stl" -o -iname "hand*.dae" 2>/dev/null | grep -v proc | head; find / -path /proc -prune -o -iname "*.xml" -print 2>/dev/null | grep -i -E "gripper|panda|robosuite" | head

# openrua op 69
timeout 300 python3 -u -c "
from rb import *
r=Robot()
q8=hand2link8(q_down(90))
ok=r.move_to([-0.255,-0.223,1.25], q8, 3.0); print('above',ok)
f0,_=r.wrench()
ok=r.move_to([-0.255,-0.223,1.058], q8, 2.0); print('down',ok, r.wrench()[0]-f0)
for y in [-0.20,-0.18,-0.16,-0.14,-0.13]:
    ok=r.move_to([-0.255,y,1.058], q8, 1.5)
    f,_=r.wrench(); print('y',y,'ok',ok,'df',f-f0,'hand',r.hand_pose()[0])
r.move_to([-0.255,-0.13,1.25], q8, 2.0)
r.close()" 2>&1 | grep -v Warning

# openrua op 70
timeout 300 python3 -u -c "
from rb import *
r=Robot()
r.move_to([-0.30,0.20,1.45], hand2link8(q_down(0)), 3.0)
r.close()" 2>&1 | grep -v Warning; for c in birdview agentview; do timeout 60 python3 tools/perception/cam_snap.py $c $c.png 2>&1 | grep -v Warning | tail -1; done; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null 2>&1; python3 -c "
from PIL import Image, ImageDraw
im=Image.open('birdview.png').crop((245,200,305,250)).resize((960,800),Image.NEAREST)
dr=ImageDraw.Draw(im)
for i,u in enumerate(range(245,305,5)):
    x=i*80; dr.line([(x,0),(x,800)],fill=(255,0,0),width=1); dr.text((x+2,2),str(u),fill=(255,0,0))
for j,v in enumerate(range(200,250,5)):
    y=j*80; dr.line([(0,y),(960,y)],fill=(0,255,0),width=1); dr.text((2,y+2),str(v),fill=(0,255,0))
im.save('bird_grid.png')
Image.open('agentview.png').crop((100,100,500,350)).resize((800,500),Image.NEAREST).save('agent_crop.png')
import numpy as np
d=np.load('bird_depth.npy')
rows=range(206,246,2); cols=range(250,304,2)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows:
    print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
"

# openrua op 71
timeout 600 python3 -u -c "
from rb import *
r=Robot()
q8=hand2link8(q_down(0))
cy0=-0.1045; th0=np.radians(24); cz=0.93; Rh=0.074; rr=0.05; X=-0.255
def bar(th):
    cy=cy0+rr*(th-th0); return cy-Rh*np.cos(th), cz+Rh*np.sin(th)
# approach: block top below bar bottom
yb,zb=bar(th0); print('bar now',yb,zb)
r.gripper(0.0)
print(r.move_to([X,yb-0.035,1.25], q8, 3.0))
f0,_=r.wrench()
print(r.move_to([X,yb-0.035,1.02], q8, 2.0), r.wrench()[0]-f0)
print(r.move_to([X,yb,1.02], q8, 1.5), r.wrench()[0]-f0)
# arc: block top at zb+0.005 -> hand z = zb+0.005-0.02+0.1034
pts=[]; seed=r.arm_q()
for deg in [24,35,45,55,65,75,85,95]:
    th=np.radians(deg); yb,zb=bar(th); p=[X,yb,zb+0.005-0.02+0.1034]
    q=r.solve_ik(p,q8,seed)
    print(deg, np.round(p,4), 'ik', q is not None)
    if q is None: break
    pts.append(q); seed=q
if len(pts)==8:
    code,err=r.traj(pts,[1.5*(i+1) for i in range(8)])
    f,_=r.wrench(); print('after arc df',f-f0,'hand',r.hand_pose()[0])
r.close()" 2>&1 | grep -v Warning

# openrua op 72
timeout 300 python3 -u -c "
from rb import *
r=Robot()
r.move_to([-0.30,0.20,1.45], hand2link8(q_down(0)), 3.0)
r.close()" 2>&1 | grep -v Warning; for c in birdview agentview; do timeout 60 python3 tools/perception/cam_snap.py $c $c.png 2>&1 | grep -v Warning | tail -1; done; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null 2>&1; python3 -c "
from PIL import Image, ImageDraw
im=Image.open('birdview.png').crop((245,195,325,255)).resize((960,720),Image.NEAREST)
dr=ImageDraw.Draw(im)
for i,u in enumerate(range(245,325,5)):
    x=i*60; dr.line([(x,0),(x,720)],fill=(255,0,0),width=1); dr.text((x+2,2),str(u),fill=(255,0,0))
for j,v in enumerate(range(195,255,5)):
    y=j*60; dr.line([(0,y),(960,y)],fill=(0,255,0),width=1); dr.text((2,y+2),str(v),fill=(0,255,0))
im.save('bird_grid.png')
Image.open('agentview.png').crop((100,100,500,350)).resize((800,500),Image.NEAREST).save('agent_crop.png')
import numpy as np
d=np.load('bird_depth.npy')
rows=range(204,250,2); cols=range(256,320,2)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows:
    print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
"

# openrua op 73
python3 -c "
from PIL import Image, ImageDraw
im=Image.open('birdview.png').crop((285,195,345,245)).resize((960,800),Image.NEAREST)
dr=ImageDraw.Draw(im)
for i,u in enumerate(range(285,345,5)):
    x=i*80; dr.line([(x,0),(x,800)],fill=(255,0,0),width=1); dr.text((x+2,2),str(u),fill=(255,0,0))
for j,v in enumerate(range(195,245,5)):
    y=j*80; dr.line([(0,y),(960,y)],fill=(0,255,0),width=1); dr.text((2,y+2),str(v),fill=(0,255,0))
im.save('bird_grid.png')
import numpy as np
d=np.load('bird_depth.npy')
rows=range(206,240,2); cols=range(300,340,2)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows:
    print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
"

# openrua op 74
timeout 300 python3 -u -c "
from rb import *
r=Robot()
r.gripper(0.04)
q=hand2link8(q_down(0))
print(r.move_to([-0.262,-0.015,1.30], q, 4.0))
print('hand', r.hand_pose()[0], 'F', r.wrench()[0])
print(r.move_to([-0.262,-0.015,1.16], q, 3.0))
print('hand', r.hand_pose()[0], 'F', r.wrench()[0])
print(r.move_to([-0.262,-0.015,1.108], q, 3.0))
print('hand', r.hand_pose()[0], 'F', r.wrench()[0])
r.close()
" 2>&1 | grep -v Warning

# openrua op 75
grep -n -i "eye\|hand\|robot0" machine.yaml | head; timeout 60 python3 tools/perception/cam_snap.py eye_in_hand eih.png 2>&1 | tail -2 || true; ls -la eih.png

# openrua op 76
timeout 300 python3 -u -c "
from rb import *
r=Robot()
r.gripper(0.0)
print('F', r.wrench()[0])
r.close()
" 2>&1 | grep -v Warning

# openrua op 77
timeout 300 python3 -u -c "
from rb import *
r=Robot()
q=hand2link8(q_down(0))
print(r.move_to([-0.262,-0.015,1.16], q, 3.0))
print('fingers', r.fingers(), 'F', r.wrench()[0])
print(r.move_to([-0.262,-0.015,1.25], q, 3.0))
print('fingers', r.fingers(), 'F', r.wrench()[0])
r.close()
" 2>&1 | grep -v Warning; timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png 2>&1 | tail -1

# openrua op 78
timeout 600 python3 -u -c "
from rb import *
r=Robot()
q0=hand2link8(q_down(0)); q90=hand2link8(q_down(90))
print(r.move_to([-0.262,-0.015,1.35], q0, 3.0))
sol=r.solve_ik([-0.422,-0.143,1.35], q90)
print('ik90', sol is not None)
if sol is None:
    q90=hand2link8(q_down(-90)); print('ik-90', r.solve_ik([-0.422,-0.143,1.35], q90) is not None)
print(r.move_to([-0.422,-0.143,1.35], q90, 6.0, via=[[-0.34,-0.08,1.35]]))
print('hand', r.hand_pose()[0], 'fingers', r.fingers(), 'F', r.wrench()[0])
r.close()
" 2>&1 | grep -v Warning; timeout 60 python3 tools/perception/cam_snap.py birdview birdview.png 2>&1 | tail -1

# openrua op 79
python3 -c "
from PIL import Image, ImageDraw
im=Image.open('birdview.png').crop((235,150,335,230)).resize((1000,800),Image.NEAREST)
dr=ImageDraw.Draw(im)
for i,u in enumerate(range(235,335,5)):
    x=i*50; dr.line([(x,0),(x,800)],fill=(255,0,0),width=1); dr.text((x+2,2),str(u),fill=(255,0,0))
for j,v in enumerate(range(150,230,5)):
    y=j*50; dr.line([(0,y),(1000,y)],fill=(0,255,0),width=1); dr.text((2,y+2),str(v),fill=(0,255,0))
im.save('bird_grid.png')
"

# openrua op 80
timeout 60 python3 tools/perception/cam_snap.py agentview agentview.png 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py sideview sideview.png 2>&1 | tail -1; python3 -c "
from PIL import Image
Image.open('agentview.png').crop((180,80,420,300)).resize((720,660)).save('agent_crop.png')
Image.open('sideview.png').save('sidev_crop.png')"

# openrua op 81
timeout 600 python3 -u -c "
from rb import *
r=Robot()
q90=hand2link8(q_down(90))
for z in [1.30, 1.26, 1.24, 1.225, 1.21]:
    ok=r.move_to([-0.415,-0.143,z], q90, 3.0)
    p=r.hand_pose()[0]; F=r.wrench()[0]
    print(f'z={z} ok={ok} hand={p.round(4)} Fz={F[2]:.3f} F={F.round(3)}')
    if F[2] > -5.25: print('cup unloaded -> resting'); break
r.close()
" 2>&1 | grep -v Warning

# openrua op 82
timeout 600 python3 -u -c "
from rb import *
r=Robot()
q90=hand2link8(q_down(90))
r.gripper(0.04)
print('F', r.wrench()[0].round(3))
print(r.move_to([-0.415,-0.143,1.40], q90, 3.0))
print(r.move_to([-0.30,0.20,1.45], hand2link8(q_down(0)), 4.0))
print('F', r.wrench()[0].round(3), 'fingers', r.fingers())
r.close()
" 2>&1 | grep -v Warning; for c in birdview agentview frontview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c $c.png 2>&1 | tail -1; done; timeout 120 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png 2>&1 | tail -1

# openrua op 83
python3 -c "
from PIL import Image
Image.open('agentview.png').crop((100,120,400,300)).resize((900,540)).save('agent_crop.png')
Image.open('birdview.png').crop((240,150,340,230)).resize((1000,800),Image.NEAREST).save('bird_grid.png')
import numpy as np
d=np.load('bird_depth.npy')
cols=range(256,300,2); rows=range(160,206,2)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows: print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
"

# openrua op 84
timeout 600 python3 -u -c "
from rb import *
r=Robot()
r.gripper(0.0)
qh=qmul(q_axis([0,1,0],45), q_down(90)); q=hand2link8(qh)
R=q2R(qh); print('hand z axis', R[:,2].round(3), 'finger axis', R[:,0].round(3))
off=np.array([0.0731,0,0.0731])
def tip_to(tip, s=3.0):
    ok=r.move_to(np.array(tip)+off, q, s); p=r.hand_pose()[0]; F=r.wrench()[0]
    print(f'tip={np.round(tip,3)} ok={ok} hand={p.round(4)} F={F.round(2)}'); return F
tip_to([-0.325,-0.15,1.15], 4.0)
F0=tip_to([-0.325,-0.15,1.05])
for x in [-0.34,-0.352,-0.362,-0.372]:
    F=tip_to([x,-0.15,1.05], 2.0)
    print('  dF', (F-F0).round(2))
    if np.linalg.norm(F-F0)>15: print('high force, stop'); break
tip_to([-0.325,-0.15,1.15], 3.0)
print(r.move_to([-0.30,0.20,1.45], hand2link8(q_down(0)), 4.0))
r.gripper(0.04)
r.close()
" 2>&1 | grep -v Warning; for c in birdview agentview; do timeout 60 python3 tools/perception/cam_snap.py $c $c.png 2>&1 | tail -1; done; timeout 120 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png 2>&1 | tail -1

# openrua op 85
python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
cols=range(256,300,2); rows=range(160,200,2)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows: print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
from PIL import Image
Image.open('agentview.png').crop((100,120,400,300)).resize((900,540)).save('agent_crop.png')
"

# openrua op 86
timeout 300 python3 -u -c "
from rb import *
r=Robot()
print(r.move_to([-0.30,0.20,1.45], hand2link8(q_down(0)), 4.0))
print('hand', r.hand_pose()[0].round(3), 'fingers', r.fingers(), 'F', r.wrench()[0].round(2))
r.close()
" 2>&1 | grep -v Warning; for c in birdview agentview frontview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c $c.png 2>&1 | tail -1; done; timeout 120 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png 2>&1 | tail -1
python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
cols=range(254,300,2); rows=range(158,200,2)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows: print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
from PIL import Image
Image.open('agentview.png').crop((100,120,400,300)).resize((900,540)).save('agent_crop.png')
Image.open('birdview.png').crop((240,150,340,230)).resize((1000,800),Image.NEAREST).save('bird_grid.png')
"

# openrua op 87
timeout 300 python3 -u -c "
from rb import *
r=Robot()
print(r.move_to([-0.15,0.35,1.40], hand2link8(q_down(0)), 4.0))
print('hand', r.hand_pose()[0].round(3), 'fingers', r.fingers(), 'F', r.wrench()[0].round(2))
r.close()
" 2>&1 | grep -v Warning; for c in birdview agentview frontview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c $c.png 2>&1 | tail -1; done; timeout 120 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png 2>&1 | tail -1
python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
cols=range(254,300,2); rows=range(156,200,2)
print('     '+' '.join(f'{u:4d}' for u in cols))
for v in rows: print(f'{v:4d} '+' '.join(f'{3.0-d[v,u]:4.2f}' for u in cols))
from PIL import Image
Image.open('agentview.png').crop((100,120,400,300)).resize((900,540)).save('agent_crop.png')
Image.open('birdview.png').crop((240,150,340,230)).resize((1000,800),Image.NEAREST).save('bird_grid.png')
"

# openrua op 88
python3 -c "
from PIL import Image
Image.open('frontview.png').crop((150,100,500,330)).resize((875,575)).save('front_crop.png')"
