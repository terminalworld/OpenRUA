#!/usr/bin/env python3
"""Pick-and-place helper for this Panda (world-frame, hand pointing down).

Usage:
  python3 pp.py pose                      # report TCP pose + finger gap
  python3 pp.py move <x> <y> <z> [secs]   # TCP (fingertip centre) to world xyz,
                                          # hand down, fingers along world Y
  python3 pp.py grip <per_finger_m>       # 0.04 open, 0.0 close
  python3 pp.py seq <step>;<step>;...     # several of the above in one process

Facts established on this machine (see notes): /compute_ik solves for
panda_link8 in the WORLD frame; panda_link8 = panda_hand * rot_z(+45deg);
TCP is tcp_offset_m along hand +Z (which points down when grasping).
"""
import sys, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open('/workspace/machine.yaml'))
FJT = next(a for a in M['actuators'] if a['kind'] == 'joint_trajectory')
GRIP = next(a for a in M['actuators'] if a['kind'] == 'gripper')
JOINTS = FJT['joints']
TCP = float(M['hand']['tcp_offset_m'])
# link8 orientation for hand pointing down with fingers along world Y
Q_DOWN = (0.9238795, -0.3826834, 0.0, 0.0)


class PP:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node('pp')
        self.js = None
        self.n.create_subscription(JointState, '/joint_states', self._js, 1)
        self.ik = self.n.create_client(GetPositionIK, M['planning']['ik_service'])
        self.fk = self.n.create_client(GetPositionFK, '/compute_fk')
        self.fjt = ActionClient(self.n, FollowJointTrajectory, FJT['port'])
        self.gr = ActionClient(self.n, GripperCommand, GRIP['port'])
        assert self.ik.wait_for_service(20) and self.fk.wait_for_service(20)
        assert self.fjt.wait_for_server(20) and self.gr.wait_for_server(20)
        self.fresh_js()

    def _js(self, m):
        self.js = m

    def fresh_js(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self.n, timeout_sec=0.2)
        return dict(zip(self.js.name, self.js.position))

    def arm_seed(self):
        d = self.fresh_js()
        s = JointState()
        for j in JOINTS:
            s.name.append(j); s.position.append(float(d[j]))
        return s

    def pose(self):
        d = self.fresh_js()
        req = GetPositionFK.Request()
        req.fk_link_names = ['panda_link8']
        req.robot_state.joint_state = self.arm_seed()
        f = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=60)
        p = f.result().pose_stamped[0].pose
        q = p.orientation
        R = quat_R(q.x, q.y, q.z, q.w)
        tcp = np.array([p.position.x, p.position.y, p.position.z]) + TCP * R[:, 2]
        fingers = (d['panda_finger_joint1'], d['panda_finger_joint2'])
        print(f"TCP world=({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}) "
              f"link8 q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f}) "
              f"fingers=({fingers[0]:.4f},{fingers[1]:.4f}) "
              f"joints={[round(d[j],3) for j in JOINTS]}")
        return tcp

    def move(self, x, y, z, secs=3.0, q8=Q_DOWN):
        R = quat_R(*q8)
        tgt = np.array([x, y, z]) - TCP * R[:, 2]  # link8 origin
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = M['planning']['group']
        r.pose_stamped.header.frame_id = ''
        p = r.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, tgt)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
        r.robot_state.joint_state = self.arm_seed()
        r.avoid_collisions = False
        f = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=60)
        res = f.result()
        if res is None or res.error_code.val != 1:
            print(f"IK FAILED for TCP ({x},{y},{z}): code="
                  f"{None if res is None else res.error_code.val}; no motion")
            return False
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        pos = [float(sol[j]) for j in JOINTS]
        print(f"IK ok -> {[round(v,3) for v in pos]}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=pos)
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        t0 = time.time()
        sf = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, sf)
        rf = sf.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf)
        code = rf.result().result.error_code
        print(f"FJT done error_code={code} ({time.time()-t0:.1f}s wall)")
        d = self.fresh_js()
        err = max(abs(d[j] - v) for j, v in zip(JOINTS, pos))
        print(f"max joint err vs target: {err:.4f} rad")
        tcp = self.pose()
        print(f"TCP err vs target: {np.linalg.norm(tcp - np.array([x,y,z])):.4f} m")
        return code == 0

    def grip(self, w):
        goal = GripperCommand.Goal()
        goal.command.position = float(w)
        goal.command.max_effort = float(GRIP.get('max_effort', 30.0))
        sf = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, sf, timeout_sec=60)
        rf = sf.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=180)
        res = rf.result().result
        d = self.fresh_js()
        print(f"gripper -> {w}: reached_goal={res.reached_goal} stalled={res.stalled} "
              f"fingers=({d['panda_finger_joint1']:.4f},{d['panda_finger_joint2']:.4f})")

    def run(self, step):
        a = step.split()
        if a[0] == 'pose':
            self.pose()
        elif a[0] == 'move':
            ok = self.move(*map(float, a[1:4]), *(map(float, a[4:5])))
            if not ok:
                raise SystemExit('move failed; stopping sequence')
        elif a[0] == 'movq':  # movq x y z hqx hqy hqz hqw secs  (HAND quaternion)
            x, y, z, hx, hy, hz, hw, secs = map(float, a[1:9])
            # link8 = hand * rot_z(+45deg)
            q8 = quat_mul((hx, hy, hz, hw), (0.0, 0.0, 0.3826834, 0.9238795))
            if not self.move(x, y, z, secs, q8=q8):
                raise SystemExit('movq failed; stopping sequence')
        elif a[0] == 'grip':
            self.grip(float(a[1]))
        else:
            raise SystemExit(f'unknown step {step}')


def quat_mul(a, b):
    """Hamilton product a*b, quaternions as (x, y, z, w)."""
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    pp = PP()
    steps = [' '.join(sys.argv[1:])] if sys.argv[1] != 'seq' else \
        [s.strip() for s in ' '.join(sys.argv[2:]).split(';') if s.strip()]
    for s in steps:
        print(f"== {s}", flush=True)
        pp.run(s)
    print("SEQUENCE COMPLETE", flush=True)
