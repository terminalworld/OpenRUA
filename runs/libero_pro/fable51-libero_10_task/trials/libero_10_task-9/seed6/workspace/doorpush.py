"""Close the microwave door by pushing its outer face with the closed fingertips along an arc about the hinge.
usage: doorpush.py th0 th1 [--go]   (without --go: plan only)"""
import sys, numpy as np, rclpy, scene
from rob import *; from plan import Planner
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState

th0, th1 = float(sys.argv[1]), float(sys.argv[2]); GO = "--go" in sys.argv
LEAN, RHO, Z, OFF = 15.0, 0.12, 1.075, -0.040
H = np.array([-0.165, 0.27]); DOOR_OFF = -0.013          # door centre line offset from hinge line (birdview)
START_Q = [-1.032, -0.943, 1.63, -2.566, 1.209, 2.377, 0.879]   # doorplan3 lean15 z1.07 full-arc start (margin 0.50)
lim = np.array(ARM["limits_rad"])

def dn(th_deg):
    th = np.radians(th_deg)
    return np.array([np.cos(th), np.sin(th), 0]), np.array([-np.sin(th), np.cos(th), 0])
def pose(th_deg, back=0.0):
    d, n1 = dn(th_deg)
    tcp = np.array([*H, 0]) + RHO*d + (OFF - back)*n1; tcp[2] = Z
    b = np.radians(LEAN); a = np.sin(b)*n1 + np.array([0, 0, -np.cos(b)])
    return tcp, R_from_axes(a, d)
def door_box(th_deg, thick=0.035):
    d, n1 = dn(th_deg); c = np.array([*H, 1.0035]) + 0.15*d + DOOR_OFF*n1
    return scene.box("mw_door", c, (0.30, thick, 0.207), yaw=np.radians(th_deg))

r = Planner("doorpush")
sv = r.node.create_client(GetStateValidity, "/check_state_validity")
def valid(q):
    req = GetStateValidity.Request(); req.group_name = M["planning"]["group"]
    rs = RobotState(); rs.joint_state.name = list(JOINTS) + ["panda_finger_joint1", "panda_finger_joint2"]
    rs.joint_state.position = [float(v) for v in q] + [0.0, 0.0]; req.robot_state = rs
    res = r._call(sv, req)
    bad = [(c.contact_body_1, c.contact_body_2) for c in res.contacts]
    return res.valid, bad

# --- plan
tcp_c, Rc = pose(th0); tcp_p, _ = pose(th0, back=0.04)
q_c = r.solve_ik(hand_pose_from_tcp(tcp_c, Rc), Rc, seed=START_Q, timeout=2.0)
q_p = r.solve_ik(hand_pose_from_tcp(tcp_p, Rc), Rc, seed=q_c or START_Q, timeout=2.0)
print("contact q", np.round(q_c, 3) if q_c else None, "\npre q", np.round(q_p, 3) if q_p else None)
if q_c is None or q_p is None: sys.exit("no IK")
scene.apply(r.node, [door_box(th0, thick=0.03)])          # thin door (face 2.5 mm back): tests hand body vs door, not the tip touch
print("contact state valid (thin door in scene):", valid(q_c))
print("pre state valid:", valid(q_p))
scene.apply(r.node, [door_box(th0)])
ths = np.arange(th0, th1 + 0.1, 4.0)
arc = [(hand_pose_from_tcp(*pose(t)), pose(t)[1]) for t in ths]
scene.apply(r.node, [scene.remove("mw_door")])
traj, frac = r.cartesian(arc, avoid=True, start_q=q_c)
qs = np.array([p.positions for p in traj.points]); m2 = np.min(np.minimum(qs - lim[:, 0], lim[:, 1] - qs))
print(f"arc plan from contact q: fraction {frac:.2f}, limit margin {m2:.3f}")
scene.apply(r.node, [door_box(th0)])
if not GO or frac < 0.99: sys.exit("plan only" if not GO else "arc incomplete")

# --- execute
print("== closing gripper (fingertips as pusher)"); r.gripper(0.0)
p, R = r.tcp(); print("tcp now", np.round(p, 3))
print("== lifting clear of the opening")
if not r.move_line_tcp([p + [0, 0, 0.09], p + [0, -0.08, 0.09]], R, avoid=True, min_fraction=0.95): sys.exit("retreat failed")
print("== joint move to pre-push pose")
if not r.goto_joints(q_p): sys.exit("goto pre failed")
print("== approach to contact")
if not r.move_line_tcp([tcp_c], Rc, avoid=False): sys.exit("approach failed")
scene.apply(r.node, [scene.remove("mw_door")])
print("== pushing arc", th0, "->", th1)
traj, frac = r.cartesian(arc, avoid=True)
if frac < 0.99: sys.exit(f"arc fraction {frac}")
code, err = r.execute(traj); p, R = r.tcp(); print("tcp after arc", np.round(p, 3), "err", round(err, 4))
print("== retreat")
d1, n11 = dn(th1)
r.move_line_tcp([p - 0.05*n11, p - 0.05*n11 + [0, 0, 0.06]], R, avoid=False, min_fraction=0.9)
print("done; joints", np.round(r.joints(), 3))
