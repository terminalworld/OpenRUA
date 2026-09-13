from rk import *
C = np.array([-0.055, -0.263])          # moka pot centre (x,y) world
B = np.array([-0.057, 0.207])           # burner centre
GRASP_Z = 0.965                         # hand z at grasp (pads 0.955-0.975, just above collar)
TIP_PAST = 0.015                        # fingertip past pot centre along approach
PHI = np.radians(45.0)                  # approach yaw: hand comes from (-x,+y) side
D = np.array([np.sin(PHI), -np.cos(PHI), 0.0])   # approach direction (hand z)
CC = np.array([np.cos(PHI), np.sin(PHI), 0.0])   # closing direction
Q_GRASP = hand_quat(D, +CC)             # hand x DOWN (this IK family has good joint margins)
HAND_GRASP = np.array([C[0], C[1], GRASP_Z]) + (TIP_PAST - 0.1034) * D
HAND_PRE = HAND_GRASP - 0.08 * D
SEED_GRASP = [0.38, 1.03, -0.65, -1.96, -2.33, 2.13, 0.99]
# place: approach from +y (phi=0) so the palm stays clear of the knob
PHI_P = np.radians(15.0)
DP = np.array([np.sin(PHI_P), -np.cos(PHI_P), 0.0]); CP = np.array([np.cos(PHI_P), np.sin(PHI_P), 0.0])
Q_PLACE = hand_quat(DP, +CP)
POT_OFF = 0.1034 - TIP_PAST             # pot centre = hand + POT_OFF * d
def wrench(node_robot, n=3):
    from geometry_msgs.msg import WrenchStamped
    got = []
    sub = node_robot.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", got.append, 5)
    end = time.time() + 5
    while len(got) < n and time.time() < end: rclpy.spin_once(node_robot.node, timeout_sec=0.2)
    node_robot.node.destroy_subscription(sub)
    if not got: return None
    w = got[-1].wrench
    return np.round([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z], 2)
