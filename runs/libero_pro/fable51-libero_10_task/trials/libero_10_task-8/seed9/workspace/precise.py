import numpy as np, rob

BASE_Q = np.array([0.7071068, 0.7071068, 0.0, 0.0])  # hand down, hand-y along world x

def hand_quat(yaw_deg):
    return rob.quat_mul(rob.quat_about_z(np.deg2rad(yaw_deg)), BASE_Q)

def yaw_of(quat):
    R = rob.quat_to_R(quat)
    return np.degrees(np.arctan2(R[1, 1], R[0, 1]))

def solve(r, tcp_target, yaw_deg, seed=None):
    """IK for TCP pose, then fix joint7 so hand-y yaw matches"""
    q = r.ik(tcp_target, hand_quat(yaw_deg), seed=seed, at_tcp=True)
    if q is None:
        return None
    for _ in range(3):
        _, quat = r.tcp(q)
        d = ((yaw_of(quat) - yaw_deg + 180) % 360) - 180
        if abs(d) < 0.05:
            break
        q[6] += np.deg2rad(d)  # dj7 = +x rotates hand-y by -x deg
        if q[6] > 2.85: q[6] -= np.pi / 2
        if q[6] < -2.85: q[6] += np.pi / 2
    return q

def goto(r, tcp_target, yaw_deg, seconds=3.0, tol=0.001, tries=6, seed=None):
    """iterate IK + trajectory until the FK TCP is within tol of target"""
    tcp_target = np.asarray(tcp_target, float)
    aim = tcp_target.copy()
    for i in range(tries):
        q = solve(r, aim, yaw_deg, seed=seed)
        if q is None:
            print("IK failed"); return False
        r.move(q, seconds=seconds)
        t, quat = r.tcp()
        err = tcp_target - t
        print(f"  goto {i}: tcp {t.round(4)} err {(err*1000).round(1)} mm yaw {yaw_of(quat):.2f}")
        if np.linalg.norm(err) < tol and abs(((yaw_of(quat) - yaw_deg + 180) % 360) - 180) < 0.3:
            return True
        aim = aim + err * 0.8  # compensate steady-state tracking offset
    return False
