"""Cartesian-interpolated move for the TCP: waypoints via IK (seeded sequentially)."""
from rlib import *
from scipy.spatial.transform import Rotation, Slerp

def cart_path(r, tcp0, R0, tcp1, R1, n):
    """returns list of joint solutions along a straight TCP line with slerp rotation."""
    key = Rotation.from_matrix([R0, R1]); sl = Slerp([0, 1], key)
    seed = r.joints(); qs = []
    for s in np.linspace(0, 1, n + 1)[1:]:
        Rs = sl(s).as_matrix()
        tcp = (1 - s) * np.asarray(tcp0) + s * np.asarray(tcp1)
        hand = hand_pose_from_tcp(tcp, Rs)
        q = r.ik_world(hand, quat_from_R(Rs), seed=seed, attempts=2)
        if q is None:
            raise RuntimeError(f"IK failed at s={s:.2f} tcp={tcp}")
        jump = max(abs(a - b) for a, b in zip(q, seed))
        print(f"  s={s:.2f} tcp={np.round(tcp,3)} jump={jump:.3f} q={np.round(q,2)}")
        seed = q; qs.append(q)
    return qs

def current_tcp(r, R):
    pos, _ = r.fk_world()
    return pos + M["hand"]["tcp_offset_m"] * R[:, 2]

def run(qs, r, seconds, retries=3):
    for i in range(retries):
        code, err = r.move_joints(qs, seconds)
        if err < 0.01: return True
        qs = [qs[-1]]; seconds = max(3.0, seconds/2)
    return False
