"""Sample joint-space interpolation between configs and report link poses
that come near the microwave / door / table. Usage from python:
   from pathcheck import check; check(r, q0, q1)"""
import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionFK
from rob import JOINTS

LINKS = [f"panda_link{i}" for i in range(3, 9)] + ["panda_hand"]
RAD = {"panda_link3": 0.07, "panda_link4": 0.07, "panda_link5": 0.07, "panda_link6": 0.07,
       "panda_link7": 0.05, "panda_link8": 0.05, "panda_hand": 0.05}


def fk_links(r, q):
    req = GetPositionFK.Request(); req.header.frame_id = ""
    req.fk_link_names = LINKS
    req.robot_state.joint_state.name = list(JOINTS)
    req.robot_state.joint_state.position = [float(v) for v in q]
    r.fk_cli.wait_for_service(timeout_sec=10)
    for _ in range(3):
        fut = r.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
        if fut.result() is not None:
            break
    return {n: np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
            for n, ps in zip(LINKS, fut.result().pose_stamped)}


def hazards(p, rad):
    h = []
    x, y, z = p
    # microwave body x[-0.19,0.17] y[0.265,0.465] z<1.107
    if -0.19 - rad < x < 0.17 + rad and 0.265 - rad < y < 0.465 + rad and z < 1.107 + rad:
        h.append("MICROWAVE")
    # open door: segment (-0.19,0.265)->(-0.30,0.03), z<1.1
    a, b = np.array([-0.19, 0.265]), np.array([-0.30, 0.03])
    t = np.clip(((np.array([x, y]) - a) @ (b - a)) / ((b - a) @ (b - a)), 0, 1)
    if np.linalg.norm(np.array([x, y]) - (a + t * (b - a))) < rad and z < 1.1 + rad:
        h.append("DOOR")
    if z < 0.90 + rad and x > -0.55:
        h.append("TABLE")
    return h


def check(r, q0, q1, n=8, verbose=False):
    bad = False
    for i in range(n + 1):
        q = q0 + (q1 - q0) * i / n
        L = fk_links(r, q)
        zmin = min(p[2] for p in L.values())
        flags = {k: hazards(p, RAD[k]) for k, p in L.items()}
        flags = {k: v for k, v in flags.items() if v}
        if flags or verbose:
            print(f"  step {i}/{n}: zmin={zmin:.3f} " + " ".join(f"{k[-5:]}{v}@{np.round(L[k],2).tolist()}" for k, v in flags.items()))
        bad |= bool(flags)
    print("  path", "HAZARD" if bad else "clear")
    return not bad
