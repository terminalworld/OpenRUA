from rob import *
from refine import refine
from geometry_msgs.msg import WrenchStamped
r = Robot()
wr={}
r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: wr.__setitem__('m',m), 1)
def wrench():
    wr.pop('m',None)
    for _ in range(50):
        rclpy.spin_once(r.node,timeout_sec=0.1)
        if 'm' in wr: break
    f=wr['m'].wrench.force; return np.round([f.x,f.y,f.z],2)
CX, CY, ang = 0.219, 0.337, math.radians(30.6)
def grasp_quat(ang, tilt_deg):
    c = np.array([math.cos(ang), math.sin(ang), 0.0])
    f = np.array([-math.sin(ang), math.cos(ang), 0.0])
    t = math.radians(tilt_deg)
    a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
    return quat_from_axes(a, -f)
q = grasp_quat(ang, 35)
r.gripper(0.04)
ok = r.move_tcp(CX, CY, 0.60, quat=q, secs=5)
if not ok: log("retry"); ok = r.move_tcp(CX, CY, 0.60, quat=q, secs=5)
res = refine(r, CX, CY, 0.44, 0.505, rad=0.08)
if res is not None:
    c, pts = res
    xy = pts[:, :2] - pts[:, :2].mean(0)
    w, v = np.linalg.eigh(xy.T @ xy)
    axis = v[:, 1]  # principal direction = can axis
    ang2 = math.atan2(axis[1], axis[0])
    if math.cos(ang2 - ang) < 0: ang2 += math.pi  # keep pointing away from base
    log(f"refined center {c[0]:.4f},{c[1]:.4f} axis angle {math.degrees(ang2):.1f} deg (was {math.degrees(ang):.1f})")
    CX, CY, ang = c[0], c[1], ang2
    q = grasp_quat(ang, 35)
    ok = r.move_tcp(CX, CY, 0.60, quat=q, secs=3)
log("wrench", wrench())
ok = r.move_tcp_line(CX, CY, 0.50, quat=q, secs=3, n=2)
log("wrench", wrench(), ok)
ok = r.move_tcp_line(CX, CY, 0.445, quat=q, secs=3, n=2)
log("wrench", wrench(), ok)
_,_,tcp=r.hand_pose(); log("tcp before close", tcp.round(4))
r.gripper(0.0); r.spin(0.5); f=r.fingers()
log(f"finger gap after close = {f[0]-f[1]:.4f}")
ok2 = r.move_tcp_line(CX, CY, 0.70, quat=q, secs=4, n=2)
if not ok2: log("retry lift"); r.move_tcp_line(CX, CY, 0.70, quat=q, secs=4, n=2)
r.spin(0.5); f=r.fingers(); log("fingers after lift", f, "gap", f[0]-f[1], "wrench", wrench())
log("PHASE5 DONE")
