#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/ik_move.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
timeout 30 ros2 topic list; echo ---ACTIONS; timeout 20 ros2 action list; echo ---SERVICES; timeout 20 ros2 service list | grep -v -E "parameter|describe_|get_type" ; echo ---NODES; timeout 20 ros2 node list

# openrua op 3
timeout 20 ros2 topic echo /joint_states --once; echo ---; timeout 10 ros2 topic echo /pipeline_state --once; echo ---; timeout 10 ros2 topic info /pipeline_state

# openrua op 4
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once --qos-durability transient_local --qos-reliability reliable 2>/dev/null | grep -E "frame_id|x:|y:|z:|w:" | head -120

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
t0=time.time()
while time.time()-t0 < 5: rclpy.spin_once(node, timeout_sec=0.2)
for (p,c),tr in sorted(seen.items()):
    print(f"{p} -> {c}: t=({tr.translation.x:.4f},{tr.translation.y:.4f},{tr.translation.z:.4f}) q=({tr.rotation.x:.4f},{tr.rotation.y:.4f},{tr.rotation.z:.4f},{tr.rotation.w:.4f})")
EOF
timeout 30 python3 tfdump.py

# openrua op 7
cat > cloud.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
qos = QoSProfile(depth=100); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
def cb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
while not all(k in got for k in "dci") or "tf" not in got: rclpy.spin_once(node, timeout_sec=0.2)
b = CvBridge()
depth = b.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float32)
color = b.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
tr = got["tf"]; q = tr.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
X = (uu-cx)*depth/fx; Y = (vv-cy)*depth/fy
P = np.stack([X,Y,depth],-1) @ R.T + t
np.save(f"{cam}_world.npy", P); np.save(f"{cam}_color.npy", color)
print("saved", P.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
EOF
timeout 60 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy")
Z = P[...,2]
print("z percentiles", np.nanpercentile(Z,[1,5,25,50,75,95,99]))
# table height = mode of z within the table region
import collections
zs = np.round(Z[np.isfinite(Z)],3)
c = collections.Counter(zs.tolist()).most_common(8); print(c)
EOF

# openrua op 8
python3 - <<'EOF'
import numpy as np, cv2
P = np.load("birdview_world.npy"); Z = P[...,2]
mask = (Z > 0.905) & (Z < 1.2) & (P[...,0] > -0.35)  # exclude robot near x<-0.35
mask = mask.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 20: continue
    m = lab == i
    pts = P[m]
    print(f"comp {i}: area={stats[i,4]} px centroid=({cents[i][0]:.0f},{cents[i][1]:.0f}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f}")
EOF

# openrua op 9
python3 - <<'EOF'
import numpy as np, cv2
P = np.load("birdview_world.npy"); Z = P[...,2]; C = np.load("birdview_color.npy")
reg = (P[...,0] > -0.2) & (P[...,0] < 0.15) & (P[...,1] > -0.15) & (P[...,1] < 0.32)
for lo, hi in [(0.905,0.915),(0.915,0.935),(0.935,0.96),(0.96,0.99),(0.99,1.03)]:
    m = reg & (Z>lo) & (Z<hi)
    if m.sum()<10: print(lo,hi,"none"); continue
    pts = P[m]
    print(f"z[{lo},{hi}] n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) color={C[m].mean(0).astype(int)}")
# Gray (stove plate) vs black (pan): use brightness
gray = reg & (Z>0.905) & (Z<0.95) & (C.mean(-1) > 90)
pts = P[gray]; print("gray plate:", gray.sum(), pts[:,0].min(), pts[:,0].max(), pts[:,1].min(), pts[:,1].max(), "center", pts[:,0].mean(), pts[:,1].mean())
dark = reg & (Z>0.905) & (C.mean(-1) < 90)
pts = P[dark]; print("dark:", dark.sum(), pts[:,0].min(), pts[:,0].max(), pts[:,1].min(), pts[:,1].max())
# handle = dark pixels with y > 0.12 (over stove)
h = dark & (P[...,1] > 0.10)
pts = P[h]; print("handle over stove:", h.sum(), "x", pts[:,0].min(), pts[:,0].max(), "y", pts[:,1].min(), pts[:,1].max(), "z", pts[:,2].min(), pts[:,2].max())
EOF

# openrua op 10
python3 - <<'EOF'
import numpy as np, cv2
P = np.load("birdview_world.npy"); Z = P[...,2]; C = np.load("birdview_color.npy")
reg = (P[...,0] > -0.2) & (P[...,0] < 0.15) & (P[...,1] > 0.07) & (P[...,1] < 0.32)
h = reg & (Z>0.945) & (C.mean(-1) < 90)
pts = P[h]; print("handle:", h.sum(), "x", pts[:,0].min(), pts[:,0].max(), "y", pts[:,1].min(), pts[:,1].max(), "z", pts[:,2].min(), pts[:,2].max())
for y0 in np.arange(0.07,0.26,0.02):
    m = h & (P[...,1]>=y0) & (P[...,1]<y0+0.02)
    if m.sum(): print(f" y {y0:.2f}: x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] z[{P[m][:,2].min():.3f},{P[m][:,2].max():.3f}] n={m.sum()}")
# burner coil: dark on plate, z<0.945
b = reg & (Z>0.905)&(Z<0.945) & (C.mean(-1)<90) & (P[...,1]>0.11)
pts=P[b]; print("burner dark:", b.sum(), "x", pts[:,0].min(), pts[:,0].max(), "y", pts[:,1].min(), pts[:,1].max(), "center", pts[:,0].mean(), pts[:,1].mean(), "z", np.median(pts[:,2]))
# knob
k = (P[...,0] > -0.3) & (P[...,0] < -0.15) & (P[...,1] > 0.12) & (P[...,1] < 0.3) & (Z>0.93)
pts=P[k]; print("knob:", k.sum(), "x", pts[:,0].min(), pts[:,0].max(), "y", pts[:,1].min(), pts[:,1].max(), "center", pts[:,0].mean(), pts[:,1].mean(), "z", pts[:,2].min(), pts[:,2].max())
for lo in [0.93,0.94,0.95]:
    m = k & (Z>lo); pts=P[m]; print(f"  knob z>{lo}: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] c=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
# moka
mk = (P[...,0] > -0.15) & (P[...,0] < 0.1) & (P[...,1] > -0.4) & (P[...,1] < -0.15) & (Z>0.91)
pts=P[mk]; print("moka:", mk.sum(), "x", pts[:,0].min(), pts[:,0].max(), "y", pts[:,1].min(), pts[:,1].max(), "center", pts[:,0].mean(), pts[:,1].mean(), "z", pts[:,2].min(), pts[:,2].max())
for lo,hi in [(0.91,0.95),(0.95,1.0),(1.0,1.04),(1.04,1.07)]:
    m = mk&(Z>lo)&(Z<hi); 
    if m.sum(): pts=P[m]; print(f"  moka z[{lo},{hi}]: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] c=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
EOF

# openrua op 11
timeout 60 python3 cloud.py agentview && timeout 60 python3 cloud.py sideview && python3 - <<'EOF'
import numpy as np
for cam in ["birdview","agentview","sideview"]:
    P = np.load(f"{cam}_world.npy"); Z=P[...,2]; C=np.load(f"{cam}_color.npy")
    k = (P[...,0] > -0.32) & (P[...,0] < -0.15) & (P[...,1] > 0.10) & (P[...,1] < 0.3) & (Z>0.905) & (Z<1.0)
    pts=P[k]
    print(cam, "knob n=",k.sum(), "x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f] c=(%.3f,%.3f)"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max(),pts[:,0].mean(),pts[:,1].mean()))
    for lo,hi in [(0.905,0.92),(0.92,0.935),(0.935,0.95),(0.95,0.96),(0.96,0.98)]:
        m=k&(Z>=lo)&(Z<hi)
        if m.sum(): pts=P[m]; print(f"   z[{lo},{hi}] n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] c=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) col={C[m].mean(0).astype(int)}")
EOF

# openrua op 12
timeout 20 ros2 node info /robot_bridge; timeout 10 ros2 interface show moveit_msgs/msg/PipelineState | head -20

# openrua op 13
cat > fk.py <<'EOF'
import sys, rclpy, yaml
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open("/workspace/machine.yaml"))
arm = M["actuators"][0]["joints"]
rclpy.init(); node = rclpy.create_node("fk")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
cli = node.create_client(GetPositionFK, "/compute_fk"); cli.wait_for_service()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
seed = JointState()
if len(sys.argv) > 1:
    seed.name = arm; seed.position = [float(x) for x in sys.argv[1].split(",")]
else:
    for n,p in zip(js["m"].name, js["m"].position):
        if n in arm: seed.name.append(n); seed.position.append(p)
req.robot_state.joint_state = seed
fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
r = fut.result()
p = r.pose_stamped[0].pose
print("frame", r.pose_stamped[0].header.frame_id, "err", r.error_code.val)
print(f"hand(base) pos=({p.position.x:.4f},{p.position.y:.4f},{p.position.z:.4f}) q=({p.orientation.x:.4f},{p.orientation.y:.4f},{p.orientation.z:.4f},{p.orientation.w:.4f})")
print(f"hand(world) pos=({p.position.x-0.66:.4f},{p.position.y:.4f},{p.position.z+0.912:.4f})")
print("joints", dict(zip(seed.name, [round(x,4) for x in seed.position])))
EOF
timeout 60 python3 fk.py

# openrua op 14
cat > ik.py <<'EOF'
"""ik.py x y z qx qy qz qw  -> prints joint solution (no motion). Pose in whatever frame IK uses."""
import sys, rclpy, yaml
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open("/workspace/machine.yaml"))
arm = M["actuators"][0]["joints"]
x,y,z,qx,qy,qz,qw = map(float, sys.argv[1:8])
rclpy.init(); node = rclpy.create_node("ik")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
cli = node.create_client(GetPositionIK, "/compute_ik"); cli.wait_for_service()
req = GetPositionIK.Request()
req.ik_request.group_name = "panda_arm"
req.ik_request.pose_stamped.header.frame_id = ""
p = req.ik_request.pose_stamped.pose
p.position.x, p.position.y, p.position.z = x,y,z
p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx,qy,qz,qw
seed = JointState()
if len(sys.argv) > 8:
    seed.name = arm; seed.position = [float(v) for v in sys.argv[8].split(",")]
else:
    for n,pp in zip(js["m"].name, js["m"].position):
        if n in arm: seed.name.append(n); seed.position.append(pp)
req.ik_request.robot_state.joint_state = seed
req.ik_request.avoid_collisions = False
req.ik_request.timeout.sec = 2
fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
r = fut.result()
print("err", r.error_code.val)
if r.error_code.val == 1:
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    print(",".join(f"{sol[j]:.5f}" for j in arm))
EOF
echo "world-frame pose:"; timeout 60 python3 ik.py -0.2030 0 1.2696 0.9996 0 -0.0284 0
echo "base-frame pose:"; timeout 60 python3 ik.py 0.457 0 0.3576 0.9996 0 -0.0284 0

# openrua op 15
timeout 60 python3 fk.py 0.05846,-0.16131,-0.05792,-2.44466,-0.01162,2.22677,0.00891

# openrua op 16
echo "same pose, seed = home:"; timeout 60 python3 ik.py -0.2030 0 1.2696 0.9996 0 -0.0284 0 0,-0.161,0,-2.4446,0,2.2268,0.7854
echo "pose with q=(1,0,0,0):"; timeout 60 python3 ik.py -0.2030 0 1.2696 1 0 0 0
echo "pose with q=(0.7071,0.7071,0,0) (hand x along world diagonal):"; timeout 60 python3 ik.py -0.2030 0 1.2696 0.7071 0.7071 0 0

# openrua op 17
cat > /workspace/move.py <<'EOF'
#!/usr/bin/env python3
"""move.py x y z yaw_deg [--sec S] [--hand] [--dry] [--j7 VAL]
Top-down hand (hand z pointing down) at world TCP position (x,y,z) with hand
x-axis yawed yaw_deg about world z (yaw 0 => fingers span world y).
Uses IK (tip = panda_link8, 45deg offset handled), FK-verifies, sends FJT.
--hand: (x,y,z) is the hand-frame origin instead of the TCP.
"""
import sys, math, time, subprocess
import numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = M["actuators"][0]["joints"]
TCP = M["hand"]["tcp_offset_m"]

def parse():
    a = sys.argv[1:]
    opts = {"sec": 4.0, "hand": False, "dry": False, "j7": None}
    pos = []
    i = 0
    while i < len(a):
        if a[i] == "--sec": opts["sec"] = float(a[i+1]); i += 2
        elif a[i] == "--j7": opts["j7"] = float(a[i+1]); i += 2
        elif a[i] == "--hand": opts["hand"] = True; i += 1
        elif a[i] == "--dry": opts["dry"] = True; i += 1
        else: pos.append(float(a[i])); i += 1
    return pos, opts

def main():
    pos, o = parse()
    x, y, z, yaw = pos
    if not o["hand"]: z += TCP
    R_hand = Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    R_l8 = R_hand * Rot.from_euler("z", 45, degrees=True)
    q = R_l8.as_quat()  # x,y,z,w
    rclpy.init(); node = rclpy.create_node("mv")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 1)
    while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
    cur = {n: p for n, p in zip(js["m"].name, js["m"].position)}
    ik = node.create_client(GetPositionIK, "/compute_ik"); ik.wait_for_service()
    fk = node.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service()
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    seed = JointState(); seed.name = list(ARM); seed.position = [cur[j] for j in ARM]
    req.ik_request.robot_state.joint_state = seed
    req.ik_request.avoid_collisions = False
    req.ik_request.timeout.sec = 3
    fut = ik.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    r = fut.result()
    if r is None or r.error_code.val != 1:
        raise SystemExit(f"IK failed: {None if r is None else r.error_code.val}")
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    target = [sol[j] for j in ARM]
    if o["j7"] is not None: target[6] = o["j7"]
    # FK verify
    fr = GetPositionFK.Request(); fr.fk_link_names = ["panda_hand"]
    fr.robot_state.joint_state.name = list(ARM); fr.robot_state.joint_state.position = target
    fut = fk.call_async(fr); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    hp = fut.result().pose_stamped[0].pose
    hq = Rot.from_quat([hp.orientation.x, hp.orientation.y, hp.orientation.z, hp.orientation.w])
    tcp = np.array([hp.position.x, hp.position.y, hp.position.z]) + hq.apply([0, 0, TCP])
    ang = (hq.inv() * R_hand).magnitude() * 180 / math.pi
    perr = np.linalg.norm(tcp - np.array([x, y, z + (TCP if o["hand"] else 0) - TCP]))
    print("target joints:", ",".join(f"{v:.4f}" for v in target))
    print(f"FK check: tcp=({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}) pos_err={perr*1000:.1f}mm orient_err={ang:.1f}deg")
    dj = np.abs(np.array(target) - np.array(seed.position)); print("max joint delta: %.2f rad" % dj.max())
    if perr > 0.005 or ang > 3:
        raise SystemExit("FK mismatch; not moving")
    if o["dry"]: return
    rclpy.shutdown()
    rc = subprocess.run([sys.executable, "/workspace/tools/action/fjt_send.py",
                         ",".join(f"{v:.6f}" for v in target), str(o["sec"])])
    subprocess.run([sys.executable, "/workspace/fk.py"])

if __name__ == "__main__":
    main()
EOF
timeout 120 python3 move.py -0.214 0.205 1.03 0 --dry

# openrua op 18
timeout 300 python3 tools/action/gripper_cmd.py 0.04 && timeout 600 python3 move.py -0.214 0.205 1.03 0 --sec 4 && timeout 20 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 19
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 20
timeout 60 python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_world.npy"); Z=P[...,2]; C=np.load("robot0_eye_in_hand_color.npy")
k = (P[...,0] > -0.32) & (P[...,0] < -0.15) & (P[...,1] > 0.12) & (P[...,1] < 0.3) & (Z>0.905) & (Z<1.0)
for lo,hi in [(0.905,0.92),(0.92,0.935),(0.935,0.95),(0.95,0.965),(0.965,1.0)]:
    m=k&(Z>=lo)&(Z<hi)
    if m.sum(): pts=P[m]; print(f"   z[{lo},{hi}] n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] c=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) col={C[m].mean(0).astype(int)}")
# bar: z>0.945 -> per-x-slice y center
m = k & (Z>0.948)
pts = P[m]
for x0 in np.arange(-0.26,-0.16,0.01):
    s = pts[(pts[:,0]>=x0)&(pts[:,0]<x0+0.01)]
    if len(s): print(f"x {x0:.2f}: y[{s[:,1].min():.3f},{s[:,1].max():.3f}] mid={(s[:,1].min()+s[:,1].max())/2:.4f} ztop={s[:,2].max():.3f}")
print("bar x extent", pts[:,0].min(), pts[:,0].max(), "mid", (pts[:,0].min()+pts[:,0].max())/2)
# disc: z in [0.925,0.94] dark
d = k & (Z>0.92)&(Z<0.945)&(C.mean(-1)<80)
pts=P[d]; print("disc x[%.3f,%.3f] y[%.3f,%.3f] center (%.4f,%.4f)"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),(pts[:,0].min()+pts[:,0].max())/2,(pts[:,1].min()+pts[:,1].max())/2))
EOF

# openrua op 21
timeout 600 python3 move.py -0.216 0.206 0.935 0 --sec 3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 22
timeout 300 python3 tools/action/gripper_cmd.py 0.0 ; timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2

# openrua op 23
timeout 600 python3 tools/action/fjt_send.py 0.2391,0.4698,0.1564,-2.3907,-0.2427,2.8456,-0.175 3 && timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position" && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A7 wrench

# openrua op 24
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 25
timeout 300 python3 tools/action/gripper_cmd.py 0.04 && timeout 600 python3 move.py -0.216 0.206 1.05 0 --sec 3 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 26
timeout 60 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy"); Z=P[...,2]; C=np.load("birdview_color.npy")
reg = (P[...,0] > -0.2) & (P[...,0] < 0.15) & (P[...,1] > -0.15) & (P[...,1] < 0.32)
# red burner pixels
red = reg & (C[...,2] > 150) & (C[...,1] < 100)
pts=P[red]; print("red burner n=",red.sum()," x[%.3f,%.3f] y[%.3f,%.3f] center (%.4f,%.4f) z=%.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),(pts[:,0].min()+pts[:,0].max())/2,(pts[:,1].min()+pts[:,1].max())/2, np.median(pts[:,2])))
gray = reg & (Z>0.905) & (Z<0.95) & (C.mean(-1) > 90) & (np.abs(C[...,2].astype(int)-C[...,1].astype(int))<20)
pts=P[gray]; print("plate x[%.3f,%.3f] y[%.3f,%.3f]"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max()))
# pan (dark, z>0.95)
pan = reg & (Z>0.95) & (C.mean(-1) < 90)
pts=P[pan]; print("pan+handle x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].max()))
for y0 in np.arange(-0.12,0.26,0.02):
    m = pan & (P[...,1]>=y0) & (P[...,1]<y0+0.02)
    if m.sum(): print(f" y {y0:.2f}: x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] z[{P[m][:,2].min():.3f},{P[m][:,2].max():.3f}] n={m.sum()}")
EOF

# openrua op 27
python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy"); Z=P[...,2]; C=np.load("birdview_color.npy").astype(int)
reg = (P[...,0] > -0.2) & (P[...,0] < 0.15) & (P[...,1] > -0.15) & (P[...,1] < 0.32)
red = reg & (C[...,2] > C[...,1] + 40)
pts=P[red]; print("red burner n=",red.sum()," x[%.3f,%.3f] y[%.3f,%.3f] center (%.4f,%.4f) z=%.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),(pts[:,0].min()+pts[:,0].max())/2,(pts[:,1].min()+pts[:,1].max())/2, np.median(pts[:,2])))
gray = reg & (Z>0.905) & (Z<0.95) & (C.mean(-1) > 90) & (np.abs(C[...,2]-C[...,1])<20)
pts=P[gray]; print("plate x[%.3f,%.3f] y[%.3f,%.3f]"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max()))
pan = reg & (Z>0.95) & (C.mean(-1) < 90)
pts=P[pan]; print("pan+handle x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].max()))
for y0 in np.arange(-0.14,0.26,0.02):
    m = pan & (P[...,1]>=y0) & (P[...,1]<y0+0.02)
    if m.sum(): print(f" y {y0:.2f}: x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] z[{P[m][:,2].min():.3f},{P[m][:,2].max():.3f}] n={m.sum()}")
mk = (P[...,0] > -0.15) & (P[...,0] < 0.1) & (P[...,1] > -0.4) & (P[...,1] < -0.15) & (Z>0.91)
pts=P[mk]; print("moka x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].max()))
for lo,hi in [(0.91,0.95),(0.95,1.0),(1.0,1.04),(1.04,1.07)]:
    m = mk&(Z>lo)&(Z<hi)
    if m.sum(): pts=P[m]; print(f"  moka z[{lo},{hi}]: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] c=({(pts[:,0].min()+pts[:,0].max())/2:.3f},{(pts[:,1].min()+pts[:,1].max())/2:.3f})")
EOF

# openrua op 28
python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy"); Z=P[...,2]; C=np.load("birdview_color.npy").astype(int)
reg = (P[...,0] > -0.25) & (P[...,0] < 0.2) & (P[...,1] > -0.2) & (P[...,1] < 0.32) & (Z<1.1)
dark = reg & (Z>0.905) & (C.mean(-1) < 90)
for y0 in np.arange(-0.16,0.26,0.02):
    m = dark & (P[...,1]>=y0) & (P[...,1]<y0+0.02)
    if m.sum(): print(f" y {y0:.2f}: x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] z[{P[m][:,2].min():.3f},{P[m][:,2].max():.3f}] n={m.sum()}")
pan = dark & (P[...,1] < 0.10)
pts=P[pan]; print("pan body x[%.3f,%.3f] y[%.3f,%.3f] center (%.3f,%.3f) ztop %.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),(pts[:,0].min()+pts[:,0].max())/2,(pts[:,1].min()+pts[:,1].max())/2,pts[:,2].max()))
EOF

# openrua op 29
timeout 600 python3 move.py -0.032 -0.237 1.16 90 --sec 4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 30
timeout 600 python3 tools/action/fjt_send.py -0.4673,0.4485,0.1245,-1.6583,-0.0625,2.1029,-1.1086 4 && timeout 60 python3 fk.py

# openrua op 31
timeout 600 python3 tools/action/fjt_send.py -0.4673,0.4485,0.1245,-1.6583,-0.0625,2.1029,-1.1086 6 && timeout 60 python3 fk.py

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 33
timeout 60 python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_world.npy"); Z=P[...,2]; C=np.load("robot0_eye_in_hand_color.npy").astype(int)
mk = (P[...,0] > -0.15) & (P[...,0] < 0.1) & (P[...,1] > -0.4) & (P[...,1] < -0.12) & (Z>0.905) & (Z<1.1)
pts=P[mk]; print("moka x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].max()))
for lo in np.arange(0.905,1.06,0.01):
    m = mk&(Z>=lo)&(Z<lo+0.01)
    if m.sum(): pts=P[m]; print(f"  z[{lo:.3f},{lo+0.01:.3f}]: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] c=({(pts[:,0].min()+pts[:,0].max())/2:.3f},{(pts[:,1].min()+pts[:,1].max())/2:.3f}) bright={C[m].mean():.0f}")
EOF

# openrua op 34
timeout 60 python3 cloud.py sideview && timeout 60 python3 cloud.py frontview && python3 - <<'EOF'
import numpy as np
for cam in ["sideview","frontview","agentview"]:
    P = np.load(f"{cam}_world.npy"); Z=P[...,2]
    mk = (P[...,0] > -0.15) & (P[...,0] < 0.1) & (P[...,1] > -0.4) & (P[...,1] < -0.12) & (Z>0.903) & (Z<1.1)
    print(cam)
    for lo in np.arange(0.905,1.06,0.01):
        m = mk&(Z>=lo)&(Z<lo+0.01)
        if m.sum(): pts=P[m]; print(f"  z[{lo:.3f},{lo+0.01:.3f}]: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] w={pts[:,0].max()-pts[:,0].min():.3f} y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}]")
EOF

# openrua op 35
python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_world.npy"); Z=P[...,2]; C=np.load("robot0_eye_in_hand_color.npy").astype(int)
mk = (P[...,0] > -0.15) & (P[...,0] < 0.1) & (P[...,1] > -0.4) & (P[...,1] < -0.12) & (Z>1.02) & (Z<1.04) & (C.mean(-1)>150)
pts = P[mk][:, :2]
c = np.array([-0.029, -0.235])
# exclude spout/handle: keep points within 0.045 of center
d = pts - c
for ang in range(0, 180, 5):
    a = np.deg2rad(ang); u = np.array([np.cos(a), np.sin(a)])
    proj = d @ u
    # robust width: exclude handle/spout by using only points with |perp| small? use 1st/99th pct
    print(ang, f"width={proj.max()-proj.min():.4f}  p1-p99={np.percentile(proj,99)-np.percentile(proj,1):.4f} center_off={(proj.max()+proj.min())/2:.4f}")
EOF

# openrua op 36
timeout 600 python3 move.py -0.029 -0.235 1.07 67.5 --sec 5 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 37
timeout 60 python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_world.npy"); Z=P[...,2]; C=np.load("robot0_eye_in_hand_color.npy").astype(int)
mk = (P[...,0] > -0.15) & (P[...,0] < 0.1) & (P[...,1] > -0.4) & (P[...,1] < -0.12) & (Z>1.02) & (Z<1.04) & (C.mean(-1)>150)
pts = P[mk][:, :2]
tcp = np.array([-0.0289, -0.2350])
u = np.array([np.cos(np.deg2rad(-22.5)), np.sin(np.deg2rad(-22.5))])
v = np.array([-u[1], u[0]])
pu = (pts-tcp)@u; pv = (pts-tcp)@v
print(f"along finger axis: min={pu.min():.4f} max={pu.max():.4f} center={(pu.min()+pu.max())/2:.4f} width={pu.max()-pu.min():.4f}")
print(f"perp: min={pv.min():.4f} max={pv.max():.4f}")
# points near the flats: |pv|<0.015
m = np.abs(pv)<0.015
print(f"flat region along axis: min={pu[m].min():.4f} max={pu[m].max():.4f} center={(pu[m].min()+pu[m].max())/2:.4f} width={pu[m].max()-pu[m].min():.4f}")
EOF

# openrua op 38
timeout 600 python3 move.py -0.029 -0.235 0.955 67.5 --sec 4 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 39
python3 - <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n = rclpy.create_node("rd")
qos = QoSProfile(depth=1); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
got = {}
n.create_subscription(String, "/robot_description", lambda m: got.setdefault("m", m), qos)
import time; t=time.time()
while "m" not in got and time.time()-t<10: rclpy.spin_once(n, timeout_sec=0.2)
open("robot.urdf","w").write(got["m"].data); print(len(got["m"].data))
EOF
grep -n -A25 'link name="panda_leftfinger"' robot.urdf | head -60; grep -n -B2 -A8 'finger_joint1' robot.urdf | head -40

# openrua op 40
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 41
timeout 60 python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_world.npy"); C=np.load("robot0_eye_in_hand_color.npy").astype(int)
import struct
# finger points: high z (close to camera) i.e., z > 1.05 world (hand at 1.118, fingers span 1.015..1.06)
tcp = np.array([-0.0207+0.0, -0.2386])  # hand xy (approx TCP xy)
u = np.array([np.cos(np.deg2rad(-22.5)), np.sin(np.deg2rad(-22.5))])
Z = P[...,2]
m = (Z > 1.02) & (Z < 1.12) & np.isfinite(Z)
pts = P[m]
pu = (pts[:,:2]-tcp)@u
# fingers are dark; pot is bright
dark = C[m].mean(-1) < 100
print("dark (finger) points along axis: neg side max", pu[dark & (pu<0)].max(), " pos side min", pu[dark & (pu>0)].min())
# per z-slice
for lo in np.arange(1.02,1.12,0.01):
    s = (pts[:,2]>=lo)&(pts[:,2]<lo+0.01)&dark
    if s.sum(): print(f" z[{lo:.2f}] finger neg max {pu[s & (pu<0)].max() if (s&(pu<0)).any() else None}  pos min {pu[s & (pu>0)].min() if (s&(pu>0)).any() else None}")
bright = ~dark
print("pot (bright) points along axis: min", pu[bright].min(), "max", pu[bright].max(), "z range", pts[bright][:,2].min(), pts[bright][:,2].max())
EOF

# openrua op 42
python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_world.npy"); C=np.load("robot0_eye_in_hand_color.npy").astype(int)
tcp = np.array([-0.0207, -0.2386])
u = np.array([np.cos(np.deg2rad(-22.5)), np.sin(np.deg2rad(-22.5))])
Z = P[...,2]
for name, (c0,c1) in {"left":(60,230),"right":(420,600)}.items():
    sub = P[340:400, c0:c1]; subc = C[340:400, c0:c1]
    ok = np.isfinite(sub[...,2]) & (sub[...,2]>1.0)
    pts = sub[ok]; pu = (pts[:,:2]-tcp)@u
    print(name, "n", ok.sum(), "along-axis range", pu.min(), pu.max(), "z range", pts[:,2].min(), pts[:,2].max())
    # inner-most edge: for left, max pu; for right, min pu
    # histogram
    h, e = np.histogram(pu, bins=20); print(list(zip(np.round(e[:-1],4), h)))
EOF

# openrua op 43
timeout 600 python3 move.py -0.029 -0.235 1.10 67.5 --sec 4 && timeout 60 python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_world.npy"); C=np.load("robot0_eye_in_hand_color.npy").astype(int)
Z = P[...,2]
hand_z = 1.10+0.1034
u = np.array([np.cos(np.deg2rad(-22.5)), np.sin(np.deg2rad(-22.5))])
tcp = np.array([-0.029,-0.235])
m = np.isfinite(Z) & (Z > 1.08)
pts = P[m]; pu = (pts[:,:2]-tcp)@u
print("finger points n", m.sum(), "z range", pts[:,2].min(), pts[:,2].max())
for lo in np.arange(1.08, 1.21, 0.01):
    s = (pts[:,2]>=lo)&(pts[:,2]<lo+0.01)
    if s.sum():
        neg = pu[s & (pu<0)]; pos = pu[s & (pu>0)]
        print(f" z[{lo:.2f}] n={s.sum()} neg[{neg.min() if len(neg) else None},{neg.max() if len(neg) else None}] pos[{pos.min() if len(pos) else None},{pos.max() if len(pos) else None}]")
EOF

# openrua op 44
python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_world.npy"); C=np.load("robot0_eye_in_hand_color.npy").astype(int)
Z = P[...,2]
m = np.isfinite(Z) & (Z > 0.93) & (Z < 1.06) & (P[...,1] < -0.27) & (P[...,1] > -0.35) & (np.abs(P[...,0]+0.03) < 0.05)
pts = P[m]
print("handle region n", m.sum())
for y0 in np.arange(-0.33, -0.27, 0.005):
    s = (pts[:,1]>=y0)&(pts[:,1]<y0+0.005)
    if s.sum(): print(f" y[{y0:.3f}] n={s.sum()} x[{pts[s][:,0].min():.4f},{pts[s][:,0].max():.4f}] z[{pts[s][:,2].min():.4f},{pts[s][:,2].max():.4f}] bright={C[m][s].mean():.0f}")
EOF

# openrua op 45
python3 - <<'EOF'
import numpy as np
for cam in ["frontview","agentview"]:
    P = np.load(f"{cam}_world.npy"); C=np.load(f"{cam}_color.npy").astype(int)
    Z = P[...,2]
    m = np.isfinite(Z) & (Z > 0.92) & (Z < 1.06) & (P[...,1] < -0.27) & (P[...,1] > -0.36) & (np.abs(P[...,0]+0.03) < 0.06)
    pts = P[m]; print(cam, "handle region n", m.sum())
    for z0 in np.arange(0.93, 1.05, 0.01):
        s = (pts[:,2]>=z0)&(pts[:,2]<z0+0.01)
        if s.sum(): print(f" z[{z0:.2f}] n={s.sum()} y[{pts[s][:,1].min():.4f},{pts[s][:,1].max():.4f}] x[{pts[s][:,0].min():.4f},{pts[s][:,0].max():.4f}]")
EOF

# openrua op 46
timeout 600 python3 move.py -0.028 -0.295 1.08 90 --sec 5 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 47
timeout 60 python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_world.npy"); C=np.load("robot0_eye_in_hand_color.npy").astype(int)
Z = P[...,2]
m = np.isfinite(Z) & (Z > 0.93) & (Z < 1.06) & (P[...,1] < -0.272) & (P[...,1] > -0.36) & (np.abs(P[...,0]+0.03) < 0.05) & (C.mean(-1) < 120)
pts = P[m]
print("handle n", m.sum())
for y0 in np.arange(-0.33, -0.27, 0.005):
    s = (pts[:,1]>=y0)&(pts[:,1]<y0+0.005)
    if s.sum(): print(f" y[{y0:.3f}] n={s.sum()} x[{pts[s][:,0].min():.4f},{pts[s][:,0].max():.4f}] xc={(pts[s][:,0].min()+pts[s][:,0].max())/2:.4f} z[{pts[s][:,2].min():.4f},{pts[s][:,2].max():.4f}]")
EOF

# openrua op 48
timeout 600 python3 move.py -0.028 -0.297 1.0 90 --sec 4 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 49
timeout 300 python3 tools/action/gripper_cmd.py 0.0; timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 50
timeout 600 python3 move.py -0.028 -0.297 1.06 90 --sec 4 && timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 51
timeout 600 python3 move.py -0.028 -0.297 1.15 90 --sec 4 && timeout 60 python3 cloud.py birdview && timeout 60 python3 cloud.py sideview && python3 - <<'EOF'
import numpy as np
for cam in ["birdview","sideview"]:
    P = np.load(f"{cam}_world.npy"); C=np.load(f"{cam}_color.npy").astype(int); Z=P[...,2]
    m = np.isfinite(Z) & (Z>0.95) & (Z<1.3) & (P[...,0]>-0.12) & (P[...,0]<0.06) & (P[...,1]>-0.4) & (P[...,1]<-0.15) & (C.mean(-1)>120)
    pts = P[m]; print(cam, "bright pot pts n", m.sum())
    for z0 in np.arange(0.98, 1.30, 0.02):
        s=(pts[:,2]>=z0)&(pts[:,2]<z0+0.02)
        if s.sum()>5: print(f" z[{z0:.2f}] n={s.sum()} x[{pts[s][:,0].min():.3f},{pts[s][:,0].max():.3f}] xc={(pts[s][:,0].min()+pts[s][:,0].max())/2:.4f} y[{pts[s][:,1].min():.3f},{pts[s][:,1].max():.3f}] yc={(pts[s][:,1].min()+pts[s][:,1].max())/2:.4f}")
EOF

# openrua op 52
timeout 600 python3 move.py -0.07 0.154 1.15 90 --sec 6 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 53
timeout 60 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy"); C=np.load("birdview_color.npy").astype(int); Z=P[...,2]
m = np.isfinite(Z) & (Z>1.0) & (Z<1.3) & (P[...,0]>-0.2) & (P[...,0]<0.1) & (P[...,1]>0.05) & (P[...,1]<0.35) & (C.mean(-1)>120)
pts = P[m]; print("bright pot pts n", m.sum())
for z0 in np.arange(1.0, 1.30, 0.02):
    s=(pts[:,2]>=z0)&(pts[:,2]<z0+0.02)
    if s.sum()>5: print(f" z[{z0:.2f}] n={s.sum()} x[{pts[s][:,0].min():.3f},{pts[s][:,0].max():.3f}] xc={(pts[s][:,0].min()+pts[s][:,0].max())/2:.4f} y[{pts[s][:,1].min():.3f},{pts[s][:,1].max():.3f}] yc={(pts[s][:,1].min()+pts[s][:,1].max())/2:.4f}")
# lowest points of anything in the pot column (base)
m2 = np.isfinite(Z) & (Z>0.95) & (Z<1.3) & (np.abs(P[...,0]+0.07)<0.05) & (np.abs(P[...,1]-0.204)<0.05)
print("min z in pot column:", P[m2][:,2].min())
red = (C[...,2] > C[...,1] + 40) & (P[...,1]>0.05)
pts=P[red]; print("visible red burner x[%.3f,%.3f] y[%.3f,%.3f]"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max()))
EOF

# openrua op 54
timeout 600 python3 move.py -0.07 0.154 1.04 90 --sec 4 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 55
timeout 600 python3 move.py -0.07 0.154 1.03 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 56
timeout 300 python3 tools/action/gripper_cmd.py 0.04 && timeout 600 python3 move.py -0.07 0.154 1.15 90 --sec 4 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 57
timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 58
timeout 600 python3 move.py -0.07 0.154 1.05 90 --sec 4 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 59
timeout 600 python3 move.py -0.07 0.154 1.0 90 --sec 4 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 60
timeout 300 python3 tools/action/gripper_cmd.py 0.0; timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 61
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 62
timeout 600 python3 move.py -0.07 0.094 1.05 90 --sec 4 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 63
timeout 600 python3 move.py -0.07 0.094 1.02 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 64
timeout 120 python3 tools/perception/cam_snap.py sideview && timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); C=np.load('birdview_color.npy')
m=np.isfinite(P[...,2])&(P[...,2]>0.93)&(P[...,2]<1.12)&(P[...,0]>-0.25)&(P[...,0]<0.05)&(P[...,1]>0.05)&(P[...,1]<0.32)
pts=P[m]; col=C[m]
print('n',len(pts))
# cluster by z bands
for lo,hi in [(0.93,0.96),(0.96,1.0),(1.0,1.04),(1.04,1.08),(1.08,1.12)]:
    s=(pts[:,2]>=lo)&(pts[:,2]<hi)
    if s.sum(): print(lo,hi,s.sum(),'x',pts[s,0].min().round(3),pts[s,0].max().round(3),'y',pts[s,1].min().round(3),pts[s,1].max().round(3))
"

# openrua op 65
timeout 600 python3 move.py -0.07 0.094 1.07 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 20 ros2 topic echo /joint_states --once | grep -A12 name | head -30

# openrua op 66
timeout 600 python3 move.py -0.07 0.0 1.12 90 --sec 4 && timeout 600 python3 move.py -0.10 -0.05 1.25 90 --sec 4 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np
P=np.load('birdview_world.npy')
m=np.isfinite(P[...,2])&(P[...,2]>0.96)&(P[...,2]<1.12)&(P[...,0]>-0.25)&(P[...,0]<0.05)&(P[...,1]>0.05)&(P[...,1]<0.32)
pts=P[m]
for lo,hi in [(0.96,1.0),(1.0,1.04),(1.04,1.08),(1.08,1.12)]:
    s=(pts[:,2]>=lo)&(pts[:,2]<hi)
    if s.sum(): print(lo,hi,s.sum(),'x',pts[s,0].min().round(3),pts[s,0].max().round(3),'y',pts[s,1].min().round(3),pts[s,1].max().round(3))
"

# openrua op 67
timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 tools/perception/cam_snap.py birdview && python3 -c "
import numpy as np
P=np.load('birdview_world.npy')
m=np.isfinite(P[...,2])&(P[...,2]>0.95)&(P[...,2]<1.2)
pts=P[m]
print(len(pts))
import collections
for lo,hi in [(0.95,1.0),(1.0,1.04),(1.04,1.08),(1.08,1.2)]:
    s=(pts[:,2]>=lo)&(pts[:,2]<hi)
    if s.sum(): print(lo,hi,s.sum(),'x',pts[s,0].min().round(3),pts[s,0].max().round(3),'y',pts[s,1].min().round(3),pts[s,1].max().round(3))
"

# openrua op 68
timeout 120 python3 tools/perception/cam_snap.py sideview && timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 69
timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np
P=np.load('birdview_world.npy')
m=np.isfinite(P[...,2])&(P[...,2]>1.0)&(P[...,2]<1.6)&(P[...,0]>-0.4)
pts=P[m]
print(len(pts))
for lo in np.arange(1.0,1.6,0.03):
    s=(pts[:,2]>=lo)&(pts[:,2]<lo+0.03)
    if s.sum(): print(round(lo,2),s.sum(),'x',pts[s,0].min().round(3),pts[s,0].max().round(3),'y',pts[s,1].min().round(3),pts[s,1].max().round(3))
"

# openrua op 70
timeout 600 python3 move.py -0.02 0.144 1.20 90 --sec 4 && timeout 600 python3 move.py -0.02 0.144 1.045 90 --sec 4 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 71
timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 tools/perception/cam_snap.py sideview

# openrua op 72
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py sideview && python3 -c "
import numpy as np
for cam in ['birdview','sideview']:
    P=np.load(cam+'_world.npy')
    m=np.isfinite(P[...,2])&(P[...,2]>0.93)&(P[...,2]<1.09)&(P[...,0]>-0.2)&(P[...,0]<0.05)&(P[...,1]>0.08)&(P[...,1]<0.30)
    pts=P[m]
    print(cam,len(pts))
    for lo in np.arange(0.93,1.09,0.02):
        s=(pts[:,2]>=lo)&(pts[:,2]<lo+0.02)
        if s.sum(): print(' ',round(lo,2),s.sum(),'x',pts[s,0].min().round(3),pts[s,0].max().round(3),'mean',pts[s,0].mean().round(3),'y',pts[s,1].min().round(3),pts[s,1].max().round(3),'mean',pts[s,1].mean().round(3))
"

# openrua op 73
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 74
timeout 600 python3 move.py -0.02 0.144 1.072 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 75
timeout 600 python3 move.py 0.015 0.109 1.072 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 76
timeout 120 python3 tools/action/gripper_cmd.py 0.0; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position | tail -2 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 77
timeout 600 python3 move.py 0.015 0.109 1.13 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 20 ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 78
timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 79
timeout 600 python3 move.py -0.06 0.154 1.13 90 --sec 4 && timeout 600 python3 move.py -0.06 0.154 1.075 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 tools/perception/cam_snap.py sideview

# openrua op 80
timeout 600 python3 move.py -0.06 0.154 1.062 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 81
timeout 600 python3 move.py -0.06 0.154 1.056 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 82
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position | tail -2 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 83
timeout 600 python3 move.py -0.06 0.07 1.062 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 84
timeout 600 python3 move.py -0.06 0.0 1.20 90 --sec 4 && timeout 600 python3 move.py -0.15 -0.10 1.30 90 --sec 4 && timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 tools/perception/cam_snap.py sideview && timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 85
timeout 120 python3 cloud.py birdview && timeout 120 python3 tools/perception/cam_snap.py birdview && python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); C=np.load('birdview_color.npy')
m=np.isfinite(P[...,2])&(P[...,2]>0.935)&(P[...,2]<1.2)&(P[...,0]>-0.25)&(P[...,0]<0.08)&(P[...,1]>0.05)&(P[...,1]<0.35)
pts=P[m]; col=C[m]
print(len(pts))
for lo in np.arange(0.935,1.2,0.015):
    s=(pts[:,2]>=lo)&(pts[:,2]<lo+0.015)
    if s.sum(): print(round(lo,3),s.sum(),'x',pts[s,0].min().round(3),pts[s,0].max().round(3),'y',pts[s,1].min().round(3),pts[s,1].max().round(3),'bright',int((col[s].mean(axis=1)>150).sum()))
"

# openrua op 86
python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); C=np.load('birdview_color.npy').astype(int)
red=(C[...,2]>150)&(C[...,1]<90)&(C[...,0]<90)&np.isfinite(P[...,2])
pts=P[red]
print('red n',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
print('center',pts[:,0].mean().round(3),pts[:,1].mean().round(3))
# plate (grey) extents: points z 0.92-0.935 in region
m=np.isfinite(P[...,2])&(P[...,2]>0.915)&(P[...,2]<0.935)&(P[...,0]>-0.3)&(P[...,0]<0.1)&(P[...,1]>0.05)&(P[...,1]<0.35)
g=P[m]; print('plate-ish n',len(g),'x',g[:,0].min().round(3),g[:,0].max().round(3),'y',g[:,1].min().round(3),g[:,1].max().round(3))
"

# openrua op 87
python3 - <<'EOF'
s=open('move.py').read()
s=s.replace('''    opts = {"sec": 4.0, "hand": False, "dry": False, "j7": None}''','''    opts = {"sec": 4.0, "hand": False, "dry": False, "j7": None, "tilt": 0.0}''')
s=s.replace('''        elif a[i] == "--j7": opts["j7"] = float(a[i+1]); i += 2''','''        elif a[i] == "--j7": opts["j7"] = float(a[i+1]); i += 2
        elif a[i] == "--tilt": opts["tilt"] = float(a[i+1]); i += 2''')
s=s.replace('''    if not o["hand"]: z += TCP
    R_hand = Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)''','''    R_hand = Rot.from_euler("x", o["tilt"], degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    tcp_goal = np.array([x, y, z])
    if not o["hand"]:
        x, y, z = tcp_goal - R_hand.apply([0, 0, TCP])
    else:
        tcp_goal = tcp_goal + R_hand.apply([0, 0, TCP])''')
s=s.replace('''    perr = np.linalg.norm(tcp - np.array([x, y, z + (TCP if o["hand"] else 0) - TCP]))''','''    perr = np.linalg.norm(tcp - tcp_goal)''')
open('move.py','w').write(s)
EOF
timeout 120 python3 move.py -0.15 -0.10 1.30 90 --dry && timeout 120 python3 move.py -0.15 -0.10 1.30 90 --tilt 20 --dry

# openrua op 88
timeout 600 python3 move.py -0.048 0.253 1.15 90 --sec 4 && timeout 600 python3 move.py -0.048 0.253 1.05 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 89
timeout 120 python3 tools/action/gripper_cmd.py 0.0; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 90
timeout 600 python3 move.py -0.048 0.253 1.28 90 --sec 4 && timeout 20 ros2 topic echo /joint_states --once | grep -A9 position | tail -2 && timeout 120 python3 tools/perception/cam_snap.py sideview && timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 91
timeout 120 python3 move.py -0.048 0.253 1.28 -90 --dry; timeout 120 python3 move.py -0.048 0.253 1.28 -90 --tilt -90 --dry; timeout 120 python3 move.py -0.048 0.253 1.28 90 --tilt 90 --dry

# openrua op 92
timeout 600 python3 move.py -0.048 0.253 1.28 90 --tilt 90 --sec 6 && timeout 20 ros2 topic echo /joint_states --once | grep -A9 position | tail -2 && timeout 120 python3 tools/perception/cam_snap.py sideview && timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 93
timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); C=np.load('birdview_color.npy')
m=np.isfinite(P[...,2])&(P[...,2]>1.1)&(P[...,2]<1.5)&(P[...,0]>-0.3)&(P[...,0]<0.1)&(P[...,1]>0.1)&(P[...,1]<0.5)
pts=P[m]; col=C[m]
for lo in np.arange(1.1,1.5,0.02):
    s=(pts[:,2]>=lo)&(pts[:,2]<lo+0.02)
    if s.sum(): print(round(lo,2),s.sum(),'x',pts[s,0].min().round(3),pts[s,0].max().round(3),'mean',pts[s,0].mean().round(3),'y',pts[s,1].min().round(3),pts[s,1].max().round(3),'mean',pts[s,1].mean().round(3))
"

# openrua op 94
timeout 120 python3 cloud.py frontview && timeout 120 python3 cloud.py sideview && python3 -c "
import numpy as np
for cam in ['frontview','sideview']:
    P=np.load(cam+'_world.npy')
    m=np.isfinite(P[...,2])&(P[...,2]>1.12)&(P[...,2]<1.36)&(P[...,0]>-0.2)&(P[...,0]<0.1)&(P[...,1]>0.27)&(P[...,1]<0.5)
    pts=P[m]
    print(cam,len(pts))
    for lo in np.arange(1.12,1.36,0.02):
        s=(pts[:,2]>=lo)&(pts[:,2]<lo+0.02)
        if s.sum(): print(' ',round(lo,2),s.sum(),'x',pts[s,0].min().round(3),pts[s,0].max().round(3),'y',pts[s,1].min().round(3),pts[s,1].max().round(3))
"

# openrua op 95
timeout 600 python3 move.py -0.061 0.117 1.20 90 --tilt 90 --sec 4 && timeout 600 python3 move.py -0.061 0.117 1.06 90 --tilt 90 --sec 4 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force && timeout 120 python3 tools/perception/cam_snap.py sideview

# openrua op 96
timeout 600 python3 move.py -0.061 0.117 1.045 90 --tilt 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 97
timeout 600 python3 move.py -0.061 0.117 1.035 90 --tilt 90 --sec 3 && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 98
for z in 1.025 1.015 1.005; do timeout 600 python3 move.py -0.061 0.117 $z 90 --tilt 90 --sec 3 | grep -E "error_code|hand\(base\)"; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force | tr '\n' ' '; echo; done

# openrua op 99
timeout 600 python3 move.py -0.061 0.117 1.028 90 --tilt 90 --sec 3 | grep -E "error_code|hand\(base\)"; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force | tr '\n' ' '; echo; timeout 120 python3 tools/perception/cam_snap.py sideview; timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 100
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position | tail -2; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force | tr '\n' ' '; echo; timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 101
timeout 600 python3 move.py -0.061 0.03 1.028 90 --tilt 90 --sec 4 | grep -E "error_code|hand\(base\)"; timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 102
timeout 600 python3 move.py -0.10 -0.05 1.20 90 --tilt 45 --sec 5 | grep -E "error_code|hand\(base\)"; timeout 600 python3 move.py -0.20 -0.10 1.30 90 --sec 5 | grep -E "error_code|hand\(base\)"; timeout 120 python3 tools/perception/cam_snap.py agentview; timeout 120 python3 tools/perception/cam_snap.py sideview; timeout 120 python3 tools/perception/cam_snap.py birdview

# openrua op 103
timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); C=np.load('birdview_color.npy').astype(int)
m=np.isfinite(P[...,2])&(P[...,2]>1.0)&(P[...,2]<1.15)&(P[...,0]>-0.25)&(P[...,0]<0.08)&(P[...,1]>0.05)&(P[...,1]<0.35)
pts=P[m]; print('pot top-ish points',len(pts),'z max',pts[:,2].max().round(3),'center x,y',pts[:,0].mean().round(3),pts[:,1].mean().round(3))
red=(C[...,2]>150)&(C[...,1]<90)&(C[...,0]<90)&np.isfinite(P[...,2])&(P[...,2]<1.0)
r=P[red]; print('red coil points',len(r),'center',r[:,0].mean().round(3),r[:,1].mean().round(3),'z',r[:,2].mean().round(3))
"
