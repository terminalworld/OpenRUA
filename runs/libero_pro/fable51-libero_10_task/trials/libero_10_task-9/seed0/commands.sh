#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 30 ros2 action list; echo ----; timeout 30 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ----; timeout 30 ros2 node list

# openrua op 4
mkdir -p snaps && for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -60; echo ----; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child" | head

# openrua op 6
timeout 20 ros2 topic echo /tf --once 2>&1 | head -120

# openrua op 7
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab depth+color+info+TF for a camera, save a world-frame point cloud.

Usage: python3 cloud.py <camera>  -> snaps/<camera>_cloud.npz
  (xyz: HxWx3 world coords, rgb: HxWx3 uint8, depth: HxW)
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=30.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "rgb8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    v, u = np.mgrid[0:H, 0:W]
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1)
    xyz = pc @ R.T + p0
    np.savez(f"snaps/{cam}_cloud.npz", xyz=xyz, rgb=color, depth=depth, K=np.array(info.k).reshape(3, 3), R=R, p0=p0)
    print(f"saved snaps/{cam}_cloud.npz {H}x{W} depth range {np.nanmin(depth):.3f}-{np.nanmax(depth):.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview && timeout 120 python3 cloud.py sideview

# openrua op 9
python3 -c "
import numpy as np
d=np.load('snaps/birdview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']
z=xyz[...,2]
# table height: histogram of z
h,e=np.histogram(z[np.isfinite(z)],bins=100)
for c,lo in zip(h,e[:-1]):
    if c>500: print(f'{lo:.3f} {c}')
"


# openrua op 10
python3 -c "
import numpy as np
d=np.load('snaps/birdview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']
# white mug pixel region approx (235,270), yellow mug (325,300), microwave (390-460, 245-350)
def stats(name, u0,u1,v0,v1):
    P=xyz[v0:v1,u0:u1].reshape(-1,3); C=rgb[v0:v1,u0:u1].reshape(-1,3)
    m=P[:,2]>0.92
    P=P[m]; C=C[m]
    print(name, 'n',len(P), 'x',P[:,0].min().round(3),P[:,0].max().round(3),'y',P[:,1].min().round(3),P[:,1].max().round(3),'z',P[:,2].min().round(3),P[:,2].max().round(3), 'mean',P.mean(0).round(3), 'rgb',C.mean(0).round(0))
stats('white mug',215,265,250,295)
stats('yellow mug',300,345,280,320)
stats('microwave',385,465,240,355)
stats('mw door',380,400,230,250)
# table extent
t=xyz[(np.abs(xyz[...,2]-0.895)<0.01)]
print('table x',t[:,0].min().round(3),t[:,0].max().round(3),'y',t[:,1].min().round(3),t[:,1].max().round(3))
"


# openrua op 11
python3 -c "
import numpy as np
d=np.load('snaps/agentview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']
def stats(name, u0,u1,v0,v1, zmin=0.92):
    P=xyz[v0:v1,u0:u1].reshape(-1,3); C=rgb[v0:v1,u0:u1].reshape(-1,3)
    m=(P[:,2]>zmin)&np.isfinite(P[:,2])
    P=P[m]; C=C[m]
    print(name, 'n',len(P), 'x',P[:,0].min().round(3),P[:,0].max().round(3),'y',P[:,1].min().round(3),P[:,1].max().round(3),'z',P[:,2].min().round(3),P[:,2].max().round(3), 'mean',P.mean(0).round(3))
stats('door frame left', 325,345,160,240)
stats('door frame right', 450,470,160,240)
stats('door window', 360,440,180,220)
stats('body face', 500,620,200,380)
stats('body strip', 470,495,250,400)
stats('body top', 480,620,170,200)
"


# openrua op 12
timeout 20 ros2 topic echo /tf --once 2>&1 | sed -n '120,200p'

# openrua op 13
timeout 120 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']
def stats(name, u0,u1,v0,v1, zmin=0.92):
    P=xyz[v0:v1,u0:u1].reshape(-1,3); C=rgb[v0:v1,u0:u1].reshape(-1,3)
    m=(P[:,2]>zmin)&np.isfinite(P[:,2])
    P=P[m]; C=C[m]
    print(name, 'n',len(P), 'x',P[:,0].min().round(3),P[:,0].max().round(3),'y',P[:,1].min().round(3),P[:,1].max().round(3),'z',P[:,2].min().round(3),P[:,2].max().round(3), 'mean',P.mean(0).round(3))
stats('left dark', 0,70,0,230)
stats('left window', 10,60,20,200)
stats('slab', 0,320,245,350)
stats('slab window', 80,270,268,298)
stats('yellow mug', 230,400,0,80)
stats('fingers', 200,260,365,400, 0.5)
"


# openrua op 14
python3 -c "
import numpy as np
for cam in ['agentview','sideview']:
    d=np.load(f'snaps/{cam}_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']
    P=xyz.reshape(-1,3)
    # white mug region: x -0.16..-0.04, y -0.34..-0.19
    m=(P[:,0]>-0.17)&(P[:,0]<-0.03)&(P[:,1]>-0.35)&(P[:,1]<-0.18)&(P[:,2]>0.90)
    Q=P[m]; print(cam,'white mug z',Q[:,2].min().round(3),Q[:,2].max().round(3),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3), 'n',len(Q))
    # table height near mug
    m=(P[:,0]>-0.3)&(P[:,0]<0.0)&(P[:,1]>-0.5)&(P[:,1]<-0.4)
    Q=P[m]; print(cam,'table z near', np.median(Q[:,2]).round(4))
    # microwave cavity floor / opening: points with y in 0.25..0.43, x -0.15..0.1
    m=(P[:,0]>-0.15)&(P[:,0]<0.12)&(P[:,1]>0.24)&(P[:,1]<0.44)&(P[:,2]>0.90)&(P[:,2]<1.05)
    Q=P[m]; print(cam,'cavity pts',len(Q), (Q.mean(0).round(3) if len(Q) else None))
"


# openrua op 15
python3 -c "
import numpy as np
for cam in ['agentview','sideview','birdview']:
    d=np.load(f'snaps/{cam}_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']
    P=xyz.reshape(-1,3); C=rgb.reshape(-1,3)
    m=(P[:,0]>-0.2)&(P[:,0]<0.0)&(P[:,1]>-0.36)&(P[:,1]<-0.17)&(P[:,2]>0.905)
    Q=P[m]; CC=C[m]
    print(cam)
    for lo in np.arange(0.90,1.10,0.02):
        s=(Q[:,2]>=lo)&(Q[:,2]<lo+0.02)
        if s.sum(): print(f'  z {lo:.2f}: n={s.sum():4d} x[{Q[s,0].min():.3f},{Q[s,0].max():.3f}] y[{Q[s,1].min():.3f},{Q[s,1].max():.3f}] rgb {CC[s].mean(0).round(0)}')
"


# openrua op 16
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 17
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview && timeout 120 python3 cloud.py sideview && python3 -c "
import numpy as np
for cam in ['agentview','sideview','birdview']:
    d=np.load(f'snaps/{cam}_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']
    P=xyz.reshape(-1,3); C=rgb.reshape(-1,3)
    m=(P[:,0]>-0.2)&(P[:,0]<0.0)&(P[:,1]>-0.36)&(P[:,1]<-0.17)&(P[:,2]>0.905)
    Q=P[m]; CC=C[m]
    print(cam, 'mug z', Q[:,2].min().round(3), Q[:,2].max().round(3))
    m=(P[:,0]>-0.25)&(P[:,0]<0.2)&(P[:,1]>0.19)&(P[:,1]<0.46)&(P[:,2]>0.905)
    Q=P[m]; print(cam, 'microwave z', Q[:,2].min().round(3), Q[:,2].max().round(3), 'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3))
"


# openrua op 18
python3 -c "
import numpy as np
d=np.load('snaps/agentview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']
P=xyz.reshape(-1,3); C=rgb.reshape(-1,3)
# cavity: y>0.24, x in body
m=(P[:,0]>-0.16)&(P[:,0]<0.18)&(P[:,1]>0.235)&(P[:,1]<0.46)&(P[:,2]>0.905)&(P[:,2]<1.10)
Q=P[m]
print('cavity-ish pts',len(Q))
for lo in np.arange(0.92,1.10,0.02):
    s=(Q[:,2]>=lo)&(Q[:,2]<lo+0.02)
    if s.sum(): print(f'  z {lo:.2f}: n={s.sum():4d} x[{Q[s,0].min():.3f},{Q[s,0].max():.3f}] y[{Q[s,1].min():.3f},{Q[s,1].max():.3f}]')
# floor of cavity: z in 0.94..0.955
s=(Q[:,2]>0.935)&(Q[:,2]<0.96)
print('floor x',Q[s,0].min().round(3),Q[s,0].max().round(3),'y',Q[s,1].min().round(3),Q[s,1].max().round(3), 'zmed',np.median(Q[s,2]).round(3))
# door
m=(P[:,0]>-0.25)&(P[:,0]<-0.16)&(P[:,1]>-0.05)&(P[:,1]<0.28)&(P[:,2]>0.905)
Q=P[m]; print('door x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z',Q[:,2].min().round(3),Q[:,2].max().round(3))
for lo in np.arange(-0.25,-0.15,0.01):
    s=(Q[:,0]>=lo)&(Q[:,0]<lo+0.01)
    if s.sum(): print(f'  x {lo:.2f}: n={s.sum():4d} y[{Q[s,1].min():.3f},{Q[s,1].max():.3f}] z[{Q[s,2].min():.3f},{Q[s,2].max():.3f}]')
# mug
m=(P[:,0]>-0.2)&(P[:,0]<0.0)&(P[:,1]>-0.36)&(P[:,1]<-0.17)&(P[:,2]>0.91)
Q=P[m]
for lo in np.arange(0.90,1.02,0.02):
    s=(Q[:,2]>=lo)&(Q[:,2]<lo+0.02)
    if s.sum(): print(f'  mug z {lo:.2f}: n={s.sum():4d} x[{Q[s,0].min():.3f},{Q[s,0].max():.3f}] y[{Q[s,1].min():.3f},{Q[s,1].max():.3f}]')
"


# openrua op 19
python3 -c "
import numpy as np
for cam in ['agentview','frontview']:
    try: d=np.load(f'snaps/{cam}_cloud.npz')
    except Exception as e: print(e); continue
    xyz=d['xyz']; P=xyz.reshape(-1,3)
    # front plane points
    m=(P[:,0]>-0.16)&(P[:,0]<0.18)&(P[:,1]>0.225)&(P[:,1]<0.25)&(P[:,2]>0.90)&(P[:,2]<1.12)
    Q=P[m]; print(cam,'front plane pts',len(Q))
    for lo in np.arange(0.90,1.12,0.01):
        s=(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)
        if s.sum(): print(f'  z {lo:.2f}: n={s.sum():4d} x[{Q[s,0].min():.3f},{Q[s,0].max():.3f}]')
    # inner back-ish points (y>0.26) : z distribution => ceiling?
    m=(P[:,0]>-0.14)&(P[:,0]<0.10)&(P[:,1]>0.26)&(P[:,1]<0.45)&(P[:,2]>0.90)&(P[:,2]<1.12)
    Q=P[m]; print(cam,'interior pts',len(Q))
    for lo in np.arange(0.90,1.12,0.01):
        s=(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)
        if s.sum(): print(f'  z {lo:.2f}: n={s.sum():4d} x[{Q[s,0].min():.3f},{Q[s,0].max():.3f}] y[{Q[s,1].min():.3f},{Q[s,1].max():.3f}]')
"


# openrua op 20
timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand','panda_link8','panda_leftfinger'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | grep -v "^$" | head -80

# openrua op 21
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
got=[]
n.create_subscription(String,'/robot_description',got.append,qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data if got else '')
print(len(got[0].data) if got else 'none')
" && grep -n -E "<link name|<joint name|origin|<box|<cylinder|<mesh|<sphere|parent|child" robot.urdf | grep -A12 -E "link name=\"panda_hand|link name=\"panda_leftfinger|link name=\"panda_rightfinger|joint name=\"panda_hand_joint|joint name=\"panda_finger|joint name=\"panda_joint8|link name=\"world|joint name=\"virtual|joint name=\"panda_joint1\"" | head -120

# openrua op 22
timeout 60 python3 -c "
import numpy as np
def T(rpy,xyz):
    r,p,y=rpy
    Rx=np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry=np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz=np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    M=np.eye(4); M[:3,:3]=Rz@Ry@Rx; M[:3,3]=xyz; return M
def Rz(t): return T((0,0,t),(0,0,0))
q=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
origins=[((0,0,0),(0,0,0.333)),((-np.pi/2,0,0),(0,0,0)),((np.pi/2,0,0),(0,-0.316,0)),((np.pi/2,0,0),(0.0825,0,0)),((-np.pi/2,0,0),(-0.0825,0.384,0)),((np.pi/2,0,0),(0,0,0)),((np.pi/2,0,0),(0.088,0,0))]
M=np.eye(4)
for (rpy,xyz),qi in zip(origins,q): M=M@T(rpy,xyz)@Rz(qi)
M=M@T((0,0,0),(0,0,0.107))@T((0,0,-0.785398163397),(0,0,0))
print('hand in link0:',M[:3,3].round(4)); print(M[:3,:3].round(3))
"


# openrua op 23
mkdir -p "$(dirname /workspace/ik.py)"
cat > /workspace/ik.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""IK helper: python3 ik.py x y z qx qy qz qw  -> prints joint solution (world-frame pose of panda_hand).
Optional: --seed j1,...,j7 ; --link tcp (pose given for TCP, converted to hand)
"""
import sys
import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def solve(node, cli, pose, seed=None, timeout=60.0):
    """pose = (x,y,z,qx,qy,qz,qw) of panda_hand in world. Returns list of 7 or None."""
    if seed is None:
        js = {}
        sub = node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
        while "m" not in js:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_subscription(sub)
        d = dict(zip(js["m"].name, js["m"].position))
        seed = [d[j] for j in JOINTS]
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = map(float, pose[:3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, pose[3:])
    req.ik_request.robot_state.joint_state.name = JOINTS
    req.ik_request.robot_state.joint_state.position = [float(s) for s in seed]
    req.ik_request.avoid_collisions = False
    req.ik_request.timeout.sec = 2
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    res = fut.result()
    if res is None or res.error_code.val != 1:
        return None
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    return [sol[j] for j in JOINTS]


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    x, y, z, qx, qy, qz, qw = map(float, args[:7])
    if "--link" in sys.argv and "tcp" in sys.argv:
        R = quat_R(qx, qy, qz, qw)
        x, y, z = np.array([x, y, z]) - TCP * R[:, 2]
    seed = None
    if "--seed" in sys.argv:
        seed = [float(v) for v in sys.argv[sys.argv.index("--seed") + 1].split(",")]
    rclpy.init()
    node = rclpy.create_node("ik_helper")
    cli = node.create_client(GetPositionIK, "/compute_ik")
    cli.wait_for_service(10)
    sol = solve(node, cli, (x, y, z, qx, qy, qz, qw), seed)
    if sol is None:
        print("IK FAILED"); sys.exit(1)
    print(",".join(f"{v:.5f}" for v in sol))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 24
timeout 90 python3 ik.py -0.202985648078355 0 1.2696238083193787 0.9995966352021468 0 -0.028400121347386207 0; echo "---base-frame test:"; timeout 90 python3 ik.py 0.457 0 0.3576 0.9995966352021468 0 -0.028400121347386207 0

# openrua op 25
timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.05846,-0.16131,-0.05792,-2.44458,-0.01162,2.22670,0.00891]}}}" 2>&1 | grep -o "position=.*orientation=[^)]*)"

# openrua op 26
python3 -c "
import numpy as np
for cam in ['agentview','sideview','birdview']:
    d=np.load(f'snaps/{cam}_cloud.npz'); xyz=d['xyz']; P=xyz.reshape(-1,3)
    m=(P[:,0]>-0.16)&(P[:,0]<-0.03)&(P[:,1]>-0.245)&(P[:,1]<-0.17)&(P[:,2]>0.905)&(P[:,2]<1.03)
    Q=P[m]; print(cam,'handle-ish n',len(Q))
    for lo in np.arange(0.90,1.03,0.01):
        s=(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)
        if s.sum(): print(f'  z {lo:.2f}: n={s.sum():4d} x[{Q[s,0].min():.3f},{Q[s,0].max():.3f}] y[{Q[s,1].min():.3f},{Q[s,1].max():.3f}]')
# body center estimate from rim ring (z 1.0-1.013) in birdview
d=np.load('snaps/birdview_cloud.npz'); P=d['xyz'].reshape(-1,3)
m=(P[:,0]>-0.2)&(P[:,0]<0.0)&(P[:,1]>-0.36)&(P[:,1]<-0.23)&(P[:,2]>0.995)
Q=P[m]; print('rim ring center', Q[:,:2].mean(0).round(4), 'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3), 'z', Q[:,2].max().round(3))
"


# openrua op 27
timeout 60 python3 -c "
import numpy as np, struct
def load_stl(p):
    data=open(p,'rb').read()
    if data[:5]==b'solid' and b'facet' in data[:300]:
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',data)])
    else:
        n=struct.unpack('<I',data[80:84])[0]
        arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v
for name in ['hand','finger']:
    v=load_stl(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{name}.stl')
    print(name, 'min',v.min(0).round(4),'max',v.max(0).round(4))
v=load_stl('/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/hand.stl')
for lo in np.arange(0,0.07,0.01):
    s=(v[:,2]>=lo)&(v[:,2]<lo+0.01)
    if s.sum(): print(f'hand z {lo:.2f}: x[{v[s,0].min():.3f},{v[s,0].max():.3f}] y[{v[s,1].min():.3f},{v[s,1].max():.3f}]')
"


# openrua op 28
find / -iname "hand*.stl" -o -iname "finger*.stl" -o -iname "hand*.dae" 2>/dev/null | head; find / -path /proc -prune -o -iname "*panda*" -print 2>/dev/null | grep -v -E "^/proc|robot.urdf" | head -20

# openrua op 29
mkdir -p "$(dirname /workspace/motion.py)"
cat > /workspace/motion.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Motion helper for the Panda (world-frame poses, hand-frame semantics).

Subcommands:
  js                                  print arm joints + finger positions
  fk                                  print current panda_hand pose (world)
  ik  x y z  R(9 values row-major) [--tcp]   IK for a HAND pose (or TCP pose with --tcp); prints joints
  goto x y z R(9) secs [--tcp] [--dry]        IK then trajectory (single point)
  path secs_per_pt "x,y,z,R9[;...]" [--tcp]   multi-waypoint smooth trajectory
  joints j1,...,j7 secs
  grip open|close
Notes: IK tip is panda_link8 = hand rotated by -45deg about z; handled here.
"""
import sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034
FJT = "/panda_arm_controller/follow_joint_trajectory"
GRIP = "/franka_gripper/gripper_action"


def R_to_quat(R):
    # returns x,y,z,w
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s; x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return np.array([x, y, z, w])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


HAND_TO_L8 = Rz(np.pi / 4)  # R_link8 = R_hand @ Rz(+45deg)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("motion_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT)
        self.grip = ActionClient(self.node, GripperCommand, GRIP)

    def _on_js(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js = {}
        end = time.time() + 15
        while not all(j in self.js for j in JOINTS) and time.time() < end:
            self.spin()
        return [self.js[j] for j in JOINTS]

    def fingers(self):
        self.joints()
        return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")

    def fk(self, q=None):
        q = q if q is not None else self.joints()
        self.fk_cli.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        R = quat_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return np.array([p.position.x, p.position.y, p.position.z]), R

    def ik(self, pos, R_hand, seed=None, tcp=False, tries=3):
        """pos: world position of hand (or TCP if tcp=True); R_hand: 3x3 hand rotation (world)."""
        pos = np.array(pos, float)
        R_hand = np.array(R_hand, float).reshape(3, 3)
        if tcp:
            pos = pos - TCP * R_hand[:, 2]
        R8 = R_hand @ HAND_TO_L8
        q = R_to_quat(R8)
        seed = list(seed) if seed is not None else self.joints()
        self.ik_cli.wait_for_service(10)
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state.name = JOINTS
            req.ik_request.robot_state.joint_state.position = [float(s) for s in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                out = [sol[j] for j in JOINTS]
                # verify
                fp, fR = self.fk(out)
                err_p = np.linalg.norm(fp - pos)
                err_R = np.linalg.norm(fR - R_hand)
                if err_p < 2e-3 and err_R < 0.02:
                    return out
                print(f"  ik verify mismatch pos {err_p:.4f} rot {err_R:.4f}", file=sys.stderr)
            # perturb seed slightly and retry
            seed = [s + np.random.uniform(-0.15, 0.15) for s in seed]
        return None

    def move_joints(self, waypoints, secs_per_pt, first_secs=None):
        """waypoints: list of 7-lists. Returns error_code."""
        if not self.fjt.wait_for_server(timeout_sec=15):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        t = 0.0
        for i, w in enumerate(waypoints):
            t += (first_secs if (i == 0 and first_secs) else secs_per_pt)
            pt = JointTrajectoryPoint(positions=[float(v) for v in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise SystemExit("goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=1500)
        if rf.result() is None:
            print("no result (timeout)"); return -99
        code = rf.result().result.error_code
        q = self.joints()
        err = max(abs(a - b) for a, b in zip(q, waypoints[-1]))
        print(f"traj error_code={code} final_joint_err={err:.4f}")
        return code

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=15):
            raise SystemExit("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        f = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}")


def parse_R(vals):
    return np.array([float(v) for v in vals]).reshape(3, 3)


def main():
    a = sys.argv[1:]
    flags = [x for x in a if x.startswith("--")]
    a = [x for x in a if not x.startswith("--")]
    cmd = a[0]
    r = Robot()
    if cmd == "js":
        print("arm:", [round(v, 4) for v in r.joints()], "fingers:", r.fingers())
    elif cmd == "fk":
        p, R = r.fk(); print("hand pos", p.round(4)); print("R", R.round(3)); print("tcp", (p + TCP * R[:, 2]).round(4))
    elif cmd == "ik":
        pos = [float(v) for v in a[1:4]]; R = parse_R(a[4:13])
        sol = r.ik(pos, R, tcp="--tcp" in flags)
        print("IK FAILED" if sol is None else ",".join(f"{v:.5f}" for v in sol))
    elif cmd == "goto":
        pos = [float(v) for v in a[1:4]]; R = parse_R(a[4:13]); secs = float(a[13])
        sol = r.ik(pos, R, tcp="--tcp" in flags)
        if sol is None:
            print("IK FAILED"); sys.exit(1)
        print("target joints", [round(v, 4) for v in sol])
        if "--dry" not in flags:
            r.move_joints([sol], secs)
            p, R2 = r.fk(); print("hand now", p.round(4), "tcp", (p + TCP * R2[:, 2]).round(4))
    elif cmd == "path":
        secs = float(a[1]); pts = a[2].split(";")
        seed = r.joints(); sols = []
        for s in pts:
            v = [float(x) for x in s.split(",")]
            sol = r.ik(v[:3], parse_R(v[3:12]), seed=seed, tcp="--tcp" in flags)
            if sol is None:
                print("IK FAILED at", s); sys.exit(1)
            sols.append(sol); seed = sol
        for s in sols: print("wp", [round(v, 3) for v in s])
        if "--dry" not in flags:
            r.move_joints(sols, secs)
            p, R2 = r.fk(); print("hand now", p.round(4), "tcp", (p + TCP * R2[:, 2]).round(4))
    elif cmd == "joints":
        q = [float(v) for v in a[1].split(",")]; r.move_joints([q], float(a[2]))
    elif cmd == "grip":
        r.gripper(0.04 if a[1] == "open" else 0.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 30
timeout 120 python3 motion.py fk && timeout 120 python3 motion.py ik -0.203 0 1.2696 0.998 0 -0.057 0 -1 0 -0.057 0 -0.998

# openrua op 31
python3 - <<'EOF'
import numpy as np, subprocess
a=np.radians(55); s,c=np.sin(a),np.cos(a)
Rpick=[0,1,0, c,0,-s, -s,0,-c]
Rplace=[0,-1,0, -c,0,s, -s,0,-c]
def fmt(p,R): return ",".join(f"{v:.5f}" for v in list(p)+list(R))
pts=[("prepick",(-0.100,-0.218,1.07),Rpick),("pick",(-0.100,-0.218,0.982),Rpick),("lift",(-0.100,-0.218,1.25),Rpick),
     ("preplace_hi",(-0.03,0.105,1.25),Rplace),("preplace_lo",(-0.03,0.105,1.04),Rplace),("place",(-0.03,0.255,1.04),Rplace)]
for name,p,R in pts:
    out=subprocess.run(["python3","motion.py","ik",*fmt(p,R).split(","),"--tcp"],capture_output=True,text=True,timeout=300)
    print(name, out.stdout.strip(), out.stderr.strip()[-200:])
EOF

# openrua op 32
python3 - <<'EOF'
import numpy as np, subprocess
a=np.radians(55); s,c=np.sin(a),np.cos(a)
Rpick=np.array([[0,1,0],[c,0,-s],[-s,0,-c]])
def Rz(t):
    c,s=np.cos(t),np.sin(t); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
p0=np.array([-0.100,-0.218,1.25]); p1=np.array([-0.03,0.105,1.25])
pts=[]
for k,th in enumerate(np.linspace(0,np.pi,7)):
    f=k/6; p=p0+(p1-p0)*f; R=Rz(th)@Rpick
    pts.append(",".join(f"{v:.5f}" for v in list(p)+list(R.flatten())))
out=subprocess.run(["python3","motion.py","path","2",";".join(pts),"--tcp","--dry"],capture_output=True,text=True,timeout=600)
print(out.stdout, out.stderr[-500:])
EOF

# openrua op 33
python3 - <<'EOF'
import numpy as np, subprocess
a=np.radians(55); s,c=np.sin(a),np.cos(a)
Rpick=np.array([[0,1,0],[c,0,-s],[-s,0,-c]])
def Rz(t):
    c,s=np.cos(t),np.sin(t); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
for p in [np.array([-0.10,-0.05,1.25]), np.array([-0.25,-0.05,1.30])]:
    pts=[]
    for th in np.radians(np.arange(0,181,15)):
        R=Rz(th)@Rpick
        pts.append(",".join(f"{v:.5f}" for v in list(p)+list(R.flatten())))
    out=subprocess.run(["python3","motion.py","path","2",";".join(pts),"--tcp","--dry"],capture_output=True,text=True,timeout=900)
    print('pos',p); print(out.stdout, out.stderr[-300:])
EOF

# openrua op 34
python3 - <<'EOF'
import numpy as np, subprocess
a=np.radians(55); s,c=np.sin(a),np.cos(a)
Rpick=np.array([[0,1,0],[c,0,-s],[-s,0,-c]])
def Rz(t):
    c,s=np.cos(t),np.sin(t); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
for p in [np.array([-0.25,-0.05,1.30]), np.array([-0.15,0.0,1.35])]:
    pts=[]
    for th in -np.radians(np.arange(0,181,15)):
        R=Rz(th)@Rpick
        pts.append(",".join(f"{v:.5f}" for v in list(p)+list(R.flatten())))
    out=subprocess.run(["python3","motion.py","path","2",";".join(pts),"--tcp","--dry"],capture_output=True,text=True,timeout=900)
    print('pos',p); print(out.stdout, out.stderr[-300:])
EOF

# openrua op 35
python3 - <<'EOF' > poses.txt
import numpy as np
a=np.radians(60); s,c=np.sin(a),np.cos(a)
Rpick=[0,1,0, c,0,-s, -s,0,-c]
Rplace=[0,-1,0, -c,0,s, -s,0,-c]
print("RPICK="+" ".join(f"{v:.6f}" for v in Rpick))
print("RPLACE="+" ".join(f"{v:.6f}" for v in Rplace))
EOF
cat poses.txt; source poses.txt; timeout 600 python3 motion.py goto -0.100 -0.215 1.06 $RPICK 4 --tcp

# openrua op 36
timeout 600 python3 motion.py goto -0.100 -0.215 1.06 0 1 0 0.5 0 -0.866025 -0.866025 0 -0.5 4 --tcp

# openrua op 37
timeout 60 python3 motion.py js; timeout 600 python3 motion.py joints -0.1231,0.4738,-0.0086,-2.051,-1.3425,2.0668,0.0758 3; timeout 60 python3 motion.py fk

# openrua op 38
timeout 600 python3 motion.py joints -0.1231,0.4738,-0.0086,-2.051,-1.3425,2.0668,0.0758 4; timeout 60 python3 motion.py js; timeout 60 python3 motion.py fk

# openrua op 39
timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side_pregrasp.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_pregrasp.png

# openrua op 40
timeout 600 python3 motion.py goto -0.100 -0.215 0.977 0 1 0 0.5 0 -0.866025 -0.866025 0 -0.5 3 --tcp

# openrua op 41
timeout 600 python3 motion.py joints -0.1155,0.6637,-0.0216,-1.9771,-1.402,2.0985,0.2041 2; timeout 60 python3 motion.py fk; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_grasp.png; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side_grasp.png

# openrua op 42
timeout 600 python3 motion.py grip close

# openrua op 43
timeout 600 python3 motion.py goto -0.100 -0.215 1.15 0 1 0 0.5 0 -0.866025 -0.866025 0 -0.5 4 --tcp; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_lift.png; timeout 60 python3 motion.py js

# openrua op 44
timeout 600 python3 motion.py goto -0.25 -0.05 1.30 0 1 0 0.5 0 -0.866025 -0.866025 0 -0.5 4 --tcp && python3 - <<'EOF'
import numpy as np, subprocess
a=np.radians(60); s,c=np.sin(a),np.cos(a)
Rpick=np.array([[0,1,0],[c,0,-s],[-s,0,-c]])
def Rz(t):
    c,s=np.cos(t),np.sin(t); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
p=np.array([-0.25,-0.05,1.30])
pts=[]
for th in -np.radians(np.arange(15,181,15)):
    R=Rz(th)@Rpick
    pts.append(",".join(f"{v:.5f}" for v in list(p)+list(R.flatten())))
out=subprocess.run(["python3","motion.py","path","1.5",";".join(pts),"--tcp"],capture_output=True,text=True,timeout=1500)
print(out.stdout, out.stderr[-500:])
EOF

# openrua op 45
timeout 60 python3 motion.py fk; timeout 60 python3 motion.py js; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_sweep.png

# openrua op 46
python3 motion.py goto -0.03 0.105 1.25 0 -1 0 -0.5 0 0.866025 -0.866025 0 -0.5 4 --tcp 2>&1 | tail -5

# openrua op 47
python3 motion.py goto -0.03 0.105 1.04 0 -1 0 -0.5 0 0.866025 -0.866025 0 -0.5 3 --tcp 2>&1 | tail -3 && python3 tools/perception/cam_snap.py agentview snaps/agent_prelo.png 2>&1 | tail -1

# openrua op 48
python3 tools/perception/cam_snap.py sideview snaps/side_prelo.png 2>&1 | tail -1; python3 tools/perception/cam_snap.py frontview snaps/front_prelo.png 2>&1 | tail -1

# openrua op 49
python3 cloud.py frontview 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
# white mug: bright, low saturation, in region x -0.15..0.1, y 0.0..0.25, z 0.9..1.15
m=(rgb.min(-1)>150)&(np.ptp(rgb,-1)<25)&(xyz[...,2]>0.90)&(xyz[...,2]<1.12)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.12)&(xyz[...,1]>-0.05)&(xyz[...,1]<0.3)
p=xyz[m]; print(len(p))
if len(p): print('x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
# histogram of y for those points
h,e=np.histogram(p[:,1],bins=12); print(list(zip(e.round(3),h)))
EOF

# openrua op 50
python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(rgb.min(-1)>150)&(np.ptp(rgb,-1)<25)&(xyz[...,2]>0.90)&(xyz[...,2]<1.12)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.12)&(xyz[...,1]>0.09)&(xyz[...,1]<0.3)
p=xyz[m]; print(len(p))
print('x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
for zlo in np.arange(0.95,1.12,0.02):
    s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.02)]
    if len(s): print(f"z {zlo:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 51
python3 - <<'EOF'
import numpy as np
for c in ['frontview','agentview','sideview']:
    try:
        d=np.load(f'snaps/{c}_cloud.npz'); print(c,'cam pos',d['p0'].round(3),'view dir',d['R'][:,2].round(3))
    except Exception as e: print(c,e)
EOF

# openrua op 52
python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(rgb.min(-1)>120)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.92)&(xyz[...,2]<0.99)&(xyz[...,0]>-0.1)&(xyz[...,0]<0.05)&(xyz[...,1]>0.08)&(xyz[...,1]<0.3)
p=xyz[m]; print(len(p)); z=np.sort(p[:,2]); print(z[:10].round(3), np.percentile(z,[1,5,10]).round(3))
lo=p[p[:,2]<0.975]; print('low pts y',lo[:,1].round(3), 'x',lo[:,0].round(3))
EOF

# openrua op 53
python3 motion.py goto -0.03 0.105 1.045 0 -1 0 -0.5 0 0.866025 -0.866025 0 -0.5 2 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.03 0.25 1.045 0 -1 0 -0.5 0 0.866025 -0.866025 0 -0.5 4 --tcp 2>&1 | tail -3 && python3 motion.py js

# openrua op 54
timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>&1 | grep -A7 wrench; python3 tools/perception/cam_snap.py frontview snaps/front_place.png | tail -1; python3 tools/perception/cam_snap.py agentview snaps/agent_place.png | tail -1

# openrua op 55
python3 cloud.py frontview 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(rgb.min(-1)>120)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.92)&(xyz[...,2]<1.2)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.1)&(xyz[...,1]>0.0)&(xyz[...,1]<0.45)
p=xyz[m]; print(len(p))
for zlo in np.arange(0.94,1.2,0.02):
    s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.02)]
    if len(s): print(f"z {zlo:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF
python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_place.png | tail -1

# openrua op 56
python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(rgb.min(-1)>120)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.92)&(xyz[...,2]<1.2)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.1)&(xyz[...,1]>0.15)&(xyz[...,1]<0.45)
p=xyz[m]; print(len(p))
for zlo in np.arange(0.94,1.2,0.01):
    s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)]
    if len(s): print(f"z {zlo:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 57
python3 motion.py joints 0.0561,0.6786,0.1384,-1.7631,1.2563,2.1265,1.6178 3 2>&1 | tail -1; python3 motion.py fk | tail -1; python3 motion.py js; timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>&1 | grep -A3 force

# openrua op 58
python3 motion.py grip open 2>&1 | tail -1; python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_release.png | tail -1

# openrua op 59
for c in agentview frontview robot0_robotview birdview; do python3 tools/perception/cam_snap.py $c snaps/${c}_rel.png | tail -1; done

# openrua op 60
python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
ok=np.isfinite(xyz).all(-1)&(d['depth']>0.05)
p=xyz[ok]; c=rgb[ok]
print('cam',d['p0'].round(3),d['R'][:,2].round(3))
# dark microwave points
dark=c.max(-1)<80
q=p[dark]
print('dark pts',len(q))
# floor of cavity: dark points with y>0.25 and z<1.0 , horizontal
for ylo in np.arange(0.20,0.46,0.02):
    s=q[(q[:,1]>=ylo)&(q[:,1]<ylo+0.02)&(q[:,2]<1.02)&(q[:,0]>-0.14)&(q[:,0]<0.06)]
    if len(s): print(f"y {ylo:.2f}: n={len(s)} z min {s[:,2].min():.3f} 5% {np.percentile(s[:,2],5):.3f} med {np.median(s[:,2]):.3f} max {s[:,2].max():.3f}")
EOF

# openrua op 61
python3 - <<'EOF'
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
ok=np.isfinite(xyz).all(-1)&(d['depth']>0.05)
p=xyz[ok]; c=rgb[ok]
q=p[c.max(-1)<80]
s=q[(q[:,1]>=0.19)&(q[:,1]<0.245)&(q[:,2]<1.05)]
print(len(s))
for zlo in np.arange(0.90,1.05,0.01):
    t=s[(s[:,2]>=zlo)&(s[:,2]<zlo+0.01)]
    if len(t): print(f"z {zlo:.2f}: n={len(t)} x {t[:,0].min():.3f}..{t[:,0].max():.3f} y {np.percentile(t[:,1],5):.3f}..{np.percentile(t[:,1],95):.3f}")
# also: all points (any colour) near x in opening range, z 0.90-0.96, y 0.18-0.26 -> profile of the sill
s=p[(p[:,0]>-0.12)&(p[:,0]<0.04)&(p[:,1]>0.18)&(p[:,1]<0.27)&(p[:,2]<0.97)]
print('sill profile (y bins -> z max):')
for ylo in np.arange(0.18,0.27,0.005):
    t=s[(s[:,1]>=ylo)&(s[:,1]<ylo+0.005)]
    if len(t): print(f"  y {ylo:.3f}: n={len(t)} zmax {t[:,2].max():.3f} zmed {np.median(t[:,2]):.3f}")
EOF

# openrua op 62
python3 cloud.py frontview 2>&1 | tail -1 && python3 cloud.py agentview 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
for cam in ['frontview','agentview']:
    d=np.load(f'snaps/{cam}_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
    m=(rgb.min(-1)>110)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.905)&(xyz[...,2]<1.05)&(xyz[...,0]>-0.2)&(xyz[...,0]<0.15)&(xyz[...,1]>0.05)&(xyz[...,1]<0.24)
    p=xyz[m]; print(cam,len(p))
    for zlo in np.arange(0.91,1.03,0.01):
        s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)]
        if len(s)>5: print(f"  z {zlo:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 63
R="0 -1 0 -0.5 0 0.866025 -0.866025 0 -0.5"
python3 motion.py goto -0.022 0.110 1.10 $R 3 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.022 0.110 0.984 $R 3 --tcp 2>&1 | tail -2 && python3 tools/perception/cam_snap.py frontview snaps/front_regrasp.png | tail -1

# openrua op 64
R="0 -1 0 -0.5 0 0.866025 -0.866025 0 -0.5"
python3 motion.py goto -0.022 0.110 0.984 $R 2 --tcp 2>&1 | tail -2; timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>&1 | grep -A3 force

# openrua op 65
python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_regrasp.png | tail -1; python3 tools/perception/cam_snap.py agentview snaps/agent_regrasp.png | tail -1

# openrua op 66
R="0 -1 0 -0.5 0 0.866025 -0.866025 0 -0.5"
python3 motion.py goto -0.022 0.110 1.12 $R 3 --tcp 2>&1 | tail -1 && python3 cloud.py agentview 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/agentview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
r,g,b=rgb[...,0],rgb[...,1],rgb[...,2]
m=(r>150)&(g>130)&(b<120)&(xyz[...,2]>0.905)&(xyz[...,2]<1.05)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.2)&(xyz[...,1]>-0.2)&(xyz[...,1]<0.12)
p=xyz[m]; print('yellow pts',len(p))
for zlo in np.arange(0.90,1.03,0.01):
    s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)]
    if len(s)>5: print(f"  z {zlo:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
h=p[p[:,1]<-0.03]; print('handle-ish pts',len(h))
if len(h): print('  x',h[:,0].min().round(3),h[:,0].max().round(3),'y',h[:,1].min().round(3),h[:,1].max().round(3),'z',h[:,2].min().round(3),h[:,2].max().round(3))
EOF

# openrua op 67
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py goto 0.010 -0.052 1.10 $RV 4 --tcp 2>&1 | tail -1 && python3 motion.py goto 0.010 -0.052 0.984 $RV 3 --tcp 2>&1 | tail -2 && python3 tools/perception/cam_snap.py agentview snaps/agent_ygrasp.png | tail -1

# openrua op 68
python3 motion.py js; python3 motion.py fk

# openrua op 69
RC="0.941 0.339 -0.018 0.337 -0.926 0.168 0.04 -0.164 -0.986"
python3 motion.py goto 0.0081 -0.0331 1.12 $RC 3 --tcp 2>&1 | tail -2
for R in "0 1 0 1 0 0 0 0 -1" "0 -1 0 -1 0 0 0 0 -1"; do echo "R=$R"; python3 motion.py goto 0.010 -0.052 1.12 $R 3 --tcp --dry 2>&1 | tail -1; done; python3 motion.py js

# openrua op 70
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py goto 0.010 -0.052 1.12 $RV 6 --tcp 2>&1 | tail -2 && python3 motion.py goto 0.010 -0.052 0.984 $RV 4 --tcp 2>&1 | tail -2

# openrua op 71
python3 motion.py grip close 2>&1 | tail -1; python3 tools/perception/cam_snap.py agentview snaps/agent_ygrip.png | tail -1

# openrua op 72
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py goto 0.010 -0.052 1.15 $RV 3 --tcp 2>&1 | tail -1 && python3 motion.py goto 0.05 -0.30 1.15 $RV 4 --tcp 2>&1 | tail -1 && python3 motion.py goto 0.05 -0.30 0.995 $RV 3 --tcp 2>&1 | tail -2 && python3 motion.py grip open | tail -1 && python3 motion.py goto 0.05 -0.30 1.15 $RV 3 --tcp 2>&1 | tail -1 && python3 tools/perception/cam_snap.py agentview snaps/agent_ymoved.png | tail -1

# openrua op 73
R="0 -1 0 -0.5 0 0.866025 -0.866025 0 -0.5"
python3 motion.py goto -0.022 0.110 1.12 $R 6 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.022 0.110 0.984 $R 3 --tcp 2>&1 | tail -2

# openrua op 74
python3 motion.py js; python3 motion.py fk; python3 tools/perception/cam_snap.py agentview snaps/agent_oops.png | tail -1; python3 tools/perception/cam_snap.py frontview snaps/front_oops.png | tail -1

# openrua op 75
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/motion.py", "old_string": "    elif cmd == \\"goto\\":\\n        pos = [float(v) for v in a[1:4]]; R = parse_R(a[4:13]); secs = float(a[13])\\n        sol = r.ik(pos, R, tcp=\\"--tcp\\" in flags)\\n        if sol is None:\\n            print(\\"IK FAILED\\"); sys.exit(1)\\n        print(\\"target joints\\", [round(v, 4) for v in sol])", "new_string": "    elif cmd == \\"goto\\":\\n        pos = [float(v) for v in a[1:4]]; R = parse_R(a[4:13]); secs = float(a[13])\\n        seed = None\\n        for f in flags:\\n            if f.startswith(\\"--seed=\\"):\\n                seed = [float(v) for v in f[7:].split(\\",\\")]\\n        sol = r.ik(pos, R, seed=seed, tcp=\\"--tcp\\" in flags)\\n        if sol is None:\\n            print(\\"IK FAILED\\"); sys.exit(1)\\n        cur = r.joints()\\n        print(\\"target joints\\", [round(v, 4) for v in sol], \\"max_dq\\", round(max(abs(x - y) for x, y in zip(sol, cur)), 3))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 76
RC="0.741 -0.554 -0.38 -0.236 -0.745 0.624 -0.629 -0.373 -0.682"
python3 motion.py goto -0.1038 0.0582 1.20 $RC 4 --tcp 2>&1 | tail -3

# openrua op 77
R="0 -1 0 -0.5 0 0.866025 -0.866025 0 -0.5"
python3 motion.py goto -0.022 0.110 1.15 $R 8 --tcp --seed=-0.1096,0.6709,0.0364,-1.7717,1.1433,1.9309,1.5429 --dry 2>&1 | tail -1

# openrua op 78
python3 - <<'EOF' 2>&1 | grep -v "^\[" 
import numpy as np, sys
sys.argv=['x']
from motion import Robot, TCP
r=Robot()
q0=np.array(r.joints()); q1=np.array([-0.1034, 0.4932, 0.0279, -1.7741, 1.1, 1.8501, 1.7066])
for s in np.linspace(0,1,9):
    p,R=r.fk(q0+(q1-q0)*s); print(round(s,3),'hand',p.round(3),'tcp',(p+TCP*R[:,2]).round(3))
EOF

# openrua op 79
python3 motion.py joints -0.1034,0.4932,0.0279,-1.7741,1.1,1.8501,1.7066 8 2>&1 | tail -1; python3 motion.py joints -0.1034,0.4932,0.0279,-1.7741,1.1,1.8501,1.7066 3 2>&1 | tail -1; python3 motion.py fk | tail -1

# openrua op 80
R="0 -1 0 -0.5 0 0.866025 -0.866025 0 -0.5"
python3 motion.py goto -0.022 0.110 0.984 $R 4 --tcp 2>&1 | tail -3

# openrua op 81
python3 motion.py grip close 2>&1 | tail -1; python3 tools/perception/cam_snap.py agentview snaps/agent_wgrip.png | tail -1

# openrua op 82
R="0 -1 0 -0.5 0 0.866025 -0.866025 0 -0.5"
python3 motion.py goto -0.022 0.110 1.06 $R 3 --tcp 2>&1 | tail -1; python3 motion.py js | tail -1; python3 tools/perception/cam_snap.py frontview snaps/front_wlift.png | tail -1

# openrua op 83
python3 cloud.py frontview 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(rgb.min(-1)>90)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.905)&(xyz[...,2]<1.03)&(xyz[...,0]>-0.1)&(xyz[...,0]<0.06)&(xyz[...,1]>0.07)&(xyz[...,1]<0.135)
p=xyz[m]; print('handle region pts',len(p))
for zlo in np.arange(0.90,1.03,0.005):
    s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.005)]
    if len(s)>3: print(f"  z {zlo:.3f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
# body
m=(rgb.min(-1)>90)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.905)&(xyz[...,2]<1.03)&(xyz[...,0]>-0.1)&(xyz[...,0]<0.06)&(xyz[...,1]>0.135)&(xyz[...,1]<0.24)
p=xyz[m]; print('body x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
EOF

# openrua op 84
R="0 -1 0 -0.5 0 0.866025 -0.866025 0 -0.5"
python3 motion.py grip open | tail -1 && python3 motion.py goto -0.0215 0.121 0.984 $R 3 --tcp 2>&1 | tail -1 && python3 motion.py grip close | tail -1 && python3 motion.py goto -0.0215 0.121 1.06 $R 3 --tcp 2>&1 | tail -1 && python3 motion.py js | tail -1 && python3 tools/perception/cam_snap.py frontview snaps/front_wlift2.png | tail -1

# openrua op 85
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py grip open | tail -1 && python3 motion.py goto -0.0215 0.122 1.10 $RV 6 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.0215 0.122 0.980 $RV 3 --tcp 2>&1 | tail -1 && python3 motion.py grip close | tail -1 && python3 motion.py goto -0.0215 0.122 1.03 $RV 3 --tcp 2>&1 | tail -1 && python3 motion.py js | tail -1

# openrua op 86
python3 motion.py fk; python3 tools/perception/cam_snap.py agentview snaps/agent_chk.png | tail -1; python3 tools/perception/cam_snap.py frontview snaps/front_chk.png | tail -1

# openrua op 87
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py grip open | tail -1 && python3 motion.py goto -0.0215 0.122 0.980 $RV 3 --tcp 2>&1 | tail -2

# openrua op 88
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py grip close | tail -1 && python3 motion.py goto -0.0215 0.122 1.03 $RV 3 --tcp 2>&1 | tail -1 && python3 motion.py js | tail -1

# openrua op 89
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py grip open | tail -1 && python3 motion.py goto -0.0215 0.126 0.988 $RV 3 --tcp 2>&1 | tail -2 && python3 motion.py grip close | tail -1 && python3 motion.py goto -0.0215 0.126 1.02 $RV 3 --tcp 2>&1 | tail -1 && python3 motion.py js | tail -1

# openrua op 90
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py grip open | tail -1 && python3 motion.py goto -0.0215 0.126 0.988 $RV 3 --tcp 2>&1 | tail -1 && python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_pre.png | tail -1 && python3 tools/perception/cam_snap.py frontview snaps/front_pre.png | tail -1 && python3 motion.py grip close | tail -1 && python3 tools/perception/cam_snap.py frontview snaps/front_closed.png | tail -1

# openrua op 91
python3 motion.py grip open | tail -1; python3 motion.py goto -0.0215 0.126 1.15 0 1 0 1 0 0 0 0 -1 3 --tcp 2>&1 | tail -1; python3 cloud.py birdview 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/birdview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
print('cam',d['p0'].round(3),d['R'][:,2].round(3))
m=(rgb.min(-1)>90)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.905)&(xyz[...,2]<1.03)&(xyz[...,0]>-0.1)&(xyz[...,0]<0.06)&(xyz[...,1]>0.07)&(xyz[...,1]<0.14)
p=xyz[m]; print('handle pts',len(p))
for ylo in np.arange(0.07,0.14,0.005):
    s=p[(p[:,1]>=ylo)&(p[:,1]<ylo+0.005)]
    if len(s)>2: print(f"  y {ylo:.3f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}")
m=(rgb.min(-1)>90)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.99)&(xyz[...,2]<1.03)&(xyz[...,0]>-0.12)&(xyz[...,0]<0.08)&(xyz[...,1]>0.13)&(xyz[...,1]<0.26)
p=xyz[m]; print('rim x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'n',len(p))
EOF

# openrua op 92
python3 motion.py goto 0.15 -0.05 1.20 0 1 0 1 0 0 0 0 -1 4 --tcp 2>&1 | tail -2; python3 cloud.py birdview 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/birdview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(rgb.min(-1)>90)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.905)&(xyz[...,2]<1.03)&(xyz[...,0]>-0.1)&(xyz[...,0]<0.06)&(xyz[...,1]>0.07)&(xyz[...,1]<0.14)
p=xyz[m]; print('handle pts',len(p))
for ylo in np.arange(0.07,0.14,0.005):
    s=p[(p[:,1]>=ylo)&(p[:,1]<ylo+0.005)]
    if len(s)>2: print(f"  y {ylo:.3f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}")
m=(rgb.min(-1)>90)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.99)&(xyz[...,2]<1.03)&(xyz[...,0]>-0.12)&(xyz[...,0]<0.08)&(xyz[...,1]>0.13)&(xyz[...,1]<0.26)
p=xyz[m]; print('rim x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'n',len(p))
EOF

# openrua op 93
python3 motion.py goto 0.10 -0.02 1.20 0 1 0 1 0 0 0 0 -1 4 --tcp 2>&1 | tail -2; python3 cloud.py birdview 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/birdview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
box=(xyz[...,0]>-0.12)&(xyz[...,0]<0.08)&(xyz[...,1]>0.05)&(xyz[...,1]<0.26)&(xyz[...,2]>0.905)&(xyz[...,2]<1.05)
p=xyz[box]; c=rgb[box]; print('pts in box',len(p))
for ylo in np.arange(0.05,0.26,0.01):
    s=p[(p[:,1]>=ylo)&(p[:,1]<ylo+0.01)]
    if len(s)>2: print(f"  y {ylo:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}")
EOF

# openrua op 94
python3 motion.py goto -0.03 0.16 1.40 0 1 0 1 0 0 0 0 -1 4 --tcp 2>&1 | tail -2; python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
print('cam',d['p0'].round(3),d['R'][:,2].round(3))
ok=np.isfinite(xyz).all(-1)
box=ok&(xyz[...,0]>-0.15)&(xyz[...,0]<0.08)&(xyz[...,1]>0.05)&(xyz[...,1]<0.26)&(xyz[...,2]>0.905)&(xyz[...,2]<1.05)
p=xyz[box]; print('pts',len(p))
# rim: z>1.0
rim=p[p[:,2]>1.0]; 
if len(rim):
    # fit circle by least squares on x,y
    A=np.c_[2*rim[:,0],2*rim[:,1],np.ones(len(rim))]; b=rim[:,0]**2+rim[:,1]**2
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx*cx+cy*cy)
    print(f'rim circle centre ({cx:.4f},{cy:.4f}) r {r:.4f} n {len(rim)} zmax {rim[:,2].max():.3f}')
# handle: points outside the circle by >0.005 with z<1.0
dist=np.hypot(p[:,0]-cx,p[:,1]-cy)
h=p[(dist>r+0.004)&(p[:,2]>0.92)&(p[:,2]<1.0)&(dist<r+0.05)]
print('handle pts',len(h))
if len(h):
    ang=np.degrees(np.arctan2(h[:,0]-cx,-(h[:,1]-cy)))  # angle from -y toward +x
    print('handle angle deg: med',np.median(ang).round(1),'range',np.percentile(ang,[5,95]).round(1))
    print('handle radial dist',np.percentile(dist[(dist>r+0.004)&(p[:,2]>0.92)&(p[:,2]<1.0)&(dist<r+0.05)],[5,50,95]).round(3))
    print('handle z',h[:,2].min().round(3),h[:,2].max().round(3))
    for zlo in np.arange(0.92,1.0,0.01):
        s=h[(h[:,2]>=zlo)&(h[:,2]<zlo+0.01)]
        if len(s)>2:
            ds=np.hypot(s[:,0]-cx,s[:,1]-cy); print(f"  z {zlo:.2f}: n={len(s)} radial {ds.min():.3f}..{ds.max():.3f}")
np.save('snaps/mug_fit.npy',np.array([cx,cy,r]))
EOF

# openrua op 95
python3 - <<'EOF'
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
ok=np.isfinite(xyz).all(-1)&(rgb.min(-1)>100)&(np.ptp(rgb,-1)<40)
box=ok&(xyz[...,0]>-0.15)&(xyz[...,0]<0.08)&(xyz[...,1]>0.05)&(xyz[...,1]<0.245)&(xyz[...,2]>0.905)&(xyz[...,2]<1.03)
p=xyz[box]; print('white pts',len(p), 'zmax',p[:,2].max().round(3))
rim=p[p[:,2]>1.005]
A=np.c_[2*rim[:,0],2*rim[:,1],np.ones(len(rim))]; b=rim[:,0]**2+rim[:,1]**2
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx*cx+cy*cy)
print(f'rim circle centre ({cx:.4f},{cy:.4f}) r {r:.4f} n {len(rim)}')
res=np.hypot(rim[:,0]-cx,rim[:,1]-cy)-r; print('rim residual std',res.std().round(4))
dist=np.hypot(p[:,0]-cx,p[:,1]-cy)
sel=(dist>r+0.006)&(p[:,2]<1.0)&(dist<r+0.05)
h=p[sel]; dh=dist[sel]
print('handle pts',len(h))
ang=np.degrees(np.arctan2(h[:,0]-cx,-(h[:,1]-cy)))
print('handle angle deg: med',np.median(ang).round(1),'pct',np.percentile(ang,[10,50,90]).round(1))
print('handle radial',np.percentile(dh,[5,50,95]).round(3),'z',h[:,2].min().round(3),h[:,2].max().round(3))
# handle width: project onto tangential direction at median angle
a=np.radians(np.median(ang)); rad=np.array([np.sin(a),-np.cos(a)]); tan=np.array([np.cos(a),np.sin(a)])
rel=h[:,:2]-[cx,cy]; t=rel@tan; rr=rel@rad
print('tangential extent',np.percentile(t,[2,98]).round(4))
for zlo in np.arange(0.92,1.0,0.01):
    s=(h[:,2]>=zlo)&(h[:,2]<zlo+0.01)
    if s.sum()>2: print(f"  z {zlo:.2f}: n={s.sum()} radial {rr[s].min():.3f}..{rr[s].max():.3f} tang {t[s].min():.3f}..{t[s].max():.3f}")
np.save('snaps/mug_fit.npy',np.array([cx,cy,r,np.median(ang)]))
EOF

# openrua op 96
RY="-0.340 0.940 0 0.940 0.340 0 0 0 -1"
python3 motion.py goto -0.0104 0.1378 1.10 $RY 5 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.0104 0.1378 0.982 $RY 3 --tcp 2>&1 | tail -2

# openrua op 97
RY="-0.340 0.940 0 0.940 0.340 0 0 0 -1"
python3 motion.py grip close | tail -1 && python3 motion.py goto -0.0104 0.1378 1.03 $RY 3 --tcp 2>&1 | tail -1 && python3 motion.py js | tail -1

# openrua op 98
RY="-0.340 0.940 0 0.940 0.340 0 0 0 -1"
python3 motion.py goto -0.0104 0.1378 1.20 $RY 3 --tcp 2>&1 | tail -1 && python3 motion.py js | tail -1 && python3 tools/perception/cam_snap.py frontview snaps/front_lifted.png | tail -1

# openrua op 99
python3 - <<'EOF' > /tmp/path.txt
import numpy as np
Rv=np.array([[0,1,0],[1,0,0],[0,0,-1.0]])
def Rz(t): c,s=np.cos(t),np.sin(t); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def Rx(t): c,s=np.cos(t),np.sin(t); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
pts=[]
for th,al in [(15,15),(10,30),(5,45),(0,60)]:
    R=Rz(np.radians(th))@Rx(np.radians(al))@Rv
    pts.append(",".join(f"{v:.5f}" for v in [-0.0104,0.1378,1.20]+list(R.flatten())))
print(";".join(pts))
EOF
cat /tmp/path.txt; python3 motion.py path 2.5 "$(cat /tmp/path.txt)" --tcp 2>&1 | tail -3; python3 motion.py js | tail -1

# openrua op 100
RP="0 1 0 0.5 0 0.866025 0.866025 0 -0.5"
python3 tools/perception/cam_snap.py frontview snaps/front_tilted.png | tail -1; for p in "-0.03 0.10 1.045" "-0.03 0.20 1.045" "-0.03 0.29 1.045"; do python3 motion.py goto $p $RP 3 --tcp --dry 2>&1 | tail -1; done

# openrua op 101
python3 cloud.py frontview 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(rgb.min(-1)>110)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>1.05)&(xyz[...,2]<1.30)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.1)&(xyz[...,1]>0.12)&(xyz[...,1]<0.40)
p=xyz[m]; print('pts',len(p))
tcp=np.array([-0.0105,0.1374,1.1996])
print('z rel tcp: min',(p[:,2].min()-tcp[2]).round(3),'max',(p[:,2].max()-tcp[2]).round(3))
print('y rel tcp: min',(p[:,1].min()-tcp[1]).round(3),'max',(p[:,1].max()-tcp[1]).round(3))
print('x: ',p[:,0].min().round(3),p[:,0].max().round(3))
for zlo in np.arange(1.10,1.30,0.01):
    s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)]
    if len(s)>3: print(f"  z {zlo-tcp[2]:+.3f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} yrel {s[:,1].min()-tcp[1]:+.3f}..{s[:,1].max()-tcp[1]:+.3f}")
EOF

# openrua op 102
python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
tcp=np.array([-0.0105,0.1374,1.1996])
m=(rgb.min(-1)>110)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>1.0)&(xyz[...,2]<1.35)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.1)&(xyz[...,1]>tcp[1]+0.03)&(xyz[...,1]<0.45)
p=xyz[m]; print('pts',len(p))
print('z rel tcp: min',(p[:,2].min()-tcp[2]).round(3),'max',(p[:,2].max()-tcp[2]).round(3))
print('y rel tcp: min',(p[:,1].min()-tcp[1]).round(3),'max',(p[:,1].max()-tcp[1]).round(3))
for zlo in np.arange(1.05,1.35,0.01):
    s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)]
    if len(s)>3: print(f"  z {zlo-tcp[2]:+.3f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} yrel {s[:,1].min()-tcp[1]:+.3f}..{s[:,1].max()-tcp[1]:+.3f}")
EOF

# openrua op 103
python3 - <<'EOF' > /tmp/path2.txt
import numpy as np
Rv=np.array([[0,1,0],[1,0,0],[0,0,-1.0]])
def Rx(t): c,s=np.cos(t),np.sin(t); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
pts=[]
for al in [45,30,15,0]:
    R=Rx(np.radians(al))@Rv
    pts.append(",".join(f"{v:.5f}" for v in [-0.0104,0.1378,1.20]+list(R.flatten())))
print(";".join(pts))
EOF
python3 motion.py path 2.5 "$(cat /tmp/path2.txt)" --tcp 2>&1 | tail -2; python3 motion.py js | tail -1

# openrua op 104
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py goto -0.0104 0.1378 0.992 $RV 4 --tcp 2>&1 | tail -2 && python3 motion.py grip open | tail -1 && python3 motion.py goto -0.0104 0.1378 1.25 $RV 3 --tcp 2>&1 | tail -1 && python3 tools/perception/cam_snap.py frontview snaps/front_setdown.png | tail -1

# openrua op 105
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py goto -0.03 0.19 1.40 $RV 3 --tcp 2>&1 | tail -1; python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
ok=np.isfinite(xyz).all(-1)&(rgb.min(-1)>100)&(np.ptp(rgb,-1)<40)
box=ok&(xyz[...,0]>-0.15)&(xyz[...,0]<0.08)&(xyz[...,1]>0.03)&(xyz[...,1]<0.245)&(xyz[...,2]>0.905)&(xyz[...,2]<1.03)
p=xyz[box]; print('white pts',len(p),'zmax',p[:,2].max().round(3))
rim=p[p[:,2]>1.005]
A=np.c_[2*rim[:,0],2*rim[:,1],np.ones(len(rim))]; b=rim[:,0]**2+rim[:,1]**2
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx*cx+cy*cy)
print(f'rim centre ({cx:.4f},{cy:.4f}) r {r:.4f} n {len(rim)} resid {(np.hypot(rim[:,0]-cx,rim[:,1]-cy)-r).std():.4f}')
dist=np.hypot(p[:,0]-cx,p[:,1]-cy)
sel=(dist>r+0.006)&(p[:,2]<1.0)&(dist<r+0.05); h=p[sel]; dh=dist[sel]
ang=np.degrees(np.arctan2(h[:,0]-cx,-(h[:,1]-cy)))
print('handle n',len(h),'angle med',np.median(ang).round(1),'pct',np.percentile(ang,[10,90]).round(1),'radial',np.percentile(dh,[5,50,95]).round(3),'z',h[:,2].min().round(3),h[:,2].max().round(3))
a=np.radians(np.median(ang)); rad=np.array([np.sin(a),-np.cos(a)]); tan=np.array([np.cos(a),np.sin(a)])
rel=h[:,:2]-[cx,cy]; t=rel@tan; rr=rel@rad
for zlo in np.arange(0.92,1.0,0.01):
    s=(h[:,2]>=zlo)&(h[:,2]<zlo+0.01)
    if s.sum()>2: print(f"  z {zlo:.2f}: n={s.sum()} radial {rr[s].min():.3f}..{rr[s].max():.3f} tang {t[s].min():.3f}..{t[s].max():.3f}")
EOF

# openrua op 106
python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
o=np.array([-0.0104,0.048,1.2514]); zh=np.array([0,0.866,-0.5]); xh=np.array([0,0.5,0.866]); yh=np.array([1,0,0.])
m=(xyz[...,2]>1.15)&(xyz[...,2]<1.40)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.12)&(xyz[...,1]>-0.10)&(xyz[...,1]<0.14)
p=xyz[m]-o; c=rgb[m]
white=(c.min(-1)>100)
p=p[white]
t=p@zh; u=p@xh; v=p@yh
print('n',len(p))
for tlo in np.arange(-0.10,0.12,0.01):
    s=(t>=tlo)&(t<tlo+0.01)
    if s.sum()>3: print(f"  t(z_hand) {tlo:+.2f}: n={s.sum()} x_hand {u[s].min():+.3f}..{u[s].max():+.3f}  y_hand {v[s].min():+.3f}..{v[s].max():+.3f}")
EOF

# openrua op 107
python3 - <<'EOF' > /tmp/path3.txt
import numpy as np
Rv=np.array([[0,1,0],[1,0,0],[0,0,-1.0]])
def Rz(t): c,s=np.cos(t),np.sin(t); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def Rx(t): c,s=np.cos(t),np.sin(t); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
th=np.radians(-2.9)
pts=[]
for al,z in [(20,1.30),(40,1.25),(60,1.20),(60,1.10)]:
    R=Rz(th)@Rx(np.radians(al))@Rv
    pts.append(",".join(f"{v:.5f}" for v in [-0.0105,0.1013,z]+list(R.flatten())))
print(";".join(pts))
R=Rz(th)@Rx(np.radians(60))@Rv
open('/tmp/RG.txt','w').write(" ".join(f"{v:.5f}" for v in R.flatten()))
EOF
python3 motion.py path 2.5 "$(cat /tmp/path3.txt)" --tcp 2>&1 | tail -2; cat /tmp/RG.txt

# openrua op 108
RG="$(cat /tmp/RG.txt)"
python3 motion.py goto -0.0105 0.1013 0.984 $RG 4 --tcp 2>&1 | tail -2 && python3 motion.py grip close | tail -1 && python3 motion.py goto -0.0105 0.1013 1.03 $RG 3 --tcp 2>&1 | tail -1 && python3 motion.py js | tail -1

# openrua op 109
RG="$(cat /tmp/RG.txt)"
python3 motion.py goto -0.0105 0.1013 1.15 $RG 3 --tcp 2>&1 | tail -1 && python3 motion.py js | tail -1 && python3 tools/perception/cam_snap.py frontview snaps/front_lift3.png | tail -1 && python3 cloud.py frontview | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
tcp=np.array([-0.0105,0.1013,1.15])
m=(rgb.min(-1)>110)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.95)&(xyz[...,2]<1.30)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.1)&(xyz[...,1]>tcp[1]+0.015)&(xyz[...,1]<0.45)
p=xyz[m]; print('pts',len(p))
print('z rel tcp: min',(p[:,2].min()-tcp[2]).round(3),'max',(p[:,2].max()-tcp[2]).round(3),' y rel: min',(p[:,1].min()-tcp[1]).round(3),'max',(p[:,1].max()-tcp[1]).round(3))
for zlo in np.arange(1.04,1.30,0.01):
    s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)]
    if len(s)>3: print(f"  z {zlo-tcp[2]:+.3f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} yrel {s[:,1].min()-tcp[1]:+.3f}..{s[:,1].max()-tcp[1]:+.3f}")
EOF

# openrua op 110
RG="$(cat /tmp/RG.txt)"
python3 motion.py goto -0.03 0.10 1.045 $RG 3 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.03 0.20 1.045 $RG 3 --tcp 2>&1 | tail -2 && timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>&1 | grep -A3 force

# openrua op 111
RG="$(cat /tmp/RG.txt)"
python3 motion.py goto -0.03 0.28 1.045 $RG 3 --tcp 2>&1 | tail -2 && python3 motion.py js | tail -1 && timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>&1 | grep -A3 force && python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_ins.png | tail -1 && python3 tools/perception/cam_snap.py agentview snaps/agent_ins.png | tail -1

# openrua op 112
python3 cloud.py frontview | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
tcp=np.array([-0.0194,0.2756,1.0591])
m=(rgb.min(-1)>110)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.90)&(xyz[...,2]<1.12)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.1)&(xyz[...,1]>0.14)&(xyz[...,1]<0.45)
p=xyz[m]; print('pts',len(p))
for zlo in np.arange(0.90,1.12,0.01):
    s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)]
    if len(s)>3: print(f"  z {zlo:.3f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 113
python3 cloud.py frontview | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(rgb.min(-1)>110)&(np.ptp(rgb,-1)<30)&(xyz[...,2]>0.90)&(xyz[...,2]<1.12)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.1)&(xyz[...,1]>0.14)&(xyz[...,1]<0.45)
p=xyz[m]; print('pts',len(p))
for zlo in np.arange(0.90,1.12,0.01):
    s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)]
    if len(s)>3: print(f"  z {zlo:.3f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 114
python3 tools/perception/cam_snap.py frontview snaps/front_ins.png | tail -1

# openrua op 115
python3 tools/perception/cam_snap.py robot0_robotview snaps/rv_ins.png | tail -1

# openrua op 116
RG="$(cat /tmp/RG.txt)"
python3 motion.py goto -0.03 0.10 1.06 $RG 4 --tcp 2>&1 | tail -2 && python3 motion.py js | tail -1 && python3 tools/perception/cam_snap.py frontview snaps/front_retr.png | tail -1

# openrua op 117
RG="$(cat /tmp/RG.txt)"
for z in 1.00 0.97 0.95; do echo "z=$z"; python3 motion.py goto -0.03 0.10 $z $RG 2.5 --tcp 2>&1 | tail -2; done

# openrua op 118
RG="$(cat /tmp/RG.txt)"
python3 motion.py grip open | tail -1 && python3 motion.py goto -0.03 0.12 1.25 $RG 3 --tcp 2>&1 | tail -1 && python3 motion.py goto -0.03 0.16 1.40 0 1 0 1 0 0 0 0 -1 5 --tcp 2>&1 | tail -2 && python3 tools/perception/cam_snap.py frontview snaps/front_laid.png | tail -1

# openrua op 119
python3 motion.py goto -0.03 0.16 1.40 0 1 0 1 0 0 0 0 -1 3 --tcp 2>&1 | tail -2

# openrua op 120
python3 cloud.py robot0_eye_in_hand 2>&1 | tail -1 && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
ok=np.isfinite(xyz).all(-1)&(rgb.min(-1)>100)&(np.ptp(rgb,-1)<40)
box=ok&(xyz[...,0]>-0.2)&(xyz[...,0]<0.15)&(xyz[...,1]>0.0)&(xyz[...,1]<0.245)&(xyz[...,2]>0.905)&(xyz[...,2]<1.1)
p=xyz[box]; print('white pts',len(p),'zmax',p[:,2].max().round(3))
for zlo in np.arange(0.90,1.08,0.01):
    s=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)]
    if len(s)>3: print(f"  z {zlo:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
top=p[p[:,2]>p[:,2].max()-0.012]
print('top pts',len(top),'x',np.percentile(top[:,0],[5,50,95]).round(3),'y',np.percentile(top[:,1],[5,50,95]).round(3),'z',np.percentile(top[:,2],[5,50,95]).round(3))
EOF

# openrua op 121
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py goto -0.027 0.098 1.10 $RV 4 --tcp 2>&1 | tail -1 && python3 motion.py goto -0.027 0.098 1.017 $RV 3 --tcp 2>&1 | tail -2 && python3 motion.py grip close | tail -1 && python3 motion.py goto -0.027 0.098 1.06 $RV 3 --tcp 2>&1 | tail -1 && python3 motion.py js | tail -1

# openrua op 122
P="-0.027,0.098,1.25,0,1,0,1,0,0,0,0,-1;-0.027,0.098,1.25,0,1,0,0.866,0,0.5,0.5,0,-0.866;-0.027,0.098,1.25,0,1,0,0.5,0,0.866,0.866,0,-0.5;-0.027,0.098,1.25,0,1,0,0,0,1,1,0,0"
python3 motion.py path 4 "$P" --tcp --dry 2>&1 | tail -5

# openrua op 123
P="-0.027,0.098,1.25,0,1,0,1,0,0,0,0,-1;-0.027,0.098,1.25,0,1,0,0.866,0,0.5,0.5,0,-0.866;-0.027,0.098,1.25,0,1,0,0.5,0,0.866,0.866,0,-0.5;-0.027,0.098,1.25,0,1,0,0,0,1,1,0,0"
python3 motion.py path 4 "$P" --tcp 2>&1 | tail -2 && python3 motion.py js | tail -1 && python3 tools/perception/cam_snap.py frontview snaps/front_rot.png && python3 cloud.py frontview

# openrua op 124
python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
w=(rgb.min(-1)>170)&(np.abs(rgb[...,0]-rgb[...,2])<25)
m=w&(xyz[...,2]>1.10)&(xyz[...,2]<1.40)&(xyz[...,1]>0.10)&(xyz[...,1]<0.30)&(np.abs(xyz[...,0])<0.12)
P=xyz[m]; print(len(P))
for a,n in zip(range(3),'xyz'): print(n, np.percentile(P[:,a],[1,5,50,95,99]).round(3))
# slices in z
for z0 in np.arange(1.12,1.36,0.03):
    s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.03)]
    if len(s): print(f"z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 125
python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(xyz[...,2]>1.08)&(xyz[...,2]<1.40)&(xyz[...,1]>0.112)&(xyz[...,1]<0.30)&(xyz[...,0]>-0.12)&(xyz[...,0]<0.10)
P=xyz[m]; C=rgb[m]; print(len(P))
for z0 in np.arange(1.10,1.34,0.02):
    s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.02)]; c=C[(P[:,2]>=z0)&(P[:,2]<z0+0.02)]
    if len(s): print(f"z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f} rgb~{c.mean(0).round(0)}")
EOF

# openrua op 126
R90="0 1 0 0 0 1 1 0 0"
python3 motion.py goto -0.027 0.10 1.03 $R90 5 --tcp 2>&1 | tail -2 && python3 motion.py js | tail -1 && python3 cloud.py frontview && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']
m=(xyz[...,2]>0.90)&(xyz[...,2]<1.12)&(xyz[...,1]>0.112)&(xyz[...,1]<0.232)&(xyz[...,0]>-0.12)&(xyz[...,0]<0.10)
P=xyz[m]
for z0 in np.arange(0.92,1.12,0.02):
    s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.02)]
    if len(s): print(f"z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 127
P="-0.027,0.18,1.03,0,1,0,0,0,1,1,0,0;-0.027,0.26,1.03,0,1,0,0,0,1,1,0,0"
python3 motion.py path 4 "$P" --tcp 2>&1 | tail -2 && python3 motion.py js | tail -1 && python3 tools/perception/cam_snap.py frontview snaps/front_ins2.png && python3 tools/perception/cam_snap.py agentview snaps/agent_ins2.png

# openrua op 128
R90="0 1 0 0 0 1 1 0 0"
python3 motion.py goto -0.027 0.26 1.016 $R90 3 --tcp 2>&1 | tail -2 && python3 motion.py grip open | tail -1 && python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(xyz[...,2]>0.94)&(xyz[...,2]<1.10)&(xyz[...,1]>0.24)&(xyz[...,1]<0.41)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.08)
P=xyz[m]; C=rgb[m]; print(len(P))
w=C.min(1)>140; Q=P[w]; print("white", len(Q))
for z0 in np.arange(0.94,1.10,0.02):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
    if len(s): print(f"z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 129
R90="0 1 0 0 0 1 1 0 0"
python3 motion.py goto -0.027 0.10 1.02 $R90 4 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.03 -0.02 1.03 $R90 3 --tcp 2>&1 | tail -1 && python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_final.png && python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(xyz[...,2]>0.94)&(xyz[...,2]<1.10)&(xyz[...,1]>0.20)&(xyz[...,1]<0.43)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.08)&np.isfinite(xyz[...,2])
P=xyz[m]; C=rgb[m]
w=C.min(1)>140; Q=P[w]; print("white", len(Q))
for z0 in np.arange(0.94,1.10,0.02):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
    if len(s): print(f"z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 130
R90="0 1 0 0 0 1 1 0 0"
python3 motion.py goto -0.028 0.20 1.00 $R90 4 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.028 0.33 1.00 $R90 4 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.028 0.10 1.00 $R90 4 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.03 -0.02 1.03 $R90 3 --tcp 2>&1 | tail -2 && python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(xyz[...,2]>0.94)&(xyz[...,2]<1.10)&(xyz[...,1]>0.20)&(xyz[...,1]<0.43)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.08)&np.isfinite(xyz[...,2])
P=xyz[m]; C=rgb[m]
w=C.min(1)>140; Q=P[w]; print("white", len(Q))
for z0 in np.arange(0.94,1.10,0.02):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
    if len(s): print(f"z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 131
R90="0 1 0 0 0 1 1 0 0"
python3 motion.py goto -0.028 0.22 1.00 $R90 4 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.028 0.33 1.00 $R90 7 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.028 0.10 1.00 $R90 4 --tcp 2>&1 | tail -2 && python3 motion.py goto -0.03 -0.02 1.10 $R90 3 --tcp 2>&1 | tail -2 && python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
m=(xyz[...,2]>0.94)&(xyz[...,2]<1.10)&(xyz[...,1]>0.20)&(xyz[...,1]<0.43)&(xyz[...,0]>-0.15)&(xyz[...,0]<0.08)&np.isfinite(xyz[...,2])
P=xyz[m]; C=rgb[m]
w=C.min(1)>140; Q=P[w]; print("white", len(Q))
for z0 in np.arange(0.94,1.10,0.02):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
    if len(s): print(f"z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
EOF

# openrua op 132
P="-0.10,-0.05,1.20,0,1,0,0.707,0,0.707,0.707,0,-0.707;-0.20,0.0,1.20,0,1,0,1,0,0,0,0,-1;-0.26,0.03,1.20,1,0,0,0,-1,0,0,0,-1;-0.26,0.03,1.06,1,0,0,0,-1,0,0,0,-1"
python3 motion.py path 4 "$P" --tcp --dry 2>&1 | tail -5

# openrua op 133
python3 - <<'EOF' 2>/dev/null
import numpy as np
from motion import Robot, TCP
r=Robot()
wps=[r.joints(),[-1.4, 0.532, 1.225, -1.984, 0.299, 1.999, -1.118],[-1.139, -0.464, 1.081, -2.396, 0.478, 2.111, -1.151],[-1.16, -0.943, 1.102, -2.526, 0.847, 1.842, 0.19],[-1.336, -0.806, 1.53, -2.628, 1.101, 2.2, 0.102]]
for i in range(len(wps)-1):
    a,b=np.array(wps[i]),np.array(wps[i+1])
    for s in np.linspace(0,1,6):
        p,R=r.fk(a+(b-a)*s); t=p+TCP*R[:,2]
        print(i, round(s,1), "hand",p.round(3),"tcp",t.round(3))
EOF

# openrua op 134
python3 cloud.py agentview >/dev/null 2>&1; python3 cloud.py birdview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
for cam in ['agentview','birdview']:
    d=np.load(f'snaps/{cam}_cloud.npz'); xyz=d['xyz']
    m=(xyz[...,2]>0.95)&(xyz[...,2]<1.09)&(xyz[...,0]>-0.30)&(xyz[...,0]<-0.12)&(xyz[...,1]>-0.10)&(xyz[...,1]<0.32)&np.isfinite(xyz[...,2])
    P=xyz[m]; print(cam, len(P))
    for y0 in np.arange(-0.10,0.32,0.03):
        s=P[(P[:,1]>=y0)&(P[:,1]<y0+0.03)]
        if len(s): print(f"  y {y0:.2f}: n={len(s)} x {np.percentile(s[:,0],2):.3f}..{np.percentile(s[:,0],98):.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}")
EOF

# openrua op 135
python3 - <<'EOF'
import numpy as np
hx,hy=-0.187,0.235; r=0.13; d=0.047; z=1.06
def wp(beta_deg,z):
    b=np.radians(beta_deg); c,s=np.cos(b),np.sin(b)
    p=np.array([hx+r*c+d*s, hy+r*s-d*c, z])
    R=[-s,c,0, c,s,0, 0,0,-1]
    return ",".join(f"{v:.4f}" for v in list(p)+R)
pre=["-0.10,-0.05,1.20,0,1,0,0.707,0,0.707,0.707,0,-0.707","-0.20,0.0,1.20,0,1,0,1,0,0,0,0,-1",wp(-97,1.20),wp(-97,z)]
arc=[wp(b,z) for b in [-82,-67,-52,-37,-22,-7,2]]
open("/tmp/door_pre.txt","w").write(";".join(pre)); open("/tmp/door_arc.txt","w").write(";".join(arc))
for s in pre+arc: print(s[:30])
EOF
python3 motion.py grip close 2>/dev/null | tail -1
python3 motion.py path 4 "$(cat /tmp/door_pre.txt)" --tcp --dry 2>/dev/null | tail -4

# openrua op 136
python3 - <<'EOF' 2>/dev/null
import numpy as np
from motion import Robot, TCP
r=Robot()
wps=[r.joints(),[-1.4, 0.532, 1.225, -1.984, 0.299, 1.999, -1.118],[-1.139, -0.464, 1.081, -2.396, 0.478, 2.111, -1.151],[-1.09, -0.986, 1.18, -2.446, 0.911, 1.793, 0.47],[-1.237, -0.922, 1.578, -2.543, 1.172, 2.097, 0.381]]
for i in range(len(wps)-1):
    a,b=np.array(wps[i]),np.array(wps[i+1]); zs=[]
    for s in np.linspace(0,1,6):
        p,R=r.fk(a+(b-a)*s); t=p+TCP*R[:,2]; zs.append((round(t[0],3),round(t[1],3),round(t[2],3)))
    print(i, zs)
EOF

# openrua op 137
python3 motion.py path 5 "$(cat /tmp/door_pre.txt)" --tcp 2>/dev/null | tail -2 && python3 motion.py path 3 "$(cat /tmp/door_arc.txt)" --tcp --dry 2>/dev/null | tail -7

# openrua op 138
python3 motion.py path 3 "$(cat /tmp/door_arc.txt)" --tcp 2>/dev/null | tail -2 && python3 tools/perception/cam_snap.py agentview snaps/agent_door.png && python3 tools/perception/cam_snap.py frontview snaps/front_door.png

# openrua op 139
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py goto -0.05 0.0 1.35 $RV 4 --tcp 2>/dev/null | tail -2 && python3 cloud.py agentview >/dev/null 2>&1 && python3 cloud.py frontview >/dev/null 2>&1 && python3 tools/perception/cam_snap.py frontview snaps/front_final.png && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/agentview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
# door region: in front of microwave, x -0.22..0.10, y 0.15..0.25, z 0.95..1.09
m=(xyz[...,2]>0.95)&(xyz[...,2]<1.09)&(xyz[...,0]>-0.22)&(xyz[...,0]<0.10)&(xyz[...,1]>0.10)&(xyz[...,1]<0.26)&np.isfinite(xyz[...,2])
P=xyz[m]; print("door pts",len(P))
for x0 in np.arange(-0.22,0.10,0.04):
    s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.04)]
    if len(s): print(f"  x {x0:.2f}: n={len(s)} y {np.percentile(s[:,1],2):.3f}..{np.percentile(s[:,1],98):.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}")
# any panel still sticking out along old open position?
m2=(xyz[...,2]>0.95)&(xyz[...,2]<1.09)&(xyz[...,0]>-0.30)&(xyz[...,0]<-0.15)&(xyz[...,1]>-0.05)&(xyz[...,1]<0.15)&np.isfinite(xyz[...,2])
print("pts in old open-door volume:", m2.sum())
# white mug points on table / outside microwave
w=(rgb.min(-1)>170)&(np.abs(rgb[...,0]-rgb[...,2])<25)&(xyz[...,2]>0.90)&(xyz[...,2]<1.05)&(xyz[...,1]<0.22)&(xyz[...,0]>-0.3)&(xyz[...,0]<0.4)&np.isfinite(xyz[...,2])
print("white pts outside microwave near table:", w.sum())
EOF

# openrua op 140
python3 - <<'EOF'
import numpy as np
for cam in ['frontview','agentview']:
    d=np.load(f'snaps/{cam}_cloud.npz'); xyz=d['xyz']; rgb=d['rgb'].astype(int)
    m=(xyz[...,2]>0.96)&(xyz[...,2]<1.08)&(xyz[...,0]>-0.19)&(xyz[...,0]<0.0)&(xyz[...,1]>0.17)&(xyz[...,1]<0.27)&np.isfinite(xyz[...,2])
    P=xyz[m]; print(cam,len(P))
    xs=[];ys=[]
    for x0 in np.arange(-0.19,0.0,0.02):
        s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.02)]
        if len(s)>5:
            yb=np.percentile(s[:,1],10); xs.append(x0+0.01); ys.append(yb); print(f"  x {x0+0.01:.3f} front y {yb:.4f} n={len(s)}")
    if len(xs)>2:
        k,b=np.polyfit(xs,ys,1); print("  slope",round(k,3),"deg",round(np.degrees(np.arctan(k)),1))
    # white outside
    w=(rgb.min(-1)>170)&(np.abs(rgb[...,0]-rgb[...,2])<25)&(xyz[...,2]>0.90)&(xyz[...,2]<1.05)&(xyz[...,1]<0.22)&(xyz[...,0]>-0.3)&(xyz[...,0]<0.4)&np.isfinite(xyz[...,2])
    W=xyz[w]
    if len(W): print("  white outside: x",np.percentile(W[:,0],[5,50,95]).round(3),"y",np.percentile(W[:,1],[5,50,95]).round(3))
EOF

# openrua op 141
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py goto -0.05 0.20 1.42 $RV 4 --tcp 2>/dev/null | tail -1 && python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1 && python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_door.png && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']
m=(xyz[...,2]>1.0)&(xyz[...,2]<1.20)&(xyz[...,0]>-0.22)&(xyz[...,0]<0.12)&(xyz[...,1]>0.12)&(xyz[...,1]<0.45)&np.isfinite(xyz[...,2])
P=xyz[m]; print(len(P))
# heights histogram
for z0 in np.arange(1.06,1.13,0.01):
    s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.3f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {np.percentile(s[:,1],2):.3f}..{np.percentile(s[:,1],98):.3f}")
# door top edge: points z 1.08-1.10 (door top) per x bin, min y
D=P[(P[:,2]>1.075)&(P[:,2]<1.10)]
xs=[];ys=[]
for x0 in np.arange(-0.20,0.10,0.03):
    s=D[(D[:,0]>=x0)&(D[:,0]<x0+0.03)]
    if len(s)>5: yb=np.percentile(s[:,1],3); yt=np.percentile(s[:,1],97); xs.append(x0+0.015); ys.append(yb); print(f"  x {x0+0.015:.3f}: front y {yb:.4f} back {yt:.4f} n={len(s)}")
if len(xs)>2:
    k,b=np.polyfit(xs,ys,1); print("slope",round(k,4),"deg",round(np.degrees(np.arctan(k)),2))
EOF

# openrua op 142
python3 - <<'EOF'
import numpy as np
d=np.load('snaps/robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']
m=(xyz[...,2]>1.04)&(xyz[...,2]<1.12)&(xyz[...,0]>-0.20)&(xyz[...,0]<0.10)&(xyz[...,1]>0.12)&(xyz[...,1]<0.30)&np.isfinite(xyz[...,2])
P=xyz[m]
xs=[];ys=[]
for x0 in np.arange(-0.18,0.09,0.02):
    s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.02)&(P[:,2]>1.05)&(P[:,2]<1.105)]
    if len(s)>3:
        yb=np.percentile(s[:,1],3); print(f"x {x0+0.01:.3f}: n={len(s)} y {yb:.4f}..{np.percentile(s[:,1],97):.4f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}")
        if x0<0.03: xs.append(x0+0.01); ys.append(yb)
k,b=np.polyfit(xs,ys,1); print("front-face slope",round(k,4),"deg",round(np.degrees(np.arctan(k)),2), "y at x=-0.17:",round(k*-0.17+b,4),"at x=0.07:",round(k*0.07+b,4))
EOF

# openrua op 143
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py goto -0.25 -0.15 1.30 $RV 4 --tcp 2>/dev/null | tail -1 && python3 cloud.py frontview >/dev/null 2>&1 && python3 tools/perception/cam_snap.py frontview snaps/front_final2.png && python3 - <<'EOF'
import numpy as np
d=np.load('snaps/frontview_cloud.npz'); xyz=d['xyz']
m=(xyz[...,2]>0.95)&(xyz[...,2]<1.09)&(xyz[...,0]>-0.19)&(xyz[...,0]<0.10)&(xyz[...,1]>0.15)&(xyz[...,1]<0.27)&np.isfinite(xyz[...,2])
P=xyz[m]; print(len(P))
xs=[];ys=[]
for x0 in np.arange(-0.19,0.10,0.02):
    s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.02)]
    if len(s)>3:
        yb=np.percentile(s[:,1],5); print(f"x {x0+0.01:.3f}: n={len(s)} y {yb:.4f}..{np.percentile(s[:,1],95):.4f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}")
        if x0<0.03: xs.append(x0+0.01); ys.append(yb)
if len(xs)>2:
    k,b=np.polyfit(xs,ys,1); print("front-face slope",round(k,4),"deg",round(np.degrees(np.arctan(k)),2),"y@-0.17",round(k*-0.17+b,4),"y@0.07",round(k*0.07+b,4))
EOF

# openrua op 144
python3 motion.py js 2>/dev/null | tail -1; python3 motion.py fk 2>/dev/null | tail -4

# openrua op 145
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py path 4 "-0.10,0.05,1.30,$( echo $RV | tr ' ' ',');-0.057,0.16,1.20,$( echo $RV | tr ' ' ',');-0.057,0.205,1.06,$( echo $RV | tr ' ' ',')" --tcp --dry 2>/dev/null | tail -3

# openrua op 146
RV="0 1 0 1 0 0 0 0 -1"; RVc=$(echo $RV | tr ' ' ',')
python3 motion.py path 5 "-0.10,0.05,1.30,$RVc;-0.057,0.16,1.20,$RVc;-0.057,0.205,1.06,$RVc" --tcp 2>/dev/null | tail -2 && python3 motion.py goto -0.057 0.226 1.06 $RV 4 --tcp 2>/dev/null | tail -2

# openrua op 147
RV="0 1 0 1 0 0 0 0 -1"
python3 motion.py goto -0.057 0.10 1.40 $RV 4 --tcp 2>/dev/null | tail -2 && python3 cloud.py frontview >/dev/null 2>&1 && python3 cloud.py agentview >/dev/null 2>&1 && python3 - <<'EOF'
import numpy as np
for cam in ['frontview','agentview']:
    d=np.load(f'snaps/{cam}_cloud.npz'); xyz=d['xyz']
    m=(xyz[...,2]>0.95)&(xyz[...,2]<1.09)&(xyz[...,0]>-0.19)&(xyz[...,0]<0.10)&(xyz[...,1]>0.15)&(xyz[...,1]<0.27)&np.isfinite(xyz[...,2])
    P=xyz[m]; print(cam,len(P))
    xs=[];ys=[]
    for x0 in np.arange(-0.19,0.10,0.02):
        s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.02)]
        if len(s)>3:
            yb=np.percentile(s[:,1],5); print(f"  x {x0+0.01:.3f}: n={len(s)} y {yb:.4f}..{np.percentile(s[:,1],95):.4f}")
            if x0<0.03: xs.append(x0+0.01); ys.append(yb)
    if len(xs)>2:
        k,b=np.polyfit(xs,ys,1); print("  slope",round(k,4),"deg",round(np.degrees(np.arctan(k)),2),"y@-0.17",round(k*-0.17+b,4),"y@0.07",round(k*0.07+b,4))
EOF

# openrua op 148
python3 tools/perception/cam_snap.py agentview snaps/final.png && python3 motion.py js 2>/dev/null | tail -1
