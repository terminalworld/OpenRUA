#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -iE "ik|fk|scene|plan" ; echo ----; ros2 node list

# openrua op 4
cd /workspace; ros2 topic echo /joint_states --once; for c in agentview frontview sideview birdview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c; done

# openrua op 5
cd /workspace; ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|child_frame_id" | head -60; echo ----; timeout 5 ros2 topic echo /tf --once 2>/dev/null | grep -E "frame_id" | head

# openrua op 6
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab depth+color+intrinsics+TF for a camera; save world-frame point cloud.

Usage: python3 cloud.py <camera>   -> <camera>_cloud.npz (xyz Nx3, rgb Nx3, uv Nx2)
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, typ, timeout=20.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
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
    buf = Buffer()
    TransformListener(buf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
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
    p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    vv, uu = np.mgrid[0:h, 0:w]
    z = depth
    ok = np.isfinite(z) & (z > 0)
    X = (uu - cx) * z / fx
    Y = (vv - cy) * z / fy
    pc = np.stack([X[ok], Y[ok], z[ok]], axis=1)
    xyz = pc @ R.T + p
    np.savez(f"{cam}_cloud.npz", xyz=xyz, rgb=color[ok], uv=np.stack([uu[ok], vv[ok]], 1),
             cam_pos=p, cam_R=R, k=np.array(info.k))
    print(cam, "cam_pos", p, "n", len(xyz))
    print("bbox", xyz.min(0), xyz.max(0))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
cd /workspace; for c in birdview agentview sideview frontview; do python3 cloud.py $c; done

# openrua op 8
cd /workspace; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>/dev/null | head -8; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int); uv=d["uv"]
# table height: mode of z in region
zs=xyz[:,2]
hist,edges=np.histogram(zs,bins=200,range=(0,1.5))
i=hist.argmax(); print("table z mode ~", edges[i], edges[i+1])
# objects above table
tab=edges[i+1]
m=(zs>tab+0.01)&(zs<1.0)&(xyz[:,0]>-1.0)&(xyz[:,0]<0.3)
pts=xyz[m]; col=rgb[m]
print("above-table pts", len(pts))
# grid summary
import collections
g=collections.defaultdict(list)
for p,c in zip(pts,col):
    g[(round(p[0],1),round(p[1],1))].append((p[2],c))
for k in sorted(g):
    arr=g[k]; z=[a[0] for a in arr]; c=np.mean([a[1] for a in arr],0)
    print(k, len(arr), "zmax %.3f zmean %.3f"%(max(z),np.mean(z)), "rgb", c.astype(int))
EOF

# openrua op 9
cd /workspace; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int); uv=d["uv"]
zs=xyz[:,2]
m=(zs>0.89)&(zs<1.25)&(xyz[:,0]>-0.6)&(xyz[:,0]<0.15)&(xyz[:,1]>-0.5)&(xyz[:,1]<0.3)
pts=xyz[m]; col=rgb[m]; puv=uv[m]
print(len(pts))
# 2cm grid height map
xs=np.arange(-0.6,0.15,0.02); ys=np.arange(-0.5,0.3,0.02)
H=np.full((len(xs),len(ys)),np.nan)
for p in pts:
    i=int((p[0]+0.6)/0.02); j=int((p[1]+0.5)/0.02)
    if 0<=i<len(xs) and 0<=j<len(ys):
        H[i,j]=np.nanmax([H[i,j],p[2]])
np.set_printoptions(linewidth=250)
print("cols y from -0.5 to 0.3 step 0.02; rows x from -0.6")
print("      "+"".join("%4d"%(int(y*100)) for y in ys))
for i,x in enumerate(xs):
    row="".join("   ." if np.isnan(H[i,j]) else "%4d"%int((H[i,j]-0.88)*100) for j in range(len(ys)))
    print("%5.2f "%x+row)
EOF

# openrua op 10
cd /workspace; python3 - <<'EOF'
import numpy as np
allp=[]
for c in ["birdview","agentview","sideview","frontview"]:
    d=np.load(f"{c}_cloud.npz"); allp.append(d["xyz"])
xyz=np.concatenate(allp)
zs=xyz[:,2]
m=(zs>0.885)&(zs<1.2)&(xyz[:,0]>-0.52)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.40)&(xyz[:,1]<-0.02)
pts=xyz[m]
res=0.01
xs=np.arange(-0.52,-0.30,res); ys=np.arange(-0.40,-0.02,res)
H=np.full((len(xs),len(ys)),np.nan); N=np.zeros_like(H)
for p in pts:
    i=int((p[0]+0.52)/res); j=int((p[1]+0.40)/res)
    if 0<=i<len(xs) and 0<=j<len(ys):
        H[i,j]=np.nanmax([H[i,j],p[2]]); N[i,j]+=1
print("      "+"".join("%3d"%(int(round(y*100))) for y in ys))
for i,x in enumerate(xs):
    row="".join("  ." if np.isnan(H[i,j]) else "%3d"%int(round((H[i,j]-0.88)*100)) for j in range(len(ys)))
    print("%5.2f "%x+row)
EOF

# openrua op 11
cd /workspace; python3 - <<'EOF'
import numpy as np, cv2
allp=[];allc=[]
for c in ["birdview","agentview","sideview","frontview"]:
    d=np.load(f"{c}_cloud.npz"); allp.append(d["xyz"]); allc.append(d["rgb"])
xyz=np.concatenate(allp); rgb=np.concatenate(allc)
x0,x1,y0,y1=-0.55,-0.25,-0.45,0.05
res=0.0025
m=(xyz[:,2]>0.87)&(xyz[:,2]<1.2)&(xyz[:,0]>x0)&(xyz[:,0]<x1)&(xyz[:,1]>y0)&(xyz[:,1]<y1)
P=xyz[m];C=rgb[m]
W=int((y1-y0)/res);Hh=int((x1-x0)/res)
img=np.zeros((Hh,W,3),np.uint8); Z=np.full((Hh,W),-1.0)
hm=np.zeros((Hh,W),np.uint8)
order=np.argsort(P[:,2])
for p,c in zip(P[order],C[order]):
    i=int((p[0]-x0)/res); j=int((p[1]-y0)/res)
    if 0<=i<Hh and 0<=j<W:
        img[i,j]=c[::-1]; Z[i,j]=p[2]; hm[i,j]=int(min(255,max(0,(p[2]-0.88)*1000)))
img=cv2.resize(img,(W*4,Hh*4),interpolation=cv2.INTER_NEAREST)
hmc=cv2.applyColorMap(cv2.resize(hm,(W*4,Hh*4),interpolation=cv2.INTER_NEAREST),cv2.COLORMAP_JET)
# gridlines every 5cm
for k in range(0,W*4,int(0.05/res)*4):
    cv2.line(img,(k,0),(k,Hh*4-1),(255,255,255),1); cv2.line(hmc,(k,0),(k,Hh*4-1),(255,255,255),1)
for k in range(0,Hh*4,int(0.05/res)*4):
    cv2.line(img,(0,k),(W*4-1,k),(255,255,255),1); cv2.line(hmc,(0,k),(W*4-1,k),(255,255,255),1)
cv2.imwrite("top_color.png",img); cv2.imwrite("top_height.png",hmc)
print(img.shape, "rows: x from",x0,"(top) to",x1,"; cols: y from",y0,"(left) to",y1)
EOF

# openrua op 12
mkdir -p "$(dirname /workspace/fk.py)"
cat > /workspace/fk.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Print current hand pose (panda_hand in panda_link0 and world) via /compute_fk."""
import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE = np.array([-0.75, 0.0, 0.912])  # world -> panda_link0 (tf2_echo)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("fk_probe")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cur = dict(zip(js["m"].name, js["m"].position))
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(10)
    req = GetPositionFK.Request()
    req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [cur[j] for j in ARM]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    res = fut.result()
    p = res.pose_stamped[0].pose
    pos = np.array([p.position.x, p.position.y, p.position.z])
    q = p.orientation
    R = quat_R(q.x, q.y, q.z, q.w)
    tcp = pos + 0.1034 * R[:, 2]
    print("joints:", [round(cur[j], 4) for j in ARM])
    print("fingers:", cur.get("panda_finger_joint1"), cur.get("panda_finger_joint2"))
    print("hand in base :", pos.round(4), "quat xyzw:", [round(v, 4) for v in (q.x, q.y, q.z, q.w)])
    print("hand in world:", (pos + BASE).round(4))
    print("tcp  in world:", (tcp + BASE).round(4))
    print("hand z-axis (approach) in world:", R[:, 2].round(3), " x-axis:", R[:, 0].round(3))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 13
cd /workspace; python3 fk.py

# openrua op 14
cd /workspace; python3 - <<'EOF'
import rclpy, numpy as np
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
ARM=[f"panda_joint{i}" for i in range(1,8)]
rclpy.init(); node=rclpy.create_node("probe")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
cur=dict(zip(js["m"].name,js["m"].position))
fk=node.create_client(GetPositionFK,"/compute_fk"); fk.wait_for_service(10)
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand","panda_link0"]
req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=[cur[j] for j in ARM]
f=fk.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=30)
for ps in f.result().pose_stamped:
    print("FK frame:",ps.header.frame_id, ps.pose.position)
ik=node.create_client(GetPositionIK,"/compute_ik"); ik.wait_for_service(10)
hand=f.result().pose_stamped[0].pose
for fid in ["", "world", "panda_link0"]:
    r=GetPositionIK.Request(); r.ik_request.group_name="panda_arm"
    r.ik_request.pose_stamped.header.frame_id=fid
    r.ik_request.pose_stamped.pose=hand
    r.ik_request.robot_state.joint_state.name=ARM
    r.ik_request.robot_state.joint_state.position=[cur[j] for j in ARM]
    r.ik_request.timeout.sec=2
    fu=ik.call_async(r); rclpy.spin_until_future_complete(node,fu,timeout_sec=60)
    res=fu.result()
    sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
    print("IK frame_id=%r code=%d"%(fid,res.error_code.val), [round(sol.get(j,float('nan')),3) for j in ARM])
rclpy.shutdown()
EOF

# openrua op 15
cd /workspace; python3 tools/action/fjt_send.py 0.9,-0.161,0,-2.4446,0,2.2268,0.7854 3 && python3 fk.py

# openrua op 16
cd /workspace; sed -i 's/print("hand in world:", (pos + BASE).round(4))/print("(FK is already in world frame)")/; s/print("tcp  in world:", (tcp + BASE).round(4))/print("tcp  in world:", tcp.round(4))/; s/print("hand in base :", pos.round(4)/print("hand in world:", pos.round(4)/' fk.py; python3 tools/action/fjt_send.py 0.9,-0.161,0,-2.4446,0,2.2268,0.7854 3 && python3 fk.py

# openrua op 17
cd /workspace; for c in birdview agentview; do python3 tools/perception/cam_snap.py $c; python3 cloud.py $c; done

# openrua op 18
cd /workspace; python3 - <<'EOF'
import numpy as np, cv2
allp=[];allc=[]
for c in ["birdview","agentview"]:
    d=np.load(f"{c}_cloud.npz"); allp.append(d["xyz"]); allc.append(d["rgb"])
xyz=np.concatenate(allp); rgb=np.concatenate(allc)
def hm(x0,x1,y0,y1,res,zlo=0.885,zhi=1.2):
    m=(xyz[:,2]>zlo)&(xyz[:,2]<zhi)&(xyz[:,0]>x0)&(xyz[:,0]<x1)&(xyz[:,1]>y0)&(xyz[:,1]<y1)
    P=xyz[m]
    xs=np.arange(x0,x1,res); ys=np.arange(y0,y1,res)
    H=np.full((len(xs),len(ys)),np.nan)
    for p in P:
        i=int((p[0]-x0)/res); j=int((p[1]-y0)/res)
        if 0<=i<len(xs) and 0<=j<len(ys): H[i,j]=np.nanmax([H[i,j],p[2]])
    print("      "+"".join("%3d"%(int(round(y*100))) for y in ys))
    for i,x in enumerate(xs):
        print("%5.2f "%x+"".join("  ." if np.isnan(H[i,j]) else "%3d"%int(round((H[i,j]-0.88)*100)) for j in range(len(ys))))
print("CADDY"); hm(-0.50,-0.31,-0.38,0.12,0.01)
print("CUP"); hm(-0.20,-0.02,-0.08,0.10,0.005)
EOF

# openrua op 19
cd /workspace; python3 - <<'EOF'
import numpy as np
for c in ["birdview","agentview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"]
    print(c, "fx", d["k"][0])
    m=(xyz[:,2]>0.97)&(xyz[:,0]>-0.2)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.1)&(xyz[:,1]<0.12)
    P=xyz[m]
    print(" rim pts",len(P),"x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"z",P[:,2].min().round(3),P[:,2].max().round(3))
    # fit circle to rim xy
    x,y=P[:,0],P[:,1]
    A=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
    cx,cy,c0=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c0+cx**2+cy**2)
    print(" circle fit center (%.4f,%.4f) r=%.4f"%(cx,cy,r))
    # handle
    m2=(xyz[:,2]>0.93)&(xyz[:,2]<0.975)&(xyz[:,0]>-0.2)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.1)&(xyz[:,1]<-0.02)
    P2=xyz[m2]
    if len(P2): print(" handle pts",len(P2),"x",P2[:,0].min().round(3),P2[:,0].max().round(3),"y",P2[:,1].min().round(3),P2[:,1].max().round(3),"z",P2[:,2].min().round(3),P2[:,2].max().round(3))
    # cup interior floor
    m3=(xyz[:,2]<0.92)&(xyz[:,2]>0.885)&(xyz[:,0]>-0.15)&(xyz[:,0]<-0.05)&(xyz[:,1]>-0.02)&(xyz[:,1]<0.08)
    P3=xyz[m3]
    if len(P3): print(" interior pts",len(P3),"x",P3[:,0].min().round(3),P3[:,0].max().round(3),"y",P3[:,1].min().round(3),P3[:,1].max().round(3),"z",P3[:,2].min().round(3),P3[:,2].max().round(3))
EOF

# openrua op 20
cd /workspace; python3 - <<'EOF'
import numpy as np
for c in ["birdview","agentview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"]
    m=(xyz[:,2]>0.975)&(xyz[:,2]<1.0)&(xyz[:,0]>-0.17)&(xyz[:,0]<-0.02)&(xyz[:,1]>-0.03)&(xyz[:,1]<0.1)
    P=xyz[m]; C=rgb[m]
    print(c," rim pts",len(P),"x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"z",P[:,2].min().round(3),P[:,2].max().round(3), "mean rgb",C.mean(0).astype(int))
    x,y=P[:,0],P[:,1]
    A=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
    cx,cy,c0=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c0+cx**2+cy**2)
    print(" circle fit center (%.4f,%.4f) r=%.4f"%(cx,cy,r))
    # the book
    m=(xyz[:,2]>0.95)&(xyz[:,0]>-0.3)&(xyz[:,0]<-0.1)&(xyz[:,1]>0.08)&(xyz[:,1]<0.2)
    P=xyz[m]; print(" book x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"zmax",P[:,2].max().round(3))
EOF

# openrua op 21
find / -iname "*caddy*" 2>/dev/null | head; find / -iname "*mug*" 2>/dev/null | grep -v proc | head; find / -iname "*libero*" -maxdepth 4 2>/dev/null | head

# openrua op 22
cd /workspace; ls .ros/log | head; grep -il "mug\|caddy\|mujoco\|libero" .ros/log/* 2>/dev/null | head; ros2 param list /robot_bridge 2>/dev/null | head -30; ros2 node info /robot_bridge 2>/dev/null | head -60

# openrua op 23
mkdir -p "$(dirname /workspace/kin.py)"
cat > /workspace/kin.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Shared kinematics helpers: one node, reusable FK/IK/joint-state/trajectory/gripper clients."""
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP_OFF = 0.1034


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self, name="kin"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.fk_cli.wait_for_service(10); self.ik_cli.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, m):
        self._js["m"] = m

    def _on_wr(self, m):
        self._wr["m"] = m

    def spin(self, t=0.3):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self, fresh=True):
        if fresh:
            self._js.clear()
            while "m" not in self._js:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[a] for a in ARM]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr.clear()
        end = time.time() + 3
        while "m" not in self._wr and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if "m" not in self._wr:
            return None
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def fk(self, q, link="panda_hand"):
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp(self, q=None):
        q = self.arm_q() if q is None else q
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        return pos + TCP_OFF * R[:, 2], quat, R

    def ik(self, pos, quat, seed=None, at_tcp=True, timeout=2.0):
        """pos/quat: world-frame pose of TCP (at_tcp) or hand. Returns joint list or None."""
        pos = np.asarray(pos, float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP_OFF * R[:, 2]
        seed = self.arm_q() if seed is None else seed
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = "panda_arm"
        r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = map(float, pos)
        r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y, r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = map(float, quat)
        r.robot_state.joint_state.name = ARM
        r.robot_state.joint_state.position = list(map(float, seed))
        r.timeout.sec = int(timeout); r.timeout.nanosec = int((timeout % 1) * 1e9)
        r.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[a] for a in ARM]

    def move(self, q, seconds=3.0, retries=2, via=None):
        """Send trajectory (optionally through via points: list of (q, t)). Returns error code."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
            for vq, vt in via:
                p = JointTrajectoryPoint(positions=list(map(float, vq)))
                p.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
                pts.append(p)
        p = JointTrajectoryPoint(positions=list(map(float, q)))
        p.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(p)
        goal.trajectory.points = pts
        code = None
        for attempt in range(retries + 1):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"  move attempt {attempt}: code={code} max joint err={err:.4f}")
            if code == 0 or err < 0.01:
                break
        return code

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def report(self, tag=""):
        pos, quat, R = self.tcp()
        f = self.fingers()
        print(f"[{tag}] tcp={pos.round(4)} quat={quat.round(4)} zaxis={R[:,2].round(3)} xaxis={R[:,0].round(3)} fingers={f[0]:.4f},{f[1]:.4f}")
        return pos
OPENRUA_EOF

# openrua op 24
cd /workspace; python3 - <<'EOF'
import numpy as np
from kin import Robot, quat_R
r=Robot("probe")
q=r.arm_q(); print("q",np.round(q,3))
pos=r.report("now")
# verify the earlier suspicious IK solution
sol=[0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009]
p,qq=r.fk(sol); print("FK of earlier IK sol:",p.round(4),qq.round(4))
# test IK for a handle-grasp pose: fingers close along world x
qd=np.array([0.7071,0.7071,0,0])
for target in [(-0.0975,-0.04,1.10),(-0.0975,-0.04,0.962),(-0.435,-0.135,1.15)]:
    s=r.ik(target,qd,seed=q)
    if s is None: print("IK fail",target); continue
    tp,tq,R=r.tcp(s)
    print("target",target,"-> q",np.round(s,3),"tcp",tp.round(4),"z",R[:,2].round(3),"x",R[:,0].round(3))
EOF

# openrua op 25
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/kin.py", "old_string": "        pos = np.asarray(pos, float)\\n        if at_tcp:\\n            R = quat_R(*quat)\\n            pos = pos - TCP_OFF * R[:, 2]\\n        seed = self.arm_q() if seed is None else seed", "new_string": "        pos = np.asarray(pos, float)\\n        if at_tcp:\\n            R = quat_R(*quat)\\n            pos = pos - TCP_OFF * R[:, 2]\\n        # machine quirk: /compute_ik solves for panda_link8 (hand yawed -45deg\\n        # about its z); pre-rotate the request by +45deg about local z so the\\n        # HAND ends up at the requested orientation (verified via /compute_fk)\\n        quat = quat_mul(quat, np.array([0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8)]))\\n        seed = self.arm_q() if seed is None else seed", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/kin.py", "old_string": "class Robot:", "new_string": "def quat_mul(q1, q2):\\n    \\"\\"\\"Hamilton product, xyzw convention: q1 then q2 (q2 in q1\'s local frame).\\"\\"\\"\\n    x1, y1, z1, w1 = q1\\n    x2, y2, z2, w2 = q2\\n    return np.array([\\n        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,\\n        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,\\n        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,\\n        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,\\n    ])\\n\\n\\nclass Robot:", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
cd /workspace; python3 - <<'EOF'
import numpy as np
from kin import Robot, quat_R
r=Robot("probe")
q=r.arm_q()
qd=np.array([0.7071,0.7071,0,0])
for target in [(-0.0975,-0.04,1.10),(-0.0975,-0.04,0.962),(-0.435,-0.135,1.15)]:
    s=r.ik(target,qd,seed=q)
    if s is None: print("IK fail",target); continue
    tp,tq,R=r.tcp(s)
    print("target",target,"-> q",np.round(s,3),"tcp",tp.round(4),"quat",tq.round(3),"z",R[:,2].round(3),"x",R[:,0].round(3))
EOF

# openrua op 28
cd /workspace; python3 -c "
import cv2
im=cv2.imread('agentview.png'); c=im[290:400,260:400]; cv2.imwrite('cup_zoom.png',cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('birdview.png'); c=im[245:300,295:360]; cv2.imwrite('cup_zoom_bird.png',cv2.resize(c,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"

# openrua op 29
mkdir -p "$(dirname /workspace/pick1.py)"
cat > /workspace/pick1.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Step 1: open gripper, move above the mug handle, descend to grasp height (no closing yet)."""
import sys
import numpy as np
from kin import Robot

QD = np.array([0.7071, 0.7071, 0.0, 0.0])  # hand z down, fingers close along world x
HX, HY = -0.098, -0.042                     # handle top-arm centre (world)
Z_PRE, Z_GRASP = 1.10, float(sys.argv[1]) if len(sys.argv) > 1 else 0.960

r = Robot("pick1")
r.report("start")
r.gripper(0.04)

q = r.arm_q()
pre = r.ik((HX, HY, Z_PRE), QD, seed=q)
assert pre is not None, "IK pre-grasp failed"
print("pre-grasp q", np.round(pre, 3))
r.move(pre, 4.0)
r.report("pre-grasp")

# straight-line descent through IK waypoints, 2 cm apart
zs = np.arange(Z_PRE - 0.02, Z_GRASP - 1e-6, -0.02)
zs = np.append(zs, Z_GRASP)
seed = pre
via = []
for i, z in enumerate(zs):
    s = r.ik((HX, HY, z), QD, seed=seed)
    assert s is not None, f"IK failed at z={z}"
    seed = s
    via.append((s, 0.6 * (i + 1)))
last_q, last_t = via.pop()
r.move(last_q, last_t, via=via)
r.report("at grasp height")
w = r.wrench()
print("wrench", None if w is None else w.round(2))
OPENRUA_EOF

# openrua op 30
cd /workspace; python3 -u pick1.py 0.960 2>&1 | tail -20

# openrua op 31
cd /workspace; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png; python3 tools/perception/cam_snap.py agentview av1.png; python3 -c "
import cv2; im=cv2.imread('av1.png'); cv2.imwrite('av1_zoom.png', cv2.resize(im[250:420,220:420],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 32
cd /workspace; python3 cloud.py agentview >/dev/null; python3 - <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"]
# fingers: around handle, x offsets +-0.02..0.06, y in [-0.07,-0.02]
for side,(x0,x1) in {"-x finger":(-0.16,-0.115),"+x finger":(-0.085,-0.04)}.items():
    m=(xyz[:,0]>x0)&(xyz[:,0]<x1)&(xyz[:,1]>-0.075)&(xyz[:,1]<-0.02)&(xyz[:,2]>0.9)&(xyz[:,2]<1.05)
    P=xyz[m]; C=rgb[m]
    if len(P):
        gray=(np.abs(C[:,0].astype(int)-C[:,2].astype(int))<15)&(C[:,0]>60)
        Pg=P[gray]
        print(side,len(P),"zmin all %.3f"%P[:,2].min(), "gray pts",len(Pg), "gray zmin %.3f"%(Pg[:,2].min() if len(Pg) else -1), "gray x range", Pg[:,0].min().round(3) if len(Pg) else None, Pg[:,0].max().round(3) if len(Pg) else None)
# handle now
m=(xyz[:,0]>-0.115)&(xyz[:,0]<-0.085)&(xyz[:,1]>-0.07)&(xyz[:,1]<-0.02)&(xyz[:,2]>0.9)&(xyz[:,2]<1.0)
P=xyz[m]; C=rgb[m]; yellow=(C[:,0].astype(int)-C[:,2].astype(int)>40)
print("handle yellow pts",yellow.sum(),"z range",P[yellow][:,2].min().round(3) if yellow.sum() else None,P[yellow][:,2].max().round(3) if yellow.sum() else None)
EOF

# openrua op 33
cd /workspace; python3 - <<'EOF'
from kin import Robot
r=Robot("grasp")
r.gripper(0.0)
r.report("closed")
print("wrench", r.wrench().round(2))
EOF

# openrua op 34
cd /workspace; python3 - <<'EOF'
import numpy as np
from kin import Robot
QD=np.array([0.7071,0.7071,0,0])
r=Robot("lift")
q=r.arm_q()
s=r.ik((-0.0985,-0.0422,1.20),QD,seed=q); assert s
r.move(s,3.0)
r.report("lifted")
print("wrench", r.wrench().round(2))
EOF
python3 tools/perception/cam_snap.py agentview av2.png

# openrua op 35
cd /workspace; python3 cloud.py agentview >/dev/null; python3 cloud.py birdview > /dev/null; python3 - <<'EOF'
import numpy as np
for c in ["agentview","birdview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"]
    # cup rim now: expected around z ~ 1.20-0.96+0.99 = 1.23
    m=(xyz[:,2]>1.20)&(xyz[:,2]<1.26)&(xyz[:,0]>-0.2)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.03)&(xyz[:,1]<0.1)
    P=xyz[m]; C=rgb[m]
    keep=(C[:,0]>90)  # exclude dark
    P=P[keep]
    print(c,"rim pts",len(P),"x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"z",P[:,2].min().round(3),P[:,2].max().round(3))
    x,y=P[:,0],P[:,1]; A=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
    cx,cy,c0=np.linalg.lstsq(A,b,rcond=None)[0]; rr=np.sqrt(c0+cx**2+cy**2)
    print(" circle center (%.4f,%.4f) r=%.4f"%(cx,cy,rr))
    # cup bottom z: lowest cup points
    m=(xyz[:,2]>1.0)&(xyz[:,2]<1.2)&(xyz[:,0]>-0.16)&(xyz[:,0]<-0.03)&(xyz[:,1]>-0.03)&(xyz[:,1]<0.09)
    P=xyz[m]; C=rgb[m]; P=P[C[:,0]>90]
    if len(P): print(" cup body zmin %.3f"%P[:,2].min())
EOF

# openrua op 36
mkdir -p "$(dirname /workspace/place1.py)"
cat > /workspace/place1.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Step 2: carry the cup over slot B, lower until contact, release, retreat."""
import sys
import numpy as np
from kin import Robot

QD = np.array([0.7071, 0.7071, 0.0, 0.0])
CUP_OFF = np.array([0.0, 0.069])          # cup body centre minus TCP (xy, world)
TX, TY = float(sys.argv[1]), float(sys.argv[2])   # desired cup-centre xy
Z_HI, Z_START, Z_MIN = 1.20, 1.16, 1.07
tcp_xy = np.array([TX, TY]) - CUP_OFF

r = Robot("place1")
r.report("start")
base_w = r.wrench()
print("baseline wrench", base_w.round(2))

q = r.arm_q()
s = r.ik((tcp_xy[0], tcp_xy[1], Z_HI), QD, seed=q); assert s
r.move(s, 6.0)
r.report("over slot")

seed = s
z = Z_START
while z >= Z_MIN - 1e-6:
    s = r.ik((tcp_xy[0], tcp_xy[1], z), QD, seed=seed); assert s, f"IK fail z={z}"
    seed = s
    r.move(s, 1.0, retries=1)
    w = r.wrench()
    dz = w[2] - base_w[2]
    p = r.report(f"z={z:.3f}")
    print(f"   Fz={w[2]:.2f} dFz={dz:.2f} torque={w[3:].round(2)}")
    if abs(dz) > 1.5 or abs(w[3]) > 0.3 or abs(w[4]) > 0.3:
        print("   contact detected")
        break
    z -= 0.01
OPENRUA_EOF

# openrua op 37
cd /workspace; python3 -u place1.py -0.435 -0.135 2>&1 | grep -v "^  move attempt"

# openrua op 38
cd /workspace; python3 tools/perception/cam_snap.py agentview av3.png; python3 tools/perception/cam_snap.py sideview sv3.png; python3 -c "
import cv2; im=cv2.imread('av3.png'); cv2.imwrite('av3_zoom.png', cv2.resize(im[120:300,180:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 39
cd /workspace; python3 -u place1.py -0.44 -0.105 2>&1 | grep -v "^  move attempt" | tail -8

# openrua op 40
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "^  move attempt"
import numpy as np
from kin import Robot
QD=np.array([0.7071,0.7071,0,0]); tx,ty=-0.44,-0.174
r=Robot("lower"); seed=r.arm_q()
for z in np.arange(1.15,1.09,-0.01):
    s=r.ik((tx,ty,z),QD,seed=seed); assert s; seed=s
    r.move(s,1.0,retries=1)
    w=r.wrench(); p=r.report(f"z={z:.3f}")
    print(f"   Fz={w[2]:.2f} torque={w[3:].round(2)}")
    if abs(w[2]+5.4)>1.5 or abs(w[3])>0.3 or abs(w[4])>0.3:
        print("   contact"); break
EOF

# openrua op 41
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "^  move attempt"
import numpy as np
from kin import Robot
QD=np.array([0.7071,0.7071,0,0])
r=Robot("release")
r.gripper(0.04)
seed=r.arm_q()
s=r.ik((-0.44,-0.174,1.25),QD,seed=seed); assert s
r.move(s,3.0)
r.report("retreated")
EOF
python3 tools/perception/cam_snap.py agentview av4.png; python3 tools/perception/cam_snap.py sideview sv4.png

# openrua op 42
cd /workspace; python3 cloud.py agentview >/dev/null; python3 cloud.py birdview >/dev/null; python3 - <<'EOF'
import numpy as np
allp=[];allc=[]
for c in ["birdview","agentview"]:
    d=np.load(f"{c}_cloud.npz"); allp.append(d["xyz"]); allc.append(d["rgb"])
xyz=np.concatenate(allp); rgb=np.concatenate(allc).astype(int)
m=(xyz[:,2]>0.9)&(xyz[:,2]<1.2)&(xyz[:,0]>-0.5)&(xyz[:,0]<-0.3)&(xyz[:,1]>-0.25)&(xyz[:,1]<0.0)
P=xyz[m];C=rgb[m]
cup=(C[:,0]>90)&(C[:,1]>80)   # light cup pixels (white/yellow) vs dark caddy
Pc=P[cup]
print("cup pts",len(Pc),"x",Pc[:,0].min().round(3),Pc[:,0].max().round(3),"y",Pc[:,1].min().round(3),Pc[:,1].max().round(3),"z",Pc[:,2].min().round(3),Pc[:,2].max().round(3))
print("cup centroid",Pc.mean(0).round(3))
res=0.01
xs=np.arange(-0.5,-0.3,res); ys=np.arange(-0.25,0.0,res)
H=np.full((len(xs),len(ys)),np.nan)
for p in Pc:
    i=int((p[0]+0.5)/res); j=int((p[1]+0.25)/res)
    H[i,j]=np.nanmax([H[i,j],p[2]])
print("      "+"".join("%3d"%int(round(y*100)) for y in ys))
for i,x in enumerate(xs):
    print("%5.2f "%x+"".join("  ." if np.isnan(H[i,j]) else "%3d"%int(round((H[i,j]-0.88)*100)) for j in range(len(ys))))
EOF

# openrua op 43
cd /workspace; python3 -c "
import cv2; im=cv2.imread('av4.png'); cv2.imwrite('av4_zoom.png', cv2.resize(im[140:280,120:380],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sv4.png'); cv2.imwrite('sv4_zoom.png', cv2.resize(im[180:340,280:520],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 44
cd /workspace; python3 - <<'EOF'
import numpy as np
allp=[];allc=[]
for c in ["birdview","agentview"]:
    d=np.load(f"{c}_cloud.npz"); allp.append(d["xyz"]); allc.append(d["rgb"])
xyz=np.concatenate(allp); rgb=np.concatenate(allc).astype(int)
m=(xyz[:,2]>0.95)&(xyz[:,2]<1.1)&(xyz[:,0]>-0.45)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.26)&(xyz[:,1]<-0.185)
P=xyz[m];C=rgb[m]
yel=(C[:,0]-C[:,2]>40)&(C[:,0]>100)
Py=P[yel]
print("handle yellow pts",len(Py))
if len(Py):
    print(" x",Py[:,0].min().round(3),Py[:,0].max().round(3)," y",Py[:,1].min().round(3),Py[:,1].max().round(3)," z",Py[:,2].min().round(3),Py[:,2].max().round(3))
    # per-x column summary
    for x0 in np.arange(-0.43,-0.33,0.01):
        s=Py[(Py[:,0]>=x0)&(Py[:,0]<x0+0.01)]
        if len(s): print("  x=%.2f n=%3d y[%.3f,%.3f] z[%.3f,%.3f]"%(x0,len(s),s[:,1].min(),s[:,1].max(),s[:,2].min(),s[:,2].max()))
# cup body extents (light pts)
m=(xyz[:,2]>0.95)&(xyz[:,0]>-0.45)&(xyz[:,0]<-0.28)&(xyz[:,1]>-0.20)&(xyz[:,1]<-0.06)
P=xyz[m];C=rgb[m]; light=(C[:,0]>90)&(C[:,1]>85)
Pb=P[light]
print("body x",Pb[:,0].min().round(3),Pb[:,0].max().round(3),"y",Pb[:,1].min().round(3),Pb[:,1].max().round(3),"z",Pb[:,2].min().round(3),Pb[:,2].max().round(3))
for x0 in np.arange(-0.43,-0.29,0.01):
    s=Pb[(Pb[:,0]>=x0)&(Pb[:,0]<x0+0.01)]
    if len(s): print("  x=%.2f n=%3d y[%.3f,%.3f] zmax %.3f"%(x0,len(s),s[:,1].min(),s[:,1].max(),s[:,2].max()))
EOF

# openrua op 45
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "^  move attempt"
import numpy as np
from kin import Robot
QD=np.array([0.7071,0.7071,0,0]); gx,gy=-0.378,-0.232
r=Robot("regrasp"); r.gripper(0.04)
seed=r.arm_q()
s=r.ik((gx,gy,1.15),QD,seed=seed); assert s; r.move(s,4.0); r.report("above handle")
seed=s; via=[]
zs=list(np.arange(1.13,1.012,-0.02))+[1.012]
for i,z in enumerate(zs):
    s=r.ik((gx,gy,z),QD,seed=seed); assert s; seed=s; via.append((s,0.6*(i+1)))
lq,lt=via.pop(); r.move(lq,lt,via=via)
r.report("at handle"); print("wrench",r.wrench().round(2))
EOF
python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png

# openrua op 46
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "^  move attempt"
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot, quat_R
R=np.array([[0,0,-1],[1,0,0],[0,-1,0]],float)   # cols: x=(0,1,0) y=(0,0,-1) z=(-1,0,0)
q=Rot.from_matrix(R).as_quat(); print("quat",q.round(4), "check", quat_R(*q).round(2).tolist())
r=Robot("rimtest")
seed=r.arm_q()
# retreat up first
QD=np.array([0.7071,0.7071,0,0])
s=r.ik((-0.378,-0.232,1.20),QD,seed=seed); assert s; r.move(s,3.0)
seed=s
for x in [-0.20]:
    s=r.ik((x,-0.148,1.064),q,seed=seed)
    print("IK at x",x, None if s is None else np.round(s,3))
    if s: seed=s; r.move(s,5.0); r.report("front of rim")
EOF

# openrua op 47
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "^  move attempt"
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
r=Robot("iktest")
cur=r.arm_q()
seeds=[cur,[0,-0.16,0,-2.44,0,2.23,0.78],[0,-0.5,0,-2.6,0,2.1,0.78],[0.3,0.3,-0.3,-2.0,0.2,2.3,0.0],[0,0.5,0,-1.8,0,2.3,0.8]]
for name,R in {"x=+y,y=-z":[[0,0,-1],[1,0,0],[0,-1,0]], "x=-y,y=+z":[[0,0,-1],[-1,0,0],[0,1,0]]}.items():
    q=Rot.from_matrix(np.array(R,float)).as_quat()
    for x in [-0.20,-0.25,-0.15]:
        for z in [1.064,1.10]:
            for i,sd in enumerate(seeds):
                s=r.ik((x,-0.148,z),q,seed=sd,timeout=1.0)
                if s: print(name,"x",x,"z",z,"seed",i,"->",np.round(s,3)); break
EOF

# openrua op 48
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "^  move attempt" | tail -20
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
r=Robot("iktest")
cur=r.arm_q()
seeds=[cur,[0,-0.16,0,-2.44,0,2.23,0.78],[0,-0.5,0,-2.6,0,2.1,0.78],[0.3,0.3,-0.3,-2.0,0.2,2.3,0.0],[0,0.5,0,-1.8,0,2.3,0.8]]
for name,R in {"x=+y,y=-z":[[0,0,-1],[1,0,0],[0,-1,0]], "x=-y,y=+z":[[0,0,-1],[-1,0,0],[0,1,0]]}.items():
    q=Rot.from_matrix(np.array(R,float)).as_quat()
    for x in [-0.20,-0.25,-0.15]:
        for z in [1.064,1.10]:
            for i,sd in enumerate(seeds):
                try:
                    s=r.ik((x,-0.148,z),q,seed=sd,timeout=1.0)
                except Exception as e:
                    print("err",e); s=None
                if s: print(name,"x",x,"z",z,"seed",i,"->",np.round(s,3)); break
            else:
                print(name,"x",x,"z",z,"no solution")
EOF

# openrua op 49
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "^  move attempt" | tail -20
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
r=Robot("iktest")
cur=r.arm_q()
seeds=[cur,[0,-0.16,0,-2.44,0,2.23,0.78],[0,0.5,0,-1.8,0,2.3,0.8],[0.3,0.3,-0.3,-2.0,0.2,2.3,0.0],[-0.3,0.6,0.3,-1.9,0,2.5,0.5]]
tcp=np.array([-0.325,-0.148,1.064])
for deg in [30,45,60]:
    a=np.radians(deg)
    z=np.array([-np.cos(a),0,-np.sin(a)])         # approach: toward -x and down
    for sgn in [1,-1]:
        y=sgn*np.array([-np.sin(a),0,np.cos(a)])   # pinch dir, perpendicular to z in x-z plane
        x=np.cross(y,z)
        R=np.stack([x,y,z],1); q=Rot.from_matrix(R).as_quat()
        for back in [0.0,0.08]:
            p=tcp-back*z
            ok=None
            for i,sd in enumerate(seeds):
                s=r.ik(p,q,seed=sd,timeout=1.0)
                if s: ok=(i,np.round(s,3)); break
            print(f"tilt {deg} sgn {sgn} back {back}: ", ok)
EOF

# openrua op 50
mkdir -p "$(dirname /workspace/grasp2.py)"
cat > /workspace/grasp2.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Re-grasp the fallen cup: tilted approach through the open rim, pinch the top wall of the rim.

Cup lies with axis along x, opening toward +x at x~-0.305, top of rim z~1.067, y centre -0.148.
Hand approaches from +x/above (tilt A deg below horizontal); fingers pinch in the x-z plane.
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot

A = np.radians(float(sys.argv[1]) if len(sys.argv) > 1 else 60.0)
TCP = np.array([float(v) for v in sys.argv[2:5]]) if len(sys.argv) > 4 else np.array([-0.330, -0.148, 1.065])
BACK = 0.08

z = np.array([-np.cos(A), 0, -np.sin(A)])       # approach: toward -x and down
y = np.array([-np.sin(A), 0, np.cos(A)])        # pinch direction (finger slide)
x = np.cross(y, z)
Q = Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()
print("hand z", z.round(3), "pinch", y.round(3), "quat", Q.round(4))

r = Robot("grasp2")
r.report("start")
r.gripper(0.04)
base_w = r.wrench()
print("baseline wrench", base_w.round(2))

q = r.arm_q()
pre = r.ik(TCP - BACK * z, Q, seed=q); assert pre, "IK pre failed"
r.move(pre, 5.0)
r.report("pre")

seed = pre
for d in np.arange(BACK - 0.02, -1e-6, -0.02):
    p = TCP - d * z
    s = r.ik(p, Q, seed=seed); assert s, f"IK fail at d={d}"
    seed = s
    r.move(s, 1.0, retries=1)
    w = r.wrench()
    pos = r.report(f"d={d:.2f}")
    dw = w - base_w
    print("   dwrench", dw.round(2))
    if abs(dw[2]) > 2.0 or abs(dw[0]) > 2.0:
        print("   contact! stopping approach")
        break

f = r.gripper(0.0)
w = r.wrench()
print("after close: fingers", f, "dwrench", (w - base_w).round(2))
OPENRUA_EOF

# openrua op 51
cd /workspace; python3 grasp2.py 60 2>&1 | tail -30

# openrua op 52
cd /workspace; python3 tools/cam_snap.py agentview av5.png 2>&1 | tail -2

# openrua op 53
cd /workspace; ls tools; ls *.py

# openrua op 54
cd /workspace; ls tools/perception tools/action; head -30 tools/perception/cam_snap.py 2>/dev/null

# openrua op 55
cd /workspace; python3 tools/perception/cam_snap.py agentview av5.png 2>&1 | tail -1; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png 2>&1 | tail -1

# openrua op 56
mkdir -p "$(dirname /workspace/grasp2b.py)"
cat > /workspace/grasp2b.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Continue the tilted rim approach from the current pose (baseline wrench taken here)."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot

A = np.radians(60.0)
TCP = np.array([float(v) for v in sys.argv[1:4]]) if len(sys.argv) > 3 else np.array([-0.330, -0.148, 1.065])
STEP = 0.01
z = np.array([-np.cos(A), 0, -np.sin(A)])
y = np.array([-np.sin(A), 0, np.cos(A)])
x = np.cross(y, z)
Q = Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()

r = Robot("grasp2b")
p0 = r.report("start")
r.gripper(0.04)
base_w = r.wrench()
print("baseline wrench", base_w.round(2))
d0 = float(np.dot(TCP - p0, z))            # remaining distance along approach
print("remaining approach", round(d0, 3))
seed = r.arm_q()
for d in np.arange(d0 - STEP, -1e-6, -STEP):
    p = TCP - d * z
    s = r.ik(p, Q, seed=seed); assert s, f"IK fail at d={d}"
    seed = s
    r.move(s, 0.8, retries=1)
    w = r.wrench()
    r.report(f"d={d:.3f}")
    dw = w - base_w
    print("   dwrench", dw.round(2))
    if np.abs(dw[:3]).max() > 2.5:
        print("   contact! stopping approach")
        break
f = r.gripper(0.0)
w = r.wrench()
print("after close: fingers", f, "dwrench", (w - base_w).round(2))
OPENRUA_EOF

# openrua op 57
cd /workspace; python3 grasp2b.py 2>&1 | grep -v "move attempt" | tail -30

# openrua op 58
cd /workspace; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png 2>&1 | tail -1; python3 tools/perception/cam_snap.py agentview av6.png 2>&1 | tail -1

# openrua op 59
cd /workspace; python3 cloud.py agentview 2>&1 | tail -2; python3 - <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); P=d["xyz"]; C=d["rgb"]
# points in cup region
m=(P[:,1]>-0.20)&(P[:,1]<-0.09)&(P[:,2]>0.97)&(P[:,2]<1.09)&(P[:,0]>-0.47)&(P[:,0]<-0.25)
Q=P[m]; K=C[m]
print("n",m.sum())
# For the axis region: |y+0.148|<0.02, z in [1.0,1.04] -> x distribution
a=(np.abs(Q[:,1]+0.148)<0.02)&(Q[:,2]>1.0)&(Q[:,2]<1.04)
print("axis-region x percentiles", np.percentile(Q[a,0],[0,5,25,50,75,95,100]).round(3))
# histogram of x for those
h,e=np.histogram(Q[a,0],bins=np.arange(-0.47,-0.25,0.01)); print(list(zip(e[:-1].round(2),h)))
# rgb of axis-region points near max x
top=Q[a,0]>np.percentile(Q[a,0],90); print("colour near +x face", K[a][top].mean(0).round(0))
# top surface: z max per x column
for x0 in np.arange(-0.44,-0.28,0.01):
    s=(np.abs(Q[:,0]-x0)<0.005)&(np.abs(Q[:,1]+0.148)<0.03)
    if s.sum(): print(f"x={x0:.2f} zmax={Q[s,2].max():.3f} n={s.sum()}")
EOF

# openrua op 60
cd /workspace; python3 cloud.py birdview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
for cam in ["birdview","agentview"]:
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"]
    m=(P[:,1]>-0.26)&(P[:,1]<-0.06)&(P[:,2]>0.975)&(P[:,2]<1.09)&(P[:,0]>-0.45)&(P[:,0]<-0.26)
    Q=P[m]
    print(cam, "n", len(Q))
    for x0 in np.arange(-0.43,-0.27,0.01):
        s=np.abs(Q[:,0]-x0)<0.005
        if s.sum()>3:
            ys=Q[s,1]; zs=Q[s,2]
            print(f" x={x0:.2f} y[{ys.min():.3f},{ys.max():.3f}] w={ys.max()-ys.min():.3f} zmax={zs.max():.3f} n={s.sum()}")
EOF

# openrua op 61
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "move attempt"
import numpy as np
from kin import Robot
r=Robot("retreat")
r.report("start")
QD=np.array([0.7071,0.7071,0,0])
q=r.arm_q()
p1,quat,R=r.tcp()
s=r.ik(p1+np.array([0.05,0,0.12]),quat,seed=q); assert s
r.move(s,3.0)
s2=r.ik((-0.10,-0.25,1.30),QD,seed=s); assert s2
r.move(s2,5.0)
r.report("parked")
EOF
python3 cloud.py birdview 2>&1 | tail -1; python3 cloud.py agentview 2>&1 | tail -1; python3 cloud.py sideview 2>&1 | tail -1

# openrua op 62
cd /workspace; python3 - <<'EOF'
import numpy as np
Ps=[]
for cam in ["birdview","agentview","sideview"]:
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"]
    m=(P[:,1]>-0.26)&(P[:,1]<-0.075)&(P[:,2]>0.975)&(P[:,2]<1.09)&(P[:,0]>-0.45)&(P[:,0]<-0.26)
    Ps.append(P[m]); print(cam, m.sum())
Q=np.vstack(Ps)
for x0 in np.arange(-0.43,-0.27,0.01):
    s=np.abs(Q[:,0]-x0)<0.005
    if s.sum()>3:
        ys=Q[s,1]; zs=Q[s,2]
        # body only: y > -0.20
        b=ys>-0.205
        print(f" x={x0:.2f} all y[{ys.min():.3f},{ys.max():.3f}] zmax={zs.max():.3f} | body y[{ys[b].min() if b.any() else 0:.3f},{ys[b].max() if b.any() else 0:.3f}] w={ys[b].max()-ys[b].min() if b.any() else 0:.3f} n={s.sum()}")
# side profile: for body points, z-extent at a given x, from sideview (which sees the -y side?)
d=np.load("sideview_cloud.npz"); print("sideview cam", d["cam_pos"])
EOF

# openrua op 63
cd /workspace; python3 - <<'EOF'
import numpy as np
Ps=[]
for cam in ["birdview","agentview","sideview"]:
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"]
    m=(P[:,1]>-0.20)&(P[:,1]<-0.075)&(P[:,2]>0.90)&(P[:,2]<1.09)&(P[:,0]>-0.46)&(P[:,0]<-0.26)
    Ps.append(P[m])
Q=np.vstack(Ps)
print("x slice: z-range of body points (y in [-0.19,-0.09])")
for x0 in np.arange(-0.44,-0.27,0.01):
    s=np.abs(Q[:,0]-x0)<0.005
    if s.sum()>3:
        zs=Q[s,2]; ys=Q[s,1]
        print(f" x={x0:.2f} z[{zs.min():.3f},{zs.max():.3f}]  y at zmax={ys[zs.argmax()]:.3f}  n={s.sum()}")
print("y slice at x=-0.31: (y,zmax)")
s=np.abs(Q[:,0]+0.31)<0.005
for y0 in np.arange(-0.20,-0.08,0.01):
    t=s&(np.abs(Q[:,1]-y0)<0.005)
    if t.sum(): print(f"  y={y0:.2f} z[{Q[t,2].min():.3f},{Q[t,2].max():.3f}] n={t.sum()}")
EOF

# openrua op 64
cd /workspace; python3 tools/perception/cam_snap.py sideview sv2.png 2>&1 | tail -1; python3 tools/perception/cam_snap.py frontview fv2.png 2>&1 | tail -1; python3 -c "
import cv2
im=cv2.imread('sv2.png'); h,w=im.shape[:2]; print(h,w)
"

# openrua op 65
cd /workspace; python3 -c "
import cv2
im=cv2.imread('fv2.png'); print(im.shape)
cv2.imwrite('fv2_big.png', cv2.resize(im,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
a=cv2.imread('av6.png'); c=a[140:290,100:400]; cv2.imwrite('av6_zoom.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 66
cd /workspace; python3 - <<'EOF'
import numpy as np
Ps=[];Cs=[]
for cam in ["birdview","agentview","sideview"]:
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"]; C=d["rgb"]
    m=(P[:,1]>-0.30)&(P[:,1]<-0.19)&(P[:,2]>0.95)&(P[:,2]<1.10)&(P[:,0]>-0.47)&(P[:,0]<-0.30)
    Ps.append(P[m]); Cs.append(C[m])
Q=np.vstack(Ps); K=np.vstack(Cs)
yellow=(K[:,0]>150)&(K[:,1]>120)&(K[:,2]<120)
print("yellow pts", yellow.sum())
H=Q[yellow]
print("handle bbox", H.min(0).round(3), H.max(0).round(3))
for x0 in np.arange(-0.43,-0.33,0.01):
    s=np.abs(H[:,0]-x0)<0.005
    if s.sum()>2: print(f" x={x0:.2f} y[{H[s,1].min():.3f},{H[s,1].max():.3f}] z[{H[s,2].min():.3f},{H[s,2].max():.3f}] n={s.sum()}")
print("outer arm (y<-0.215):")
O=H[H[:,1]<-0.215]
for x0 in np.arange(-0.43,-0.33,0.01):
    s=np.abs(O[:,0]-x0)<0.005
    if s.sum()>2: print(f" x={x0:.2f} y[{O[s,1].min():.3f},{O[s,1].max():.3f}] z[{O[s,2].min():.3f},{O[s,2].max():.3f}] n={s.sum()}")
# divider top: dark points y in [-0.215,-0.195]
D=Q[(~yellow)&(Q[:,1]>-0.215)&(Q[:,1]<-0.195)]
print("divider zmax per x:")
for x0 in np.arange(-0.46,-0.33,0.01):
    s=np.abs(D[:,0]-x0)<0.005
    if s.sum()>2: print(f" x={x0:.2f} zmax={D[s,2].max():.3f} n={s.sum()}")
EOF

# openrua op 67
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "move attempt"
import numpy as np
from kin import Robot
r=Robot("look")
QD=np.array([0.7071,0.7071,0,0])
s=r.ik((-0.36,-0.15,1.36),QD,seed=[0,-0.16,0,-2.44,0,2.23,0.78]); assert s
r.move(s,5.0)
r.report("look")
EOF
python3 tools/perception/cam_snap.py robot0_eye_in_hand eih5.png | tail -1

# openrua op 68
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "move attempt"
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
r=Robot("iktest2")
cur=r.arm_q()
seeds=[cur,[0,-0.16,0,-2.44,0,2.23,0.78],[0,0.5,0,-1.8,0,2.3,0.8],[0.3,0.3,-0.3,-2.0,0.2,2.3,0.0],[-0.3,0.6,0.3,-1.9,0,2.5,0.5],[-0.5,0.45,0.3,-1.95,-0.28,1.6,2.27]]
def solve(p,R,tag):
    q=Rot.from_matrix(R).as_quat()
    for i,sd in enumerate(seeds):
        s=r.ik(p,q,seed=sd,timeout=1.0)
        if s: print(tag, "OK seed",i,np.round(s,2)); return s
    print(tag,"no solution"); return None
# grasp frame: z=(-0.698,0,-0.716), pinch p=(-0.716,0,0.698)
z=np.array([-0.698,0,-0.716]); z/=np.linalg.norm(z)
p=np.array([-0.716,0,0.698]); p/=np.linalg.norm(p)
x=np.cross(p,z)
Rg=np.stack([x,p,z],1)
c=np.array([-0.36,-0.229,1.008])
solve(c,Rg,"grasp"); solve(c-0.08*z,Rg,"pre"); solve(c+[0,0,0.15],Rg,"lift")
Ry=Rot.from_euler('y',45,degrees=True).as_matrix()
for yaw in [0,90,180,-90,135,-135,45,-45]:
    Rz=Rot.from_euler('z',yaw,degrees=True).as_matrix()
    Rf=Rz@Ry@Rg
    print(f"yaw {yaw}: hand z {Rf[:,2].round(2)} pinch {Rf[:,1].round(2)}")
    # handle offset from cup centre: originally (0,-0.069) -> rotated
    off=Rz@np.array([0,-0.069,0])
    tcp=np.array([-0.455,-0.085,1.12])+off
    solve(tcp,Rf,f"  place yaw{yaw} tcp {tcp.round(3)}")
    solve(tcp+[0,0,0.10],Rf,f"  above yaw{yaw}")
EOF

# openrua op 69
cd /workspace; python3 - <<'EOF'
import numpy as np
Ps=[];Cs=[]
for cam in ["birdview","agentview","sideview"]:
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"]; C=d["rgb"]
    m=(P[:,1]>-0.24)&(P[:,1]<-0.19)&(P[:,2]>0.95)&(P[:,2]<1.10)&(P[:,0]>-0.45)&(P[:,0]<-0.30)
    Ps.append(P[m]); Cs.append(C[m])
Q=np.vstack(Ps); K=np.vstack(Cs).astype(int)
yellow=(K[:,0]>140)&(K[:,1]>110)&(K[:,2]<130)&(K[:,0]-K[:,2]>50)
H=Q[yellow]; print("n yellow", len(H))
p=np.array([-0.716,0,0.698]); p/=np.linalg.norm(p); z=np.array([-0.698,0,-0.716]); z/=np.linalg.norm(z)
c=np.array([-0.36,-0.222,1.008])
sp=(H-c)@p; sz=(H-c)@z
print("along p: min %.3f max %.3f ; along z_hand: min %.3f max %.3f"%(sp.min(),sp.max(),sz.min(),sz.max()))
print("y of yellow:", np.percentile(H[:,1],[0,10,50,90,100]).round(3))
# outer arm only (y<-0.215): p-extent
O=H[H[:,1]<-0.215]; spo=(O-c)@p
print("outer arm p-extent", spo.min().round(3), spo.max().round(3), "n", len(O))
print("outer arm mid", O.mean(0).round(3))
EOF

# openrua op 70
mkdir -p "$(dirname /workspace/grasp3.py)"
cat > /workspace/grasp3.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grasp the fallen cup by clamping its handle loop along the loop's long axis.

Loop centre c, loop axis p (tilted 45 deg in the x-z plane), hand approach z perpendicular to p
coming from +x/above. Fingers open to +-4 cm about c along p, loop is ~6.1 cm long.
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot

c = np.array([-0.360, -0.225, 1.008])
p = np.array([-0.716, 0, 0.698]); p /= np.linalg.norm(p)
z = np.array([-0.698, 0, -0.716]); z /= np.linalg.norm(z)
x = np.cross(p, z)
Q = Rot.from_matrix(np.stack([x, p, z], 1)).as_quat()
BACK, STEP = 0.08, 0.01
SEED = [-0.76, 0.59, 0.45, -1.77, -0.44, 1.51, 2.23]

r = Robot("grasp3")
r.report("start")
r.gripper(0.04)
pre = r.ik(c - BACK * z, Q, seed=SEED); assert pre, "IK pre"
r.move(pre, 5.0)
r.report("pre")
base_w = r.wrench(); print("baseline wrench", base_w.round(2))

seed = pre
for d in np.arange(BACK - STEP, -1e-6, -STEP):
    s = r.ik(c - d * z, Q, seed=seed); assert s, f"IK d={d}"
    seed = s
    r.move(s, 0.8, retries=1)
    w = r.wrench(); dw = w - base_w
    r.report(f"d={d:.3f}")
    print("   dwrench", dw.round(2))
    if np.abs(dw[:3]).max() > 2.5:
        print("   contact! stopping approach"); break

f = r.gripper(0.0)
w = r.wrench(); print("after close: fingers", np.round(f, 4), "dwrench", (w - base_w).round(2))
if f[0] > 0.015:
    print("loop clamped (fingers stopped early) -> lifting")
    s = r.ik(r.tcp()[0] + [0, 0, 0.15], Q, seed=r.arm_q()); assert s
    r.move(s, 3.0)
    w = r.wrench(); print("after lift: fingers", np.round(r.fingers(), 4), "dwrench", (w - base_w).round(2))
    r.report("lifted")
else:
    print("fingers closed fully -> missed")
OPENRUA_EOF

# openrua op 71
cd /workspace; python3 grasp3.py 2>&1 | grep -v "move attempt"

# openrua op 72
cd /workspace; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih6.png | tail -1; python3 tools/perception/cam_snap.py agentview av7.png | tail -1; python3 -c "
import cv2
a=cv2.imread('av7.png'); c=a[140:290,100:400]; cv2.imwrite('av7_zoom.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 73
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "move attempt"
import numpy as np
from kin import Robot
r=Robot("lift")
p,quat,R=r.tcp()
r.report("start")
w0=r.wrench()
s=r.ik(p+[0,0,0.12],quat,seed=r.arm_q()); assert s
r.move(s,3.0)
r.report("lifted")
print("fingers",np.round(r.fingers(),4),"wrench",r.wrench().round(2),"before",w0.round(2))
EOF
python3 tools/perception/cam_snap.py agentview av8.png | tail -1; python3 -c "
import cv2
a=cv2.imread('av8.png'); c=a[100:300,100:400]; cv2.imwrite('av8_zoom.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 74
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "move attempt"
import numpy as np
from kin import Robot
r=Robot("park")
QD=np.array([0.7071,0.7071,0,0])
p,quat,R=r.tcp()
s=r.ik(p+[0.10,0,0.15],quat,seed=r.arm_q()); assert s
r.move(s,3.0)
s=r.ik((-0.10,-0.30,1.35),QD,seed=[0,-0.16,0,-2.44,0,2.23,0.78]); assert s
r.move(s,5.0)
r.report("parked")
EOF
for c in birdview agentview sideview frontview; do python3 cloud.py $c 2>&1 | tail -1; done
python3 - <<'EOF'
import numpy as np
Ps=[];Cs=[]
for cam in ["birdview","agentview","sideview","frontview"]:
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"]; C=d["rgb"]
    m=(P[:,1]>-0.26)&(P[:,1]<-0.06)&(P[:,2]>0.90)&(P[:,2]<1.15)&(P[:,0]>-0.46)&(P[:,0]<-0.25)
    # exclude dark caddy points
    bright=C[m].astype(int).sum(1)>250
    Ps.append(P[m][bright]); Cs.append(C[m][bright])
Q=np.vstack(Ps); K=np.vstack(Cs)
print("bright pts", len(Q), "bbox", Q.min(0).round(3), Q.max(0).round(3))
for z0 in np.arange(0.90,1.15,0.01):
    s=np.abs(Q[:,2]-z0)<0.005
    if s.sum()>3:
        print(f" z={z0:.2f} x[{Q[s,0].min():.3f},{Q[s,0].max():.3f}] y[{Q[s,1].min():.3f},{Q[s,1].max():.3f}] n={s.sum()}")
EOF

# openrua op 75
cd /workspace; python3 - <<'EOF'
import numpy as np
Ps=[];Cs=[]
for cam in ["birdview","agentview","sideview","frontview"]:
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"]; C=d["rgb"]
    m=(P[:,1]>-0.26)&(P[:,1]<-0.06)&(P[:,2]>0.90)&(P[:,2]<1.15)&(P[:,0]>-0.46)&(P[:,0]<-0.25)
    bright=C[m].astype(int).sum(1)>250
    Ps.append(P[m][bright]); Cs.append(C[m][bright])
Q=np.vstack(Ps); K=np.vstack(Cs).astype(int)
yellow=(K[:,0]-K[:,2]>50)
H=Q[yellow]; B=Q[~yellow]
print("yellow n",len(H),"bbox",H.min(0).round(3),H.max(0).round(3))
print("handle slices (yellow):")
for z0 in np.arange(0.95,1.08,0.01):
    s=np.abs(H[:,2]-z0)<0.005
    if s.sum()>2: print(f" z={z0:.2f} x[{H[s,0].min():.3f},{H[s,0].max():.3f}] y[{H[s,1].min():.3f},{H[s,1].max():.3f}] n={s.sum()}")
print("outer arm (yellow, y<-0.212) per z: mean x")
for z0 in np.arange(0.95,1.08,0.01):
    s=(np.abs(H[:,2]-z0)<0.005)&(H[:,1]<-0.212)
    if s.sum()>2: print(f" z={z0:.2f} x mean {H[s,0].mean():.3f} [{H[s,0].min():.3f},{H[s,0].max():.3f}] y mean {H[s,1].mean():.3f} n={s.sum()}")
print("body (white) slices:")
for z0 in np.arange(0.95,1.08,0.01):
    s=np.abs(B[:,2]-z0)<0.005
    if s.sum()>2: print(f" z={z0:.2f} x[{B[s,0].min():.3f},{B[s,0].max():.3f}] y[{B[s,1].min():.3f},{B[s,1].max():.3f}] cx={B[s,0].mean():.3f} cy={B[s,1].mean():.3f} n={s.sum()}")
EOF

# openrua op 76
cd /workspace; python3 tools/perception/cam_snap.py frontview fv3.png | tail -1; python3 tools/perception/cam_snap.py birdview bv3.png | tail -1; python3 - <<'EOF'
import numpy as np, cv2
for cam,img in [("frontview","fv3.png"),("birdview","bv3.png")]:
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"]; uv=d["uv"]
    m=(P[:,0]>-0.50)&(P[:,0]<-0.25)&(P[:,1]>-0.30)&(P[:,1]<-0.02)&(P[:,2]>0.9)
    u0,v0=uv[m].min(0); u1,v1=uv[m].max(0)
    print(cam, "caddy region px", u0,v0,u1,v1)
    im=cv2.imread(img); pad=20
    c=im[max(0,v0-pad):v1+pad, max(0,u0-pad):u1+pad]
    cv2.imwrite(img.replace(".png","_zoom.png"), cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 77
cd /workspace; python3 - <<'EOF'
import numpy as np
from kin import Robot, quat_R
r=Robot("meas")
pos,quat,R=r.tcp()
hand=pos-0.1034*R[:,2]
print("tcp",pos.round(3),"hand",hand.round(3),"R\n",R.round(2))
for cam in ["frontview","sideview","agentview"]:
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"]
    L=(P-hand)@R   # coords in hand frame
    # hand body: between flange (-0.06 along z) and finger base (+0.05)
    m=(np.abs(L[:,0])<0.15)&(np.abs(L[:,1])<0.15)&(L[:,2]>-0.05)&(L[:,2]<0.045)
    if m.sum()<10: print(cam,"few pts",m.sum()); continue
    Q=L[m]
    print(cam,"n",m.sum(),"hand-x extent",Q[:,0].min().round(3),Q[:,0].max().round(3)," hand-y extent",Q[:,1].min().round(3),Q[:,1].max().round(3))
    for zc in np.arange(-0.04,0.045,0.01):
        s=np.abs(Q[:,2]-zc)<0.005
        if s.sum()>5: print(f"   z={zc:+.2f} x[{Q[s,0].min():+.3f},{Q[s,0].max():+.3f}] y[{Q[s,1].min():+.3f},{Q[s,1].max():+.3f}] n={s.sum()}")
EOF

# openrua op 78
mkdir -p "$(dirname /workspace/grasp4.py)"
cat > /workspace/grasp4.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Clamp the (now inverted, tilted) cup's handle loop lengthwise.

Usage: grasp4.py approach|close|lift
  approach: open, go to pre-pose and creep to loop centre c (wrench-monitored), then stop (no close)
  close:    close gripper and report
  lift:     lift 12 cm straight up and report
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot

c = np.array([-0.350, -0.222, 1.010])           # loop centre (world)
a = np.array([-0.65, 0, 0.76]); a /= np.linalg.norm(a)   # loop long axis (= cup axis, bottom up/back)
z = np.array([-a[2], 0, a[0]])                  # approach: perpendicular to a, toward -x and down
x = np.cross(a, z)
Q = Rot.from_matrix(np.stack([x, a, z], 1)).as_quat()
BACK, STEP = 0.08, 0.01
SEED = [-0.76, 0.59, 0.45, -1.77, -0.44, 1.51, 2.23]

r = Robot("grasp4")
mode = sys.argv[1]
r.report("start")
if mode == "approach":
    print("hand z", z.round(3), "pinch", a.round(3))
    r.gripper(0.04)
    pre = r.ik(c - BACK * z, Q, seed=SEED); assert pre, "IK pre"
    r.move(pre, 5.0, retries=4)
    r.report("pre")
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    seed = pre
    for d in np.arange(BACK - STEP, -1e-6, -STEP):
        s = r.ik(c - d * z, Q, seed=seed); assert s, f"IK d={d}"
        seed = s
        r.move(s, 0.8, retries=1)
        w = r.wrench(); dw = w - base_w
        r.report(f"d={d:.3f}")
        print("   dwrench", dw.round(2))
        if np.abs(dw[:3]).max() > 2.0:
            print("   contact! stopping approach"); break
elif mode == "close":
    w0 = r.wrench()
    f = r.gripper(0.0)
    print("after close: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4), "dwrench", (r.wrench() - w0).round(2))
elif mode == "lift":
    w0 = r.wrench()
    p, quat, R = r.tcp()
    s = r.ik(p + [0, 0, 0.12], quat, seed=r.arm_q()); assert s
    r.move(s, 3.0)
    r.report("lifted")
    print("fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
OPENRUA_EOF

# openrua op 79
cd /workspace; python3 grasp4.py approach 2>&1 | grep -v "move attempt"; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih7.png | tail -1

# openrua op 80
cd /workspace; python3 grasp4.py close 2>&1 | grep -v "move attempt"

# openrua op 81
cd /workspace; python3 grasp4.py lift 2>&1 | grep -v "move attempt"; python3 tools/perception/cam_snap.py agentview av9.png | tail -1; python3 -c "
import cv2
a=cv2.imread('av9.png'); c=a[60:300,100:420]; cv2.imwrite('av9_zoom.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 82
cd /workspace; grep -n -i -B2 -A6 "binary\|partial\|max_effort\|position" docs/30-action.md | head -60

# openrua op 83
cd /workspace; grep -n -A12 "kind: gripper" machine.yaml

# openrua op 84
cd /workspace; sed -n 40,60p machine.yaml

# openrua op 85
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "move attempt"
import numpy as np
from kin import Robot
QD=np.array([0.7071,0.7071,0,0])
r=Robot("push")
r.report("start")
r.gripper(0.0)
q=r.arm_q()
s=r.ik((-0.43,-0.135,1.20),QD,seed=[0,-0.16,0,-2.44,0,2.23,0.78]); assert s
r.move(s,5.0,retries=4); r.report("above-behind")
s=r.ik((-0.43,-0.135,1.045),QD,seed=s); assert s
r.move(s,2.0); r.report("push start")
w0=r.wrench(); print("base",w0.round(2))
seed=s
for x in np.arange(-0.41,-0.27,0.02):
    s=r.ik((x,-0.135,1.045),QD,seed=seed); assert s, x
    seed=s
    r.move(s,1.0,retries=1)
    w=r.wrench(); p=r.report(f"x={x:.2f}")
    print("   dw",(w-w0).round(2))
s=r.ik((-0.28,-0.135,1.25),QD,seed=seed); assert s
r.move(s,2.0); r.report("retreat")
EOF
python3 tools/perception/cam_snap.py agentview av10.png | tail -1; python3 -c "
import cv2
a=cv2.imread('av10.png'); c=a[60:360,60:460]; cv2.imwrite('av10_zoom.png', cv2.resize(c,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 86
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "move attempt"
import numpy as np
from kin import Robot
QD=np.array([0.7071,0.7071,0,0])
r=Robot("push2")
r.report("start")
s=r.ik((-0.41,-0.135,1.25),QD,seed=r.arm_q()); assert s
r.move(s,3.0,retries=3); r.report("above")
w0=r.wrench(); print("base",w0.round(2))
seed=s
for z in np.arange(1.20,1.04,-0.02):
    s=r.ik((-0.41,-0.135,z),QD,seed=seed); assert s; seed=s
    r.move(s,0.8,retries=1)
    w=r.wrench(); r.report(f"z={z:.2f}"); print("   dw",(w-w0).round(2))
    if np.abs(w-w0)[:3].max()>8: print("contact during descent"); break
for x in np.arange(-0.39,-0.27,0.02):
    s=r.ik((x,-0.135,1.045),QD,seed=seed); assert s; seed=s
    r.move(s,1.0,retries=1)
    w=r.wrench(); r.report(f"x={x:.2f}"); print("   dw",(w-w0).round(2))
s=r.ik((-0.28,-0.135,1.25),QD,seed=seed); assert s
r.move(s,2.0); r.report("retreat")
EOF
python3 tools/perception/cam_snap.py agentview av11.png | tail -1; python3 -c "
import cv2
a=cv2.imread('av11.png'); c=a[60:360,60:460]; cv2.imwrite('av11_zoom.png', cv2.resize(c,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 87
cd /workspace; python3 - <<'EOF' 2>&1 | grep -v "move attempt"
import numpy as np
from kin import Robot
QD=np.array([0.7071,0.7071,0,0])
r=Robot("park2")
s=r.ik((-0.20,0.35,1.30),QD,seed=r.arm_q()); assert s
r.move(s,5.0,retries=3); r.report("parked")
EOF
for c in birdview agentview frontview sideview; do python3 cloud.py $c 2>&1 | tail -1; done
python3 - <<'EOF'
import numpy as np
Ps=[];Cs=[]
for cam in ["birdview","agentview","frontview","sideview"]:
    d=np.load(f"{cam}_cloud.npz"); P=d["xyz"]; C=d["rgb"]
    m=(P[:,0]>-0.34)&(P[:,0]<-0.05)&(P[:,1]>-0.35)&(P[:,1]<0.05)&(P[:,2]>0.885)&(P[:,2]<1.05)
    Ps.append(P[m]); Cs.append(C[m])
Q=np.vstack(Ps); K=np.vstack(Cs).astype(int)
print("n",len(Q),"bbox",Q.min(0).round(3),Q.max(0).round(3))
yellow=(K[:,0]-K[:,2]>50); H=Q[yellow]; B=Q[~yellow]
print("handle bbox",H.min(0).round(3),H.max(0).round(3),"n",len(H))
print("body slices by x:")
for x0 in np.arange(-0.33,-0.05,0.01):
    s=np.abs(B[:,0]-x0)<0.005
    if s.sum()>5: print(f" x={x0:.2f} y[{B[s,1].min():.3f},{B[s,1].max():.3f}] z[{B[s,2].min():.3f},{B[s,2].max():.3f}] n={s.sum()}")
print("handle slices by x:")
for x0 in np.arange(-0.33,-0.05,0.01):
    s=np.abs(H[:,0]-x0)<0.005
    if s.sum()>3: print(f" x={x0:.2f} y[{H[s,1].min():.3f},{H[s,1].max():.3f}] z[{H[s,2].min():.3f},{H[s,2].max():.3f}] n={s.sum()}")
EOF

# openrua op 88
cd /workspace; python3 tools/perception/cam_snap.py frontview fv4.png | tail -1; python3 - <<'EOF'
import numpy as np, cv2
d=np.load("frontview_cloud.npz"); P=d["xyz"]; uv=d["uv"]
m=(P[:,0]>-0.34)&(P[:,0]<-0.15)&(P[:,1]>-0.32)&(P[:,1]<-0.10)&(P[:,2]>0.885)&(P[:,2]<1.0)
u0,v0=uv[m].min(0); u1,v1=uv[m].max(0); print(u0,v0,u1,v1)
im=cv2.imread("fv4.png"); pad=25
c=im[max(0,v0-pad):v1+pad, max(0,u0-pad):u1+pad]
cv2.imwrite("fv4_zoom.png", cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
d=np.load("agentview_cloud.npz"); P=d["xyz"]; uv=d["uv"]
m=(P[:,0]>-0.34)&(P[:,0]<-0.15)&(P[:,1]>-0.32)&(P[:,1]<-0.10)&(P[:,2]>0.885)&(P[:,2]<1.0)
u0,v0=uv[m].min(0); u1,v1=uv[m].max(0); print(u0,v0,u1,v1)
im=cv2.imread("av11.png"); c=im[max(0,v0-pad):v1+pad, max(0,u0-pad):u1+pad]
cv2.imwrite("av11_cup.png", cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 89
find / -iname "*desk_caddy*" 2>/dev/null | head; find / -iname "*white_yellow_mug*" 2>/dev/null | head

# openrua op 90
cat > iktest2.py <<'EOF'
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
r = Robot("iktest2")
def Q(z, y):
    z = np.array(z, float); y = np.array(y, float); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()
cases = {
 "+y horiz, pinch up, (-0.19,-0.33,0.98)": ((-0.19,-0.33,0.98), Q([0,1,0],[0,0,1])),
 "+y horiz, pinch dn, (-0.19,-0.33,0.98)": ((-0.19,-0.33,0.98), Q([0,1,0],[0,0,-1])),
 "-y horiz, pinch up, (-0.19,-0.05,0.98)": ((-0.19,-0.05,0.98), Q([0,-1,0],[0,0,1])),
 "-y horiz, pinch dn, (-0.19,-0.05,0.98)": ((-0.19,-0.05,0.98), Q([0,-1,0],[0,0,-1])),
 "+x horiz, pinch up, (-0.15,-0.19,0.98)": ((-0.15,-0.19,0.98), Q([1,0,0],[0,0,1])),
 "+x horiz, pinch dn, (-0.15,-0.19,0.98)": ((-0.15,-0.19,0.98), Q([1,0,0],[0,0,-1])),
 "+x horiz, pinch up, (-0.05,-0.19,0.98)": ((-0.05,-0.19,0.98), Q([1,0,0],[0,0,1])),
}
seeds = [r.arm_q(), [0,-0.785,0,-2.356,0,1.571,0.785], [-0.5,0.3,0.2,-2.0,0,2.3,0.5], [0.5,0.5,-0.5,-1.8,0.3,2.4,1.0]]
for name,(p,q) in cases.items():
    sols=[]
    for s in seeds:
        sol = r.ik(p, q, seed=s, timeout=0.5)
        if sol: sols.append(np.round(sol,2))
    print(name, "->", len(sols), "sols", sols[:2])
EOF
python3 iktest2.py 2>&1 | grep -v "^\[" | tail -20

# openrua op 91
cat > push3.py <<'EOF'
#!/usr/bin/env python3
"""Side push: closed fingers descend at (X,Y0,Z) then move along +y to Y1 in 1 cm steps (wrench-monitored), retreat up.
Usage: push3.py X Y0 Y1 [Z]"""
import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
X, Y0, Y1 = map(float, sys.argv[1:4]); Z = float(sys.argv[4]) if len(sys.argv) > 4 else 0.935
Q2 = Rot.from_matrix(np.stack([[-1,0,0],[0,1,0],[0,0,-1]], 1)).as_quat()   # hand x=-x, slide=y, z down
r = Robot("push3"); r.report("start")
r.gripper(0.0)
s = r.ik((X, Y0, 1.08), Q2, seed=r.arm_q()); assert s
r.move(s, 5.0, retries=3); r.report("above")
w0 = r.wrench(); seed = s
for z in np.arange(1.07, Z - 1e-6, -0.01):
    s = r.ik((X, Y0, z), Q2, seed=seed); assert s; seed = s
    r.move(s, 0.6, retries=1)
    dw = r.wrench() - w0
    if abs(dw[2]) > 3: print("descent contact at z", z, dw.round(2)); break
r.report("down"); w0 = r.wrench()
sgn = 1 if Y1 > Y0 else -1
for y in np.arange(Y0 + sgn*0.01, Y1 + sgn*1e-6, sgn*0.01):
    s = r.ik((X, y, Z), Q2, seed=seed); assert s; seed = s
    r.move(s, 0.8, retries=1)
    dw = r.wrench() - w0
    print(f"y={y:.3f} dF={dw[:3].round(2)}")
    if np.abs(dw[:2]).max() > 10: print("jam, stop"); break
p = r.report("pushed")
s = r.ik((p[0], p[1], 1.10), Q2, seed=seed); assert s; r.move(s, 2.0); r.report("up")
EOF
python3 push3.py -0.29 -0.275 -0.235 2>&1 | grep -v "^\[" | tail -20

# openrua op 92
python3 tools/perception/cam_snap.py agentview av12.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py frontview fv5.png >/dev/null 2>&1; python3 -c "
from PIL import Image
im=Image.open('av12.png'); w,h=im.size; im.crop((int(w*0.35),int(h*0.35),int(w*0.75),int(h*0.75))).resize((880,880)).save('av12_zoom.png')
im=Image.open('fv5.png'); w,h=im.size; im.crop((int(w*0.3),int(h*0.3),int(w*0.75),int(h*0.75))).resize((880,880)).save('fv5_zoom.png')
"

# openrua op 93
cat > park.py <<'EOF'
import sys, numpy as np
from kin import Robot
QD = np.array([0.7071, 0.7071, 0.0, 0.0])
r = Robot("park")
p = r.report("start")
s = r.ik((p[0], p[1], 1.30), QD, seed=r.arm_q()); assert s; r.move(s, 3.0, retries=3)
s = r.ik((-0.20, 0.35, 1.30), QD, seed=s); assert s; r.move(s, 5.0, retries=3)
r.report("parked")
EOF
python3 park.py 2>&1 | grep -E "parked|start"
for c in birdview frontview agentview sideview; do python3 cloud.py $c >/dev/null 2>&1; done; ls -la *_cloud.npz

# openrua op 94
cat > cupmeas.py <<'EOF'
import numpy as np, sys
pts=[]; cols=[]
for c in ["birdview","frontview","agentview","sideview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
    m=(xyz[:,2]>0.885)&(xyz[:,2]<1.05)&(xyz[:,0]>-0.33)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.40)&(xyz[:,1]<0.05)
    pts.append(xyz[m]); cols.append(rgb[m])
xyz=np.concatenate(pts); rgb=np.concatenate(cols)
yellow=(rgb[:,0]-rgb[:,2])>50
white=(rgb.min(1)>120)&~yellow
print("white n",white.sum(),"yellow n",yellow.sum())
W=xyz[white]; Y=xyz[yellow]
print("white bbox x",W[:,0].min().round(3),W[:,0].max().round(3)," y",W[:,1].min().round(3),W[:,1].max().round(3)," z",W[:,2].min().round(3),W[:,2].max().round(3))
print("yellow bbox x",Y[:,0].min().round(3),Y[:,0].max().round(3)," y",Y[:,1].min().round(3),Y[:,1].max().round(3)," z",Y[:,2].min().round(3),Y[:,2].max().round(3))
# PCA of white body for axis
c=W.mean(0); u,s,vt=np.linalg.svd(W-c,full_matrices=False)
print("white centroid",c.round(3),"axis",vt[0].round(3),"sv",s.round(2))
# slices along the principal axis
a=vt[0]; t=(W-c)@a
for lo in np.arange(t.min(),t.max(),0.01):
    m=(t>=lo)&(t<lo+0.01)
    if m.sum()<5: continue
    P=W[m]; print(f"t={lo:+.3f} n={m.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
print("yellow centroid",Y.mean(0).round(3))
for lo in np.arange(Y[:,2].min(),Y[:,2].max(),0.01):
    m=(Y[:,2]>=lo)&(Y[:,2]<lo+0.01)
    if m.sum()<3: continue
    P=Y[m]; print(f"yz={lo:.3f} n={m.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
EOF
python3 cupmeas.py

# openrua op 95
cat > cupmeas2.py <<'EOF'
import numpy as np
pts=[]; cols=[]
for c in ["birdview","frontview","agentview","sideview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
    m=(xyz[:,2]>0.885)&(xyz[:,2]<1.05)&(xyz[:,0]>-0.33)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.40)&(xyz[:,1]<0.05)
    pts.append(xyz[m]); cols.append(rgb[m])
xyz=np.concatenate(pts); rgb=np.concatenate(cols)
# colour clusters: print a few samples of non-white points by y band
nw = rgb.min(1)<=120
for ylo in np.arange(-0.30,-0.10,0.02):
    m=nw&(xyz[:,1]>=ylo)&(xyz[:,1]<ylo+0.02)&(xyz[:,2]>0.895)
    if m.sum()==0: continue
    print(f"y[{ylo:.2f},{ylo+0.02:.2f}] n={m.sum():4d} meanrgb={rgb[m].mean(0).round(0)} z[{xyz[m,2].min():.3f},{xyz[m,2].max():.3f}] x[{xyz[m,0].min():.3f},{xyz[m,0].max():.3f}]")
# candidate handle: yellow-ish AND bright: R>170,G>140
yel=(rgb[:,0]>160)&(rgb[:,1]>130)&(rgb[:,2]<120)&(xyz[:,2]>0.893)
Y=xyz[yel]; print("strict yellow n",len(Y))
if len(Y):
    print("bbox x",Y[:,0].min().round(3),Y[:,0].max().round(3)," y",Y[:,1].min().round(3),Y[:,1].max().round(3)," z",Y[:,2].min().round(3),Y[:,2].max().round(3))
    for lo in np.arange(Y[:,0].min(),Y[:,0].max(),0.01):
        m=(Y[:,0]>=lo)&(Y[:,0]<lo+0.01)
        if m.sum()<3: continue
        P=Y[m]; print(f"x={lo:.3f} n={m.sum():4d} y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
EOF
python3 cupmeas2.py

# openrua op 96
python3 tools/perception/cam_snap.py birdview bv4.png >/dev/null 2>&1; python3 -c "
from PIL import Image
im=Image.open('fv5.png'); w,h=im.size; im.crop((int(w*0.45),int(h*0.25),int(w*0.80),int(h*0.60))).resize((880,880)).save('fv5_zoom.png')
im=Image.open('bv4.png'); w,h=im.size; print(w,h)
"

# openrua op 97
python3 -c "
from PIL import Image
im=Image.open('fv5.png'); im.crop((180,270,320,380)).resize((1120,880)).save('fv5_zoom.png')
im=Image.open('bv4.png'); im.crop((200,150,440,330)).resize((1200,900)).save('bv4_zoom.png')
"

# openrua op 98
cat > pivot.py <<'EOF'
#!/usr/bin/env python3
"""Pinch the cup's bottom end from above (slide along y), lift slightly, yaw the hand by ANG deg
about the vertical, lower, release. Usage: pivot.py X Y ZTIP ANG"""
import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
X, Y, ZT, ANG = map(float, sys.argv[1:5])
def quat_yaw(deg):
    R0 = np.stack([[-1,0,0],[0,1,0],[0,0,-1]], 1)   # hand x=-x, slide=y, z down
    return Rot.from_matrix(Rot.from_euler('z', deg, degrees=True).as_matrix() @ R0).as_quat()
r = Robot("pivot"); r.report("start")
r.gripper(0.04)
s = r.ik((X, Y, 1.08), quat_yaw(0), seed=r.arm_q()); assert s
r.move(s, 5.0, retries=3); r.report("above")
w0 = r.wrench(); seed = s
for z in np.arange(1.07, ZT - 1e-6, -0.01):
    s = r.ik((X, Y, z), quat_yaw(0), seed=seed); assert s; seed = s
    r.move(s, 0.6, retries=1)
    dw = r.wrench() - w0
    if abs(dw[2]) > 3: print("descent contact at z", z, dw.round(2)); break
r.report("down")
f = r.gripper(0.0); print("fingers", np.round(f, 4), "gap", round(f[0]-f[1], 4))
if f[0] - f[1] < 0.05:
    print("missed -> open and stop"); r.gripper(0.04); sys.exit(1)
p = r.tcp()[0]
s = r.ik((X, Y, p[2] + 0.01), quat_yaw(0), seed=seed); assert s; r.move(s, 1.0); seed = s
step = 15.0 if ANG > 0 else -15.0
angs = list(np.arange(step, ANG + step/2, step))
if abs(angs[-1]) > abs(ANG): angs[-1] = ANG
for a in angs:
    s = r.ik((X, Y, p[2] + 0.01), quat_yaw(a), seed=seed); assert s, f"IK yaw {a}"; seed = s
    r.move(s, 1.5, retries=2)
    print(f"yaw {a:.0f}: fingers {np.round(r.fingers(),4)}")
s = r.ik((X, Y, p[2]), quat_yaw(ANG), seed=seed); assert s; r.move(s, 1.0)
r.gripper(0.04)
s = r.ik((X, Y, 1.10), quat_yaw(ANG), seed=seed); assert s; r.move(s, 2.0); r.report("up")
EOF
python3 pivot.py -0.194 -0.175 0.94 -85 2>&1 | grep -v "^\[\|attempt" | tail -20

# openrua op 99
python3 park.py 2>&1 | grep parked; for c in birdview frontview agentview; do python3 tools/perception/cam_snap.py $c ${c}_6.png >/dev/null 2>&1; done; python3 -c "
from PIL import Image
im=Image.open('birdview_6.png'); im.crop((200,150,440,330)).resize((1200,900)).save('bv6_zoom.png')
im=Image.open('frontview_6.png'); im.crop((160,250,340,390)).resize((1080,840)).save('fv6_zoom.png')
"

# openrua op 100
for c in birdview frontview agentview sideview; do python3 cloud.py $c >/dev/null 2>&1; done
cat > cupmeas3.py <<'EOF'
import numpy as np
pts=[]; cols=[]
for c in ["birdview","frontview","agentview","sideview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
    m=(xyz[:,2]>0.885)&(xyz[:,2]<1.10)&(xyz[:,0]>-0.33)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.40)&(xyz[:,1]<0.05)
    pts.append(xyz[m]); cols.append(rgb[m])
xyz=np.concatenate(pts); rgb=np.concatenate(cols)
yel=(rgb[:,0]>140)&(rgb[:,1]>110)&(rgb[:,2]<110)&((rgb[:,0]-rgb[:,2])>60)&(xyz[:,2]>0.90)
white=(rgb.min(1)>120)&~yel
W=xyz[white]; Y=xyz[yel]
print("white n",len(W),"bbox x",W[:,0].min().round(3),W[:,0].max().round(3)," y",W[:,1].min().round(3),W[:,1].max().round(3)," z",W[:,2].min().round(3),W[:,2].max().round(3))
print("yellow n",len(Y),"bbox x",Y[:,0].min().round(3),Y[:,0].max().round(3)," y",Y[:,1].min().round(3),Y[:,1].max().round(3)," z",Y[:,2].min().round(3),Y[:,2].max().round(3))
# white slices along x and along y
for lo in np.arange(W[:,0].min(),W[:,0].max(),0.01):
    m=(W[:,0]>=lo)&(W[:,0]<lo+0.01)
    if m.sum()<5: continue
    P=W[m]; print(f"wx={lo:.3f} n={m.sum():4d} y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
for lo in np.arange(Y[:,0].min(),Y[:,0].max(),0.01):
    m=(Y[:,0]>=lo)&(Y[:,0]<lo+0.01)
    if m.sum()<3: continue
    P=Y[m]; print(f"yx={lo:.3f} n={m.sum():4d} y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
for lo in np.arange(Y[:,2].min(),Y[:,2].max(),0.01):
    m=(Y[:,2]>=lo)&(Y[:,2]<lo+0.01)
    if m.sum()<3: continue
    P=Y[m]; print(f"yz={lo:.3f} n={m.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
np.savez("cup_pts.npz", W=W, Y=Y)
EOF
python3 cupmeas3.py

# openrua op 101
cat > axisfit.py <<'EOF'
import numpy as np
d=np.load("cup_pts.npz"); W=d["W"]; Y=d["Y"]
# fit axis direction in xy from ridge: for each x-slice, take the top-z points' mean y
a=np.array([-0.85,0.53]); a/=np.linalg.norm(a); n=np.array([-a[1],a[0]])
s=W[:,:2]@a; t=W[:,:2]@n
print("s range",s.min().round(3),s.max().round(3))
for lo in np.arange(s.min(),s.max(),0.01):
    m=(s>=lo)&(s<lo+0.01)
    if m.sum()<5: continue
    print(f"s={lo:.3f} n={m.sum():4d} t[{t[m].min():.3f},{t[m].max():.3f}] tmid={(t[m].min()+t[m].max())/2:.3f} zmax={W[m,2].max():.3f}")
# refine direction: ridge = top 20% z per slice
mids=[]; ss=[]
for lo in np.arange(s.min()+0.01,s.max()-0.01,0.01):
    m=(s>=lo)&(s<lo+0.01)
    if m.sum()<20: continue
    P=W[m]; k=P[:,2]>np.percentile(P[:,2],85)
    mids.append(P[k,:2].mean(0)); ss.append(lo)
mids=np.array(mids); c=mids.mean(0); u,sv,vt=np.linalg.svd(mids-c)
print("ridge dir",vt[0].round(3),"centre",c.round(3))
EOF
python3 axisfit.py

# openrua op 102
sed -i 's/a=np.array(\[-0.85,0.53\])/a=np.array([-0.638,0.77])/' axisfit.py && python3 axisfit.py

# openrua op 103
cat >> axisfit.py <<'EOF'
a=vt[0]; 
if a[0]>0: a=-a
n=np.array([-a[1],a[0]]); s=W[:,:2]@a; t=W[:,:2]@n
for lo in np.arange(s.min(),s.max(),0.01):
    m=(s>=lo)&(s<lo+0.01)
    if m.sum()<5: continue
    print(f"s={lo:.3f} n={m.sum():4d} tmid={(t[m].min()+t[m].max())/2:.3f} w={t[m].max()-t[m].min():.3f} zmax={W[m,2].max():.3f}")
sb=s.min()+0.005; sr=np.percentile(s,99)
tm=np.median(t[(s>s.min()+0.02)&(s<s.max()-0.02)])
print("axis dir",a.round(3),"bottom centre",(a*sb+n*tm).round(3),"rim centre",(a*sr+n*tm).round(3), "tm",tm.round(3))
EOF
python3 axisfit.py | tail -14

# openrua op 104
cat >> axisfit.py <<'EOF'
print("ridge points (world xy, zmax):")
for lo in np.arange(s.min(),s.max(),0.01):
    m=(s>=lo)&(s<lo+0.01)
    if m.sum()<20: continue
    P=W[m]; k=P[:,2]>np.percentile(P[:,2],85)
    print(f"  s={lo:.3f} ridge={P[k,:2].mean(0).round(3)} zmax={P[:,2].max():.3f} n={m.sum()}")
EOF
python3 axisfit.py | tail -12

# openrua op 105
cat > pinchbottom.py <<'EOF'
#!/usr/bin/env python3
"""Pinch the lying cup's bottom end from above. TCP (X,Y), slide axis n=(NX,NY), fingertip Z.
Descend with wrench monitoring; on contact shift along lateral force and retry. Then close, lift LIFT, report.
Usage: pinchbottom.py X Y NX NY Z LIFT"""
import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
X, Y, NX, NY, ZT, LIFT = map(float, sys.argv[1:7])
n = np.array([NX, NY, 0.0]); n /= np.linalg.norm(n)
z = np.array([0, 0, -1.0]); x = np.cross(n, z)
Q = Rot.from_matrix(np.stack([x, n, z], 1)).as_quat()
r = Robot("pinchbottom"); r.report("start")
r.gripper(0.04)
tcp = np.array([X, Y, 1.06])
s = r.ik(tcp, Q, seed=r.arm_q()); assert s
r.move(s, 5.0, retries=3); r.report("above")
w0 = r.wrench(); seed = s
for attempt in range(4):
    ok = True
    for zz in np.arange(1.05, ZT - 1e-6, -0.01):
        s = r.ik((tcp[0], tcp[1], zz), Q, seed=seed); assert s; seed = s
        r.move(s, 0.5, retries=1)
        dw = r.wrench() - w0
        if abs(dw[2]) > 2.5:
            lat = np.dot(dw[:3], n)
            print(f"descent contact z={zz:.3f} dF={dw[:3].round(2)} lateral={lat:.2f}")
            ok = False; break
    if ok: break
    # retreat and shift
    s = r.ik((tcp[0], tcp[1], 1.02), Q, seed=seed); r.move(s, 1.0); seed = s
    shift = 0.006 * np.sign(lat) if abs(lat) > 0.3 else 0.006
    tcp[:2] += shift * n[:2]; print("shift to", tcp.round(4))
p = r.report("down")
f = r.gripper(0.0); print("fingers", np.round(f, 4), "gap", round(f[0]-f[1], 4))
if f[0] - f[1] < 0.05:
    print("missed -> open"); r.gripper(0.04); sys.exit(1)
w1 = r.wrench()
s = r.ik(p + [0, 0, LIFT], Q, seed=seed); assert s; r.move(s, 2.0, retries=2)
print("after lift fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w1).round(2))
r.report("lifted")
EOF
python3 pinchbottom.py -0.2096 -0.1655 0.816 0.575 0.915 0.04 2>&1 | grep -v "^\[\|attempt" | tail -12

# openrua op 106
python3 park.py 2>&1 | grep parked; for c in birdview frontview agentview sideview; do python3 cloud.py $c >/dev/null 2>&1; done; python3 cupmeas3.py | head -3; python3 axisfit.py | tail -12; python3 tools/perception/cam_snap.py birdview bv7.png >/dev/null 2>&1; python3 -c "
from PIL import Image
im=Image.open('bv7.png'); im.crop((200,150,440,330)).resize((1200,900)).save('bv7_zoom.png')"

# openrua op 107
python3 park.py 2>&1 | grep -v "^\[" | tail -8

# openrua op 108
python3 fk.py 2>&1 | tail -3; for c in birdview agentview frontview sideview; do python3 cloud.py $c >/dev/null 2>&1; done
cat > findcup.py <<'EOF'
import numpy as np
pts=[]; cols=[]
for c in ["birdview","frontview","agentview","sideview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
    m=(xyz[:,2]>0.885)&(xyz[:,2]<1.05)&(xyz[:,0]>-0.33)&(xyz[:,0]<0.3)&(xyz[:,1]>-0.5)&(xyz[:,1]<0.5)
    pts.append(xyz[m]); cols.append(rgb[m])
xyz=np.concatenate(pts); rgb=np.concatenate(cols)
yel=(rgb[:,0]>140)&(rgb[:,1]>110)&(rgb[:,2]<110)&((rgb[:,0]-rgb[:,2])>60)&(xyz[:,2]>0.90)
Y=xyz[yel]; print("yellow n",len(Y))
if len(Y): print(" bbox x",Y[:,0].min().round(3),Y[:,0].max().round(3)," y",Y[:,1].min().round(3),Y[:,1].max().round(3)," z",Y[:,2].min().round(3),Y[:,2].max().round(3), "centroid", Y.mean(0).round(3))
# white near yellow centroid
c=Y.mean(0)
near=(np.linalg.norm(xyz[:,:2]-c[:2],axis=1)<0.12)
white=(rgb.min(1)>120)&~yel&near
W=xyz[white]; print("white n",len(W)," bbox x",W[:,0].min().round(3),W[:,0].max().round(3)," y",W[:,1].min().round(3),W[:,1].max().round(3)," z",W[:,2].min().round(3),W[:,2].max().round(3))
for lo in np.arange(W[:,2].min(),W[:,2].max(),0.01):
    m=(W[:,2]>=lo)&(W[:,2]<lo+0.01)
    if m.sum()<5: continue
    P=W[m]; print(f"wz={lo:.3f} n={m.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] cx={P[:,0].mean():.3f} cy={P[:,1].mean():.3f}")
for lo in np.arange(Y[:,2].min(),Y[:,2].max(),0.01):
    m=(Y[:,2]>=lo)&(Y[:,2]<lo+0.01)
    if m.sum()<3: continue
    P=Y[m]; print(f"yz={lo:.3f} n={m.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
# book: dark points
dark=(rgb.max(1)<60)&(xyz[:,2]>0.89)&(xyz[:,0]>-0.3)
D=xyz[dark]; print("dark n",len(D)," bbox x",D[:,0].min().round(3),D[:,0].max().round(3)," y",D[:,1].min().round(3),D[:,1].max().round(3)," z",D[:,2].min().round(3),D[:,2].max().round(3))
EOF
python3 findcup.py

# openrua op 109
python3 tools/perception/cam_snap.py birdview bv8.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py agentview av13.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview sv3.png >/dev/null 2>&1; python3 -c "
from PIL import Image
im=Image.open('bv8.png'); im.crop((280,190,440,310)).resize((1200,900)).save('bv8_zoom.png')
im=Image.open('av13.png'); im.crop((300,200,500,350)).resize((1200,900)).save('av13_zoom.png')
"

# openrua op 110
cat > bookmeas.py <<'EOF'
import numpy as np
d=np.load("birdview_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
m=(xyz[:,2]>0.888)&(xyz[:,2]<0.95)&(xyz[:,0]>-0.40)&(xyz[:,0]<-0.05)&(xyz[:,1]>0.0)&(xyz[:,1]<0.35)&(rgb.max(1)<70)
D=xyz[m]; print("dark n",len(D))
print("bbox x",D[:,0].min().round(3),D[:,0].max().round(3)," y",D[:,1].min().round(3),D[:,1].max().round(3)," z",D[:,2].min().round(3),D[:,2].max().round(3))
for lo in np.arange(D[:,2].min(),D[:,2].max(),0.005):
    k=(D[:,2]>=lo)&(D[:,2]<lo+0.005)
    if k.sum()<5: continue
    P=D[k]; print(f"z={lo:.3f} n={k.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
# corners via PCA of top face
top=D[D[:,2]>np.percentile(D[:,2],50)]
c=top[:,:2].mean(0); u,s,vt=np.linalg.svd(top[:,:2]-c,full_matrices=False)
proj=(top[:,:2]-c)@vt.T
print("top centre",c.round(3),"dirs",vt.round(3),"extent",(proj.max(0)-proj.min(0)).round(3))
EOF
python3 bookmeas.py

# openrua op 111
cat > iktest3.py <<'EOF'
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
r = Robot("iktest3")
def Q(z, y):
    z = np.array(z, float); y = np.array(y, float); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()
sl = [0, 0.866, 0.5]
Qa = Q([1,0,0], sl)
Qb = Q([0,0,-1], [0.5,0.866,0])
cases = {
 "pre (-0.335,0.118,0.954)": ((-0.335,0.118,0.954), Qa),
 "grasp (-0.215,0.118,0.954)": ((-0.215,0.118,0.954), Qa),
 "lift (-0.215,0.118,1.05)": ((-0.215,0.118,1.05), Qa),
 "lift2 (-0.215,0.118,1.15)": ((-0.215,0.118,1.15), Qa),
 "rot down (-0.30,0.0,1.25)": ((-0.30,0.0,1.25), Qb),
 "perch (-0.434,-0.041,1.15)": ((-0.434,-0.041,1.15), Qb),
}
seeds = [r.arm_q(), [0,-0.785,0,-2.356,0,1.571,0.785], [-0.5,0.3,0.2,-2.0,0,2.3,0.5], [0.5,0.5,-0.5,-1.8,0.3,2.4,1.0], [0.06,0.8,-0.43,-2.23,2.9,1.65,-0.45], [1.93,-0.96,-2.44,-2.15,2.41,2.07,-1.09]]
for name,(p,q) in cases.items():
    sols=[]
    for s in seeds:
        sol = r.ik(p, q, seed=s, timeout=0.5)
        if sol: sols.append(np.round(sol,2))
    print(name, "->", len(sols), "sols"); [print("    ", s) for s in sols[:3]]
EOF
python3 iktest3.py 2>&1 | grep -v "^\[" | tail -30

# openrua op 112
cat > axisfit2.py <<'EOF'
import numpy as np
pts=[]
for c in ["birdview","frontview","agentview","sideview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
    m=(xyz[:,2]>0.90)&(xyz[:,2]<1.0)&(xyz[:,0]>-0.30)&(xyz[:,0]<-0.09)&(xyz[:,1]>0.0)&(xyz[:,1]<0.135)&(rgb.min(1)>90)&~((rgb.max(1)<70))
    # exclude arm: arm is far (parked at y 0.35)
    pts.append(xyz[m])
B=np.concatenate(pts); print("body pts",len(B))
print("bbox x",B[:,0].min().round(3),B[:,0].max().round(3)," y",B[:,1].min().round(3),B[:,1].max().round(3)," z",B[:,2].min().round(3),B[:,2].max().round(3))
for lo in np.arange(B[:,0].min(),B[:,0].max(),0.01):
    m=(B[:,0]>=lo)&(B[:,0]<lo+0.01)
    if m.sum()<10: continue
    P=B[m]; k=P[:,2]>np.percentile(P[:,2],90)
    print(f"x={lo:.3f} n={m.sum():4d} y[{P[:,1].min():.3f},{P[:,1].max():.3f}] ymid={(P[:,1].min()+P[:,1].max())/2:.3f} zmax={P[:,2].max():.3f} ridge_y={P[k,1].mean():.3f}")
EOF
python3 axisfit2.py

# openrua op 113
cat > grasp5.py <<'EOF'
#!/usr/bin/env python3
"""Horizontal rim-wall pinch of the cup lying on its side, then rotate upright and perch.
Modes: approach | lift | rotate | place X Y
"""
import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot

def unit(v): v = np.array(v, float); return v / np.linalg.norm(v)
def quat(z, y):
    z = unit(z); y = unit(y); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()

RIM_C = np.array([-0.240, 0.079, 0.942])          # rim centre (world)
A = unit([0.964, -0.217, -0.154])                 # approach = into the opening (rim -> bottom)
U = unit(np.array([0, 0, 1.0]) - np.dot([0, 0, 1.0], A) * A)   # "up" perpendicular to A
S = unit(np.cross(U, A))                          # side toward +y (handle side)
ANG = np.radians(60.0)
SLIDE = unit(np.sin(ANG) * S + np.cos(ANG) * U)   # outer finger direction
R_WALL, INSIDE = 0.0475, 0.02
TCP = RIM_C + INSIDE * A + R_WALL * SLIDE
QA = quat(A, SLIDE)
# after rotation: hand down, slide horizontal
SLIDE2 = unit([0.678, 0.734, 0.0])
QB = quat([0, 0, -1], SLIDE2)
CUP_OFF2 = R_WALL * SLIDE2[:2]                    # cup centre = TCP_xy - CUP_OFF2

r = Robot("grasp5")
mode = sys.argv[1]
r.report("start")
if mode == "approach":
    print("A", A.round(3), "SLIDE", SLIDE.round(3), "TCP", TCP.round(4))
    r.gripper(0.04)
    pre = r.ik(TCP - 0.12 * A, QA, seed=r.arm_q()); assert pre, "IK pre"
    r.move(pre, 6.0, retries=4)
    r.report("pre")
    w0 = r.wrench(); seed = pre
    for d in np.arange(0.11, -1e-6, -0.01):
        s = r.ik(TCP - d * A, QA, seed=seed); assert s, f"IK d={d}"; seed = s
        r.move(s, 0.7, retries=1)
        dw = r.wrench() - w0
        r.report(f"d={d:.2f}"); print("   dF", dw[:3].round(2))
        if np.abs(dw[:3]).max() > 3.0:
            print("   contact! stop"); break
    f = r.gripper(0.0)
    print("closed: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4))
elif mode == "lift":
    p, q, _ = r.tcp(); w0 = r.wrench(); seed = r.arm_q()
    for dz in (0.03, 0.08, 0.15, 0.21):
        s = r.ik(p + [0, 0, dz], q, seed=seed); assert s; seed = s
        r.move(s, 2.0, retries=2)
        print(f"dz={dz}: fingers {np.round(r.fingers(),4)} dF {(r.wrench()-w0)[:3].round(2)}")
    r.report("lifted")
elif mode == "rotate":
    p, q, _ = r.tcp(); seed = r.arm_q()
    key = Rot.from_quat([QA, QB])
    for t in (0.33, 0.66, 1.0):
        qi = Rot.from_quat(QA) * (Rot.from_quat(QA).inv() * Rot.from_quat(QB)) ** t
        s = r.ik(p, qi.as_quat(), seed=seed); assert s, f"IK rot t={t}"; seed = s
        r.move(s, 3.0, retries=3)
        r.report(f"rot t={t}"); print("   fingers", np.round(r.fingers(), 4))
elif mode == "place":
    X, Y = float(sys.argv[2]), float(sys.argv[3])
    txy = np.array([X, Y]) + CUP_OFF2
    p, q, _ = r.tcp(); seed = r.arm_q()
    s = r.ik((txy[0], txy[1], 1.20), QB, seed=seed); assert s; seed = s
    r.move(s, 6.0, retries=3); r.report("over perch")
    w0 = r.wrench(); print("baseline", w0[:3].round(2))
    z = 1.19
    while z >= 1.10:
        s = r.ik((txy[0], txy[1], z), QB, seed=seed); assert s; seed = s
        r.move(s, 0.8, retries=1)
        dw = r.wrench() - w0
        r.report(f"z={z:.3f}"); print("   dF", dw[:3].round(2), "fingers", np.round(r.fingers(), 4))
        if abs(dw[2]) > 1.5 or np.abs(dw[3:5]).max() > 0.4:
            print("   contact -> release"); break
        z -= 0.01
    r.gripper(0.04)
    s = r.ik((txy[0], txy[1], 1.25), QB, seed=seed); assert s; r.move(s, 2.0, retries=2)
    r.report("retreated")
EOF
python3 grasp5.py approach 2>&1 | grep -v "^\[\|attempt" | tail -30

# openrua op 114
python3 tools/perception/cam_snap.py agentview av14.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview sv4.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih8.png >/dev/null 2>&1; python3 -c "
from PIL import Image
im=Image.open('av14.png'); im.crop((280,180,520,360)).resize((1200,900)).save('av14_zoom.png')
im=Image.open('sv4.png'); print(im.size)"

# openrua op 115
python3 fk.py 2>&1 | tail -4

# openrua op 116
cat > chk.py <<'EOF'
import numpy as np
from kin import Robot
import grasp5_defs as g
EOF
python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, sys
sys.argv=["x","none"]
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
exec(open("grasp5.py").read().split("r = Robot")[0])
r = Robot("chk")
q = r.arm_q(); print("current q", np.round(q,3))
p, quat, R = r.tcp(); print("tcp", p.round(4), "z", R[:,2].round(3))
for seed in [q, [0,-0.785,0,-2.356,0,1.571,0.785], [-0.5,0.3,0.2,-2.0,0,2.3,0.5]]:
    s = r.ik(TCP, QA, seed=seed)
    if s:
        s = np.array(s); print("ik sol", s.round(3))
        pp, qq, RR = r.tcp(s) if False else (None,None,None)
        # FK of the solution
        pos, qu = r.fk(list(s), "panda_hand")
        print("   fk hand pos", np.round(pos,4))
EOF

# openrua op 117
cat > ikbest.py <<'EOF'
import numpy as np
LO = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
HI = np.array([ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973])
def margin(q):
    q = np.array(q); return float(np.min(np.minimum(q-LO, HI-q)))
def ik_best(r, pos, quat, n=25, seed0=None, timeout=0.3, rng=np.random.default_rng(0)):
    sols = []
    seeds = [seed0] if seed0 is not None else []
    seeds += [[0,-0.785,0,-2.356,0,1.571,0.785]]
    seeds += [list(rng.uniform(LO+0.3, HI-0.3)) for _ in range(n)]
    for s in seeds:
        sol = r.ik(pos, quat, seed=s, timeout=timeout)
        if sol: sols.append((margin(sol), np.array(sol)))
    if not sols: return None, []
    sols.sort(key=lambda t: -t[0])
    return sols[0][1], sols
EOF
python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, sys
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
from ikbest import ik_best, margin
def unit(v): v=np.array(v,float); return v/np.linalg.norm(v)
def quat(z,y):
    z=unit(z); y=unit(y); x=np.cross(y,z); return Rot.from_matrix(np.stack([x,y,z],1)).as_quat()
RIM_C=np.array([-0.240,0.079,0.942]); A=unit([0.964,-0.217,-0.154])
U=unit(np.array([0,0,1.0])-np.dot([0,0,1.0],A)*A); S=unit(np.cross(U,A))
r=Robot("ikb")
for ang in (60,30,0,-30,-60):
    for sgn in (1,-1):
        a=np.radians(ang); sl=unit(np.sin(a)*S+np.cos(a)*U)
        TCP=RIM_C+0.02*A+0.0475*sl
        q=quat(A, sgn*sl)
        best,sols=ik_best(r, TCP-0.12*A, q, n=15)
        best2,sols2=ik_best(r, TCP, q, n=15)
        m1 = margin(best) if best is not None else -1; m2 = margin(best2) if best2 is not None else -1
        print(f"ang={ang:+d} sgn={sgn:+d}: pre n={len(sols)} margin={m1:.2f} | grasp n={len(sols2)} margin={m2:.2f}", np.round(best2,2) if best2 is not None else "")
EOF

# openrua op 118
python3 - <<'EOF'
import re
src = open("grasp5.py").read()
src = src.replace("from kin import Robot\n", "from kin import Robot\nfrom ikbest import ik_best, margin\n")
old = src[src.index('if mode == "approach":'):src.index('elif mode == "lift":')]
new = '''if mode == "approach":
    print("A", A.round(3), "SLIDE", SLIDE.round(3), "TCP", TCP.round(4))
    r.gripper(0.04)
    p, q, _ = r.tcp()
    s = r.ik(p + [0, 0, 1.20 - p[2]], q, seed=r.arm_q()); assert s
    r.move(s, 3.0, retries=2); r.report("up")
    qg, sols = ik_best(r, TCP, QA, n=30); assert qg is not None
    print("grasp config", qg.round(2), "margin", round(margin(qg), 2))
    qpre = r.ik(TCP - 0.12 * A, QA, seed=list(qg)); assert qpre, "IK pre"
    print("pre config", np.round(qpre, 2), "dist to grasp", round(float(np.abs(np.array(qpre) - qg).max()), 2))
    qhi = r.ik(TCP - 0.12 * A + [0, 0, 0.15], QA, seed=qpre); assert qhi, "IK hi"
    print("hi config", np.round(qhi, 2))
    r.move(qhi, 7.0, retries=4); r.report("hi")
    r.move(qpre, 4.0, retries=3); r.report("pre")
    w0 = r.wrench(); seed = qpre
    for d in np.arange(0.11, -1e-6, -0.01):
        s = r.ik(TCP - d * A, QA, seed=seed); assert s, f"IK d={d}"; seed = s
        r.move(s, 0.7, retries=1)
        dw = r.wrench() - w0
        r.report(f"d={d:.2f}"); print("   dF", dw[:3].round(2))
        if np.abs(dw[:3]).max() > 3.0:
            print("   contact! stop"); break
    f = r.gripper(0.0)
    print("closed: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4))
'''
src = src.replace(old, new)
open("grasp5.py", "w").write(src)
EOF
python3 grasp5.py approach 2>&1 | grep -v "attempt" | tail -30

# openrua op 119
grep -n "def move" -A 40 kin.py | head -80

# openrua op 120
grep -n "def arm_q\|def joints\|def fk\|def tcp\|def report" -A 14 kin.py | head -90

# openrua op 121
python3 - <<'EOF' 2>&1 | grep -v "^\[x"
import numpy as np, time
from kin import Robot
r = Robot("drift")
q1 = np.array(r.arm_q()); p1 = r.tcp()[0]
time.sleep(2)
q2 = np.array(r.arm_q()); p2 = r.tcp()[0]
print("q1", q1.round(3)); print("q2", q2.round(3)); print("tcp", p1.round(4), p2.round(4))
# small move: command current joints (no motion) and check
code = r.move(list(q2), 1.0, retries=0)
q3 = np.array(r.arm_q()); print("after null move q3", q3.round(3), "tcp", r.tcp()[0].round(4))
# command a small joint change on joint 3 (+0.1)
qt = q3.copy(); qt[2] += 0.1
r.move(list(qt), 1.5, retries=0)
q4 = np.array(r.arm_q()); print("target", qt.round(3)); print("reached", q4.round(3), "err", np.abs(q4-qt).round(3))
time.sleep(1); q5 = np.array(r.arm_q()); print("later", q5.round(3))
EOF

# openrua op 122
python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
from ikbest import ik_best, margin
def unit(v): v=np.array(v,float); return v/np.linalg.norm(v)
def quat(z,y):
    z=unit(z); y=unit(y); x=np.cross(y,z); return Rot.from_matrix(np.stack([x,y,z],1)).as_quat()
RIM_C=np.array([-0.240,0.079,0.942]); c=unit([-0.964,0.217,0.154])
U=unit(np.array([0,0,1.0])-np.dot([0,0,1.0],c)*c)
r=Robot("ikb2")
for pitch in (45,55):
    P=np.radians(pitch)
    A=unit(-c*np.cos(P)-U*np.sin(P)); SL=unit(-c*np.sin(P)+U*np.cos(P))
    TCP=RIM_C+0.018*(-c)+0.0475*U
    for sgn in (1,-1):
        q=quat(A,sgn*SL)
        b1,s1=ik_best(r,TCP-0.10*A,q,n=12); b2,s2=ik_best(r,TCP,q,n=12); b3,s3=ik_best(r,TCP+[0,0,0.15],q,n=12)
        f=lambda b: f"{margin(b):.2f}" if b is not None else "none"
        print(f"pitch={pitch} sgn={sgn:+d}: pre {f(b1)} grasp {f(b2)} lifted {f(b3)}", np.round(b2,2) if b2 is not None else "")
EOF

# openrua op 123
mkdir -p "$(dirname /workspace/grasp6.py)"
cat > /workspace/grasp6.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pitched rim-wall pinch of the cup lying on its side, then upright it and perch it on the caddy.

Modes: plan | approach | lift | rotate | place X Y | retreat
Cup: rim centre RIM_C, axis c (bottom->rim).  Hand approaches into the opening along A (= -c pitched
PITCH deg downward), fingers pinch the top wall (slide axis in the vertical plane through c).
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
from ikbest import ik_best, margin

def unit(v):
    v = np.array(v, float); return v / np.linalg.norm(v)

def quat(z, y):
    z = unit(z); y = unit(y); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()

RIM_C = np.array([-0.240, 0.079, 0.942])
c = unit([-0.964, 0.217, 0.154])                    # cup axis bottom -> rim
U = unit(np.array([0, 0, 1.0]) - c[2] * c)          # "up" perpendicular to c
PITCH = np.radians(45.0)
SGN = 1
INSIDE, WALL_R = 0.018, 0.0475
A = unit(-c * np.cos(PITCH) - U * np.sin(PITCH))     # approach direction
SL = unit(-c * np.sin(PITCH) + U * np.cos(PITCH))    # inner -> outer finger
TCP = RIM_C - INSIDE * c + WALL_R * U
QA = quat(A, SGN * SL)
# rotation that brings the cup upright (c -> +z) about a horizontal axis
ax = unit(np.cross(c, [0, 0, 1.0])); ang = np.arccos(np.clip(c[2], -1, 1))
R_UP = Rot.from_rotvec(ax * ang)
QB = (R_UP * Rot.from_quat(QA)).as_quat()
H = R_UP.apply(U)                                   # world direction TCP lies from the cup axis when upright
CUP_OFF2 = WALL_R * H[:2]                           # cup centre = TCP_xy - CUP_OFF2
BACK, STEP, LIFT_Z = 0.10, 0.01, 1.15

r = Robot("grasp6")

def go(q, secs, tag, retries=3, tol=0.02):
    r.move(q, secs, retries=retries)
    err = np.abs(np.array(r.arm_q()) - np.array(q)).max()
    p = r.report(tag)
    print(f"   joint err after move {err:.4f}")
    if err > tol:
        print("   !!! move did not reach goal"); sys.exit(1)
    return p

def check(q, pos, quat_, tag):
    p, qq, _ = r.fk_tcp(q) if hasattr(r, "fk_tcp") else (None, None, None)

mode = sys.argv[1]
r.report("start")
print("A", A.round(3), "SL", SL.round(3), "TCP", TCP.round(4), "QB hand z", Rot.from_quat(QB).apply([0, 0, 1]).round(3),
      "H", H.round(3))
if mode == "plan":
    for tag, pos, qt in [("grasp", TCP, QA), ("pre", TCP - BACK * A, QA), ("hi", TCP - BACK * A + [0, 0, 0.15], QA),
                         ("lifted", TCP + [0, 0, LIFT_Z - TCP[2]], QA), ("liftedB", TCP + [0, 0, LIFT_Z - TCP[2]], QB),
                         ("perchB", np.array([-0.458, -0.082, 1.16]) + [CUP_OFF2[0], CUP_OFF2[1], 0], QB)]:
        b, s = ik_best(r, pos, qt, n=12)
        print(tag, "margin", None if b is None else round(margin(b), 2), None if b is None else np.round(b, 2))
elif mode == "approach":
    r.gripper(0.04)
    qg, _ = ik_best(r, TCP, QA, n=30); assert qg is not None
    print("grasp cfg", np.round(qg, 2), "margin", round(margin(qg), 2))
    qpre = r.ik(TCP - BACK * A, QA, seed=qg); assert qpre
    qhi = r.ik(TCP - BACK * A + [0, 0, 0.15], QA, seed=qpre); assert qhi
    for nm, q in (("pre", qpre), ("hi", qhi)):
        print(nm, np.round(q, 2), "dq from grasp", np.abs(np.array(q) - np.array(qg)).max().round(2))
    q0 = r.arm_q()
    qlift = r.ik(r.tcp()[0] + [0, 0, 1.20 - r.tcp()[0][2]], r.tcp()[1], seed=q0)
    if qlift: go(qlift, 4.0, "lift-current")
    go(qhi, 7.0, "hi", retries=4)
    go(qpre, 4.0, "pre")
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    seed = qpre
    for d in np.arange(BACK - STEP, -1e-6, -STEP):
        s = r.ik(TCP - d * A, QA, seed=seed); assert s, f"IK d={d}"
        if np.abs(np.array(s) - np.array(seed)).max() > 0.4:
            print("   branch jump, abort"); sys.exit(1)
        seed = s
        r.move(s, 0.8, retries=1)
        dw = r.wrench() - base_w
        r.report(f"d={d:.3f}")
        print("   dwrench", dw.round(2))
        if np.abs(dw[:3]).max() > 3.0:
            print("   contact! stopping approach"); break
    f = r.gripper(0.0)
    print("after close: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4))
elif mode == "lift":
    w0 = r.wrench(); p, qt, _ = r.tcp(); seed = r.arm_q()
    for dz in (0.03, 0.08, 0.14, LIFT_Z - p[2]):
        s = r.ik(p + [0, 0, dz], qt, seed=seed); assert s
        seed = s; r.move(s, 1.5, retries=2)
        r.report(f"lift {dz:.2f}")
        print("   fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
elif mode == "rotate":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    q0 = Rot.from_quat(qt); q1 = Rot.from_quat(QB)
    rel = q1 * q0.inv(); rv = rel.as_rotvec(); n = 4
    for i in range(1, n + 1):
        qi = (Rot.from_rotvec(rv * i / n) * q0).as_quat()
        s = r.ik(p, qi, seed=seed); assert s, f"IK rot {i}"
        seed = s; r.move(s, 2.0, retries=2)
        r.report(f"rot {i}/{n}")
        print("   fingers", np.round(r.fingers(), 4), "wrench", r.wrench().round(2))
elif mode == "place":
    X, Y = float(sys.argv[2]), float(sys.argv[3])
    txy = np.array([X, Y]) + CUP_OFF2
    p, qt, _ = r.tcp(); seed = r.arm_q()
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    s = r.ik((txy[0], txy[1], 1.20), QB, seed=seed); assert s
    go(s, 6.0, "over perch"); seed = s
    z = 1.19
    while z >= 1.09:
        s = r.ik((txy[0], txy[1], z), QB, seed=seed); assert s
        seed = s; r.move(s, 1.0, retries=1)
        w = r.wrench(); dw = w - base_w
        r.report(f"z={z:.3f}")
        print("   dwrench", dw.round(2), "fingers", np.round(r.fingers(), 4))
        if abs(dw[2]) > 1.5 or np.abs(w[3:5]).max() > 0.4:
            print("   contact"); break
        z -= 0.01
    r.gripper(0.04)
    print("released; fingers", np.round(r.fingers(), 4))
elif mode == "retreat":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    for dz in (0.04, 0.08):
        s = r.ik(p + [0, 0, dz], qt, seed=seed); assert s
        seed = s; r.move(s, 1.5, retries=2); r.report(f"up {dz}")
    s = r.ik(p + [0.10, 0.10, 0.20], qt, seed=seed)
    if s: r.move(s, 3.0, retries=2); r.report("away")
OPENRUA_EOF

# openrua op 124
python3 grasp6.py plan 2>&1 | grep -v "^\[" | tail -12

# openrua op 125
python3 grasp6.py approach 2>&1 | grep -v "^\[" | tail -45

# openrua op 126
grep -n "def move" -A 40 kin.py | head -70; python3 fk.py 2>&1 | grep -v "^\[" | tail -5

# openrua op 127
python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, time
from kin import Robot
r=Robot("vt")
q=np.array(r.arm_q()); print("q", q.round(3))
for dq,secs in ((-0.5,5.0),(0.5,2.0),(-0.5,1.0)):
    t=np.array(r.arm_q()); t[0]+=dq
    t0=time.time(); r.move(t,secs,retries=0); print(f"dq={dq} in {secs}s -> wall {time.time()-t0:.1f}s  q1={r.arm_q()[0]:.3f} target {t[0]:.3f}")
EOF

# openrua op 128
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/kin.py", "old_string": "        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = ARM\\n        pts = []", "new_string": "        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = ARM\\n        pts = []\\n        # controller tracks only ~0.2 rad/s reliably: stretch duration for long moves\\n        dmax = float(np.abs(np.array(self.arm_q()) - np.array(q)).max())\\n        need = dmax / 0.12 + 0.5\\n        if need > seconds:\\n            print(f\\"  move: stretching duration {seconds:.1f}s -> {need:.1f}s (dq={dmax:.2f})\\")\\n            seconds = need", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 129
python3 grasp6.py approach 2>&1 | grep -v "^\[" | tail -50

# openrua op 130
python3 tools/perception/cam_snap.py agentview av15.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview sv5.png >/dev/null 2>&1; python3 fk.py 2>&1 | grep -v "^\[" | tail -4

# openrua op 131
cat ikbest.py

# openrua op 132
python3 - <<'EOF'
s=open('ikbest.py').read()
s=s.replace("HI = np.array([ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973])",
"HI = np.array([ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973])\n# q6 > ~2.9 folds the hand back onto the forearm (self-collision): treat as a hard limit\nHI_M = HI.copy(); HI_M[5] = 2.9")
s=s.replace("np.min(np.minimum(q-LO, HI-q))","np.min(np.minimum(q-LO, HI_M-q))")
open('ikbest.py','w').write(s)
EOF
python3 grasp6.py plan 2>&1 | grep -v "^\[" | tail -7

# openrua op 133
python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
from ikbest import ik_best, margin
def unit(v): v=np.array(v,float); return v/np.linalg.norm(v)
def quat(z,y):
    z=unit(z); y=unit(y); x=np.cross(y,z); return Rot.from_matrix(np.stack([x,y,z],1)).as_quat()
RIM_C=np.array([-0.240,0.079,0.942]); c=unit([-0.964,0.217,0.154])
U=unit(np.array([0,0,1.0])-c[2]*c)
r=Robot("ikb3")
for pitch in (40,50,60):
    P=np.radians(pitch)
    A=unit(-c*np.cos(P)-U*np.sin(P)); SL=unit(-c*np.sin(P)+U*np.cos(P))
    TCP=RIM_C+0.018*(-c)+0.0475*U
    for sgn in (1,-1):
        q=quat(A,sgn*SL)
        bg,sg=ik_best(r,TCP,q,n=20)
        if bg is None: print(pitch,sgn,"no grasp IK"); continue
        # chain along the branch
        out=[]
        for tag,pos in (("pre",TCP-0.10*A),("hi",TCP-0.10*A+[0,0,0.15]),("lift",TCP+[0,0,0.16])):
            s=r.ik(pos,q,seed=bg,timeout=0.5)
            out.append(f"{tag} {margin(s):.2f}/{np.abs(np.array(s)-bg).max():.2f}" if s else f"{tag} none")
        print(f"pitch={pitch} sgn={sgn:+d} grasp m={margin(bg):.2f} {np.round(bg,2)} | "+"  ".join(out))
EOF

# openrua op 134
mkdir -p "$(dirname /workspace/grasp7.py)"
cat > /workspace/grasp7.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Top-down flat pinch on the SIDE wall (-y side) of the cup lying on its side, lift, pitch cup
upright about the pinch axis, perch it on the caddy corner.

Modes: plan | approach | close | lift | rotate | place X Y | retreat
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
from ikbest import ik_best, margin

def unit(v):
    v = np.array(v, float); return v / np.linalg.norm(v)

def quat(z, y):
    z = unit(z); y = unit(y); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()

RIM_C = np.array([-0.240, 0.079, 0.942])
c = unit([-0.964, 0.217, 0.154])                 # cup axis bottom -> rim
n = unit(np.cross(c, [0, 0, 1.0]))               # horizontal, perpendicular to axis (+y side)
ch = unit([c[0], c[1], 0])                       # horizontal projection of axis
T_IN, R_W, DZ = 0.030, 0.047, 0.010               # depth inside rim, wall radius, pinch height above axis
p_ax = RIM_C - T_IN * c                          # axis point at pinch depth
r_h = np.sqrt(R_W**2 - DZ**2)
TCP = p_ax - r_h * n + [0, 0, DZ]                # -y side wall point
DESC = 0.049                                     # descend this far outside the rim (along ch), then insert
QA = quat([0, 0, -1], n)                         # hand down, slide along n
R_UP = Rot.from_rotvec(n * np.arccos(np.clip(c[2], -1, 1)))   # c -> z
QB = (R_UP * Rot.from_quat(QA)).as_quat()        # hand z -> +c (toward -x), slide still along n
CUP_OFF = -r_h * n[:2]                           # TCP_xy - cup centre_xy when upright
LIFT_Z = 1.16

r = Robot("grasp7")

def go(q, secs, tag, retries=3, tol=0.02):
    r.move(q, secs, retries=retries)
    err = np.abs(np.array(r.arm_q()) - np.array(q)).max()
    p = r.report(tag)
    print(f"   joint err after move {err:.4f}")
    if err > tol:
        print("   !!! move did not reach goal"); sys.exit(1)
    return p

mode = sys.argv[1]
r.report("start")
print("n", n.round(3), "TCP", TCP.round(4), "QB hand z", Rot.from_quat(QB).apply([0, 0, 1]).round(3), "CUP_OFF", CUP_OFF.round(4))
if mode == "plan":
    for tag, pos, qt in [("grasp", TCP, QA), ("desc", TCP - DESC * ch, QA), ("hi", TCP - DESC * ch + [0, 0, 0.20], QA),
                         ("lifted", TCP + [0, 0, LIFT_Z - TCP[2]], QA), ("liftedB", TCP + [0, 0, LIFT_Z - TCP[2]], QB),
                         ("perchB", np.array([-0.458, -0.082, 1.16]) + [CUP_OFF[0], CUP_OFF[1], 0], QB)]:
        b, s = ik_best(r, pos, qt, n=12)
        print(tag, "margin", None if b is None else round(margin(b), 2), None if b is None else np.round(b, 2))
elif mode == "approach":
    r.gripper(0.04)
    qg, _ = ik_best(r, TCP, QA, n=30); assert qg is not None
    print("grasp cfg", np.round(qg, 2), "margin", round(margin(qg), 2))
    qd = r.ik(TCP - DESC * ch, QA, seed=qg); assert qd
    qhi = r.ik(TCP - DESC * ch + [0, 0, 0.20], QA, seed=qd); assert qhi
    print("desc", np.round(qd, 2), "hi", np.round(qhi, 2))
    go(qhi, 6.0, "hi", retries=4)
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    seed = qhi
    for dz in (0.14, 0.09, 0.05, 0.03, 0.015, 0.0):        # descend outside the rim
        s = r.ik(TCP - DESC * ch + [0, 0, dz], QA, seed=seed); assert s
        seed = s; r.move(s, 1.2, retries=1)
        dw = r.wrench() - base_w; r.report(f"desc dz={dz:.3f}"); print("   dwrench", dw.round(2))
        if np.abs(dw[:3]).max() > 3.0:
            print("   contact during descent! abort"); sys.exit(1)
    for d in np.arange(DESC - 0.01, -1e-6, -0.01):        # insert along axis
        s = r.ik(TCP - d * ch, QA, seed=seed); assert s
        seed = s; r.move(s, 0.8, retries=1)
        dw = r.wrench() - base_w; r.report(f"insert d={d:.3f}"); print("   dwrench", dw.round(2))
        if np.abs(dw[:3]).max() > 3.0:
            print("   contact during insertion! stopping"); break
    f = r.gripper(0.0)
    print("after close: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4))
elif mode == "lift":
    w0 = r.wrench(); p, qt, _ = r.tcp(); seed = r.arm_q()
    for dz in (0.02, 0.05, 0.10, LIFT_Z - p[2]):
        s = r.ik(p + [0, 0, dz], qt, seed=seed); assert s
        seed = s; r.move(s, 1.5, retries=2)
        r.report(f"lift {dz:.2f}")
        print("   fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
elif mode == "rotate":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    q0 = Rot.from_quat(qt); q1 = Rot.from_quat(QB)
    rv = (q1 * q0.inv()).as_rotvec(); N = 5
    for i in range(1, N + 1):
        qi = (Rot.from_rotvec(rv * i / N) * q0).as_quat()
        s = r.ik(p, qi, seed=seed); assert s, f"IK rot {i}"
        if np.abs(np.array(s) - np.array(seed)).max() > 0.6:
            print("   branch jump; abort"); sys.exit(1)
        seed = s; r.move(s, 2.0, retries=2)
        r.report(f"rot {i}/{N}")
        print("   fingers", np.round(r.fingers(), 4), "wrench", r.wrench().round(2))
elif mode == "place":
    X, Y = float(sys.argv[2]), float(sys.argv[3])
    txy = np.array([X, Y]) + CUP_OFF
    seed = r.arm_q(); base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    s = r.ik((txy[0], txy[1], 1.21), QB, seed=seed); assert s
    go(s, 6.0, "over perch"); seed = s
    z = 1.20
    while z >= 1.10:
        s = r.ik((txy[0], txy[1], z), QB, seed=seed); assert s
        seed = s; r.move(s, 1.0, retries=1)
        w = r.wrench(); dw = w - base_w
        r.report(f"z={z:.3f}")
        print("   dwrench", dw.round(2), "fingers", np.round(r.fingers(), 4))
        if abs(dw[2]) > 1.5 or np.abs(dw[3:5]).max() > 0.4:
            print("   contact"); break
        z -= 0.01
    r.gripper(0.04)
    print("released; fingers", np.round(r.fingers(), 4))
elif mode == "retreat":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    for dz in (0.04, 0.08):
        s = r.ik(p + [0, 0, dz], qt, seed=seed); assert s
        seed = s; r.move(s, 1.5, retries=2); r.report(f"up {dz}")
    s = r.ik(p + [0.12, 0.15, 0.20], qt, seed=seed)
    if s: r.move(s, 3.0, retries=2); r.report("away")
OPENRUA_EOF

# openrua op 135
python3 grasp7.py plan 2>&1 | grep -v "^\[" | tail -7

# openrua op 136
python3 grasp7.py approach 2>&1 | grep -v "^\[\[" | tail -60

# openrua op 137
python3 - <<'EOF' 2>&1 | grep -v "^\[\["
import numpy as np
from kin import Robot
r=Robot("up")
p,qt,_=r.tcp(); s=r.ik(p+[0,0,0.12],qt,seed=r.arm_q()); r.move(s,2.0); r.report("up")
EOF
python3 tools/perception/cam_snap.py birdview bv9.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py agentview av16.png >/dev/null 2>&1

# openrua op 138
python3 park.py 2>&1 | grep -v "^\[\[" | tail -3; for c in agentview birdview frontview sideview; do python3 cloud.py $c >/dev/null 2>&1; done; head -30 axisfit2.py

# openrua op 139
python3 axisfit2.py 2>&1 | tail -20

# openrua op 140
python3 - <<'EOF'
import numpy as np
pts=[]
for c in ["birdview","frontview","agentview","sideview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
    m=(xyz[:,2]>0.92)&(xyz[:,2]<1.0)&(xyz[:,0]>-0.32)&(xyz[:,0]<-0.09)&(xyz[:,1]>-0.0)&(xyz[:,1]<0.14)
    pts.append(xyz[m])
B=np.concatenate(pts)
print("all pts",len(B),"x range",B[:,0].min().round(3),B[:,0].max().round(3))
for z0 in (0.92,0.94,0.96,0.98):
    m=(B[:,2]>z0)&(B[:,2]<z0+0.02); P=B[m]
    if len(P): print(f"z {z0}: xmin {P[:,0].min():.3f} xmax {P[:,0].max():.3f}  at xmin y={P[np.argmin(P[:,0]),1]:.3f}; y range {P[:,1].min():.3f} {P[:,1].max():.3f}")
# rim: points with x < xmin+0.01
P=B[B[:,0]<B[:,0].min()+0.006]
print("rim-ish pts", len(P), "y", P[:,1].min().round(3), P[:,1].max().round(3), "z", P[:,2].min().round(3), P[:,2].max().round(3), "mean", P.mean(0).round(3))
EOF

# openrua op 141
python3 - <<'EOF'
s=open('grasp6.py').read()
s=s.replace('RIM_C = np.array([-0.240, 0.079, 0.942])\nc = unit([-0.964, 0.217, 0.154])',
 'RIM_C = np.array([-0.219, 0.088, 0.943])\nc = unit([-0.964, 0.150, 0.150])')
s=s.replace('PITCH = np.radians(45.0)\nSGN = 1\nINSIDE, WALL_R = 0.018, 0.0475',
 'PITCH = np.radians(float(sys.argv[2]) if len(sys.argv) > 2 and mode_has_pitch else 0.0)\nSGN = 1\nINSIDE, WALL_R = 0.020, 0.0475\nROLL = np.radians(60.0)                             # pinch point rotated from top toward +y side\nS = unit(np.cross(U, -c))                           # side direction (toward +y)\nW = unit(np.sin(ROLL) * S + np.cos(ROLL) * U)       # radial direction of pinch point')
s=s.replace('A = unit(-c * np.cos(PITCH) - U * np.sin(PITCH))     # approach direction\nSL = unit(-c * np.sin(PITCH) + U * np.cos(PITCH))    # inner -> outer finger\nTCP = RIM_C - INSIDE * c + WALL_R * U',
 'A = unit(-c * np.cos(PITCH) - W * np.sin(PITCH))     # approach direction\nSL = unit(-c * np.sin(PITCH) + W * np.cos(PITCH))    # inner -> outer finger\nTCP = RIM_C - INSIDE * c + WALL_R * W')
s=s.replace('H = R_UP.apply(U)  ','H = R_UP.apply(W)  ')
s=s.replace('mode = sys.argv[1]\n','')
s=s.replace('r = Robot("grasp6")','mode = sys.argv[1]\nmode_has_pitch = mode in ("plan", "approach")\nr = Robot("grasp8")')
# move the constants block below mode parsing: simplest - define mode at top
s=s.replace('def unit(v):', 'mode = sys.argv[1]\nmode_has_pitch = mode in ("plan", "approach")\n\ndef unit(v):',1)
s=s.replace('mode = sys.argv[1]\nmode_has_pitch = mode in ("plan", "approach")\nr = Robot("grasp8")','r = Robot("grasp8")')
s=s.replace('"""Pitched rim-wall pinch','"""Rim-wall pinch (approach along cup axis, optional pitch; pinch point ROLL deg from top toward +y)')
open('grasp8.py','w').write(s)
EOF
grep -n "PITCH\|ROLL\|W =\|S =\|mode" grasp8.py | head -20

# openrua op 142
for p in 0 15; do echo "== pitch $p"; python3 grasp8.py plan $p 2>&1 | grep -v "^\[\[" | tail -7; done

# openrua op 143
python3 grasp8.py approach 0 2>&1 | grep -v "^\[\[" | grep -v "stretching" | tail -45

# openrua op 144
python3 - <<'EOF' 2>&1 | grep -v "^\[\["
import numpy as np
from kin import Robot
r=Robot("w"); print("wrench", r.wrench().round(2)); print("q", np.round(r.arm_q(),3))
EOF
python3 tools/perception/cam_snap.py agentview av17.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py frontview fv7.png >/dev/null 2>&1

# openrua op 145
python3 - <<'EOF' 2>&1 | grep -v "^\[\["
import numpy as np
from kin import Robot
r=Robot("fkl"); q=r.arm_q()
for l in ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]:
    p=r.fk(q,l); print(l, np.round(p[0] if isinstance(p,tuple) else p,3))
EOF

# openrua op 146
python3 tools/perception/cam_snap.py birdview bv10.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview sv6.png >/dev/null 2>&1

# openrua op 147
python3 - <<'EOF'
import numpy as np
for c in ["birdview","agentview","frontview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]
    m=(xyz[:,2]>0.95)&(xyz[:,2]<1.08)&(xyz[:,0]>-0.50)&(xyz[:,0]<-0.30)
    P=xyz[m]
    print(c, len(P), "y range", P[:,1].min().round(3), P[:,1].max().round(3), " x range", P[:,0].min().round(3), P[:,0].max().round(3))
    for y0 in np.arange(-0.1,0.2,0.02):
        mm=(P[:,1]>=y0)&(P[:,1]<y0+0.02)
        if mm.sum()>5: print(f"   y {y0:.2f}: n={mm.sum()} zmax={P[mm,2].max():.3f} x[{P[mm,0].min():.3f},{P[mm,0].max():.3f}]")
EOF

# openrua op 148
python3 - <<'EOF'
import numpy as np
d=np.load("birdview_cloud.npz"); xyz=d["xyz"]
m=(xyz[:,0]>-0.49)&(xyz[:,0]<-0.31)&(xyz[:,1]>-0.37)&(xyz[:,1]<0.11)&(xyz[:,2]>0.885)
P=xyz[m]
print("y profile (x in [-0.45,-0.42], back part):")
for y0 in np.arange(-0.36,0.10,0.01):
    mm=(P[:,1]>=y0)&(P[:,1]<y0+0.01)&(P[:,0]>-0.45)&(P[:,0]<-0.42)
    if mm.sum()>2: print(f"  y {y0:+.2f}: zmax={P[mm,2].max():.3f} zmed={np.median(P[mm,2]):.3f} n={mm.sum()}")
print("x profile at y in [-0.15,-0.10]:")
for x0 in np.arange(-0.49,-0.31,0.01):
    mm=(P[:,0]>=x0)&(P[:,0]<x0+0.01)&(P[:,1]>-0.15)&(P[:,1]<-0.10)
    if mm.sum()>2: print(f"  x {x0:.2f}: zmax={P[mm,2].max():.3f} zmed={np.median(P[mm,2]):.3f} n={mm.sum()}")
EOF

# openrua op 149
python3 - <<'EOF'
s=open('grasp8.py').read()
s=s.replace("ROLL = np.radians(60.0)                             # pinch point rotated from top toward +y side",
"ROLL = np.radians(-50.0)                            # pinch point rotated from top toward -y side (away from handle & caddy)")
s=s.replace("RIM_C = np.array([-0.219, 0.088, 0.943])","RIM_C = np.array([-0.219, 0.088, 0.943])  # re-measured after bump")
open('grasp8.py','w').write(s)
EOF
python3 grasp8.py plan 0 2>&1 | grep -v "^\[\[" | tail -7

# openrua op 150
python3 - <<'EOF' 2>&1 | grep -v "^\[\["
import numpy as np, sys
sys.argv=["x","plan","0"]
from scipy.spatial.transform import Rotation as Rot
exec(open('grasp8.py').read().split('r = Robot("grasp8")')[0])
from kin import Robot
r=Robot("chain")
qg,_=ik_best(r,TCP,QA,n=30); print("grasp",np.round(qg,2),round(margin(qg),2))
for LZ in (1.06,1.10):
    seed=qg; ok=True; out=[]
    for dz in (0.03,0.06,LZ-TCP[2]):
        s=r.ik(TCP+[0,0,dz],QA,seed=seed,timeout=0.5)
        if not s: out.append(f"lift{dz:.2f} none"); ok=False; break
        seed=s; out.append(f"lift{dz:.2f} m={margin(s):.2f}")
    if ok:
        q0=Rot.from_quat(QA); rv=(Rot.from_quat(QB)*q0.inv()).as_rotvec()
        for i in range(1,6):
            qi=(Rot.from_rotvec(rv*i/5)*q0).as_quat()
            s=r.ik(TCP+[0,0,LZ-TCP[2]],qi,seed=seed,timeout=0.5)
            if not s: out.append(f"rot{i} none"); break
            jump=np.abs(np.array(s)-np.array(seed)).max(); seed=s; out.append(f"rot{i} m={margin(s):.2f} dq={jump:.2f}")
    print("LZ",LZ," | ".join(out))
    print("   final", np.round(seed,2))
EOF

# openrua op 151
python3 - <<'EOF' 2>&1 | grep -v "^\[\["
import numpy as np, sys
sys.argv=["x","plan","0"]
from scipy.spatial.transform import Rotation as Rot
exec(open('grasp8.py').read().split('r = Robot("grasp8")')[0])
from kin import Robot
r=Robot("scan")
q0=Rot.from_quat(QA); rv=(Rot.from_quat(QB)*q0.inv()).as_rotvec()
qs=[(Rot.from_rotvec(rv*i/5)*q0).as_quat() for i in range(0,6)]
res=[]
for x in (-0.35,-0.28,-0.21):
    for y in (0.0,0.1,0.2):
        for z in (1.05,1.15):
            P=np.array([x,y,z]); b,_=ik_best(r,P,QA,n=10,timeout=0.2)
            if b is None: print(x,y,z,"no start"); continue
            seed=b; mm=margin(b); jumps=0
            for qi in qs[1:]:
                s=r.ik(P,qi,seed=seed,timeout=0.3)
                if not s: mm=-9; break
                if np.abs(np.array(s)-np.array(seed)).max()>0.8: jumps+=1
                seed=s; mm=min(mm,margin(s))
            print(f"{x:.2f} {y:.2f} {z:.2f}: chain min margin {mm:.2f} jumps {jumps} start {np.round(b,2)}")
EOF

# openrua op 152
python3 - <<'EOF' 2>&1 | grep -v "^\[\["
import numpy as np, sys
sys.argv=["x","plan","0"]
from scipy.spatial.transform import Rotation as Rot
exec(open('grasp8.py').read().split('r = Robot("grasp8")')[0])
from kin import Robot
r=Robot("scan2")
QA2=quat(A,-SL); QB2=(R_UP*Rot.from_quat(QA2)).as_quat()
q0=Rot.from_quat(QA2); rv=(Rot.from_quat(QB2)*q0.inv()).as_rotvec()
qs=[(Rot.from_rotvec(rv*i/5)*q0).as_quat() for i in range(0,6)]
for P in ([-0.2005,0.0483,0.9701],[-0.2005,0.0483,1.06],[-0.28,0.1,1.1],[-0.21,0.1,1.05],[-0.21,0.15,1.10],[-0.30,0.15,1.10]):
    P=np.array(P); b,sols=ik_best(r,P,QA2,n=12,timeout=0.2)
    if b is None: print(P,"no start"); continue
    best=None
    for m0,st in sols[:4]:
        seed=st; mm=margin(st); jumps=0
        for qi in qs[1:]:
            s=r.ik(P,qi,seed=seed,timeout=0.3)
            if not s: mm=-9; break
            if np.abs(np.array(s)-np.array(seed)).max()>0.8: jumps+=1
            seed=s; mm=min(mm,margin(s))
        if best is None or mm>best[0]: best=(mm,jumps,st)
    print(f"{P}: best chain min margin {best[0]:.2f} jumps {best[1]} start {np.round(best[2],2)}")
EOF

# openrua op 153
sleep 5; python3 - <<'EOF' 2>&1 | grep -v "^\[\[" | grep -v XMLPARSER
import numpy as np, sys
sys.argv=["x","plan","0"]
from scipy.spatial.transform import Rotation as Rot
exec(open('grasp8.py').read().split('r = Robot("grasp8")')[0])
from kin import Robot
r=Robot("scan2")
QA2=quat(A,-SL); QB2=(R_UP*Rot.from_quat(QA2)).as_quat()
q0=Rot.from_quat(QA2); rv=(Rot.from_quat(QB2)*q0.inv()).as_rotvec()
qs=[(Rot.from_rotvec(rv*i/5)*q0).as_quat() for i in range(0,6)]
for P in ([-0.2005,0.0483,0.9701],[-0.2005,0.0483,1.06],[-0.28,0.1,1.1],[-0.21,0.1,1.05],[-0.21,0.15,1.10],[-0.30,0.15,1.10]):
    P=np.array(P); b,sols=ik_best(r,P,QA2,n=12,timeout=0.2)
    if b is None: print(P,"no start"); continue
    best=None
    for m0,st in sols[:4]:
        seed=st; mm=margin(st); jumps=0
        for qi in qs[1:]:
            s=r.ik(P,qi,seed=seed,timeout=0.3)
            if not s: mm=-9; break
            if np.abs(np.array(s)-np.array(seed)).max()>0.8: jumps+=1
            seed=s; mm=min(mm,margin(s))
        if best is None or mm>best[0]: best=(mm,jumps,st)
    print(f"{P}: best chain min margin {best[0]:.2f} jumps {best[1]} start {np.round(best[2],2)}")
EOF

# openrua op 154
python3 - <<'EOF' 2>&1 | grep -v "^\[\[" | grep -v XMLPARSER
import numpy as np, sys
sys.argv=["x","plan","0"]
from scipy.spatial.transform import Rotation as Rot
exec(open('grasp8.py').read().split('r = Robot("grasp8")')[0])
from kin import Robot
r=Robot("scan3")
results=[]
for sgn in (1,-1):
    QAs=quat(A,sgn*SL)
    for P in ([-0.2005,0.0483,1.05],[-0.25,0.10,1.10],[-0.21,0.15,1.10]):
        P=np.array(P); b,sols=ik_best(r,P,QAs,n=10,timeout=0.2)
        if b is None: continue
        for psi in (-135,-90,-45,0,45,90,135,180):
            Rp=Rot.from_euler('z',psi,degrees=True)*R_UP
            QBs=(Rp*Rot.from_quat(QAs)).as_quat()
            q0=Rot.from_quat(QAs); rv=(Rot.from_quat(QBs)*q0.inv()).as_rotvec()
            qs=[(Rot.from_rotvec(rv*i/5)*q0).as_quat() for i in range(1,6)]
            best=None
            for m0,st in sols[:3]:
                seed=st; mm=margin(st); jumps=0
                for qi in qs:
                    s=r.ik(P,qi,seed=seed,timeout=0.25)
                    if not s: mm=-9; break
                    if np.abs(np.array(s)-np.array(seed)).max()>0.8: jumps+=1
                    seed=s; mm=min(mm,margin(s))
                if best is None or mm>best[0]: best=(mm,jumps,st)
            results.append((best[0],sgn,tuple(P),psi,best[1],np.round(best[2],2),np.degrees(np.linalg.norm(rv)).round(0)))
results.sort(key=lambda t:-t[0])
for t in results[:12]: print(t)
EOF

# openrua op 155
find / -iname "*caddy*" 2>/dev/null | grep -v proc | head; ls /workspace; ls /workspace/docs 2>/dev/null

# openrua op 156
python3 - <<'EOF'
s=open('grasp8.py').read()
s=s.replace('SGN = 1\n','SGN = -1\nPSI = -135.0                                        # extra yaw of the uprighting rotation\nROT_P = np.array([-0.21, 0.15, 1.10])               # TCP position where the cup is uprighted\n')
s=s.replace('R_UP = Rot.from_rotvec(ax * ang)\n','R_UP = Rot.from_euler("z", PSI, degrees=True) * Rot.from_rotvec(ax * ang)\n')
s=s.replace('r = Robot("grasp8")','r = Robot("grasp9")')
# lift mode: lift 3,6 cm then translate to ROT_P
s=s.replace('''    for dz in (0.03, 0.08, 0.14, LIFT_Z - p[2]):
        s = r.ik(p + [0, 0, dz], qt, seed=seed); assert s
        seed = s; r.move(s, 1.5, retries=2)
        r.report(f"lift {dz:.2f}")
        print("   fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))''',
'''    for dz in (0.03, 0.06):
        s = r.ik(p + [0, 0, dz], qt, seed=seed); assert s
        seed = s; r.move(s, 1.5, retries=2)
        r.report(f"lift {dz:.2f}")
        print("   fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
    p1 = p + [0, 0, 0.06]
    for k in range(1, 4):                          # translate to ROT_P in 3 steps
        pk = p1 + (ROT_P - p1) * k / 3
        s = r.ik(pk, qt, seed=seed); assert s, "IK translate"
        if np.abs(np.array(s) - np.array(seed)).max() > 0.7:
            print("   branch jump; abort"); sys.exit(1)
        seed = s; r.move(s, 2.0, retries=2)
        r.report(f"translate {k}/3")
        print("   fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))''')
s=s.replace('rel = q1 * q0.inv(); rv = rel.as_rotvec(); n = 4','rel = q1 * q0.inv(); rv = rel.as_rotvec(); n = 6')
open('grasp9.py','w').write(s)
EOF
python3 grasp9.py plan 0 2>&1 | grep -v "^\[\[" | grep -v XMLPARSER | tail -7

# openrua op 157
python3 - <<'EOF' 2>&1 | grep -v "^\[\[" | grep -v XMLPARSER
import numpy as np, sys
sys.argv=["x","plan","0"]
from scipy.spatial.transform import Rotation as Rot
exec(open('grasp9.py').read().split('r = Robot("grasp9")')[0])
from kin import Robot
r=Robot("yawchk")
QC=(Rot.from_euler('z',180,degrees=True)*Rot.from_quat(QB)).as_quat()
for tag,pos,qt in [("perch H",np.array([-0.458,-0.082,1.16])+[CUP_OFF2[0],CUP_OFF2[1],0],QB),("perch -H",np.array([-0.458,-0.082,1.16])-[CUP_OFF2[0],CUP_OFF2[1],0],QC),("rotP QC",ROT_P,QC)]:
    b,_=ik_best(r,pos,qt,n=12); print(tag, round(margin(b),2) if b is not None else None, np.round(b,2) if b is not None else "")
# yaw chain at ROT_P from QB branch (liftedB best) to QC
b,_=ik_best(r,ROT_P,QB,n=12); seed=b; print("start",np.round(b,2),round(margin(b),2))
for i in range(1,7):
    qi=(Rot.from_euler('z',180*i/6,degrees=True)*Rot.from_quat(QB)).as_quat()
    s=r.ik(ROT_P,qi,seed=seed,timeout=0.4)
    if not s: print("yaw",i,"none"); break
    print("yaw",i,round(margin(s),2),"dq",np.abs(np.array(s)-np.array(seed)).max().round(2)); seed=s
print("final",np.round(seed,2))
# then move from there to perch -H at 1.21
s=r.ik(np.array([-0.458,-0.082,1.21])-[CUP_OFF2[0],CUP_OFF2[1],0],QC,seed=seed,timeout=0.5)
print("perch from chain", None if not s else (round(margin(s),2), np.round(s,2), np.abs(np.array(s)-np.array(seed)).max().round(2)))
EOF

# openrua op 158
python3 - <<'EOF' 2>&1 | grep -v "^\[\[" | grep -v XMLPARSER
import numpy as np, sys
sys.argv=["x","plan","0"]
from scipy.spatial.transform import Rotation as Rot
src=open('grasp9.py').read().split('r = Robot("grasp9")')[0]
from kin import Robot
r=Robot("psiscan")
R0=None
for psi in range(-180,180,45):
    s2=src.replace("PSI = -135.0","PSI = %d.0"%psi)
    g={}; exec(s2,g)
    QB=g["QB"]; CUP=g["CUP_OFF2"]; QA=g["QA"]; R_UP=g["R_UP"]
    pos=np.array([-0.458,-0.082,1.16])+[CUP[0],CUP[1],0]
    b,_=ik_best(r,pos,QB,n=10,timeout=0.2)
    pm=None if b is None else round(margin(b),2)
    # rotation chain at a few ROT_P
    best=(-9,None)
    for P in ([-0.21,0.15,1.10],[-0.25,0.10,1.10],[-0.30,0.15,1.12],[-0.21,0.05,1.10]):
        P=np.array(P); bb,sols=ik_best(r,P,QA,n=8,timeout=0.2)
        if bb is None: continue
        q0=Rot.from_quat(QA); rv=(Rot.from_quat(QB)*q0.inv()).as_rotvec()
        for m0,st in sols[:3]:
            seed=st; mm=margin(st)
            for i in range(1,7):
                qi=(Rot.from_rotvec(rv*i/6)*q0).as_quat()
                s=r.ik(P,qi,seed=seed,timeout=0.25)
                if not s or np.abs(np.array(s)-np.array(seed)).max()>0.8: mm=-9; break
                seed=s; mm=min(mm,margin(s))
            if mm>best[0]: best=(mm,tuple(P),np.round(st,2),np.round(seed,2))
    print(f"psi={psi:4d} H={np.round(g['H'][:2],2)} perch m={pm} | chain m={best[0]:.2f} at {best[1]} start {best[2]} end {best[3]}")
EOF

# openrua op 159
python3 - <<'EOF' 2>&1 | grep -v "^\[\[" | grep -v XMLPARSER
import numpy as np, sys
sys.argv=["x","plan","0"]
from scipy.spatial.transform import Rotation as Rot
from ikbest import ik_best, margin
src=open('grasp9.py').read().split('r = Robot("grasp9")')[0]
from kin import Robot
r=Robot("psiscan")
for psi in range(-180,180,45):
    s2=src.replace("PSI = -135.0","PSI = %d.0"%psi)
    g={}; exec(s2,g)
    QB=g["QB"]; CUP=g["CUP_OFF2"]; QA=g["QA"]
    pos=np.array([-0.458,-0.082,1.16])+[CUP[0],CUP[1],0]
    b,_=ik_best(r,pos,QB,n=10,timeout=0.2)
    pm=None if b is None else round(margin(b),2)
    best=(-9,None,None,None)
    for P in ([-0.21,0.15,1.10],[-0.25,0.10,1.10],[-0.30,0.15,1.12],[-0.21,0.05,1.10]):
        P=np.array(P); bb,sols=ik_best(r,P,QA,n=8,timeout=0.2)
        if bb is None: continue
        q0=Rot.from_quat(QA); rv=(Rot.from_quat(QB)*q0.inv()).as_rotvec()
        for m0,st in sols[:3]:
            seed=st; mm=margin(st)
            for i in range(1,7):
                qi=(Rot.from_rotvec(rv*i/6)*q0).as_quat()
                s=r.ik(P,qi,seed=seed,timeout=0.25)
                if not s or np.abs(np.array(s)-np.array(seed)).max()>0.8: mm=-9; break
                seed=s; mm=min(mm,margin(s))
            if mm>best[0]: best=(mm,tuple(P),np.round(st,2),np.round(seed,2))
    print(f"psi={psi:4d} H={np.round(g['H'][:2],2)} perch m={pm} | chain m={best[0]:.2f} at {best[1]} start {best[2]} end {best[3]}")
EOF

# openrua op 160
python3 grasp9.py approach 0 2>&1 | grep -v "^\[\[" | grep -v "stretching\|XMLPARSER" | tail -40

# openrua op 161
python3 - <<'EOF' 2>&1 | grep -v "^\[\[" | grep -v XMLPARSER
import numpy as np
from kin import Robot
r=Robot("dbg"); q=np.array(r.arm_q()); pre=np.array([1.7,1.07,-0.96,-2.62,-2.34,2.29,0.79])
print("wrench", r.wrench().round(2)); print("q   ", q.round(3)); print("pre ", pre); print("diff", (q-pre).round(3))
for l in ["panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]:
    p=r.fk(q,l); print(l, np.round(p[0] if isinstance(p,tuple) else p,3))
EOF
python3 tools/perception/cam_snap.py agentview av18.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview sv7.png >/dev/null 2>&1

# openrua op 162
mkdir -p "$(dirname /workspace/grasp10.py)"
cat > /workspace/grasp10.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Rim-wall pinch of the cup lying on its side, approaching obliquely (THETA deg from the cup axis toward
the wall tangent) so the wrist stays clear of the caddy; then lift, translate, upright the cup and perch it.

Modes: plan [psi] | approach | lift | rotate | place X Y | retreat
Geometry: cup axis c (bottom->rim), rim centre RIM_C.  Pinch point on the rim wall at ROLL deg from the top
(toward -y for negative), INSIDE m inside the rim.  Slide axis = radial W (pads flat on the wall).
"""
import sys, json
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
from ikbest import ik_best, margin

def unit(v):
    v = np.array(v, float); return v / np.linalg.norm(v)

def quat(z, y):
    z = unit(z); y = unit(y); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()

mode = sys.argv[1]
RIM_C = np.array([-0.219, 0.088, 0.943])
c = unit([-0.964, 0.150, 0.150])
U = unit(np.array([0, 0, 1.0]) - c[2] * c)          # up, perpendicular to c
S = unit(np.cross(U, -c))                           # side (+y)
ROLL, THETA = np.radians(-50.0), np.radians(50.0)
INSIDE, WALL_R = 0.018, 0.0475
PSI = float(sys.argv[2]) if mode == "plan" and len(sys.argv) > 2 else json.load(open("grasp10.json"))["psi"] if mode != "plan" else 0.0
ROT_P = np.array([-0.24, 0.12, 1.12])               # where the cup is uprighted
W = unit(np.sin(ROLL) * S + np.cos(ROLL) * U)       # radial direction of pinch point (slide axis)
T = unit(np.cross(W, c))                            # tangent at pinch point
if np.dot(T, S) < 0: T = -T                         # choose tangent going toward +y / up
A = unit(np.cos(THETA) * (-c) - np.sin(THETA) * T)  # approach: into cup, drifting so wrist goes up/+y
SL = W
TCP = RIM_C - INSIDE * c + WALL_R * W
QA = quat(A, SL)
ax = unit(np.cross(c, [0, 0, 1.0])); ang = np.arccos(np.clip(c[2], -1, 1))
R_UP = Rot.from_euler("z", PSI, degrees=True) * Rot.from_rotvec(ax * ang)
QB = (R_UP * Rot.from_quat(QA)).as_quat()
H = R_UP.apply(W)                                   # horizontal: TCP = cup centre + WALL_R*H when upright
CUP_OFF2 = WALL_R * H[:2]
BACK, STEP = 0.10, 0.01
PERCH = np.array([-0.458, -0.082])

r = Robot("grasp10")

def go(q, secs, tag, retries=3, tol=0.02):
    r.move(q, secs, retries=retries)
    err = np.abs(np.array(r.arm_q()) - np.array(q)).max()
    p = r.report(tag)
    print(f"   joint err after move {err:.4f}   wrench {r.wrench().round(1)}")
    if err > tol:
        print("   !!! move did not reach goal"); sys.exit(1)
    return p

def chain(seed, steps, label):
    """IK along a list of (pos, quat); returns list of solutions or None; prints margins."""
    out = []
    for i, (pos, qt) in enumerate(steps):
        s = r.ik(pos, qt, seed=seed, timeout=0.4)
        if not s:
            print(f"   {label}[{i}] IK fail"); return None
        jump = np.abs(np.array(s) - np.array(seed)).max()
        if jump > 0.8:
            print(f"   {label}[{i}] branch jump {jump:.2f}"); return None
        out.append(s); seed = s
    print(f"   {label}: min margin {min(margin(s) for s in out):.2f}")
    return out

def rot_steps(p, q_from, q_to, n=6):
    q0 = Rot.from_quat(q_from); rv = (Rot.from_quat(q_to) * q0.inv()).as_rotvec()
    return [(p, (Rot.from_rotvec(rv * i / n) * q0).as_quat()) for i in range(1, n + 1)]

r.report("start")
print("A", A.round(3), "SL", SL.round(3), "TCP", TCP.round(4), "| QB hand z", Rot.from_quat(QB).apply([0, 0, 1]).round(3),
      "H", H.round(3), "psi", PSI)
if mode == "plan":
    qg, sols = ik_best(r, TCP, QA, n=20)
    for m0, st in sols[:4]:
        print("grasp cfg", np.round(st, 2), "margin", round(m0, 2))
        pre = chain(st, [(TCP - BACK * A, QA), (TCP - BACK * A + [0, 0, 0.12], QA)], "pre/hi")
        lift = chain(st, [(TCP + [0, 0, 0.03], QA), (TCP + [0, 0, 0.06], QA)] +
                     [(TCP + [0, 0, 0.06] + (ROT_P - TCP - [0, 0, 0.06]) * k / 3, QA) for k in (1, 2, 3)], "lift/translate")
        if lift is None: continue
        rot = chain(lift[-1], rot_steps(ROT_P, QA, QB), "rotate")
        if rot is None: continue
        perch_tcp = np.array([PERCH[0] + CUP_OFF2[0], PERCH[1] + CUP_OFF2[1], 1.21])
        pl = chain(rot[-1], [(ROT_P + (perch_tcp - ROT_P) * k / 3, QB) for k in (1, 2, 3)] +
                   [(perch_tcp - [0, 0, dz], QB) for dz in (0.04, 0.08, 0.11)], "to perch/lower")
        if pl is not None:
            print("   >>> full chain OK for this grasp cfg; saving")
            json.dump({"psi": PSI, "qg": list(map(float, st))}, open("grasp10.json", "w"))
            break
elif mode == "approach":
    cfg = json.load(open("grasp10.json")); qg = cfg["qg"]
    r.gripper(0.04)
    qpre = r.ik(TCP - BACK * A, QA, seed=qg); assert qpre
    qhi = r.ik(TCP - BACK * A + [0, 0, 0.12], QA, seed=qpre); assert qhi
    print("grasp", np.round(qg, 2), "pre", np.round(qpre, 2), "hi", np.round(qhi, 2))
    p0, qt0, _ = r.tcp()
    ql = r.ik(p0 + [0, 0, max(0, 1.20 - p0[2])], qt0, seed=r.arm_q())
    if ql: go(ql, 4.0, "lift-current")
    go(qhi, 8.0, "hi", retries=4)
    go(qpre, 4.0, "pre")
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    seed = qpre
    for d in np.arange(BACK - STEP, -1e-6, -STEP):
        s = r.ik(TCP - d * A, QA, seed=seed); assert s, f"IK d={d}"
        if np.abs(np.array(s) - np.array(seed)).max() > 0.4:
            print("   branch jump, abort"); sys.exit(1)
        seed = s
        r.move(s, 0.8, retries=1)
        dw = r.wrench() - base_w
        r.report(f"d={d:.3f}")
        print("   dwrench", dw.round(2))
        if np.abs(dw[:3]).max() > 3.0:
            print("   contact! stopping approach"); break
    f = r.gripper(0.0)
    print("after close: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4))
elif mode == "lift":
    w0 = r.wrench(); p, qt, _ = r.tcp(); seed = r.arm_q()
    steps = [(p + [0, 0, 0.03], qt), (p + [0, 0, 0.06], qt)] + \
            [(p + [0, 0, 0.06] + (ROT_P - p - [0, 0, 0.06]) * k / 3, qt) for k in (1, 2, 3)]
    sols = chain(seed, steps, "lift/translate"); assert sols
    for i, s in enumerate(sols):
        r.move(s, 2.0, retries=2); r.report(f"lift step {i}")
        print("   fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
elif mode == "rotate":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    sols = chain(seed, rot_steps(p, qt, QB), "rotate"); assert sols
    for i, s in enumerate(sols):
        r.move(s, 2.5, retries=2); r.report(f"rot {i + 1}")
        print("   fingers", np.round(r.fingers(), 4), "wrench", r.wrench().round(2))
elif mode == "place":
    X, Y = float(sys.argv[2]), float(sys.argv[3])
    txy = np.array([X, Y]) + CUP_OFF2
    p, qt, _ = r.tcp(); seed = r.arm_q()
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    top = np.array([txy[0], txy[1], 1.21])
    sols = chain(seed, [(p + (top - p) * k / 3, QB) for k in (1, 2, 3)], "to perch"); assert sols
    for i, s in enumerate(sols):
        go(s, 3.0, f"to perch {i + 1}")
    seed = sols[-1]; z = 1.20
    while z >= 1.09:
        s = r.ik((txy[0], txy[1], z), QB, seed=seed); assert s
        seed = s; r.move(s, 1.0, retries=1)
        w = r.wrench(); dw = w - base_w
        r.report(f"z={z:.3f}")
        print("   dwrench", dw.round(2), "fingers", np.round(r.fingers(), 4))
        if abs(dw[2]) > 1.5 or np.abs(dw[3:5]).max() > 0.4:
            print("   contact"); break
        z -= 0.01
    r.gripper(0.04)
    print("released; fingers", np.round(r.fingers(), 4))
elif mode == "retreat":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    for dz in (0.04, 0.08):
        s = r.ik(p + [0, 0, dz], qt, seed=seed); assert s
        seed = s; r.move(s, 1.5, retries=2); r.report(f"up {dz}")
    s = r.ik(p + [0.12, 0.15, 0.20], qt, seed=seed)
    if s: r.move(s, 3.0, retries=2); r.report("away")
OPENRUA_EOF

# openrua op 163
for psi in 0 -90 90 180 -45 45; do echo "=== psi $psi"; python3 grasp10.py plan $psi 2>&1 | grep -v "^\[\[" | grep -v XMLPARSER | tail -n +2; test -f grasp10.json && { echo "saved: $(cat grasp10.json)"; break; }; done

# openrua op 164
python3 grasp10.py approach 2>&1 | grep -v "^\[\[" | grep -v "stretching\|XMLPARSER" | tail -42

# openrua op 165
python3 - <<'EOF' 2>&1 | grep -v "^\[\[" | grep -v "XMLPARSER\|stretching"
import numpy as np
from kin import Robot
r=Robot("liftchk"); w0=r.wrench(); p,qt,_=r.tcp()
s=r.ik(p+[0,0,0.03],qt,seed=r.arm_q()); r.move(s,1.5); r.report("up3")
print("fingers", np.round(r.fingers(),4), "dwrench", (r.wrench()-w0).round(2))
EOF
python3 tools/perception/cam_snap.py agentview av19.png >/dev/null 2>&1

# openrua op 166
timeout 600 python3 grasp10.py lift 2>&1 | grep -v "^\s*move attempt" | tail -30

# openrua op 167
timeout 600 python3 grasp10.py rotate 2>&1 | grep -v "^\s*move attempt" | tail -20; python3 tools/perception/cam_snap.py agentview av20.png >/dev/null 2>&1; echo done

# openrua op 168
python3 tools/perception/cam_snap.py frontview fv8.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview sv8.png >/dev/null 2>&1; echo ok

# openrua op 169
timeout 900 python3 grasp10.py place -0.458 -0.082 2>&1 | grep -v "^\s*move attempt" | tail -30

# openrua op 170
cat > backoff.py <<'EOF'
import numpy as np
from kin import Robot
r = Robot("backoff")
p, qt, _ = r.tcp(); seed = r.arm_q(); r.report("start"); w0 = r.wrench()
for d in ([0.02, 0.01, 0.0], [0.03, 0.02, 0.03], [0.03, 0.02, 0.07], [0.03, 0.02, 0.10]):
    s = r.ik(p + d, qt, seed=seed); assert s
    seed = s; r.move(s, 1.5, retries=1); r.report(f"back {d}")
    print("   fingers", np.round(r.fingers(), 4), "wrench", r.wrench().round(2))
EOF
timeout 300 python3 backoff.py 2>&1 | grep -v "move attempt"; python3 tools/perception/cam_snap.py frontview fv9.png >/dev/null 2>&1

# openrua op 171
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/grasp10.py", "old_string": "    top = np.array([txy[0], txy[1], 1.21])\\n    sols = chain(seed, [(p + (top - p) * k / 3, QB) for k in (1, 2, 3)], \\"to perch\\"); assert sols\\n    for i, s in enumerate(sols):\\n        go(s, 3.0, f\\"to perch {i + 1}\\")\\n    seed = sols[-1]; z = 1.20", "new_string": "    top = np.array([txy[0], txy[1], max(p[2], 1.24)])   # translate high: cup bottom must clear caddy top (1.056)\\n    sols = chain(seed, [(p + (top - p) * k / 3, QB) for k in (1, 2, 3)], \\"to perch\\"); assert sols\\n    for i, s in enumerate(sols):\\n        go(s, 3.0, f\\"to perch {i + 1}\\")\\n    seed = sols[-1]; z = top[2] - 0.02", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 172
timeout 900 python3 grasp10.py place -0.458 -0.082 2>&1 | grep -v "^\s*move attempt" | tail -30

# openrua op 173
timeout 300 python3 grasp10.py retreat 2>&1 | grep -v "^\s*move attempt" | tail -5; for c in agentview frontview birdview; do python3 tools/perception/cam_snap.py $c ${c}_21.png >/dev/null 2>&1; done; echo ok

# openrua op 174
python3 cloud.py birdview >/dev/null 2>&1; python3 cloud.py agentview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
for cam in ("birdview","agentview"):
    d=np.load(f"{cam}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(float)
    if rgb.max()>1.5: rgb/=255
    # cup: white or yellow, in caddy area, above caddy floor
    m=(xyz[:,0]>-0.50)&(xyz[:,0]<-0.30)&(xyz[:,1]>-0.25)&(xyz[:,1]<0.10)&(xyz[:,2]>0.90)
    r,g,b=rgb[:,0],rgb[:,1],rgb[:,2]
    yellow=(r>0.5)&(g>0.4)&(b<0.45)&(r-b>0.25)
    white=(r>0.7)&(g>0.7)&(b>0.7)
    cup=m&(yellow|white)
    p=xyz[cup]
    print(cam, "cup pts", len(p))
    if len(p):
        print("  x", p[:,0].min().round(3), p[:,0].max().round(3), " y", p[:,1].min().round(3), p[:,1].max().round(3), " z", p[:,2].min().round(3), p[:,2].max().round(3))
        print("  centroid", p.mean(0).round(3))
        # percentiles
        for a,n in enumerate("xyz"): print("  ",n, np.percentile(p[:,a],[5,50,95]).round(3))
EOF

# openrua op 175
timeout 300 python3 park.py 2>&1 | grep -v "move attempt" | tail -3; for c in agentview birdview frontview sideview; do python3 cloud.py $c >/dev/null 2>&1; done; python3 - <<'EOF'
import numpy as np
P=[];C=[]
for cam in ("birdview","agentview","frontview","sideview"):
    d=np.load(f"{cam}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(float)
    if rgb.max()>1.5: rgb/=255
    m=(xyz[:,0]>-0.50)&(xyz[:,0]<-0.28)&(xyz[:,1]>-0.25)&(xyz[:,1]<0.10)&(xyz[:,2]>0.90)
    r,g,b=rgb[:,0],rgb[:,1],rgb[:,2]
    yellow=(r>0.5)&(g>0.4)&(b<0.45)&(r-b>0.25); white=(r>0.7)&(g>0.7)&(b>0.7)
    cup=m&(yellow|white); print(cam,cup.sum()); P.append(xyz[cup]); C.append(rgb[cup])
p=np.vstack(P); c=np.vstack(C)
np.savez("cup_now.npz",xyz=p,rgb=c)
print("all",len(p)); 
for a,n in enumerate("xyz"): print(n, np.percentile(p[:,a],[2,50,98]).round(3))
# PCA axis
mu=p.mean(0); u,s,vt=np.linalg.svd(p-mu,full_matrices=False); print("centroid",mu.round(3),"axis",vt[0].round(3),"sv",s.round(2))
# slices along x
for x0 in np.arange(-0.45,-0.33,0.01):
    q=p[(p[:,0]>=x0)&(p[:,0]<x0+0.01)]
    if len(q)>3: print(f"x {x0:.2f} n={len(q):3d} y[{q[:,1].min():.3f},{q[:,1].max():.3f}] z[{q[:,2].min():.3f},{q[:,2].max():.3f}]")
EOF

# openrua op 176
python3 - <<'EOF'
import numpy as np
for cam in ("birdview","agentview","sideview","frontview"):
    d=np.load(f"{cam}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(float)
    if rgb.max()>1.5: rgb/=255
    m=(xyz[:,0]>-0.50)&(xyz[:,0]<-0.25)&(xyz[:,1]>-0.25)&(xyz[:,1]<0.10)&(xyz[:,2]>0.90)
    r,g,b=rgb[:,0],rgb[:,1],rgb[:,2]
    yellow=(r>0.5)&(g>0.4)&(b<0.45)&(r-b>0.25); white=(r>0.7)&(g>0.7)&(b>0.7)
    p=xyz[m&(yellow|white)]; col=np.where(yellow[m&(yellow|white)],"Y","W")
    print("==",cam,len(p))
    for x0 in np.arange(-0.46,-0.32,0.01):
        q=p[(p[:,0]>=x0)&(p[:,0]<x0+0.01)]; cc=col[(p[:,0]>=x0)&(p[:,0]<x0+0.01)]
        if len(q)>2: print(f" x {x0:.2f} n={len(q):3d} Y={np.sum(cc=='Y'):3d} y[{q[:,1].min():.3f},{q[:,1].max():.3f}] z[{q[:,2].min():.3f},{q[:,2].max():.3f}]")
EOF

# openrua op 177
cat > lookdown.py <<'EOF'
import sys, numpy as np
from kin import Robot
r = Robot("look")
x, y, z = map(float, sys.argv[1:4])
q = r.ik([x, y, z], [1, 0, 0, 0], seed=r.arm_q()) or r.ik([x, y, z], [1, 0, 0, 0], seed=[0,-0.785,0,-2.356,0,1.571,0.785])
assert q; r.move(q, 4.0); r.report("look")
EOF
timeout 300 python3 lookdown.py -0.40 -0.09 1.36 2>&1 | grep -v "move attempt"; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png 2>&1 | tail -1

# openrua op 178
python3 -c "
from PIL import Image
im=Image.open('birdview_21.png').crop((200,140,360,220)).resize((640,320),Image.LANCZOS); im.save('bv_crop.png')
im=Image.open('agentview_21.png').crop((200,140,380,260)).resize((720,480),Image.LANCZOS); im.save('av_crop.png')
"

# openrua op 179
mkdir -p "$(dirname /workspace/grasp11.py)"
cat > /workspace/grasp11.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Second grasp: cup lies on the caddy's B/C divider, rim toward +x.  Pinch the rim wall on the +y side just
below horizontal (below the handle), approaching THETA deg from the cup axis toward the up-tangent so the hand is a
horizontal bar in front of the caddy and the wrist stays high.  Lift, yaw the cup by PSI about z (rim -> -y,
handle -> +x), lower it into the B channel until contact, release.

Modes: plan | approach | lift | rotate | place X Y | retreat
"""
import sys, json
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
from ikbest import ik_best, margin

def unit(v):
    v = np.array(v, float); return v / np.linalg.norm(v)

def quat(z, y):
    z = unit(z); y = unit(y); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()

mode = sys.argv[1]
RIM_C = np.array([-0.357, -0.110, 1.040])
c = unit([0.90, -0.35, -0.25])                       # cup axis bottom -> rim
U = unit(np.array([0, 0, 1.0]) - c[2] * c)          # up, perpendicular to c
Y = unit(np.array([0, 1.0, 0]) - c[1] * c)          # +y, perpendicular to c
ROLL, THETA = np.radians(100.0), np.radians(45.0)   # pinch angle from top toward +y; approach tilt
INSIDE, WALL_R = 0.018, 0.0475
PSI = -90.0                                         # yaw applied to the cup after lifting
ROT_P = np.array([-0.33, -0.10, 1.20])              # where the cup is yawed
W = unit(np.sin(ROLL) * Y + np.cos(ROLL) * U)       # radial direction of pinch point (slide axis)
T = unit(np.cross(W, c))
if T[2] < 0: T = -T                                 # tangent going up
A = unit(np.cos(THETA) * (-c) - np.sin(THETA) * T)  # approach: into the cup, coming from above
SL = W
TCP = RIM_C - INSIDE * c + WALL_R * W
CENTRE = RIM_C - 0.055 * c
QA = quat(A, SL)
R_Z = Rot.from_euler("z", PSI, degrees=True)
QB = (R_Z * Rot.from_quat(QA)).as_quat()
OFF2 = R_Z.apply(TCP - CENTRE)                      # TCP - cup centre after the yaw
BACK, STEP = 0.10, 0.01
TARGET = np.array([-0.435, -0.135])                 # cup centre xy in the B channel

r = Robot("grasp11")

def go(q, secs, tag, retries=3, tol=0.02):
    r.move(q, secs, retries=retries)
    err = np.abs(np.array(r.arm_q()) - np.array(q)).max()
    p = r.report(tag)
    print(f"   joint err after move {err:.4f}   wrench {r.wrench().round(1)}")
    if err > tol:
        print("   !!! move did not reach goal"); sys.exit(1)
    return p

def chain(seed, steps, label):
    out = []
    for i, (pos, qt) in enumerate(steps):
        s = r.ik(pos, qt, seed=seed, timeout=0.4)
        if not s:
            print(f"   {label}[{i}] IK fail"); return None
        jump = np.abs(np.array(s) - np.array(seed)).max()
        if jump > 0.8:
            print(f"   {label}[{i}] branch jump {jump:.2f}"); return None
        out.append(s); seed = s
    print(f"   {label}: min margin {min(margin(s) for s in out):.2f}")
    return out

def rot_steps(p, q_from, q_to, n=6):
    q0 = Rot.from_quat(q_from); rv = (Rot.from_quat(q_to) * q0.inv()).as_rotvec()
    return [(p, (Rot.from_rotvec(rv * i / n) * q0).as_quat()) for i in range(1, n + 1)]

def lift_steps(p, qt):
    return [(p + [0, 0, 0.04], qt), (p + [0, 0, 0.08], qt), (p + [0, 0, 0.12], qt)] + \
           [(p + [0, 0, 0.12] + (ROT_P - p - [0, 0, 0.12]) * k / 2, qt) for k in (1, 2)]

r.report("start")
print("A", A.round(3), "SL", SL.round(3), "TCP", TCP.round(4), "centre", CENTRE.round(4),
      "| QB hand z", Rot.from_quat(QB).apply([0, 0, 1]).round(3), "OFF2", OFF2.round(4))
if mode == "plan":
    qg, sols = ik_best(r, TCP, QA, n=20)
    for m0, st in sols[:5]:
        print("grasp cfg", np.round(st, 2), "margin", round(m0, 2))
        pre = chain(st, [(TCP - BACK * A, QA), (TCP - BACK * A + [0, 0, 0.12], QA)], "pre/hi")
        if pre is None: continue
        lift = chain(st, lift_steps(TCP, QA), "lift/translate")
        if lift is None: continue
        rot = chain(lift[-1], rot_steps(ROT_P, QA, QB), "rotate")
        if rot is None: continue
        top = np.array([TARGET[0] + OFF2[0], TARGET[1] + OFF2[1], ROT_P[2]])
        pl = chain(rot[-1], [(ROT_P + (top - ROT_P) * k / 3, QB) for k in (1, 2, 3)] +
                   [(top - [0, 0, dz], QB) for dz in (0.04, 0.08, 0.12, 0.15)], "to target/lower")
        if pl is not None:
            print("   >>> full chain OK for this grasp cfg; saving")
            json.dump({"qg": list(map(float, st))}, open("grasp11.json", "w"))
            break
elif mode == "approach":
    qg = json.load(open("grasp11.json"))["qg"]
    r.gripper(0.04)
    qpre = r.ik(TCP - BACK * A, QA, seed=qg); assert qpre
    qhi = r.ik(TCP - BACK * A + [0, 0, 0.12], QA, seed=qpre); assert qhi
    print("grasp", np.round(qg, 2), "pre", np.round(qpre, 2), "hi", np.round(qhi, 2))
    go(qhi, 8.0, "hi", retries=4)
    go(qpre, 4.0, "pre")
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    seed = qpre
    for d in np.arange(BACK - STEP, -1e-6, -STEP):
        s = r.ik(TCP - d * A, QA, seed=seed); assert s, f"IK d={d}"
        if np.abs(np.array(s) - np.array(seed)).max() > 0.4:
            print("   branch jump, abort"); sys.exit(1)
        seed = s
        r.move(s, 0.8, retries=1)
        dw = r.wrench() - base_w
        r.report(f"d={d:.3f}")
        print("   dwrench", dw.round(2))
        if np.abs(dw[:3]).max() > 3.0:
            print("   contact! stopping approach"); break
    f = r.gripper(0.0)
    print("after close: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4))
elif mode == "lift":
    w0 = r.wrench(); p, qt, _ = r.tcp(); seed = r.arm_q()
    sols = chain(seed, lift_steps(p, qt), "lift/translate"); assert sols
    for i, s in enumerate(sols):
        r.move(s, 2.0, retries=2); r.report(f"lift step {i}")
        print("   fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
elif mode == "rotate":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    sols = chain(seed, rot_steps(p, qt, QB), "rotate"); assert sols
    for i, s in enumerate(sols):
        r.move(s, 2.5, retries=2); r.report(f"rot {i + 1}")
        print("   fingers", np.round(r.fingers(), 4), "wrench", r.wrench().round(2))
elif mode == "place":
    X, Y2 = float(sys.argv[2]), float(sys.argv[3])
    txy = np.array([X, Y2]) + OFF2[:2]
    p, qt, _ = r.tcp(); seed = r.arm_q()
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    top = np.array([txy[0], txy[1], p[2]])
    sols = chain(seed, [(p + (top - p) * k / 3, QB) for k in (1, 2, 3)], "to target"); assert sols
    for i, s in enumerate(sols):
        go(s, 3.0, f"to target {i + 1}")
    seed = sols[-1]; z = top[2] - 0.02
    while z >= 1.00:
        s = r.ik((txy[0], txy[1], z), QB, seed=seed); assert s
        seed = s; r.move(s, 1.0, retries=1)
        w = r.wrench(); dw = w - base_w
        r.report(f"z={z:.3f}")
        print("   dwrench", dw.round(2), "fingers", np.round(r.fingers(), 4))
        if np.abs(dw[:3]).max() > 1.5 or np.abs(dw[3:5]).max() > 0.4:
            print("   contact"); break
        z -= 0.01
    r.gripper(0.04)
    print("released; fingers", np.round(r.fingers(), 4))
elif mode == "retreat":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    for dz in (0.04, 0.08):
        s = r.ik(p + [0, 0, dz], qt, seed=seed); assert s
        seed = s; r.move(s, 1.5, retries=2); r.report(f"up {dz}")
    s = r.ik(p + [0.12, 0.15, 0.20], qt, seed=seed)
    if s: r.move(s, 3.0, retries=2); r.report("away")
OPENRUA_EOF

# openrua op 180
timeout 600 python3 grasp11.py plan 2>&1 | grep -v "move attempt" | tail -30

# openrua op 181
timeout 900 python3 grasp11.py approach 2>&1 | grep -v "move attempt" | tail -32

# openrua op 182
python3 tools/perception/cam_snap.py agentview av22.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview sv22.png >/dev/null 2>&1; python3 -c "
from PIL import Image
Image.open('av22.png').crop((180,100,480,300)).resize((900,600),Image.LANCZOS).save('av22c.png')"

# openrua op 183
python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png >/dev/null 2>&1; echo ok

# openrua op 184
cat > nudge.py <<'EOF'
import sys, numpy as np
from kin import Robot
r = Robot("nudge")
d = np.array(list(map(float, sys.argv[1:4])))
p, qt, _ = r.tcp(); w0 = r.wrench()
s = r.ik(p + d, qt, seed=r.arm_q()); assert s
r.move(s, 1.5, retries=1); r.report("nudged")
print("fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
EOF
timeout 120 python3 nudge.py 0 0 0.03 2>&1 | grep -v "move attempt"; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py agentview av23.png >/dev/null 2>&1

# openrua op 185
python3 -c "
from PIL import Image
Image.open('av23.png').crop((180,100,480,300)).resize((900,600),Image.LANCZOS).save('av23c.png')"; python3 tools/perception/cam_snap.py birdview bv23.png >/dev/null 2>&1; python3 -c "
from PIL import Image
Image.open('bv23.png').crop((180,120,400,260)).resize((880,560),Image.LANCZOS).save('bv23c.png')"

# openrua op 186
python3 - <<'EOF' 2>&1 | grep -v "move attempt"
import numpy as np
from kin import Robot
r = Robot("chk"); q = r.arm_q(); print("q", np.round(q,3)); print("wrench", r.wrench().round(2))
for l in ("panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"):
    p,_ = r.fk(q, l); print(l, p.round(3))
EOF
python3 tools/perception/cam_snap.py frontview fv23.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview sv23.png >/dev/null 2>&1

# openrua op 187
timeout 300 python3 nudge.py 0.05 0 0.10 2>&1 | grep -v "move attempt" | tail -2; timeout 300 python3 lookdown.py -0.40 -0.09 1.36 2>&1 | grep -v "move attempt"; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png >/dev/null 2>&1; python3 -c "
from PIL import Image
Image.open('eih4.png').crop((180,180,500,360)).resize((960,540),Image.LANCZOS).save('eih4c.png')"

# openrua op 188
python3 - <<'EOF'
import re
s = open("grasp11.py").read()
s = s.replace('RIM_C = np.array([-0.357, -0.110, 1.040])', 'RIM_C = np.array([-0.349, -0.118, 1.040])')
s = s.replace('c = unit([0.90, -0.35, -0.25])                       # cup axis bottom -> rim',
              'c = unit([0.96, -0.07, 0.26])                        # cup axis bottom -> rim (rim tilted up ~15 deg)')
s = s.replace('ROLL, THETA = np.radians(100.0), np.radians(45.0)   # pinch angle from top toward +y; approach tilt',
              'ROLL, THETA = np.radians(-25.0), np.radians(0.0)    # pinch 25 deg from top toward -y (handle is at +y); approach along axis')
s = s.replace('ROT_P = np.array([-0.33, -0.10, 1.20])', 'ROT_P = np.array([-0.36, -0.13, 1.21])')
s = s.replace('TARGET = np.array([-0.435, -0.135])', 'TARGET = np.array([-0.44, -0.135])')
open("grasp11.py", "w").write(s)
EOF
grep -n "RIM_C =\|^c = \|ROLL, THETA\|ROT_P =\|TARGET =" grasp11.py

# openrua op 189
timeout 600 python3 grasp11.py plan 2>&1 | grep -v "move attempt" | tail -30

# openrua op 190
timeout 900 python3 grasp11.py approach 2>&1 | grep -v "move attempt" | tail -34

# openrua op 191
timeout 120 python3 nudge.py 0 0 0.03 2>&1 | grep -v "move attempt"; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih5.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py agentview av24.png >/dev/null 2>&1

# openrua op 192
timeout 600 python3 grasp11.py lift 2>&1 | grep -v "move attempt" | grep -A1 "lift step" | grep fingers; timeout 600 python3 grasp11.py rotate 2>&1 | grep -v "move attempt" | grep -A1 "^\[rot" | tail -8; python3 tools/perception/cam_snap.py frontview fv25.png >/dev/null 2>&1

# openrua op 193
timeout 900 python3 grasp11.py place -0.44 -0.135 2>&1 | grep -v "move attempt" | tail -24

# openrua op 194
timeout 300 python3 grasp11.py retreat 2>&1 | grep "^\[" | tail -3; timeout 300 python3 park.py 2>&1 | grep "^\[parked"; for c in agentview birdview frontview; do python3 tools/perception/cam_snap.py $c ${c}_26.png >/dev/null 2>&1; done; echo ok

# openrua op 195
for c in agentview birdview frontview sideview; do python3 cloud.py $c >/dev/null 2>&1; done; python3 - <<'EOF'
import numpy as np
P=[]
for cam in ("birdview","agentview","frontview","sideview"):
    d=np.load(f"{cam}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(float)
    if rgb.max()>1.5: rgb/=255
    m=(xyz[:,0]>-0.50)&(xyz[:,0]<-0.25)&(xyz[:,1]>-0.25)&(xyz[:,1]<0.10)&(xyz[:,2]>0.90)
    r,g,b=rgb[:,0],rgb[:,1],rgb[:,2]
    yellow=(r>0.5)&(g>0.4)&(b<0.45)&(r-b>0.25); white=(r>0.6)&(g>0.6)&(b>0.6)&(abs(r-b)<0.15)
    p=xyz[m&(yellow|white)]; print(cam,len(p)); P.append(p)
p=np.vstack(P)
for a,n in enumerate("xyz"): print(n, np.percentile(p[:,a],[2,50,98]).round(3))
for x0 in np.arange(-0.48,-0.30,0.01):
    q=p[(p[:,0]>=x0)&(p[:,0]<x0+0.01)]
    if len(q)>3: print(f" x {x0:.2f} n={len(q):3d} y[{q[:,1].min():.3f},{q[:,1].max():.3f}] z[{q[:,2].min():.3f},{q[:,2].max():.3f}]")
for z0 in np.arange(0.90,1.15,0.02):
    q=p[(p[:,2]>=z0)&(p[:,2]<z0+0.02)]
    if len(q)>3: print(f" z {z0:.2f} n={len(q):3d} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]")
EOF

# openrua op 196
timeout 300 python3 lookdown.py -0.40 -0.09 1.36 2>&1 | tail -5 && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih6.png 2>&1 | tail -2

# openrua op 197
timeout 120 python3 cloud.py robot0_eye_in_hand 2>&1 | tail -3; python3 - <<'EOF'
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); print(d.files)
P=d[d.files[0]] if d[d.files[0]].ndim==2 and d[d.files[0]].shape[1]==3 else None
for k in d.files: print(k, d[k].shape)
EOF

# openrua op 198
python3 - <<'EOF'
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); P=d['xyz']; C=d['rgb']
# cup points: within caddy footprint, above compartment floors, not caddy colour (dark brown), z>0.95
m=(P[:,0]>-0.50)&(P[:,0]<-0.20)&(P[:,1]>-0.30)&(P[:,1]<0.0)&(P[:,2]>0.90)
Q=P[m]; col=C[m]
# caddy is dark: brightness<80 ; cup is white/yellow bright
br=col.astype(float).sum(1)
cup=Q[br>200]
print("cup n",len(cup))
for a,n in zip(range(3),'xyz'): print(n, np.percentile(cup[:,a],[1,10,50,90,99]).round(3))
# slice by z
for z0 in np.arange(0.95,1.15,0.02):
    s=cup[(cup[:,2]>=z0)&(cup[:,2]<z0+0.02)]
    if len(s): print(f"z {z0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].mean():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].mean():.3f},{s[:,1].max():.3f}]")
# yellow vs white
yel=cup[(col[br>200][:,2]<150)]; wh=cup[(col[br>200][:,2]>=150)]
print("yellow mean",yel.mean(0).round(3),len(yel)," white mean",wh.mean(0).round(3),len(wh))
EOF

# openrua op 199
python3 - <<'EOF'
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); P=d['xyz']; C=d['rgb'].astype(float)
m=(P[:,0]>-0.50)&(P[:,0]<-0.20)&(P[:,1]>-0.30)&(P[:,1]<0.0)&(P[:,2]>0.90)&(P[:,2]<1.2)
Q=P[m]; col=C[m]
white=(col.min(1)>140)
yellow=(col[:,0]>140)&(col[:,1]>110)&(col[:,2]<110)
print("caddy colour sample", np.median(col[(Q[:,2]>1.05)&(Q[:,0]<-0.45)],0))
for name,sel in (("white",white),("yellow",yellow)):
    s=Q[sel]; print(name,len(s))
    for a,n in zip(range(3),'xyz'): print("  ",n, np.percentile(s[:,a],[1,10,50,90,99]).round(3))
cup=Q[white|yellow]
for z0 in np.arange(0.95,1.13,0.02):
    s=cup[(cup[:,2]>=z0)&(cup[:,2]<z0+0.02)]
    if len(s): print(f"z {z0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].mean():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].mean():.3f},{s[:,1].max():.3f}]")
np.save('cup_pts6.npy',cup)
EOF

# openrua op 200
python3 - <<'EOF'
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); P=d['xyz']; C=d['rgb'].astype(float); UV=d['uv']
print(UV.min(0),UV.max(0))
m=(UV[:,0]>=300)&(UV[:,0]<=400)&(UV[:,1]>=180)&(UV[:,1]<=280)&(P[:,2]<1.2)
Q=P[m]; col=C[m]
br=col.sum(1)
print(np.percentile(br,[5,25,50,75,95]))
cup=Q[br>120]
print("n",len(cup))
for z0 in np.arange(0.93,1.13,0.02):
    s=cup[(cup[:,2]>=z0)&(cup[:,2]<z0+0.02)]
    if len(s): print(f"z {z0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].mean():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].mean():.3f},{s[:,1].max():.3f}]")
for x0 in np.arange(-0.42,-0.26,0.02):
    s=cup[(cup[:,0]>=x0)&(cup[:,0]<x0+0.02)]
    if len(s): print(f"x {x0:.2f}: n={len(s):4d} z[{s[:,2].min():.3f},{s[:,2].mean():.3f},{s[:,2].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].mean():.3f},{s[:,1].max():.3f}]")
np.save('cup_pts6.npy',cup)
EOF

# openrua op 201
find / -iname "*desk_caddy*" 2>/dev/null | head; find / -iname "*libero*" -maxdepth 4 2>/dev/null | head; ls /workspace

# openrua op 202
python3 - <<'EOF'
import numpy as np
cup=np.load('cup_pts6.npy')
top=cup[cup[:,2]>1.078]; print("top arc", len(top), top.mean(0).round(4), top.min(0).round(3), top.max(0).round(3))
fr=cup[cup[:,0]>-0.285]; fr=fr[fr[:,2]>0.99]; print("front arc", len(fr), fr.mean(0).round(4), fr.min(0).round(3), fr.max(0).round(3))
# side extremes in y at z>1.0
s=cup[cup[:,2]>1.0]
print("y range at z>1", np.percentile(s[:,1],[0.5,99.5]).round(3))
# handle: points at y< -0.175
h=cup[(cup[:,1]<-0.172)&(cup[:,2]>0.99)]; print("handle?", len(h), h.mean(0).round(3), h.min(0).round(3), h.max(0).round(3))
EOF

# openrua op 203
python3 - <<'EOF'
import re
s=open('grasp11.py').read()
s=s.replace('"""Second grasp','"""Third grasp: cup stands in slot C tilted ~38 deg toward +x (rim up-front), handle at -y.  Pinch the rim wall 30 deg\nfrom the top toward +y (opposite the handle), lift, rotate so the axis is along -y with the handle pointing DOWN (keel into\nthe B channel), lower into B until contact, release.\n\nModes: plan | approach | close | lift | rotate | place X Y | retreat\n\nOld docstring: Second grasp',1)
s=s.replace('RIM_C = np.array([-0.349, -0.118, 1.040])','RIM_C = np.array([-0.314, -0.119, 1.055])')
s=s.replace('c = unit([0.96, -0.07, 0.26])                        # cup axis bottom -> rim (rim tilted up ~15 deg)',
            'c = unit([0.61, 0.0, 0.79])                          # cup axis bottom -> rim (tilted ~38 deg toward +x)')
s=s.replace('ROLL, THETA = np.radians(-25.0), np.radians(0.0)    # pinch 25 deg from top toward -y (handle is at +y); approach along axis',
            'ROLL, THETA = np.radians(30.0), np.radians(0.0)     # pinch 30 deg from top toward +y (handle is at -y); approach along axis')
s=s.replace('PSI = -90.0                                         # yaw applied to the cup after lifting\n','')
s=s.replace('ROT_P = np.array([-0.36, -0.13, 1.21])              # where the cup is yawed','ROT_P = np.array([-0.36, -0.13, 1.23])              # where the cup is re-oriented')
s=s.replace('''R_Z = Rot.from_euler("z", PSI, degrees=True)
QB = (R_Z * Rot.from_quat(QA)).as_quat()
OFF2 = R_Z.apply(TCP - CENTRE)                      # TCP - cup centre after the yaw''',
'''H0 = np.array([0, -1.0, 0]); A1 = np.array([0, -1.0, 0]); H1 = np.array([0, 0, -1.0])   # handle dir now / axis & handle after
F0 = np.stack([c, H0, np.cross(c, H0)], 1); F1 = np.stack([A1, H1, np.cross(A1, H1)], 1)
R_C = Rot.from_matrix(F1 @ F0.T)                    # cup re-orientation: axis -> -y, handle -> down
print("re-orientation angle", np.degrees(R_C.magnitude()).round(1), "W after", R_C.apply(W).round(3))
QB = (R_C * Rot.from_quat(QA)).as_quat()
OFF2 = R_C.apply(TCP - CENTRE)                      # TCP - cup centre after re-orientation''')
s=s.replace('TARGET = np.array([-0.44, -0.135])                 # cup centre xy in the B channel','TARGET = np.array([-0.43, -0.135])                 # cup centre xy in the B channel')
s=s.replace('r = Robot("grasp11")','r = Robot("grasp12")')
s=s.replace('grasp11.json','grasp12.json')
s=s.replace('def rot_steps(p, q_from, q_to, n=6):','def rot_steps(p, q_from, q_to, n=8):')
# approach: don't close; new close mode
s=s.replace('''        if np.abs(dw[:3]).max() > 3.0:
            print("   contact! stopping approach"); break
    f = r.gripper(0.0)
    print("after close: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4))
elif mode == "lift":''','''        if np.abs(dw[:3]).max() > 3.0:
            print("   contact! stopping approach"); break
elif mode == "close":
    f = r.gripper(0.0)
    print("after close: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4))
elif mode == "lift":''')
# place: lower until contact, then continue while force moderate down to ZMIN
s=s.replace('''    seed = sols[-1]; z = top[2] - 0.02
    while z >= 1.00:
        s = r.ik((txy[0], txy[1], z), QB, seed=seed); assert s
        seed = s; r.move(s, 1.0, retries=1)
        w = r.wrench(); dw = w - base_w
        r.report(f"z={z:.3f}")
        print("   dwrench", dw.round(2), "fingers", np.round(r.fingers(), 4))
        if np.abs(dw[:3]).max() > 1.5 or np.abs(dw[3:5]).max() > 0.4:
            print("   contact"); break
        z -= 0.01''','''    seed = sols[-1]; z = top[2] - 0.02
    ZMIN = 1.065 + OFF2[2]                          # cup centre no lower than 1.065 (wrist over the left wall)
    while z >= ZMIN - 1e-6:
        s = r.ik((txy[0], txy[1], z), QB, seed=seed); assert s
        seed = s; r.move(s, 1.0, retries=1)
        w = r.wrench(); dw = w - base_w
        r.report(f"z={z:.3f} (cup centre z={z - OFF2[2]:.3f})")
        print("   dwrench", dw.round(2), "fingers", np.round(r.fingers(), 4))
        if np.abs(dw[:3]).max() > 6.0 or np.abs(dw[3:5]).max() > 1.5:
            print("   firm contact, stop"); break
        z -= 0.01''')
open('grasp12.py','w').write(s)
EOF
grep -n "RIM_C\|^c =\|ROLL\|R_C\|ZMIN\|TARGET" grasp12.py | head -20

# openrua op 204
timeout 900 python3 grasp12.py plan 2>&1 | grep -v "^\[INFO\]" | tail -40

# openrua op 205
sed -i 's/ROLL, THETA = np.radians(30.0), np.radians(0.0)     # pinch 30 deg from top toward +y (handle is at -y); approach along axis/ROLL, THETA = np.radians(30.0), np.radians(-20.0)   # pinch 30 deg from top toward +y (handle is at -y); approach tilted 20 deg from above/' grasp12.py && sed -i 's/TARGET = np.array(\[-0.43, -0.135\])/TARGET = np.array([-0.425, -0.135])/; s/ZMIN = 1.065 + OFF2\[2\]                          # cup centre no lower than 1.065 (wrist over the left wall)/ZMIN = 1.07 + OFF2[2]                           # cup centre no lower than 1.07/; s/if np.abs(dw\[:3\]).max() > 6.0 or np.abs(dw\[3:5\]).max() > 1.5:/if np.abs(dw[:3]).max() > 4.0 or np.abs(dw[3:5]).max() > 1.0:/' grasp12.py && grep -n "THETA =\|TARGET =\|ZMIN =\|> 4.0" grasp12.py && timeout 900 python3 grasp12.py plan 2>&1 | grep -v "^\[INFO\]" | tail -30

# openrua op 206
cat > chk12.py <<'EOF'
import sys, json, numpy as np
sys.argv=[sys.argv[0],"noop"]
exec(open('grasp12.py').read().split('r.report("start")')[0])
from ikbest import LO, HI_M
qg=json.load(open('grasp12.json'))["qg"]
lift = chain(qg, lift_steps(TCP, QA), "lift"); rot = chain(lift[-1], rot_steps(ROT_P, QA, QB), "rotate")
top = np.array([TARGET[0] + OFF2[0], TARGET[1] + OFF2[1], ROT_P[2]])
pl = chain(rot[-1], [(ROT_P + (top - ROT_P) * k / 3, QB) for k in (1, 2, 3)] + [(top - [0, 0, dz], QB) for dz in (0.04, 0.08, 0.12, 0.15)], "place")
for name,ch in (("rot",rot),("pl",pl)):
    for q in ch:
        q=np.array(q); m=np.minimum(q-LO,HI_M-q); print(name, q.round(2), "min", m.min().round(2), "joint", m.argmin()+1)
EOF
timeout 600 python3 chk12.py 2>&1 | grep -v INFO | tail -20

# openrua op 207
cat > chk12b.py <<'EOF'
import sys, json, numpy as np
sys.argv=[sys.argv[0],"noop"]
exec(open('grasp12.py').read().split('r.report("start")')[0])
from ikbest import LO, HI_M
top = np.array([TARGET[0] + OFF2[0], TARGET[1] + OFF2[1], ROT_P[2]])
for tag,pos in (("low",top-[0,0,0.15]),("rotp",ROT_P)):
    b,sols=ik_best(r,pos,QB,n=40)
    print(tag, [(round(m,2), np.round(q,2).tolist()) for m,q in sols[:6]])
EOF
timeout 600 python3 chk12b.py 2>&1 | grep -v INFO | tail -4

# openrua op 208
cat > chk12c.py <<'EOF'
import sys, json, numpy as np
sys.argv=[sys.argv[0],"noop"]
exec(open('grasp12.py').read().split('r.report("start")')[0])
b,sols=ik_best(r,TCP,QA,n=60)
for m,q in sols[:12]: print(round(m,2), np.round(q,2).tolist())
EOF
timeout 600 python3 chk12c.py 2>&1 | grep -v INFO | tail -12

# openrua op 209
python3 - <<'EOF'
s=open('grasp12.py').read()
s=s.replace('''    qg, sols = ik_best(r, TCP, QA, n=20)
    for m0, st in sols[:5]:''','''    qg, sols = ik_best(r, TCP, QA, n=40)
    for m0, st in sols[:8]:''')
s=s.replace('''        if pl is not None:
            print("   >>> full chain OK for this grasp cfg; saving")''','''        if pl is not None and min(margin(q) for q in pre + lift + rot + pl) > 0.15:
            print("   >>> full chain OK for this grasp cfg; saving")''')
open('grasp12.py','w').write(s)
EOF
timeout 900 python3 grasp12.py plan 2>&1 | grep -v "^\[INFO\]" | tail -30

# openrua op 210
timeout 1200 python3 grasp12.py approach 2>&1 | grep -v "^\[INFO\]\|move attempt\|stretching" | tail -30

# openrua op 211
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih9.png 2>&1 | tail -1; timeout 120 python3 tools/perception/cam_snap.py agentview av27.png 2>&1 | tail -1

# openrua op 212
timeout 300 python3 grasp12.py close 2>&1 | grep -v "^\[INFO\]" | tail -3

# openrua op 213
timeout 900 python3 grasp12.py lift 2>&1 | grep -v "^\[INFO\]\|move attempt\|stretching" | tail -12

# openrua op 214
timeout 120 python3 tools/perception/cam_snap.py agentview av28.png 2>&1 | tail -1

# openrua op 215
timeout 900 python3 grasp12.py rotate 2>&1 | grep -v "^\[INFO\]\|move attempt\|stretching" | tail -18

# openrua op 216
timeout 120 python3 tools/perception/cam_snap.py agentview av29.png 2>&1 | tail -1

# openrua op 217
timeout 120 python3 cloud.py agentview 2>&1 | tail -1; timeout 120 python3 cloud.py birdview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
for cam in ("agentview","birdview"):
    d=np.load(f'{cam}_cloud.npz'); P=d['xyz']; C=d['rgb'].astype(float)
    m=(P[:,2]>1.10)&(P[:,2]<1.27)&(P[:,0]>-0.45)&(P[:,0]<-0.28)&(P[:,1]>-0.25)&(P[:,1]<-0.03)
    Q=P[m]; col=C[m]
    white=(col.min(1)>150); yellow=(col[:,0]>140)&(col[:,1]>110)&(col[:,2]<110)
    cup=Q[white|yellow]
    print(cam, len(cup))
    for a,n in zip(range(3),'xyz'): print("  ",n, np.percentile(cup[:,a],[0.5,5,50,95,99.5]).round(4))
    # y extent per z band
    for z0 in np.arange(1.12,1.27,0.02):
        s=cup[(cup[:,2]>=z0)&(cup[:,2]<z0+0.02)]
        if len(s)>5: print(f"   z{z0:.2f} n={len(s)} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
EOF

# openrua op 218
python3 - <<'EOF'
import numpy as np
for cam in ("agentview","birdview"):
    d=np.load(f'{cam}_cloud.npz'); P=d['xyz']; C=d['rgb'].astype(float)
    m=(P[:,2]>1.08)&(P[:,2]<1.30)&(P[:,0]>-0.45)&(P[:,0]<-0.20)&(P[:,1]>-0.30)&(P[:,1]<0.05)
    Q=P[m]; col=C[m]
    yellow=(col[:,0]>120)&(col[:,1]>90)&(col[:,2]<100)&(col[:,0]-col[:,2]>60)
    cup=Q[yellow]
    print(cam, len(cup))
    for a,n in zip(range(3),'xyz'): print("  ",n, np.percentile(cup[:,a],[0.5,5,50,95,99.5]).round(4))
    for z0 in np.arange(1.10,1.30,0.02):
        s=cup[(cup[:,2]>=z0)&(cup[:,2]<z0+0.02)]
        if len(s)>5: print(f"   z{z0:.2f} n={len(s)} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
EOF

# openrua op 219
sed -i 's/ZMIN = 1.07 + OFF2\[2\]                           # cup centre no lower than 1.07/ZMIN = 1.045 + OFF2[2]                          # cup centre no lower than 1.045 (settled wedge height 1.036)/' grasp12.py && grep -n "ZMIN =" grasp12.py && timeout 900 python3 grasp12.py place -0.42 -0.143 2>&1 | grep -v "^\[INFO\]\|move attempt\|stretching" | tail -40

# openrua op 220
timeout 120 python3 tools/perception/cam_snap.py agentview av30.png 2>&1 | tail -1

# openrua op 221
timeout 600 python3 grasp12.py retreat 2>&1 | grep -v "^\[INFO\]\|move attempt\|stretching" | tail -4 && timeout 300 python3 park.py 2>&1 | tail -1 && timeout 120 python3 tools/perception/cam_snap.py birdview bv31.png | tail -1 && timeout 120 python3 tools/perception/cam_snap.py agentview av31.png | tail -1
