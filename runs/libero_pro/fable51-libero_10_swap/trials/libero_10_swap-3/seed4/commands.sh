#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | head -50; echo ---; ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 60 python3 tools/perception/px2world.py birdview 340 250; timeout 60 python3 tools/perception/px2world.py agentview 360 208; timeout 60 python3 tools/perception/px2world.py birdview 280 265; timeout 60 python3 tools/perception/px2world.py birdview 215 265

# openrua op 6
cat > tfq.py <<'EOF'
import sys, rclpy
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = rclpy.create_node("tfq"); b = Buffer(); TransformListener(b, n)
pairs = [a.split(":") for a in sys.argv[1:]]
import time
for _ in range(50):
    rclpy.spin_once(n, timeout_sec=0.1)
for a, c in pairs:
    try:
        t = b.lookup_transform(a, c, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(a, "->", c, f"{tr.x:.4f} {tr.y:.4f} {tr.z:.4f} | q {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}")
    except Exception as e:
        print(a, "->", c, "ERR", e)
print(b.all_frames_as_string())
EOF
timeout 60 python3 tfq.py world:panda_link0 world:panda_hand panda_link0:panda_hand world:birdview_optical_frame world:agentview_optical_frame

# openrua op 7
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png; ros2 topic echo /birdview/color/camera_info --once | grep -A3 "^k:"; python3 -c "
import numpy as np
d=np.load('bird_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
# table z: pixel far from objects
for (u,v) in [(100,400),(500,400),(340,250),(320,300),(280,265),(215,265),(300,240)]:
    print((u,v), d[v,u], 'world z', 3.0-d[v,u])
"

# openrua op 8
cat > birdmap.py <<'EOF'
import numpy as np
d=np.load('bird_depth.npy'); fx=fy=579.4112549695428; cx=320; cy=240
v,u=np.mgrid[0:480,0:640]
Xc=(u-cx)*d/fx; Yc=(v-cy)*d/fy
wx=Yc-0.2; wy=Xc; wz=3.0-d
np.save('bird_wx.npy',wx); np.save('bird_wy.npy',wy); np.save('bird_wz.npy',wz)
def report(name,mask):
    if mask.sum()==0: print(name,'empty'); return
    print(f"{name}: n={mask.sum()} x[{wx[mask].min():.3f},{wx[mask].max():.3f}] y[{wy[mask].min():.3f},{wy[mask].max():.3f}] z[{wz[mask].min():.3f},{wz[mask].max():.3f}] centroid x={wx[mask].mean():.3f} y={wy[mask].mean():.3f}")
# bowl: near (340,250) region, z 0.93-1.0
reg=(abs(u-340)<45)&(abs(v-250)<45)
report('bowl rim', reg&(wz>0.95)&(wz<1.0))
report('bowl any', reg&(wz>0.905)&(wz<1.0))
# cabinet region: u<310, v 200-330
creg=(u<310)&(v>190)&(v<340)
report('cabinet top >1.1', creg&(wz>1.1))
report('cabinet 1.0-1.1', creg&(wz>1.0)&(wz<1.1))
report('drawer front 0.95-1.0', creg&(wz>0.95)&(wz<1.0))
report('drawer bottom 0.91-0.95', creg&(wz>0.91)&(wz<0.95))
# print z histogram in cabinet region
zs=wz[creg]; h,e=np.histogram(zs,bins=np.arange(0.89,1.2,0.01))
for hh,ee in zip(h,e): 
    if hh: print(f"{ee:.2f} {hh}")
EOF
python3 birdmap.py

# openrua op 9
cat > bowl.py <<'EOF'
import numpy as np, cv2
wx=np.load('bird_wx.npy'); wy=np.load('bird_wy.npy'); wz=np.load('bird_wz.npy')
m=((wz>0.905)&(wz<1.0)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    mk=lab==i
    if stats[i,4]<50: continue
    print(f"comp {i}: n={stats[i,4]} px bbox u{stats[i,0]}-{stats[i,0]+stats[i,2]} v{stats[i,1]}-{stats[i,1]+stats[i,3]} x[{wx[mk].min():.3f},{wx[mk].max():.3f}] y[{wy[mk].min():.3f},{wy[mk].max():.3f}] z[{wz[mk].min():.3f},{wz[mk].max():.3f}] cen x={wx[mk].mean():.3f} y={wy[mk].mean():.3f}")
EOF
python3 bowl.py

# openrua op 10
cat > cammap.py <<'EOF'
"""Build world-coordinate maps for a camera's current depth frame: <cam>_wx/wy/wz.npy"""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
cam=sys.argv[1]
rclpy.init(); n=rclpy.create_node("cammap"); b=Buffer(); TransformListener(b,n)
got={}
n.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault('d',m),1)
n.create_subscription(Image,f"/{cam}/color/image_raw",lambda m:got.setdefault('c',m),1)
n.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault('i',m),1)
while not all(k in got for k in 'dci') or not b.can_transform('world',f'{cam}_optical_frame',rclpy.time.Time()):
    rclpy.spin_once(n,timeout_sec=0.2)
d=CvBridge().imgmsg_to_cv2(got['d'],'passthrough').astype(np.float64)
c=CvBridge().imgmsg_to_cv2(got['c'],'bgr8')
import cv2; cv2.imwrite(f'{cam}.png',c)
k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
t=b.lookup_transform('world',f'{cam}_optical_frame',rclpy.time.Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
H,W=d.shape; v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)
Wp=P@R.T+T
np.save(f'{cam}_wx.npy',Wp[...,0]); np.save(f'{cam}_wy.npy',Wp[...,1]); np.save(f'{cam}_wz.npy',Wp[...,2])
print(cam, 'cam pos',T, 'saved', d.shape)
rclpy.shutdown()
EOF
timeout 120 python3 cammap.py agentview && timeout 120 python3 cammap.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np, cv2
for cam in ['agentview','robot0_eye_in_hand']:
    wx=np.load(f'{cam}_wx.npy'); wy=np.load(f'{cam}_wy.npy'); wz=np.load(f'{cam}_wz.npy')
    m=((wz>0.93)&(wz<1.01)&(wx>-0.4)&(wx<0.1)&(wy>-0.05)&(wy<0.25)).astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(m)
    for i in range(1,n):
        mk=lab==i
        if stats[i,4]<100: continue
        print(f"{cam} comp {i}: n={stats[i,4]} bbox u{stats[i,0]}-{stats[i,0]+stats[i,2]} v{stats[i,1]}-{stats[i,1]+stats[i,3]} x[{wx[mk].min():.3f},{wx[mk].max():.3f}] y[{wy[mk].min():.3f},{wy[mk].max():.3f}] z[{wz[mk].min():.3f},{wz[mk].max():.3f}] cen x={wx[mk].mean():.3f} y={wy[mk].mean():.3f}")
EOF

# openrua op 11
python3 - <<'EOF'
import numpy as np
wx=np.load('bird_wx.npy'); wy=np.load('bird_wy.npy'); wz=np.load('bird_wz.npy')
# drawer region: y in [-0.24,-0.03], x in [-0.30,0.05]
reg=(wy>-0.25)&(wy<-0.03)&(wx>-0.30)&(wx<0.05)
# for each z band list extents
for lo,hi in [(0.905,0.93),(0.93,0.96),(0.96,0.99),(0.99,1.02),(1.02,1.05),(1.05,1.08),(1.08,1.11),(1.11,1.2)]:
    m=reg&(wz>=lo)&(wz<hi)
    if m.sum(): print(f"z[{lo},{hi}) n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}]")
# cross-section along y at x=-0.11 (drawer center), print z as function of y
print('--- profile along y at x=-0.11')
m=(abs(wx+0.11)<0.004)&(wy>-0.45)&(wy<0.0)
ys=wy[m]; zs=wz[m]; o=np.argsort(ys)
for yy,zz in zip(ys[o][::3],zs[o][::3]): print(f"y={yy:.3f} z={zz:.3f}")
print('--- profile along x at y=-0.14')
m=(abs(wy+0.14)<0.004)&(wx>-0.35)&(wx<0.1)
xs=wx[m]; zs=wz[m]; o=np.argsort(xs)
for xx,zz in zip(xs[o][::3],zs[o][::3]): print(f"x={xx:.3f} z={zz:.3f}")
EOF

# openrua op 12
python3 - <<'EOF'
import numpy as np
cam='robot0_eye_in_hand'
wx=np.load(f'{cam}_wx.npy'); wy=np.load(f'{cam}_wy.npy'); wz=np.load(f'{cam}_wz.npy')
reg=(wx>-0.23)&(wx<-0.09)&(wy>0.0)&(wy<0.15)
print('max z in bowl region', wz[reg].max())
for lo,hi in [(0.90,0.92),(0.92,0.94),(0.94,0.96),(0.96,0.98),(0.98,1.0),(1.0,1.02),(1.02,1.1)]:
    m=reg&(wz>=lo)&(wz<hi)
    if m.sum(): print(f"z[{lo},{hi}) n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}]")
# profile along y through center x=-0.162
m=reg&(abs(wx+0.162)<0.002)
ys=wy[m]; zs=wz[m]; o=np.argsort(ys)
for yy,zz in zip(ys[o][::2],zs[o][::2]): print(f"y={yy:.3f} z={zz:.3f}")
EOF

# openrua op 13
timeout 120 python3 cammap.py sideview; python3 - <<'EOF'
import numpy as np
for cam in ['agentview','sideview']:
    wx=np.load(f'{cam}_wx.npy'); wy=np.load(f'{cam}_wy.npy'); wz=np.load(f'{cam}_wz.npy')
    reg=(wx>-0.23)&(wx<-0.09)&(wy>0.0)&(wy<0.15)&(wz<1.1)
    print(cam,'bowl region z range',wz[reg].min(),wz[reg].max())
    for lo,hi in [(0.90,0.92),(0.92,0.94),(0.94,0.96),(0.96,0.98),(0.98,1.0),(1.0,1.02),(1.02,1.04),(1.04,1.1)]:
        m=reg&(wz>=lo)&(wz<hi)
        if m.sum(): print(f"  z[{lo},{hi}) n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}]")
EOF

# openrua op 14
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 15
timeout 120 python3 cammap.py agentview; python3 - <<'EOF'
import numpy as np
cam='agentview'
wx=np.load(f'{cam}_wx.npy'); wy=np.load(f'{cam}_wy.npy'); wz=np.load(f'{cam}_wz.npy')
reg=(wx>-0.25)&(wx<-0.07)&(wy>-0.02)&(wy<0.17)&(wz<1.1)
print(cam,'bowl region z range',wz[reg].min(),wz[reg].max())
for lo,hi in np.arange(0.90,1.04,0.01)[:,None]*[1,1]+[0,0.01]:
    m=reg&(wz>=lo)&(wz<hi)
    if m.sum(): print(f"  z[{lo:.2f},{hi:.2f}) n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}]")
EOF

# openrua op 16
timeout 600 python3 tools/action/ik_move.py -0.210 0.0735 1.03 0.7071068 0.7071068 0 0 4 --at tcp; timeout 60 python3 tfq.py world:panda_hand 2>/dev/null | head -1

# openrua op 17
mkdir -p "$(dirname /workspace/move.py)"
cat > /workspace/move.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Move the FINGERTIP point (TCP) to a world pose: IK -> trajectory -> verify.

Usage: python3 move.py <x> <y> <z> <qx> <qy> <qz> <qw> [seconds=4]
Prints the resulting hand/TCP pose from TF and the joint tracking error.
"""
import sys
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    a = [float(v) for v in sys.argv[1:]]
    x, y, z, qx, qy, qz, qw = a[:7]
    secs = a[7] if len(a) > 7 else 4.0
    M = yaml.safe_load(open("/workspace/machine.yaml"))
    tj = next(e for e in M["actuators"] if e["kind"] == "joint_trajectory")
    off = M["hand"]["tcp_offset_m"]
    base = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (TF, identity rotation)
    R = quat_R(qx, qy, qz, qw)
    hand_w = np.array([x, y, z]) - off * R[:, 2]
    hand_b = hand_w - base  # IK is in the arm base frame

    rclpy.init()
    node = rclpy.create_node("mover")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cli = node.create_client(GetPositionIK, M["planning"]["ik_service"])
    cli.wait_for_service(timeout_sec=10)
    req = GetPositionIK.Request()
    req.ik_request.group_name = M["planning"]["group"]
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = hand_b
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    seed = JointState()
    cur = dict(zip(js["m"].name, js["m"].position))
    for j in tj["joints"]:
        seed.name.append(j); seed.position.append(cur[j])
    req.ik_request.robot_state.joint_state = seed
    req.ik_request.avoid_collisions = False
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    res = fut.result()
    if res is None or res.error_code.val != 1:
        print("IK FAILED", None if res is None else res.error_code.val); sys.exit(2)
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    target = [sol[j] for j in tj["joints"]]
    print("IK ok:", " ".join(f"{v:.3f}" for v in target))

    ac = ActionClient(node, FollowJointTrajectory, tj["port"])
    ac.wait_for_server(timeout_sec=10)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = tj["joints"]
    pt = JointTrajectoryPoint(positions=target)
    pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
    goal.trajectory.points = [pt]
    send = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    rf = send.result().get_result_async()
    rclpy.spin_until_future_complete(node, rf)
    print("traj error_code", rf.result().result.error_code)

    # verify
    js.pop("m", None)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cur = dict(zip(js["m"].name, js["m"].position))
    err = max(abs(cur[j] - t) for j, t in zip(tj["joints"], target))
    print(f"max joint err {err:.4f} rad; fingers {cur['panda_finger_joint1']:.4f} {cur['panda_finger_joint2']:.4f}")
    buf = Buffer(); TransformListener(buf, node)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform("world", "panda_hand", rclpy.time.Time()):
            break
    t = buf.lookup_transform("world", "panda_hand", rclpy.time.Time())
    tr, q = t.transform.translation, t.transform.rotation
    Rh = quat_R(q.x, q.y, q.z, q.w)
    tcp = np.array([tr.x, tr.y, tr.z]) + off * Rh[:, 2]
    print(f"hand world {tr.x:.4f} {tr.y:.4f} {tr.z:.4f} q {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}")
    print(f"TCP world {tcp[0]:.4f} {tcp[1]:.4f} {tcp[2]:.4f}  (target {x} {y} {z})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 18
timeout 600 python3 -u move.py -0.210 0.0735 1.03 0.7071068 0.7071068 0 0 4

# openrua op 19
cat > iktest.py <<'EOF'
import sys, numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
sys.path.insert(0,'/workspace'); from move import quat_R
M = yaml.safe_load(open("/workspace/machine.yaml")); tj=next(e for e in M["actuators"] if e["kind"]=="joint_trajectory")
rclpy.init(); node=rclpy.create_node("ikt"); js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.__setitem__("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
cur=dict(zip(js["m"].name,js["m"].position))
cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service()
def ik(tcp, q, seed=None):
    R=quat_R(*q); hand=np.array(tcp)-0.1034*R[:,2]-np.array([-0.66,0,0.912])
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"; req.ik_request.pose_stamped.header.frame_id=""
    p=req.ik_request.pose_stamped.pose; p.position.x,p.position.y,p.position.z=hand
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q
    s=JointState()
    for j in tj["joints"]: s.name.append(j); s.position.append((seed or cur)[j])
    req.ik_request.robot_state.joint_state=s; req.ik_request.avoid_collisions=False
    req.ik_request.timeout.sec=1
    f=cli.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=60); r=f.result()
    if r.error_code.val==1:
        sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
        return [round(sol[j],3) for j in tj["joints"]]
    return r.error_code.val
tests=[
 ((-0.210,0.0735,1.03),(1,0,0,0)),
 ((-0.210,0.0735,1.03),(0.7071068,0.7071068,0,0)),
 ((-0.210,0.0735,1.03),(0.7071068,-0.7071068,0,0)),
 ((-0.210,0.0735,1.03),(0,0.7071068,0.7071068,0)),
 ((-0.203,0.0,1.166),(1,0,0,0)),
 ((-0.203,0.0,1.166),(0.7071068,0.7071068,0,0)),
]
for tcp,q in tests: print(tcp,q,'->',ik(tcp,q))
EOF
timeout 300 python3 iktest.py

# openrua op 20
sed -i 's/p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q/p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=[float(v) for v in q]/; s/p.position.x,p.position.y,p.position.z=hand/p.position.x,p.position.y,p.position.z=[float(v) for v in hand]/' iktest.py && timeout 300 python3 iktest.py

# openrua op 21
cat > fktest.py <<'EOF'
import rclpy, yaml
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open("/workspace/machine.yaml")); tj=next(e for e in M["actuators"] if e["kind"]=="joint_trajectory")
rclpy.init(); node=rclpy.create_node("fkt"); js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.__setitem__("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
cur=dict(zip(js["m"].name,js["m"].position))
cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand","panda_link8","panda_link0"]
s=JointState()
for j in tj["joints"]: s.name.append(j); s.position.append(cur[j])
req.robot_state.joint_state=s
f=cli.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=60); r=f.result()
print('err',r.error_code.val)
for n,ps in zip(r.fk_link_names,r.pose_stamped):
    p=ps.pose.position;q=ps.pose.orientation
    print(n, ps.header.frame_id, f"{p.x:.4f} {p.y:.4f} {p.z:.4f} | {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}")
EOF
timeout 120 python3 fktest.py; ros2 param get /move_group robot_description_kinematics.panda_arm.kinematics_solver 2>&1 | tail -1; ros2 param list /move_group 2>/dev/null | grep -i -E "kinematics|panda_arm" | head

# openrua op 22
sed -i 's/-np.array(\[-0.66,0,0.912\])//' iktest.py && timeout 300 python3 iktest.py

# openrua op 23
cat >> fktest.py <<'EOF'
print('--- FK of IK solutions')
import numpy as np
from move import quat_R
for sol in [[0.126, 0.131, 0.034, -2.549, -0.01, 2.68, 0.169],[0.23, 0.131, -0.065, -2.549, 0.019, 2.68, -1.423],[0.059, -0.171, -0.058, -2.47, -0.013, 2.299, 0.01]]:
    s=JointState()
    for j,v in zip(tj["joints"],sol): s.name.append(j); s.position.append(float(v))
    req.robot_state.joint_state=s; req.fk_link_names=["panda_hand","panda_leftfinger","panda_rightfinger"]
    f=cli.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=60); r=f.result()
    for n,ps in zip(r.fk_link_names,r.pose_stamped):
        p=ps.pose.position;q=ps.pose.orientation
        print(n, f"{p.x:.4f} {p.y:.4f} {p.z:.4f} | {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}", 'hand y-axis in world', np.round(quat_R(q.x,q.y,q.z,q.w)[:,1],3) if n=='panda_hand' else '')
EOF
timeout 120 python3 fktest.py 2>&1 | tail -12

# openrua op 24
python3 - <<'EOF'
s=open('move.py').read()
s=s.replace('''    base = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (TF, identity rotation)
    R = quat_R(qx, qy, qz, qw)
    hand_w = np.array([x, y, z]) - off * R[:, 2]
    hand_b = hand_w - base  # IK is in the arm base frame
''','''    R = quat_R(qx, qy, qz, qw)
    hand_b = np.array([x, y, z]) - off * R[:, 2]  # FK/IK here report in world
    # IK tip link is panda_link8 = hand rotated +45 deg about its z
    lx, ly, lz, lw = qmul((qx, qy, qz, qw), (0.0, 0.0, 0.3826834, 0.9238795))
''')
s=s.replace('''    p.position.x, p.position.y, p.position.z = hand_b
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw''','''    p.position.x, p.position.y, p.position.z = [float(v) for v in hand_b]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = lx, ly, lz, lw''')
s=s.replace('''def main():''','''def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def main():''')
open('move.py','w').write(s)
EOF
timeout 600 python3 -u move.py -0.210 0.0735 1.03 0.7071068 0.7071068 0 0 4

# openrua op 25
ros2 topic echo /joint_states --once | grep -A7 position; timeout 600 python3 tools/action/fjt_send.py 0.179,0.131,-0.016,-2.549,0.005,2.680,-0.627 3; ros2 topic echo /joint_states --once | grep -A7 position

# openrua op 26
timeout 600 python3 tools/action/fjt_send.py 0.179,0.131,-0.016,-2.549,0.005,2.680,-0.627 3; ros2 topic echo /joint_states --once | grep -A7 position; timeout 60 python3 tfq.py world:panda_hand | head -1

# openrua op 27
timeout 120 python3 cammap.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
cam='robot0_eye_in_hand'
wx=np.load(f'{cam}_wx.npy'); wy=np.load(f'{cam}_wy.npy'); wz=np.load(f'{cam}_wz.npy')
reg=(wx>-0.25)&(wx<-0.07)&(wy>-0.02)&(wy<0.17)&(wz<1.05)
for lo,hi in [(0.905,0.92),(0.92,0.93),(0.93,0.94),(0.94,0.95),(0.95,0.97)]:
    m=reg&(wz>=lo)&(wz<hi)
    if m.sum(): print(f"  z[{lo:.3f},{hi:.2f}) n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}]")
m=reg&(wz>0.94)
print('rim centroid', wx[m].mean(), wy[m].mean())
EOF

# openrua op 28
timeout 600 python3 -u move.py -0.210 0.0737 0.935 0.7071068 0.7071068 0 0 3

# openrua op 29
timeout 300 python3 tools/action/gripper_cmd.py 0.0; ros2 topic echo /joint_states --once | grep -A9 position | tail -2; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 30
timeout 600 python3 -u move.py -0.210 0.0737 1.06 0.7071068 0.7071068 0 0 3 && ros2 topic echo /joint_states --once | grep -A9 position | tail -2 && timeout 120 python3 cammap.py agentview && python3 - <<'EOF'
import numpy as np
cam='agentview'
wx=np.load(f'{cam}_wx.npy'); wy=np.load(f'{cam}_wy.npy'); wz=np.load(f'{cam}_wz.npy')
reg=(wx>-0.30)&(wx<-0.05)&(wy>-0.02)&(wy<0.2)&(wz<1.2)
for lo,hi in [(0.905,0.95),(0.95,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.09),(1.09,1.2)]:
    m=reg&(wz>=lo)&(wz<hi)
    if m.sum(): print(f"  z[{lo:.3f},{hi:.2f}) n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}]")
EOF

# openrua op 31
timeout 600 python3 -u move.py -0.160 -0.160 1.06 0.7071068 0.7071068 0 0 4

# openrua op 32
timeout 120 python3 cammap.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
cam='robot0_eye_in_hand'
wx=np.load(f'{cam}_wx.npy'); wy=np.load(f'{cam}_wy.npy'); wz=np.load(f'{cam}_wz.npy')
# what's visible below: drawer floor / walls, bowl
reg=(wz<1.15)&np.isfinite(wz)
for lo,hi in [(0.89,0.915),(0.915,0.935),(0.935,0.96),(0.96,0.99),(0.99,1.02),(1.02,1.05),(1.05,1.08),(1.08,1.15)]:
    m=reg&(wz>=lo)&(wz<hi)
    if m.sum(): print(f"  z[{lo:.3f},{hi:.3f}) n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}]")
EOF

# openrua op 33
timeout 600 python3 -u move.py -0.160 -0.160 0.968 0.7071068 0.7071068 0 0 3 && timeout 300 python3 tools/action/gripper_cmd.py 0.04 && ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 34
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 35
timeout 120 python3 cammap.py agentview && timeout 120 python3 cammap.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
for cam in ['agentview','robot0_eye_in_hand']:
    wx=np.load(f'{cam}_wx.npy'); wy=np.load(f'{cam}_wy.npy'); wz=np.load(f'{cam}_wz.npy')
    reg=(wx>-0.25)&(wx<0.03)&(wy>-0.25)&(wy<-0.03)&(wz<1.1)&np.isfinite(wz)
    print(cam)
    for lo,hi in [(0.915,0.935),(0.935,0.95),(0.95,0.96),(0.96,0.97),(0.97,0.98),(0.98,0.99),(0.99,1.0),(1.0,1.02),(1.02,1.05),(1.05,1.1)]:
        m=reg&(wz>=lo)&(wz<hi)
        if m.sum(): print(f"  z[{lo:.3f},{hi:.3f}) n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}]")
EOF

# openrua op 36
timeout 600 python3 -u move.py -0.150 -0.160 1.20 0.7071068 0.7071068 0 0 3 && timeout 600 python3 -u move.py -0.150 0.10 1.20 0.7071068 0.7071068 0 0 3 && timeout 120 python3 cammap.py birdview && timeout 120 python3 cammap.py agentview && python3 - <<'EOF'
import numpy as np
for cam in ['birdview','agentview']:
    wx=np.load(f'{cam}_wx.npy'); wy=np.load(f'{cam}_wy.npy'); wz=np.load(f'{cam}_wz.npy')
    reg=(wx>-0.25)&(wx<0.03)&(wy>-0.25)&(wy<-0.03)&(wz<1.1)&np.isfinite(wz)
    print(cam)
    for lo,hi in [(0.915,0.935),(0.935,0.95),(0.95,0.96),(0.96,0.97),(0.97,0.98),(0.98,0.99),(0.99,1.0),(1.0,1.02),(1.02,1.05),(1.05,1.1)]:
        m=reg&(wz>=lo)&(wz<hi)
        if m.sum(): print(f"  z[{lo:.3f},{hi:.3f}) n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}]")
EOF

# openrua op 37
python3 - <<'EOF'
import numpy as np
for cam in ['birdview','agentview']:
    wx=np.load(f'{cam}_wx.npy'); wy=np.load(f'{cam}_wy.npy'); wz=np.load(f'{cam}_wz.npy')
    reg=(wx>-0.205)&(wx<-0.02)&(wy>-0.225)&(wy<-0.095)&(wz<1.1)&np.isfinite(wz)
    print(cam, 'interior z range', wz[reg].min(), wz[reg].max())
    for lo,hi in [(0.915,0.93),(0.93,0.94),(0.94,0.95),(0.95,0.96),(0.96,0.97),(0.97,0.98),(0.98,0.99),(0.99,1.0),(1.0,1.02),(1.02,1.1)]:
        m=reg&(wz>=lo)&(wz<hi)
        if m.sum(): print(f"  z[{lo:.3f},{hi:.3f}) n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] cen ({wx[m].mean():.3f},{wy[m].mean():.3f})")
EOF

# openrua op 38
python3 - <<'EOF'
import numpy as np
wx=np.load('birdview_wx.npy'); wy=np.load('birdview_wy.npy'); wz=np.load('birdview_wz.npy')
# bottom drawer handle: in front of the panel face
m=(wy>-0.07)&(wy<-0.03)&(wz>0.93)&(wz<0.99)&(wx>-0.3)&(wx<0.1)
print('bottom handle', m.sum(), f"x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] z[{wz[m].min():.3f},{wz[m].max():.3f}]")
# panel top
m=(wy>-0.092)&(wy<-0.068)&(wz>0.97)&(wz<1.0)
print('panel top', m.sum(), f"x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] z[{wz[m].min():.3f},{wz[m].max():.3f}]")
# top handle
m=(wy>-0.24)&(wy<-0.19)&(wz>0.995)&(wz<1.12)&(wx>-0.2)&(wx<0.05)
print('top handle', m.sum(), f"x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] z[{wz[m].min():.3f},{wz[m].max():.3f}]")
import sys; sys.path.insert(0,'/workspace'); from move import qmul, quat_R
th=np.radians(-40); qx=(np.sin(th/2),0,0,np.cos(th/2)); q0=(0.7071068,0.7071068,0,0)
q=qmul(qx,q0); print('tilted q', np.round(q,6)); R=quat_R(*q); print('hand z axis world', np.round(R[:,2],3), 'hand y axis', np.round(R[:,1],3))
EOF

# openrua op 39
timeout 300 python3 tools/action/gripper_cmd.py 0.0 && timeout 600 python3 -u move.py -0.19 -0.03 0.95 0.664463 0.664463 -0.241845 0.241845 4

# openrua op 40
timeout 600 python3 tools/action/fjt_send.py 0.345,0.477,-0.235,-2.324,-0.983,2.417,0.219 3 && timeout 60 python3 tfq.py world:panda_hand | head -1 && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 41
mkdir -p "$(dirname /workspace/line.py)"
cat > /workspace/line.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Straight-line TCP move: IK at N waypoints -> one multi-point trajectory.

Usage: python3 line.py x0 y0 z0 x1 y1 z1 qx qy qz qw seconds [n=6]
Quaternion is the HAND orientation (converted to link8 for IK).
"""
import sys
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, "/workspace")
from move import quat_R, qmul  # noqa: E402


def main():
    a = [float(v) for v in sys.argv[1:]]
    p0, p1 = np.array(a[0:3]), np.array(a[3:6])
    q = tuple(a[6:10]); secs = a[10]; n = int(a[11]) if len(a) > 11 else 6
    M = yaml.safe_load(open("/workspace/machine.yaml"))
    tj = next(e for e in M["actuators"] if e["kind"] == "joint_trajectory")
    off = M["hand"]["tcp_offset_m"]
    R = quat_R(*q)
    lq = qmul(q, (0.0, 0.0, 0.3826834, 0.9238795))

    rclpy.init()
    node = rclpy.create_node("liner")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cur = dict(zip(js["m"].name, js["m"].position))
    seed = [cur[j] for j in tj["joints"]]
    cli = node.create_client(GetPositionIK, M["planning"]["ik_service"])
    cli.wait_for_service(timeout_sec=10)

    pts = []
    for i in range(1, n + 1):
        tcp = p0 + (p1 - p0) * i / n
        hand = tcp - off * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = [float(v) for v in hand]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = lq
        s = JointState()
        for j, v in zip(tj["joints"], seed):
            s.name.append(j); s.position.append(float(v))
        req.ik_request.robot_state.joint_state = s
        req.ik_request.avoid_collisions = False
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK FAILED at waypoint {i} {tcp}", None if res is None else res.error_code.val); sys.exit(2)
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        seed = [sol[j] for j in tj["joints"]]
        pt = JointTrajectoryPoint(positions=seed)
        t = secs * i / n
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        pts.append(pt)
        print(f"wp{i} {np.round(tcp,3)} ->", " ".join(f"{v:.3f}" for v in seed))

    ac = ActionClient(node, FollowJointTrajectory, tj["port"])
    ac.wait_for_server(timeout_sec=10)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = tj["joints"]
    goal.trajectory.points = pts
    send = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    rf = send.result().get_result_async()
    rclpy.spin_until_future_complete(node, rf)
    print("traj error_code", rf.result().result.error_code)

    js.pop("m", None)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cur = dict(zip(js["m"].name, js["m"].position))
    err = max(abs(cur[j] - t) for j, t in zip(tj["joints"], seed))
    print(f"max joint err vs final {err:.4f} rad; fingers {cur['panda_finger_joint1']:.4f} {cur['panda_finger_joint2']:.4f}")
    buf = Buffer(); TransformListener(buf, node)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform("world", "panda_hand", rclpy.time.Time()):
            break
    t = buf.lookup_transform("world", "panda_hand", rclpy.time.Time())
    tr, qq = t.transform.translation, t.transform.rotation
    Rh = quat_R(qq.x, qq.y, qq.z, qq.w)
    tcp = np.array([tr.x, tr.y, tr.z]) + off * Rh[:, 2]
    print(f"TCP world {tcp[0]:.4f} {tcp[1]:.4f} {tcp[2]:.4f}  (target {p1})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 42
timeout 900 python3 -u line.py -0.19 -0.03 0.95 -0.19 -0.235 0.95 0.664463 0.664463 -0.241845 0.241845 5 6

# openrua op 43
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 44
timeout 600 python3 -u move.py -0.19 -0.08 1.05 0.664463 0.664463 -0.241845 0.241845 3 | tail -1 && timeout 600 python3 -u move.py -0.20 0.0 1.20 0.7071068 0.7071068 0 0 4 | tail -1 && timeout 120 python3 cammap.py birdview && timeout 120 python3 cammap.py agentview && python3 - <<'EOF'
import numpy as np
for cam in ['birdview','agentview']:
    wx=np.load(f'{cam}_wx.npy'); wy=np.load(f'{cam}_wy.npy'); wz=np.load(f'{cam}_wz.npy')
    fin=np.isfinite(wz)
    # anything cabinet-related in front of the cabinet body face (y>-0.233) at drawer heights, x in cabinet span
    m=fin&(wx>-0.24)&(wx<0.02)&(wy>-0.233)&(wy<-0.0)&(wz>0.91)&(wz<1.12)
    print(cam,'points in front of cabinet face z 0.91-1.12:', m.sum(), '' if not m.sum() else f"x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] z[{wz[m].min():.3f},{wz[m].max():.3f}]")
    # bowl anywhere on table? points z 0.93-0.99 excluding cabinet, wooden stand (y>0.15), bottle (x>0)
    m=fin&(wz>0.93)&(wz<0.99)&(wy>-0.233)&(wy<0.15)&(wx<0.0)&(wx>-0.5)
    print(cam,'bowl-height points on open table:', m.sum(), '' if not m.sum() else f"x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}]")
EOF

# openrua op 45
python3 - <<'EOF'
import numpy as np
wx=np.load('birdview_wx.npy'); wy=np.load('birdview_wy.npy'); wz=np.load('birdview_wz.npy')
for x0 in [-0.11,-0.19,-0.05]:
    print(f'--- profile along y at x={x0}')
    m=(abs(wx-x0)<0.004)&(wy>-0.30)&(wy<0.02)
    ys=wy[m]; zs=wz[m]; o=np.argsort(ys)
    last=None
    for yy,zz in zip(ys[o],zs[o]):
        if last is None or abs(zz-last)>0.004: print(f"y={yy:.3f} z={zz:.3f}"); last=zz
EOF
