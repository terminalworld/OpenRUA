#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ======; cat /workspace/tools/action/fjt_send.py; echo ======; cat /workspace/tools/action/ik_move.py; echo ======; cat /workspace/tools/action/gripper_cmd.py; echo ======; cat /workspace/tools/perception/cam_snap.py; echo ======; cat /workspace/tools/perception/px2world.py

# openrua op 2
timeout 30 ros2 topic list; echo ====; timeout 30 ros2 action list; echo ====; timeout 30 ros2 service list | head -50; echo ====; timeout 30 ros2 node list

# openrua op 3
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 4
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|translation|x:|y:|z:|w:" | head -80; echo ====; timeout 10 ros2 topic echo /tf --once 2>&1 | head -40

# openrua op 5
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab depth+info+TF for a camera; convert pixels to world, save arrays.
Usage: python3 scene.py <camera> [u v]...
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, mt, timeout=20.0):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    px = [(int(sys.argv[i]), int(sys.argv[i + 1])) for i in range(2, len(sys.argv) - 1, 2)]
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer(); TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
    color = CvBridge().imgmsg_to_cv2(color_msg, "bgr8")
    np.save(f"{cam}_depth.npy", depth)
    cv2.imwrite(f"{cam}.png", color)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4); T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    np.save(f"{cam}_T.npy", T)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    np.save(f"{cam}_K.npy", np.array([fx, fy, cx, cy]))
    print("cam pos", T[:3, 3], "K", fx, fy, cx, cy, "size", depth.shape)
    for (u, v) in px:
        z = depth[v, u]
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"px({u},{v}) depth={z:.3f} world=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 6
timeout 90 python3 scene.py birdview 338 258 305 300 360 300 415 300 300 350 230 280 && timeout 90 python3 scene.py agentview 350 200 350 230 400 300 420 330 560 300 350 400

# openrua op 7
python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent.png', cv2.resize(im[150:400,300:640],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('birdview.png'); cv2.imwrite('crop_bird.png', cv2.resize(im[200:360,260:470],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 8
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); fx,fy,cx,cy=np.load('birdview_K.npy')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
w=pc@T.T
X,Y,Z=w[...,0],w[...,1],w[...,2]
np.save('bird_xyz.npy',w[...,:3])
# table height
print('table z sample', Z[400,100], Z[450,500])
# bottle region: around u 338 v 258
m=(Z>0.95)&(np.abs(X+0.14)<0.08)&(np.abs(Y-0.06)<0.08)
print('bottle pts', m.sum(), 'x',X[m].min(),X[m].max(),'y',Y[m].min(),Y[m].max(),'zmax',Z[m].max())
# top of bottle (highest)
mm=m&(Z>Z[m].max()-0.01)
print('bottle cap center', X[mm].mean(), Y[mm].mean(), Z[mm].max())
# body of bottle (z between 0.93 and 1.0)
mb=m&(Z>0.93)&(Z<1.0)
print('bottle body x',X[mb].min(),X[mb].max(),'y',Y[mb].min(),Y[mb].max(), 'center', X[mb].mean(), Y[mb].mean())
# cabinet region
mc=(Z>0.91)&(X>-0.15)&(X<0.35)&(Y>0.0)&(Y<0.5)
print('cabinet+drawer x',X[mc].min(),X[mc].max(),'y',Y[mc].min(),Y[mc].max(),'zmax',Z[mc].max())
mt=mc&(Z>1.1)
print('cabinet top x',X[mt].min(),X[mt].max(),'y',Y[mt].min(),Y[mt].max(),'z',Z[mt].mean())
# drawer: z between 0.91 and 1.0, y<0.2
md=mc&(Z<1.05)&(Y<0.22)
print('drawer region x',X[md].min(),X[md].max(),'y',Y[md].min(),Y[md].max(),'z range',Z[md].min(),Z[md].max())
# drawer walls: highest in drawer region
mw=md&(Z>0.95)
print('drawer walls x',X[mw].min(),X[mw].max(),'y',Y[mw].min(),Y[mw].max(),'z',Z[mw].mean(), Z[mw].max())
mf=md&(Z<0.94)
print('drawer floor x',X[mf].min(),X[mf].max(),'y',Y[mf].min(),Y[mf].max(),'z',Z[mf].mean())
"


# openrua op 9
python3 -c "
import numpy as np, cv2
for cam in ['agentview','birdview']:
    d=np.load(cam+'_depth.npy'); T=np.load(cam+'_T.npy'); fx,fy,cx,cy=np.load(cam+'_K.npy')
    H,W=d.shape
    v,u=np.mgrid[0:H,0:W]
    pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
    w=pc@T.T
    X,Y,Z=w[...,0],w[...,1],w[...,2]
    im=cv2.imread(cam+'.png')
    b,g,r=im[...,0].astype(int),im[...,1].astype(int),im[...,2].astype(int)
    # dark green bottle: g > r, g > b, dark
    m=(g>r+5)&(g>b+5)&(g<120)&(Z>0.9)&(Z<1.2)&(np.abs(X)<0.4)&(np.abs(Y)<0.4)
    print(cam,'bottle px',m.sum())
    if m.sum():
        print(' x',X[m].min(),X[m].max(),' y',Y[m].min(),Y[m].max(),' z',Z[m].min(),Z[m].max())
        print(' centroid',X[m].mean(),Y[m].mean(),Z[m].mean())
        print(' px bbox u',u[m].min(),u[m].max(),'v',v[m].min(),v[m].max())
"


# openrua op 10
python3 -c "
import numpy as np, cv2
cam='agentview'
d=np.load(cam+'_depth.npy'); T=np.load(cam+'_T.npy'); fx,fy,cx,cy=np.load(cam+'_K.npy')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
w=pc@T.T
X,Y,Z=w[...,0],w[...,1],w[...,2]
im=cv2.imread(cam+'.png')
mx=im.max(-1)
m=(mx<90)&(Z>0.9)&(Z<1.2)&(np.abs(X+0.13)<0.1)&(np.abs(Y-0.05)<0.1)
print('dark px',m.sum())
print(' x',X[m].min(),X[m].max(),' y',Y[m].min(),Y[m].max(),' z',Z[m].min(),Z[m].max())
print(' px bbox u',u[m].min(),u[m].max(),'v',v[m].min(),v[m].max())
# per z-slice the y extent (width)
for z0 in np.arange(0.90,1.14,0.02):
    s=m&(Z>=z0)&(Z<z0+0.02)
    if s.sum(): print(f'z {z0:.2f}: n={s.sum()} y {Y[s].min():.3f}..{Y[s].max():.3f} x {X[s].min():.3f}..{X[s].max():.3f}')
# also cork/top pixels (brownish) above
mc=(Z>1.1)&(np.abs(X+0.13)<0.05)&(np.abs(Y-0.05)<0.05)
print('top', Z[mc].max() if mc.sum() else None, X[mc].mean() if mc.sum() else None, Y[mc].mean() if mc.sum() else None)
"


# openrua op 11
python3 -c "
import numpy as np
w=np.load('bird_xyz.npy'); X,Y,Z=w[...,0],w[...,1],w[...,2]
xs=np.arange(-0.22,0.20,0.02); ys=np.arange(-0.06,0.44,0.02)
print('      '+' '.join(f'{y:5.2f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(np.abs(X-x)<0.01)&(np.abs(Y-y)<0.01)
        row.append(f'{Z[m].max()-0.9:5.2f}' if m.sum() else '  .  ')
    print(f'{x:5.2f} '+' '.join(row))
"


# openrua op 12
python3 -c "
import numpy as np
cam='agentview'
d=np.load(cam+'_depth.npy'); T=np.load(cam+'_T.npy'); fx,fy,cx,cy=np.load(cam+'_K.npy')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
w=pc@T.T
X,Y,Z=w[...,0],w[...,1],w[...,2]
np.save('agent_xyz.npy',w[...,:3])
# front face profile: for each (x,z) cell, min Y among points with Y in [0.0,0.5]
xs=np.arange(-0.16,0.18,0.02); zs=np.arange(0.90,1.20,0.02)
print('       '+' '.join(f'{x:5.2f}' for x in xs))
for z in zs[::-1]:
    row=[]
    for x in xs:
        m=(np.abs(X-x)<0.01)&(np.abs(Z-z)<0.01)&(Y>0.0)&(Y<0.5)
        row.append(f'{Y[m].min():5.2f}' if m.sum() else '  .  ')
    print(f'{z:5.2f}  '+' '.join(row))
"


# openrua op 13
python3 -c "
import cv2
im=cv2.imread('frontview.png'); cv2.imwrite('crop_front.png', cv2.resize(im[250:430,330:560],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 14
python3 -c "
import numpy as np
w=np.load('bird_xyz.npy'); X,Y,Z=w[...,0],w[...,1],w[...,2]
def grid(xs,ys):
    print('      '+' '.join(f'{y:5.2f}' for y in ys))
    for x in xs:
        row=[]
        for y in ys:
            m=(np.abs(X-x)<0.005)&(np.abs(Y-y)<0.005)
            row.append(f'{Z[m].max()-0.9:5.2f}' if m.sum() else '  .  ')
        print(f'{x:5.2f} '+' '.join(row))
print('DRAWER'); grid(np.arange(-0.13,0.14,0.01), np.arange(0.02,0.26,0.01))
print('BOTTLE'); grid(np.arange(-0.19,-0.09,0.01), np.arange(0.00,0.12,0.01))
"


# openrua op 15
find / \( -iname "*cabinet*" -o -iname "*wine_bottle*" -o -iname "*drawer*" \) 2>/dev/null | grep -v proc | head -30; echo ===; ls /opt; ls /; pip list 2>/dev/null | grep -i -E "libero|robosuite|mujoco|robocasa"

# openrua op 16
ls -la /log /workspace/.ros/log; tail -5 /workspace/.ros/log/*.log | head -80; timeout 10 ros2 topic echo /pipeline_state --once 2>&1 | head; timeout 20 ros2 node info /robot_bridge 2>&1 | head -60

# openrua op 17
timeout 60 python3 scene.py sideview 345 228 345 260 270 260 270 380 && python3 -c "
import numpy as np, cv2
cam='sideview'
d=np.load(cam+'_depth.npy'); T=np.load(cam+'_T.npy'); fx,fy,cx,cy=np.load(cam+'_K.npy')
H,W=d.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
w=pc@T.T
X,Y,Z=w[...,0],w[...,1],w[...,2]
# bottle: dark pixels near x=-0.15,y=0.05
im=cv2.imread(cam+'.png'); mx=im.max(-1)
m=(Z>0.9)&(np.abs(X+0.15)<0.06)&(np.abs(Y-0.05)<0.06)
print('bottle region z max', Z[m].max(), 'n', m.sum())
for z0 in np.arange(0.90,1.16,0.02):
    s=m&(Z>=z0)&(Z<z0+0.02)
    if s.sum(): print(f'z {z0:.2f}: n={s.sum()} x {X[s].min():.3f}..{X[s].max():.3f} y {Y[s].min():.3f}..{Y[s].max():.3f}')
# cabinet: x -0.15..0.15, y 0.2..0.45
mc=(Z>0.9)&(X>-0.15)&(X<0.15)&(Y>0.05)&(Y<0.45)
print('cabinet z max',Z[mc].max(),'x',X[mc].min(),X[mc].max(),'y',Y[mc].min(),Y[mc].max())
"


# openrua op 18
python3 -c "
import numpy as np, cv2
w=np.load('agent_xyz.npy'); im=cv2.imread('agentview.png')
for v in range(150,300,4):
    u=352
    print(v, im[v,u], np.round(w[v,u],3))
"


# openrua op 19
timeout 120 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 1.0; timeout 20 ros2 topic echo /clock --once

# openrua op 20
timeout 60 python3 scene.py birdview >/dev/null && timeout 60 python3 scene.py agentview > /dev/null && python3 -c "
import numpy as np, cv2
for cam in ['birdview','agentview']:
    d=np.load(cam+'_depth.npy'); T=np.load(cam+'_T.npy'); fx,fy,cx,cy=np.load(cam+'_K.npy')
    H,W=d.shape
    v,u=np.mgrid[0:H,0:W]
    pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)
    w=pc@T.T; np.save(cam[:5]+'_xyz.npy',w[...,:3])
    X,Y,Z=w[...,0],w[...,1],w[...,2]
    m=(Z>0.905)&(Z<1.2)&(np.abs(X+0.145)<0.05)&(np.abs(Y-0.053)<0.05)
    print(cam,'bottle n',m.sum(),'zmax',Z[m].max())
    for z0 in np.arange(0.90,1.16,0.02):
        s=m&(Z>=z0)&(Z<z0+0.02)
        if s.sum(): print(f'  z {z0:.2f}: n={s.sum()} x {X[s].min():.3f}..{X[s].max():.3f} y {Y[s].min():.3f}..{Y[s].max():.3f}')
"


# openrua op 21
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<15: rclpy.spin_once(n,timeout_sec=0.2)
open('robot.urdf','w').write(got[0] if got else '')
print(len(got[0]) if got else 'none')
"; grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n "mesh filename" robot.urdf | head -5

# openrua op 22
python3 -c "
import numpy as np, struct
def stl_bounds(p):
    data=open(p,'rb').read()
    n=struct.unpack_from('<I',data,80)[0]
    arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
    v=arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ['hand','finger','link7']:
    print(f, stl_bounds(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'))
"

# openrua op 23
find / -name "hand*.stl" -o -name "finger*.stl" 2>/dev/null | grep -v proc | head

# openrua op 24
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable control helpers for this Panda (one node, clients built once)."""
import sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import WrenchStamped

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])
TCP_OFF = 0.1034


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    m = R
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s; x = (m[2, 1] - m[1, 2]) / s; y = (m[0, 2] - m[2, 0]) / s; z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s; x = 0.25 * s; y = (m[0, 1] + m[1, 0]) / s; z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s; x = (m[0, 1] + m[1, 0]) / s; y = 0.25 * s; z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s; x = (m[0, 2] + m[2, 0]) / s; y = (m[1, 2] + m[2, 1]) / s; z = 0.25 * s
    q = np.array([x, y, z, w]); return q / np.linalg.norm(q)


def rot_from_axes(approach, closing):
    """Hand rotation: z = approach, y = finger closing dir, x = y cross z."""
    z = np.array(approach, float); z /= np.linalg.norm(z)
    y = np.array(closing, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.stack([x, y, z], 1)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = None
        self._wrench = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_w, 10)
        self.traj = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_w(self, m): self._wrench = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            end = time.time() + 10
            while self._js is None and time.time() < end:
                self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wrench = None
        end = time.time() + 5
        while self._wrench is None and time.time() < end:
            self.spin(0.1)
        if self._wrench is None:
            return None
        f = self._wrench.wrench.force; t = self._wrench.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def fk(self, q=None):
        """hand pose in WORLD: (pos, R)."""
        if q is None: q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = quat_to_R([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk(q)
        return pos + TCP_OFF * R[:, 2], R

    def ik(self, hand_pos_world, R, seed=None, timeout=30):
        """returns joint array or None"""
        if seed is None: seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        bp = np.array(hand_pos_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, bp)
        q = R_to_quat(R)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            print("IK: no answer"); return None
        if res.error_code.val != 1:
            print(f"IK failed code={res.error_code.val}"); return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    def ik_tcp(self, tcp_pos_world, R, seed=None):
        hand = np.array(tcp_pos_world) - TCP_OFF * R[:, 2]
        return self.ik(hand, R, seed)

    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if waypoints:
            for (wq, wt) in waypoints:
                pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
                pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            print("traj goal rejected"); return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = r.result.error_code if r else None
        err = np.abs(self.arm_q() - np.array(q)).max()
        print(f"move_q: code={code} max_err={err:.4f}")
        return code

    def move_tcp(self, tcp_pos, R, seconds=3.0, seed=None):
        q = self.ik_tcp(tcp_pos, R, seed)
        if q is None:
            return None
        code = self.move_q(q, seconds)
        p, _ = self.tcp()
        print(f"  tcp now {np.round(p,4)} target {np.round(tcp_pos,4)}")
        return code

    def gripper(self, width, effort=30.0):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(effort)
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f


DOWN = rot_from_axes([0, 0, -1], [0, 1, 0])   # approach down, fingers close along y
DOWN_X = rot_from_axes([0, 0, -1], [1, 0, 0])  # approach down, fingers close along x
OPENRUA_EOF

# openrua op 25
timeout 120 python3 -c "
from robot import *
r=Robot()
q=r.arm_q(); print('q',np.round(q,3))
pos,R=r.fk(q); print('hand world',np.round(pos,4)); print('R',np.round(R,3))
print('tcp',np.round(r.tcp(q)[0],4))
print('fingers',r.fingers())
print('wrench',r.wrench())
"

# openrua op 26
timeout 120 python3 -c "
import numpy as np
def dh(a,d,alpha,theta):
    ca,sa,ct,st=np.cos(alpha),np.sin(alpha),np.cos(theta),np.sin(theta)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
def fk(q):
    P=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
    T=np.eye(4)
    for (a,d,al),th in zip(P,q): T=T@dh(a,d,al,th)
    T=T@dh(0,0.107,0,0)  # flange
    T=T@dh(0,0,0,-np.pi/4) # hand
    return T
q=[0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483]
T=fk(q); print(np.round(T,4))
print('world hand', np.round(T[:3,3]+[-0.66,0,0.912],4))
"


# openrua op 27
timeout 200 python3 -c "
from robot import *
import robot
r=Robot()
q=r.arm_q()
pos,R=r.fk(q)
raw=pos-BASE_IN_WORLD   # raw FK output
print('raw fk', np.round(raw,4))
# test IK with raw (world) coords: pass hand_pos_world=raw+BASE so ik subtracts BASE -> raw
sol=r.ik(raw+BASE_IN_WORLD, R, seed=q)
print('IK(raw coords) ->', None if sol is None else np.round(sol,3))
# test IK with base-rel coords: raw - BASE
sol2=r.ik(raw, R, seed=q)
print('IK(raw-BASE) ->', None if sol2 is None else np.round(sol2,3))
print('current q', np.round(q,3))
"

# openrua op 28
timeout 200 python3 -c "
from robot import *
r=Robot()
q=r.arm_q()
pos,R=r.fk(q)
sol=np.array([0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009])
p2,R2=r.fk(sol)
print(np.round(pos,4), np.round(p2,4)); print(np.round(R,3)); print(np.round(R2,3))
"

# openrua op 29
python3 - <<'EOF'
import re
s=open('robot.py').read()
s=s.replace('        req.ik_request.group_name = "panda_arm"\n', '        req.ik_request.group_name = "panda_arm"\n        req.ik_request.ik_link_name = "panda_hand"\n')
s=s.replace('        bp = np.array(hand_pos_world) - BASE_IN_WORLD\n', '        bp = np.array(hand_pos_world)  # MoveIt model frame == world here (verified by FK)\n')
s=s.replace('        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD\n', '        pos = np.array([p.position.x, p.position.y, p.position.z])  # already world\n')
open('robot.py','w').write(s)
EOF
timeout 200 python3 -c "
from robot import *
r=Robot()
q=r.arm_q()
pos,R=r.fk(q)
print('hand world', np.round(pos,4))
sol=r.ik(pos, R, seed=q)
print('IK ->', None if sol is None else np.round(sol,3))
p2,R2=r.fk(sol); print(np.round(p2,4)); print(np.round(R2,3))
"

# openrua op 30
python3 -c "
import numpy as np
w=np.load('bird_xyz.npy'); X,Y,Z=w[...,0],w[...,1],w[...,2]
# wall positions: along x at y=0.15
for x in np.arange(-0.13,0.14,0.005):
    m=(np.abs(X-x)<0.0025)&(np.abs(Y-0.15)<0.01)
    if m.sum(): print(f'x {x:.3f} zmax {Z[m].max()-0.9:.3f}', end=' | ')
print()
for y in np.arange(0.02,0.24,0.005):
    m=(np.abs(Y-y)<0.0025)&(np.abs(X-0.07)<0.01)
    if m.sum(): print(f'y {y:.3f} zmax {Z[m].max()-0.9:.3f}', end=' | ')
print()
# handle
for y in np.arange(0.02,0.10,0.005):
    m=(np.abs(Y-y)<0.0025)&(np.abs(X-0.0)<0.02)
    if m.sum(): print(f'y {y:.3f} zmax {Z[m].max()-0.9:.3f}', end=' | ')
"


# openrua op 31
cat > /workspace/poses.py <<'EOF'
import numpy as np
from robot import rot_from_axes
c20, s20 = np.cos(np.radians(20)), np.sin(np.radians(20))
R_GRASP = rot_from_axes([c20, 0, -s20], [0, 1, 0])           # approach +x tilted 20 deg down, close along y
c45 = np.sqrt(0.5)
R_PLACE = rot_from_axes([0, c45, -c45], [0, c45, c45])         # approach +y 45deg down, close along tilted z
c40, s40 = np.cos(np.radians(40)), np.sin(np.radians(40))
R_PUSH = rot_from_axes([0, s40, -c40], [1, 0, 0])              # approach +y 40deg from vertical, close along x
BOTTLE = np.array([-0.148, 0.053])
PRE_GRASP = np.array([-0.26, BOTTLE[1], 0.955])
GRASP = np.array([BOTTLE[0], BOTTLE[1], 0.955])
LIFT = np.array([BOTTLE[0], BOTTLE[1], 1.25])
REORIENT = np.array([-0.15, 0.10, 1.25])
PRE_PLACE = np.array([0.005, 0.15, 1.15])
PLACE = np.array([0.005, 0.15, 1.02])
RETREAT = np.array([0.005, 0.10, 1.25])
PRE_PUSH = np.array([0.06, 0.05, 1.10])
PUSH0 = np.array([0.06, 0.05, 0.965])
PUSH1 = np.array([0.06, 0.207, 0.965])
EOF
timeout 300 python3 -c "
from robot import *; from poses import *
r=Robot()
q=r.arm_q()
for name,p,R in [('pre_grasp',PRE_GRASP,R_GRASP),('grasp',GRASP,R_GRASP),('lift',LIFT,R_GRASP),('reorient',REORIENT,R_PLACE),('pre_place',PRE_PLACE,R_PLACE),('place',PLACE,R_PLACE),('retreat',RETREAT,R_PLACE),('pre_push',PRE_PUSH,R_PUSH),('push0',PUSH0,R_PUSH),('push1',PUSH1,R_PUSH)]:
    sol=r.ik_tcp(p,R,seed=q)
    if sol is not None:
        tp,tR=r.tcp(sol)
        print(name, np.round(sol,3), 'tcp err', np.round(np.linalg.norm(tp-p),4), 'Rerr', np.round(np.abs(tR-R).max(),3))
        q=sol
    else:
        print(name,'NO IK')
"


# openrua op 32
cat >> /workspace/robot.py <<'EOF'


REF = np.array([0.0, 0.0, 0.0, -2.0, 0.0, 2.2, 0.785])
SEEDS = [
    np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),
    np.array([0.0, 0.0, 0.0, -2.0, 0.0, 2.2, 0.785]),
    np.array([0.0, 0.5, 0.0, -2.2, 0.0, 2.7, 0.785]),
    np.array([0.0, 1.0, 0.0, -1.8, 0.0, 2.8, 0.785]),
    np.array([0.0, 0.3, 0.0, -1.5, 0.0, 1.8, 0.785]),
    np.array([0.0, -0.3, 0.0, -2.6, 0.0, 2.3, 0.785]),
]


def best_ik(r, tcp_pos, R, extra_seeds=(), n_rand=6, ref=REF, w=(3, 1, 3, 1, 3, 1, 2), verbose=False):
    """Try many seeds; return the solution closest (weighted) to a natural reference config."""
    rng = np.random.default_rng(0)
    seeds = list(extra_seeds) + SEEDS
    for _ in range(n_rand):
        seeds.append(ref + rng.normal(0, 0.4, 7))
    best, bestc = None, 1e9
    for s in seeds:
        sol = r.ik_tcp(tcp_pos, R, seed=s)
        if sol is None:
            continue
        # wrap j7 preference: near 0.785 or -2.356 equivalent? keep simple: distance to ref
        c = float(np.sum(np.array(w) * (sol - ref) ** 2))
        if verbose:
            print("  cand", np.round(sol, 2), round(c, 2))
        if c < bestc:
            best, bestc = sol, c
    return best
EOF
timeout 600 python3 -c "
from robot import *; from poses import *
r=Robot()
for name,p,R in [('pre_grasp',PRE_GRASP,R_GRASP),('grasp',GRASP,R_GRASP),('lift',LIFT,R_GRASP),('reorient',REORIENT,R_PLACE),('place',PLACE,R_PLACE),('push0',PUSH0,R_PUSH),('push1',PUSH1,R_PUSH)]:
    sol=best_ik(r,p,R)
    print(name, None if sol is None else np.round(sol,3))
" 2>&1 | grep -v "IK failed"


# openrua op 33
cat >> /workspace/robot.py <<'EOF'


LINKS = ["panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5",
         "panda_link6", "panda_link7", "panda_hand"]


def fk_links(r, q, links=LINKS):
    req = GetPositionFK.Request()
    req.fk_link_names = list(links)
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk_cli.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    out = {}
    for name, ps in zip(res.fk_link_names, res.pose_stamped):
        p = ps.pose
        out[name] = (np.array([p.position.x, p.position.y, p.position.z]),
                     quat_to_R([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))
    return out


def hand_points(pos, R, finger=0.04):
    """Sample points of the hand/finger volume in world (approximate box model)."""
    pts = []
    # palm box: x_h +-0.032, y_h +-0.1, z_h 0..0.066
    for xs in (-0.032, 0.032):
        for ys in (-0.1, -0.05, 0, 0.05, 0.1):
            for zs in (0.0, 0.033, 0.066):
                pts.append(pos + R @ np.array([xs, ys, zs]))
    # fingers: at y = +-finger, x +-0.01, z 0.066..0.112
    for ys in (-finger, finger):
        for zs in (0.08, 0.1, 0.112):
            for xs in (-0.01, 0.01):
                pts.append(pos + R @ np.array([xs, ys, zs]))
    return np.array(pts)


def link_clearance(r, q, finger=0.04, verbose=False):
    """Return min clearance summary vs table (z=0.90) and cabinet box, for arm links + hand."""
    fk = fk_links(r, q)
    worst = []
    for name, (p, R) in fk.items():
        if name == "panda_hand":
            pts = hand_points(p, R, finger)
            rad = 0.0
        else:
            pts = p[None, :]
            rad = 0.06 if name in ("panda_link4", "panda_link5", "panda_link6", "panda_link7") else 0.08
        # table
        dz = pts[:, 2].min() - rad - 0.90
        # cabinet box x[-0.12,0.13] y[0.213,0.42] z[0.90,1.13]
        inside = ((pts[:, 0] > -0.12 - rad) & (pts[:, 0] < 0.13 + rad) & (pts[:, 1] > 0.213 - rad)
                  & (pts[:, 1] < 0.42 + rad) & (pts[:, 2] < 1.13 + rad)).any()
        worst.append((name, round(float(dz), 3), bool(inside)))
    if verbose:
        for w in worst: print("   ", w)
    return worst
EOF
sed -i 's/R_GRASP = rot_from_axes(\[c20, 0, -s20\], \[0, 1, 0\])/R_GRASP = rot_from_axes([c20, 0, -s20], [0, -1, 0])/' /workspace/poses.py
timeout 600 python3 -c "
from robot import *; from poses import *
r=Robot()
for name,p,R in [('pre_grasp',PRE_GRASP,R_GRASP),('grasp',GRASP,R_GRASP),('lift',LIFT,R_GRASP),('reorient',REORIENT,R_PLACE),('place',PLACE,R_PLACE),('push0',PUSH0,R_PUSH),('push1',PUSH1,R_PUSH)]:
    sol=best_ik(r,p,R)
    print(name, None if sol is None else np.round(sol,3))
    if sol is not None: link_clearance(r,sol,verbose=True)
" 2>&1 | grep -v "IK failed"


# openrua op 34
timeout 300 python3 -c "
import numpy as np
def dh(a,d,alpha,theta):
    ca,sa,ct,st=np.cos(alpha),np.sin(alpha),np.cos(theta),np.sin(theta)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
P=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
def fk(q):
    T=np.eye(4)
    for (a,d,al),th in zip(P,q): T=T@dh(a,d,al,th)
    T=T@dh(0,0.107,0,0)@dh(0,0,0,-np.pi/4)
    return T
target=np.array([0.415,0.0,0.078]); tz=np.array([np.cos(np.radians(20)),0,-np.sin(np.radians(20))])
best=[]
for j2 in np.arange(-1.7,1.71,0.05):
  for j4 in np.arange(-3.0,-0.1,0.05):
    for j6 in np.arange(0,3.7,0.05):
      T=fk([0,j2,0,j4,0,j6,0.785])
      e=np.linalg.norm(T[:3,3]-target)+0.3*np.linalg.norm(T[:3,2]-tz)
      best.append((e,j2,j4,j6))
best.sort(); 
for b in best[:5]:
    T=fk([0,b[1],0,b[2],0,b[3],0.785]); print(np.round(b,3), np.round(T[:3,3],3), np.round(T[:3,2],3), 'xh',np.round(T[:3,0],2))
"


# openrua op 35
timeout 600 python3 -c "
from robot import *
r=Robot()
for deg in (30,45,60,70):
    c,s=np.cos(np.radians(deg)),np.sin(np.radians(deg))
    R=rot_from_axes([s,0,-c],[0,-1,0])
    for name,p in [('grasp',[-0.148,0.053,0.955]),('pre',[-0.26,0.053,0.955])]:
        sol=best_ik(r,p,R)
        print(deg,name, None if sol is None else np.round(sol,2))
" 2>&1 | grep -v "IK failed"


# openrua op 36
timeout 600 python3 -c "
from robot import *
r=Robot()
seed=np.array([0.19,0.54,-0.04,-2.55,0.14,3.62,0.81])
for deg in (35,40,45,50):
    c,s=np.cos(np.radians(deg)),np.sin(np.radians(deg))
    R=rot_from_axes([s,0,-c],[0,-1,0])
    for name,p in [('grasp',[-0.148,0.053,0.955]),('grasp94',[-0.148,0.053,0.94]),('pre',[-0.148-0.1*s,0.053,0.955+0.1*c])]:
        sol=best_ik(r,p,R,extra_seeds=[seed],n_rand=10)
        print(deg,name, None if sol is None else np.round(sol,2))
" 2>&1 | grep -v "IK failed"


# openrua op 37
python3 -c "
import numpy as np
for cam in ('agent','bird'):
    P=np.load(f'{cam}_xyz.npy').reshape(-1,3)
    m=np.isfinite(P).all(1)&(np.abs(P[:,0]+0.148)<0.05)&(np.abs(P[:,1]-0.053)<0.05)&(P[:,2]>0.905)
    Q=P[m]
    print(cam, len(Q))
    for z0 in np.arange(0.91,1.07,0.01):
        s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
        if len(s): print(f'  z {z0:.2f}: n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"


# openrua op 38
timeout 300 python3 -c "
from robot import *
r=Robot()
for q in ([-1.456,1.496,1.002,-2.513,2.897,2.102,2.609],[-1.437,1.156,0.865,-2.828,2.79,1.89,2.897],[0.19,0.54,-0.04,-2.55,0.14,3.62,0.81]):
    q=np.array(q)
    print('q',q)
    for name,(p,R) in fk_links(r,q).items(): print('  ',name,np.round(p,3))
    print('  clearance',link_clearance(r,q))
    print('  tcp',np.round(r.tcp(q),3))
" 2>&1 | tail -40


# openrua op 39
python3 -c "
import numpy as np
P=np.load('bird_xyz.npy').reshape(-1,3)
m=np.isfinite(P).all(1)
P=P[m]
# height map over x -0.2..0.2, y 0.0..0.25 at 1cm
xs=np.arange(-0.20,0.20,0.01); ys=np.arange(0.0,0.26,0.01)
print('     '+' '.join(f'{x*100:+4.0f}' for x in xs))
for y in ys[::-1]:
    row=[]
    for x in xs:
        s=P[(np.abs(P[:,0]-x)<0.005)&(np.abs(P[:,1]-y)<0.005)]
        row.append(f'{s[:,2].max()*100-90:4.0f}' if len(s) else '   .')
    print(f'{y*100:4.0f} '+' '.join(row))
"


# openrua op 40
cat > /workspace/plan.py <<'EOF'
from robot import *
c20,s20=np.cos(np.radians(20)),np.sin(np.radians(20))
c45=np.cos(np.radians(45))
c40,s40=np.cos(np.radians(40)),np.sin(np.radians(40))
R_GRASP=rot_from_axes([c20,0,-s20],[0,-1,0])
R_PLACE=rot_from_axes([0,c45,-c45],[0,c45,c45])
R_PUSH=rot_from_axes([0,s40,-c40],[1,0,0])
A=np.array([c20,0,-s20])
GRASP=np.array([-0.148,0.053,0.95])
POSES=[('pre_grasp',GRASP-0.11*A,R_GRASP),
       ('grasp',GRASP,R_GRASP),
       ('lift',[-0.148,0.053,1.15],R_GRASP),
       ('reorient',[-0.10,0.08,1.20],R_PLACE),
       ('pre_place',[0.03,0.13,1.15],R_PLACE),
       ('place',[0.03,0.13,1.02],R_PLACE),
       ('retreat',[0.03,0.09,1.25],R_PLACE),
       ('pre_push',[0.07,0.05,1.10],R_PUSH),
       ('push0',[0.07,0.05,0.965],R_PUSH),
       ('push1',[0.07,0.212,0.965],R_PUSH)]
def chain(r,start_q):
    qs={}; prev=start_q
    for name,p,R in POSES:
        sol=best_ik(r,np.array(p),R,extra_seeds=[prev],n_rand=8)
        qs[name]=sol
        print(name,None if sol is None else np.round(sol,3))
        if sol is not None: prev=sol
    return qs
def seg_check(r,q0,q1,n=8,finger=0.04):
    worst=(9,None)
    for t in np.linspace(0,1,n+1):
        q=q0+(q1-q0)*t
        cl=link_clearance(r,q,finger=finger)
        for name,c,inside in cl:
            if c<worst[0]: worst=(c,f'{name}@t={t:.2f}')
            if inside: print('   INSIDE CABINET',name,'t',round(t,2))
    return worst
if __name__=='__main__':
    r=Robot()
    q0=r.arm_q()
    print('current',np.round(q0,3))
    qs=chain(r,q0)
    names=['current']+[n for n,_,_ in POSES]
    allq={'current':q0,**qs}
    for a,b in zip(names[:-1],names[1:]):
        if allq[a] is None or allq[b] is None: continue
        print(a,'->',b,'max dq',np.round(np.abs(allq[b]-allq[a]).max(),2),'worst clearance',seg_check(r,allq[a],allq[b]))
    np.save('plan_q.npy',allq,allow_pickle=True)
EOF
timeout 900 python3 plan.py 2>&1 | grep -v "IK failed"

# openrua op 41
grep -n "def link_clearance" -A 30 /workspace/robot.py; grep -n "def best_ik" -A 30 /workspace/robot.py | head -50; grep -n "def hand_points" -A 15 /workspace/robot.py

# openrua op 42
timeout 300 python3 -c "
from robot import *
from plan import *
r=Robot()
allq=np.load('plan_q.npy',allow_pickle=True).item()
q=allq['push0']
fk=fk_links(r,q)
p,R=fk['panda_hand']
print('hand',np.round(p,3)); print(np.round(R,3))
print('tcp',np.round(r.tcp(q),3))
pts=hand_points(p,R)
print(np.round(pts[(pts[:,1]>0.213)],3))
" 2>&1 | tail -20

# openrua op 43
timeout 300 python3 -c "
from robot import *
from plan import *
r=Robot()
allq=np.load('plan_q.npy',allow_pickle=True).item()
q=allq['push0']
fk=fk_links(r,q)
p,R=fk['panda_hand']
pts=hand_points(p,R)
print(np.round(pts[(pts[:,1]>0.213)],3))
print(pts[:,1].max(), pts[:,2].min())
" 2>&1 | tail -20

# openrua op 44
timeout 300 python3 -c "
from robot import *
from plan import *
r=Robot()
allq=np.load('plan_q.npy',allow_pickle=True).item()
print(link_clearance(r,allq['push0']))
print(link_clearance(r,allq['push1']))
print(seg_check(r,allq['pre_push'],allq['push0']))
" 2>&1 | tail -20

# openrua op 45
timeout 600 python3 -c "
from robot import *
from plan import *
r=Robot()
LIM=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
rng=np.random.default_rng(1)
cands=[]
for i in range(40):
    s=REF+rng.normal(0,0.6,7) if i>0 else np.array([-1.456,1.496,1.002,-2.513,2.897,2.102,2.609])
    sol=r.ik_tcp(GRASP,R_GRASP,seed=s)
    if sol is None: continue
    margin=np.minimum(sol-LIM[:,0],LIM[:,1]-sol).min()
    if not any(np.abs(sol-c).max()<0.05 for c,_ in cands):
        cands.append((sol,margin))
for sol,m in sorted(cands,key=lambda c:-c[1]):
    print(np.round(sol,3),'margin',round(m,3),'worst',min(link_clearance(r,sol),key=lambda w:w[1]))
" 2>&1 | grep -v "IK failed"

# openrua op 46
timeout 600 python3 -c "
from robot import *
from plan import *
r=Robot()
for q in ([1.226,0.871,-0.837,-2.486,-2.836,2.182,-1.737],[1.694,1.486,-1.012,-2.427,-2.818,2.408,-1.035],[-1.949,-0.842,2.325,-2.488,-2.841,2.171,-1.77]):
    q=np.array(q); print('q',q)
    for name,(p,R) in fk_links(r,q).items(): print('  ',name,np.round(p,3))
    print('  ',link_clearance(r,q))
" 2>&1 | grep -v Traceback

# openrua op 47
cat > /workspace/plan2.py <<'EOF'
from robot import *
from plan import R_GRASP,R_PLACE,R_PUSH,A,GRASP,seg_check
Q_GRASP=np.array([1.226,0.871,-0.837,-2.486,-2.836,2.182,-1.737])
POSES=[('above',[-0.20,0.053,1.15],R_GRASP,'grasp'),
       ('pre_grasp',GRASP-0.11*A,R_GRASP,'grasp'),
       ('grasp',GRASP,R_GRASP,None),
       ('lift',[-0.148,0.053,1.15],R_GRASP,'grasp'),
       ('reorient',[-0.10,0.08,1.20],R_PLACE,'ref'),
       ('pre_place',[0.03,0.13,1.15],R_PLACE,'prev'),
       ('place',[0.03,0.13,1.02],R_PLACE,'prev'),
       ('retreat',[0.03,0.09,1.25],R_PLACE,'prev'),
       ('pre_push',[0.07,0.05,1.10],R_PUSH,'ref'),
       ('push0',[0.07,0.05,0.965],R_PUSH,'prev'),
       ('push1',[0.07,0.212,0.965],R_PUSH,'prev')]
def build(r):
    qs={'grasp':Q_GRASP}; prev=None
    for name,p,R,mode in POSES:
        if mode is None: prev=Q_GRASP; continue
        ref={'grasp':Q_GRASP,'prev':prev,'ref':REF}[mode]
        seeds=[ref] if mode!='ref' else [prev]
        sol=best_ik(r,np.array(p),R,extra_seeds=seeds,n_rand=8,ref=ref)
        qs[name]=sol; prev=sol
        print(name,None if sol is None else np.round(sol,3), 'tcp',np.round(r.tcp(sol),3) if sol is not None else '')
    return qs
if __name__=='__main__':
    r=Robot()
    q0=r.arm_q(); print('current',np.round(q0,3))
    qs=build(r); qs['current']=q0
    names=['current']+[n for n,_,_,_ in POSES]
    for a,b in zip(names[:-1],names[1:]):
        if qs[a] is None or qs[b] is None: continue
        # tcp path
        tcps=[r.tcp(qs[a]+(qs[b]-qs[a])*t) for t in np.linspace(0,1,9)]
        tcps=np.array(tcps)
        print(a,'->',b,'max dq',np.round(np.abs(qs[b]-qs[a]).max(),2),'worst',seg_check(r,qs[a],qs[b]),
              'tcp z min',np.round(tcps[:,2].min(),3),'tcp y max',np.round(tcps[:,1].max(),3))
    np.save('plan_q.npy',qs,allow_pickle=True)
EOF
timeout 900 python3 plan2.py 2>&1 | grep -v "IK failed"

# openrua op 48
grep -n "def tcp" -A 6 /workspace/robot.py

# openrua op 49
sed -i 's/np.round(r.tcp(sol),3)/np.round(r.tcp(sol)[0],3)/; s/tcps=\[r.tcp(qs\[a\]+(qs\[b\]-qs\[a\])\*t) for t/tcps=[r.tcp(qs[a]+(qs[b]-qs[a])*t)[0] for t/' plan2.py && timeout 900 python3 plan2.py 2>&1 | grep -v "IK failed"

# openrua op 50
timeout 600 python3 -c "
from robot import *
from plan2 import *
r=Robot()
prev=Q_GRASP
for z in (0.98,1.01,1.04,1.07,1.10,1.13):
    sol=best_ik(r,np.array([-0.148,0.053,z]),R_GRASP,extra_seeds=[prev],n_rand=4,ref=prev)
    print(z,np.round(sol,3)); prev=sol
" 2>&1 | grep -v "IK failed"

# openrua op 51
timeout 600 python3 -c "
from robot import *
from plan2 import *
r=Robot()
prev=Q_GRASP
seq=[(0.98,20),(1.01,30),(1.04,40),(1.07,50),(1.10,60),(1.14,70),(1.18,80)]
out=[]
for z,deg in seq:
    c,s=np.cos(np.radians(deg)),np.sin(np.radians(deg))
    R=rot_from_axes([s,0,-c],[0,-1,0])
    sol=best_ik(r,np.array([-0.148,0.053,z]),R,extra_seeds=[prev],n_rand=4,ref=prev)
    if sol is None: print(z,deg,'NONE'); continue
    print(z,deg,np.round(sol,3),'dq',np.round(np.abs(sol-prev).max(),2),'worst',min(link_clearance(r,sol),key=lambda w:w[1]))
    prev=sol; out.append(sol)
np.save('lift_q.npy',np.array(out))
" 2>&1 | grep -v "IK failed"

# openrua op 52
ls *.py && grep -ln "def dh\|def fk_dh\|0.0825" *.py

# openrua op 53
cat > /workspace/dh.py <<'EOF'
import numpy as np
# Panda modified DH (Craig): a, d, alpha per joint; flange d=0.107; hand rotated -45deg about z
DH=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
def T_dh(a,d,alpha,theta):
    ca,sa,ct,st=np.cos(alpha),np.sin(alpha),np.cos(theta),np.sin(theta)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
def fk_hand(q, base=np.array([-0.66,0,0.912])):
    T=np.eye(4); T[:3,3]=base
    for (a,d,al),th in zip(DH,q): T=T@T_dh(a,d,al,th)
    T=T@T_dh(0,0.107,0,0)          # flange
    T=T@T_dh(0,0,0,-np.pi/4)       # hand
    return T[:3,3],T[:3,:3]
def fk_links(q, base=np.array([-0.66,0,0.912])):
    T=np.eye(4); T[:3,3]=base; out=[]
    for (a,d,al),th in zip(DH,q):
        T=T@T_dh(a,d,al,th); out.append(T[:3,3].copy())
    return out
if __name__=='__main__':
    q=np.array([0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483])
    p,R=fk_hand(q); print(p); print(np.round(R,3))
EOF
python3 dh.py

# openrua op 54
cat > /workspace/grid.py <<'EOF'
import numpy as np, sys
from dh import fk_hand
tcp=np.array([-0.148,0.053,0.95]); deg=float(sys.argv[1]) if len(sys.argv)>1 else 20
a=np.array([np.cos(np.radians(deg)),0,-np.sin(np.radians(deg))])
target=tcp-0.1034*a
j1=np.arctan2(0.053,0.512)
res=[]
for j2 in np.arange(-1.7,1.71,0.1):
  for j4 in np.arange(-3.0,-0.1,0.1):
    for j6 in np.arange(0,3.71,0.1):
      p,R=fk_hand([j1,j2,0,j4,0,j6,0.785])
      err=np.linalg.norm(p-target); ang=np.degrees(np.arccos(np.clip(R[:,2]@a,-1,1)))
      res.append((err,ang,j2,j4,j6))
res=np.array(res)
m=res[:,0]<0.03
print('within 3cm:',m.sum())
for r in res[m][np.argsort(res[m][:,1])][:15]: print(np.round(r,2))
EOF
python3 grid.py 20

# openrua op 55
sed -n 124,175p /workspace/robot.py

# openrua op 56
python3 - <<'EOF'
import re
s=open('robot.py').read()
s=s.replace("def ik(self, hand_pos_world, R, seed=None, timeout=30):","def ik(self, hand_pos_world, R, seed=None, timeout=30, avoid_collisions=False):")
s=s.replace("req.ik_request.avoid_collisions = False","req.ik_request.avoid_collisions = bool(avoid_collisions)")
s=s.replace("""    def ik_tcp(self, tcp_pos_world, R, seed=None):
        hand = np.array(tcp_pos_world) - TCP_OFF * R[:, 2]
        return self.ik(hand, R, seed)""","""    def ik_tcp(self, tcp_pos_world, R, seed=None, avoid_collisions=False):
        hand = np.array(tcp_pos_world) - TCP_OFF * R[:, 2]
        return self.ik(hand, R, seed, avoid_collisions=avoid_collisions)""")
open('robot.py','w').write(s)
EOF
timeout 300 python3 -c "
from robot import *
from plan2 import *
r=Robot()
for q in (Q_GRASP, np.array([1.494,0.633,-0.959,-2.828,-2.755,2.1,-1.825]), np.array([1.016,-0.392,-0.96,-2.789,-2.843,2.442,-2.86])):
    p,R=r.tcp(q)
    for ac in (False,True):
        sol=r.ik_tcp(p,R,seed=q,avoid_collisions=ac)
        print(ac, None if sol is None else np.round(sol-q,3))
" 2>&1 | grep -v Traceback

# openrua op 57
cat >> /workspace/robot.py <<'EOF'


def ik_near(r, tcp_pos, R, prev, max_dq=0.5, tries=10, avoid_collisions=True, rng=np.random.default_rng(3)):
    """IK solution continuous with prev (max joint jump < max_dq), or None."""
    best = None
    for i in range(tries):
        seed = prev if i == 0 else prev + rng.normal(0, 0.05 * i, 7)
        sol = r.ik_tcp(tcp_pos, R, seed=seed, avoid_collisions=avoid_collisions)
        if sol is None:
            continue
        d = np.abs(sol - prev).max()
        if d < max_dq and (best is None or d < np.abs(best - prev).max()):
            best = sol
            if d < max_dq / 2:
                break
    return best


def tilt_R(deg, closing=(0, -1, 0)):
    """side-grasp orientation: approach +x tilted deg below horizontal."""
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return rot_from_axes([c, 0, -s], closing)
EOF
timeout 900 python3 -c "
from robot import *
from plan2 import Q_GRASP
r=Robot()
prev=Q_GRASP
for z in (0.96,0.97,0.98,0.99,1.0):
    sol=ik_near(r,[-0.148,0.053,z],tilt_R(20),prev)
    print(z,20,None if sol is None else np.round(sol,3)); 
    if sol is not None: prev=sol
for deg in range(25,95,5):
    sol=ik_near(r,[-0.148,0.053,1.0],tilt_R(deg),prev)
    print(1.0,deg,None if sol is None else np.round(sol,3))
    if sol is not None: prev=sol
" 2>&1 | grep -v "IK failed"

# openrua op 58
timeout 900 python3 -c "
from robot import *
r=Robot()
prev=np.array([1.299,0.76,-0.946,-2.535,-2.897,2.238,-1.766])
for deg in range(22,92,3):
    sol=ik_near(r,[-0.148,0.053,0.99],tilt_R(deg),prev,max_dq=0.8,tries=15)
    print(0.99,deg,None if sol is None else np.round(sol,3), '' if sol is None else np.round(np.abs(sol-prev).max(),2))
    if sol is not None: prev=sol
" 2>&1 | grep -v "IK failed"

# openrua op 59
timeout 900 python3 -c "
from robot import *
r=Robot()
prev=np.array([1.299,0.76,-0.946,-2.535,-2.897,2.238,-1.766])
for deg in (20,21,22,25):
  for ac in (False,True):
    sol=r.ik_tcp([-0.148,0.053,0.99],tilt_R(deg),seed=prev,avoid_collisions=ac)
    print(deg,ac,None if sol is None else np.round(sol,3))
" 2>&1 | grep -v "IK failed"

# openrua op 60
timeout 900 python3 -c "
from robot import *
from plan2 import Q_GRASP
r=Robot()
prev=Q_GRASP
for deg in (22,25,30,35,40,50,60,70):
    sol=r.ik_tcp([-0.148,0.053,0.95],tilt_R(deg),seed=prev)
    print(deg,None if sol is None else np.round(sol,3))
# also try pitching 'up' (less tilt)
for deg in (15,10,5,0):
    sol=r.ik_tcp([-0.148,0.053,0.95],tilt_R(deg),seed=prev)
    print(deg,None if sol is None else np.round(sol,3))
" 2>&1 | grep -v "IK failed"

# openrua op 61
timeout 900 python3 -c "
from robot import *
r=Robot()
prev=np.array([1.222,1.054,-0.702,-2.479,-2.469,1.791,-1.743])
for z in np.arange(0.95,1.30,0.025):
    sol=ik_near(r,[-0.148,0.053,z],tilt_R(0),prev,max_dq=0.5,tries=6)
    print(round(z,3),None if sol is None else np.round(sol,3), '' if sol is None else min(link_clearance(r,sol),key=lambda w:w[1]))
    if sol is not None: prev=sol
" 2>&1 | grep -v "IK failed"

# openrua op 62
cat > /workspace/plan3.py <<'EOF'
from robot import *
TILT=10
R_G=tilt_R(TILT)
A=R_G[:,2]
c45=np.cos(np.radians(45)); c40,s40=np.cos(np.radians(40)),np.sin(np.radians(40))
R_PLACE=rot_from_axes([0,c45,-c45],[0,c45,c45])
R_PUSH=rot_from_axes([0,s40,-c40],[1,0,0])
GRASP=np.array([-0.148,0.053,0.955])
PRE=GRASP-0.11*A
ABOVE=np.array([-0.22,0.053,1.15])
LIFT=np.array([-0.148,0.053,1.15])
REOR=np.array([-0.10,0.08,1.20]); PREPL=np.array([0.03,0.13,1.15]); PLACE=np.array([0.03,0.13,1.02]); RETR=np.array([0.03,0.09,1.25])
PREPUSH=np.array([0.07,0.05,1.10]); PUSH0=np.array([0.07,0.05,0.965]); PUSH1=np.array([0.07,0.212,0.965])

def line(r,p0,p1,R,prev,n,max_dq=0.5):
    out=[]
    for t in np.linspace(0,1,n+1)[1:]:
        p=p0+(p1-p0)*t
        sol=ik_near(r,p,R,prev,max_dq=max_dq,tries=8)
        if sol is None: print('  line fail at',np.round(p,3)); return None
        out.append(sol); prev=sol
    return out

def bottle_pts(tcp,R):
    xh=R[:,0]; pts=[]
    for s in np.linspace(-0.055,0.103,6):
        for ang in np.linspace(0,2*np.pi,6,endpoint=False):
            pts.append(tcp+s*xh+0.02*(np.cos(ang)*R[:,1]+np.sin(ang)*R[:,2]))
    return np.array(pts)

def obstacles(pts):
    """return list of obstacle names hit by any point"""
    hits=[]
    x,y,z=pts[:,0],pts[:,1],pts[:,2]
    if (z<0.905).any(): hits.append('table')
    if ((x>-0.125)&(x<0.135)&(y>0.065)&(y<0.215)&(z<0.985)&~((x>-0.09)&(x<0.10)&(y>0.095)&(y<0.21))).any(): hits.append('drawer_walls')
    if ((x>-0.12)&(x<0.13)&(y>0.21)&(z<1.135)).any(): hits.append('cabinet')
    if ((x>-0.055)&(x<0.045)&(y>0.175)&(y<0.215)&(z>0.995)&(z<1.105)).any(): hits.append('handles')
    if ((x>-0.045)&(x<0.045)&(y>0.035)&(y<0.062)&(z<0.96)).any(): hits.append('low_handle')
    if (((x)**2+(y+0.05)**2<0.1**2)&(z<1.03)).any(): hits.append('bowl')
    return hits

def check_path(r,qs,held=False,n_sub=4,label=''):
    worst=9; bad=set()
    for a,b in zip(qs[:-1],qs[1:]):
        for t in np.linspace(0,1,n_sub,endpoint=False):
            q=a+(b-a)*t
            fk=fk_links(r,q)
            p,R=fk['panda_hand']
            hp=hand_points(p,R,0.02 if held else 0.04)
            for h in obstacles(hp): bad.add('hand:'+h)
            tcp=p+TCP_OFF*R[:,2]
            if held:
                for h in obstacles(bottle_pts(tcp,R)): bad.add('bottle:'+h)
            for name,c,inside in link_clearance(r,q):
                worst=min(worst,c)
                if inside: bad.add(name+':cabinet')
    print(f'{label}: worst link clearance {worst:.3f} hits {sorted(bad)}')
    return bad

if __name__=='__main__':
    r=Robot(); q0=r.arm_q()
    plan={}
    qg=best_ik(r,GRASP,R_G,extra_seeds=[np.array([1.226,0.871,-0.837,-2.486,-2.836,2.182,-1.737])],n_rand=6,ref=np.array([1.226,0.871,-0.837,-2.486,-2.836,2.182,-1.737]))
    print('grasp',np.round(qg,3),link_clearance(r,qg))
    plan['grasp']=[qg]
    # backwards: grasp -> pre -> above
    seg=line(r,GRASP,PRE,R_G,qg,4); plan['approach']=seg[::-1]  # pre..grasp order later
    seg2=line(r,PRE,ABOVE,R_G,seg[-1],6); plan['descend']=seg2[::-1]
    print('above',np.round(seg2[-1],3))
    lift=line(r,GRASP,LIFT,R_G,qg,6); plan['lift']=lift
    print('lift',np.round(lift[-1],3))
    qre=best_ik(r,REOR,R_PLACE,n_rand=8); print('reorient',np.round(qre,3))
    plan['reorient']=[qre]
    pp=line(r,REOR,PREPL,R_PLACE,qre,3); plan['preplace']=pp
    pl=line(r,PREPL,PLACE,R_PLACE,pp[-1],4); plan['place']=pl
    rt=line(r,PLACE,RETR,R_PLACE,pl[-1],3); plan['retreat']=rt
    qpp=best_ik(r,PREPUSH,R_PUSH,extra_seeds=[rt[-1]],n_rand=8); print('prepush',np.round(qpp,3)); plan['prepush']=[qpp]
    p0=line(r,PREPUSH,PUSH0,R_PUSH,qpp,3); plan['push0']=p0
    p1=line(r,PUSH0,PUSH1,R_PUSH,p0[-1],6); plan['push1']=p1
    np.save('plan3.npy',plan,allow_pickle=True)
    # checks
    check_path(r,[q0]+plan['descend'][:1],label='current->above')
    check_path(r,plan['descend']+plan['approach'],label='above->pre->grasp')
    check_path(r,[qg]+plan['lift'],held=True,label='lift')
    check_path(r,[plan['lift'][-1],qre],held=True,n_sub=12,label='lift->reorient (family switch)')
    check_path(r,[qre]+pp+pl,held=True,label='reorient->place')
    check_path(r,[pl[-1]]+rt+[qpp]+p0+p1,label='retreat->push')
EOF
timeout 1500 python3 plan3.py 2>&1 | grep -v "IK failed"

# openrua op 63
sed -i 's/ABOVE=np.array(\[-0.22,0.053,1.15\])/ABOVE=np.array([-0.22,0.053,1.10])/; s/LIFT=np.array(\[-0.148,0.053,1.15\])/LIFT=np.array([-0.148,0.053,1.10])/' plan3.py && timeout 1500 python3 plan3.py 2>&1 | grep -v "IK failed"

# openrua op 64
python3 -c "
import numpy as np
P=np.load('bird_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# bowl: points z>0.95 in region x -0.2..0.2, y -0.25..0.05
m=(P[:,2]>0.95)&(P[:,2]<1.1)&(np.abs(P[:,0])<0.2)&(P[:,1]>-0.25)&(P[:,1]<0.05)
Q=P[m]; print('bowl pts',len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'z max',Q[:,2].max().round(3))
r=np.hypot(Q[:,0]-0.0,Q[:,1]+0.05); print('r percentiles',np.percentile(r,[50,90,99,100]).round(3))
# rim height by ring
for rr in np.arange(0.05,0.11,0.01):
    s=Q[(r>=rr)&(r<rr+0.01)]
    if len(s): print(f' r {rr:.2f}: n={len(s)} zmax {s[:,2].max():.3f}')
A=np.load('agent_xyz.npy').reshape(-1,3); A=A[np.isfinite(A).all(1)]
# handles on cabinet face: x -0.06..0.05, y 0.17..0.215
m=(A[:,0]>-0.07)&(A[:,0]<0.06)&(A[:,1]>0.15)&(A[:,1]<0.212)&(A[:,2]>0.99)
H=A[m]
for z0 in np.arange(0.99,1.13,0.01):
    s=H[(H[:,2]>=z0)&(H[:,2]<z0+0.01)]
    if len(s): print(f'z {z0:.2f}: n={len(s)} y {s[:,1].min():.3f}..{s[:,1].max():.3f} x {s[:,0].min():.3f}..{s[:,0].max():.3f}')
"


# openrua op 65
python3 - <<'EOF'
s=open('plan3.py').read()
s=s.replace("R_PUSH=rot_from_axes([0,s40,-c40],[1,0,0])",
"R_PUSH=rot_from_axes([0,s40,-c40],[1,0,0])\nc50,s50=np.cos(np.radians(50)),np.sin(np.radians(50))\nR_PUSH2=rot_from_axes([0,s50,-c50],[0,c50,s50])")
s=s.replace("PREPUSH=np.array([0.07,0.05,1.10]); PUSH0=np.array([0.07,0.05,0.965]); PUSH1=np.array([0.07,0.212,0.965])",
"PREPUSH=np.array([0.09,0.05,1.10]); PUSH0=np.array([0.09,0.05,0.975]); PUSH1=np.array([0.09,0.13,0.975])\nPREPUSH2=np.array([0.11,0.14,1.10]); PUSH2=np.array([0.11,0.14,0.965]); PUSH3=np.array([0.11,0.207,0.965]); PUSHUP=np.array([0.11,0.15,1.15])")
s=s.replace("if (((x)**2+(y+0.05)**2<0.1**2)&(z<1.03)).any(): hits.append('bowl')",
"if (((x)**2+(y+0.05)**2<0.085**2)&(z<1.025)).any(): hits.append('bowl')")
s=s.replace("""    p0=line(r,PREPUSH,PUSH0,R_PUSH,qpp,3); plan['push0']=p0
    p1=line(r,PUSH0,PUSH1,R_PUSH,p0[-1],6); plan['push1']=p1""",
"""    p0=line(r,PREPUSH,PUSH0,R_PUSH,qpp,3); plan['push0']=p0
    p1=line(r,PUSH0,PUSH1,R_PUSH,p0[-1],4); plan['push1']=p1
    qpp2=best_ik(r,PREPUSH2,R_PUSH2,extra_seeds=[p1[-1]],n_rand=8); print('prepush2',np.round(qpp2,3)); plan['prepush2']=[qpp2]
    p2=line(r,PREPUSH2,PUSH2,R_PUSH2,qpp2,3); plan['push2']=p2
    p3=line(r,PUSH2,PUSH3,R_PUSH2,p2[-1],4); plan['push3']=p3
    pu=line(r,PUSH3,PUSHUP,R_PUSH2,p3[-1],3); plan['pushup']=pu""")
s=s.replace("""    check_path(r,[qg]+plan['lift'],held=True,label='lift')""","""    check_path(r,plan['lift'],held=True,label='lift')""")
s=s.replace("""    check_path(r,[pl[-1]]+rt+[qpp]+p0+p1,label='retreat->push')""",
"""    check_path(r,[pl[-1]]+rt+[qpp],label='retreat->prepush')
    check_path(r,[qpp]+p0+p1,label='push phase1')
    check_path(r,[p1[-1],qpp2],label='push1->prepush2',n_sub=12)
    check_path(r,[qpp2]+p2+p3+pu,label='push phase2')""")
open('plan3.py','w').write(s)
EOF
timeout 1700 python3 plan3.py 2>&1 | grep -v "IK failed"

# openrua op 66
cat > /workspace/check_detail.py <<'EOF'
from robot import *
from plan3 import *
import sys
r=Robot()
plan=np.load('plan3.npy',allow_pickle=True).item()
def detail(qs,label,held=False):
    print('==',label)
    for i,(a,b) in enumerate(zip(qs[:-1],qs[1:])):
        for t in np.linspace(0,1,4,endpoint=False):
            q=a+(b-a)*t
            p,R=r.fk(q)
            hp=hand_points(p,R,0.02 if held else 0.04)
            tcp=p+TCP_OFF*R[:,2]
            for h in obstacles(hp):
                m=[pt for pt in hp if h in obstacles(pt[None,:])]
                print(f'  seg{i} t={t:.2f} tcp {np.round(tcp,3)} hand:{h} pts {np.round(m[0],3)} .. n={len(m)}')
            if held:
                for h in obstacles(bottle_pts(tcp,R)): print(f'  seg{i} t={t:.2f} bottle:{h}')
pl=plan['place']; pp=plan['preplace']; rt=plan['retreat']
detail([plan['reorient'][0]]+pp+pl,'reorient->place',held=True)
detail([pl[-1]]+rt+plan['prepush'],'retreat->prepush')
detail(plan['prepush']+plan['push0']+plan['push1'],'push1')
detail(plan['prepush2']+plan['push2']+plan['push3']+plan['pushup'],'push2')
EOF
timeout 900 python3 check_detail.py 2>&1 | grep -v "IK failed"

# openrua op 67
timeout 300 python3 -c "
from robot import *
from plan3 import *
r=Robot()
plan=np.load('plan3.npy',allow_pickle=True).item()
q=plan['push0'][-1]
p,R=r.fk(q); print('hand origin',np.round(p,3)); print(np.round(R,3)); print('R_PUSH'); print(np.round(R_PUSH,3))
hp=hand_points(p,R); print(np.round(hp[:6],3))
" 2>&1 | grep -v "IK failed"

# openrua op 68
python3 -c "
import numpy as np
P=np.load('bird_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(np.hypot(P[:,0],P[:,1]+0.05)<0.12)&(P[:,2]>0.85)&(P[:,0]>-0.15)
Q=P[m]; r=np.hypot(Q[:,0],Q[:,1]+0.05)
for rr in np.arange(0,0.12,0.01):
    s=Q[(r>=rr)&(r<rr+0.01)]
    if len(s): print(f'r {rr:.2f}: n={len(s)} z {s[:,2].min():.3f}..{s[:,2].max():.3f}')
"


# openrua op 69
python3 -c "
import numpy as np
P=np.load('bird_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.0)&(P[:,2]<1.03)&(P[:,0]>-0.15)&(P[:,0]<0.15)&(P[:,1]>-0.25)&(P[:,1]<0.06)
Q=P[m]; print(len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3))
cx,cy=(Q[:,0].min()+Q[:,0].max())/2,(Q[:,1].min()+Q[:,1].max())/2; print('center',round(cx,3),round(cy,3))
r=np.hypot(Q[:,0]-cx,Q[:,1]-cy); print('rim r pct',np.percentile(r,[5,50,95]).round(3))
# interior heights
m2=(np.hypot(P[:,0]-cx,P[:,1]-cy)<0.09)&(P[:,2]>0.85)
R2=P[m2]; rr=np.hypot(R2[:,0]-cx,R2[:,1]-cy)
for a in np.arange(0,0.09,0.01):
    s=R2[(rr>=a)&(rr<a+0.01)]
    if len(s): print(f'r {a:.2f}: n={len(s)} z {s[:,2].min():.3f}..{s[:,2].max():.3f} median {np.median(s[:,2]):.3f}')
"


# openrua op 70
python3 -c "
import numpy as np
P=np.load('bird_xyz.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.93)&(P[:,2]<1.05)&(P[:,0]>-0.10)&(P[:,0]<0.12)&(P[:,1]>-0.16)&(P[:,1]<0.06)
Q=P[m]; print(len(Q),'x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'zmax',Q[:,2].max().round(3))
cx,cy=(Q[:,0].min()+Q[:,0].max())/2,(Q[:,1].min()+Q[:,1].max())/2; print('center',round(cx,3),round(cy,3))
m2=(np.hypot(P[:,0]-cx,P[:,1]-cy)<0.10)&(P[:,2]>0.85)
R2=P[m2]; rr=np.hypot(R2[:,0]-cx,R2[:,1]-cy)
for a in np.arange(0,0.10,0.01):
    s=R2[(rr>=a)&(rr<a+0.01)]
    if len(s): print(f'r {a:.2f}: n={len(s)} z {s[:,2].min():.3f}..{s[:,2].max():.3f} median {np.median(s[:,2]):.3f}')
"


# openrua op 71
python3 - <<'EOF'
s=open('plan3.py').read()
s=s.replace("REOR=np.array([-0.10,0.08,1.20]); PREPL=np.array([0.03,0.13,1.15]); PLACE=np.array([0.03,0.13,1.02]); RETR=np.array([0.03,0.09,1.25])",
"REOR=np.array([-0.10,0.08,1.20]); PREPL=np.array([0.03,0.16,1.15]); PLACE=np.array([0.03,0.16,1.045]); RETR=np.array([0.03,0.10,1.25])")
s=s.replace("PREPUSH=np.array([0.09,0.05,1.10]); PUSH0=np.array([0.09,0.05,0.975]); PUSH1=np.array([0.09,0.13,0.975])\nPREPUSH2=np.array([0.11,0.14,1.10]); PUSH2=np.array([0.11,0.14,0.965]); PUSH3=np.array([0.11,0.207,0.965]); PUSHUP=np.array([0.11,0.15,1.15])",
"PREPUSH=np.array([0.115,0.05,1.10]); PUSH0=np.array([0.115,0.05,0.965]); PUSH1=np.array([0.115,0.207,0.965]); PUSHUP=np.array([0.115,0.15,1.15])")
s=s.replace("if (((x)**2+(y+0.05)**2<0.085**2)&(z<1.025)).any(): hits.append('bowl')",
"if (((x-0.013)**2+(y+0.03)**2<0.068**2)&(z<1.026)).any(): hits.append('bowl')")
s=s.replace("""    qpp=best_ik(r,PREPUSH,R_PUSH,extra_seeds=[rt[-1]],n_rand=8); print('prepush',np.round(qpp,3)); plan['prepush']=[qpp]
    p0=line(r,PREPUSH,PUSH0,R_PUSH,qpp,3); plan['push0']=p0
    p1=line(r,PUSH0,PUSH1,R_PUSH,p0[-1],4); plan['push1']=p1
    qpp2=best_ik(r,PREPUSH2,R_PUSH2,extra_seeds=[p1[-1]],n_rand=8); print('prepush2',np.round(qpp2,3)); plan['prepush2']=[qpp2]
    p2=line(r,PREPUSH2,PUSH2,R_PUSH2,qpp2,3); plan['push2']=p2
    p3=line(r,PUSH2,PUSH3,R_PUSH2,p2[-1],4); plan['push3']=p3
    pu=line(r,PUSH3,PUSHUP,R_PUSH2,p3[-1],3); plan['pushup']=pu""",
"""    qpp=best_ik(r,PREPUSH,R_PUSH2,extra_seeds=[rt[-1]],n_rand=8); print('prepush',np.round(qpp,3)); plan['prepush']=[qpp]
    p0=line(r,PREPUSH,PUSH0,R_PUSH2,qpp,3); plan['push0']=p0
    p1=line(r,PUSH0,PUSH1,R_PUSH2,p0[-1],8); plan['push1']=p1
    pu=line(r,PUSH1,PUSHUP,R_PUSH2,p1[-1],3); plan['pushup']=pu""")
s=s.replace("""    check_path(r,[qpp]+p0+p1,label='push phase1')
    check_path(r,[p1[-1],qpp2],label='push1->prepush2',n_sub=12)
    check_path(r,[qpp2]+p2+p3+pu,label='push phase2')""",
"""    check_path(r,[qpp]+p0,label='prepush->push0')
    check_path(r,p0[-1:]+p1+pu,label='push')""")
open('plan3.py','w').write(s)
EOF
timeout 1700 python3 plan3.py 2>&1 | grep -v "IK failed"

# openrua op 72
cat > /workspace/check_detail.py <<'EOF'
from robot import *
from plan3 import *
r=Robot()
plan=np.load('plan3.npy',allow_pickle=True).item()
def detail(qs,label,held=False,skip=()):
    print('==',label)
    for i,(a,b) in enumerate(zip(qs[:-1],qs[1:])):
        for t in np.linspace(0,1,4,endpoint=False):
            q=a+(b-a)*t
            p,R=r.fk(q)
            hp=hand_points(p,R,0.02 if held else 0.04)
            tcp=p+TCP_OFF*R[:,2]
            for h in obstacles(hp):
                if h in skip: continue
                m=[pt for pt in hp if h in obstacles(pt[None,:])]
                print(f'  seg{i} t={t:.2f} tcp {np.round(tcp,3)} hand:{h} pts {np.round(m[0],3)} .. n={len(m)}')
            if held:
                bp=bottle_pts(tcp,R)
                for h in obstacles(bp):
                    m=[pt for pt in bp if h in obstacles(pt[None,:])]
                    print(f'  seg{i} t={t:.2f} tcp {np.round(tcp,3)} bottle:{h} pts {np.round(m[0],3)} n={len(m)}')
pl=plan['place']; pp=plan['preplace']; rt=plan['retreat']
detail([plan['reorient'][0]]+pp+pl,'reorient->place',held=True)
detail([pl[-1]]+rt+plan['prepush'],'retreat->prepush')
detail(plan['push0'][-1:]+plan['push1']+plan['pushup'],'push',skip=('drawer_walls',))
EOF
timeout 900 python3 check_detail.py 2>&1 | grep -v "IK failed"

# openrua op 73
cat > /workspace/pushsearch.py <<'EOF'
import numpy as np
from plan3 import obstacles
from robot import rot_from_axes, hand_points, TCP_OFF
def rot_axis(axis,ang):
    axis=np.asarray(axis,float); axis/=np.linalg.norm(axis)
    K=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
    return np.eye(3)+np.sin(ang)*K+(1-np.cos(ang))*K@K
def hand_pts_fine(tcp,R,finger=0.0):
    pos=tcp-TCP_OFF*R[:,2]
    pts=[]
    for xs in np.linspace(-0.032,0.032,3):
        for ys in np.linspace(-0.1,0.1,9):
            for zs in np.linspace(0,0.066,4):
                pts.append(pos+R@np.array([xs,ys,zs]))
    for ys in (-finger-0.01,finger+0.01):
        for zs in (0.066,0.09,0.112):
            for xs in (-0.01,0.01):
                pts.append(pos+R@np.array([xs,ys,zs]))
    return np.array(pts)
def bad(pts,skip=()):
    h=[o for o in obstacles(pts) if o not in skip]
    if (pts[:,2]<0.91).any(): h.append('table')
    return h
results=[]
for tilt in np.radians(np.arange(0,80,5)):
    for az in np.radians(np.arange(0,360,15)):
        zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
        if tilt==0 and az>0: continue
        # reference perpendicular
        ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0])
        y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
        for roll in np.radians(np.arange(0,180,15)):
            yh=rot_axis(zh,roll)@y0
            R=rot_from_axes(zh,yh)
            for xc in (0.06,0.08,0.10,0.115):
                for zc in (0.94,0.955,0.97):
                    ok=True; margin=9
                    for ywall in (0.073,0.213):
                        tip=np.array([xc,ywall-0.012,zc])
                        tcp=tip-0.0086*zh
                        pts=hand_pts_fine(tcp,R)
                        h=bad(pts,skip=('drawer_walls',))
                        if ywall==0.213: h=[o for o in h if o!='cabinet' or True]
                        if h: ok=False; break
                    if ok:
                        results.append((np.degrees(tilt),np.degrees(az),np.degrees(roll),xc,zc))
print(len(results))
for r in results[:60]: print(np.round(r,1))
EOF
python3 pushsearch.py

# openrua op 74
cat > pushsearch2.py <<'EOF'
import numpy as np
from pushsearch import rot_axis, hand_pts_fine, bad
from robot import rot_from_axes
def ok_pose(tcp,R,m):
    pts=hand_pts_fine(tcp,R)
    shifts=[np.zeros(3)]+[s*m*np.eye(3)[i] for i in range(3) for s in (-1,1)]
    for s in shifts:
        if bad(pts+s,skip=('drawer_walls',)): return False
    return True
best=[]
for tilt in np.radians(np.arange(30,75,2.5)):
    for az in np.radians(np.arange(100,160,5)):
        zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
        ref=np.array([1,0,0]); y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
        for roll in np.radians(np.arange(80,150,5)):
            yh=rot_axis(zh,roll)@y0; R=rot_from_axes(zh,yh)
            for xc in (0.09,0.10,0.11):
                for zc in (0.955,0.965,0.975):
                    m=0
                    for mm in (0.004,0.008,0.012,0.016):
                        good=True
                        for ywall in (0.073,0.14,0.213):
                            tip=np.array([xc,ywall-0.012,zc]); tcp=tip-0.0086*zh
                            if not ok_pose(tcp,R,mm): good=False;break
                        if good: m=mm
                        else: break
                    if m>0: best.append((m,np.degrees(tilt),np.degrees(az),np.degrees(roll),xc,zc))
best.sort(key=lambda t:-t[0])
print(len(best))
for b in best[:30]: print(np.round(b,3))
EOF
python3 pushsearch2.py

# openrua op 75
cat > pushsearch3.py <<'EOF'
import numpy as np
from pushsearch import rot_axis, hand_pts_fine
from plan3 import obstacles
from robot import rot_from_axes
def bad(pts,skip):
    h=[o for o in obstacles(pts) if o not in skip]
    if (pts[:,2]<0.91).any(): h.append('table')
    return h
def ok_pose(tcp,R,m,skip):
    pts=hand_pts_fine(tcp,R)
    shifts=[np.zeros(3)]+[s*m*np.eye(3)[i] for i in range(3) for s in (-1,1)]
    return not any(bad(pts+s,skip) for s in shifts)
modes={'wall':dict(xs=(0.06,0.08,0.10,0.115),zs=(0.945,0.96,0.975),ywalls=(0.073,0.14,0.213),skip=('drawer_walls',)),
       'handle':dict(xs=(-0.02,0.0,0.02),zs=(0.93,0.94,0.95),ywalls=(0.04,0.11,0.18),skip=('drawer_walls','low_handle'))}
for name,md in modes.items():
    best=[]
    for tilt in np.radians(np.arange(0,80,5)):
        for az in np.radians(np.arange(0,360,15)):
            if tilt==0 and az>0: continue
            zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
            ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0])
            y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
            for roll in np.radians(np.arange(0,180,10)):
                yh=rot_axis(zh,roll)@y0; R=rot_from_axes(zh,yh)
                for xc in md['xs']:
                    for zc in md['zs']:
                        m=-1
                        for mm in (0.0,0.004,0.008,0.012,0.016):
                            if all(ok_pose(np.array([xc,yw-0.012,zc])-0.0086*zh,R,mm,md['skip']) for yw in md['ywalls']): m=mm
                            else: break
                        if m>=0: best.append((m,np.degrees(tilt),np.degrees(az),np.degrees(roll),xc,zc))
    best.sort(key=lambda t:-t[0])
    print(name,len(best))
    for b in best[:15]: print(np.round(b,3))
EOF
python3 pushsearch3.py 2>&1 | grep -v '^\[.*e[+-]' 

# openrua op 76
python3 pushsearch3.py 2>&1 | tail -20

# openrua op 77
cat > pushsearch4.py <<'EOF'
import numpy as np
from pushsearch import rot_axis, hand_pts_fine
from pushsearch3 import ok_pose
from robot import rot_from_axes
phases={'A':(0.073,0.11,0.15),'B':(0.14,0.18,0.213)}
for name,yw in phases.items():
    best=[]
    for tilt in np.radians(np.arange(0,80,5)):
        for az in np.radians(np.arange(0,360,15)):
            if tilt==0 and az>0: continue
            zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
            ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0])
            y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
            for roll in np.radians(np.arange(0,180,10)):
                yh=rot_axis(zh,roll)@y0; R=rot_from_axes(zh,yh)
                for xc in (0.06,0.08,0.10,0.115):
                    for zc in (0.945,0.96,0.975):
                        m=-1
                        for mm in (0.0,0.005,0.01,0.015,0.02,0.025,0.03):
                            if all(ok_pose(np.array([xc,y-0.012,zc])-0.0086*zh,R,mm,('drawer_walls',)) for y in yw): m=mm
                            else: break
                        if m>=0: best.append((m,np.degrees(tilt),np.degrees(az),np.degrees(roll),xc,zc))
    best.sort(key=lambda t:-t[0])
    print(name,len(best))
    for b in best[:12]: print(np.round(b,3))
EOF
python3 pushsearch4.py 2>&1 | grep -v e+ | grep -v e-

# openrua op 78
python3 pushsearch4.py 2>&1 | grep -A12 '^A ' | grep -v e+; python3 pushsearch4.py 2>&1 | grep -A12 '^B '

# openrua op 79
python3 - <<'EOF'
import re
s=open('plan3.py').read()
s=s.replace("R_PLACE=rot_from_axes([0,c45,-c45],[0,c45,c45])","c35,s35=np.cos(np.radians(35)),np.sin(np.radians(35))\nR_PLACE=rot_from_axes([0,s35,-c35],[0,c35,s35])")
s=s.replace("PREPL=np.array([0.03,0.16,1.15]); PLACE=np.array([0.03,0.16,1.045])","PREPL=np.array([0.03,0.13,1.15]); PLACE=np.array([0.03,0.13,1.06])")
s=s.replace("PUSH1=np.array([0.115,0.207,0.965]); PUSHUP=np.array([0.115,0.15,1.15])",
 "PUSH1=np.array([0.115,0.13,0.965]); PUSHUP=np.array([0.115,0.13,1.15])\n"
 "PREPUSHB=np.array([0.15,0.10,1.10]); PUSH0B=np.array([0.15,0.10,0.975]); PUSH1B=np.array([0.15,0.20,0.975]); PUSHUPB=np.array([0.15,0.20,1.15])")
s=s.replace("    pu=line(r,PUSH1,PUSHUP,R_PUSH2,p1[-1],3); plan['pushup']=pu\n",
 "    pu=line(r,PUSH1,PUSHUP,R_PUSH2,p1[-1],3); plan['pushup']=pu\n"
 "    qpb=best_ik(r,PREPUSHB,R_PUSH,extra_seeds=[pu[-1]],n_rand=8); print('prepushB',np.round(qpb,3)); plan['prepushB']=[qpb]\n"
 "    p0b=line(r,PREPUSHB,PUSH0B,R_PUSH,qpb,3); plan['push0B']=p0b\n"
 "    p1b=line(r,PUSH0B,PUSH1B,R_PUSH,p0b[-1],6); plan['push1B']=p1b\n"
 "    pub=line(r,PUSH1B,PUSHUPB,R_PUSH,p1b[-1],3); plan['pushupB']=pub\n")
s=s.replace("    check_path(r,p0[-1:]+p1+pu,label='push')\n",
 "    check_path(r,p0[-1:]+p1+pu,label='pushA')\n"
 "    check_path(r,[pu[-1],qpb],label='pushupA->prepushB')\n"
 "    check_path(r,[qpb]+p0b,label='prepushB->push0B')\n"
 "    check_path(r,p0b[-1:]+p1b+pub,label='pushB')\n")
open('plan3.py','w').write(s)
EOF
grep -n "PLACE\|PUSH" plan3.py | head -30

# openrua op 80
timeout 1700 python3 plan3.py 2>&1 | grep -v '^\[' | tail -40

# openrua op 81
cat > pushsearch5.py <<'EOF'
import numpy as np
from pushsearch import rot_axis, hand_pts_fine
from plan3 import obstacles
from robot import rot_from_axes, TCP_OFF, BASE_IN_WORLD
def bad(pts,skip):
    h=[o for o in obstacles(pts) if o not in skip]
    if (pts[:,2]<0.91).any(): h.append('table')
    return h
def ok_pose(tcp,R,m,skip,finger):
    pts=hand_pts_fine(tcp,R,finger)
    shifts=[np.zeros(3)]+[s*m*np.eye(3)[i] for i in range(3) for s in (-1,1)]
    return not any(bad(pts+s,skip) for s in shifts)
SH=np.array(BASE_IN_WORLD)+np.array([0,0,0.333])
best=[]
for tilt in np.radians(np.arange(0,80,5)):
    for az in np.radians(np.arange(0,360,15)):
        if tilt==0 and az>0: continue
        zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
        ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0])
        y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
        for roll in np.radians(np.arange(0,180,10)):
            yh=rot_axis(zh,roll)@y0; R=rot_from_axes(zh,yh)
            for fsign in (-1,0,1):
                finger=0.04 if fsign else 0.0
                for xc in (0.06,0.08,0.10,0.115):
                    for zc in (0.945,0.96,0.975):
                        m=-1
                        for mm in (0.0,0.005,0.01,0.015,0.02,0.025):
                            good=True
                            for yw in (0.14,0.18,0.213):
                                tcp=np.array([xc,yw-0.012,zc])-0.0086*zh+fsign*0.05*yh
                                origin=tcp-TCP_OFF*zh
                                if np.linalg.norm(origin-SH)>0.80: good=False;break
                                if not ok_pose(tcp,R,mm,('drawer_walls',),finger): good=False;break
                            if good: m=mm
                            else: break
                        if m>=0: best.append((m,np.degrees(tilt),np.degrees(az),np.degrees(roll),fsign,xc,zc))
best.sort(key=lambda t:-t[0])
print(len(best))
for b in best[:25]: print(np.round(b,3))
EOF
python3 pushsearch5.py 2>&1 | grep -v e+ | grep -v e-

# openrua op 82
python3 pushsearch5.py 2>&1 | tail -7

# openrua op 83
cat > diag.py <<'EOF'
import numpy as np
from pushsearch import rot_axis, hand_pts_fine
from plan3 import obstacles
from robot import rot_from_axes, TCP_OFF, BASE_IN_WORLD
SH=np.array(BASE_IN_WORLD)+np.array([0,0,0.333])
def diag(tilt,az,roll,fsign,xc,zc,m=0.005):
    tilt,az,roll=np.radians([tilt,az,roll])
    zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
    ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0])
    y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
    yh=rot_axis(zh,roll)@y0; R=rot_from_axes(zh,yh)
    finger=0.04 if fsign else 0.0
    for yw in (0.14,0.18,0.213):
        tcp=np.array([xc,yw-0.012,zc])-0.0086*zh+fsign*0.05*yh
        origin=tcp-TCP_OFF*zh
        pts=hand_pts_fine(tcp,R,finger)
        hits=set()
        for i in range(3):
            for s in (-1,0,1):
                p=pts+s*m*np.eye(3)[i]
                for h in obstacles(p): hits.add(h)
                if (p[:,2]<0.91).any(): hits.add('table')
        print(f'yw {yw} tcp {np.round(tcp,3)} reach {np.linalg.norm(origin-SH):.3f} hits {sorted(hits)}  ymax {pts[:,1].max():.3f} zmin {pts[:,2].min():.3f}')
    print('R=\n',np.round(R,3))
for c in [(40,135,80,1,0.10,0.975),(50,135,90,1,0.08,0.975),(40,90,0,1,0.115,0.975),(40,90,0,0,0.115,0.975)]:
    print(c); diag(*c)
EOF
python3 diag.py 2>&1 | grep -v '^\[.*e'

# openrua op 84
cat >> diag.py <<'EOF'
print('---- detail')
def detail(tilt,az,roll,fsign,xc,zc,yw=0.213,m=0.0):
    tilt,az,roll=np.radians([tilt,az,roll])
    zh=np.array([np.sin(tilt)*np.cos(az),np.sin(tilt)*np.sin(az),-np.cos(tilt)])
    ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0])
    y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
    yh=rot_axis(zh,roll)@y0; R=rot_from_axes(zh,yh)
    finger=0.04 if fsign else 0.0
    tcp=np.array([xc,yw-0.012,zc])-0.0086*zh+fsign*0.05*yh
    pts=hand_pts_fine(tcp,R,finger)
    x,y,z=pts.T
    cab=(x>-0.12-m)&(x<0.13+m)&(y>0.21-m)&(z<1.135+m)
    han=(x>-0.055-m)&(x<0.045+m)&(y>0.175-m)&(y<0.215+m)&(z>0.995-m)&(z<1.105+m)
    print('tcp',np.round(tcp,3)); print('cabinet pts',np.round(pts[cab],3)); print('handle pts',np.round(pts[han],3))
    print('R',np.round(R,3))
detail(50,135,90,1,0.08,0.975,m=0.005)
EOF
python3 diag.py 2>&1 | sed -n '/detail/,$p'

# openrua op 85
cat > pushsearch6.py <<'EOF'
import numpy as np
from pushsearch import rot_axis, hand_pts_fine
from pushsearch5 import ok_pose, SH
from robot import rot_from_axes, TCP_OFF
best=[]
for tilt in np.arange(35,80,2.5):
    for az in np.arange(105,165,5):
        zh=np.array([np.sin(np.radians(tilt))*np.cos(np.radians(az)),np.sin(np.radians(tilt))*np.sin(np.radians(az)),-np.cos(np.radians(tilt))])
        ref=np.array([1,0,0]); y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
        for roll in np.arange(40,150,5):
            yh=rot_axis(zh,np.radians(roll))@y0; R=rot_from_axes(zh,yh)
            for fsign in (-1,0,1):
                finger=0.04 if fsign else 0.0
                for xc in (0.05,0.06,0.07,0.08,0.09,0.10,0.11):
                    for zc in (0.955,0.965,0.975):
                        m=-1
                        for mm in (0.005,0.01,0.015,0.02):
                            good=True
                            for yw in (0.14,0.18,0.213):
                                tcp=np.array([xc,yw-0.012,zc])-0.0086*zh+fsign*0.05*yh
                                if np.linalg.norm(tcp-TCP_OFF*zh-SH)>0.80: good=False;break
                                if not ok_pose(tcp,R,mm,('drawer_walls',),finger): good=False;break
                            if good: m=mm
                            else: break
                        if m>0: best.append((m,tilt,az,roll,fsign,xc,zc))
best.sort(key=lambda t:-t[0])
print(len(best))
for b in best[:25]: print(np.round(b,3))
EOF
python3 pushsearch6.py 2>&1 | grep -v '^\[.*e[+-]' | tail -26

# openrua op 86
cat > pushsearch7.py <<'EOF'
import numpy as np
from pushsearch import rot_axis, hand_pts_fine
from pushsearch5 import ok_pose, SH
from robot import rot_from_axes, TCP_OFF
best=[]
for tilt in np.arange(0,80,5):
    for az in np.arange(0,360,15):
        if tilt==0 and az>0: continue
        zh=np.array([np.sin(np.radians(tilt))*np.cos(np.radians(az)),np.sin(np.radians(tilt))*np.sin(np.radians(az)),-np.cos(np.radians(tilt))])
        ref=np.array([1,0,0]) if abs(zh[0])<0.9 else np.array([0,1,0]); y0=np.cross(zh,ref); y0/=np.linalg.norm(y0)
        for roll in np.arange(0,180,10):
            yh=rot_axis(zh,np.radians(roll))@y0; R=rot_from_axes(zh,yh)
            for fsign in (-1,0,1):
                finger=0.04 if fsign else 0.0
                for xc in (-0.02,0.0,0.02):
                    for zc in (0.935,0.945):
                        m=-1
                        for mm in (0.0,0.005,0.01,0.015,0.02,0.025):
                            good=True
                            for yw in (0.14,0.16,0.18):
                                tcp=np.array([xc,yw-0.012,zc])-0.0086*zh+fsign*0.05*yh
                                if np.linalg.norm(tcp-TCP_OFF*zh-SH)>0.80: good=False;break
                                if not ok_pose(tcp,R,mm,('drawer_walls','low_handle'),finger): good=False;break
                            if good: m=mm
                            else: break
                        if m>=0: best.append((m,tilt,az,roll,fsign,xc,zc))
best.sort(key=lambda t:-t[0])
print(len(best))
for b in best[:25]: print(np.round(b,3))
EOF
python3 pushsearch7.py 2>&1 | grep -v '^\[.*e[+-]' | tail -26

# openrua op 87
python3 pushsearch7.py 2>&1 | tail -25

# openrua op 88
cat >> diag.py <<'EOF'
print('=====')
for c in [(60,135,100,1,0.06,0.975),(70,135,110,1,0.06,0.975),(40,135,80,1,0.10,0.975)]:
    print(c); detail(*c,m=0.005)
EOF
python3 diag.py 2>&1 | sed -n '/=====/,$p'

# openrua op 89
cat > probe.py <<'EOF'
from robot import *
from plan3 import obstacles, bottle_pts
r=Robot()
for tilt in (15,25,35):
    c,s=np.cos(np.radians(tilt)),np.sin(np.radians(tilt))
    R=rot_from_axes([0,c,-s],[1,0,0])
    for x in (0.09,0.10,0.115):
        for y in (0.15,0.20):
            tcp=np.array([x,y,0.965])
            q=best_ik(r,tcp,R,n_rand=6)
            if q is None: print(tilt,x,y,'FAIL'); continue
            p,Rh=fk_links(r,q)['panda_hand']
            hp=hand_points(p,Rh,0.0)
            print(tilt,x,y,np.round(q,2),'hits',obstacles(hp),'clr',min(c for _,c,_ in link_clearance(r,q)))
EOF
timeout 900 python3 probe.py 2>&1 | grep -v 'IK failed'

# openrua op 90
grep -n "def hand_points" -A 16 robot.py; cat > probe2.py <<'EOF'
from robot import *
from plan3 import obstacles
for tilt in (15,25):
    c,s=np.cos(np.radians(tilt)),np.sin(np.radians(tilt))
    R=rot_from_axes([0,c,-s],[1,0,0])
    tcp=np.array([0.10,0.20,0.965]); p=tcp-TCP_OFF*R[:,2]
    hp=hand_points(p,R,0.0)
    x,y,z=hp.T
    print(tilt,'cab',np.round(hp[(y>0.21)&(z<1.135)],3),'han',np.round(hp[(x>-0.055)&(x<0.045)&(y>0.175)&(y<0.215)&(z>0.995)&(z<1.105)],3))
    print(' ymax',y.max(),'zmin',z.min(),'zmax',z.max())
EOF
python3 probe2.py

# openrua op 91
python3 - <<'EOF'
s=open('plan3.py').read()
s=s.replace("PUSH1=np.array([0.115,0.13,0.965]); PUSHUP=np.array([0.115,0.13,1.15])","PUSH1=np.array([0.115,0.165,0.965]); PUSHUP=np.array([0.115,0.13,1.15])")
s=s.replace("PREPUSHB=np.array([0.15,0.10,1.10]); PUSH0B=np.array([0.15,0.10,0.975]); PUSH1B=np.array([0.15,0.20,0.975]); PUSHUPB=np.array([0.15,0.20,1.15])",
"c15,s15=np.cos(np.radians(15)),np.sin(np.radians(15))\nR_PUSHB=rot_from_axes([0,c15,-s15],[1,0,0])\nPREPUSHB=np.array([0.10,0.16,1.10]); PUSH0B=np.array([0.10,0.16,0.965]); PUSH1B=np.array([0.10,0.202,0.965]); PUSHUPB=np.array([0.10,0.16,1.15])")
s=s.replace("best_ik(r,PREPUSHB,R_PUSH,","best_ik(r,PREPUSHB,R_PUSHB,")
s=s.replace("line(r,PREPUSHB,PUSH0B,R_PUSH,","line(r,PREPUSHB,PUSH0B,R_PUSHB,").replace("line(r,PUSH0B,PUSH1B,R_PUSH,","line(r,PUSH0B,PUSH1B,R_PUSHB,").replace("line(r,PUSH1B,PUSHUPB,R_PUSH,","line(r,PUSH1B,PUSHUPB,R_PUSHB,")
open('plan3.py','w').write(s)
EOF
grep -n "R_PUSHB\|PUSH1=" plan3.py; timeout 1700 python3 plan3.py 2>&1 | grep -v '^\[' | tail -30

# openrua op 92
cat check_detail.py | head -40

# openrua op 93
cat > check_detail2.py <<'EOF'
from check_detail import *
detail(plan['push0'][-1:]+plan['push1']+plan['pushup'],'pushA')
detail([plan['pushup'][-1]]+plan['prepushB']+plan['push0B'],'pushupA->push0B')
detail(plan['push0B'][-1:]+plan['push1B']+plan['pushupB'],'pushB')
# link clearances during pushA
for q in plan['push0'][-1:]+plan['push1']:
    lc=link_clearance(r,q); print(np.round(r.tcp(q)[0],3),[(n,round(c,3)) for n,c,_ in lc if c<0.05])
EOF
python3 check_detail2.py 2>&1 | grep -v '^\[' | grep -v "^==\|seg. t=.*drawer_walls" | head -60; python3 check_detail2.py 2>&1 | grep -c drawer_walls

# openrua op 94
python3 check_detail2.py 2>&1 | grep "^\[.*panda" 

# openrua op 95
grep -n "def move_q" -A 40 robot.py | head -60; grep -n "def gripper" -A 25 robot.py | head -40

# openrua op 96
cat > exec_step.py <<'EOF'
import sys
from robot import *
plan=np.load('plan3.npy',allow_pickle=True).item()
r=Robot()
def run(qs,vmax=0.4,tmin=1.5):
    q0=r.arm_q(); t=0; wps=[]
    prev=q0
    for q in qs:
        dt=max(np.abs(q-prev).max()/vmax,0.4); t+=dt; wps.append((q,t)); prev=q
    t=max(t,tmin)
    # rescale times if tmin dominated
    scale=t/wps[-1][1]; wps=[(q,tt*scale) for q,tt in wps]
    code=r.move_q(wps[-1][0],wps[-1][1],waypoints=wps[:-1])
    return code
args=sys.argv[1:]
vmax=0.4
for a in args:
    if a.startswith('v='): vmax=float(a[2:]); continue
    if a=='open': r.gripper(0.04); continue
    if a=='close': r.gripper(0.0); continue
    qs=plan[a]; print('step',a,len(qs),'pts'); run(qs,vmax)
q=r.arm_q(); p,R=r.tcp(q)
print('q',np.round(q,3)); print('tcp',np.round(p,4)); print('fingers',r.fingers()); print('wrench',np.round(r.wrench(),2))
EOF
python3 exec_step.py 2>&1 | tail -4

# openrua op 97
timeout 600 python3 exec_step.py descend 2>&1 | tail -6

# openrua op 98
timeout 120 python3 exec_step.py 2>&1 | tail -4; timeout 60 python3 tools/perception/cam_snap.py agentview /workspace/now_agent.png; timeout 60 python3 tools/perception/cam_snap.py frontview /workspace/now_front.png

# openrua op 99
timeout 30 ros2 service list 2>/dev/null | grep -i -E "valid|collision|scene|plan" ; timeout 20 ros2 action list 2>/dev/null

# openrua op 100
grep -n "^from\|^import\|self.ik_cli\|create_client" robot.py | head -20

# openrua op 101
cat >> robot.py <<'EOF'


from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState as _RobotState

def state_valid(r, q, finger=0.04, verbose=False):
    """MoveIt /check_state_validity (self-collision + joint limits). Returns (valid, contact pairs)."""
    if not hasattr(r, 'sv_cli'):
        r.sv_cli = r.node.create_client(GetStateValidity, '/check_state_validity')
        r.sv_cli.wait_for_service(10)
    req = GetStateValidity.Request()
    js = JointState(); js.name = list(ARM) + ['panda_finger_joint1', 'panda_finger_joint2']
    js.position = [float(v) for v in q] + [finger, finger]
    req.robot_state = _RobotState(joint_state=js)
    req.group_name = 'panda_arm'
    fut = r.sv_cli.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    pairs = [(c.contact_body_1, c.contact_body_2) for c in res.contacts]
    if verbose: print('valid', res.valid, pairs)
    return res.valid, pairs


def path_valid(r, qs, step=0.05, finger=0.04):
    """check interpolated joint path for self-collision; returns list of (idx, t, pairs) failures."""
    bad = []
    for i, (a, b) in enumerate(zip(qs[:-1], qs[1:])):
        n = max(2, int(np.ceil(np.abs(b - a).max() / step)))
        for t in np.linspace(0, 1, n + 1):
            v, pairs = state_valid(r, a + (b - a) * t, finger)
            if not v: bad.append((i, round(float(t), 3), pairs))
    return bad
EOF
cat > sv_check.py <<'EOF'
from robot import *
r=Robot()
plan=np.load('plan3.npy',allow_pickle=True).item()
q0=np.array([0,-0.161,0,-2.445,0,2.227,0.785]); qc=r.arm_q()
print('current state', state_valid(r,qc))
print('start->above', path_valid(r,[q0,plan['descend'][0]])[:5])
order=['descend','approach','grasp','lift','reorient','preplace','place','retreat','prepush','push0','push1','pushup','prepushB','push0B','push1B','pushupB']
prev=plan['descend'][0]
for k in order:
    qs=[prev]+list(plan[k])
    bad=path_valid(r,qs,finger=0.02 if k in('lift','reorient','preplace','place') else 0.04)
    print(k,'bad' if bad else 'ok',bad[:3])
    prev=qs[-1]
EOF
timeout 900 python3 sv_check.py 2>&1 | grep -v '^\['

# openrua op 102
cat > back.py <<'EOF'
from robot import *
r=Robot()
q0=np.array([0,-0.161,0,-2.445,0,2.227,0.785]); qc=r.arm_q()
for t in (1/3,2/3,1.0):
    q=qc+(q0-qc)*t
    r.move_q(q,3.0)
print(np.round(r.arm_q(),3))
EOF
timeout 300 python3 back.py 2>&1 | grep -v '^\[.*e'

# openrua op 103
cat > durtest.py <<'EOF'
from robot import *
import sys
r=Robot(); q=r.arm_q().copy(); q[0]+=0.3
print('7s move:'); r.move_q(q,7.0)
q[0]-=0.3
print('2s move back:'); r.move_q(q,2.0)
print(np.round(r.arm_q(),3))
EOF
timeout 300 python3 durtest.py 2>&1 | grep -v '^\[.*e'

# openrua op 104
cat > wptest.py <<'EOF'
from robot import *
r=Robot(); q=r.arm_q().copy()
w1=q.copy(); w1[0]+=0.2; w2=q.copy(); w2[0]+=0.4
r.move_q(q,6.0,waypoints=[(w1,2.0),(w2,4.0)])
print(np.round(r.arm_q(),3))
EOF
timeout 300 python3 wptest.py 2>&1 | grep -v '^\[.*e'

# openrua op 105
cat > segfine.py <<'EOF'
from robot import *
from plan3 import obstacles, check_path
r=Robot()
plan=np.load('plan3.npy',allow_pickle=True).item()
q0=np.array([0,-0.161,0,-2.445,0,2.227,0.785]); q1=plan['descend'][0]
qstop=np.array([1.459,0.701,-0.963,-2.845,-1.468,1.889,-0.582])
for t in np.linspace(0,1,21):
    q=q0+(q1-q0)*t
    fk=fk_links(r,q); p,R=fk['panda_hand']; tcp=p+TCP_OFF*R[:,2]
    hp=hand_points(p,R,0.04)
    lc=link_clearance(r,q)
    print(f't={t:.2f} tcp {np.round(tcp,3)} hand zmin {hp[:,2].min():.3f} hits {obstacles(hp)} minlink {min(c for _,c,_ in lc):.3f}')
print('stop tcp',np.round(r.tcp(qstop)[0],3))
EOF
timeout 300 python3 segfine.py 2>&1 | grep -v '^\[.*e'

# openrua op 106
cat > speedtest.py <<'EOF'
from robot import *
r=Robot(); q=r.arm_q().copy(); q1=q.copy(); q1[0]+=0.8
r.move_q(q1,2.0); print(np.round(r.arm_q(),3))
r.move_q(q,2.0); print(np.round(r.arm_q(),3))
EOF
timeout 300 python3 speedtest.py 2>&1 | grep -v '^\[.*e'

# openrua op 107
python3 - <<'EOF'
s=open('exec_step.py').read()
s=s.replace("""def run(qs,vmax=0.4,tmin=1.5):
    q0=r.arm_q(); t=0; wps=[]
    prev=q0
    for q in qs:
        dt=max(np.abs(q-prev).max()/vmax,0.4); t+=dt; wps.append((q,t)); prev=q
    t=max(t,tmin)
    # rescale times if tmin dominated
    scale=t/wps[-1][1]; wps=[(q,tt*scale) for q,tt in wps]
    code=r.move_q(wps[-1][0],wps[-1][1],waypoints=wps[:-1])
    return code""","""def run(qs,vmax=0.15):
    # one goal per waypoint; bridge velocity limit ~0.23 rad/s, stay well under it
    for q in qs:
        dt=max(np.abs(q-r.arm_q()).max()/vmax,0.6)
        code=r.move_q(q,dt)
        if code!=0: print('ABORT step'); return code
    return 0""")
s=s.replace("vmax=0.4\n","vmax=0.15\n")
open('exec_step.py','w').write(s)
EOF
timeout 900 python3 exec_step.py descend 2>&1 | grep -v '^\[.*e'

# openrua op 108
cat > j7test.py <<'EOF'
from robot import *
r=Robot(); q=r.arm_q().copy(); q[6]=-2.081
r.move_q(q,3.0); print(np.round(r.arm_q(),3)); print(r.wrench())
EOF
timeout 300 python3 j7test.py 2>&1 | grep -v '^\[.*e'

# openrua op 109
python3 - <<'EOF'
s=open('exec_step.py').read()
s=s.replace("    qs=plan[a]; print('step',a,len(qs),'pts'); run(qs,vmax)",
"""    if '[' in a:
        name,sl=a.split('['); qs=eval('plan[name]['+sl)
    else: qs=plan[a]
    print('step',a,len(qs),'pts')
    if run(qs,vmax)!=0: break""")
s=s.replace("vmax=0.15\n","vmax=0.1\n")
open('exec_step.py','w').write(s)
EOF
timeout 900 python3 exec_step.py "descend[1:]" approach grasp 2>&1 | grep -v '^\[.*e'

# openrua op 110
timeout 60 python3 tools/perception/cam_snap.py agentview /workspace/g_agent.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand /workspace/g_eye.png

# openrua op 111
timeout 300 python3 exec_step.py close 2>&1 | grep -v '^\[.*e'

# openrua op 112
timeout 900 python3 exec_step.py lift 2>&1 | grep -v '^\[.*e'; timeout 60 python3 tools/perception/cam_snap.py agentview /workspace/l_agent.png

# openrua op 113
timeout 1200 python3 exec_step.py reorient preplace place 2>&1 | grep -v '^\[.*e'; timeout 60 python3 tools/perception/cam_snap.py agentview /workspace/p_agent.png

# openrua op 114
timeout 600 python3 exec_step.py retreat open 2>&1 | grep -v '^\[.*e'; timeout 120 python3 scene.py birdview 2>&1 | tail -1

# openrua op 115
cat > findbottle.py <<'EOF'
import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); fx,fy,cx,cy=np.load('birdview_K.npy')
H,W=d.shape; v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)@T.T
xyz=P[...,:3]
x,y,z=xyz[...,0],xyz[...,1],xyz[...,2]
# region left of drawer: x -0.35..-0.02, y -0.05..0.2, z between table+0.01 and 0.99 (bottle lying)
m=(x>-0.35)&(x<-0.0)&(y>-0.05)&(y<0.2)&(z>0.91)&(z<0.99)
pts=xyz[m]; print('n',len(pts))
print('x range',pts[:,0].min(),pts[:,0].max(),'y range',pts[:,1].min(),pts[:,1].max(),'zmax',pts[:,2].max())
# PCA
c=pts[:,:2].mean(0); U,S,Vt=np.linalg.svd(pts[:,:2]-c,full_matrices=False)
print('center',c,'axis',Vt[0],'extent along axis',(pts[:,:2]-c)@Vt[0]).min() if False else None
proj=(pts[:,:2]-c)@Vt[0]; print('center',np.round(c,3),'axis',np.round(Vt[0],3),'along',proj.min(),proj.max())
# top ridge: highest points
top=pts[pts[:,2]>pts[:,2].max()-0.01]; print('ridge pts',len(top),'ridge center',np.round(top.mean(0),3))
# body vs neck: heights along axis
for s in np.linspace(proj.min(),proj.max(),9):
    sel=np.abs(proj-s)<0.01
    if sel.any(): print(f' s={s:.3f} zmax={pts[sel,2].max():.3f} n={sel.sum()}')
EOF
python3 findbottle.py

# openrua op 116
cat > findbottle2.py <<'EOF'
import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); fx,fy,cx,cy=np.load('birdview_K.npy')
H,W=d.shape; v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)@T.T
xyz=P[...,:3]; x,y,z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(x>-0.4)&(x<-0.0)&(y>-0.1)&(y<0.25)&(z>0.905)&(z<0.99)&~((x>-0.13)&(y>0.06))
pts=xyz[m]; print('n',len(pts))
print('x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'zmax',pts[:,2].max())
c=pts[:,:2].mean(0); U,S,Vt=np.linalg.svd(pts[:,:2]-c,full_matrices=False); ax=Vt[0]
proj=(pts[:,:2]-c)@ax; print('center',np.round(c,3),'axis',np.round(ax,3),'along',proj.min(),proj.max())
for s in np.arange(proj.min(),proj.max(),0.01):
    sel=np.abs(proj-s)<0.005
    if sel.any():
        perp=(pts[sel,:2]-c)@Vt[1]
        print(f' s={s:.3f} zmax={pts[sel,2].max():.3f} perp {perp.min():.3f}..{perp.max():.3f} n={sel.sum()}')
np.save('bottle_pts.npy',pts)
EOF
python3 findbottle2.py

# openrua op 117
cat > hmap.py <<'EOF'
import numpy as np,sys
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); fx,fy,cx,cy=np.load('birdview_K.npy')
H,W=d.shape; v,u=np.mgrid[0:H,0:W]
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d,np.ones_like(d)],-1)@T.T
xyz=P[...,:3].reshape(-1,3); np.save('bird_xyz2.npy',P[...,:3])
x0,x1,y0,y1=[float(a) for a in sys.argv[1:5]]; res=float(sys.argv[5]) if len(sys.argv)>5 else 0.01
xs=np.arange(x0,x1,res); ys=np.arange(y0,y1,res)
print('      '+' '.join(f'{y*100:4.0f}' for y in ys))
for xx in xs:
    row=[]
    for yy in ys:
        m=(np.abs(xyz[:,0]-xx)<res/2)&(np.abs(xyz[:,1]-yy)<res/2)
        row.append(f'{(xyz[m,2].max()-0.9)*100:4.0f}' if m.any() else '   .')
    print(f'{xx*100:5.0f} '+' '.join(row))
EOF
python3 hmap.py -0.30 -0.04 -0.10 0.22 0.01

# openrua op 118
cat > park.py <<'EOF'
from robot import *
from robot import state_valid
r=Robot()
tcp=np.array([-0.15,-0.32,1.25])
q=best_ik(r,tcp,DOWN,n_rand=6)
print(np.round(q,3), state_valid(r,q))
qc=r.arm_q()
# check link clearance along interpolation
for t in np.linspace(0,1,11):
    qq=qc+(q-qc)*t; lc=link_clearance(r,qq); print(f'{t:.1f}',np.round(r.tcp(qq)[0],3),min(c for _,c,_ in lc), [n for n,c,i in lc if i])
EOF
timeout 300 python3 park.py 2>&1 | grep -v '^\[.*e'

# openrua op 119
cat > park_exec.py <<'EOF'
from robot import *
r=Robot()
tcp=np.array([-0.15,-0.32,1.25])
q=best_ik(r,tcp,DOWN,n_rand=6); qc=r.arm_q()
for t in (0.5,1.0):
    qq=qc+(q-qc)*t; r.move_q(qq,max(np.abs(qq-r.arm_q()).max()/0.1,1))
print('q',np.round(r.arm_q(),3),'tcp',np.round(r.tcp()[0],3))
EOF
timeout 600 python3 park_exec.py 2>&1 | grep -v '^\[.*e'; timeout 120 python3 scene.py birdview | tail -1; timeout 60 python3 tools/perception/cam_snap.py agentview /workspace/b_agent.png; python3 hmap.py -0.30 -0.04 -0.10 0.22 0.01

# openrua op 120
timeout 120 python3 scene.py agentview 315 232 300 215 335 250 | tail -4

# openrua op 121
cat > findbottle3.py <<'EOF'
import numpy as np
xyz=np.load('bird_xyz2.npy').reshape(-1,3); x,y,z=xyz.T
m=(x>-0.30)&(x<-0.12)&(y>-0.08)&(y<0.06)&(z>0.925)&(z<0.96)
pts=xyz[m]; print('n',len(pts))
c=pts[:,:2].mean(0); U,S,Vt=np.linalg.svd(pts[:,:2]-c,full_matrices=False); ax=Vt[0]
if ax[0]>0: ax=-ax
proj=(pts[:,:2]-c)@ax; perp=(pts[:,:2]-c)@Vt[1]
print('body center',np.round(c,4),'axis (toward cork)',np.round(ax,3),'angle deg',np.degrees(np.arctan2(ax[1],ax[0])))
print('along',proj.min(),proj.max(),'perp',perp.min(),perp.max(),'zmax',pts[:,2].max())
for s in np.arange(proj.min(),proj.max(),0.01):
    sel=np.abs(proj-s)<0.005
    if sel.any(): print(f' s={s:.3f} zmax={pts[sel,2].max():.3f} perp {perp[sel].min():.3f}..{perp[sel].max():.3f} n={sel.sum()}')
EOF
python3 findbottle3.py

# openrua op 122
grep -n "def rot_from_axes" -A 12 robot.py; grep -n "def ik_near" -A 22 robot.py | head -30

# openrua op 123
cat > plan4.py <<'EOF'
from robot import *
from robot import state_valid, path_valid
from plan3 import obstacles, check_path, line
r=Robot()
ax=np.array([-0.947,-0.322,0.0]); ax/=np.linalg.norm(ax)   # bottle axis toward cork
C=np.array([-0.2186,-0.0153,0.92])                          # bottle mass-center (5.5cm from bottom)
closing=np.cross([0,0,-1],ax)                              # y_h = z_h x x_h so that x_h = ax
R_G=rot_from_axes([0,0,-1],closing); print('x_h',np.round(R_G[:,0],3))
R_D=rot_from_axes([0,0,-1],[0,1,0]); print('drop x_h',np.round(R_D[:,0],3))   # x_h=-x -> cork toward -x
PRE=C+np.array([0,0,0.13]); LIFT=C+np.array([0,0,0.23])
DROP=np.array([0.03,0.15,1.11]); PREDROP=np.array([0.03,0.15,1.20]); MID=np.array([-0.10,0.05,1.22])
plan={}
qc=r.arm_q()
qpre=best_ik(r,PRE,R_G,n_rand=8); print('pre',np.round(qpre,3))
plan['pre4']=[qpre]
g=line(r,PRE,C,R_G,qpre,4); plan['grasp4']=g; print('grasp',np.round(g[-1],3))
l=line(r,C,LIFT,R_G,g[-1],4); plan['lift4']=l
qmid=ik_near(r,MID,R_D,l[-1],max_dq=1.2,tries=12); print('mid',np.round(qmid,3))
plan['mid4']=[qmid]
pd=line(r,MID,PREDROP,R_D,qmid,4); plan['predrop4']=pd
dr=line(r,PREDROP,DROP,R_D,pd[-1],3); plan['drop4']=dr
up=line(r,DROP,PREDROP,R_D,dr[-1],2); plan['up4']=up
np.save('plan4.npy',plan,allow_pickle=True)
check_path(r,[qc,qpre],n_sub=12,label='park->pre')
check_path(r,[qpre]+g,label='pre->grasp')
check_path(r,g[-1:]+l,held=True,label='lift')
check_path(r,l[-1:]+[qmid]+pd+dr,held=True,n_sub=12,label='lift->drop')
for k,v in [('park->pre',[qc,qpre]),('pre->grasp',[qpre]+g),('lift',g[-1:]+l),('lift->drop',l[-1:]+[qmid]+pd+dr)]:
    print(k,'selfcoll',path_valid(r,v,finger=0.02)[:3])
for q in [qpre,g[-1],l[-1],qmid,pd[-1],dr[-1]]:
    print(np.round(q,3),np.round(r.tcp(q)[0],3))
EOF
timeout 1200 python3 plan4.py 2>&1 | grep -v '^\[.*e'

# openrua op 124
cat > probe4.py <<'EOF'
from robot import *
from robot import state_valid
r=Robot()
ax=np.array([-0.947,-0.322,0.0]); closing=np.cross([0,0,-1],ax)
R_G=rot_from_axes([0,0,-1],closing)
C=np.array([-0.2186,-0.0153,0.92]); PRE=C+[0,0,0.13]
for seed in [np.array([0,0.3,0,-2.3,0,2.6,0.46]),np.array([0,0.0,0,-2.4,0,2.4,0.46]),np.array([0.1,0.5,-0.1,-2.0,0,2.5,0.5]),np.array([0,0.3,0,-2.3,0,2.6,-2.68])]:
    for p in (PRE,C):
        q=r.ik_tcp(p,R_G,seed=seed,avoid_collisions=True)
        print(np.round(p,3),None if q is None else np.round(q,3))
EOF
timeout 600 python3 probe4.py 2>&1 | grep -v '^\[.*e'

# openrua op 125
cat > probe5.py <<'EOF'
from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot()
ax=np.array([-0.947,-0.322,0.0]); closing=np.cross([0,0,-1],ax)
R_G=rot_from_axes([0,0,-1],closing)
C=np.array([-0.2186,-0.0153,0.92])
rng=np.random.default_rng(0)
sols=[]
for i in range(25):
    seed=np.array([0,0.3,0,-2.3,0,2.6,0.46])+rng.normal(0,0.6,7)
    q=r.ik_tcp(C,R_G,seed=seed,avoid_collisions=(i%2==0))
    if q is not None: sols.append((abs(q[0])+abs(q[2])+abs(q[4]),i%2==0,q))
sols.sort(key=lambda s:s[0])
for s in sols[:10]: print(round(s[0],2),s[1],s[2])
EOF
timeout 900 python3 probe5.py 2>&1 | grep -v "IK failed"

# openrua op 126
python3 - <<'EOF'
s=open('plan4.py').read()
s=s.replace("closing=np.cross([0,0,-1],ax)                              # y_h = z_h x x_h so that x_h = ax",
            "closing=-np.cross([0,0,-1],ax)                             # x_h = -ax (toward bottle bottom); keeps j7 away from limit")
s=s.replace("R_D=rot_from_axes([0,0,-1],[0,1,0]); print('drop x_h',np.round(R_D[:,0],3))   # x_h=-x -> cork toward -x",
            "R_D=rot_from_axes([0,0,-1],[0,-1,0]); print('drop x_h',np.round(R_D[:,0],3))   # x_h=+x (bottom) -> cork toward -x")
s=s.replace("qpre=best_ik(r,PRE,R_G,n_rand=8)","qpre=best_ik(r,PRE,R_G,extra_seeds=[np.array([0.1,0.445,-0.1,-2.53,0.24,2.97,0.245])],n_rand=8)")
s=s.replace("from plan3 import obstacles, check_path, line","from plan3 import obstacles, check_path, line\nnp.set_printoptions(suppress=True,precision=3)")
open('plan4.py','w').write(s)
EOF
timeout 1200 python3 plan4.py 2>&1 | grep -v "IK failed"

# openrua op 127
sed -i "s/plan=np.load('plan3.npy',allow_pickle=True).item()/plan=np.load('plan3.npy',allow_pickle=True).item(); plan.update(np.load('plan4.npy',allow_pickle=True).item())/" exec_step.py
timeout 900 python3 exec_step.py pre4 grasp4 2>&1 | grep -v '^\[.*e'; timeout 60 python3 tools/perception/cam_snap.py agentview /workspace/g2_agent.png

# openrua op 128
cat > unstick.py <<'EOF'
from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q=r.arm_q(); p,R=r.tcp(q); print('tcp',p,'R z',R[:,2])
for dz in (0.04,0.08,0.12):
    sol=ik_near(r,p+[0,0,dz],R,q,max_dq=0.6,tries=10,avoid_collisions=True)
    print(dz,sol)
    if sol is not None:
        r.move_q(sol,3.0); q=sol
print('tcp',r.tcp()[0],'wrench',np.round(r.wrench(),2))
EOF
timeout 600 python3 unstick.py 2>&1 | grep -v "IK failed"

# openrua op 129
timeout 60 python3 tools/perception/cam_snap.py frontview /workspace/s_front.png; timeout 60 python3 tools/perception/cam_snap.py sideview /workspace/s_side.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand /workspace/s_eye.png

# openrua op 130
cat > nudge.py <<'EOF'
from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q=r.arm_q(); print('q',q,'tcp',r.tcp()[0],'w',np.round(r.wrench(),1))
q2=q.copy(); q2[1]-=0.08; q2[3]-=0.05
r.move_q(q2,2.0); print('q',r.arm_q(),'tcp',r.tcp()[0],'w',np.round(r.wrench(),1))
EOF
timeout 300 python3 nudge.py

# openrua op 131
cat > relax.py <<'EOF'
from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q=r.arm_q(); print('q',q,'w',np.round(r.wrench(),1))
r.move_q(q,1.0); print('after hold q',r.arm_q(),'w',np.round(r.wrench(),1),'tcp',r.tcp()[0])
q=r.arm_q(); p,R=r.tcp(q)
for i in range(5):
    sol=ik_near(r,p+[0,0,0.01*(i+1)],R,q,max_dq=0.3,tries=6,avoid_collisions=False)
    if sol is None: print('no ik'); break
    r.move_q(sol,1.0); q=r.arm_q(); print(' tcp',r.tcp()[0],'w',np.round(r.wrench(),1),'dq',np.round(q-sol,3))
EOF
timeout 300 python3 relax.py 2>&1 | grep -v "IK failed"

# openrua op 132
cat > feel.py <<'EOF'
from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot()
for d in ([0,0,-0.01],[0.015,0,0],[-0.015,0,0],[0,0.015,0],[0,-0.015,0]):
    q=r.arm_q(); r.move_q(q,0.8)   # relax
    p,R=r.tcp(q)
    sol=ik_near(r,p+np.array(d),R,q,max_dq=0.3,tries=6,avoid_collisions=False)
    if sol is None: print(d,'no ik'); continue
    r.move_q(sol,1.0); q2=r.arm_q(); print(d,'-> tcp',r.tcp(q2)[0],'w',np.round(r.wrench(),1),'moved',np.round(r.tcp(q2)[0]-p,3))
EOF
timeout 300 python3 feel.py 2>&1 | grep -v "IK failed"

# openrua op 133
timeout 60 python3 tools/perception/cam_snap.py agentview /workspace/s_agent.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('s_agent.png'); crop=im[0:200,0:260]; cv2.imwrite('crop_stuck_agent.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('s_side.png'); crop=im[80:260,240:420]; cv2.imwrite('crop_stuck_side.png',cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 134
cat > where.py <<'EOF'
from robot import *
from robot import state_valid
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q=r.arm_q(); print('q',q); print('valid',state_valid(r,q))
for n,(p,R) in fk_links(r,q).items(): print(n,p)
p,R=r.tcp(q); print('tcp',p); print('R\n',R)
print('fingers',r.fingers())
EOF
timeout 120 python3 where.py

# openrua op 135
cat > slide.py <<'EOF'
from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot()
for d in ([0.02,0,0],[0.02,0,0],[0.02,0,0],[0,0,0.02],[0,0,0.03],[0,0,0.05]):
    q=r.arm_q(); r.move_q(q,0.6)
    p,R=r.tcp(q)
    sol=ik_near(r,p+np.array(d),R,q,max_dq=0.3,tries=6,avoid_collisions=False)
    if sol is None: print(d,'no ik'); break
    r.move_q(sol,1.5); q2=r.arm_q(); print(d,'-> tcp',r.tcp(q2)[0],'w',np.round(r.wrench(),1),'moved',np.round(r.tcp(q2)[0]-p,3))
EOF
timeout 300 python3 slide.py 2>&1 | grep -v "IK failed\|move_q"

# openrua op 136
timeout 120 python3 scene.py birdview >/dev/null 2>&1; cat > stand.py <<'EOF'
import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); fx,fy,cx,cy=np.load('birdview_K.npy')
v,u=np.mgrid[0:d.shape[0],0:d.shape[1]]
z=d; X=(u-cx)*z/fx; Y=(v-cy)*z/fy
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
np.save('bird_xyz3.npy',P[...,:3])
x,y,zz=P[...,0],P[...,1],P[...,2]
res=0.02
for yy in np.arange(-0.60,-0.15,res):
    row=''
    for xx in np.arange(-0.40,0.06,res):
        m=(x>=xx)&(x<xx+res)&(y>=yy)&(y<yy+res)
        row+=f'{int(round((zz[m].max()-0.9)*100)):3d}' if m.any() else '  .'
    print(f'{yy:6.2f}',row)
print('x cols from -0.40 step 0.02')
EOF
python3 stand.py

# openrua op 137
timeout 60 python3 tools/perception/cam_snap.py sideview s_side2.png >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py agentview s_agent2.png >/dev/null 2>&1; python3 -c "
from robot import *
r=Robot(); q=r.arm_q(); fk=fk_links(r,q)
for k,(p,R) in fk.items(): print(k,np.round(p,3))
" 2>/dev/null

# openrua op 138
timeout 120 python3 scene.py sideview 318 152 318 160 290 183 350 183 290 190 290 233 350 233 290 240 300 255 345 262 2>/dev/null

# openrua op 139
cat > stand2.py <<'EOF'
import numpy as np
P=np.load('bird_xyz3.npy'); x,y,z=P[...,0],P[...,1],P[...,2]
m=(x>-0.35)&(x<0.1)&(y>-0.6)&(y<-0.15)&(z>0.95)&(z<1.3)
print('z hist',np.histogram(z[m],bins=np.arange(0.95,1.31,0.02)))
res=0.02
for yy in np.arange(-0.56,-0.15,res):
    row=''
    for xx in np.arange(-0.34,0.10,res):
        mm=m&(x>=xx)&(x<xx+res)&(y>=yy)&(y<yy+res)
        row+=f'{int(round((z[mm].max()-0.9)*100)):3d}' if mm.any() else '  .'
    print(f'{yy:6.2f}',row)
print('x cols from -0.34 step 0.02')
EOF
python3 stand2.py

# openrua op 140
python3 - <<'EOF'
import re
s=open('plan3.py').read()
s=s.replace("    if (((x-0.013)**2+(y+0.03)**2<0.068**2)&(z<1.026)).any(): hits.append('bowl')",
"    if (((x-0.013)**2+(y+0.03)**2<0.068**2)&(z<1.026)).any(): hits.append('bowl')\n    if ((x>-0.26)&(x<0.08)&(y>-0.52)&(y<-0.18)&(z<1.25)).any(): hits.append('stand')")
open('plan3.py','w').write(s)
EOF
grep -n stand plan3.py
cat > plan5.py <<'EOF'
from robot import *
from plan3 import line, check_path, obstacles
import plan4
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q0=r.arm_q(); p0,R0=r.tcp(q0)
p4=np.load('plan4.npy',allow_pickle=True).item()
qpre=p4['pre4'][0] if isinstance(p4['pre4'],list) else p4['pre4']
print('pre4 q',qpre, 'tcp',r.tcp(qpre)[0])
R_G=plan4.R_G
UP=np.array([p0[0],p0[1],1.40])
q1=ik_near(r,UP,R0,q0,max_dq=0.6,tries=10)
print('q1',q1)
W2=np.array([-0.20,-0.08,1.40])
q2=ik_near(r,W2,R_G,qpre,max_dq=1.5,tries=12)
print('q2',q2, r.tcp(q2)[0])
PRE=r.tcp(qpre)[0]
seg=line(r,W2,PRE,R_G,q2,5)
print('seg end',seg[-1])
check_path(r,[q0,q1],n_sub=10,label='up')
check_path(r,[q1,q2],n_sub=30,label='over+rotate')
check_path(r,[q2]+seg+[qpre],n_sub=6,label='down to pre4')
plan={'up5':[q1],'over5':[q2],'down5':seg+[qpre]}
np.save('plan5.npy',plan,allow_pickle=True)
EOF
timeout 300 python3 plan5.py 2>&1 | grep -v "IK failed"

# openrua op 141
grep -n "npy" exec_step.py

# openrua op 142
sed -i "3s/$/; plan.update(np.load('plan5.npy',allow_pickle=True).item())/" exec_step.py && timeout 600 python3 exec_step.py 0.1 up5 over5 down5 2>&1 | grep -v "IK failed\|WARN"

# openrua op 143
sed -n 12,30p exec_step.py

# openrua op 144
timeout 900 python3 exec_step.py up5 over5 down5 2>&1 | grep -v "IK failed\|WARN"

# openrua op 145
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand s_eye2.png >/dev/null 2>&1; timeout 120 python3 scene.py birdview >/dev/null 2>&1; python3 hmap.py -0.32 -0.12 -0.08 0.04 0.01 2>/dev/null | head -30

# openrua op 146
timeout 600 python3 exec_step.py grasp4 close 2>&1 | grep -v "IK failed\|WARN"

# openrua op 147
timeout 600 python3 exec_step.py lift4 2>&1 | grep -v "IK failed\|WARN"; timeout 60 python3 tools/perception/cam_snap.py sideview s_side3.png >/dev/null 2>&1

# openrua op 148
cat > plan6.py <<'EOF'
from robot import *
from plan3 import line, check_path
import plan4
np.set_printoptions(suppress=True,precision=3)
r=Robot()
p4=np.load('plan4.npy',allow_pickle=True).item()
qmid=p4['mid4'][-1]; MID=r.tcp(qmid)[0]; R_D=plan4.R_D
PREDROP=np.array([0.0,0.15,1.20]); DROP=np.array([0.0,0.15,1.11])
pd=line(r,MID,PREDROP,R_D,qmid,4); dr=line(r,PREDROP,DROP,R_D,pd[-1],3); up=line(r,DROP,PREDROP,R_D,dr[-1],2)
print('predrop',pd[-1],'drop',dr[-1])
# custom bottle points for centre grasp: +-0.06 along x_h, radius 0.02
def bpts(tcp,R):
    pts=[]
    for s in np.linspace(-0.06,0.06,5):
        for ang in np.linspace(0,2*np.pi,6,endpoint=False):
            pts.append(tcp+s*R[:,0]+0.02*(np.cos(ang)*R[:,1]+np.sin(ang)*R[:,2]))
    return np.array(pts)
import plan3; plan3.bottle_pts=bpts
qs=[p4['lift4'][-1],qmid]+pd+dr
check_path(r,qs,held=True,n_sub=8,label='lift->mid->drop')
np.save('plan6.npy',{'predrop6':pd,'drop6':dr,'up6':up},allow_pickle=True)
EOF
timeout 300 python3 plan6.py 2>&1 | grep -v "IK failed\|WARN" | tail -4
sed -i "3s/$/; plan.update(np.load('plan6.npy',allow_pickle=True).item())/" exec_step.py

# openrua op 149
timeout 900 python3 exec_step.py mid4 predrop6 drop6 2>&1 | grep -v "IK failed\|WARN"; timeout 60 python3 tools/perception/cam_snap.py agentview s_agent3.png >/dev/null 2>&1

# openrua op 150
timeout 600 python3 exec_step.py open up6 2>&1 | grep -v "IK failed\|WARN"; timeout 120 python3 scene.py birdview >/dev/null 2>&1; python3 hmap.py -0.14 0.15 0.05 0.25 0.01 2>/dev/null

# openrua op 151
cat > upcheck.py <<'EOF'
from robot import *
from plan3 import check_path
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q0=r.arm_q(); p,R=r.tcp(q0)
q1=ik_near(r,np.array([-0.12,0.12,1.42]),R,q0,max_dq=0.8,tries=10)
check_path(r,[q0,q1],n_sub=10,label='up-away')
r.move_q(q1, max(np.abs(q1-q0).max()/0.1,0.6))
q=r.arm_q(); print('tcp',r.tcp(q)[0],'w',np.round(r.wrench(),1))
np.save('q_view.npy',q)
EOF
timeout 300 python3 upcheck.py 2>&1 | grep -v "IK failed\|WARN"; timeout 120 python3 scene.py birdview >/dev/null 2>&1; python3 hmap.py -0.14 0.15 0.05 0.25 0.01 2>/dev/null

# openrua op 152
timeout 60 python3 tools/perception/cam_snap.py agentview s_agent4.png >/dev/null 2>&1; python3 -c "
import cv2; im=cv2.imread('s_agent4.png'); cv2.imwrite('crop_drawer.png', cv2.resize(im[180:400,300:560],None,fx=2,fy=2))"

# openrua op 153
cat > trans.py <<'EOF'
from robot import *
from plan3 import check_path
import plan3
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q0=r.arm_q()
plan=np.load('plan3.npy',allow_pickle=True).item()
qpp=plan['prepush'][0]; print('prepush q',qpp,'tcp',r.tcp(qpp)[0])
check_path(r,[q0,qpp],n_sub=25,label='view->prepush')
print('dq',np.round(qpp-q0,2))
EOF
timeout 120 python3 trans.py 2>&1 | grep -v "IK failed\|WARN"

# openrua op 154
python3 -c "
from robot import *
r=Robot(); q0=r.arm_q(); plan=np.load('plan3.npy',allow_pickle=True).item()
print('selfcoll failures:',path_valid(r,[q0,plan['prepush'][0]],step=0.05,finger=0.0))
" 2>&1 | grep -v WARN && timeout 900 python3 exec_step.py close prepush push0 2>&1 | grep -v "IK failed\|WARN"

# openrua op 155
timeout 60 python3 tools/perception/cam_snap.py agentview s_agent5.png >/dev/null 2>&1; python3 -c "
import cv2; im=cv2.imread('s_agent5.png'); cv2.imwrite('crop_push0.png', cv2.resize(im[150:420,280:600],None,fx=2,fy=2))"

# openrua op 156
timeout 900 python3 exec_step.py push1 2>&1 | grep -v "IK failed\|WARN"; timeout 60 python3 tools/perception/cam_snap.py agentview s_agent6.png >/dev/null 2>&1; python3 -c "
import cv2; im=cv2.imread('s_agent6.png'); cv2.imwrite('crop_push1.png', cv2.resize(im[150:420,280:600],None,fx=2,fy=2))"

# openrua op 157
timeout 900 python3 exec_step.py pushup prepushB push0B push1B 2>&1 | grep -v "IK failed\|WARN"

# openrua op 158
timeout 900 python3 exec_step.py pushupB 2>&1 | grep -v "IK failed\|WARN"; timeout 120 python3 scene.py birdview >/dev/null 2>&1; python3 hmap.py -0.14 0.15 0.03 0.25 0.01 2>/dev/null

# openrua op 159
timeout 60 python3 tools/perception/cam_snap.py agentview s_agent7.png >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py frontview s_front7.png >/dev/null 2>&1; python3 -c "
import cv2; im=cv2.imread('s_agent7.png'); cv2.imwrite('crop_closed.png', cv2.resize(im[150:420,280:600],None,fx=2,fy=2))"

# openrua op 160
timeout 120 python3 scene.py frontview 445 300 445 335 445 375 445 395 445 410 450 300 450 400 2>/dev/null

# openrua op 161
python3 - <<'EOF'
import numpy as np
d=np.load('frontview_depth.npy'); T=np.load('frontview_T.npy'); fx,fy,cx,cy=np.load('frontview_K.npy')
v,u=np.mgrid[0:480,0:640]; X=(u-cx)*d/fx; Y=(v-cy)*d/fy
P=(np.stack([X,Y,d,np.ones_like(d)],-1)@T.T)[...,:3]
x,y,z=P[...,0],P[...,1],P[...,2]
# front face points: within cabinet x range, y near 0.2, exclude side face x>0.125
m=(x>-0.10)&(x<0.12)&(y>0.15)&(y<0.26)&(z>0.92)&(z<1.13)
for z0 in np.arange(0.92,1.13,0.01):
    mm=m&(z>=z0)&(z<z0+0.01)
    if mm.sum()>3: print(f'z {z0:.2f}-{z0+0.01:.2f}: n={mm.sum():4d} y median {np.median(y[mm]):.4f} min {y[mm].min():.3f} max {y[mm].max():.3f}  x range {x[mm].min():.3f}..{x[mm].max():.3f}')
EOF

# openrua op 162
python3 - <<'EOF' 2>&1 | grep -v WARN
from robot import *
from pushsearch import hand_pts_fine
np.set_printoptions(suppress=True,precision=3)
r=Robot(); plan=np.load('plan3.npy',allow_pickle=True).item()
q=plan['push1B'][-1]; p,R=r.tcp(q); print('tcp',p); print('R\n',R)
pts=hand_pts_fine(p,R,0.0)
print('n pts',len(pts))
# points with y>0.19: where is the hand front?
front=pts[pts[:,1]>pts[:,1].max()-0.02]
print('max y',pts[:,1].max(),'points near front: z range',front[:,2].min(),front[:,2].max(),'x range',front[:,0].min(),front[:,0].max())
# palm region: hand box z_h in [0,0.066]; find lowest z of hand box
hb=pts[(pts-p)@R[:,2] < -0.03]  # behind fingers
print('hand box y max',hb[:,1].max(),'z min',hb[:,2].min())
low=pts[pts[:,2]<0.985]; print('points below wall top: y range',low[:,1].min(),low[:,1].max(),'x',low[:,0].min(),low[:,0].max())
EOF

# openrua op 163
cat > pushmore.py <<'EOF'
from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot(); plan=np.load('plan3.npy',allow_pickle=True).item()
seq=plan['push0B']+plan['push1B']
# go back down to end of push1B via push0B end, then push1B
for q in [plan['push0B'][-1]]+plan['push1B']:
    if r.move_q(q,max(np.abs(q-r.arm_q()).max()/0.1,0.6))!=0: raise SystemExit('abort')
q=r.arm_q(); p,R=r.tcp(q); print('at',p,'w',np.round(r.wrench(),1))
for dy in [0.005,0.005,0.005]:
    tgt=p+np.array([0,dy,0]); sol=ik_near(r,tgt,R,q,max_dq=0.3,tries=8,avoid_collisions=False)
    if sol is None: print('no ik'); break
    code=r.move_q(sol,1.0); q=r.arm_q(); p2=r.tcp(q)[0]; w=r.wrench()
    print('target y',round(tgt[1],4),'reached',p2,'w',np.round(w,1),'code',code)
    p=p2
    if np.linalg.norm(w[:3])>15: print('force high, stop'); break
r.move_q(r.arm_q(),0.6)
EOF
timeout 600 python3 pushmore.py 2>&1 | grep -v "IK failed\|WARN\|move_q"

# openrua op 164
python3 - <<'EOF' 2>&1 | grep -v "WARN\|IK failed"
from robot import *
np.set_printoptions(suppress=True,precision=3)
r=Robot(); q=r.arm_q(); p,R=r.tcp(q)
for z in [1.05,1.15]:
    sol=ik_near(r,np.array([p[0],0.16,z]),R,q,max_dq=0.5,tries=8)
    r.move_q(sol,max(np.abs(sol-q).max()/0.1,0.6)); q=r.arm_q()
print('tcp',r.tcp(q)[0],'w',np.round(r.wrench(),1))
EOF
timeout 120 python3 scene.py frontview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
d=np.load('frontview_depth.npy'); T=np.load('frontview_T.npy'); fx,fy,cx,cy=np.load('frontview_K.npy')
v,u=np.mgrid[0:480,0:640]; X=(u-cx)*d/fx; Y=(v-cy)*d/fy
P=(np.stack([X,Y,d,np.ones_like(d)],-1)@T.T)[...,:3]
x,y,z=P[...,0],P[...,1],P[...,2]
m=(x>-0.10)&(x<0.12)&(y>0.15)&(y<0.26)&(z>0.92)&(z<1.13)
for z0 in np.arange(0.92,1.13,0.01):
    mm=m&(z>=z0)&(z<z0+0.01)
    if mm.sum()>3: print(f'z {z0:.2f}-{z0+0.01:.2f}: n={mm.sum():4d} y median {np.median(y[mm]):.4f} min {y[mm].min():.3f} max {y[mm].max():.3f}  x range {x[mm].min():.3f}..{x[mm].max():.3f}')
EOF

# openrua op 165
timeout 120 python3 scene.py birdview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); fx,fy,cx,cy=np.load('birdview_K.npy')
v,u=np.mgrid[0:480,0:640]; X=(u-cx)*d/fx; Y=(v-cy)*d/fy
P=(np.stack([X,Y,d,np.ones_like(d)],-1)@T.T)[...,:3]
x,y,z=P[...,0],P[...,1],P[...,2]
# table area, objects between 0.91 and 0.99 (bottle-height), excluding cabinet footprint and stand and bowl
m=(z>0.91)&(z<0.99)&(x>-0.55)&(x<0.45)&(y>-0.7)&(y<0.7)
m&=~((x>-0.13)&(x<0.14)&(y>0.19)&(y<0.43))   # cabinet
m&=~((x>-0.26)&(x<0.08)&(y>-0.52)&(y<-0.18)) # stand
m&=~(((x-0.013)**2+(y+0.03)**2<0.075**2))     # bowl
print('bottle-height points outside cabinet/stand/bowl:',m.sum())
if m.sum(): print('at x',x[m].min(),x[m].max(),'y',y[m].min(),y[m].max())
# anything protruding in front of the cabinet (drawer open?)
m2=(z>0.91)&(z<1.12)&(x>-0.13)&(x<0.14)&(y>0.10)&(y<0.205)
print('points in front of cabinet face (y 0.10-0.205, z 0.91-1.12):',m2.sum())
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview final_agent.png >/dev/null 2>&1

# openrua op 166
python3 - <<'EOF'
import numpy as np
d=np.load('birdview_depth.npy'); T=np.load('birdview_T.npy'); fx,fy,cx,cy=np.load('birdview_K.npy')
v,u=np.mgrid[0:480,0:640]; X=(u-cx)*d/fx; Y=(v-cy)*d/fy
P=(np.stack([X,Y,d,np.ones_like(d)],-1)@T.T)[...,:3]
x,y,z=P[...,0],P[...,1],P[...,2]
m=(z>0.91)&(z<0.99)&(x>-0.55)&(x<0.45)&(y>-0.7)&(y<0.7)
m&=~((x>-0.13)&(x<0.14)&(y>0.19)&(y<0.43)); m&=~((x>-0.26)&(x<0.08)&(y>-0.52)&(y<-0.18)); m&=~(((x-0.013)**2+(y+0.03)**2<0.075**2))
from collections import Counter
c=Counter(zip(np.round(x[m],1),np.round(y[m],1)))
for k,n in sorted(c.items()): print(k,n,'z',np.round(np.median(z[m&(np.round(x,1)==k[0])&(np.round(y,1)==k[1])]),3))
m3=(z>0.91)&(z<1.12)&(x>-0.13)&(x<0.14)&(y>0.10)&(y<0.17)
print('points y 0.10-0.17 in front of cabinet:',m3.sum())
EOF

# openrua op 167
ls /claude-config/projects/-workspace/memory/ 2>/dev/null; cat /claude-config/projects/-workspace/memory/MEMORY.md 2>/dev/null

# openrua op 168
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-bridge-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-bridge-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-bridge-quirks
description: Non-obvious facts about the /workspace Franka Panda MuJoCo/ROS2 bridge (velocity limit, unmodeled obstacles, gripper behavior) learned while doing the bottle-in-drawer task
metadata:
  type: project
---

Learned 2026-09-13 while completing "put the bottle in the bottom drawer and close it" on the Panda sim (/workspace):

- FollowJointTrajectory bridge silently enforces ~0.23 rad/s joint speed; faster goals return error -5 and stop partway with joints lagging unevenly. Send one goal per waypoint at <=0.1 rad/s (see /workspace/exec_step.py, robot.py).
- Error -5 also means "blocked by contact"; re-commanding the measured q relaxes forces. When trapped, probe 1-2 cm moves per axis and slide along the free axis before lifting.
- Scene has a tall two-tier wooden stand (x -0.26..0.08, y -0.52..-0.18, post top z~1.235) that is NOT in MoveIt's planning scene — the docs/plan obstacle models must include it (added to plan3.py obstacles()). MoveIt state validity = self-collision only.
- Gripper is open/closed only; a 4 cm bottle grasped near its neck slipped out during a wrist reorientation. Grasp at the mass center, top-down, and only yaw the hand during transit.
- Drawer "closed" = front face flush at world y~0.217 (frontview depth profile); the pre-planned push left it 1.2 cm short, extra 1.5 cm push in 5 mm steps finished it.

**Why:** none of this is in machine.yaml/docs; each cost a failed attempt.
**How to apply:** reuse robot.py/plan3.py/exec_step.py helpers; always check paths against the full obstacle model including the stand; verify with birdview height maps (hmap.py) and frontview depth.
OPENRUA_EOF

# openrua op 169
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda sim bridge quirks](panda-sim-bridge-quirks.md) — 0.23 rad/s velocity limit, unmodeled wooden stand, gripper/grasp lessons, drawer-flush criterion
OPENRUA_EOF
