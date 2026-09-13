#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---ACTIONS && ros2 action list && echo ---SERVICES && ros2 service list | head -50 && echo ---NODES && ros2 node list

# openrua op 4
mkdir -p img && for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c img/$c.png; done; ros2 topic echo /joint_states --once

# openrua op 5
for p in "257 292" "548 237" "65 220" "320 240"; do echo -n "eih $p -> "; timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand $p; done; ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12 & sleep 4; kill %1 2>/dev/null; ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|child" | head -40

# openrua op 6
timeout 60 ros2 run tf2_tools view_frames -o /workspace/frames >/dev/null 2>&1; ls frames*; python3 - <<'EOF'
import yaml
d=yaml.safe_load(open('/workspace/frames.yaml'))
for k,v in d.items(): print(k,'<-',v['parent'])
EOF

# openrua op 7
grep -oE '"[a-z0-9_]+" -> "[a-z0-9_]+"' frames.gv

# openrua op 8
cat > /workspace/cam3d.py <<'EOF'
#!/usr/bin/env python3
"""cam3d.py <camera> [u v]...  : world xyz for pixels; also table plane estimate & world->panda_link0."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def grab(node, topic, T, timeout=30):
    got={}
    sub=node.create_subscription(T, topic, lambda m: got.setdefault('m',m),1)
    import time; t0=time.time()
    while 'm' not in got and time.time()-t0<timeout: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(sub); return got.get('m')

def tfmat(t):
    q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]; return T

cam=sys.argv[1]; px=[(int(sys.argv[i]),int(sys.argv[i+1])) for i in range(2,len(sys.argv)-1,2)]
rclpy.init(); node=rclpy.create_node('cam3d'); buf=Buffer(); TransformListener(buf,node)
depth=CvBridge().imgmsg_to_cv2(grab(node,f'/{cam}/depth/image_raw',Image),'passthrough').astype(float)
info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
import time; t0=time.time()
while time.time()-t0<20:
    rclpy.spin_once(node,timeout_sec=0.2)
    if buf.can_transform('world',f'{cam}_optical_frame',rclpy.time.Time()) and buf.can_transform('world','panda_link0',rclpy.time.Time()): break
T=tfmat(buf.lookup_transform('world',f'{cam}_optical_frame',rclpy.time.Time()))
Tb=tfmat(buf.lookup_transform('world','panda_link0',rclpy.time.Time()))
print('world->panda_link0 t=',Tb[:3,3].round(4)); print('cam pos',T[:3,3].round(4))
print('depth shape',depth.shape,'min/max',np.nanmin(depth),np.nanmax(depth))
H,W=depth.shape
vv,uu=np.mgrid[0:H,0:W]
Z=depth; X=(uu-cx)*Z/fx; Y=(vv-cy)*Z/fy
P=np.stack([X,Y,Z,np.ones_like(Z)],-1)@T.T
np.save(f'/workspace/img/{cam}_world.npy',P[...,:3])
zs=P[...,2][np.isfinite(Z)&(Z>0)]
hist,edges=np.histogram(zs,bins=60)
top=np.argsort(hist)[-4:]
print('dominant world-z bins:',[(round(edges[i],3),hist[i]) for i in sorted(top)])
for u,v in px:
    print(f'px({u},{v}) depth={Z[v,u]:.4f} world={P[v,u,:3].round(4)}')
rclpy.shutdown()
EOF
timeout 120 python3 /workspace/cam3d.py robot0_eye_in_hand 257 292 548 237 65 220 320 240 240 130

# openrua op 9
python3 - <<'EOF'
import numpy as np, cv2
P=np.load('img/robot0_eye_in_hand_world.npy')
img=cv2.imread('img/robot0_eye_in_hand.png')
z=P[...,2]
mask=(z>0.435)&(z<0.70)&np.isfinite(z)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<200: continue
    m=lab==i
    pts=P[m]
    x0,y0,w,h,a=stats[i]
    print(f'comp{i} px_centroid=({cent[i][0]:.0f},{cent[i][1]:.0f}) bbox=({x0},{y0},{w},{h}) area={a}')
    print('   world xy min',pts[:,:2].min(0).round(3),'max',pts[:,:2].max(0).round(3),'z top',np.percentile(pts[:,2],95).round(3),'mean xy',pts[:,:2].mean(0).round(3))
    print('   mean color BGR', img[m].mean(0).round(0))
EOF

# openrua op 10
timeout 120 python3 /workspace/cam3d.py birdview 383 288 && python3 - <<'EOF'
import numpy as np, cv2
P=np.load('img/birdview_world.npy')
z=P[...,2]
mask=((z>0.50)&(z<0.70)&np.isfinite(z)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<50: continue
    pts=P[lab==i]
    print(f'comp{i} px=({cent[i][0]:.0f},{cent[i][1]:.0f}) area={stats[i,4]} xy min',pts[:,:2].min(0).round(3),'max',pts[:,:2].max(0).round(3),'ztop',np.percentile(pts[:,2],95).round(3))
# basket interior floor: points within basket bbox with z near table
EOF

# openrua op 11
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -A3 "Translation" | head -4; ros2 interface show moveit_msgs/srv/GetPositionFK | head -30

# openrua op 12
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick-and-place controller for this Panda. All coordinates are WORLD frame.

Subcommands (chain several in one invocation, separated by '--'):
  js                          print arm joints + finger positions
  fk                          FK of panda_hand / panda_link8 (world frame)
  grip open|close             gripper, then report finger gap
  tcp X Y Z [secs] [yaw_deg]  IK+trajectory so the TCP (fingertip point) lands
                              at world X Y Z, hand pointing down, fingers
                              closing along world Y (yaw 0) or rotated yaw_deg
  home                        go to the start configuration
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
LIM = FJT["limits_rad"]
TCP_OFF = M["hand"]["tcp_offset_m"]
W2B = np.array([-0.510, 0.0, 0.420])  # world -> panda_link0 translation (rotation = identity)
HOME = [0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854]


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait(self, fut, timeout):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            self.spin(0.1)
        return fut.result() if fut.done() else None

    # ---- sensing
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 30:
            self.spin(0.2)
        m = self._js["m"]
        d = dict(zip(m.name, m.position))
        return [d[j] for j in ARM], d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def cmd_js(self):
        q, f1, f2 = self.joints()
        log("arm", np.round(q, 4).tolist(), "fingers", round(f1, 4), round(f2, 4))

    def cmd_fk(self):
        q, _, _ = self.joints()
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand", "panda_link8"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = q
        res = self.wait(self.fk.call_async(req), 60)
        for name, ps in zip(res.fk_link_names, res.pose_stamped):
            p, o = ps.pose.position, ps.pose.orientation
            w = np.array([p.x, p.y, p.z]) + W2B
            log(f"FK {name}: frame='{ps.header.frame_id}' base=({p.x:.4f},{p.y:.4f},{p.z:.4f}) "
                f"world=({w[0]:.4f},{w[1]:.4f},{w[2]:.4f}) q=({o.x:.3f},{o.y:.3f},{o.z:.3f},{o.w:.3f})")

    # ---- acting
    def move(self, target, secs, tol=0.02, retries=2):
        self.fjt.wait_for_server(10)
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ARM
            pt = JointTrajectoryPoint(positions=[float(x) for x in target])
            pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
            goal.trajectory.points = [pt]
            gh = self.wait(self.fjt.send_goal_async(goal), 120)
            if gh is None:
                log("FJT goal not accepted in time")
                return False
            res = self.wait(gh.get_result_async(), 900)
            code = res.result.error_code if res else None
            q, _, _ = self.joints()
            err = float(np.max(np.abs(np.array(q) - np.array(target))))
            log(f"FJT done code={code} max_joint_err={err:.4f}")
            if err < tol:
                return True
            log("target not reached, resending")
        return False

    def cmd_home(self, secs=4.0):
        return self.move(HOME, secs)

    def cmd_grip(self, what):
        width = GRIP["open_m"] if what == "open" else GRIP["closed_m"]
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        gh = self.wait(self.grip.send_goal_async(g), 120)
        res = self.wait(gh.get_result_async(), 600)
        r = res.result if res else None
        _, f1, f2 = self.joints()
        log(f"grip {what}: reached={getattr(r,'reached_goal',None)} stalled={getattr(r,'stalled',None)} "
            f"fingers={f1:.4f},{f2:.4f} gap={f1 - f2:.4f}")
        return f1, f2

    def solve_ik(self, pos_base, quat, seed):
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = [float(x) for x in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        res = self.wait(self.ik.call_async(req), 120)
        if res is None or res.error_code.val != 1:
            return None, (None if res is None else res.error_code.val)
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM], 1

    def cmd_tcp(self, x, y, z, secs=3.0, yaw_deg=0.0):
        """TCP at world (x,y,z), hand z pointing down, fingers along world Y rotated by yaw."""
        # hand orientation: R = Rz(yaw) * Rx(pi)  -> hand x ~ world x, hand z = -world z
        yaw = math.radians(yaw_deg)
        qx, qy, qz, qw = math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0  # Rz(yaw)*Rx(pi)
        # hand frame origin = TCP - TCP_OFF * (hand z axis in world) = TCP + TCP_OFF * world z
        hand_world = np.array([x, y, z + TCP_OFF])
        hand_base = hand_world - W2B
        seed, _, _ = self.joints()
        best = None
        for k in range(6):
            sol, code = self.solve_ik(hand_base, (qx, qy, qz, qw), seed)
            if sol is None:
                log(f"IK attempt {k} failed code={code}")
                continue
            d = np.abs(np.array(sol) - np.array(seed))
            d = np.minimum(d, 2 * np.pi - d)
            jump = float(d.max())
            ok_lim = all(lo <= v <= hi for v, (lo, hi) in zip(sol, LIM))
            log(f"IK attempt {k}: max jump {jump:.3f} limits_ok={ok_lim} sol={np.round(sol,3).tolist()}")
            if ok_lim and (best is None or jump < best[0]):
                best = (jump, sol)
            if ok_lim and jump < 1.0:
                break
        if best is None:
            log("IK: no solution; NO MOTION")
            return False
        jump, sol = best
        if jump > 2.0:
            log("IK: solution jumps too far from current config; refusing")
            return False
        ok = self.move(sol, secs)
        self.cmd_fk()
        return ok


def main():
    argv = sys.argv[1:]
    if not argv:
        raise SystemExit(__doc__)
    cmds, cur = [], []
    for a in argv:
        if a == "--":
            cmds.append(cur); cur = []
        else:
            cur.append(a)
    cmds.append(cur)
    c = Ctl()
    for cmd in cmds:
        if not cmd:
            continue
        log(">>", " ".join(cmd))
        name, args = cmd[0], cmd[1:]
        if name == "js":
            c.cmd_js()
        elif name == "fk":
            c.cmd_fk()
        elif name == "grip":
            c.cmd_grip(args[0])
        elif name == "home":
            c.cmd_home(*map(float, args))
        elif name == "tcp":
            ok = c.cmd_tcp(*map(float, args))
            if not ok:
                log("ABORT: tcp move failed")
                break
        else:
            log("unknown command", name)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 13
timeout 300 python3 -u ctl.py js -- fk 2>&1 | tail -20

# openrua op 14
timeout 120 python3 - <<'EOF'
import rclpy, numpy as np
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('u')
got={}
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',lambda m: got.setdefault('m',m),qos)
import time; t0=time.time()
while 'm' not in got and time.time()-t0<20: rclpy.spin_once(n,timeout_sec=0.2)
open('robot.urdf','w').write(got['m'].data)
from urdf_parser_py.urdf import URDF
r=URDF.from_xml_string(got['m'].data)
from scipy.spatial.transform import Rotation as R
q={'panda_joint1':0,'panda_joint2':-0.161,'panda_joint3':0,'panda_joint4':-2.4446,'panda_joint5':0,'panda_joint6':2.2268,'panda_joint7':0.7854}
def T_of(j, val):
    o=j.origin; T=np.eye(4)
    if o is not None:
        T[:3,:3]=R.from_euler('xyz',o.rpy).as_matrix(); T[:3,3]=o.xyz
    if j.type in ('revolute','continuous'):
        Tj=np.eye(4); Tj[:3,:3]=R.from_rotvec(np.array(j.axis)*val).as_matrix(); T=T@Tj
    return T
chain=r.get_chain(r.get_root(),'panda_hand',joints=True,links=False)
print('root',r.get_root(),chain)
T=np.eye(4)
for jn in chain:
    j=r.joint_map[jn]; T=T@T_of(j,q.get(jn,0))
    print(jn, j.type, T[:3,3].round(4))
print('hand quat xyzw', R.from_matrix(T[:3,:3]).as_quat().round(3))
EOF

# openrua op 15
python3 - <<'EOF'
import re
s=open('ctl.py').read()
s=s.replace('W2B = np.array([-0.510, 0.0, 0.420])  # world -> panda_link0 translation (rotation = identity)',
 'W2B = np.array([0.0, 0.0, 0.0])  # measured: FK/IK model frame == world on this machine (FK of home pose == TF world->panda_hand)')
s=s.replace('''    def cmd_home(self, secs=4.0):''','''    def cmd_iktest(self):
        """IK for the current hand pose must give back ~the current joints."""
        q, _, _ = self.joints()
        sol, code = self.solve_ik(np.array([-0.053, 0.0, 0.7776]) - W2B, (1.0, 0.0, -0.028, 0.0), q)
        log("iktest code", code, "sol", None if sol is None else np.round(sol, 3).tolist(), "cur", np.round(q, 3).tolist())

    def cmd_home(self, secs=4.0):''')
s=s.replace('''        elif name == "fk":
            c.cmd_fk()''','''        elif name == "fk":
            c.cmd_fk()
        elif name == "iktest":
            c.cmd_iktest()''')
open('ctl.py','w').write(s)
EOF
timeout 300 python3 -u ctl.py iktest 2>&1 | tail

# openrua op 16
python3 - <<'EOF'
s=open('ctl.py').read()
s=s.replace('''        yaw = math.radians(yaw_deg)
        qx, qy, qz, qw = math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0  # Rz(yaw)*Rx(pi)
''','''        # measured: the IK tip link is panda_link8 (hand = link8 rotated -45deg about z),
        # so the link8 target rotation is R_hand * Rz(+45deg)
        from scipy.spatial.transform import Rotation as R
        R_hand = R.from_euler("z", yaw_deg, degrees=True) * R.from_euler("x", 180, degrees=True)
        qx, qy, qz, qw = (R_hand * R.from_euler("z", 45, degrees=True)).as_quat()
''')
s=s.replace('''        sol, code = self.solve_ik(np.array([-0.053, 0.0, 0.7776]) - W2B, (1.0, 0.0, -0.028, 0.0), q)''',
'''        sol, code = self.solve_ik(np.array([-0.053, 0.0, 0.7776]) - W2B, (0.924, -0.383, -0.026, 0.011), q)''')
open('ctl.py','w').write(s)
EOF
timeout 300 python3 -u ctl.py iktest 2>&1 | tail -3

# openrua op 17
python3 - <<'EOF'
import numpy as np, cv2
P=np.load('img/robot0_eye_in_hand_world.npy'); z=P[...,2]
for name,(x0,y0,x1,y1) in {'butter':(-0.13,0.03,-0.03,0.10),'cheese':(-0.08,-0.27,0.03,-0.20)}.items():
    m=(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(z>0.435)&(z<0.5)
    pts=P[m]; top=np.percentile(pts[:,2],90)
    tp=pts[pts[:,2]>top-0.006]
    print(name,'top z',round(top,4),'topface n',len(tp),'center',tp[:,:2].mean(0).round(4),'xrange',tp[:,0].min().round(3),tp[:,0].max().round(3),'yrange',tp[:,1].min().round(3),tp[:,1].max().round(3))
EOF

# openrua op 18
nohup python3 -u ctl.py grip open -- tcp -0.079 0.066 0.55 4 -- tcp -0.079 0.066 0.434 3 -- grip close -- tcp -0.079 0.066 0.60 3 > log_butter1.txt 2>&1 &
echo started

# openrua op 19
sleep 60; cat log_butter1.txt

# openrua op 20
timeout 120 python3 tools/perception/cam_snap.py agentview img/agent_butter_lift.png

# openrua op 21
timeout 600 python3 -u ctl.py tcp -0.005 0.25 0.68 4 -- grip open -- tcp -0.005 0.25 0.78 3 2>&1 | grep -v "^$"

# openrua op 22
timeout 200 python3 tools/perception/cam_snap.py agentview img/agent_after_butter.png && timeout 200 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih_after_butter.png

# openrua op 23
timeout 900 python3 -u ctl.py tcp -0.022 -0.235 0.58 4 -- tcp -0.022 -0.235 0.436 3 -- grip close -- tcp -0.022 -0.235 0.62 3 2>&1 | grep -v "^$"

# openrua op 24
timeout 200 python3 tools/perception/cam_snap.py agentview img/agent_cheese_lift.png

# openrua op 25
timeout 900 python3 -u ctl.py tcp -0.02 0.0 0.70 4 -- tcp 0.01 0.25 0.68 4 -- grip open -- tcp 0.01 0.25 0.80 3 2>&1 | grep -v "^$"

# openrua op 26
timeout 200 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih_final.png && timeout 200 python3 tools/perception/cam_snap.py agentview img/agent_final.png

# openrua op 27
timeout 120 python3 /workspace/cam3d.py robot0_eye_in_hand 320 285 2>&1 | tail -3; python3 - <<'EOF'
import numpy as np, cv2
P=np.load('img/robot0_eye_in_hand_world.npy'); img=cv2.imread('img/eih_final.png')
z=P[...,2]
# basket footprint (from birdview): x -0.085..0.075, y 0.165..0.337 ; look at what sits inside below the rim
m=(P[...,0]>-0.07)&(P[...,0]<0.06)&(P[...,1]>0.18)&(P[...,1]<0.32)&(z>0.43)&(z<0.55)&np.isfinite(z)
pts=P[m]; print('points inside basket above floor:',len(pts),'z range',pts[:,2].min().round(3),pts[:,2].max().round(3),'xy center',pts[:,:2].mean(0).round(3))
# color check of those pixels: red (butter) and blue/white (cheese)
c=img[m]; print('mean BGR of inside-basket object pixels',c.mean(0).round(0))
# nothing at the old object positions?
for name,(x,y) in {'butter_old':(-0.079,0.066),'cheese_old':(-0.022,-0.235)}.items():
    mm=(np.abs(P[...,0]-x)<0.03)&(np.abs(P[...,1]-y)<0.02)&np.isfinite(z)
    print(name,'max z at old spot',np.nanmax(z[mm]).round(3) if mm.any() else 'not in view')
EOF
