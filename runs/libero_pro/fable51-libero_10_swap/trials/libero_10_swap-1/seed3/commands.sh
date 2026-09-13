#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ======; cat /workspace/tools/perception/cam_snap.py; echo ======; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ======; cat tools/action/gripper_cmd.py; echo ======; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; timeout 30 ros2 topic list; echo ====; timeout 20 ros2 action list; echo ====; timeout 20 ros2 node list

# openrua op 4
cd /workspace; timeout 20 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 30 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
cd /workspace; python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[170:420,100:420]
cv2.imwrite('crop_a.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
crop=im[240:400,300:420]
cv2.imwrite('crop_b.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
cd /workspace; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo ====; timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once

# openrua op 7
cd /workspace; cat > tfdump.py <<'EOF'
import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node('tfdump')
got={}
def cb(m, static):
    for t in m.transforms:
        got[(t.header.frame_id,t.child_frame_id)] = (t.transform.translation, t.transform.rotation, static)
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(TFMessage,'/tf_static',lambda m: cb(m,True),qos)
n.create_subscription(TFMessage,'/tf',lambda m: cb(m,False),100)
t0=time.time()
while time.time()-t0<4: rclpy.spin_once(n,timeout_sec=0.2)
for k,(tr,q,s) in sorted(got.items()):
    print(('S ' if s else 'D '),k, f"t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
timeout 30 python3 tfdump.py

# openrua op 8
cd /workspace; for p in "265 315" "555 225" "50 210" "250 140" "490 335" "450 130" "320 240"; do echo -n "$p -> "; timeout 40 python3 tools/perception/px2world.py robot0_eye_in_hand $p; done

# openrua op 9
cd /workspace; timeout 30 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png; python3 -c "
import numpy as np
d=np.load('eih_depth.npy')
print(d.shape, np.nanmin(d), np.nanmax(d))
print('table center', d[240,320])
print('butter region rows 280-350 cols 240-290:'); print(np.round(d[280:355:8,240:295:8],3))
print('cream cheese rows 180-270 cols 520-590:'); print(np.round(d[180:275:10,520:595:10],3))
print('basket rows 100-320 cols 0-130:'); print(np.round(d[100:330:20,0:135:15],3))
"

# openrua op 10
cd /workspace; for p in "385 290" "365 270" "405 310" "290 300" "265 283" "320 265"; do echo -n "bird $p -> "; timeout 40 python3 tools/perception/px2world.py birdview $p; done

# openrua op 11
cd /workspace; timeout 30 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null; python3 -c "
import numpy as np
d=np.load('bird_depth.npy')
# birdview: cam at z=3.0 looking straight down; world z = 3.0 - depth
z=3.0-d
fx=fy=None
import yaml
# basket region in pixels approx cols 350-420 rows 255-325
sub=z[250:330,345:425]
mask=(sub>0.55)&(sub<0.65)
ys,xs=np.nonzero(mask)
print('rim pixel count',mask.sum(),'centroid px',xs.mean()+345, ys.mean()+250, 'bbox', xs.min()+345, xs.max()+345, ys.min()+250, ys.max()+250)
inner=(sub>0.43)&(sub<0.55)
ys,xs=np.nonzero(inner); print('inner centroid px', xs.mean()+345, ys.mean()+250, 'z range', sub[inner].min(), sub[inner].max())
"


# openrua op 12
cd /workspace; timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A3 "^k:"; python3 -c "
# birdview optical frame: t=(-0.2,0,3.0), q=(0.7071,0.7071,0,0): R maps cam x->world x, cam y->world -y? compute
import numpy as np
x,y,z,w=0.7071,0.7071,0,0
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print(R.round(3))
"

# openrua op 13
cd /workspace; cat > objloc.py <<'EOF'
import numpy as np, sys
d=np.load('eih_depth.npy')
fx=fy=312.77408948188935; cx=320; cy=240
# world<-cam for robot0_eye_in_hand_optical_frame (from TF dump)
t=np.array([-0.0030,0.0,0.7753]); x,y,z,w=0.7070,-0.7066,-0.0201,0.0201
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("R=\n",R.round(3))
vs,us=np.mgrid[0:480,0:640]
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1)
W=P@R.T+t
def report(name,r0,r1,c0,c1):
    sub=W[r0:r1,c0:c1]; zz=sub[...,2]
    m=(zz>0.435)&(zz<0.48)
    pts=sub[m]
    c=pts.mean(0)
    xy=pts[:,:2]-c[:2]
    ev,evec=np.linalg.eigh(xy.T@xy/len(xy))
    ax=evec[:,1]; yaw=np.arctan2(ax[1],ax[0])
    ext_long=xy@ax; ext_short=xy@evec[:,0]
    print(f"{name}: n={m.sum()} center={c.round(4)} top_z={zz[m].max():.4f} long_axis={ax.round(3)} yaw={np.degrees(yaw):.1f}deg len={ext_long.max()-ext_long.min():.3f} wid={ext_short.max()-ext_short.min():.3f}")
report('butter',270,360,235,300)
report('creamcheese',175,280,515,600)
EOF
python3 objloc.py

# openrua op 14
cd /workspace; cat > fk.py <<'EOF'
import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('fk')
b=Buffer(); TransformListener(b,n)
t0=time.time()
while time.time()-t0<5 and not (b.can_transform('world','panda_hand',rclpy.time.Time()) and b.can_transform('panda_link0','panda_hand',rclpy.time.Time())):
    rclpy.spin_once(n,timeout_sec=0.2)
for parent in ['world','panda_link0']:
    t=b.lookup_transform(parent,'panda_hand',rclpy.time.Time())
    tr=t.transform.translation; q=t.transform.rotation
    print(f"{parent}->panda_hand t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
timeout 30 python3 fk.py

# openrua op 15
cd /workspace; cat > iktest.py <<'EOF'
import rclpy, sys
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node('iktest')
js={}
n.create_subscription(JointState,'/joint_states',lambda m: js.setdefault('m',m),1)
while 'm' not in js: rclpy.spin_once(n,timeout_sec=0.2)
cli=n.create_client(GetPositionIK,'/compute_ik'); cli.wait_for_service(10)
def ik(x,y,z,qx,qy,qz,qw,frame=''):
    req=GetPositionIK.Request(); r=req.ik_request
    r.group_name='panda_arm'; r.pose_stamped.header.frame_id=frame
    p=r.pose_stamped.pose; p.position.x,p.position.y,p.position.z=x,y,z
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=qx,qy,qz,qw
    seed=JointState()
    for a,b in zip(js['m'].name,js['m'].position):
        if a.startswith('panda_joint'): seed.name.append(a); seed.position.append(b)
    r.robot_state.joint_state=seed
    f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=60)
    res=f.result()
    if res is None: print('timeout'); return
    print('code',res.error_code.val, [f"{v:.3f}" for k,v in zip(res.solution.joint_state.name,res.solution.joint_state.position) if k.startswith('panda_joint')])
print('current', [f"{v:.3f}" for k,v in zip(js['m'].name,js['m'].position) if k.startswith('panda_joint')])
print('base-frame coords:'); ik(0.4570,0.0,0.3576,0.9996,0,-0.0284,0)
print('world-frame coords:'); ik(-0.0530,0.0,0.7776,0.9996,0,-0.0284,0)
EOF
timeout 120 python3 iktest.py

# openrua op 16
cd /workspace; sed -i 's/p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=qx,qy,qz,qw/p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=float(qx),float(qy),float(qz),float(qw)/; s/p.position.x,p.position.y,p.position.z=x,y,z/p.position.x,p.position.y,p.position.z=float(x),float(y),float(z)/' iktest.py; timeout 120 python3 iktest.py

# openrua op 17
cd /workspace; timeout 10 ros2 service list | grep -i -E "fk|ik|kinematic"; cat > fktest.py <<'EOF'
import rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node('fktest')
js={}
n.create_subscription(JointState,'/joint_states',lambda m: js.setdefault('m',m),1)
while 'm' not in js: rclpy.spin_once(n,timeout_sec=0.2)
cli=n.create_client(GetPositionFK,'/compute_fk'); print('svc',cli.wait_for_service(10))
req=GetPositionFK.Request()
req.header.frame_id=''
req.fk_link_names=['panda_link8','panda_hand','panda_link0']
seed=JointState()
for a,b in zip(js['m'].name,js['m'].position):
    if a.startswith('panda_joint'): seed.name.append(a); seed.position.append(b)
req.robot_state.joint_state=seed
f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=60)
res=f.result()
print('code',res.error_code.val)
for name,ps in zip(res.fk_link_names,res.pose_stamped):
    p=ps.pose.position;q=ps.pose.orientation
    print(name, ps.header.frame_id, f"({p.x:.4f},{p.y:.4f},{p.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
timeout 90 python3 fktest.py

# openrua op 18
mkdir -p "$(dirname /workspace/pp.py)"
cat > /workspace/pp.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick-and-place helper for this Panda (world-frame, hand pointing down).

Usage:
  python3 pp.py pose                      # report TCP pose + finger gap
  python3 pp.py move <x> <y> <z> [secs]   # TCP (fingertip centre) to world xyz,
                                          # hand down, fingers along world Y
  python3 pp.py grip <per_finger_m>       # 0.04 open, 0.0 close
  python3 pp.py seq <step>;<step>;...     # several of the above in one process

Facts established on this machine (see notes): /compute_ik solves for
panda_link8 in the WORLD frame; panda_link8 = panda_hand * rot_z(+45deg);
TCP is tcp_offset_m along hand +Z (which points down when grasping).
"""
import sys, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open('/workspace/machine.yaml'))
FJT = next(a for a in M['actuators'] if a['kind'] == 'joint_trajectory')
GRIP = next(a for a in M['actuators'] if a['kind'] == 'gripper')
JOINTS = FJT['joints']
TCP = float(M['hand']['tcp_offset_m'])
# link8 orientation for hand pointing down with fingers along world Y
Q_DOWN = (0.9238795, -0.3826834, 0.0, 0.0)


class PP:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node('pp')
        self.js = None
        self.n.create_subscription(JointState, '/joint_states', self._js, 1)
        self.ik = self.n.create_client(GetPositionIK, M['planning']['ik_service'])
        self.fk = self.n.create_client(GetPositionFK, '/compute_fk')
        self.fjt = ActionClient(self.n, FollowJointTrajectory, FJT['port'])
        self.gr = ActionClient(self.n, GripperCommand, GRIP['port'])
        assert self.ik.wait_for_service(20) and self.fk.wait_for_service(20)
        assert self.fjt.wait_for_server(20) and self.gr.wait_for_server(20)
        self.fresh_js()

    def _js(self, m):
        self.js = m

    def fresh_js(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self.n, timeout_sec=0.2)
        return dict(zip(self.js.name, self.js.position))

    def arm_seed(self):
        d = self.fresh_js()
        s = JointState()
        for j in JOINTS:
            s.name.append(j); s.position.append(float(d[j]))
        return s

    def pose(self):
        d = self.fresh_js()
        req = GetPositionFK.Request()
        req.fk_link_names = ['panda_link8']
        req.robot_state.joint_state = self.arm_seed()
        f = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=60)
        p = f.result().pose_stamped[0].pose
        q = p.orientation
        R = quat_R(q.x, q.y, q.z, q.w)
        tcp = np.array([p.position.x, p.position.y, p.position.z]) + TCP * R[:, 2]
        fingers = (d['panda_finger_joint1'], d['panda_finger_joint2'])
        print(f"TCP world=({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}) "
              f"link8 q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f}) "
              f"fingers=({fingers[0]:.4f},{fingers[1]:.4f}) "
              f"joints={[round(d[j],3) for j in JOINTS]}")
        return tcp

    def move(self, x, y, z, secs=3.0):
        R = quat_R(*Q_DOWN)
        tgt = np.array([x, y, z]) - TCP * R[:, 2]  # link8 origin
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = M['planning']['group']
        r.pose_stamped.header.frame_id = ''
        p = r.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, tgt)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = Q_DOWN
        r.robot_state.joint_state = self.arm_seed()
        r.avoid_collisions = False
        f = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=60)
        res = f.result()
        if res is None or res.error_code.val != 1:
            print(f"IK FAILED for TCP ({x},{y},{z}): code="
                  f"{None if res is None else res.error_code.val}; no motion")
            return False
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        pos = [float(sol[j]) for j in JOINTS]
        print(f"IK ok -> {[round(v,3) for v in pos]}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=pos)
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        t0 = time.time()
        sf = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, sf)
        rf = sf.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf)
        code = rf.result().result.error_code
        print(f"FJT done error_code={code} ({time.time()-t0:.1f}s wall)")
        d = self.fresh_js()
        err = max(abs(d[j] - v) for j, v in zip(JOINTS, pos))
        print(f"max joint err vs target: {err:.4f} rad")
        tcp = self.pose()
        print(f"TCP err vs target: {np.linalg.norm(tcp - np.array([x,y,z])):.4f} m")
        return code == 0

    def grip(self, w):
        goal = GripperCommand.Goal()
        goal.command.position = float(w)
        goal.command.max_effort = float(GRIP.get('max_effort', 30.0))
        sf = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, sf, timeout_sec=60)
        rf = sf.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=180)
        res = rf.result().result
        d = self.fresh_js()
        print(f"gripper -> {w}: reached_goal={res.reached_goal} stalled={res.stalled} "
              f"fingers=({d['panda_finger_joint1']:.4f},{d['panda_finger_joint2']:.4f})")

    def run(self, step):
        a = step.split()
        if a[0] == 'pose':
            self.pose()
        elif a[0] == 'move':
            ok = self.move(*map(float, a[1:4]), *(map(float, a[4:5])))
            if not ok:
                raise SystemExit('move failed; stopping sequence')
        elif a[0] == 'grip':
            self.grip(float(a[1]))
        else:
            raise SystemExit(f'unknown step {step}')


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    pp = PP()
    steps = [' '.join(sys.argv[1:])] if sys.argv[1] != 'seq' else \
        [s.strip() for s in ' '.join(sys.argv[2:]).split(';') if s.strip()]
    for s in steps:
        print(f"== {s}", flush=True)
        pp.run(s)
    print("SEQUENCE COMPLETE", flush=True)
OPENRUA_EOF

# openrua op 19
cd /workspace; timeout 120 python3 -u pp.py pose

# openrua op 20
cd /workspace; nohup python3 -u pp.py seq "grip 0.04; move -0.0992 0.055 0.56 3; move -0.0992 0.055 0.440 2.5; grip 0.0; move -0.0992 0.055 0.62 2.5" > pick1.log 2>&1 &
echo started; sleep 45; cat pick1.log

# openrua op 21
cd /workspace; timeout 30 python3 tools/perception/cam_snap.py agentview after_pick1.png >/dev/null; timeout 30 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_pick1.png >/dev/null; python3 -c "
import cv2
a=cv2.imread('after_pick1.png'); b=cv2.imread('eih_pick1.png')
cv2.imwrite('pick1_combo.png', cv2.hconcat([a,b]))"

# openrua op 22
cd /workspace; nohup python3 -u pp.py seq "move 0.005 0.27 0.68 3; move 0.005 0.27 0.65 1.5; grip 0.04; move 0.005 0.27 0.72 2" > place1.log 2>&1 &
sleep 50; cat place1.log

# openrua op 23
cd /workspace; timeout 30 python3 tools/perception/cam_snap.py agentview after_place1.png >/dev/null; timeout 30 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_place1.png >/dev/null; python3 -c "
import cv2
a=cv2.imread('after_place1.png'); b=cv2.imread('eih_place1.png')
cv2.imwrite('place1_combo.png', cv2.hconcat([a,b]))"

# openrua op 24
cd /workspace; nohup python3 -u pp.py seq "move -0.0049 -0.2371 0.56 3.5; move -0.0049 -0.2371 0.442 2.5; grip 0.0; move -0.0049 -0.2371 0.62 2.5" > pick2.log 2>&1 &
sleep 50; cat pick2.log

# openrua op 25
cd /workspace; nohup python3 -u pp.py seq "move -0.0049 -0.2371 0.56 2.5; move -0.0049 -0.2371 0.442 2.5; grip 0.0; move -0.0049 -0.2371 0.62 2.5" > pick2b.log 2>&1 &
sleep 55; cat pick2b.log

# openrua op 26
cd /workspace; timeout 30 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_pick2.png

# openrua op 27
cd /workspace; nohup python3 -u pp.py seq "move 0.005 0.27 0.68 4.5; move 0.005 0.27 0.65 1.5; grip 0.04; move 0.005 0.27 0.72 2" > place2.log 2>&1 &
sleep 55; cat place2.log

# openrua op 28
cd /workspace; nohup python3 -u pp.py seq "move 0.005 0.27 0.65 2; grip 0.04; move 0.005 0.27 0.72 2" > place2b.log 2>&1 &
sleep 40; cat place2b.log

# openrua op 29
cd /workspace; for c in robot0_eye_in_hand agentview birdview; do timeout 30 python3 tools/perception/cam_snap.py $c final_$c.png >/dev/null; done; python3 -c "
import cv2
a=cv2.imread('final_agentview.png'); b=cv2.imread('final_robot0_eye_in_hand.png')
cv2.imwrite('final_combo.png', cv2.hconcat([a,b]))
bv=cv2.imread('final_birdview.png'); crop=bv[240:340,330:440]; cv2.imwrite('final_bird_crop.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 30
cd /workspace; timeout 30 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw final_eih_depth.png >/dev/null; timeout 30 python3 fk.py | head -1; python3 -c "
import numpy as np
d=np.load('final_eih_depth.npy')
# camera is ~ at TCP z+0.1034+? ; report depths in basket interior region (cols 240-400, rows 220-380)
sub=d[220:380,240:400]
print('basket interior depth: min %.3f  median %.3f  max %.3f'%(sub.min(),np.median(sub),sub.max()))
# cream cheese top region approx cols 275-345 rows 255-330 in the 640x480 image
cc=d[255:330,275:345]; print('cream cheese top depth median %.3f'%np.median(cc))
floor=d[340:375,260:380]; print('basket floor-ish depth median %.3f'%np.median(floor))
"

# openrua op 31
cd /workspace; timeout 300 python3 -u tools/action/fjt_send.py 0,-0.161,0,-2.445,0,2.227,0.785 4; timeout 20 ros2 topic echo /joint_states --once | grep -A7 "^position" 

# openrua op 32
cd /workspace; for c in birdview agentview sideview; do timeout 30 python3 tools/perception/cam_snap.py $c final2_$c.png >/dev/null; done; timeout 30 python3 tools/perception/cam_snap.py /birdview/depth/image_raw final2_bird_depth.png >/dev/null; python3 -c "
import cv2, numpy as np
bv=cv2.imread('final2_birdview.png'); crop=bv[250:330,340:430]; cv2.imwrite('final2_bird_crop.png', cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
d=np.load('final2_bird_depth.npy'); z=3.0-d
sub=z[265:312,360:410]
np.set_printoptions(linewidth=200)
print('basket region world z (rows 265-312 step 3, cols 360-410 step 3):'); print(np.round(sub[::3,::3],2))
"

# openrua op 33
cd /workspace; nohup python3 -u pp.py seq "move 0.005 0.27 0.78 4" > look.log 2>&1 & sleep 25; cat look.log; timeout 30 python3 tools/perception/cam_snap.py robot0_eye_in_hand look_eih.png; timeout 30 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw look_eih_depth.png

# openrua op 34
cd /workspace; python3 -c "
import cv2, numpy as np
im=cv2.imread('look_eih.png'); crop=im[210:380,225:420]; cv2.imwrite('look_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
d=np.load('look_eih_depth.npy')
# camera z approx TCP z + 0.1034 (~0.883)
camz=0.7798+0.1034-0.0023
z=camz-d
np.set_printoptions(linewidth=220)
print('world z inside basket rows 230-360 step 8, cols 245-400 step 8'); print(np.round(z[230:365:8,245:405:8],2))
"

# openrua op 35
cd /workspace; python3 -c "
import numpy as np
d0=np.load('bird_depth.npy'); z0=3.0-d0
d1=np.load('final2_bird_depth.npy'); z1=3.0-d1
# basket interior in birdview: rows ~272-305, cols ~367-403 (inside rim)
for name,z in (('before',z0),('after',z1)):
    sub=z[274:304,369:401]
    h,e=np.histogram(sub,bins=np.arange(0.40,0.62,0.01))
    print(name, ' '.join(f'{a:.2f}:{b}' for a,b in zip(e[:-1],h) if b))
np.set_printoptions(linewidth=250)
print('before interior grid'); print(np.round(z0[272:308:3,366:404:3],2))
"

# openrua op 36
cd /workspace; python3 -c "
import numpy as np
d=np.load('look_eih_depth.npy'); camz=0.7798+0.1034-0.0023; z=camz-d
np.set_printoptions(linewidth=250)
for r in (265,285,300,315):
    print('row',r, ' '.join(f'{v:.3f}' for v in z[r,255:395:5]))
print('vertical col 335 rows 220-370:', ' '.join(f'{v:.3f}' for v in z[220:370:5,335]))
print('vertical col 290 rows 220-370:', ' '.join(f'{v:.3f}' for v in z[220:370:5,290]))
"

# openrua op 37
cd /workspace; python3 -c "
import numpy as np
d=np.load('look_eih_depth.npy'); camz=0.7798+0.1034-0.0023; z=camz-d
# tomato can top near (500,235) in look_eih
sub=z[215:255,480:520]; print('tomato can top now: max %.3f  median %.3f'%(sub.max(), np.median(sub)))
# table now
print('table now near basket:', np.round(z[150,300],3), np.round(z[420,300],3))
d0=np.load('bird_depth.npy'); z0=3.0-d0
print('tomato can top initially (birdview):', np.round(z0[296:304,286:294].max(),3))
d1=np.load('final2_bird_depth.npy'); z1=3.0-d1
print('tomato can top now (birdview):', np.round(z1[290:310,280:300].max(),3))
print('table (birdview) before/after:', np.round(z0[200,320],3), np.round(z1[200,320],3))
# other objects: cream cheese initial top 0.4547 vs table 0.4249
"

# openrua op 38
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pp.py", "old_string": "    def move(self, x, y, z, secs=3.0):\\n        R = quat_R(*Q_DOWN)\\n        tgt = np.array([x, y, z]) - TCP * R[:, 2]  # link8 origin\\n        req = GetPositionIK.Request(); r = req.ik_request\\n        r.group_name = M[\'planning\'][\'group\']\\n        r.pose_stamped.header.frame_id = \'\'\\n        p = r.pose_stamped.pose\\n        p.position.x, p.position.y, p.position.z = map(float, tgt)\\n        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = Q_DOWN", "new_string": "    def move(self, x, y, z, secs=3.0, q8=Q_DOWN):\\n        R = quat_R(*q8)\\n        tgt = np.array([x, y, z]) - TCP * R[:, 2]  # link8 origin\\n        req = GetPositionIK.Request(); r = req.ik_request\\n        r.group_name = M[\'planning\'][\'group\']\\n        r.pose_stamped.header.frame_id = \'\'\\n        p = r.pose_stamped.pose\\n        p.position.x, p.position.y, p.position.z = map(float, tgt)\\n        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 39
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pp.py", "old_string": "        elif a[0] == \'grip\':", "new_string": "        elif a[0] == \'movq\':  # movq x y z hqx hqy hqz hqw secs  (HAND quaternion)\\n            x, y, z, hx, hy, hz, hw, secs = map(float, a[1:9])\\n            # link8 = hand * rot_z(+45deg)\\n            q8 = quat_mul((hx, hy, hz, hw), (0.0, 0.0, 0.3826834, 0.9238795))\\n            if not self.move(x, y, z, secs, q8=q8):\\n                raise SystemExit(\'movq failed; stopping sequence\')\\n        elif a[0] == \'grip\':", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 40
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pp.py", "old_string": "def quat_R(x, y, z, w):", "new_string": "def quat_mul(a, b):\\n    \\"\\"\\"Hamilton product a*b, quaternions as (x, y, z, w).\\"\\"\\"\\n    ax, ay, az, aw = a; bx, by, bz, bw = b\\n    return (aw * bx + ax * bw + ay * bz - az * by,\\n            aw * by - ax * bz + ay * bw + az * bx,\\n            aw * bz + ax * by - ay * bx + az * bw,\\n            aw * bw - ax * bx - ay * by - az * bz)\\n\\n\\ndef quat_R(x, y, z, w):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 41
cd /workspace; nohup python3 -u pp.py seq "movq -0.045 0.189 0.64 0.9763 0 0 -0.2164 4" > look2.log 2>&1 & sleep 28; cat look2.log; timeout 30 python3 tools/perception/cam_snap.py robot0_eye_in_hand look2_eih.png

# openrua op 42
cd /workspace; python3 -c "
import cv2
im=cv2.imread('look2_eih.png'); crop=im[180:310,250:380]; cv2.imwrite('look2_crop.png', cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))"

# openrua op 43
cd /workspace; nohup python3 -u pp.py seq "movq -0.132 0.27 0.716 0.9537 0 0.3007 0 4" > look3.log 2>&1 & sleep 28; cat look3.log; timeout 30 python3 tools/perception/cam_snap.py robot0_eye_in_hand look3_eih.png

# openrua op 44
cd /workspace; nohup python3 -u pp.py seq "movq -0.132 0.27 0.716 0.9537 0 0.3007 0 3" > look3b.log 2>&1 & sleep 25; cat look3b.log; timeout 30 python3 tools/perception/cam_snap.py robot0_eye_in_hand look3_eih.png

# openrua op 45
cd /workspace; python3 -c "
import cv2
im=cv2.imread('look3_eih.png'); crop=im[250:350,270:380]; cv2.imwrite('look3_crop.png', cv2.resize(crop,None,fx=7,fy=7,interpolation=cv2.INTER_CUBIC))"

# openrua op 46
cd /workspace; python3 -c "
import cv2
im=cv2.imread('look2_eih.png'); crop=im[195:295,300:365]; cv2.imwrite('look2_crop2.png', cv2.resize(crop,None,fx=8,fy=8,interpolation=cv2.INTER_CUBIC))
print(im[230:270:8, 334:346:3].tolist())"

# openrua op 47
cd /workspace; timeout 300 python3 -u tools/action/fjt_send.py 0,-0.161,0,-2.445,0,2.227,0.785 4; timeout 30 python3 -u pp.py pose; pgrep -af "pp.py|fjt_send|cam_snap" || echo "no background robot processes"

# openrua op 48
cd /workspace; timeout 300 python3 -u tools/action/fjt_send.py 0,-0.161,0,-2.445,0,2.227,0.785 3; timeout 30 python3 -u pp.py pose; timeout 30 python3 tools/perception/cam_snap.py agentview end_agentview.png >/dev/null; pgrep -f "pp.py|fjt_send" >/dev/null && echo "still running" || echo "no background robot processes"

# openrua op 49
pgrep -af "pp.py|fjt_send" | grep -v pgrep | grep -v "bin/bash -c" || echo "no background robot processes"
