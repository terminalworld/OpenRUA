#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | head -50; echo ---; timeout 30 ros2 node list

# openrua op 2
timeout 60 ros2 topic echo /joint_states --once 2>&1 | head -40; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 3
python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[160:420,70:400]
cv2.imwrite('crop_agent.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 4
timeout 60 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40; echo ---; timeout 60 ros2 topic echo /agentview/color/camera_info --once 2>&1 | head -20

# openrua op 5
cat > /workspace/tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node=rclpy.create_node('tfdump')
seen={}
def cb(msg):
    for t in msg.transforms:
        seen[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage,'/tf_static',cb,qos)
node.create_subscription(TFMessage,'/tf',cb,10)
import time
for _ in range(30): rclpy.spin_once(node,timeout_sec=0.2)
for k,v in sorted(seen.items()):
    print(k, 'xyz=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)'%(v.translation.x,v.translation.y,v.translation.z,v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w))
EOF
timeout 60 python3 /workspace/tfdump.py

# openrua op 6
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png && cat > /workspace/cloud.py <<'EOF'
import numpy as np, cv2, sys
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
CAMS={'birdview':((-0.2,0,3.0),(0.7071,0.7071,0,0),579.4112549695428),
      'agentview':((0.6066,0,0.96),(0.6182,0.6182,-0.3432,-0.3432),579.4112549695428)}
def cloud(cam):
    d=np.load(f'{cam}_depth.npy'); t,q,f=CAMS[cam]
    h,w=d.shape; v,u=np.mgrid[0:h,0:w]
    X=(u-320)*d/f; Y=(v-240)*d/f
    P=np.stack([X,Y,d],-1)@quat_R(*q).T+np.array(t)
    return P
if __name__=='__main__':
    P=cloud('birdview')
    z=P[...,2]
    print('table z candidates (mode):', np.round(np.median(z[200:330,230:420]),3))
    # find objects: z above table
    tz=np.median(z[200:330,230:420])
    mask=(z>tz+0.01)&(z<tz+0.4)
    img=cv2.imread('birdview.png')
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for i in range(1,n):
        if stats[i,4]<10: continue
        m=lab==i
        pts=P[m]
        print(f'comp {i}: px centroid ({cent[i][0]:.0f},{cent[i][1]:.0f}) area {stats[i,4]} world xy min/max x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} zmed {np.median(pts[:,2]):.3f}')
EOF
timeout 60 python3 /workspace/cloud.py

# openrua op 7
cat > /workspace/probe.py <<'EOF'
from cloud import cloud
import numpy as np
P=cloud('agentview')
pts={'ketchup':(232,215),'bluecan':(215,275),'tomato_can':(338,270),'oj':(115,310),'milk':(245,320),'cream_cheese':(165,385),'butter':(365,370),'basket_rim':(530,190),'table_near':(320,440),'table_far':(400,200)}
for k,(u,v) in pts.items():
    p=P[v,u]; print(f'{k:14s} px({u},{v}) world {p.round(3)}')
EOF
timeout 60 python3 /workspace/probe.py

# openrua op 8
cat > /workspace/measure.py <<'EOF'
from cloud import cloud
import numpy as np
B=cloud('birdview'); A=cloud('agentview')
def seg(P, xr, yr, zr, name):
    m=(P[...,0]>xr[0])&(P[...,0]<xr[1])&(P[...,1]>yr[0])&(P[...,1]<yr[1])&(P[...,2]>zr[0])&(P[...,2]<zr[1])
    pts=P[m]
    if len(pts)==0: print(name,'none'); return
    c=pts[:,:2].mean(0)
    cov=np.cov((pts[:,:2]-c).T); w,v=np.linalg.eigh(cov)
    ax=v[:,1]; ang=np.degrees(np.arctan2(ax[1],ax[0]))
    proj=(pts[:,:2]-c)@v
    print(f'{name}: n={len(pts)} center=({c[0]:.3f},{c[1]:.3f}) ztop={pts[:,2].max():.3f} zmin={pts[:,2].min():.3f} long-axis angle={ang:.1f} deg  extents long={proj[:,1].ptp():.3f} short={proj[:,0].ptp():.3f} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}]')
seg(B,(0.0,0.2),(-0.3,-0.1),(0.44,0.5),'cream_cheese birdview')
seg(A,(0.0,0.2),(-0.3,-0.1),(0.44,0.5),'cream_cheese agentview')
seg(A,(-0.15,0.02),(-0.04,0.1),(0.48,0.53),'tomato can top agentview')
seg(A,(-0.15,0.02),(-0.04,0.1),(0.43,0.53),'tomato can all agentview')
seg(B,(-0.15,0.02),(-0.04,0.1),(0.43,0.6),'tomato can birdview')
seg(B,(-0.12,0.12),(0.12,0.4),(0.5,0.7),'basket birdview')
EOF
timeout 60 python3 /workspace/measure.py

# openrua op 9
cat >> /workspace/measure.py <<'EOF'
print('--- refined')
seg(B,(0.03,0.17),(-0.24,-0.14),(0.44,0.47),'cream_cheese birdview')
seg(A,(0.03,0.17),(-0.24,-0.14),(0.44,0.47),'cream_cheese agentview')
seg(A,(-0.15,0.02),(-0.04,0.1),(0.515,0.53),'tomato can topface agentview')
EOF
timeout 60 python3 /workspace/measure.py 2>&1 | sed -n '/refined/,$p'

# openrua op 10
python3 -c "
import cv2, numpy as np
from cloud import cloud
B=cloud('birdview')
im=cv2.imread('birdview.png')
m=(B[...,2]>0.44)&(B[...,2]<0.47)&(B[...,0]>0.0)&(B[...,0]<0.2)&(B[...,1]>-0.3)&(B[...,1]<-0.1)
ov=im.copy(); ov[m]=(0,255,0)
crop=np.hstack([im[280:340,240:320],ov[280:340,240:320]])
cv2.imwrite('cc_bird.png',cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_NEAREST))
d=B[...,2]
print(np.round(d[300:320:2,260:300:2]*1000).astype(int))
"

# openrua op 11
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Persistent helper: IK + trajectory + gripper + joint-state reads.

World->base offset from TF (world -> panda_link0). Poses passed in WORLD
coordinates of the TCP (fingertip centre); converted to the planner's
base frame here.
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

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (from /tf_static)
TOPDOWN = (1.0, 0.0, 0.0, 0.0)       # hand z down, fingers close along world y


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_quat(yaw):
    """Top-down grasp rotated by yaw about world z (yaw=0 -> fingers along y)."""
    # q = Rz(yaw) * (1,0,0,0)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # (c,0,0,s)*(1,0,0,0): w=c*0 - s*0..., do generic multiply
    w1, x1, y1, z1 = c, 0.0, 0.0, s
    w2, x2, y2, z2 = 0.0, 1.0, 0.0, 0.0
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"
        assert self.traj.wait_for_server(10), "no traj server"
        assert self.grip.wait_for_server(10), "no gripper server"
        self.joints()

    def _js(self, m):
        self.js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        self.js = None
        while self.js is None:
            self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def hand_pose(self):
        """FK of panda_hand in base frame -> world coords."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = self.arm_q()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP * quat_R(*q)[:, 2]
        return pos, tcp, q

    def solve_ik(self, tcp_world, quat=TOPDOWN, seed=None):
        R = quat_R(*quat)
        hand_world = np.array(tcp_world) - TCP * R[:, 2]
        hb = hand_world - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = seed or self.arm_q()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK FAILED for tcp={np.round(tcp_world,3)} code={None if r is None else r.error_code.val}")
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, q, seconds=3.0, tries=2):
        for attempt in range(tries):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ARM
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.traj.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"  traj error_code={code} max joint err={err:.4f} rad")
            if code == 0 and err < 0.02:
                return True
        return err < 0.05

    def move_tcp(self, tcp_world, quat=TOPDOWN, seconds=3.0):
        q = self.solve_ik(tcp_world, quat)
        if q is None:
            return False
        ok = self.move_joints(q, seconds)
        _, tcp, _ = self.hand_pose()
        print(f"  TCP now {np.round(tcp,3)} (target {np.round(tcp_world,3)}) err={np.linalg.norm(tcp-tcp_world):.4f}")
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        for _ in range(5):
            self.spin(0.1)
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={np.round(f,4)}")
        return f

    def close(self):
        rclpy.shutdown()
OPENRUA_EOF

# openrua op 12
timeout 120 python3 -c "
from arm import *
a=Arm()
print('arm q', np.round(a.arm_q(),3)); print('fingers', a.fingers())
pos,tcp,q=a.hand_pose(); print('hand world', pos.round(3), 'tcp', tcp.round(3), 'quat', np.round(q,3))
for name,t in [('can pre',(-0.095,0.027,0.62)),('can grasp',(-0.095,0.027,0.47)),('cc pre',(0.104,-0.187,0.60)),('cc grasp',(0.104,-0.187,0.44)),('basket',(0.005,0.264,0.75))]:
    s=a.solve_ik(t); print(name, None if s is None else np.round(s,3))
a.close()
"

# openrua op 13
timeout 120 python3 -c "
from arm import *
a=Arm()
from moveit_msgs.srv import GetPositionFK
def fk(q):
    req=GetPositionFK.Request(); req.fk_link_names=['panda_hand','panda_link8']
    req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=q
    fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=30); r=fut.result()
    print('fk code',r.error_code.val)
    for ps in r.pose_stamped:
        p=ps.pose; print('  ',ps.header.frame_id, np.round([p.position.x,p.position.y,p.position.z],3), np.round([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],3))
print('ready pose'); fk([0,-0.785,0,-2.356,0,1.571,0.785])
print('current'); fk(a.arm_q())
# IK at current FK pose (hand), frame empty
pos,tcp,q=a.hand_pose()
print('IK at current tcp:', a.solve_ik(tcp, q))
print('IK at ready-ish pose, topdown', a.solve_ik(np.array([0.307,0,0.59-0.1034])+BASE, TOPDOWN, seed=[0,-0.785,0,-2.356,0,1.571,0.785]))
a.close()
"

# openrua op 14
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (from /tf_static)", "new_string": "# Verified: FK of the ready pose returns (-0.203, 0, 1.01) = (0.307, 0, 0.59)\\n# + world->panda_link0, so the planner\'s model frame IS world here.\\nBASE = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 15
timeout 120 python3 -c "
from arm import *
a=Arm()
pos,tcp,q=a.hand_pose(); print('hand world', pos.round(3), 'tcp', tcp.round(3))
for name,t in [('can pre',(-0.095,0.027,0.62)),('can grasp',(-0.095,0.027,0.47)),('cc pre',(0.104,-0.187,0.60)),('cc grasp',(0.104,-0.187,0.44)),('basket',(0.005,0.264,0.75))]:
    s=a.solve_ik(t); print(name, None if s is None else np.round(s,3))
a.close()
"

# openrua op 16
mkdir -p "$(dirname /workspace/pick_can.py)"
cat > /workspace/pick_can.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the tomato sauce can and place it in the basket."""
from arm import *

CAN = np.array([-0.095, 0.027])
BASKET = np.array([0.005, 0.264])
a = Arm()
print("start tcp", a.hand_pose()[1].round(3), "fingers", a.fingers())

print("open gripper"); a.gripper(GRIP["open_m"])
print("pregrasp"); a.move_tcp([*CAN, 0.62], seconds=4)
print("descend"); a.move_tcp([*CAN, 0.47], seconds=3)
print("close"); f = a.gripper(GRIP["closed_m"])
print("lift"); a.move_tcp([*CAN, 0.72], seconds=3)
print("fingers after lift", a.fingers())
print("over basket"); a.move_tcp([*BASKET, 0.76], seconds=4)
print("release"); a.gripper(GRIP["open_m"])
print("retreat"); a.move_tcp([*BASKET, 0.85], seconds=3)
print("end tcp", a.hand_pose()[1].round(3), "fingers", a.fingers())
a.close()
print("DONE")
OPENRUA_EOF

# openrua op 17
nohup python3 -u pick_can.py > pick_can.log 2>&1 &
sleep 45; cat /workspace/pick_can.log

# openrua op 18
sleep 60; cat /workspace/pick_can.log

# openrua op 19
sleep 20; cat /workspace/pick_can.log | tail -4; timeout 60 python3 tools/perception/cam_snap.py agentview after_can.png

# openrua op 20
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null && python3 -c "
from cloud import cloud
import numpy as np
B=cloud('birdview')
m=(B[...,2]>0.44)&(B[...,2]<0.47)&(B[...,0]>0.03)&(B[...,0]<0.17)&(B[...,1]>-0.24)&(B[...,1]<-0.14)
pts=B[m]; c=pts[:,:2].mean(0)
print('cream cheese n',len(pts),'center',c.round(4),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'ztop',pts[:,2].max().round(3))
# basket contents
m2=(B[...,2]>0.44)&(B[...,2]<0.62)&(B[...,0]>-0.07)&(B[...,0]<0.07)&(B[...,1]>0.18)&(B[...,1]<0.33)
p2=B[m2]; print('inside basket n',len(p2), 'ztop', p2[:,2].max().round(3) if len(p2) else None, 'center', p2[:,:2].mean(0).round(3) if len(p2) else None)
"

# openrua op 21
cat > /workspace/pick_cc.py <<'EOF'
#!/usr/bin/env python3
"""Pick the cream cheese box and place it in the basket."""
from arm import *

CC = np.array([0.103, -0.187])
BASKET = np.array([-0.01, 0.30])   # slightly off the can's drop spot
a = Arm()
print("start tcp", a.hand_pose()[1].round(3), "fingers", a.fingers())
print("open gripper"); a.gripper(GRIP["open_m"])
print("pregrasp"); a.move_tcp([*CC, 0.60], seconds=4)
print("descend"); a.move_tcp([*CC, 0.438], seconds=3)
print("close"); a.gripper(GRIP["closed_m"])
print("lift"); a.move_tcp([*CC, 0.70], seconds=3)
print("fingers after lift", a.fingers())
print("over basket"); a.move_tcp([*BASKET, 0.76], seconds=4)
print("fingers over basket", a.fingers())
print("release"); a.gripper(GRIP["open_m"])
print("retreat"); a.move_tcp([*BASKET, 0.85], seconds=3)
print("end tcp", a.hand_pose()[1].round(3), "fingers", a.fingers())
a.close()
print("DONE")
EOF
nohup python3 -u pick_cc.py > pick_cc.log 2>&1 &
sleep 90; cat /workspace/pick_cc.log

# openrua op 22
sleep 40; tail -5 /workspace/pick_cc.log; timeout 60 python3 tools/perception/cam_snap.py agentview after_cc1.png; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null

# openrua op 23
python3 -c "
from cloud import cloud
import numpy as np, cv2
B=cloud('birdview')
m=(B[...,2]>0.44)&(B[...,2]<0.50)&(B[...,0]>0.0)&(B[...,0]<0.25)&(B[...,1]>-0.30)&(B[...,1]<-0.12)
pts=B[m]; c=pts[:,:2].mean(0)
cov=np.cov((pts[:,:2]-c).T); w,v=np.linalg.eigh(cov); ax=v[:,1]
ang=np.degrees(np.arctan2(ax[1],ax[0])); proj=(pts[:,:2]-c)@v
print('n',len(pts),'center',c.round(4),'ztop',pts[:,2].max().round(3),'zmin',pts[:,2].min().round(3),'long axis angle from +x: %.1f deg'%ang,'long %.3f short %.3f'%(np.ptp(proj[:,1]),np.ptp(proj[:,0])))
im=cv2.imread('birdview.png'); ov=im.copy(); ov[m]=(0,255,0)
cv2.imwrite('cc_bird2.png',cv2.resize(np.hstack([im[280:340,240:320],ov[280:340,240:320]]),None,fx=6,fy=6,interpolation=cv2.INTER_NEAREST))
"

# openrua op 24
python3 -c "
from cloud import cloud
import numpy as np
B=cloud('birdview')
m=(B[...,2]>0.44)&(B[...,2]<0.47)&(B[...,0]>0.05)&(B[...,0]<0.20)&(B[...,1]>-0.25)&(B[...,1]<-0.14)
pts=B[m]; c=pts[:,:2].mean(0)
cov=np.cov((pts[:,:2]-c).T); w,v=np.linalg.eigh(cov); ax=v[:,1]
ang=np.degrees(np.arctan2(ax[1],ax[0])); proj=(pts[:,:2]-c)@v
print('n',len(pts),'center',c.round(4),'ztop',pts[:,2].max().round(3),'long axis angle from +x: %.1f deg'%ang,'long %.3f short %.3f'%(np.ptp(proj[:,1]),np.ptp(proj[:,0])))
print('x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3))
"

# openrua op 25
python3 - <<'EOF'
import re
s=open('arm.py').read()
s=s.replace("""            if code == 0 and err < 0.02:
                return True
        return err < 0.05""","""            if err < 0.006:
                return True
        return err < 0.02""")
s=s.replace("def move_joints(self, q, seconds=3.0, tries=2):","def move_joints(self, q, seconds=3.0, tries=3):")
open('arm.py','w').write(s)
EOF
cat > /workspace/pick_cc2.py <<'EOF'
#!/usr/bin/env python3
from arm import *
CC = np.array([0.1096, -0.192]); Q = yaw_quat(np.radians(3.8))
BASKET = np.array([-0.01, 0.30])
a = Arm()
print("open"); a.gripper(GRIP["open_m"])
print("pregrasp"); a.move_tcp([*CC, 0.58], Q, seconds=4)
print("descend"); a.move_tcp([*CC, 0.435], Q, seconds=3)
print("close"); a.gripper(GRIP["closed_m"])
print("lift"); a.move_tcp([*CC, 0.70], Q, seconds=3)
f = a.fingers(); print("fingers after lift", f)
if f[0] < 0.01:
    print("GRASP FAILED - stopping before basket"); a.close(); raise SystemExit(1)
print("over basket"); a.move_tcp([*BASKET, 0.76], seconds=4)
print("fingers over basket", a.fingers())
print("release"); a.gripper(GRIP["open_m"])
print("retreat"); a.move_tcp([*BASKET, 0.85], seconds=3)
print("end tcp", a.hand_pose()[1].round(3), "fingers", a.fingers())
a.close(); print("DONE")
EOF
nohup python3 -u pick_cc2.py > pick_cc2.log 2>&1 &
sleep 100; cat /workspace/pick_cc2.log

# openrua op 26
sleep 45; tail -8 /workspace/pick_cc2.log

# openrua op 27
timeout 60 python3 tools/perception/cam_snap.py agentview final_agent.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview final_bird.png >/dev/null; python3 -c "
from cloud import cloud
import numpy as np, cv2
B=cloud('birdview')
# anything left at the original object spots?
for name,(x,y) in {'can spot':(-0.095,0.027),'cc spot':(0.110,-0.192)}.items():
    m=(np.abs(B[...,0]-x)<0.06)&(np.abs(B[...,1]-y)<0.05)&(B[...,2]>0.44)&(B[...,2]<0.6)
    print(name,'points above table:',m.sum())
# basket interior
m=(B[...,2]>0.43)&(B[...,2]<0.62)&(np.abs(B[...,0]-0.005)<0.065)&(B[...,1]>0.20)&(B[...,1]<0.33)
p=B[m]; print('basket interior points above floor:',len(p),'ztop',p[:,2].max().round(3))
im=cv2.imread('final_bird.png'); cv2.imwrite('final_bird_crop.png',cv2.resize(im[240:330,340:420],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"

# openrua op 28
timeout 300 python3 -c "
from arm import *
a=Arm(); a.move_tcp([-0.15,-0.05,0.85], seconds=4); a.close()
" && timeout 60 python3 tools/perception/cam_snap.py birdview final_bird2.png >/dev/null && python3 -c "
import cv2; im=cv2.imread('final_bird2.png'); cv2.imwrite('final_bird2_crop.png',cv2.resize(im[240:330,340:420],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))"
