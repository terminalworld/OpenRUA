#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | head -50; echo ---; ros2 node list

# openrua op 3
for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ros2 topic echo /joint_states --once

# openrua op 4
python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[150:420,100:420]
cv2.imwrite('crop1.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -60; echo ---; timeout 10 ros2 topic echo /agentview/color/camera_info --once

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node('tfdump')
seen={}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(TFMessage,'/tf_static',cb,qos)
n.create_subscription(TFMessage,'/tf',cb,10)
t0=time.time()
while time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.2)
for k,v in sorted(seen.items()):
    print(k, "t=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)"%(v.translation.x,v.translation.y,v.translation.z,v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w))
EOF
python3 tfdump.py

# openrua op 7
cat > cloud.py <<'EOF'
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
cam=sys.argv[1]
rclpy.init(); n=rclpy.create_node('cloud')
got={}
n.create_subscription(Image,f'/{cam}/depth/image_raw',lambda m:got.setdefault('d',m),1)
n.create_subscription(Image,f'/{cam}/color/image_raw',lambda m:got.setdefault('c',m),1)
n.create_subscription(CameraInfo,f'/{cam}/color/camera_info',lambda m:got.setdefault('i',m),1)
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(TFMessage,'/tf_static',lambda m:[got.setdefault(('tf',t.child_frame_id),t.transform) for t in m.transforms],qos)
while not all(k in got for k in ['d','c','i',('tf',cam+'_optical_frame')]): rclpy.spin_once(n,timeout_sec=0.3)
d=CvBridge().imgmsg_to_cv2(got['d'],'passthrough').astype(np.float32)
c=CvBridge().imgmsg_to_cv2(got['c'],'bgr8')
k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
t=got[('tf',cam+'_optical_frame')]
q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
H,W=d.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)
Pw=P@R.T+T
np.save(f'{cam}_xyz.npy',Pw); np.save(f'{cam}_bgr.npy',c)
print('depth range',np.nanmin(d),np.nanmax(d))
print('world z range',np.nanmin(Pw[...,2]),np.nanmax(Pw[...,2]))
# table height: median z of central region
print('median z centre', np.nanmedian(Pw[200:480,:,2]))
EOF
python3 cloud.py agentview && python3 cloud.py birdview

# openrua op 8
sleep 20; cat /tmp/claude-1000/-workspace/ea69ab09-cb2f-4e47-b554-0e8d02ebaef2/tasks/blynglhr1.output; ls *.npy

# openrua op 9
timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png; ls -la *.npy *.png

# openrua op 10
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png
cat > geo.py <<'EOF'
import numpy as np, cv2
CAMS={'agentview':((0.6066,0.0,0.9600),(0.6182,0.6182,-0.3432,-0.3432),579.4112549695428),
      'birdview':((-0.2,0.0,3.0),(0.7071,0.7071,0.0,0.0),None)}
def quatR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def cloud(cam,f):
    d=np.load(f'{cam}_depth.npy').astype(np.float64)
    T,q,_=CAMS[cam]; R=quatR(*q)
    H,W=d.shape; cx,cy=W/2,H/2
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    P=np.stack([(u-cx)*d/f,(v-cy)*d/f,d],-1)
    return P@R.T+np.array(T)
EOF
timeout 10 ros2 topic echo /birdview/color/camera_info --once | grep -A1 "^k:"

# openrua op 11
cat > analyze.py <<'EOF'
import numpy as np, cv2
from geo import cloud
f=579.4112549695428
for cam in ['agentview','birdview']:
    Pw=cloud(cam,f)
    z=Pw[...,2]
    print(cam,'z range',np.nanmin(z),np.nanmax(z))
    # histogram of z to find table height
    h,edges=np.histogram(z[np.isfinite(z)],bins=np.arange(0.3,1.2,0.01))
    top=np.argsort(h)[-5:]
    print(' most common z:',[(round(edges[i],2),h[i]) for i in top])
    np.save(f'{cam}_xyz.npy',Pw)
EOF
python3 analyze.py

# openrua op 12
cat > seg.py <<'EOF'
import numpy as np, cv2
Pw=np.load('birdview_xyz.npy'); img=cv2.imread('birdview.png')
z=Pw[...,2]
mask=((z>0.435)&(z<0.75)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<15: continue
    m=lab==i
    pts=Pw[m]
    col=img[m].mean(0)
    x0,y0,w,h=stats[i,:4]
    print(f"comp{i}: px({x0},{y0},{w}x{h}) area={m.sum()} world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) bgr={col.astype(int)}")
EOF
python3 seg.py

# openrua op 13
cat > topdown.py <<'EOF'
import numpy as np, cv2
Pw=np.load('agentview_xyz.npy'); img=cv2.imread('agentview.png')
res=0.005
x0,x1,y0,y1=-0.45,0.35,-0.45,0.45
W=int((x1-x0)/res); H=int((y1-y0)/res)
hm=np.full((H,W),np.nan); cm=np.zeros((H,W,3),np.uint8)
z=Pw[...,2]
sel=np.isfinite(z)&(z>0.43)&(z<0.8)&(Pw[...,0]>x0)&(Pw[...,0]<x1)&(Pw[...,1]>y0)&(Pw[...,1]<y1)
xs=((Pw[...,0][sel]-x0)/res).astype(int); ys=((Pw[...,1][sel]-y0)/res).astype(int)
zs=z[sel]; cs=img[sel]
order=np.argsort(zs)
for i in order:
    hm[ys[i],xs[i]]=zs[i]; cm[ys[i],xs[i]]=cs[i]
# draw: rows = y, cols = x; flip so +x down? keep: col=x, row=y
big=cv2.resize(cm,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
# grid lines every 10cm
for gx in np.arange(x0,x1+1e-9,0.1):
    c=int((gx-x0)/res)*4; cv2.line(big,(c,0),(c,big.shape[0]-1),(80,80,80),1); cv2.putText(big,f"{gx:.1f}",(c+2,12),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
for gy in np.arange(y0,y1+1e-9,0.1):
    r=int((gy-y0)/res)*4; cv2.line(big,(0,r),(big.shape[1]-1,r),(80,80,80),1); cv2.putText(big,f"{gy:.1f}",(2,r-2),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,255,255),1)
cv2.imwrite('topdown.png',big)
np.save('hm.npy',hm)
EOF
python3 topdown.py

# openrua op 14
cat > objs.py <<'EOF'
import numpy as np
Pw=np.load('agentview_xyz.npy')
def stats(name,xr,yr,zmin=0.432):
    m=(Pw[...,0]>xr[0])&(Pw[...,0]<xr[1])&(Pw[...,1]>yr[0])&(Pw[...,1]<yr[1])&(Pw[...,2]>zmin)&(Pw[...,2]<0.9)
    p=Pw[m]
    if len(p)==0: print(name,'none'); return
    print(f"{name}: n={len(p)} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] centroid=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) top-centroid=({p[p[:,2]>p[:,2].max()-0.01][:,0].mean():.3f},{p[p[:,2]>p[:,2].max()-0.01][:,1].mean():.3f})")
stats('cream_cheese',(0.03,0.18),(-0.25,-0.15))
stats('tomato_sauce',(-0.16,-0.05),(-0.02,0.09))
stats('blue_can',(-0.16,-0.05),(-0.21,-0.11))
stats('milk',(0.03,0.15),(-0.14,-0.05))
stats('butter',(0.0,0.1),(0.04,0.12))
stats('OJ',(-0.02,0.06),(-0.26,-0.18))
stats('basket',(-0.12,0.12),(0.15,0.38),0.5)
stats('ketchup',(-0.3,-0.18),(-0.2,-0.09))
EOF
python3 objs.py; echo ---; timeout 15 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12

# openrua op 15
cat >> objs.py <<'EOF'
print('--- refined')
stats('cream_cheese',(0.065,0.18),(-0.25,-0.15))
Pw2=Pw.copy()
m=(Pw[...,0]>-0.16)&(Pw[...,0]<-0.055)&(Pw[...,1]>-0.0)&(Pw[...,1]<0.1)&(Pw[...,2]>0.432)&(Pw[...,2]<0.53)
p=Pw[m]; print(f"tomato_sauce (z<0.53): n={len(p)} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] centroid=({p[:,0].mean():.3f},{p[:,1].mean():.3f})")
top=p[p[:,2]>0.49]; print(' top: n',len(top),'x[%.3f,%.3f] y[%.3f,%.3f] c=(%.3f,%.3f)'%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max(),top[:,0].mean(),top[:,1].mean()))
# cream cheese top surface
m=(Pw[...,0]>0.065)&(Pw[...,0]<0.18)&(Pw[...,1]>-0.25)&(Pw[...,1]<-0.15)&(Pw[...,2]>0.445)&(Pw[...,2]<0.5)
p=Pw[m]; print(f"cc top: n={len(p)} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] centroid=({p[:,0].mean():.3f},{p[:,1].mean():.3f})")
EOF
python3 objs.py | sed -n '/refined/,$p'

# openrua op 16
cat > ctl.py <<'EOF'
#!/usr/bin/env python3
"""Controller helpers: goto (IK->FJT), grip, pose, joints. World-frame inputs.
world->panda_link0 = (-0.51, 0, 0.42), identity rotation (from /tf_static)."""
import sys, math, time
import numpy as np, rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

ARM=[f'panda_joint{i}' for i in range(1,8)]
BASE=np.array([-0.51,0.0,0.42])
TCP=0.1034

class Ctl:
    def __init__(s):
        rclpy.init(); s.n=rclpy.create_node('ctl')
        s.js={}
        s.n.create_subscription(JointState,'/joint_states',s._js,1)
        s.fjt=ActionClient(s.n,FollowJointTrajectory,'/panda_arm_controller/follow_joint_trajectory')
        s.grp=ActionClient(s.n,GripperCommand,'/franka_gripper/gripper_action')
        s.ik=s.n.create_client(GetPositionIK,'/compute_ik')
        s.fk=s.n.create_client(GetPositionFK,'/compute_fk')
        assert s.fjt.wait_for_server(10) and s.grp.wait_for_server(10)
        assert s.ik.wait_for_service(10) and s.fk.wait_for_service(10)
    def _js(s,m):
        s.js=dict(zip(m.name,m.position))
    def joints(s):
        s.js={}
        while not s.js: rclpy.spin_once(s.n,timeout_sec=0.2)
        return dict(s.js)
    def arm_state(s):
        j=s.joints(); st=JointState(); st.name=ARM; st.position=[j[a] for a in ARM]; return st,j
    def pose(s):
        st,j=s.arm_state()
        r=GetPositionFK.Request(); r.fk_link_names=['panda_hand']; r.robot_state.joint_state=st
        f=s.fk.call_async(r); rclpy.spin_until_future_complete(s.n,f,timeout_sec=30)
        p=f.result().pose_stamped[0].pose
        w=np.array([p.position.x,p.position.y,p.position.z])+BASE
        q=(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)
        R=quatR(*q); tcp=w+TCP*R[:,2]
        return w,q,tcp,j
    def ik(s,xw,yw,zw,yaw,tcp=True):
        # hand z down; fingers along world y when yaw=0
        q=(math.cos(yaw/2),math.sin(yaw/2),0.0,0.0)
        p=np.array([xw,yw,zw])-BASE
        if tcp: p=p-TCP*quatR(*q)[:,2]
        st,_=s.arm_state()
        r=GetPositionIK.Request(); r.ik_request.group_name='panda_arm'
        r.ik_request.pose_stamped.header.frame_id=''
        pp=r.ik_request.pose_stamped.pose
        pp.position.x,pp.position.y,pp.position.z=map(float,p)
        pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=q
        r.ik_request.robot_state.joint_state=st
        r.ik_request.avoid_collisions=False
        f=s.ik.call_async(r); rclpy.spin_until_future_complete(s.n,f,timeout_sec=60)
        res=f.result()
        if res is None or res.error_code.val!=1:
            raise SystemExit(f'IK failed code={None if res is None else res.error_code.val}')
        sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
        return [sol[a] for a in ARM]
    def move(s,positions,secs):
        g=FollowJointTrajectory.Goal(); g.trajectory.joint_names=ARM
        pt=JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start=Duration(sec=int(secs),nanosec=int((secs%1)*1e9))
        g.trajectory.points=[pt]
        f=s.fjt.send_goal_async(g); rclpy.spin_until_future_complete(s.n,f)
        rf=f.result().get_result_async(); rclpy.spin_until_future_complete(s.n,rf)
        code=rf.result().result.error_code
        j=s.joints(); err=max(abs(j[a]-p) for a,p in zip(ARM,positions))
        print(f'  move done code={code} max_joint_err={err:.4f}')
        return code,err
    def goto(s,xw,yw,zw,yaw,secs=3.0):
        sol=s.ik(xw,yw,zw,yaw)
        s.move(sol,secs)
        w,q,tcp,_=s.pose()
        print(f'  now hand=({w[0]:.3f},{w[1]:.3f},{w[2]:.3f}) tcp=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f}) target_tcp=({xw:.3f},{yw:.3f},{zw:.3f})')
        return tcp
    def grip(s,width):
        g=GripperCommand.Goal(); g.command.position=float(width); g.command.max_effort=30.0
        f=s.grp.send_goal_async(g); rclpy.spin_until_future_complete(s.n,f,timeout_sec=30)
        rf=f.result().get_result_async(); rclpy.spin_until_future_complete(s.n,rf,timeout_sec=120)
        r=rf.result().result; j=s.joints()
        print(f'  grip reached={r.reached_goal} stalled={r.stalled} fingers=({j["panda_finger_joint1"]:.4f},{j["panda_finger_joint2"]:.4f})')
        return j

def quatR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

if __name__=='__main__':
    c=Ctl(); cmd=sys.argv[1]
    if cmd=='pose':
        w,q,tcp,j=c.pose(); print('hand',w.round(4),'q',np.round(q,4),'tcp',tcp.round(4)); print('joints',{k:round(v,4) for k,v in j.items()})
    elif cmd=='goto':
        x,y,z,yaw=map(float,sys.argv[2:6]); secs=float(sys.argv[6]) if len(sys.argv)>6 else 3.0
        c.goto(x,y,z,yaw,secs)
    elif cmd=='grip':
        c.grip(float(sys.argv[2]))
    elif cmd=='ikcheck':
        x,y,z,yaw=map(float,sys.argv[2:6]); print(np.round(c.ik(x,y,z,yaw),4))
    rclpy.shutdown()
EOF
timeout 120 python3 ctl.py pose

# openrua op 17
cat > fkcheck.py <<'EOF'
import numpy as np
q=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
# modified DH (Craig) for Panda
dh=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
def T(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
M=np.eye(4)
for (a,d,al),th in zip(dh,q): M=M@T(a,d,al,th)
M=M@T(0,0.107,0,0)  # flange (link8)
print('link8 in base:',M[:3,3].round(4))
# hand = link8 rotated -45deg about z
print('hand z axis (base frame):',M[:3,2].round(3))
EOF
python3 fkcheck.py

# openrua op 18
sed -i 's/^BASE=np.array(\[-0.51,0.0,0.42\])/BASE=np.array([0.0,0.0,0.0])  # MoveIt model frame == world here (verified by FK)/' ctl.py
# current hand at world (-0.053,0,0.778): tcp = hand + 0.1034*z_axis; z axis ~(-0.057,0,-0.998) -> tcp ~(-0.059,0,0.675)
timeout 120 python3 ctl.py ikcheck -0.059 0.0 0.675 0.0

# openrua op 19
sed -i 's/s\.ik=s\.n\.create_client(GetPositionIK/s.ikc=s.n.create_client(GetPositionIK/; s/s\.fk=s\.n\.create_client(GetPositionFK/s.fkc=s.n.create_client(GetPositionFK/; s/assert s\.ik\.wait_for_service(10) and s\.fk\.wait_for_service(10)/assert s.ikc.wait_for_service(10) and s.fkc.wait_for_service(10)/; s/f=s\.fk\.call_async(r)/f=s.fkc.call_async(r)/; s/f=s\.ik\.call_async(r)/f=s.ikc.call_async(r)/' ctl.py
timeout 120 python3 ctl.py ikcheck -0.059 0.0 0.675 0.0

# openrua op 20
sed -i "s/r.ik_request.group_name='panda_arm'/r.ik_request.group_name='panda_arm'; r.ik_request.ik_link_name='panda_hand'/" ctl.py
timeout 120 python3 ctl.py ikcheck -0.059 0.0 0.675 0.0

# openrua op 21
cat > step.py <<'EOF'
import sys; from ctl import Ctl
c=Ctl()
for cmd in sys.argv[1:]:
    p=cmd.split(':')
    if p[0]=='grip': print('grip',p[1]); c.grip(float(p[1]))
    elif p[0]=='goto':
        x,y,z,yaw=map(float,p[1:5]); secs=float(p[5]) if len(p)>5 else 3.0
        print('goto',x,y,z,yaw,secs); c.goto(x,y,z,yaw,secs)
    elif p[0]=='pose':
        w,q,tcp,j=c.pose(); print('hand',w.round(4),'tcp',tcp.round(4),'fingers',round(j['panda_finger_joint1'],4),round(j['panda_finger_joint2'],4))
EOF
timeout 600 python3 -u step.py grip:0.04 goto:0.108:-0.189:0.62:0.0:3 2>&1 | tee -a run.log

# openrua op 22
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png && timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png

# openrua op 23
timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 320 295; timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 320 260; timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 320 330

# openrua op 24
timeout 900 python3 -u step.py goto:0.102:-0.187:0.50:0.0:2 goto:0.102:-0.187:0.434:0.0:2 grip:0.0 goto:0.102:-0.187:0.65:0.0:2.5 pose 2>&1 | tee -a run.log

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png

# openrua op 26
timeout 900 python3 -u step.py goto:0.0:0.26:0.72:0.0:3.5 grip:0.04 pose 2>&1 | tee -a run.log && timeout 60 python3 tools/perception/cam_snap.py agentview agent4.png

# openrua op 27
timeout 60 python3 tools/perception/cam_snap.py birdview bird2.png && python3 -c "
import cv2; im=cv2.imread('bird2.png'); c=im[240:330,340:430]; cv2.imwrite('bird2_crop.png',cv2.resize(c,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))"

# openrua op 28
timeout 900 python3 -u step.py goto:-0.097:0.040:0.65:0.0:3.5 2>&1 | tee -a run.log && timeout 60 python3 tools/perception/cam_snap.py birdview bird3.png && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png && python3 -c "
import cv2; im=cv2.imread('bird3.png'); c=im[240:330,340:430]; cv2.imwrite('bird3_crop.png',cv2.resize(c,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))"

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih2_depth.png >/dev/null
cat > candisc.py <<'EOF'
import numpy as np, rclpy
from tf2_msgs.msg import TFMessage
from geo import quatR
rclpy.init(); n=rclpy.create_node('x'); got={}
n.create_subscription(TFMessage,'/tf',lambda m:[got.setdefault(t.child_frame_id,t.transform) for t in m.transforms],10)
import time; t0=time.time()
while 'robot0_eye_in_hand_optical_frame' not in got and time.time()-t0<10: rclpy.spin_once(n,timeout_sec=0.2)
t=got['robot0_eye_in_hand_optical_frame']; print('cam at',t.translation.x,t.translation.y,t.translation.z)
R=quatR(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w); T=np.array([t.translation.x,t.translation.y,t.translation.z])
d=np.load('eih2_depth.npy').astype(float); H,W=d.shape; f=579.4112549695428
u,v=np.meshgrid(np.arange(W),np.arange(H)); P=np.stack([(u-W/2)*d/f,(v-H/2)*d/f,d],-1)@R.T+T
# can region: pixels near (320,305) radius 60
m=((u-320)**2+(v-305)**2<60**2)&np.isfinite(P[...,2])
p=P[m]; print('z range in region',p[:,2].min().round(3),p[:,2].max().round(3))
top=p[p[:,2]>p[:,2].max()-0.012]
print('can top: n',len(top),'centre',top[:,0].mean().round(4),top[:,1].mean().round(4),'z',top[:,2].mean().round(4),'x[%.3f,%.3f] y[%.3f,%.3f]'%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max()))
EOF
timeout 60 python3 candisc.py

# openrua op 30
timeout 10 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:"; timeout 10 ros2 topic echo /robot0_eye_in_hand/depth/image_raw --once --no-arr | grep -E "height|width"

# openrua op 31
sed -i 's/f=579.4112549695428/f=312.77408948188935/' candisc.py && timeout 60 python3 candisc.py

# openrua op 32
timeout 900 python3 -u step.py grip:0.04 goto:-0.102:0.041:0.56:0.0:2.5 pose 2>&1 | tee -a run.log

# openrua op 33
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png

# openrua op 34
timeout 900 python3 -u step.py goto:-0.102:0.041:0.462:0.0:2 grip:0.0 goto:-0.102:0.041:0.68:0.0:2.5 pose 2>&1 | tee -a run.log && timeout 60 python3 tools/perception/cam_snap.py agentview agent5.png

# openrua op 35
timeout 900 python3 -u step.py goto:0.0:0.26:0.72:0.0:4 pose 2>&1 | tee -a run.log

# openrua op 36
timeout 900 python3 -u step.py grip:0.04 goto:-0.15:0.0:0.75:0.0:3.5 pose 2>&1 | tee -a run.log && timeout 60 python3 tools/perception/cam_snap.py birdview bird4.png && timeout 60 python3 tools/perception/cam_snap.py agentview agent6.png && python3 -c "
import cv2; im=cv2.imread('bird4.png'); c=im[240:330,340:430]; cv2.imwrite('bird4_crop.png',cv2.resize(c,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))"

# openrua op 37
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird4_depth.png >/dev/null
python3 - <<'EOF'
import numpy as np
from geo import quatR
d=np.load('bird4_depth.npy').astype(float); H,W=d.shape; f=579.4112549695428
u,v=np.meshgrid(np.arange(W),np.arange(H))
R=quatR(0.7071,0.7071,0,0); T=np.array([-0.2,0,3.0])
P=np.stack([(u-W/2)*d/f,(v-H/2)*d/f,d],-1)@R.T+T
# basket interior footprint (inside walls)
m=(P[...,0]>-0.06)&(P[...,0]<0.065)&(P[...,1]>0.195)&(P[...,1]<0.33)&np.isfinite(P[...,2])
p=P[m]
print('basket interior: z range %.3f..%.3f'%(p[:,2].min(),p[:,2].max()))
inner=p[p[:,2]<0.60]  # below rim -> contents/floor
print('contents below rim: z max %.3f ; floor-ish min %.3f'%(inner[:,2].max(),inner[:,2].min()))
hi=inner[inner[:,2]>0.45]
print('objects inside (z>0.45): n=%d x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f'%(len(hi),hi[:,0].min(),hi[:,0].max(),hi[:,1].min(),hi[:,1].max(),hi[:,2].max()))
# anything left on the table where the two objects were?
for name,xr,yr in [('cream_cheese_spot',(0.065,0.14),(-0.211,-0.169)),('tomato_spot',(-0.135,-0.07),(0.007,0.076))]:
    m=(P[...,0]>xr[0])&(P[...,0]<xr[1])&(P[...,1]>yr[0])&(P[...,1]<yr[1])&np.isfinite(P[...,2])
    print(name,'max z on table now: %.3f (table=0.42)'%P[m][:,2].max())
EOF
