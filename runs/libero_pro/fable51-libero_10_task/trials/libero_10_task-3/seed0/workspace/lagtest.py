import rclpy, time, numpy as np
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from rob import Robot, ARM, rot_from_axes
r=Robot("lag")
clk=[0.0]
r.node.create_subscription(Clock, "/clock", lambda m: clk.__setitem__(0, m.clock.sec+m.clock.nanosec*1e-9), 10)
log=[]
target=[None]
def on_js(m):
    d=dict(zip(m.name,m.position))
    q=np.array([d[j] for j in ARM])
    err = None if target[0] is None else float(np.abs(q-target[0]).max())
    log.append((time.time(), clk[0], err, d["panda_finger_joint1"]))
r.node.create_subscription(JointState,"/joint_states",on_js,10)
# small move: raise TCP by 3 cm at the same orientation
p,quat,R=r.tcp()
q=r.ik_tcp(p+[0,0,0.03],R)
target[0]=np.array(q)
t0=time.time()
code,err=r.move_q(q,2.0)
t1=time.time()
print("result after", round(t1-t0,2),"s wall; code",code,"err",err)
# keep listening 4 s
while time.time()-t1<4: r.spin(0.1)
for (tw,tc,e,f) in log[::max(1,len(log)//40)]:
    print(f"wall+{tw-t0:6.2f} clock={tc:7.3f} err={e if e is None else round(e,4)} finger={f:.4f}")
print("n msgs",len(log),"final clock",clk[0])
