import rclpy, time, numpy as np, yaml
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
tw = next(a for a in yaml.safe_load(open("/workspace/machine.yaml"))["actuators"] if a["kind"] == "cartesian_twist")
rclpy.init(); n=rclpy.create_node("cal")
st={"js":None, "clk":None}
n.create_subscription(JointState,"/joint_states",lambda m:st.__setitem__("js",np.array(m.position[:7])),10)
n.create_subscription(Clock,"/clock",lambda m:st.__setitem__("clk",m.clock.sec+m.clock.nanosec*1e-9),10)
pub=n.create_publisher(TwistStamped,tw["port"],10)
def settle(quiet=3.0):
    """spin until the sim clock value has not changed for `quiet` wall seconds"""
    last=time.time(); c=st["clk"]
    while time.time()-last<quiet:
        rclpy.spin_once(n,timeout_sec=0.2)
        if st["clk"]!=c: c=st["clk"]; last=time.time()
    return c
def test(nmsg, dt, vz):
    c0=settle(); j0=st["js"].copy()
    msg=TwistStamped(); msg.header.frame_id=tw["frame"]; msg.twist.linear.z=vz
    t0=time.time()
    for _ in range(nmsg):
        msg.header.stamp=n.get_clock().now().to_msg(); pub.publish(msg); rclpy.spin_once(n,timeout_sec=dt)
    c1=settle(); j1=st["js"]
    print(f"{nmsg} msgs @dt={dt} vz={vz}: clock +{c1-c0:.3f}s, wall {time.time()-t0:.1f}s, dq={np.round(j1-j0,4)}")
if __name__=="__main__":
    import sys
    for spec in sys.argv[1:]:
        a,b,c=spec.split(","); test(int(a),float(b),float(c))
