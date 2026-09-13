import rclpy, time, numpy as np
from rosgraph_msgs.msg import Clock
from rob import Robot
r=Robot("cc")
clk=[]
r.node.create_subscription(Clock, "/clock", lambda m: clk.append(m.clock.sec+m.clock.nanosec*1e-9), 10)
def now():
    clk.clear(); t=time.time()
    while not clk and time.time()-t<3: r.spin(0.1)
    return clk[-1] if clk else None
print("clock:", now()); time.sleep(3); print("clock after 3s wall idle:", now())
# spin (listen) for 3 s
t=time.time()
while time.time()-t<3: r.spin(0.1)
print("clock after 3s spinning:", now())
