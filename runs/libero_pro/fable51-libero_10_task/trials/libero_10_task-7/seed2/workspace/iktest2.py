import sys; sys.argv=["x","state"]
from arm import *
a = Arm()
a.hand_world()
t = a.buf.lookup_transform("world", "panda_link8", Time())
q = t.transform.rotation; tr = t.transform.translation
print("link8 world", (round(tr.x,4),round(tr.y,4),round(tr.z,4)), (round(q.x,4),round(q.y,4),round(q.z,4),round(q.w,4)))
p = np.array([tr.x,tr.y,tr.z])
sol = a.solve_ik(p + a.base, (q.x,q.y,q.z,q.w))
print("IK link8 world-frame:", None if sol is None else [round(v,3) for v in sol])
print("current:", [round(a.joints()[n],3) for n in JOINTS])
rclpy.shutdown()
