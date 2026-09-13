from robot import *
r = Robot("lift")
def tq(pitch): return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat([1,0,0,0])).as_quat()
r.goto([-0.196, 0.056, 0.70], tq(10), 2.5)
print("wrench", np.round(r.wrench(),2))
