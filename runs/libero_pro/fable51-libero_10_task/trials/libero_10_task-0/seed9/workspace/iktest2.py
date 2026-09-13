import sys; sys.argv=["x","none"]
src=open("pick_place.py").read().split("names = sys.argv")[0]
exec(src)
for tgt in [(-0.067,0.070,0.64),(-0.067,0.070,0.47),(0.107,-0.191,0.437),(0.0,0.265,0.70),(0.0,0.265,0.72)]:
    sol=ik(tgt)
    if sol is None: print(tgt,"NO IK"); continue
    print(tgt,"-> joints",np.round(sol,3),"fk tcp",np.round(fk(sol),4))
rclpy.shutdown()
