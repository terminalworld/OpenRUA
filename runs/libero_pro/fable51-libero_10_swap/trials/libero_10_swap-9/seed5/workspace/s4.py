import arm, numpy as np
a = arm.Arm()
def tilt(deg, fx=True):
    s,c=np.sin(np.radians(deg)),np.cos(np.radians(deg))
    Z=np.array([0,c,-s]); Y=np.array([1.0,0,0]) if fx else np.array([-1.0,0,0])
    X=np.cross(Y,Z)
    return arm.quat_from_axes(X,Y,Z)
tests=[]
for deg in (30,45,60,75):
    for xyz in [(-0.17,-0.45,1.0),(-0.17,-0.45,0.93),(-0.17,-0.31,0.93),(-0.17,-0.5,1.05)]:
        tests.append((f"tilt{deg}",xyz,tilt(deg)))
for name,xyz,q in tests:
    sol=a.ik(xyz,q,at_tcp=True)
    if sol is None: sol=a.ik(xyz,q,at_tcp=True)
    print(f"{name:8s} {xyz} ->", None if sol is None else np.round(sol,3))
