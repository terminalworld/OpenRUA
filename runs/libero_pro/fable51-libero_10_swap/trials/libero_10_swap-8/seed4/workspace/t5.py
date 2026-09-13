from robot import *
r=Robot()
q0=r.arm_q()
def R_tilt(tilt, yaw=0.0):
    # hand z-axis pointing down-forward (+x) by `tilt` below horizontal; fingers along world y
    # start from straight-down (rot_x(pi)): z=-Z, x=X, y=-Y. Rotate about world y by +(90-tilt) so z tilts toward +x?
    # rot_y(a) maps (0,0,-1) -> (-sin a, 0, -cos a). want (cos t, 0, -sin t) => -sin a = cos t => a = -(90-t)
    a = -(np.pi/2 - tilt)
    return rot_z(yaw) @ rot_y(a) @ rot_x(np.pi)
for tilt in [np.radians(30), np.radians(45), np.radians(60)]:
    R=R_tilt(tilt); print('tilt',np.degrees(tilt),'z-axis',R[:,2].round(3),'y-axis',R[:,1].round(3))
    for (x,y) in [(0.17,-0.02),(0.20,0.08),(0.23,0.08),(0.25,0.08)]:
        s=r.ik_tcp([x,y,0.97],R,seed=q0,timeout=2)
        print(f'  ({x},{y}): {"OK "+str(s.round(2)) if s is not None else "fail"}')
