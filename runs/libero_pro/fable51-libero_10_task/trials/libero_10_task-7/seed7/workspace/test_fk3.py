from robot import *
r = Robot()
for yaw in (0, 90):
    quat = Robot.quat_topdown(yaw)
    sol = r.ik_world([-0.22, -0.125, 0.62], quat)
    fk = r.fk_world(sol)
    Rh = Rot.from_quat(fk[1]).as_matrix()
    print(f"yaw {yaw}: tcp {fk[0].round(4)} hand y-axis (finger open dir) in world {Rh[:,1].round(3)} hand z {Rh[:,2].round(3)}")
