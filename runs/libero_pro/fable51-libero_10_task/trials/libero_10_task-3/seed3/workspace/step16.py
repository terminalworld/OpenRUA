import numpy as np, math, sys, subprocess, shutil
from panda import *
from collide import check
np.set_printoptions(precision=4, suppress=True)
r = Robot()
R = topdown(math.pi/2)
def go(p, n=1):
    q = ik(make_T(p, R), r.q()); c = check(q, ignore=('bottle','drawer'))
    print('->', p, 'clear', c); assert c[0] > -0.005
    print(r.move_tcp(make_T(p, R), n_wp=n), 'tcp', r.tcp()[:3,3].round(3), 'fingers', np.round(r.fingers(),4), 'wrench', r.wrench().round(1), flush=True)
def snap(tag):
    subprocess.run(['python3','grab.py','birdview','agentview'], check=True, stdout=subprocess.DEVNULL)
    for c in ['birdview','agentview']:
        shutil.copy(f'{c}.png', f'{tag}_{c}.png'); shutil.copy(f'{c}.npz', f'{tag}_{c}.npz')
print('start tcp', r.tcp()[:3,3].round(3), 'fingers', r.fingers())
r.gripper(0.08)
go([0.056, 0.086, 1.15]); go([0.056, 0.086, 1.0], 3)
r.gripper(0.0)
go([0.065, 0.10, 1.0], 2); go([0.075, 0.125, 1.0], 2); go([0.085, 0.145, 1.0], 2)
snap('drag')
r.gripper(0.08)
go([0.085, 0.145, 1.2], 2)
