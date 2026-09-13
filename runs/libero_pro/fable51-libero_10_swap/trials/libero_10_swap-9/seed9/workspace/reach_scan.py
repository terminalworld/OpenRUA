from rob import *
r = Robot('scan')
q_side = quat_from_axes([0,0,1],[1,0,0],[0,1,0])
seed = r.arm_q()
print('hand +y horizontal, z=1.00 ; rows x, cols y')
ys = np.arange(-0.40,-0.66,-0.04)
print('      '+' '.join(f'{y:+.2f}' for y in ys))
for x in [-0.30,-0.25,-0.20,-0.155,-0.10,-0.05]:
    row=[]
    for y in ys:
        sol = r.ik([x,y,1.0], q_side, seed=seed, timeout=1.0, attempts=2)
        row.append('  ok ' if sol is not None else '  -- ')
    print(f'{x:+.3f}', ' '.join(row), flush=True)
