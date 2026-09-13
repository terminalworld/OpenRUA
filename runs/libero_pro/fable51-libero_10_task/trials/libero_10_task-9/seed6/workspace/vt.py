import sys; sys.argv=["x","-114","2"]
src=open('/workspace/doorpush.py').read().split("# --- plan")[0]
exec(src)
scene.apply(r.node, [door_box(th0)])
for back in [0.0,0.005,0.01,0.015,0.02,0.03]:
    t,R=pose(th0,back=back); q=r.solve_ik(hand_pose_from_tcp(t,R),R,seed=START_Q,timeout=2.0)
    print(back, valid(q) if q else "noik")
