from pp import *
pp_=PP(); r=pp_.r
LIM=np.array(FJT['limits_rad'])
for pitch in [0.0,-0.2,-0.35,-0.5]:
    R=grasp_R(yaw=0,pitch=pitch)
    print('pitch',pitch,'z',R[:,2].round(2))
    for x in [0.14,0.16,0.18,0.20,0.21]:
        for y in [-0.015,0.085]:
            s=r.ik_best([x,y,1.09],R,n_random=40)
            if s is None: print(f'  ({x},{y}) FAIL'); continue
            margin=np.minimum(s-LIM[:,0],LIM[:,1]-s).min()
            print(f'  ({x},{y}) margin={margin:.2f}',s.round(2))
