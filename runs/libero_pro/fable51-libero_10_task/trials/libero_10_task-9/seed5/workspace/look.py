import numpy as np, sys, subprocess
sys.path.insert(0,'.')
subprocess.run(['python3','push.py','to','0',sys.argv[1] if len(sys.argv)>1 else '-0.05','0.14','1.01'],check=True,stdout=subprocess.DEVNULL)
subprocess.run(['python3','tools/perception/cam_snap.py','robot0_eye_in_hand','robot0_eye_in_hand.png'],stdout=subprocess.DEVNULL)
subprocess.run(['python3','tools/perception/cam_snap.py','/robot0_eye_in_hand/depth/image_raw','robot0_eye_in_hand_depth.png'],stdout=subprocess.DEVNULL)
subprocess.run(['python3','geo.py','tf'],stdout=subprocess.DEVNULL)
from geo import cloud
D=np.load('robot0_eye_in_hand_depth.npy'); P=cloud('robot0_eye_in_hand',D)
Y=P[...,1]; X=P[...,0]; Z=P[...,2]
print('y (cm) rows 230..360 step 5, cols 230..430 step 5')
for v in range(230,361,5):
    print(v, ' '.join(f'{int(round(Y[v,u]*100)):3d}' if Y[v,u]<1 else '  .' for u in range(230,431,5)))
print('x (cm) rows step 10')
for v in range(230,361,10):
    print(v, ' '.join(f'{int(round(X[v,u]*100)):3d}' if Y[v,u]<1 else '  .' for u in range(230,431,5)))
# mug points: inside cavity region, exclude walls (y<0.43), floor (z>0.95), ceiling
m=(Y>0.2)&(Y<0.43)&(Z>0.95)&(Z<1.075)&(X>-0.137)&(X<0.064)
Q=P[m]; print('mug pts',len(Q),'y min %.3f'%Q[:,1].min(),'x range %.3f %.3f'%(Q[:,0].min(),Q[:,0].max()))
near=Q[Q[:,1]<Q[:,1].min()+0.01]; print('nearest 1cm band: n',len(near),'x %.3f..%.3f z %.3f..%.3f'%(near[:,0].min(),near[:,0].max(),near[:,2].min(),near[:,2].max()))
