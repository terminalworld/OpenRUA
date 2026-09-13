import numpy as np
c=np.array([-0.121,-0.299,0.9465]); R=0.0465
th=np.deg2rad(34.0)
t=np.array([np.cos(th),np.sin(th),0.0]); n_in=np.array([-np.sin(th),np.cos(th),0.0])   # n_in = into the mug (mouth->base)
p=c-R*t                     # -x side wall point
b=np.deg2rad(45); d=np.cos(b)*n_in-np.sin(b)*np.array([0,0,1.0])
