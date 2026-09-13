import numpy as np
q=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
# modified DH (Craig) for Panda
dh=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
def T(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
M=np.eye(4)
for (a,d,al),th in zip(dh,q): M=M@T(a,d,al,th)
M=M@T(0,0.107,0,0)  # flange (link8)
print('link8 in base:',M[:3,3].round(4))
# hand = link8 rotated -45deg about z
print('hand z axis (base frame):',M[:3,2].round(3))
