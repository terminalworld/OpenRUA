import numpy as np
# Panda modified DH (Craig): a, d, alpha per joint; flange d=0.107; hand rotated -45deg about z
DH=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
def T_dh(a,d,alpha,theta):
    ca,sa,ct,st=np.cos(alpha),np.sin(alpha),np.cos(theta),np.sin(theta)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
def fk_hand(q, base=np.array([-0.66,0,0.912])):
    T=np.eye(4); T[:3,3]=base
    for (a,d,al),th in zip(DH,q): T=T@T_dh(a,d,al,th)
    T=T@T_dh(0,0.107,0,0)          # flange
    T=T@T_dh(0,0,0,-np.pi/4)       # hand
    return T[:3,3],T[:3,:3]
def fk_links(q, base=np.array([-0.66,0,0.912])):
    T=np.eye(4); T[:3,3]=base; out=[]
    for (a,d,al),th in zip(DH,q):
        T=T@T_dh(a,d,al,th); out.append(T[:3,3].copy())
    return out
if __name__=='__main__':
    q=np.array([0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483])
    p,R=fk_hand(q); print(p); print(np.round(R,3))
