import numpy as np, math
def Rq(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def T(t,q):
    M=np.eye(4); M[:3,:3]=Rq(*q); M[:3,3]=t; return M
chain=[((0,0,0.333),(0,0,0,1)),((0,0,0),(-0.7048,-0.0569,-0.0569,0.7048)),((0,-0.316,0),(0.7071,0,0,0.7071)),
((0.0825,0,0),(-0.2415,-0.6646,0.6646,-0.2415)),((-0.0825,0.384,0),(-0.7071,0,0,0.7071)),((0,0,0),(0.3123,-0.6344,0.6344,0.3123)),
((0.088,0,0),(0.6533,-0.2706,0.2706,0.6533)),((0,0,0.107),(0,0,0,1)),((0,0,0),(0,0,-0.38268,0.92388))]
M=np.eye(4)
for t,q in chain: M=M@T(t,q)
print("hand in panda_link0 via TF chain:", np.round(M[:3,3],4)); print(np.round(M[:3,:3],3))
# Standard Panda DH FK
def dh(a,d,alpha,theta):
    ca,sa,ct,st=math.cos(alpha),math.sin(alpha),math.cos(theta),math.sin(theta)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483]
params=[(0,0.333,0),(0,0,-math.pi/2),(0,0.316,math.pi/2),(0.0825,0,math.pi/2),(-0.0825,0.384,-math.pi/2),(0,0,math.pi/2),(0.088,0,math.pi/2)]
F=np.eye(4)
for (a,d,al),th in zip(params,q): F=F@dh(a,d,al,th)
F=F@dh(0,0.107,0,0)
print("flange via DH:", np.round(F[:3,3],4))
