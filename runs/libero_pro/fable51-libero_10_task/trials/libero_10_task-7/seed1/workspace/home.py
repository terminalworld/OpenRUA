from arm import *
a=Arm()
home=[0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854]
for i in range(3):
    code,err=a.move(home,4)
    if code==0 and err<0.01: break
print("fingers",a.fingers(),flush=True)
