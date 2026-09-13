import sys; from ctl import Ctl
c=Ctl()
for cmd in sys.argv[1:]:
    p=cmd.split(':')
    if p[0]=='grip': print('grip',p[1]); c.grip(float(p[1]))
    elif p[0]=='goto':
        x,y,z,yaw=map(float,p[1:5]); secs=float(p[5]) if len(p)>5 else 3.0
        print('goto',x,y,z,yaw,secs); c.goto(x,y,z,yaw,secs)
    elif p[0]=='pose':
        w,q,tcp,j=c.pose(); print('hand',w.round(4),'tcp',tcp.round(4),'fingers',round(j['panda_finger_joint1'],4),round(j['panda_finger_joint2'],4))
