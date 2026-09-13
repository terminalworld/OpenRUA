from rob import *
from pick_place import go
r=Robot('home')
HOME=[0.0,-0.161,0.0,-2.445,0.0,2.227,0.785]
go(r,[HOME],5.0)
print('tcp',r.tcp_world().round(3))
