import numpy as np, rclpy
from rob import *
rclpy.init(); r=Robot()
R=R_from_axes([0,0,-1],[0,1,0])
goto(r,[-0.08,-0.45,1.071],R,seconds=2)
goto(r,[-0.10,-0.50,1.30],R,seconds=3)
for c in ['sideview','agentview','birdview','galleryview']:
    try: r.snap(c,f'/workspace/{c}_final.png'); print('saved',c)
    except Exception as e: print('snap fail',c,e)
print('tcp',r.tcp()[0])
