import sys, numpy as np
cam=sys.argv[1]
P=np.load(f"{cam}_world.npy")
import cv2
color=cv2.imread(f"{cam}.png")
pts=[tuple(map(int,a.split(","))) for a in sys.argv[2:]]
for u,v in pts:
    print((u,v), P[v,u].round(4), color[v,u])
