import numpy as np
d=np.load("snaps/birdview_depth.npy"); fx=579.4112549695428; cx=320; cy=240; camz=3.0; camx=-0.2
h=camz-d
np.set_printoptions(linewidth=250, precision=2)
print("white mug region heights (rows 248-282, cols 224-270, step 2):")
print(h[248:283:2, 224:271:2])
print("\nmicrowave region rows 236-360 step 4, cols 300-464 step 4:")
print(h[236:361:4, 300:465:4])
