import numpy as np, cv2
P = np.load("birdview_world.npy")
Z = P[...,2]
# crop to table region rows 150-420, cols 150-500, upscale
img = np.clip((Z-0.9)/0.6,0,1)
img8 = (img*255).astype(np.uint8)
col = cv2.applyColorMap(img8, cv2.COLORMAP_JET)
crop = col[150:420, 150:500]
crop = cv2.resize(crop, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
# grid lines every 0.1 m in world: world x = ? need mapping; just draw pixel grid every 20px (~7cm)
for i in range(0, crop.shape[1], 40): cv2.line(crop,(i,0),(i,crop.shape[0]-1),(255,255,255),1); cv2.putText(crop,str(150+i//2),(i,10),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
for j in range(0, crop.shape[0], 40): cv2.line(crop,(0,j),(crop.shape[1]-1,j),(255,255,255),1); cv2.putText(crop,str(150+j//2),(0,j+10),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
cv2.imwrite("snaps/birdview_height.png", crop)
