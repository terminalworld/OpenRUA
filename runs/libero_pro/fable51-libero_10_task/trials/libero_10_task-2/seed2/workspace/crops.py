import numpy as np, cv2
img = cv2.imread("snaps/birdview.png"); H = np.load("snaps/bird_H.npy")
def crop(name, u0,v0,u1,v1, s=6):
    c = img[v0:v1, u0:u1]; c = cv2.resize(c, None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(f"snaps/crop_{name}.png", c)
crop("knob", 355,220,400,262, 10)
crop("pan", 205,245,325,320, 5)
crop("stove", 345,250,410,315, 8)
np.set_printoptions(linewidth=250, precision=3, suppress=True)
print("knob heights (rows 228..254, cols 364..390):")
print(H[228:255:2, 364:391:2].round(3))
