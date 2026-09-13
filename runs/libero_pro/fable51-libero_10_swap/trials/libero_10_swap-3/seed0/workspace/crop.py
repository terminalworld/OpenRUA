import cv2, sys
img = cv2.imread(sys.argv[1]); x0,y0,x1,y1 = map(int, sys.argv[2:6]); s = int(sys.argv[6]) if len(sys.argv)>6 else 4
crop = cv2.resize(img[y0:y1, x0:x1], None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST)
cv2.imwrite(sys.argv[7] if len(sys.argv)>7 else "crop.png", crop); print("ok")
