from refine import *
r = Robot()
cen, pts = refine(r, 0.269, 0.3535, 0.435, 0.51, rad=0.10, cam="birdview")
pts = np.asarray(pts)
ang = math.radians(31.1); c = np.array([math.cos(ang), math.sin(ang)]); f = np.array([-math.sin(ang), math.cos(ang)])
rel = pts[:, :2] - [0.269, 0.3535]
pc = rel @ c; pf = rel @ f
mid = np.abs(pc) < 0.025
log("along-axis extent", round(pc.min(), 3), round(pc.max(), 3))
for zlo in (0.435, 0.45, 0.46, 0.47):
    m = mid & (pts[:, 2] > zlo)
    log(f"z>{zlo}: n={m.sum()} f-extent [{pf[m].min():.4f},{pf[m].max():.4f}] width={pf[m].max()-pf[m].min():.4f} f-center={(pf[m].max()+pf[m].min())/2:.4f}")
# hand/finger points? (exclude z>0.51 already)
