from refine import *
r = Robot()
BX, BY = 0.032, 0.607
cen, pts = refine(r, BX, BY, 0.44, 0.545, rad=0.075, cam="birdview")
pts = np.asarray(pts)
inner = pts[(np.abs(pts[:, 0] - BX) < 0.07) & (np.abs(pts[:, 1] - BY) < 0.065)]
soup = inner[(inner[:, 2] > 0.505)]
tom = inner[(inner[:, 2] > 0.46) & (inner[:, 2] < 0.505)]
log("soup-lid pts", len(soup), "center", soup[:, :2].mean(0).round(3) if len(soup) else None, "z", soup[:, 2].max().round(3) if len(soup) else None)
log("tomato pts", len(tom), "center", tom[:, :2].mean(0).round(3) if len(tom) else None, "ztop", tom[:, 2].max().round(3) if len(tom) else None,
    "extent x", tom[:, 0].min().round(3), tom[:, 0].max().round(3), "y", tom[:, 1].min().round(3), tom[:, 1].max().round(3))
# anything left at the old can spot?
cen2, pts2 = refine(r, 0.269, 0.3535, 0.44, 0.52, rad=0.12, cam="birdview")
log("points at old can location:", len(pts2))
_, _, tcp = r.hand_pose(); log("arm tcp", tcp.round(3), "fingers", r.fingers())
