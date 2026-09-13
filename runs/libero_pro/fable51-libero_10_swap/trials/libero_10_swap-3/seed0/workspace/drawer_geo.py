import numpy as np
for cam in ["birdview","agentview","frontview","sideview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
    print("==", cam)
    # y-profile of max height along the drawer center line x in [-0.15,-0.05], for y from -0.26 to -0.04 at 5mm
    row = []
    for yy in np.arange(-0.26, -0.03, 0.005):
        m = ok&(x>-0.15)&(x<-0.05)&(np.abs(y-yy)<0.0025)
        row.append(f"{yy:+.3f}:{(np.max(z[m])-0.9)*100:4.1f}" if m.sum() else f"{yy:+.3f}:  --")
    print(" y-profile (max z cm) x∈[-0.15,-0.05]:"); print("  " + "  ".join(row[:16])); print("  " + "  ".join(row[16:32])); print("  " + "  ".join(row[32:]))
    row = []
    for xx in np.arange(-0.26, 0.06, 0.005):
        m = ok&(y>-0.20)&(y<-0.11)&(np.abs(x-xx)<0.0025)
        row.append(f"{xx:+.3f}:{(np.max(z[m])-0.9)*100:4.1f}" if m.sum() else f"{xx:+.3f}:  --")
    print(" x-profile (max z cm) y∈[-0.20,-0.11]:"); print("  " + "  ".join(row[:16])); print("  " + "  ".join(row[16:32])); print("  " + "  ".join(row[32:48])); print("  " + "  ".join(row[48:]))
