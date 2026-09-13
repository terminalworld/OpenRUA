"""Pose geometry checks for the lying mug held by the rim (rim at TCP-0.035 z_h, mug centre line = TCP - 0.044 y_h)."""
from cart import *
Rf = np.array([[-1,0,0],[0,0,1],[0,1,0]], float)
def Rrp(roll, pitch):
    return Rotation.from_euler('x', -pitch, degrees=True).as_matrix() @ Rotation.from_euler('y', roll, degrees=True).as_matrix() @ Rf
def report(tcp, R, tag=""):
    tcp=np.asarray(tcp); xh,yh,zh=R[:,0],R[:,1],R[:,2]
    rimc = tcp - 0.035*zh - 0.044*yh; nosec = rimc + 0.115*zh
    # ring extreme points: centre +- 0.0465 in plane perpendicular to zh -> vertical extent 0.0465*sqrt(1-zh_z^2)
    rz = 0.0465*np.sqrt(1-zh[2]**2); ry = 0.0465*np.sqrt(1-zh[1]**2)
    hand = tcp - 0.1034*zh; wrist = tcp - 0.21*zh; bodyfront = tcp - 0.045*zh
    fo = tcp + 0.05*yh  # outer finger (open) centre, tip at +0.01 zh
    print(f"{tag} tcp={np.round(tcp,3)} yh={np.round(yh,2)} zh={np.round(zh,2)}")
    print(f"   nose: front y={nosec[1]+ry:.3f} low z={nosec[2]-rz:.3f} top z={nosec[2]+rz:.3f} x[{nosec[0]-0.0465:.3f},{nosec[0]+0.0465:.3f}]")
    print(f"   rim : y={rimc[1]-ry:.3f} top z={rimc[2]+rz:.3f} low z={rimc[2]-rz:.3f}")
    print(f"   body front y={bodyfront[1]:.3f} z={bodyfront[2]:.3f}; hand={np.round(hand,3)} body ends {np.round(hand+0.1*yh,3)} {np.round(hand-0.1*yh,3)}")
    print(f"   wrist centre={np.round(wrist,3)} bottom z={wrist[2]-0.043:.3f}; outer finger open {np.round(fo,3)}")
if __name__=="__main__":
    import sys
    for a in sys.argv[1:]:
        x,y,z,roll,pitch=[float(v) for v in a.split(",")]
        report((x,y,z), Rrp(roll,pitch), f"roll{roll:.0f} pitch{pitch:.0f}")
