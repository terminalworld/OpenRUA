"""Measure the held pot relative to the TCP: knob centre, base z, chamber extents.
Usage: python3 heldpot.py [cams...]"""
import sys, subprocess, numpy as np
from rob import *
cams=sys.argv[1:] or ["sideview","frontview","birdview"]
subprocess.run(["python3","sidecloud.py",*cams],check=True,timeout=300)
r=Robot(); t,R=r.tcp(); print("TCP",t.round(4),"gap",round(r.finger_gap(),4))
ap=R[:,2].copy(); ap[2]=0; ap/=np.linalg.norm(ap); sd=R[:,1]
for cam in cams:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    # box around the TCP, excluding the hand region behind the TCP
    rel=pw-t; f=rel@ap; s=rel@sd
    m=np.isfinite(Z)&(f>-0.02)&(f<0.12)&(np.abs(s)<0.09)&(Z>t[2]-0.12)&(Z<t[2]+0.20)
    P=pw[m]
    if not len(P): print(cam,"nothing"); continue
    zt=P[:,2].max(); knob=P[P[:,2]>zt-0.02]
    fk=(knob-t)@ap; sk=(knob-t)@sd
    print(f"{cam}: n={len(P)} top z={zt:.3f} base z={P[:,2].min():.3f} knob fwd={fk.mean():.4f} side={sk.mean():.4f}")
    # lower chamber slab at TCP height
    lc=P[np.abs(P[:,2]-t[2])<0.01]; fl=(lc-t)@ap; sl=(lc-t)@sd
    if len(lc): print(f"   at TCP z: fwd[{fl.min():.3f},{fl.max():.3f}] side[{sl.min():.3f},{sl.max():.3f}] n={len(lc)}")
