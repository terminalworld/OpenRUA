"""Locate a pot: knob (z>0.15 above table) centroid + lower-chamber centre, from given cams.
Usage: python3 potfind.py x0 x1 y0 y1 [cams...]"""
import sys, subprocess, numpy as np
x0,x1,y0,y1=map(float,sys.argv[1:5]); cams=sys.argv[5:] or ["birdview","sideview","frontview","agentview"]
subprocess.run(["python3","sidecloud.py",*cams],check=True,timeout=300)
table=0.894
for cam in cams:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    m=(pw[...,0]>x0)&(pw[...,0]<x1)&(pw[...,1]>y0)&(pw[...,1]<y1)&np.isfinite(Z)
    k=m&(Z>table+0.15)&(Z<table+0.175)
    lo=m&(Z>table+0.015)&(Z<table+0.05)
    P=pw[k]; L=pw[lo]
    if len(P): print(f"{cam}: knob n={len(P)} centre=({P[:,0].mean():.4f},{P[:,1].mean():.4f}) top z={P[:,2].max():.3f}")
    if len(L): print(f"{cam}: lower n={len(L)} x[{L[:,0].min():.3f},{L[:,0].max():.3f}] y[{L[:,1].min():.3f},{L[:,1].max():.3f}]")
