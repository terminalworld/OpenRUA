import numpy as np
A = np.load("snaps/agentview_xyz.npy")
S = np.load("snaps/sideview_xyz.npy")
print("agentview bottle pixels:")
for (u,v) in [(362,200),(362,215),(360,190),(368,155),(366,170),(365,180)]:
    print((u,v), A[v,u])
print("sideview bottle pixels:")
for (u,v) in [(358,280),(358,265),(360,250),(358,290),(360,240)]:
    print((u,v), S[v,u])
print("agentview drawer pixels (open drawer interior/rim):")
for (u,v) in [(420,300),(400,280),(380,260),(440,320),(370,250),(455,340),(375,330),(360,300),(470,285),(480,300)]:
    print((u,v), A[v,u])
print("agentview cabinet top:", A[250,540], A[220,500], A[200,470])
print("agentview table:", A[400,300], A[420,200])
