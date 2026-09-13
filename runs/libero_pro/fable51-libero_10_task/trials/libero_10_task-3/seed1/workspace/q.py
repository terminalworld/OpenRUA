import numpy as np, sys
P = np.load("agentview_xyz.npy")
def at(u,v): print(f"agentview px({u},{v}) -> {P[v,u].round(4)}")
# table surface samples
for (u,v) in [(300,400),(150,450),(500,450),(250,350)]: at(u,v)
print("--- bottle column (u=340):")
for v in range(150,245,5): at(340,v)
print("--- bottle body row v=210:")
for u in range(320,360,3): at(u,210)
