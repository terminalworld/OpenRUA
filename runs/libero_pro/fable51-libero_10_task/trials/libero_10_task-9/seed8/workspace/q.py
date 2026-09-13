import numpy as np, sys
d = np.load(sys.argv[1]); pw = d["pw"]; col = d["color"]
def at(u,v): 
    p = pw[v,u]; print(f"px({u},{v}) -> world ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) bgr={col[v,u]}")
for a in sys.argv[2:]:
    u,v = map(int,a.split(",")); at(u,v)
