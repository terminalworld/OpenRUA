from arm import Arm
a = Arm("fktest")
for l in ["panda_link0", "panda_link8", "panda_hand", "panda_leftfinger", "panda_rightfinger"]:
    try:
        p, q = a.fk(link=l); print(l, p.round(4), q.round(4))
    except Exception as e: print(l, "ERR", e)
