ok = A.move_to([-0.09, 0.06, 0.72], DOWN, seconds=2.5)
A.log("waypoint ok=", ok, "fingers:", A.fingers())
ok = A.move_to([-0.02, 0.25, 0.74], DOWN, seconds=2.5)
A.log("STEP3 above basket ok=", ok, "fingers:", A.fingers())
