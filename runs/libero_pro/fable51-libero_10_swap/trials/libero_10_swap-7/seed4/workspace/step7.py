ok = A.move_to([0.06, 0.03, 0.72], DOWN, seconds=2.5)
A.log("waypoint ok=", ok, "fingers:", A.fingers())
ok = A.move_to([0.03, 0.26, 0.72], DOWN, seconds=2.5)
A.log("above basket ok=", ok, "fingers:", A.fingers())
