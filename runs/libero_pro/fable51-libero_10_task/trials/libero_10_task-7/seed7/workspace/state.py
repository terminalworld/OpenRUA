from robot import *
r = Robot()
q = r.arm_q(); log("q", q.round(3)); log("fingers", r.fingers()); log("tcp", r.fk_world(q))
