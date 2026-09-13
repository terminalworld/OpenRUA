from rob import *
from scene import box
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
r = Robot()
# drop the basket collision object (we're about to move it)
cli = r.node.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(timeout_sec=20)
req = ApplyPlanningScene.Request(); req.scene.is_diff = True
co = CollisionObject(); co.header.frame_id = "world"; co.id = "basket"; co.operation = CollisionObject.REMOVE
req.scene.world.collision_objects = [co]
fut = cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30); log("removed basket CO", fut.result().success)
# back off from the can region first (straight up along current approach)
r.gripper(0.04)
ok = r.move_tcp(0.0, 0.125, 0.75, yaw=math.pi/2, secs=6)
if not ok: log("retry"); r.move_tcp(0.0, 0.125, 0.75, yaw=math.pi/2, secs=6)
ok = r.move_tcp_line(0.0, 0.125, 0.50, yaw=math.pi/2, secs=4, n=3); log("down ok", ok)
ok = r.move_tcp_line(0.0, 0.37, 0.50, yaw=math.pi/2, secs=6, n=4); log("push ok", ok)
if not ok: log("retry push"); r.move_tcp_line(0.0, 0.37, 0.50, yaw=math.pi/2, secs=6, n=2)
ok = r.move_tcp_line(0.0, 0.30, 0.78, yaw=math.pi/2, secs=4, n=2); log("up ok", ok)
log("PHASE7 DONE")
