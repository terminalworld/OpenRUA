from ctl import *; import rclpy
from moveit_msgs.srv import GetPlanningScene
r=Robot("gs"); c=r.node.create_client(GetPlanningScene,"/get_planning_scene"); c.wait_for_service(5)
req=GetPlanningScene.Request(); req.components.components=req.components.WORLD_OBJECT_NAMES|req.components.ROBOT_STATE_ATTACHED_OBJECTS
f=c.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=20); sc=f.result().scene
print("world:", [o.id for o in sc.world.collision_objects]); print("attached:", [(a.object.id,a.link_name,a.touch_links) for a in sc.robot_state.attached_collision_objects])
