#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la .ros tools

# openrua op 2
ls -la tools/action tools/perception; cat tools/action/*.py tools/perception/*.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do python3 tools/perception/cam_snap.py $c; done

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('agentview.png')
cv2.imwrite('crop_cans.png', cv2.resize(im[220:320,170:410], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
im=cv2.imread('robot0_robotview.png')
cv2.imwrite('crop_cans2.png', cv2.resize(im[30:140,130:430], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
cd /workspace; python3 tools/perception/px2world.py agentview 370 250; python3 tools/perception/px2world.py agentview 370 275; python3 tools/perception/px2world.py agentview 155 385; python3 tools/perception/px2world.py agentview 540 255; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12

# openrua op 7
cd /workspace; echo table; python3 tools/perception/px2world.py agentview 320 450; echo soupcan; python3 tools/perception/px2world.py agentview 210 250; echo bird_tomato; python3 tools/perception/px2world.py birdview 280 253; echo bird_other; python3 tools/perception/px2world.py birdview 275 228; echo bird_cream; python3 tools/perception/px2world.py birdview 275 310; echo bird_basket_center; python3 tools/perception/px2world.py birdview 385 290; echo bird_table; python3 tools/perception/px2world.py birdview 450 200

# openrua op 8
mkdir -p "$(dirname /workspace/scan.py)"
cat > /workspace/scan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Point-cloud scan of a camera: cluster points above the table, print
world centroid, extents and mean color per cluster.
Usage: python3 scan.py <camera> [table_z=0.43]
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy import ndimage


def grab(node, topic, T, timeout=15.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    table_z = float(sys.argv[2]) if len(sys.argv) > 2 else 0.43
    rclpy.init()
    node = rclpy.create_node("scan")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "rgb8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + tr
    valid = np.isfinite(depth) & (depth > 0)
    above = valid & (P[..., 2] > table_z) & (P[..., 2] < table_z + 0.35)
    # exclude robot: anything within 0.2 m xy of base (-0.51, 0) roughly and its arm above
    lab, n = ndimage.label(above)
    print(f"{cam}: {n} clusters above z={table_z}")
    for i in range(1, n + 1):
        m = lab == i
        if m.sum() < 30:
            continue
        pts = P[m]
        c = color[m].mean(0)
        uu, vv = u[m].mean(), v[m].mean()
        print(f"  #{i} n={m.sum():5d} px=({uu:5.0f},{vv:5.0f}) "
              f"cen=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f},{pts[:,2].mean():.3f}) "
              f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
              f"z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] rgb=({c[0]:.0f},{c[1]:.0f},{c[2]:.0f})")
    np.save(f"{cam}_P.npy", P)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 9
cd /workspace; python3 scan.py birdview 0.435 2>&1; python3 scan.py agentview 0.435 2>&1

# openrua op 10
cd /workspace; python3 -c "
import numpy as np
P=np.load('birdview_P.npy').reshape(-1,3)
m=(P[:,0]>0.05)&(P[:,0]<0.17)&(P[:,1]>-0.23)&(P[:,1]<-0.15)&(P[:,2]>0.445)&(P[:,2]<0.47)
pts=P[m][:,:2]; c=pts.mean(0); 
w,v=np.linalg.eigh(np.cov((pts-c).T)); 
ax=v[:,1]; print('cream center',c,'long axis',ax,'yaw deg',np.degrees(np.arctan2(ax[1],ax[0])),'n',m.sum())
proj=(pts-c)@v; print('extents long',proj[:,1].min(),proj[:,1].max(),'short',proj[:,0].min(),proj[:,0].max())
m=(P[:,0]>-0.13)&(P[:,0]<-0.03)&(P[:,1]>0.02)&(P[:,1]<0.12)&(P[:,2]>0.50)&(P[:,2]<0.53)
pts=P[m][:,:2]; print('can top center',pts.mean(0), 'n',m.sum(), 'x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max())
"
ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | grep -A 12 "pose:"

# openrua op 11
cd /workspace; python3 -c "
import numpy as np
P=np.load('birdview_P.npy').reshape(-1,3)
for cx,cy in [(0.2,0.0),(0.1,-0.19),(-0.07,0.07),(0.0,0.4)]:
  m=(abs(P[:,0]-cx)<0.03)&(abs(P[:,1]-cy)<0.03)&np.isfinite(P[:,2])
  print(cx,cy,'z median',np.median(P[m][:,2]),'min',P[m][:,2].min(),'max',P[m][:,2].max())
"
ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | tail -25

# openrua op 12
mkdir -p "$(dirname /workspace/iktest.py)"
cat > /workspace/iktest.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Ask IK for the current FK pose, in world coords vs base coords; see which returns the current joints."""
import rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

JOINTS = ["panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"]
CUR = [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]

rclpy.init(); node = rclpy.create_node("iktest")
cli = node.create_client(GetPositionIK, "/compute_ik"); cli.wait_for_service(10)

def ik(x,y,z,qx,qy,qz,qw):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z = x,y,z
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w = qx,qy,qz,qw
    req.ik_request.robot_state.joint_state = JointState(name=JOINTS, position=CUR)
    req.ik_request.avoid_collisions = False
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    r = fut.result()
    if r is None: return "timeout"
    if r.error_code.val != 1: return f"err {r.error_code.val}"
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return [round(sol[j],3) for j in JOINTS]

q = (0.9995966352021468, 0.0, -0.028400121347386207, 0.0)
print("world-frame pose :", ik(-0.05298564807835497, 0.0, 0.7776238083193787, *q))
print("base-frame pose  :", ik(-0.05298564807835497+0.51, 0.0, 0.7776238083193787-0.42, *q))
rclpy.shutdown()
OPENRUA_EOF

# openrua op 13
cd /workspace; timeout 120 python3 iktest.py

# openrua op 14
cd /workspace; ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand','panda_link8'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.058, -0.161, -0.058, -2.445, -0.012, 2.227, 0.009]}}}" 2>&1 | grep -o "position=geometry_msgs.msg.Point([^)]*), orientation=geometry_msgs.msg.Quaternion([^)]*)"

# openrua op 15
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick each target with a top-down grasp and drop it into the basket.
Reusable IK / trajectory / gripper clients; verifies each step from
/joint_states. Usage: python3 -u pick_place.py [can] [cream] > log
"""
import sys, time
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["panda_joint1","panda_joint2","panda_joint3","panda_joint4",
          "panda_joint5","panda_joint6","panda_joint7"]
TCP = 0.1034          # hand frame -> fingertip, along hand +Z
Q_DOWN = (1.0, 0.0, 0.0, 0.0)   # hand Z down, fingers along world Y
TABLE = 0.425
BASKET = (0.0, 0.265)
TARGETS = {
    # name: (x, y, grasp_tcp_z, top_z, release_tcp_z)
    "can":   (-0.067, 0.070, 0.470, 0.520, 0.70),
    "cream": (0.107, -0.191, 0.437, 0.454, 0.68),
}

rclpy.init()
node = rclpy.create_node("pick_place")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.update(m=m), 10)
ik_cli = node.create_client(GetPositionIK, "/compute_ik")
fk_cli = node.create_client(GetPositionFK, "/compute_fk")
fjt = ActionClient(node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
grip = ActionClient(node, GripperCommand, "/franka_gripper/gripper_action")
ik_cli.wait_for_service(20); fk_cli.wait_for_service(20)
fjt.wait_for_server(20); grip.wait_for_server(20)


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def joints(fresh=True):
    if fresh:
        js.pop("m", None)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    d = dict(zip(js["m"].name, js["m"].position))
    return [d[j] for j in JOINTS], (d["panda_finger_joint1"], d["panda_finger_joint2"])


def fk(q):
    req = GetPositionFK.Request()
    req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state = JointState(name=JOINTS, position=q)
    fut = fk_cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
    p = fut.result().pose_stamped[0].pose
    x, y, z, w = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    zaxis = np.array([2*(x*z + y*w), 2*(y*z - x*w), 1 - 2*(x*x + y*y)])
    tcp = np.array([p.position.x, p.position.y, p.position.z]) + TCP * zaxis
    return tcp


def ik(tcp_xyz, q=Q_DOWN, seed=None):
    """TCP pose (world) -> arm joints, or None."""
    x, y, z, w = q
    zaxis = np.array([2*(x*z + y*w), 2*(y*z - x*w), 1 - 2*(x*x + y*y)])
    hand = np.array(tcp_xyz) - TCP * zaxis
    req = GetPositionIK.Request()
    r = req.ik_request
    r.group_name = "panda_arm"
    r.ik_link_name = "panda_hand"
    r.pose_stamped.header.frame_id = ""
    r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = map(float, hand)
    r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y = x, y
    r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = z, w
    r.robot_state.joint_state = JointState(name=JOINTS, position=seed or joints()[0])
    r.avoid_collisions = False
    r.timeout = Duration(sec=2)
    for attempt in range(3):
        fut = ik_cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
        res = fut.result()
        if res is not None and res.error_code.val == 1:
            sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            return [sol[j] for j in JOINTS]
        log(f"  IK attempt {attempt} failed:", None if res is None else res.error_code.val)
    return None


def move(q, secs):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    pt = JointTrajectoryPoint(positions=[float(v) for v in q])
    pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
    goal.trajectory.points = [pt]
    for attempt in range(3):
        fut = fjt.send_goal_async(goal); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
        rf = fut.result().get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        cur, _ = joints()
        err = max(abs(a - b) for a, b in zip(cur, q))
        log(f"  move done code={code} max_joint_err={err:.4f} tcp={np.round(fk(cur),3)}")
        if err < 0.02:
            return True
    return False


def gripper(width):
    g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
    fut = grip.send_goal_async(g); rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    rf = fut.result().get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=300)
    r = rf.result().result
    _, f = joints()
    log(f"  gripper({width}) reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
    return f


def goto_tcp(xyz, secs, q=Q_DOWN):
    log(f" goto tcp {np.round(xyz,3)}")
    sol = ik(xyz, q)
    if sol is None:
        raise SystemExit(f"IK failed for {xyz}")
    if not move(sol, secs):
        raise SystemExit(f"move failed for {xyz}")


def pick_and_place(name):
    x, y, gz, top, rz = TARGETS[name]
    log(f"=== {name} ===")
    gripper(0.04)
    cur = fk(joints()[0])
    goto_tcp((cur[0], cur[1], 0.72), 3.0) if cur[2] < 0.70 else None
    goto_tcp((x, y, top + 0.12), 4.0)            # pre-grasp above
    goto_tcp((x, y, gz), 2.5)                    # descend
    f = gripper(0.0)
    if f[0] < 0.004:
        log("  !! closed on air")
        return False
    goto_tcp((x, y, 0.72), 2.5)                  # lift
    _, f = joints()
    log(f"  after lift fingers={f[0]:.4f} (still holding: {f[0] > 0.004})")
    goto_tcp((BASKET[0], BASKET[1], 0.72), 4.0)  # over basket
    goto_tcp((BASKET[0], BASKET[1], rz), 2.0)
    gripper(0.04)
    goto_tcp((BASKET[0], BASKET[1], 0.75), 2.0)  # retreat
    return True


names = sys.argv[1:] or ["can", "cream"]
log("start joints", np.round(joints()[0], 3), "tcp", np.round(fk(joints()[0]), 3))
for n in names:
    ok = pick_and_place(n)
    log(f"=== {n}: {'OK' if ok else 'FAILED'}")
    if not ok:
        break
rclpy.shutdown()
OPENRUA_EOF

# openrua op 16
cd /workspace; cat > iktest2.py <<'EOF'
import sys; sys.argv=["x","none"]
src=open("pick_place.py").read().split("names = sys.argv")[0]
exec(src)
for tgt in [(-0.067,0.070,0.64),(-0.067,0.070,0.47),(0.107,-0.191,0.437),(0.0,0.265,0.70),(0.0,0.265,0.72)]:
    sol=ik(tgt)
    if sol is None: print(tgt,"NO IK"); continue
    print(tgt,"-> joints",np.round(sol,3),"fk tcp",np.round(fk(sol),4))
rclpy.shutdown()
EOF
timeout 300 python3 iktest2.py

# openrua op 17
cd /workspace; nohup python3 -u pick_place.py can > run_can.log 2>&1 &
sleep 45; cat run_can.log

# openrua op 18
cd /workspace; sleep 60; cat run_can.log | tail -8

# openrua op 19
cd /workspace; sleep 15; tail -3 run_can.log; pgrep -f pick_place.py || echo finished; python3 tools/perception/cam_snap.py agentview after_can.png

# openrua op 20
cd /workspace; nohup python3 -u pick_place.py cream > run_cream.log 2>&1 &
sleep 75; cat run_cream.log

# openrua op 21
cd /workspace; sleep 40; tail -6 run_cream.log; pgrep -f pick_place.py || echo finished

# openrua op 22
cd /workspace; sleep 3; pgrep -f pick_place.py || echo finished; python3 tools/perception/cam_snap.py agentview final_agent.png; python3 tools/perception/cam_snap.py birdview final_bird.png; python3 scan.py birdview 0.435 2>&1 | grep -v "^  #1 "

# openrua op 23
cd /workspace; python3 -c "
import cv2
im=cv2.imread('final_agent.png'); cv2.imwrite('basket_zoom.png', cv2.resize(im[200:330,440:630], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))"; python3 tools/perception/cam_snap.py frontview final_front.png

# openrua op 24
cd /workspace; python3 -c "
import cv2
im=cv2.imread('final_front.png'); cv2.imwrite('basket_zoom2.png', cv2.resize(im[170:270,360:480], None, fx=5, fy=5, interpolation=cv2.INTER_CUBIC))"
