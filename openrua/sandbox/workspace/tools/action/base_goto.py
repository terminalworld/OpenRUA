#!/usr/bin/env python3
"""Drive the mobile base to a world pose, closed-loop on odometry.

Usage: python3 tools/action/base_goto.py <x> <y> [yaw_rad] \
           [--tol 0.05] [--yaw-tol 0.1]
Streams velocity bursts, re-reads /odom between bursts, corrects.
Exits 0 at the target; exits 3 on stall, printing the LAST POSE as a
fact ("stalled at x y yaw"); why it stalled is yours to find out.
Absent base_twist in the manifest = this machine's base is fixed.
"""
import math
import sys
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def _pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return p.x, p.y, yaw


def _read_odom(node, odom, timeout=10.0):
    got = {}
    sub = node.create_subscription(Odometry, odom,
                                   lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no odometry on {odom}")
    return _pose(got["m"])


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) not in (2, 3):
        raise SystemExit(__doc__)
    gx, gy = float(args[0]), float(args[1])
    gyaw = float(args[2]) if len(args) == 3 else None
    tol = float(sys.argv[sys.argv.index("--tol") + 1]) \
        if "--tol" in sys.argv else 0.05
    ytol = float(sys.argv[sys.argv.index("--yaw-tol") + 1]) \
        if "--yaw-tol" in sys.argv else 0.10

    root = Path(__file__).resolve().parents[2]
    m = yaml.safe_load((root / "machine.yaml").read_text())
    base = next((a for a in m["actuators"] if a["kind"] == "base_twist"), None)
    odom_entry = next((s for s in m["sensors"] if s["kind"] == "odometry"), None)
    if base is None or odom_entry is None:
        raise SystemExit("this machine has no mobile base (see machine.yaml)")

    rclpy.init()
    node = rclpy.create_node("base_goto")
    pub = node.create_publisher(Twist, base["port"], 10)
    latest = {}
    node.create_subscription(Odometry, odom_entry["port"],
                             lambda m: latest.update(p=_pose(m)), 10)
    end = node.get_clock().now().nanoseconds / 1e9 + 15
    while "p" not in latest and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "p" not in latest:
        raise SystemExit(f"no odometry on {odom_entry['port']}")

    # Continuous stream, passive odometry: pauses reset the base's
    # velocity ramp, so publish steadily and let the same spin loop
    # deliver odometry updates on the side.
    no_progress = 0
    last_err = None
    for i in range(20000):
        x, y, yaw = latest["p"]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        dyaw = 0.0
        if gyaw is not None:
            dyaw = math.atan2(math.sin(gyaw - yaw), math.cos(gyaw - yaw))
        if dist < tol and abs(dyaw) < ytol:
            print(f"arrived at {x:.3f} {y:.3f} {yaw:.3f}")
            rclpy.shutdown()
            return
        bvx = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        bvy = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        n = max(dist, 1e-6)
        t = Twist()
        t.linear.x = bvx / n
        t.linear.y = bvy / n
        t.angular.z = max(-1.0, min(1.0, dyaw * 2))
        pub.publish(t)
        rclpy.spin_once(node, timeout_sec=0.03)
        if i % 100 == 99:  # progress check every ~100 ticks
            err = dist + abs(dyaw)
            if last_err is not None and err > last_err - 5e-3:
                no_progress += 1
                if no_progress >= 5:
                    print(f"stalled at {x:.3f} {y:.3f} {yaw:.3f} "
                          f"(remaining {dist:.3f} m, {dyaw:.2f} rad)")
                    rclpy.shutdown()
                    sys.exit(3)
            else:
                no_progress = 0
            last_err = err
    print("cycle budget exhausted before arrival")
    rclpy.shutdown()
    sys.exit(4)


if __name__ == "__main__":
    main()
