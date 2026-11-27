#!/usr/bin/env python3
"""Check valid first trajectories for the legacy scene or a configured PCR fleet."""

import argparse
import json
import math
import os
from pathlib import Path
import socket
import tempfile
import threading
import time

import roslaunch
import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from traj_utils.msg import PolyTraj


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=11359)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--scene", choices=("fixed_wall", "sparse_random", "cluttered_random"),
                        help="Use the reusable PCR launcher; omitted: legacy eight-UAV scene")
    parser.add_argument("--rows", type=int, default=2)
    parser.add_argument("--columns", type=int, default=4)
    parser.add_argument("--manual-start", action="store_true",
                        help="Wait for all odometry streams, then publish the shared trigger")
    parser.add_argument("--min-displacement", type=float, default=0.0,
                        help="Also require every UAV to move this far from its initial odometry pose")
    parser.add_argument("--rviz", action="store_true", help="Include RViz in the startup/movement check")
    args = parser.parse_args()
    if not math.isfinite(args.min_displacement) or args.min_displacement < 0:
        parser.error("--min-displacement must be finite and nonnegative")
    workspace = Path(__file__).resolve().parents[1]
    package = workspace / "src/planner/plan_manage"
    if Path(rospkg.RosPack().get_path("ego_planner")).resolve() != package:
        raise RuntimeError("Source this workspace's devel/setup before running the test.")
    with socket.socket() as probe:
        probe.settimeout(0.5)
        if probe.connect_ex(("127.0.0.1", args.port)) == 0:
            raise RuntimeError(f"Test port {args.port} is already in use.")

    output = Path(tempfile.mkdtemp(prefix="pcr-startup-test-", dir="/tmp"))
    os.environ.update(
        ROS_MASTER_URI=f"http://127.0.0.1:{args.port}",
        ROS_HOSTNAME="127.0.0.1", ROS_IP="127.0.0.1",
        ROS_HOME=str(output / "ros"), ROS_LOG_DIR=str(output / "logs"),
    )
    run_id = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(run_id)
    scene = package / "launch/rectangle_8uav_fixed_split_merge_wall.launch"
    count = 8
    launch_args = ["rviz:=" + str(args.rviz).lower()]
    if args.scene:
        scene = package / "launch/pcr_formation.launch"
        count = args.rows * args.columns
        if args.rows < 2 or args.columns < 2 or count > 16:
            parser.error("Use at least two rows/columns, at most 16 UAVs")
        launch_args += [f"scene:={args.scene}", f"formation_rows:={args.rows}",
                        f"formation_columns:={args.columns}"]
    if args.manual_start:
        launch_args.append("auto_start:=false")
    # Use automatic startup unless the manual trigger path is explicitly tested.
    parent = roslaunch.parent.ROSLaunchParent(
        run_id, [(str(scene), launch_args)], port=args.port,
    )
    counts = [0] * count
    invalid = []
    lock = threading.Lock()
    odom_seen = [False] * count
    initial_positions = [None] * count
    max_displacements = [0.0] * count
    triggered = not args.manual_start

    def receive(message, drone):
        values = list(message.duration) + list(message.coef_x) + list(message.coef_y) + list(message.coef_z)
        valid = (
            bool(message.duration)
            and all(math.isfinite(value) for value in values)
            and all(duration > 0 for duration in message.duration)
            and all(len(axis) == 6 * len(message.duration)
                    for axis in (message.coef_x, message.coef_y, message.coef_z))
        )
        with lock:
            counts[drone] += 1
            if not valid:
                invalid.append(drone)

    def odometry(message, drone):
        position = message.pose.pose.position
        xyz = (position.x, position.y, position.z)
        with lock:
            if not all(math.isfinite(value) for value in xyz):
                invalid.append(drone)
                return
            odom_seen[drone] = True
            if initial_positions[drone] is None:
                initial_positions[drone] = xyz
            distance = math.sqrt(sum((a-b)**2 for a, b in zip(xyz, initial_positions[drone])))
            max_displacements[drone] = max(max_displacements[drone], distance)

    started = time.monotonic()
    try:
        parent.start()
        rospy.init_node("comparison_startup_test", anonymous=True, disable_signals=True)
        subscribers = [
            rospy.Subscriber(f"/drone_{drone}_planning/trajectory", PolyTraj,
                             receive, callback_args=drone, queue_size=10)
            for drone in range(count)
        ]
        trigger = None
        if args.manual_start:
            trigger = rospy.Publisher("/traj_start_trigger", PoseStamped, queue_size=1)
        if args.manual_start or args.min_displacement > 0:
            subscribers += [rospy.Subscriber(f"/drone_{drone}_visual_slam/odom", Odometry,
                            odometry, callback_args=drone, queue_size=1) for drone in range(count)]
        while time.monotonic() - started < args.timeout:
            parent.spin_once()
            with lock:
                if not triggered:
                    if any(counts):
                        raise RuntimeError("A UAV published a trajectory before the manual trigger")
                    if all(odom_seen) and trigger.get_num_connections() >= count:
                        message = PoseStamped()
                        message.header.frame_id = "world"
                        message.header.stamp = rospy.Time.now()
                        message.pose.orientation.w = 1.0
                        trigger.publish(message)
                        triggered = True
                if invalid:
                    raise RuntimeError(f"Invalid trajectory from UAVs {invalid}.")
                if (all(count > 0 for count in counts)
                        and all(distance >= args.min_displacement for distance in max_displacements)):
                    result = {"elapsed_seconds": round(time.monotonic() - started, 2),
                              "trajectory_counts": list(counts), "auto_start": not args.manual_start,
                              "scene": args.scene or "legacy_fixed_wall", "uav_count": count,
                              "rviz": args.rviz,
                              "minimum_displacement": args.min_displacement,
                              "max_displacements": list(max_displacements)}
                    (output / "startup_result.json").write_text(json.dumps(result, indent=2))
                    print(f"PASS: all {count} UAVs published finite trajectories: {result}")
                    print(f"Test logs: {output}")
                    return 0
            time.sleep(0.05)
        missing = [drone for drone, count in enumerate(counts) if count == 0]
        unmoved = [drone for drone, distance in enumerate(max_displacements) if distance < args.min_displacement]
        raise RuntimeError(f"No first trajectory from UAVs {missing}; UAVs below displacement threshold: {unmoved}; test logs: {output}")
    finally:
        if rospy.core.is_initialized():
            rospy.signal_shutdown("startup test finished")
        parent.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
