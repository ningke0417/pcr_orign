#!/usr/bin/env python3
"""Check all eight planners using the comparison scene's default auto-start."""

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
from traj_utils.msg import PolyTraj


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=11359)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
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
    # Only disable the GUI. Keep the default automatic startup behavior.
    parent = roslaunch.parent.ROSLaunchParent(
        run_id, [(str(scene), ["rviz:=false"])], port=args.port,
    )
    counts = [0] * 8
    invalid = []
    lock = threading.Lock()

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

    started = time.monotonic()
    try:
        parent.start()
        rospy.init_node("comparison_startup_test", anonymous=True, disable_signals=True)
        subscribers = [
            rospy.Subscriber(f"/drone_{drone}_planning/trajectory", PolyTraj,
                             receive, callback_args=drone, queue_size=10)
            for drone in range(8)
        ]
        while time.monotonic() - started < args.timeout:
            parent.spin_once()
            with lock:
                if invalid:
                    raise RuntimeError(f"Invalid trajectory from UAVs {invalid}.")
                if all(count > 0 for count in counts):
                    result = {"elapsed_seconds": round(time.monotonic() - started, 2),
                              "trajectory_counts": list(counts), "auto_start": True}
                    (output / "startup_result.json").write_text(json.dumps(result, indent=2))
                    print(f"PASS: all eight UAVs published finite trajectories: {result}")
                    print(f"Test logs: {output}")
                    return 0
            time.sleep(0.05)
        missing = [drone for drone, count in enumerate(counts) if count == 0]
        raise RuntimeError(f"No first trajectory from UAVs {missing}; test logs: {output}")
    finally:
        if rospy.core.is_initialized():
            rospy.signal_shutdown("startup test finished")
        parent.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
