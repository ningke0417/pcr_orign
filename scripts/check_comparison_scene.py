#!/usr/bin/env python3
"""Check the resolved PCR scene against the local distributed reference."""

import argparse
import os
from pathlib import Path
import subprocess
import tempfile

import yaml


SCENE = "rectangle_8uav_fixed_split_merge_wall.launch"


def launch_output(workspace, *arguments):
    env = dict(os.environ)
    env["ROS_PACKAGE_PATH"] = str(workspace / "src") + ":/opt/ros/noetic/share"
    with tempfile.TemporaryDirectory(prefix="pcr-scene-check-") as temp:
        env["ROS_HOME"] = temp
        env["ROS_LOG_DIR"] = temp
        return subprocess.check_output(
            ["roslaunch", *arguments], env=env, text=True
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--reference", type=Path,
        default=Path.home() / "Codes/EGO-Planner-criterion-validation",
        help="Reference repository, checked out on the distributed branch",
    )
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    reference = args.reference.resolve() / "swarm-playground/formation_ws"
    own = yaml.safe_load(launch_output(
        workspace, "--dump-params", "ego_planner", SCENE, "rviz:=false"
    ))
    ref = yaml.safe_load(launch_output(
        reference, "--dump-params", "ego_planner", SCENE,
        "coordination_mode:=distributed", "rviz:=false"
    ))

    def equal(actual, expected, name):
        if actual != expected:
            raise AssertionError(f"{name}: PCR={actual!r}; reference={expected!r}")

    for path in (
        "src/uav_simulator/map_generator/src/fixed_split_merge_wall_map.cpp",
        "src/Utils/odom_visualization/meshes/fake_drone.dae",
    ):
        equal((workspace / path).read_bytes(), (reference / path).read_bytes(), path)

    for key in ref:
        if key.startswith("/fixed_split_merge_wall_map/"):
            equal(own.get(key), ref[key], key)

    keys = (
        "manager/max_vel", "manager/max_acc", "manager/planning_horizon",
        "grid_map/resolution", "grid_map/map_size_x", "grid_map/map_size_y",
        "grid_map/map_size_z", "grid_map/ground_height",
        "grid_map/local_update_range_x", "grid_map/local_update_range_y",
        "grid_map/local_update_range_z", "grid_map/local_map_margin",
        "grid_map/obstacles_inflation", "grid_map/visualization_truncate_height",
        "optimization/obstacle_clearance", "optimization/swarm_clearance",
    )
    original = yaml.safe_load((workspace / "src/planner/plan_manage/config/20.yaml").read_text())
    for drone in range(8):
        planner = f"/drone_{drone}_ego_planner_node/"
        equal(own[planner + "size_swarm"], 8, "swarm size")
        equal(own[planner + "initial_swarm_size"], 8, "initial swarm size")
        equal(own[planner + "search/astar_resolution"], 0.25, "original A* search spacing")
        for key, value in original["optimization"].items():
            if key not in {"swarm_clearance", "fixed_formation_z", "is_opt_formation_z"}:
                equal(own[planner + "optimization/" + key], value, "original PCR " + key)
        for key in keys:
            equal(own[planner + key], ref[planner + key], planner + key)
        for node, fields in (
            ("poscmd_2_odom", ("init_x", "init_y", "init_z")),
            ("odom_visualization", ("robot_scale", "tf45", "color/r", "color/g", "color/b", "color/a")),
            ("pcl_render_node", ("sensing_horizon", "full_360_sensing", "sensing_rate")),
        ):
            for field in fields:
                key = f"/drone_{drone}_{node}/{field}"
                equal(own[key], ref[key], key)
        for slot in range(8):
            slot_key = planner + f"t1_relative_pos_{slot}"
            actual = own.get(slot_key)
            if actual is None:
                actual = {axis: own[slot_key + "/" + axis] for axis in "xyz"}
            equal([actual[axis] for axis in "xyz"], ref[planner + f"formation/drone{slot}"], f"slot {slot}")
        goal = [own[planner + "global_goal/swarm_central_pos/" + axis] for axis in "xyz"]
        equal(goal, [ref[planner + "fsm/waypoint0_" + axis] for axis in "xyz"], "formation goal")

    own_rviz = yaml.safe_load((workspace / "src/planner/plan_manage/launch/comparison_8uav.rviz").read_text())
    ref_rviz = yaml.safe_load((reference / "src/planner/plan_manage/launch/default.rviz").read_text())
    own_manager = own_rviz["Visualization Manager"]
    ref_manager = ref_rviz["Visualization Manager"]
    for key in ("Views", "Global Options"):
        equal(own_manager[key], ref_manager[key], "RViz " + key)
    ref_displays = {display["Name"]: display for display in ref_manager["Displays"]}
    for display in own_manager["Displays"]:
        equal(display, ref_displays[display["Name"]], "RViz display " + display["Name"])

    nodes = launch_output(workspace, "--nodes", "ego_planner", SCENE, "rviz:=false").splitlines()
    equal(len(nodes), 41, "one map plus five nodes per UAV")
    for drone in range(8):
        if f"/drone_{drone}_ego_planner_node" not in nodes:
            raise AssertionError(f"Missing planner {drone}")
    print("PASS: identical wall source/geometry, mesh, RViz camera/displays, eight slots,")
    print("      start/goal positions, motion limits, sensing and map parameters; 41 nodes.")


if __name__ == "__main__":
    main()
