#!/usr/bin/env python3
"""Resolve every PCR scene/formation and optionally compare the local reference."""

import argparse
import importlib.util
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / 'src/planner/plan_manage'


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check(reference=None):
    # Both repositories use ego_planner; all includes here must resolve to PCR.
    os.environ['ROS_PACKAGE_PATH'] = str(WORKSPACE / 'src') + ':/opt/ros/noetic/share'
    import roslaunch
    from roslaunch.xmlloader import XmlLoader

    pcr = load_module('pcr_launcher', PACKAGE / 'scripts/launch_pcr_formation.py')
    ref = None
    if reference:
        root = reference / 'swarm-playground/formation_ws'
        ref = load_module('reference_launcher', root / 'src/planner/plan_manage/scripts/launch_distributed_formation.py')
        for relative in ('src/uav_simulator/map_generator/src/fixed_split_merge_wall_map.cpp',
                         'src/uav_simulator/map_generator/src/sparse_random_obstacle_map.cpp',
                         'src/Utils/odom_visualization/meshes/fake_drone.dae'):
            assert (WORKSPACE / relative).read_bytes() == (root / relative).read_bytes(), relative
    baseline = yaml.safe_load((PACKAGE / 'config/rectangle_8uav_pcr.yaml').read_text())
    for scene in ('fixed_wall', 'sparse_random', 'cluttered_random'):
        for rows, columns in ((2, 3), (2, 4), (3, 4), (4, 4)):
            count = rows * columns
            # Resolve the user-facing entry, then expand the runtime child XML.
            outer = roslaunch.config.ROSLaunchConfig()
            XmlLoader().load(str(PACKAGE / 'launch/pcr_formation.launch'), outer,
                             argv=['scene:=' + scene, 'formation_rows:=' + str(rows),
                                   'formation_columns:=' + str(columns), 'rviz:=false'], verbose=False)
            prefix = '/pcr_formation_launcher/'
            overrides = {key[len(prefix):]: v.value for key, v in outer.params.items() if key.startswith(prefix)}
            params = pcr.normalized_params(overrides)
            child_xml = pcr.build_launch_xml(params)
            config = roslaunch.config.ROSLaunchConfig()
            XmlLoader().load_string(child_xml, config, verbose=False)
            values = {name: value.value for name, value in config.params.items()}
            assert len(config.nodes) == 1 + 5 * count
            for drone in range(count):
                prefix = '/drone_%d_ego_planner_node/' % drone
                assert values[prefix + 'size_swarm'] == count
                assert values[prefix + 'initial_swarm_size'] == count
                assert values[prefix + 'sparse_num'] == count
                for slot in range(count):
                    row, col = divmod(slot, columns)
                    expected = [(0.5 * (rows - 1) - row) * params['row_spacing'],
                                (col - 0.5 * (columns - 1)) * params['column_spacing'], 0.0]
                    actual = [values[prefix + 't1_relative_pos_%d/%s' % (slot, axis)] for axis in 'xyz']
                    assert actual == expected
                    assert values[prefix + 'sparse_%d' % slot] == slot
                row, col = divmod(drone, columns)
                assert values['/drone_%d_poscmd_2_odom/init_x' % drone] == params['start_x'] + (0.5 * (rows - 1) - row) * params['row_spacing']
                assert values['/drone_%d_poscmd_2_odom/init_y' % drone] == (col - 0.5 * (columns - 1)) * params['column_spacing']
                assert values[prefix + 'global_goal/swarm_central_pos/x'] == params['goal_x']
                assert values[prefix + 'global_goal/swarm_central_pos/z'] == params['altitude']
                for key, value in baseline['optimization'].items():
                    assert values[prefix + 'optimization/' + key] == value, key
                assert values[prefix + 'manager/max_vel'] == params['max_vel']
                assert values[prefix + 'search/astar_time_limit'] == params['astar_time_limit']
                assert values[prefix + 'fsm/auto_start'] is True
                assert values['/drone_%d_pcl_render_node/full_360_sensing' % drone] is True
            rviz = pcr.build_rviz_config(params)
            groups = [d for d in rviz['Visualization Manager']['Displays'] if d['Name'].startswith('drone')]
            assert [g['Name'] for g in groups] == ['drone%d' % i for i in range(count)]
            for drone, group in enumerate(groups):
                topic_ids = re.findall(r'/drone_(\d+)_', yaml.safe_dump(group))
                assert topic_ids and set(topic_ids) == {str(drone)}
            if ref:
                reference_params = dict(scene=scene, formation_rows=rows, formation_columns=columns, rviz=False)
                reference_xml = ET.fromstring(ref.build_launch_xml(reference_params))
                expected_map = {p.get('name'): p.get('value') for p in reference_xml.find('node').findall('param')}
                actual_map = {p.get('name'): p.get('value') for p in ET.fromstring(child_xml).find('node').findall('param')}
                assert actual_map == expected_map, (scene, actual_map, expected_map)
                assert pcr.formation_offsets(params) == ref.formation_offsets(reference_params)
            print('PASS: %s %dx%d (%d UAVs), %d child nodes' % (scene, rows, columns, count, len(config.nodes)))
    # Check configuration changes and reject malformed geometry before startup.
    changed = pcr.normalized_params(dict(formation_rows='3', formation_columns='4',
        row_spacing='0.8', column_spacing='1.2', altitude='1.7', goal_x='9',
        max_vel='0.7', astar_time_limit='0.8', auto_start='false', rviz='false', odom_topic='test/odom'))
    config = roslaunch.config.ROSLaunchConfig()
    XmlLoader().load_string(pcr.build_launch_xml(changed), config, verbose=False)
    assert config.params['/drone_11_ego_planner_node/optimization/fixed_formation_z'].value == 1.7
    assert config.params['/drone_11_ego_planner_node/fsm/auto_start'].value is False
    assert config.params['/drone_11_ego_planner_node/optimization/max_vel2'].value == 0.7
    assert config.params['/drone_11_ego_planner_node/search/astar_time_limit'].value == 0.8
    visualizer = next(n for n in config.nodes if n.name == 'drone_11_odom_visualization')
    assert ['~odom', 'drone_11_test/odom'] in visualizer.remap_args
    for invalid in (dict(formation_rows=0), dict(formation_rows=1), dict(formation_rows=2.5),
                    dict(formation_rows=5), dict(row_spacing=-1), dict(altitude=4),
                    dict(max_vel=float('nan')), dict(astar_time_limit=0), dict(random_seed=2.5),
                    dict(random_seed=2147483648), dict(small_obstacle_count=-1),
                    dict(start_x=10), dict(goal_x=100), dict(scene='unknown'),
                    dict(auto_start='maybe'), dict(typo=1)):
        try:
            pcr.normalized_params(invalid)
        except (ValueError, TypeError, OverflowError):
            continue
        raise AssertionError('accepted invalid config: %r' % invalid)
    print('PASS: custom geometry/topics, manual start and invalid-configuration rejection')
    if ref:
        print('PASS: map sources, mesh, scene parameters and slots match the local reference')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=Path.home() / 'Codes/EGO-Planner-criterion-validation')
    parser.add_argument('--skip-reference', action='store_true')
    args = parser.parse_args()
    check(None if args.skip_reference else args.reference.resolve())
