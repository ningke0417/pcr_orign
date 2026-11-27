#!/usr/bin/env python3
"""Build rectangular PCR simulations without per-size launch/YAML copies.

The XML builder is also usable without a ROS master. The runtime launcher only
starts existing PCR members and a map; formation decisions remain in PCR.
"""

import copy
import json
import math
from pathlib import Path
import re
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def normalized_params(params, package_dir=PACKAGE):
    config = yaml.safe_load((Path(package_dir) / 'config/pcr_formation_scenes.yaml').read_text())
    scene = params.get('scene', 'fixed_wall')
    if scene not in config['scenes']:
        raise ValueError('scene must be fixed_wall, sparse_random or cluttered_random')
    defaults = dict(config['common'], **config['scenes'][scene])
    unknown = set(params) - set(defaults) - {'scene'}
    if unknown:
        raise ValueError('unknown formation parameters: %s' % sorted(unknown))
    result = dict(defaults, scene=scene)
    # Empty launch args inherit the YAML value, so scene defaults live once.
    for name, default in defaults.items():
        value = params.get(name, default)
        if value == '':
            value = default
        if isinstance(default, bool):
            if str(value).lower() not in ('true', 'false', '1', '0'):
                raise ValueError('%s must be true or false' % name)
            value = str(value).lower() in ('true', '1')
        elif isinstance(default, (int, float)):
            if isinstance(value, bool) or not math.isfinite(float(value)):
                raise ValueError('%s must be finite and numeric' % name)
            if isinstance(default, int):
                if int(float(value)) != float(value):
                    raise ValueError('%s must be an integer' % name)
                value = int(float(value))
            else:
                value = float(value)
        else:
            value = str(value)
        result[name] = value
    p = result
    rows, columns = p['formation_rows'], p['formation_columns']
    # ICP requires a non-collinear formation and at least four correspondences.
    if rows < 2 or columns < 2 or rows * columns > 16:
        raise ValueError('use at least two rows and two columns, with at most 16 UAVs')
    for name in ('row_spacing', 'column_spacing', 'max_vel', 'max_acc', 'astar_time_limit',
                 'map_size_x', 'map_size_y', 'map_size_z', 'altitude',
                 'wall_size_x', 'wall_size_y', 'second_wall_size_x',
                 'second_wall_total_size_y', 'second_wall_gap_size_y',
                 'small_size_min', 'small_size_max', 'small_center_y_limit',
                 'narrow_gap_size_y'):
        if p[name] <= 0:
            raise ValueError('%s must be positive' % name)
    for name in ('safety_distance', 'large_obstacle_count', 'small_obstacle_count',
                 'endpoint_clearance', 'boundary_clearance', 'obstacle_separation',
                 'large_longitudinal_separation', 'goal_side_obstacle_size_y'):
        if p[name] < 0:
            raise ValueError('%s must be nonnegative' % name)
    if not 0 <= p['random_seed'] <= 2147483647 or not -1 <= p['small_position_seed'] <= 2147483647:
        raise ValueError('seeds must fit ROS int32 (small_position_seed also accepts -1)')
    if p['small_size_min'] > p['small_size_max']:
        raise ValueError('small_size_min must not exceed small_size_max')
    if p['second_wall_gap_size_y'] >= p['second_wall_total_size_y']:
        raise ValueError('second wall gap must be smaller than its total width')
    if p['start_x'] >= p['goal_x']:
        raise ValueError('start_x must be less than goal_x')
    half_x = 0.5 * (rows - 1) * p['row_spacing']
    half_y = 0.5 * (columns - 1) * p['column_spacing']
    if (max(abs(p['start_x']), abs(p['goal_x'])) + half_x >= 0.5 * p['map_size_x'] - 1e-4
            or half_y >= 0.5 * p['map_size_y'] - 1e-4
            or p['altitude'] >= p['map_size_z'] - 0.01 - 1e-4):
        raise ValueError('initial/goal slots must fit inside the PCR map')
    if scene != 'fixed_wall' and p['goal_x'] - p['start_x'] <= 2 * p['endpoint_clearance']:
        raise ValueError('endpoint_clearance leaves no space for random obstacles')
    suffix = p['odom_topic']
    if not suffix or suffix.startswith('/') or any(c.isspace() for c in suffix):
        raise ValueError('odom_topic must be a relative topic suffix')
    return result


def formation_offsets(params, package_dir=PACKAGE):
    p = normalized_params(params, package_dir)
    return [[(0.5 * (p['formation_rows'] - 1) - row) * p['row_spacing'],
             (column - 0.5 * (p['formation_columns'] - 1)) * p['column_spacing'], 0.0]
            for row in range(p['formation_rows'])
            for column in range(p['formation_columns'])]


def _text(value):
    return str(value).lower() if isinstance(value, bool) else str(value)


def _param(parent, name, value):
    kind = ('bool' if isinstance(value, bool) else 'int' if isinstance(value, int)
            else 'double' if isinstance(value, float) else 'str')
    ET.SubElement(parent, 'param', name=name, value=_text(value), type=kind)


def map_parameters(p):
    """Match the reference map generator's parameters, independently of fleet size."""
    if p['scene'] == 'fixed_wall':
        return dict(first_wall_x=p['wall_x'], first_wall_y=p['wall_y'],
                    first_wall_size_x=p['wall_size_x'], first_wall_size_y=p['wall_size_y'],
                    second_wall_x=p['second_wall_x'], second_wall_y=p['wall_y'],
                    second_wall_size_x=p['second_wall_size_x'],
                    second_wall_total_size_y=p['second_wall_total_size_y'],
                    second_wall_gap_size_y=p['second_wall_gap_size_y'],
                    safety_distance=p['safety_distance'], resolution=0.1,
                    height=p['map_size_z'], randomize=False, random_seed=1)
    result = {name: p[name] for name in (
        'small_position_seed', 'large_obstacle_count', 'small_obstacle_count',
        'map_size_y', 'start_x', 'goal_x', 'endpoint_clearance', 'boundary_clearance',
        'obstacle_separation', 'large_longitudinal_separation', 'small_center_y_limit',
        'structured_challenge_enabled', 'stacked_cluster_x', 'narrow_gap_x',
        'narrow_gap_size_y', 'small_size_min', 'small_size_max')}
    result.update(seed=p['random_seed'], stacked_cluster_y=-0.3,
                  stacked_block_size=1.2, stacked_block_step=0.75,
                  narrow_gap_wall_size_x=0.8, narrow_gap_segment_size_y=4.6,
                  large_size_x_min=0.6, large_size_x_max=1.2,
                  large_size_y_min=2.5, large_size_y_max=4.5,
                  goal_side_large_size_y_max=p['goal_side_obstacle_size_y'],
                  center_goal_side_large=True,
                  goal_side_large_center_y=p['goal_side_obstacle_center_y'],
                  resolution=0.1, height=p['map_size_z'])
    return result


def build_launch_xml(params, package_dir=PACKAGE, rviz_config=None):
    p = normalized_params(params, package_dir)
    offsets = formation_offsets(p, package_dir)
    launch = ET.Element('launch')
    executable = ('fixed_split_merge_wall_map' if p['scene'] == 'fixed_wall'
                  else 'sparse_random_obstacle_map')
    node = ET.SubElement(launch, 'node', pkg='map_generator', type=executable,
                         name=executable, output='screen', required='true')
    ET.SubElement(node, 'remap', {'from': 'global_cloud', 'to': '/map_generator/global_cloud'})
    for name, value in map_parameters(p).items():
        _param(node, name, value)
    for drone, offset in enumerate(offsets):
        include = ET.SubElement(launch, 'include', file=str(
            Path(package_dir) / 'launch/comparison_8uav_member.launch'))
        arguments = dict(drone_id=drone, init_x=p['start_x'] + offset[0],
                         init_y=offset[1], start_z=p['altitude'], goal_x=p['goal_x'],
                         max_vel=p['max_vel'], max_acc=p['max_acc'], auto_start=p['auto_start'],
                         safety_distance=p['safety_distance'], odom_topic=p['odom_topic'],
                         map_size_x=p['map_size_x'], map_size_y=p['map_size_y'],
                         map_size_z=p['map_size_z'], swarm_formation_yaml='rectangle_8uav_pcr')
        for name, value in arguments.items():
            ET.SubElement(include, 'arg', name=name, value=_text(value))
    # Applied after includes so YAML defaults cannot overwrite runtime geometry.
    for drone in range(len(offsets)):
        prefix = '/drone_%d_ego_planner_node/' % drone
        shape = dict(size_swarm=len(offsets), initial_swarm_size=len(offsets),
                     sparse_num=len(offsets))
        for slot, offset in enumerate(offsets):
            shape['t1_relative_pos_%d' % slot] = dict(zip('xyz', offset))
            shape['sparse_%d' % slot] = slot
        node = ET.SubElement(launch, 'rosparam', ns=prefix)
        node.text = json.dumps(shape)
        _param(launch, prefix + 'global_goal/swarm_central_pos/y', 0.0)
        _param(launch, prefix + 'global_goal/swarm_scale', 1.0)
        _param(launch, prefix + 'search/astar_time_limit', p['astar_time_limit'])
    if p['rviz']:
        if rviz_config is None:
            raise ValueError('rviz_config is required when rviz is enabled')
        ET.SubElement(launch, 'node', pkg='rviz', type='rviz', name='rviz',
                      args='-d "%s"' % rviz_config, output='screen')
    return ET.tostring(launch, encoding='unicode')


def build_rviz_config(params, package_dir=PACKAGE):
    """Reuse existing robot/path displays, including members above ID seven."""
    p = normalized_params(params, package_dir)
    config = yaml.safe_load((Path(package_dir) / 'launch/comparison_8uav.rviz').read_text())
    manager = config['Visualization Manager']
    groups = {d['Name']: d for d in manager['Displays'] if d['Name'].startswith('drone')}
    manager['Displays'] = [d for d in manager['Displays'] if not d['Name'].startswith('drone')]
    random_geometry = copy.deepcopy(next(d for d in manager['Displays'] if d['Name'] == 'wall_geometry'))
    random_geometry.update(Name='random_obstacles', **{'Marker Topic': '/map_generator/sparse_random_geometry'})
    random_geometry['Namespaces'] = {'sparse_random_obstacles': True}
    manager['Displays'].append(random_geometry)

    def replace(value, old, new):
        if isinstance(value, str):
            return re.sub(r'drone_\d+_', 'drone_%d_' % new, value)
        if isinstance(value, list):
            return [replace(item, old, new) for item in value]
        if isinstance(value, dict):
            return {key: replace(item, old, new) for key, item in value.items()}
        return value

    for drone in range(p['formation_rows'] * p['formation_columns']):
        source = drone % len(groups)
        group = replace(groups['drone%d' % source], source, drone)
        group['Name'] = 'drone%d' % drone
        manager['Displays'].append(group)
    # Retain fixed-wall view; fit the longer random courses into the same view.
    if p['scene'] != 'fixed_wall':
        current = manager['Views']['Current']
        current['Distance'] = max(p['map_size_x'], p['map_size_y']) * 1.4
        current['Focal Point'].update(X=0.5 * (p['start_x'] + p['goal_x']), Y=0.0, Z=p['altitude'])
    return config


def main():
    import roslaunch
    import rospkg
    import rospy

    rospy.init_node('pcr_formation_launcher')
    package = Path(rospkg.RosPack().get_path('ego_planner'))
    params = normalized_params(rospy.get_param('~', {}), package)
    # Expose the resolved defaults to rosparam dump and experiment recorders.
    rospy.set_param('~resolved', params)
    with tempfile.TemporaryDirectory(prefix='pcr-formation-') as temp:
        rviz_config = Path(temp) / 'formation.rviz'
        if params['rviz']:
            rviz_config.write_text(yaml.safe_dump(build_rviz_config(params, package), sort_keys=False))
        xml = build_launch_xml(params, package, rviz_config)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        parent = roslaunch.parent.ROSLaunchParent(uuid, [], roslaunch_strs=[xml])
        rospy.on_shutdown(parent.shutdown)
        rospy.loginfo('Starting PCR %sx%s formation, scene=%s',
                      params['formation_rows'], params['formation_columns'], params['scene'])
        try:
            parent.start()
            parent.spin()
        finally:
            parent.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('Unable to launch PCR formation: %s' % error, file=sys.stderr)
        sys.exit(1)
