# -----------------------------------------------------------------------------
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Basalt VIO on any set of ZED X of the AMIAD rig (Isaac Sim or a bag of it):
#   ros2 launch basalt_ros vio_composed_zedx_pairs.launch.py camera:=rear,right
# camera: comma separated front, rear, left, right, sorted in this order (duplicates removed),
# image_0.. = each ZED, left then right.
# The calibration is extracted from zedx_4stereo_calib[_zed].json (cameras in SOURCE_ORDER)
# and written to <tmp>/basalt_zedx_<arms>_calib[_zed].json, unless calib_file is given.
# Sim only: the IMU is the Pulse80 (/pulse80/imu, pulse80_imu_link).
#

import json
import os
import tempfile

import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


# ZEDs of the rig, in the order the selected ones are given to basalt
ARMS = ('front', 'rear', 'left', 'right')
# order of the stereo pairs in zedx_4stereo_calib[_zed].json (image 2i = left, 2i+1 = right)
SOURCE_ORDER = ('front', 'left', 'rear', 'right')
# calibration entries with one element per camera
PER_CAMERA = ('T_imu_cam', 'intrinsics', 'resolution')


def pkg_file(fname):
    # an absolute fname (generated calibration) replaces the package config folder
    return PathJoinSubstitution(
        [FindPackageShare('basalt_ros'), 'config', fname])


def parse_arms(value):
    """'right, rear,rear' -> ['rear', 'right'] (rig order, no duplicates)."""
    names = {a.strip() for a in value.split(',') if a.strip()}
    unknown = sorted(names - set(ARMS))
    if unknown or not names:
        raise RuntimeError(f"camera must be a comma separated list of {list(ARMS)}, got '{value}'")
    return [a for a in ARMS if a in names]


def make_calib(arms, zed_mode):
    """Write the calibration of arms (from the 4 stereo one) and return its path."""
    suffix = '_zed' if zed_mode else ''
    src = os.path.join(get_package_share_directory('basalt_ros'), 'config',
                       f'zedx_4stereo_calib{suffix}.json')
    with open(src) as f:
        calib = json.load(f)
    value = calib['value0']
    n = len(value['T_imu_cam'])
    if n != 2 * len(SOURCE_ORDER):
        raise RuntimeError(f'{src}: {n} cameras, expected {2 * len(SOURCE_ORDER)}')
    cams = [2 * SOURCE_ORDER.index(arm) + side for arm in arms for side in (0, 1)]
    keys = list(PER_CAMERA)
    if len(value.get('vignette', [])) == n:   # empty in the AMIAD calibrations
        keys.append('vignette')
    for key in keys:
        value[key] = [value[key][i] for i in cams]
    out = os.path.join(tempfile.gettempdir(), f'basalt_zedx_{"_".join(arms)}_calib{suffix}.json')
    with open(out, 'w') as f:
        json.dump(calib, f, indent=4)
    return out


def launch_setup(context, *args, **kwargs):
    """Create container."""
    use_sim_time = LaunchConfig('use_sim_time').perform(context).lower() == 'true'
    # ros: sim images (rgb8, made gray by the frontend); zed: ZED wrapper gray images
    zed_mode = LaunchConfig('camera_mode').perform(context).lower() == 'zed'
    # '' when an including launch (vio_algorithms) declares camera without setting it
    arms = parse_arms(LaunchConfig('camera').perform(context) or 'front,rear')
    # the ZED SDK rectification has its own intrinsics (fx 370.8 vs 336.1 for the sim pinhole)
    calib_file = LaunchConfig('calib_file').perform(context) or make_calib(arms, zed_mode)
    # the ZED streams are stamped independently: looser sync between the ZEDs
    sync_tolerance = float(LaunchConfig('sync_tolerance').perform(context)
                           or (0.02 if zed_mode and len(arms) > 1 else 0.005))
    common_parameters = \
        {'calibration_file': pkg_file(calib_file),
         'vio_config_file': pkg_file(LaunchConfig('config_file')),
         'use_sim_time': use_sim_time}
    frontend_parameters = {
        **common_parameters,
        **{'sync_tolerance': sync_tolerance}}
    backend_parameters = {**common_parameters,
                          **{'has_split_accel_and_gyro_topics': False,
                             'world_frame_id': LaunchConfig('world_frame'),
                             'child_frame_id': 'pulse80_imu_link',
                             'base_frame_id': LaunchConfig('base_frame'),
                             'publish_tf': LaunchConfig('publish_tf'),
                             'align_to_start': LaunchConfig('align_to_start').perform(context).lower() == 'true',
                             'align_frame': LaunchConfig('align_frame')}}

    frontend_remappings = []
    for arm in arms:
        for side in ('left', 'right'):
            frontend_remappings.append(
                (f'image_{len(frontend_remappings)}',
                 f'/zed_{arm}/zed_node/{side}/gray/rect/image' if zed_mode
                 else f'/zed_{arm}/zed_node/{side}/image_rect_color'))
    backend_remappings = [
        ('optical_flow', LaunchConfig('flow_topic')),
        ('imu', LaunchConfig('imu_topic'))]

    if LaunchConfig('composed').perform(context).lower() != 'true':
        # Separate processes: no load_node service handshake, which can time
        # out under heavy DDS discovery (Isaac Sim + several ZED) and then leaves
        # the backend unloaded.
        return [
            Node(package='basalt_ros', executable='vio_frontend_node',
                 name='vio_frontend', namespace='basalt',
                 parameters=[frontend_parameters],
                 remappings=frontend_remappings, output='screen'),
            Node(package='basalt_ros', executable='vio_backend_node',
                 name='vio_backend', namespace='basalt',
                 parameters=[backend_parameters],
                 remappings=backend_remappings, output='screen'),
        ]

    container = ComposableNodeContainer(
            name='basalt_vio_container',
            namespace='basalt',
            package='rclcpp_components',
            executable='component_container',
            parameters=[{'use_sim_time': use_sim_time}],
            composable_node_descriptions=[
                ComposableNode(
                    package='basalt_ros',
                    plugin='basalt_ros::VIOFrontEndNode',
                    name='vio_frontend',
                    namespace='basalt',
                    parameters=[frontend_parameters],
                    remappings=frontend_remappings,
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='basalt_ros',
                    plugin='basalt_ros::VIOBackEndNode',
                    name='vio_backend',
                    namespace='basalt',
                    parameters=[backend_parameters],
                    remappings=backend_remappings,
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen')
    return [container]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription([
        LaunchArg('camera', default_value=['front,rear'],
                  description='ZED X stereo pairs, comma separated: front, rear, left, right '
                  '(sorted in this order)'),
        LaunchArg('calib_file', default_value=[''],
                  description='camera calibration file, cameras in the sorted camera order '
                  '(default: extracted from zedx_4stereo_calib[_zed].json)'),
        LaunchArg('config_file', default_value=['zedx_vio_config_best2.json'],
                  description='vio config file'),
        LaunchArg('flow_topic',
                  default_value=['optical_flow'],
                  description='optical flow topic'),
        LaunchArg('imu_topic',
                  default_value=['/pulse80/imu'],
                  description='combined accel/gyro topic (Isaac Sim Pulse80)'),
        LaunchArg('base_frame', default_value=['base_link'],
                  description='frame published as child of world_frame '
                  '(empty: the IMU frame)'),
        LaunchArg('publish_tf', default_value=['true'],
                  description='broadcast world_frame -> base_frame on TF'),
        LaunchArg('align_to_start', default_value=['true'],
                  description='world frame = first pose (zero position and yaw, gravity aligned), '
                  'as the Isaac Sim /odom and cuVSLAM; false: basalt gravity-aligned IMU frame'),
        LaunchArg('align_frame', default_value=[''],
                  description='TF frame (e.g. base_link_gt, Isaac Sim ground truth) whose pose in '
                  'world_frame sets the first basalt pose: same reference as that odometry. '
                  'Overrides align_to_start'),
        LaunchArg('world_frame',
                  default_value=['odom'],
                  description='frame of the published VIO odometry'),
        LaunchArg('sync_tolerance', default_value=[''],
                  description='max stamp difference [s] of the images of a frame '
                  '(default 0.005, 0.02 with camera_mode:=zed and several ZEDs)'),
        LaunchArg('use_sim_time', default_value=['true'],
                  description='use the Isaac Sim /clock'),
        LaunchArg('camera_mode', default_value=['zed'],
                  description='ros: sim /zed_<arm>/zed_node/<side>/image_rect_color; '
                  'zed: ZED wrapper /zed_<arm>/zed_node/<side>/gray/rect/image'),
        LaunchArg('composed', default_value=['false'],
                  description='true: one component container (intra-process); '
                  'false: frontend and backend as separate processes'),
        OpaqueFunction(function=launch_setup)
        ])
