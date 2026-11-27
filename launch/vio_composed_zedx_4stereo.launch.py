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
# Basalt VIO on the 4 ZED X of the AMIAD rig (Isaac Sim, zed camera mode),
# as 4 stereo pairs: image_0..7 = front, left, rear, right, left then right.
# The ZED wrappers must publish the gray images:
#   ros2 launch amiad_description zed_sim_cameras.launch.py \
#     depth_mode:=NONE param_overrides:='video.publish_gray:=true'
#

import launch
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


# calibration order of the stereo pairs
ARMS = ['front', 'left', 'rear', 'right']


def pkg_file(fname):
    return PathJoinSubstitution(
        [FindPackageShare('basalt_ros'), 'config', fname])


def launch_setup(context, *args, **kwargs):
    """Create container."""
    use_sim_time = LaunchConfig('use_sim_time').perform(context).lower() == 'true'
    # ros: sim images (rgb8, made gray by the frontend); zed: ZED wrapper gray images
    zed_mode = LaunchConfig('camera_mode').perform(context).lower() == 'zed'
    # the ZED SDK rectification has its own intrinsics (fx 370.8 vs 336.1 for the sim pinhole)
    calib_file = (LaunchConfig('calib_file').perform(context)
                  or f'zedx_4stereo_calib{"_zed" if zed_mode else ""}.json')
    # the ZED streams are stamped independently: looser sync between the ZEDs
    sync_tolerance = float(LaunchConfig('sync_tolerance').perform(context)
                           or (0.02 if zed_mode else 0.005))
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
    for arm in ARMS:
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
        # out under heavy DDS discovery (Isaac Sim + 4 ZED) and then leaves
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
        LaunchArg('calib_file', default_value=[''],
                  description='camera calibration file (default zedx_4stereo_calib.json, '
                  'zedx_4stereo_calib_zed.json with camera_mode:=zed)'),
        LaunchArg('config_file', default_value=['zedx_vio_config_best2.json'],
                  description='vio config file'),
        LaunchArg('flow_topic',
                  default_value=['optical_flow'],
                  description='optical flow topic'),
        LaunchArg('imu_topic',
                  default_value=['/pulse80/imu'],
                  description='combined accel/gyro topic (Isaac Sim, zed_front_camera_center)'),
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
                  '(default 0.005, 0.02 with camera_mode:=zed)'),
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
