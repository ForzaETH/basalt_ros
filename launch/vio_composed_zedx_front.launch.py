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
# Basalt VIO on ONE ZED X of the AMIAD rig (Isaac Sim): camera:=front (default), rear, left or right.
# Real robot (Epson IMU, Kalibr calibration): vio_composed_zedx_front_real.launch.py
# Default: zed camera mode (launch_sim_teleop.sh default), ZED wrapper gray images
# /zed_<camera>/zed_node/{left,right}/gray/rect/image,
# calibration zedx_<camera>_calib_zed.json (ZED SDK rectified intrinsics):
#   CAMERA_MODE=zed CAMERAS=front src/amiad_description/scripts/launch_sim_teleop.sh
#   ros2 launch basalt_ros vio_composed_zedx_front.launch.py
#   ros2 launch basalt_ros vio_composed_zedx_front.launch.py camera:=rear   # CAMERAS=rear
# ros camera mode (CAMERA_MODE=ros): the sim publishes /zed_<camera>/zed_node/{left,right}/image_rect_color
# (rgb8, converted to gray by the frontend), calibration zedx_<camera>_calib.json:
#   ros2 launch basalt_ros vio_composed_zedx_front.launch.py camera_mode:=ros
# The zedx_<camera>_calib*.json extrinsics are relative to the Pulse80 IMU (/pulse80/imu,
# pulse80_imu_link), as in the multi-stereo launches.
#

import launch
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# ZED X stereo pairs of the rig, one zedx_<camera>_calib[_zed].json each
CAMERAS = ('front', 'rear', 'left', 'right')


def pkg_file(fname):
    return PathJoinSubstitution(
        [FindPackageShare('basalt_ros'), 'config', fname])


def launch_setup(context, *args, **kwargs):
    """Create container."""
    use_sim_time = LaunchConfig('use_sim_time').perform(context).lower() == 'true'
    # '' when an including launch (vio_algorithms) declares camera without setting it
    camera = LaunchConfig('camera').perform(context).strip() or 'front'
    if camera not in CAMERAS:
        raise RuntimeError(f"camera must be one of {CAMERAS}, got '{camera}'")
    # ros: sim images (rgb8, made gray by the frontend); zed: ZED wrapper gray images
    zed_mode = LaunchConfig('camera_mode').perform(context).lower() == 'zed'
    # the ZED SDK rectification has its own intrinsics (fx 370.8 vs 336.1 for the sim pinhole)
    calib_file = (LaunchConfig('calib_file').perform(context)
                  or f'zedx_{camera}_calib{"_zed" if zed_mode else ""}.json')
    common_parameters = \
        {'calibration_file': pkg_file(calib_file),
         'vio_config_file': pkg_file(LaunchConfig('config_file')),
         'use_sim_time': use_sim_time}
    frontend_parameters = common_parameters
    backend_parameters = {**common_parameters,
                          **{'has_split_accel_and_gyro_topics': False,
                             'world_frame_id': LaunchConfig('world_frame'),
                             'child_frame_id': 'pulse80_imu_link',
                             'base_frame_id': LaunchConfig('base_frame'),
                             'publish_tf': LaunchConfig('publish_tf'),
                             'align_to_start': LaunchConfig('align_to_start').perform(context).lower() == 'true',
                             'align_frame': LaunchConfig('align_frame')}}

    frontend_remappings = [
        (f'{side}_image', LaunchConfig(f'{side}_image').perform(context)
         or (f'/zed_{camera}/zed_node/{side}/gray/rect/image' if zed_mode
             else f'/zed_{camera}/zed_node/{side}/image_rect_color'))
        for side in ('left', 'right')]
    backend_remappings = [
        ('optical_flow', LaunchConfig('flow_topic')),
        ('imu', LaunchConfig('imu_topic'))]

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
        LaunchArg('camera', default_value=['front'],
                  description='ZED X stereo pair: front, rear, left or right '
                  '(sets the image topics and the calibration file)'),
        LaunchArg('camera_mode', default_value=['zed'],
                  description='ros: sim /zed_<camera>/zed_node/<side>/image_rect_color; '
                  'zed: ZED wrapper /zed_<camera>/zed_node/<side>/gray/rect/image'),
        LaunchArg('left_image', default_value=[''],
                  description='left rectified image (mono8 or rgb8/bgr8/rgba8/bgra8), '
                  'default from camera_mode'),
        LaunchArg('right_image', default_value=[''],
                  description='right rectified image (mono8 or rgb8/bgr8/rgba8/bgra8), '
                  'default from camera_mode'),
        LaunchArg('calib_file', default_value=[''],
                  description='camera calibration file (default zedx_<camera>_calib.json, '
                  'zedx_<camera>_calib_zed.json with camera_mode:=zed)'),
        LaunchArg('config_file', default_value=['zedx_vio_config_best2.json'],
                  description='vio config file'),
        LaunchArg('flow_topic',
                  default_value=['optical_flow'],
                  description='optical flow topic'),
        LaunchArg('imu_topic',
                  default_value=['/pulse80/imu'],
                  description='combined accel/gyro topic (Isaac Sim Pulse80, pulse80_imu_link: '
                  'the frame of the calibration extrinsics)'),
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
        LaunchArg('use_sim_time', default_value=['true'],
                  description='use the Isaac Sim /clock'),
        OpaqueFunction(function=launch_setup)
        ])
