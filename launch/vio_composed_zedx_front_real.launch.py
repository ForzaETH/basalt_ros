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
# Basalt VIO on the front ZED X of the real AMIAD rig.
# Isaac Sim version: vio_composed_zedx_front.launch.py
#
# ZED wrapper gray images /zed_front/zed_node/{left,right}/gray/rect/image, Epson IMU /imu/data
# (frame imu_link), no sim time, calibration zedx_front_calib_zed_real.json
# (Kalibr camera-IMU calibration with the Epson IMU):
#   ros2 launch amiad_nav onboard.launch.py        # cameras, IMU, rover URDF (imu_link -> base_link TF)
#   ros2 launch basalt_ros vio_composed_zedx_front_real.launch.py
#

import launch
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def pkg_file(fname):
    return PathJoinSubstitution(
        [FindPackageShare('basalt_ros'), 'config', fname])


def launch_setup(context, *args, **kwargs):
    """Create container."""
    use_sim_time = LaunchConfig('use_sim_time').perform(context).lower() == 'true'
    common_parameters = \
        {'calibration_file': pkg_file(LaunchConfig('calib_file').perform(context)),
         'vio_config_file': pkg_file(LaunchConfig('config_file')),
         'use_sim_time': use_sim_time}
    frontend_parameters = common_parameters
    backend_parameters = {**common_parameters,
                          **{'has_split_accel_and_gyro_topics': False,
                             'world_frame_id': LaunchConfig('world_frame'),
                             # fallback only: the frame_id of the IMU messages is used
                             'child_frame_id': LaunchConfig('imu_frame'),
                             'base_frame_id': LaunchConfig('base_frame'),
                             'publish_tf': LaunchConfig('publish_tf')}}

    frontend_remappings = [
        (f'{side}_image', LaunchConfig(f'{side}_image').perform(context))
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
        LaunchArg('left_image', default_value=['/zed_front/zed_node/left/gray/rect/image'],
                  description='left rectified image (mono8)'),
        LaunchArg('right_image', default_value=['/zed_front/zed_node/right/gray/rect/image'],
                  description='right rectified image (mono8)'),
        LaunchArg('calib_file', default_value=['zedx_front_calib_zed_real.json'],
                  description='camera-IMU calibration file in basalt_ros/config'),
        LaunchArg('config_file', default_value=['zedx_vio_config.json'],
                  description='vio config file'),
        LaunchArg('flow_topic',
                  default_value=['optical_flow'],
                  description='optical flow topic'),
        LaunchArg('imu_topic',
                  default_value=['/imu/data'],
                  description='combined accel/gyro topic (Epson IMU)'),
        LaunchArg('imu_frame', default_value=['imu_link'],
                  description='IMU frame used when the IMU messages have no frame_id'),
        LaunchArg('base_frame', default_value=['base_link'],
                  description='frame published as child of world_frame '
                  '(empty: the IMU frame)'),
        LaunchArg('publish_tf', default_value=['true'],
                  description='broadcast world_frame -> base_frame on TF'),
        LaunchArg('world_frame',
                  default_value=['odom'],
                  description='frame of the published VIO odometry'),
        LaunchArg('use_sim_time', default_value=['false'],
                  description='use /clock (true with a bag played with --clock)'),
        OpaqueFunction(function=launch_setup)
        ])
