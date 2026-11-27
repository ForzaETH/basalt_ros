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
# Basalt VIO on the ZED X of the real AMIAD rig, 1 to 4 stereo pairs.
# Isaac Sim versions: vio_composed_zedx_{front,2stereo,3stereo,4stereo}.launch.py
#
# ZED wrapper gray images /zed_<arm>/zed_node/{left,right}/gray/rect/image, Epson IMU /imu/data
# (frame imu_link), no sim time. Calibration (Kalibr camera-IMU calibrations with the Epson IMU):
# config/<resolution>/<file>, selected by resolution and num_cameras:
#   num_cameras  ZED (calibration order)       calibration file
#   2            front                         zedx_front_calib_zed_real.json
#   4            front, rear                   zedx_2stereo_calib_zed_real.json
#   6            front, rear, left             zedx_3stereo_calib_zed_real.json
#   8            front, rear, left, right      zedx_4stereo_calib_zed_real.json
# resolution: svga (ZED SVGA 960x600) or 1080p (ZED HD1080 1920x1080): must match the ZED wrapper
# grab_resolution.
# cameras:=<list> (overrides num_cameras) selects any set of ZED, e.g. cameras:=left,right or
# cameras:=rear,left,right, sorted front, rear, left, right. A set of the table above uses its file;
# any other set gets a calibration extracted from zedx_4stereo_calib_zed_real.json, written to
# <tmp>/basalt_zedx_real_<resolution>_<arms>_calib.json, with cam_time_offset_ns = mean of the
# Kalibr time offsets of its ZED (PAIR_TIME_OFFSET_NS, same rule as the files above).
#
#   ros2 launch amiad_nav onboard.launch.py        # cameras, IMU, rover URDF (imu_link -> base_link TF)
#   ros2 launch basalt_ros vio_composed_zedx_real.launch.py resolution:=1080p num_cameras:=4
#   ros2 launch basalt_ros vio_composed_zedx_real.launch.py resolution:=svga cameras:=left,right
#

import json
import os
import tempfile

import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import LogInfo, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

RESOLUTIONS = ['svga', '1080p']

# num_cameras -> (stereo pairs in calibration order, calibration file in config/<resolution>)
SETUPS = {
    '2': (['front'], 'zedx_front_calib_zed_real.json'),
    '4': (['front', 'rear'], 'zedx_2stereo_calib_zed_real.json'),
    '6': (['front', 'rear', 'left'], 'zedx_3stereo_calib_zed_real.json'),
    '8': (['front', 'rear', 'left', 'right'], 'zedx_4stereo_calib_zed_real.json'),
}

# ZED of the rig, in calibration order (also the order of zedx_4stereo_calib_zed_real.json)
ARMS = ('front', 'rear', 'left', 'right')
# calibration entries with one element per camera
PER_CAMERA = ('T_imu_cam', 'intrinsics', 'resolution')
# Kalibr timeshift_cam_imu of each ZED (mean of its left and right camera) [ns], recovered from the
# cam_time_offset_ns of the files above (mean over their cameras). The right ZED was not calibrated at
# SVGA (None): left out of the mean, as in svga/zedx_4stereo_calib_zed_real.json.
PAIR_TIME_OFFSET_NS = {
    'svga': {'front': 3051634, 'rear': 3457916, 'left': 2901755, 'right': None},
    '1080p': {'front': 3093210, 'rear': 4662360, 'left': 3781773, 'right': 4952353},
}


def parse_arms(value):
    """'right, rear,rear' -> ['rear', 'right'] (rig order, no duplicates)."""
    names = {a.strip() for a in value.split(',') if a.strip()}
    if not names or names - set(ARMS):
        raise RuntimeError(f"cameras must be a comma separated list of {list(ARMS)}, got '{value}'")
    return [a for a in ARMS if a in names]


def make_calib(resolution, arms):
    """Calibration of arms extracted from the 4 stereo one; returns its absolute path."""
    src = os.path.join(get_package_share_directory('basalt_ros'), 'config', resolution,
                       'zedx_4stereo_calib_zed_real.json')
    with open(src) as f:
        calib = json.load(f)
    value = calib['value0']
    if len(value['T_imu_cam']) != 2 * len(ARMS):
        raise RuntimeError(f'{src}: {len(value["T_imu_cam"])} cameras, expected {2 * len(ARMS)}')
    cams = [2 * ARMS.index(arm) + side for arm in arms for side in (0, 1)]
    keys = list(PER_CAMERA) + (['vignette'] if len(value.get('vignette', [])) == 2 * len(ARMS) else [])
    for key in keys:
        value[key] = [value[key][i] for i in cams]
    offsets = [PAIR_TIME_OFFSET_NS[resolution][a] for a in arms]
    offsets = [o for o in offsets if o is not None] or \
        [o for o in PAIR_TIME_OFFSET_NS[resolution].values() if o is not None]
    value['cam_time_offset_ns'] = round(sum(offsets) / len(offsets))
    out = os.path.join(tempfile.gettempdir(),
                       f'basalt_zedx_real_{resolution}_{"_".join(arms)}_calib.json')
    with open(out, 'w') as f:
        json.dump(calib, f, indent=4)
    return out


def config_with_overrides(config_file, overrides):
    """Copy of the VIO config (config.<key> of value0) with 'key=value;key=value' replaced (JSON /
    YAML typed values, 'config.' optional); returns its absolute path (in the temp folder)."""
    import yaml
    path = config_file if os.path.isabs(config_file) else os.path.join(
        get_package_share_directory('basalt_ros'), 'config', config_file)
    with open(path) as f:
        config = json.load(f)
    value = config['value0']
    for item in [i.strip() for i in overrides.replace(',', ';').split(';') if i.strip()]:
        if '=' not in item:
            raise RuntimeError(f"overrides: expected key=value, got '{item}'")
        key, val = (x.strip() for x in item.split('=', 1))
        key = key if key.startswith('config.') else 'config.' + key
        if key not in value:
            raise RuntimeError(f"overrides: no '{key}' in {path}")
        val = yaml.safe_load(val)
        if isinstance(value[key], float) and isinstance(val, int) and not isinstance(val, bool):
            val = float(val)
        value[key] = val
    fd, out = tempfile.mkstemp(prefix='basalt_vio_config_override_', suffix='.json')
    with os.fdopen(fd, 'w') as f:
        json.dump(config, f, indent=4)
    return out


def pkg_file(fname):
    # an absolute fname (generated calibration) replaces the package config folder
    return PathJoinSubstitution(
        [FindPackageShare('basalt_ros'), 'config', fname])


def launch_setup(context, *args, **kwargs):
    """Create the frontend and backend (container or separate processes)."""
    use_sim_time = LaunchConfig('use_sim_time').perform(context).lower() == 'true'

    resolution = LaunchConfig('resolution').perform(context).strip().lower()
    if resolution not in RESOLUTIONS:
        raise RuntimeError(f"resolution must be one of {RESOLUTIONS}, got '{resolution}'")
    camera_arg = LaunchConfig('cameras').perform(context).strip()
    if camera_arg:
        arms = parse_arms(camera_arg)
        num_cameras = str(2 * len(arms))
        # a standard set uses its calibration file, any other one an extracted calibration
        default_calib = next((f'{resolution}/{f}' for a, f in SETUPS.values() if a == arms), None)
    else:
        num_cameras = LaunchConfig('num_cameras').perform(context).strip()
        if num_cameras not in SETUPS:
            raise RuntimeError(f"num_cameras must be one of {list(SETUPS)}, got '{num_cameras}'")
        arms, default_calib = SETUPS[num_cameras]
        default_calib = f'{resolution}/{default_calib}'
    calib_file = LaunchConfig('calib_file').perform(context).strip() \
        or default_calib or make_calib(resolution, arms)

    # the ZED streams are stamped independently: loose sync between the ZEDs
    sync_tolerance = float(LaunchConfig('sync_tolerance').perform(context))
    config_file = LaunchConfig('config_file').perform(context).strip()
    overrides = LaunchConfig('overrides').perform(context).strip().strip('\'"')
    override_info = []
    if overrides and overrides.lower() != 'none':
        base_config = config_file
        config_file = config_with_overrides(base_config, overrides)
        override_info = [LogInfo(msg=f'[basalt] overrides {overrides} -> {config_file} (from {base_config})')]
    common_parameters = \
        {'calibration_file': pkg_file(calib_file),
         'vio_config_file': pkg_file(config_file),
         'use_sim_time': use_sim_time}
    frontend_parameters = {
        **common_parameters,
        **{'sync_tolerance': sync_tolerance}}
    backend_parameters = {**common_parameters,
                          **{'has_split_accel_and_gyro_topics': False,
                             'world_frame_id': LaunchConfig('world_frame'),
                             # fallback only: the frame_id of the IMU messages is used
                             'child_frame_id': LaunchConfig('imu_frame'),
                             'base_frame_id': LaunchConfig('base_frame'),
                             'publish_tf': LaunchConfig('publish_tf')}}

    # frontend inputs (vio_frontend.cpp): 2 cameras -> left_image, right_image;
    # more -> image_0, image_1, ... in calibration order
    topics = [f'/zed_{arm}/zed_node/{side}/gray/rect/image'
              for arm in arms for side in ('left', 'right')]
    if len(topics) == 2:
        frontend_remappings = list(zip(('left_image', 'right_image'), topics))
    else:
        frontend_remappings = [(f'image_{i}', t) for i, t in enumerate(topics)]
    backend_remappings = [
        ('optical_flow', LaunchConfig('flow_topic')),
        ('imu', LaunchConfig('imu_topic'))]

    actions = [LogInfo(msg=f'[basalt] {resolution}, {num_cameras} cameras ({", ".join(arms)}), '
                           f'calibration {calib_file if os.path.isabs(calib_file) else "config/" + calib_file}')] + override_info

    if LaunchConfig('composed').perform(context).lower() != 'true':
        # Separate processes: no load_node service handshake, which can time
        # out under heavy DDS discovery (Isaac Sim + ZED) and then leaves
        # the backend unloaded.
        return actions + [
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
    return actions + [container]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription([
        LaunchArg('resolution', default_value=['1080p'],
                  description=f'ZED grab resolution {RESOLUTIONS}: selects config/<resolution>'),
        LaunchArg('num_cameras', default_value=['2'],
                  description='2 (front), 4 (+ rear), 6 (+ left) or 8 (+ right): '
                  'ZED X images, 2 per stereo pair'),
        LaunchArg('cameras', default_value=[''],
                  description=f'ZED set, comma separated among {list(ARMS)} (e.g. left,right); '
                  'overrides num_cameras. Empty: num_cameras'),
        LaunchArg('calib_file', default_value=[''],
                  description='camera-IMU calibration file relative to basalt_ros/config '
                  '(overrides resolution and num_cameras; must match num_cameras)'),
        LaunchArg('config_file', default_value=['zedx_vio_config_best2.json'],
                  description='vio config file'),
        LaunchArg('overrides', default_value=[''],
                  description='VIO config values replaced in a copy of config_file, '
                  '"key=value;..." e.g. "vio_obs_std_dev=0.3;vio_max_kfs=10" (parameter sweeps)'),
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
        LaunchArg('sync_tolerance', default_value=['0.02'],
                  description='max stamp difference [s] of the images of a frame'),
        LaunchArg('use_sim_time', default_value=['false'],
                  description='use /clock (true with a bag played with --clock)'),
        LaunchArg('composed', default_value=['false'],
                  description='true: one component container (intra-process); '
                  'false: frontend and backend as separate processes'),
        OpaqueFunction(function=launch_setup)
        ])
