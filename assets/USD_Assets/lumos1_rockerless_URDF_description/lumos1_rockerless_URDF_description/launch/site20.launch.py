"""Start Site20 and spawn Lumos on its measured lunar surface."""

import json
import math
import os
from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'lumos1_rockerless_URDF_description'


def start(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = Path(get_package_share_directory(PACKAGE))
    prefix = Path(get_package_prefix(PACKAGE))
    cache = Path(arg('output')).expanduser().resolve()
    preparation = [sys.executable, str(prefix/'lib'/PACKAGE/'prepare_site20.py'),
                    '--package', str(share), '--dem', arg('dem'), '--output', str(cache),
                    '--samples', arg('samples'), '--x', arg('x'), '--y', arg('y'), '--yaw', arg('yaw'),
                    '--crater-count',arg('crater_count'),'--crater-seed',arg('crater_seed'),
                    '--sun-elevation',arg('sun_elevation'),'--sun-azimuth',arg('sun_azimuth'),
                    '--sun-intensity',arg('sun_intensity'),'--texture-size',arg('texture_size')]
    if arg('gui').lower() == 'true':
        vendor = Path(get_package_prefix('gz_rendering_vendor'))
        rendering = os.environ.get('GZ_RENDERING_RESOURCE_PATH',
                                   str(vendor/'opt/gz_rendering_vendor/share/gz/gz-rendering8'))
        preparation.extend(['--rendering-resources',rendering])
    subprocess.run(preparation,check=True)
    spawn = json.loads((cache/'spawn.json').read_text())
    command = ['gz', 'sim', '-r', '-v', '3']
    if arg('gui').lower() != 'true':
        command.append('-s')
    command.append(str(cache/'site20.sdf'))
    environment = {'GZ_SIM_RESOURCE_PATH': str(share.parent)+os.pathsep+os.environ.get('GZ_SIM_RESOURCE_PATH', '')}
    if 'rendering_resources' in spawn:
        environment['GZ_RENDERING_RESOURCE_PATH'] = spawn['rendering_resources']
    server = ExecuteProcess(cmd=command, output='screen', additional_env=environment)
    rover = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'site20', '-file', str(cache/'lumos.urdf'),
                            '-name', 'lumos', '-allow_renaming', 'false',
                            '-x', str(spawn['x']), '-y', str(spawn['y']), '-z', str(spawn['z']), '-Y', str(spawn['yaw'])])
    actions = [
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))])),
        server,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description': (cache/'lumos.urdf').read_text()}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share/'config/bridge.yaml')}], output='screen'),
    ]
    if arg('gui').lower() == 'true':
        # Set the view after the GUI scene is ready, overriding a saved overview camera.
        pitch, yaw = 0.48, 3*math.pi/4
        qx, qy = -math.sin(pitch/2)*math.sin(yaw/2), math.sin(pitch/2)*math.cos(yaw/2)
        qz, qw = math.cos(pitch/2)*math.sin(yaw/2), math.cos(pitch/2)*math.cos(yaw/2)
        request = (f'pose: {{position: {{x: {spawn["x"]+1.2}, y: {spawn["y"]-1.2}, z: {spawn["z"]+0.9}}}, '
                   f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}')
        focus = ExecuteProcess(cmd=['gz', 'service', '-s', '/gui/move_to/pose',
                                    '--reqtype', 'gz.msgs.GUICamera', '--reptype', 'gz.msgs.Boolean',
                                    '--timeout', '30000', '--req', request], output='screen')
        actions.append(RegisterEventHandler(OnProcessExit(target_action=rover, on_exit=[focus])))
    # create waits for the world's creation service; no startup sleep is needed.
    return [*actions, rover]


def generate_launch_description():
    root = Path(os.environ.get('OMNILRS_ROOT', Path.cwd()))
    defaults = {
        'dem': (str(root/'assets/Terrains/SouthPole/Site20_final_adj_5mpp_surf/dem.npy'), 'Existing Site20 dem.npy with adjacent dem.yaml'),
        'output': (str(root/'build/gazebo_site20'), 'Generated world, heightmap and spawn pose'),
        'samples': ('0', '0: full map; 257/513/1025/2049: centered crop, still 5 m/pixel'),
        'gui': ('true', 'Start Gazebo GUI'),
        'x': ('0', 'Spawn X in meters from the center of Site20'),
        'y': ('0', 'Spawn Y in meters toward the top of the DEM raster'),
        'yaw': ('0', 'Spawn yaw in radians; zero faces +X'),
        'crater_count': ('64', 'Number of synthetic craters; 0 keeps only the measured DEM'),
        'crater_seed': ('20', 'Repeatable crater seed, or -1 for a fresh random layout'),
        'sun_elevation': ('5', 'Sun elevation above the grid horizon, in degrees'),
        'sun_azimuth': ('135', 'Direction toward the sun, degrees from +X toward +Y in the DEM grid'),
        'sun_intensity': ('3', 'Gazebo light multiplier (not calibrated solar irradiance)'),
        'texture_size': ('2', 'Regolith texture tile size in meters'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value, description=description)
          for name, (value, description) in defaults.items()],
        OpaqueFunction(function=start),
    ])
