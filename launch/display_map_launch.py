import os, xacro, yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

package_name = 'slam_sim'

def generate_launch_description():

    this_pkg_path = os.path.join(get_package_share_directory(package_name))
    # ~/colcon_ws/install/slam_sim/share/slam_sim/

    # --- Load config ---   
    config_path = os.path.join(this_pkg_path, 'config', 'config.yaml')
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    
    
    root = cfg.get("/**", cfg)
    params = root.get("ros__parameters", root)

    sim   = params["localization"]


    # Simulation parameters
    map_file_path = sim.get('map_path', 'PCD/test.pcd')


    open_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(this_pkg_path+"/rviz/display_map.rviz")],
    )

    pcd_visualizer = Node(
        package=package_name,
        executable="pcd_visualizer",
        output="screen",
        parameters=[{
            'map_file_path':  map_file_path,
        }],
    )


    return LaunchDescription(
        [
            open_rviz,
            pcd_visualizer,
        ]
    )
