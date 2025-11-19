import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

import lifecycle_msgs.msg
import yaml

from ament_index_python.packages import get_package_share_directory

package_name = 'slam_sim'

def generate_launch_description():

    ld = launch.LaunchDescription()
    this_pkg_path = os.path.join(get_package_share_directory(package_name))
    # ~/colcon_ws/install/slam_sim/share/slam_sim/

    # Load only the `localization` section from the YAML config and pass it
    # as parameters to the node. This allows you to put multiple sections in
    # the same YAML and only use the localization subsection here.
    config_path = os.path.join(this_pkg_path, 'config', 'config.yaml')
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    root = cfg.get("/**", cfg)
    params = root.get("ros__parameters", root)

    lidar = params["lidar"]
    imu   = params["imu"]
    localization_params  = params.get("localization", {})  
    lidar_out_topic = lidar.get('lidar_out_topic', '/lidar/points')
    imu_out_topic   = imu.get('imu_out_topic', '/imu/data')

    localization_params['lidar_topic_in_'] = lidar_out_topic+'_ign'
    localization_params['imu_topic_in_']   = imu_out_topic+'_ign'


    lidar_localization = launch_ros.actions.LifecycleNode(
        name='lidar_localization',
        namespace='',
        package='slam_sim',
        executable='lidar_localization_node',
        # pass the extracted dict as ros parameters; wrap under ros__parameters
        # so the node receives them correctly
        parameters=[localization_params],
        output='screen')

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    simu_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    open_rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(this_pkg_path+"/rviz/localization.rviz")],
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    
    ld.add_action(simu_time)
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)

    ld.add_action(lidar_localization)
    ld.add_action(to_inactive)
    ld.add_action(open_rviz)

    return ld