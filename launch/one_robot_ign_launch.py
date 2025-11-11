import os, xacro, yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

package_name = 'ackermann_slam_sim'

def generate_launch_description():

    this_pkg_path = os.path.join(get_package_share_directory(package_name))
    # ~/colcon_ws/install/ackermann_slam_sim/share/ackermann_slam_sim/

    # --- Load config ---   
    config_path = os.path.join(this_pkg_path, 'config', 'config.yaml')
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    
    
    root = cfg.get("/**", cfg)
    params = root.get("ros__parameters", root)

    sim   = params["simulation"]
    lidar = params["lidar"]
    imu   = params["imu"]

    # Simulation parameters
    robot_model = sim.get('robot_model', 'ackermann')
    robot_ns = sim.get('robot_ns', 'r1')
    pose = sim.get('pose', ['1.0', '0.0', '0.0', '0.0'])
    robot_base_color = sim.get('robot_base_color', '0.0 0.0 1.0 0.95')
    world_file = sim.get('world_file', 'depot.sdf')

    # Lidar parameters
    lidar_frequency = lidar.get('lidar_frequency', 10.0)
    period = lidar.get('period', 0.1)
    lidar_out_topic = lidar.get('lidar_out_topic', '/lidar')
    horizontal_samples = lidar.get('horizontal_samples', 360)
    horizontal_resolution = lidar.get('horizontal_resolution', 1.0)
    horizontal_min_angle = lidar.get('horizontal_min_angle', -3.14159)
    horizontal_max_angle = lidar.get('horizontal_max_angle', 3.14159)
    vertical_samples = lidar.get('vertical_samples', 32)
    vertical_resolution = lidar.get('vertical_resolution', 1.0)
    vertical_min_angle = lidar.get('vertical_min_angle', -0.78539)
    vertical_max_angle = lidar.get('vertical_max_angle', 0.78539)
    min_distance = lidar.get('min_distance', 0.2)
    max_distance = lidar.get('max_distance', 100.0)
    resolution = lidar.get('resolution', 0.017453)
    

    # IMU parameters
    imu_frequency = imu.get('imu_frequency', 50.0)
    imu_out_topic = imu.get('imu_out_topic', '/imu')


    # --- End load ---

    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    
    # Set ign sim resource path
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(this_pkg_path, 'worlds'), ':' + str(Path(this_pkg_path).parent.resolve())
        ]
    )

    open_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(this_pkg_path+"/rviz/ackermann.rviz")],
    )

    open_ign = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [this_pkg_path+"/worlds/"+world_file, ' -v 4', ' -r'])

        ]
    )

    xacro_file = os.path.join(this_pkg_path, 'urdf', robot_model+'.xacro') #.urdf
    print("Loading config file: ", xacro_file, flush=True)

    try:
        doc = xacro.process_file(xacro_file,
            mappings={
                'base_color' : str(robot_base_color),  
                'lidar_frequency': str(lidar_frequency), 
                'horizontal_samples': str(horizontal_samples),
                'horizontal_resolution': str(horizontal_resolution),
                'horizontal_min_angle': str(horizontal_min_angle),
                'horizontal_max_angle': str(horizontal_max_angle),
                'vertical_samples': str(vertical_samples),
                'vertical_resolution': str(vertical_resolution),
                'vertical_min_angle': str(vertical_min_angle),
                'vertical_max_angle': str(vertical_max_angle),
                'min_distance': str(min_distance),
                'max_distance': str(max_distance),
                'resolution': str(resolution),
                'imu_frequency': str(imu_frequency), 
                })
    except Exception as e:
        import traceback, sys
        print("Error processing xacro:", xacro_file, file=sys.stderr)
        traceback.print_exc()
        raise

    robot_desc = doc.toprettyxml(indent='  ')
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', pose[0], '-y', pose[1], '-z', pose[2],
                   '-R', '0.0', '-P', '0.0', '-Y', pose[3],
                   '-name', robot_ns,
                   '-allow_renaming', 'false'],
    )

    # Esperar 3s antes de intentar crear la entidad para dar tiempo a que el mundo cargue
    gz_spawn_entity = TimerAction(
        period=3.0,
        actions=[gz_spawn_entity]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        #namespace=robot_ns,
        output="screen",
        parameters=[{'robot_description': robot_desc}]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[             # ign topic -t <topic_name> --info
            '/model/'+robot_ns+'/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/'+robot_ns+'/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/world/world_model/model/'+robot_ns+'/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',

            #LIDAR
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',

            #DEEP CAMERA
            # '/depth_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            # '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            # '/depth_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',

            #IMU
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
        ],
        parameters=[{'qos_overrides./model/'+robot_ns+'.subscriber.reliability': 'reliable'}],
        output='screen',
        remappings=[            # ign topic -l
            ('/model/'+robot_ns+'/cmd_vel', '/cmd_vel'),
            ('/model/'+robot_ns+'/odometry', '/odom'),
            ('/world/world_model/model/'+robot_ns+'/joint_state', 'joint_states'),
            ('/lidar', lidar_out_topic+'/scan'),
            ('/lidar/points', lidar_out_topic+'_ign'),
            ('/imu', imu_out_topic+'_ign'),
        ]
    )

    tf_broadcaster_odom = Node(
        package=package_name,
        executable="tf_broadcaster",
        output="screen",
    )

    pc2_to_xyzi = Node(
        package=package_name,
        executable="pc2_to_xyzi",
        output="screen",
        parameters=[{
            'lidar_topic_in_':  lidar_out_topic+'_ign',
            'lidar_topic_out_': lidar_out_topic+'_pcl',
        }],
    )


    lidar_imu_sync = Node(
        package=package_name,                 
        executable='lidar_imu_sync',  # el nombre del ejecutable que compilas
        output='screen',
        parameters=[{
            'lidar_topic_in_':  lidar_out_topic+'_pcl',  # If you activate the PCL mode change ign for pcl, for specific lidar formats
            'imu_topic_in_':    imu_out_topic+'_ign',
            'lidar_topic_out_': lidar_out_topic,
            'imu_topic_out_':   imu_out_topic,
        }],
        
    )


    return LaunchDescription(
        [
            simu_time,
            ign_resource_path,
            open_rviz,
            open_ign,
            gz_spawn_entity,
            robot_state_publisher,
            bridge,
            tf_broadcaster_odom,
            pc2_to_xyzi,
            lidar_imu_sync
        ]
    )
