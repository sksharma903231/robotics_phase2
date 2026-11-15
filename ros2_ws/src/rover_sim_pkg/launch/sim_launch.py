import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_rover = get_package_share_directory('rover_sim_pkg')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(pkg_rover, 'urdf', 'rover.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # Launch configuration options
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo simulation headless (-s) without GUI'
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )

    # Gazebo Simulation Launch
    gz_args = LaunchConfiguration('gz_args', default='-r empty.sdf')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # Spawn Rover Entity into Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', urdf_path, '-name', 'enterprise_rover', '-z', '0.2'],
        output='screen'
    )

    # ROS-Gz Topic Bridge
    # Bridges joint states, cmd_vel, and lidar scan between Gazebo and ROS 2
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Python Telemetry Bridge Node
    telemetry_bridge_node = Node(
        package='rover_sim_pkg',
        executable='telemetry_bridge.py',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'telemetry_file': '/tmp/rover_telemetry.json'
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_headless,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity,
        bridge_node,
        telemetry_bridge_node
    ])