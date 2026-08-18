import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to your URDF file
    urdf_path = os.path.join(
        get_package_share_directory('rover_sim_pkg'),
        'urdf', 'rover.urdf'
    )

    # Path to the modern Gazebo simulation launch file
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Node to spawn the robot entity into Gazebo using the modern bridge
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', urdf_path, '-name', 'enterprise_rover', '-z', '0.5'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_entity
    ])