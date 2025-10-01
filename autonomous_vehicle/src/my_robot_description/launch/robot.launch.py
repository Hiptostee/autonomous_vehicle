import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command


from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )

    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')

    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    node_scan = Node(
        namespace='lidar', package='lidar',
        executable='scan_to_cloud_node', output='screen')
    node_obstacle = Node(
        namespace='obstacle_mitigation', package='obstacle_mitigation',
        executable='obstacle_mitigation_node', output='screen')

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_robot',
                                   '-x', '0', '-y', '0', '-z', '0.1'],
                        output='screen')

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        node_scan,
        node_obstacle,
        spawn_entity,
        load_joint_state_broadcaster,
        load_diff_drive_controller
    ])
