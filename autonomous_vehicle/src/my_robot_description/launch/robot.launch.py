import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package
    pkg_path = get_package_share_directory('my_robot_description')

    # Get the path to the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )

    # Get the path to the URDF file
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')

    # Process the URDF file using the xacro command
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Create a node for the robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Create a node to spawn the robot in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_robot'],
                        output='screen')

    # Create a node for the joint_state_broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # Create a node for the diff_drive_controller
    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    # Launch description
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_diff_drive_controller
    ])
