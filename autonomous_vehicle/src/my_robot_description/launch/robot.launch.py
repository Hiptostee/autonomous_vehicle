import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command


from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    
    # Path to the world file
    world_file = os.path.join(pkg_path, 'worlds', 'my_world.world')

    # Log the world path and verify it exists early so the launch output is clear
    if not os.path.isfile(world_file):
        raise FileNotFoundError(f"World file not found: {world_file}\nMake sure the package is built and you've sourced the workspace install/setup.bash before launching.")

    # Start Gazebo server
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={'world': world_file}.items()
    )

    # Get robot description from URDF
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Run the robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Run the spawner node from the gazebo_ros package.
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_robot'],
        output='screen'
    )
    
    # Load the joint state broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # Load the differential drive controller
    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )
    
    # Other nodes from your original file
    node_scan = Node(
        namespace='lidar', package='lidar',
        executable='scan_to_cloud_node', output='screen')
    node_obstacle = Node(
        namespace='obstacle_mitigation', package='obstacle_mitigation',
        executable='obstacle_mitigation_node', output='screen')


    # LAUNCH SEQUENCE CONFIGURATION
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        node_scan,
        node_obstacle,
        
        # Use an event handler to launch the controllers AFTER the robot is spawned
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diff_drive_controller],
            )
        ),
    ])