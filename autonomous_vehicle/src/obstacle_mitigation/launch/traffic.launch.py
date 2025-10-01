from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='traffic_system',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='traffic_light_controller',
                    plugin='CentralController',
                    name='central_controller'),
                ComposableNode(
                    package='traffic_light_controller',
                    plugin='CarController',
                    name='car_controller'),
            ],
            output='screen',
        ),
    ])