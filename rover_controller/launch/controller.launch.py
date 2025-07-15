import os

# Core launch functionalities
from launch import LaunchDescription

# For launching ROS 2 nodes
from launch_ros.actions import Node


def generate_launch_description():

    # Name of the package containing the rover controller nodes
    package_name = 'rover_controller'

    # Rover controller node
    rover_controller_node = Node(
        package=package_name,
        executable='rover_controller_node',
        name='rover_controller_node',
        output='screen'
    )

    # Serial message generator node
    serial_message_generator = Node(
        package=package_name,
        executable='serial_message_generator',
        name='serial_message_generator',
        output='screen'
    )

    # Serial passer node
    serial_passer_node = Node(
        package=package_name,
        executable='serial_passer_node',
        name='serial_passer_node',
        output='screen'
    )

    # Combine and return all launch elements
    return LaunchDescription([
        rover_controller_node,
        serial_message_generator
        #serial_passer_node
    ])