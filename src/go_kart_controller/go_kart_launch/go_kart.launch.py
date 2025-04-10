#!/usr/bin/env python3
"""
go_kart.launch.py
Launch file to start the RC car simulation display node and UDP forwarder node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node that runs the display node for the car simulation.
    display_node = Node(
        package='go_kart_controller',
        executable='display_node',
        name='display_node',
        output='screen'
    )

    # Node that runs the UDP forwarder, which reformats and sends control packets.
    udp_forwarder_node = Node(
        package='go_kart_controller',
        executable='udp_forwarder_node',
        name='udp_forwarder_node',
        output='screen'
    )

    return LaunchDescription([
        display_node,
        udp_forwarder_node
    ])
