#!/usr/bin/env python3
"""
udp_forwarder_node.py
A ROS2 node that subscribes to the /go_kart/commands topic and forwards
the received commands as UDP packets to a specified target IP/port.
It now implements the auto control packet format expected by the Teensy,
which is a comma-separated string in the format:
    "steering,throttle,emergency"
where:
    - steering: The steering value (odrive_steer) from the display node.
    - throttle: The drive RPM value (vesc_rpm) from the display node.
    - emergency: Set to 0 during normal operation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json

class UDPForwarderNode(Node):
    def __init__(self):
        super().__init__('udp_forwarder_node')
        
        # Subscribe to the '/go_kart/commands' topic published by the display node.
        self.subscription = self.create_subscription(
            String,
            'go_kart/commands',
            self.command_callback,
            10
        )
        self.subscription  # Prevent unused variable warning.
        
        # Setup UDP socket settings.
        # Change the target IP and port to match your Teensy configuration.
        self.udp_target_ip = "192.168.1.177"  # Teensy's IP address.
        self.udp_target_port = 8888           # UDP port that the Teensy is listening on.
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.get_logger().info(
            f"UDP Forwarder Node initialized. Forwarding to {self.udp_target_ip}:{self.udp_target_port}"
        )
    
    def command_callback(self, msg: String):
        """
        Callback function for received commands.
        Parses the incoming JSON string, extracts the throttle and steering values,
        and repackages them into the auto control packet format: "steering,throttle,emergency".
        """
        try:
            # Parse the incoming JSON command from the display node.
            command_json = json.loads(msg.data)
            # Extract values; default to zero if absent.
            throttle = float(command_json.get("vesc_rpm", 0))
            steering = float(command_json.get("odrive_steer", 0))
            # For now, set emergency flag to 0 (normal operation).
            emergency = 0
            
            # Create the auto control packet using the expected format.
            packet = f"{steering:.3f},{throttle:.3f},{emergency}"
        except Exception as e:
            self.get_logger().error(f"Error parsing JSON command: {e}")
            # Fallback: just pass the original data.
            packet = msg.data

        # Send the packet as a UDP datagram.
        try:
            self.udp_socket.sendto(packet.encode('utf-8'), (self.udp_target_ip, self.udp_target_port))
            self.get_logger().info(f"Forwarded UDP packet: {packet}")
        except Exception as e:
            self.get_logger().error(f"Failed to send UDP packet: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UDPForwarderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
