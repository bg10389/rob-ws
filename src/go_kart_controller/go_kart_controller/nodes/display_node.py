#!/usr/bin/env python3
"""
display_node.py
A ROS2 node that simulates an RC car:
    - Uses a birds eye view car image (with the top edge as the front).
    - Processes continuous key presses to update driving speed and steering input.
    - Maintains a separate heading that is updated continuously based on steering input.
    - Moves the car accordingly so that any nonzero steering makes the car continuously turn.
    - Publishes a JSON-formatted command with:
             'vesc_rpm': the drive RPM value, and
             'odrive_steer': the steering angle (clamped between -27 and +27).
"""

import sys
import os
import math
import json
import rclpy
from rclpy.node import Node
import pygame
from std_msgs.msg import String

class DisplayNode(Node):
        def __init__(self):
                super().__init__('display_node')

                # Initialize pygame and create a window.
                pygame.init()
                self.screen = pygame.display.set_mode((800, 600))
                pygame.display.set_caption("RC Car Simulation")
                self.clock = pygame.time.Clock()

                # Car's center starting position.
                self.kart_x, self.kart_y = 400, 300

                # Car's heading: the actual direction of travel in degrees.
                # 0 degrees means facing upward.
                self.heading = 0

                # Steering input (in degrees); user controlled, clamped between -27 and +27.
                self.steering_angle = 0

                # Driving speed (interpreted here as pixels per frame).
                self.target_rpm = 0

                # Load an optional background image.
                try:
                        self.background = pygame.image.load("background.png")
                        self.get_logger().info("Loaded background image.")
                except Exception:
                        self.background = None
                        self.get_logger().info("No background image found; using plain white background.")

                # Load the car image from the assets folder.
                try:
                        assets_path = os.path.join(os.path.dirname(__file__), "..", "assets")
                        car_image_path = os.path.join(assets_path, "car.jpg")
                        self.car_img_orig = pygame.image.load(car_image_path)
                        self.car_img_orig = pygame.transform.scale(self.car_img_orig, (80, 160))
                        self.get_logger().info("Loaded car image from assets/car.jpg.")
                except Exception as e:
                        self.car_img_orig = None
                        self.get_logger().error(f"Failed to load car image: {str(e)}")

                # Create a publisher on the /go_kart/commands topic.
                self.command_publisher = self.create_publisher(String, 'go_kart/commands', 10)

        def publish_command(self, command: str):
                """Publish a command message on the /go_kart/commands topic."""
                msg = String()
                msg.data = command
                self.command_publisher.publish(msg)
                self.get_logger().info(f"Published command: {command}")

        def run(self):
                """Main loop for processing inputs, updating the RC car simulation, and publishing commands."""
                # For the simple kinematic model, we use an arbitrary wheelbase length.
                L = 100.0  # Adjust this constant as needed to tune turning rate.
                
                while rclpy.ok():
                        # Process events (e.g., window close).
                        for event in pygame.event.get():
                                if event.type == pygame.QUIT:
                                        rclpy.shutdown()
                                        pygame.quit()
                                        sys.exit()

                        # Fetch the current keyboard state.
                        keys = pygame.key.get_pressed()

                        # Update driving speed.
                        if keys[pygame.K_UP]:
                                self.target_rpm = 5   # Forward speed.
                        elif keys[pygame.K_DOWN]:
                                self.target_rpm = -5  # Reverse speed.
                        else:
                                self.target_rpm = 0

                        # Update steering input.
                        if keys[pygame.K_LEFT]:
                                self.steering_angle += 2
                        if keys[pygame.K_RIGHT]:
                                self.steering_angle -= 2

                        # If neither left nor right is pressed, gradually recenter steering.
                        if not (keys[pygame.K_LEFT] or keys[pygame.K_RIGHT]):
                                if self.steering_angle > 0:
                                        self.steering_angle = max(0, self.steering_angle - 2)
                                elif self.steering_angle < 0:
                                        self.steering_angle = min(0, self.steering_angle + 2)

                        # Clamp the steering input between -27 and +27 degrees.
                        self.steering_angle = max(min(self.steering_angle, 27), -27)

                        # Update the car's heading based on steering input and speed.
                        # Using a simplified kinematic model: d(heading)/dt = (v / L) * delta, where delta is in radians.
                        if self.target_rpm != 0:
                                turn_rate = (self.target_rpm / L) * math.radians(self.steering_angle)
                                # Convert the turn rate (in radians per frame) to degrees per frame.
                                self.heading += math.degrees(turn_rate)

                        # Update the car's position based on the current heading.
                        rad_heading = math.radians(self.heading)
                        dx = -self.target_rpm * math.sin(rad_heading)
                        dy = -self.target_rpm * math.cos(rad_heading)
                        self.kart_x += dx
                        self.kart_y += dy

                        # Create and publish the command as a JSON string.
                        command_data = {
                                "vesc_rpm": self.target_rpm,
                                "odrive_steer": self.steering_angle
                        }
                        self.publish_command(json.dumps(command_data))

                        # Draw the background.
                        if self.background:
                                self.screen.blit(self.background, (0, 0))
                        else:
                                self.screen.fill((255, 255, 255))

                        # Draw the car image rotated to match the current heading.
                        if self.car_img_orig:
                                rotated_img = pygame.transform.rotate(self.car_img_orig, self.heading)
                                rotated_rect = rotated_img.get_rect(center=(self.kart_x, self.kart_y))
                                self.screen.blit(rotated_img, rotated_rect)
                        else:
                                pygame.draw.rect(self.screen, (0, 0, 255), (self.kart_x - 25, self.kart_y - 15, 50, 30))

                        pygame.display.update()
                        self.clock.tick(60)
                        rclpy.spin_once(self, timeout_sec=0.001)

def main(args=None):
        rclpy.init(args=args)
        node = DisplayNode()
        try:
                node.run()
        except KeyboardInterrupt:
                pass
        finally:
                node.destroy_node()
                rclpy.shutdown()

if __name__ == '__main__':
        main()
