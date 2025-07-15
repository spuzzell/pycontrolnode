#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

class SerialMessageGenerator(Node):
    def __init__(self):
        super().__init__('serial_message_generator')
        
        self._declare_parameters()
        self._load_parameters()
        
        self.vector_subscriber = self.create_subscription(
            Vector3,
            '/wheel_velocities',
            self.wheel_vel_callback,
            10
        )
        
        self.command_publisher = self.create_publisher(
            String,
            '/serial_commands',
            10
        )
        
    def _declare_parameters(self):
        self.declare_parameter('count_rotation', 2096)
        self.declare_parameter('loop_rate', 30)
        
    def _load_parameters(self):
        self.count_rotation = self.get_parameter('count_rotation').get_parameter_value().integer_value
        self.loop_rate = self.get_parameter('loop_rate').get_parameter_value().integer_value
        self.rads_per_count = (2 * math.pi) / self.count_rotation
        
    def wheel_vel_callback(self, msg: Vector3):
        try:
            #left_motor = int((msg.x/self.rads_per_count)/self.loop_rate)*15
            #right_motor = int((msg.y/self.rads_per_count)/self.loop_rate)*15
            left_motor = int(msg.x)
            right_motor = int(msg.y)
            
            command_msg = String()
            command_msg.data = f"o {left_motor} {right_motor}\r"
            self.command_publisher.publish(command_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in wheel_vel_callback: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = SerialMessageGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()