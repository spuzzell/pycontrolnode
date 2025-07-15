#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
import serial

class SerialInterface(Node):
    def __init__(self):
        super().__init__('serial_interface')
        
        self._declare_parameters()
        self._load_parameters()
        
        self.serial_conn = None
        self.connect()
        
        self.command_subscriber = self.create_subscription(
            String,
            '/serial_commands',
            self.command_callback,
            1
        )
        
        self.encoder_publisher = self.create_publisher(
            Int32MultiArray,
            '/encoder_feedback',
            10
        )
        
        self.encoder_timer = self.create_timer(0.1, self.read_encoder_timer_callback)

            
    def _declare_parameters(self) -> None:
        """Declare all ROS parameters with default values."""
        self.declare_parameter('serial_device', '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout_ms', 1000)
        self.declare_parameter('loop_rate', 30)
        self.declare_parameter('count_rotation', 2096)

    def _load_parameters(self) -> None:
        """Load parameter values and calculate derived values."""
        # Get parameter values
        self.serial_device = self.get_parameter('serial_device').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout_ms = self.get_parameter('timeout_ms').get_parameter_value().integer_value
        self.loop_rate = self.get_parameter('loop_rate').get_parameter_value().integer_value
        self.count_rotation = self.get_parameter('count_rotation').get_parameter_value().integer_value
        self.rads_per_count = (2 * math.pi) / self.count_rotation
        self.time_encoder= time.time()
        self.encoder_left = 0
        self.encoder_right = 0

    def connect(self):
        """Connect to serial device"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_device,
                baudrate=self.baud_rate,
                timeout=self.timeout_ms / 1000.0,  # Convert to seconds
                write_timeout=1.0
            )
            time.sleep(2)  # Give time for connection to establish
            self.get_logger().info(f"Connected to {self.serial_device} at {self.baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.serial_device}: {e}")
            self.serial_conn = None


    def connected(self) -> bool:
        """Check if serial connection is established"""
        return self.serial_conn is not None and self.serial_conn.is_open


    def command_callback(self, msg: String):
        """Send received command via serial"""
        if self.connected():
            self.send_msg(msg.data)
        else:
            self.get_logger().warn("Cannot send command: Serial not connected")
    

    def send_msg(self, msg_to_send: str, print_output: bool = False) -> str:
        """Send message via serial and return response"""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().error("Serial connection not established")
            return ""
        
        try:
            # Flush buffers
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            # Send message
            self.serial_conn.write(msg_to_send.encode())
            
            # Read response until newline
            response = self.serial_conn.readline().decode().strip()
            
            if print_output:
                self.get_logger().info(f"Sent: {msg_to_send.strip()} Recv: {response}")
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in send_msg: {e}")
            return ""
    

    def read_encoder_values(self):
        """Read encoder values from ESP and return as tuple (val_1, val_2)"""
        try:
            response = self.send_msg("e\r")
            if response:
                values = response.split(" ")
                if len(values) >= 2:
                    val_1 = int(values[0])
                    val_2 = int(values[1])
                    return val_1, val_2
                else:
                    self.get_logger().error(f"Invalid encoder response format: {response}")
                    return 0, 0
            return 0, 0
        except Exception as e:
            self.get_logger().error(f"Error reading encoder values: {e}")
            return 0, 0
        


    def read_encoder_timer_callback(self):
        """Timer callback to read encoder values at 10Hz"""
 
        time_diff = time.time() - self.time_encoder

        if self.connected():
            try:
                val_1, val_2 = self.read_encoder_values()

                left_wheel_position = val_1 * self.rads_per_count
                left_wheel_speed = (val_1 - self.encoder_left) / (time_diff)* self.rads_per_count
                
                right_wheel_position = val_2 * self.rads_per_count
                right_wheel_speed = (val_2 - self.encoder_right) / (time_diff)* self.rads_per_count

                encoder_msg = Int32MultiArray()
                encoder_msg.data = [
                    int(left_wheel_position * 1000),   # Convert to milliradians
                    int(left_wheel_speed * 1000),      # Convert to milliradians/sec
                    int(right_wheel_position * 1000),  # Convert to milliradians
                    int(right_wheel_speed * 1000)      # Convert to milliradians/sec
                ]
                self.encoder_publisher.publish(encoder_msg)
                
                # Update previous encoder values
                self.encoder_left = val_1
                self.encoder_right = val_2

                
                self.get_logger().info(f"Encoder values: L={val_1}, R={val_2}", throttle_duration_sec=1.0)
            except Exception as e:
                self.get_logger().error(f"Error in encoder timer callback: {e}")
        else:
            self.get_logger().warn("Cannot read encoders: Serial not connected", throttle_duration_sec=5.0)
        
        self.time_encoder= time.time()



def main(args=None):
    rclpy.init(args=args)
    node = SerialInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()













