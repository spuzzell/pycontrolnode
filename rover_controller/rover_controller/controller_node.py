#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
import math

class KinematicPUB(Node):
    def __init__(self):
        super().__init__('rover_controller_node')

        self.declare_parameter('wheel_radius', 0.08)  # meters
        self.declare_parameter('wheel_base', 0.358)     # meters
        self.declare_parameter('count_rotation', 2096)

        self.declare_parameter('min_speed', -8.7)
        self.declare_parameter('max_speed', 8.7)

        # Get the parameter values
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value

        self.count_rotation = self.get_parameter('count_rotation').get_parameter_value().integer_value


        self.twist_subscriber = self.create_subscription(
            Twist,
            #'/diff_cont/cmd_vel_unstamped',
            '/cmd_vel_joy',
            self.cmd_vel_callback,
            10
        )

        self.publisher = self.create_publisher(
            Int16MultiArray,
            '/wheel_velocities',
            10
        )


    def cmd_vel_callback(self, msg: Twist):

        try:
            # Validate input
            if not self._is_valid_twist(msg):
                self.get_logger().warn("Invalid twist message received")
                return
                

            v = msg.linear.x     # Forward velocity
            omega = msg.angular.z   # Angular velocity (yaw)

            # Differential drive kinematics
            L = self.wheel_base
            R = self.wheel_radius

            v_l = (v - (omega * L / 2)) / R
            v_r = (v + (omega * L / 2)) / R


            v_l = max(self.min_speed, min(self.max_speed, v_l))
            v_r = max(self.min_speed, min(self.max_speed, v_r))


            L_motor_counts_per_second = (v_l*self.count_rotation) / (2 * math.pi)
            R_motor_counts_per_second = (v_r*self.count_rotation) / (2 * math.pi)

            L_motor = int(L_motor_counts_per_second)
            R_motor = int(R_motor_counts_per_second)

            # Round and clamp to int16 range expected by struct.pack('<hhhh', ...)
            def to_int16(v):
                iv = int(round(v))
                if iv < -32768:
                    return -32768
                if iv > 32767:
                    return 32767
                return iv

            L_motor = to_int16(L_motor)
            R_motor = to_int16(R_motor)

            wheel_vel_msg = Int16MultiArray()
            wheel_vel_msg.data = [L_motor, R_motor, L_motor, R_motor]

            self.publisher.publish(wheel_vel_msg)
            self.get_logger().debug(
                f"Published wheel velocities (remapped): FL={L_motor:.3f}, "
                f"FR={R_motor:.3f}, RL={L_motor:.3f}, RR={R_motor:.3f}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error in cmd_vel_callback: {e}")
    

    def _is_valid_twist(self, msg: Twist) -> bool:
        """Validate twist message for NaN or infinite values"""
        import math
        return (not math.isnan(msg.linear.x) and not math.isinf(msg.linear.x) and
                not math.isnan(msg.angular.z) and not math.isinf(msg.angular.z))   
    


    def _publish_stop_command(self):
        """Publish zero velocities to stop the rover"""
        wheel_vel_msg = Int16MultiArray()
        wheel_vel_msg.data = [0, 0, 0, 0]
        self.publisher.publish(wheel_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicPUB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
