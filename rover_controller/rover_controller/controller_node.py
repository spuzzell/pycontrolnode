#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import math

class KinematicPUB(Node):
    def __init__(self):
        super().__init__('rover_controller_node')

        self.declare_parameter('wheel_radius', 0.08)  # meters
        self.declare_parameter('wheel_base', 0.358)     # meters

        self.declare_parameter('min_speed', -8.7)
        self.declare_parameter('max_speed', 8.7)

        
        # Get the parameter values
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value

        self.twist_subscriber = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            self.cmd_vel_callback,
            10
        )

        self.publisher = self.create_publisher(
            Vector3,
            '/wheel_velocities',
            10
        )

        # Safety timeout - stop if no commands received
        #self.declare_parameter('cmd_timeout', 0.5)  # seconds
        #self.cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        
        #self.last_cmd_time = self.get_clock().now()
        #self.safety_timer = self.create_timer(0.1, self._safety_check)
        
        # Emergency stop flag
        #self.emergency_stop = False



    
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.emergency_stop = False

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


            # Limit wheel speeds to min/max range
            #v_l, v_r = self._limit_wheel_speeds(v_l, v_r)
            v_l_remapped = (v_l / 8.0) * 255.0
            v_r_remapped = (v_r / 8.0) * 255.0
            

            wheel_vel_msg = Vector3()
            wheel_vel_msg.x = v_l_remapped  # left wheel velocity in rad/s
            wheel_vel_msg.y = v_r_remapped  # right wheel velocity in rad/s
            wheel_vel_msg.z = 0.0  # unused

            self.publisher.publish(wheel_vel_msg)
            self.get_logger().debug(f"Published wheel velocities: L={v_l:.2f}, R={v_r:.2f}")
            
        except Exception as e:
            self.get_logger().error(f"Error in cmd_vel_callback: {e}")
    

    def _is_valid_twist(self, msg: Twist) -> bool:
        """Validate twist message for NaN or infinite values"""
        import math
        return (not math.isnan(msg.linear.x) and not math.isinf(msg.linear.x) and
                not math.isnan(msg.angular.z) and not math.isinf(msg.angular.z))   
    

    def _limit_wheel_speeds(self, v_l: float, v_r: float) -> tuple:
        """Limit wheel speeds while maintaining differential ratio"""
        max_wheel_speed = max(abs(v_l), abs(v_r))
        
        if max_wheel_speed > self.max_speed:
            # Scale both wheels proportionally
            scale_factor = self.max_speed / max_wheel_speed
            v_l *= scale_factor
            v_r *= scale_factor
        
        # Apply individual limits
        v_l = max(self.min_speed, min(self.max_speed, v_l))
        v_r = max(self.min_speed, min(self.max_speed, v_r))
        
        return v_l, v_r

    def _safety_check(self):
        """Check for command timeout and apply emergency stop if needed"""
        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_last_cmd > self.cmd_timeout and not self.emergency_stop:
            self.get_logger().warn("Command timeout - stopping rover")
            self._publish_stop_command()
            self.emergency_stop = True

    def _publish_stop_command(self):
        """Publish zero velocities to stop the rover"""
        wheel_vel_msg = Vector3()
        wheel_vel_msg.x = 0.0
        wheel_vel_msg.y = 0.0
        wheel_vel_msg.z = 0.0
        self.publisher.publish(wheel_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicPUB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()