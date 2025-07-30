#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CircleFlightController(Node):
    def __init__(self):
        super().__init__('circle_flight_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.publish_flight_command)
        
        # Flight parameters
        self.radius = 3.0  # Circle radius in meters
        self.angular_speed = 0.3  # rad/s
        self.altitude_target = 2.0  # Flight altitude
        self.start_time = time.time()
        self.takeoff_duration = 5.0  # Take 5 seconds to reach altitude
        
        self.get_logger().info('Velocity-based circle flight controller started - taking off...')

    def publish_flight_command(self):
        twist = Twist()
        
        current_time = time.time() - self.start_time
        
        if current_time < self.takeoff_duration:
            # Takeoff phase - move upward
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 1.0  # Upward velocity
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            
            if current_time > self.takeoff_duration - 1.0:
                self.get_logger().info('Starting circle flight pattern...', throttle_duration_sec=1.0)
        else:
            # Circle flight phase
            flight_time = current_time - self.takeoff_duration
            
            # Circular motion velocities
            angle = self.angular_speed * flight_time
            
            # Velocity commands for circular motion
            linear_speed = 1.0  # m/s
            twist.linear.x = linear_speed * math.cos(angle)
            twist.linear.y = linear_speed * math.sin(angle)
            twist.linear.z = 0.0  # Maintain altitude
            
            # Angular velocity for smooth rotation
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.angular_speed
        
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CircleFlightController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Circle flight controller stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()