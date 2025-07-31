#!/usr/bin/env python3

"""
Usage:
ros2 run vio pattern_commander <pattern> [params]
Examples:
ros2 run vio pattern_commander hover --duration 5.0 --perturbation 0.2
ros2 run vio pattern_commander circle --radius 4.0 --speed 0.8 --direction -1
ros2 run vio pattern_commander aggressive --max_speed 2.0 --change_interval 1.0
ros2 run vio pattern_commander figure8 --radius 3.0 --speed 0.6
ros2 run vio pattern_commander square --side_length 5.0 --speed 1.0
ros2 run vio pattern_commander spiral --radius 2.0 --height 10.0 --speed 0.5
ros2 run vio pattern_commander pure_rotation --angle 360 --speed 1.0
ros2 run vio pattern_commander random_walk --max_distance 2.0 --duration 10.0
ros2 run vio pattern_commander exploration --waypoints 5 --speed 0.5
ros2 run vio pattern_commander landing

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys

class PatternCommander(Node):
    def __init__(self):
        super().__init__('pattern_commander')
        
        self.command_publisher = self.create_publisher(
            String, 'flight_pattern_command', 10)
        
        self.status_subscriber = self.create_subscription(
            String, 'flight_pattern_status', self.status_callback, 10)
        
        self.get_logger().info('Pattern Commander ready. Available patterns:')
        self.print_available_patterns()

    def status_callback(self, msg):
        """Print pattern status updates"""
        try:
            status = json.loads(msg.data)
            self.get_logger().info(f'Pattern Status: {status["pattern"]} - {status["params"]}')
        except:
            pass

    def print_available_patterns(self):
        """Print available patterns for user reference"""
        patterns = {
            "hover": "Stationary hover with small perturbations",
            "circle": "Circular flight pattern",
            "linear_x": "Linear forward/backward movement",
            "figure8": "Figure-8 flight pattern",
            "square": "Square flight pattern with sharp turns",
            "spiral": "Ascending/descending spiral",
            "aggressive": "Aggressive maneuvers with quick changes",
            "pure_rotation": "Pure rotation without translation",
            "random_walk": "Random walk pattern",
            "exploration": "Exploration pattern with waypoints",
            "landing": "Land the drone"
        }
        
        for pattern, description in patterns.items():
            self.get_logger().info(f'  {pattern}: {description}')

    def send_pattern_command(self, pattern, params=None):
        """Send pattern command to flight controller"""
        command = {"pattern": pattern}
        if params:
            command["params"] = params
        
        msg = String()
        msg.data = json.dumps(command)
        self.command_publisher.publish(msg)
        
        self.get_logger().info(f'Sent command: {pattern} with params: {params}')

def main(args=None):
    rclpy.init(args=args)
    node = PatternCommander()
    
    if len(sys.argv) > 1:
        # Command line usage
        pattern = sys.argv[1]
        params = {}
        
        # Parse additional parameters
        for i in range(2, len(sys.argv), 2):
            if i + 1 < len(sys.argv):
                key = sys.argv[i].lstrip('-')
                try:
                    value = float(sys.argv[i + 1])
                except ValueError:
                    value = sys.argv[i + 1]
                params[key] = value
        
        node.send_pattern_command(pattern, params if params else None)
        
        # Spin briefly to send the message
        rclpy.spin_once(node, timeout_sec=1.0)
    else:
        # Interactive mode
        node.get_logger().info('Interactive mode - use: ros2 run vio pattern_commander <pattern> [params]')
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()