#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time
import random
import json

class PatternFlightController(Node):
    def __init__(self):
        super().__init__('pattern_flight_controller')
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pattern_status_publisher = self.create_publisher(String, 'flight_pattern_status', 10)
        
        # Subscribers for pattern control
        self.pattern_command_subscriber = self.create_subscription(
            String, 'flight_pattern_command', self.pattern_command_callback, 10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz control
        
        # Flight state
        self.start_time = time.time()
        self.current_pattern = "takeoff"
        self.pattern_start_time = time.time()
        self.pattern_params = {}
        
        # Pattern definitions
        self.pattern_sequence = [
            ("takeoff", {"duration": 5.0, "target_altitude": 2.0}),
            ("hover", {"duration": 3.0, "perturbation": 0.1}),
            ("circle", {"duration": 15.0, "radius": 3.0, "speed": 0.5, "direction": 1}),
            ("linear_x", {"duration": 8.0, "distance": 4.0, "speed": 0.8}),
            ("figure8", {"duration": 20.0, "radius": 2.5, "speed": 0.4}),
            ("square", {"duration": 16.0, "side_length": 4.0, "speed": 0.6}),
            ("spiral", {"duration": 12.0, "radius": 2.0, "height": 1.5, "speed": 0.3}),
            ("aggressive", {"duration": 10.0, "max_speed": 1.5, "change_interval": 2.0}),
            ("pure_rotation", {"duration": 8.0, "angular_speed": 0.8}),
            ("random_walk", {"duration": 15.0, "step_size": 1.0, "change_interval": 1.5}),
            ("exploration", {"duration": 25.0, "area_size": 5.0, "speed": 0.5}),
            ("hover", {"duration": 3.0, "perturbation": 0.05}),
            ("landing", {"duration": 5.0})
        ]
        
        self.current_pattern_index = 0
        self.load_current_pattern()
        
        self.get_logger().info(f'Pattern flight controller started - Beginning pattern sequence')

    def load_current_pattern(self):
        """Load the current pattern from sequence"""
        if self.current_pattern_index < len(self.pattern_sequence):
            pattern_name, params = self.pattern_sequence[self.current_pattern_index]
            self.set_pattern(pattern_name, params)
        else:
            self.get_logger().info('All patterns completed - hovering')
            self.set_pattern("hover", {"duration": float('inf'), "perturbation": 0.05})

    def set_pattern(self, pattern_name, params=None):
        """Set current flight pattern"""
        self.current_pattern = pattern_name
        self.pattern_params = params or {}
        self.pattern_start_time = time.time()
        
        # Reset pattern-specific state
        if pattern_name == "random_walk":
            self.random_target = [0, 0, 0]
            self.last_random_change = time.time()
        elif pattern_name == "exploration":
            self.exploration_targets = self.generate_exploration_points()
            self.current_target_index = 0
        elif pattern_name == "aggressive":
            self.last_aggressive_change = time.time()
            self.aggressive_direction = [random.uniform(-1, 1), random.uniform(-1, 1), 0]
        
        status_msg = String()
        status_msg.data = json.dumps({
            "pattern": pattern_name,
            "params": params,
            "timestamp": time.time()
        })
        self.pattern_status_publisher.publish(status_msg)
        
        self.get_logger().info(f'Starting pattern: {pattern_name} with params: {params}')

    def pattern_command_callback(self, msg):
        """Handle external pattern commands"""
        try:
            command = json.loads(msg.data)
            if "pattern" in command:
                params = command.get("params", {})
                self.set_pattern(command["pattern"], params)
                self.get_logger().info(f'Received pattern command: {command["pattern"]}')
        except json.JSONDecodeError:
            self.get_logger().warning(f'Invalid pattern command: {msg.data}')

    def control_loop(self):
        """Main control loop"""
        current_time = time.time()
        pattern_time = current_time - self.pattern_start_time
        
        # Check if current pattern should end
        if "duration" in self.pattern_params:
            if pattern_time >= self.pattern_params["duration"]:
                self.current_pattern_index += 1
                self.load_current_pattern()
                return
        
        # Generate velocity command based on current pattern
        twist = self.generate_pattern_velocity(pattern_time)
        self.cmd_vel_publisher.publish(twist)

    def generate_pattern_velocity(self, t):
        """Generate velocity commands for current pattern"""
        twist = Twist()
        
        if self.current_pattern == "takeoff":
            twist.linear.z = 1.0
            if t > self.pattern_params.get("duration", 5.0) - 1.0:
                twist.linear.z = 0.5  # Slow down near end
                
        elif self.current_pattern == "landing":
            twist.linear.z = -0.8
            
        elif self.current_pattern == "hover":
            # Small random perturbations for realistic hovering
            perturbation = self.pattern_params.get("perturbation", 0.1)
            twist.linear.x = random.uniform(-perturbation, perturbation)
            twist.linear.y = random.uniform(-perturbation, perturbation)
            twist.linear.z = random.uniform(-perturbation/2, perturbation/2)
            
        elif self.current_pattern == "circle":
            radius = self.pattern_params.get("radius", 3.0)
            speed = self.pattern_params.get("speed", 0.5)
            direction = self.pattern_params.get("direction", 1)
            
            angular_vel = speed / radius * direction
            twist.linear.x = speed * math.cos(angular_vel * t)
            twist.linear.y = speed * math.sin(angular_vel * t)
            twist.angular.z = angular_vel
            
        elif self.current_pattern == "linear_x":
            distance = self.pattern_params.get("distance", 4.0)
            speed = self.pattern_params.get("speed", 0.8)
            duration = self.pattern_params.get("duration", 8.0)
            
            if t < duration / 2:
                twist.linear.x = speed
            else:
                twist.linear.x = -speed
                
        elif self.current_pattern == "figure8":
            radius = self.pattern_params.get("radius", 2.5)
            speed = self.pattern_params.get("speed", 0.4)
            
            # Figure-8 parametric equations
            omega = speed / radius
            twist.linear.x = speed * math.cos(omega * t)
            twist.linear.y = speed * math.sin(2 * omega * t) / 2
            twist.angular.z = omega * (math.cos(2 * omega * t) - 1)
            
        elif self.current_pattern == "square":
            side_length = self.pattern_params.get("side_length", 4.0)
            speed = self.pattern_params.get("speed", 0.6)
            duration = self.pattern_params.get("duration", 16.0)
            
            # Divide into 4 sides
            side_duration = duration / 4
            side_index = int(t // side_duration)
            
            if side_index == 0:  # Forward
                twist.linear.x = speed
            elif side_index == 1:  # Left
                twist.linear.y = speed
            elif side_index == 2:  # Backward
                twist.linear.x = -speed
            else:  # Right
                twist.linear.y = -speed
                
        elif self.current_pattern == "spiral":
            radius = self.pattern_params.get("radius", 2.0)
            height = self.pattern_params.get("height", 1.5)
            speed = self.pattern_params.get("speed", 0.3)
            duration = self.pattern_params.get("duration", 12.0)
            
            # Expanding spiral with vertical movement
            r = radius * (t / duration)
            omega = speed / max(r, 0.5)
            
            twist.linear.x = speed * math.cos(omega * t)
            twist.linear.y = speed * math.sin(omega * t)
            twist.linear.z = height / duration  # Gradual ascent
            twist.angular.z = omega
            
        elif self.current_pattern == "aggressive":
            max_speed = self.pattern_params.get("max_speed", 1.5)
            change_interval = self.pattern_params.get("change_interval", 2.0)
            
            if time.time() - self.last_aggressive_change > change_interval:
                self.aggressive_direction = [
                    random.uniform(-1, 1),
                    random.uniform(-1, 1),
                    random.uniform(-0.5, 0.5)
                ]
                self.last_aggressive_change = time.time()
            
            twist.linear.x = max_speed * self.aggressive_direction[0]
            twist.linear.y = max_speed * self.aggressive_direction[1]
            twist.linear.z = max_speed * self.aggressive_direction[2]
            twist.angular.z = random.uniform(-1, 1)
            
        elif self.current_pattern == "pure_rotation":
            angular_speed = self.pattern_params.get("angular_speed", 0.8)
            twist.angular.z = angular_speed
            # No linear movement - tests pure gyroscope data
            
        elif self.current_pattern == "random_walk":
            step_size = self.pattern_params.get("step_size", 1.0)
            change_interval = self.pattern_params.get("change_interval", 1.5)
            
            if time.time() - self.last_random_change > change_interval:
                self.random_target = [
                    random.uniform(-step_size, step_size),
                    random.uniform(-step_size, step_size),
                    random.uniform(-step_size/2, step_size/2)
                ]
                self.last_random_change = time.time()
            
            twist.linear.x = self.random_target[0]
            twist.linear.y = self.random_target[1]
            twist.linear.z = self.random_target[2]
            
        elif self.current_pattern == "exploration":
            # Navigate between predefined exploration points
            if hasattr(self, 'exploration_targets') and self.exploration_targets:
                target = self.exploration_targets[self.current_target_index % len(self.exploration_targets)]
                speed = self.pattern_params.get("speed", 0.5)
                
                # Simple navigation toward target
                twist.linear.x = speed * math.cos(target[2])
                twist.linear.y = speed * math.sin(target[2])
                
                # Switch targets periodically
                if t % 5.0 < 0.1:  # Every 5 seconds
                    self.current_target_index += 1
        
        return twist

    def generate_exploration_points(self):
        """Generate random exploration waypoints"""
        area_size = self.pattern_params.get("area_size", 5.0)
        num_points = 8
        
        points = []
        for _ in range(num_points):
            x = random.uniform(-area_size/2, area_size/2)
            y = random.uniform(-area_size/2, area_size/2)
            angle = random.uniform(0, 2*math.pi)
            points.append([x, y, angle])
        
        return points

def main(args=None):
    rclpy.init(args=args)
    node = PatternFlightController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Pattern flight controller stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()