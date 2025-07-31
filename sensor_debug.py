#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu

class SensorDebug(Node):
    def __init__(self):
        super().__init__('sensor_debug')
        
        # Subscribe to both topics
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        
        self.camera_count = 0
        self.imu_count = 0
        
        # Report every 2 seconds
        self.timer = self.create_timer(2.0, self.report)
        
        self.get_logger().info('Sensor debug started...')
    
    def camera_cb(self, msg):
        self.camera_count += 1
        self.get_logger().info(f'Camera: {msg.width}x{msg.height}, encoding: {msg.encoding}', once=True)
    
    def imu_cb(self, msg):
        self.imu_count += 1
        self.get_logger().info(f'IMU: acc=({msg.linear_acceleration.x:.2f},{msg.linear_acceleration.y:.2f},{msg.linear_acceleration.z:.2f})', once=True)
    
    def report(self):
        self.get_logger().info(f'Received - Camera: {self.camera_count} msgs, IMU: {self.imu_count} msgs')

def main():
    rclpy.init()
    node = SensorDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
