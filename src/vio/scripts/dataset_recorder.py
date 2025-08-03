#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import csv
import os
import json
import time
from datetime import datetime
import numpy as np
from scipy.spatial.transform import Rotation as R

class DatasetRecorder(Node):
    def __init__(self):
        super().__init__('dataset_recorder')
        
        # Parameters
        self.declare_parameter('dataset_path', '/tmp/vio_dataset')
        self.declare_parameter('session_name', '')
        self.declare_parameter('record_rate', 10.0)  # Hz - every 0.1 simulation seconds
        
        self.dataset_path = self.get_parameter('dataset_path').value
        session_name = self.get_parameter('session_name').value
        self.record_rate = self.get_parameter('record_rate').value
        
        # Generate session name if not provided
        if not session_name:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session_name = f"session_{timestamp}"
        
        # Setup directories
        self.session_path = os.path.join(self.dataset_path, session_name)
        self.images_path = os.path.join(self.session_path, 'images')
        os.makedirs(self.images_path, exist_ok=True)
        
        # Initialize CSV files
        self.imu_csv_path = os.path.join(self.session_path, 'imu.csv')
        self.poses_csv_path = os.path.join(self.session_path, 'poses.csv')
        
        # Initialize CSV writers
        self.imu_csv_file = open(self.imu_csv_path, 'w', newline='')
        self.poses_csv_file = open(self.poses_csv_path, 'w', newline='')
        
        self.imu_writer = csv.writer(self.imu_csv_file)
        self.poses_writer = csv.writer(self.poses_csv_file)
        
        # Write CSV headers
        self.imu_writer.writerow(['index', 'timestamp', 'accel_x', 'accel_y', 'accel_z', 
                                 'gyro_x', 'gyro_y', 'gyro_z', 'orient_x', 'orient_y', 'orient_z', 'orient_w'])
        self.poses_writer.writerow(['index', 'timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Data storage
        self.latest_image = None
        self.latest_imu = None
        self.latest_pose = None
        self.frame_index = 0
        
        # Subscribers
        self.image_subscriber = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.imu_subscriber = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        
        # FIX: Subscribe to the correct TF topic format
        self.tf_subscriber = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10)
        
        # ALTERNATIVE: Subscribe to static transforms too
        self.tf_static_subscriber = self.create_subscription(
            TFMessage, '/tf_static', self.tf_static_callback, 10)
        
        # Recording timer
        self.record_timer = self.create_timer(1.0 / self.record_rate, self.record_frame)
        
        # Status publisher
        self.status_publisher = self.create_publisher(String, '/dataset_recorder_status', 10)
        
        self.get_logger().info(f'Dataset recorder started - Session: {session_name}')
        self.get_logger().info(f'Recording to: {self.session_path}')
        self.get_logger().info(f'Record rate: {self.record_rate} Hz')
        
        self.start_time = time.time()
        
        # For debugging - track what data we're receiving
        self.received_image = False
        self.received_imu = False
        self.received_pose = False

    def image_callback(self, msg):
        """Store latest image"""
        self.latest_image = msg
        if not self.received_image:
            self.get_logger().info('Received first image')
            self.received_image = True

    def imu_callback(self, msg):
        """Store latest IMU data"""
        self.latest_imu = msg
        if not self.received_imu:
            self.get_logger().info('Received first IMU data')
            self.received_imu = True

    def tf_callback(self, msg):
        """Extract drone pose from TF messages"""
        for transform in msg.transforms:
            # Look for drone-related transforms
            if ('quadcopter' in transform.child_frame_id or 
                'base_link' in transform.child_frame_id or
                transform.child_frame_id == 'quadcopter' or
                transform.header.frame_id == 'world'):
                
                self.latest_pose = transform.transform
                if not self.received_pose:
                    self.get_logger().info(f'Received first pose from: {transform.header.frame_id} -> {transform.child_frame_id}')
                    self.received_pose = True
                break

    def tf_static_callback(self, msg):
        """Handle static transforms"""
        # This might help with initial positioning
        for transform in msg.transforms:
            if ('quadcopter' in transform.child_frame_id or 
                'base_link' in transform.child_frame_id):
                if self.latest_pose is None:  # Only use if we don't have dynamic pose
                    self.latest_pose = transform.transform

    def record_frame(self):
        """Record synchronized frame data"""
        current_time = time.time()
        timestamp = current_time - self.start_time
        
        # Check if we have all required data
        missing_data = []
        if self.latest_image is None:
            missing_data.append("image")
        if self.latest_imu is None:
            missing_data.append("IMU")
        if self.latest_pose is None:
            missing_data.append("pose")
            
        if missing_data:
            if self.frame_index < 10:  # Only warn for first 10 attempts
                self.get_logger().warn(f'Waiting for data: {", ".join(missing_data)}')
            return

        try:
            # Save image
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            image_filename = f"{self.frame_index + 1}.jpg"  # 1-indexed as requested
            image_path = os.path.join(self.images_path, image_filename)
            cv2.imwrite(image_path, cv_image)
            
            # Record IMU data
            imu_data = [
                self.frame_index + 1,  # 1-indexed
                timestamp,
                self.latest_imu.linear_acceleration.x,
                self.latest_imu.linear_acceleration.y,
                self.latest_imu.linear_acceleration.z,
                self.latest_imu.angular_velocity.x,
                self.latest_imu.angular_velocity.y,
                self.latest_imu.angular_velocity.z,
                self.latest_imu.orientation.x,
                self.latest_imu.orientation.y,
                self.latest_imu.orientation.z,
                self.latest_imu.orientation.w
            ]
            self.imu_writer.writerow(imu_data)
            
            # Convert quaternion to Euler angles for pose
            translation = self.latest_pose.translation
            rotation = self.latest_pose.rotation
            
            # Convert quaternion to Euler (roll, pitch, yaw)
            r = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
            roll, pitch, yaw = r.as_euler('xyz', degrees=False)
            
            # Record pose data
            pose_data = [
                self.frame_index + 1,  # 1-indexed
                timestamp,
                translation.x,
                translation.y,
                translation.z,
                roll,
                pitch,
                yaw
            ]
            self.poses_writer.writerow(pose_data)
            
            # Flush files to ensure data is written
            self.imu_csv_file.flush()
            self.poses_csv_file.flush()
            
            self.frame_index += 1
            
            # Publish status every 50 frames
            if self.frame_index % 50 == 0:
                status_msg = String()
                status_msg.data = json.dumps({
                    "session": os.path.basename(self.session_path),
                    "frames_recorded": self.frame_index,
                    "timestamp": timestamp,
                    "recording_duration": timestamp
                })
                self.status_publisher.publish(status_msg)
                self.get_logger().info(f'Recorded {self.frame_index} frames ({timestamp:.1f}s)')
                
        except Exception as e:
            self.get_logger().error(f'Error recording frame {self.frame_index}: {str(e)}')

    def __del__(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'imu_csv_file') and self.imu_csv_file and not self.imu_csv_file.closed:
            self.imu_csv_file.close()
        if hasattr(self, 'poses_csv_file') and self.poses_csv_file and not self.poses_csv_file.closed:
            self.poses_csv_file.close()
        
        if hasattr(self, 'frame_index'):
            self.get_logger().info(f'Dataset recording completed - {self.frame_index} frames saved')

def main(args=None):
    rclpy.init(args=args)
    node = DatasetRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Dataset recorder stopped by user')
    except Exception as e:
        node.get_logger().error(f'Dataset recorder error: {str(e)}')
    finally:
        try:
            node.__del__()  # Ensure cleanup
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore shutdown errors

if __name__ == '__main__':
    main()