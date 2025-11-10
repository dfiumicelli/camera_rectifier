#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import yaml
import os

class CameraRectifyNode(Node):
    def __init__(self):
        super().__init__('camera_rectify')
        
        # Carica camera_info
        yaml_path = os.path.join(os.path.dirname(__file__), 'mmal_service_16.1.yaml')
        self.get_logger().info(f"Cercando YAML in: {yaml_path}")
        
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            
            self.K = np.array(data['camera_matrix']['data']).reshape(3, 3)
            self.D = np.array(data['distortion_coefficients']['data'])
            self.width = data['image_width']
            self.height = data['image_height']
            
            self.get_logger().info(f"✓ YAML caricato: {self.width}x{self.height}")
            
        except Exception as e:
            self.get_logger().error(f"✗ Errore loading YAML: {e}")
            raise
        
        # Pre-calcola map
        try:
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.K, self.D,
                np.eye(3), self.K,
                (self.width, self.height),
                cv2.CV_32F
            )
            self.get_logger().info("✓ Rectification maps initialized")
        except Exception as e:
            self.get_logger().error(f"✗ Map init error: {e}")
            self.map1 = None
            self.map2 = None
        
        # Subscribe
        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.callback,
            10
        )
        
        # Publish
        self.pub = self.create_publisher(CompressedImage, '/image_rect/compressed', 10)
        
        self.callback_count = 0
        self.get_logger().info('✓ Camera rectify node started')

    def callback(self, msg):
        self.callback_count += 1
        
        # Log ogni 30 messaggi per non spam
        if self.callback_count % 30 == 0:
            self.get_logger().info(f"Processato {self.callback_count} messaggi")
        
        try:
            if self.map1 is None:
                self.get_logger().error("Map è None!")
                return
            
            # Decomprimi
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                self.get_logger().warning(f"Failed to decode image! Data size: {len(msg.data)}")
                return
            
            if self.callback_count % 30 == 0:
                self.get_logger().info(f"Frame decodificato: {frame.shape}")
            
            # Rettifica
            rectified = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
            
            # Comprimi
            ret, buffer = cv2.imencode('.jpg', rectified, [cv2.IMWRITE_JPEG_QUALITY, 100])
            
            if not ret:
                self.get_logger().error("Failed to encode image!")
                return
            
            # Pubblica
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = 'camera'
            compressed_msg.format = 'jpeg'
            compressed_msg.data = buffer.tobytes()
            
            self.pub.publish(compressed_msg)
            
            if self.callback_count % 30 == 0:
                self.get_logger().info(f"✓ Pubblicato: {len(compressed_msg.data)} bytes")
            
        except Exception as e:
            self.get_logger().error(f'✗ Callback error: {e}', exc_info=True)

def main(args=None):
    rclpy.init(args=args)
    node = CameraRectifyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
