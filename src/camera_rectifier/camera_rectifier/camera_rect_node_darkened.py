#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
import yaml
import os

class CameraRectifyNode(Node):
    def __init__(self):
        super().__init__('camera_rectify')
        
        # Carica camera_info
        yaml_path = os.path.join(os.path.dirname(__file__), 'mmal_service_16.1.yaml')
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        self.K = np.array(data['camera_matrix']['data']).reshape(3, 3)
        self.D = np.array(data['distortion_coefficients']['data'])
        self.width = data['image_width']
        self.height = data['image_height']
        
        self.get_logger().info(f"Camera: {self.width}x{self.height}")
        self.get_logger().info(f"Distortion coefficients: {self.D}")
        
        # ⭐ BRIGHTNESS CONTROL
        self.brightness_reduction = 0.7  # 0.7 = 30% oscuramento (70% dell'originale)
        # Usa valori tra:
        # 1.0 = nessun oscuramento (originale)
        # 0.9 = 10% oscuramento
        # 0.8 = 20% oscuramento
        # 0.7 = 30% oscuramento (default)
        # 0.6 = 40% oscuramento
        # 0.5 = 50% oscuramento (molto scuro)
        
        self.get_logger().info(f"⭐ Brightness reduction: {(1 - self.brightness_reduction)*100:.0f}% darker")
        
        # Pre-calcola map per plumb_bob (radtan)
        try:
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.K, self.D,
                np.eye(3), self.K,
                (self.width, self.height),
                cv2.CV_32F
            )
            self.get_logger().info("Rectification maps initialized")
        except Exception as e:
            self.get_logger().error(f"Map init error: {e}")
            self.map1 = None
            self.map2 = None
        
        # Subscribe a compressed
        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.callback,
            10
        )
        
        # Publish rettificata COMPRESSO
        self.pub = self.create_publisher(CompressedImage, '/image_rect/compressed', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Camera rectify node started (plumb_bob)')
    
    def _darken_image(self, image):
        """⭐ Oscura l'immagine moltiplicando per il fattore di riduzione
        
        Metodo: moltiplicazione pixel-wise
        - Efficiente (single pass)
        - Mantiene i dettagli
        - Preserva il contrasto relativo
        """
        # Converti a float per evitare overflow
        darkened = image.astype(np.float32) * self.brightness_reduction
        
        # Clamp ai limiti [0, 255]
        darkened = np.clip(darkened, 0, 255)
        
        # Converti back a uint8
        darkened = darkened.astype(np.uint8)
        
        return darkened
    
    def callback(self, msg):
        try:
            if self.map1 is None:
                return
            
            # Decomprimi da CompressedImage
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                return
            
            # ⭐ Rettifica
            rectified = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
            
            # ⭐ OSCURA L'IMMAGINE
            darkened = self._darken_image(rectified)
            
            # Comprimi l'immagine
            ret, buffer = cv2.imencode('.jpg', darkened, [cv2.IMWRITE_JPEG_QUALITY, 100])
            
            if not ret:
                return
            
            # Crea messaggio CompressedImage
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = 'camera'
            compressed_msg.format = 'jpeg'
            compressed_msg.data = buffer.tobytes()
            
            self.pub.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Callback error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraRectifyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
