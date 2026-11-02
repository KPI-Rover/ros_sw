#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class YUVToRGBConverter(Node):
    def __init__(self):
        super().__init__('yuv_to_rgb_converter')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage,
            '/in',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            Image,
            '/out',
            10
        )

    def image_callback(self, msg):
       # Decode the incoming YUV422 image
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        yuv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if yuv_image is None:
            self.get_logger().error('Failed to decode YUV422 image')
            return

        # Convert YUV422 to RGB
        rgb_image = cv2.cvtColor(yuv_image, cv2.COLOR_BGR2RGB)

        # Convert the RGB OpenCV image to a ROS Image message
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
        rgb_msg.header = msg.header  # Preserve the original message header

        # Publish the converted image
        self.publisher.publish(rgb_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YUVToRGBConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
