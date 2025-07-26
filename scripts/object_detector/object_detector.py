#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import ncnn

class YOLO11nDetector(Node):
    def __init__(self):
        super().__init__('yolo11n_ncnn_detector')
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/detections/image', 10)
        self.bridge = CvBridge()
        self.net = ncnn.Net()
        self.net.load_param('/workspace/src/kpi_rover/scripts/object_detector/yolo11n_ncnn_model/model.ncnn.param')
        self.net.load_model('/workspace/src/kpi_rover/scripts/object_detector/yolo11n_ncnn_model/model.ncnn.bin')
        self.labels = [
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
        "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
        "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
        "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
        "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
        "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
        "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
        "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        ]
        self.get_logger().info("Object detector started")


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        mat = ncnn.Mat.from_pixels_resize(frame, ncnn.Mat.PixelType.PIXEL_BGR, frame.shape[1], frame.shape[0], 640, 640)
        ex = self.net.create_extractor()
        ex.input('in0', mat)
        ret, output = ex.extract('out0')

        for i in range(output.h):
            values = output.row(i)
            conf = values[1]
            if conf > 0.4:
                x1, y1, x2, y2 = int(values[2]), int(values[3]), int(values[4]), int(values[5])
                label = self.labels[int(values[0])]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                self.get_logger().info(f"found object {label}")

        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YOLO11nDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
