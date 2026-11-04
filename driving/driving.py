from typing import Optional, Literal, get_args, cast

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

Steering = Literal['basic_p', 'pure_pursuit']
Detection = Literal['basic_sobel', 'basic_sat', 'yolo']


class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving')

        self.camera_msg: Optional[CompressedImage] = None
        self.camera_raw_msg: Optional[CompressedImage] = None
        self.camera_raw_16_9_msg: Optional[CompressedImage] = None

        self.steering_algorithm: Steering = 'basic_p'
        self.lane_detection_algorithm: Detection = 'basic_sat'

        self.camera_subscription = self.create_subscription(
            CompressedImage,
            'camera',
            self.camera_callback,
            qos_profile_sensor_data,
        )
        self.camera_raw_subscription = self.create_subscription(
            CompressedImage,
            'camera_raw',
            self.camera_raw_callback,
            qos_profile_sensor_data,
        )
        self.camera_raw_16_9_subscription = self.create_subscription(
            CompressedImage,
            'camera_raw_16_9',
            self.camera_raw_16_9_callback,
            qos_profile_sensor_data,
        )
        self.steering_selection_subscription = self.create_subscription(
            String,
            'driving/steering',
            self.steering_selection_callback,
            10,
        )
        self.lane_detection_selection_subscription = self.create_subscription(
            String,
            'driving/detection',
            self.lane_detection_selection_callback,
            10,
        )

    def camera_callback(self, msg: CompressedImage):
        self.camera_msg = msg

    def camera_raw_callback(self, msg: CompressedImage):
        self.camera_raw_msg = msg

    def camera_raw_16_9_callback(self, msg: CompressedImage):
        self.camera_raw_16_9_msg = msg

    def steering_selection_callback(self, msg: String):
        algorithm = msg.data.strip()
        allowed = get_args(Steering)

        if not algorithm:
            self.get_logger().warning('Ignoring empty steering algorithm name.')
            return
        if algorithm not in allowed:
            self.get_logger().warning(f'not allowed {algorithm}. allowed: {allowed}')
            return
        if algorithm == self.steering_algorithm:
            return
        self.steering_algorithm = cast(Steering, algorithm)
        self.get_logger().info('Steering selection: ' + algorithm)

    def lane_detection_selection_callback(self, msg: String):
        algorithm = msg.data.strip()
        allowed = get_args(Detection)

        if not algorithm:
            self.get_logger().warning('Ignoring empty lane detection algorithm name.')
            return
        if algorithm not in allowed:
            self.get_logger().warning(f'not allowed {algorithm}. allowed: {allowed}')
            return
        if algorithm == self.lane_detection_algorithm:
            return
        self.lane_detection_algorithm = cast(Detection, algorithm)
        self.get_logger().info('Lane detection selection: ' + algorithm)


def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
