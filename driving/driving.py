from typing import Optional, Literal, get_args, cast

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool

from driving.detection.basic_sat import Basic_Sat
from driving.detection.basic_sobel import Basic_Sobel
from driving.steering.basic_p import Basic_P



Steering = Literal['basic_p']
Detection = Literal['basic_sobel', 'basic_sat']


class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving')

        self.camera_msg: Optional[CompressedImage] = None
        self.camera_raw_msg: Optional[CompressedImage] = None
        self.camera_raw_16_9_msg: Optional[CompressedImage] = None

        self.lane_detection_algorithm: Detection = 'basic_sat'
        self.steering_algorithm: Steering = 'basic_p'

        # 1단계에서는 basic 조합을 고정해서 사용한다. 객체는 시작할 때 한 번 만든다.

        ## detector
        self.basic_sat = Basic_Sat()
        self.basic_sobel = Basic_Sobel()

        # controller
        self.basic_p = Basic_P()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.debug_pub1 = self.create_publisher(CompressedImage, 'debug1', 10)
        self.debug_pub2 = self.create_publisher(CompressedImage, 'debug2', 10)


        ## 카메라 소스 입력
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


        ## 알고리즘 선택
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

        ## 파라미터
        self.declare_parameter('basic_sat.beta', -5080.1)   # for basic_sat
        self.declare_parameter('basic.lane_mode', True)   # for basic_sat & sobel



## ==== CALL_BACK ====

    def camera_callback(self, msg: CompressedImage):
        self.camera_msg = msg

        if self.lane_detection_algorithm == 'basic_sat':
            try:
                # 영상 한 장을 검출 → 조향 순서로 처리한다.
                beta = self.get_parameter('basic_sat.beta').value
                lane_mode = self.get_parameter('basic.lane_mode').value
                # self.get_logger().info(f'beta: {beta}')
                center = self.basic_sat.detectCenterSaturation(msg, beta, lane_mode)
                # 정상 영상의 미검출(None)은 basic_p가 기존 극단값으로 처리한다.
                twist = self.basic_p.makeTwist(center)
            except (ValueError, cv2.error) as error:
                # 손상되거나 규격이 맞지 않는 영상은 차선 미검출과 구분한다.
                self.get_logger().warning(f'Camera processing failed: {error}')
                self.cmd_vel_pub.publish(Twist())
                return

            self.cmd_vel_pub.publish(twist)
            self.publish_debug_basic_sat()

        if self.lane_detection_algorithm == 'basic_sobel':
            try:
                lane_mode = self.get_parameter('basic.lane_mode').value
                center = self.basic_sobel.detectCenterSobel(msg, lane_mode)
                twist = self.basic_p.makeTwist(center)
            except (ValueError, cv2.error) as error:
                self.get_logger().warning(f'Camera processing failed: {error}')
                self.cmd_vel_pub.publish(Twist())
                return
            self.cmd_vel_pub.publish(twist)
            self.publish_debug_basic_sobel()

    def camera_raw_callback(self, msg: CompressedImage):
        self.camera_raw_msg = msg

    def camera_raw_16_9_callback(self, msg: CompressedImage):
        self.camera_raw_16_9_msg = msg

    # 선택 토픽은 아직 이름만 저장한다. 실제 객체 교체는 다음 단계에서 연결한다.
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


## ==== 개별 클래스 제어 ===

    def publish_debug_basic_sat(self):
        img = self.basic_sat.debug()
        if img is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.publish_compressed(self.debug_pub1, img, stamp)
        cv2.imshow("test", img)
        cv2.waitKey(1)

    def publish_debug_basic_sobel(self):
        img = self.basic_sobel.debug()
        if img is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.publish_compressed(self.debug_pub1, img, stamp)
        cv2.imshow("test", img)
        cv2.waitKey(1)


    def publish_compressed(self, publisher, frame, stamp):
        """Publish a JPEG frame with the timestamp shared by all three views."""
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            self.get_logger().warn(f'Failed to encode frame for {publisher.topic_name}.')
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = 'jpeg'
        msg.data = buffer.tobytes()
        publisher.publish(msg)



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
