from typing import Optional, Literal, get_args, cast
import math
import os
import time

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from driving.detection.basic_sat import Basic_Sat
from driving.detection.basic_sobel import Basic_Sobel
from driving.steering.basic_p import Basic_P
from driving.detection.yolo_lane import YoloLane
from driving.steering.pure_pursuit import LanePurePursuit



Steering = Literal['basic_p', 'pure_pursuit']
Detection = Literal['basic_sobel', 'basic_sat', 'yolo_lane']


class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving')

        self.camera_msg: Optional[CompressedImage] = None
        self.camera_raw_msg: Optional[CompressedImage] = None
        self.camera_raw_16_9_msg: Optional[CompressedImage] = None

        self.lane_detection_algorithm: Detection = 'basic_sat'
        self.steering_algorithm: Steering = 'basic_p'

        # 계산 객체를 재사용한다. 무거운 YOLO 모델은 첫 처리 때 한 번 읽는다.

        ## detector
        self.basic_sat = Basic_Sat()
        self.basic_sobel = Basic_Sobel()
        self.yolo_lane: Optional[YoloLane] = None
        self.yolo_msg: Optional[CompressedImage] = None
        self.last_yolo_processing_seconds = 0.0

        # controller
        self.basic_p = Basic_P()
        self.pure_pursuit = LanePurePursuit()

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
        self.declare_parameter('basic.lane_mode', True)   # for basic_sat, basic_sobel & basic_p

        default_model = os.path.join(
            get_package_share_directory('driving'), 'models', 'lane_yolo26n_seg.pt',
        )
        # 이 세 값은 시작할 때 적용한다. 실행 중 set은 거부하여 미반영을 숨기지 않는다.
        self.declare_parameter('yolo_lane.model_path', default_model,
                               ParameterDescriptor(read_only=True))
        self.declare_parameter('yolo_lane.device', '', ParameterDescriptor(read_only=True))
        self.declare_parameter('yolo_lane.process_hz', 15.0,
                               ParameterDescriptor(read_only=True))
        self.declare_parameter('yolo_lane.conf', 0.4)
        self.declare_parameter('yolo_lane.imgsz', 640)
        self.declare_parameter('yolo_lane.publish_debug', True)
        self.declare_parameter('yolo_lane.camera_height', 0.129)
        self.declare_parameter('yolo_lane.roi_top_row', 220)
        self.declare_parameter('yolo_lane.roi_half_width', 250)
        self.declare_parameter('yolo_lane.lookahead_row', 400)
        self.declare_parameter('yolo_lane.single_offset', 0.5)
        self.declare_parameter('yolo_lane.lane_width', 355.0)
        self.declare_parameter('pure_pursuit.v_max', 0.18)
        self.declare_parameter('pure_pursuit.v_lost', 0.05)
        self.declare_parameter('pure_pursuit.k_lat', 1.0)
        self.declare_parameter('pure_pursuit.max_ang', 1.2)
        self.declare_parameter('pure_pursuit.k_v', 0.6)
        self.declare_parameter('pure_pursuit.ang_smooth', 0.6)
        self.declare_parameter('pure_pursuit.v_single', 0.8)

        hz = self.get_parameter('yolo_lane.process_hz').value
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError('yolo_lane.process_hz must be finite and positive.')
        self.yolo_timer = self.create_timer(1.0 / hz, self.process_yolo)



## ==== CALL_BACK ====

    def camera_callback(self, msg: CompressedImage):
        self.camera_msg = msg

        # YOLO는 수신과 처리를 분리한다. 타이머가 소비하기 전에는 최신 수신본으로 덮어쓴다.
        if self.lane_detection_algorithm == 'yolo_lane':
            if self.steering_algorithm == 'pure_pursuit':
                self.yolo_msg = msg
            else:
                self.cmd_vel_pub.publish(Twist())
            return

        # basic의 픽셀 중심을 미터 목표점용 조향기에 전달하지 않는다.
        if self.steering_algorithm != 'basic_p':
            self.cmd_vel_pub.publish(Twist())
            return

        if self.lane_detection_algorithm == 'basic_sat':
            try:
                # 영상 한 장을 검출 → 조향 순서로 처리한다.
                beta = self.get_parameter('basic_sat.beta').value
                lane_mode = self.get_parameter('basic.lane_mode').value
                # self.get_logger().info(f'beta: {beta}')
                center = self.basic_sat.detectCenterSaturation(msg, beta, lane_mode)
                twist = self.basic_p.makeTwist(center, lane_mode)
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
                twist = self.basic_p.makeTwist(center, lane_mode)
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

    def process_yolo(self):
        """기본 15Hz. 새 영상 한 장을 검출 → 조향 → 발행하고 같은 영상을 재처리하지 않는다."""
        if (self.lane_detection_algorithm != 'yolo_lane'
                or self.steering_algorithm != 'pure_pursuit'):
            return
        msg, self.yolo_msg = self.yolo_msg, None
        if msg is None:
            return
        started = time.perf_counter()
        p = lambda name: self.get_parameter(name).value
        try:
            if self.yolo_lane is None:
                self.yolo_lane = YoloLane(p('yolo_lane.model_path'), p('yolo_lane.device'))
                self.get_logger().info('YOLO lane model loaded: ' + p('yolo_lane.model_path'))
            target, mode = self.yolo_lane.detectLane(
                msg, conf=p('yolo_lane.conf'), imgsz=p('yolo_lane.imgsz'),
                camera_height=p('yolo_lane.camera_height'),
                roi_top_row=p('yolo_lane.roi_top_row'),
                roi_half_width=p('yolo_lane.roi_half_width'),
                lookahead_row=p('yolo_lane.lookahead_row'),
                single_offset=p('yolo_lane.single_offset'),
                lane_width=p('yolo_lane.lane_width'),
            )
            twist = self.pure_pursuit.makeTwist(
                target, mode, v_max=p('pure_pursuit.v_max'),
                v_lost=p('pure_pursuit.v_lost'), k_lat=p('pure_pursuit.k_lat'),
                max_ang=p('pure_pursuit.max_ang'), k_v=p('pure_pursuit.k_v'),
                ang_smooth=p('pure_pursuit.ang_smooth'), v_single=p('pure_pursuit.v_single'),
            )
            self.cmd_vel_pub.publish(twist)
            self.publish_debug_yolo(msg)
        except Exception as error:
            # 모델 로딩/추론 실패도 노드를 종료시키지 않고 정지 처리한다.
            # 정상 미검출(None → v_lost 직진)과는 별개의 경로다.
            self.get_logger().error(f'YOLO processing failed: {error}')
            self.cmd_vel_pub.publish(Twist())
            self.reset_yolo()
        finally:
            self.last_yolo_processing_seconds = time.perf_counter() - started

    # 선택은 이름을 바꾼 뒤, 다음 새 영상부터 해당 분기를 실행한다.
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
        self.selection_changed()
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
        self.selection_changed()
        self.get_logger().info('Lane detection selection: ' + algorithm)


## ==== 개별 클래스 제어 ===

    def reset_yolo(self):
        self.yolo_msg = None
        if self.yolo_lane is not None:
            self.yolo_lane.reset()
        self.pure_pursuit.reset()

    def selection_changed(self):
        self.cmd_vel_pub.publish(Twist())
        self.reset_yolo()
        if ((self.lane_detection_algorithm == 'yolo_lane' and self.steering_algorithm != 'pure_pursuit')
                or (self.lane_detection_algorithm != 'yolo_lane' and self.steering_algorithm != 'basic_p')):
            self.get_logger().warning('Unsupported detection/steering pair; waiting with zero velocity.')

    def publish_debug_yolo(self, camera_msg):
        if not self.get_parameter('yolo_lane.publish_debug').value:
            return
        img = self.yolo_lane.debug(self.pure_pursuit.pursuit)
        if img is None:
            return
        # 추론 지연이 있어도 어느 입력 영상의 결과인지 알 수 있도록 header를 보존한다.
        self.publish_compressed(self.debug_pub1, img, camera_msg.header.stamp,
                                camera_msg.header.frame_id)

    def publish_debug_basic_sat(self):
        img = self.basic_sat.debug()
        if img is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.publish_compressed(self.debug_pub1, img, stamp)

    def publish_debug_basic_sobel(self):
        img = self.basic_sobel.debug()
        if img is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.publish_compressed(self.debug_pub1, img, stamp)


    def publish_compressed(self, publisher, frame, stamp, frame_id=''):
        """알고리즘의 표시용 영상을 JPEG로 발행한다."""
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            self.get_logger().warn(f'Failed to encode frame for {publisher.topic_name}.')
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
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
        if rclpy.ok():
            node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
