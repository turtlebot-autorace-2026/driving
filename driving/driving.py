from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from math import isfinite
from time import monotonic
from typing import Optional, Tuple, TypeVar, cast, get_args

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from driving.interfaces import CameraTopic, LaneDetector, SteeringController
from driving.registry import (
    DEFAULT_DETECTION, DEFAULT_STEERING, Detection, Steering, create_algorithms,
)


ResultT = TypeVar('ResultT')


@dataclass
class FrameJob:
    generation: int
    received_at: float
    future: Future


class DrivingNode(Node):
    # 수신 시각 기준 유효 결과의 최대 나이. 촬영 장치의 시계와는 비교하지 않는다.
    FRAME_TIMEOUT_SEC = 0.5
    POLL_PERIOD_SEC = 0.05

    def __init__(self):
        super().__init__('driving')

        self.camera_msg: Optional[CompressedImage] = None
        self.camera_raw_msg: Optional[CompressedImage] = None
        self.camera_raw_16_9_msg: Optional[CompressedImage] = None

        self.steering_algorithm: Steering = DEFAULT_STEERING
        self.lane_detection_algorithm: Detection = DEFAULT_DETECTION
        self.detector, self.controller = create_algorithms(
            self.lane_detection_algorithm, self.steering_algorithm,
        )

        # ROS 콜백은 가볍게 유지하고, 영상 처리는 한 worker에서 순서대로 한다.
        # 처리 중 새 영상이 오면 대기 영상 한 장만 최신값으로 교체한다.
        self._worker = ThreadPoolExecutor(max_workers=1)
        self._generation = 0
        self._pending_frame: Optional[Tuple[float, CompressedImage]] = None
        self._job: Optional[FrameJob] = None
        self._last_valid_frame_at: Optional[float] = None
        self._closed = False
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

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
        # /clock 또는 use_sim_time이 멈춰도 수신 중단을 감시한다.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.processing_timer = self.create_timer(
            self.POLL_PERIOD_SEC, self._poll_pipeline, clock=self._steady_clock,
        )

    def camera_callback(self, msg: CompressedImage):
        self.camera_msg = msg
        self.process_frame('camera', msg)

    def camera_raw_callback(self, msg: CompressedImage):
        self.camera_raw_msg = msg
        self.process_frame('camera_raw', msg)

    def camera_raw_16_9_callback(self, msg: CompressedImage):
        self.camera_raw_16_9_msg = msg
        self.process_frame('camera_raw_16_9', msg)

    def process_frame(self, input_topic: CameraTopic, msg: CompressedImage):
        # 세 영상 모두 구독하지만 선택된 검출기의 입력만 실행한다.
        if self._closed or input_topic != self.detector.input_topic:
            return
        self._pending_frame = (monotonic(), msg)
        self._start_next_frame()

    def _start_next_frame(self):
        if self._job is not None or self._pending_frame is None:
            return
        received_at, msg = self._pending_frame
        self._pending_frame = None
        if monotonic() - received_at > self.FRAME_TIMEOUT_SEC:
            return
        self._job = FrameJob(
            self._generation, received_at,
            self._worker.submit(self._run_algorithms, self.detector, self.controller, msg),
        )

    @staticmethod
    def _run_algorithms(
        detector: LaneDetector[ResultT], controller: SteeringController[ResultT],
        msg: CompressedImage,
    ) -> Twist:
        # 알고리즘마다 다른 메서드명을 노드에서 분기하지 않는다.
        result = detector.detect(msg)
        twist = controller.compute(result)
        if not isinstance(twist, Twist):
            raise TypeError('Steering compute() must return geometry_msgs/Twist.')
        values = (
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError('Steering returned a non-finite velocity.')
        return twist

    def _poll_pipeline(self):
        if self._closed:
            return
        now = monotonic()
        if self._job is not None and self._job.future.done():
            job, self._job = self._job, None
            # 선택 변경 전 결과와 처리 시간이 너무 긴 결과는 발행하지 않는다.
            if job.generation == self._generation and now - job.received_at <= self.FRAME_TIMEOUT_SEC:
                try:
                    twist = job.future.result()
                except Exception as error:
                    # 결과의 의미는 알고리즘이 처리하고, 노드는 실행 오류를 처리한다.
                    self.get_logger().warning(f'Frame processing failed: {error}')
                    self._last_valid_frame_at = None
                else:
                    self._last_valid_frame_at = job.received_at
                    self.cmd_vel_pub.publish(twist)

        if self._last_valid_frame_at is None or now - self._last_valid_frame_at > self.FRAME_TIMEOUT_SEC:
            self.cmd_vel_pub.publish(Twist())
        self._start_next_frame()

    def _replace_algorithms(
        self, detection: Detection, steering: Steering,
    ) -> bool:
        try:
            detector, controller = create_algorithms(detection, steering)
        except Exception as error:
            self.get_logger().warning(f'Algorithm selection rejected: {error}')
            return False

        # 생성/호환성 확인 후 이름과 객체를 함께 교체한다.
        # 두 객체를 새로 만들어 이전 조합의 처리 이력을 초기화한다.
        self.detector, self.controller = detector, controller
        self.lane_detection_algorithm = detection
        self.steering_algorithm = steering
        self._generation += 1
        self._pending_frame = None
        self._last_valid_frame_at = None
        self.camera_msg = None
        self.camera_raw_msg = None
        self.camera_raw_16_9_msg = None
        self.cmd_vel_pub.publish(Twist())
        return True

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
        if self._replace_algorithms(
            self.lane_detection_algorithm, cast(Steering, algorithm),
        ):
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
        if self._replace_algorithms(
            cast(Detection, algorithm), self.steering_algorithm,
        ):
            self.get_logger().info('Lane detection selection: ' + algorithm)

    def destroy_node(self):
        self._closed = True
        self.processing_timer.cancel()
        self._pending_frame = None
        if self.context.ok():
            self.cmd_vel_pub.publish(Twist())
        # Worker는 ROS 객체를 사용하지 않으며 처리 완료 후 함께 정리한다.
        self._worker.shutdown(wait=True)
        return super().destroy_node()


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
