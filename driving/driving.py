"""영상 → 검출 → 조향 → 발행을 연결하는 단일 ROS 노드."""

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from math import isfinite
from time import monotonic
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from driving.registry import (
    DEFAULT_DETECTION, DEFAULT_STEERING, Detection, Steering, create_algorithms,
)
from driving.steering.basic_p import Basic_P


# ROS에 공개할 이름과 시작 기본값이다. 값의 유효성 검사는 _validate_parameters가 맡는다.
# 각 알고리즘의 모든 상수를 자동으로 파라미터로 만드는 구조는 아니다.
BASIC_P_PARAMETERS = {
    'basic_p.left.normal_angular_divisor': Basic_P.LEFT_LANE_CONFIG.normal_angular_divisor,
    'basic_p.right.normal_angular_divisor': Basic_P.RIGHT_LANE_CONFIG.normal_angular_divisor,
}


@dataclass
class FrameJob:
    # 노드가 worker에 맡긴 한 작업을 추적하는 정보다. ROS 메시지가 아닌 Python 객체다.
    # dataclass가 아래 필드를 받는 생성자를 만들어 준다.
    # 예: FrameJob(3, 수신시각, future) → 3번 설정으로 시작한 영상 처리 작업.
    # worker가 끝났더라도 감시 콜백이 회수하기 전까지 self._job에 남을 수 있다.
    # 선택/설정 변경 때 증가하는 번호. 영상마다 증가하는 프레임 번호는 아니다.
    generation: int
    # 활성 영상 콜백에서 기록한 monotonic() 시각. 카메라 header.stamp와는 다르다.
    received_at: float
    # submit()이 반환하는 Future. 완료 여부, 반환값 또는 처리 중 예외를 확인하는 통로다.
    future: Future


@dataclass
class FrameResult:
    # _run_algorithms가 성공적으로 끝났을 때 반환하는 한 프레임의 계산 결과 묶음이다.
    # Future.result()로 이 객체를 꺼낸 뒤, 노드가 안의 메시지를 각각 발행한다.
    # 차선 검출 원본 결과(center/경로 등)는 detect → compute 사이에서만 전달한다.
    twist: Twist
    # 선택 기능이다. 메서드 미구현/영상 없음/디버깅 실패 시 None을 유지한다.
    debug_image: Optional[CompressedImage] = None
    # 디버깅 과정만 실패한 경우의 경고 문구. 유효한 twist는 함께 반환할 수 있다.
    debug_error: Optional[str] = None


class DrivingNode(Node):
    # 영상 수신부터 결과를 발행하려는 순간까지 허용하는 최대 나이다.
    # 대기·계산·JPEG 생성·타이머 대기를 포함하며, 촬영부터 수신까지의 지연은 제외한다.
    FRAME_TIMEOUT_SEC = 0.5
    # 완료 결과 확인과 입력 중단 감시를 요청하는 타이머 주기. 실제 실행 간격은 지연될 수 있다.
    POLL_PERIOD_SEC = 0.05

    def __init__(self, *, parameter_overrides=None):
        super().__init__('driving', parameter_overrides=parameter_overrides)
        self.steering_algorithm: Steering = DEFAULT_STEERING
        self.lane_detection_algorithm: Detection = DEFAULT_DETECTION
        try:
            # 1) 파라미터 준비 → 2) 시작 값 검사 → 3) 기본 알고리즘 생성/설정 적용.
            # CLI/YAML에서 넣은 시작 값도 declare_parameter를 거친 뒤 검사한다.
            for name, default in BASIC_P_PARAMETERS.items():
                self.declare_parameter(name, default)
            validation = self._validate_parameters([
                self.get_parameter(name) for name in BASIC_P_PARAMETERS
            ])
            if not validation.successful:
                raise ValueError(validation.reason)
            self.add_on_set_parameters_callback(self._validate_parameters)
            self.detector, self.controller = create_algorithms(
                self.lane_detection_algorithm, self.steering_algorithm,
            )
            self.controller = self._configure_basic_p(self.controller)
        except Exception:
            # 초기 설정이 잘못되면 부분 생성된 ROS 노드 자원을 정리하고 시작을 중단한다.
            super().destroy_node()
            raise

        # 세 토픽의 마지막 메시지는 각각 보관한다. 동기화된 영상 묶음은 아니다.
        self.camera_msg = self.camera_raw_msg = self.camera_raw_16_9_msg = None
        # 오래 걸릴 수 있는 영상 계산을 ROS 콜백 밖의 한 스레드에 맡긴다.
        # 같은 검출/조향 객체의 프레임 처리가 동시에 여러 번 실행되지 않도록 worker는 하나다.
        self._worker = ThreadPoolExecutor(max_workers=1)
        self._generation = 0  # 선택/설정 변경 전 작업 결과를 구분하는 번호
        # 아직 시작하지 않은 최신 영상 한 장: (수신 시각, 메시지).
        self._pending_frame: Optional[Tuple[float, CompressedImage]] = None
        # 맡겨 둔 작업 한 개. None이면 다음 영상을 worker에 제출할 수 있다.
        self._job: Optional[FrameJob] = None
        # 마지막으로 유효 결과를 발행한 영상의 '수신 시각'. 발행 시각을 저장하지 않는다.
        self._last_valid_frame_at: Optional[float] = None
        self._closed = False
        # 토픽 이름은 상대 이름이므로 namespace/remap에 따라 실제 경로가 바뀐다.
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.debug_image_pub = self.create_publisher(
            CompressedImage, 'driving/debug/roi/compressed', qos_profile_sensor_data,
        )
        self.camera_subscription = self.create_subscription(
            CompressedImage, 'camera', self.camera_callback, qos_profile_sensor_data,
        )
        self.camera_raw_subscription = self.create_subscription(
            CompressedImage, 'camera_raw', self.camera_raw_callback, qos_profile_sensor_data,
        )
        self.camera_raw_16_9_subscription = self.create_subscription(
            CompressedImage, 'camera_raw_16_9', self.camera_raw_16_9_callback, qos_profile_sensor_data,
        )
        self.steering_selection_subscription = self.create_subscription(
            String, 'driving/steering', self.steering_selection_callback, 10,
        )
        self.lane_detection_selection_subscription = self.create_subscription(
            String, 'driving/detection', self.lane_detection_selection_callback, 10,
        )
        # /clock이나 use_sim_time이 멈춰도 감시 타이머가 진행하도록 steady clock을 사용한다.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.processing_timer = self.create_timer(
            self.POLL_PERIOD_SEC, self._poll_pipeline, clock=self._steady_clock,
        )

    # 1. 실제 주행 계산: 결과의 형식과 미검출 의미는 두 알고리즘이 정한다.
    @staticmethod
    def _run_algorithms(detector, controller, msg: CompressedImage) -> FrameResult:
        # worker가 실행하는 함수다. self를 받지 않고, 제출 당시의 객체와 영상만 사용한다.
        # 이 함수는 ROS 발행을 하지 않는다. 지금 발행해도 되는지는 감시 콜백이 판단한다.
        result = detector.detect(msg)
        twist = controller.compute(result)
        # 결과 형식/미검출 정책은 알고리즘이 정하지만, 최종 출력은 유한한 Twist여야 한다.
        if not isinstance(twist, Twist):
            raise TypeError('compute() must return geometry_msgs/Twist.')
        values = (
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError('Steering returned a non-finite velocity.')

        frame = FrameResult(twist)
        try:
            # 디버깅 메서드가 있으면 같은 프레임의 JPEG를 받는다. 없으면 조향만 반환한다.
            get_debug_image = getattr(detector, 'get_debug_image', None)
            if get_debug_image is not None:
                debug = get_debug_image()
                if debug is not None:
                    if not isinstance(debug, CompressedImage) or not debug.data:
                        raise TypeError('get_debug_image() must return a nonempty CompressedImage or None.')
                    frame.debug_image = debug
        except Exception as error:
            frame.debug_error = str(error)  # 디버깅 실패만으로 정상 조향을 버리지 않는다.
        return frame

    # 2. 영상 입력: 모두 보관하되 활성 검출기의 입력 한 종류만 처리한다.
    def camera_callback(self, msg: CompressedImage):
        self.camera_msg = msg
        self.process_frame('camera', msg)

    def camera_raw_callback(self, msg: CompressedImage):
        self.camera_raw_msg = msg
        self.process_frame('camera_raw', msg)

    def camera_raw_16_9_callback(self, msg: CompressedImage):
        self.camera_raw_16_9_msg = msg
        self.process_frame('camera_raw_16_9', msg)

    def process_frame(self, input_topic: str, msg: CompressedImage):
        # 세 영상 콜백이 여기로 모인다. 현재 검출기가 요구하는 입력만 계산 대상으로 삼는다.
        if self._closed or input_topic != self.detector.input_topic:
            return
        self._refresh_controller_parameters()
        # 처리 중 A가 있고 B가 대기 중일 때 C가 오면, B를 C로 교체한다.
        # 입력이 빨라도 오래된 영상을 무한히 쌓지 않는다. monotonic은 시간 차 계산용 시계다.
        self._pending_frame = (monotonic(), msg)
        # worker가 비어 있으면 바로 시작하고, 작업이 남아 있으면 최신 영상만 대기시킨다.
        self._start_next_frame()

    # 3. 선택 토픽: 이름으로 생성한다. 검출 결과와 조향 입력을 맞추는 것은 팀 코드의 책임이다.
    def steering_selection_callback(self, msg: String):
        self._replace_algorithms(self.lane_detection_algorithm, msg.data.strip())

    def lane_detection_selection_callback(self, msg: String):
        self._replace_algorithms(msg.data.strip(), self.steering_algorithm)

    def _replace_algorithms(self, detection: Detection, steering: Steering) -> bool:
        # 같은 선택은 객체를 다시 만들거나 이전 처리 상태를 지우지 않는다.
        if (detection, steering) == (self.lane_detection_algorithm, self.steering_algorithm):
            return False
        try:
            # 생성이 모두 성공하기 전까지 기존 선택을 유지한다. 생성 함수는 빠르게 끝나야 한다.
            detector, controller = create_algorithms(detection, steering)
            controller = self._configure_basic_p(controller)
        except Exception as error:
            self.get_logger().warning(f'Algorithm selection rejected: {error}')
            return False
        # 한쪽 이름만 바꾸어도 두 객체를 새로 만들어 내부 처리 이력을 초기화한다.
        self.detector, self.controller = detector, controller
        self.lane_detection_algorithm, self.steering_algorithm = detection, steering
        self._reset_processing()
        self.camera_msg = self.camera_raw_msg = self.camera_raw_16_9_msg = None
        self.get_logger().info(f'Algorithms: {detection} / {steering}')
        return True

    # 4. basic_p 파라미터 두 개: 선언은 __init__, 검증과 적용은 아래에서 한다.
    def _validate_parameters(self, parameters) -> SetParametersResult:
        # ROS가 변경을 확정하기 전에 호출한다. 여기서는 검사 결과만 반환한다.
        # 나중의 다른 검증에서 거부될 수도 있으므로 알고리즘 객체를 여기서 바꾸지 않는다.
        for parameter in parameters:
            if parameter.name in BASIC_P_PARAMETERS:
                value = parameter.value
                if type(value) is not float or not isfinite(value) or value <= 0:
                    return SetParametersResult(
                        successful=False, reason=parameter.name + ' must be a finite double > 0.',
                    )
        return SetParametersResult(successful=True)

    def _configure_basic_p(self, controller):
        # basic_p만의 설정이다. 다른 조향기나 별도로 만든 하위 클래스에는 적용하지 않는다.
        if type(controller) is not Basic_P:
            return controller
        left = self.get_parameter('basic_p.left.normal_angular_divisor').value
        right = self.get_parameter('basic_p.right.normal_angular_divisor').value
        if (left, right) == (controller.left_divisor, controller.right_divisor):
            return controller
        # 실행 중인 worker의 객체는 건드리지 않고, 확정된 값으로 새 객체를 만든다.
        return Basic_P(controller.lane_mode, left_divisor=left, right_divisor=right)

    def _refresh_controller_parameters(self):
        # 영상/감시 콜백에서 ROS에 확정 저장된 값을 읽는다. 같은 값이면 아무것도 바꾸지 않는다.
        controller = self._configure_basic_p(self.controller)
        if controller is not self.controller:
            self.controller = controller
            self._reset_processing()

    # 5. 실행 관리: 한 worker + 최신 대기 영상 한 장. 계산 결과는 감시 콜백에서 발행한다.
    def _start_next_frame(self):
        # _job이 완료된 Future를 담고 있어도, 감시 콜백이 회수할 때까지 새 제출은 기다린다.
        if self._job is not None or self._pending_frame is None:
            return
        received_at, msg = self._pending_frame
        self._pending_frame = None
        # worker를 기다리는 동안 이미 오래된 영상은 계산도 시작하지 않는다.
        if monotonic() - received_at > self.FRAME_TIMEOUT_SEC:
            return
        self._job = FrameJob(
            self._generation, received_at,
            self._worker.submit(self._run_algorithms, self.detector, self.controller, msg),
        )

    def _poll_pipeline(self):
        if self._closed:
            return
        self._refresh_controller_parameters()
        now = monotonic()
        # done()을 먼저 확인하므로 result()가 영상 처리 완료를 기다리며 콜백을 막지 않는다.
        # Future는 정상 반환뿐 아니라 예외로 끝났을 때도 done()이 True다.
        if self._job is not None and self._job.future.done():
            # 로컬 변수로 회수하고 슬롯을 비운다. 마지막 단계에서 다음 작업을 시작할 수 있다.
            job, self._job = self._job, None
            # 이전 선택/설정의 결과와 수신 후 0.5초 넘게 걸린 결과는 발행하지 않는다.
            if job.generation == self._generation and now - job.received_at <= self.FRAME_TIMEOUT_SEC:
                try:
                    # 정상 완료면 FrameResult, 처리 실패면 worker에서 발생한 예외가 다시 나온다.
                    frame = job.future.result()
                except Exception as error:
                    self.get_logger().warning(f'Frame processing failed: {error}')
                    # 아래 공통 감시 분기에서 0 Twist를 발행하도록 유효 시각을 지운다.
                    self._last_valid_frame_at = None
                else:
                    self._last_valid_frame_at = job.received_at
                    self.cmd_vel_pub.publish(frame.twist)
                    if frame.debug_error is not None:
                        self.get_logger().warning(f'Debug image failed: {frame.debug_error}')
                    if frame.debug_image is not None:
                        self.debug_image_pub.publish(frame.debug_image)
        # 유효 결과를 아직 받지 못했거나 그 영상이 0.5초보다 오래됐으면 정지 명령을 보낸다.
        # 따라서 worker가 계산 중이어도 감시 콜백이 실행되는 동안 중단 감시가 가능하다.
        if self._last_valid_frame_at is None or now - self._last_valid_frame_at > self.FRAME_TIMEOUT_SEC:
            self.cmd_vel_pub.publish(Twist())
        self._start_next_frame()

    def _reset_processing(self):
        # 실행 중인 작업을 강제로 취소하지 않는다. 번호를 바꿔 늦게 온 결과를 무효로 만든다.
        # 예: job.generation=2, 현재=3이면 그 작업의 Twist/JPEG를 발행하지 않는다.
        # _job은 남겨 두므로 기존 작업이 끝나 회수된 후에야 새 영상을 처리할 수 있다.
        self._generation += 1
        self._pending_frame = None
        self._last_valid_frame_at = None
        self.cmd_vel_pub.publish(Twist())

    def destroy_node(self):
        # 새 입력/타이머 처리를 막고, ROS가 살아 있으면 정지 명령을 보낸 뒤 worker를 정리한다.
        self._closed = True
        self.processing_timer.cancel()
        self._pending_frame = None
        if self.context.ok():
            self.cmd_vel_pub.publish(Twist())
        # 유한 시간에 반환하는 알고리즘이 전제다. 영구 정지한 worker는 강제 종료하지 못한다.
        self._worker.shutdown(wait=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
    try:
        # ROS가 영상·선택·파라미터·타이머 콜백을 호출하도록 실행한다.
        # 기본 콜백 그룹의 상태 변경은 직렬로 처리하고, 영상 계산만 별도 worker가 맡는다.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
