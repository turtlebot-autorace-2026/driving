"""합성 입력만 사용한다. ROS 통신은 /driving_test namespace로 격리한다."""

import time
import unittest
from dataclasses import dataclass
from threading import Event
from typing import Tuple
from unittest.mock import patch

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from driving import basic
from driving.basic import ROI_CENTER_320
from driving.detection.basic_sat import Basic_Sat
from driving.driving import DrivingNode
from driving.registry import CONTROLLERS, DETECTORS, create_algorithms
from driving.steering.basic_p import Basic_P


def image_message(image):
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 100])
    assert ok
    return CompressedImage(format='jpeg', data=encoded.tobytes())


def straight_lane():
    bev = np.zeros((120, 640, 3), dtype=np.uint8)
    for x in (60, 580):
        cv2.rectangle(bev, (x - 12, 0), (x + 12, 119), (255, 255, 255), -1)
    matrix = cv2.getPerspectiveTransform(
        np.float32([[90, 360], [550, 360], [640, 480], [0, 480]]),
        np.float32([[20, 0], [620, 0], [640, 120], [0, 120]]),
    )
    return image_message(cv2.warpPerspective(bev, np.linalg.inv(matrix), (640, 480)))


class RawDetector:
    input_topic = 'camera_raw'
    output_type = ROI_CENTER_320

    def __init__(self):
        self.seen = []

    def detect(self, msg):
        center = msg.data[0]
        self.seen.append(center)
        return center


class AlternateController:
    input_type = ROI_CENTER_320

    def compute(self, center):
        twist = Twist()
        twist.linear.x = center / 1000.0
        return twist


@dataclass(frozen=True)
class PathResult:
    points: Tuple[Tuple[float, float], ...]


class PathDetector:
    input_topic = 'camera_raw_16_9'
    output_type = 'test_path_in_meters_v1'

    def detect(self, msg: CompressedImage) -> PathResult:
        # 이 조합은 미검출을 None이 아닌 빈 경로로 표현한다.
        points = ((1.0, msg.data[0] / 100.0),) if msg.data else ()
        return PathResult(points)


class PathController:
    input_type = 'test_path_in_meters_v1'

    def compute(self, result: PathResult) -> Twist:
        self.last_result = result
        twist = Twist()
        if result.points:
            twist.linear.x = 0.1
            twist.angular.z = result.points[0][1] / 2.0
        return twist


class AlgorithmTests(unittest.TestCase):
    def test_straight_lane_in_both_modes(self):
        msg = straight_lane()
        for lane_mode, expected in ((True, 60), (False, 260)):
            with patch.object(basic, 'SETTINGS', basic.BasicSettings(lane_mode=lane_mode)):
                detector, controller = create_algorithms('basic_sat', 'basic_p')
            center = detector.detect(msg)
            self.assertEqual(center, expected)
            twist = controller.compute(center)
            self.assertAlmostEqual(twist.linear.x, 0.22)
            self.assertEqual(twist.angular.z, 0.0)

    def test_valid_blank_frame_preserves_extreme_value_policy(self):
        msg = image_message(np.zeros((480, 640, 3), dtype=np.uint8))
        for mode, velocity, angular in (
            (True, 0.132, 1.0 / 1.7),
            (False, 0.14478632478632478, (-60.0 / 77.0) / 1.3),
        ):
            with patch.object(basic, 'SETTINGS', basic.BasicSettings(lane_mode=mode)):
                detector, controller = create_algorithms('basic_sat', 'basic_p')
            self.assertIsNone(detector.detect(msg))
            twist = controller.compute(None)
            self.assertAlmostEqual(twist.linear.x, velocity)
            self.assertAlmostEqual(twist.angular.z, angular)

    def test_invalid_images_raise_instead_of_returning_missing_lane(self):
        detector = Basic_Sat()
        cases = [
            None, CompressedImage(), CompressedImage(data=b'broken JPEG'),
            image_message(np.zeros((720, 1280, 3), dtype=np.uint8)),
        ]
        for msg in cases:
            with self.subTest(msg=type(msg).__name__):
                with self.assertRaises((ValueError, cv2.error)):
                    detector.detect(msg)

    def test_invalid_centers_are_rejected(self):
        for center in (float('nan'), float('inf'), -1, 320, 580):
            with self.subTest(center=center), self.assertRaises(ValueError):
                Basic_P().compute(center)

    def test_registry_rejects_unimplemented_names(self):
        for detection, steering in (
            ('yolo', 'basic_p'), ('basic_sobel', 'basic_p'),
            ('basic_sat', 'pure_pursuit'),
        ):
            with self.assertRaises(ValueError):
                create_algorithms(detection, steering)

    def test_basic_settings_do_not_affect_other_algorithms(self):
        with patch.object(basic, 'SETTINGS', basic.BasicSettings(lane_mode=False)), \
                patch.dict(DETECTORS, yolo=PathDetector), \
                patch.dict(CONTROLLERS, pure_pursuit=PathController):
            detector, controller = create_algorithms('yolo', 'pure_pursuit')
        self.assertFalse(hasattr(detector, 'lane_mode'))
        self.assertFalse(hasattr(controller, 'lane_mode'))
        result = detector.detect(CompressedImage(data=b'\x0c'))
        self.assertEqual(result.points, ((1.0, 0.12),))
        twist = controller.compute(result)
        self.assertAlmostEqual(twist.angular.z, 0.06)
        self.assertIs(controller.last_result, result)


class NodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-r', '__ns:=/driving_test'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = DrivingNode()
        self.commands = []
        # 노드 자원은 실제로 만들되 콜백 단위 검사에서 속도를 외부로 보내지 않는다.
        self.publisher_patch = patch.object(self.node.cmd_vel_pub, 'publish', self.commands.append)
        self.publisher_patch.start()

    def tearDown(self):
        self.node.destroy_node()
        self.publisher_patch.stop()

    def finish_frame(self):
        deadline = time.monotonic() + 2.0
        while self.node._job is not None or self.node._pending_frame is not None:
            self.assertLess(time.monotonic(), deadline, 'Frame processing did not finish')
            self.node._poll_pipeline()
            time.sleep(0.002)

    def register_alternates(self, detector=RawDetector, controller=AlternateController):
        detector_patch = patch.dict(DETECTORS, basic_sobel=detector)
        controller_patch = patch.dict(CONTROLLERS, pure_pursuit=controller)
        detector_patch.start()
        controller_patch.start()
        self.addCleanup(detector_patch.stop)
        self.addCleanup(controller_patch.stop)

    def test_default_pipeline_and_unused_inputs(self):
        self.node.camera_raw_callback(CompressedImage(data=b'invalid'))
        self.node.camera_raw_16_9_callback(CompressedImage(data=b'invalid'))
        self.assertIsNone(self.node._job)
        self.assertEqual(self.commands, [])
        self.node.camera_callback(straight_lane())
        self.finish_frame()
        self.assertAlmostEqual(self.commands[-1].linear.x, 0.22)
        self.assertEqual(self.commands[-1].angular.z, 0.0)

    def test_selection_changes_actual_algorithms_and_source(self):
        self.register_alternates()
        self.node.lane_detection_selection_callback(String(data=' basic_sobel '))
        self.node.steering_selection_callback(String(data='pure_pursuit'))
        self.assertIsInstance(self.node.detector, RawDetector)
        self.assertIsInstance(self.node.controller, AlternateController)
        self.node.camera_callback(straight_lane())
        self.assertIsNone(self.node._job)
        self.node.camera_raw_callback(CompressedImage(data=b'\x64'))
        self.finish_frame()
        self.assertAlmostEqual(self.commands[-1].linear.x, 0.1)
        self.assertEqual(self.node.detector.seen, [100])

    def test_rejected_and_same_selections_preserve_objects(self):
        original = (self.node.detector, self.node.controller)
        for name in ('', 'unknown', 'yolo', 'basic_sobel', 'basic_sat'):
            self.node.lane_detection_selection_callback(String(data=name))
        for name in (' ', 'unknown', 'pure_pursuit', 'basic_p'):
            self.node.steering_selection_callback(String(data=name))
        self.assertEqual(original, (self.node.detector, self.node.controller))
        self.assertEqual(self.node.lane_detection_algorithm, 'basic_sat')
        self.assertEqual(self.node.steering_algorithm, 'basic_p')
        self.assertEqual(self.commands, [])

    def test_incompatible_pair_and_constructor_failure_keep_current_pair(self):
        class Incompatible(RawDetector):
            output_type = 'path_in_meters'

        original = (self.node.detector, self.node.controller)
        with patch.dict(DETECTORS, basic_sobel=Incompatible):
            self.node.lane_detection_selection_callback(String(data='basic_sobel'))

        def broken_factory():
            raise RuntimeError('model is unavailable')

        with patch.dict(DETECTORS, basic_sobel=broken_factory):
            self.node.lane_detection_selection_callback(String(data='basic_sobel'))
        self.assertEqual(original, (self.node.detector, self.node.controller))
        self.assertEqual(self.commands, [])

    def test_invalid_frame_stops_and_next_valid_frame_recovers(self):
        msg = straight_lane()
        self.node.camera_callback(msg)
        self.finish_frame()
        self.assertGreater(self.commands[-1].linear.x, 0)
        self.node.camera_callback(CompressedImage(data=b'corrupt'))
        self.finish_frame()
        self.assertEqual(self.commands[-1], Twist())
        self.node.camera_callback(msg)
        self.finish_frame()
        self.assertAlmostEqual(self.commands[-1].linear.x, 0.22)

    def test_basic_pair_uses_its_own_settings_after_selection(self):
        self.register_alternates()
        self.node.camera_callback(straight_lane())
        self.finish_frame()
        with patch.object(basic, 'SETTINGS', basic.BasicSettings(lane_mode=False)):
            self.node.lane_detection_selection_callback(String(data='basic_sobel'))
            self.node.lane_detection_selection_callback(String(data='basic_sat'))
        self.assertFalse(self.node.detector.lane_mode)
        self.assertFalse(self.node.controller.lane_mode)
        self.assertFalse(hasattr(self.node, 'lane_mode'))
        self.assertFalse(hasattr(self.node, 'set_lane_mode'))
        self.assertEqual(self.commands[-1], Twist())
        self.assertIsNone(self.node.camera_msg)
        self.node.camera_callback(straight_lane())
        self.finish_frame()
        self.assertAlmostEqual(self.commands[-1].linear.x, 0.22)
        self.assertEqual(self.commands[-1].angular.z, 0.0)

    def test_structured_result_and_algorithm_specific_missing_detection(self):
        with patch.dict(DETECTORS, yolo=PathDetector), \
                patch.dict(CONTROLLERS, pure_pursuit=PathController):
            self.assertTrue(self.node._replace_algorithms('yolo', 'pure_pursuit'))
        self.assertFalse(hasattr(self.node.detector, 'lane_mode'))
        self.assertFalse(hasattr(self.node.controller, 'lane_mode'))
        self.node.camera_callback(straight_lane())
        self.assertIsNone(self.node._job)
        self.node.camera_raw_16_9_callback(CompressedImage(data=b'\x0c'))
        self.finish_frame()
        self.assertEqual(self.node.controller.last_result.points, ((1.0, 0.12),))
        self.assertAlmostEqual(self.commands[-1].linear.x, 0.1)
        self.assertAlmostEqual(self.commands[-1].angular.z, 0.06)

        self.node.camera_raw_16_9_callback(CompressedImage())
        self.finish_frame()
        self.assertEqual(self.node.controller.last_result, PathResult(()))
        self.assertEqual(self.commands[-1], Twist())

    def test_switch_discards_inflight_result_and_queued_old_frame(self):
        started, release = Event(), Event()

        class SlowDetector(RawDetector):
            def detect(self, msg):
                started.set()
                release.wait(2.0)
                return super().detect(msg)

        self.register_alternates(detector=SlowDetector)
        self.node.lane_detection_selection_callback(String(data='basic_sobel'))
        self.node.camera_raw_callback(CompressedImage(data=b'\x64'))
        self.assertTrue(started.wait(1.0))
        self.node.camera_raw_callback(CompressedImage(data=b'\x65'))
        try:
            self.node.lane_detection_selection_callback(String(data='basic_sat'))
        finally:
            release.set()
        self.finish_frame()
        self.assertTrue(all(msg == Twist() for msg in self.commands))
        self.node.camera_callback(straight_lane())
        self.finish_frame()
        self.assertAlmostEqual(self.commands[-1].linear.x, 0.22)

    def test_latest_frame_only_while_worker_is_busy(self):
        started, release = Event(), Event()

        class SlowDetector(RawDetector):
            def detect(self, msg):
                started.set()
                release.wait(2.0)
                return super().detect(msg)

        self.register_alternates(detector=SlowDetector)
        self.node.lane_detection_selection_callback(String(data='basic_sobel'))
        self.node.camera_raw_callback(CompressedImage(data=b'\x01'))
        self.assertTrue(started.wait(1.0))
        try:
            for value in range(2, 10):
                self.node.camera_raw_callback(CompressedImage(data=bytes([value])))
        finally:
            release.set()
        self.finish_frame()
        self.assertEqual(self.node.detector.seen, [1, 9])

    def test_timeout_stops_even_if_worker_has_not_finished(self):
        started, release = Event(), Event()
        self.register_alternates()
        self.node.lane_detection_selection_callback(String(data='basic_sobel'))
        self.node.camera_raw_callback(CompressedImage(data=b'\x3c'))
        self.finish_frame()
        self.assertGreater(self.commands[-1].linear.x, 0)

        def slow_detect(msg):
            started.set()
            release.wait(2.0)
            return 60

        self.node.detector.detect = slow_detect
        self.node.camera_raw_callback(CompressedImage(data=b'\x3c'))
        self.assertTrue(started.wait(1.0))
        future_time = self.node._job.received_at + self.node.FRAME_TIMEOUT_SEC + 0.01
        try:
            with patch('driving.driving.monotonic', return_value=future_time):
                self.node._poll_pipeline()
                self.assertEqual(self.commands[-1], Twist())
                release.set()
                self.node._job.future.result(timeout=1.0)
                self.node._poll_pipeline()
                self.assertEqual(self.commands[-1], Twist())
        finally:
            release.set()

    def test_nonfinite_controller_output_stops(self):
        class InvalidController(AlternateController):
            def compute(self, center):
                twist = Twist()
                twist.angular.z = float('nan')
                return twist

        self.register_alternates(controller=InvalidController)
        self.node.steering_selection_callback(String(data='pure_pursuit'))
        self.node.camera_callback(straight_lane())
        self.finish_frame()
        self.assertEqual(self.commands[-1], Twist())

    def test_real_dds_callbacks_switch_and_watchdog(self):
        self.publisher_patch.stop()
        self.register_alternates()
        probe = Node('probe')
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        executor.add_node(probe)
        camera = probe.create_publisher(CompressedImage, 'camera', 10)
        raw = probe.create_publisher(CompressedImage, 'camera_raw', 10)
        detection = probe.create_publisher(String, 'driving/detection', 10)
        steering = probe.create_publisher(String, 'driving/steering', 10)
        probe.create_subscription(Twist, 'cmd_vel', self.commands.append, 10)

        def spin_until(predicate, timeout=3.0):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            self.assertTrue(predicate(), 'DDS condition timed out')

        try:
            spin_until(lambda: all(pub.get_subscription_count() for pub in (camera, raw, detection, steering)))
            camera.publish(straight_lane())
            spin_until(lambda: any(msg.linear.x == 0.22 for msg in self.commands))
            detection.publish(String(data='basic_sobel'))
            steering.publish(String(data='pure_pursuit'))
            spin_until(lambda: self.node.lane_detection_algorithm == 'basic_sobel' and self.node.steering_algorithm == 'pure_pursuit')
            raw.publish(CompressedImage(data=b'\x64'))
            spin_until(lambda: any(msg.linear.x == 0.1 for msg in self.commands))
            self.commands.clear()
            spin_until(lambda: any(msg == Twist() for msg in self.commands), timeout=1.5)
        finally:
            executor.remove_node(probe)
            executor.remove_node(self.node)
            executor.shutdown()
            probe.destroy_node()
            self.publisher_patch.start()


if __name__ == '__main__':
    unittest.main()
