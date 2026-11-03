"""GUI만 대체하고 실제 DDS 수신과 JPEG 디코딩을 검사한다."""

import time
from unittest.mock import patch

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from driving.debug_viewer import DebugViewerNode, main


@pytest.fixture
def viewer():
    rclpy.init(args=[
        '--ros-args', '-r', '__ns:=/driving_debug_test',
        '-r', 'camera:=/driving_debug_test/input',
    ])
    node = DebugViewerNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def image_message(value, height=24, width=40):
    image = np.full((height, width, 3), value, np.uint8)
    ok, data = cv2.imencode('.jpg', image)
    assert ok
    return CompressedImage(format='jpeg', data=data.tobytes())


def test_all_four_topics_reach_their_windows_over_dds(viewer):
    probe = Node('viewer_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(viewer)
    executor.add_node(probe)
    shown = {}
    publishers = [
        probe.create_publisher(CompressedImage, sub.topic_name, qos_profile_sensor_data)
        for sub in viewer.image_subscriptions
    ]

    def spin_until(predicate):
        deadline = time.monotonic() + 3.0
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), 'DDS condition timed out'

    try:
        assert viewer.image_subscriptions[0].topic_name == '/driving_debug_test/input'
        with patch('driving.debug_viewer.cv2.imshow', side_effect=lambda name, frame: shown.update({name: frame.copy()})):
            spin_until(lambda: all(pub.get_subscription_count() for pub in publishers))
            for index, pub in enumerate(publishers):
                pub.publish(image_message(40 * index, height=24 + index))
            spin_until(lambda: len(shown) == 4)
        for index, topic in enumerate(viewer.IMAGE_TOPICS):
            assert shown[topic].shape == (24 + index, 40, 3)
            np.testing.assert_allclose(shown[topic], 40 * index, atol=2)
    finally:
        executor.shutdown()
        probe.destroy_node()


def test_bad_images_are_skipped_and_next_valid_image_is_shown(viewer):
    with patch('driving.debug_viewer.cv2.imshow') as show:
        viewer.image_callback('camera', CompressedImage())
        viewer.image_callback('camera', CompressedImage(data=b'broken'))
        show.assert_not_called()
        viewer.image_callback('camera', image_message(120))
        assert show.call_count == 1
        assert show.call_args[0][0] == 'camera'
        np.testing.assert_allclose(show.call_args[0][1], 120, atol=2)


@pytest.mark.parametrize('last_event, exit_code', [
    (ord('q'), 0), (27, 0), (KeyboardInterrupt(), 0), (cv2.error('GUI unavailable'), 1),
])
def test_idle_viewer_handles_exit_and_cleans_up(last_event, exit_code):
    # 영상이 없어도 이벤트 처리가 계속되어 키 종료와 GUI 오류 정리가 가능하다.
    with patch('driving.debug_viewer.cv2.waitKey', side_effect=[-1, last_event]) as wait_key, \
            patch('driving.debug_viewer.cv2.destroyAllWindows') as close:
        if isinstance(last_event, cv2.error):
            close.side_effect = cv2.error('HighGUI unavailable during cleanup')
        assert main(args=['--ros-args', '-r', '__ns:=/driving_debug_test']) == exit_code
        assert wait_key.call_count == 2
        close.assert_called_once_with()
    assert not rclpy.ok()
