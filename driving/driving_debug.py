"""카메라/검출 디버그 JPEG를 토픽별 창으로 보여 주는 독립 노드."""
from functools import partial
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


# 현재 패키지에서 사용하는 영상 토픽. 새 영상 출력은 이 목록에 추가한다.
IMAGE_TOPICS = (
    'camera', 'camera_raw', 'camera_raw_16_9', 'camera2',
    'debug1', 'debug2',
)


class DrivingDebugNode(Node):
    def __init__(self):
        super().__init__('driving_debug')
        self.running = True
        self.window_names = set()
        self.image_subscriptions = []
        for topic in IMAGE_TOPICS:
            self.image_subscriptions.append(self.create_subscription(
                CompressedImage, topic,
                partial(self.image_callback, topic=topic),
                qos_profile_sensor_data,
            ))
        # 영상 수신이 멈춰도 창 이벤트와 종료 키를 계속 처리한다.
        # 기본 단일 executor에서 imshow와 waitKey가 같은 스레드로 실행된다.
        self.gui_timer = self.create_timer(1.0 / 30.0, self.handle_keys)
        self.get_logger().info('Viewing: ' + ', '.join(IMAGE_TOPICS) + ' (q / Esc / Ctrl+C to exit)')

    def image_callback(self, msg: CompressedImage, topic: str):
        try:
            if not msg.data:
                raise ValueError('Empty compressed image.')
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError('Could not decode the compressed image.')
        except (ValueError, cv2.error) as error:
            self.get_logger().warning(f'{topic}: {error}')
            return

        try:
            if topic not in self.window_names:
                cv2.namedWindow(topic, cv2.WINDOW_NORMAL)
                self.window_names.add(topic)
            cv2.imshow(topic, frame)
        except cv2.error as error:
            self.get_logger().error(f'Could not display {topic}: {error}')
            self.running = False

    def handle_keys(self):
        if not self.window_names:
            return
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self.running = False


def main(args=None):
    # 화면 없는 터미널에서 Qt가 강제 종료하기 전에 실행 조건을 알려 준다.
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise SystemExit('driving_debug requires a GUI display. Run it from a desktop terminal.')
    rclpy.init(args=args)
    node = DrivingDebugNode()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
