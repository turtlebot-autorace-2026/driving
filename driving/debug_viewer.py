"""카메라 입력과 driving의 디버깅 영상을 OpenCV 창에 표시하는 로컬 뷰어."""

from functools import partial

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class DebugViewerNode(Node):
    # 상대 토픽 이름이므로 driving과 같은 namespace 또는 remap을 사용할 수 있다.
    IMAGE_TOPICS = (
        'camera',
        'camera_raw',
        'camera_raw_16_9',
        'driving/debug/roi/compressed',
    )

    def __init__(self):
        super().__init__('driving_debug')
        # partial은 공통 콜백에 창 이름을 미리 넣는다. ROS는 msg만 전달하면 된다.
        self.image_subscriptions = [
            self.create_subscription(
                CompressedImage, topic, partial(self.image_callback, topic),
                qos_profile_sensor_data,
            )
            for topic in self.IMAGE_TOPICS
        ]
        self.get_logger().info('Waiting for images. Press q/Esc in an image window or Ctrl+C to exit.')

    def image_callback(self, window_name: str, msg: CompressedImage):
        # 세 camera 토픽도 CompressedImage다. JPEG 등을 BGR 영상으로 복원한다.
        try:
            if not msg.data:
                raise ValueError('Empty image data.')
            image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('Could not decode the image.')
        except (ValueError, cv2.error) as error:
            # 손상된 한 장은 건너뛰고 다음 정상 영상을 계속 받는다.
            self.get_logger().warning(f'{window_name}: {error}')
            return
        # 첫 정상 영상에서 창이 생긴다. 모든 창은 main의 ROS 실행 스레드에서 다룬다.
        cv2.imshow(window_name, image)


def main(args=None):
    rclpy.init(args=args)
    node = DebugViewerNode()
    try:
        while rclpy.ok():
            # 영상이 잠시 없어도 waitKey를 호출하여 창 갱신과 키 입력을 처리한다.
            rclpy.spin_once(node, timeout_sec=0.03)
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                break
    except KeyboardInterrupt:
        pass
    except cv2.error as error:
        node.get_logger().error(f'OpenCV display failed. Check the desktop/HighGUI environment: {error}')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass  # HighGUI 없는 OpenCV에서는 창 정리도 지원하지 않을 수 있다.
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
