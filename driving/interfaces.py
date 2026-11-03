"""팀 알고리즘의 공통 호출 규약 (Python 3.8 / Galactic).

등록표의 생성 함수는 인자 없이 호출하며 빠르게 반환해야 한다.
detect/compute는 유한 시간에 반환하고, 결과 형식과 미검출 표현은 각 조합이 정한다.
노드는 결과를 해석하지 않고 조향기로 전달한다. 여러 영상의 동기화는 별도 확장이다.
"""

from typing import Literal, Protocol, TypeVar

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage


CameraTopic = Literal['camera', 'camera_raw', 'camera_raw_16_9']
DetectionResult = TypeVar('DetectionResult', covariant=True)
SteeringInput = TypeVar('SteeringInput', contravariant=True)


class LaneDetector(Protocol[DetectionResult]):
    input_topic: CameraTopic
    output_type: str

    def detect(self, camera_msg: CompressedImage) -> DetectionResult:
        """디코딩/전처리/검출을 담당한다. 잘못된 입력은 예외로 알린다."""
        ...


class SteeringController(Protocol[SteeringInput]):
    input_type: str

    def compute(self, result: SteeringInput) -> Twist:
        ...
