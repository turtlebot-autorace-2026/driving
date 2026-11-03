"""알고리즘 이름과 생성 함수. 새 구현은 아래 딕셔너리에 추가한다."""

from typing import Literal

from driving.detection.basic_sat import Basic_Sat
from driving.steering.basic_p import Basic_P


# Literal은 편집기의 타입 힌트용이다. 실제 선택 가능 여부는 등록표로 판단한다.
Steering = Literal['basic_p', 'pure_pursuit']
Detection = Literal['basic_sobel', 'basic_sat', 'yolo']
DEFAULT_STEERING: Steering = 'basic_p'
DEFAULT_DETECTION: Detection = 'basic_sat'

DETECTORS = {'basic_sat': Basic_Sat.from_settings}
CONTROLLERS = {'basic_p': Basic_P.from_settings}
CAMERA_TOPICS = ('camera', 'camera_raw', 'camera_raw_16_9')


def create_algorithms(detection: Detection, steering: Steering):
    if detection not in DETECTORS:
        raise ValueError('Detection is not implemented: ' + detection)
    if steering not in CONTROLLERS:
        raise ValueError('Steering is not implemented: ' + steering)
    detector = DETECTORS[detection]()
    controller = CONTROLLERS[steering]()
    if detector.input_topic not in CAMERA_TOPICS:
        raise ValueError('Unsupported camera topic: ' + detector.input_topic)
    return detector, controller
