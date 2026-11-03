"""선택 이름과 구현 연결. 새 알고리즘은 Literal과 해당 등록표에 추가한다."""

from typing import Any, Callable, Dict, Literal, Tuple, get_args

from driving.detection.basic_sat import Basic_Sat
from driving.interfaces import CameraTopic, LaneDetector, SteeringController
from driving.steering.basic_p import Basic_P


Steering = Literal['basic_p', 'pure_pursuit']
Detection = Literal['basic_sobel', 'basic_sat', 'yolo']

DEFAULT_STEERING: Steering = 'basic_p'
DEFAULT_DETECTION: Detection = 'basic_sat'

# 인자 없는 생성 함수를 등록한다. 필요한 설정은 각 생성 함수 내부에서 처리한다.
# 아직 구현하지 않은 이름은 등록하지 않는다.
# 등록표에는 여러 결과 타입이 섞이므로 Any를 쓰고, 선택 시 규약 이름을 확인한다.
DETECTORS: Dict[Detection, Callable[[], LaneDetector[Any]]] = {
    'basic_sat': Basic_Sat.from_settings,
}
CONTROLLERS: Dict[Steering, Callable[[], SteeringController[Any]]] = {
    'basic_p': Basic_P.from_settings,
}


def create_algorithms(
    detection: Detection, steering: Steering,
) -> Tuple[LaneDetector[Any], SteeringController[Any]]:
    if detection not in get_args(Detection) or detection not in DETECTORS:
        raise ValueError('Detection is not implemented: ' + detection)
    if steering not in get_args(Steering) or steering not in CONTROLLERS:
        raise ValueError('Steering is not implemented: ' + steering)

    detector = DETECTORS[detection]()
    controller = CONTROLLERS[steering]()
    if detector.input_topic not in get_args(CameraTopic):
        raise ValueError('Unsupported camera topic: ' + detector.input_topic)
    if not callable(detector.detect) or not callable(controller.compute):
        raise ValueError('Algorithms must provide detect() and compute().')
    if detector.output_type != controller.input_type:
        raise ValueError(
            'Incompatible detection/steering: '
            + detector.output_type + ' -> ' + controller.input_type
        )
    return detector, controller
