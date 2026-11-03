"""basic_sat/basic_p의 한쪽 차선 추종에만 사용하는 설정과 결과 규약."""

from dataclasses import dataclass
from typing import Optional


# 선택한 좌/우 ROI 내부 x좌표(폭 320px). None은 basic 검출기의 미검출 표현.
LaneCenter = Optional[float]
ROI_CENTER_320 = 'roi_center_x_320_v1'


@dataclass(frozen=True)
class BasicSettings:
    lane_mode: bool = True  # True: 왼쪽, False: 오른쪽


# 두 basic 생성 함수가 함께 참조한다. 새 객체 생성 시 적용하는 설정이다.
# 공통 노드/다른 알고리즘은 이 설정을 알 필요가 없다.
SETTINGS = BasicSettings()
