from geometry_msgs.msg import Twist
from dataclasses import dataclass
from math import isfinite
from typing import Optional
import numpy as np

from driving import basic
from driving.basic import LaneCenter

@dataclass
class Config:
    roi_start_x: int
    roi_end_x: int
    target_x: int
    error_scale: float
    normal_angular_divisor: float
    intersection_angular_divisor: float
    debug_window: str
    debug_color: tuple  # (0, 0, 255): red


class Basic_P:
    LEFT_LANE_CONFIG = Config(
            roi_start_x=0,
            roi_end_x=320,
            target_x=60,
            error_scale=60.0,
            normal_angular_divisor=1.7,
            intersection_angular_divisor=2.5,
            debug_window='left_lane',
            debug_color=(0, 0, 255),
        )
    RIGHT_LANE_CONFIG = Config(
            roi_start_x=320,
            roi_end_x=640,
            target_x=260,  # 전체 좌표 580(목표위치) - ROI 시작 좌표 320
            error_scale=77.0,
            normal_angular_divisor=1.3,
            intersection_angular_divisor=3.2,
            debug_window='right_lane',
            debug_color=(255, 0, 0),
        )

    def __init__(
        self, lane_mode: bool = True,
        left_divisor: Optional[float] = None, right_divisor: Optional[float] = None,
    ):
        self.lane_mode = lane_mode  # True: Left
        self.left_divisor = (
            self.LEFT_LANE_CONFIG.normal_angular_divisor if left_divisor is None else left_divisor
        )
        self.right_divisor = (
            self.RIGHT_LANE_CONFIG.normal_angular_divisor if right_divisor is None else right_divisor
        )

    @classmethod
    def from_settings(cls) -> 'Basic_P':
        """검출기와 같은 basic 설정으로 조향기를 생성한다."""
        return cls(lane_mode=basic.SETTINGS.lane_mode)

    def changeLaneMode(self):
        self.lane_mode = not self.lane_mode

    def compute(self, lane_center: LaneCenter) -> Twist:
        """공통 진입점. 기존 makeTwist도 직접 호출할 수 있다."""
        return self.makeTwist(lane_center)

    def makeTwist(self, lane_center: LaneCenter) -> Twist:
        config = self.LEFT_LANE_CONFIG
        if not self.lane_mode:
            config = self.RIGHT_LANE_CONFIG

        if lane_center is None:
            # 정상 영상에서 미검출이면 기존 극단값 조향 정책을 유지한다.
            lane_center = config.roi_start_x
        elif not isfinite(lane_center) or not 0 <= lane_center < 320:
            raise ValueError('Lane center must be a finite ROI x coordinate in [0, 320).')

        normalized_error = (config.target_x - lane_center) / config.error_scale
        normalized_error = float(np.clip(normalized_error, -1.65, 1.5))

        twist = Twist()
        # if self.is_intersection:
        #     base_speed = 0.15
        #     linear_divisor = 1.3
        #     angular_divisor = config.intersection_angular_divisor
        # else:
        base_speed = 0.22
        linear_divisor = 1.5
        angular_divisor = (
            self.left_divisor if self.lane_mode else self.right_divisor
        )


        twist.linear.x = base_speed / (1 + abs(normalized_error / linear_divisor))
        twist.angular.z = normalized_error / angular_divisor

        return twist

