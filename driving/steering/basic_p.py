from geometry_msgs.msg import Twist
from dataclasses import dataclass
import numpy as np


@dataclass
class Config:
    roi_start_x: int
    roi_end_x: int
    target_x: int
    error_scale: float
    normal_angular_divisor: float
    intersection_angular_divisor: float
    debug_color: tuple  # (0, 0, 255): red



class Basic_P:
    LEFT_LANE_CONFIG = Config(
            roi_start_x=0,
            roi_end_x=320,
            target_x=60,
            error_scale=60.0,
            normal_angular_divisor=1.7,
            intersection_angular_divisor=2.5,
            debug_color=(0, 0, 255),
        )
    RIGHT_LANE_CONFIG = Config(
            roi_start_x=320,
            roi_end_x=640,
            target_x=260,  # 전체 좌표 580(목표위치) - ROI 시작 좌표 320
            error_scale=77.0,
            normal_angular_divisor=1.3,
            intersection_angular_divisor=3.2,
            debug_color=(255, 0, 0),
        )

    def __init__(self):
        # self.lane_mode=True # True: Left
        pass

    # def changeLaneMode(self):
    #     self.lane_mode = not self.lane_mode

    def makeTwist(self, lane_center, lane_mode=True):
        config = self.LEFT_LANE_CONFIG
        if lane_mode == False:
            config = self.RIGHT_LANE_CONFIG

        # if lane_center == None: lane_center = config.roi_start_x

        normalized_error = (config.target_x - lane_center) / config.error_scale
        normalized_error = np.clip(normalized_error, -1.65, 1.5)

        twist = Twist()
        # if self.is_intersection:
        #     base_speed = 0.15
        #     linear_divisor = 1.3
        #     angular_divisor = config.intersection_angular_divisor
        # else:
        base_speed = 0.22
        linear_divisor = 1.5
        angular_divisor = config.normal_angular_divisor


        twist.linear.x = base_speed / (1 + abs(normalized_error / linear_divisor))
        twist.angular.z = normalized_error / angular_divisor

        return twist

