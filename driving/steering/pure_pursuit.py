# Adapted from turtlebot-autorace-2026/lane_detection_dl, commit f1dcce7.
'''
Pure pursuit for a differential drive robot: path in, (linear, angular) velocity out.

Give it the path you want to drive and it returns the command for /cmd_vel. Nothing
here depends on lanes or cameras.

Path: (N, 2) points in metres, ordered from start to end, in the robot frame
(x forward, y left, origin at the wheel axle) - or in any world frame (e.g. odom)
together with the robot pose (x, y, yaw) in that frame.

  from driving.steering.pure_pursuit import PurePursuit

  pp = PurePursuit(lookahead=0.25, v_max=0.15)
  v, w = pp.compute(path)                              # robot frame path
  v, w = pp.compute(path_odom, pose=(x, y, yaw))       # odom frame path + robot pose
  v, w = pp.compute(path, lookahead=0.2, v_max=0.1)    # per-call values override the defaults
  cmd_vel_publisher.publish(pp.to_twist(v, w))
  if pp.reached: ...                                   # end of the path reached, command is (0, 0)

  v, w = pure_pursuit(path, lookahead=0.25, v_max=0.15)   # one-shot, no state

Target point: the point on the path at distance `lookahead` from the robot, searched
forward from the path point nearest to the robot (end of the path if it is shorter).
Steering: curvature of the arc through the target, kappa = 2 y / d^2, scaled by k_lat.
Speed: v = v_max / (1 + k_v |kappa|), so the robot slows down in curves.
'''
import math

import numpy as np


class PurePursuit:
    def __init__(self, lookahead=0.25, v_max=0.15, k_lat=1.0, kappa_max=7.0, k_v=0.6, max_ang=1.2,
                 ang_smooth=1.0, goal_tolerance=0.03):
        self.lookahead = lookahead            # m, distance to the target point
        self.v_max = v_max                    # m/s
        self.k_lat = k_lat                    # gain on the curvature (1 = plain pure pursuit)
        self.kappa_max = kappa_max            # 1/m, sharpest allowed turn
        self.k_v = k_v                        # slow-down in curves, 0 = constant speed
        self.max_ang = max_ang                # rad/s
        self.ang_smooth = ang_smooth          # weight of the new angular command (low-pass), 1 = none
        self.goal_tolerance = goal_tolerance  # m, stop when the path end is this close (0 = never)
        self.reset()

    def reset(self):
        '''Forget the smoothing state and the last result (call after a stop or a new path).'''
        self.prev_ang = 0.0
        self.target = None    # (x, y) of the last target point, robot frame
        self.kappa = 0.0
        self.v = self.w = 0.0
        self.reached = False

    # ---- main entry points ----
    def compute(self, path, lookahead=None, v_max=None, pose=None, kappa_scale=1.0, v_scale=1.0):
        '''
        path: (N, 2) points, start -> end [m]; robot frame, or world frame with `pose`
        lookahead, v_max: override the defaults for this call
        pose: (x, y, yaw) of the robot in the path's frame; None = path is in the robot frame
        kappa_scale, v_scale: extra factors on the curvature / speed for this call
        returns (linear_x [m/s], angular_z [rad/s])
        '''
        path = np.asarray(path, float).reshape(-1, 2)
        if pose is not None:
            path = to_robot_frame(path, pose)
        end = path[-1]
        if self.goal_tolerance > 0 and math.hypot(end[0], end[1]) < self.goal_tolerance:
            self.stop()
            self.reached = True
            return 0.0, 0.0
        self.reached = False
        x, y = find_target(path, self.lookahead if lookahead is None else lookahead)
        return self.compute_to(x, y, v_max=v_max, kappa_scale=kappa_scale, v_scale=v_scale)

    def compute_to(self, x, y, v_max=None, kappa_scale=1.0, v_scale=1.0):
        '''Command towards one target point (x forward, y left) [m] in the robot frame.'''
        kappa = self.k_lat * 2 * y / (x ** 2 + y ** 2) * kappa_scale
        kappa = float(np.clip(kappa, -self.kappa_max, self.kappa_max))
        v = (self.v_max if v_max is None else v_max) / (1 + self.k_v * abs(kappa)) * v_scale
        ang = float(np.clip(v * kappa, -self.max_ang, self.max_ang))
        ang = self.ang_smooth * ang + (1 - self.ang_smooth) * self.prev_ang
        self.prev_ang = ang
        self.target, self.kappa, self.v, self.w = (float(x), float(y)), kappa, float(v), ang
        return float(v), ang

    def stop(self):
        '''Zero command; also clears the smoothing state.'''
        self.prev_ang = 0.0
        self.v = self.w = 0.0
        return 0.0, 0.0

    @staticmethod
    def to_twist(v, w):
        '''(v, w) -> geometry_msgs/Twist for /cmd_vel.'''
        from geometry_msgs.msg import Twist
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        return t


def pure_pursuit(path, lookahead, v_max, pose=None, **params):
    '''One-shot pure pursuit without state (no smoothing). Returns (linear_x, angular_z).'''
    return PurePursuit(lookahead=lookahead, v_max=v_max, **params).compute(path, pose=pose)


def to_robot_frame(path, pose):
    '''World frame points -> robot frame, robot at pose (x, y, yaw).'''
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    d = np.asarray(path, float).reshape(-1, 2) - (px, py)
    return np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]]


def find_target(path, lookahead):
    '''
    Target point in the robot frame: from the path point nearest to the robot, the first
    point at distance `lookahead` along the path (interpolated on the segment that crosses
    the circle); the path end if the rest of the path stays inside the circle.
    '''
    path = np.asarray(path, float).reshape(-1, 2)
    dist = np.hypot(path[:, 0], path[:, 1])
    i0 = int(np.argmin(dist))
    for i in range(i0, len(path) - 1):
        if dist[i + 1] >= lookahead > dist[i] or (i == i0 and dist[i] >= lookahead):
            if dist[i] >= lookahead:       # nearest point already outside the circle
                return tuple(path[i])
            a, b = path[i], path[i + 1]    # solve |a + t (b - a)| = lookahead, t in [0, 1]
            d = b - a
            A, B, C = d @ d, 2 * a @ d, a @ a - lookahead ** 2
            t = (-B + math.sqrt(max(B * B - 4 * A * C, 0.0))) / (2 * A) if A > 0 else 0.0
            return tuple(a + min(max(t, 0.0), 1.0) * d)
    return tuple(path[-1])


class LanePurePursuit:
    """원본 follow_lane의 일반/교차로 속도 정책 + PurePursuit → Twist.

    검출기의 target은 차축 기준 (전방 x, 좌측 y)[m]이다.
    mode는 both/yellow_only/white_only/lost이며 basic의 좌우 모드와 별개다.
    """

    def __init__(self):
        self.pursuit = PurePursuit(v_max=0.18, ang_smooth=0.6)
        self.ang_scale_intersection = 0.7  # 원본 교차로 곡률 배율(ROS 파라미터 아님)

    def reset(self):
        self.pursuit.reset()

    def makeTwist(self, target, mode, v_max=0.18, v_lost=0.05, k_lat=1.0,
                  max_ang=1.2, k_v=0.6, ang_smooth=0.6, v_single=0.8,
                  intersection=False, v_max_intersection=0.12):
        values = (v_max, v_lost, k_lat, max_ang, k_v, ang_smooth, v_single,
                  v_max_intersection)
        if (not np.isfinite(values).all() or min(values) < 0
                or ang_smooth > 1 or v_single > 1):
            raise ValueError('Invalid lane pursuit parameters.')
        if mode not in ('both', 'yellow_only', 'white_only', 'lost'):
            raise ValueError('Unknown lane detection mode: ' + str(mode))
        pursuit = self.pursuit
        pursuit.v_max, pursuit.k_lat = v_max, k_lat
        pursuit.max_ang, pursuit.k_v = max_ang, k_v
        pursuit.ang_smooth = ang_smooth
        if target is None:
            # 원본 미검출 정책: 평활화 상태 초기화 후 느린 직진.
            pursuit.reset()
            pursuit.v, pursuit.w = float(v_lost), 0.0
            return pursuit.to_twist(pursuit.v, pursuit.w)
        if not np.isfinite(target).all() or target[0] <= 0:
            raise ValueError('Lane target must be finite and ahead of the robot.')
        pursuit.lookahead = float(target[0])
        v, w = pursuit.compute_to(
            *target, v_max=v_max_intersection if intersection else None,
            kappa_scale=self.ang_scale_intersection if intersection else 1.0,
            v_scale=1.0 if mode == 'both' else v_single,
        )
        return pursuit.to_twist(v, w)
