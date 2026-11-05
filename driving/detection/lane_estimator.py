# Adapted from turtlebot-autorace-2026/lane_detection_dl, commit f1dcce7.
'''
Deep learning (YOLO seg) based lane following for TurtleBot3 AutoRace.

All geometry is in the BEV image (640x480) made from the undistorted /camera image
with the BEV parameters below. BEV is metric-isotropic: 160 px = 1 camera height.
Left lane = yellow, right lane = white (fixed).

Pipeline (one frame):
  LanePath.update()   1. keep only the lane masks inside the ROI (near the robot, own road)
                      2. fit one curve x(row) per lane color
                      3. path center: both lanes -> middle, one lane -> offset from it, none -> lost
                      4. center -> path in the robot frame [m]
  LanePath.target()   5. target point at the lookahead row -> steering/LanePurePursuit
'''
import cv2
import numpy as np


# ---- BEV (from bev_params.yaml) ----
BEV_W, BEV_H = 640, 480
BEV_SRC = np.float32([[18, 451], [633, 477], [445, 157], [216, 153]])
BEV_DST = np.float32([[166, 474], [474, 474], [474, 122], [166, 122]])
BEV_M = cv2.getPerspectiveTransform(BEV_SRC, BEV_DST)
BEV_PX_PER_CAMERA_HEIGHT = 160.0   # BEV scale
BEV_BOTTOM_AHEAD = 0.76            # ground distance of the BEV bottom row from the camera [camera heights]

# model class ids
WHITE, YELLOW = 0, 1


def to_bev(undistorted):
    if undistorted.shape[:2] != (480, 640):
        undistorted = cv2.resize(undistorted, (640, 480))
    return cv2.warpPerspective(undistorted, BEV_M, (BEV_W, BEV_H))


class LaneConfig:
    def __init__(self, **kw):
        # geometry
        self.robot_x = 326.0          # BEV x of the robot center line
        self.lane_width = 355.0       # yellow-center to white-center distance [px] (286 mm road)
        self.roi_top_row = 220        # ROI: rows below this only (~31 cm ahead of the camera)
        self.roi_half_width = 250     # ROI: +-px around the robot center line (own lane lines at ~+-160 px)
        self.row_step = 10            # rows sampled for the lane curves
        self.min_points = 5           # sampled rows needed to accept a lane
        self.single_offset = 0.5      # distance from the line when only one lane is seen [x lane width]
        self.lookahead_row = 400      # target point row (~16 cm ahead of the camera)
        self.camera_height = 0.129    # m, sets the BEV metric scale
        self.camera_ahead_of_axle = 0.076   # m
        for k, v in kw.items():
            setattr(self, k, v)
        self.rows = np.arange(BEV_H - 10, self.roi_top_row - 1, -self.row_step)   # bottom -> top


def _lane_curve(mask, rows, min_points):
    '''x(row) of one lane: per-row center of the mask, fitted with a polynomial. None if too few points.'''
    pts = []
    for r in rows:
        cols = np.nonzero(mask[max(r - 2, 0):r + 3].any(axis=0))[0]
        if len(cols):
            pts.append((r, np.median(cols)))
    if len(pts) < min_points:
        return None
    r, x = np.array(pts).T
    coef = np.polyfit(r, x, 2 if len(pts) >= 8 else 1)
    return np.polyval(coef, rows)


def _offset_curve(xs, rows, d):
    '''Shift a lane curve by d px along its normal (+: to the right when driving up the image).'''
    dxdr = np.gradient(xs, rows)
    n = np.sqrt(1 + dxdr ** 2)
    nx, nr = xs + d / n, rows - d * dxdr / n
    order = np.argsort(nr)
    return np.interp(rows, nr[order], nx[order])


class LanePath:
    '''Lane detections in the BEV image -> path to follow in the robot frame.'''

    def __init__(self, cfg=None):
        self.cfg = cfg or LaneConfig()
        self.m_per_px = self.cfg.camera_height / BEV_PX_PER_CAMERA_HEIGHT
        self.reset()

    def reset(self):
        self.yellow = self.white = self.center = None   # x(row) curves in BEV px
        self.mode = 'lost'            # both | yellow_only | white_only | lost
        self.roi_side = None          # None | 'left' | 'right' (turn sign: half ROI)
        self.roi_height_scale = 1.0   # turn sign: ROI height x this (extended upward)
        self.rows = self.cfg.rows     # rows sampled in the last update (bottom -> top)
        self.prev_x_la = None         # lookahead point of the previous frame (lane continuity)

    # ---- BEV px <-> robot frame (x forward from the wheel axle, y left) [m] ----
    def forward(self, row):
        return (BEV_BOTTOM_AHEAD + (BEV_H - row) / BEV_PX_PER_CAMERA_HEIGHT) * self.cfg.camera_height \
            + self.cfg.camera_ahead_of_axle

    def to_robot(self, xs, rows):
        return np.c_[self.forward(np.asarray(rows, float)), -(np.asarray(xs, float) - self.cfg.robot_x) * self.m_per_px]

    def to_bev(self, x, y):
        row = BEV_H - ((x - self.cfg.camera_ahead_of_axle) / self.cfg.camera_height - BEV_BOTTOM_AHEAD) \
            * BEV_PX_PER_CAMERA_HEIGHT
        return self.cfg.robot_x - y / self.m_per_px, row

    def target(self, path):
        '''Lane target point: the path at the lookahead row (fixed distance straight ahead) [m].'''
        x = self.forward(self.cfg.lookahead_row)
        order = np.argsort(path[:, 0])
        return x, float(np.interp(x, path[order, 0], path[order, 1]))

    def roi_x_range(self):
        '''ROI columns [x0, x1). roi_side 'left': image left edge ~ robot center line,
        'right': robot center line ~ image right edge, None: robot_x +- roi_half_width.'''
        cfg = self.cfg
        if self.roi_side == 'left':
            return 0, int(cfg.robot_x)
        if self.roi_side == 'right':
            return int(cfg.robot_x), BEV_W
        x0 = int(max(0, cfg.robot_x - cfg.roi_half_width))
        x1 = int(min(BEV_W, cfg.robot_x + cfg.roi_half_width))
        return x0, x1

    def roi_top(self):
        '''ROI top row. With a turn sign the ROI height is scaled by roi_height_scale, extended upward.'''
        top = self.cfg.roi_top_row
        if self.roi_side is not None:
            top = BEV_H - (BEV_H - top) * self.roi_height_scale
        return int(min(max(top, 0), BEV_H - 1))

    def roi_mask(self):
        roi = np.zeros((BEV_H, BEV_W), bool)
        x0, x1 = self.roi_x_range()
        roi[self.roi_top():, x0:x1] = True
        return roi

    def update(self, detections, roi_side=None, roi_height_scale=1.0):
        '''
        detections: list of (cls, mask HxW bool in BEV coords, conf)
        roi_side: None (normal ROI), 'left' / 'right' (turn sign: center line to that image edge)
        roi_height_scale: with a turn sign, ROI height x this (top row moves up)
        returns the path as an (N, 2) array of (x forward, y left) [m], or None when no lane is seen
        '''
        cfg = self.cfg
        self.roi_side = roi_side
        self.roi_height_scale = roi_height_scale
        # lane curves are sampled over the ROI rows, so a taller sign ROI also samples higher rows
        rows = np.arange(BEV_H - 10, self.roi_top() - 1, -cfg.row_step) if roi_side else cfg.rows
        self.rows = rows

        # 1. lane masks inside the ROI, one union mask per color
        roi = self.roi_mask()
        masks = {YELLOW: np.zeros_like(roi), WHITE: np.zeros_like(roi)}
        for cls, mask, _ in detections:
            if cls in masks:
                masks[cls] |= mask & roi

        # 2. one curve per lane
        y = _lane_curve(masks[YELLOW], rows, cfg.min_points)
        w = _lane_curve(masks[WHITE], rows, cfg.min_points)

        # both seen but not a normal lane (order swapped / too narrow / fork): keep the one lane whose
        # path stays closest to the previous target point, i.e. keep going the way the robot was going
        single = cfg.lane_width * cfg.single_offset
        if y is not None and w is not None:
            gap = np.median(w - y)
            if not (0.6 * cfg.lane_width < gap < 1.4 * cfg.lane_width):
                ref = cfg.robot_x if self.prev_x_la is None else self.prev_x_la
                x_y = np.interp(cfg.lookahead_row, rows[::-1], _offset_curve(y, rows, +single)[::-1])
                x_w = np.interp(cfg.lookahead_row, rows[::-1], _offset_curve(w, rows, -single)[::-1])
                y, w = (y, None) if abs(x_y - ref) <= abs(x_w - ref) else (None, w)
        self.yellow, self.white = y, w

        # 3. path center
        if y is not None and w is not None:
            center, self.mode = (y + w) / 2, 'both'
        elif y is not None:
            center, self.mode = _offset_curve(y, rows, +single), 'yellow_only'
        elif w is not None:
            center, self.mode = _offset_curve(w, rows, -single), 'white_only'
        else:
            self.center, self.mode, self.prev_x_la = None, 'lost', None
            return None
        self.center = center
        self.prev_x_la = float(np.interp(cfg.lookahead_row, rows[::-1], center[::-1]))

        # 4. robot frame
        return self.to_robot(center, rows)

    def draw(self, bev, detections=None, pursuit=None):
        '''BEV debug view; pass the PurePursuit to show its target point and command.'''
        cfg, rows = self.cfg, self.rows
        img = bev.copy()
        if detections:
            ov = img.copy()
            for cls, mask, _ in detections:
                ov[mask] = (0, 255, 255) if cls == YELLOW else (255, 0, 255)
            img = cv2.addWeighted(ov, 0.4, img, 0.6, 0)
        for xs, col in ((self.yellow, (0, 200, 255)), (self.white, (255, 255, 255)), (self.center, (0, 255, 0))):
            if xs is not None:
                cv2.polylines(img, [np.int32(np.c_[xs, rows])], False, col, 2)
        if pursuit is not None and pursuit.target is not None and self.mode != 'lost':
            tx, tr = self.to_bev(*pursuit.target)
            cv2.circle(img, (int(round(tx)), int(round(tr))), 7, (0, 0, 255), -1)
        cv2.line(img, (int(cfg.robot_x), BEV_H), (int(cfg.robot_x), BEV_H - 60), (255, 0, 0), 2)
        x0, x1 = self.roi_x_range()
        cv2.rectangle(img, (x0, self.roi_top()), (min(x1, BEV_W - 1), BEV_H - 1), (0, 255, 255), 1)
        txt = self.mode + (f' [roi {self.roi_side}]' if self.roi_side else '') + (f' v={pursuit.v:.2f} w={pursuit.w:+.2f}' if pursuit is not None else '')
        cv2.putText(img, txt, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return img
