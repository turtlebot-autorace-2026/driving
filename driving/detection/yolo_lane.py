"""/camera JPEG → 원본 BEV/YOLO → 미터 단위 목표점과 검출 상태."""
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

from driving.detection.lane_estimator import LaneConfig, LanePath, to_bev


class YoloLane:
    def __init__(self, model_path: str, device: str = ''):
        # basic만 사용하는 환경에서는 ultralytics를 import하거나 모델을 읽지 않는다.
        if not Path(model_path).is_file():
            raise ValueError('YOLO model not found: ' + model_path)
        from ultralytics import YOLO
        self.model = YOLO(model_path, task='segment')
        self.device = device or None
        self.conf = 0.4
        self.imgsz = 640
        self.lane = LanePath()
        self.geometry = None
        self.reset()

    def reset(self):
        """선택 전환 시 추적/표시 상태만 비운다. 모델은 재사용한다."""
        self.lane.reset()
        self.bev = None
        self.detections = []

    def detect(self, bev):
        """원본 detection.py의 YOLO 추론 및 마스크 추출 계산."""
        result = self.model.predict(bev, imgsz=self.imgsz, conf=self.conf, device=self.device,
                                    retina_masks=True, verbose=False)[0]
        if result.masks is None:
            return []
        masks = result.masks.data.cpu().numpy() > 0.5
        classes = result.boxes.cls.cpu().numpy().astype(int)
        confs = result.boxes.conf.cpu().numpy()
        return [(c, m, float(p)) for c, m, p in zip(classes, masks, confs)]

    def detectLane(
        self, msg, conf=0.4, imgsz=640, camera_height=0.129,
        roi_top_row=220, roi_half_width=250, lookahead_row=400,
        single_offset=0.5, lane_width=355.0, roi_side=None, roi_height_scale=1.0,
    ) -> Tuple[Optional[Tuple[float, float]], str]:
        """target=(전방 x, 좌측 y)[m] 또는 None, mode=both/단일 차선/lost.

        roi_side: None=기본 ROI, 'left'/'right'=로봇 중앙선부터 해당 방향 영상 끝까지(회전 표지판).
        roi_height_scale: 회전 표지판 ROI의 높이 배율(위쪽으로 늘림).
        """
        self.bev = None
        self.detections = []
        geometry = (camera_height, roi_top_row, roi_half_width,
                    lookahead_row, single_offset, lane_width)
        if not np.isfinite((conf, imgsz) + geometry).all():
            raise ValueError('YOLO parameters must be finite.')
        if not 0.0 <= conf <= 1.0 or imgsz <= 0:
            raise ValueError('conf must be 0..1 and imgsz must be positive.')
        if (camera_height <= 0 or lane_width <= 0 or roi_half_width <= 0
                or not 0 <= roi_top_row < 480 or not 0 <= lookahead_row < 480
                or single_offset < 0):
            raise ValueError('Invalid YOLO lane geometry parameters.')
        if msg is None or not msg.data:
            raise ValueError('Camera image is missing or empty.')
        src = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if src is None:
            raise ValueError('Could not decode the compressed camera image.')

        # 기하 설정을 바꾼 경우에만 추적을 새로 시작한다. 매 프레임 생성하지 않는다.
        if geometry != self.geometry:
            self.lane = LanePath(LaneConfig(
                camera_height=camera_height, roi_top_row=roi_top_row,
                roi_half_width=roi_half_width, lookahead_row=lookahead_row,
                single_offset=single_offset, lane_width=lane_width,
            ))
            self.geometry = geometry
        self.conf, self.imgsz = conf, imgsz
        self.bev = to_bev(src)
        self.detections = self.detect(self.bev)
        path = self.lane.update(self.detections, roi_side, roi_height_scale)
        target = None if path is None else self.lane.target(path)
        return target, self.lane.mode

    def debug(self, pursuit=None):
        if self.bev is None:
            return None
        return self.lane.draw(self.bev, self.detections, pursuit)
