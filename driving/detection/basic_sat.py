import cv2
import numpy as np
from copy import deepcopy
from typing import Optional, Tuple
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

from driving import basic


class Basic_Sat:
    input_topic = 'camera'

    def __init__(self, lane_mode: bool = True):
        self.lane_mode = lane_mode
        self._debug_frame: Optional[Tuple[np.ndarray, Optional[int], Header]] = None

    @classmethod
    def from_settings(cls) -> 'Basic_Sat':
        """등록표에서 호출하는 basic 전용 생성 함수."""
        return cls(lane_mode=basic.SETTINGS.lane_mode)

    def changeLaneMode(self):
        self.lane_mode = not self.lane_mode
        self._debug_frame = None

    def __perspectiveTransformation(self, src, src_ptr: Optional[np.ndarray]=None, dst_ptr: Optional[np.ndarray]=None) -> np.ndarray:

        if src_ptr is None:
            src_ptr = np.array([[90,360],[550,360],[640,480],[0,480]], dtype=np.float32)
        if dst_ptr is None:
            dst_ptr = np.array([[20,0],[620,0],[640,120],[0,120]], dtype=np.float32)

        mtrx = cv2.getPerspectiveTransform(src_ptr, dst_ptr)
        src = cv2.warpPerspective(src, mtrx, (640, 120))
        return src

    def __morphology(self, src: np.ndarray) -> np.ndarray:
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        src = cv2.erode(src, kernel)
        src = cv2.dilate(src, kernel2)
        return src

    def __linear_saturate(self, src, alpha, beta):
        src_f = src.astype(np.float32)
        
        sat = src_f * alpha + beta
        sat = np.clip(sat, 0, 255)
        return sat.astype(np.uint8)

    def __findMoment(self, src: np.ndarray) -> Optional[int]:
        contours, _ = cv2.findContours(src, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        sums = 0
        count = 0

        if np.count_nonzero(src) < len(src)*len(src[0])//4:
            for contour in contours:
                if cv2.contourArea(contour) < 100: continue
                m = cv2.moments(contour)
                sums += int(m['m10']/m['m00'])
                count += 1

        if count == 0: return None     # center 검출 실패
        return sums // count            # center 반환

    def detect(self, camera_msg: CompressedImage) -> Optional[int]:
        """공통 진입점. 전처리와 입력 영상 검사는 검출기가 담당한다."""
        return self.detectCenterSaturation(camera_msg)

    def detectCenterSaturation(self, camera_msg: Optional[CompressedImage]) -> Optional[int]:
        # 잘못된 새 입력 뒤에 이전 프레임의 디버깅 영상이 남지 않도록 한다.
        self._debug_frame = None
        if camera_msg is None or not camera_msg.data:
            raise ValueError('Camera image is missing or empty.')
        np_arr = np.frombuffer(camera_msg.data, np.uint8)
        src_origin = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if src_origin is None or src_origin.size == 0:
            raise ValueError('Could not decode the compressed camera image.')
        if src_origin.shape != (480, 640, 3):
            raise ValueError('basic_sat requires a 640x480 BGR image from camera.')
        src = self.__perspectiveTransformation(src_origin)

        if self.lane_mode:
            src = src[:, 0:320]
        else:
            src = src[:, 320:640]
        
        src = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        src = cv2.GaussianBlur(src, (15, 15), 0)
        src = self.__linear_saturate(src, 29.12, -5080.1)
        _, src = cv2.threshold(src, 160, 255, cv2.THRESH_BINARY)
        src = self.__morphology(src)


        center = self.__findMoment(src)  # 검출 실패시 None 반환
        self._debug_frame = (src, center, deepcopy(camera_msg.header))

        return center      # @src: binary image of detected lane, sums: x value of moment

    def get_debug_image(self) -> Optional[CompressedImage]:
        """검출에 사용한 이진 ROI 위에 center의 x 위치를 표시한다."""
        if self._debug_frame is None:
            return None
        mask, center, header = self._debug_frame
        overlay = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        if center is not None:
            # 검출 결과는 x뿐이다. y는 표시용 ROI 중앙이며 검출한 y좌표가 아니다.
            cv2.circle(overlay, (center, overlay.shape[0] // 2), 5, (0, 0, 255), -1)
        ok, encoded = cv2.imencode('.jpg', overlay)
        if not ok:
            raise ValueError('Could not encode the basic_sat debug ROI.')
        return CompressedImage(
            header=deepcopy(header), format='jpeg', data=encoded.tobytes(),
        )
