import cv2
import numpy as np
from typing import Optional, Tuple
from sensor_msgs.msg import CompressedImage

class Basic_Sat:
    def __init__(self):
        self.lane_mode=True

    def changeLaneMode(self):
        self.lane_mode = not self.lane_mode

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

    def detectCenterSaturation(self, camera_msg: Optional[CompressedImage]) -> Optional[int]:

        np_arr = np.frombuffer(camera_msg.data, np.uint8)
        src_origin = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
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

        return center      # @src: binary image of detected lane, sums: x value of moment
