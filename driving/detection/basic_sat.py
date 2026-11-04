import cv2
import numpy as np
from typing import Optional
from sensor_msgs.msg import CompressedImage

class Basic_Sat:
    def __init__(self):
        self.lane_mode=True

    # def changeLaneMode(self):
    #     self.lane_mode = not self.lane_mode

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

        if count == 0:     # center 검출 실패
            if self.lane_mode: return 0
            else: return 320
        return sums // count            # center 반환

    def detectCenterSaturation(self, camera_msg: Optional[CompressedImage], beta=-5080.1, lane_mode=True):

        # 입력 오류는 예외로 알리고, 정상 영상에서의 차선 미검출만 None으로 반환한다.
        if camera_msg is None or not camera_msg.data:
            raise ValueError('Camera image is missing or empty.')
        np_arr = np.frombuffer(camera_msg.data, np.uint8)
        src_origin = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if src_origin is None:
            raise ValueError('Could not decode the compressed camera image.')
        if src_origin.shape != (480, 640, 3):
            raise ValueError('basic_sat requires a 640x480 BGR image from camera.')
        src = self.__perspectiveTransformation(src_origin)

        self.lane_mode = lane_mode

        if self.lane_mode:
            src = src[:, 0:320]
        else:
            src = src[:, 320:640]
        
        src = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        src = cv2.GaussianBlur(src, (15, 15), 0)
        src = self.__linear_saturate(src, 29.12, beta)
        _, src = cv2.threshold(src, 160, 255, cv2.THRESH_BINARY)
        src = self.__morphology(src)


        self.center = self.__findMoment(src)
        self.debug_image = src

        return self.center      # @src: binary image of detected lane, sums: x value of moment

    def debug(self):
        img = cv2.cvtColor(self.debug_image, cv2.COLOR_GRAY2BGR)
        if self.center is not None:
            cv2.circle(img, center=(self.center, 60), radius=5, color=(0, 0, 255), thickness=-1)
        return img