# 알고리즘 이식과 기여 방법

다른 환경에서 시험한 알고리즘을 현재 `driving` 패키지에 연결하는 방법이다. [README.md](README.md)에서 현재 동작과 제한점을 먼저 확인한다. 에이전트는 [AGENTS.md](AGENTS.md)도 따른다.

## 1. 연결할 내용을 먼저 정한다

팀원의 기존 ROS 노드 전체를 추가하기 전에, 그 안의 영상 처리와 조향 계산을 Python 클래스로 분리한다. 구독·발행·파라미터 조회는 `DrivingNode`에 연결하고, 전처리와 계산은 팀원의 알고리즘 파일 안에 둔다.

PR 설명이나 클래스의 docstring에 다음 내용을 남긴다.

| 항목 | 작성할 내용 |
| --- | --- |
| 선택 이름 | 예: `basic_threshold` |
| 입력 | 토픽, 메시지 타입, 해상도, 색상 순서 |
| 전처리 | 디코딩, 원근변환, ROI 등을 어느 코드가 수행하는지 |
| 결과 | 반환 타입, 좌표 원점·단위·범위 또는 곡선/오차 등의 의미 |
| 미검출 | 어떤 결과를 반환하고, 연결된 조향기가 어떻게 처리하는지 |
| 조향기 | 기존 `Basic_P`를 쓰는지, 새 조향기가 필요한지 |
| 파라미터 | 이름, 타입, 기본값, 조절 효과 |
| 디버그 | 표시할 처리 단계와 점·선의 의미 |

예를 들어 기존 `Basic_P`는 선택한 폭 320 ROI 안의 x좌표를 받는다. 전체 영상의 x좌표나 각도 오차를 그대로 넘기면 계산의 의미가 달라진다. 다른 결과가 필요한 알고리즘은 그 결과에 맞는 조향기를 연결한다. 검출기와 조향기의 함수 이름을 전부 같은 이름으로 바꿀 필요는 없다.

현재 `None` 처리 등 남은 제한은 [README의 알려진 문제](README.md#확인한-결과와-남은-범위)를 확인한다. 현재 Sat/Sobel 분기는 검출 함수와 `Basic_P.makeTwist(center, lane_mode)`에 같은 좌우 모드를 인자로 전달한다.

## 2. 검출기 하나를 추가하는 전체 예제

아래 `basic_threshold`는 **이식 위치를 보여 주기 위한 설명용 예제**다. 현재 설치된 검출기는 `basic_sat`, `basic_sobel`, `yolo_lane`이며 조향기는 `basic_p`, `pure_pursuit`다. 예제를 따라 파일과 분기를 추가한 뒤에만 `basic_threshold`를 선택할 수 있다. 실제 주행 성능을 확인한 알고리즘으로 간주하지 않는다.

이 예제는 회색 영상의 임계값으로 차선을 검출하며 기존 `Basic_P`를 재사용한다. 자신의 알고리즘을 이식할 때는 계산 부분과 파라미터를 자신의 코드로 바꾸고 아래 연결 순서를 따른다.

### 2-1. 검출 파일을 만든다

패키지 루트 기준 `driving/detection/basic_threshold.py`를 만든다. 같은 디렉터리의 `__init__.py`는 유지한다.

```python
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage


class Basic_Threshold:
    """640x480 입력 → 한쪽 320x120 ROI의 x좌표. 미검출은 좌0/우320."""

    def __init__(self):
        self.center = None
        self.debug_image = None

    def detectCenterThreshold(
        self, camera_msg: CompressedImage, threshold: int = 180,
        lane_mode: bool = True,
    ) -> int:
        self.center = None
        self.debug_image = None
        if camera_msg is None or not camera_msg.data:
            raise ValueError('Camera image is missing or empty.')
        if not 0 <= threshold <= 255:
            raise ValueError('threshold must be between 0 and 255.')

        data = np.frombuffer(camera_msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError('Could not decode the compressed camera image.')
        if frame.shape != (480, 640, 3):
            raise ValueError('basic_threshold requires a 640x480 BGR image.')

        # 이 예제의 ROI는 기존 Basic_P와 같은 좌표 기준을 사용한다.
        src_points = np.float32([[90, 360], [550, 360], [640, 480], [0, 480]])
        dst_points = np.float32([[20, 0], [620, 0], [640, 120], [0, 120]])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped = cv2.warpPerspective(frame, matrix, (640, 120))
        roi = warped[:, :320] if lane_mode else warped[:, 320:640]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (15, 15), 0)
        _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        centers = []
        if np.count_nonzero(mask) < mask.size // 4:
            contours, _ = cv2.findContours(
                mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE,
            )
            for contour in contours:
                if cv2.contourArea(contour) < 100:
                    continue
                moment = cv2.moments(contour)
                centers.append(int(moment['m10'] / moment['m00']))

        fallback = 0 if lane_mode else 320
        self.center = sum(centers) // len(centers) if centers else fallback
        self.debug_image = mask
        return self.center

    def debug(self):
        if self.debug_image is None:
            return None
        image = cv2.cvtColor(self.debug_image, cv2.COLOR_GRAY2BGR)
        cv2.circle(image, (self.center, 60), 5, (0, 0, 255), thickness=-1)
        return image
```

이 예제는 기존 Basic_P와 같은 좌표 기준과 미검출 대체값을 사용한다. 디버그 영상은 검출 반환값에 섞지 않고 `debug()`로 따로 가져온다.

### 2-2. 노드에서 import와 선택 이름을 추가한다

`driving/driving.py`의 import 부분에 추가한다.

```python
from driving.detection.basic_threshold import Basic_Threshold
```

기존 `Detection` 선언을 다음처럼 바꾼다. 기본 선택값 `self.lane_detection_algorithm = 'yolo_lane'`는 유지하고 새 알고리즘은 `/driving/detection`으로 선택한다. 조향기는 아래 검출 분기에서 직접 호출하므로 별도 선택 이름을 등록하지 않는다.

```python
Detection = Literal['basic_sobel', 'basic_sat', 'yolo_lane', 'basic_threshold']
```

현재 선택 콜백은 `get_args(Detection)`을 읽으므로 허용 목록은 이 선언에서 확장된다. 이 단계만으로는 검출 코드가 실행되지 않는다. 아래 객체와 호출 분기까지 연결한다.

### 2-3. 객체와 파라미터를 추가한다

다음 두 줄을 각각 `DrivingNode.__init__`의 detector 생성 구역과 파라미터 선언 구역에 추가한다.

```python
self.basic_threshold = Basic_Threshold()
self.declare_parameter('basic_threshold.threshold', 180)
```

객체를 매 프레임 생성하지 않는다. 이미 선언된 `basic.lane_mode`는 이 basic 예제에서 함께 사용하며 중복 선언하지 않는다. 새 알고리즘에 좌우 모드가 필요 없다면 해당 파라미터를 전달할 필요도 없다.

### 2-4. 카메라 콜백에 실행 분기를 추가한다

아래 코드는 `camera_callback` 안의 기존 두 분기 다음에 넣는 예제다. 대기 중에도 검출·조향 계산과 디버그 발행을 유지하고, 정상 속도와 오류 시의 0 속도 발행에 각각 `lane_enabled` 조건을 둔다. `msg`, `cv2`, `Twist`는 기존 함수 인자와 import를 사용한다.

```python
if self.lane_detection_algorithm == 'basic_threshold':
    try:
        threshold = self.get_parameter('basic_threshold.threshold').value
        lane_mode = self.get_parameter('basic.lane_mode').value
        center = self.basic_threshold.detectCenterThreshold(
            msg, threshold=threshold, lane_mode=lane_mode,
        )
        # 같은 ROI를 해석하도록 조향기에도 동일한 모드를 전달한다.
        twist = self.basic_p.makeTwist(center, lane_mode)
    except (ValueError, cv2.error) as error:
        self.get_logger().warning(f'Camera processing failed: {error}')
        if self.lane_enabled:
            self.cmd_vel_pub.publish(Twist())
        return

    if self.lane_enabled:
        self.cmd_vel_pub.publish(twist)
    self.publish_debug_basic_threshold()
```

**좌우 모드 전달:** 기존 Sat/Sobel과 마찬가지로 새 분기에서도 `makeTwist(center, lane_mode)`로 모드를 전달한다. `Basic_P`는 전달받은 인자로 설정을 선택하며 객체에 좌우 모드를 저장하지 않는다. 두 번째 인자를 생략하면 기본값 `True`가 사용되므로 오른쪽 모드에서도 왼쪽 기준으로 계산된다. 각 분기에서 현재 모드를 빠짐없이 전달한다.

검출 결과의 타입이나 의미가 다르면 해당 조향 호출을 바꾼다. 기존 `Basic_P`는 현재 `None`을 처리하지 않으므로 `None`을 그대로 전달하지 않는다. 새 조합의 미검출 정책을 해당 알고리즘과 연결부에서 정한다.

### 2-5. 디버그 영상을 연결한다

다음 메서드를 `DrivingNode` 클래스의 기존 디버그 메서드들 옆에 추가한다.

```python
def publish_debug_basic_threshold(self):
    image = self.basic_threshold.debug()
    if image is None:
        return
    stamp = self.get_clock().now().to_msg()
    self.publish_compressed(self.debug_pub1, image, stamp)
```

`debug()`는 현재 프레임의 표시용 NumPy 영상을 제공하고, 노드가 기존 `publish_compressed`로 JPEG 메시지를 만든다. 예제는 BGR `uint8` 영상을 반환한다. 원본 계산용 마스크를 직접 덧그려 바꾸지 않고 표시본을 만든다.

현재 Sat/Sobel과 같은 `/debug1`을 사용하면 구독자는 선택한 검출기의 영상을 계속 볼 수 있다. 알고리즘 하나를 추가할 때마다 새 토픽이 필요한 것은 아니다. 여러 처리 단계의 영상을 동시에 보고 싶을 때 별도 발행을 연결한다. `/debug2`는 현재 선언만 돼 있으므로 사용하려면 실제 발행 코드를 넣어야 한다.

모든 검출기는 디버그 표시본을 만들고 노드는 토픽으로만 발행한다. 새 알고리즘에도 GUI 창이나 키 입력 코드를 넣지 않는다. 화면 표시는 `driving_debug` 노드가 담당한다. 현재 camera/camera_raw/camera_raw_16_9/camera2/debug1/debug2를 모두 구독하므로 기존 debug1을 쓰면 뷰어 수정이 필요 없다. 새 영상 토픽을 만들면 `driving/driving_debug.py`의 `IMAGE_TOPICS`와 README도 갱신한다. 공통 발행 함수는 전달한 stamp와 선택적 frame_id를 사용한다. basic 예제는 출력 시각을 사용한다. YOLO처럼 입력과 대응시키려면 `publish_compressed(self.debug_pub1, image, msg.header.stamp, msg.header.frame_id)`로 호출한다.

### 2-6. 빌드하고 선택해 본다

README의 빌드·실행 절차를 따른 뒤, ROS 환경을 읽은 다른 터미널에서 실행한다. 아래 명령은 위 파일과 분기를 실제로 추가했을 때만 유효하다.

```bash
ros2 topic pub --once /driving/detection std_msgs/msg/String "{data: basic_threshold}"
ros2 param set /driving basic_threshold.threshold 190
ros2 param get /driving basic_threshold.threshold
ros2 topic pub --once /lane_state std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /driving/detection std_msgs/msg/String "{data: basic_sat}"
```

위 예제는 `basic_threshold`를 선택하면 해당 분기에서 `Basic_P`를 직접 호출한다. 노드가 새 분기에 들어가고 새 검출기가 만든 중심으로 조향하는지 확인한다. 선택 로그만 보고 연결 완료라고 판단하지 않는다. 같은 `detection` Python 패키지 안에 파일을 추가하면 현재 `setup.py`의 `find_packages`가 포함하므로 실행 엔트리 포인트를 더 만들 필요는 없다. 외부 라이브러리를 새로 쓰는 경우에는 의존성과 Galactic 설치 가능 여부도 확인한다.

## 3. 기존 파라미터 전달 방식을 따른다

현재 Sat 연결은 다음 순서다.

```python
# __init__에서 한 번 선언
self.declare_parameter('basic_sat.beta', -5080.1)

# Sat 카메라 분기에서 해당 프레임에 사용할 값을 읽는다.
beta = self.get_parameter('basic_sat.beta').value
lane_mode = self.get_parameter('basic.lane_mode').value
center = self.basic_sat.detectCenterSaturation(msg, beta, lane_mode)
twist = self.basic_p.makeTwist(center, lane_mode)
```

위 선언은 현재 코드에 이미 있다. 설명을 따라 중복 추가하지 않는다. 새 값도 알고리즘 이름을 앞에 둔 파라미터로 선언하고 해당 분기에서 함수 인자로 넘긴다. 공통 설정이라면 사용하는 모든 검출기와 조향기에 빠짐없이 전달한다.

기본값이 실수이면 실수로, 정수이면 정수로 설정한다. `get_parameter`로 읽은 값은 다음에 실행되는 해당 영상 분기에서 사용되며, 실행 중인 계산의 인자를 중간에 바꾸지는 않는다. 프레임마다 읽는 값에는 별도 파라미터 변경 콜백이나 설정 파일 저장이 없다. YOLO 모델 경로·장치·처리 주기는 시작 전용(read_only)이며 실행 중 변경을 거부한다. 모델 경로는 ROS share 설치 경로에서 구한다. 숫자의 유효 범위를 검사해야 하면 실제 알고리즘의 조건을 명시한다. 위 예제는 0~255를 벗어난 threshold를 검출 시 오류로 처리하므로, 파라미터 설정 명령 자체의 사전 거부와는 다르다.

## 4. 조향기를 추가할 때

현재 Sat/Sobel은 Basic_P, YOLO는 LanePurePursuit를 항상 호출한다. 조향 선택용 `Steering` 타입이나 `/driving/steering` 구독은 없다. 새 조향기는 사용할 검출기의 분기에 직접 연결한다.

1. `driving/steering/`에 계산 클래스를 만든다. 입력의 좌표·단위·미검출 의미와 반환하는 `Twist`의 의미를 적는다.
2. 노드에서 import하고 `__init__`에서 객체를 한 번 만든다.
3. 해당 검출 분기의 조향 호출을 새 객체의 메서드로 연결한다. Sat/Sobel 같은 즉시 처리는 `camera_callback`, YOLO는 `process_yolo`에 호출이 있다.
4. 필요한 파라미터, 검출기와 공유하는 모드, 입력 결과 형식을 연결한다. 이전 프레임 상태를 보관하는 조향기는 검출 선택 변경 시 초기화도 연결한다.

예를 들어 `My_Controller`와 `self.my_controller`를 구현했고 같은 ROI 중심을 입력받는다면, 사용할 검출 분기 안의 조향 호출을 다음처럼 작성한다. `my_controller`는 설명용 이름이며 현재 설치된 조향기가 아니다.

```python
twist = self.my_controller.makeTwist(center, lane_mode=lane_mode)
```

`lane_mode`가 필요 없는 새 조향기는 그 인자를 생략하면 된다. 이 예제의 메서드 시그니처를 모든 조향기의 공통 인터페이스로 강제하지 않는다. 객체를 추가한 뒤 해당 검출 분기에서 실제로 호출하는지 확인한다.

검출 결과의 좌표·단위·미검출 의미를 해석할 수 있는 조향기를 연결한다. 검출기 하나에 여러 조향기를 실행 중 바꿔야 하는 요구가 생기면 그때 선택 방식을 설계한다. 현재는 각 분기의 직접 호출을 유지하며 범용 호환성 등록 시스템을 추가하지 않는다.

### 실제 추가 사례: YOLO + Pure Pursuit

다음은 설명용 가상 예제가 아니라 현재 구현된 연결이다.

1. `detection/yolo_lane.py`: `detectLane(msg, ...)`에서 디코딩, BEV, 원본 `detect()` 추론과 `LanePath.update/target`을 호출한다. `(target, mode)`를 반환하고 `debug(pursuit)`로 원본 표시본을 만든다.
2. `detection/lane_estimator.py`: 원본의 기하 계산과 `LanePath`를 보관한다. ROS 구독·파라미터 조회를 넣지 않는다.
3. `steering/pure_pursuit.py`: 원본 PurePursuit 계산을 보관한다. `LanePurePursuit.makeTwist(target, mode, ...)`가 일반·교차로 속도 정책과 `compute_to()`를 연결한다. 노드에서 전달한 `intersection`이 참이면 `v_max_intersection`과 내부 곡률 배율 0.7을 적용한다. 미검출과 한쪽 차선 정책은 원본대로 유지한다.
4. 노드: `YoloLane`은 첫 유효 YOLO 프레임에서 생성하고 모델을 재사용한다. `LanePurePursuit`는 시작 때 생성한다. `camera_callback`은 YOLO 조합이면 최신 메시지만 저장하고 반환하며, `process_yolo` 타이머에서 계산·발행한다.
5. 파라미터: 원본 값들은 노드에서 `yolo_lane.*`, `pure_pursuit.*`로 선언하고 함수 인자로 전달한다. 원본의 창 표시 옵션은 제거하고 디버그 토픽 발행만 유지했다. 모델 경로·장치·처리 주기는 시작 전용이며 나머지는 처리할 때 읽는다. 기하 설정 변경 시 검출 추적을 초기화한다.
6. 선택 전환: `selection_changed`가 0 Twist를 발행하고 `reset_yolo`로 대기 영상·차선 추적·조향 평활화를 비운다. 새로운 상태 보유 알고리즘은 선택 변경과 `lane_state_callback`의 정지 처리에서 그 알고리즘의 reset도 연결한다. 매 프레임 초기화하면 추적과 평활화가 사라지므로 전환과 프레임 처리를 구분한다.
7. 자산: 모델을 `models/`에 두고 `setup.py`의 data_files로 share 경로에 설치했다. Python 외 의존성은 package.xml, 선택적 추론 의존성은 requirements-yolo.txt와 README의 환경 안내에 기록했다. 참조 저장소가 없어도 빌드한 패키지만으로 기본 모델을 찾을 수 있다.

`target`은 basic의 픽셀 중심과 달리 차축 기준 전방 x/좌측 y 미터 좌표다. `None`은 정상 차선 미검출이며 기본 0.05m/s 직진으로 처리한다. `mode`는 both/yellow_only/white_only/lost이고 외부 enable `/lane_state`와는 별개다. `/lane_state`는 초기 False이며 검출·조향 계산과 디버그 발행은 대기 중에도 수행한다. 정상·오류 속도 발행에만 허용 상태를 적용한다. 정지 신호는 미처리 영상·추적·평활화를 초기화하지만 검출 선택과 교차로 상태는 유지한다. 이후 대기 중에도 추적·평활화는 새 영상으로 갱신된다. False→True에서는 미처리 영상만 비우고 추적·평활화는 유지하며, 재개 후 수신한 영상으로 속도를 발행한다. `/intersection_state` Bool은 노드가 구독해 `is_intersection`에 저장하고 YOLO 조향에만 전달한다. 초기값은 False이고 알고리즘 선택 변경에도 마지막 수신값을 유지한다. 교차로 상태 변경 자체는 추적·평활화를 초기화하지 않는다. `pure_pursuit.v_max_intersection`은 노드가 선언·조회하는 실시간 파라미터다. 새 조향기에 교차로 정책이 필요하면 해당 호출부에 상태와 설정을 명시적으로 전달한다. 원본 목표점을 `compute_to`에 전달하는 순서와 정상 미검출·한쪽 차선 정책을 보존해야 한다.

15Hz는 YOLO에만 적용한다. 타이머는 새 메시지가 없으면 건너뛰고, 선택 전환 시 저장한 메시지를 버린다. 단일 executor이므로 느린 추론을 비동기로 바꾸는 기능은 아니다. 타이머는 대기 중에도 처리하며 `lane_enabled`는 정상·오류 속도 발행에 적용한다. 새 타이머 기반 알고리즘도 False→True 전환 때 미처리 영상을 비워 재개 이전 영상으로 속도를 발행하지 않도록 한다. 반복 True에서는 영상을 버리지 않는다. 영상 콜백과 타이머에서 중복 계산하지 않는다.

원본 계산을 나눌 때는 같은 마스크 **순서**에 대한 목표점·속도·평활화 상태를 비교한다. 독립 영상만 비교하면 이전 프레임을 사용하는 추적 차이를 놓칠 수 있다. 실제 추론 시간은 디버그 생성까지 포함해 측정하고, 타이머 검사에서 대체한 추론을 실제 모델 성능으로 보고하지 않는다.

## 5. 다른 카메라 입력을 사용할 때

현재 `camera_raw_callback`과 `camera_raw_16_9_callback`은 마지막 메시지만 저장한다. 새 알고리즘에서 이 값을 사용하려면 어떤 영상이 계산을 시작하는지 정해야 한다.

- `/camera` 수신을 계기로 저장된 raw 영상을 함께 쓸 경우, raw 메시지가 아직 `None`인 경우를 처리한다. 저장된 영상이 같은 시각에 촬영됐다고 가정하지 않는다.
- raw 토픽 수신을 계기로 계산한다면 해당 콜백의 정상·오류 속도 발행에 `lane_enabled` 검사를 넣고, 대기 중 검출·디버그 처리는 유지한다. `/camera` 콜백에서도 같은 알고리즘을 다시 실행해 속도를 중복 발행하지 않도록 한다.
- 새 검출기에서 해상도·디코딩·전처리를 처리한다. 기존 basic의 640×480 조건을 없애 다른 알고리즘까지 한꺼번에 맞추는 방식으로 변경하지 않는다.

현재 세 입력을 동기화하는 기능은 없다. 여러 영상의 정확한 시간 대응이 필요한 알고리즘은 그 요구와 구현 범위를 따로 설명한다.

## 6. 연결을 확인한다

속도 출력은 README 예제처럼 `/cmd_vel_preview`로 remap해서 확인할 수 있다. 파라미터와 선택 명령은 노드의 실제 namespace에 맞춰 실행한다.

| 확인할 상황 | 확인할 결과 |
| --- | --- |
| 주행 시작·정지·재개 | 대기 중 검출·디버그는 계속되고 정상·오류 속도 발행은 없는가? False 수신 시 0 속도·상태 초기화, False→True 시 미처리 영상 폐기 후 새 영상으로 주행하는가? 반복 True는 영상·추적을 유지하는가? |
| 기존 기본 조합 | 변경 전과 같은 입력에서 계산 결과가 유지되는가 |
| 새 알고리즘 선택 | 해당 분기가 실제로 실행되고 그 결과로 속도를 계산하는가 |
| 기존 알고리즘으로 복귀 | 이전 모드나 오래된 디버그 영상이 잘못 사용되지 않는가 |
| 정상 영상 / 차선 미검출 | 약속한 중심·오차 또는 미검출 정책대로 동작하는가 |
| 빈 데이터 / 손상 영상 / 다른 해상도 | 입력 오류 경로로 처리되고 기본 노드 연결에서는 0 Twist가 발행되는가 |
| 파라미터 변경 | 다음에 처리하는 해당 알고리즘의 프레임에 실제 계산 변화가 있는가 |
| YOLO 교차로 진입·해제 | 원본의 속도 기준·곡률 배율·평활화가 반영되고, 미검출·한쪽 차선·basic 복귀가 정책대로 동작하는가 |
| 좌우 모드 | 검출 ROI와 조향 목표가 같은 모드인가 |
| 디버그 영상 | 현재 처리 프레임의 표시본이며 JPEG로 디코딩할 수 있는가 |
| 입력 이름·타입·QoS | 발행기와 구독기가 실제로 연결되는가 |

빌드 성공과 주행 성능은 별개의 확인이다. 가능하면 같은 입력 영상으로 변경 전후를 비교하고, 새 알고리즘에 필요한 검사를 추가한다. 모든 PR에 대규모 테스트 기반을 새로 만들 필요는 없다. 실행한 환경과 관찰 결과를 남기고, 합성 영상·bag·실차 결과를 구분한다.

현재 README에 기록된 미해결 문제를 새 기능이 이미 해결한 것처럼 보고하지 않는다. 검증 중 문제를 발견하면 재현 조건과 남은 영향을 기록한다.

## 7. 브랜치에서 작업하고 PR을 만든다

저장소 루트에서 작업한다. PR 대상 브랜치는 팀이 합의한 기준을 사용한다. 현재 기준 브랜치는 `main`이며, 아래 예제는 이 브랜치를 기준으로 한다. 이후 기본 브랜치가 바뀌면 기준과 PR 대상을 함께 바꾼다.

먼저 `git status`로 작업 중인 변경을 확인한다. 변경이 남아 있다면 보존한 뒤 브랜치를 옮긴다.

```bash
git status
git fetch origin
git switch main
git pull --ff-only origin main
git switch -c feat/basic-threshold
```

알고리즘과 연결 코드를 작성하고 확인한 뒤, 필요한 파일만 커밋한다. 아래 파일명과 커밋 메시지는 위 예제를 완료한 경우다. 문서를 바꾸었다면 해당 문서도 `git add`에 포함한다.

```bash
git diff --check
git diff
git add driving/detection/basic_threshold.py driving/driving.py
git diff --cached
git commit -m "feat: threshold 차선 검출 추가"
git push -u origin feat/basic-threshold
```

GitHub에서 새 브랜치의 PR을 열고 **base가 합의한 대상 브랜치, compare가 자신의 기여 브랜치**인지 확인한다. 저장소에 push 권한이 없다면 fork의 원격으로 브랜치를 push하고 원본 저장소의 대상 브랜치에 PR을 만든다. 빌드 결과, 로컬 데이터와 bag은 커밋하지 않는다. 이번 YOLO처럼 실행에 필요한 모델을 포함할 때는 출처·해시·설치 경로를 명시한다. 큰 모델은 팀의 배포 방식을 따르고 PR에 설명한다.

PR 설명에는 다음 내용을 넣는다.

- 해결하려는 문제와 추가한 선택 이름
- 입력·전처리·반환값·미검출 의미 및 연결한 조향기
- 추가하거나 변경한 파라미터와 디버그 영상의 의미
- 기존 동작에 영향을 주는 변경
- 검증 환경, 사용한 입력, 실제 결과, 미확인 사항

예를 들어 “Sobel과 비교하기 위해 threshold 검출을 추가했다. 동일한 ROI 좌표를 반환해 Basic_P를 사용한다. 합성 직선·미검출·선택 복귀를 확인했으며 실제 Galactic과 실차에서는 아직 검증하지 않았다”처럼 구현과 확인 범위를 함께 적는다.


## 8. 외부 미션 패키지와 연결할 때

미션은 사용자가 지정한 외부 저장소의 패키지를 사용한다. `driving`에 미션 노드나 실행 엔트리를 추가하거나 `/driving/detection` 목록에 미션 이름을 넣지 않는다. 외부 미션의 실행 예제와 입력 조건은 README의 외부 미션 연결 절을 참고한다.

- 주행 허용은 `/lane_state` Bool로 연결한다. False 수신은 정지와 미처리 영상·추적 초기화, False→True 전환은 미처리 영상 폐기 후 새 수신 영상부터 주행을 의미한다. 대기 중에도 검출·조향 계산·디버그 발행은 계속되며 처리 오류로 0 속도를 발행하지 않는다.
- 현재 `/lane_mode`는 구독하지 않는다. `/intersection_state`는 YOLO의 속도·곡률 정책이며 미션 전체의 방향 전환을 구현하는 신호가 아니다. 미연결 토픽의 동작을 가정하지 않는다.
- 원본 신호등은 시작할 때 lane False를 보내지 않으므로 먼저 정지 상태를 설정한다. 외부 미션의 시작/취소와 주행 허용은 실제 코드에 맞춰 구분한다.
- 직접 속도를 발행하는 외부 미션을 연결할 때는 차선 주행과의 제어권 전환을 확인한다. 모든 미션의 속도 경쟁이 자동 해결된다고 가정하지 않는다.
- 새 외부 디버그 토픽을 표시하려면 실제 메시지 타입·영상 내용을 확인하고 뷰어의 IMAGE_TOPICS와 문서를 갱신한다. 현재는 camera2 원본 영상만 추가로 표시하며 신호등 전용 JPEG 발행은 없다.
- 외부 패키지의 소스 수정과 검증은 요청 범위를 확인해 별도로 진행한다. 주행 인터페이스 검사와 실제 미션 전체의 성공을 구분해서 기록한다.
