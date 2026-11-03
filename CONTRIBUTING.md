# driving 기여 안내

이 문서는 각자 환경에서 개발한 차선 검출·조향 코드를 `driving`에 이식하고,
팀 저장소에 PR로 제출하는 과정을 설명한다. 대상 환경은 **ROS 2 Galactic /
Ubuntu 20.04 / Python 3.8**이다.

작업 규칙은 [AGENTS.md](AGENTS.md), 구현된 기능의 범위와 한계는
[KNOWN_LIMITATIONS.md](KNOWN_LIMITATIONS.md)를 함께 확인한다.
파일 이름은 GitHub에서 기여 안내로 인식하는 `CONTRIBUTING.md`를 사용한다.
([GitHub의 기여 안내 파일 설명](https://docs.github.com/en/communities/setting-up-your-project-for-healthy-contributions/setting-guidelines-for-repository-contributors))

## 목차

1. [전체 구조와 파일 위치](#1-전체-구조와-파일-위치)
2. [개발 환경과 작업 브랜치 준비](#2-개발-환경과-작업-브랜치-준비)
3. [기존 코드에서 가져올 부분 정하기](#3-기존-코드에서-가져올-부분-정하기)
4. [검출 결과 계약 정하기](#4-검출-결과-계약-정하기)
5. [이식 예제: 검출·조향·JPEG 디버깅](#5-이식-예제-검출조향jpeg-디버깅)
6. [알고리즘 등록과 선택](#6-알고리즘-등록과-선택)
7. [파라미터·의존성·리소스](#7-파라미터의존성리소스)
8. [빌드와 검증](#8-빌드와-검증)
9. [커밋·push·GitHub PR](#9-커밋pushgithub-pr)
10. [자주 발생하는 문제](#10-자주-발생하는-문제)

## 1. 전체 구조와 파일 위치

### 한 노드 안에서 알고리즘을 교체한다

```text
camera / camera_raw / camera_raw_16_9 (CompressedImage)
                         │
              활성 검출기의 입력 하나 선택
                         │
                 detector.detect(msg)
                 디코딩·전처리·차선 검출
                         │ 결과 계약에 맞는 객체
                 controller.compute(result)
                         │ Twist
              detector.get_debug_image()  ← 선택 기능
                         │
            driving 노드가 유효한 프레임 결과 발행
             ├─ cmd_vel
             └─ driving/debug/roi/compressed
```

노드는 영상 수신과 ROS 통신을 담당한다. 팀원은 **한 영상에서 결과를 구하는 코드**와
**그 결과로 Twist를 만드는 코드**를 가져오면 된다. 기존 프로그램의 ROS 노드를 통째로
추가하는 방식은 이 구조와 맞지 않는다.

| 경로 | 역할 / 수정하는 경우 |
| --- | --- |
| `driving/driving.py` | 구독·선택·worker·발행·감시. 일반적인 알고리즘 추가에는 수정 불필요 |
| `driving/interfaces.py` | 검출·조향·선택적 디버깅 메서드 규약 |
| `driving/registry.py` | 선택 이름과 실제 클래스/생성 함수 연결 |
| `driving/detection/` | 전처리를 포함한 팀 검출 코드 |
| `driving/steering/` | 팀 조향 코드 |
| `driving/basic.py` | basic 조합 전용 결과 형식과 좌우 설정 |
| `driving/steering/basic_p_parameters.py` | basic_p의 ROS 파라미터 선언·검증·적용 |
| `test/test_pipeline.py` | 기존 알고리즘 및 ROS 파이프라인 회귀 검사 |
| `package.xml`, `setup.py` | 의존성, Python 패키지와 리소스 설치 |

현재 실제 구현은 `basic_sat` / `basic_p`다. `basic_sobel`, `yolo`, `pure_pursuit`는
선택 이름의 타입에만 있으며 구현·등록 전에는 선택할 수 없다.

### 토픽과 실행 방식

아래는 namespace를 주지 않았을 때의 이름이다. 소스의 토픽 이름은 상대 이름이므로
namespace/remap을 사용하면 실제 경로가 바뀐다.

| 토픽 | 타입 | 용도 / QoS |
| --- | --- | --- |
| `/camera` | `sensor_msgs/msg/CompressedImage` | 카메라 입력, sensor-data QoS |
| `/camera_raw` | 같은 타입 | 다른 카메라 입력, sensor-data QoS |
| `/camera_raw_16_9` | 같은 타입 | 다른 카메라 입력, sensor-data QoS |
| `/driving/detection` | `std_msgs/msg/String` | 검출기 선택, depth 10 |
| `/driving/steering` | `std_msgs/msg/String` | 조향기 선택, depth 10 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 속도 명령, depth 10 |
| `/driving/debug/roi/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG 디버깅 영상, sensor-data QoS |

`raw`라는 이름도 이 패키지에서는 압축 영상 메시지를 뜻한다. 외부 환경의
`sensor_msgs/Image`를 그대로 넣을 수 있다는 뜻이 아니다. `input_topic`에는
선행 `/` 없이 `camera`, `camera_raw`, `camera_raw_16_9` 중 하나를 적는다.
기존 발행 토픽 이름이 다르면 remap을 사용한다. 메시지 타입이 다르면 별도 변환이 필요하다.

세 입력을 모두 구독해도 검출기는 하나만 받는다. 최신 메시지 세 개를 같은 촬영 시각의
동기화된 입력으로 간주하지 않는다. 기본 basic_sat은 `/camera`의 640×480 영상만 받는다.
다른 검출기는 자신의 입력 조건을 검사해야 한다.

## 2. 개발 환경과 작업 브랜치 준비

### 경로와 원격 저장소

이 문서에서는 workspace를 `~/turtlebot3_ws`, 패키지 저장소를
`~/turtlebot3_ws/src/driving`으로 가정한다. 자신의 workspace가 `turtlebot_ws` 등이라면
명령의 경로를 바꾼다. `src/driving` 안에 `package.xml`과 `setup.py`가 있어야 한다.

최초 복제 예시다. **`TEAM/REPOSITORY`는 실제 팀 저장소 주소로 바꾼다.**
문서 작성 시 로컬 저장소에는 원격이 설정되어 있지 않아 URL을 고정하지 않았다.
이미 저장소가 있으면 다시 clone하지 않는다.

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone https://github.com/TEAM/REPOSITORY.git driving
cd driving
git remote -v
```

이미 가진 로컬 저장소에 `origin`이 없는 경우에는 실제 팀 저장소 주소를 확인하고
`git remote add origin https://github.com/TEAM/REPOSITORY.git`으로 연결한다.
관련 없는 저장소를 원격으로 지정하거나 이미 있는 `origin`을 무작정 덮어쓰지 않는다.

### 최신 main에서 새 브랜치 만들기

먼저 `git status --short`를 보고 미완료 변경이 있다면 해당 작업의 커밋 또는 별도 보관을
마친다. 아래 명령은 작업 트리가 깨끗하고 원격 `main`이 존재하는 상황을 가정한다.

```bash
cd ~/turtlebot3_ws/src/driving
git status --short
git switch main
git fetch origin
git pull --ff-only origin main
git switch -c feat/team-lane-detector
```

`feat/team-lane-detector`는 예시다. `feat/sobel-detector`, `feat/pure-pursuit`,
`fix/lost-lane-handling`, `docs/algorithm-guide`처럼 목적을 드러내는 이름을 쓴다.
`pull --ff-only`가 실패하면 로컬과 원격의 이력이 갈라졌는지 확인한다.
작업을 잃는 `reset --hard`나 강제 push로 해결하지 않는다.

### ROS 환경 준비

Galactic이 설치된 터미널에서 다음을 실행한다. Python 환경은 ROS 설치본과 호환되어야 한다.

```bash
source /opt/ros/galactic/setup.bash
python3 --version
cd ~/turtlebot3_ws
rosdep install --from-paths src/driving --ignore-src -r -y --rosdistro galactic
```

`rosdep`, `colcon`, `pytest`가 없는 PC에서는 먼저 팀의 ROS 개발 환경을 준비한다.
새 터미널마다 ROS setup을 source한다. Galactic과 Humble setup을 같은 터미널에 겹쳐
source하지 않는다. Humble에서 개발했다면 PR에 그 사실과 Galactic 미검증 범위를 적는다.

## 3. 기존 코드에서 가져올 부분 정하기

먼저 기존 프로그램에서 아래 부분을 구분한다.

| 기존 코드 | 이 저장소에 옮기는 방법 |
| --- | --- |
| 카메라 구독 콜백, `VideoCapture`, `while` 영상 루프 | 제거하고 `detect(msg)`가 한 장을 받도록 변경 |
| JPEG 디코딩, ROI, HSV, BEV, 모델용 정규화 | 팀 검출기 또는 그 검출기가 호출하는 모듈에 둠 |
| 차선 검출 함수/학습 모델 추론 | 검출기의 핵심 계산으로 재사용 |
| 중심점·경로·오차 결과 | 좌표계·단위·미검출 의미를 정해 반환 |
| PID, P 제어, pure pursuit 등 | 조향기의 `compute(result)`로 옮김 |
| `cmd_vel.publish(...)` | 제거하고 `Twist`를 반환. 공통 노드가 발행 |
| `imshow`, `waitKey`, 별도 디버깅 publisher | `get_debug_image()`가 JPEG 메시지를 반환하도록 변경 |
| 개인 PC의 파일 경로, 모델/보정 파일 | 패키지 리소스 또는 명시적인 배포 경로로 변경 |

예를 들어 기존 `lane_center = find_lane(frame)` 함수는 그대로 재사용할 수 있다.
그 함수 앞에서 `CompressedImage`를 OpenCV 영상으로 바꾸고, 뒤에서 결과 계약에 맞는
값을 만들어 반환하는 얇은 연결 코드를 작성하면 된다.

이식 전에 다음을 기록해 두면 검출과 조향을 연결할 때 혼동을 줄일 수 있다.

- 사용한 카메라 토픽, 메시지 타입, 해상도, BGR/RGB 순서.
- 원본/보정 영상 여부, ROI 범위, BEV 또는 왜곡 보정의 적용 위치.
- 학습 모델·라이브러리 버전, CPU/GPU 필요 여부, 입력 한 장의 처리 시간.
- 검출 결과의 자료형, 좌표 원점·단위·방향, 정상 미검출 값.
- 조향 입력이 실제로 기대하는 의미와 선속도·각속도의 단위/부호.

특히 이미 보정된 영상에 같은 보정을 두 번 적용하거나, RGB로 학습한 모델에 BGR을
그대로 넣지 않도록 기존 환경의 전처리를 확인한다.

## 4. 검출 결과 계약 정하기

### 기존 basic 조합을 재사용하는 경우

`basic_p`를 조향기로 쓸 새 검출기는 아래 계약을 **모두** 맞춰야 한다.

| 항목 | basic 계약 |
| --- | --- |
| 식별자 | `driving.basic.ROI_CENTER_320`, 값은 `roi_center_x_320_v1` |
| 정상 결과 | 선택한 좌/우 ROI 내부 x좌표, 유한수, `0 <= x < 320` |
| 좌표 의미 | basic과 같은 투영·ROI 의미. 전체 영상 x좌표가 아님 |
| 미검출 | `None` |
| 좌우 일치 | 검출기와 조향기가 같은 `basic.SETTINGS.lane_mode`를 사용 |

오른쪽 ROI의 시작이 전체 영상 x=320이고 검출점이 전체 x=580이라면 반환값은
`580 - 320 = 260`이다. 다른 BEV 투영에서 얻은 x=260이 같은 물리 위치를 뜻하는지는
별도로 맞춰야 한다. 범위만 0~319로 바꿔서 호환된다고 판단하면 안 된다.

`Basic_Sat.from_settings()`처럼 생성 시 같은 basic 설정을 읽는 방식을 참고한다.
기존 `Basic_P`는 정상 미검출 `None`을 극단값으로 바꾸어 전진·회전을 계속한다.
이는 basic의 정책이며 새 조합의 미검출 정책으로 강제되는 것은 아니다.

### 새 결과 형식과 조향기를 함께 넣는 경우

두 차선 경계, 주행 경로, 곡률, 미터 단위 오차 등을 전달하고 싶으면 별도 결과 타입과
식별자를 정의한다. 예를 들어 `team_lane_error_normalized_v1`은 basic의 ROI 좌표와 다르다.

검출기의 `output_type`과 조향기의 `input_type`은 같은 식별자를 사용한다.
노드는 이 문자열만 비교하고 결과 객체 자체를 변환하지 않는다. 아래 항목은 두 구현이
같이 지켜야 한다.

- Python 자료형과 각 필드의 의미.
- 좌표계와 단위, 양수·음수 방향, 정상 범위.
- 미검출 표현과 그때의 조향 행동.
- 시간 정보가 필요하다면 결과에 담을 timestamp와 그 시계의 의미.

결과 의미가 바뀌면 식별자의 버전도 바꾸는 편이 좋다.
새 형식을 기존 식별자 이름으로 숨기면 선택 검사는 통과해도 잘못된 조향이 나올 수 있다.

## 5. 이식 예제: 검출·조향·JPEG 디버깅

아래는 **이식 구조를 설명하는 템플릿**이다. 저장소에 실제 등록된 알고리즘은 아니다.
`find_center_x()`를 자신의 검출 코드로 채워야 하며, 그대로는 검출이 실행되지 않는다.
예시 조향의 속도·이득도 설명용 값이므로 실차 튜닝값으로 간주하지 않는다.

이 예제는 원본 영상 안에서 검출한 주행 목표 x좌표를 정규화 오차로 바꾼다.
양수 오차는 목표가 영상 왼쪽에 있다는 뜻이며 조향기는 양의 `angular.z`를 만든다.
이 결과는 basic의 한쪽 차선 좌표와 호환되지 않는다.

### 5.1 검출기와 조향기가 공유할 결과 모듈

파일: `driving/team_contract.py`

```python
# file: driving/team_contract.py
from dataclasses import dataclass


TEAM_LANE_ERROR = 'team_lane_error_normalized_v1'


@dataclass(frozen=True)
class LaneObservation:
    found: bool
    # found=True일 때 [-1, 1]. 양수: 목표가 영상 왼쪽에 있음.
    error: float = 0.0
```

`found=False`가 이 조합의 미검출 표현이다. 공통 노드가 이 필드를 해석하지는 않는다.

### 5.2 다른 환경에서 개발한 핵심 검출 함수

파일: `driving/detection/team_core.py`

```python
# file: driving/detection/team_core.py
from typing import Optional

import numpy as np


def find_center_x(frame_bgr: np.ndarray) -> Optional[float]:
    """BGR 영상에서 주행 목표의 전체 영상 x좌표를 반환. 미검출이면 None."""
    # 이 부분을 자신의 기존 함수 호출 또는 계산 코드로 바꾼다.
    # 예: return existing_detector(frame_bgr)
    # ROI 좌표를 구했다면 전체 영상 좌표로 환산한 뒤 반환한다.
    raise NotImplementedError('여기에 팀의 검출 알고리즘을 이식하세요.')
```

기존 코드가 이미지가 아닌 경로·마스크·여러 점을 반환한다면 이 함수에 억지로 맞추지 않고
4절의 결과 계약부터 자신의 데이터에 맞게 변경한다. 상태가 필요한 모델이라면 함수 대신
객체로 보관하되 생성·추론 시간이 공통 실행 구조와 맞는지 확인한다.

### 5.3 ROS 영상 입력과 JPEG 디버깅을 연결하는 검출기

파일: `driving/detection/team_detector.py`

```python
# file: driving/detection/team_detector.py
from copy import deepcopy
from math import isfinite
from typing import Optional

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

from driving.detection.team_core import find_center_x
from driving.interfaces import CameraTopic
from driving.team_contract import LaneObservation, TEAM_LANE_ERROR


class TeamDetector:
    input_topic: CameraTopic = 'camera'
    output_type = TEAM_LANE_ERROR

    def __init__(self):
        self._debug_frame = None

    def detect(self, msg: CompressedImage) -> LaneObservation:
        self._debug_frame = None
        if msg is None or not msg.data:
            raise ValueError('Camera image is missing or empty.')
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None or frame.size == 0:
            raise ValueError('Could not decode the camera image.')

        width = frame.shape[1]
        # 고정 해상도/보정 조건이 필요한 알고리즘은 이 위치에서 검사한다.
        center = find_center_x(frame)
        if center is None:
            observation = LaneObservation(found=False)
        else:
            center = float(center)
            if not isfinite(center) or not 0 <= center < width:
                raise ValueError('Invalid center coordinate.')
            error = (width / 2.0 - center) / (width / 2.0)
            observation = LaneObservation(found=True, error=error)

        self._debug_frame = (frame.copy(), center, deepcopy(msg.header))
        return observation

    def get_debug_image(self) -> Optional[CompressedImage]:
        if self._debug_frame is None:
            return None
        frame, center, header = self._debug_frame
        overlay = frame.copy()
        if center is not None:
            cv2.circle(overlay, (int(center), overlay.shape[0] // 2),
                       5, (0, 0, 255), -1)
        ok, encoded = cv2.imencode('.jpg', overlay)
        if not ok:
            raise ValueError('Could not encode the debug image.')
        return CompressedImage(
            header=deepcopy(header), format='jpeg', data=encoded.tobytes(),
        )
```

`get_debug_image()`는 선택 사항이다. 이 메서드가 없어도 검출·조향은 동작한다.
구현하면 별도 publisher 없이 활성 검출기의 영상이 `/driving/debug/roi/compressed`로 나온다.
토픽 이름에 `roi`가 있어도 표시 영상의 크기나 내용이 basic ROI로 제한되지는 않는다.

JPEG 데이터는 실제로 `.jpg`로 인코딩하고 `format='jpeg'`로 표시한다.
입력 header를 복사하여 어떤 입력 프레임인지 알 수 있게 한다. 이전 영상이 남지 않도록
매 `detect()` 시작 시 디버깅 상태를 지운다. 발행 순서는 검출·조향 성공 및 최신성 검사를
통과한 뒤이며, 디버깅 생성만 실패하면 경고를 남기고 정상 조향 결과는 유지한다.

현재 자동 호출 대상은 검출기 하나다. 조향기가 그린 예상 경로를 별도로 반환하거나
여러 종류의 영상을 서로 다른 토픽으로 발행하는 기능은 추가 연결이 필요하다.

### 5.4 조향기

파일: `driving/steering/team_p.py`

```python
# file: driving/steering/team_p.py
from math import isfinite

from geometry_msgs.msg import Twist

from driving.team_contract import LaneObservation, TEAM_LANE_ERROR


class TeamP:
    input_type = TEAM_LANE_ERROR

    def __init__(self):
        self.speed = 0.1
        self.kp = 0.6
        self.max_angular = 0.8

    def compute(self, result: LaneObservation) -> Twist:
        if not isinstance(result, LaneObservation):
            raise TypeError('Expected LaneObservation.')
        twist = Twist()
        if not result.found:
            return twist  # 이 예제 조합은 미검출 시 정지한다.
        if not isfinite(result.error) or not -1.0 <= result.error <= 1.0:
            raise ValueError('Invalid normalized lane error.')
        twist.linear.x = float(self.speed)
        twist.angular.z = float(max(
            -self.max_angular, min(self.max_angular, self.kp * result.error),
        ))
        return twist
```

일반적인 이동 로봇 좌표 규약에서 `linear.x`는 전진 속도(m/s), `angular.z`는
회전 속도(rad/s)이고 양의 z 회전은 왼쪽 회전에 해당한다.
([ROS 좌표·단위 규약 REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst))
카메라가 좌우 반전되어 있거나
기존 오차 정의가 반대라면 이식 시 부호를 확인한다.

기존 PID를 이식할 때는 과거 오차·적분값을 객체에 보관하고 실제 처리 간격을 사용한다.
이 파이프라인은 바쁜 동안 중간 프레임을 버리므로 카메라 FPS가 곧 제어 호출 주기는 아니다.
알고리즘 선택 시 객체가 다시 만들어져 상태가 초기화된다는 점도 고려한다.

## 6. 알고리즘 등록과 선택

### 등록표의 세 부분을 함께 수정한다

5절의 파일을 추가했다면 `driving/registry.py`에 import를 추가한다.

```python
from driving.detection.team_detector import TeamDetector
from driving.steering.team_p import TeamP
```

기존 이름을 유지하면서 `Literal`에도 새 이름을 추가한다.

```python
Steering = Literal['basic_p', 'pure_pursuit', 'team_p']
Detection = Literal['basic_sobel', 'basic_sat', 'yolo', 'team_detector']
```

기존 등록표에 새 항목을 추가한다. 아래처럼 basic 항목도 유지한다.

```python
DETECTORS: Dict[Detection, Callable[[], LaneDetector[Any]]] = {
    'basic_sat': Basic_Sat.from_settings,
    'team_detector': TeamDetector,
}
CONTROLLERS: Dict[Steering, Callable[[], SteeringController[Any]]] = {
    'basic_p': Basic_P.from_settings,
    'team_p': TeamP,
}
```

등록표에는 객체가 아니라 **인자 없이 호출할 수 있는 생성 함수**를 넣는다.
`TeamDetector`는 가능하지만 `TeamDetector()`를 넣으면 이미 생성된 객체이므로 맞지 않는다.
설정 인자가 필요하면 인자 없는 `create_team_detector()` 함수를 작성해 등록한다.

생성 시 모든 팀의 선택적 ML 라이브러리를 무조건 import하면 basic만 쓰는 환경에서도
노드 시작이 실패할 수 있다. 선택적 의존성의 import 위치와 설치 안내를 함께 설계한다.
다만 생성 함수 안으로 옮겼다고 무거운 모델 로딩이 비동기가 되지는 않는다.
현재 생성 함수는 ROS 콜백에서 실행되며, 느린 초기화가 필요한 경우 공통 실행 구조에 대한
별도 설계가 필요하다.

### 같은 결과 계약이면 실행 중 개별 변경 가능

다음은 namespace 없는 기본 노드에 선택 문자열을 보내는 형식이다.

```bash
ros2 topic pub --once /driving/detection std_msgs/msg/String "{data: 'basic_sat'}"
ros2 topic pub --once /driving/steering std_msgs/msg/String "{data: 'basic_p'}"
```

처음부터 basic 조합이라면 위 요청은 같은 이름이므로 변경하지 않는다.
새 알고리즘 이름으로 바꾸려면 이름이 실제 등록되어 있어야 하고, **현재 상대 알고리즘**과
결과 계약이 맞아야 한다. 문자열 토픽에는 응답 메시지가 없으므로 노드의 선택 성공/거부 로그를 확인한다.
`--once` 옵션은 [Galactic의 ros2 topic pub 구현](https://github.com/ros2/ros2cli/blob/galactic/ros2topic/ros2topic/verb/pub.py)에 있는 단발 발행 옵션이다.

### 새 계약의 검출·조향 쌍은 동시에 시작해야 한다

5절 예제는 basic과 결과 계약이 다르다. basic_p가 활성화된 상태에서 `team_detector`만
요청하면 거부되고, basic_sat 상태에서 `team_p`만 요청해도 거부된다.
두 토픽을 연달아 보내도 이 제한을 해결하지 못한다.

현재는 테스트 브랜치에서 `registry.py`의 두 기본값을 함께 지정하고 노드를 재시작하는
방식으로 새 쌍을 실행한다.

```python
DEFAULT_DETECTION: Detection = 'team_detector'
DEFAULT_STEERING: Steering = 'team_p'
```

이때 5절의 핵심 검출 함수를 먼저 구현해야 한다. 기본값은 ROS 파라미터가 아니므로
`ros2 param set`으로 알고리즘을 고를 수는 없다. 두 이름을 원자적으로 교체하는 외부
서비스/토픽도 현재 없다. 내부 `_replace_algorithms()`가 있다는 사실과 외부 API 제공은 다르다.

PR에 기본 조합 변경까지 포함할지는 팀과 정한다. 알고리즘만 추가하는 PR이라면 임시로
바꾼 기본값을 복구하고, 테스트에서 `create_algorithms('team_detector', 'team_p')`로
생성해 검증한 방법을 적을 수 있다. 추가만 했는데 모든 팀원의 기본 주행이 바뀌지 않게 한다.

## 7. 파라미터·의존성·리소스

### 현재 ROS 파라미터

| 이름 | 기본값 | 조건 |
| --- | ---: | --- |
| `basic_p.left.normal_angular_divisor` | 1.7 | 양수인 유한 double |
| `basic_p.right.normal_angular_divisor` | 1.3 | 양수인 유한 double |

실행 중 변경 예시다. `2`가 아닌 `2.0`처럼 실수로 전달한다.

```bash
ros2 param set /driving basic_p.left.normal_angular_divisor 2.0
ros2 param get /driving basic_p.left.normal_angular_divisor
```

시작 값도 `--ros-args -p basic_p.left.normal_angular_divisor:=2.0` 또는
ROS 파라미터 YAML로 지정할 수 있다. 실행 중 변경값은 재시작 후 자동 저장되지 않는다.

이 값은 현재 `Basic_P`에만 연결되어 있다. 다른 조향 클래스나 Basic_P를 상속한 별도
구현의 설정을 자동으로 바꾸지 않는다. 예제 `TeamP.kp` 역시 단순 Python 속성이다.
새 파라미터가 필요하면 선언·검증·실제 객체 반영까지 추가해야 한다.

참고할 구현은 `basic_p_parameters.py`다. Galactic의 검증 콜백은 성공 여부를 반환할 뿐,
최종 확정 전에 실행 객체를 변경하지 않는다. 이후 콜백에서 실제 저장된 값을 읽어
새 스냅샷을 적용하고 이전 설정의 프레임 결과를 폐기하는 방식이다.
새 구현에서는 시작 override, 잘못된 값, 다른 알고리즘을 쓰는 동안의 변경,
재선택 시 값 유지까지 정의한다. 현재 기능 추가 범위와 무관한 계수까지 한 번에 노출하지 않는다.

### 의존성과 파일 경로

- ROS/시스템 실행 의존성은 `package.xml`에 선언한다. 현재 OpenCV·NumPy·메시지 의존성은
  이미 선언되어 있으므로 중복 설치 방식을 만들기 전에 확인한다.
- 새 Python 라이브러리는 Python 3.8과 대상 장치에서 쓸 수 있는 버전을 선택한다.
  설치 방법과 버전을 남기고 개인 개발 환경의 모든 패키지를 그대로 강제하지 않는다.
- `setup.py`는 `driving.*` 하위 패키지를 찾는다. 새 Python 하위 디렉터리에는
  `__init__.py`를 두고 설치 후 import를 확인한다.
- 모델·YAML·보정 파일은 Python 소스처럼 자동 설치되지 않는다. 필요한 파일을
  `setup.py`의 `data_files` 등에 명시하고 설치 위치에서 읽도록 한다.
- `/home/내이름/...`나 현재 실행 디렉터리에 의존하는 경로를 없앤다.
  ROS 패키지 share 경로를 찾는 API를 쓰면 해당 의존성도 선언한다.
- 큰 모델이나 bag은 소스와 분리하고 받는 위치, 버전/해시, 필요한 입력 조건을 문서화한다.
  코드만 clone한 팀원이 무엇을 추가로 준비해야 하는지 알 수 있어야 한다.

## 8. 빌드와 검증

### 8.1 검증할 동작

| 영역 | 확인할 내용 |
| --- | --- |
| 정상 입력 | 좌·중앙·우 상황에서 결과 좌표/단위와 회전 부호가 의도대로인지 |
| 정상 미검출 | 자신의 미검출 표현과 조향 정책이 일치하는지 |
| 잘못된 입력 | 빈 payload, 손상 JPEG, 지원하지 않는 크기를 처리하는지 |
| 결과 범위 | NaN/Inf·잘못된 좌표를 거부하고 유한 Twist를 반환하는지 |
| 등록 | 실제 이름으로 생성되며 호환 조합은 수락, 비호환은 거부되는지 |
| 전환/지연 | 상태 초기화, 이전 작업 폐기, 최신 프레임 처리, timeout 동작 |
| 디버깅 | JPEG 디코딩, 크기·표시 위치·입력 header, 미검출 화면, 오류 격리 |
| 파라미터 추가 시 | 시작 값, 실행 중 변경, 거부된 값이 동작을 바꾸지 않는지 |
| 설치 | 소스 디렉터리 밖에서도 import·설정/모델 경로가 동작하는지 |

이미 기존 테스트가 확인하는 항목은 활용한다. 새 테스트는 새 로직의 중요한 행동과
회귀 위험에 집중한다. 실제 검출기의 평가는 대표 영상과 정답/기대 결과를 이용한다.

### 8.2 예제 어댑터를 검사하는 테스트

파일: `test/test_team_adapter.py`. 이 예제는 **이식한 핵심 검출기의 성능**이 아니라
그 주변의 좌표 변환·조향·JPEG 연결을 검사한다. 핵심 함수는 지정한 좌표를 반환하도록
대체하므로, 실제 검출 함수에는 별도의 영상 기반 테스트를 작성한다.

```python
# file: test/test_team_adapter.py
from unittest.mock import patch

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import CompressedImage

from driving.detection.team_detector import TeamDetector
from driving.steering.team_p import TeamP


@pytest.mark.parametrize('center, expected_error', [
    (40.0, 0.5), (80.0, 0.0), (120.0, -0.5), (None, 0.0),
])
def test_adapter_and_steering(center, expected_error):
    ok, encoded = cv2.imencode('.jpg', np.zeros((120, 160, 3), dtype=np.uint8))
    assert ok
    msg = CompressedImage(format='jpeg', data=encoded.tobytes())
    msg.header.stamp.sec = 123
    msg.header.frame_id = 'camera_optical_frame'
    detector = TeamDetector()
    with patch('driving.detection.team_detector.find_center_x', return_value=center):
        result = detector.detect(msg)
    assert result.found == (center is not None)
    assert result.error == pytest.approx(expected_error)
    twist = TeamP().compute(result)
    assert twist.linear.x == pytest.approx(0.0 if center is None else 0.1)
    assert twist.angular.z == pytest.approx(0.6 * expected_error)

    debug = detector.get_debug_image()
    assert debug.header == msg.header
    assert debug.format == 'jpeg'
    image = cv2.imdecode(np.frombuffer(debug.data, np.uint8), cv2.IMREAD_COLOR)
    assert image.shape == (120, 160, 3)
    if center is not None:
        # JPEG의 손실 압축을 허용한다.
        np.testing.assert_allclose(image[60, int(center)], [0, 0, 255], atol=40)
```

### 8.3 로컬 검사와 빌드

먼저 소스 테스트를 실행한다. ROS 메시지를 import하므로 ROS setup을 source한 상태여야 한다.
domain 94와 localhost로 테스트 통신을 분리한다.

```bash
source /opt/ros/galactic/setup.bash
cd ~/turtlebot3_ws/src/driving
PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 \
  python3 -m pytest test -q
git diff --check
```

그다음 workspace에서 빌드한다. 의존성 선언을 바꿨다면 먼저 2절의 rosdep 절차를 수행한다.

```bash
cd ~/turtlebot3_ws
PYTHONNOUSERSITE=1 colcon build --packages-select driving --symlink-install
source install/local_setup.bash
PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 \
  colcon test --packages-select driving --return-code-on-test-failure
colcon test-result --test-result-base build/driving --verbose
```

`PYTHONNOUSERSITE=1`은 개인 user-site의 setuptools 등과 ROS 빌드 환경의 충돌을 피하려고
사용한다. 필요한 라이브러리가 user-site에만 있다면 숨겨지므로 팀의 설치 방식에 맞춰
의존성을 준비한다. 로컬에서 성공한 명령과 실제 사용한 Python/ROS 버전을 PR에 적는다.

새 `.py`는 symlink 설치에서 바로 반영될 수 있어도 등록·의존성·리소스 변경은 재빌드를
기준으로 확인한다. 이미 실행 중인 프로세스의 모듈은 자동 교체되지 않으므로 노드도 재시작한다.
설치 확인은 패키지 소스 밖에서 수행해야 소스 경로가 잘못된 설치를 가리지 않는다.

```bash
cd /tmp
python3 -c "import driving; print(driving.__file__)"
# 예제 파일을 실제 추가했다면 아래 import도 확인한다.
python3 -c "from driving.detection.team_detector import TeamDetector; print(TeamDetector)"
```

### 8.4 실제 ROS 연결을 미리보기로 확인

처음 통합 검사는 다음처럼 별도 domain/namespace에서 하고 속도 출력은 미리보기 토픽으로
보낸다. 아래 명령 자체가 카메라 입력을 만들지는 않는다. 합성 영상 발행기나 준비한 bag을
같은 domain에서 실행해 입력을 공급한다.

터미널 A:

```bash
source /opt/ros/galactic/setup.bash
source ~/turtlebot3_ws/install/local_setup.bash
export ROS_DOMAIN_ID=94
export ROS_LOCALHOST_ONLY=1
ros2 run driving driving --ros-args \
  -r __ns:=/driving_preview \
  -r camera:=/camera \
  -r camera_raw:=/camera_raw \
  -r camera_raw_16_9:=/camera_raw_16_9 \
  -r cmd_vel:=cmd_vel_preview
```

터미널 B에서도 같은 ROS setup, workspace setup, 두 환경변수를 적용한다.

```bash
ros2 node info /driving_preview/driving
ros2 topic echo /driving_preview/cmd_vel_preview
```

이 실행의 선택 토픽은 `/driving_preview/driving/detection`,
`/driving_preview/driving/steering`이고 디버깅 영상은
`/driving_preview/driving/debug/roi/compressed`다.
파라미터 명령의 노드 경로도 `/driving_preview/driving`으로 바꾼다.
이미지 뷰어의 구독 QoS는 **Best Effort**로 설정한다.

로컬 bag 입력 예시다. 실제 bag 경로로 바꾸고 터미널 B와 같은 환경을 적용한다.

```bash
ros2 bag play /path/to/test_bag --topics /camera
```

bag에 `/camera_raw`만 들어 있다면 해당 토픽을 재생하고 검출기의 `input_topic`도 맞춘다.
`--topics`는 [Galactic의 ros2 bag play 구현](https://github.com/ros2/rosbag2/blob/galactic/ros2bag/ros2bag/verb/play.py)에서 지원하는 토픽 필터다.
`ROS_LOCALHOST_ONLY=1`은 다른 컴퓨터의 카메라와 통신하는 설정이 아니므로 로컬 검사와
다른 PC/실차 연결 검사를 구분한다.

동작 중 입력을 중단해 속도가 0으로 바뀌는지, 잘못된 영상을 보낸 뒤 정상 영상으로
회복하는지 확인한다. 영상 처리·디버깅 압축까지 포함한 시간이 0.5초를 넘으면 현재 노드는
그 결과를 발행하지 않는다. 이 제한을 피하려고 임의로 timeout을 늘리기 전에 처리 시간을 측정한다.

현재는 worker를 강제로 종료하지 못한다. 영구 대기·긴 모델 초기화·GIL 점유에 대한
하드 실시간 보호가 있는 구조는 아니다. 실제 카메라나 실차 검증을 하지 않았다면 PR에
미실시로 적고, 합성 영상 테스트 통과를 실차 성능 검증으로 표현하지 않는다.

## 9. 커밋·push·GitHub PR

기본 흐름은 **새 브랜치에서 개발 → 검증 → 커밋 → 브랜치 push → GitHub에서 PR 생성**이다.
아래는 팀 저장소에 쓰기 권한이 있는 기여자를 기준으로 한다.

### 9.1 변경 검토와 커밋

패키지 저장소에서 작업 브랜치를 확인한다.

```bash
cd ~/turtlebot3_ws/src/driving
git branch --show-current
git status --short
git diff
git diff --check
```

원하는 파일만 stage한다. 다음은 5절 예제를 실제 이식한 경우의 예시이며 자신이 만든
파일에 맞춰 경로를 바꾼다. `git diff`에는 새 untracked 파일 내용이 안 보이므로
stage한 뒤의 diff도 확인한다.

```bash
git add driving/team_contract.py driving/detection/team_core.py \
  driving/detection/team_detector.py driving/steering/team_p.py \
  driving/registry.py test/test_team_adapter.py
# 의존성이나 설명을 수정했다면 해당 파일도 명시해서 추가한다.
git diff --cached --stat
git diff --cached
git diff --cached --check
git commit -m "feat: Add team lane detector and steering controller"
```

커밋 제목 예:

- `feat: Add sobel lane detector`
- `fix: Reject invalid lane coordinates`
- `test: Cover lane loss recovery`
- `docs: Explain detector integration`

한 PR에는 설명 가능한 하나의 목적을 담는다. 관련 없는 튜닝·포맷 변경·모델 교체를
섞지 않는다. `build/`, `install/`, `log/`, 가상 환경, bag, 개인 설정이 들어가지 않았는지 확인한다.

### 9.2 작업 브랜치를 push

```bash
git push -u origin feat/team-lane-detector
```

실제 브랜치 이름이 다르면 바꾼다. `-u`로 upstream을 설정하면 같은 브랜치의 후속 커밋은
`git push`로 보낼 수 있다. 이 명령은 작업 브랜치를 올리는 것이며 `main`으로의 병합은 PR에서 진행한다.

### 9.3 GitHub에서 PR 생성

1. 팀 저장소의 GitHub 페이지를 연다.
2. push한 브랜치의 **Compare & pull request**를 선택한다.
   배너가 없으면 **Pull requests → New pull request**에서 브랜치를 고른다.
3. **base는 `main`**, **compare는 자신의 작업 브랜치**인지 확인한다.
4. `Files changed`에서 의도한 파일만 들어 있는지 확인한다.
5. 변경 목적·실행 방법·검증 결과를 작성하고 리뷰할 팀원을 지정한다.
6. 준비가 끝났으면 PR을 생성한다. 아직 검증 중이면 Draft로 상태를 명확히 한다.

base는 변경을 받을 브랜치, compare는 변경을 만든 브랜치다.
PR 작성과 브랜치 선택 방식은 [GitHub 공식 안내](https://docs.github.com/en/pull-requests/how-tos/create-pull-requests/creating-a-pull-request)에서도 확인할 수 있다.

### 9.4 PR 본문에 넣을 내용

다음 템플릿을 복사해 채운다. 설명만 길게 쓰기보다 리뷰어가 같은 결과를 재현할 수 있도록
설정과 명령, 기대 결과를 구체적으로 적는다.

```markdown
## 변경 목적
- 어떤 상황을 해결하는가:
- 추가/수정한 검출기·조향기 이름:

## 결과 계약과 연결
- 입력 토픽·메시지·해상도·보정/색상 조건:
- output_type / input_type:
- 결과 자료형·좌표계·단위·부호:
- 미검출 표현과 그때의 조향 정책:
- 함께 사용할 알고리즘 / 기본값 변경 여부:

## 설정과 의존성
- 새 파라미터·기본값·유효 범위:
- 추가 라이브러리·버전·모델/보정 파일 준비 방법:
- 디버깅 영상에서 확인할 내용:

## 재현과 검증
- 환경: OS / ROS 배포판 / Python / 주요 라이브러리
- 실행·선택·테스트 명령:
- 테스트 결과 및 처리 시간:
- 정상/미검출/손상 입력/전환 확인 결과:
- 실제 Galactic 실행: 수행 / 미수행
- 실제 카메라·bag·실차 확인: 각각 수행 / 미수행

## 남은 제한점
- 미지원 입력, 알려진 실패 상황, 추가 확인이 필요한 부분:
```

제출 전 확인:

- [ ] 새 작업 브랜치에서 커밋했고 base/compare를 확인했다.
- [ ] 기존 기본 동작을 유지했거나 변경 이유를 명시했다.
- [ ] Literal과 등록표, 입력/결과 계약이 일치한다.
- [ ] 정상·미검출·손상 입력과 조향 부호를 확인했다.
- [ ] 디버깅을 제공한다면 실제 JPEG·입력 header·현재 프레임을 반환한다.
- [ ] 의존성과 리소스 경로를 다른 PC에서도 재현할 수 있다.
- [ ] 관련 테스트와 빌드 결과, 미검증 환경을 정확히 적었다.
- [ ] 개인 파일·빌드 산출물·불필요한 대형 파일이 없다.

### 9.5 리뷰 수정, 최신 main 반영, 병합 후 정리

리뷰 수정은 **같은 작업 브랜치**에서 커밋하고 push한다. 기존 PR에 후속 커밋이 반영되므로
같은 변경에 새 PR을 만들 필요는 없다.

```bash
git add 수정한_파일_경로
git commit -m "fix: Address review feedback"
git push
```

개발 중 `main`이 바뀌어 반영해야 한다면 깨끗한 작업 트리에서 현재 작업 브랜치로 merge한다.

```bash
git fetch origin
git merge origin/main
```

충돌이 나면 `git status`로 파일을 확인하고 양쪽 변경의 의도를 반영해 수정한다.
충돌 표시를 제거한 뒤 해당 파일을 `git add`하고 `git commit`으로 merge를 마친다.
즉시 해결하기 어렵다면 `git merge --abort`로 merge 전 상태로 돌아갈 수 있다.
충돌 해결로 바뀐 동작을 다시 검사한 뒤 push한다. 공유 브랜치의 이력을 강제 push로 덮어쓰지 않는다.

리뷰와 검증 후 팀의 병합 담당자가 GitHub에서 `main`으로 병합한다.
병합된 것을 확인한 뒤 로컬 `main`을 갱신한다.

```bash
git switch main
git pull --ff-only origin main
git branch -d feat/team-lane-detector
```

Squash merge를 사용했다면 `-d`가 아직 병합되지 않았다고 거부할 수 있다.
PR의 병합 여부와 브랜치에 남은 작업을 먼저 확인하고 자동으로 강제 삭제하지 않는다.

### 팀 저장소 쓰기 권한이 없다면

GitHub에서 fork한 뒤 **fork를 `origin`, 팀 저장소를 `upstream`**으로 둔다.
팀 `upstream/main` 기준으로 작업 브랜치를 만들고 자신의 fork에 push한 다음,
팀 저장소의 `main`을 base로 하는 PR을 연다. 직접 push 권한이 있는 팀원은 앞의 기본 흐름만 쓰면 된다.
([GitHub의 fork PR 안내](https://docs.github.com/en/pull-requests/how-tos/create-pull-requests/creating-a-pull-request))

## 10. 자주 발생하는 문제

| 증상 | 먼저 확인할 내용 |
| --- | --- |
| 이름을 보냈는데 선택이 안 됨 | Literal·등록표 누락, 미구현 이름, 현재 상대와 결과 계약 불일치, 로그 |
| 새 검출·조향을 순서대로 보내도 둘 다 거부됨 | 새 결과 계약이면 두 기본값을 함께 맞춰 시작해야 함. 외부 동시 선택 API 없음 |
| 영상 콜백은 오는데 처리가 안 됨 | 활성 검출기의 input_topic, namespace/remap, 실제 CompressedImage 타입 |
| debug 토픽은 있는데 화면이 안 나옴 | 활성 검출기에 get_debug_image가 있는지, 유효 결과가 나오는지, 구독 QoS Best Effort |
| 미검출인데 로봇이 계속 회전함 | basic의 None 극단값 정책인지, 자신의 조향 정책이 무엇인지 |
| angular.z 부호가 반대임 | ROI/전체 좌표 혼동, 영상 좌우 반전, 오차 부호, rad/s와 다른 단위 혼동 |
| 결과를 계산했는데 0 Twist만 나옴 | 입력 나이 0.5초 초과, 알고리즘 전환 후 이전 결과, 예외, NaN/Inf |
| 새 ROS 파라미터를 넣었는데 동작이 안 바뀜 | 선언만 했는지, 실제 알고리즘 객체까지 적용했는지, 노드 경로가 맞는지 |
| 내 PC에서만 import/모델 로드 성공 | 의존성 미선언, 절대 경로, 리소스 설치 누락, source한 overlay |
| 노드 재시작 후 설정이 사라짐 | 실행 중 param set은 자동 저장되지 않음. 시작 설정을 별도로 지정 |

현재 제한점 전체는 [KNOWN_LIMITATIONS.md](KNOWN_LIMITATIONS.md)에 기록되어 있다.
기여하면서 해결한 제한점이나 새로 생긴 입력 조건은 해당 문서도 함께 갱신한다.
