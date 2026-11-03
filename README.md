# driving

팀원들이 각자 개발한 차선 검출과 조향 알고리즘을 하나의 ROS 2 주행 노드에서
연결해 사용하는 패키지다. 카메라 영상을 구독하고, 선택한 검출·조향 코드로 계산한
속도를 `cmd_vel`에 발행한다. 디버깅 영상 발행과 ROS 파라미터 조절도 지원한다.

공통 노드는 ROS 통신과 실행을 관리하고, 전처리·검출·조향 계산은 각 알고리즘에 둔다.
**팀원이 코드를 읽고 직접 수정하기 쉬운 구조**를 우선한다.

```text
카메라 영상 (CompressedImage)
    → detector.detect(msg): 디코딩·전처리·차선 검출
    → controller.compute(result): Twist 계산
    → driving 노드: cmd_vel 발행

detector.get_debug_image() (선택 기능)
    → driving 노드: JPEG 디버깅 영상 발행
    → driving_debug 노드: OpenCV 창 표시
```

## 대상 환경

| 항목 | 기준 |
| --- | --- |
| OS | Ubuntu 20.04 |
| ROS 2 | Galactic |
| Python | 3.8 |
| 주요 의존성 | rclpy, ROS 메시지 패키지, OpenCV, NumPy |

의존성은 [package.xml](package.xml)에 선언한다. 새 코드는 Python 3.8과 Galactic에서
사용 가능한 문법과 API를 기준으로 작성한다.
현재 자동 검사는 Humble / Python 3.10에서 수행했으며, 실제 Galactic 런타임 검증은 남아 있다.

## 빌드와 실행

### 1. 빌드

ROS 2 Galactic, `rosdep`, `colcon`이 준비된 환경에서 패키지를 워크스페이스의
`src/driving`에 둔다. 아래의 `~/turtlebot3_ws`는 자신의 워크스페이스 경로로 바꾼다.

```bash
source /opt/ros/galactic/setup.bash
cd ~/turtlebot3_ws
rosdep install --from-paths src/driving --ignore-src -r -y --rosdistro galactic
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select driving
source install/local_setup.bash
```

`PYTHONNOUSERSITE=1`은 빌드할 때 개인 user-site의 setuptools 등과 시스템 패키지가
섞이는 것을 피하기 위한 설정이다. user-site에만 설치한 라이브러리도 제외하므로
필요한 의존성은 팀의 개발 환경에 맞춰 준비한다.

### 2. 영상 입력과 주행 노드

카메라 발행기나 bag 재생은 별도로 실행한다. 기본 검출기 `basic_sat`은
**`/camera`의 640×480 압축 영상**을 사용한다. `/camera_raw_16_9`만 재생해도
`/camera`가 자동으로 만들어지지는 않는다.

```bash
source /opt/ros/galactic/setup.bash
source ~/turtlebot3_ws/install/local_setup.bash
ros2 run driving driving
```

카메라 발행기, 주행 노드, 뷰어가 서로 통신할 수 있도록 `ROS_DOMAIN_ID`와 네트워크
설정을 맞춘다. 입력과 속도 출력을 분리해서 시험하는 방법은
[CONTRIBUTING.md의 빌드와 검증 절차](CONTRIBUTING.md#8-빌드와-검증)를 참고한다.

### 3. 디버깅 뷰어

GUI가 있는 로컬 환경에서 별도 터미널을 열어 실행한다.

```bash
source /opt/ros/galactic/setup.bash
source ~/turtlebot3_ws/install/local_setup.bash
ros2 run driving driving_debug
```

`camera`, `camera_raw`, `camera_raw_16_9`, `driving/debug/roi/compressed`를 각각
OpenCV 창에 표시한다. 정상 영상이 처음 도착한 토픽부터 창이 열린다.
영상 창에 포커스를 둔 상태에서 `q` 또는 `Esc`, 터미널에서는 `Ctrl+C`로 종료한다.
뷰어를 종료해도 별도 실행 중인 주행 노드는 계속 동작한다.

## 토픽

아래는 namespace를 지정하지 않았을 때의 이름이다.
소스에서는 상대 이름을 사용하므로 namespace와 remap을 적용할 수 있다.

| 토픽 | 메시지 타입 | driving 기준 역할 |
| --- | --- | --- |
| `/camera` | `sensor_msgs/msg/CompressedImage` | 영상 구독, basic_sat의 기본 입력 |
| `/camera_raw` | `sensor_msgs/msg/CompressedImage` | 영상 구독 |
| `/camera_raw_16_9` | `sensor_msgs/msg/CompressedImage` | 영상 구독 |
| `/driving/detection` | `std_msgs/msg/String` | 검출 알고리즘 선택 |
| `/driving/steering` | `std_msgs/msg/String` | 조향 알고리즘 선택 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 속도 명령 발행 |
| `/driving/debug/roi/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG 디버깅 영상 발행 |

영상 구독과 디버깅 발행은 sensor-data QoS(Best Effort, depth 5)를 사용한다.
선택 토픽과 `cmd_vel`은 depth 10을 사용한다.
토픽 이름에 `raw`가 있어도 메시지 타입은 압축 영상이다.

세 카메라 입력을 모두 구독하지만, 실제 계산에는 활성 검출기의 `input_topic`
하나만 사용한다. 여러 카메라 영상을 동기화하거나 합치는 기능은 없다.

## 알고리즘 선택과 파라미터

현재 구현·등록된 기본 조합은 **`basic_sat` 검출기 + `basic_p` 조향기**다.
기본 선택값과 등록표는 [registry.py](driving/registry.py)에 있다.

```bash
ros2 topic pub --once /driving/detection std_msgs/msg/String "{data: basic_sat}"
ros2 topic pub --once /driving/steering std_msgs/msg/String "{data: basic_p}"
```

등록된 다른 이름으로 바꾸면 실행 중 알고리즘을 교체한다. 현재 선택과 같은 이름은
아무 작업도 하지 않으며, 미등록 이름이나 생성 실패 시 기존 선택을 유지한다.
`basic_sobel`, `yolo`, `pure_pursuit`는 타입 힌트에만 있고 아직 선택할 수 없다.

두 선택 토픽은 독립적이다. 검출 결과의 자료형·좌표·단위·미검출 의미는 검출기와
조향기가 맞춰야 한다. 결과 형식이 다른 조합을 함께 추가한다면 두 기본값을 맞추고
노드를 재시작해 시험한다. 노드가 알고리즘 사이의 의미상 호환성을 검사하지는 않는다.

현재 조절 가능한 ROS 파라미터는 다음 두 개다.

| 파라미터 | 기본값 | 조건 |
| --- | --- | --- |
| `basic_p.left.normal_angular_divisor` | `1.7` | 양수인 유한 double |
| `basic_p.right.normal_angular_divisor` | `1.3` | 양수인 유한 double |

```bash
ros2 param get /driving basic_p.left.normal_angular_divisor
ros2 param set /driving basic_p.left.normal_angular_divisor 2.0
```

현재 선택한 좌우 모드에 해당하는 분모가 조향 계산에 적용된다. 실행 중 변경한 값은
재시작 후 자동 보존되지 않으므로 시작 값은 `--ros-args -p` 또는 YAML로 지정한다.
좌우 모드는 [basic.py](driving/basic.py)의 `SETTINGS`에서 정하며, 현재 기본값은 왼쪽이다.
좌우 모드를 실행 중 외부 토픽이나 파라미터로 바꾸는 기능은 없다.

## 코드 구조와 알고리즘 추가

| 파일 / 디렉터리 | 역할 |
| --- | --- |
| [driving/driving.py](driving/driving.py) | 영상 구독, 알고리즘 선택, 파라미터, 계산 호출과 결과 발행 |
| [driving/detection/](driving/detection/) | 디코딩·ROI·보정·색 변환을 포함한 검출 코드 |
| [driving/steering/](driving/steering/) | 검출 결과로 Twist를 계산하는 조향 코드 |
| [driving/registry.py](driving/registry.py) | 알고리즘 이름과 인자 없이 호출하는 생성 함수 등록 |
| [driving/basic.py](driving/basic.py) | basic 조합 전용 결과 타입 힌트와 좌우 설정 |
| [driving/debug_viewer.py](driving/debug_viewer.py) | 입력 영상과 디버깅 JPEG를 표시하는 보조 노드 |
| [test/](test/) | 알고리즘·ROS 통신·파라미터·뷰어 검사 |

다른 환경에서 개발한 알고리즘을 가져오는 기본 절차는 다음과 같다.

1. 검출 코드를 `driving/detection/`에 넣고 `input_topic`과 `detect(msg)`를 제공한다.
   입력 한 장을 디코딩하고 전처리한 뒤 검출 결과를 반환한다.
2. 조향 코드를 `driving/steering/`에 넣고 `compute(result)`에서 `Twist`를 반환한다.
   검출기와 결과의 자료형·단위·미검출 표현을 맞춘다.
3. `registry.py`의 `DETECTORS` / `CONTROLLERS`에 생성 함수를 등록하고
   `Detection` / `Steering`의 `Literal`도 갱신한다. 별도 상속은 요구하지 않는다.
4. 디버깅 영상이 필요하면 검출기에 `get_debug_image()`를 추가한다.
   현재 프레임의 JPEG `CompressedImage` 또는 `None`을 반환하며 입력 header를 보존한다.
5. ROS 파라미터가 필요하면 노드의 선언·검증·적용 부분을 연결하고,
   추가 의존성과 리소스의 설치도 함께 반영한다.

팀 알고리즘에는 별도 ROS 노드·구독·publisher·spin·영상 반복 루프를 넣지 않는다.
한 worker가 계산하므로 처리 함수는 유한 시간에 반환해야 한다.
파라미터 검증 콜백에서 실행 중인 알고리즘 객체를 직접 바꾸지 않고,
확정된 설정을 이후 콜백에서 적용한다.

`None`, 폭 320의 ROI 좌표, `lane_mode`는 basic 전용 규칙이다.
다른 알고리즘이 같은 결과 형식을 쓸 필요는 없다.
basic의 기존 미검출 정책과 확장 예정 설정·교차로 주석은 유지한다.
실제 이식 예제와 등록·검증 절차는 [CONTRIBUTING.md](CONTRIBUTING.md)에 있다.

## 현재 동작과 제한점

- basic은 원근변환 후 좌우 중 한쪽의 320×120 ROI를 사용한다.
  ROI 안에서 조건을 통과한 여러 윤곽의 중심을 평균하므로 반사광이나 다른 선이
  검출 결과에 포함될 수 있다. 한쪽 영역 선택만으로 차선 하나의 검출을 보장하지 않는다.
- basic의 정상 미검출은 조향기에서 극단값으로 처리한다. 디버깅 영상은 검출기의
  원래 결과를 그리므로 **미검출이면 점이 없고, 조향에 사용한 대체 좌표도 표시되지 않는다.**
- 입력 손상·처리 예외·잘못된 Twist·입력 중단은 0 Twist로 처리한다.
  대기 영상은 최신 한 장만 유지하며, 감시 주기는 0.05초, 수신 시각 기준 입력 나이
  제한은 0.5초다. 선택·설정 변경 전 결과와 오래된 결과는 발행하지 않는다.
- 뷰어의 각 창은 독립적으로 갱신된다. 영상이 끊기면 마지막 화면이 남을 수 있고,
  GUI 세션과 OpenCV HighGUI가 필요하다.
- 기존 자동 검사 34개는 Humble / Python 3.10에서 통과했다.
  테스트 통과가 실제 Galactic 동작이나 실차 주행·차선 검출 정확도를 보장하지는 않는다.

추가 제약과 미구현 기능은 [KNOWN_LIMITATIONS.md](KNOWN_LIMITATIONS.md)를 참고한다.

## 기여 방법과 문서

**새 작업 브랜치 → 코드 이식·문서 갱신·검증 → 커밋 → 브랜치 push → GitHub PR**
순서로 기여한다. `main`에 직접 push하지 않는다.
커밋 제목에는 `feat:`, `fix:`, `refactor:`, `docs:` 등을 사용한다.
PR에는 입력과 결과의 의미, 함께 사용할 알고리즘, 의존성, 실행 방법,
검증 환경·결과와 미검증 범위를 적는다. 빌드 산출물·bag·개인 환경·대형 모델은
소스와 섞어 커밋하지 않는다.

| 문서 | 내용 |
| --- | --- |
| [CONTRIBUTING.md](CONTRIBUTING.md) | 다른 환경의 알고리즘 이식 예제, 빌드·검증, 브랜치·PR 절차 |
| [AGENTS.md](AGENTS.md) | 저장소 작업 원칙과 코드 변경 시 지킬 규칙 |
| [KNOWN_LIMITATIONS.md](KNOWN_LIMITATIONS.md) | 현재 제약과 미구현 사항 |
| [driving 노드 동작 안내](docs/driving-flow.html) | 처리 흐름, FrameJob·FrameResult, 상황별 동작 설명 |
