# driving 저장소 작업 규칙

이 문서는 패키지 저장소 전체에 적용한다. 알고리즘 이식과 Git 기여 절차는
[CONTRIBUTING.md](CONTRIBUTING.md), 현재 한계는
[KNOWN_LIMITATIONS.md](KNOWN_LIMITATIONS.md)를 참고한다.
개인 PC에만 있는 인수인계 파일이나 절대 경로를 팀 작업의 전제 조건으로 삼지 않는다.

## 1. 우선할 원칙

- **사용자가 읽고 직접 수정하기 쉬운 단일 주행 통합 노드 `driving`**을 유지한다.
  필요한 동작을 직접 표현하고, 아직 필요하지 않은 범용 계층은 추가하지 않는다.
  영상 표시용 `driving_debug`는 로컬에서 따로 실행하는 보조 노드다.
- 대상은 **ROS 2 Galactic / Ubuntu 20.04 / Python 3.8**이다.
  `list[str]`, `X | None`, `match/case` 등 이후 버전에 의존하는 문법을 사용하지 않는다.
- 작업 전에 `git status --short`, 관련 소스와 테스트를 확인한다.
  기존 사용자 변경을 보존하고, 문서의 향후 과제를 현재 실행 지시로 간주하지 않는다.
- `basic_p`의 `roi_end_x`, `debug_window`, `debug_color`, `intersection_angular_divisor`와
  주석 처리된 교차로 분기는 사용자가 이후 확장할 예정이므로 보존한다.
  현재 사용하지 않는다는 이유만으로 정리하거나 삭제하지 않는다.
- 인터페이스·동작·설정이 달라지면 관련 문서를 함께 수정한다.

## 2. 코드의 역할

```text
CompressedImage → detector.detect(msg) → controller.compute(result) → Twist 발행
                            └─ get_debug_image()가 있으면 JPEG 영상도 발행
```

- `driving/driving.py`: 영상 구독, 선택 토픽, 파라미터, 계산 호출과 결과 발행.
- `driving/debug_viewer.py`: 세 카메라 입력과 디버깅 JPEG를 구독하는 OpenCV 뷰어.
  영상 표시와 키 처리는 main 스레드에서 수행하고, 주행 계산이나 속도 발행은 넣지 않는다.
- `driving/detection/`: 팀 검출 코드. 디코딩·ROI·보정·색 변환 등 전처리도 여기서 한다.
- `driving/steering/`: 팀 조향 코드. 계산한 `Twist`를 반환한다.
- `driving/registry.py`: 이름별 생성 함수와 기본 선택값.
- `driving/basic.py`: basic 조합의 결과 타입 힌트와 좌우 설정.
- 팀 알고리즘에 별도 ROS 노드·구독·publisher·spin·영상 반복 루프를 넣지 않는다.
  `detect()`는 입력 한 장을 처리하고, `compute()`는 결과 하나로 조향을 계산한다.
  영상 생성은 선택적 `get_debug_image()`, 화면 표시는 `driving_debug`가 맡는다.

## 3. 알고리즘 추가와 선택

- 검출기는 `input_topic`과 `detect(msg)`, 조향기는 `compute(result)`를 제공한다.
  별도 상속이나 결과 형식 식별자는 요구하지 않는다.
- `input_topic`은 `camera`, `camera_raw`, `camera_raw_16_9` 중 하나다.
  모두 `CompressedImage`이며 활성 검출기의 한 입력만 처리한다. 다중 영상 동기화는 없다.
- 검출 결과는 그대로 조향기에 전달한다. 자료형·단위·좌표·미검출 표현은 두 코드가 합의하고
  필요한 검사를 구현한다. 노드가 알고리즘끼리의 호환성을 검사하거나 결과를 변환하지 않는다.
- `None`, ROI320 좌표, `lane_mode`는 basic 전용이다. 다른 알고리즘에 강제하지 않는다.
  basic의 정상 미검출을 극단값으로 처리하는 기존 정책도 임의로 바꾸지 않는다.
- 새 클래스 import와 이름을 `registry.py`의 딕셔너리에 추가하고 `Literal`도 갱신한다.
  Literal은 편집기용 힌트이며 실제 선택 가능 여부는 딕셔너리로 판단한다.
  등록값은 인자 없이 호출할 수 있는 생성 함수다. 이름만 있는 미구현 알고리즘은 선택할 수 없다.
- 두 선택 토픽은 독립적이다. 한쪽만 변경하면 결과 의미가 다른 중간 조합이 생길 수 있다.
  다른 결과 형식의 쌍을 시험할 때는 두 기본값을 함께 맞춰 재시작한다.
  알고리즘을 추가한다는 이유만으로 기존 기본값을 바꾸지 않는다.

## 4. 실행·디버깅·파라미터

- 처리 함수는 유한 시간에 반환해야 한다. 생성 함수는 ROS 콜백에서 빠르게 반환해야 한다.
- 한 worker가 검출·조향·디버깅 생성을 처리한다. 대기 영상은 최신 한 장이며 호출 주기는
  카메라 FPS와 다를 수 있다. 알고리즘 안에서 ROS 발행이나 노드 상태 변경을 하지 않는다.
- worker를 유지하는 이유는 영상 처리 중에도 ROS 콜백에서 입력 중단을 감시하기 위해서다.
  선택/설정 변경 전 결과를 버리는 번호와 수신 시각 검사를 우회하지 않는다.
  현재 입력 나이 제한은 0.5초, 감시 주기는 0.05초다. 하드 실시간 보장은 없다.
- 손상 입력·처리 예외·잘못된 Twist는 0 Twist로 처리한다. 정상 미검출은 각 조합이 정한다.
  선택 변경은 두 객체를 다시 만들므로 객체 내부의 과거 오차·필터 상태가 초기화된다.
- 선택적인 `get_debug_image()`는 현재 프레임의 `CompressedImage` 또는 `None`을 반환한다.
  JPEG 인코딩, `format='jpeg'`, 입력 header 보존을 지킨다. 영상 내용·크기는 자유다.
  노드가 `/driving/debug/roi/compressed`로 발행하며 디버깅 실패는 정상 조향과 분리한다.
- `driving_debug`는 선택적으로 실행하며 `driving`과 별도 프로세스다. 카메라 세 입력과
  디버깅 출력을 CompressedImage/sensor-data QoS로 구독하고 각각의 창에 표시한다.
  토픽은 상대 이름으로 유지하여 namespace/remap을 지원한다. 영상 동기화는 하지 않는다.
- 뷰어의 `imshow()`와 `waitKey()`는 main 스레드에서 호출한다. 영상이 없을 때도
  GUI 이벤트를 처리하고, q/Esc/Ctrl+C 종료 시 노드와 창을 정리한다.
  실행 방법과 GUI 조건은 [로컬 뷰어 안내](CONTRIBUTING.md#85-opencv-창으로-입력과-디버깅-영상-보기)를 따른다.
- 현재 ROS 파라미터는 `basic_p.left.normal_angular_divisor`와
  `basic_p.right.normal_angular_divisor`다. 노드에서 선언·검증·적용하며 Basic_P에만 연결한다.
- 파라미터 검증 콜백은 성공 여부만 반환한다. 확정된 값을 이후 콜백에서 반영하며,
  worker가 쓰고 있는 객체를 직접 수정하지 않는다. 새 파라미터는 필요할 때 해당 연결을 추가한다.

## 5. 검증과 기여

- 의존성은 `package.xml` 및 실제 설치 방식에 맞게 선언한다. Python 3.8과 대상 장치의
  사용 가능 여부를 확인한다. 모델·설정 파일은 설치 위치에서도 읽을 수 있어야 한다.
- 동작 변경에는 관련 기존 테스트와 중요한 새 동작 검사를 실행한다.
  문서만 바뀌면 링크·예제·명령을 확인한다. 검증 완료 후 같은 검사를 불필요하게 반복하지 않는다.
- [빌드와 검증 절차](CONTRIBUTING.md#8-빌드와-검증)를 따른다.
  ROS 통신 테스트는 별도 domain/namespace와 속도 미리보기 토픽을 사용한다.
- 검증 환경·결과·미검증 범위를 보고한다. Humble 성공이나 Python 3.8 문법 검사만으로
  실제 Galactic·카메라·실차 동작을 검증했다고 쓰지 않는다.
  뷰어 테스트는 GUI 함수를 대체하므로 실제 창 표시 검증 여부를 따로 기록한다.
- Git 기여 순서는 **새 작업 브랜치 → 변경·검증 → 커밋 → 브랜치 push → GitHub PR**이다.
  `main`에 직접 push하지 않는다. 커밋 제목에는 `feat:`, `fix:`, `refactor:`, `docs:` 등을 쓴다.
- 커밋·push·PR·merge는 현재 사용자의 요청과 승인 범위에 맞게 실행한다.
  빌드 산출물·bag·개인 환경·대형 모델을 소스와 섞어 커밋하지 않는다.
- PR에는 입력/결과 의미, 함께 쓰는 조합, 의존성, 실행 방법과 검증 결과를 기록한다.
