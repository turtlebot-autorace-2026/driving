# YOLO 차선 모델의 출처

- 원본 저장소: https://github.com/turtlebot-autorace-2026/lane_detection_dl
- 기준 커밋: `f1dcce74456cb0eea68c4db89346cd02137d440b`
- 원본 경로: `lane_detection/models/lane_yolo26n_seg.pt`
- SHA256: `b7afc9eec9ec74c475a4227d4acb780288028568d3530db1ce3b763f2fc3bcd6`
- 클래스: `0 = white`, `1 = yellow`

이 파일은 원본 가중치를 변경 없이 복사한 것이다. `setup.py`는 모델을
`share/driving/models`에 설치한다. 노드의 `yolo_lane.model_path` 시작 파라미터로
다른 모델 경로를 지정할 수 있지만 클래스 의미와 출력 마스크 형식을 맞춰야 한다.

`driving/detection/lane_estimator.py`와 `driving/steering/pure_pursuit.py`의
기하 및 조향 계산도 위 커밋을 기준으로 이식했다. 검출기의 `detect()`는 원본
`detection.py`의 추론 계산을 유지한다. 검출/조향 호출을 분리했으며 외부 차선
enable 토픽(`/lane_state`)은 공통 노드가 처리한다. 초기 False에서도 검출·조향 계산과 디버그 발행을 계속하고, True로 전환한 뒤 새로 수신한 영상부터 주행 속도를 발행한다. 교차로 상태(`/intersection_state`)와
최대 속도·곡률 배율 정책은 원본대로 연결한다. 최대 속도 파라미터 이름은
`pure_pursuit.v_max_intersection`이며 원본 기본 0.12m/s(현재 노드 기본 0.30m/s), 내부 곡률 배율은 0.7이다.
