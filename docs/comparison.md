# Method comparison protocol / 방법 비교 규약

Each completed sensor combination requires a paired figure: its generated SLAM
point cloud beside an RGB rendering of the actual test field. Both use a matching
elevated isometric viewpoint, metric extent and axes. Record camera pose and
projection; perspective RGB imagery must not be labelled orthographic. Keep the
same input scene and trajectory across methods, or disclose separate runs.

Use Turbo height colours with a shared blue-to-red scale in metres. State the
displayed and total point counts and retain gaps and outliers. An overview may
show the full cloud; the application's default remains 1,000 points, with
1,000 / 2,000 / 5,000 / full display choices.

The caption identifies sensors, exact library revisions, scene, time interval,
map segment, environment effects and any rigid evaluation-frame alignment.
Preserve the original cloud and input hashes. Report measured trajectory error
and geometry coverage when available. Ground truth is for evaluation and the
reference rendering; it is not an estimator input. Do not rescale the estimate
to fit the reference or substitute an illustration for measured output.

Publish the overview here and in WWW-POSIM's SLAM/fusion guide. Synthetic unit
tests and bounded solver examples do not qualify a completed live combination.
The sonar/IMU/Ping reference solver still needs live integration before its
comparison can be presented as an operational method.

완료된 센서 조합마다 생성된 SLAM 점군과 실제 시험장의 RGB 렌더링을
나란히 제시합니다. 두 영상은 높은 위치에서 내려다보는 등각 시점과 미터
단위 범위·축을 맞춥니다. 카메라 자세와 투영 방식을 기록하며, 원근 RGB
영상을 직교 투영으로 표기하지 않습니다. 방법 간에는 같은 장면과 경로를
사용하고, 별도 주행이면 이를 명시합니다.

높이는 파랑에서 빨강으로 이어지는 Turbo 색상과 공통 미터 단위 범례로
표시합니다. 표시 점 수와 전체 점 수를 밝히고 누락 영역과 이상점을
유지합니다. 비교 개요는 전체 점군을 사용할 수 있습니다. 앱 기본값은
1,000점이며 1,000 / 2,000 / 5,000 / 전체 선택을 유지합니다.

캡션에는 센서, 정확한 라이브러리 리비전, 장면, 시간 구간, 지도 구간,
환경 효과와 평가 좌표계의 강체 정렬 여부를 적습니다. 원본 점군과 입력
해시를 보존하고, 측정된 경로 오차와 형상 관측 범위가 있으면 함께
보고합니다. 정답값은 평가와 참조 렌더링에만 사용합니다. 추정 결과의
축척을 정답에 맞추거나 실제 출력 대신 삽화를 사용하지 않습니다.

비교 개요는 이 저장소와 WWW-POSIM의 SLAM·센서 융합 안내 양쪽에
게시합니다. 합성 단위 시험이나 짧은 구간의 최적화 예제만으로 실제 조합의
완료를 인정하지 않습니다. 소나·IMU·Ping 참조 최적화는 실제 운용 방법으로
비교하기 전에 라이브 스트림 연결이 필요합니다.
