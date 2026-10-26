# Experimental moving visual-inertial seed

`src/posim_slam/visual_inertial_seed.py` estimates initial metric velocity and
gravity from calibrated camera bearings and raw-IMU preintegrals. It is an
independent experimental calculation, not connected to the live worker.

In the first IMU frame, position at time t is initial velocity times t, plus
half gravity times t squared, plus preintegrated specific-force displacement.
Each tracked bearing constrains its unknown landmark to a camera ray. The
camera lever arm and gyro-integrated rotation transform that ray. Per-landmark
orthogonal elimination removes landmark coordinates; a six-variable linear
system estimates velocity and gravity. No known object size, depth, pose or
metric visual trajectory is an input. Constant-velocity and zero-parallax
degeneracies are rejected. Input bounds are 4–32 frames, 8–256 tracks and at
most 10 seconds; each track needs at least three observations.

This calculation conditions on supplied gyro/accelerometer biases and relative
rotations. It reports numerical conditioning, residuals and positive depths;
these are not a calibrated covariance or an observability certificate. Gravity
magnitude is unconstrained in this linear seed. Robust feature selection,
gravity-magnitude-constrained nonlinear refinement, bias uncertainty and live
worker integration remain required. Do not enable navigation from this result.

The moving BlueROV recording starting at 69.2 s uses actual native optical-flow
pixels, including tracks not yet triangulated by the failing VIO worker. Only
pixel coordinates and IDs enter the seed; existing estimated map coordinates
are ignored. Pixel-centre scaling matches native camera preprocessing. After
three seconds (177 tracks), an independent truth comparison gives initial
velocity error 0.00578 m/s and gravity-direction error 0.0140 degrees. Truth is
used only after estimation. This is one offline record, not trajectory or
product acceptance. Five synthetic tests include raw-IMU integration, nonzero
camera extrinsics and rejection of constant-velocity/no-parallax inputs.

The approach is established visual-inertial initialization work, not a novelty
claim. [OpenVINS's dynamic initialization description](https://docs.openvins.com/classov__init_1_1DynamicInitializer.html)
discusses bearing/IMU initialization with velocity/gravity estimation and
subsequent refinement/covariance recovery. This module is an independent
implementation of projective kinematics; no external implementation was copied.

## 한국어

이 실험 모듈은 보정된 영상 특징점 방향과 원시 IMU 적분값으로 초기 속도와
중력을 계산합니다. 첫 IMU 좌표계에서 이동 방정식과 카메라 광선 제약을
구성하고, 특징점 위치를 소거한 선형 시스템을 풉니다. 물체의 크기·거리,
시뮬레이터 위치·자세, 기존 SLAM 지도 좌표는 입력하지 않습니다.
카메라의 회전·위치 외부 보정을 포함하며, 일정 속도 이동과 시차가 없는
경우는 거부합니다. 입력은 4–32개 프레임, 8–256개 특징점, 최대 10초이고
각 특징점은 세 프레임 이상에서 관측되어야 합니다.

69.2초부터 시작하는 BlueROV 기록의 3초 구간에서는 177개 특징점으로
초기 속도 오차 0.00578m/s, 중력 방향 오차 0.0140°를 얻었습니다.
정답은 계산 후 평가에만 사용했습니다. 합성 검사 5개에는 원시 IMU 적분,
카메라 외부 보정, 일정 속도·시차 부족 거부가 포함됩니다.

현재는 실시간 실행 경로에 연결하지 않은 독립 초기값 계산입니다.
바이어스와 상대 회전은 주어진 값으로 취급하며 중력 크기는 아직 제약하지
않습니다. 이상 특징점 처리, 중력 크기를 제약한 비선형 보정, 바이어스
불확실성과 공분산 평가, 실제 재시작 연결·궤적 검증이 남아 있습니다.
수치 조건과 잔차가 좋다는 이유만으로 관측 가능성이나 항법 준비 완료를
인정하지 않습니다. 기존 시각·관성 초기화 방법에 해당하며 연구 신규성을
주장하지 않습니다.
