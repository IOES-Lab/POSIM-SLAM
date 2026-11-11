# Unknown-surface initialization and numerical stopping

Unknown plane references are initialized from the original range likelihood
before the final spatial support check. A coarse 20 cm distance error must not
reject observations whose fitted reference passes that same check. An
inconsistent fitted surface is still rejected. Initialization supplies no
additional plane prior or independent pose observations.

The initializer tests the minimum-norm **undamped** least-squares step, retaining
the existing 1e-9 step tolerance and 50-trial limit. A heavily damped tiny step
does not establish convergence. Actual improvement is computed as
`(old_residual - trial_residual) @ (old_residual + trial_residual)`; this avoids
subtracting two rounded total costs when other owned surfaces contribute a
large unchanged residual. No sensor noise, support or map threshold changes.

The counter-controls retain the previous `893bec1` failures. A 20 cm coarse
reference was rejected before fitting even though its fitted reference passed.
An unchanged owned-surface cost of 50,000,000 hid valid new-plane progress at
1e-6 and 1e-7 m offsets. The new tests also reject a genuinely inconsistent
final surface, and preserve original-reading ownership.

The original [332-range replay](../sampled-dvl-background-20261101/README.md)
still verifies with every original recorded file unchanged. Its coarse plane
still fails spatial support. A local trace at the previous source showed a
gradient of about 1.36e-5 but an undamped step of 8.34e-11, without ray-branch
changes. Predicted improvement was about one scalar-cost ULP. The
[Python 3.12 PR failure](https://github.com/IOES-Lab/POSIM-SLAM/actions/runs/41263964464/job/124722762783)
is retained as the cross-environment failure, separately from successful push
checks. These are optimizer/reference corrections, not map or hardware accuracy
acceptance and not a research novelty claim.

```sh
PYTHONPATH=src:tests python -m unittest test_native_surface_ranges
PYTHONPATH=src python docs/diagnostics/sampled-dvl-background-20261101/verify.py
```

## 미지 표면 초기화와 수치 수렴

원본 거리 제약으로 미지 평면을 먼저 초기화한 뒤 최종 관측 영역을 검사합니다.
초기 거리 오차가 20 cm여도 최적화한 평면이 같은 검사를 통과하면 갱신을
준비할 수 있습니다. 실제로 모순되는 최종 평면은 거부하며, 초기화 결과의
헤시안이나 공분산을 추가 사전 정보로 쓰지 않습니다.

기존 1e-9 스텝 기준과 50회 시도 한도를 유지하면서 감쇠 없는 최소제곱
스텝으로 수렴을 판단합니다. 비용 차이는 잔차 벡터의 차와 합의 내적으로
계산합니다. 다른 면의 큰 고정 잔차가 작은 개선량을 가리는 문제를 피합니다.
센서 잡음·관측 영역·지도 판정 기준은 같습니다.

기존 소스의 두 실패와 수정 후 대조를 보존합니다. 20 cm 초기 오차의 정상
관측은 최적화 전에 거부됐고, 50,000,000의 고정 비용은 새 평면의 작은 갱신을
가렸습니다. 실제 거리 332개의 기존 기록은 그대로 다시 검사하며, 그 평면의
관측 영역 실패도 유지합니다. 최적화 절차의 수정이며 지도 합격이나 새 연구
기여를 뜻하지 않습니다. 위 명령으로 새 회귀 검사와 원본 재생을 실행합니다.
