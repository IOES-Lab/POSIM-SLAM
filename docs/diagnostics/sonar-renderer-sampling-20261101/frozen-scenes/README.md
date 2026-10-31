# Frozen inferred scenes / 고정한 추정 장면

The earlier inferred baseline and selected false birth are held fixed on five
training and three unused frames. Only ideal versus source-calibrated sampled
rays change. All 32 complete images (4,907,008 pixel comparisons) are scored.
No geometry, navigation state or time is refitted; scene/pose truth and RGB are
excluded. All 16 ideal controls reproduce the original complete-image scores.

| Group | Ideal birth-minus-baseline score | Sampled birth-minus-baseline score |
| --- | ---: | ---: |
| Five training frames | −402,190.48 | −229,895.85 |
| Three unused frames | −297,937.26 | −159,142.68 |

Lower scores prefer the birth in both arms. Its geometry is unchanged, including
the previously measured approximately 15.24 cm new-surface RMSE. Calibration
closes the actual-scene forward discrepancy but does not turn this partial-scene
inverse into a useful map. The next controls must isolate navigation uncertainty
and incomplete/imperfect background surfaces. No map is admitted.

```sh
python docs/diagnostics/sonar-renderer-sampling-20261101/frozen-scenes/replay.py --output /path/to/new-result
```

The script reuses hash-bound original images/scenes in neighbouring diagnostic
records. It imports only the public independent library and NumPy. The
[derivation receipt](driver-derivation.json) distinguishes the original executed
driver from the portable version's path-only changes. Original observations,
point clouds and failed candidates remain unchanged.

이전 기준 장면과 잘못된 반사체 후보를 고정하고, 학습 5프레임과 미사용
3프레임의 영상 전체를 대조합니다. 바꾸는 것은 음선 샘플링뿐입니다.
원래 방식의 16개 대조 결과는 보존 기록과 일치합니다. 보정 뒤에도 전력
점수는 같은 잘못된 후보를 선호합니다. 약 15.24 cm의 새 표면 RMSE와
전체 점은 그대로입니다. 실제 장면의 관측 모델 대조가 맞아도, 부분 장면의
역추정이 지도를 검증한 것은 아닙니다. 다음은 항법 불확실성과 불완전한
배경 표면의 영향을 분리해 검사합니다. 이 대조로 지도를 승인하지 않습니다.
