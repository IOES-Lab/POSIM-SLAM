# Local surface first returns / 국소 면의 첫 반사 거리

`ground_surface_return.surface_returns` intersects every supplied unit ray with
an explicit local height surface, planar or quadratic. It selects the closest
visible positive intersection across the authored vertical fan for each beam.
A near blind-range intersection occludes a later root on that ray. Tangencies
and missing returns are invalid and produce NaN, never a zero-error observation.
Returned Jacobians are local to the selected smooth root/ray branch. The caller
must bound the region where the supplied surface is supported; the mathematical
height surface is otherwise unbounded.

`ground_surface_return.surface_returns`는 입력된 모든 단위 광선과 명시적인
국소 높이 면의 교차점을 계산합니다. 면은 평면 또는 이차 곡면이며, 빔마다
설정된 수직 광선 전체에서 가장 가까운 유효 양의 교차점을 선택합니다.
최소 측정 거리보다 가까운 교차점은 같은 광선의 뒤쪽 교차점을 가립니다.
접선과 반사 없음은 NaN으로 표시하며 오차 0인 관측으로 처리하지 않습니다.
반환된 미분은 선택된 매끄러운 근·광선이 유지되는 구간에서만 유효합니다.
수학적 면은 무한히 연장되므로 호출자가 실제 면의 지지 영역을 제한해야 합니다.

This is geometric prediction, not an acoustic intensity-onset model. It does
not fit a surface, associate a pixel, estimate a pose or qualify a map. Frequency,
material reflectivity, beam mixing and intensity threshold effects are absent.
Six analytic tests cover flat ground, an interior-ray minimum on curved ground,
near occlusion, origin/coefficient/direction derivatives and invalid geometry.
The curved case is checked against a dense independent surface parameterization.
No application default uses this experimental module.

이 함수는 기하학적 예측이며 음향 강도 상승 위치의 모델은 아닙니다. 면 추정,
픽셀 대응, 자세 추정, 지도 품질 판정은 수행하지 않습니다. 주파수, 재질 반사율,
빔 혼합, 강도 임계값 효과는 포함하지 않습니다. 해석적 테스트 6개로 평면,
곡면에서 내부 광선이 만드는 최소 거리, 가까운 가림, 위치·계수·방향 미분,
잘못된 기하를 검증했습니다. 곡면 사례는 독립적으로 촘촘하게 생성한 면과
비교했습니다. 앱 기본 동작에는 이 실험 모듈을 사용하지 않습니다.
