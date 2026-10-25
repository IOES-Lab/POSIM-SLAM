// SPDX-License-Identifier: Apache-2.0
#pragma once

namespace posim_slam {
// One contract for calibration and all coupled sensor matrix layouts. Include
// the incoming state/keyframe before marginalization, not just the retained
// window. Observation counts remain independently bounded by each sensor.
inline constexpr int kMaxRecentStates=64;
inline constexpr int kMaxKeyframes=7;
inline constexpr int kMaxLivePoseBlocks=kMaxRecentStates+1+kMaxKeyframes+1;
inline constexpr int kMaxStateColumns=15*(kMaxRecentStates+1)+6*(kMaxKeyframes+1);
inline constexpr int kMaxPingLandmarks=64;
inline constexpr int kMaxJointColumns=kMaxStateColumns+3+3*kMaxPingLandmarks;
}  // namespace posim_slam
