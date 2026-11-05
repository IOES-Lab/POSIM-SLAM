// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cstddef>

namespace posim_slam {
// Existing conditional-information address space. These are history/identity
// quotas, independent of local dense matrices and retained numeric-byte limits.
inline constexpr std::size_t kMaxMappingHistoryCoordinates=65536;
inline constexpr std::size_t kMaxMappingHistoryFrames=kMaxMappingHistoryCoordinates/6;
} // namespace posim_slam
