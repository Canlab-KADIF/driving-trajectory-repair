/*
 * Copyright 2026 Korea Electronics Technology Institute (KETI)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cmath>

namespace keti {
namespace kadif {
namespace driving_trajectory_repair {

// Wraps an angle into [-pi, pi)
inline double NormalizeAngle(double angle) {
  double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (wrapped < 0.0) {
    wrapped += 2.0 * M_PI;
  }
  return wrapped - M_PI;
}

// Signed shortest rotation from `from` to `to`, in [-pi, pi)
inline double AngleDiff(double from, double to) {
  return NormalizeAngle(to - from);
}

inline double Square(double value) {
  return value * value;
}

}  // namespace driving_trajectory_repair
}  // namespace kadif
}  // namespace keti
