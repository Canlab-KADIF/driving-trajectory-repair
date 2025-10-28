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

#include <cstddef>
#include <deque>
#include <unordered_map>

#include "bag_modifier/geometry/box2d.h"
#include "bag_modifier/track/obstacle_sample.h"

namespace keti {
namespace kadif {
namespace bag_modifier {

struct ObstaclePoseEstimatorInitOptions {
  // Below this speed the obstacle is treated as stationary and its last known
  // pose is reused. 0.83 m/s is 3 km/h, the point at which perception yaw is no
  // longer reliable for a slow-moving object.
  double stationary_speed_threshold = 0.83;

  // Number of past samples kept per track. Bounds memory and limits how far
  // back the yaw-rate estimate reaches.
  std::size_t history_size = 10;
};

// Predicts where a track that perception has stopped reporting would be at a
// later time.
//
// Three motion models are used, in decreasing order of preference:
//   - stationary: average speed below the threshold, pose is held
//   - linear:     a pedestrian, a single sample, or a negligible yaw rate
//   - CTRV:       constant turn rate and velocity, for everything else
//
// The class owns the per-track history, so both the offline repairer and the
// live visualizer share exactly one implementation of the prediction.
class ObstaclePoseEstimator {
 public:
  ObstaclePoseEstimator() = default;

  bool Init(const ObstaclePoseEstimatorInitOptions& options);

  // Appends a sample to the history of its track, evicting the oldest entry
  // once `history_size` is exceeded.
  void AddSample(const ObstacleSample& sample);

  // Drops the history of a track that is no longer of interest.
  void ForgetTrack(int track_id);

  bool HasHistory(int track_id) const;

  // Predicts the footprint of `last_seen` at `time`.
  // Returns false when the track has no recorded history, in which case
  // `box` and `speed` are left untouched.
  bool Estimate(const ObstacleSample& last_seen, double time, Box2d* box,
                double* speed) const;

  void Clear();

 private:
  double AverageSpeed(const std::deque<ObstacleSample>& history) const;

  ObstaclePoseEstimatorInitOptions options_;
  std::unordered_map<int, std::deque<ObstacleSample>> history_;
};

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
