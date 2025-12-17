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

#include "bag_modifier/track/obstacle_pose_estimator.h"

#include <cmath>

#include "bag_modifier/geometry/angle_math.h"

namespace keti {
namespace kadif {
namespace bag_modifier {
namespace {

// Yaw rates below this magnitude are indistinguishable from straight motion,
// and dividing the speed by them would blow up the CTRV displacement.
constexpr double kMinYawRate = 1e-5;

}  // namespace

bool ObstaclePoseEstimator::Init(
    const ObstaclePoseEstimatorInitOptions& options) {
  if (options.history_size == 0 || options.stationary_speed_threshold < 0.0) {
    return false;
  }
  options_ = options;
  history_.clear();
  return true;
}

void ObstaclePoseEstimator::AddSample(const ObstacleSample& sample) {
  std::deque<ObstacleSample>& track = history_[sample.id];
  track.push_back(sample);
  while (track.size() > options_.history_size) {
    track.pop_front();
  }
}

void ObstaclePoseEstimator::ForgetTrack(int track_id) {
  history_.erase(track_id);
}

bool ObstaclePoseEstimator::HasHistory(int track_id) const {
  return history_.find(track_id) != history_.end();
}

void ObstaclePoseEstimator::Clear() {
  history_.clear();
}

double ObstaclePoseEstimator::AverageSpeed(
    const std::deque<ObstacleSample>& history) const {
  if (history.empty()) {
    return 0.0;
  }
  double total = 0.0;
  for (const ObstacleSample& sample : history) {
    total += sample.Speed();
  }
  return total / static_cast<double>(history.size());
}

bool ObstaclePoseEstimator::Estimate(const ObstacleSample& last_seen,
                                     double time, Box2d* box,
                                     double* speed) const {
  const auto found = history_.find(last_seen.id);
  if (found == history_.end() || found->second.empty()) {
    return false;
  }
  const std::deque<ObstacleSample>& track = found->second;
  const double average_speed = AverageSpeed(track);

  // Stationary: hold the last reported pose
  if (average_speed < options_.stationary_speed_threshold) {
    *box = last_seen.ToBox();
    *speed = average_speed;
    return true;
  }

  const ObstacleSample& oldest = track.front();
  const ObstacleSample& newest = track.back();
  const double history_span = newest.timestamp - oldest.timestamp;
  const double yaw_rate =
      history_span > 0.0 ? AngleDiff(oldest.theta, newest.theta) / history_span
                         : 0.0;

  const double dt = time - last_seen.timestamp;

  // Linear: a single sample, a pedestrian, or effectively no turning.
  // Pedestrian heading from perception is too noisy to drive a turn rate.
  if (track.size() == 1 ||
      last_seen.obstacle_class == ObstacleClass::kPedestrian ||
      std::fabs(yaw_rate) < kMinYawRate) {
    const Vec2d predicted_center(
        last_seen.position.x() + last_seen.velocity.x() * dt,
        last_seen.position.y() + last_seen.velocity.y() * dt);
    *box = Box2d(predicted_center, last_seen.theta, last_seen.length,
                 last_seen.width);
    *speed = average_speed;
    return true;
  }

  // CTRV: constant turn rate and velocity
  const double heading = newest.theta;
  const double dx = average_speed / yaw_rate *
                    (std::sin(heading + yaw_rate * dt) - std::sin(heading));
  const double dy = average_speed / yaw_rate *
                    (-std::cos(heading + yaw_rate * dt) + std::cos(heading));

  const Vec2d predicted_center(newest.position.x() + dx,
                               newest.position.y() + dy);
  *box = Box2d(predicted_center, NormalizeAngle(heading + yaw_rate * dt),
               last_seen.length, last_seen.width);
  *speed = average_speed;
  return true;
}

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
