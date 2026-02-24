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
#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>

#include "cyber_perception_msgs/msg/perception_obstacle.hpp"
#include "cyber_perception_msgs/msg/perception_obstacles.hpp"
#include "driving_trajectory_repair/spline/spline_solver.h"
#include "driving_trajectory_repair/track/obstacle_pose_estimator.h"
#include "driving_trajectory_repair/track/obstacle_sample.h"

namespace keti {
namespace kadif {
namespace driving_trajectory_repair {

// One perception frame together with the bag time it was recorded at.
//
// The stamp is plain nanoseconds rather than a ROS time type, so this header
// stays free of any ROS client library and the algorithm survives a change of
// ROS version.
struct TimedObstacles {
  std::int64_t stamp_ns = 0;
  cyber_perception_msgs::msg::PerceptionObstacles obstacles;
};

// A track that perception stopped reporting, kept as a re-identification
// candidate until the matching window expires.
struct DisappearedTrack {
  ObstacleSample last_sample;
  cyber_perception_msgs::msg::PerceptionObstacle last_message;
  // Index of the frame in which the track was first missing
  std::size_t frame_index = 0;
};

// Where a new track was matched back onto an older one.
struct TrackMatch {
  int matched_id = -1;
  // Last frame in which the old track was still reported
  std::size_t start_index = 0;
  // First frame in which the new track appeared
  std::size_t end_index = 0;
};

struct TrackRepairerInitOptions {
  // Largest speed difference, in m/s, still considered the same object.
  // 13.9 m/s is 50 km/h: generous, because the estimate spans a gap in which
  // the object was unobserved and may have accelerated.
  double max_speed_error = 13.9;

  // Largest heading difference still considered the same object.
  double max_heading_error = M_PI / 2.0;

  // How long a lost track stays available for re-identification, in seconds.
  double reidentification_window = 2.0;

  // Relative weight of the heading and speed terms against the IoU term when
  // scoring a candidate pair. IoU carries an implicit weight of 1.
  double heading_similarity_weight = 0.5;
  double speed_similarity_weight = 0.5;

  ObstaclePoseEstimatorInitOptions pose_estimator;
  SplineSolverInitOptions spline;
};

// Repairs broken object tracks in a recorded perception stream.
//
// Perception drops a track when an object is occluded or leaves the sensor
// field of view, and issues a fresh id when it comes back. Downstream analysis
// then sees two short trajectories instead of one continuous one.
//
// This class replays the recorded frames in order, re-identifies each newly
// appearing track against the tracks that recently vanished, and fills the gap
// between them with spline-interpolated poses.
//
// Usage: Init(), then AddFrame() for every frame in bag order, then Repair().
class TrackRepairer {
 public:
  TrackRepairer() = default;

  TrackRepairer(const TrackRepairer&) = delete;
  TrackRepairer& operator=(const TrackRepairer&) = delete;

  bool Init(const TrackRepairerInitOptions& options);

  // Ingests one perception frame. Frames must be supplied in recording order,
  // because gap indices refer to positions in this sequence.
  bool AddFrame(
      std::int64_t stamp_ns,
      const cyber_perception_msgs::msg::PerceptionObstacles& obstacles);

  // Interpolates every matched gap and rewrites the re-identified ids.
  // Call once, after the last AddFrame().
  bool Repair();

  // Every input frame, with interpolated obstacles inserted and ids unified.
  const std::vector<TimedObstacles>& repaired_frames() const {
    return repaired_frames_;
  }

  // The interpolated obstacles alone, on the same time base, so that the
  // synthetic poses can be inspected separately from the recorded ones.
  const std::vector<TimedObstacles>& interpolated_frames() const {
    return interpolated_frames_;
  }

  std::size_t matched_track_count() const { return matches_.size(); }
  std::size_t interpolated_pose_count() const {
    return interpolated_pose_count_;
  }

 private:
  // Scored re-identification candidate between a new and a vanished track.
  struct MatchCandidate {
    int new_id = -1;
    int disappeared_id = -1;
    std::size_t start_index = 0;
    std::size_t end_index = 0;
    double similarity = 0.0;
  };

  void UpdateTracks(
      const cyber_perception_msgs::msg::PerceptionObstacles& obstacles,
      double timestamp, std::size_t frame_index,
      std::vector<cyber_perception_msgs::msg::PerceptionObstacle>*
          new_obstacles);

  void ExpireDisappearedTracks(double timestamp);

  void MatchNewTracks(
      const std::vector<cyber_perception_msgs::msg::PerceptionObstacle>&
          new_obstacles,
      double timestamp, std::size_t frame_index);

  bool InterpolateGaps();
  void UnifyMatchedIds();
  void RewriteIds(
      cyber_perception_msgs::msg::PerceptionObstacles* obstacles) const;

  TrackRepairerInitOptions options_;
  ObstaclePoseEstimator pose_estimator_;
  SplineSolver spline_solver_;

  std::vector<cyber_perception_msgs::msg::PerceptionObstacle>
      tracked_obstacles_;
  std::unordered_map<int, DisappearedTrack> disappeared_tracks_;
  // Ordered so that repeated runs over the same bag produce identical output
  std::map<int, TrackMatch> matches_;

  std::vector<TimedObstacles> repaired_frames_;
  std::vector<TimedObstacles> interpolated_frames_;
  std::size_t interpolated_pose_count_ = 0;
  bool initialized_ = false;
};

}  // namespace driving_trajectory_repair
}  // namespace kadif
}  // namespace keti
