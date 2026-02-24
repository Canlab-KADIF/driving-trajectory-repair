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

#include "driving_trajectory_repair/track/track_repairer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_set>

#include "driving_trajectory_repair/geometry/angle_math.h"
#include "driving_trajectory_repair/geometry/polygon_iou.h"
#include "driving_trajectory_repair/ros/obstacle_conversion.h"

namespace keti {
namespace kadif {
namespace driving_trajectory_repair {
namespace {

// Heading differences below this are treated as straight motion, so the gap is
// bridged with a straight line instead of an arc.
constexpr double kStraightHeadingTolerance = 1e-6;

// Beyond this angle between the travelled segment and the reported heading the
// object is taken to be reversing, and the spline tangents are flipped so the
// fit does not have to turn the object around.
constexpr double kReverseHeadingThreshold = M_PI * 5.0 / 6.0;

// Guards the id chain rewrite against a cycle in the match table.
constexpr int kMaxIdChainLength = 64;

const cyber_perception_msgs::PerceptionObstacle* FindObstacle(
    const cyber_perception_msgs::PerceptionObstacles& obstacles, int id) {
  const auto found = std::find_if(
      obstacles.perception_obstacle.begin(),
      obstacles.perception_obstacle.end(),
      [id](const cyber_perception_msgs::PerceptionObstacle& candidate) {
        return candidate.id == id;
      });
  if (found == obstacles.perception_obstacle.end()) {
    return nullptr;
  }
  return &(*found);
}

}  // namespace

bool TrackRepairer::Init(const TrackRepairerInitOptions& options) {
  if (options.reidentification_window <= 0.0 ||
      options.max_speed_error <= 0.0) {
    std::cerr << "invalid TrackRepairer options" << std::endl;
    return false;
  }
  options_ = options;

  if (!pose_estimator_.Init(options_.pose_estimator)) {
    std::cerr << "failed to initialise the pose estimator" << std::endl;
    return false;
  }
  if (!spline_solver_.Init(options_.spline)) {
    std::cerr << "failed to initialise the spline solver" << std::endl;
    return false;
  }
  initialized_ = true;
  return true;
}

bool TrackRepairer::AddFrame(
    const ros::Time& stamp,
    const cyber_perception_msgs::PerceptionObstacles& obstacles) {
  if (!initialized_) {
    std::cerr << "TrackRepairer::Init() was not called" << std::endl;
    return false;
  }

  const double timestamp = obstacles.cyber_header.timestamp_sec;
  const std::size_t frame_index = repaired_frames_.size();

  std::vector<cyber_perception_msgs::PerceptionObstacle> new_obstacles;
  UpdateTracks(obstacles, timestamp, frame_index, &new_obstacles);
  ExpireDisappearedTracks(timestamp);
  MatchNewTracks(new_obstacles, timestamp, frame_index);

  TimedObstacles recorded;
  recorded.stamp = stamp;
  recorded.obstacles = obstacles;
  repaired_frames_.push_back(recorded);

  // Same time base, but only the synthetic poses are added to it later
  TimedObstacles interpolated;
  interpolated.stamp = stamp;
  interpolated_frames_.push_back(interpolated);

  return true;
}

void TrackRepairer::UpdateTracks(
    const cyber_perception_msgs::PerceptionObstacles& obstacles,
    double timestamp, std::size_t frame_index,
    std::vector<cyber_perception_msgs::PerceptionObstacle>* new_obstacles) {
  // Tracks that are no longer reported become re-identification candidates
  for (auto it = tracked_obstacles_.begin(); it != tracked_obstacles_.end();) {
    const cyber_perception_msgs::PerceptionObstacle* still_present =
        FindObstacle(obstacles, it->id);

    if (still_present == nullptr) {
      DisappearedTrack track;
      track.last_message = *it;
      track.last_sample = ToObstacleSample(*it, timestamp);
      track.frame_index = frame_index;
      disappeared_tracks_[it->id] = track;
      it = tracked_obstacles_.erase(it);
      continue;
    }

    *it = *still_present;
    pose_estimator_.AddSample(ToObstacleSample(*it, timestamp));
    ++it;
  }

  // Tracks reported for the first time
  for (const cyber_perception_msgs::PerceptionObstacle& obstacle :
       obstacles.perception_obstacle) {
    const bool already_tracked = std::any_of(
        tracked_obstacles_.begin(), tracked_obstacles_.end(),
        [&obstacle](const cyber_perception_msgs::PerceptionObstacle& kept) {
          return kept.id == obstacle.id;
        });
    if (already_tracked) {
      continue;
    }
    new_obstacles->push_back(obstacle);
    tracked_obstacles_.push_back(obstacle);
    pose_estimator_.AddSample(ToObstacleSample(obstacle, timestamp));
  }
}

void TrackRepairer::ExpireDisappearedTracks(double timestamp) {
  for (auto it = disappeared_tracks_.begin();
       it != disappeared_tracks_.end();) {
    if (std::fabs(timestamp - it->second.last_sample.timestamp) >
        options_.reidentification_window) {
      // Releasing the history here also bounds memory over a long recording,
      // which the previous implementation did not do.
      pose_estimator_.ForgetTrack(it->first);
      it = disappeared_tracks_.erase(it);
    } else {
      ++it;
    }
  }
}

void TrackRepairer::MatchNewTracks(
    const std::vector<cyber_perception_msgs::PerceptionObstacle>& new_obstacles,
    double timestamp, std::size_t frame_index) {
  if (new_obstacles.empty() || disappeared_tracks_.empty()) {
    return;
  }

  std::vector<MatchCandidate> candidates;

  for (const cyber_perception_msgs::PerceptionObstacle& obstacle :
       new_obstacles) {
    const ObstacleSample new_sample = ToObstacleSample(obstacle, timestamp);
    const Box2d new_box = new_sample.ToBox();
    const double new_speed = new_sample.Speed();

    for (const auto& entry : disappeared_tracks_) {
      const DisappearedTrack& track = entry.second;
      if (obstacle.type.type != track.last_message.type.type) {
        continue;
      }

      Box2d estimated_box;
      double estimated_speed = 0.0;
      if (!pose_estimator_.Estimate(track.last_sample, timestamp,
                                    &estimated_box, &estimated_speed)) {
        continue;
      }

      const double heading_error =
          std::fabs(AngleDiff(new_sample.theta, estimated_box.heading()));
      const double speed_error = std::fabs(estimated_speed - new_speed);

      if (!estimated_box.HasOverlap(new_box) ||
          heading_error >= options_.max_heading_error ||
          speed_error >= options_.max_speed_error) {
        continue;
      }

      const double iou = BoxIou(estimated_box, new_box);
      const double heading_similarity = std::cos(heading_error);
      const double speed_similarity =
          std::sqrt(1.0 - Square(speed_error / options_.max_speed_error));

      MatchCandidate candidate;
      candidate.new_id = obstacle.id;
      candidate.disappeared_id = track.last_message.id;
      // The frame before the track went missing still contains its last pose
      candidate.start_index = track.frame_index > 0 ? track.frame_index - 1 : 0;
      candidate.end_index = frame_index;
      candidate.similarity =
          iou + options_.heading_similarity_weight * heading_similarity +
          options_.speed_similarity_weight * speed_similarity;
      candidates.push_back(candidate);
    }
  }

  // Greedy assignment, best score first. Each new track and each vanished
  // track may take part in at most one match.
  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const MatchCandidate& lhs, const MatchCandidate& rhs) {
                     return lhs.similarity > rhs.similarity;
                   });

  std::unordered_set<int> claimed_new_ids;
  std::unordered_set<int> claimed_disappeared_ids;
  for (const MatchCandidate& candidate : candidates) {
    if (claimed_new_ids.count(candidate.new_id) != 0 ||
        claimed_disappeared_ids.count(candidate.disappeared_id) != 0) {
      continue;
    }
    if (candidate.start_index >= candidate.end_index) {
      continue;
    }

    TrackMatch match;
    match.matched_id = candidate.disappeared_id;
    match.start_index = candidate.start_index;
    match.end_index = candidate.end_index;
    matches_[candidate.new_id] = match;

    claimed_new_ids.insert(candidate.new_id);
    claimed_disappeared_ids.insert(candidate.disappeared_id);
    disappeared_tracks_.erase(candidate.disappeared_id);
  }
}

bool TrackRepairer::Repair() {
  if (!initialized_) {
    std::cerr << "TrackRepairer::Init() was not called" << std::endl;
    return false;
  }
  if (!InterpolateGaps()) {
    return false;
  }
  UnifyMatchedIds();
  return true;
}

bool TrackRepairer::InterpolateGaps() {
  for (const auto& entry : matches_) {
    const int new_id = entry.first;
    const TrackMatch& match = entry.second;

    const TimedObstacles& start_frame = repaired_frames_[match.start_index];
    const TimedObstacles& end_frame = repaired_frames_[match.end_index];

    const cyber_perception_msgs::PerceptionObstacle* start_obstacle =
        FindObstacle(start_frame.obstacles, match.matched_id);
    const cyber_perception_msgs::PerceptionObstacle* end_obstacle =
        FindObstacle(end_frame.obstacles, new_id);
    if (start_obstacle == nullptr || end_obstacle == nullptr) {
      // The pair was matched but one endpoint is not in the frame it was
      // recorded at, so there is nothing reliable to interpolate between.
      std::cerr << "skipping gap for track " << new_id
                << ": endpoint obstacle not found" << std::endl;
      continue;
    }

    const double total_time = end_frame.obstacles.cyber_header.timestamp_sec -
                              start_frame.obstacles.cyber_header.timestamp_sec;
    if (total_time <= 0.0) {
      continue;
    }

    const Vec2d start_point(start_obstacle->position.x,
                            start_obstacle->position.y);
    const Vec2d end_point(end_obstacle->position.x, end_obstacle->position.y);
    const double start_angle = start_obstacle->theta;
    const double end_angle = end_obstacle->theta;
    const double heading_change = AngleDiff(start_angle, end_angle);

    const bool is_pedestrian = ToObstacleClass(start_obstacle->type.type) ==
                               ObstacleClass::kPedestrian;
    const bool is_straight =
        std::fabs(heading_change) <= kStraightHeadingTolerance || is_pedestrian;

    const Vec2d segment = end_point - start_point;

    double angular_velocity = 0.0;
    double speed = 0.0;
    if (is_straight) {
      speed = segment.Length() / total_time;
    } else {
      angular_velocity = heading_change / total_time;
      // Chord length and subtended angle give the arc radius
      const double radius =
          segment.Length() / 2.0 / std::fabs(std::sin(heading_change / 2.0));
      speed = std::fabs(radius * angular_velocity);
    }

    // Travelling opposite to where the object is pointing means it reversed
    const bool is_reverse = std::fabs(AngleDiff(segment.Angle(), start_angle)) >
                            kReverseHeadingThreshold;
    if (is_reverse) {
      speed = -speed;
    }

    const auto tangent_of = [is_reverse](double angle) {
      return is_reverse ? NormalizeAngle(angle + M_PI) : angle;
    };

    // Control points are expressed relative to the start pose so the QP works
    // on metre-scale numbers rather than raw UTM coordinates.
    std::vector<SplineControlPoint> control_points;
    control_points.emplace_back(Vec2d(0.0, 0.0), 0.0, tangent_of(start_angle));

    for (std::size_t i = match.start_index + 1; i < match.end_index; ++i) {
      const double dt =
          repaired_frames_[i].obstacles.cyber_header.timestamp_sec -
          start_frame.obstacles.cyber_header.timestamp_sec;

      double dx = 0.0;
      double dy = 0.0;
      if (is_straight) {
        dx = speed * std::cos(start_angle) * dt;
        dy = speed * std::sin(start_angle) * dt;
      } else {
        dx = speed / angular_velocity *
             (std::sin(start_angle + angular_velocity * dt) -
              std::sin(start_angle));
        dy = speed / angular_velocity *
             (-std::cos(start_angle + angular_velocity * dt) +
              std::cos(start_angle));
      }

      const Vec2d control_point(dx, dy);
      const double s =
          control_points.back().s +
          (control_point - control_points.back().control_point).Length();
      control_points.emplace_back(
          control_point, s,
          tangent_of(NormalizeAngle(start_angle + angular_velocity * dt)));
    }

    const double end_s =
        control_points.back().s +
        (segment - control_points.back().control_point).Length();
    control_points.emplace_back(segment, end_s, tangent_of(end_angle));

    spline_solver_.SetSplineControlPoints(control_points);
    if (!spline_solver_.Solve()) {
      std::cerr << "failed to fit a spline for track " << new_id
                << " matched to " << match.matched_id << std::endl;
      continue;
    }

    const double scale = control_points.back().s;
    for (std::size_t i = match.start_index + 1; i < match.end_index; ++i) {
      const double dt =
          repaired_frames_[i].obstacles.cyber_header.timestamp_sec -
          start_frame.obstacles.cyber_header.timestamp_sec;
      const double t = control_points[i - match.start_index].s / scale;
      const Vec2d point = spline_solver_.Evaluate(t);

      cyber_perception_msgs::PerceptionObstacle interpolated;
      // The new id is used throughout; UnifyMatchedIds() collapses the pair
      interpolated.id = new_id;
      interpolated.position.x = point.x() + start_point.x();
      interpolated.position.y = point.y() + start_point.y();
      interpolated.position.z = start_obstacle->position.z;
      interpolated.theta = NormalizeAngle(start_angle + angular_velocity * dt);
      interpolated.velocity.x =
          speed * std::cos(start_angle + angular_velocity * dt);
      interpolated.velocity.y =
          speed * std::sin(start_angle + angular_velocity * dt);
      interpolated.type = start_obstacle->type;
      interpolated.length = start_obstacle->length;
      interpolated.width = start_obstacle->width;
      interpolated.height = start_obstacle->height;

      repaired_frames_[i].obstacles.perception_obstacle.push_back(interpolated);
      interpolated_frames_[i].obstacles.perception_obstacle.push_back(
          interpolated);
      ++interpolated_pose_count_;
    }
  }
  return true;
}

void TrackRepairer::UnifyMatchedIds() {
  for (std::size_t i = 0; i < repaired_frames_.size(); ++i) {
    RewriteIds(&repaired_frames_[i].obstacles);
    RewriteIds(&interpolated_frames_[i].obstacles);
  }
}

void TrackRepairer::RewriteIds(
    cyber_perception_msgs::PerceptionObstacles* obstacles) const {
  for (cyber_perception_msgs::PerceptionObstacle& obstacle :
       obstacles->perception_obstacle) {
    // A track can be re-identified more than once, so follow the chain back to
    // the id perception first assigned. The bound stops a malformed table from
    // looping forever.
    int hops = 0;
    auto match = matches_.find(obstacle.id);
    while (match != matches_.end() && hops < kMaxIdChainLength) {
      obstacle.id = match->second.matched_id;
      match = matches_.find(obstacle.id);
      ++hops;
    }
    if (hops >= kMaxIdChainLength) {
      std::cerr << "id chain for obstacle " << obstacle.id
                << " exceeded the maximum length, leaving it as is"
                << std::endl;
    }
  }
}

}  // namespace driving_trajectory_repair
}  // namespace kadif
}  // namespace keti
