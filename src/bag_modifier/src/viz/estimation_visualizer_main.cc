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

// Live view of what the repairer predicts for tracks perception has dropped.
//
// Subscribing to the same obstacle stream the offline tool reads, this node
// draws a box wherever a vanished track is estimated to be right now, so the
// motion model can be checked against the recorded scene in rviz before a bag
// is rewritten.

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "bag_modifier/geometry/box2d.h"
#include "bag_modifier/ros/obstacle_conversion.h"
#include "bag_modifier/track/obstacle_pose_estimator.h"
#include "bag_modifier/viz/marker_style.h"
#include "cyber_perception_msgs/msg/perception_obstacle.hpp"
#include "cyber_perception_msgs/msg/perception_obstacles.hpp"

namespace keti {
namespace kadif {
namespace bag_modifier {

class EstimationVisualizer : public rclcpp::Node {
 public:
  EstimationVisualizer();

 private:
  struct DisappearedEntry {
    ObstacleSample sample;
    double height = 0.0;
    double z = 0.0;
    int obstacle_type = 0;
  };

  void OnObstacles(
      const cyber_perception_msgs::msg::PerceptionObstacles::SharedPtr message);
  void UpdateTracks(
      const cyber_perception_msgs::msg::PerceptionObstacles& obstacles,
      double timestamp);
  void DrawEstimates(const builtin_interfaces::msg::Time& stamp,
                     double timestamp,
                     visualization_msgs::msg::MarkerArray* markers);

  rclcpp::Subscription<cyber_perception_msgs::msg::PerceptionObstacles>::
      SharedPtr obstacle_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_publisher_;

  ObstaclePoseEstimator pose_estimator_;
  std::vector<cyber_perception_msgs::msg::PerceptionObstacle>
      tracked_obstacles_;
  std::unordered_map<int, DisappearedEntry> disappeared_tracks_;

  std::string frame_id_ = "map";
  double x_offset_ = 0.0;
  double y_offset_ = 0.0;
  double reidentification_window_ = 2.0;
  double class_label_height_ = 3.0;
  double track_id_height_ = 1.0;
};

EstimationVisualizer::EstimationVisualizer()
    : rclcpp::Node("estimation_visualizer") {
  frame_id_ = declare_parameter("frame_id", frame_id_);
  // Defaulted rather than required: the ROS 1 version read these with an
  // unchecked getParam, leaving them uninitialised when they were not set.
  x_offset_ = declare_parameter("x_offset", 0.0);
  y_offset_ = declare_parameter("y_offset", 0.0);
  reidentification_window_ =
      declare_parameter("reidentification_window", reidentification_window_);

  ObstaclePoseEstimatorInitOptions estimator_options;
  const int history_size = declare_parameter(
      "history_size", static_cast<int>(estimator_options.history_size));
  if (history_size > 0) {
    estimator_options.history_size = static_cast<std::size_t>(history_size);
  }
  estimator_options.stationary_speed_threshold =
      declare_parameter("stationary_speed_threshold",
                        estimator_options.stationary_speed_threshold);
  if (!pose_estimator_.Init(estimator_options)) {
    throw std::runtime_error("failed to initialise the pose estimator");
  }

  // A recording is replayed with the profile it was captured with, which for a
  // high-rate perception stream is best effort; a reliable subscription would
  // not match it.
  const rclcpp::QoS qos = rclcpp::SensorDataQoS();
  obstacle_subscription_ =
      create_subscription<cyber_perception_msgs::msg::PerceptionObstacles>(
          "obstacles", qos,
          std::bind(&EstimationVisualizer::OnObstacles, this,
                    std::placeholders::_1));
  marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "obstacles_estimation_vis", rclcpp::QoS(1));
}

void EstimationVisualizer::OnObstacles(
    const cyber_perception_msgs::msg::PerceptionObstacles::SharedPtr message) {
  const double timestamp = message->cyber_header.timestamp_sec;
  const builtin_interfaces::msg::Time stamp =
      rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(MakeDeleteAllMarker(frame_id_));

  DrawEstimates(stamp, timestamp, &markers);
  UpdateTracks(*message, timestamp);

  marker_publisher_->publish(markers);
}

void EstimationVisualizer::DrawEstimates(
    const builtin_interfaces::msg::Time& stamp, double timestamp,
    visualization_msgs::msg::MarkerArray* markers) {
  for (auto it = disappeared_tracks_.begin();
       it != disappeared_tracks_.end();) {
    if (std::fabs(timestamp - it->second.sample.timestamp) >
        reidentification_window_) {
      pose_estimator_.ForgetTrack(it->first);
      it = disappeared_tracks_.erase(it);
      continue;
    }

    const DisappearedEntry& entry = it->second;
    Box2d estimated;
    double estimated_speed = 0.0;
    if (!pose_estimator_.Estimate(entry.sample, timestamp, &estimated,
                                  &estimated_speed)) {
      ++it;
      continue;
    }

    BoundingBoxOptions box_options;
    box_options.frame_id = frame_id_;
    box_options.stamp = stamp;
    box_options.footprint = estimated;
    box_options.height = entry.height;
    box_options.z = entry.z;
    box_options.x_offset = x_offset_;
    box_options.y_offset = y_offset_;
    box_options.marker_id = MarkerId(entry.sample.id, kMarkerSlotArrow);
    const std_msgs::msg::ColorRGBA color = CategoryColor(entry.obstacle_type);
    box_options.color = color;
    markers->markers.push_back(MakeBoundingBox(box_options));

    const double x = estimated.center().x() - x_offset_;
    const double y = estimated.center().y() - y_offset_;

    TextMarkerOptions label_options;
    label_options.frame_id = frame_id_;
    label_options.stamp = stamp;
    label_options.text = ObstacleTypeLabel(entry.obstacle_type);
    label_options.x = x;
    label_options.y = y;
    label_options.z = entry.z + class_label_height_;
    label_options.marker_id = MarkerId(entry.sample.id, kMarkerSlotClassLabel);
    label_options.color = color;
    markers->markers.push_back(MakeTextMarker(label_options));

    TextMarkerOptions id_options = label_options;
    id_options.text = std::to_string(entry.sample.id);
    id_options.z = entry.z + track_id_height_;
    id_options.marker_id = MarkerId(entry.sample.id, kMarkerSlotTrackId);
    markers->markers.push_back(MakeTextMarker(id_options));

    ++it;
  }
}

void EstimationVisualizer::UpdateTracks(
    const cyber_perception_msgs::msg::PerceptionObstacles& obstacles,
    double timestamp) {
  for (auto it = tracked_obstacles_.begin(); it != tracked_obstacles_.end();) {
    const auto found = std::find_if(
        obstacles.perception_obstacle.begin(),
        obstacles.perception_obstacle.end(),
        [&it](const cyber_perception_msgs::msg::PerceptionObstacle& candidate) {
          return candidate.id == it->id;
        });

    if (found == obstacles.perception_obstacle.end()) {
      DisappearedEntry entry;
      entry.sample = ToObstacleSample(*it, timestamp);
      entry.height = it->height;
      entry.z = it->position.z;
      entry.obstacle_type = static_cast<int>(it->type.type);
      disappeared_tracks_[it->id] = entry;
      it = tracked_obstacles_.erase(it);
      continue;
    }

    *it = *found;
    pose_estimator_.AddSample(ToObstacleSample(*it, timestamp));
    ++it;
  }

  for (const cyber_perception_msgs::msg::PerceptionObstacle& obstacle :
       obstacles.perception_obstacle) {
    const bool already_tracked = std::any_of(
        tracked_obstacles_.begin(), tracked_obstacles_.end(),
        [&obstacle](
            const cyber_perception_msgs::msg::PerceptionObstacle& kept) {
          return kept.id == obstacle.id;
        });
    if (already_tracked) {
      continue;
    }
    tracked_obstacles_.push_back(obstacle);
    pose_estimator_.AddSample(ToObstacleSample(obstacle, timestamp));
  }
}

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
        std::make_shared<keti::kadif::bag_modifier::EstimationVisualizer>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("estimation_visualizer"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
