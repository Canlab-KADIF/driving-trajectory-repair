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

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "cyber_perception_msgs/PerceptionObstacle.h"
#include "cyber_perception_msgs/PerceptionObstacles.h"

#include "bag_modifier/geometry/box2d.h"
#include "bag_modifier/ros/obstacle_conversion.h"
#include "bag_modifier/track/obstacle_pose_estimator.h"
#include "bag_modifier/viz/marker_style.h"

namespace keti {
namespace kadif {
namespace bag_modifier {

class EstimationVisualizer {
 public:
  EstimationVisualizer() = default;

  bool Init(ros::NodeHandle* node, ros::NodeHandle* private_node);

 private:
  struct DisappearedEntry {
    ObstacleSample sample;
    double height = 0.0;
    double z = 0.0;
    int obstacle_type = 0;
  };

  void OnObstacles(const cyber_perception_msgs::PerceptionObstacles& obstacles);
  void UpdateTracks(const cyber_perception_msgs::PerceptionObstacles& obstacles,
                    double timestamp);
  void DrawEstimates(const ros::Time& stamp, double timestamp,
                     jsk_recognition_msgs::BoundingBoxArray* boxes,
                     visualization_msgs::MarkerArray* markers);

  ros::Subscriber obstacle_subscriber_;
  ros::Publisher box_publisher_;
  ros::Publisher marker_publisher_;

  ObstaclePoseEstimator pose_estimator_;
  std::vector<cyber_perception_msgs::PerceptionObstacle> tracked_obstacles_;
  std::unordered_map<int, DisappearedEntry> disappeared_tracks_;

  std::string frame_id_ = "map";
  double x_offset_ = 0.0;
  double y_offset_ = 0.0;
  double reidentification_window_ = 2.0;
  double class_label_height_ = 3.0;
  double track_id_height_ = 1.0;
};

bool EstimationVisualizer::Init(ros::NodeHandle* node,
                                ros::NodeHandle* private_node) {
  private_node->param("frame_id", frame_id_, frame_id_);
  // Defaulted rather than required: the previous version called getParam
  // without checking the result, leaving the offsets uninitialised when they
  // were not set.
  private_node->param("x_offset", x_offset_, 0.0);
  private_node->param("y_offset", y_offset_, 0.0);
  private_node->param("reidentification_window", reidentification_window_,
                      reidentification_window_);

  ObstaclePoseEstimatorInitOptions estimator_options;
  int history_size = static_cast<int>(estimator_options.history_size);
  private_node->param("history_size", history_size, history_size);
  if (history_size > 0) {
    estimator_options.history_size = static_cast<std::size_t>(history_size);
  }
  private_node->param("stationary_speed_threshold",
                      estimator_options.stationary_speed_threshold,
                      estimator_options.stationary_speed_threshold);
  if (!pose_estimator_.Init(estimator_options)) {
    ROS_ERROR("failed to initialise the pose estimator");
    return false;
  }

  obstacle_subscriber_ = node->subscribe(
      "obstacles", 10, &EstimationVisualizer::OnObstacles, this);
  box_publisher_ = node->advertise<jsk_recognition_msgs::BoundingBoxArray>(
      "obstacles_estimation_vis", 1);
  marker_publisher_ = node->advertise<visualization_msgs::MarkerArray>(
      "obstacles_estimation_vis_vel", 1);
  return true;
}

void EstimationVisualizer::OnObstacles(
    const cyber_perception_msgs::PerceptionObstacles& obstacles) {
  const double timestamp = obstacles.cyber_header.timestamp_sec;
  const ros::Time stamp(timestamp);

  jsk_recognition_msgs::BoundingBoxArray boxes;
  boxes.header.seq = obstacles.cyber_header.sequence_num;
  boxes.header.stamp = stamp;
  boxes.header.frame_id = frame_id_;

  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(MakeDeleteAllMarker(frame_id_));

  DrawEstimates(stamp, timestamp, &boxes, &markers);
  UpdateTracks(obstacles, timestamp);

  box_publisher_.publish(boxes);
  marker_publisher_.publish(markers);
}

void EstimationVisualizer::DrawEstimates(
    const ros::Time& stamp, double timestamp,
    jsk_recognition_msgs::BoundingBoxArray* boxes,
    visualization_msgs::MarkerArray* markers) {
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
    box_options.label = 1;
    boxes->boxes.push_back(MakeBoundingBox(box_options));

    const std_msgs::ColorRGBA color = CategoryColor(entry.obstacle_type);
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
    const cyber_perception_msgs::PerceptionObstacles& obstacles,
    double timestamp) {
  for (auto it = tracked_obstacles_.begin();
       it != tracked_obstacles_.end();) {
    const auto found = std::find_if(
        obstacles.perception_obstacle.begin(),
        obstacles.perception_obstacle.end(),
        [&it](const cyber_perception_msgs::PerceptionObstacle& candidate) {
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
    tracked_obstacles_.push_back(obstacle);
    pose_estimator_.AddSample(ToObstacleSample(obstacle, timestamp));
  }
}

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti

int main(int argc, char** argv) {
  ros::init(argc, argv, "estimation_visualizer");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  keti::kadif::bag_modifier::EstimationVisualizer visualizer;
  if (!visualizer.Init(&node, &private_node)) {
    return 1;
  }

  ros::spin();
  return 0;
}
