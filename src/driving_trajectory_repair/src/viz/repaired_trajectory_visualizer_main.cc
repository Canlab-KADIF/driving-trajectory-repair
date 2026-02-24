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

// Draws the obstacle stream of a repaired bag in rviz: footprint, class label,
// track id, heading arrow, and separate velocity and acceleration arrows.
//
// Point it at the interpolated-only output of the offline tool to see just the
// synthesised poses, or at the full repaired stream to see them in context.

#include <cmath>
#include <string>

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

#include "cyber_perception_msgs/PerceptionObstacle.h"
#include "cyber_perception_msgs/PerceptionObstacles.h"
#include "driving_trajectory_repair/geometry/box2d.h"
#include "driving_trajectory_repair/geometry/vec2d.h"
#include "driving_trajectory_repair/viz/marker_style.h"

namespace keti {
namespace kadif {
namespace driving_trajectory_repair {
namespace {

// Length of the fixed heading arrow, in metres. The velocity arrow is scaled by
// the speed instead, and the acceleration arrow is exaggerated so that the
// small values typical of recorded traffic stay visible.
constexpr double kHeadingArrowLength = 5.0;
constexpr double kAccelerationArrowGain = 3.0;
constexpr double kArrowThickness = 0.2;

visualization_msgs::Marker MakeArrow(const std::string& frame_id,
                                     const ros::Time& stamp, int marker_id,
                                     double x, double y, double yaw,
                                     double length,
                                     const std_msgs::ColorRGBA& color) {
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = frame_id;
  arrow.header.stamp = stamp;
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.action = visualization_msgs::Marker::ADD;
  arrow.id = marker_id;

  arrow.pose.position.x = x;
  arrow.pose.position.y = y;
  arrow.pose.position.z = 0.0;
  const tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, 0.0, yaw);
  tf::quaternionTFToMsg(quaternion, arrow.pose.orientation);

  arrow.color = color;
  arrow.scale.x = length;
  arrow.scale.y = kArrowThickness;
  arrow.scale.z = kArrowThickness;
  return arrow;
}

std_msgs::ColorRGBA MakeColor(float r, float g, float b) {
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = 1.0f;
  return color;
}

}  // namespace

class RepairedTrajectoryVisualizer {
 public:
  RepairedTrajectoryVisualizer() = default;

  bool Init(ros::NodeHandle* node, ros::NodeHandle* private_node);

 private:
  void OnObstacles(
      const cyber_perception_msgs::PerceptionObstacles::ConstPtr& obstacles);

  ros::Subscriber obstacle_subscriber_;
  ros::Publisher box_publisher_;
  ros::Publisher marker_publisher_;
  ros::Publisher velocity_publisher_;
  ros::Publisher acceleration_publisher_;

  std::string frame_id_ = "map";
  double x_offset_ = 0.0;
  double y_offset_ = 0.0;
  double class_label_height_ = 3.0;
  double track_id_height_ = 1.0;
};

bool RepairedTrajectoryVisualizer::Init(ros::NodeHandle* node,
                                        ros::NodeHandle* private_node) {
  private_node->param("frame_id", frame_id_, frame_id_);
  private_node->param("x_offset", x_offset_, 0.0);
  private_node->param("y_offset", y_offset_, 0.0);

  obstacle_subscriber_ =
      node->subscribe("obstacles_modified", 10,
                      &RepairedTrajectoryVisualizer::OnObstacles, this);
  box_publisher_ = node->advertise<jsk_recognition_msgs::BoundingBoxArray>(
      "obstacles_vis_modified", 1);
  marker_publisher_ = node->advertise<visualization_msgs::MarkerArray>(
      "obstacles_vis_vel_modified", 1);
  velocity_publisher_ =
      node->advertise<visualization_msgs::MarkerArray>("converter_vis_vel", 1);
  acceleration_publisher_ = node->advertise<visualization_msgs::MarkerArray>(
      "converter_vis_accel", 1);
  return true;
}

void RepairedTrajectoryVisualizer::OnObstacles(
    const cyber_perception_msgs::PerceptionObstacles::ConstPtr& obstacles) {
  const ros::Time stamp(obstacles->cyber_header.timestamp_sec);

  jsk_recognition_msgs::BoundingBoxArray boxes;
  boxes.header.seq = obstacles->cyber_header.sequence_num;
  boxes.header.stamp = stamp;
  boxes.header.frame_id = frame_id_;

  visualization_msgs::MarkerArray markers;
  visualization_msgs::MarkerArray velocity_arrows;
  visualization_msgs::MarkerArray acceleration_arrows;
  markers.markers.push_back(MakeDeleteAllMarker(frame_id_));
  velocity_arrows.markers.push_back(MakeDeleteAllMarker(frame_id_));
  acceleration_arrows.markers.push_back(MakeDeleteAllMarker(frame_id_));

  for (const cyber_perception_msgs::PerceptionObstacle& obstacle :
       obstacles->perception_obstacle) {
    const int obstacle_type = static_cast<int>(obstacle.type.type);
    const std_msgs::ColorRGBA color = CategoryColor(obstacle_type);

    const double x = obstacle.position.x - x_offset_;
    const double y = obstacle.position.y - y_offset_;
    // The recorded position is on the ground plane, so the box is raised by
    // half its height to sit on the road rather than through it.
    const double z = obstacle.height / 2.0;

    BoundingBoxOptions box_options;
    box_options.frame_id = frame_id_;
    box_options.stamp = ros::Time(obstacle.timestamp);
    box_options.footprint =
        Box2d(Vec2d(obstacle.position.x, obstacle.position.y), obstacle.theta,
              obstacle.length, obstacle.width);
    box_options.height = obstacle.height;
    box_options.z = z;
    box_options.x_offset = x_offset_;
    box_options.y_offset = y_offset_;
    box_options.label = 2;
    boxes.boxes.push_back(MakeBoundingBox(box_options));

    TextMarkerOptions label_options;
    label_options.frame_id = frame_id_;
    label_options.stamp = stamp;
    label_options.text = ObstacleTypeLabel(obstacle_type);
    label_options.x = x;
    label_options.y = y;
    label_options.z = z + class_label_height_;
    label_options.marker_id = MarkerId(obstacle.id, kMarkerSlotClassLabel);
    label_options.color = color;
    markers.markers.push_back(MakeTextMarker(label_options));

    TextMarkerOptions id_options = label_options;
    id_options.text = std::to_string(obstacle.id);
    id_options.z = z + track_id_height_;
    id_options.marker_id = MarkerId(obstacle.id, kMarkerSlotTrackId);
    markers.markers.push_back(MakeTextMarker(id_options));

    markers.markers.push_back(
        MakeArrow(frame_id_, stamp, MarkerId(obstacle.id, kMarkerSlotArrow), x,
                  y, obstacle.theta, kHeadingArrowLength, color));

    const double speed = std::sqrt(obstacle.velocity.x * obstacle.velocity.x +
                                   obstacle.velocity.y * obstacle.velocity.y +
                                   obstacle.velocity.z * obstacle.velocity.z);
    if (speed > 0.0) {
      const Vec2d velocity(obstacle.velocity.x, obstacle.velocity.y);
      velocity_arrows.markers.push_back(MakeArrow(frame_id_, stamp, obstacle.id,
                                                  x, y, velocity.Angle(), speed,
                                                  MakeColor(1.0f, 1.0f, 0.0f)));
    }

    const double acceleration_magnitude =
        std::sqrt(obstacle.acceleration.x * obstacle.acceleration.x +
                  obstacle.acceleration.y * obstacle.acceleration.y +
                  obstacle.acceleration.z * obstacle.acceleration.z);
    if (acceleration_magnitude > 0.0) {
      const Vec2d acceleration(obstacle.acceleration.x,
                               obstacle.acceleration.y);
      acceleration_arrows.markers.push_back(
          MakeArrow(frame_id_, stamp, obstacle.id, x, y, acceleration.Angle(),
                    acceleration_magnitude * kAccelerationArrowGain,
                    MakeColor(1.0f, 0.0f, 0.0f)));
    }
  }

  box_publisher_.publish(boxes);
  marker_publisher_.publish(markers);
  velocity_publisher_.publish(velocity_arrows);
  acceleration_publisher_.publish(acceleration_arrows);
}

}  // namespace driving_trajectory_repair
}  // namespace kadif
}  // namespace keti

int main(int argc, char** argv) {
  ros::init(argc, argv, "repaired_trajectory_visualizer");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  keti::kadif::driving_trajectory_repair::RepairedTrajectoryVisualizer
      visualizer;
  if (!visualizer.Init(&node, &private_node)) {
    return 1;
  }

  ros::spin();
  return 0;
}
