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

#include "driving_trajectory_repair/viz/marker_style.h"

#include <array>

#include <tf/tf.h>

namespace keti {
namespace kadif {
namespace driving_trajectory_repair {
namespace {

struct Rgb {
  float r;
  float g;
  float b;
};

// Twenty-colour qualitative palette, paired light and dark, so that adjacent
// track ids stay visually distinct.
constexpr std::array<Rgb, 20> kCategoryPalette = {{
    {0.121569f, 0.466667f, 0.705882f}, {0.682353f, 0.780392f, 0.909804f},
    {1.000000f, 0.498039f, 0.054902f}, {1.000000f, 0.733333f, 0.470588f},
    {0.172549f, 0.627451f, 0.172549f}, {0.596078f, 0.874510f, 0.541176f},
    {0.839216f, 0.152941f, 0.156863f}, {1.000000f, 0.596078f, 0.588235f},
    {0.580392f, 0.403922f, 0.741176f}, {0.772549f, 0.690196f, 0.835294f},
    {0.549020f, 0.337255f, 0.294118f}, {0.768627f, 0.611765f, 0.580392f},
    {0.890196f, 0.466667f, 0.760784f}, {0.968627f, 0.713725f, 0.823529f},
    {0.498039f, 0.498039f, 0.498039f}, {0.780392f, 0.780392f, 0.780392f},
    {0.737255f, 0.741176f, 0.133333f}, {0.858824f, 0.858824f, 0.552941f},
    {0.090196f, 0.745098f, 0.811765f}, {0.619608f, 0.854902f, 0.898039f},
}};

// Index order matches cyber_perception_msgs::ObstacleType
constexpr std::array<const char*, 6> kObstacleTypeNames = {{
    "UNKNOWN",
    "UNKNOWN_MOVABLE",
    "UNKNOWN_UNMOVABLE",
    "PEDESTRIAN",
    "BICYCLE",
    "VEHICLE",
}};

}  // namespace

std_msgs::ColorRGBA CategoryColor(int index) {
  const std::size_t slot = static_cast<std::size_t>(
      ((index % kCategoryPalette.size()) + kCategoryPalette.size()) %
      kCategoryPalette.size());
  const Rgb& rgb = kCategoryPalette[slot];

  std_msgs::ColorRGBA color;
  color.r = rgb.r;
  color.g = rgb.g;
  color.b = rgb.b;
  color.a = 1.0f;
  return color;
}

std::string ObstacleTypeLabel(int obstacle_type) {
  if (obstacle_type < 0 ||
      static_cast<std::size_t>(obstacle_type) >= kObstacleTypeNames.size()) {
    return "UNKNOWN";
  }
  return kObstacleTypeNames[static_cast<std::size_t>(obstacle_type)];
}

visualization_msgs::Marker MakeDeleteAllMarker(const std::string& frame_id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  return marker;
}

visualization_msgs::Marker MakeTextMarker(const TextMarkerOptions& options) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = options.frame_id;
  marker.header.stamp = options.stamp;

  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = options.marker_id;
  marker.text = options.text;
  marker.color = options.color;
  marker.scale.z = options.text_height;

  marker.pose.position.x = options.x;
  marker.pose.position.y = options.y;
  marker.pose.position.z = options.z;
  marker.pose.orientation.w = 1.0;
  return marker;
}

jsk_recognition_msgs::BoundingBox MakeBoundingBox(
    const BoundingBoxOptions& options) {
  jsk_recognition_msgs::BoundingBox box;
  box.header.frame_id = options.frame_id;
  box.header.stamp = options.stamp;

  box.dimensions.x = options.footprint.length();
  box.dimensions.y = options.footprint.width();
  box.dimensions.z = options.height;

  box.pose.position.x = options.footprint.center().x() - options.x_offset;
  box.pose.position.y = options.footprint.center().y() - options.y_offset;
  box.pose.position.z = options.z;

  const tf::Quaternion quaternion =
      tf::createQuaternionFromRPY(0.0, 0.0, options.footprint.heading());
  tf::quaternionTFToMsg(quaternion, box.pose.orientation);

  box.label = options.label;
  return box;
}

}  // namespace driving_trajectory_repair
}  // namespace kadif
}  // namespace keti
