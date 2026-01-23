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

#include "bag_modifier/viz/marker_style.h"

#include <array>
#include <cmath>

namespace keti {
namespace kadif {
namespace bag_modifier {
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

// Index order matches cyber_perception_msgs::msg::ObstacleType
constexpr std::array<const char*, 6> kObstacleTypeNames = {{
    "UNKNOWN",
    "UNKNOWN_MOVABLE",
    "UNKNOWN_UNMOVABLE",
    "PEDESTRIAN",
    "BICYCLE",
    "VEHICLE",
}};

}  // namespace

const char* const kFootprintNamespace = "footprint";
const char* const kTextNamespace = "text";
const char* const kArrowNamespace = "arrow";

std_msgs::msg::ColorRGBA CategoryColor(int index) {
  const std::size_t slot = static_cast<std::size_t>(
      ((index % kCategoryPalette.size()) + kCategoryPalette.size()) %
      kCategoryPalette.size());
  const Rgb& rgb = kCategoryPalette[slot];

  std_msgs::msg::ColorRGBA color;
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

visualization_msgs::msg::Marker MakeDeleteAllMarker(
    const std::string& frame_id) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

visualization_msgs::msg::Marker MakeTextMarker(
    const TextMarkerOptions& options) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = options.frame_id;
  marker.header.stamp = options.stamp;

  marker.ns = kTextNamespace;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
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

visualization_msgs::msg::Marker MakeBoundingBox(
    const BoundingBoxOptions& options) {
  visualization_msgs::msg::Marker box;
  box.header.frame_id = options.frame_id;
  box.header.stamp = options.stamp;
  box.ns = kFootprintNamespace;
  box.id = options.marker_id;
  box.type = visualization_msgs::msg::Marker::CUBE;
  box.action = visualization_msgs::msg::Marker::ADD;

  box.scale.x = options.footprint.length();
  box.scale.y = options.footprint.width();
  box.scale.z = options.height;

  box.pose.position.x = options.footprint.center().x() - options.x_offset;
  box.pose.position.y = options.footprint.center().y() - options.y_offset;
  box.pose.position.z = options.z;

  // Yaw-only rotation, so the quaternion reduces to a half-angle pair
  const double half_yaw = options.footprint.heading() / 2.0;
  box.pose.orientation.x = 0.0;
  box.pose.orientation.y = 0.0;
  box.pose.orientation.z = std::sin(half_yaw);
  box.pose.orientation.w = std::cos(half_yaw);

  box.color = options.color;
  box.color.a = options.alpha;
  return box;
}

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
