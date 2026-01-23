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

#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "bag_modifier/geometry/box2d.h"

namespace keti {
namespace kadif {
namespace bag_modifier {

// rviz styling shared by the two visualisation nodes. Both used to carry their
// own copy of the palette and the label table.

// Colour from the 20-entry qualitative palette, wrapping on overflow.
std_msgs::msg::ColorRGBA CategoryColor(int index);

// Human-readable name of a cyber_perception_msgs::msg::ObstacleType value.
std::string ObstacleTypeLabel(int obstacle_type);

// Marker id offsets, so that the class label, the track id and the arrow of one
// obstacle never collide in the marker namespace.
enum MarkerSlot {
  kMarkerSlotArrow = 0,
  kMarkerSlotClassLabel = 1,
  kMarkerSlotTrackId = 2,
  kMarkerSlotsPerObstacle = 3,
};

inline int MarkerId(int obstacle_id, MarkerSlot slot) {
  return kMarkerSlotsPerObstacle * obstacle_id + static_cast<int>(slot);
}

// A DELETEALL marker, which every array starts with so that obstacles removed
// since the previous frame do not linger in rviz.
visualization_msgs::msg::Marker MakeDeleteAllMarker(
    const std::string& frame_id);

// Namespaces keep the three marker kinds separately toggleable in rviz.
extern const char* const kFootprintNamespace;
extern const char* const kTextNamespace;
extern const char* const kArrowNamespace;

struct TextMarkerOptions {
  std::string frame_id = "map";
  builtin_interfaces::msg::Time stamp;
  std::string text;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double text_height = 1.0;
  int marker_id = 0;
  std_msgs::msg::ColorRGBA color;
};

visualization_msgs::msg::Marker MakeTextMarker(
    const TextMarkerOptions& options);

struct BoundingBoxOptions {
  std::string frame_id = "map";
  builtin_interfaces::msg::Time stamp;
  Box2d footprint;
  double height = 0.0;
  double z = 0.0;
  // Subtracted from the box centre so that large UTM coordinates do not lose
  // precision in the rviz float32 transform.
  double x_offset = 0.0;
  double y_offset = 0.0;
  int marker_id = 0;
  std_msgs::msg::ColorRGBA color;
  // Boxes are drawn translucent so that overlapping obstacles stay readable
  float alpha = 0.45f;
};

// Footprint of an obstacle, drawn as an oriented box.
//
// Earlier revisions emitted jsk_recognition_msgs/BoundingBox, which is not part
// of a standard ROS 2 installation. A CUBE marker renders in stock rviz2 and
// removes the dependency.
visualization_msgs::msg::Marker MakeBoundingBox(
    const BoundingBoxOptions& options);

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
