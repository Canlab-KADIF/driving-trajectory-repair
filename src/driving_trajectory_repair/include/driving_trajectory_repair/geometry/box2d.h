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

#include <array>

#include "driving_trajectory_repair/geometry/vec2d.h"

namespace keti {
namespace kadif {
namespace driving_trajectory_repair {

// Oriented (rotated) rectangle used as the footprint of a perceived obstacle.
//
// `length` runs along the heading direction, `width` perpendicular to it, and
// `heading` is measured counter-clockwise from the +x axis in radians.
class Box2d {
 public:
  Box2d() = default;
  Box2d(const Vec2d& center, double heading, double length, double width);

  const Vec2d& center() const { return center_; }
  double heading() const { return heading_; }
  double length() const { return length_; }
  double width() const { return width_; }
  double half_length() const { return half_length_; }
  double half_width() const { return half_width_; }

  double Area() const { return length_ * width_; }

  // Corners in counter-clockwise order, starting from the front-left corner.
  // The winding order matters: the polygon clipping in polygon_iou.h assumes a
  // consistently oriented convex polygon.
  std::array<Vec2d, 4> GetAllCorners() const;

  // True when the two boxes share any area. Implemented with the separating
  // axis theorem over the four box edge normals, so it is exact for convex
  // rectangles and needs no intersection polygon.
  bool HasOverlap(const Box2d& other) const;

 private:
  Vec2d center_;
  double heading_ = 0.0;
  double length_ = 0.0;
  double width_ = 0.0;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  double cos_heading_ = 1.0;
  double sin_heading_ = 0.0;
};

}  // namespace driving_trajectory_repair
}  // namespace kadif
}  // namespace keti
