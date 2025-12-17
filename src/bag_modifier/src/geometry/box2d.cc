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

#include "bag_modifier/geometry/box2d.h"

#include <algorithm>
#include <cmath>

namespace keti {
namespace kadif {
namespace bag_modifier {
namespace {

// Half-extent of a box when projected onto `axis`
double ProjectedHalfExtent(const Box2d& box, const Vec2d& axis) {
  const Vec2d longitudinal(std::cos(box.heading()), std::sin(box.heading()));
  const Vec2d lateral(-std::sin(box.heading()), std::cos(box.heading()));
  return box.half_length() * std::fabs(longitudinal.DotProduct(axis)) +
         box.half_width() * std::fabs(lateral.DotProduct(axis));
}

}  // namespace

Box2d::Box2d(const Vec2d& center, double heading, double length, double width)
    : center_(center),
      heading_(heading),
      length_(length),
      width_(width),
      half_length_(length / 2.0),
      half_width_(width / 2.0),
      cos_heading_(std::cos(heading)),
      sin_heading_(std::sin(heading)) {}

std::array<Vec2d, 4> Box2d::GetAllCorners() const {
  // Offsets of the corner from the centre, expressed in the map frame
  const double dx_long = half_length_ * cos_heading_;
  const double dy_long = half_length_ * sin_heading_;
  const double dx_lat = half_width_ * -sin_heading_;
  const double dy_lat = half_width_ * cos_heading_;

  return {
      Vec2d(center_.x() + dx_long + dx_lat, center_.y() + dy_long + dy_lat),
      Vec2d(center_.x() - dx_long + dx_lat, center_.y() - dy_long + dy_lat),
      Vec2d(center_.x() - dx_long - dx_lat, center_.y() - dy_long - dy_lat),
      Vec2d(center_.x() + dx_long - dx_lat, center_.y() + dy_long - dy_lat),
  };
}

bool Box2d::HasOverlap(const Box2d& other) const {
  // Two convex shapes are disjoint if and only if a separating axis exists.
  // For rectangles it is sufficient to test the four edge normals, which are
  // the longitudinal and lateral directions of each box.
  const std::array<Vec2d, 4> axes = {
      Vec2d(std::cos(heading_), std::sin(heading_)),
      Vec2d(-std::sin(heading_), std::cos(heading_)),
      Vec2d(std::cos(other.heading()), std::sin(other.heading())),
      Vec2d(-std::sin(other.heading()), std::cos(other.heading())),
  };

  const Vec2d center_offset = other.center() - center_;
  for (const Vec2d& axis : axes) {
    const double gap = std::fabs(center_offset.DotProduct(axis));
    if (gap >
        ProjectedHalfExtent(*this, axis) + ProjectedHalfExtent(other, axis)) {
      return false;
    }
  }
  return true;
}

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
