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

#include <cmath>

namespace keti {
namespace kadif {
namespace bag_modifier {

// Two-dimensional vector in the map (UTM) plane.
//
// This type exists so that the package stays free of any external computational
// geometry library. See docs in README: the original implementation linked
// CGAL, which is distributed under the GPL and is therefore incompatible with
// the Apache-2.0 license of this repository.
class Vec2d {
 public:
  Vec2d() = default;
  Vec2d(double x, double y) : x_(x), y_(y) {}

  double x() const { return x_; }
  double y() const { return y_; }

  void set_x(double x) { x_ = x; }
  void set_y(double y) { y_ = y; }

  double Length() const { return std::hypot(x_, y_); }
  double LengthSquare() const { return x_ * x_ + y_ * y_; }

  // Direction of the vector in [-pi, pi), measured from the +x axis
  double Angle() const { return std::atan2(y_, x_); }

  double DotProduct(const Vec2d& other) const {
    return x_ * other.x_ + y_ * other.y_;
  }

  // Z component of the 3D cross product, i.e. the signed parallelogram area
  double CrossProduct(const Vec2d& other) const {
    return x_ * other.y_ - y_ * other.x_;
  }

  double DistanceTo(const Vec2d& other) const {
    return std::hypot(x_ - other.x_, y_ - other.y_);
  }

  Vec2d operator+(const Vec2d& other) const {
    return Vec2d(x_ + other.x_, y_ + other.y_);
  }
  Vec2d operator-(const Vec2d& other) const {
    return Vec2d(x_ - other.x_, y_ - other.y_);
  }
  Vec2d operator*(double ratio) const { return Vec2d(x_ * ratio, y_ * ratio); }

 private:
  double x_ = 0.0;
  double y_ = 0.0;
};

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
