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

#include "bag_modifier/geometry/polygon_iou.h"

#include <algorithm>
#include <cmath>

namespace keti {
namespace kadif {
namespace bag_modifier {
namespace {

// Tolerance on the cross product used to decide which side of an edge a vertex
// lies on. Obstacle coordinates are UTM metres, so 1e-12 m^2 is far below any
// meaningful overlap while still absorbing floating point noise.
constexpr double kSideEpsilon = 1e-12;

// Positive when `point` lies to the left of the directed edge a -> b.
double SignedSide(const Vec2d& a, const Vec2d& b, const Vec2d& point) {
  return (b - a).CrossProduct(point - a);
}

// Intersection of segment p -> q with the infinite line through a -> b.
// The caller guarantees that p and q lie on opposite sides, so the denominator
// is non-zero.
Vec2d LineIntersection(const Vec2d& a, const Vec2d& b, const Vec2d& p,
                       const Vec2d& q) {
  const Vec2d edge = b - a;
  const Vec2d segment = q - p;
  const double denominator = edge.CrossProduct(segment);
  const double t = edge.CrossProduct(p - a) / denominator;
  return Vec2d(p.x() - segment.x() * t, p.y() - segment.y() * t);
}

}  // namespace

double PolygonArea(const std::vector<Vec2d>& polygon) {
  if (polygon.size() < 3) {
    return 0.0;
  }
  double twice_area = 0.0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const Vec2d& current = polygon[i];
    const Vec2d& next = polygon[(i + 1) % polygon.size()];
    twice_area += current.CrossProduct(next);
  }
  return std::fabs(twice_area) / 2.0;
}

std::vector<Vec2d> ClipConvexPolygon(const std::vector<Vec2d>& subject,
                                     const std::vector<Vec2d>& clip) {
  if (subject.size() < 3 || clip.size() < 3) {
    return {};
  }

  std::vector<Vec2d> output = subject;
  for (size_t i = 0; i < clip.size() && !output.empty(); ++i) {
    const Vec2d& edge_start = clip[i];
    const Vec2d& edge_end = clip[(i + 1) % clip.size()];

    const std::vector<Vec2d> input = output;
    output.clear();

    for (size_t j = 0; j < input.size(); ++j) {
      const Vec2d& current = input[j];
      const Vec2d& previous = input[(j + input.size() - 1) % input.size()];

      const double current_side = SignedSide(edge_start, edge_end, current);
      const double previous_side = SignedSide(edge_start, edge_end, previous);
      const bool current_inside = current_side >= -kSideEpsilon;
      const bool previous_inside = previous_side >= -kSideEpsilon;

      if (current_inside) {
        if (!previous_inside) {
          output.push_back(
              LineIntersection(edge_start, edge_end, previous, current));
        }
        output.push_back(current);
      } else if (previous_inside) {
        output.push_back(
            LineIntersection(edge_start, edge_end, previous, current));
      }
    }
  }
  return output;
}

double BoxIou(const Box2d& lhs, const Box2d& rhs) {
  const double lhs_area = lhs.Area();
  const double rhs_area = rhs.Area();
  if (lhs_area <= 0.0 || rhs_area <= 0.0) {
    return 0.0;
  }
  if (!lhs.HasOverlap(rhs)) {
    return 0.0;
  }

  const std::array<Vec2d, 4> lhs_corners = lhs.GetAllCorners();
  const std::array<Vec2d, 4> rhs_corners = rhs.GetAllCorners();
  const std::vector<Vec2d> intersection = ClipConvexPolygon(
      std::vector<Vec2d>(lhs_corners.begin(), lhs_corners.end()),
      std::vector<Vec2d>(rhs_corners.begin(), rhs_corners.end()));

  const double intersection_area = PolygonArea(intersection);
  // For two convex polygons the union area is the sum minus the overlap, so no
  // separate polygon union has to be constructed.
  const double union_area = lhs_area + rhs_area - intersection_area;
  if (union_area <= 0.0) {
    return 0.0;
  }
  return std::min(1.0, intersection_area / union_area);
}

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
