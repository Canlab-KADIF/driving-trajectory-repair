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

#include <vector>

#include "bag_modifier/geometry/box2d.h"
#include "bag_modifier/geometry/vec2d.h"

namespace keti {
namespace kadif {
namespace bag_modifier {

// Convex-polygon geometry sufficient for oriented bounding-box overlap.
//
// Obstacle footprints are always convex quadrilaterals, so Sutherland-Hodgman
// clipping plus the shoelace formula gives an exact intersection area without
// a general-purpose geometry library. This replaces the previous CGAL-based
// implementation, whose license (GPLv3+) is incompatible with Apache-2.0.

// Signed area x 2 is positive when `polygon` is wound counter-clockwise.
// Returns 0 for degenerate input (fewer than three vertices).
double PolygonArea(const std::vector<Vec2d>& polygon);

// Clips `subject` against every edge of the convex polygon `clip`.
// Both polygons must be convex and counter-clockwise. Returns an empty
// polygon when the two do not overlap.
std::vector<Vec2d> ClipConvexPolygon(const std::vector<Vec2d>& subject,
                                     const std::vector<Vec2d>& clip);

// Intersection-over-union of two oriented boxes, in [0, 1].
// Returns 0 when the boxes are disjoint or when either has zero area.
double BoxIou(const Box2d& lhs, const Box2d& rhs);

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
