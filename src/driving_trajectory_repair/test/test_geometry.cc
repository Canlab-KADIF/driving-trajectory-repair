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

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "driving_trajectory_repair/geometry/angle_math.h"
#include "driving_trajectory_repair/geometry/box2d.h"
#include "driving_trajectory_repair/geometry/polygon_iou.h"
#include "driving_trajectory_repair/geometry/vec2d.h"

namespace keti {
namespace kadif {
namespace driving_trajectory_repair {
namespace {

constexpr double kEps = 1e-9;

TEST(AngleMath, NormalizeAngleWrapsIntoHalfOpenRange) {
  EXPECT_NEAR(NormalizeAngle(0.0), 0.0, kEps);
  EXPECT_NEAR(NormalizeAngle(M_PI / 2.0), M_PI / 2.0, kEps);
  EXPECT_NEAR(NormalizeAngle(3.0 * M_PI), -M_PI, kEps);
  EXPECT_NEAR(NormalizeAngle(-3.0 * M_PI), -M_PI, kEps);
  EXPECT_NEAR(NormalizeAngle(2.0 * M_PI + 0.25), 0.25, kEps);
  EXPECT_NEAR(NormalizeAngle(-2.0 * M_PI - 0.25), -0.25, kEps);
}

TEST(AngleMath, AngleDiffIsSignedShortestRotation) {
  EXPECT_NEAR(AngleDiff(0.0, 1.0), 1.0, kEps);
  EXPECT_NEAR(AngleDiff(1.0, 0.0), -1.0, kEps);
  // Crossing the +/-pi seam must take the short way round
  EXPECT_NEAR(AngleDiff(3.0, -3.0), 2.0 * M_PI - 6.0, kEps);
  EXPECT_NEAR(AngleDiff(-3.0, 3.0), 6.0 - 2.0 * M_PI, kEps);
}

TEST(Vec2d, BasicOperations) {
  const Vec2d a(3.0, 4.0);
  EXPECT_NEAR(a.Length(), 5.0, kEps);
  EXPECT_NEAR(a.LengthSquare(), 25.0, kEps);
  EXPECT_NEAR(Vec2d(1.0, 0.0).Angle(), 0.0, kEps);
  EXPECT_NEAR(Vec2d(0.0, 1.0).Angle(), M_PI / 2.0, kEps);
  EXPECT_NEAR(a.DotProduct(Vec2d(1.0, 2.0)), 11.0, kEps);
  EXPECT_NEAR(a.CrossProduct(Vec2d(1.0, 2.0)), 2.0, kEps);
  EXPECT_NEAR((a - Vec2d(3.0, 0.0)).Length(), 4.0, kEps);
}

TEST(Box2d, CornersOfAnAxisAlignedBox) {
  const Box2d box(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);
  const std::array<Vec2d, 4> corners = box.GetAllCorners();

  EXPECT_NEAR(corners[0].x(), 2.0, kEps);
  EXPECT_NEAR(corners[0].y(), 1.0, kEps);
  EXPECT_NEAR(corners[2].x(), -2.0, kEps);
  EXPECT_NEAR(corners[2].y(), -1.0, kEps);
  EXPECT_NEAR(box.Area(), 8.0, kEps);
}

TEST(Box2d, CornersRotateWithHeading) {
  const Box2d box(Vec2d(0.0, 0.0), M_PI / 2.0, 4.0, 2.0);
  const std::array<Vec2d, 4> corners = box.GetAllCorners();
  // Front-left corner of a box facing +y sits at (-1, +2)
  EXPECT_NEAR(corners[0].x(), -1.0, 1e-9);
  EXPECT_NEAR(corners[0].y(), 2.0, 1e-9);
}

TEST(Box2d, CornersAreWoundCounterClockwise) {
  const Box2d box(Vec2d(5.0, -3.0), 0.7, 4.0, 2.0);
  const std::array<Vec2d, 4> corners = box.GetAllCorners();
  double twice_area = 0.0;
  for (size_t i = 0; i < corners.size(); ++i) {
    twice_area += corners[i].CrossProduct(corners[(i + 1) % corners.size()]);
  }
  EXPECT_GT(twice_area, 0.0);
}

TEST(Box2d, OverlapDetection) {
  const Box2d reference(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);

  EXPECT_TRUE(reference.HasOverlap(reference));
  EXPECT_TRUE(reference.HasOverlap(Box2d(Vec2d(1.0, 0.0), 0.0, 4.0, 2.0)));
  EXPECT_FALSE(reference.HasOverlap(Box2d(Vec2d(4.1, 0.0), 0.0, 4.0, 2.0)));
  EXPECT_FALSE(reference.HasOverlap(Box2d(Vec2d(0.0, 2.1), 0.0, 4.0, 2.0)));

  // A rotated box that only a proper separating-axis test rejects: its
  // axis-aligned bounding box overlaps but the rectangles themselves do not.
  const Box2d rotated(Vec2d(2.6, 2.6), M_PI / 4.0, 4.0, 0.2);
  EXPECT_FALSE(reference.HasOverlap(rotated));
}

TEST(PolygonIou, AreaOfSimplePolygons) {
  const std::vector<Vec2d> unit_square = {Vec2d(0.0, 0.0), Vec2d(1.0, 0.0),
                                          Vec2d(1.0, 1.0), Vec2d(0.0, 1.0)};
  EXPECT_NEAR(PolygonArea(unit_square), 1.0, kEps);

  const std::vector<Vec2d> triangle = {Vec2d(0.0, 0.0), Vec2d(4.0, 0.0),
                                       Vec2d(0.0, 3.0)};
  EXPECT_NEAR(PolygonArea(triangle), 6.0, kEps);

  EXPECT_NEAR(PolygonArea({Vec2d(0.0, 0.0), Vec2d(1.0, 1.0)}), 0.0, kEps);
}

TEST(PolygonIou, ClippingProducesTheOverlapRegion) {
  const std::vector<Vec2d> a = {Vec2d(0.0, 0.0), Vec2d(2.0, 0.0),
                                Vec2d(2.0, 2.0), Vec2d(0.0, 2.0)};
  const std::vector<Vec2d> b = {Vec2d(1.0, 1.0), Vec2d(3.0, 1.0),
                                Vec2d(3.0, 3.0), Vec2d(1.0, 3.0)};
  EXPECT_NEAR(PolygonArea(ClipConvexPolygon(a, b)), 1.0, 1e-9);

  const std::vector<Vec2d> disjoint = {Vec2d(10.0, 10.0), Vec2d(11.0, 10.0),
                                       Vec2d(11.0, 11.0), Vec2d(10.0, 11.0)};
  EXPECT_TRUE(ClipConvexPolygon(a, disjoint).empty());
}

TEST(PolygonIou, IdenticalBoxesScoreOne) {
  const Box2d box(Vec2d(1.5, -2.5), 0.9, 4.6, 1.9);
  EXPECT_NEAR(BoxIou(box, box), 1.0, 1e-9);
}

TEST(PolygonIou, DisjointBoxesScoreZero) {
  const Box2d a(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);
  const Box2d b(Vec2d(50.0, 50.0), 0.0, 4.0, 2.0);
  EXPECT_NEAR(BoxIou(a, b), 0.0, kEps);
}

TEST(PolygonIou, PartialOverlapMatchesTheAnalyticValue) {
  // Two 2 x 2 squares offset by 1 m along x: overlap 1 x 2 = 2, union 6
  const Box2d a(Vec2d(0.0, 0.0), 0.0, 2.0, 2.0);
  const Box2d b(Vec2d(1.0, 0.0), 0.0, 2.0, 2.0);
  EXPECT_NEAR(BoxIou(a, b), 2.0 / 6.0, 1e-9);

  // Half the length offset along the heading of a 4 x 2 box
  const Box2d c(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);
  const Box2d d(Vec2d(2.0, 0.0), 0.0, 4.0, 2.0);
  EXPECT_NEAR(BoxIou(c, d), 4.0 / 12.0, 1e-9);
}

TEST(PolygonIou, RotatedOverlapMatchesTheAnalyticValue) {
  // A unit square and the same square turned 45 degrees about its centre
  // intersect in a regular octagon of area 2 * (sqrt(2) - 1).
  const Box2d upright(Vec2d(0.0, 0.0), 0.0, 1.0, 1.0);
  const Box2d turned(Vec2d(0.0, 0.0), M_PI / 4.0, 1.0, 1.0);

  const double expected_intersection = 2.0 * (std::sqrt(2.0) - 1.0);
  const double expected_iou =
      expected_intersection / (2.0 - expected_intersection);
  EXPECT_NEAR(BoxIou(upright, turned), expected_iou, 1e-9);
}

TEST(PolygonIou, ResultIsSymmetric) {
  const Box2d a(Vec2d(0.0, 0.0), 0.3, 4.7, 2.1);
  const Box2d b(Vec2d(1.7, 0.6), -0.4, 4.2, 1.8);
  EXPECT_NEAR(BoxIou(a, b), BoxIou(b, a), 1e-12);
}

TEST(PolygonIou, DegenerateBoxesScoreZero) {
  const Box2d valid(Vec2d(0.0, 0.0), 0.0, 4.0, 2.0);
  const Box2d zero_width(Vec2d(0.0, 0.0), 0.0, 4.0, 0.0);
  EXPECT_NEAR(BoxIou(valid, zero_width), 0.0, kEps);
}

TEST(PolygonIou, ContainedBoxScoresRatioOfAreas) {
  const Box2d outer(Vec2d(0.0, 0.0), 0.0, 4.0, 4.0);
  const Box2d inner(Vec2d(0.0, 0.0), 0.0, 2.0, 2.0);
  // Intersection is the inner box: 4 / 16
  EXPECT_NEAR(BoxIou(outer, inner), 4.0 / 16.0, 1e-9);
}

}  // namespace
}  // namespace driving_trajectory_repair
}  // namespace kadif
}  // namespace keti
