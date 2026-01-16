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

#include "bag_modifier/geometry/angle_math.h"
#include "bag_modifier/spline/spline_solver.h"

namespace keti {
namespace kadif {
namespace bag_modifier {
namespace {

// The tangent constraint fixes a direction, not a sign, so the fitted heading
// may differ from the requested angle by pi. This measures the axis error.
double AxisError(double heading, double angle) {
  return std::fabs(std::sin(AngleDiff(angle, heading)));
}

// Control points along a straight segment, tangents pointing along it.
std::vector<SplineControlPoint> StraightLine(int count, double angle) {
  std::vector<SplineControlPoint> points;
  for (int i = 0; i < count; ++i) {
    const double s = static_cast<double>(i);
    points.emplace_back(Vec2d(s * std::cos(angle), s * std::sin(angle)), s,
                        angle);
  }
  return points;
}

// Control points along a circular arc of the given radius.
std::vector<SplineControlPoint> Arc(int count, double radius,
                                    double sweep_rad) {
  std::vector<SplineControlPoint> points;
  for (int i = 0; i < count; ++i) {
    const double fraction = static_cast<double>(i) / (count - 1);
    const double theta = sweep_rad * fraction;
    // Circle centred at (0, radius), starting at the origin heading +x
    points.emplace_back(
        Vec2d(radius * std::sin(theta), radius * (1.0 - std::cos(theta))),
        radius * theta, theta);
  }
  return points;
}

TEST(SplineSolver, RejectsInvalidOptions) {
  SplineSolver solver;
  SplineSolverInitOptions options;
  options.segment_count = 0;
  EXPECT_FALSE(solver.Init(options));

  options.segment_count = 8;
  options.second_derivative_weight = -1.0;
  EXPECT_FALSE(solver.Init(options));
}

TEST(SplineSolver, RefusesToSolveBeforeInit) {
  SplineSolver solver;
  solver.SetSplineControlPoints(StraightLine(5, 0.0));
  EXPECT_FALSE(solver.Solve());
}

TEST(SplineSolver, RefusesDegenerateInput) {
  SplineSolver solver;
  ASSERT_TRUE(solver.Init({}));

  // Fewer than three points: a straight line is already the answer
  solver.SetSplineControlPoints(StraightLine(2, 0.0));
  EXPECT_FALSE(solver.Solve());

  // All points on top of each other, so there is no arc length
  std::vector<SplineControlPoint> stacked(3);
  solver.SetSplineControlPoints(stacked);
  EXPECT_FALSE(solver.Solve());
}

TEST(SplineSolver, PinsBothEndpointsExactly) {
  SplineSolver solver;
  ASSERT_TRUE(solver.Init({}));

  const std::vector<SplineControlPoint> points = Arc(9, 20.0, 0.6);
  solver.SetSplineControlPoints(points);
  ASSERT_TRUE(solver.Solve());

  const Vec2d start = solver.Evaluate(0.0);
  const Vec2d end = solver.Evaluate(1.0);

  EXPECT_NEAR(start.x(), points.front().control_point.x(), 1e-9);
  EXPECT_NEAR(start.y(), points.front().control_point.y(), 1e-9);
  EXPECT_NEAR(end.x(), points.back().control_point.x(), 1e-9);
  EXPECT_NEAR(end.y(), points.back().control_point.y(), 1e-9);
}

TEST(SplineSolver, PinsBothEndpointTangents) {
  SplineSolver solver;
  ASSERT_TRUE(solver.Init({}));

  const std::vector<SplineControlPoint> points = Arc(9, 20.0, 0.6);
  solver.SetSplineControlPoints(points);
  ASSERT_TRUE(solver.Solve());

  EXPECT_LT(AxisError(solver.EvaluateHeading(0.0), points.front().angle), 1e-7);
  EXPECT_LT(AxisError(solver.EvaluateHeading(1.0), points.back().angle), 1e-7);
}

TEST(SplineSolver, ReproducesAStraightLine) {
  SplineSolver solver;
  ASSERT_TRUE(solver.Init({}));

  const double angle = 0.7;
  solver.SetSplineControlPoints(StraightLine(9, angle));
  ASSERT_TRUE(solver.Solve());

  // Every sample must stay on the line through the origin at `angle`
  const Vec2d direction(std::cos(angle), std::sin(angle));
  for (int i = 0; i <= 20; ++i) {
    const Vec2d point = solver.Evaluate(i / 20.0);
    const double lateral = point.CrossProduct(direction);
    EXPECT_NEAR(lateral, 0.0, 1e-6) << "at t = " << i / 20.0;
  }
}

TEST(SplineSolver, StaysCloseToAnArc) {
  SplineSolver solver;
  ASSERT_TRUE(solver.Init({}));

  constexpr double kRadius = 20.0;
  const std::vector<SplineControlPoint> points = Arc(11, kRadius, 0.8);
  solver.SetSplineControlPoints(points);
  ASSERT_TRUE(solver.Solve());

  // The fitted curve must stay on the same circle, orders of magnitude
  // inside the 0.7 m corridor the previous quadratic program allowed.
  double worst = 0.0;
  for (int i = 0; i <= 40; ++i) {
    const Vec2d point = solver.Evaluate(i / 40.0);
    const double radial =
        std::hypot(point.x() - 0.0, point.y() - kRadius) - kRadius;
    worst = std::max(worst, std::fabs(radial));
  }
  EXPECT_LT(worst, 0.01) << "worst radial deviation " << worst << " m";
}

TEST(SplineSolver, FollowsTheInteriorControlPoints) {
  SplineSolver solver;
  ASSERT_TRUE(solver.Init({}));

  const std::vector<SplineControlPoint> points = Arc(11, 15.0, 0.9);
  solver.SetSplineControlPoints(points);
  ASSERT_TRUE(solver.Solve());

  const double total = points.back().s;
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const Vec2d fitted = solver.Evaluate(points[i].s / total);
    EXPECT_LT(fitted.DistanceTo(points[i].control_point), 0.1)
        << "control point " << i;
  }
}

TEST(SplineSolver, HandlesReversedTangents) {
  SplineSolver solver;
  ASSERT_TRUE(solver.Init({}));

  // A reversing vehicle has its tangents flipped by pi before fitting
  std::vector<SplineControlPoint> points = Arc(9, 25.0, 0.4);
  for (SplineControlPoint& point : points) {
    point.angle = NormalizeAngle(point.angle + M_PI);
  }
  solver.SetSplineControlPoints(points);
  ASSERT_TRUE(solver.Solve());

  EXPECT_LT(AxisError(solver.EvaluateHeading(0.0), points.front().angle), 1e-7);
  EXPECT_LT(AxisError(solver.EvaluateHeading(1.0), points.back().angle), 1e-7);
}

TEST(SplineSolver, EvaluateIsSafeBeforeSolve) {
  SplineSolver solver;
  ASSERT_TRUE(solver.Init({}));
  const Vec2d point = solver.Evaluate(0.5);
  EXPECT_DOUBLE_EQ(point.x(), 0.0);
  EXPECT_DOUBLE_EQ(point.y(), 0.0);
  EXPECT_DOUBLE_EQ(solver.EvaluateHeading(0.5), 0.0);
}

TEST(SplineSolver, RejectsNoiseOnTheInteriorControlPoints) {
  // The control points come from a constant-turn-rate guess, so they carry the
  // error of that model. The fit must follow the underlying shape rather than
  // chase the deviations, otherwise the repaired trajectory zigzags.
  constexpr double kRadius = 20.0;
  constexpr double kNoise = 0.30;

  std::vector<SplineControlPoint> noisy = Arc(11, kRadius, 0.8);
  for (std::size_t i = 1; i + 1 < noisy.size(); ++i) {
    const double sign = (i % 2 == 0) ? 1.0 : -1.0;
    noisy[i] =
        SplineControlPoint(Vec2d(noisy[i].control_point.x(),
                                 noisy[i].control_point.y() + sign * kNoise),
                           noisy[i].s, noisy[i].angle);
  }

  SplineSolver solver;
  ASSERT_TRUE(solver.Init({}));
  solver.SetSplineControlPoints(noisy);
  ASSERT_TRUE(solver.Solve());

  // The curve stays near the true arc, not near the perturbed points
  double worst = 0.0;
  for (int i = 0; i <= 40; ++i) {
    const Vec2d point = solver.Evaluate(i / 40.0);
    worst = std::max(
        worst, std::fabs(std::hypot(point.x(), point.y() - kRadius) - kRadius));
  }
  EXPECT_LT(worst, kNoise / 4.0) << "worst radial deviation " << worst << " m";

  // And it turns monotonically, rather than reversing once per noisy point
  double turning = 0.0;
  double previous = solver.EvaluateHeading(0.0);
  for (int i = 1; i <= 40; ++i) {
    const double heading = solver.EvaluateHeading(i / 40.0);
    turning += std::fabs(AngleDiff(previous, heading));
    previous = heading;
  }
  // A curve that chased the noise would accumulate several radians of turning
  EXPECT_LT(turning, 1.0) << "total absolute turning " << turning << " rad";
}

}  // namespace
}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
