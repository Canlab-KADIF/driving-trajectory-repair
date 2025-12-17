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

#include <memory>
#include <vector>

#include "bag_modifier/geometry/vec2d.h"

// Forward declaration of the quadratic-programming spline solver supplied by
// the internal KETI planning library. Keeping it out of this header means the
// heavy solver headers are only pulled into spline_solver.cc, and the rest of
// the package compiles without them. See README for how to obtain the library.
namespace keti {
namespace planning {
class Spline2dSolver;
}  // namespace planning
}  // namespace keti

namespace keti {
namespace kadif {
namespace bag_modifier {

// One anchor the fitted curve has to pass through or stay close to.
// `s` is the arc length from the first point, `angle` the desired tangent.
struct SplineControlPoint {
  SplineControlPoint() = default;
  SplineControlPoint(const Vec2d& control_point, double s, double angle)
      : control_point(control_point), s(s), angle(angle) {}

  Vec2d control_point;
  double s = 0.0;
  double angle = 0.0;
};

struct SplineSolverInitOptions {
  // Degree of each polynomial segment. Fifth order keeps curvature continuous
  // across the joint, which a lower order would not.
  int order = 5;

  // Corridor half-width, in metres, that intermediate control points may be
  // displaced by. The endpoints are pinned far more tightly because they are
  // actual perception measurements, not interpolated guesses.
  double lateral_bound = 0.7;
  double endpoint_bound = 0.001;
  double longitudinal_bound = 0.001;

  // Relative weights of the smoothness terms in the cost function. Penalising
  // the third derivative far more than the second favours a curve with slowly
  // changing curvature, which looks like a driven line rather than a fitted
  // one.
  double second_derivative_weight = 200.0;
  double third_derivative_weight = 1000.0;
  double regularization_weight = 1.0e-5;
};

// Fits a smooth 2D curve through a sequence of control points by solving a
// quadratic program.
//
// Used to bridge the gap between the last pose of a track that perception lost
// and the first pose of the track it was re-identified as, so that the repaired
// trajectory does not contain a straight-line jump.
class SplineSolver {
 public:
  SplineSolver();
  ~SplineSolver();

  SplineSolver(const SplineSolver&) = delete;
  SplineSolver& operator=(const SplineSolver&) = delete;

  bool Init(const SplineSolverInitOptions& options);

  void SetSplineControlPoints(
      const std::vector<SplineControlPoint>& control_points) {
    control_points_ = control_points;
  }

  // Solves for the curve through the control points set previously.
  // Returns false when the problem is infeasible, in which case the caller
  // should leave the trajectory unrepaired rather than emit a bad one.
  bool Solve();

  // Evaluates the fitted curve at normalised arc length t in [0, 1].
  // Only valid after a successful Solve().
  Vec2d Evaluate(double t) const;

 private:
  bool AddConstraints();
  void AddKernel();

  SplineSolverInitOptions options_;
  std::unique_ptr<keti::planning::Spline2dSolver> solver_;
  std::vector<SplineControlPoint> control_points_;
};

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
