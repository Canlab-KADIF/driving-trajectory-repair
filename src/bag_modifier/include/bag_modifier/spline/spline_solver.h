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

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

#include "bag_modifier/geometry/vec2d.h"

namespace keti {
namespace kadif {
namespace bag_modifier {

// One anchor the fitted curve should pass through or stay close to.
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
  // Number of polynomial segments the curve is built from. More segments
  // follow the control points more closely and smooth less.
  std::size_t segment_count = 8;

  // Weight of the fit against the smoothness terms. The endpoints are not
  // weighted, they are hard constraints, so this only governs how closely the
  // interior control points are followed. Lower means smoother.
  double fit_weight = 1.0;

  // Penalties on the curvature and its rate of change, kept at the 1:5 ratio
  // the previous quadratic program used. Penalising the third difference far
  // more than the second favours a curve whose curvature changes slowly, which
  // looks like a driven line rather than a fitted one.
  //
  // The magnitudes are calibrated for this formulation, where the penalty acts
  // on finite differences of the coefficients rather than on an integrated
  // derivative matrix, so they are not the numbers the old solver used. At
  // these values a 20 m arc is followed to about 3 mm while 30 cm of noise on
  // the interior control points is rejected rather than tracked.
  double second_derivative_weight = 0.2;
  double third_derivative_weight = 1.0;

  // Keeps the normal equations invertible when the control points are nearly
  // collinear and the penalties alone leave a null space.
  double regularization_weight = 1.0e-6;
};

// Fits a smooth 2D curve through a sequence of control points.
//
// Used to bridge the gap between the last pose of a track that perception lost
// and the first pose of the track it was re-identified as, so that the repaired
// trajectory does not contain a straight-line jump.
//
// The curve is a uniform cubic B-spline, fitted by penalised least squares:
// the interior control points are followed as data, the curvature and its rate
// of change are penalised, and the two endpoints are pinned exactly in both
// position and tangent direction because they are real detections rather than
// interpolated guesses. Cubic keeps the curvature continuous across segment
// joints, which is what the repaired trajectory needs.
//
// Earlier revisions delegated this to a quadratic-programming solver from an
// internal planning library, which could also impose a corridor around each
// interior point. That library is not publicly distributed, so the corridor
// became a least-squares weight and the dependency was dropped.
class SplineSolver {
 public:
  SplineSolver() = default;

  SplineSolver(const SplineSolver&) = delete;
  SplineSolver& operator=(const SplineSolver&) = delete;

  bool Init(const SplineSolverInitOptions& options);

  void SetSplineControlPoints(
      const std::vector<SplineControlPoint>& control_points) {
    control_points_ = control_points;
    solved_ = false;
  }

  // Solves for the curve through the control points set previously.
  // Returns false when the problem is degenerate, in which case the caller
  // should leave the trajectory unrepaired rather than emit a bad one.
  bool Solve();

  // Evaluates the fitted curve at normalised arc length t in [0, 1].
  // Only meaningful after a successful Solve().
  Vec2d Evaluate(double t) const;

  // Tangent direction at t, in radians. Exposed for verification.
  double EvaluateHeading(double t) const;

 private:
  std::size_t CoefficientCount() const { return options_.segment_count + 3; }

  // Locates t in the uniform knot sequence and returns the index of the first
  // of the four coefficients that influence it, plus the local parameter.
  void LocateSegment(double t, std::size_t* first_coefficient,
                     double* local_t) const;

  // Row of the design matrix at t: the four non-zero basis values, placed at
  // the coefficients they belong to. `derivative` selects the basis (0, 1).
  Eigen::RowVectorXd BasisRow(double t, int derivative) const;

  // Finite-difference operator of the given order over the coefficients.
  Eigen::MatrixXd DifferenceMatrix(int order) const;

  SplineSolverInitOptions options_;
  std::vector<SplineControlPoint> control_points_;

  // Coefficients of the x and y curves, stacked as [cx; cy]
  Eigen::VectorXd coefficients_;
  bool initialized_ = false;
  bool solved_ = false;
};

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
