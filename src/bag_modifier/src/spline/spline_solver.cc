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

#include "bag_modifier/spline/spline_solver.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace keti {
namespace kadif {
namespace bag_modifier {
namespace {

// A cubic B-spline segment is influenced by exactly four coefficients.
constexpr std::size_t kSupport = 4;

// Two endpoint positions (x and y each) plus two tangent directions.
constexpr int kConstraintCount = 6;

// Uniform cubic B-spline basis on a segment, as a function of the local
// parameter u in [0, 1]. The four values sum to one, which is what keeps the
// curve inside the convex hull of its coefficients.
void BasisValues(double u, double* basis) {
  const double v = 1.0 - u;
  basis[0] = v * v * v / 6.0;
  basis[1] = (3.0 * u * u * u - 6.0 * u * u + 4.0) / 6.0;
  basis[2] = (-3.0 * u * u * u + 3.0 * u * u + 3.0 * u + 1.0) / 6.0;
  basis[3] = u * u * u / 6.0;
}

// Derivative of the basis with respect to the local parameter. The caller
// scales by du/dt to obtain the derivative with respect to t.
void BasisDerivatives(double u, double* basis) {
  const double v = 1.0 - u;
  basis[0] = -v * v / 2.0;
  basis[1] = (3.0 * u * u - 4.0 * u) / 2.0;
  basis[2] = (-3.0 * u * u + 2.0 * u + 1.0) / 2.0;
  basis[3] = u * u / 2.0;
}

}  // namespace

bool SplineSolver::Init(const SplineSolverInitOptions& options) {
  if (options.segment_count < 1 || options.fit_weight < 0.0 ||
      options.second_derivative_weight < 0.0 ||
      options.third_derivative_weight < 0.0) {
    std::cerr << "invalid SplineSolver options" << std::endl;
    return false;
  }
  options_ = options;
  initialized_ = true;
  solved_ = false;
  return true;
}

void SplineSolver::LocateSegment(double t, std::size_t* first_coefficient,
                                 double* local_t) const {
  const double clamped = std::min(std::max(t, 0.0), 1.0);
  const double scaled = clamped * static_cast<double>(options_.segment_count);

  // The last segment owns t = 1, otherwise the lookup would run past the end
  std::size_t segment = static_cast<std::size_t>(std::floor(scaled));
  if (segment >= options_.segment_count) {
    segment = options_.segment_count - 1;
  }

  *first_coefficient = segment;
  *local_t = scaled - static_cast<double>(segment);
}

Eigen::RowVectorXd SplineSolver::BasisRow(double t, int derivative) const {
  Eigen::RowVectorXd row =
      Eigen::RowVectorXd::Zero(static_cast<int>(CoefficientCount()));

  std::size_t first = 0;
  double local_t = 0.0;
  LocateSegment(t, &first, &local_t);

  double basis[kSupport];
  double scale = 1.0;
  if (derivative == 0) {
    BasisValues(local_t, basis);
  } else {
    BasisDerivatives(local_t, basis);
    // Chain rule: the local parameter advances segment_count times faster
    scale = static_cast<double>(options_.segment_count);
  }

  for (std::size_t i = 0; i < kSupport; ++i) {
    row(static_cast<int>(first + i)) = basis[i] * scale;
  }
  return row;
}

Eigen::MatrixXd SplineSolver::DifferenceMatrix(int order) const {
  const int columns = static_cast<int>(CoefficientCount());
  const int rows = columns - order;
  if (rows <= 0) {
    return Eigen::MatrixXd::Zero(0, columns);
  }

  Eigen::MatrixXd difference = Eigen::MatrixXd::Zero(rows, columns);
  for (int r = 0; r < rows; ++r) {
    // Binomial coefficients with alternating signs give the finite difference
    double coefficient = 1.0;
    for (int k = 0; k <= order; ++k) {
      difference(r, r + k) = ((k % 2 == 0) ? 1.0 : -1.0) * coefficient;
      coefficient = coefficient * (order - k) / (k + 1);
    }
  }
  return difference;
}

bool SplineSolver::Solve() {
  if (!initialized_) {
    std::cerr << "SplineSolver::Init() was not called" << std::endl;
    return false;
  }
  // Two endpoints plus at least one interior point, otherwise a straight line
  // is already the answer and there is nothing to fit.
  if (control_points_.size() < 3) {
    return false;
  }
  const double total_length = control_points_.back().s;
  if (!(total_length > 0.0)) {
    std::cerr << "spline control points have no arc length" << std::endl;
    return false;
  }

  const int n = static_cast<int>(CoefficientCount());
  const int points = static_cast<int>(control_points_.size());

  // Design matrix and target values for the interior control points. The
  // endpoints are left out here because they enter as hard constraints.
  Eigen::MatrixXd design = Eigen::MatrixXd::Zero(points - 2, n);
  Eigen::VectorXd target_x = Eigen::VectorXd::Zero(points - 2);
  Eigen::VectorXd target_y = Eigen::VectorXd::Zero(points - 2);
  for (int i = 1; i < points - 1; ++i) {
    const double t = control_points_[i].s / total_length;
    design.row(i - 1) = BasisRow(t, 0);
    target_x(i - 1) = control_points_[i].control_point.x();
    target_y(i - 1) = control_points_[i].control_point.y();
  }

  const Eigen::MatrixXd second = DifferenceMatrix(2);
  const Eigen::MatrixXd third = DifferenceMatrix(3);

  Eigen::MatrixXd normal =
      options_.fit_weight * design.transpose() * design +
      options_.second_derivative_weight * second.transpose() * second +
      options_.third_derivative_weight * third.transpose() * third +
      options_.regularization_weight * Eigen::MatrixXd::Identity(n, n);

  // x and y share the objective but are coupled by the tangent constraints,
  // so they are solved together as one stacked system.
  Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(2 * n, 2 * n);
  hessian.topLeftCorner(n, n) = normal;
  hessian.bottomRightCorner(n, n) = normal;

  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(2 * n);
  gradient.head(n) = options_.fit_weight * design.transpose() * target_x;
  gradient.tail(n) = options_.fit_weight * design.transpose() * target_y;

  // Hard constraints: both endpoints exactly, in position and tangent.
  Eigen::MatrixXd constraint = Eigen::MatrixXd::Zero(kConstraintCount, 2 * n);
  Eigen::VectorXd bound = Eigen::VectorXd::Zero(kConstraintCount);

  const SplineControlPoint& start = control_points_.front();
  const SplineControlPoint& end = control_points_.back();

  const Eigen::RowVectorXd value_at_start = BasisRow(0.0, 0);
  const Eigen::RowVectorXd value_at_end = BasisRow(1.0, 0);
  const Eigen::RowVectorXd slope_at_start = BasisRow(0.0, 1);
  const Eigen::RowVectorXd slope_at_end = BasisRow(1.0, 1);

  constraint.block(0, 0, 1, n) = value_at_start;
  bound(0) = start.control_point.x();
  constraint.block(1, n, 1, n) = value_at_start;
  bound(1) = start.control_point.y();
  constraint.block(2, 0, 1, n) = value_at_end;
  bound(2) = end.control_point.x();
  constraint.block(3, n, 1, n) = value_at_end;
  bound(3) = end.control_point.y();

  // A tangent is a direction, not a vector, so the constraint states that the
  // velocity is parallel to it: sin(a) * x'(t) - cos(a) * y'(t) = 0.
  constraint.block(4, 0, 1, n) = std::sin(start.angle) * slope_at_start;
  constraint.block(4, n, 1, n) = -std::cos(start.angle) * slope_at_start;
  constraint.block(5, 0, 1, n) = std::sin(end.angle) * slope_at_end;
  constraint.block(5, n, 1, n) = -std::cos(end.angle) * slope_at_end;

  // Karush-Kuhn-Tucker system of the equality-constrained least squares
  const int size = 2 * n + kConstraintCount;
  Eigen::MatrixXd kkt = Eigen::MatrixXd::Zero(size, size);
  kkt.topLeftCorner(2 * n, 2 * n) = hessian;
  kkt.topRightCorner(2 * n, kConstraintCount) = constraint.transpose();
  kkt.bottomLeftCorner(kConstraintCount, 2 * n) = constraint;

  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(size);
  rhs.head(2 * n) = gradient;
  rhs.tail(kConstraintCount) = bound;

  const Eigen::FullPivLU<Eigen::MatrixXd> decomposition(kkt);
  if (!decomposition.isInvertible()) {
    std::cerr << "spline system is singular" << std::endl;
    return false;
  }

  const Eigen::VectorXd solution = decomposition.solve(rhs);
  if (!solution.allFinite()) {
    std::cerr << "spline solution is not finite" << std::endl;
    return false;
  }

  coefficients_ = solution.head(2 * n);
  solved_ = true;
  return true;
}

Vec2d SplineSolver::Evaluate(double t) const {
  if (!solved_) {
    return Vec2d();
  }
  const int n = static_cast<int>(CoefficientCount());
  const Eigen::RowVectorXd row = BasisRow(t, 0);
  return Vec2d(row * coefficients_.head(n), row * coefficients_.tail(n));
}

double SplineSolver::EvaluateHeading(double t) const {
  if (!solved_) {
    return 0.0;
  }
  const int n = static_cast<int>(CoefficientCount());
  const Eigen::RowVectorXd row = BasisRow(t, 1);
  return std::atan2(row * coefficients_.tail(n), row * coefficients_.head(n));
}

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
