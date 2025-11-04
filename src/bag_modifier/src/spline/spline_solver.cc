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

#include <iostream>

#include "math/smoothing_spline/osqp_spline_2d_solver.h"

namespace keti {
namespace kadif {
namespace bag_modifier {
namespace {

using SolverVec2d = keti::common::math::Vec2d;

SolverVec2d ToSolverVec(const Vec2d& point) {
  return SolverVec2d(point.x(), point.y());
}

}  // namespace

SplineSolver::SplineSolver() = default;
SplineSolver::~SplineSolver() = default;

bool SplineSolver::Init(const SplineSolverInitOptions& options) {
  if (options.order < 1) {
    std::cerr << "spline order must be positive, got " << options.order
              << std::endl;
    return false;
  }
  options_ = options;

  // A single segment spanning the whole normalised arc length. Reset() below
  // replaces these knots with the actual control point spacing on every solve.
  const std::vector<double> initial_knots{0.0, 1.0};
  solver_.reset(
      new keti::planning::OsqpSpline2dSolver(initial_knots, options_.order));
  return solver_ != nullptr;
}

bool SplineSolver::Solve() {
  if (solver_ == nullptr) {
    std::cerr << "SplineSolver::Init() was not called" << std::endl;
    return false;
  }
  // Two endpoints plus at least one interior point, otherwise a straight line
  // is already the answer and the QP is degenerate.
  if (control_points_.size() < 3) {
    return false;
  }
  if (control_points_.back().s <= 0.0) {
    std::cerr << "spline control points have no arc length" << std::endl;
    return false;
  }

  if (!AddConstraints()) {
    return false;
  }
  AddKernel();

  if (!solver_->Solve()) {
    std::cerr << "spline QP is infeasible" << std::endl;
    return false;
  }
  return true;
}

Vec2d SplineSolver::Evaluate(double t) const {
  const auto point = solver_->spline()(t);
  return Vec2d(point.first, point.second);
}

bool SplineSolver::AddConstraints() {
  const double scale = control_points_.back().s;

  std::vector<double> knots;
  std::vector<SolverVec2d> reference_points;
  std::vector<double> angles;
  std::vector<double> lateral_bounds;
  const std::vector<double> longitudinal_bounds(control_points_.size(),
                                                options_.longitudinal_bound);

  knots.reserve(control_points_.size());
  reference_points.reserve(control_points_.size());
  angles.reserve(control_points_.size());
  lateral_bounds.reserve(control_points_.size());

  for (size_t i = 0; i < control_points_.size(); ++i) {
    const SplineControlPoint& point = control_points_[i];
    knots.push_back(point.s / scale);
    reference_points.push_back(ToSolverVec(point.control_point));
    angles.push_back(point.angle);

    const bool is_endpoint = (i == 0 || i + 1 == control_points_.size());
    lateral_bounds.push_back(is_endpoint ? options_.endpoint_bound
                                         : options_.lateral_bound);
  }

  solver_->Reset(knots, options_.order);
  keti::planning::Spline2dConstraint* constraint =
      solver_->mutable_constraint();

  if (!constraint->Add2dBoundary(knots, angles, reference_points,
                                 longitudinal_bounds, lateral_bounds)) {
    std::cerr << "failed to add the 2d corridor boundary" << std::endl;
    return false;
  }
  // Pin both ends to the measured pose: those are real detections, and letting
  // the curve drift there would reintroduce the jump this repair removes.
  if (!constraint->AddPointAngleConstraint(knots.front(), angles.front()) ||
      !constraint->AddPointAngleConstraint(knots.back(), angles.back())) {
    std::cerr << "failed to pin the endpoint headings" << std::endl;
    return false;
  }
  if (!constraint->AddPointConstraint(knots.front(),
                                      reference_points.front().x(),
                                      reference_points.front().y()) ||
      !constraint->AddPointConstraint(knots.back(), reference_points.back().x(),
                                      reference_points.back().y())) {
    std::cerr << "failed to pin the endpoint positions" << std::endl;
    return false;
  }
  if (!constraint->AddSecondDerivativeSmoothConstraint()) {
    std::cerr << "failed to enforce curvature continuity" << std::endl;
    return false;
  }
  return true;
}

void SplineSolver::AddKernel() {
  keti::planning::Spline2dKernel* kernel = solver_->mutable_kernel();
  kernel->AddSecondOrderDerivativeMatrix(options_.second_derivative_weight);
  kernel->AddThirdOrderDerivativeMatrix(options_.third_derivative_weight);
  kernel->AddRegularization(options_.regularization_weight);
}

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
