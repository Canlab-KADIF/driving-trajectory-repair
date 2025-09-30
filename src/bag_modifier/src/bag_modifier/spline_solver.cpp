#include "spline_solver.h"

SplineSolver::SplineSolver(){
  std::vector<double> t_knots{0.0, 1.0};
  spline_2d_solver_.reset(new OsqpSpline2dSolver(t_knots, ORDER));  
}

Vec2d SplineSolver::operator()(const double t){
  const auto pt = spline_2d_solver_->spline()(t);
  return Vec2d(pt.first, pt.second);
}

bool SplineSolver::Solve(){
  if(!AddConstraint()){
    std::cout << "failed to AddConstraint" << std::endl;
    return false;
  }

  AddKernel();

  if (!spline_2d_solver_->Solve()){
    std::cout << "Failed to solve spline_2d_solver_!" << std::endl;
    return false;
  }

  return true;
}

bool SplineSolver::AddConstraint(){
  std::vector<double> t_knots;
  std::vector<Vec2d> ref_points;
  std::vector<double> angles, lateral_bound;
  std::vector<double> longitudinal_bound(spline_control_points_.size(), 0.001);

  const double scale = spline_control_points_.back().s;
  for(int i = 0 ; i < spline_control_points_.size() ; i++){
    auto control_pt = spline_control_points_[i];
    t_knots.push_back(control_pt.s / scale);
    ref_points.push_back(control_pt.control_point);
    angles.push_back(control_pt.angle);
    if(i == 0 || i == spline_control_points_.size() - 1){
      lateral_bound.push_back(0.001);
    }
    else{
      lateral_bound.push_back(0.7);
    }
  }

  spline_2d_solver_->Reset(t_knots, ORDER);
  Spline2dConstraint* constraint = spline_2d_solver_->mutable_constraint();

  if ( !constraint->Add2dBoundary(t_knots, angles, ref_points, longitudinal_bound, lateral_bound) ){
    std::cout << "Failed to Add2dBoundary!" << std::endl;
    return false;
  }
  if (!constraint->AddPointAngleConstraint(t_knots.front(), angles.front())) {
    std::cout << "failed to AddPointAngleConstraint" << std::endl;
    return false;
  }
  if (!constraint->AddPointAngleConstraint(t_knots.back(), angles.back())){
    std::cout << "failed to AddPointAngleConstraint" << std::endl;
    return false;
  }
  if (!constraint->AddPointConstraint(t_knots.front(), ref_points.front().x(), ref_points.front().y())) {
    std::cout << "failed to AddPointConstraint" << std::endl;
    return false;
  }
  if (!constraint->AddPointConstraint(t_knots.back(), ref_points.back().x(), ref_points.back().y())) {
    std::cout << "failed! to AddPointConstraint" << std::endl;
    return false;
  }
  if (!constraint->AddSecondDerivativeSmoothConstraint()) {
    std::cout << "failed to AddSecondDerivativeSmoothConstraint" << std::endl;
    return false;
  }
  return true;
}

void SplineSolver::AddKernel(){
  Spline2dKernel* kernel = spline_2d_solver_->mutable_kernel();

  kernel->AddSecondOrderDerivativeMatrix(200);
  kernel->AddThirdOrderDerivativeMatrix(1000);
  kernel->AddRegularization(1.0e-5);
}