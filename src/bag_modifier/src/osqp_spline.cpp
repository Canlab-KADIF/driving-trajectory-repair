#include "math/smoothing_spline/osqp_spline_2d_solver.h"

#include <chrono>

// #include "gtest/gtest.h"
#include "math/curve_math.h"

using namespace keti::planning;
using keti::common::math::Vec2d;
using Eigen::MatrixXd;

int main() {
  std::vector<double> t_knots{0.0, 1.0};
  uint32_t order = 5;
  OsqpSpline2dSolver spline_solver(t_knots, order);

  Spline2dConstraint* constraint = spline_solver.mutable_constraint();
  Spline2dKernel* kernel = spline_solver.mutable_kernel();

  std::vector<double> et;
  et.emplace_back(0.0);
  et.emplace_back(1.0);

  std::vector<double> bound(2, 0.001);
  std::vector<std::vector<double>> constraint_data{
      {0.0467834, 332950.792155, 4140495.49493},
      {0.3776341, 332958.6773986, 4140498.1915200}};
  std::vector<double> angle;
  std::vector<Vec2d> ref_point;

  double ref_x = 332950.792155;
  double ref_y = 4140495.49493;

  for (size_t i = 0; i < 2; ++i) {
    angle.push_back(constraint_data[i][0]);
    Vec2d prev_point(constraint_data[i][1] - ref_x, constraint_data[i][2] - ref_y);

    Vec2d new_point = prev_point;
    ref_point.emplace_back(new_point.x(), new_point.y());
  }
  std::cout << "flag" << std::endl;
  if ( !constraint->Add2dBoundary(t_knots, angle, ref_point, bound, bound) ){
    std::cout << "Failed to Add2dBoundary!" << std::endl;
    return 0;
  }

  std::cout << "flag" << std::endl;

  if (!constraint->AddPointAngleConstraint(t_knots.front(),
                                           angle.front())) {
    std::cout << "failed!" << std::endl;
    return false;
  }

  if (!constraint->AddPointAngleConstraint(t_knots.back(),
                                           angle.back())) {
    std::cout << "failed!2" << std::endl;
    return false;
  }

  if (!constraint->AddSecondDerivativeSmoothConstraint()) {
    return false;
  }

  kernel->AddSecondOrderDerivativeMatrix(200);
  // kernel->add_second_order_derivative_matrix(100);
  // kernel->add_derivative_kernel_matrix(100);

  kernel->AddThirdOrderDerivativeMatrix(1000);

  kernel->AddRegularization(1.0e-5);
  // constraint->add_point_angle_constraint(0, -1.21);
  // TODO(all): fix the test.

  auto start = std::chrono::system_clock::now();
  if ( !spline_solver.Solve() ){
    std::cout << "Failed to solve spline_solver!" << std::endl;
    return 0;
  }

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end - start;
  std::cout << "Time to solver is " << diff.count() << " s\n";

  double t = 0;
  for (int i = 0; i < 101; ++i) {
    auto xy = spline_solver.spline()(t);
    // if ( (xy.first -  gold_res(i, 1)) > std::fmax(3e-3, gold_res(i, 1) * 1e-4) ) {
    //   std::cout << "Solved result is very far from original points." << std::endl;
    //   return 0;
    // }
                
    // if ( (xy.second - gold_res(i, 2)) > std::fmax(3e-3, gold_res(i, 2) * 1e-4) ) {
    //   std::cout << "Solved result is very far from original points." << std::endl;
    //   return 0;
    // }
    t += 0.01;

    std::cout.precision(5);
    std::cout << std::fixed << xy.first + ref_x << "," << xy.second + ref_y << std::endl;
  }

  std::cout << "=====================================" << std::endl;
  for ( auto& p: ref_point )
    std::cout << p.x() << "," << p.y() << std::endl;

  return 0;
}