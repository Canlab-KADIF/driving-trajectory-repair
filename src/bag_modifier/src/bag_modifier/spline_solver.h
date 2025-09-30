#include <vector>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include "math/smoothing_spline/osqp_spline_2d_solver.h"
#include "math/curve_math.h"
#include "math/math_utils.h"
#include <memory>

#define ORDER 5

using namespace keti::common::math;
using namespace keti::planning;

struct SplineControlPoint{
  SplineControlPoint() = default;
  SplineControlPoint(Vec2d control_point, double s, double angle)
    : control_point(control_point), s(s), angle(angle) {}

  Vec2d control_point;
  double s;
  double angle;
};

class SplineSolver{
public:
  SplineSolver();
  void SetSplineControlPoints(std::vector<SplineControlPoint> spline_control_points) {spline_control_points_ = spline_control_points;}
  bool Solve();
  Vec2d operator()(const double t);

private:
  bool AddConstraint();
  void AddKernel();

private:
  std::unique_ptr<Spline2dSolver> spline_2d_solver_;
  std::vector<SplineControlPoint> spline_control_points_;
};