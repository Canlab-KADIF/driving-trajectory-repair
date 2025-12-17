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

#include <gtest/gtest.h>

#include "bag_modifier/geometry/angle_math.h"
#include "bag_modifier/track/obstacle_pose_estimator.h"

namespace keti {
namespace kadif {
namespace bag_modifier {
namespace {

ObstacleSample MakeSample(int id, double timestamp, double x, double y,
                          double theta, double vx, double vy,
                          ObstacleClass obstacle_class) {
  ObstacleSample sample;
  sample.id = id;
  sample.obstacle_class = obstacle_class;
  sample.timestamp = timestamp;
  sample.position = Vec2d(x, y);
  sample.theta = theta;
  sample.velocity = Vec2d(vx, vy);
  sample.length = 4.5;
  sample.width = 1.9;
  return sample;
}

ObstaclePoseEstimator MakeEstimator(std::size_t history_size = 10) {
  ObstaclePoseEstimator estimator;
  ObstaclePoseEstimatorInitOptions options;
  options.history_size = history_size;
  EXPECT_TRUE(estimator.Init(options));
  return estimator;
}

TEST(ObstaclePoseEstimator, RejectsInvalidOptions) {
  ObstaclePoseEstimator estimator;
  ObstaclePoseEstimatorInitOptions options;
  options.history_size = 0;
  EXPECT_FALSE(estimator.Init(options));

  options.history_size = 5;
  options.stationary_speed_threshold = -1.0;
  EXPECT_FALSE(estimator.Init(options));
}

TEST(ObstaclePoseEstimator, FailsWithoutHistory) {
  ObstaclePoseEstimator estimator = MakeEstimator();
  const ObstacleSample sample =
      MakeSample(7, 1.0, 0.0, 0.0, 0.0, 10.0, 0.0, ObstacleClass::kVehicle);

  Box2d box;
  double speed = -1.0;
  EXPECT_FALSE(estimator.Estimate(sample, 2.0, &box, &speed));
  EXPECT_DOUBLE_EQ(speed, -1.0);
}

TEST(ObstaclePoseEstimator, KeepsHistoryBounded) {
  ObstaclePoseEstimator estimator = MakeEstimator(3);
  for (int i = 0; i < 10; ++i) {
    estimator.AddSample(MakeSample(1, i * 0.1, i * 1.0, 0.0, 0.0, 10.0, 0.0,
                                   ObstacleClass::kVehicle));
  }
  EXPECT_TRUE(estimator.HasHistory(1));
  estimator.ForgetTrack(1);
  EXPECT_FALSE(estimator.HasHistory(1));
}

TEST(ObstaclePoseEstimator, HoldsPoseOfAStationaryObstacle) {
  ObstaclePoseEstimator estimator = MakeEstimator();
  for (int i = 0; i < 5; ++i) {
    estimator.AddSample(MakeSample(3, i * 0.1, 12.0, -4.0, 0.5, 0.01, 0.0,
                                   ObstacleClass::kVehicle));
  }
  const ObstacleSample last =
      MakeSample(3, 0.4, 12.0, -4.0, 0.5, 0.01, 0.0, ObstacleClass::kVehicle);

  Box2d box;
  double speed = 0.0;
  ASSERT_TRUE(estimator.Estimate(last, 2.4, &box, &speed));

  // Two seconds later a parked car has not moved
  EXPECT_NEAR(box.center().x(), 12.0, 1e-9);
  EXPECT_NEAR(box.center().y(), -4.0, 1e-9);
  EXPECT_NEAR(box.heading(), 0.5, 1e-9);
  EXPECT_LT(speed, 0.83);
}

TEST(ObstaclePoseEstimator, PredictsPedestriansLinearly) {
  ObstaclePoseEstimator estimator = MakeEstimator();
  // A heading that rotates would drive the CTRV branch for a vehicle, but a
  // pedestrian must always fall back to the linear model.
  for (int i = 0; i < 5; ++i) {
    estimator.AddSample(MakeSample(4, i * 0.1, i * 0.15, 0.0, i * 0.2, 1.5, 0.0,
                                   ObstacleClass::kPedestrian));
  }
  const ObstacleSample last =
      MakeSample(4, 0.4, 0.6, 0.0, 0.8, 1.5, 0.0, ObstacleClass::kPedestrian);

  Box2d box;
  double speed = 0.0;
  ASSERT_TRUE(estimator.Estimate(last, 1.4, &box, &speed));

  EXPECT_NEAR(box.center().x(), 0.6 + 1.5 * 1.0, 1e-9);
  EXPECT_NEAR(box.center().y(), 0.0, 1e-9);
  EXPECT_NEAR(box.heading(), 0.8, 1e-9);
}

TEST(ObstaclePoseEstimator, PredictsStraightDrivingLinearly) {
  ObstaclePoseEstimator estimator = MakeEstimator();
  for (int i = 0; i < 5; ++i) {
    estimator.AddSample(MakeSample(5, i * 0.1, i * 1.0, 3.0, 0.0, 10.0, 0.0,
                                   ObstacleClass::kVehicle));
  }
  const ObstacleSample last =
      MakeSample(5, 0.4, 4.0, 3.0, 0.0, 10.0, 0.0, ObstacleClass::kVehicle);

  Box2d box;
  double speed = 0.0;
  ASSERT_TRUE(estimator.Estimate(last, 0.9, &box, &speed));

  EXPECT_NEAR(box.center().x(), 4.0 + 10.0 * 0.5, 1e-9);
  EXPECT_NEAR(box.center().y(), 3.0, 1e-9);
  EXPECT_NEAR(speed, 10.0, 1e-9);
}

TEST(ObstaclePoseEstimator, PredictsATurnAlongTheCtrvCircle) {
  // A vehicle driving a constant-radius arc: speed v, yaw rate w,
  // so the turn centre sits at distance R = v / w to the left of the vehicle.
  constexpr double kSpeed = 8.0;
  constexpr double kYawRate = 0.4;
  constexpr double kRadius = kSpeed / kYawRate;
  constexpr double kDt = 0.1;

  ObstaclePoseEstimator estimator = MakeEstimator();
  for (int i = 0; i < 5; ++i) {
    const double t = i * kDt;
    const double theta = kYawRate * t;
    // Position on a circle centred at (0, R), starting at the origin
    const double x = kRadius * std::sin(theta);
    const double y = kRadius * (1.0 - std::cos(theta));
    estimator.AddSample(MakeSample(6, t, x, y, theta, kSpeed * std::cos(theta),
                                   kSpeed * std::sin(theta),
                                   ObstacleClass::kVehicle));
  }

  const double last_t = 4 * kDt;
  const double last_theta = kYawRate * last_t;
  const ObstacleSample last =
      MakeSample(6, last_t, kRadius * std::sin(last_theta),
                 kRadius * (1.0 - std::cos(last_theta)), last_theta,
                 kSpeed * std::cos(last_theta), kSpeed * std::sin(last_theta),
                 ObstacleClass::kVehicle);

  Box2d box;
  double speed = 0.0;
  const double predict_at = last_t + 0.5;
  ASSERT_TRUE(estimator.Estimate(last, predict_at, &box, &speed));

  // The prediction must stay on the same circle and advance the heading
  const double distance_to_centre =
      std::hypot(box.center().x() - 0.0, box.center().y() - kRadius);
  EXPECT_NEAR(distance_to_centre, kRadius, 1e-6);
  EXPECT_NEAR(box.heading(), NormalizeAngle(kYawRate * predict_at), 1e-6);
  EXPECT_NEAR(speed, kSpeed, 1e-6);
}

}  // namespace
}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
