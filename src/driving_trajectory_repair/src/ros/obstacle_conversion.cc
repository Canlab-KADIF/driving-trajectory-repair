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

#include "driving_trajectory_repair/ros/obstacle_conversion.h"

#include "cyber_perception_msgs/ObstacleType.h"

namespace keti {
namespace kadif {
namespace driving_trajectory_repair {

ObstacleClass ToObstacleClass(int message_type) {
  switch (message_type) {
    case cyber_perception_msgs::ObstacleType::PEDESTRIAN:
      return ObstacleClass::kPedestrian;
    case cyber_perception_msgs::ObstacleType::VEHICLE:
      return ObstacleClass::kVehicle;
    case cyber_perception_msgs::ObstacleType::BICYCLE:
      return ObstacleClass::kBicycle;
    case cyber_perception_msgs::ObstacleType::UNKNOWN:
      return ObstacleClass::kUnknown;
    default:
      return ObstacleClass::kOther;
  }
}

ObstacleSample ToObstacleSample(
    const cyber_perception_msgs::PerceptionObstacle& message,
    double timestamp) {
  ObstacleSample sample;
  sample.id = message.id;
  sample.obstacle_class = ToObstacleClass(message.type.type);
  sample.timestamp = timestamp;
  sample.position = Vec2d(message.position.x, message.position.y);
  sample.theta = message.theta;
  sample.velocity = Vec2d(message.velocity.x, message.velocity.y);
  sample.length = message.length;
  sample.width = message.width;
  sample.height = message.height;
  return sample;
}

}  // namespace driving_trajectory_repair
}  // namespace kadif
}  // namespace keti
