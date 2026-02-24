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

#include "cyber_perception_msgs/msg/perception_obstacle.hpp"
#include "driving_trajectory_repair/track/obstacle_sample.h"

namespace keti {
namespace kadif {
namespace driving_trajectory_repair {

// Boundary between the perception message definitions and the algorithm types.
//
// Everything below include/driving_trajectory_repair/geometry and .../track is
// free of ROS includes; this header is the single place where the two meet.
// Swapping the message package therefore only requires editing this file.

// Maps the perception taxonomy onto the coarse classes the motion models need.
ObstacleClass ToObstacleClass(int message_type);

// `timestamp` comes from the enclosing PerceptionObstacles header rather than
// the obstacle itself, because the per-obstacle stamp is not always populated.
ObstacleSample ToObstacleSample(
    const cyber_perception_msgs::msg::PerceptionObstacle& message,
    double timestamp);

}  // namespace driving_trajectory_repair
}  // namespace kadif
}  // namespace keti
