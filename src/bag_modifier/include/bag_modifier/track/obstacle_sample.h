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

#include "bag_modifier/geometry/box2d.h"
#include "bag_modifier/geometry/vec2d.h"

namespace keti {
namespace kadif {
namespace bag_modifier {

// Coarse obstacle class. Only the distinctions that change the motion model are
// represented here; the full perception taxonomy stays in the message layer.
enum class ObstacleClass {
  kUnknown = 0,
  kPedestrian,
  kVehicle,
  kBicycle,
  kOther,
};

// Framework-free snapshot of one perceived obstacle at one instant.
//
// The tracking and interpolation logic operates on this type rather than on
// ROS messages, so the algorithm can be unit tested without a ROS graph and
// stays reusable when the message definitions change.
struct ObstacleSample {
  int id = -1;
  ObstacleClass obstacle_class = ObstacleClass::kUnknown;
  double timestamp = 0.0;

  Vec2d position;
  double theta = 0.0;

  Vec2d velocity;

  double length = 0.0;
  double width = 0.0;
  double height = 0.0;

  Box2d ToBox() const { return Box2d(position, theta, length, width); }
  double Speed() const { return velocity.Length(); }
};

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
