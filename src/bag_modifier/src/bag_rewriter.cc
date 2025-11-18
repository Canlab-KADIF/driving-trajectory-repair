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

#include "bag_modifier/bag_rewriter.h"

#include <rosbag/view.h>

#include <cmath>
#include <iostream>

#include "cyber_perception_msgs/PerceptionObstacles.h"

namespace keti {
namespace kadif {
namespace bag_modifier {

std::string MakeOutputPath(const std::string& input_path,
                           const std::string& suffix) {
  const std::string extension = ".bag";
  const std::size_t position = input_path.rfind(extension);
  if (position == std::string::npos ||
      position + extension.size() != input_path.size()) {
    return input_path + suffix + extension;
  }
  return input_path.substr(0, position) + suffix + extension;
}

BagRewriter::~BagRewriter() {
  if (initialized_) {
    input_bag_.close();
    repaired_bag_.close();
    interpolated_bag_.close();
  }
}

bool BagRewriter::Init(const BagRewriterInitOptions& options) {
  options_ = options;

  try {
    input_bag_.open(options_.input_bag_path, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& error) {
    std::cerr << "cannot read " << options_.input_bag_path << ": "
              << error.what() << std::endl;
    return false;
  }

  try {
    repaired_bag_.open(options_.repaired_bag_path, rosbag::bagmode::Write);
    interpolated_bag_.open(options_.interpolated_bag_path,
                           rosbag::bagmode::Write);
  } catch (const rosbag::BagException& error) {
    std::cerr << "cannot write the output bags: " << error.what() << std::endl;
    input_bag_.close();
    return false;
  }

  if (!repairer_.Init(options_.repairer)) {
    return false;
  }

  initialized_ = true;
  return true;
}

bool BagRewriter::Run() {
  if (!initialized_) {
    std::cerr << "BagRewriter::Init() was not called" << std::endl;
    return false;
  }
  if (!ScanInputBag()) {
    return false;
  }
  if (!repairer_.Repair()) {
    return false;
  }

  statistics_.matched_tracks = repairer_.matched_track_count();
  statistics_.interpolated_poses = repairer_.interpolated_pose_count();

  return WriteOutputBags();
}

bool BagRewriter::ScanInputBag() {
  rosbag::View view(input_bag_);

  double previous_timestamp = 0.0;

  for (const rosbag::MessageInstance& message : view) {
    ++statistics_.total_messages;

    if (message.getTopic() != options_.obstacle_topic) {
      // Everything that is not the repaired stream is copied verbatim, so the
      // output bag stays a drop-in replacement for the recording.
      repaired_bag_.write(message.getTopic(), message.getTime(), message);
      continue;
    }

    const cyber_perception_msgs::PerceptionObstacles::ConstPtr obstacles =
        message.instantiate<cyber_perception_msgs::PerceptionObstacles>();
    if (obstacles == nullptr) {
      std::cerr << "message on " << options_.obstacle_topic
                << " is not a PerceptionObstacles, copying it unchanged"
                << std::endl;
      repaired_bag_.write(message.getTopic(), message.getTime(), message);
      continue;
    }

    const double timestamp = obstacles->cyber_header.timestamp_sec;
    if (timestamp - previous_timestamp <= options_.duplicate_frame_tolerance) {
      ++statistics_.duplicate_frames_skipped;
      continue;
    }
    previous_timestamp = timestamp;

    if (!repairer_.AddFrame(message.getTime(), *obstacles)) {
      return false;
    }
    ++statistics_.obstacle_frames;
  }

  return true;
}

bool BagRewriter::WriteOutputBags() {
  const std::vector<TimedObstacles>& repaired = repairer_.repaired_frames();
  const std::vector<TimedObstacles>& interpolated =
      repairer_.interpolated_frames();

  for (std::size_t i = 0; i < repaired.size(); ++i) {
    repaired_bag_.write(options_.obstacle_topic, repaired[i].stamp,
                        repaired[i].obstacles);
    interpolated_bag_.write(options_.interpolated_topic, interpolated[i].stamp,
                            interpolated[i].obstacles);
  }
  return true;
}

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
