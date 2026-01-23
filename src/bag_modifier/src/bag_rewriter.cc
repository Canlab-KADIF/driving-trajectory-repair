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

#include <cmath>
#include <iostream>
#include <stdexcept>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include "cyber_perception_msgs/msg/perception_obstacles.hpp"

namespace keti {
namespace kadif {
namespace bag_modifier {
namespace {

const char* const kObstacleMessageType =
    "cyber_perception_msgs/msg/PerceptionObstacles";

// Strips a trailing path separator so that a uri given as "recording/" and one
// given as "recording" produce the same output names.
std::string StripTrailingSlash(const std::string& uri) {
  if (uri.size() > 1 && uri.back() == '/') {
    return uri.substr(0, uri.size() - 1);
  }
  return uri;
}

}  // namespace

std::string MakeOutputUri(const std::string& input_uri,
                          const std::string& suffix) {
  return StripTrailingSlash(input_uri) + suffix;
}

bool BagRewriter::Init(const BagRewriterInitOptions& options) {
  options_ = options;

  reader_ = std::make_unique<rosbag2_cpp::Reader>();
  try {
    reader_->open(options_.input_bag_uri);
  } catch (const std::exception& error) {
    std::cerr << "cannot read " << options_.input_bag_uri << ": "
              << error.what() << std::endl;
    return false;
  }

  const auto open_writer = [this](const std::string& uri) {
    auto writer = std::make_unique<rosbag2_cpp::Writer>();
    if (options_.storage_id.empty()) {
      writer->open(uri);
    } else {
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = uri;
      storage_options.storage_id = options_.storage_id;
      writer->open(storage_options, {});
    }
    return writer;
  };

  try {
    repaired_writer_ = open_writer(options_.repaired_bag_uri);
    interpolated_writer_ = open_writer(options_.interpolated_bag_uri);
  } catch (const std::exception& error) {
    std::cerr << "cannot write the output bags: " << error.what() << std::endl;
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
  // Recreate every input topic on the repaired output, so that topics which
  // are copied through keep their type and serialisation format.
  for (const rosbag2_storage::TopicMetadata& topic :
       reader_->get_all_topics_and_types()) {
    repaired_writer_->create_topic(topic);
  }

  rclcpp::Serialization<cyber_perception_msgs::msg::PerceptionObstacles>
      serialization;
  double previous_timestamp = 0.0;

  while (reader_->has_next()) {
    const auto message = reader_->read_next();
    ++statistics_.total_messages;

    if (message->topic_name != options_.obstacle_topic) {
      // Copied as bytes: the message definition does not have to be available
      repaired_writer_->write(message);
      continue;
    }

    rclcpp::SerializedMessage serialized(*message->serialized_data);
    cyber_perception_msgs::msg::PerceptionObstacles obstacles;
    try {
      serialization.deserialize_message(&serialized, &obstacles);
    } catch (const std::exception& error) {
      std::cerr << "cannot deserialise a message on " << options_.obstacle_topic
                << ", copying it unchanged: " << error.what() << std::endl;
      repaired_writer_->write(message);
      continue;
    }

    const double timestamp = obstacles.cyber_header.timestamp_sec;
    if (timestamp - previous_timestamp <= options_.duplicate_frame_tolerance) {
      ++statistics_.duplicate_frames_skipped;
      continue;
    }
    previous_timestamp = timestamp;

    if (!repairer_.AddFrame(message->recv_timestamp, obstacles)) {
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

  interpolated_writer_->create_topic(rosbag2_storage::TopicMetadata{
      0, options_.interpolated_topic, kObstacleMessageType, "cdr", {}, ""});

  for (std::size_t i = 0; i < repaired.size(); ++i) {
    repaired_writer_->write(repaired[i].obstacles, options_.obstacle_topic,
                            rclcpp::Time(repaired[i].stamp_ns));
    interpolated_writer_->write(interpolated[i].obstacles,
                                options_.interpolated_topic,
                                rclcpp::Time(interpolated[i].stamp_ns));
  }
  return true;
}

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
