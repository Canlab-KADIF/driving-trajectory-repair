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

#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "bag_modifier/bag_rewriter.h"

namespace {

void PrintUsage(const char* program_name) {
  std::cerr
      << "Repairs broken object tracks in a recorded ROS 2 bag.\n\n"
      << "usage: " << program_name << " <input_bag> [output_suffix]"
      << " [--ros-args ...]\n\n"
      << "  input_bag      recording to read, never modified\n"
      << "  output_suffix  appended to the output bag names,"
      << " default \"_repaired\"\n\n"
      << "Writes <input><suffix> with the repaired obstacle stream and\n"
      << "<input><suffix>_interpolated_only with just the synthesised poses.\n"
      << "Tuning parameters are node parameters; see config/bag_modifier.yaml\n"
      << "and pass them with --ros-args --params-file <file>.\n";
}

// Reads every tuning parameter, declaring each with the value the struct
// already holds so that an unset parameter keeps the built-in default.
void DeclareOptions(rclcpp::Node* node,
                    keti::kadif::bag_modifier::BagRewriterInitOptions* out) {
  auto& repairer = out->repairer;
  auto& estimator = repairer.pose_estimator;
  auto& spline = repairer.spline;

  out->obstacle_topic =
      node->declare_parameter("obstacle_topic", out->obstacle_topic);
  out->interpolated_topic =
      node->declare_parameter("interpolated_topic", out->interpolated_topic);
  out->storage_id = node->declare_parameter("storage_id", out->storage_id);
  out->duplicate_frame_tolerance = node->declare_parameter(
      "duplicate_frame_tolerance", out->duplicate_frame_tolerance);

  repairer.max_speed_error =
      node->declare_parameter("max_speed_error", repairer.max_speed_error);
  repairer.max_heading_error =
      node->declare_parameter("max_heading_error", repairer.max_heading_error);
  repairer.reidentification_window = node->declare_parameter(
      "reidentification_window", repairer.reidentification_window);
  repairer.heading_similarity_weight = node->declare_parameter(
      "heading_similarity_weight", repairer.heading_similarity_weight);
  repairer.speed_similarity_weight = node->declare_parameter(
      "speed_similarity_weight", repairer.speed_similarity_weight);

  estimator.stationary_speed_threshold = node->declare_parameter(
      "stationary_speed_threshold", estimator.stationary_speed_threshold);
  const int history_size = node->declare_parameter(
      "history_size", static_cast<int>(estimator.history_size));
  if (history_size > 0) {
    estimator.history_size = static_cast<std::size_t>(history_size);
  }

  const int segment_count = node->declare_parameter(
      "spline_segment_count", static_cast<int>(spline.segment_count));
  if (segment_count > 0) {
    spline.segment_count = static_cast<std::size_t>(segment_count);
  }
  spline.fit_weight =
      node->declare_parameter("spline_fit_weight", spline.fit_weight);
  spline.second_derivative_weight = node->declare_parameter(
      "spline_second_derivative_weight", spline.second_derivative_weight);
  spline.third_derivative_weight = node->declare_parameter(
      "spline_third_derivative_weight", spline.third_derivative_weight);
}

}  // namespace

int main(int argc, char** argv) {
  // Strips --ros-args and everything after it, leaving our own arguments
  const std::vector<std::string> arguments =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  if (arguments.size() < 2) {
    PrintUsage(argv[0]);
    rclcpp::shutdown();
    return 1;
  }

  keti::kadif::bag_modifier::BagRewriterInitOptions options;
  options.input_bag_uri = arguments[1];
  const std::string suffix =
      (arguments.size() >= 3) ? arguments[2] : "_repaired";

  options.repaired_bag_uri =
      keti::kadif::bag_modifier::MakeOutputUri(options.input_bag_uri, suffix);
  options.interpolated_bag_uri = keti::kadif::bag_modifier::MakeOutputUri(
      options.input_bag_uri, suffix + "_interpolated_only");

  const auto node = std::make_shared<rclcpp::Node>("bag_modifier");
  DeclareOptions(node.get(), &options);

  keti::kadif::bag_modifier::BagRewriter rewriter;
  if (!rewriter.Init(options)) {
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "repairing " << options.input_bag_uri);
  if (!rewriter.Run()) {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                        "failed to repair " << options.input_bag_uri);
    rclcpp::shutdown();
    return 1;
  }

  const auto& statistics = rewriter.statistics();
  RCLCPP_INFO_STREAM(node->get_logger(),
                     "wrote " << options.repaired_bag_uri << " and "
                              << options.interpolated_bag_uri);
  RCLCPP_INFO_STREAM(
      node->get_logger(),
      "messages: " << statistics.total_messages
                   << ", obstacle frames: " << statistics.obstacle_frames
                   << ", duplicate frames skipped: "
                   << statistics.duplicate_frames_skipped);
  RCLCPP_INFO_STREAM(node->get_logger(),
                     "re-identified tracks: " << statistics.matched_tracks
                                              << ", interpolated poses: "
                                              << statistics.interpolated_poses);
  rclcpp::shutdown();
  return 0;
}
