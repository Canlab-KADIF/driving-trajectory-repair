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

#include <iostream>
#include <string>

#include <ros/ros.h>

#include "driving_trajectory_repair/bag_rewriter.h"

namespace {

void PrintUsage(const char* program_name) {
  std::cerr
      << "Repairs broken object tracks in a recorded ROS bag.\n\n"
      << "usage: " << program_name << " <input.bag> [output_suffix]\n\n"
      << "  input.bag      recording to read, never modified\n"
      << "  output_suffix  appended to the output file names,"
      << " default \"_repaired\"\n\n"
      << "Writes <input><suffix>.bag with the repaired obstacle stream and\n"
      << "<input><suffix>_interpolated_only.bag with just the synthesised\n"
      << "poses. Tuning parameters are read from the private node handle;\n"
      << "see config/driving_trajectory_repair.yaml.\n";
}

// Reads a parameter from the private namespace, keeping the value already in
// `value` when the parameter is not set.
template <typename T>
void LoadParam(const ros::NodeHandle& node, const std::string& name, T* value) {
  T loaded;
  if (node.getParam(name, loaded)) {
    *value = loaded;
  }
}

void LoadOptions(
    const ros::NodeHandle& node,
    keti::kadif::driving_trajectory_repair::BagRewriterInitOptions* options) {
  LoadParam(node, "obstacle_topic", &options->obstacle_topic);
  LoadParam(node, "interpolated_topic", &options->interpolated_topic);
  LoadParam(node, "duplicate_frame_tolerance",
            &options->duplicate_frame_tolerance);

  LoadParam(node, "max_speed_error", &options->repairer.max_speed_error);
  LoadParam(node, "max_heading_error", &options->repairer.max_heading_error);
  LoadParam(node, "reidentification_window",
            &options->repairer.reidentification_window);
  LoadParam(node, "heading_similarity_weight",
            &options->repairer.heading_similarity_weight);
  LoadParam(node, "speed_similarity_weight",
            &options->repairer.speed_similarity_weight);

  LoadParam(node, "stationary_speed_threshold",
            &options->repairer.pose_estimator.stationary_speed_threshold);
  int history_size =
      static_cast<int>(options->repairer.pose_estimator.history_size);
  LoadParam(node, "history_size", &history_size);
  if (history_size > 0) {
    options->repairer.pose_estimator.history_size =
        static_cast<std::size_t>(history_size);
  }

  int segment_count = static_cast<int>(options->repairer.spline.segment_count);
  LoadParam(node, "spline_segment_count", &segment_count);
  if (segment_count > 0) {
    options->repairer.spline.segment_count =
        static_cast<std::size_t>(segment_count);
  }
  LoadParam(node, "spline_fit_weight", &options->repairer.spline.fit_weight);
  LoadParam(node, "spline_second_derivative_weight",
            &options->repairer.spline.second_derivative_weight);
  LoadParam(node, "spline_third_derivative_weight",
            &options->repairer.spline.third_derivative_weight);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "driving_trajectory_repair");

  // ros::init strips its own arguments, so the remaining ones are ours
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 1;
  }

  keti::kadif::driving_trajectory_repair::BagRewriterInitOptions options;
  options.input_bag_path = argv[1];
  const std::string suffix = (argc >= 3) ? argv[2] : "_repaired";

  options.repaired_bag_path =
      keti::kadif::driving_trajectory_repair::MakeOutputPath(
          options.input_bag_path, suffix);
  options.interpolated_bag_path =
      keti::kadif::driving_trajectory_repair::MakeOutputPath(
          options.input_bag_path, suffix + "_interpolated_only");

  ros::NodeHandle private_node("~");
  LoadOptions(private_node, &options);

  keti::kadif::driving_trajectory_repair::BagRewriter rewriter;
  if (!rewriter.Init(options)) {
    return 1;
  }

  ROS_INFO_STREAM("repairing " << options.input_bag_path);
  if (!rewriter.Run()) {
    ROS_ERROR_STREAM("failed to repair " << options.input_bag_path);
    return 1;
  }

  const auto& statistics = rewriter.statistics();
  ROS_INFO_STREAM("wrote " << options.repaired_bag_path << " and "
                           << options.interpolated_bag_path);
  ROS_INFO_STREAM("messages: " << statistics.total_messages
                               << ", obstacle frames: "
                               << statistics.obstacle_frames
                               << ", duplicate frames skipped: "
                               << statistics.duplicate_frames_skipped);
  ROS_INFO_STREAM("re-identified tracks: " << statistics.matched_tracks
                                           << ", interpolated poses: "
                                           << statistics.interpolated_poses);
  return 0;
}
