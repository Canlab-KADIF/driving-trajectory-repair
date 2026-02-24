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

#include <cstddef>
#include <string>

#include <rosbag/bag.h>

#include "driving_trajectory_repair/track/track_repairer.h"

namespace keti {
namespace kadif {
namespace driving_trajectory_repair {

struct BagRewriterInitOptions {
  std::string input_bag_path;

  // Every recorded topic is copied through unchanged; this one is repaired.
  std::string obstacle_topic = "/obstacles";

  // Output bag holding all original topics plus the repaired obstacle stream.
  std::string repaired_bag_path;
  // Output bag holding only the synthesised obstacles, for inspection.
  std::string interpolated_bag_path;
  std::string interpolated_topic = "/obstacles_modified";

  // Consecutive frames whose perception timestamps differ by less than this are
  // duplicates of the same detection cycle, which the recorder occasionally
  // writes twice. Only the first is kept.
  double duplicate_frame_tolerance = 1e-5;

  TrackRepairerInitOptions repairer;
};

struct BagRewriterStatistics {
  std::size_t total_messages = 0;
  std::size_t obstacle_frames = 0;
  std::size_t duplicate_frames_skipped = 0;
  std::size_t matched_tracks = 0;
  std::size_t interpolated_poses = 0;
};

// Reads a recorded bag, repairs the object-tracking stream in it, and writes
// two new bags. The input bag is never modified.
class BagRewriter {
 public:
  BagRewriter() = default;
  ~BagRewriter();

  BagRewriter(const BagRewriter&) = delete;
  BagRewriter& operator=(const BagRewriter&) = delete;

  // Opens the input and output bags. Returns false if any of them cannot be
  // opened, so the caller does not proceed against a closed handle.
  bool Init(const BagRewriterInitOptions& options);

  // Runs the whole pipeline: scan, repair, write.
  bool Run();

  const BagRewriterStatistics& statistics() const { return statistics_; }

 private:
  bool ScanInputBag();
  bool WriteOutputBags();

  BagRewriterInitOptions options_;
  rosbag::Bag input_bag_;
  rosbag::Bag repaired_bag_;
  rosbag::Bag interpolated_bag_;
  TrackRepairer repairer_;
  BagRewriterStatistics statistics_;
  bool initialized_ = false;
};

// Derives the two output paths from the input path by appending suffixes
// before the .bag extension.
std::string MakeOutputPath(const std::string& input_path,
                           const std::string& suffix);

}  // namespace driving_trajectory_repair
}  // namespace kadif
}  // namespace keti
