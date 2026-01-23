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
#include <memory>
#include <string>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>

#include "bag_modifier/track/track_repairer.h"

namespace keti {
namespace kadif {
namespace bag_modifier {

struct BagRewriterInitOptions {
  // Directory of the recording to read. Never modified.
  std::string input_bag_uri;

  // Every recorded topic is copied through unchanged; this one is repaired.
  std::string obstacle_topic = "/obstacles";

  // Output bag holding all original topics plus the repaired obstacle stream.
  std::string repaired_bag_uri;
  // Output bag holding only the synthesised obstacles, for inspection.
  std::string interpolated_bag_uri;
  std::string interpolated_topic = "/obstacles_modified";

  // Storage plugin for the outputs. Left empty to follow the default, which is
  // mcap on Jazzy and sqlite3 on Humble.
  std::string storage_id;

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
//
// Topics other than the repaired one are copied as serialised bytes without
// being deserialised, so the tool does not need their message definitions to
// be available and passes them through byte for byte.
class BagRewriter {
 public:
  BagRewriter() = default;
  ~BagRewriter() = default;

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
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::unique_ptr<rosbag2_cpp::Writer> repaired_writer_;
  std::unique_ptr<rosbag2_cpp::Writer> interpolated_writer_;
  TrackRepairer repairer_;
  BagRewriterStatistics statistics_;
  bool initialized_ = false;
};

// Derives an output bag uri from the input one by appending a suffix.
std::string MakeOutputUri(const std::string& input_uri,
                          const std::string& suffix);

}  // namespace bag_modifier
}  // namespace kadif
}  // namespace keti
