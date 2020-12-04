// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// todo: forward decl?
#include "Recorder.h"
#include "RenderKeyframeReader.h"

namespace esp {
namespace gfx {

class RenderReplayManager {
public:
  void setWriter(const std::shared_ptr<Recorder>& writer) {
    writer_ = writer;
  }
  std::shared_ptr<Recorder> getWriter() const { return writer_; }

  void setReaderCallback(const RenderKeyframeReader::LoadAndCreateRenderAssetInstanceCallback& callback) {
    readerCallback_ = callback;
  }

  std::shared_ptr<RenderKeyframeReader> readKeyframesFromFile(const std::string& filepath);

private:
  std::shared_ptr<Recorder> writer_;
  RenderKeyframeReader::LoadAndCreateRenderAssetInstanceCallback readerCallback_;

  ESP_SMART_POINTERS(RenderReplayManager)
};



}  // namespace gfx
}  // namespace esp
