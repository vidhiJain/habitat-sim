// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// todo: forward decl?
#include "RenderKeyframeWriter.h"
#include "RenderKeyframeReader.h"

namespace esp {
namespace gfx {

class RenderReplayManager {
public:
  void setWriter(const std::shared_ptr<RenderKeyframeWriter>& writer) {
    writer_ = writer;
  }
  std::shared_ptr<RenderKeyframeWriter> getWriter() const { return writer_; }

  void setReader(const std::shared_ptr<RenderKeyframeReader>& reader) {
    reader_ = reader;
  }
  std::shared_ptr<RenderKeyframeReader> getReader() const { return reader_; }

#if 0
  void saveKeyframe();
  // todo: switch to Mat4?
  void addUserTransformToKeyframe(const std::string& name,
    const Magnum::Vector3& translation,
    const Magnum::Quaternion& rotation,
    const Magnum::Vector3& scaling = Magnum::Vector3{1.0, 1.0, 1.0});
  void writeKeyframesToFile(const std::string& filepath);

  void readKeyframesFromFile(const std::string& filepath);
  // returns -1 if no frame index set yet
  int getFrameIndex();
  int getNumFrames();
  void setFrame(int frameIndex);
#endif

private:
  std::shared_ptr<RenderKeyframeWriter> writer_;
  std::shared_ptr<RenderKeyframeReader> reader_;

  ESP_SMART_POINTERS(RenderReplayManager)
};



}  // namespace gfx
}  // namespace esp
