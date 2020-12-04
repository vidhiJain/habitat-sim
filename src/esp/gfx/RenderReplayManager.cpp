// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderReplayManager.h"

namespace esp {
namespace gfx {

std::shared_ptr<RenderKeyframeReader> RenderReplayManager::readKeyframesFromFile(const std::string& filepath) {

  auto reader = std::make_shared<RenderKeyframeReader>(readerCallback_);
  reader->readKeyframesFromFile(filepath);
  if (!reader->getNumFrames()) {
    LOG(ERROR) << "player_load_from_file: failed to load any keyframes from [" << filepath << "]";
    return nullptr;
  }
  return reader;
}

}  // namespace gfx
}  // namespace esp
