// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Player.h"
#include "Recorder.h"

namespace esp {
namespace gfx {
namespace replay {

class ReplayManager {
 public:
  void setRecorder(const std::shared_ptr<Recorder>& writer) {
    recorder_ = writer;
  }

  // may return nullptr if no active recorder
  std::shared_ptr<Recorder> getRecorder() const { return recorder_; }

  void setPlayerCallback(
      const Player::LoadAndCreateRenderAssetInstanceCallback& callback) {
    playerCallback_ = callback;
  }

  std::shared_ptr<Player> readKeyframesFromFile(const std::string& filepath);

 private:
  std::shared_ptr<Recorder> recorder_;
  Player::LoadAndCreateRenderAssetInstanceCallback playerCallback_;

  ESP_SMART_POINTERS(ReplayManager)
};

}  // namespace replay
}  // namespace gfx
}  // namespace esp
