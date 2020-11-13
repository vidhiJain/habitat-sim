// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderReplayManager.h"

using namespace rapidjson;

namespace esp {
namespace gfx {

#if 0
void RenderReplayManager::saveKeyframe() {
  if (!writer_) {
    LOG(ERROR) << "RenderReplayManager::saveKeyframe: not enabled. See SimulatorConfiguration::enableRenderReplaySave.";
    return;
  }
  writer_->saveKeyframe();
}

void RenderReplayManager::addUserTransformToKeyframe(
    const std::string& name,
    const Magnum::Vector3& translation,
    const Magnum::Quaternion& rotation,
    const Magnum::Vector3& scaling) {
  if (!writer_) {
    LOG(ERROR) << "RenderReplayManager::addUserObjectToKeyframe: not enabled. See SimulatorConfiguration::enableRenderReplaySave.";
    return;
  }
  writer_->addUserTransformToKeyframe(name, translation, rotation, scaling);
}

void RenderReplayManager::writeKeyframesToFile(const std::string& filepath) {
  if (!writer_) {
    LOG(ERROR) << "RenderReplayManager::writeKeyframesToFile: not enabled. See SimulatorConfiguration::enableRenderReplaySave.";
    return;
  }
  writer_->writeSavedKeyframesToFile(filepath);
}

void RenderReplayManager::readKeyframesFromFile(const std::string& filepath) {
  reader_->readKeyframesFromFile(filepath);
}

// returns -1 if no frame index set yet
int RenderReplayManager::getFrameIndex() {
  // todo: error handling
  return reader_->getFrameIndex();
}

int RenderReplayManager::getNumFrames() {
  // todo: error handling
  return reader_->getNumFrames();
}

void RenderReplayManager::setFrame(int frameIndex) {
  // todo: error handling
  reader_->setFrame(frameIndex);
}
#endif

}  // namespace gfx
}  // namespace esp
