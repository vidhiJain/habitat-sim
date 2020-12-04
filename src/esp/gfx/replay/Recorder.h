// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Keyframe.h"

#include "esp/assets/Asset.h"                            // todo: forward decl
#include "esp/assets/RenderAssetInstanceCreationInfo.h"  // todo: forward decl
#include "esp/scene/SceneNode.h"                         // todo: forward decl
#include "esp/sensor/Sensor.h"
#include "esp/sensor/VisualSensor.h"

#include <rapidjson/document.h>

#include <string>

namespace esp {
namespace gfx {
namespace replay {

class NodeDeletionHelper;

class Recorder {
 public:
  ~Recorder();
  void onCreateRenderAssetInstance(
      scene::SceneNode* node,
      const esp::assets::RenderAssetInstanceCreationInfo& creation);

  void onLoadRenderAsset(const esp::assets::AssetInfo& assetInfo);

  void saveKeyframe();

  void addUserTransformToKeyframe(const std::string& name,
                                  const Magnum::Vector3& translation,
                                  const Magnum::Quaternion& rotation);

  void writeSavedKeyframesToFile(const std::string& filepath);
  rapidjson::Document writeKeyframesToJsonDocument();

 private:
  // NodeDeletionHelper calls onDeleteRenderAssetInstance
  friend class NodeDeletionHelper;

  using KeyframeIterator = std::vector<Keyframe>::const_iterator;

  void onDeleteRenderAssetInstance(const scene::SceneNode* node);
  Keyframe& getKeyframe();
  void advanceKeyframe();
  RenderAssetInstanceKey getNewInstanceKey();
  int findInstance(const scene::SceneNode* queryNode);
  RenderAssetInstanceState getInstanceState(const scene::SceneNode* node);
  void updateInstanceStates();
  void checkAndAddDeletion(Keyframe* keyframe,
                           RenderAssetInstanceKey instanceKey);
  void addLoadsCreationsDeletions(KeyframeIterator begin,
                                  KeyframeIterator end,
                                  Keyframe* dest);

  std::vector<RenderAssetInstanceRecord> instanceRecords_;
  Keyframe currKeyframe_;
  std::vector<Keyframe> savedKeyframes_;
  RenderAssetInstanceKey nextInstanceKey_ = 0;
};

}  // namespace replay
}  // namespace gfx
}  // namespace esp
