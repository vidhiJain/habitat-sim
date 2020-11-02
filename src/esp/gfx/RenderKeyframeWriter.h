// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "RenderKeyframe.h"

#include "esp/assets/Asset.h"                        // todo: forward decl
#include "esp/assets/RenderAssetInstanceCreation.h"  // todo: forward decl
#include "esp/scene/SceneNode.h"                     // todo: forward decl
#include "esp/sensor/Sensor.h"
#include "esp/sensor/VisualSensor.h"

#include <rapidjson/document.h>

#include <string>

namespace esp {
namespace gfx {

class NodeDeletionHelper;

class RenderKeyframeWriter {
 public:
  void onCreateRenderAssetInstance(
      scene::SceneNode* node,
      const esp::assets::RenderAssetInstanceCreation& creation);

  void onLoadRenderAsset(const esp::assets::AssetInfo& assetInfo);

  void onDrawObservation(const sensor::VisualSensor& visualSensor);

  void writeKeyframesToFile(const std::string& filepath);
  rapidjson::Document writeKeyframesToJsonDocument();

 private:
  // NodeDeletionHelper calls onDeleteRenderAssetInstance
  friend class NodeDeletionHelper;

  void onDeleteRenderAssetInstance(const scene::SceneNode* node);
  RenderKeyframe& getKeyframe();
  void advanceKeyframe();
  RenderAssetInstanceKey getNewInstanceKey();
  int findInstance(const scene::SceneNode* queryNode);
  RenderAssetInstanceState getInstanceState(const scene::SceneNode* node);
  void updateInstanceStates();

  std::vector<RenderAssetInstanceRecord> instanceRecords_;
  std::vector<RenderKeyframe> keyframes_;
  RenderAssetInstanceKey nextInstanceKey_ = 0;
};

}  // namespace gfx
}  // namespace esp
