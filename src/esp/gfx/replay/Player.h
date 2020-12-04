// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Keyframe.h"

#include "esp/assets/Asset.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/scene/SceneNode.h"  // todo: forward decl
#include "esp/sensor/Sensor.h"

#include <rapidjson/document.h>

#include <string>

namespace esp {
namespace gfx {
namespace replay {

class Player {
 public:
  using LoadAndCreateRenderAssetInstanceCallback =
      std::function<esp::scene::SceneNode*(
          const esp::assets::AssetInfo&,
          const esp::assets::RenderAssetInstanceCreationInfo&)>;

  Player(const LoadAndCreateRenderAssetInstanceCallback& callback);

  void readKeyframesFromFile(const std::string& filepath);
  void readKeyframesFromJsonDocument(const rapidjson::Document& d);

  // returns -1 if no frame index set yet
  int getFrameIndex();
  int getNumFrames();
  void setFrame(int frameIndex);

  bool getUserTransform(const std::string& name,
                        Magnum::Vector3* translation,
                        Magnum::Quaternion* rotation);

 private:
  void clearFrame();  // todo: better name for setting to frame -1
  void applyKeyframe(const Keyframe& keyframe);

  LoadAndCreateRenderAssetInstanceCallback
      loadAndCreateRenderAssetInstanceCallback;
  int frameIndex_ = -1;
  std::vector<Keyframe> keyframes_;
  std::map<std::string, esp::assets::AssetInfo> assetInfos_;
  std::map<RenderAssetInstanceKey, scene::SceneNode*> createdInstances_;
  std::set<std::string> failedFilepaths_;

  ESP_SMART_POINTERS(Player)
};

}  // namespace replay
}  // namespace gfx
}  // namespace esp
