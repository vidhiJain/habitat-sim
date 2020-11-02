// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "RenderKeyframe.h"

#include "esp/assets/Asset.h"
#include "esp/assets/RenderAssetInstanceCreation.h"
#include "esp/scene/SceneNode.h"  // todo: forward decl
#include "esp/sensor/Sensor.h"
#include "esp/sensor/VisualSensor.h"

#include <rapidjson/document.h>

#include <string>

namespace esp {
namespace gfx {

class RenderKeyframeReader {
 public:
  using LoadAndAddRenderAssetInstanceCallback =
      std::function<esp::scene::SceneNode*(
          const esp::assets::AssetInfo&,
          const esp::assets::RenderAssetInstanceCreation&)>;

  RenderKeyframeReader(const LoadAndAddRenderAssetInstanceCallback& callback);

  void readKeyframesFromFile(const std::string& filepath);
  void readKeyframesFromJsonDocument(const rapidjson::Document& d);

  // returns -1 if no frame index set yet
  int getFrameIndex();
  int getNumFrames();
  void setFrame(int frameIndex);

 private:
  void clearFrame();  // todo: better name for setting to frame -1
  void applyKeyframe(const RenderKeyframe& keyframe);

  LoadAndAddRenderAssetInstanceCallback loadAndAddRenderAssetInstanceCallback;
  int frameIndex_ = -1;
  std::vector<RenderKeyframe> keyframes_;
  std::map<std::string, esp::assets::AssetInfo> assetInfos_;
  std::map<RenderAssetInstanceKey, scene::SceneNode*> createdInstances_;
};

}  // namespace gfx
}  // namespace esp
