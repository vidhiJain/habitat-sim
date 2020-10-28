// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/assets/Asset.h"
#include "esp/scene/SceneNode.h"  // todo: forward decl
#include "esp/sensor/Sensor.h"
#include "esp/sensor/VisualSensor.h"

#include <Corrade/Containers/Optional.h>
#include "Magnum/Resource.h"

#include <string>

namespace esp {
namespace gfx {

// todo: put all these structs in a replay/keyframe namespace

class NodeDeletionHelper;

using RenderAssetInstanceKey = uint32_t;

// to serialize creation event
struct RenderAssetInstanceCreation {
  RenderAssetInstanceKey instanceKey;
  esp::assets::AssetInfo assetInfo;
  bool isSemantic;
  bool isRGBD;
  Magnum::ResourceKey lightSetupKey;
};

struct RenderAssetInstanceState {
  Magnum::Matrix4 absTransform;  // todo: Matrix4 or Matrix4x4?
  // todo: support semanticId per drawable?
  int semanticId = -1;
  // todo: support mutable lightSetupKey?

  bool operator==(const RenderAssetInstanceState& rhs) const {
    return absTransform == rhs.absTransform && semanticId == rhs.semanticId;
  }
};

// to serialize drawObservation event
struct ObservationRecord {
  esp::sensor::SensorType sensorType;
  Magnum::Matrix4
      cameraTransform;  // world to camera; todo: Matrix4 or Matrix4x4?
  // todo: include full camera/sensor data so we can reproduce original
  // observation
};

// to serialize a drawObservation event plus all scene graph changes since the
// previous drawObservation
struct RenderKeyframe {
  // int simStepCount; // todo later
  std::vector<RenderAssetInstanceCreation> creations;
  std::vector<RenderAssetInstanceKey> deletions;
  std::vector<std::pair<RenderAssetInstanceKey, RenderAssetInstanceState>>
      stateUpdates;
  Corrade::Containers::Optional<ObservationRecord> observation;
};

struct RenderAssetInstanceRecord {
  scene::SceneNode* node;
  RenderAssetInstanceKey instanceKey;
  Corrade::Containers::Optional<RenderAssetInstanceState> recentState;
};

class RenderKeyframeWriter {
 public:
  void onCreateRenderAssetInstance(scene::SceneNode* node,
                                   const esp::assets::AssetInfo& assetInfo,
                                   bool isSemantic,
                                   bool isRGBD,
                                   Magnum::ResourceKey lightSetupKey);

  void onDrawObservation(const sensor::VisualSensor& visualSensor);

 private:
  // NodeDeletionHelper calls onDeleteRenderAssetInstance
  friend class NodeDeletionHelper;

  void onDeleteRenderAssetInstance(const scene::SceneNode* node);
  RenderKeyframe& getKeyframe();
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
