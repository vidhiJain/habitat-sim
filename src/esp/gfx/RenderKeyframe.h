// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/assets/Asset.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/scene/SceneNode.h"  // todo: forward decl
#include "esp/sensor/Sensor.h"
#include "esp/sensor/VisualSensor.h"

#include <string>

namespace esp {
namespace gfx {

// todo: put all these structs in a replay/keyframe namespace

class NodeDeletionHelper;

using RenderAssetInstanceKey = uint32_t;

struct Transform {
  Magnum::Vector3 translation;
  Magnum::Quaternion rotation;

  bool operator==(const Transform& rhs) const {
    return translation == rhs.translation && rotation == rhs.rotation;
  }
};

// to serialize creation event
struct RenderAssetInstanceState {
  Transform absTransform;  // todo: Matrix4 or Matrix4x4?
  // todo: support semanticId per drawable?
  int semanticId = -1;
  // todo: support mutable lightSetupKey?

  bool operator==(const RenderAssetInstanceState& rhs) const {
    return absTransform == rhs.absTransform && semanticId == rhs.semanticId;
  }
};

#if 0  // for reference; todo: #define to disable all this code
// to serialize drawObservation event
struct ObservationRecord {
  esp::sensor::SensorType sensorType;
  Magnum::Matrix4
      cameraTransform;  // world to camera; todo: Matrix4 or Matrix4x4?
  // todo: include full camera/sensor data so we can reproduce original
  // observation
};
#endif

// to serialize a drawObservation event plus all scene graph changes since the
// previous drawObservation
struct RenderKeyframe {
  // int simStepCount; // todo later
  std::vector<esp::assets::AssetInfo> loads;
  std::vector<std::pair<RenderAssetInstanceKey,
                        esp::assets::RenderAssetInstanceCreationInfo>>
      creations;
  std::vector<RenderAssetInstanceKey> deletions;
  std::vector<std::pair<RenderAssetInstanceKey, RenderAssetInstanceState>>
      stateUpdates;
  std::unordered_map<std::string, Transform> userTransforms;
#if 0  // for reference
  Corrade::Containers::Optional<ObservationRecord> observation;
#endif
};

struct RenderAssetInstanceRecord {
  scene::SceneNode* node = nullptr;
  RenderAssetInstanceKey instanceKey = -1;
  Corrade::Containers::Optional<RenderAssetInstanceState> recentState;
  NodeDeletionHelper* deletionHelper = nullptr;
};

}  // namespace gfx
}  // namespace esp
