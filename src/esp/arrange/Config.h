// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ARRANGE_CONFIG_H_
#define ESP_ARRANGE_CONFIG_H_

#include <vector>

#include <Magnum/Math/Vector3.h>

#include "esp/io/JsonAllTypes.h"

namespace esp {
namespace arrange {

// see Arranger::updateForLinkAnimation
struct ConfigArticulatedLink {
  float divisionEps = 1e-2f;
  float distEps = 0.02f;
  float maxVelMag = 20.f;
  float baseVelMag0 = 0.2f;
  float baseVelMag1 = 0.05f;
  float rampUpTime = 2.0f;
};

struct ConfigColorSet {
  Magnum::Color4 goodPlacement = Mn::Color4(0.f, 1.0, 0.f, 1.f);
  Magnum::Color4 badPlacement = Mn::Color4(1.f, 0.0, 0.f, 1.f);
  Magnum::Color4 artObj = Mn::Color4(1.f, 0.5, 0.f, 1.f);
  Magnum::Color4 rigidObj = Mn::Color4(1.f, 0.0, 0.f, 1.f);
  // set alpha to 0 to disable placementGuide
  Magnum::Color4 placementGuide = Mn::Color4(0.f, 1.0, 0.f, 0.3f);
  Magnum::Color4 debugLines = Mn::Color4(1.f, 0.0, 1.f, 0.5f);
};

struct ConfigCamera {
  Magnum::Vector3 eye{Magnum::Math::ZeroInit};
  Magnum::Vector3 lookat{Magnum::Math::ZeroInit};
};

struct ConfigLineList {
  std::vector<Magnum::Vector3> verts;
};

struct Config {
  // save a keyframe every n physics steps (see also Arranger::physicsTimestep_)
  int keyframeSavePeriod = 4;
  // see Bullet gDeactivationTime
  float physicsDeactivationTime = 0.75f;
  // In cases where Bullet objects take too long to deactivate, we cancel
  // the wait for settling and force the action to end.
  float physicsMaxSettleTime = 4.f;
  // In cases where the link animation takes too long to complete (e.g. an
  // object is blocking and preventing closing a door), we cancel the link
  // animation (e.g. leave the door half-closed).
  float linkAnimationMaxDuration = 12.f;
  // During cursor-based placement, we search up for a collision-free placement
  float placementSearchHeight = 0.5f;
  // The mouse picker casts a sphere into the scene to start placement. A
  // smaller radius gives you more control but makes it harder to place on thin
  // objects like a dishwasher rack.
  float pickerSphereRadius = 0.015f;
  ConfigArticulatedLink link;
  ConfigColorSet colors;
  std::vector<ConfigCamera> cameras;
  std::vector<ConfigLineList> debugLineLists;
};

esp::io::JsonGenericValue toJsonValue(const ConfigArticulatedLink& x,
                                      esp::io::JsonAllocator& allocator);
bool fromJsonValue(const esp::io::JsonGenericValue& obj,
                   ConfigArticulatedLink& x);

esp::io::JsonGenericValue toJsonValue(const ConfigColorSet& x,
                                      esp::io::JsonAllocator& allocator);
bool fromJsonValue(const esp::io::JsonGenericValue& obj, ConfigColorSet& x);

esp::io::JsonGenericValue toJsonValue(const ConfigCamera& x,
                                      esp::io::JsonAllocator& allocator);
bool fromJsonValue(const esp::io::JsonGenericValue& obj, ConfigCamera& x);

esp::io::JsonGenericValue toJsonValue(const ConfigLineList& x,
                                      esp::io::JsonAllocator& allocator);
bool fromJsonValue(const esp::io::JsonGenericValue& obj, ConfigLineList& x);

esp::io::JsonGenericValue toJsonValue(const Config& x,
                                      esp::io::JsonAllocator& allocator);
bool fromJsonValue(const esp::io::JsonGenericValue& obj, Config& x);

}  // namespace arrange
}  // namespace esp

#endif
