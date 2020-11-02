// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Asset.h"

#include "esp/scene/SceneNode.h"

#include "Magnum/Resource.h"

#include <Corrade/Containers/Optional.h>

namespace esp {
namespace assets {

struct RenderAssetInstanceCreation {
  std::string renderAssetHandle;
  Corrade::Containers::Optional<Magnum::Vector3> scale;
  bool isSemantic = false;
  bool isRGBD = false;
  bool isStatic = false;
  Magnum::ResourceKey lightSetupKey;
};

}  // namespace assets
}  // namespace esp
