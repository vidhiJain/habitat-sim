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

struct ConfigCamera {
  Magnum::Vector3 eye{Magnum::Math::ZeroInit};
  Magnum::Vector3 lookat{Magnum::Math::ZeroInit};
};

struct Config {
  std::vector<ConfigCamera> cameras;
};

esp::io::JsonGenericValue toJsonValue(const ConfigCamera& x,
                                      esp::io::JsonAllocator& allocator);

bool fromJsonValue(const esp::io::JsonGenericValue& obj, ConfigCamera& x);

esp::io::JsonGenericValue toJsonValue(const Config& x,
                                      esp::io::JsonAllocator& allocator);

bool fromJsonValue(const esp::io::JsonGenericValue& obj, Config& x);

}  // namespace arrange
}  // namespace esp

#endif
