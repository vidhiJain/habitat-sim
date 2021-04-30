// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCRIPTED_KITCHENSETUP_H_
#define ESP_SCRIPTED_KITCHENSETUP_H_

#include "esp/sim/Simulator.h"

namespace esp {
namespace scripted {

class KitchenSetup {
 public:
  KitchenSetup(esp::sim::Simulator* sim);
};

}  // namespace scripted
}  // namespace esp

#endif  // ESP_SCRIPTED_KITCHENSETUP_H_
