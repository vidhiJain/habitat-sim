// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "KitchenSetup.h"

#include "CookableEntity.h"
#include "OvenEntity.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace scripted {

KitchenSetup::KitchenSetup(esp::sim::Simulator* sim) {
  // perhaps configure SimConfig with stage as desired?

  // add objects as desired
  new OvenEntity(sim, Mn::Vector3(0.f, 0.f, 3.0),
                 Mn::Quaternion(Magnum::Math::IdentityInit));

  new CookableEntity(sim, Mn::Vector3(0.f, 1.5f, 3.0),
                     Mn::Quaternion(Magnum::Math::IdentityInit));
  new CookableEntity(sim, Mn::Vector3(0.f, 1.3f, 3.0),
                     Mn::Quaternion(Magnum::Math::IdentityInit));
}

}  // namespace scripted
}  // namespace esp
