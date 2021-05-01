// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "EntityManagerHelper.h"
#include <esp/gfx/Debug3DText.h>
#include <esp/gfx/DebugRender.h>
#include "CookableEntity.h"
#include "FluidVesselEntity.h"
#include "EntityManager.hpp"
#include "OvenEntity.h"

namespace esp {
namespace scripted {

void EntityManagerHelper::update(float dt) {
  for (auto* ent : EntityManager<OvenEntity>::get().getVector()) {
    ent->update(dt);
  }
  for (auto* ent : EntityManager<CookableEntity>::get().getVector()) {
    ent->update(dt);
  }
  for (auto* ent : EntityManager<FluidVesselEntity>::get().getVector()) {
    ent->update(dt);
  }
}

void EntityManagerHelper::debugRender(esp::gfx::Debug3DText& debug3dText,
                                      esp::gfx::DebugRender& debugRender) {
  for (auto* ent : EntityManager<OvenEntity>::get().getVector()) {
    ent->debugRender(debug3dText, debugRender);
  }
  for (auto* ent : EntityManager<CookableEntity>::get().getVector()) {
    ent->debugRender(debug3dText, debugRender);
  }
  for (auto* ent : EntityManager<FluidVesselEntity>::get().getVector()) {
    ent->debugRender(debug3dText, debugRender);
  }
}

// explicit instantiation
template class EntityManager<OvenEntity>;
template class EntityManager<CookableEntity>;
template class EntityManager<FluidVesselEntity>;

}  // namespace scripted
}  // namespace esp
