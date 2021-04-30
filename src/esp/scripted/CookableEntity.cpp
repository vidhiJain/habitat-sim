// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "CookableEntity.h"
#include "EntityManager.h"
#include "OvenEntity.h"

#include <iomanip>
#include <sstream>

namespace esp {
namespace scripted {

CookableEntity::CookableEntity(esp::sim::Simulator* sim,
                               const Magnum::Vector3& translation,
                               const Magnum::Quaternion& rotation)
    : sim_(sim) {
  EntityManager<CookableEntity>::get().onConstruct(this);

  auto id =
      sim->addObjectByHandle("data/objects/cookie_ball.object_config.json");
  CORRADE_INTERNAL_ASSERT(id != -1);
  sim->setTranslation(translation, id);
  sim->setRotation(rotation, id);
  // sim->setObjectMotionType(esp::physics::MotionType::STATIC, id);

  objId_ = id;
}

void CookableEntity::update(float dt) {
  const auto pos = sim_->getTranslation(objId_);
  constexpr float minCookTemp = 250.f;
  for (auto* oven : EntityManager<OvenEntity>::get().getVector()) {
    float temp = oven->getTemperature();
    if (temp >= minCookTemp) {
      if (oven->isInsideCookVolume(pos)) {
        cookTime_ += dt;
      }
    }
  }
}

CookableEntity::~CookableEntity() {
  EntityManager<CookableEntity>::get().onDelete(this);
}

void CookableEntity::debugRender(esp::gfx::Debug3DText& debug3dText,
                                 esp::gfx::DebugRender& debugRender) {
  const auto pos = sim_->getTranslation(objId_);

  std::stringstream ss;
  ss << "cookie, cook time " << std::fixed << std::setprecision(0) << cookTime_
     << "s";
  debug3dText.addText(ss.str(), pos + Mn::Vector3(0.f, 0.75, 0.0));
}

}  // namespace scripted
}  // namespace esp
