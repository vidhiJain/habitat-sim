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
  sim_->setTranslation(translation, id);
  sim_->setRotation(rotation, id);
  // sim->setObjectMotionType(esp::physics::MotionType::STATIC, id);

  objId_ = id;
}

void CookableEntity::update(float dt) {

  constexpr float targetCookTime = 10.f;
  bool wasCooked = cookTime_ > targetCookTime;

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

  if (!wasCooked && cookTime_ > targetCookTime) {
    auto translation = sim_->getTranslation(objId_);
    // auto rotation = sim_->getRotation(objId_);
    sim_->removeObject(objId_);
    auto id =
        sim_->addObjectByHandle("data/objects/cookie_cooked.object_config.json");
    CORRADE_INTERNAL_ASSERT(id != -1);
    sim_->setTranslation(translation, id);
    // use identity transform?
    // sim_->setRotation(rotation, id);
    objId_ = id;
  }
}

CookableEntity::~CookableEntity() {
  EntityManager<CookableEntity>::get().onDelete(this);
}

void CookableEntity::debugRender(esp::gfx::Debug3DText& debug3dText,
                                 esp::gfx::DebugRender& debugRender) {
  const auto pos = sim_->getTranslation(objId_);

  if (cookTime_ > 0) {
    std::stringstream ss;
    ss << "cook time " << std::fixed << std::setprecision(1) << cookTime_
      << "s";
    debug3dText.addText(ss.str(), pos + Mn::Vector3(0.f, 0.0, 0.0));
  }
}

}  // namespace scripted
}  // namespace esp
