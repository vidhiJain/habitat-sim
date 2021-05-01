// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "OvenEntity.h"
#include "EntityManager.h"

#include <iomanip>
#include <sstream>

namespace esp {
namespace scripted {

OvenEntity::OvenEntity(esp::sim::Simulator* sim,
                       const Magnum::Vector3& translation,
                       const Magnum::Quaternion& rotation)
    : sim_(sim) {
  EntityManager<OvenEntity>::get().onConstruct(this);

  // this handle is specific to a particular cooking device and should maybe be
  // a contructor arg
  auto id = sim->addObjectByHandle(
      "data/objects/kitchen_oven_open.object_config.json");
  CORRADE_INTERNAL_ASSERT(id != -1);
  sim->setTranslation(translation, id);
  sim->setRotation(rotation, id);
  sim->setObjectMotionType(esp::physics::MotionType::STATIC, id);

  // this is specific to a particular cooking device and should maybe be a
  // contructor arg
  objId_ = id;
  constexpr Mn::Vector3 insetPad(0.01, 0.01, 0.01);
  heatVolume_ = Mn::Range3D(Mn::Vector3(-0.19, 0.28, -0.58) + insetPad,
                            Mn::Vector3(0.44, 0.71, -0.10) - insetPad);
}

bool OvenEntity::isInsideCookVolume(const Magnum::Vector3& pos) {
  const auto transform = sim_->getTransformation(objId_);
  // perf todo: cache inverted transform
  const auto posLocal = transform.invertedRigid().transformPoint(pos);
  return heatVolume_.contains(posLocal);
}

void OvenEntity::update(float dt) {
  // temp hard-coded heating logic
  constexpr float degsPerSec = 5.0;
  temp_ = std::min(350.f, temp_ + dt * degsPerSec);
}

OvenEntity::~OvenEntity() {
  EntityManager<OvenEntity>::get().onDelete(this);
}

void OvenEntity::debugRender(esp::gfx::Debug3DText& debug3dText,
                             esp::gfx::DebugRender& debugRender) {
  const auto& pos = sim_->getTranslation(objId_);

  std::stringstream ss;
  ss << "oven, " << std::fixed << std::setprecision(0) << temp_ << "°";
  debug3dText.addText(ss.str(), pos + Mn::Vector3(0.f, 0.75, 0.0));

  debugRender.pushInputTransform(sim_->getTransformation(objId_));
  debugRender.drawBox(heatVolume_.min(), heatVolume_.max(), Mn::Color3::red());
  debugRender.popInputTransform();
}

}  // namespace scripted
}  // namespace esp
