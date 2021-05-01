// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "FluidVesselEntity.h"
#include "EntityManager.h"

#include <iomanip>
#include <sstream>

namespace esp {
namespace scripted {

FluidVesselEntity::FluidVesselEntity(esp::sim::Simulator* sim,
                               const FluidVesselEntity::Blueprint& bp,
                               const Magnum::Vector3& translation,
                               const Magnum::Quaternion& rotation)
    : sim_(sim)
    , bp_(bp) {
  EntityManager<FluidVesselEntity>::get().onConstruct(this);

  // sloppy: fix up spoutDir
  bp_.spoutDir = bp_.spoutDir.normalized();

  auto id =
      sim->addObjectByHandle(bp_.objHandle);
  CORRADE_INTERNAL_ASSERT(id != -1);
  sim_->setTranslation(translation, id);
  sim_->setRotation(rotation, id);
  // sim->setObjectMotionType(esp::physics::MotionType::STATIC, id);

  if (!bp_.initialFluidType.empty()) {
    fluidVolumeByType_[bp_.initialFluidType] = bp_.volume;
  }

  objId_ = id;
}

void FluidVesselEntity::update(float dt) {

  // by default, we aren't pouring anything
  recentPourTarget_ = Cr::Containers::NullOpt;

  std::string fluidType = "milk";
  if (fluidVolumeByType_[fluidType] == 0.f) {
    // nothing to pour
    return;
  }

  const auto transform = sim_->getTransformation(objId_);
  Mn::Vector3 spoutDirWorld = transform.transformVector(bp_.spoutDir);
  if (spoutDirWorld.y() >= 0.f) {
    return;
  }
  Mn::Vector3 spoutPosWorld = transform.transformPoint(bp_.spoutPos);

  FluidVesselEntity* bestVessel = nullptr;
  Mn::Vector3 bestSpoutPosWorld(Mn::Math::ZeroInit);

  for (auto* other : EntityManager<FluidVesselEntity>::get().getVector()) {
    if (other == this) {
      continue;
    }

    if (other->bp_.volume == 0.f) {
      // can't receive anything
      continue;
    }

    const auto otherTransform = sim_->getTransformation(other->objId_);
    Mn::Vector3 otherSpoutDirWorld = otherTransform.transformVector(other->bp_.spoutDir);
    static float uprightThreshold = 0.9;
    if (otherSpoutDirWorld.y() < uprightThreshold) {
      continue;
    }

    Mn::Vector3 otherSpoutPosWorld = otherTransform.transformPoint(other->bp_.spoutPos);

    float xyDist = (Mn::Vector3(spoutPosWorld.x(), 0.f, spoutPosWorld.z())
      - Mn::Vector3(otherSpoutPosWorld.x(), 0.f, otherSpoutPosWorld.z())).length();
    static float pad = 0.25f; // make it easier to pour
    if (xyDist > other->bp_.spoutRadius + pad) {
      continue;
    }

    if (!bestVessel || otherSpoutPosWorld.y() > bestSpoutPosWorld.y()) {
      bestVessel = other;
      bestSpoutPosWorld = otherSpoutPosWorld;
    }
  }

  if (bestVessel) {
    pour(bestVessel, bestSpoutPosWorld, dt);
  } else {
    // todo
    //CORRADE_INTERNAL_ASSERT_UNREACHABLE();
  }

  // tod

}

void FluidVesselEntity::pour(FluidVesselEntity* other, const Magnum::Vector3& otherSpoutPosWorld, float dt) {

  recentPourTarget_ = otherSpoutPosWorld;

  static float pourRate = 0.01; // liter/s
  float amountToPour = pourRate * dt;

  std::string fluidType = "milk";

  amountToPour = std::min(amountToPour, fluidVolumeByType_[fluidType]);

  float freeCapacity = other->bp_.volume - other->fluidVolumeByType_[fluidType];
  amountToPour = std::min(amountToPour, freeCapacity);

  other->receiveFluid(fluidType, amountToPour);

  fluidVolumeByType_[fluidType] -= amountToPour;
}

void FluidVesselEntity::receiveFluid(const std::string& fluidType, float amount) {

  float prevAmount = 0.f;
  const auto it = fluidVolumeByType_.find(fluidType);
  if (it != fluidVolumeByType_.end()) {
    prevAmount = it->second;
  }

  fluidVolumeByType_[fluidType] = prevAmount + amount;
}

FluidVesselEntity::~FluidVesselEntity() {
  EntityManager<FluidVesselEntity>::get().onDelete(this);
}

void FluidVesselEntity::debugRender(esp::gfx::Debug3DText& debug3dText,
                                 esp::gfx::DebugRender& debugRender) {
  const auto pos = sim_->getTranslation(objId_);

  const auto transform = sim_->getTransformation(objId_);
  Mn::Vector3 spoutDirWorld = transform.transformVector(bp_.spoutDir);
  Mn::Vector3 spoutPosWorld = transform.transformPoint(bp_.spoutPos);

  debugRender.drawCircle(spoutPosWorld, spoutDirWorld, bp_.spoutRadius, 16, Mn::Color3(1, 1, 1));
  // debugRender.drawLine(spoutPosWorld, spoutPosWorld + spoutDirWorld * 0.1, Mn::Color3(1, 1, 1));

  if (recentPourTarget_) {
    const auto target = *recentPourTarget_;
    debugRender.drawLine(spoutPosWorld, target, Mn::Color3(1, 1, 1));
  }
  
  std::string fluidType = "milk";
  float amount = fluidVolumeByType_[fluidType];
  if (amount > 0) {
    std::stringstream ss;
    ss << fluidType << " " << std::fixed << std::setprecision(1) << (amount * 1000.f)
      << "ml";
    debug3dText.addText(ss.str(), pos);
  }

}

}  // namespace scripted
}  // namespace esp
