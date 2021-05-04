// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "FluidVesselEntity.h"
#include "EntityManager.h"
#include "esp/core/Check.h"

#include <iomanip>
#include <sstream>

namespace esp {
namespace scripted {

FluidVesselEntity::FluidVesselEntity(esp::sim::Simulator* sim,
                                     const FluidVesselEntity::Blueprint& bp,
                                     const Magnum::Vector3& translation,
                                     const Magnum::Quaternion& rotation)
    : sim_(sim), bp_(bp) {
  EntityManager<FluidVesselEntity>::get().onConstruct(this);

  // sloppy: fix up spoutDir
  bp_.spoutDir = bp_.spoutDir.normalized();
  ESP_CHECK(bp_.spoutConeAngle <= Mn::Deg(180),
            "bp_.spoutConeAngle <= Mn::Deg(180)");

  auto id = sim->addObjectByHandle(bp_.objHandle);
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
  // if within spoutConeAngle of straight down
  if (spoutDirWorld.y() >= -Mn::Math::cos(bp_.spoutConeAngle)) {
    return;
  }
  Mn::Vector3 spoutPosWorld = transform.transformPoint(bp_.spoutPos);
  static float fudge = 0.05f;
  Mn::Vector3 targetPourXY = spoutPosWorld + spoutDirWorld * fudge;

  FluidVesselEntity* bestVessel = nullptr;
  Mn::Vector3 bestSpoutPosWorld(Mn::Math::ZeroInit);
  float bestXyDist = -1.f;

  for (auto* other : EntityManager<FluidVesselEntity>::get().getVector()) {
    if (other == this) {
      continue;
    }

    if (other->bp_.volume == 0.f) {
      // can't receive anything
      continue;
    }

    const auto otherTransform = sim_->getTransformation(other->objId_);
    Mn::Vector3 otherSpoutDirWorld =
        otherTransform.transformVector(other->bp_.spoutDir);
    static float uprightThreshold = 0.9;
    if (otherSpoutDirWorld.y() < uprightThreshold) {
      continue;
    }

    Mn::Vector3 otherSpoutPosWorld =
        otherTransform.transformPoint(other->bp_.spoutPos);

    float xyDist =
        (Mn::Vector3(targetPourXY.x(), 0.f, targetPourXY.z()) -
         Mn::Vector3(otherSpoutPosWorld.x(), 0.f, otherSpoutPosWorld.z()))
            .length();
    static float pad = 0.1f;  // make it easier to pour
    if (xyDist > other->bp_.spoutRadius + pad) {
      continue;
    }

    if (!bestVessel || xyDist < bestXyDist) {
      bestVessel = other;
      bestSpoutPosWorld = otherSpoutPosWorld;
      bestXyDist = xyDist;
    }
  }

  if (bestVessel) {
    pour(bestVessel, bestSpoutPosWorld, dt);
  } else {
    // todo
    // CORRADE_INTERNAL_ASSERT_UNREACHABLE();
  }

  // tod
}

void FluidVesselEntity::pour(FluidVesselEntity* other,
                             const Magnum::Vector3& otherSpoutPosWorld,
                             float dt) {
  static float pourRate = 0.03;  // liter/s
  float amountToPour = pourRate * dt;

  std::string fluidType = "milk";

  amountToPour = std::min(amountToPour, fluidVolumeByType_[fluidType]);

  float freeCapacity = other->bp_.volume - other->fluidVolumeByType_[fluidType];
  amountToPour = std::min(amountToPour, freeCapacity);

  if (amountToPour > 0.f) {
    other->receiveFluid(fluidType, amountToPour);

    fluidVolumeByType_[fluidType] -= amountToPour;

    recentPourTarget_ = otherSpoutPosWorld;
  }
}

void FluidVesselEntity::receiveFluid(const std::string& fluidType,
                                     float amount) {
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

  debugRender.drawCircle(spoutPosWorld, spoutDirWorld, bp_.spoutRadius, 16,
                         Mn::Color3(1, 1, 1));
  // debugRender.drawLine(spoutPosWorld, spoutPosWorld + spoutDirWorld * 0.1,
  // Mn::Color3(1, 1, 1));

  if (recentPourTarget_) {
    const auto target = *recentPourTarget_;
    // debugRender.drawLine(spoutPosWorld, target, Mn::Color3(1, 1, 1));

    float tangentScale = (spoutPosWorld - target).length() * 0.3f;
    debugRender.drawCurve({spoutPosWorld, target},
                          {spoutDirWorld * tangentScale,
                           Mn::Vector3(0.f, -1.f, 0.f) * tangentScale},
                          Mn::Color4(1, 1, 1), 16);
  }

  std::string fluidType = "milk";
  float amount = fluidVolumeByType_[fluidType];
  if (amount > 0) {
    std::stringstream ss;
    ss << fluidType << "\n"
       << std::fixed << std::setprecision(1) << (amount * 1000.f) << "ml";
    debug3dText.addText(ss.str(), pos);
  }
}

}  // namespace scripted
}  // namespace esp
