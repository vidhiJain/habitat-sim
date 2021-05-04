// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "OvenEntity.h"
#include "EntityManager.h"

#include <iomanip>
#include <sstream>

namespace esp {
namespace scripted {

constexpr float openSensorRaycastLen = 0.03;

OvenEntity::OvenEntity(esp::sim::Simulator* sim,
                       const OvenEntity::Blueprint& bp,
                       const Magnum::Vector3& translation,
                       const Magnum::Quaternion& rotation)
    : bp_(bp), sim_(sim) {
  EntityManager<OvenEntity>::get().onConstruct(this);

  // hack normalize dir
  bp_.openSensorDir = bp_.openSensorDir.normalized();

// this handle is specific to a particular cooking device and should maybe be
// a contructor arg
#if 0
  auto id = sim->addObjectByHandle(
      "data/objects/kitchen_oven_open.object_config.json");
  CORRADE_INTERNAL_ASSERT(id != -1);
  sim->setTranslation(translation, id);
  sim->setRotation(rotation, id);
  sim->setObjectMotionType(esp::physics::MotionType::STATIC, id);
#endif
  std::string urdfFilePath = "data/URDF/kitchen_oven/kitchen_oven.urdf";
  int id = sim_->addArticulatedObjectFromURDF(bp_.urdfFilepath, true);
  Mn::Matrix4 transform = Mn::Matrix4::from(rotation.toMatrix(), translation);
  sim_->setArticulatedObjectRootState(id, transform);

  objId_ = id;
  temp_ = bp_.roomTemp;
}

bool OvenEntity::isInsideCookVolume(const Magnum::Vector3& pos) {
  // const auto transform = sim_->getTransformation(objId_);
  const auto transform = sim_->getArticulatedObjectRootState(objId_);
  // perf todo: cache inverted transform
  const auto posLocal = transform.invertedRigid().transformPoint(pos);
  return bp_.heatVolume.contains(posLocal);
}

void OvenEntity::update(float dt) {
  const auto transform = sim_->getArticulatedObjectRootState(objId_);
  const Mn::Vector3& pos = transform.translation();
  Mn::Vector3 openSensorDirWorld = transform.transformVector(bp_.openSensorDir);
  Mn::Vector3 openSensorPosWorld = transform.transformPoint(bp_.openSensorPos);

  // raycast for door to check if open
  esp::geo::Ray ray;
  ray.origin = openSensorPosWorld;
  ray.direction = openSensorDirWorld;
  constexpr float maxDistance = 5.f;
  auto raycastResults = sim_->castRay(ray, openSensorRaycastLen);
  isClosed_ = raycastResults.hasHits();

  float tempPerSec = isClosed_ ? bp_.closedTempPerSec : bp_.openTempPerSec;
  temp_ =
      std::max(bp_.roomTemp, std::min(bp_.targetTemp, temp_ + dt * tempPerSec));
}

OvenEntity::~OvenEntity() {
  EntityManager<OvenEntity>::get().onDelete(this);
}

void OvenEntity::debugRender(esp::gfx::Debug3DText& debug3dText,
                             esp::gfx::DebugRender& debugRender) {
  // const auto transform = sim_->getTransformation(objId_);
  const auto transform = sim_->getArticulatedObjectRootState(objId_);
  Mn::Vector3 openSensorPosWorld = transform.transformPoint(bp_.openSensorPos);

  std::stringstream ss;
  ss << (isClosed_ ? "on\n" : "open\n") << std::fixed << std::setprecision(0)
     << temp_ << "°";
  debug3dText.addText(ss.str(),
                      openSensorPosWorld + Mn::Vector3(0.f, 0.2f, 0.f));

  debugRender.pushInputTransform(transform);
  debugRender.drawCircle(bp_.openSensorPos, bp_.openSensorDir, 0.01f, 12,
                         Mn::Color3::green());
  debugRender.drawTransformedLine(
      bp_.openSensorPos,
      bp_.openSensorPos + bp_.openSensorDir * openSensorRaycastLen,
      Mn::Color3::green());
  debugRender.drawBox(bp_.heatVolume.min(), bp_.heatVolume.max(),
                      Mn::Color3::red());
  debugRender.popInputTransform();
}

}  // namespace scripted
}  // namespace esp
