// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Arranger.h"

namespace esp {
namespace arrange {

namespace {

static const int PHYSICS_SAVE_PERIOD =
    4;  // save a keyframe every n physics steps (see also physicsTimestep_)

float calcLerpFraction(float src0, float src1, float x) {
  CORRADE_INTERNAL_ASSERT(src1 != src0);
  return (x - src0) / (src1 - src0);
}

float smoothstep(float fraction) {
  float x = esp::geo::clamp(fraction, 0.0f, 1.0f);
  // Evaluate polynomial
  return x * x * (3 - 2 * x);
}

float getDisplayRadiusForNode(const esp::scene::SceneNode* node) {
  const auto range = node->getCumulativeBB();
  static float fudgeScale = 0.3f;
  const float radius = (range.max() - range.min()).length() * fudgeScale;
  return radius;
}

float getDisplayRadiusForObject(esp::sim::Simulator& simulator, int objId) {
  return getDisplayRadiusForNode(simulator.getObjectSceneNode(objId));
}

void forcePhysicsSceneAsleep(esp::sim::Simulator& simulator) {
  auto handles =
      simulator.getArticulatedObjectManager()->getObjectHandlesBySubstring();
  for (const auto& handle : handles) {
    auto artObj =
        simulator.getArticulatedObjectManager()->getObjectByHandle(handle);
    artObj->setActive(false);
  }

  handles = simulator.getRigidObjectManager()->getObjectHandlesBySubstring();
  for (const auto& handle : handles) {
    auto rigidObj =
        simulator.getRigidObjectManager()->getObjectByHandle(handle);
    rigidObj->setActive(false);
  }
}

}  // namespace

Arranger::Arranger(esp::sim::Simulator* simulator,
                   esp::gfx::RenderCamera* renderCamera,
                   esp::gfx::DebugRender* debugRender,
                   esp::gfx::Debug3DText* debug3dText)
    : simulator_(simulator),
      renderCamera_(renderCamera),
      debugRender_(debugRender),
      debug3dText_(debug3dText) {
  existingObjectIds_ = simulator_->getExistingObjectIDs();

  session_.scene = "placeholder";

  // see data/default.physics_config.json timestep
  physicsTimestep_ = simulator_->getMetadataMediator()
                         ->getCurrentPhysicsManagerAttributes()
                         ->getTimestep();

  // save keyframe 0
  session_.keyframes.emplace_back(simulator_->savePhysicsKeyframe());

  // restore from current physics keyframe; this will put the physics sim into a
  // more consistent state, e.g. zero velocities
  simulator_->restoreFromPhysicsKeyframe(session_.keyframes.back());
  // stepping here allows bodies that want to go to sleep to actually go to
  // sleep
  simulator_->stepWorld(-1);

  CORRADE_INTERNAL_ASSERT(!activeUserAction_);
  activeUserAction_ = UserAction();
  activeUserAction_->isInitialSettling = true;
  activeUserAction_->startFrame = session_.keyframes.size() - 1;
  LOG(INFO) << "Starting initial-settling user action on frame " +
                   activeUserAction_->startFrame;
}

void Arranger::updateForLinkAnimation(float dt,
                                      bool isPrimaryButton,
                                      bool isSecondaryButton) {
  CORRADE_INTERNAL_ASSERT(linkAnimOpt_);
  auto& linkAnim = *linkAnimOpt_;

  linkAnim.animTimer += dt;

  auto artObj = simulator_->getArticulatedObjectManager()->getObjectByID(
      linkAnim.artObjId);
  artObj->setActive(true);
  auto jointPositions = artObj->getJointPositions();
  float jointPos = jointPositions[linkAnim.jointPosOffset];

  // float animFraction = smoothstep(linkAnim.animTimer /
  // linkAnim.animDuration); float newPos =
  // Mn::Math::lerp(linkAnim.startPos, linkAnim.endPos, animFraction);

  // animation progress is based on position progress towards endPos
  // note animDuration not used right now
  float animFraction =
      calcLerpFraction(linkAnim.startPos, linkAnim.endPos, jointPos);
  float forceDir = linkAnim.endPos < linkAnim.startPos ? -1.f : 1.f;
  static float eps = 0.01;
  bool isNearEndPos = std::abs(jointPos - linkAnim.endPos) < eps;

#if 0  // force approach
  // force starts out strong and decreases over time
  float animForceMagScale = 1.f - animFraction;

  // jointPositions[linkAnim.jointPosOffset] = newPos;
  // artObj->setJointPositions(jointPositions);
  auto jointForces = artObj->getJointForces();

  static float maxAccel = 5.f;
  const float linkMass = 2.f; // temp hack because this is broken: artObj->getLink(linkAnim.linkId)->getMass();
  float force = forceDir * animForceMagScale * maxAccel * linkMass;
  jointForces[linkAnim.jointPosOffset] = force;
  artObj->setJointForces(jointForces);
#endif

  static float startVel = 0.1f;
  static float maxVel = 0.75f;
  static float endVel = 0.1f;
  static float exp0 = 0.25;  // smaller => faster accel at start of anim
  static float exp1 = 5.f;   // larger => faster completion of anim
  float velMag =
      (animFraction < 0.5)
          ? Mn::Math::lerp(
                startVel, maxVel,
                Mn::Math::pow(
                    smoothstep(calcLerpFraction(0.f, 0.5, animFraction)), exp0))
          : Mn::Math::lerp(maxVel, endVel,
                           Mn::Math::pow(smoothstep(calcLerpFraction(
                                             0.5f, 1.0, animFraction)),
                                         exp1));
  // snap to zero vel if near end pos (anim is finished)
  float vel = isNearEndPos ? 0.0f : forceDir * velMag;

  auto jointVels = artObj->getJointVelocities();
  jointVels[linkAnim.jointPosOffset] = vel;
  artObj->setJointVelocities(jointVels);

  static float animMaxDuration = 10.f;  // todo: tunable
  if (isNearEndPos || linkAnim.animTimer > animMaxDuration ||
      isSecondaryButton) {
    if (isNearEndPos) {
      // animation is finished so snap to end pos
      auto jointPositions = artObj->getJointPositions();
      jointPositions[linkAnim.jointPosOffset] = linkAnim.endPos;
      artObj->setJointPositions(jointPositions);
    }

    linkAnimOpt_ = Cr::Containers::NullOpt;
    waitingForSceneRest_ = true;
  }
}

void Arranger::updateIdle(float dt,
                          bool isPrimaryButton,
                          bool isSecondaryButton) {
  CORRADE_INTERNAL_ASSERT(heldObjId_ == -1 && !linkAnimOpt_);

  auto ray = renderCamera_->unproject(cursor_);
  static float sphereRadius = 0.01f;
  esp::physics::RaycastResults raycastResults =
      simulator_->castSphere(ray, sphereRadius);

  int mouseoverRigidObjId = -1;
  if (raycastResults.hasHits()) {
    auto hitId = raycastResults.hits[0].objectId;

    bool isStage;
    int rigidObjId;
    int artObjId;
    int linkId;
    simulator_->resolvePhysicsHitID(hitId, &isStage, &rigidObjId, &artObjId,
                                    &linkId);

    if (rigidObjId != -1) {
      if (simulator_->getObjectMotionType(rigidObjId) ==
          esp::physics::MotionType::DYNAMIC) {
        mouseoverRigidObjId = rigidObjId;
      }
    } else if (artObjId != -1 && linkId != -1) {
      auto artObj =
          simulator_->getArticulatedObjectManager()->getObjectByID(artObjId);

      const auto* link = artObj->getLink(linkId);

      // beware getDisplayRadiusForNode(&link->getSceneNode()) doesn't work
      // because getCumulativeBB is broken for links
      debugRender_->drawCircle(link->getTranslation(), Mn::Vector3(0, 1, 0),
                               0.3f, 16, Mn::Color4(1.f, 0.5, 0.f, 1.f));

      if (isPrimaryButton) {
        int numJointPos = artObj->getLinkNumJointPos(linkId);
        if (numJointPos == 1) {
          int jointPosOffset = artObj->getLinkJointPosOffset(linkId);
          const auto pair = artObj->getJointPositionLimits();
          const auto& lowerLimits = pair.first;
          const auto& upperLimits = pair.second;
          if (lowerLimits[jointPosOffset] != INFINITY) {
            float jointPos = artObj->getJointPositions()[jointPosOffset];
            float upperLimit = upperLimits[jointPosOffset];
            float lowerLimit = lowerLimits[jointPosOffset];
            float animEndPos;
            if (std::abs(jointPos - upperLimit) <
                std::abs(jointPos - lowerLimit)) {
              // closer to upperLimit
              animEndPos = lowerLimit;
            } else {
              // closer to lowerLimit
              animEndPos = upperLimit;
            }

            static float animDuration = 2.f;  // todo: tunable
            linkAnimOpt_ = LinkAnimation{.artObjId = artObjId,
                                         .linkId = linkId,
                                         .jointPosOffset = jointPosOffset,
                                         .startPos = jointPos,
                                         .endPos = animEndPos,
                                         .animTimer = 0.f,
                                         .animDuration = animDuration};
            userInputStatus_ = "animating... (F to cancel)";

            // start user action for this articulated obj
            CORRADE_INTERNAL_ASSERT(!activeUserAction_);
            activeUserAction_ = UserAction();
            activeUserAction_->articulatedObj =
                simulator_->getArticulatedObjectManager()->getObjectHandleByID(
                    artObjId);
            activeUserAction_->articulatedLink = linkId;
            activeUserAction_->startFrame = session_.keyframes.size();
            LOG(INFO) << "Starting user action " << session_.userActions.size()
                      << " on frame " << activeUserAction_->startFrame
                      << " with articulatedObj "
                      << activeUserAction_->articulatedObj << " link "
                      << activeUserAction_->articulatedLink;
          }
        }
      }
    }
  }

  if (mouseoverRigidObjId != -1) {
    debugRender_->drawCircle(
        simulator_->getTranslation(mouseoverRigidObjId), Mn::Vector3(0, 1, 0),
        getDisplayRadiusForObject(*simulator_, mouseoverRigidObjId) * 1.5, 16,
        Mn::Color4::red());

    if (isPrimaryButton) {
      heldObjId_ = mouseoverRigidObjId;
      auto heldObj =
          simulator_->getRigidObjectManager()->getObjectByID(heldObjId_);
      heldObj->setMotionType(esp::physics::MotionType::KINEMATIC);
      heldObj->overrideCollisionGroup(esp::physics::CollisionGroup::Default);
      heldObj->setRotation(getRotationByIndex(recentHeldObjRotIndex_));

      // start user action for this rigid obj
      CORRADE_INTERNAL_ASSERT(!activeUserAction_);
      activeUserAction_ = UserAction();
      activeUserAction_->rigidObj =
          simulator_->getRigidObjectManager()->getObjectHandleByID(heldObjId_);
      activeUserAction_->startFrame = session_.keyframes.size();
      LOG(INFO) << "Starting user action " << session_.userActions.size()
                << " on frame " << activeUserAction_->startFrame
                << " with rigidObj " << activeUserAction_->rigidObj;
    }
  }
}

void Arranger::updateForHeldObject(float dt,
                                   bool isPrimaryButton,
                                   bool isSecondaryButton) {
  auto heldObj = simulator_->getRigidObjectManager()->getObjectByID(heldObjId_);

  if (isSecondaryButton) {
    recentHeldObjRotIndex_ =
        (recentHeldObjRotIndex_ + 1) % getNumRotationIndices();
    heldObj->setRotation(getRotationByIndex(recentHeldObjRotIndex_));
  }

  // hide held object. move out of sight. This is for visuals, but also to
  // avoid screwing up the sphere-cast we're about to do.
  const Mn::Vector3 hiddenPos(0.f, -100.f, 0.f);
  heldObj->setTranslation(hiddenPos);

  auto ray = renderCamera_->unproject(cursor_);
  // use sphere cast instead of raycast so that we can easily hit narrow
  // objects like the dishwasher rack
  static float sphereRadius = 0.01f;
  esp::physics::RaycastResults raycastResults =
      simulator_->castSphere(ray, sphereRadius);

  bool foundPreviewPos = false;
  if (raycastResults.hasHits()) {
    Mn::Vector3 hitPos = raycastResults.hits[0].point;
    Mn::Vector3 queryPos = hitPos;

    static float maxOffsetY = 0.5f;
    static float offsetStep = 0.02f;
    for (float offsetY = 0; offsetY < maxOffsetY; offsetY += offsetStep) {
      heldObj->setTranslation(queryPos);
      if (!simulator_->contactTest(heldObjId_)) {
        foundPreviewPos = true;
        break;
      }
      queryPos.y() += offsetStep;
    }

    if (foundPreviewPos) {
      debugRender_->drawCircle(
          queryPos, Mn::Vector3(0, 1, 0),
          getDisplayRadiusForObject(*simulator_, heldObjId_) * 1.5, 16,
          Mn::Color4::green());
    }

    auto color = foundPreviewPos ? Mn::Color4::green() : Mn::Color4::red();
    static float lineLen = 1.f;
    debugRender_->drawLine(hitPos + Mn::Vector3(lineLen, 0.f, 0.f),
                           hitPos - Mn::Vector3(lineLen, 0.f, 0.f), color);
    debugRender_->drawLine(hitPos + Mn::Vector3(0.f, 0.f, lineLen),
                           hitPos - Mn::Vector3(0.f, 0.f, lineLen), color);
    debugRender_->drawLine(hitPos, hitPos - Mn::Vector3(0.f, lineLen, 0.f),
                           color);
    if (foundPreviewPos) {
      debugRender_->drawLine(hitPos, queryPos, color);
    }
  }

  if (foundPreviewPos && isPrimaryButton) {
    heldObj->setMotionType(esp::physics::MotionType::DYNAMIC);
    heldObj->overrideCollisionGroup(esp::physics::CollisionGroup::Dynamic);
    heldObjId_ = -1;
    waitingForSceneRest_ = true;
  }

  if (!foundPreviewPos) {
    // hide held object. move out of sight. This is for visuals, but also to
    // not screw up the raycast we're about to do.
    const Mn::Vector3 hiddenPos(0.f, -100.f, 0.f);
    heldObj->setTranslation(hiddenPos);
  }
}

void Arranger::updateWaitingForSceneRest(float dt,
                                         bool isPrimaryButton,
                                         bool isSecondaryButton) {
  CORRADE_INTERNAL_ASSERT(waitingForSceneRest_);

  if (isSecondaryButton) {
    forcePhysicsSceneAsleep(*simulator_);
    waitingForSceneRest_ = false;
    userInputStatus_.clear();
  } else {
    int count = markAndCountActivePhysicsObjects();
    if (count == 0) {
      waitingForSceneRest_ = false;
      userInputStatus_.clear();
    } else {
      userInputStatus_ = "settling... (F to cancel)";
    }
  }

  if (!waitingForSceneRest_) {
    // end the active user action
    CORRADE_INTERNAL_ASSERT(activeUserAction_);
    activeUserAction_->endFrame = session_.keyframes.size() - 1;
    LOG(INFO) << "Ending user action " << session_.userActions.size()
              << " on frame " << activeUserAction_->endFrame;
    session_.userActions.emplace_back(std::move(*activeUserAction_));
    activeUserAction_ = Cr::Containers::NullOpt;

    // restore from current physics keyframe; this will put the physics sim into
    // a more consistent state, e.g. zero velocities
    simulator_->restoreFromPhysicsKeyframe(session_.keyframes.back());
    // stepping here allows bodies that want to go to sleep to actually go to
    // sleep
    simulator_->stepWorld(-1);
  }
}

void Arranger::update(float dt, bool isPrimaryButton, bool isSecondaryButton) {
  bool doFreezePhysicsTime = true;

  if (linkAnimOpt_) {
    updateForLinkAnimation(dt, isPrimaryButton, isSecondaryButton);
  } else if (heldObjId_ != -1) {
    updateForHeldObject(dt, isPrimaryButton, isSecondaryButton);
  } else if (waitingForSceneRest_) {
    updateWaitingForSceneRest(dt, isPrimaryButton, isSecondaryButton);
  } else {
    updateIdle(dt, isPrimaryButton, isSecondaryButton);
  }

  bool doAdvancePhysicsTime = (linkAnimOpt_ || waitingForSceneRest_);
  if (doAdvancePhysicsTime) {
    updatePhysicsWorld(dt);
  }

  if (!userInputStatus_.empty()) {
    // get point along cursor ray, somewhat far away
    auto ray = renderCamera_->unproject(cursor_);
    auto pos = ray.origin + ray.direction * 4.f;
    debug3dText_->addText(std::string(userInputStatus_), pos);
  }
}

int Arranger::getNumRotationIndices() {
  return 6;
}

Mn::Quaternion Arranger::getRotationByIndex(int index) {
  static Mn::Quaternion rots[] = {
      Mn::Quaternion(Mn::Math::IdentityInit),
      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(1.f, 0.f, 0.f)),
      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(0.f, 0.f, 1.f)),
      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(-1.f, 0.f, 0.f)),
      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(0.f, 0.f, -1.f)),
      Mn::Quaternion::rotation(Mn::Deg(180.f), Mn::Vector3(1.f, 0.f, 0.f))};
  return rots[index];
}

void Arranger::updatePhysicsWorld(float dt) {
  timeSinceLastSimulation_ += dt;

  while (timeSinceLastSimulation_ >= physicsTimestep_) {
    // step physics at a fixed rate
    simulator_->stepWorld(-1);
    timeSinceLastSimulation_ -= physicsTimestep_;

    physicsStepCounter_++;
    if (physicsStepCounter_ % PHYSICS_SAVE_PERIOD == 0) {
      session_.keyframes.emplace_back(simulator_->savePhysicsKeyframe());
    }
  }

#if 0  // debug reference code
  static float elapsedTime = 0.f;
  elapsedTime += dt;
  if (physicsStepCounter_ % 10 == 9) {
    float stepRate = physicsStepCounter_ / elapsedTime;
    LOG(INFO) << "stepRate: " << stepRate;
  }
#endif
}

int Arranger::markAndCountActivePhysicsObjects() {
  int count = 0;

  auto handles =
      simulator_->getArticulatedObjectManager()->getObjectHandlesBySubstring();
  for (const auto& handle : handles) {
    auto artObj =
        simulator_->getArticulatedObjectManager()->getObjectByHandle(handle);
    if (artObj->isActive()) {
      debug3dText_->addText("active", artObj->getTranslation(),
                            Mn::Color4(1.f, 0.5, 0.f, 1.f));
      count++;
    }
  }

  handles = simulator_->getRigidObjectManager()->getObjectHandlesBySubstring();
  for (const auto& handle : handles) {
    auto rigidObj =
        simulator_->getRigidObjectManager()->getObjectByHandle(handle);
    if (rigidObj->isActive()) {
      debug3dText_->addText("active", rigidObj->getTranslation(),
                            Mn::Color4(1.f, 0.0, 0.f, 1.f));
      count++;
    }
  }

  return count;
}

}  // namespace arrange
}  // namespace esp
