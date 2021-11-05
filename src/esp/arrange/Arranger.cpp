// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Arranger.h"

#include "BulletDynamics/Dynamics/btRigidBody.h"  // for gDeactivationTime

namespace esp {
namespace arrange {

namespace {

static const float CIRCLE_NUM_SEGMENTS = 24;

static const esp::physics::CollisionGroup PICKER_COLLISION_GROUP =
    esp::physics::CollisionGroup::UserGroup2;

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

Arranger::Arranger(Config&& config,
                   esp::sim::Simulator* simulator,
                   esp::gfx::RenderCamera* renderCamera,
                   esp::gfx::DebugLineRender* debugLineRender,
                   esp::gfx::Debug3DText* debug3dText)
    : config_(std::move(config)),
      simulator_(simulator),
      renderCamera_(renderCamera),
      debugLineRender_(debugLineRender),
      debug3dText_(debug3dText),
      random_(0) {
  isHeadless_ = renderCamera == nullptr;
  auto camTransformMat = updateCamera(0.f, ButtonSet());
  physicsTimestep_ = simulator_->getMetadataMediator()
                         ->getCurrentPhysicsManagerAttributes()
                         ->getTimestep();

  // set some session globals
  session_.dataset =
      simulator_->getMetadataMediator()->getActiveSceneDatasetName();
  session_.scene = simulator_->getMetadataMediator()
                       ->getSimulatorConfiguration()
                       .activeSceneName;
  session_.config = config_;
  // see data/default.physics_config.json timestep
  session_.physicsTimeStep = physicsTimestep_;
  // set session.defaultCamera from result of updateCamera
  {
    const auto& absTransformMat = camTransformMat;
    esp::gfx::replay::Transform absTransform{
        absTransformMat.translation(),
        Magnum::Quaternion::fromMatrix(absTransformMat.rotationShear())};
    session_.defaultCamera = absTransform;
  }

  // save keyframe 0
  session_.keyframes.emplace_back(simulator_->savePhysicsKeyframe());

  // restore from current physics keyframe; this will put the physics sim into a
  // more consistent state, e.g. zero velocities. We also activate all bodies
  // because we're starting a settling action.
  simulator_->restoreFromPhysicsKeyframe(session_.keyframes.back(),
                                         /*activate*/ true);
  actionPhysicsStepCounter_ = 0;

  CORRADE_INTERNAL_ASSERT(!activeUserAction_);
  activeUserAction_ = UserAction();
  activeUserAction_->isSettlingAction = true;
  activeUserAction_->startFrame = session_.keyframes.size() - 1;
  waitingForSceneRest_ = true;
  restStartPhysicsStepCount_ = actionPhysicsStepCounter_;
  ESP_DEBUG() << "Starting \"settling\" user action on frame "
              << activeUserAction_->startFrame;

  if (isHeadless_) {
    waitForRest();
  }
}

Mn::Matrix4 Arranger::updateCamera(float dt, ButtonSet buttonSet) {
  if (config_.cameras.empty()) {
    return Mn::Matrix4(Mn::Math::IdentityInit);
  }

  const int numCameras = config_.cameras.size();
  cameraIndex_ = std::min(cameraIndex_, int(numCameras - 1));

  if (buttonSet & Button::NextCamera) {
    cameraIndex_++;
  } else if (buttonSet & Button::PrevCamera) {
    cameraIndex_--;
  }

  cameraIndex_ = (cameraIndex_ + numCameras) % numCameras;

  auto adjustedConfigCam = config_.cameras[cameraIndex_];
  if (adjustedConfigCam.eye.x() == adjustedConfigCam.lookat.x() &&
      adjustedConfigCam.eye.z() == adjustedConfigCam.lookat.z()) {
    adjustedConfigCam.lookat.x() += 1.f;
  }

  const auto transform =
      Mn::Matrix4::lookAt(adjustedConfigCam.eye, adjustedConfigCam.lookat,
                          Mn::Vector3(0.f, 1.f, 0.f));

  if (renderCamera_) {
    renderCamera_->node().setTransformation(transform);
  }

  return transform;
}

void Arranger::updateForLinkAnimation(float dt, ButtonSet buttonSet) {
  CORRADE_INTERNAL_ASSERT(linkAnimOpt_);
  auto& linkAnim = *linkAnimOpt_;

  linkAnim.animTimer += dt;

  auto artObj = simulator_->getArticulatedObjectManager()->getObjectByID(
      linkAnim.artObjId);
  artObj->setActive(true);
  auto jointPositions = artObj->getJointPositions();
  float jointPos = jointPositions[linkAnim.jointPosOffset];

  // animation progress is based on position progress towards endPos
  // note animDuration not used right now
  float animFraction =
      calcLerpFraction(linkAnim.startPos, linkAnim.endPos, jointPos);
  float forceDir = linkAnim.endPos < linkAnim.startPos ? -1.f : 1.f;
  constexpr float posEps = 0.001;
  bool isNearEndPos = std::abs(jointPos - linkAnim.endPos) < posEps;

  // Setting joint velocity is not absolute. It seems to work more like force in
  // that a larger velocity is needed if there's more resistance. This logic
  // ramps up velocity over time. It's meant to adapt to any amount of
  // resistance.
  const float baseVelMag = Mn::Math::lerp(
      config_.link.baseVelMag0, config_.link.baseVelMag1, animFraction);
  float velMag = std::min(config_.link.maxVelMag,
                          baseVelMag * linkAnim.animTimer /
                              (animFraction + config_.link.divisionEps));
  velMag *= (std::min(linkAnim.animTimer / config_.link.rampUpTime, 1.f));
  // ESP_DEBUG() << "animTimer: " << linkAnim.animTimer << ", velMag: " <<
  // velMag;

  // snap to zero vel if near end pos (anim is finished)
  float vel = isNearEndPos ? 0.0f : forceDir * velMag;

  auto jointVels = artObj->getJointVelocities();
  jointVels[linkAnim.jointPosOffset] = vel;
  artObj->setJointVelocities(jointVels);

  if (isNearEndPos || linkAnim.animTimer > config_.linkAnimationMaxDuration ||
      buttonSet & Button::Secondary) {
    if (isNearEndPos) {
      // animation is finished so snap to end pos
      auto jointPositions = artObj->getJointPositions();
      jointPositions[linkAnim.jointPosOffset] = linkAnim.endPos;
      artObj->setJointPositions(jointPositions);
    }

    linkAnimOpt_ = Cr::Containers::NullOpt;
    waitingForSceneRest_ = true;
    restStartPhysicsStepCount_ = actionPhysicsStepCounter_;
  }
}

void Arranger::undoPreviousUserAction() {
  ESP_CHECK(session_.userActions.size() > 1, "no previous user action to undo");
  session_.userActions.pop_back();
  const int firstKeyframeToDelete = session_.userActions.back().endFrame + 1;
  session_.keyframes.erase(session_.keyframes.begin() + firstKeyframeToDelete,
                           session_.keyframes.end());

  // rewind to recent physics keyframe; this will put the physics sim into
  // a more consistent state, e.g. zero velocities
  simulator_->restoreFromPhysicsKeyframe(session_.keyframes.back(),
                                         /*activate*/ false);
  actionPhysicsStepCounter_ = 0;

  // stepping here allows bodies that want to go to sleep to actually go to
  // sleep
  simulator_->stepWorld(-1);

  ESP_DEBUG() << "Undoing user action " << session_.userActions.size()
              << " and restoring to keyframe " << session_.keyframes.size() - 1;
}

void Arranger::updateIdle(float dt, ButtonSet buttonSet) {
  CORRADE_INTERNAL_ASSERT(heldObjId_ == -1 && !linkAnimOpt_);

  if (buttonSet & Button::Undo) {
    if (session_.userActions.size() > 1) {
      undoPreviousUserAction();
    }
  }

  auto ray = renderCamera_->unproject(cursor_);
  esp::physics::RaycastResults raycastResults = simulator_->castSphere(
      ray, config_.pickerSphereRadius, PICKER_COLLISION_GROUP);

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

      debugLineRender_->drawCircle(link->getTranslation(), 0.3f,
                                   config_.colors.artObj, CIRCLE_NUM_SEGMENTS,
                                   Mn::Vector3(0, 1, 0));

      if (buttonSet & Button::Primary) {
        startMoveArticulatedLink(artObjId, linkId,
                                 shouldMoveLinkToLowerLimit(artObjId, linkId));
      }
    }
  }

  if (mouseoverRigidObjId != -1) {
    debugLineRender_->drawCircle(
        simulator_->getTranslation(mouseoverRigidObjId),
        getDisplayRadiusForObject(*simulator_, mouseoverRigidObjId) * 1.5,
        config_.colors.rigidObj, CIRCLE_NUM_SEGMENTS, Mn::Vector3(0, 1, 0));

    if (buttonSet & Button::Primary) {
      startMoveRigidObject(mouseoverRigidObjId, recentHeldObjRotIndex_);
    }
  }

  debugLineRenderLineLists();
}

void Arranger::debugLineRenderLineLists() {
  const auto color = config_.colors.debugLines;
  for (const auto& list : config_.debugLineLists) {
    for (int i = 0; i < list.verts.size() - 1; i++) {
      debugLineRender_->drawLine(list.verts[i], list.verts[i + 1], color);
    }
  }
}

bool Arranger::tryDropHeldObj(const Mn::Vector3& dropPos) {
  bool foundContactFreePos = false;

  CORRADE_INTERNAL_ASSERT(heldObjId_ != -1);
  auto heldObj = simulator_->getRigidObjectManager()->getObjectByID(heldObjId_);

  Mn::Vector3 queryPos = dropPos;
  static float searchStartOffset = -0.05;
  queryPos.y() -= searchStartOffset;

  // search up from hitPos (using a large step size) to find a collision-free
  // position. Then, fine-tune by searching down using a small step size.
  const float maxOffsetY = config_.placementSearchHeight;
  static float largeOffsetStep = 0.03f;
  float tmpLastCollisionFreeY = 10000.f;
  for (float offsetY = 0; offsetY < maxOffsetY; offsetY += largeOffsetStep) {
    heldObj->setTranslation(queryPos);
    if (!simulator_->contactTest(heldObjId_)) {
      float coarseSearchY = queryPos.y();

      static float smallOffsetStep = 0.002f;
      for (float smallOffsetY = 0.f; smallOffsetY < largeOffsetStep;
           smallOffsetY += smallOffsetStep) {
        queryPos.y() -= smallOffsetStep;
        heldObj->setTranslation(queryPos);
        if (simulator_->contactTest(heldObjId_)) {
          queryPos.y() += smallOffsetStep;  // undo the last small offset
          break;
        }
        tmpLastCollisionFreeY = queryPos.y();
      }
      // next, add a bit of pad, but don't go above coarseSearchY
      static float padY = 0.003f;
      queryPos.y() = std::min(queryPos.y() + padY, coarseSearchY);

      // If dropOffsetY_ > 0, try the requested raised position. If it's
      // non collision-free, abort.
      CORRADE_INTERNAL_ASSERT(dropOffsetY_ >= 0.f);
      if (dropOffsetY_ > 0.f) {
        queryPos.y() += dropOffsetY_;
        heldObj->setTranslation(queryPos);
        if (simulator_->contactTest(heldObjId_)) {
          break;
        }
      }

      foundContactFreePos = true;
      heldObj->setTranslation(queryPos);
      break;
    }
    queryPos.y() += largeOffsetStep;
  }

  return foundContactFreePos;
}

void Arranger::cancelHeldObject() {
  auto heldObj = simulator_->getRigidObjectManager()->getObjectByID(heldObjId_);
  heldObj->setMotionType(esp::physics::MotionType::DYNAMIC);
  heldObj->overrideCollisionGroup(esp::physics::CollisionGroup::Dynamic);
  heldObjId_ = -1;
  // pop keyframe at start of this action
  session_.keyframes.pop_back();
  activeUserAction_ = Cr::Containers::NullOpt;
  simulator_->restoreFromPhysicsKeyframe(session_.keyframes.back(),
                                         /*activate*/ false);
  actionPhysicsStepCounter_ = 0;
  ESP_DEBUG() << "Canceling user action " << session_.userActions.size();
}

void Arranger::updateForHeldObject(float dt, ButtonSet buttonSet) {
  auto heldObj = simulator_->getRigidObjectManager()->getObjectByID(heldObjId_);

  static float dropInc = 0.01f;  // todo: tunable
  static float maxDropOffsetY = 0.5f;
  if (buttonSet & Button::Lower) {
    dropOffsetY_ = std::max(0.f, dropOffsetY_ - dropInc);
  } else if (buttonSet & Button::Raise) {
    dropOffsetY_ = std::min(maxDropOffsetY, dropOffsetY_ + dropInc);
  }

  if (buttonSet & Button::Undo) {
    cancelHeldObject();
    return;
  }

  if (buttonSet & Button::Secondary) {
    recentHeldObjRotIndex_ =
        (recentHeldObjRotIndex_ + 1) % getNumRotationIndices();
    heldObj->setRotation(getRotationByIndex(recentHeldObjRotIndex_));
  }

  // hide held object. move out of sight. This is for visuals, but also to
  // avoid screwing up the sphere-cast we're about to do.
  const Mn::Vector3 hiddenPos(0.f, -1000.f, 0.f);
  heldObj->setTranslation(hiddenPos);

  auto ray = renderCamera_->unproject(cursor_);
  // use sphere cast instead of raycast so that we can easily hit narrow
  // objects like the dishwasher rack
  esp::physics::RaycastResults raycastResults = simulator_->castSphere(
      ray, config_.pickerSphereRadius, PICKER_COLLISION_GROUP);

  bool foundPreviewPos = false;
  if (raycastResults.hasHits()) {
    // For the hit position, use the center of the sphere at the point of
    // collision. This is closer to the cursor than the hit point and so will
    // give the user more control.
    Mn::Vector3 hitPos =
        ray.origin + ray.direction * raycastResults.hits[0].rayDistance;

    foundPreviewPos = tryDropHeldObj(hitPos);

#if 0
    if (foundPreviewPos) {
      debugLineRender_->drawCircle(
          queryPos,
          getDisplayRadiusForObject(*simulator_, heldObjId_) * 1.5,
          Mn::Color4::green(), CIRCLE_NUM_SEGMENTS, Mn::Vector3(0, 1, 0));
    }
#endif

    visualizeHeldObject(hitPos, foundPreviewPos);
  }

  if (!foundPreviewPos) {
    // hide held object because we didn't find a preview position for it. move
    // out of sight.
    const Mn::Vector3 hiddenPos(0.f, -1000.f, 0.f);
    heldObj->setTranslation(hiddenPos);
  }

  if (foundPreviewPos && buttonSet & Button::Primary) {
    heldObj->setMotionType(esp::physics::MotionType::DYNAMIC);
    heldObj->overrideCollisionGroup(esp::physics::CollisionGroup::Dynamic);
    heldObjId_ = -1;
    waitingForSceneRest_ = true;
    restStartPhysicsStepCount_ = actionPhysicsStepCounter_;
  }
}

void Arranger::visualizeHeldObject(const Mn::Vector3& pickerHitPos,
                                   bool foundPreviewPos) {
  auto color = foundPreviewPos ? config_.colors.goodPlacement
                               : config_.colors.badPlacement;
  static float lineLen = 1.f;
  debugLineRender_->drawLine(
      pickerHitPos, pickerHitPos - Mn::Vector3(0.f, lineLen, 0.f), color);

  if (foundPreviewPos && config_.colors.placementGuide.a() > 0) {
    auto heldObj =
        simulator_->getRigidObjectManager()->getObjectByID(heldObjId_);
    debugLineRender_->pushTransform(heldObj->getTransformation());

    static float scale = 0.003f;
    static float fudge = 0.08f;
    int dim = std::min(
        120, int((getDisplayRadiusForObject(*simulator_, heldObjId_) + fudge) /
                 scale));
    debugLineRender_->pushTransform(
        Mn::Matrix4::scaling(Mn::Vector3(scale, scale, scale)));
    for (int row = -dim; row <= dim; row++) {
      debugLineRender_->drawTransformedLine(Mn::Vector3(-dim, 0.f, row),
                                            Mn::Vector3(dim, 0.f, row),
                                            config_.colors.placementGuide);
    }
    debugLineRender_->popTransform();
    debugLineRender_->popTransform();
  }
}

void Arranger::updateWaitingForSceneRest(float dt, ButtonSet buttonSet) {
  CORRADE_INTERNAL_ASSERT(waitingForSceneRest_);

  float settleTime = (actionPhysicsStepCounter_ - restStartPhysicsStepCount_) *
                     physicsTimestep_;
  bool cancelRest = (buttonSet & Button::Secondary) ||
                    (settleTime >= config_.physicsMaxSettleTime);

  if (cancelRest) {
    forcePhysicsSceneAsleep(*simulator_);
    waitingForSceneRest_ = false;
    userInputStatus_.clear();
  } else {
    int count = markAndCountActivePhysicsObjects();
    if (count == 0) {
      waitingForSceneRest_ = false;
      userInputStatus_.clear();
    } else {
      userInputStatus_ = "settling... (F to skip)";
    }
  }

  if (!waitingForSceneRest_) {
    endUserAction();
  }
}

void Arranger::endUserAction() {
  // If we've stepped physics since the most recent keyframe, let's save a new
  // keyframe to capture the exact end of the action
  //
  if (actionPhysicsStepCounter_ > 0) {
    session_.keyframes.emplace_back(simulator_->savePhysicsKeyframe());
    actionPhysicsStepCounter_ = 0;
  }

  CORRADE_INTERNAL_ASSERT(activeUserAction_);
  activeUserAction_->endFrame = session_.keyframes.size() - 1;
  ESP_DEBUG() << "Ending user action " << session_.userActions.size()
              << " on frame " << activeUserAction_->endFrame;
  session_.userActions.emplace_back(std::move(*activeUserAction_));
  activeUserAction_ = Cr::Containers::NullOpt;

  // rewind to recent physics keyframe; this will put the physics sim into
  // a more consistent state, e.g. zero velocities
  simulator_->restoreFromPhysicsKeyframe(session_.keyframes.back(),
                                         /*activate*/ false);
  actionPhysicsStepCounter_ = 0;

  // stepping here allows bodies that want to go to sleep to actually go to
  // sleep
  simulator_->stepWorld(-1);
}

void Arranger::update(float dt, ButtonSet buttonSet) {
  CORRADE_INTERNAL_ASSERT(!isHeadless_);
  // see src/deps/bullet3/src/BulletDynamics/Dynamics/btRigidBody.cpp
  gDeactivationTime = config_.physicsDeactivationTime;

  bool doFreezePhysicsTime = true;

  if (linkAnimOpt_) {
    updateForLinkAnimation(dt, buttonSet);
  } else if (heldObjId_ != -1) {
    updateForHeldObject(dt, buttonSet);
  } else if (waitingForSceneRest_) {
    updateWaitingForSceneRest(dt, buttonSet);
  } else {
    updateIdle(dt, buttonSet);
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

  updateCamera(dt, buttonSet);
}

int Arranger::getNumRotationIndices() const {
  return 12;
}

Mn::Quaternion Arranger::getRotationByIndex(int index) {
  auto ninetyAboutUpAxis =
      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(0.f, 1.f, 0.f));
  static Mn::Quaternion rots[] = {
      Mn::Quaternion(Mn::Math::IdentityInit),
      ninetyAboutUpAxis,

      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(1.f, 0.f, 0.f)),
      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(1.f, 0.f, 0.f)) *
          ninetyAboutUpAxis,

      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(0.f, 0.f, 1.f)),
      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(0.f, 0.f, 1.f)) *
          ninetyAboutUpAxis,

      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(-1.f, 0.f, 0.f)),
      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(-1.f, 0.f, 0.f)) *
          ninetyAboutUpAxis,

      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(0.f, 0.f, -1.f)),
      Mn::Quaternion::rotation(Mn::Deg(90.f), Mn::Vector3(0.f, 0.f, -1.f)) *
          ninetyAboutUpAxis,

      Mn::Quaternion::rotation(Mn::Deg(180.f), Mn::Vector3(1.f, 0.f, 0.f)),
      Mn::Quaternion::rotation(Mn::Deg(180.f), Mn::Vector3(1.f, 0.f, 0.f)) *
          ninetyAboutUpAxis};
  return rots[index];
}

void Arranger::updatePhysicsWorld(float dt) {
  timeSinceLastSimulation_ += dt;

  constexpr int maxStepsPerUpdate = 6;  // capped to prevent low fps in app
  int numStepsThisUpdate = 0;

  static int stepCounter = 0;
  // todo: prevent huge framerate drop
  while (timeSinceLastSimulation_ >= physicsTimestep_) {
    // step physics at a fixed rate
    simulator_->stepWorld(-1);
    timeSinceLastSimulation_ -= physicsTimestep_;

    actionPhysicsStepCounter_++;
    numStepsThisUpdate++;
    stepCounter++;
    if (actionPhysicsStepCounter_ % config_.keyframeSavePeriod == 0) {
      session_.keyframes.emplace_back(simulator_->savePhysicsKeyframe());
    }

    if (numStepsThisUpdate == maxStepsPerUpdate) {
      timeSinceLastSimulation_ = 0.f;
      break;
    }
  }

#if 0  // debug reference code
  static float elapsedTime = 0.f;
  elapsedTime += dt;
  if (stepCounter % 10 == 9) {
    float stepRate = stepCounter / elapsedTime;
    ESP_DEBUG() << "stepRate: " << stepRate;
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
      if (debug3dText_) {
        debug3dText_->addText("active", artObj->getTranslation(),
                              config_.colors.artObj);
      }
      count++;
    }
  }

  handles = simulator_->getRigidObjectManager()->getObjectHandlesBySubstring();
  for (const auto& handle : handles) {
    auto rigidObj =
        simulator_->getRigidObjectManager()->getObjectByHandle(handle);
    if (rigidObj->isActive()) {
      if (debug3dText_) {
        debug3dText_->addText("active", rigidObj->getTranslation(),
                              config_.colors.rigidObj);
      }
      count++;
    }
  }

  return count;
}

void Arranger::configureCollisionGroups() {
  const esp::physics::CollisionGroup staticUrdfLinksGroup =
      esp::physics::CollisionGroup::UserGroup0;

  // configure collision groups before initializing sim
  esp::physics::CollisionGroupHelper::setMaskForGroup(
      staticUrdfLinksGroup, esp::physics::CollisionGroups());
  esp::physics::CollisionGroupHelper::setMaskForGroup(
      PICKER_COLLISION_GROUP, esp::physics::CollisionGroups());

  auto enableInteracts = [](esp::physics::CollisionGroup a,
                            esp::physics::CollisionGroup b) {
    esp::physics::CollisionGroupHelper::setGroupInteractsWith(a, b, true);
    esp::physics::CollisionGroupHelper::setGroupInteractsWith(b, a, true);
  };

  // note that Static and Dynamic are already set to interact with certain
  // non-user groups

  enableInteracts(staticUrdfLinksGroup, staticUrdfLinksGroup);
  enableInteracts(staticUrdfLinksGroup, esp::physics::CollisionGroup::Dynamic);
  enableInteracts(staticUrdfLinksGroup, PICKER_COLLISION_GROUP);

  enableInteracts(PICKER_COLLISION_GROUP, esp::physics::CollisionGroup::Static);
  enableInteracts(PICKER_COLLISION_GROUP,
                  esp::physics::CollisionGroup::Dynamic);
  enableInteracts(PICKER_COLLISION_GROUP,
                  esp::physics::CollisionGroup::Kinematic);
}

void Arranger::startMoveRigidObject(int rigidObjId, int rotIndex) {
  CORRADE_INTERNAL_ASSERT(rigidObjId != -1);
  heldObjId_ = rigidObjId;
  auto heldObj = simulator_->getRigidObjectManager()->getObjectByID(heldObjId_);
  heldObj->setMotionType(esp::physics::MotionType::KINEMATIC);
  heldObj->overrideCollisionGroup(esp::physics::CollisionGroup::Dynamic);
  heldObj->setRotation(getRotationByIndex(rotIndex));

  // start user action for this rigid obj
  CORRADE_INTERNAL_ASSERT(!activeUserAction_);
  activeUserAction_ = UserAction();
  activeUserAction_->rigidObj =
      simulator_->getRigidObjectManager()->getObjectHandleByID(heldObjId_);

  // save a start frame that includes the dropped object at it's new pose
  session_.keyframes.emplace_back(simulator_->savePhysicsKeyframe());
  actionPhysicsStepCounter_ = 0;

  activeUserAction_->startFrame = session_.keyframes.size() - 1;
  ESP_DEBUG() << "Starting user action " << Corrade::Utility::Debug::nospace
              << session_.userActions.size() << " on frame "
              << activeUserAction_->startFrame << " with rigidObj "
              << activeUserAction_->rigidObj;
}

void Arranger::moveArticulatedLink(int artObjId,
                                   int linkId,
                                   bool moveToLowerLimit) {
  auto ok = startMoveArticulatedLink(artObjId, linkId, moveToLowerLimit);
  ESP_CHECK(ok, "startMoveArticulatedLink failed for linkId " << linkId);

  do {
    const auto dt = physicsTimestep_;
    updateForLinkAnimation(dt, ButtonSet());
    if (!linkAnimOpt_) {
      break;
    }
    updatePhysicsWorld(dt);
  } while (true);

  waitForRest();
}

// for simple one-click interactive usage, use a heuristic to decide open/close
bool Arranger::shouldMoveLinkToLowerLimit(int artObjId, int linkId) {
  auto artObj =
      simulator_->getArticulatedObjectManager()->getObjectByID(artObjId);

  int jointPosOffset = artObj->getLinkJointPosOffset(linkId);
  const auto pair = artObj->getJointPositionLimits();
  const auto& lowerLimits = pair.first;
  const auto& upperLimits = pair.second;
  if (lowerLimits[jointPosOffset] == INFINITY) {
    // doesn't matter what we return here because we won't move this link anyway
    return true;
  }

  float jointPos = artObj->getJointPositions()[jointPosOffset];
  float upperLimit = upperLimits[jointPosOffset];
  float lowerLimit = lowerLimits[jointPosOffset];
  float animEndPos;

  bool useLowerLimit = false;
  auto linkMemoryKey =
      artObjId * 1000 + linkId;  // sloppy hash of object id and link id

  // Decide whether to move to lower limit or upper limit. The logic
  // here is (1) if the link is near one limit, choose the other
  // limit, (2) do the opposite of last time we interacted, or (3) if
  // it's the first interaction, move to the further-away limit. We're
  // keeping the UX simple and limited to a single "interact" button
  // (not explicit "open" and "close" buttons). The complexity here is
  // to handle weird cases where links get stuck/blocked.
  float distFromUpper = std::abs(jointPos - upperLimit);
  float distFromLower = std::abs(jointPos - lowerLimit);
  if (distFromUpper < config_.link.distEps) {
    useLowerLimit = true;
  } else if (distFromLower < config_.link.distEps) {
    useLowerLimit = false;
  } else if (linkAnimMemory_.count(linkMemoryKey)) {
    useLowerLimit = !linkAnimMemory_[linkMemoryKey];
  } else {
    useLowerLimit = (distFromUpper < distFromLower);
  }

  // remember our interaction
  linkAnimMemory_[linkMemoryKey] = useLowerLimit;

  return useLowerLimit;
}

bool Arranger::startMoveArticulatedLink(int artObjId,
                                        int linkId,
                                        bool moveToLowerLimit) {
  auto artObj =
      simulator_->getArticulatedObjectManager()->getObjectByID(artObjId);

  int numJointPos = artObj->getLinkNumJointPos(linkId);
  if (numJointPos != 1) {
    return false;
  }

  int jointPosOffset = artObj->getLinkJointPosOffset(linkId);
  const auto pair = artObj->getJointPositionLimits();
  const auto& lowerLimits = pair.first;
  const auto& upperLimits = pair.second;
  if (lowerLimits[jointPosOffset] == INFINITY) {
    return false;
  }

  float jointPos = artObj->getJointPositions()[jointPosOffset];
  float upperLimit = upperLimits[jointPosOffset];
  float lowerLimit = lowerLimits[jointPosOffset];
  float animEndPos;

  bool useLowerLimit = moveToLowerLimit;

  animEndPos = useLowerLimit ? lowerLimit : upperLimit;

  linkAnimOpt_ = LinkAnimation{.artObjId = artObjId,
                               .linkId = linkId,
                               .jointPosOffset = jointPosOffset,
                               .startPos = jointPos,
                               .endPos = animEndPos,
                               .animTimer = 0.f};
  userInputStatus_ = "animating... (F to skip)";

  // start user action for this articulated obj
  CORRADE_INTERNAL_ASSERT(!activeUserAction_);
  activeUserAction_ = UserAction();
  activeUserAction_->articulatedObj =
      simulator_->getArticulatedObjectManager()->getObjectHandleByID(artObjId);
  activeUserAction_->articulatedLink = linkId;

  // duplicate the end frame from the last action
  session_.keyframes.push_back(session_.keyframes.back());
  actionPhysicsStepCounter_ = 0;

  activeUserAction_->startFrame = session_.keyframes.size() - 1;
  ESP_DEBUG() << "Starting user action " << session_.userActions.size()
              << " on frame " << activeUserAction_->startFrame
              << " with articulatedObj " << activeUserAction_->articulatedObj
              << " link " << activeUserAction_->articulatedLink;
  return true;
}

bool Arranger::tryMoveRigidObject(int rigidObjId,
                                  int rotIndex,
                                  const Mn::Vector3& targetPos,
                                  float dropOffsetY) {
  CORRADE_INTERNAL_ASSERT(isHeadless_);
  CORRADE_INTERNAL_ASSERT(!waitingForSceneRest_);

  startMoveRigidObject(rigidObjId, rotIndex);

  // todo: refactor to not duplicate this move-finish code with
  // updateForHeldObject
  {
    CORRADE_INTERNAL_ASSERT(heldObjId_ != -1);
    auto heldObj =
        simulator_->getRigidObjectManager()->getObjectByID(heldObjId_);

    dropOffsetY_ = dropOffsetY;

    bool foundContactFreePos = tryDropHeldObj(targetPos);
    if (foundContactFreePos) {
      heldObj->setMotionType(esp::physics::MotionType::DYNAMIC);
      heldObj->overrideCollisionGroup(esp::physics::CollisionGroup::Dynamic);
      heldObjId_ = -1;
      waitingForSceneRest_ = true;
      restStartPhysicsStepCount_ = actionPhysicsStepCounter_;
    } else {
      cancelHeldObject();
      return false;
    }
  }

  waitForRest();
  return true;
}

void Arranger::waitForRest() {
  CORRADE_INTERNAL_ASSERT(isHeadless_);

  do {
    const auto dt = physicsTimestep_;
    updateWaitingForSceneRest(dt, ButtonSet());
    if (!waitingForSceneRest_) {
      break;
    }
    updatePhysicsWorld(dt);
  } while (true);
}

}  // namespace arrange
}  // namespace esp
