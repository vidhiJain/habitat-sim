// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Arranger.h"

namespace esp {
namespace arrange_recorder {

namespace {

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

}  // namespace

Arranger::Arranger(esp::sim::Simulator* simulator,
                   esp::gfx::RenderCamera* renderCamera,
                   esp::gfx::DebugRender* debugRender)
    : simulator_(simulator),
      renderCamera_(renderCamera),
      debugRender_(debugRender) {
  existingObjectIds_ = simulator_->getExistingObjectIDs();
}

void Arranger::update(float dt, bool isPrimaryButton, bool isSecondaryButton) {
  if (linkAnimOpt_) {
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
            ? Mn::Math::lerp(startVel, maxVel,
                             Mn::Math::pow(smoothstep(calcLerpFraction(
                                               0.f, 0.5, animFraction)),
                                           exp0))
            : Mn::Math::lerp(maxVel, endVel,
                             Mn::Math::pow(smoothstep(calcLerpFraction(
                                               0.5f, 1.0, animFraction)),
                                           exp1));
    // snap to zero vel if near end pos (anim is finished)
    float vel = isNearEndPos ? 0.0f : forceDir * velMag;

    auto jointVels = artObj->getJointVelocities();
    jointVels[linkAnim.jointPosOffset] = vel;
    artObj->setJointVelocities(jointVels);

    static float animMaxDuration = 9.f;  // todo: tunable
    if (isNearEndPos || linkAnim.animTimer > animMaxDuration) {
      if (isNearEndPos) {
        // animation is finished so snap to end pos
        auto jointPositions = artObj->getJointPositions();
        jointPositions[linkAnim.jointPosOffset] = linkAnim.endPos;
        artObj->setJointPositions(jointPositions);
      }

      linkAnimOpt_ = Cr::Containers::NullOpt;
    }
    return;
  }

  if (heldObjId_ == -1) {
    auto ray = renderCamera_->unproject(cursor_);
    static float sphereRadius = 0.02f;
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
            const auto lowerLimits = artObj->getJointPositionLimits();
            const auto upperLimits =
                artObj->getJointPositionLimits(/*upperLimits*/ true);
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
      }
    }
  } else {
    auto heldObj =
        simulator_->getRigidObjectManager()->getObjectByID(heldObjId_);

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
    static float sphereRadius = 0.02f;
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
    }

    if (!foundPreviewPos) {
      // hide held object. move out of sight. This is for visuals, but also to
      // not screw up the raycast we're about to do.
      const Mn::Vector3 hiddenPos(0.f, -100.f, 0.f);
      heldObj->setTranslation(hiddenPos);
    }
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

}  // namespace arrange_recorder
}  // namespace esp
