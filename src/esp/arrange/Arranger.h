// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ARRANGE_ARRANGER_H_
#define ESP_ARRANGE_ARRANGER_H_

#include <unordered_map>

#include "esp/core/random.h"
#include "esp/gfx/Debug3DText.h"
#include "esp/gfx/DebugRender.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/sim/Simulator.h"

#include "Config.h"
#include "Session.h"

namespace esp {
namespace arrange {

class Arranger {
 public:
  enum class Button : Magnum::UnsignedShort {
    Primary = 1 << 0,
    Secondary = 1 << 1,
    Undo = 1 << 2,
    NextCamera = 1 << 3,
    PrevCamera = 1 << 4,
    Raise = 1 << 5,
    Lower = 1 << 6,
  };

  typedef Corrade::Containers::EnumSet<Button> ButtonSet;

  // for headless usage, pass nullptr for renderCamera, debugRender, and
  // debug3dText
  Arranger(Config&& config,
           esp::sim::Simulator* simulator,
           esp::gfx::RenderCamera* renderCamera,
           esp::gfx::DebugRender* debugRender,
           esp::gfx::Debug3DText* debug3dText);

  void setCursor(const Magnum::Vector2i& cursor) { cursor_ = cursor; }

  void update(float dt, ButtonSet buttonSet);

  const Session& getSession() const { return session_; }

  static void configureCollisionGroups();

  void setConfig(const Config& config) { config_ = config; }

  // headless API
  bool tryMoveRigidObject(int rigidObjId,
                          int rotIndex,
                          const Mn::Vector3& targetPos,
                          float dropOffsetY);
  bool moveArticulatedLink(int artObjId, int linkId, bool moveToLowerLimit);

  int getNumRotationIndices() const;

 private:
  struct LinkAnimation {
    int artObjId = -1;
    int linkId = -1;
    int jointPosOffset = -1;
    float startPos = -1.f;
    float endPos = -1.f;
    float animTimer = 0.f;
  };

  Mn::Quaternion getRotationByIndex(int index);
  void updatePhysicsWorld(float dt);
  void updateForLinkAnimation(float dt, ButtonSet buttonSet);
  void updateIdle(float dt, ButtonSet buttonSet);
  void updateForHeldObject(float dt, ButtonSet buttonSet);
  void updateWaitingForSceneRest(float dt, ButtonSet buttonSet);
  int markAndCountActivePhysicsObjects();
  void endUserAction();
  Mn::Matrix4 updateCamera(float dt, ButtonSet buttonSet);
  void visualizeHeldObject(const Magnum::Vector3& pickerHitPos,
                           bool foundPreviewPos);
  bool shouldMoveLinkToLowerLimit(int artObjId, int linkId);
  bool startMoveArticulatedLink(int artObjId,
                                int linkId,
                                bool moveToLowerLimit);
  void startMoveRigidObject(int rigidObjId, int rotIndex);
  bool tryDropHeldObj(const Mn::Vector3& dropPos);
  void waitForRest();
  void cancelHeldObject();
  void debugRenderLineLists();

  Config config_;
  bool isHeadless_;
  esp::sim::Simulator* simulator_ = nullptr;
  esp::gfx::RenderCamera* renderCamera_ = nullptr;
  esp::gfx::DebugRender* debugRender_ = nullptr;
  esp::gfx::Debug3DText* debug3dText_ = nullptr;
  Magnum::Vector2i cursor_;
  int heldObjId_ = -1;
  int recentHeldObjRotIndex_ = 0;
  Corrade::Containers::Optional<LinkAnimation> linkAnimOpt_;
  float timeSinceLastSimulation_ = 0.f;
  bool waitingForSceneRest_ = true;  // wait for rest on scene load
  std::string userInputStatus_;
  Session session_;
  Corrade::Containers::Optional<UserAction> activeUserAction_;
  float physicsTimestep_ = 0.f;
  int actionPhysicsStepCounter_ = 0;
  int restStartPhysicsStepCount_ = 0;
  std::unordered_map<int, bool> linkAnimMemory_;
  int cameraIndex_ = 0;
  float dropOffsetY_ = 0.f;
  core::Random random_;
};

}  // namespace arrange
}  // namespace esp

#endif