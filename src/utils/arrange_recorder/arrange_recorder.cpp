// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <math.h>
#include <stdlib.h>
#include <ctime>
#include <fstream>

#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/Assert.h>
#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Platform/GlfwApplication.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/Timeline.h>
#include <Magnum/ImGuiIntegration/Context.hpp>

#include "esp/arrange/Arranger.h"
#include "esp/core/configure.h"
#include "esp/core/esp.h"
#include "esp/gfx/DebugRender.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/physics/configure.h"
#include "esp/scene/SceneNode.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sim/Simulator.h"

constexpr float moveSensitivity = 0.07f;
constexpr float lookSensitivity = 0.9f;
constexpr float rgbSensorHeight = 1.5f;

// for ease of access
namespace Cr = Corrade;
namespace Mn = Magnum;

namespace {

//! return current time as string in format
//! "year_month_day_hour-minutes-seconds"
std::string getCurrentTimeString() {
  time_t now = time(0);
  tm* ltm = localtime(&now);
  return std::to_string(1900 + ltm->tm_year) + "_" +
         std::to_string(1 + ltm->tm_mon) + "_" + std::to_string(ltm->tm_mday) +
         "_" + std::to_string(ltm->tm_hour) + "-" +
         std::to_string(ltm->tm_min) + "-" + std::to_string(ltm->tm_sec);
}

class ArrangeRecorder : public Mn::Platform::Application {
 public:
  explicit ArrangeRecorder(const Arguments& arguments);

 private:
  // Keys for moving/looking are recorded according to whether they are
  // currently being pressed
  std::map<KeyEvent::Key, bool> keysPressed = {
      {KeyEvent::Key::Left, false}, {KeyEvent::Key::Right, false},
      {KeyEvent::Key::Up, false},   {KeyEvent::Key::Down, false},
      {KeyEvent::Key::A, false},    {KeyEvent::Key::D, false},
      {KeyEvent::Key::S, false},    {KeyEvent::Key::W, false},
      {KeyEvent::Key::X, false},    {KeyEvent::Key::Z, false}};

  void drawEvent() override;
  void viewportEvent(ViewportEvent& event) override;
  void mousePressEvent(MouseEvent& event) override;
  void mouseReleaseEvent(MouseEvent& event) override;
  void mouseMoveEvent(MouseMoveEvent& event) override;
  void mouseScrollEvent(MouseScrollEvent& event) override;
  void keyPressEvent(KeyEvent& event) override;
  void keyReleaseEvent(KeyEvent& event) override;
  void moveAndLook(int repetitions);

  void createSimulator();
  void saveScenePhysicsKeyframe();
  void restoreFromScenePhysicsKeyframe();
  void checkSaveArrangerSession();
  std::string getActiveSceneSimplifiedName();
  std::string findNewSessionSaveFilepath();
  std::string getActiveScenePhysicsKeyframeFilepath();

  esp::sensor::CameraSensor& getAgentCamera() {
    esp::sensor::Sensor& cameraSensor =
        agentBodyNode_->getNodeSensorSuite().get("rgba_camera");
    return static_cast<esp::sensor::CameraSensor&>(cameraSensor);
  }

  // single inline for logging agent state msgs, so can be easily modified
  inline void showAgentStateMsg() {
    std::stringstream strDat("");

    strDat << "Agent position "
           << Eigen::Map<esp::vec3f>(agentBodyNode_->translation().data())
           << " ";
    strDat << "Agent orientation "
           << esp::quatf(agentBodyNode_->rotation()).coeffs().transpose()
           << " ";
    strDat << "Camera position "
           << Eigen::Map<esp::vec3f>(renderCamera_->node().translation().data())
           << " ";
    strDat << "Camera orientation "
           << esp::quatf(renderCamera_->node().rotation()).coeffs().transpose();

    auto str = strDat.str();
    if (str.size() > 0) {
      LOG(INFO) << str;
    }
  }

  // The simulator object backend for this viewer instance
  std::unique_ptr<esp::sim::Simulator> simulator_;

  // store these so we can recreate the simulator
  Cr::Utility::Arguments args_;
  esp::sim::SimulatorConfiguration simConfig_;

  // The managers belonging to the simulator
  std::shared_ptr<esp::metadata::managers::ObjectAttributesManager>
      objectAttrManager_ = nullptr;
  std::shared_ptr<esp::metadata::managers::AssetAttributesManager>
      assetAttrManager_ = nullptr;
  std::shared_ptr<esp::metadata::managers::StageAttributesManager>
      stageAttrManager_ = nullptr;
  std::shared_ptr<esp::metadata::managers::PhysicsAttributesManager>
      physAttrManager_ = nullptr;

  bool debugBullet_ = false;

  esp::scene::SceneNode* agentBodyNode_ = nullptr;

  const int defaultAgentId_ = 0;
  esp::agent::Agent::ptr defaultAgent_ = nullptr;

  // Scene or stage file to load
  std::string sceneFileName;
  esp::gfx::RenderCamera* renderCamera_ = nullptr;
  esp::scene::SceneGraph* activeSceneGraph_ = nullptr;

  Mn::Timeline timeline_;

  Mn::ImGuiIntegration::Context imgui_{Mn::NoCreate};
  bool showFPS_ = false;

  esp::gfx::DebugRender debugRender_;
  esp::gfx::Debug3DText debug3dText_;
  std::unique_ptr<esp::arrange::Arranger> arranger_;
  std::string sessionSaveFilepath_;
  int numSavedArrangerUserActions_ = 0;
  Mn::Vector2i recentCursorPos_;

  void bindRenderTarget();
};

void addSensors(esp::agent::AgentConfiguration& agentConfig,
                const Cr::Utility::Arguments& args) {
  const auto viewportSize = Mn::GL::defaultFramebuffer.viewport().size();

  auto addCameraSensor = [&](const std::string& uuid,
                             esp::sensor::SensorType sensorType) {
    agentConfig.sensorSpecifications.emplace_back(
        esp::sensor::CameraSensorSpec::create());
    auto spec = static_cast<esp::sensor::CameraSensorSpec*>(
        agentConfig.sensorSpecifications.back().get());

    spec->uuid = uuid;
    spec->sensorSubType = args.isSet("orthographic")
                              ? esp::sensor::SensorSubType::Orthographic
                              : esp::sensor::SensorSubType::Pinhole;
    spec->sensorType = sensorType;
    if (sensorType == esp::sensor::SensorType::Depth ||
        sensorType == esp::sensor::SensorType::Semantic) {
      spec->channels = 1;
    }
    spec->position = {0.0f, 1.5f, 0.0f};
    spec->orientation = {0, 0, 0};
    spec->resolution = esp::vec2i(viewportSize[1], viewportSize[0]);
  };

  // add the camera color sensor
  // for historical reasons, we call it "rgba_camera"
  addCameraSensor("rgba_camera", esp::sensor::SensorType::Color);
}

void ArrangeRecorder::createSimulator() {
  auto& args = args_;

  if (simulator_) {
    simulator_->close();
    simulator_->reconfigure(simConfig_);
  } else {
    simulator_ = esp::sim::Simulator::create_unique(simConfig_);
  }

  objectAttrManager_ = simulator_->getObjectAttributesManager();
  assetAttrManager_ = simulator_->getAssetAttributesManager();
  stageAttrManager_ = simulator_->getStageAttributesManager();
  physAttrManager_ = simulator_->getPhysicsAttributesManager();

  // NavMesh customization options
  if (args.isSet("disable-navmesh")) {
    if (simulator_->getPathFinder()->isLoaded()) {
      simulator_->setPathFinder(esp::nav::PathFinder::create());
    }
  }

  // configure and initialize default Agent and Sensor
  auto agentConfig = esp::agent::AgentConfiguration();
  agentConfig.height = rgbSensorHeight;
  agentConfig.actionSpace = {
      // setup viewer action space
      {"moveForward",
       esp::agent::ActionSpec::create(
           "moveForward",
           esp::agent::ActuationMap{{"amount", moveSensitivity}})},
      {"moveBackward",
       esp::agent::ActionSpec::create(
           "moveBackward",
           esp::agent::ActuationMap{{"amount", moveSensitivity}})},
      {"moveLeft",
       esp::agent::ActionSpec::create(
           "moveLeft", esp::agent::ActuationMap{{"amount", moveSensitivity}})},
      {"moveRight",
       esp::agent::ActionSpec::create(
           "moveRight", esp::agent::ActuationMap{{"amount", moveSensitivity}})},
      {"moveDown",
       esp::agent::ActionSpec::create(
           "moveDown", esp::agent::ActuationMap{{"amount", moveSensitivity}})},
      {"moveUp",
       esp::agent::ActionSpec::create(
           "moveUp", esp::agent::ActuationMap{{"amount", moveSensitivity}})},
      {"turnLeft",
       esp::agent::ActionSpec::create(
           "turnLeft", esp::agent::ActuationMap{{"amount", lookSensitivity}})},
      {"turnRight",
       esp::agent::ActionSpec::create(
           "turnRight", esp::agent::ActuationMap{{"amount", lookSensitivity}})},
      {"lookUp",
       esp::agent::ActionSpec::create(
           "lookUp", esp::agent::ActuationMap{{"amount", lookSensitivity}})},
      {"lookDown",
       esp::agent::ActionSpec::create(
           "lookDown", esp::agent::ActuationMap{{"amount", lookSensitivity}})},
  };

  addSensors(agentConfig, args);
  // add selects a random initial state and sets up the default controls and
  // step filter
  simulator_->addAgent(agentConfig);

  // Set up camera
  activeSceneGraph_ = &simulator_->getActiveSceneGraph();
  defaultAgent_ = simulator_->getAgent(defaultAgentId_);
  agentBodyNode_ = &defaultAgent_->node();
  renderCamera_ = getAgentCamera().getRenderCamera();

  // temp hard-coded add URDF models (soon, we can include them in the scene
  // instance file)
  const std::vector<std::string> filepaths = {
      "data/lighthouse_kitchen_dataset/urdf/dishwasher_urdf/"
      "ktc_dishwasher.urdf",
      "data/lighthouse_kitchen_dataset/urdf/kitchen_oven/kitchen_oven.urdf"};

  for (const auto& filepath : filepaths) {
    const bool fixedBase = true;
    const auto& artObjMgr = simulator_->getArticulatedObjectManager();
    const auto artObj = artObjMgr->addBulletArticulatedObjectFromURDF(
        filepath, fixedBase, 1.f, 1.f, true);
    // artObj->setMotionType(esp::physics::MotionType::KINEMATIC);
    // note: positioning will be done via restoreFromPhysicsKeyframe
  }

  // temp place agent and camera for dishwasher loading
  agentBodyNode_->setTranslation(Mn::Vector3(-0.045473, 0, -0.418929));

  auto agentRot = Mn::Quaternion({0, -0.256289, 0}, 0.9666);

  agentBodyNode_->setRotation(agentRot);
  renderCamera_->node().setTranslation(Mn::Vector3(0, 0.316604, 0.610451));
  renderCamera_->node().setRotation(
      Mn::Quaternion({-0.271441, 0, 0}, 0.962455));

  arranger_ = std::make_unique<esp::arrange::Arranger>(
      simulator_.get(), renderCamera_, &debugRender_, &debug3dText_);
  restoreFromScenePhysicsKeyframe();
}

// todo: remove all these args
ArrangeRecorder::ArrangeRecorder(const Arguments& arguments)
    : Mn::Platform::Application{
          arguments,
          Configuration{}
              .setTitle("ArrangeRecorder")
              .setSize(Mn::Vector2i(1280, 720))
              .setWindowFlags(Configuration::WindowFlag::Resizable),
          GLConfiguration{}
              .setColorBufferSize(Mn::Vector4i(8, 8, 8, 8))
              .setSampleCount(4)} {
  Cr::Utility::Arguments args;
  args.addArgument("scene")
      .setHelp("scene", "scene/stage file to load")
      .addSkippedPrefix("magnum", "engine-specific options")
      .setGlobalHelp("Rearrange a scene with the mouse")
      .addOption("dataset", "default")
      .setHelp("dataset", "dataset configuration file to use")
      .addBooleanOption("stage-requires-lighting")
      .setHelp("stage-requires-lighting",
               "Stage asset should be lit with Phong shading.")
      .addBooleanOption("debug-bullet")
      .setHelp("debug-bullet", "Render Bullet physics debug wireframes.")
      .addBooleanOption("orthographic")
      .setHelp("orthographic",
               "If specified, use orthographic camera to view scene.")
      .addBooleanOption("disable-navmesh")
      .setHelp("disable-navmesh",
               "Disable the navmesh, disabling agent navigation constraints.")
      .addBooleanOption("edit-scene-physics-keyframe")
      .setHelp("edit-scene-physics-keyframe",
               "Instead of recording an arrangement session, use the arranger "
               "to edit the scene's start state. This modifies "
               "scenes/sceneName.physics_keyframe.json.")
      .parse(arguments.argc, arguments.argv);

  const auto viewportSize = Mn::GL::defaultFramebuffer.viewport().size();

  imgui_ =
      Mn::ImGuiIntegration::Context(Mn::Vector2{windowSize()} / dpiScaling(),
                                    windowSize(), framebufferSize());

  /* Set up proper blending to be used by ImGui. There's a great chance
     you'll need this exact behavior for the rest of your scene. If not, set
     this only for the drawFrame() call. */
  Mn::GL::Renderer::setBlendEquation(Mn::GL::Renderer::BlendEquation::Add,
                                     Mn::GL::Renderer::BlendEquation::Add);
  Mn::GL::Renderer::setBlendFunction(
      Mn::GL::Renderer::BlendFunction::SourceAlpha,
      Mn::GL::Renderer::BlendFunction::OneMinusSourceAlpha);

  // Setup renderer and shader defaults
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);

  sceneFileName = args.value("scene");
  bool useBullet = true;
  if (useBullet && (args.isSet("debug-bullet"))) {
    debugBullet_ = true;
  }

  // configure and intialize Simulator
  auto simConfig = esp::sim::SimulatorConfiguration();
  simConfig.activeSceneName = sceneFileName;
  simConfig.sceneDatasetConfigFile = args.value("dataset");
  LOG(INFO) << "Dataset : " << simConfig.sceneDatasetConfigFile;
  simConfig.enablePhysics = useBullet;
  simConfig.frustumCulling = true;
  simConfig.requiresTextures = true;
  simConfig.enableGfxReplaySave = false;
  if (args.isSet("stage-requires-lighting")) {
    Mn::Debug{} << "Stage using DEFAULT_LIGHTING_KEY";
    simConfig.sceneLightSetup = esp::DEFAULT_LIGHTING_KEY;
    simConfig.overrideSceneLightDefaults = true;
  }

  simConfig_ = simConfig;
  args_ = std::move(args);
  createSimulator();

  // disable v-sync?
  // setSwapInterval(0);

  timeline_.start();

}  // end ArrangeRecorder::ArrangeRecorder

float timeSinceLastSimulation = 0.0;
void ArrangeRecorder::drawEvent() {
  // Wrap profiler measurements around all methods to render images from
  // RenderCamera
  Mn::GL::defaultFramebuffer.clear(Mn::GL::FramebufferClear::Color |
                                   Mn::GL::FramebufferClear::Depth);

  // Agent actions should occur at a fixed rate per second
  timeSinceLastSimulation += timeline_.previousFrameDuration();

  // call moveAndLook at 60 Hz
  while (timeSinceLastSimulation >= 1.0 / 60.0) {
    timeSinceLastSimulation -= 1.0 / 60.0;
    moveAndLook(1);
  }

  arranger_->setCursor(recentCursorPos_);
  arranger_->update(timeline_.previousFrameDuration(), false, false);

  checkSaveArrangerSession();

  {
    // ============= regular RGB with object picking =================
    // using polygon offset to increase mesh depth to avoid z-fighting with
    // debug draw (since lines will not respond to offset).
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::PolygonOffsetFill);
    Mn::GL::Renderer::setPolygonOffset(1.0f, 0.1f);

    // ONLY draw the content to the frame buffer but not immediately blit the
    // result to the default main buffer
    // (this is the reason we do not call displayObservation)
    simulator_->drawObservation(defaultAgentId_, "rgba_camera");
    // TODO: enable other sensors to be displayed

    Mn::GL::Renderer::setDepthFunction(
        Mn::GL::Renderer::DepthFunction::LessOrEqual);
    if (debugBullet_) {
      Mn::Matrix4 camM(renderCamera_->cameraMatrix());
      Mn::Matrix4 projM(renderCamera_->projectionMatrix());

      simulator_->physicsDebugDraw(projM * camM);
    }
    Mn::GL::Renderer::setDepthFunction(Mn::GL::Renderer::DepthFunction::Less);
    Mn::GL::Renderer::setPolygonOffset(0.0f, 0.0f);
    Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::PolygonOffsetFill);

    {
      Mn::Matrix4 camM(renderCamera_->cameraMatrix());
      Mn::Matrix4 projM(renderCamera_->projectionMatrix());
      debugRender_.setTransformationProjectionMatrix(projM * camM);
      debugRender_.flushLines();
    }

    esp::gfx::RenderTarget* sensorRenderTarget =
        simulator_->getRenderTarget(defaultAgentId_, "rgba_camera");
    CORRADE_ASSERT(
        sensorRenderTarget,
        "Error in ArrangeRecorder::drawEvent: sensor's rendering target "
        "cannot be nullptr.", );

    sensorRenderTarget->blitRgbaToDefault();
  }

  // Immediately bind the main buffer back so that the "imgui" below can work
  // properly
  Mn::GL::defaultFramebuffer.bind();

  imgui_.newFrame();

  {
    const auto viewportSize = Mn::GL::defaultFramebuffer.viewport().size();
    Mn::Matrix4 camM(renderCamera_->cameraMatrix());
    Mn::Matrix4 projM(renderCamera_->projectionMatrix());
    debug3dText_.flushToImGui(projM * camM, viewportSize);
  }

  if (showFPS_) {
    ImGui::SetNextWindowPos(ImVec2(10, 10));
    ImGui::Begin("main", NULL,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
                     ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SetWindowFontScale(1.5);
    ImGui::Text("%.1f FPS", Mn::Double(ImGui::GetIO().Framerate));
    uint32_t total = activeSceneGraph_->getDrawables().size();
    ImGui::Text("%u drawables", total);
    auto& cam = getAgentCamera();

    ImGui::End();
  }

  /* Set appropriate states. If you only draw ImGui, it is sufficient to
     just enable blending and scissor test in the constructor. */
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::ScissorTest);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::FaceCulling);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::DepthTest);

  imgui_.drawFrame();

  /* Reset state. Only needed if you want to draw something else with
     different state after. */

  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::ScissorTest);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::Blending);

  swapBuffers();
  timeline_.nextFrame();
  redraw();
}

void ArrangeRecorder::moveAndLook(int repetitions) {
  for (int i = 0; i < repetitions; i++) {
    if (keysPressed[KeyEvent::Key::Left]) {
      defaultAgent_->act("turnLeft");
    }
    if (keysPressed[KeyEvent::Key::Right]) {
      defaultAgent_->act("turnRight");
    }
    if (keysPressed[KeyEvent::Key::Up]) {
      defaultAgent_->act("lookUp");
    }
    if (keysPressed[KeyEvent::Key::Down]) {
      defaultAgent_->act("lookDown");
    }

    bool moved = false;
    if (keysPressed[KeyEvent::Key::A]) {
      defaultAgent_->act("moveLeft");
      moved = true;
    }
    if (keysPressed[KeyEvent::Key::D]) {
      defaultAgent_->act("moveRight");
      moved = true;
    }
    if (keysPressed[KeyEvent::Key::S]) {
      defaultAgent_->act("moveBackward");
      moved = true;
    }
    if (keysPressed[KeyEvent::Key::W]) {
      defaultAgent_->act("moveForward");
      moved = true;
    }
    if (keysPressed[KeyEvent::Key::X]) {
      defaultAgent_->act("moveDown");
      moved = true;
    }
    if (keysPressed[KeyEvent::Key::Z]) {
      defaultAgent_->act("moveUp");
      moved = true;
    }
  }
}

void ArrangeRecorder::bindRenderTarget() {
  for (auto& it : agentBodyNode_->getSubtreeSensors()) {
    if (it.second.get().isVisualSensor()) {
      esp::sensor::VisualSensor& visualSensor =
          static_cast<esp::sensor::VisualSensor&>(it.second.get());
      simulator_->getRenderer()->bindRenderTarget(visualSensor);
    }  // if
  }    // for
}

// todo: test or cut this
void ArrangeRecorder::viewportEvent(ViewportEvent& event) {
  for (auto& it : agentBodyNode_->getSubtreeSensors()) {
    if (it.second.get().isVisualSensor()) {
      esp::sensor::VisualSensor& visualSensor =
          static_cast<esp::sensor::VisualSensor&>(it.second.get());
      visualSensor.setResolution(event.framebufferSize()[1],
                                 event.framebufferSize()[0]);
      renderCamera_->setViewport(visualSensor.framebufferSize());
    }
  }
  bindRenderTarget();
  Mn::GL::defaultFramebuffer.setViewport({{}, event.framebufferSize()});

  imgui_.relayout(Mn::Vector2{event.windowSize()} / event.dpiScaling(),
                  event.windowSize(), event.framebufferSize());
}

void ArrangeRecorder::mousePressEvent(MouseEvent& event) {
  if (event.button() == MouseEvent::Button::Left) {
    recentCursorPos_ = event.position();
    arranger_->setCursor(recentCursorPos_);
    arranger_->update(0.f, true, false);
  }

  event.setAccepted();
  redraw();
}

void ArrangeRecorder::mouseReleaseEvent(MouseEvent& event) {
  event.setAccepted();
}

void ArrangeRecorder::mouseScrollEvent(MouseScrollEvent& event) {
  // shift+scroll is forced into x direction on mac, seemingly at OS level, so
  // use both x and y offsets.
  float scrollModVal = abs(event.offset().y()) > abs(event.offset().x())
                           ? event.offset().y()
                           : event.offset().x();
  if (!(scrollModVal)) {
    return;
  }
  // Use shift for fine-grained zooming
  float modVal = (event.modifiers() & MouseEvent::Modifier::Shift) ? 1.01 : 1.1;
  float mod = scrollModVal > 0 ? modVal : 1.0 / modVal;
  auto& cam = getAgentCamera();
  cam.modifyZoom(mod);
  redraw();

  event.setAccepted();
}  // ArrangeRecorder::mouseScrollEvent

void ArrangeRecorder::mouseMoveEvent(MouseMoveEvent& event) {
  recentCursorPos_ = event.position();

  if ((event.buttons() & MouseMoveEvent::Button::Right)) {
    const Mn::Vector2i delta = event.relativePosition();
    auto& controls = *defaultAgent_->getControls().get();
    controls(*agentBodyNode_, "turnRight", delta.x());
    // apply the transformation to all sensors
    for (auto& p : agentBodyNode_->getSubtreeSensors()) {
      controls(p.second.get().object(),  // SceneNode
               "lookDown",               // action name
               delta.y(),                // amount
               false);                   // applyFilter
    }
  }

  redraw();

  event.setAccepted();
}

void ArrangeRecorder::keyPressEvent(KeyEvent& event) {
  const auto key = event.key();
  switch (key) {
    case KeyEvent::Key::F5: {
      restoreFromScenePhysicsKeyframe();
    } break;
    case KeyEvent::Key::S: {
      if (event.modifiers() & KeyEvent::Modifier::Ctrl) {
        if (args_.isSet("edit-scene-physics-keyframe")) {
          saveScenePhysicsKeyframe();
        } else {
          LOG(WARNING) << "Use --edit-scene-physics-keyframe to enable "
                          "editing/saving the scene physics-keyframe.";
        }
      }
    } break;
    case KeyEvent::Key::F:
      arranger_->update(0.f, false, true);
      break;
    case KeyEvent::Key::C:
      showFPS_ = !showFPS_;
      break;
    case KeyEvent::Key::Esc:
      /* Using Application::exit(), which exits at the next iteration of the
         event loop (same as the window close button would do). Using
         std::exit() would exit immediately, but without calling any scoped
         destructors, which could hide potential destruction order issues or
         crashes at exit. We don't want that. */
      exit(0);
    case KeyEvent::Key::Q:
      showAgentStateMsg();
      break;
  }

  // Update map of moving/looking keys which are currently pressed
  if (event.modifiers() == KeyEvent::Modifiers()) {
    if (keysPressed.count(key) > 0) {
      keysPressed[key] = true;
    }
  }
}

void ArrangeRecorder::keyReleaseEvent(KeyEvent& event) {
  // Update map of moving/looking keys which are currently pressed
  const auto key = event.key();
  if (keysPressed.count(key) > 0) {
    keysPressed[key] = false;
  }
}

std::string ArrangeRecorder::getActiveScenePhysicsKeyframeFilepath() {
  CORRADE_INTERNAL_ASSERT(!simConfig_.activeSceneName.empty());
  std::string filepathBase =
      Cr::Utility::Directory::splitExtension(
          Cr::Utility::Directory::splitExtension(simConfig_.activeSceneName)
              .first)
          .first;

  return filepathBase + ".physics_keyframe.json";
}

void ArrangeRecorder::saveScenePhysicsKeyframe() {
  auto keyframe = simulator_->savePhysicsKeyframe();

  const auto filepath = getActiveScenePhysicsKeyframeFilepath();
  rapidjson::Document d(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType& allocator = d.GetAllocator();
  esp::io::addMember(d, "keyframe", keyframe, allocator);
  esp::io::writeJsonToFile(d, filepath, /*usePrettyWriter*/ true,
                           /*maxDecimalPlaces*/ 7);

  LOG(INFO) << "Saved new scene start state at " << filepath;
}

void ArrangeRecorder::restoreFromScenePhysicsKeyframe() {
  const auto filepath = getActiveScenePhysicsKeyframeFilepath();

  if (!Corrade::Utility::Directory::exists(filepath)) {
    LOG(ERROR) << "ArrangeRecorder::restoreFromScenePhysicsKeyframe: file "
               << filepath << " not found.";
    return;
  }
  try {
    auto newDoc = esp::io::parseJsonFile(filepath);
    esp::physics::PhysicsKeyframe keyframe;
    esp::io::readMember(newDoc, "keyframe", keyframe);
    simulator_->restoreFromPhysicsKeyframe(keyframe);
  } catch (...) {
    LOG(ERROR)
        << "ArrangeRecorder::restoreFromScenePhysicsKeyframe: failed to parse "
           "keyframes from "
        << filepath << ".";
  }

  LOG(INFO) << "Reloaded scene start state from " << filepath
            << " and started new session";

  arranger_.reset();
  arranger_ = std::make_unique<esp::arrange::Arranger>(
      simulator_.get(), renderCamera_, &debugRender_, &debug3dText_);
  numSavedArrangerUserActions_ = 0;
  sessionSaveFilepath_ = "";
}

std::string ArrangeRecorder::getActiveSceneSimplifiedName() {
  auto sceneInstanceAttr =
      simulator_->getMetadataMediator()->getSceneAttributesByName(
          simConfig_.activeSceneName);
  return sceneInstanceAttr ? sceneInstanceAttr->getSimplifiedHandle()
                           : "noActiveScene";
}

std::string ArrangeRecorder::findNewSessionSaveFilepath() {
  if (!Cr::Utility::Directory::exists("data")) {
    LOG(ERROR) << "Data folder not found in working directory! Session saving "
                  "is disabled.";
    return "";
  }

  const std::string sessionsDir = "data/sessions";
  if (!Cr::Utility::Directory::exists(sessionsDir)) {
    Cr::Utility::Directory::mkpath(sessionsDir);
  }

  std::string sessionName = getActiveSceneSimplifiedName() + "_" +
                            getCurrentTimeString() + ".session.json";
  return sessionsDir + "/" + sessionName;
}

void ArrangeRecorder::checkSaveArrangerSession() {
  const auto& session = arranger_->getSession();

  if (args_.isSet("edit-scene-physics-keyframe")) {
    return;
  }

  const auto& filepath = sessionSaveFilepath_;

  // save at the end of every user action
  if (session.userActions.size() > numSavedArrangerUserActions_) {
    if (sessionSaveFilepath_.empty()) {
      sessionSaveFilepath_ = findNewSessionSaveFilepath();
      if (sessionSaveFilepath_.empty()) {
        return;
      }
    }

    rapidjson::Document d(rapidjson::kObjectType);
    rapidjson::Document::AllocatorType& allocator = d.GetAllocator();
    esp::io::addMember(d, "session", session, allocator);
    // avoid pretty writer, to reduce filesize
    esp::io::writeJsonToFile(d, filepath, /*usePrettyWriter*/ false,
                             /*maxDecimalPlaces*/ 7);
    LOG(INFO) << "Session saved to " << filepath;

    numSavedArrangerUserActions_ = session.userActions.size();
  }
}

}  // namespace

MAGNUM_APPLICATION_MAIN(ArrangeRecorder)