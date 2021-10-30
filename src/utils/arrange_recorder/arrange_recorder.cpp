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
#include "esp/core/Esp.h"
#include "esp/gfx/DebugRender.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/scene/SceneNode.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sim/Simulator.h"

constexpr float moveSensitivity = 0.07f;
constexpr float lookSensitivity = 0.9f;
constexpr float rgbSensorHeight = 1.5f;

// for ease of access
namespace Cr = Corrade;
namespace Mn = Magnum;

esp::logging::LoggingContext loggingContext_;

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
  void checkReloadArrangeConfig(float dt);
  esp::arrange::Config loadArrangeConfig();

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
      ESP_DEBUG() << str;
    }
  }

  // The simulator object backend for this viewer instance
  std::unique_ptr<esp::sim::Simulator> simulator_;

  // store these so we can recreate the simulator
  Cr::Utility::Arguments args_;
  esp::sim::SimulatorConfiguration simConfig_;
  esp::metadata::attributes::SceneAttributes::ptr sceneAttr_;

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
  bool showImgui_ = true;

  esp::scene::SceneNode* agentBodyNode_ = nullptr;

  const int defaultAgentId_ = 0;
  esp::agent::Agent::ptr defaultAgent_ = nullptr;

  // Scene or stage file to load
  std::string sceneFileName;
  esp::gfx::RenderCamera* renderCamera_ = nullptr;
  esp::scene::SceneGraph* activeSceneGraph_ = nullptr;

  Mn::Timeline timeline_;

  Mn::ImGuiIntegration::Context imgui_{Mn::NoCreate};

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
    // see also esp::sensor::SensorSubType::Orthographic
    spec->sensorSubType = esp::sensor::SensorSubType::Pinhole;
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

  esp::arrange::Arranger::configureCollisionGroups();  // run this before
                                                       // creating sim

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
  sceneAttr_ = simulator_->getMetadataMediator()->getSceneAttributesByName(
      simConfig_.activeSceneName);

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

  // place agent at origin; we'll move the camera for this app but not the agent
  agentBodyNode_->setTransformation(Mn::Matrix4(Mn::Math::IdentityInit));

  renderCamera_->node().setTranslation(Mn::Vector3(0, 0.316604, 0.610451));
  renderCamera_->node().setRotation(
      Mn::Quaternion({-0.271441, 0, 0}, 0.962455));

  arranger_ = std::make_unique<esp::arrange::Arranger>(
      loadArrangeConfig(), simulator_.get(), renderCamera_, &debugRender_,
      &debug3dText_);

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
  args.addNamedArgument("scene")
      .setHelp("scene", "Path to your scene (your_scene.scene_instance.json)")
      .addSkippedPrefix("magnum", "engine-specific options")
      .setGlobalHelp("Rearrange a scene with the mouse and keyboard")
      .addOption("dataset", "default")
      .setHelp(
          "dataset",
          "Path to your scene dataset (your_dataset.scene_dataset_config.json")
      .addBooleanOption("debug-bullet")
      .setHelp("debug-bullet", "Render Bullet physics debug wireframes.")
      .addBooleanOption("authoring-mode")
      .setHelp("authoring-mode",
               "Instead of recording an arrangement session, use the arranger "
               "to edit the scene's start state. This is saved to "
               "your_scene.physics_keyframe.json.")
      .addOption("arrange-config")
      .setHelp("arrange-config", "filepath to arrange config json file")
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
  simConfig.enablePhysics = useBullet;
  simConfig.frustumCulling = true;
  simConfig.requiresTextures = true;
  simConfig.enableGfxReplaySave = false;

  // temp hard-code default lighting
  simConfig.sceneLightSetupKey = esp::DEFAULT_LIGHTING_KEY;
  simConfig.overrideSceneLightDefaults = true;

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

#if 0  // disable movement controls
  // call moveAndLook at 60 Hz
  while (timeSinceLastSimulation >= 1.0 / 60.0) {
    timeSinceLastSimulation -= 1.0 / 60.0;
    moveAndLook(1);
  }
#endif

  if (args_.isSet("authoring-mode")) {
    checkReloadArrangeConfig(timeline_.previousFrameDuration());
  }

  arranger_->setCursor(recentCursorPos_);
  arranger_->update(timeline_.previousFrameDuration(),
                    esp::arrange::Arranger::ButtonSet());

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

  if (showImgui_) {
    ImGui::SetNextWindowPos(ImVec2(5, 5));
    static float bgAlpha = 0.3;
    ImGui::SetNextWindowBgAlpha(bgAlpha);
    ImGui::Begin(
        "main", NULL,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SetWindowFontScale(1.5);
    if (args_.isSet("authoring-mode")) {
      ImGui::Text("Save Scene: CTRL+S");
    }
    ImGui::Text("Rotate/Skip: F");
    ImGui::Text("Raise/Lower: Z, X");
    ImGui::Text("Undo/Cancel: CTRL+Z");
    ImGui::Text("Camera: C, V");
    ImGui::Text("Zoom: mouse-wheel");
    ImGui::Text("Reset: F5");
    ImGui::Text("Hide Help: H");
    ImGui::Text("%.1f FPS", Mn::Double(ImGui::GetIO().Framerate));
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
    arranger_->update(0.f, esp::arrange::Arranger::Button::Primary);
  }

  event.setAccepted();
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

  event.setAccepted();
}  // ArrangeRecorder::mouseScrollEvent

void ArrangeRecorder::mouseMoveEvent(MouseMoveEvent& event) {
  recentCursorPos_ = event.position();

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
        if (args_.isSet("authoring-mode")) {
          saveScenePhysicsKeyframe();
        } else {
          ESP_WARNING() << "Use --authoring-mode to enable "
                           "saving the scene physics-keyframe.";
        }
      }
    } break;
    case KeyEvent::Key::F:
      arranger_->update(0.f, esp::arrange::Arranger::Button::Secondary);
      break;
    case KeyEvent::Key::C:
      arranger_->update(0.f, esp::arrange::Arranger::Button::PrevCamera);
      break;
    case KeyEvent::Key::V:
      arranger_->update(0.f, esp::arrange::Arranger::Button::NextCamera);
      break;
    case KeyEvent::Key::Z: {
      if (event.modifiers() & KeyEvent::Modifier::Ctrl) {
        arranger_->update(0.f, esp::arrange::Arranger::Button::Undo);
      } else {
        arranger_->update(0.f, esp::arrange::Arranger::Button::Raise);
      }
    } break;
    case KeyEvent::Key::X:
      arranger_->update(0.f, esp::arrange::Arranger::Button::Lower);
      break;
    case KeyEvent::Key::H:
      showImgui_ = !showImgui_;
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

  ESP_DEBUG() << "Saved new scene start state at " << filepath;
}

void ArrangeRecorder::restoreFromScenePhysicsKeyframe() {
  const auto filepath = getActiveScenePhysicsKeyframeFilepath();

  if (!Corrade::Utility::Directory::exists(filepath)) {
    ESP_ERROR() << "ArrangeRecorder::restoreFromScenePhysicsKeyframe: file "
                << filepath << " not found.";
    return;
  }
  try {
    auto newDoc = esp::io::parseJsonFile(filepath);
    esp::physics::PhysicsKeyframe keyframe;
    esp::io::readMember(newDoc, "keyframe", keyframe);
    simulator_->restoreFromPhysicsKeyframe(keyframe, /*activate*/ false);
  } catch (...) {
    ESP_ERROR()
        << "ArrangeRecorder::restoreFromScenePhysicsKeyframe: failed to parse "
           "keyframes from "
        << filepath << ".";
  }

  ESP_DEBUG() << "Reloaded scene start state from " << filepath
              << " and started new session";

  arranger_.reset();
  arranger_ = std::make_unique<esp::arrange::Arranger>(
      loadArrangeConfig(), simulator_.get(), renderCamera_, &debugRender_,
      &debug3dText_);
  numSavedArrangerUserActions_ = 0;
  sessionSaveFilepath_ = "";
}

std::string ArrangeRecorder::getActiveSceneSimplifiedName() {
  return sceneAttr_ ? sceneAttr_->getSimplifiedHandle() : "noActiveScene";
}

std::string ArrangeRecorder::findNewSessionSaveFilepath() {
  if (!Cr::Utility::Directory::exists("data")) {
    ESP_ERROR() << "Data folder not found in working directory! Session saving "
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

  if (args_.isSet("authoring-mode")) {
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
    ESP_DEBUG() << "Session saved to " << filepath;

    numSavedArrangerUserActions_ = session.userActions.size();
  }
}

// hot-reload arrange config json every n seconds
void ArrangeRecorder::checkReloadArrangeConfig(float dt) {
  constexpr float reloadPeriod = 1.f;
  static float elapsed = reloadPeriod;
  elapsed += dt;
  if (elapsed >= reloadPeriod) {
    elapsed -= reloadPeriod;

    arranger_->setConfig(loadArrangeConfig());
  }
}

esp::arrange::Config ArrangeRecorder::loadArrangeConfig() {
  esp::arrange::Config config;

  const std::string filepath = args_.value("arrange-config");
  if (filepath.empty()) {
    return config;
  }

  if (!Corrade::Utility::Directory::exists(filepath)) {
    ESP_WARNING()
        << "ArrangeRecorder::checkReloadArrangeConfig: writing a new empty "
           "arrange config at "
        << filepath;
    config.cameras.push_back(esp::arrange::ConfigCamera());
    rapidjson::Document d(rapidjson::kObjectType);
    rapidjson::Document::AllocatorType& allocator = d.GetAllocator();
    esp::io::addMember(d, "config", config, allocator);
    esp::io::writeJsonToFile(d, filepath, /*usePrettyWriter*/ true,
                             /*maxDecimalPlaces*/ 5);
    return config;
  }

  try {
    auto newDoc = esp::io::parseJsonFile(filepath);
    esp::io::readMember(newDoc, "config", config);

  } catch (...) {
    ESP_ERROR() << "ArrangeRecorder::checkReloadArrangeConfig: failed to parse "
                   "arrange config from "
                << filepath;
  }

  return config;
}

}  // namespace

MAGNUM_APPLICATION_MAIN(ArrangeRecorder)
