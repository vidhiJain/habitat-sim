// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/Assert.h>

#include "esp/arrange/Arranger.h"
#include "esp/core/esp.h"
#include "esp/core/random.h"
#include "esp/sim/Simulator.h"

// for ease of access
namespace Cr = Corrade;
namespace Mn = Magnum;

namespace {

Mn::Vector3 sampleFromPatch(const std::vector<Mn::Vector3>& patchVerts,
                            esp::core::Random& random) {
  ESP_CHECK(patchVerts.size() == 4,
            "sampleFromPatch with "
                << patchVerts.size()
                << " vertices is not currently supported (must be 4)");

  Mn::Vector2 lerpFraction{random.uniform_float_01(),
                           random.uniform_float_01()};

  // assume 4 points are given as either CW or CCW sequence, e.g. (bottom-left,
  // bottom-right, top-right, top-left).
  return Mn::Math::lerp(
      Mn::Math::lerp(patchVerts[0], patchVerts[1], lerpFraction.x()),
      Mn::Math::lerp(patchVerts[3], patchVerts[2], lerpFraction.x()),
      lerpFraction.y());
}

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

class BatchArrange {
 public:
  BatchArrange(int argc, char** argv);

 private:
  void createSimulator();
  void restoreFromScenePhysicsKeyframe();
  void checkSaveArrangerSession();
  std::string getActiveSceneSimplifiedName();
  std::string findNewSessionSaveFilepath();
  std::string getActiveScenePhysicsKeyframeFilepath();
  esp::arrange::Config loadArrangeConfig();

  Mn::Vector3 getRandomDropPositionForRack(bool isUpperRack);
  void generateRandomSession();

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

  esp::scene::SceneNode* agentBodyNode_ = nullptr;

  const int defaultAgentId_ = 0;
  esp::agent::Agent::ptr defaultAgent_ = nullptr;

  // Scene or stage file to load
  std::string sceneFileName;

  std::unique_ptr<esp::arrange::Arranger> arranger_;
  std::string sessionSaveFilepath_;
  int numSavedArrangerUserActions_ = 0;
  esp::core::Random random_;
  esp::arrange::Config arrangeConfig_;
};

void BatchArrange::createSimulator() {
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

  // add selects a random initial state and sets up the default controls and
  // step filter
  simulator_->addAgent(agentConfig);

  // Set up camera
  defaultAgent_ = simulator_->getAgent(defaultAgentId_);
  agentBodyNode_ = &defaultAgent_->node();

  // place agent at origin; we'll move the camera for this app but not the agent
  agentBodyNode_->setTransformation(Mn::Matrix4(Mn::Math::IdentityInit));

  restoreFromScenePhysicsKeyframe();
}

// todo: remove all these args
BatchArrange::BatchArrange(int argc, char** argv) : random_(0) {
  Cr::Utility::Arguments args;
  args.addNamedArgument("scene")
      .setHelp("scene", "Path to your scene (your_scene.scene_instance.json)")
      .addSkippedPrefix("magnum", "engine-specific options")
      .setGlobalHelp("Generate scene arrangement sessions procedurally")
      .addOption("dataset", "default")
      .setHelp(
          "dataset",
          "Path to your scene dataset (your_dataset.scene_dataset_config.json")
      .addOption("arrange-config")
      .setHelp("arrange-config", "filepath to arrange config json file")
      .parse(argc, argv);

  sceneFileName = args.value("scene");

  // configure and intialize Simulator
  auto simConfig = esp::sim::SimulatorConfiguration();
  simConfig.activeSceneName = sceneFileName;
  simConfig.sceneDatasetConfigFile = args.value("dataset");
  simConfig.enablePhysics = true;
  simConfig.frustumCulling = true;
  simConfig.requiresTextures = true;
  simConfig.enableGfxReplaySave = false;
  // simConfig.createRenderer = false;

  // temp hard-code default lighting
  simConfig.sceneLightSetup = esp::DEFAULT_LIGHTING_KEY;
  simConfig.overrideSceneLightDefaults = true;

  simConfig_ = simConfig;
  args_ = std::move(args);
  arrangeConfig_ = loadArrangeConfig();
  createSimulator();

  generateRandomSession();
  checkSaveArrangerSession();

}  // end BatchArrange::BatchArrange

std::string BatchArrange::getActiveScenePhysicsKeyframeFilepath() {
  CORRADE_INTERNAL_ASSERT(!simConfig_.activeSceneName.empty());
  std::string filepathBase =
      Cr::Utility::Directory::splitExtension(
          Cr::Utility::Directory::splitExtension(simConfig_.activeSceneName)
              .first)
          .first;

  return filepathBase + ".physics_keyframe.json";
}

void BatchArrange::restoreFromScenePhysicsKeyframe() {
  const auto filepath = getActiveScenePhysicsKeyframeFilepath();

  if (!Corrade::Utility::Directory::exists(filepath)) {
    LOG(ERROR) << "BatchArrange::restoreFromScenePhysicsKeyframe: file "
               << filepath << " not found.";
    return;
  }
  try {
    auto newDoc = esp::io::parseJsonFile(filepath);
    esp::physics::PhysicsKeyframe keyframe;
    esp::io::readMember(newDoc, "keyframe", keyframe);
    simulator_->restoreFromPhysicsKeyframe(keyframe, /*activate*/ false);
  } catch (...) {
    LOG(ERROR)
        << "BatchArrange::restoreFromScenePhysicsKeyframe: failed to parse "
           "keyframes from "
        << filepath << ".";
  }

  LOG(INFO) << "Reloaded scene start state from " << filepath
            << " and started new session";

  if (arranger_) {
    arranger_.reset();
  }
  arranger_ = std::make_unique<esp::arrange::Arranger>(
      esp::arrange::Config(arrangeConfig_), simulator_.get(), nullptr, nullptr,
      nullptr);
  numSavedArrangerUserActions_ = 0;
  sessionSaveFilepath_ = "";
}

std::string BatchArrange::getActiveSceneSimplifiedName() {
  return sceneAttr_ ? sceneAttr_->getSimplifiedHandle() : "noActiveScene";
}

std::string BatchArrange::findNewSessionSaveFilepath() {
  if (!Cr::Utility::Directory::exists("data")) {
    LOG(ERROR) << "Data folder not found in working directory! Session saving "
                  "is disabled.";
    return "";
  }

  const std::string sessionsDir = "data/batch_sessions";
  if (!Cr::Utility::Directory::exists(sessionsDir)) {
    Cr::Utility::Directory::mkpath(sessionsDir);
  }

  std::string sessionName = getActiveSceneSimplifiedName() + "_" +
                            getCurrentTimeString() + ".session.json";
  return sessionsDir + "/" + sessionName;
}

// todo: only save at end of session
void BatchArrange::checkSaveArrangerSession() {
  const auto& session = arranger_->getSession();

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

esp::arrange::Config BatchArrange::loadArrangeConfig() {
  esp::arrange::Config config;

  const std::string filepath = args_.value("arrange-config");
  if (filepath.empty()) {
    return config;
  }

  if (!Corrade::Utility::Directory::exists(filepath)) {
    LOG(WARNING)
        << "BatchArrange::checkReloadArrangeConfig: writing a new empty "
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
    LOG(ERROR) << "BatchArrange::checkReloadArrangeConfig: failed to parse "
                  "arrange config from "
               << filepath;
  }

  return config;
}

Mn::Vector3 BatchArrange::getRandomDropPositionForRack(bool isUpperRack) {
  const auto& debugLineLists = arrangeConfig_.debugLineLists;

  int rackIndex = isUpperRack ? 0 : 1;
  ESP_CHECK(
      debugLineLists.size() == 2,
      "BatchArrange::getRandomDropPositionForRack: expecting exactly two debug "
      "line lists arrange_config.json that describe rack drop regions");

  return sampleFromPatch(debugLineLists[rackIndex].verts, random_);
}

void BatchArrange::generateRandomSession() {
  const auto& artObjMgr = simulator_->getArticulatedObjectManager();
  // See your scene_instance.json file for the articulated object name, and note
  // our convention to append the instance index. The first instance is :0000.
  // This string is also discoverable using the interactive arrange recorder
  // (it is printed to the terminal during mouse-click interaction).
  int artObjId = artObjMgr->getObjectIDByHandle("ktc_dishwasher_:0000");
  CORRADE_INTERNAL_ASSERT(artObjId != esp::ID_UNDEFINED);

  // dishwasher has 4 links (unused, lower rack, door, upper rack)
  constexpr int lowerRackLinkId = 1;
  constexpr int doorLinkId = 2;
  constexpr int upperRackLinkId = 3;
  bool success = false;
  success = arranger_->moveArticulatedLink(artObjId, doorLinkId,
                                           /*moveToLowerLimit*/ true);  // open
  ESP_CHECK(success, "moveArticulatedLink failed");

  bool loadUpperRackFirst = random_.uniform_int(0, 2) == 1;

  success = arranger_->moveArticulatedLink(
      artObjId, loadUpperRackFirst ? upperRackLinkId : lowerRackLinkId,
      /*moveToLowerLimit*/ false);  // open
  ESP_CHECK(success, "moveArticulatedLink failed");
  bool isUpperRackOpen = loadUpperRackFirst;

  int numObjectsUpperRack = random_.uniform_int(3, 6);
  int numObjectsLowerRack = random_.uniform_int(5, 8);

  const auto& rigidObjMgr = simulator_->getRigidObjectManager();
  auto rigidObjHandles = rigidObjMgr->getObjectHandlesBySubstring(
      "", true);  // sloppy: do this to get all objects

  std::vector<int> filteredRigidObjIDs;
  for (const auto handle : rigidObjHandles) {
    const auto rigidObjId = rigidObjMgr->getObjectIDByHandle(handle);
    if (simulator_->getObjectMotionType(rigidObjId) ==
        esp::physics::MotionType::DYNAMIC) {
      filteredRigidObjIDs.push_back(rigidObjId);
    }
  }
  CORRADE_INTERNAL_ASSERT(
      filteredRigidObjIDs.size());  // todo: abort action here

  int maxMoveAttempts = 100;
  int numSuccessfulMoves = 0;

  for (int i = 0; i < maxMoveAttempts; i++) {
    int randIndex = random_.uniform_int(0, filteredRigidObjIDs.size());
    int selectedRigidObjId = filteredRigidObjIDs[randIndex];

    // todo: hook maxDropOffsetY up to config
    const float dropOffsetY = random_.uniform_float(0.f, 0.5f);

    const Mn::Vector3 dropPos = getRandomDropPositionForRack(isUpperRackOpen);

    // pick a random rotation from our fixed set
    // todo: generate a random rotation quaternion instead?
    auto rotIndex = random_.uniform_int(0, arranger_->getNumRotationIndices());

    if (arranger_->tryMoveRigidObject(selectedRigidObjId, rotIndex, dropPos,
                                      dropOffsetY)) {
      numSuccessfulMoves++;

      // remove ID
      filteredRigidObjIDs.erase(
          std::remove(filteredRigidObjIDs.begin(), filteredRigidObjIDs.end(),
                      selectedRigidObjId),
          filteredRigidObjIDs.end());

      if ((loadUpperRackFirst && numSuccessfulMoves == numObjectsUpperRack) ||
          (!loadUpperRackFirst && numSuccessfulMoves == numObjectsLowerRack)) {
        // close first rack
        success = arranger_->moveArticulatedLink(
            artObjId, loadUpperRackFirst ? upperRackLinkId : lowerRackLinkId,
            /*moveToLowerLimit*/ true);  // close
        ESP_CHECK(success, "moveArticulatedLink failed");
        // open second rack
        success = arranger_->moveArticulatedLink(
            artObjId, loadUpperRackFirst ? lowerRackLinkId : upperRackLinkId,
            /*moveToLowerLimit*/ false);  // open
        ESP_CHECK(success, "moveArticulatedLink failed");

        isUpperRackOpen = !isUpperRackOpen;
      }

      if (numSuccessfulMoves == numObjectsUpperRack + numObjectsLowerRack ||
          filteredRigidObjIDs.empty()) {
        break;
      }
    }
  }

  // close whatever rack is open
  success = arranger_->moveArticulatedLink(
      artObjId, isUpperRackOpen ? upperRackLinkId : lowerRackLinkId,
      /*moveToLowerLimit*/ true);  // close
  ESP_CHECK(success, "moveArticulatedLink failed");

  success = arranger_->moveArticulatedLink(
      artObjId, doorLinkId, /*moveToLowerLimit*/ false);  // close
  ESP_CHECK(success, "moveArticulatedLink failed");
}

}  // namespace

int main(int argc, char** argv) {
  BatchArrange batchArrange(argc, argv);
}