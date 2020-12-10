// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/PythonBindings.h>
#include <Magnum/SceneGraph/PythonBindings.h>

#include "esp/gfx/replay/Player.h"
#include "esp/gfx/replay/ReplayManager.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace gfx {
namespace replay {

void initGfxReplayBindings(py::module& m) {
  py::class_<Player, Player::ptr>(m, "Player")
      .def("get_num_keyframes", &Player::getNumKeyframes, R"(todo)")

      .def("set_keyframe_index", &Player::setKeyframeIndex, R"(todo)")

      .def("get_keyframe_index", &Player::getKeyframeIndex, R"(todo)")

      .def(
          "get_user_transform",
          [](Player& self, const std::string& name) {
            Magnum::Vector3 translation;
            Magnum::Quaternion rotation;
            bool found = self.getUserTransform(name, &translation, &rotation);
            return found ? py::make_tuple(translation, rotation)
                         : py::make_tuple(nullptr, nullptr);
          },
          R"(todo)");

  py::class_<ReplayManager, ReplayManager::ptr>(m, "ReplayManager")
      .def(
          "save_keyframe",
          [](ReplayManager& self) {
            if (!self.getRecorder()) {
              LOG(ERROR) << "save_keyframe: not enabled. See "
                            "SimulatorConfiguration::enableGfxReplaySave.";
              return;
            }
            self.getRecorder()->saveKeyframe();
          },
          R"(Save a render keyframe; a render keyframe can be loaded later and used to draw observations.)")

      .def(
          "add_user_transform_to_keyframe",
          [](ReplayManager& self, const std::string& name,
             const Magnum::Vector3& translation,
             const Magnum::Quaternion& rotation) {
            if (!self.getRecorder()) {
              LOG(ERROR) << "add_user_transform_to_keyframe: not enabled. See "
                            "SimulatorConfiguration::enableGfxReplaySave.";
              return;
            }
            self.getRecorder()->addUserTransformToKeyframe(name, translation,
                                                           rotation);
          },
          R"(Add a user transform to the current render keyframe; it will get stored with the keyframe and will be available later upon loading the keyframe)")

      .def(
          "write_saved_keyframes_to_file",
          [](ReplayManager& self, const std::string& filepath) {
            if (!self.getRecorder()) {
              LOG(ERROR) << "write_saved_keyframes_to_file: not enabled. See "
                            "SimulatorConfiguration::enableGfxReplaySave.";
              return;
            }
            self.getRecorder()->writeSavedKeyframesToFile(filepath);
          },
          R"(Write all saved keyframes to a file, then discard the keyframes.)")

      .def("read_keyframes_from_file", &ReplayManager::readKeyframesFromFile,
           R"(Create a Player object from a replay file.)");
}

}  // namespace replay
}  // namespace gfx
}  // namespace esp