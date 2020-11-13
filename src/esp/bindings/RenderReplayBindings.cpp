// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/PythonBindings.h>
#include <Magnum/SceneGraph/PythonBindings.h>

#include "esp/gfx/RenderReplayManager.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace gfx {

void initRenderReplayBindings(py::module& m) {

  py::class_<RenderReplayManager, RenderReplayManager::ptr>(m, "RenderReplayManager")
      .def(
          "save_keyframe",
          [](RenderReplayManager& self) {
            if (!self.getWriter()) {
              LOG(ERROR) << "save_keyframe: not enabled. See SimulatorConfiguration::enableRenderReplaySave.";
              return;
            }
            self.getWriter()->saveKeyframe();
          },
          R"(Save a render keyframe; a render keyframe can be loaded later and used to draw observations)")

      .def(
          "add_user_transform_to_keyframe",
          [](RenderReplayManager& self,
              const std::string& name,
              const Magnum::Vector3& translation,
              const Magnum::Quaternion& rotation) {
            if (!self.getWriter()) {
              LOG(ERROR) << "add_user_transform_to_keyframe: not enabled. See SimulatorConfiguration::enableRenderReplaySave.";
              return;
            }
            self.getWriter()->addUserTransformToKeyframe(name, translation, rotation);
          },
          R"(Add a user transform to the current render keyframe; it will get stored with the keyframe and will be available later upon loading the keyframe)")

      .def(
          "write_saved_keyframes_to_file",
          [](RenderReplayManager& self, const std::string& filepath) {
            if (!self.getWriter()) {
              LOG(ERROR) << "write_saved_keyframes_to_file: not enabled. See SimulatorConfiguration::enableRenderReplaySave.";
              return;
            }
            self.getWriter()->writeSavedKeyframesToFile(filepath);
          },
          R"(Write all saved keyframes to a file, then discard the keyframes.)")

      .def(
          "player_load_from_file",
          [](RenderReplayManager& self, const std::string& filepath) {
            // todo: decide getReader() by pointer or what
            self.getReader()->readKeyframesFromFile(filepath);
          },
          R"(Load keyframes for playback. See also player_set_keyframe_index.)")

      .def(
          "player_get_num_keyframes",
          [](RenderReplayManager& self) {
            return self.getReader()->getNumFrames();
          },
          R"(todo)")

      .def(
          "player_set_keyframe_index",
          [](RenderReplayManager& self, int frameIndex) {
            // todo: error handling
            self.getReader()->setFrame(frameIndex);
          },
          R"(todo)")

      .def(
          "player_get_keyframe_index",
          [](RenderReplayManager& self) {
            return self.getReader()->getFrameIndex();
          },
          R"(todo)")

      .def(
          "player_get_user_transform",
          [](RenderReplayManager& self, const std::string& name) {
            Magnum::Vector3 translation;
            Magnum::Quaternion rotation;
            bool found = self.getReader()->getUserTransform(name, &translation, &rotation);
            return found
              ? py::make_tuple(translation, rotation)
              : py::make_tuple(nullptr, nullptr);
          },
          R"(todo)");
}

/*
      recorder_add_user_object_to_keyframe
      recorder_save_keyframe
      recorder_write_saved_keyframes_to_file

      player_load_from_file
      player_get_num_keyframes
      player_set_keyframe_index
      player_get_user_object
      player_get_keyframe_index
      */


}  // namespace gfx
}  // namespace esp
