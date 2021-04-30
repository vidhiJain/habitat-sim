// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCRIPTED_ENTITYMANAGERHELPER_H_
#define ESP_SCRIPTED_ENTITYMANAGERHELPER_H_

#include <esp/gfx/Debug3DText.h>
#include <esp/gfx/DebugRender.h>

namespace esp {
namespace scripted {

class EntityManagerHelper {
 public:
  static void update(float dt);
  static void debugRender(esp::gfx::Debug3DText& debug3dText,
                          esp::gfx::DebugRender& debugRender);

 private:
};

}  // namespace scripted
}  // namespace esp

#endif
