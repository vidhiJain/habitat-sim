// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "EntityManager.h"

#include <Corrade/Utility/Assert.h>

#include <algorithm>

namespace esp {
namespace scripted {

template <typename T>
EntityManager<T>& EntityManager<T>::get() {
  static std::unique_ptr<EntityManager> instance_;
  if (!instance_) {
    instance_ = std::make_unique<EntityManager>();
  }
  return *instance_;
}

template <typename T>
void EntityManager<T>::onConstruct(T* ent) {
  entities_.push_back(ent);
}

template <typename T>
void EntityManager<T>::onDelete(const T* ent) {
  auto it = std::find(entities_.begin(), entities_.end(), ent);
  CORRADE_INTERNAL_ASSERT(it != entities_.end());
  entities_.erase(it);
}

}  // namespace scripted
}  // namespace esp
