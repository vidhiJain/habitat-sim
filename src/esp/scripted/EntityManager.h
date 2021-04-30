// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCRIPTED_ENTITYMANAGER_H_
#define ESP_SCRIPTED_ENTITYMANAGER_H_

#include <memory>
#include <vector>

namespace esp {
namespace scripted {

template <typename T>
class EntityManager {
 public:
  static EntityManager& get();

  void onConstruct(T* ent);

  void onDelete(const T* ent);

  std::vector<T*>& getVector() { return entities_; }

 private:
  std::vector<T*> entities_;
};

}  // namespace scripted
}  // namespace esp

#endif  // ESP_SCRIPTED_ENTITYMANAGER_H_
