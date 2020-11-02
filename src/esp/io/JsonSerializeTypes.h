// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"  // vec3f

#include <Corrade/Containers/Optional.h>  // Optional, NullOpt
#include <Magnum/Magnum.h>                //  Magnum vector/matrix types

#include <rapidjson/document.h>

#include <string>

namespace Magnum {
class ResourceKey;
}
namespace esp {
namespace assets {
struct RenderAssetInstanceCreation;
struct AssetInfo;
}  // namespace assets
}  // namespace esp

namespace esp {
namespace io {

template <typename T>
void AddMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const T& obj,
               rapidjson::MemoryPoolAllocator<>& allocator) {
  value.AddMember(name, obj, allocator);
}

template <typename T>
void ReadMember(const rapidjson::Value& value, const char* name, T& obj) {
  obj = value[name].Get<T>();
}

template <typename T>
void AddMemberEnum(rapidjson::Value& value,
                   rapidjson::GenericStringRef<char> name,
                   const T& x,
                   rapidjson::MemoryPoolAllocator<>& allocator) {
  static_assert(sizeof(T) == sizeof(uint32_t), "size match");
  uint32_t xAsUint = (uint32_t)x;
  value.AddMember(name, xAsUint, allocator);
}

template <typename T>
void ReadMemberEnum(const rapidjson::Value& value, const char* name, T& x) {
  uint32_t xAsUint = value[name].Get<uint32_t>();
  x = (T)(xAsUint);
}

void AddMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               rapidjson::Value& child,
               rapidjson::MemoryPoolAllocator<>& allocator);

void AddMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const Magnum::Matrix4& obj,
               rapidjson::MemoryPoolAllocator<>& allocator);

void ReadMember(const rapidjson::Value& value,
                const char* name,
                Magnum::Matrix4& obj);

void AddMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const std::string& str,
               rapidjson::MemoryPoolAllocator<>& allocator);

void ReadMember(const rapidjson::Value& value,
                const char* name,
                std::string& str);

void AddMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const Magnum::Vector3& vec,
               rapidjson::MemoryPoolAllocator<>& allocator);

void ReadMember(const rapidjson::Value& value,
                const char* name,
                Magnum::Vector3& vec);

template <typename T>
void AddMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const Corrade::Containers::Optional<T>& x,
               rapidjson::MemoryPoolAllocator<>& allocator) {
  if (x) {
    const T& item = *x;
    AddMember(value, name, item, allocator);
  }
}

template <typename T>
void ReadMember(const rapidjson::Value& value,
                const char* name,
                Corrade::Containers::Optional<T>& x) {
  rapidjson::Value::ConstMemberIterator itr = value.FindMember(name);
  if (value.HasMember(name)) {
    x = T();
    ReadMember(value, name, *x);
  } else {
    x = Corrade::Containers::NullOpt;
  }
}

void AddMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const Magnum::ResourceKey& obj,
               rapidjson::MemoryPoolAllocator<>& allocator);

void ReadMember(const rapidjson::Value& value,
                const char* name,
                Magnum::ResourceKey& obj);

void AddMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const esp::vec3f& vec,
               rapidjson::MemoryPoolAllocator<>& allocator);

void ReadMember(const rapidjson::Value& value,
                const char* name,
                esp::vec3f& vec);

void AddMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const esp::assets::AssetInfo& x,
               rapidjson::MemoryPoolAllocator<>& allocator);
void ReadMember(const rapidjson::Value& value,
                const char* name,
                esp::assets::AssetInfo& x);
void AddMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const esp::assets::RenderAssetInstanceCreation& x,
               rapidjson::MemoryPoolAllocator<>& allocator);
void ReadMember(const rapidjson::Value& value,
                const char* name,
                esp::assets::RenderAssetInstanceCreation& x);

}  // namespace io
}  // namespace esp
