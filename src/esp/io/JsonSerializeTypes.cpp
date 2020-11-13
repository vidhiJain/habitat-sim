// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "JsonSerializeTypes.h"

#include "esp/assets/Asset.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/gfx/RenderKeyframe.h"

#include <Magnum/Math/Matrix4.h>
#include <Magnum/Resource.h>

#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <iostream>
#include "esp/io/json.h"

using namespace rapidjson;

namespace esp {
namespace io {

void AddMember(Value& value,
               GenericStringRef<char> name,
               Value& child,
               MemoryPoolAllocator<>& allocator) {
  value.AddMember(name, child, allocator);
}

void AddMember(Value& value,
               GenericStringRef<char> name,
               const Magnum::Matrix4& obj,
               MemoryPoolAllocator<>& allocator) {
  // perf todo: assert that last column is (0, 0, 0, 1) and don't write/read it
  Value floatsArray(kArrayType);
  for (int i = 0; i < obj.Rows * obj.Cols; i++) {
    floatsArray.PushBack(obj.data()[i], allocator);
  }
  value.AddMember(name, floatsArray, allocator);
}

void ReadMember(const Value& value, const char* name, Magnum::Matrix4& obj) {
  const Value& floatsArray = value[name];
  ASSERT(floatsArray.Size() == obj.Rows * obj.Cols);
  for (int i = 0; i < floatsArray.Size(); i++) {
    obj.data()[i] = floatsArray[i].GetFloat();
  }
}

void AddMember(Value& value,
               GenericStringRef<char> name,
               const std::string& str,
               MemoryPoolAllocator<>& allocator) {
  Value strObj;
  strObj.SetString(str.c_str(), allocator);
  value.AddMember(name, strObj, allocator);
}

void ReadMember(const Value& value, const char* name, std::string& str) {
  const Value& strObj = value[name];
  str = strObj.GetString();
}

void AddMember(Value& value,
               GenericStringRef<char> name,
               const Magnum::Vector3& vec,
               MemoryPoolAllocator<>& allocator) {
  Value obj(kObjectType);
  AddMember(obj, "x", vec.x(), allocator);
  AddMember(obj, "y", vec.y(), allocator);
  AddMember(obj, "z", vec.z(), allocator);
  value.AddMember(name, obj, allocator);
}

void ReadMember(const Value& value, const char* name, Magnum::Vector3& vec) {
  const Value& obj = value[name];
  ReadMember(obj, "x", vec.x());
  ReadMember(obj, "y", vec.y());
  ReadMember(obj, "z", vec.z());
}

void AddMember(Value& value,
               GenericStringRef<char> name,
               const Magnum::Quaternion& quat,
               MemoryPoolAllocator<>& allocator) {
  Value obj(kObjectType);
  AddMember(obj, "x", quat.vector().x(), allocator);
  AddMember(obj, "y", quat.vector().y(), allocator);
  AddMember(obj, "z", quat.vector().z(), allocator);
  AddMember(obj, "w", quat.scalar(), allocator);
  value.AddMember(name, obj, allocator);
}

void ReadMember(const Value& value,
                const char* name,
                Magnum::Quaternion& quat) {
  const Value& obj = value[name];
  ReadMember(obj, "x", quat.vector().x());
  ReadMember(obj, "y", quat.vector().y());
  ReadMember(obj, "z", quat.vector().z());
  ReadMember(obj, "w", quat.scalar());
}

void AddMember(Value& value,
               GenericStringRef<char> name,
               const Magnum::ResourceKey& obj,
               MemoryPoolAllocator<>& allocator) {
  static_assert(sizeof(size_t) == sizeof(uint64_t), "size match");
  uint64_t tmp = std::hash<Magnum::ResourceKey>()(obj);
  value.AddMember(name, tmp, allocator);
}

void ReadMember(const Value& value,
                const char* name,
                Magnum::ResourceKey& obj) {
  size_t tmp = value[name].GetUint64();
  obj = Magnum::ResourceKey(tmp);
}

void AddMember(Value& value,
               GenericStringRef<char> name,
               const esp::vec3f& vec,
               MemoryPoolAllocator<>& allocator) {
  Value obj(kObjectType);
  AddMember(obj, "x", vec.x(), allocator);
  AddMember(obj, "y", vec.y(), allocator);
  AddMember(obj, "z", vec.z(), allocator);
  value.AddMember(name, obj, allocator);
}

void ReadMember(const Value& value, const char* name, esp::vec3f& vec) {
  const Value& obj = value[name];
  ReadMember(obj, "x", vec.x());
  ReadMember(obj, "y", vec.y());
  ReadMember(obj, "z", vec.z());
}

/*
  AssetType type = AssetType::UNKNOWN;
  std::string filepath = EMPTY_SCENE;  // empty scene
  geo::CoordinateFrame frame;
  float virtualUnitToMeters = 1.0f;
  bool requiresLighting = false;
*/
void AddMember(Value& value,
               GenericStringRef<char> name,
               const esp::assets::AssetInfo& x,
               MemoryPoolAllocator<>& allocator) {
  Value coordinateFrameObj(kObjectType);
  AddMember(coordinateFrameObj, "up", x.frame.up(), allocator);
  AddMember(coordinateFrameObj, "front", x.frame.front(), allocator);
  AddMember(coordinateFrameObj, "origin", x.frame.origin(), allocator);

  Value obj(kObjectType);
  AddMemberEnum(obj, "type", x.type, allocator);
  AddMember(obj, "filepath", x.filepath, allocator);
  AddMember(obj, "frame", coordinateFrameObj, allocator);
  AddMember(obj, "virtualUnitToMeters", x.virtualUnitToMeters, allocator);
  AddMember(obj, "requiresLighting", x.requiresLighting, allocator);

  AddMember(value, name, obj, allocator);
}

void ReadMember(const Value& value,
                const char* name,
                esp::assets::AssetInfo& x) {
  const Value& obj = value[name];
  ReadMemberEnum(obj, "type", x.type);
  ReadMember(obj, "filepath", x.filepath);
  // ReadMember(obj, "frame", coordinateFrameObj);
  ReadMember(obj, "virtualUnitToMeters", x.virtualUnitToMeters);
  ReadMember(obj, "requiresLighting", x.requiresLighting);

  const Value& coordinateFrameObj = obj["frame"];
  esp::vec3f up;
  esp::vec3f front;
  esp::vec3f origin;
  ReadMember(coordinateFrameObj, "up", up);
  ReadMember(coordinateFrameObj, "front", front);
  ReadMember(coordinateFrameObj, "origin", origin);
  x.frame = esp::geo::CoordinateFrame(up, front, origin);
}

void AddMember(Value& value,
               GenericStringRef<char> name,
               const esp::assets::RenderAssetInstanceCreationInfo& x,
               MemoryPoolAllocator<>& allocator) {
  Value obj(kObjectType);
  AddMember(obj, "filepath", x.filepath, allocator);
  AddMember(obj, "scale", x.scale, allocator);
  AddMember(obj, "isStatic", x.isStatic(), allocator);
  AddMember(obj, "isRGBD", x.isRGBD(), allocator);
  AddMember(obj, "isSemantic", x.isSemantic(), allocator);
  AddMember(obj, "lightSetupKey", x.lightSetupKey, allocator);

  AddMember(value, name, obj, allocator);
}

void ReadMember(const Value& value,
                const char* name,
                esp::assets::RenderAssetInstanceCreationInfo& x) {
  const Value& obj = value[name];
  ReadMember(obj, "filepath", x.filepath);
  ReadMember(obj, "scale", x.scale);
  bool isStatic;
  ReadMember(obj, "isStatic", isStatic);
  bool isRGBD;
  ReadMember(obj, "isRGBD", isRGBD);
  bool isSemantic;
  ReadMember(obj, "isSemantic", isSemantic);
  x.flags |= isStatic
                 ? esp::assets::RenderAssetInstanceCreationInfo::Flag::IsStatic
                 : esp::assets::RenderAssetInstanceCreationInfo::Flags();
  x.flags |= isRGBD ? esp::assets::RenderAssetInstanceCreationInfo::Flag::IsRGBD
                    : esp::assets::RenderAssetInstanceCreationInfo::Flags();
  x.flags |=
      isSemantic
          ? esp::assets::RenderAssetInstanceCreationInfo::Flag::IsSemantic
          : esp::assets::RenderAssetInstanceCreationInfo::Flags();
  ReadMember(obj, "lightSetupKey", x.lightSetupKey);
}

void AddMember(Value& value,
               GenericStringRef<char> name,
               const esp::gfx::Transform& x,
               MemoryPoolAllocator<>& allocator) {
  Value obj(kObjectType);
  AddMember(obj, "translation", x.translation, allocator);
  AddMember(obj, "rotation", x.rotation, allocator);
  value.AddMember(name, obj, allocator);
}

void ReadMember(const Value& value, const char* name, esp::gfx::Transform& x) {
  const Value& obj = value[name];
  ReadMember(obj, "translation", x.translation);
  ReadMember(obj, "rotation", x.rotation);
}

}  // namespace io
}  // namespace esp
