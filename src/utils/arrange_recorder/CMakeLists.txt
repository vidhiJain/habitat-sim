find_package(Magnum REQUIRED DebugTools)

find_package(MagnumIntegration REQUIRED ImGui)

set(arrange_recorder_SOURCES arrange_recorder.cpp)

#set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
add_executable(arrange_recorder ${arrange_recorder_SOURCES})

target_link_libraries(
  arrange_recorder
  PRIVATE assets
          gfx
          nav
          scene
          sim
          geo
          arrange
          Magnum::DebugTools
          MagnumIntegration::ImGui
)
