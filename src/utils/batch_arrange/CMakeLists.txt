set(batch_arrange_SOURCES BatchArrange.cpp)

add_executable(batch_arrange ${batch_arrange_SOURCES})
set(DEPS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../deps")

target_link_libraries(
  batch_arrange
  PRIVATE assets scene sim geo arrange
)
