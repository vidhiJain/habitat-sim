add_library(
  io STATIC
  io.cpp
  io.h
  json.cpp
  json.h
  JsonSerializeTypes.cpp
  JsonSerializeTypes.h
)

target_link_libraries(
  io
  PUBLIC core
)
