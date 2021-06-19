add_library(
  arrange_recorder STATIC
  Arranger.cpp Arranger.h
)

target_link_libraries(
  arrange_recorder
  PUBLIC core glog gfx
)
