add_library(
  arrange STATIC
  Arranger.cpp Arranger.h
)

target_link_libraries(
  arrange
  PUBLIC core glog gfx
)
