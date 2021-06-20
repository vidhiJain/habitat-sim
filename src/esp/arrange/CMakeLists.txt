add_library(
  arrange STATIC
  Arranger.cpp Arranger.h Session.cpp Session.h
)

target_link_libraries(
  arrange
  PUBLIC core glog gfx
)
