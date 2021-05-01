add_library(
  scripted STATIC
  CookableEntity.cpp
  CookableEntity.h
  EntityManager.h
  EntityManager.hpp
  EntityManagerHelper.cpp
  EntityManagerHelper.h
  FluidVesselEntity.cpp
  FluidVesselEntity.h
  KitchenSetup.cpp
  KitchenSetup.h
  OvenEntity.cpp
  OvenEntity.h
)

target_link_libraries(
  scripted
  PUBLIC assets core gfx
)
