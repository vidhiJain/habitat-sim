// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "KitchenSetup.h"

#include "CookableEntity.h"
#include "OvenEntity.h"
#include "FluidVesselEntity.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace scripted {

namespace {

void loadStaticObjects(esp::sim::Simulator* sim) {

  int id;

// I0430 19:08:51.906787 2074313 viewer.cpp:181] trans: {0.725298,-1.43959,-0.0876577}
// I0430 19:08:51.906834 2074313 viewer.cpp:182] rot: {{0.000733241,0.0650282,-0.0104461},0.997829}

  id = sim->addObjectByHandle("data/objects/ktc_cabinets1.object_config.json");
  CORRADE_INTERNAL_ASSERT(id != -1);
  sim->setTranslation(Mn::Vector3(0.725298,-1.43959,-0.0876577), id);
  sim->setRotation(Mn::Quaternion({0.000733241,0.0650282,-0.0104461},0.997829), id);
  sim->setObjectMotionType(esp::physics::MotionType::STATIC, id);

  id = sim->addObjectByHandle("data/objects/ktc_cabinets2.object_config.json");
  CORRADE_INTERNAL_ASSERT(id != -1);
  sim->setTranslation(Mn::Vector3(-1.35265291,-1.435616,-0.272905648), id);
  sim->setRotation(Mn::Quaternion({-0.00328475982,-0.65820694,0.00287134061},0.7528244), id);
  sim->setObjectMotionType(esp::physics::MotionType::STATIC, id);

  id = sim->addObjectByHandle("data/objects/ktc_cabinets3.object_config.json");
  CORRADE_INTERNAL_ASSERT(id != -1);
  sim->setTranslation(Mn::Vector3(-0.06714147,-1.46629739,-2.338812), id);
  sim->setRotation(Mn::Quaternion({-0.007838256,-0.998030961,-0.0004889179},0.0622297749), id);
  sim->setObjectMotionType(esp::physics::MotionType::STATIC, id);

// I0430 19:09:25.283691 2074313 viewer.cpp:181] trans: {3.43251,-1.48142,-0.563389}
// I0430 19:09:25.283751 2074313 viewer.cpp:182] rot: {{-2.6077e-08,0.74753,2.98023e-08},0.664229}
  id = sim->addObjectByHandle("data/objects/ktc_fridge_open.object_config.json");
  CORRADE_INTERNAL_ASSERT(id != -1);
  sim->setTranslation(Mn::Vector3(3.43251,-1.48142,-0.563389), id);
  sim->setRotation(Mn::Quaternion({0.0,0.74753,2.98023e-08},0.664229), id);
  sim->setObjectMotionType(esp::physics::MotionType::STATIC, id);

  id = sim->addObjectByHandle("data/objects/ktc_hood.object_config.json");
  CORRADE_INTERNAL_ASSERT(id != -1);
  sim->setTranslation(Mn::Vector3(-1.419739,0.258229047,-1.16238475), id);
  sim->setRotation(Mn::Quaternion({0.0,-0.654761255,0.0},0.7558358), id);
  sim->setObjectMotionType(esp::physics::MotionType::STATIC, id);

}

}

KitchenSetup::KitchenSetup(esp::sim::Simulator* sim) {
  // perhaps configure SimConfig with stage as desired?

  auto identRot = Mn::Quaternion(Magnum::Math::IdentityInit);

  new OvenEntity(sim, {-1.35652,-1.36883,-1.15998}, {{8.9407e-08,0.641507,5.02914e-08},-0.767117});

// I0501 11:28:23.217546 2188294 viewer.cpp:199] trans: {-0.973773,-0.466525,-0.242976}
// I0501 11:28:23.217574 2188294 viewer.cpp:200] rot: {{-0.0156006,-0.813332,0.018929},0.581282}
// I0501 11:28:26.003839 2188294 viewer.cpp:199] trans: {-0.987898,-0.440564,-0.425394}
// I0501 11:28:26.003867 2188294 viewer.cpp:200] rot: {{0.016741,0.893536,-0.000887454},0.448679}
  new CookableEntity(sim, {-0.98,-0.46,-0.23}, identRot);
  new CookableEntity(sim, {-0.99,-0.46,-0.44}, identRot);
  new CookableEntity(sim, {-0.86,-0.46,-0.23}, identRot);
  new CookableEntity(sim, {-0.87,-0.46,-0.44}, identRot);

  FluidVesselEntity::Blueprint milkCartonBp{
    .objHandle = "data/objects/milk_carton.object_config.json",
    .spoutPos = Mn::Vector3(-0.05, 0.28, 0.0) * 0.5,
    .spoutDir = Mn::Vector3(-1.0, 1.0, 0.0),
    .spoutConeAngle = Mn::Deg(45),
    .spoutRadius = 0.02,
    .volume = 0.5,
    .initialFluidType = "milk"
  };
// I0501 11:25:17.516863 2188294 viewer.cpp:199] trans: {1.66911,-0.893705,0.343503}
// I0501 11:25:17.516891 2188294 viewer.cpp:200] rot: {{2.42591e-05,0.204182,6.09457e-06},0.978933}  
  new FluidVesselEntity(sim, milkCartonBp, {1.66911,-0.893705,0.343503}, {{2.42591e-05,0.204182,6.09457e-06},0.978933});

  FluidVesselEntity::Blueprint cupBp{
    .objHandle = "data/objects/frl_apartment_cup_02.object_config.json",
    .spoutPos = Mn::Vector3(0.0, 0.07, 0.0),
    .spoutDir = Mn::Vector3(0.0, 1.0, 0.0),
    .spoutConeAngle = Mn::Deg(90),
    .spoutRadius = 0.06,
    .volume = 0.2,
    .initialFluidType = ""
  };
  new FluidVesselEntity(sim, cupBp, Mn::Vector3(0.f, 0.5f, 2.0), identRot);
  new FluidVesselEntity(sim, cupBp, Mn::Vector3(0.f, 0.5f, 2.1), identRot);

// I0501 11:27:08.155495 2188294 viewer.cpp:199] trans: {-0.919036,-0.47911,-0.333129}
// I0501 11:27:08.155521 2188294 viewer.cpp:200] rot: {{-0.00436143,0.0346233,-0.000107795},0.999391}
  int id;
  id = sim->addObjectByHandle("data/objects/ktc_clutter_tray.object_config.json");
  CORRADE_INTERNAL_ASSERT(id != -1);
  sim->setTranslation({-0.919036,-0.47911,-0.333129}, id);
  sim->setRotation({{-0.00436143,0.0346233,-0.000107795},0.999391}, id);

  id = sim->addObjectByHandle("data/objects/ktc_clutter_potmatt.object_config.json");
  CORRADE_INTERNAL_ASSERT(id != -1);
  sim->setTranslation({-0.127363,-0.431755,1.4129}, id);
  sim->setRotation({{-0.00295681,0.751272,0.00389342},0.659974}, id);

  loadStaticObjects(sim);
}

}  // namespace scripted
}  // namespace esp
