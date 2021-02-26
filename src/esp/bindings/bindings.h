// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BINDINGS_BINDINGS_H_
#define ESP_BINDINGS_BINDINGS_H_

#include <pybind11/pybind11.h>
#include "esp/bindings/OpaqueTypes.h"

namespace esp {

namespace metadata {
void initAttributesBindings(pybind11::module& m);
void initMetadataMediatorBindings(pybind11::module& m);
namespace managers {
void initAttributesManagersBindings(pybind11::module& m);
}  // namespace managers
}  // namespace metadata

namespace geo {
void initGeoBindings(pybind11::module& m);
}

namespace gfx {
void initGfxBindings(pybind11::module& m);
namespace replay {
void initGfxReplayBindings(pybind11::module& m);
}
}  // namespace gfx

namespace nav {
void initShortestPathBindings(pybind11::module& m);
}

namespace physics {
void initPhysicsBindings(pybind11::module& m);
}

namespace scene {
void initSceneBindings(pybind11::module& m);
}

namespace sensor {
void initSensorBindings(pybind11::module& m);
}

namespace sim {
void initSimBindings(pybind11::module& m);
}

}  // namespace esp

// temp home for this function
#include <pybind11/pybind11.h>
#include <Corrade/Utility/VisibilityMacros.h>
#include <Corrade/Utility/Macros.h>

/* Export this function and do that with a C linkage so it can be fetched via
   an unmangled name afterwards */
extern "C" CORRADE_VISIBILITY_EXPORT CORRADE_NORETURN void habitatThrowPythonAssertionError(const char* message);

#endif  // ESP_BINDINGS_BINDINGS_H_
