// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_HABITATEXCEPTION_H_
#define ESP_CORE_HABITATEXCEPTION_H_

#include <dlfcn.h>
#include <sstream>
#include <Corrade/Containers/ScopeGuard.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/Macros.h>

// todo: put in esp::core namespace

namespace Cr = Corrade;

CORRADE_NORETURN void throwIfInPythonOtherwiseAbort(const char* message);

/* Same as CORRADE_ASSERT() except the return value, which you don't need here
   as the macro is never compiled out */
#define HABITAT_EXCEPTION(condition, ...)                                   \
    do {                                                                    \
        if(!(condition)) {                                                  \
            std::ostringstream out;                                         \
            Cr::Utility::Debug{&out, Cr::Utility::Debug::Flag::NoNewlineAtTheEnd} << __VA_ARGS__; \
            throwIfInPythonOtherwiseAbort(out.str().data());                \
        }                                                                   \
    } while(false)


#endif