// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "HabitatException.h"

#include "esp/core/logging.h"

/* NORETURN will help the compiler optimize -- it basically tells it that the
   condition passed to HABITAT_EXCEPTION() can be assumed to be always true in
   the following code, because if not then the execution ends in this
   function. */
CORRADE_NORETURN void throwIfInPythonOtherwiseAbort(const char* message) {
    /* Open the executable itself so we can look for symbols, close at the end
       of scope. Since it's an assertion that should happens at most once for
       the whole interpreter lifetime, I think it's fine to do it from scratch
       "every time" we enter this function. */
    void* self = dlopen(nullptr, RTLD_NOW|RTLD_GLOBAL);
    Cr::Containers::ScopeGuard closeSelf{self, dlclose};

    LOG(WARNING) << "throwIfInPythonOtherwiseAbort";

    /* Look for habitatThrowPythonAssertionError(). If it's defined, it means
       our bindings code is linked in, which is a pretty clear indication that
       we're running in Python. Additionally, that code also takes care of
       raising a correct Python exception, which we can't easily do without
       linking directly to Python from all our code. */
    if(auto habitatThrowPythonAssertionError = reinterpret_cast<CORRADE_NORETURN void(*)(const char*)>(dlsym(self, "habitatThrowPythonAssertionError"))) {
        LOG(WARNING) << "throwing";
        habitatThrowPythonAssertionError(message);
    }

    LOG(WARNING) << "after throw block, about to abort";

    /* If it isn't defined, do an abort the same way as CORRADE_ASSERT(). This
       could be in an `else` block but I like to play with fire, so it's not --
       the NORETURN above should tell that to the compiler and the function 
       should throw. */
    Cr::Utility::Error{Cr::Utility::Error::defaultOutput()} << message;
    std::abort();
}

