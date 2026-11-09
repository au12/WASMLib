// (see val.h) Emscripten's binding declarations, as nothing: the native replay calls the C++ functions directly.
#pragma once
#include <functional>
#include "val.h"
namespace emscripten {
template <class F> inline void function(const char*, F) {}
template <class T> inline T typed_memory_view(size_t, const T*) { return T(); }
}
#define EMSCRIPTEN_BINDINGS(name) [[maybe_unused]] static void wl_bindings_##name()
