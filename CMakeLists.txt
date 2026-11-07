# WASMLib: NVIDIA's SMLib B-rep kernel (from usd-brep, the `upstream` submodule) built for WebAssembly with
# Emscripten — and natively with the same flags, as the reference the WebAssembly build is checked against.
#
#   emcmake cmake -B build-wasm -G Ninja -DCMAKE_BUILD_TYPE=Release && ninja -C build-wasm
#   cmake -B build-native -G Ninja -DCMAKE_BUILD_TYPE=Release && ninja -C build-native   (reference)
cmake_minimum_required(VERSION 3.20)
project(WASMLib CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(UP ${CMAKE_SOURCE_DIR}/upstream)

# ---------------------------------------------------------------- flags
# Upstream's release configuration: NDEBUG, NOMINMAX, C++17, RTTI and exceptions on; TBB off (SM_USE_TBB is never
# defined upstream, so its parallel loops are compiled out there too). Upstream builds with -Werror and per-SDK
# suppressions; here warnings are silenced instead (clang finds many upstream gcc doesn't).
set(SM_DEFS NDEBUG NOMINMAX)
if(EMSCRIPTEN)
  # -O3 + whole-program LTO; SIMD for the vector maths the compiler can find; native WebAssembly exceptions
  # (no JS round trip per try/catch)
  set(OPT -O3 -flto -msimd128 -fwasm-exceptions)
  set(LINKOPT -O3 -flto -msimd128 -fwasm-exceptions
      -sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=64MB -sMAXIMUM_MEMORY=4GB
      # SMLib recurses deeply (subdivision solvers, fillets): Emscripten's 64 KB default stack is far too small
      -sSTACK_SIZE=16MB)
else()
  set(OPT -O2)
  set(LINKOPT)
endif()
add_compile_options(-w ${OPT})

# ---------------------------------------------------------------- the kernel
file(GLOB SMLIB_SRC ${UP}/source/SMLib/src/*.cpp)
file(GLOB SMAPI_SRC ${UP}/source/SM_API/src/*.cpp)
add_library(smlib STATIC ${SMLIB_SRC})
target_include_directories(smlib PUBLIC ${UP}/source/SMLib/inc)
target_compile_definitions(smlib PUBLIC ${SM_DEFS})
add_library(smapi STATIC ${SMAPI_SRC})
target_include_directories(smapi PUBLIC ${UP}/source/SM_API/inc)
target_compile_definitions(smapi PRIVATE SM_API_EXPORTS)
target_link_libraries(smapi PUBLIC smlib)

# ---------------------------------------------------------------- upstream's own SM_API test suite
# (run from upstream/tests/SM_API_test, as its SM_API_test.sh does, so its relative TestFiles paths resolve)
file(GLOB SMAPI_TEST_SRC ${UP}/tests/SM_API_test/src/*.cpp)
add_executable(sm_api_test ${SMAPI_TEST_SRC} ${UP}/Examples/SM_API/boolean_bounds.cpp ${UP}/Examples/SMLib/step_derivatives.cpp)
target_include_directories(sm_api_test PRIVATE ${UP}/tests/SM_API_test/inc ${UP}/Examples/SM_API ${UP}/Examples/SMLib)
target_compile_definitions(sm_api_test PRIVATE SM_API_TEST_STANDALONE)
target_link_libraries(sm_api_test PRIVATE smapi)
target_link_options(sm_api_test PRIVATE ${LINKOPT})
if(EMSCRIPTEN)
  # real files through Node's fs, and the exit code passed on
  target_link_options(sm_api_test PRIVATE -sNODERAWFS=1 -sEXIT_RUNTIME=1)
endif()

# ---------------------------------------------------------------- the JavaScript module
if(EMSCRIPTEN)
  file(GLOB BIND_SRC ${CMAKE_SOURCE_DIR}/bind/*.cpp ${CMAKE_SOURCE_DIR}/bind/gen/*.cpp)
  if(BIND_SRC)
    add_executable(wasmlib ${BIND_SRC})
    target_include_directories(wasmlib PRIVATE ${CMAKE_SOURCE_DIR}/bind)
    target_link_libraries(wasmlib PRIVATE smapi)
    target_link_options(wasmlib PRIVATE ${LINKOPT} -lembind
      -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createWASMLib -sENVIRONMENT=web,worker,node
      -sEXPORTED_RUNTIME_METHODS=FS -sFORCE_FILESYSTEM=1 -sINCOMING_MODULE_JS_API=locateFile,wasmBinary,print,printErr)
    set_target_properties(wasmlib PROPERTIES SUFFIX ".mjs" RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/dist)
  endif()
endif()
