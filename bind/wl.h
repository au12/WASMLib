// WASMLib's bridge between SMLib's C++ types and JavaScript values, used by the generated SM_API binding
// (gen/smapi.gen.cpp) and the hand-written object layer (objects.cpp).
//
// Conventions on the JavaScript side:
//   - kernel objects are handles: the object's address as a number (wasm32 addresses fit a double exactly); null is 0
//     or null. Who owns what follows SM_API's own contracts (see each function's notes in js/smapi.gen.d.ts);
//   - points and vectors are [x, y, z] (or [u, v]) arrays;
//   - arrays of handles or numbers are plain arrays; big numeric results (meshes) are typed arrays;
//   - a failed status or a C++ exception becomes a thrown Error naming the call, the status and the kernel's own
//     messages (its error callback's trail) since the previous call.
#pragma once

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <SmApiBrep.h>
#include <SmApiCurves.h>
#include <SmApiFillets.h>
#include <SmApiGeneral.h>
#include <SmApiHeal.h>
#include <SmApiImportExport.h>
#include <SmApiIntersectors.h>
#include <SmApiPolygons.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmApiSurfaces.h>
#include <SmApiTessellationParams.h>
#include <SmApiTrimmedSurfaces.h>
#include <SmApiTypes.h>

#include <SmAxis2Placement.h>
#include <SmBrep.h>
#include <SmPrimitiveCreation.h>
#include <SmTArray.h>
#include <SmVector2d.h>
#include <SmVector3d.h>

using emscripten::val;
using namespace emscripten;

namespace wl {

// ---------------------------------------------------------------- errors
/** the kernel's messages since the last call (its error callback, warnings left out) */
std::string take_trail();
const char* status_name(long s);

/** a failed status: throws (a C++ exception, turned into a JavaScript Error by the caller's catch) */
inline void check(SmApiStatus s, const char* name) {
    if (s == SM_SUCCESS) { take_trail(); return; }
    std::string m = std::string(name) + " failed: " + status_name(s);
    std::string t = take_trail();
    if (!t.empty()) m += " — " + t;
    throw std::runtime_error(m);
}
/** throw a JavaScript Error (called outside any try block) */
[[noreturn]] void raise(const char* name, const std::string& why);

// ---------------------------------------------------------------- handles
inline bool present(const val& v) { return !(v.isNull() || v.isUndefined()); }

template <class T> inline T* ptr(const val& v) {
    if (!present(v)) return nullptr;
    const double d = v.as<double>();
    return reinterpret_cast<T*>(static_cast<uintptr_t>(d));
}
template <class T> inline val handle(T* p) {
    return p ? val(static_cast<double>(reinterpret_cast<uintptr_t>(p))) : val::null();
}

// ---------------------------------------------------------------- numbers, points
template <class N> inline val num(N n) { return val(static_cast<double>(n)); }

inline SmVector3d v3(const val& a) { return SmVector3d(a[0].as<double>(), a[1].as<double>(), a[2].as<double>()); }
inline SmVector2d v2(const val& a) { return SmVector2d(a[0].as<double>(), a[1].as<double>()); }

inline val js(const SmVector3d& p) {
    val a = val::array();
    a.set(0, p.x); a.set(1, p.y); a.set(2, p.z);
    return a;
}
inline val js(const SmVector2d& p) {
    val a = val::array();
    a.set(0, p.x); a.set(1, p.y);
    return a;
}

// ---------------------------------------------------------------- arrays
template <class T> inline val js(const SmTArray<T*>& v) {
    val a = val::array();
    for (ULONG i = 0; i < v.GetSize(); ++i) a.set(i, handle(v[i]));
    return a;
}
template <class T, std::enable_if_t<std::is_arithmetic_v<T> || std::is_enum_v<T>, int> = 0>
inline val js(const SmTArray<T>& v) {
    val a = val::array();
    for (ULONG i = 0; i < v.GetSize(); ++i) a.set(i, static_cast<double>(v[i]));
    return a;
}
inline val js(const SmTArray<SmVector3d>& v) {
    val a = val::array();
    for (ULONG i = 0; i < v.GetSize(); ++i) a.set(i, js(v[i]));
    return a;
}
inline val js(const SmTArray<SmVector2d>& v) {
    val a = val::array();
    for (ULONG i = 0; i < v.GetSize(); ++i) a.set(i, js(v[i]));
    return a;
}

template <class T> inline void fill(const val& a, SmTArray<T*>& v) {
    const unsigned n = a["length"].as<unsigned>();
    for (unsigned i = 0; i < n; ++i) v.Add(ptr<T>(a[i]));
}
template <class T, std::enable_if_t<std::is_arithmetic_v<T> || std::is_enum_v<T>, int> = 0>
inline void fill(const val& a, SmTArray<T>& v) {
    const unsigned n = a["length"].as<unsigned>();
    for (unsigned i = 0; i < n; ++i) v.Add(static_cast<T>(a[i].as<double>()));
}
inline void fill(const val& a, SmTArray<SmVector3d>& v) {
    const unsigned n = a["length"].as<unsigned>();
    for (unsigned i = 0; i < n; ++i) v.Add(v3(a[i]));
}
inline void fill(const val& a, SmTArray<SmVector2d>& v) {
    const unsigned n = a["length"].as<unsigned>();
    for (unsigned i = 0; i < n; ++i) v.Add(v2(a[i]));
}

// ---------------------------------------------------------------- structs
inline double opt(const val& o, const char* k, double d) {
    val v = o[k];
    return present(v) ? v.as<double>() : d;
}
inline void from(const val& o, SmTessellationParams& p) {
    p.dChordHeightTol = opt(o, "chordHeightTol", p.dChordHeightTol);
    p.dCurveAngleTolDeg = opt(o, "curveAngleTolDeg", p.dCurveAngleTolDeg);
    p.dSurfaceAngleTolDeg = opt(o, "surfaceAngleTolDeg", p.dSurfaceAngleTolDeg);
    p.dMaxEdgeLength = opt(o, "maxEdgeLength", p.dMaxEdgeLength);
    p.dMaxAspectRatio = opt(o, "maxAspectRatio", p.dMaxAspectRatio);
}
inline void from(const val& o, SmSweepOptions& s) {
    s.bCapEndsArg = opt(o, "capEnds", s.bCapEndsArg) != 0;
    s.bTranslationalSweep = opt(o, "translational", s.bTranslationalSweep) != 0;
    s.bMoveProfileCenterToPath = opt(o, "moveProfileCenterToPath", s.bMoveProfileCenterToPath) != 0;
    s.bOrientProfilePerpendicularToPath = opt(o, "orientProfilePerpendicularToPath", s.bOrientProfilePerpendicularToPath) != 0;
    s.bMoveToProfile = opt(o, "moveToProfile", s.bMoveToProfile) != 0;
}
/** a placement: { origin, xAxis, yAxis } (a rotation then a move: what SmApiTransform applies) */
inline void from(const val& o, SmAxis2Placement& p) {
    p = SmAxis2Placement(v3(o["origin"]), v3(o["xAxis"]), v3(o["yAxis"]));
}
inline val js(const SmTessellationFailure& f) {
    val o = val::object();
    o.set("faceIndex", static_cast<double>(f.lFaceIndex));
    o.set("stage", static_cast<int>(f.eStage));
    o.set("status", static_cast<double>(f.eStatus));
    o.set("statusName", std::string(status_name(f.eStatus)));
    return o;
}
inline val js(const SmTArray<SmTessellationFailure>& v) {
    val a = val::array();
    for (ULONG i = 0; i < v.GetSize(); ++i) a.set(i, js(v[i]));
    return a;
}
inline val js(const SmTessellationReport& r) {
    val o = val::object();
    o.set("failures", js(r.failures));
    o.set("laminaEdges", static_cast<double>(r.laminaEdges));
    o.set("spineEdges", static_cast<double>(r.spineEdges));
    o.set("inputManifold", static_cast<bool>(r.inputManifold));
    o.set("outputManifold", static_cast<bool>(r.outputManifold));
    o.set("hasMesh", static_cast<bool>(r.hasMesh));
    return o;
}

}  // namespace wl

// one wrapper shape for every hand-written function: kernel exceptions and failed statuses become a JavaScript Error
#define WL_BEGIN std::string err__; try {
#define WL_END(name) } catch (const std::exception& e) { err__ = e.what(); } catch (...) { err__ = "unknown C++ exception"; } \
    wl::raise(name, err__);
