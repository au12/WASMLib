// Native replays of WASMLib calls, for profiling (callgrind) and for SMLib's error trace (-DSM_DEBUG_CODE=1): the
// same kernel source, the bindings built against a stand-in for Emscripten's `val` (emscripten/val.h here).
//   tools/native/build.sh && build-native/replay <case> [args]
#include "../../bind/build.cpp"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <sstream>

std::string wl::take_trail() { return ""; }
const char* wl::status_name(long s) { static thread_local char b[32]; snprintf(b, sizeof b, "status %ld", s); return b; }
void wl::raise(const char* name, const std::string& why) { throw std::runtime_error(std::string(name) + ": " + why); }

static double now() { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static void ok(SmApiStatus s, const char* what) { if (s != SM_SUCCESS) { fprintf(stderr, "%s failed: %ld\n", what, long(s)); exit(1); } }
static double volume(SmBrep* b) { double v = 0; ok(SmApiBrepComputeVolume(b, 1e-3, v), "volume"); return v; }
static ULONG faces(SmBrep* b) { SmTArray<SmFace*> f; b->GetFaces(f); return f.GetSize(); }
static std::string slurp(const char* p) { std::ifstream f(p); std::stringstream s; s << f.rdbuf(); return s.str(); }

/** a box with every edge rounded r3, its top face taken away and the rest made 2 mm thick (the benchmark's shell) */
static void shellCase(int reps) {
  for (int k = 0; k < reps; k++) {
    SmBrep* b = nullptr;
    ok(SmApiCreateBox(SmPoint3d(-30, -20, 0), 60, 40, 20, b), "box");
    SmTArray<SmEdge*> es; b->GetEdges(es);
    ok(SmApiFilletEdges(b, es, 3, 1, 1, 1.0), "fillet");
    SmTArray<SmFace*> fs; SmTArray<const SmFace*> top; b->GetFaces(fs);
    for (ULONG i = 0; i < fs.GetSize(); i++) {
      if (fs[i]->GetSurface()->GetType() != SmPlane_TYPE) continue;
      SmPoint3d lo, hi; ok(SmApiFaceBoundingBox(fs[i], FALSE, lo, hi), "box");
      if (lo.z > 19.99) top.Add(fs[i]);
    }
    const double t0 = now();
    SmBrep* s = nullptr;
    ok(SmApiShellBrepFull(b, -2, TRUE, TRUE, TRUE, top, s), "shell");
    const double t1 = now();
    printf("shell: %.1f ms, %lu faces, volume %.4f\n", t1 - t0, (unsigned long)faces(s), volume(s));
    delete b; delete s;
  }
}

/** 48 Ø4 × 8 cylinders made, and cut from a 160 × 100 × 6 plate one at a time (the benchmark's grid) */
static void holesCase(int reps, bool cut) {
  for (int k = 0; k < reps; k++) {
    const double t0 = now();
    std::vector<SmBrep*> cs;
    for (int i = 0; i < 8; i++) for (int j = 0; j < 6; j++) { SmBrep* c = nullptr; ok(SmApiCreateCylinder(SmPoint3d(-70 + 20 * i, -40 + 16 * j, -1), 2, 8, c), "cylinder"); cs.push_back(c); }
    const double t1 = now();
    double t2 = t1;
    if (cut) {
      SmBrep* p = nullptr; ok(SmApiCreateBox(SmPoint3d(-80, -50, 0), 160, 100, 6, p), "box");
      for (SmBrep* c : cs) { SmBrep* r = nullptr; ok(SmApiBooleanDifference(p, c, r), "cut"); p = r; }
      t2 = now();
      printf("48 cylinders: %.1f ms; cut one at a time: %.1f ms; %lu faces, volume %.3f\n", t1 - t0, t2 - t1, (unsigned long)faces(p), volume(p));
      delete p;
    } else {
      printf("48 cylinders: %.1f ms\n", t1 - t0);
      for (SmBrep* c : cs) delete c;
    }
  }
}

/** a call to brepFromFaces or screwSweep saved from JavaScript as JSON (its arguments, in order) */
static void jsonCase(const char* fn, const char* file, int reps) {
  const val args = emscripten::parse_json(slurp(file));
  for (int k = 0; k < reps; k++) {
    const double t0 = now();
    if (!strcmp(fn, "faces")) {
      val r = brepFromFaces(args[0], args[1].as<double>(), val::null());
      auto* b = wl::ptr<SmBrep>(r["brep"]);
      printf("brepFromFaces: %.1f ms, made %d, failed %u, solid %d, volume %.4f\n", now() - t0, r["faces"].as<int>(), r["failed"]["length"].as<unsigned>(), r["solid"].as<bool>(), r["solid"].as<bool>() ? volume(b) : 0.0);
      for (unsigned i = 0; i < r["failed"]["length"].as<unsigned>(); i++) printf("  failed %d\n", r["failed"][i].as<int>());
      delete b;
    } else if (!strcmp(fn, "screw")) {
      // args: [curves as NURBS {p,U,P,W}], origin, axis, move, angleDeg, tol
      const val cs = args[0];
      std::vector<SmBSplineCurve*> own; val handles = val::array();
      for (unsigned i = 0; i < cs["length"].as<unsigned>(); i++) { own.push_back(nurbs_curve(cs[i])); handles.set(i, wl::handle(own.back())); }
      val r = screwSweep(handles, args[1], args[2], args[3], args[4].as<double>(), args[5].as<double>());
      auto* b = wl::ptr<SmBrep>(r["brep"]);
      printf("screwSweep: %.1f ms, %d steps, glued %d, %lu faces, volume %.4f (exact %.4f)\n", now() - t0, r["steps"].as<int>(), r["glued"].as<bool>(), (unsigned long)faces(b), volume(b), r["volume"].as<double>());
      delete b; for (auto* c : own) delete c;
    }
  }
}

/** 48 cylinders as copies of one, moved into place */
static void copiesCase(int reps) {
  for (int k = 0; k < reps; k++) {
    const double t0 = now();
    SmBrep* proto = nullptr; ok(SmApiCreateCylinder(SmPoint3d(0, 0, 0), 2, 8, proto), "cylinder");
    const double t1 = now();
    std::vector<SmBrep*> cs;
    for (int i = 0; i < 8; i++) for (int j = 0; j < 6; j++) {
      SmBrep* c = nullptr; ok(SmApiBrepCopy(proto, c), "copy");
      ok(SmApiTranslate(c, SmVector3d(-70 + 20 * i, -40 + 16 * j, -1)), "move"); cs.push_back(c);
    }
    const double t2 = now();
    SmBrep* p = nullptr; ok(SmApiCreateBox(SmPoint3d(-80, -50, 0), 160, 100, 6, p), "box");
    for (SmBrep* c : cs) { SmBrep* r = nullptr; ok(SmApiBooleanDifference(p, c, r), "cut"); p = r; }
    printf("one cylinder %.2f ms, 48 copies %.2f ms; cut %.1f ms; %lu faces, volume %.3f\n", t1 - t0, t2 - t1, now() - t2, (unsigned long)faces(p), volume(p));
    delete p; delete proto;
  }
}

/** the M8 × 1.25 bolt's thread, 30 mm long: DadCAD's cross-section (120 points through the ISO profile, as
 *  engine.ts threadedSolid makes it, minus 0.1 clearance) screw-swept 24 turns */
static void boltCase(int reps) {
  const double P = 1.25, H = 0.8660254037844386 * P, rMaj = 4 - 0.1, rMin = 4 - (5.0 / 8) * H - 0.1;
  auto rad = [&](double u) { double f = std::fmod(std::fmod(u / P, 1) + 1, 1); if (f > 0.5) f = 1 - f; const double crest = 1.0 / 16, root = 1.0 / 8;
    if (f <= crest) return rMaj; if (f >= 0.5 - root) return rMin; return rMaj + (rMin - rMaj) * (f - crest) / (0.5 - root - crest); };
  SmTArray<SmPoint3d> pts;
  for (int i = 0; i < 120; i++) { const double th = 2 * M_PI * i / 120, r = rad(P * i / 120); pts.Add(SmPoint3d(r * cos(th), r * sin(th), 0)); }
  pts.Add(pts[0]);
  for (int k = 0; k < reps; k++) {
    SmCurve* c = nullptr; ok(SmApiCreateCurveInterpPoints(pts, c, SmCurveParameterizationType(1)), "interp");
    val cs = val::array(); cs.set(0u, wl::handle(c));
    val o = val::array(); o.set(0u, 0.0); o.set(1u, 0.0); o.set(2u, 0.0);
    val z = val::array(); z.set(0u, 0.0); z.set(1u, 0.0); z.set(2u, 1.0);
    val m = val::array(); m.set(0u, 0.0); m.set(1u, 0.0); m.set(2u, 30.0);
    const double t0 = now();
    val r = screwSweep(cs, o, z, m, 360 * 30 / P, 2e-4);
    const double t1 = now();
    auto* b = wl::ptr<SmBrep>(r["brep"]);
    printf("screwSweep: %.1f ms, %d steps, glued %d, %lu faces, volume %.4f (exact %.4f)\n", t1 - t0, r["steps"].as<int>(), r["glued"].as<bool>(), (unsigned long)faces(b), volume(b), r["volume"].as<double>());
    delete b; delete c;
  }
}

int main(int argc, char** argv) {
  SmApiGetOrCreateContext();
  if (argc < 2) { fprintf(stderr, "replay shell|holes|cylinders|faces <json>|screw <json> [reps]\n"); return 2; }
  const std::string c = argv[1];
  const int reps = argc > 3 ? atoi(argv[3]) : (argc > 2 && c != "faces" && c != "screw" ? atoi(argv[2]) : 1);
  if (c == "shell") shellCase(reps);
  else if (c == "holes") holesCase(reps, true);
  else if (c == "cylinders") holesCase(reps, false);
  else if (c == "copies") copiesCase(reps);
  else if (c == "bolt") boltCase(reps);
  else jsonCase(c.c_str(), argv[2], reps);
}
