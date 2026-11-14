#include <unordered_set>
#include <map>
// WASMLib's hand-written object layer: what SM_API leaves to the caller's C++ — walking a Brep's topology, reading a
// face's or an edge's geometry, meshes as typed arrays, lifetimes, files — batched so that JavaScript asks once per
// question rather than once per number.
#include "wl.h"
#include "geom.h"

#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>
#include <SmCircle.h>
#include <SmCone.h>
#include <SmContext.h>
#include <SmCopyBrepMap.h>
#include <SmCurve.h>
#include <SmCylinder.h>
#include <SmEdge.h>
#include <SmEllipse.h>
#include <SmExtent2d.h>
#include <SmFace.h>
#include <SmFaceuse.h>
#include <SmLine.h>
#include <SmLoop.h>
#include <SmMessages.h>
#include <SmPlane.h>
#include <SmPoly.h>
#include <SmRegion.h>
#include <SmShell.h>
#include <SmSphere.h>
#include <SmSurfOfExtrusion.h>
#include <SmSurface.h>
#include <SmTorus.h>
#include <SmVertex.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <unordered_map>
#include <vector>

// ---------------------------------------------------------------- errors
namespace {
std::string g_trail;

void on_kernel_error(SmStatus s, const TCHAR* const file, ULONG line, const TCHAR* const msg) {
    if (s == SM_ERR_WARNING || s == SM_ERR_MESSAGE) return;
    if (g_trail.size() > 2000) return;
    if (!g_trail.empty()) g_trail += "; ";
    g_trail += wl::status_name(s);
    if (file && *file) {
        std::string f = smos_FromTChar(file);
        const auto p = f.find_last_of("/\\");
        g_trail += " at " + (p == std::string::npos ? f : f.substr(p + 1)) + ":" + std::to_string(line);
    }
    if (msg && *msg) g_trail += " (" + smos_FromTChar(msg) + ")";
}
}  // namespace

std::string wl::take_trail() {
    std::string t;
    t.swap(g_trail);
    return t;
}

void wl::raise(const char* name, const std::string& why) {
    const std::string n(name);
    val::global("Error").new_(why.compare(0, n.size(), n) == 0 ? why : n + ": " + why).throw_();
    __builtin_unreachable();
}



namespace {
using namespace wlg;

// ---------------------------------------------------------------- small helpers
template <class T> val typed(const std::vector<T>& v) {
    // a copy owned by JavaScript (a view would go stale when the heap grows)
    return val(typed_memory_view(v.size(), v.data())).call<val>("slice");
}
inline val v3(const SmVector3d& p) { return wl::js(p); }

const char* curve_kind(const SmCurve* c) {
    if (!c) return "none";
    switch (c->GetType()) {
        case SmLine_TYPE: return "line";
        case SmCircle_TYPE: return "circle";
        case SmEllipse_TYPE: return "ellipse";
        case SmParabola_TYPE: return "parabola";
        case SmHyperbola_TYPE: return "hyperbola";
        case SmBSplineCurve_TYPE: return "bspline";
        case SmOffsetCurve_TYPE: return "offset";
        default: return "other";
    }
}
const char* surface_kind(const SmSurface* s) {
    if (!s) return "none";
    switch (s->GetType()) {
        case SmPlane_TYPE: return "plane";
        case SmCylinder_TYPE: return "cylinder";
        case SmCone_TYPE: return "cone";
        case SmSphere_TYPE: return "sphere";
        case SmTorus_TYPE: return "torus";
        case SmBSplineSurface_TYPE: return "bspline";
        case SmSurfOfRevolution_TYPE: return "revolution";
        case SmSurfOfExtrusion_TYPE: return "extrusion";
        case SmOffsetSurface_TYPE: return "offset";
        case SmBlendSurface_TYPE: return "blend";
        default: return "other";
    }
}

/** parameters at fractions of the length (a table of 96 chords, inverted linearly — exact for lines) */
std::vector<double> params_by_length(const EdgeRun& r, const std::vector<double>& fr) {
    const int N = r.c->GetType() == SmLine_TYPE ? 1 : 96;
    std::vector<double> ts(N + 1), cum(N + 1, 0.0);
    SmVector3d prev;
    for (int i = 0; i <= N; ++i) {
        ts[i] = r.t0 + (r.t1 - r.t0) * i / N;
        const SmVector3d p = at(r.c, ts[i]);
        if (i) cum[i] = cum[i - 1] + len(SmVector3d(p.x - prev.x, p.y - prev.y, p.z - prev.z));
        prev = p;
    }
    std::vector<double> out;
    out.reserve(fr.size());
    const double L = cum[N];
    for (double f : fr) {
        if (!(L > 0)) { out.push_back(r.t0 + (r.t1 - r.t0) * f); continue; }
        const double s = std::min(1.0, std::max(0.0, f)) * L;
        int i = int(std::lower_bound(cum.begin(), cum.end(), s) - cum.begin());
        if (i <= 0) { out.push_back(ts[0]); continue; }
        if (i > N) i = N;
        const double seg = cum[i] - cum[i - 1];
        const double k = seg > 0 ? (s - cum[i - 1]) / seg : 0;
        out.push_back(ts[i - 1] + (ts[i] - ts[i - 1]) * k);
    }
    return out;
}

// ---------------------------------------------------------------- the functions
val init() {
    WL_BEGIN
    SmApiGetOrCreateContext();
    smos_SetErrorCallback(on_kernel_error);
    return val::undefined();
    WL_END("init")
}

void destroy(const val& h) {
    if (SmObject* o = wl::ptr<SmObject>(h)) delete o;
}

val typeOf(const val& h) {
    WL_BEGIN
    const SmObject* o = wl::ptr<SmObject>(h);
    if (!o) return val::null();
    val r = val::object();
    r.set("id", static_cast<double>(o->GetType()));
    r.set("cls", smos_FromTChar(o->GetClassString()));
    return r;
    WL_END("typeOf")
}

// ---- topology (borrowed pointers into the Brep that owns them)
val brepTopology(const val& h) {
    WL_BEGIN
    const SmBrep* b = wl::ptr<SmBrep>(h);
    SmTArray<SmFace*> f;
    SmTArray<SmEdge*> e;
    SmTArray<SmVertex*> v;
    SmTArray<SmShell*> s;
    SmTArray<SmRegion*> rg;
    b->GetFaces(f);
    b->GetEdges(e);
    b->GetVertices(v);
    b->GetShells(s);
    b->GetRegions(rg);
    val r = val::object();
    r.set("faces", wl::js(f));
    r.set("edges", wl::js(e));
    r.set("vertices", wl::js(v));
    r.set("shells", static_cast<double>(s.GetSize()));
    r.set("regions", static_cast<double>(rg.GetSize()));
    return r;
    WL_END("brepTopology")
}
val brepFaces(const val& h) { WL_BEGIN SmTArray<SmFace*> f; wl::ptr<SmBrep>(h)->GetFaces(f); return wl::js(f); WL_END("brepFaces") }
val brepEdges(const val& h) { WL_BEGIN SmTArray<SmEdge*> e; wl::ptr<SmBrep>(h)->GetEdges(e); return wl::js(e); WL_END("brepEdges") }
val brepVertices(const val& h) { WL_BEGIN SmTArray<SmVertex*> v; wl::ptr<SmBrep>(h)->GetVertices(v); return wl::js(v); WL_END("brepVertices") }
/** faces grouped by shell (a solid's outer skin and each void's skin are shells) */
val brepShellFaces(const val& h) {
    WL_BEGIN
    SmTArray<SmFace*> all;
    wl::ptr<SmBrep>(h)->GetFaces(all);
    std::unordered_map<const SmShell*, unsigned> at;
    val out = val::array();
    unsigned n = 0;
    for (ULONG i = 0; i < all.GetSize(); ++i) {
        SmTArray<SmFaceuse*> fus;
        all[i]->GetUpwardFaceuses(fus);
        const SmFaceuse* fu = all[i]->GetUpwardFaceuse();
        const SmShell* s = fu ? fu->GetShell() : nullptr;
        auto it = at.find(s);
        if (it == at.end()) { it = at.emplace(s, n++).first; out.set(it->second, val::array()); }
        val list = out[it->second];
        list.call<void>("push", wl::handle(all[i]));
    }
    return out;
    WL_END("brepShellFaces")
}
val faceEdges(const val& h, bool outerOnly) {
    WL_BEGIN
    const SmFace* f = wl::ptr<SmFace>(h);
    SmTArray<SmEdge*> e;
    if (outerOnly) f->GetOuterLoopEdges(e); else f->GetEdges(e);
    return wl::js(e);
    WL_END("faceEdges")
}
val edgeFaces(const val& h) { WL_BEGIN SmTArray<SmFace*> f; wl::ptr<SmEdge>(h)->GetFaces(f); return wl::js(f); WL_END("edgeFaces") }
val edgeVertices(const val& h) {
    WL_BEGIN
    SmVertex *s = nullptr, *t = nullptr;
    wl::ptr<SmEdge>(h)->GetVertices(s, t);
    val r = val::array();
    r.set(0, wl::handle(s));
    r.set(1, wl::handle(t));
    return r;
    WL_END("edgeVertices")
}
val vertexPoint(const val& h) { WL_BEGIN return v3(wl::ptr<SmVertex>(h)->GetPoint()); WL_END("vertexPoint") }

// ---- geometry
/** a free curve as a run: its whole natural interval */
EdgeRun curve_run(const SmCurve* c) {
    EdgeRun r;
    r.c = c;
    const SmExtent1d iv = c->GetNaturalInterval();
    r.t0 = iv.GetMin();
    r.t1 = iv.GetMax();
    r.a = at(c, r.t0);
    r.b = at(c, r.t1);
    return r;
}
double run_length(const EdgeRun& r) {
    const int N = r.c->GetType() == SmLine_TYPE ? 1 : 256;
    double L = 0;
    SmVector3d prev = at(r.c, r.t0);
    for (int i = 1; i <= N; ++i) {
        const SmVector3d p = at(r.c, r.t0 + (r.t1 - r.t0) * i / N);
        L += len(SmVector3d(p.x - prev.x, p.y - prev.y, p.z - prev.z));
        prev = p;
    }
    return L;
}
/** a curve that is (within a hair of) a circular arc */
bool circular(const SmCurve* c, SmAxis2Placement& pl, double& rad, double& a0, double& a1) {
    const SmExtent1d iv = c->GetNaturalInterval();
    const SmVector3d p = at(c, iv.GetMin()), q = at(c, 0.5 * (iv.GetMin() + iv.GetMax()));
    const double size = std::max(1.0, len(SmVector3d(p.x - q.x, p.y - q.y, p.z - q.z)));
    return c->IsArc(24, 1e-9 * size, pl, rad, a0, a1) && rad > 0 && std::isfinite(rad);
}
void canonical(const SmCurve* c, val& o) {
    if (c && c->GetType() == SmCircle_TYPE) {
        SmAxis2Placement pl;
        double rad = 0;
        if (((const SmCircle*)c)->GetCanonical(pl, rad) == SM_SUCCESS) {
            o.set("center", v3(pl.GetOrigin()));
            o.set("axis", v3(unit(pl.GetZAxis())));
            o.set("xAxis", v3(unit(pl.GetXAxis())));
            o.set("radius", rad);
        }
    } else if (c && c->GetType() == SmEllipse_TYPE) {
        SmAxis2Placement pl;
        double r1 = 0, r2 = 0;
        if (((const SmEllipse*)c)->GetCanonical(pl, r1, r2) == SM_SUCCESS) {
            o.set("center", v3(pl.GetOrigin()));
            o.set("axis", v3(unit(pl.GetZAxis())));
            o.set("xAxis", v3(unit(pl.GetXAxis())));
            o.set("radius", r1);
            o.set("radius2", r2);
        }
    } else if (c && c->GetType() != SmLine_TYPE) {
        // a B-spline that is exactly a circular arc (sweeps, offsets and booleans make these): say so
        SmAxis2Placement pl;
        double rad = 0, a0 = 0, a1 = 0;
        SmPoint3d lp;
        SmVector3d lv;
        const SmExtent1d iv = c->GetNaturalInterval();
        const SmVector3d e0 = at(c, iv.GetMin()), e1 = at(c, iv.GetMax());
        const double size = std::max(1.0, len(SmVector3d(e1.x - e0.x, e1.y - e0.y, e1.z - e0.z)));
        if (c->IsLine(12, 1e-9 * size, lp, lv)) {
            o.set("kind", std::string("line"));
        } else if (circular(c, pl, rad, a0, a1)) {
            o.set("kind", std::string("circle"));
            o.set("center", v3(pl.GetOrigin()));
            o.set("axis", v3(unit(pl.GetZAxis())));
            o.set("xAxis", v3(unit(pl.GetXAxis())));
            o.set("radius", rad);
        }
    }
}
val sample_run(const EdgeRun& r, const val& fractions, bool tangents) {
    std::vector<double> fr = vecFromJSArray<double>(fractions);
    const std::vector<double> ts = params_by_length(r, fr);
    const double dir = r.t1 >= r.t0 ? 1.0 : -1.0;
    std::vector<double> out;
    out.reserve(ts.size() * (tangents ? 6 : 3));
    for (double t : ts) {
        SmVector3d d;
        const SmVector3d p = at(r.c, t, tangents ? &d : nullptr);
        out.insert(out.end(), {p.x, p.y, p.z});
        if (tangents) { d = unit(SmVector3d(d.x * dir, d.y * dir, d.z * dir)); out.insert(out.end(), {d.x, d.y, d.z}); }
    }
    return typed(out);
}
/** a free curve: kind, ends, length, and its circle when it is one */
val curveInfo(const val& h) {
    WL_BEGIN
    const SmCurve* c = wl::ptr<SmCurve>(h);
    const EdgeRun r = curve_run(c);
    val o = val::object();
    o.set("kind", std::string(curve_kind(c)));
    o.set("start", v3(r.a));
    o.set("end", v3(r.b));
    o.set("t0", r.t0);
    o.set("t1", r.t1);
    o.set("closed", len(SmVector3d(r.a.x - r.b.x, r.a.y - r.b.y, r.a.z - r.b.z)) < 1e-9);
    o.set("length", run_length(r));
    canonical(c, o);
    return o;
    WL_END("curveInfo")
}
val curveSample(const val& h, const val& fractions, bool tangents) {
    WL_BEGIN
    return sample_run(curve_run(wl::ptr<SmCurve>(h)), fractions, tangents);
    WL_END("curveSample")
}
/** a copy of a Brep, and where the given faces / edges / vertices of the original went in it */
val copyMapped(const val& h, const val& topos) {
    WL_BEGIN
    const SmBrep* b = wl::ptr<SmBrep>(h);
    SmCopyBrepMap rev;
    SmBrep* c = new (*SmApiGetOrCreateContext()) SmBrep(*b, nullptr, &rev, TRUE, FALSE);
    val o = val::object();
    o.set("brep", wl::handle(c));
    val m = val::array();
    const unsigned n = wl::present(topos) ? topos["length"].as<unsigned>() : 0;
    for (unsigned i = 0; i < n; ++i) m.set(i, wl::handle(rev.GetAt(wl::ptr<SmTopology>(topos[i]))));
    o.set("map", m);
    return o;
    WL_END("copyMapped")
}
/** a copy of a free curve (the caller's to delete) */
val curveCopy(const val& h) {
    WL_BEGIN
    const SmCurve* c = wl::ptr<SmCurve>(h);
    SmCurve* copy = nullptr;
    wl::check(c->Copy(*SmApiGetOrCreateContext(), copy), "SmCurve::Copy");
    return wl::handle(copy);
    WL_END("curveCopy")
}
/** a copy of a Brep with its surfaces recognized as planes, cylinders, cones, spheres and tori where they are such
 *  (sweeps and lofts build B-spline or extrusion surfaces even for round arcs) */
val withAnalytics(const val& h) {
    WL_BEGIN
    return wl::handle(new (*SmApiGetOrCreateContext()) SmBrep(*wl::ptr<SmBrep>(h), nullptr, nullptr, TRUE, TRUE));
    WL_END("withAnalytics")
}
/** a free curve's mirror image in the plane through `origin` square to `normal`: a new (B-spline) curve, the caller's */
val curveMirror(const val& h, const val& normal, const val& origin) {
    WL_BEGIN
    const SmBSplineCurve* c = dynamic_cast<const SmBSplineCurve*>(wl::ptr<SmCurve>(h));
    if (!c) throw std::runtime_error("curveMirror: the curve has no B-spline form");
    const SmVector3d n = unit(wl::v3(normal)), o = wl::v3(origin);
    const ULONG count = c->GetNumberControlPoints();
    std::vector<double> cp(4 * count);
    wl::check(c->GetControlPointsExpert(SM_CP_EUCLIDIAN_RATIONAL, 0, count - 1, 4, cp.data()), "SmBSplineCurve::GetControlPointsExpert");
    SmTArray<SmPoint3d> pts;
    SmTArray<double> ws, knots;
    SmTArray<ULONG> mults;
    for (ULONG i = 0; i < count; ++i) {
        const SmVector3d p(cp[4 * i], cp[4 * i + 1], cp[4 * i + 2]);
        const double k = 2 * dot(SmVector3d(p.x - o.x, p.y - o.y, p.z - o.z), n);
        pts.Add(SmVector3d(p.x - n.x * k, p.y - n.y * k, p.z - n.z * k));
        ws.Add(cp[4 * i + 3]);
    }
    wl::check(c->GetKnots(knots, &mults), "SmBSplineCurve::GetKnots");
    SmBSplineCurve* out = nullptr;
    wl::check(SmApiCreateCanonicalCurve(pts, knots, mults, c->IsRational() ? &ws : nullptr, c->GetDegree(), SM_CF_UNSPECIFIED, out), "SmApiCreateCanonicalCurve");
    return wl::handle(out);
    WL_END("curveMirror")
}
/** trim a free curve, in place, to [t0, t1] of its parameter */
val curveTrim(const val& h, double t0, double t1) {
    WL_BEGIN
    SmCurve* c = wl::ptr<SmCurve>(h);
    SmExtent1d iv(std::min(t0, t1), std::max(t0, t1));
    wl::check(c->Trim(iv), "SmCurve::Trim");
    return val::undefined();
    WL_END("curveTrim")
}
/** one Brep holding copies of every face of these Breps, side by side (no intersecting, no stitching: sew it after) */
val combineSheets(const val& breps) {
    WL_BEGIN
    SmBrep* dest = new (*SmApiGetOrCreateContext()) SmBrep();
    const unsigned n = breps["length"].as<unsigned>();
    for (unsigned i = 0; i < n; ++i) {
        SmBrep* b = wl::ptr<SmBrep>(breps[i]);
        SmTArray<SmFace*> fs;
        b->GetFaces(fs);
        const SmStatus s = b->CopyFaces(fs, dest, nullptr, FALSE);
        if (s != SM_SUCCESS) { delete dest; wl::check(s, "SmBrep::CopyFaces"); }
    }
    return wl::handle(dest);
    WL_END("combineSheets")
}
/** a new Brep holding copies of just these faces of a Brep, connected where they share edges (merge: TRUE) or each
 *  on its own (FALSE) */
val keepFaces(const val& h, const val& faces, bool merge) {
    WL_BEGIN
    SmBrep* b = wl::ptr<SmBrep>(h);
    SmTArray<SmFace*> fs;
    wl::fill(faces, fs);
    SmBrep* dest = new (*SmApiGetOrCreateContext()) SmBrep();
    const SmStatus s = b->CopyFaces(fs, dest, nullptr, merge ? TRUE : FALSE);
    if (s != SM_SUCCESS) { delete dest; wl::check(s, "SmBrep::CopyFaces"); }
    return wl::handle(dest);
    WL_END("keepFaces")
}

/** everything about an edge's curve the modeller asks: kind, ends, and the circle / line it lies on */
val edgeInfo(const val& h) {
    WL_BEGIN
    const SmEdge* e = wl::ptr<SmEdge>(h);
    const EdgeRun r = edge_run(e);
    SmVertex *vs = nullptr, *ve = nullptr;
    e->GetVertices(vs, ve);
    val o = val::object();
    o.set("kind", std::string(curve_kind(r.c)));
    o.set("start", v3(r.a));
    o.set("end", v3(r.b));
    o.set("t0", r.t0);
    o.set("t1", r.t1);
    o.set("closed", vs == ve);
    canonical(r.c, o);
    return o;
    WL_END("edgeInfo")
}
/** points (and unit tangents, if asked) at fractions of an edge's length: one Float64Array, xyz (+ txyz) each */
val edgeSample(const val& h, const val& fractions, bool tangents) {
    WL_BEGIN
    return sample_run(edge_run(wl::ptr<SmEdge>(h)), fractions, tangents);
    WL_END("edgeSample")
}
/** a free curve (a copy, the caller's to delete) along an edge, start to end */
val edgeCurve(const val& h) {
    WL_BEGIN
    const SmEdge* e = wl::ptr<SmEdge>(h);
    const SmCurve* c = e->GetCurve();
    SmCurve* copy = nullptr;
    wl::check(c->Copy(*c->GetContext(), copy), "SmCurve::Copy");
    SmExtent1d iv = e->GetInterval();
    const SmExtent1d nat = copy->GetNaturalInterval();
    if (iv.GetMin() > nat.GetMin() + 1e-12 || iv.GetMax() < nat.GetMax() - 1e-12) {
        if (copy->Trim(iv) != SM_SUCCESS) { delete copy; throw std::runtime_error("edgeCurve: trimming the curve failed"); }
    }
    const EdgeRun r = edge_run(e);
    if (r.t1 < r.t0) {
        // run it start to end, as the edge does
        SmExtent1d now;
        if (copy->ReverseParameterization(copy->GetNaturalInterval(), now) != SM_SUCCESS) {
            delete copy;
            throw std::runtime_error("edgeCurve: reversing the curve failed");
        }
    }
    return wl::handle(copy);
    WL_END("edgeCurve")
}
/** the analytic surface under a face, its parameter box, and whether its normal points out of the material */
val faceInfo(const val& h) {
    WL_BEGIN
    const SmFace* f = wl::ptr<SmFace>(h);
    const SmSurface* s = f->GetSurface();
    val o = val::object();
    o.set("kind", std::string(surface_kind(s)));
    o.set("sense", face_sense(f));
    const SmExtent2d uv = f->GetUVDomain();
    val box = val::array();
    box.set(0, uv.GetMin().x); box.set(1, uv.GetMax().x); box.set(2, uv.GetMin().y); box.set(3, uv.GetMax().y);
    o.set("uv", box);
    SmAxis2Placement pl;
    double a = 0, b = 0;
    auto frame = [&](const SmAxis2Placement& p) {
        o.set("origin", v3(p.GetOrigin()));
        o.set("axis", v3(unit(p.GetZAxis())));
        o.set("xAxis", v3(unit(p.GetXAxis())));
    };
    switch (s->GetType()) {
        case SmPlane_TYPE:
            if (((const SmPlane*)s)->GetCanonical(pl) == SM_SUCCESS) frame(pl);
            break;
        case SmCylinder_TYPE:
            if (((const SmCylinder*)s)->GetCanonical(pl, a) == SM_SUCCESS) { frame(pl); o.set("radius", a); }
            break;
        case SmCone_TYPE:
            if (((const SmCone*)s)->GetCanonical(pl, a, b) == SM_SUCCESS) { frame(pl); o.set("radius", a); o.set("semiAngleDeg", b); }
            break;
        case SmSphere_TYPE:
            if (((const SmSphere*)s)->GetCanonical(pl, a) == SM_SUCCESS) { frame(pl); o.set("radius", a); }
            break;
        case SmTorus_TYPE:
            if (((const SmTorus*)s)->GetCanonical(pl, a, b) == SM_SUCCESS) { frame(pl); o.set("radius", a); o.set("minorRadius", b); }
            break;
        case SmSurfOfExtrusion_TYPE: {
            // an extruded circular arc is a cylinder: find the arc among the face's edges, square to the extrusion
            const SmVector3d v = unit(((const SmSurfOfExtrusion*)s)->GetExtrusionVector());
            SmTArray<SmEdge*> es;
            f->GetEdges(es);
            for (ULONG i = 0; i < es.GetSize(); ++i) {
                double r = 0, a0 = 0, a1 = 0;
                SmAxis2Placement ap;
                if (!es[i]->GetCurve() || !circular(es[i]->GetCurve(), ap, r, a0, a1)) continue;
                const SmVector3d z = unit(ap.GetZAxis());
                if (std::fabs(std::fabs(dot(z, v)) - 1) > 1e-9) continue;
                // check: a point inside the face lies at that radius from the axis
                SmVector3d q;
                if (SmApiFaceInternalPoint(f, q) != SM_SUCCESS) break;
                const SmVector3d o0 = ap.GetOrigin(), d(q.x - o0.x, q.y - o0.y, q.z - o0.z);
                const double k = dot(d, v);
                if (std::fabs(len(SmVector3d(d.x - v.x * k, d.y - v.y * k, d.z - v.z * k)) - r) > 1e-7 * std::max(1.0, r)) break;
                o.set("kind", std::string("cylinder"));
                o.set("origin", v3(o0));
                o.set("axis", v3(v));
                o.set("xAxis", v3(unit(ap.GetXAxis())));
                o.set("radius", r);
                break;
            }
            break;
        }
        default: break;
    }
    return o;
    WL_END("faceInfo")
}
/** the outward normal at the face point nearest `p` (or at a point inside the face); { normal, point } */
val faceNormal(const val& h, const val& p) {
    WL_BEGIN
    const SmFace* f = wl::ptr<SmFace>(h);
    SmVector3d q;
    if (wl::present(p)) q = wl::v3(p);
    else q = face_point(f);
    SmVector3d where;
    const SmVector3d n = face_normal_near(f, q, &where);
    val o = val::object();
    o.set("normal", v3(n));
    o.set("point", v3(where));
    return o;
    WL_END("faceNormal")
}
/** a point on the face's surface at (u, v) as fractions of the face's parameter box */
val facePoint(const val& h, double u, double v) {
    WL_BEGIN
    const SmFace* f = wl::ptr<SmFace>(h);
    const SmExtent2d uv = f->GetUVDomain();
    const SmPoint2d q(uv.GetMin().x + (uv.GetMax().x - uv.GetMin().x) * u, uv.GetMin().y + (uv.GetMax().y - uv.GetMin().y) * v);
    SmVector3d p;
    wl::check(SmApiEvaluateSurfacePoint(f->GetSurface(), q, p), "SmApiEvaluateSurfacePoint");
    return v3(p);
    WL_END("facePoint")
}

// ---- meshes
/** triangles being gathered for a mesh: vertices and normals (xyz), triangles, and start, count per face */
struct MeshOut {
    std::vector<float> V, N;
    std::vector<uint32_t> T, G;
};
/**
 * The polyfaces SMLib's tessellator made for each face of `faces` (by[i] for faces[i]; `orig` maps a polyface's face
 * to its place, when the tessellated Brep is a copy of these faces).
 */
std::vector<std::vector<SmPolyFace*>> polyfaces_by_face(SmPolyBrep* pb, const SmTArray<SmFace*>& faces) {
    const unsigned nf = faces.GetSize();
    std::unordered_map<const SmTopology*, unsigned> index;
    index.reserve(nf * 2);
    for (unsigned i = 0; i < nf; ++i) index.emplace(faces[i], i);
    std::vector<std::vector<SmPolyFace*>> by(nf + 1);
    SmTArray<SmPolyFace*> pfs;
    if (pb) pb->GetPolyFaces(pfs);
    for (ULONG i = 0; i < pfs.GetSize(); ++i) {
        auto it = index.find(pfs[i]->GetOriginalFace());
        by[it == index.end() ? nf : it->second].push_back(pfs[i]);
    }
    return by;
}
/** one face's polyfaces as triangles (its own vertices: normals differ across an edge), normals out of the material */
void append_polyfaces(MeshOut& o, const std::vector<SmPolyFace*>& pfs, const SmFace* orig) {
    std::unordered_map<SmPolyVertex*, uint32_t> local;
    SmTArray<SmPolyEdge*> loop;
    for (SmPolyFace* pf : pfs) {
        SmPolyLoop* pl = pf->GetOuterPolyLoop();
        if (!pl) continue;
        loop.RemoveAll();
        pl->GetPolyEdges(loop);
        const ULONG n = loop.GetSize();
        if (n < 3) continue;
        const SmVector3d wind = unit(pf->GetNormal(TRUE, FALSE));
        uint32_t ids[64];
        std::vector<uint32_t> big;
        uint32_t* id = n <= 64 ? ids : (big.resize(n), big.data());
        for (ULONG j = 0; j < n; ++j) {
            SmPolyVertex* pv = loop[j]->GetStartPolyVertex();
            auto it = local.find(pv);
            if (it == local.end()) {
                const SmPoint3d q = pv->GetPoint();
                // the vertex's normal on this face's surface (a vertex on an edge has one per face), turned to
                // agree with the facet's winding, which runs round the outward normal
                const auto& ns = pv->GetNormalsRef();
                const auto& fs = pv->GetFacesRef();
                SmVector3d nv(0, 0, 0);
                bool found = false;
                for (ULONG k = 0; k < fs.GetSize() && k < ns.GetSize(); ++k)
                    if (fs[k] == static_cast<const SmTopology*>(orig)) { nv = ns[k]; found = true; break; }
                if (!found) for (ULONG k = 0; k < ns.GetSize(); ++k) nv += ns[k];
                nv = unit(nv, wind);
                if (dot(nv, wind) < 0) nv = SmVector3d(-nv.x, -nv.y, -nv.z);
                const uint32_t vi = o.V.size() / 3;
                o.V.insert(o.V.end(), {float(q.x), float(q.y), float(q.z)});
                o.N.insert(o.N.end(), {float(nv.x), float(nv.y), float(nv.z)});
                it = local.emplace(pv, vi).first;
            }
            id[j] = it->second;
        }
        for (ULONG j = 1; j + 1 < n; ++j) o.T.insert(o.T.end(), {id[0], id[j], id[j + 1]});
    }
}
val mesh_js(MeshOut& o, const SmTArray<SmTessellationFailure>& fails, bool manifold) {
    val r = val::object();
    r.set("vertices", typed(o.V));
    r.set("normals", typed(o.N));
    r.set("triangles", typed(o.T));
    r.set("groups", typed(o.G));
    r.set("failures", wl::js(fails));
    r.set("outputManifold", manifold);
    return r;
}
/**
 * Triangles for a whole Brep, grouped by face in the Brep's face order: { vertices, normals (Float32Array xyz),
 * triangles (Uint32Array), groups (Uint32Array: start, count per face, in triangle-index units), failures }. Each
 * face has its own vertices (normals differ across an edge); normals point out of the material.
 */
val mesh(const val& h, const val& params) {
    WL_BEGIN
    SmBrep* b = wl::ptr<SmBrep>(h);
    SmTessellationParams p;
    if (wl::present(params)) wl::from(params, p);
    SmPolyBrep* pb = nullptr;
    SmTArray<SmTessellationFailure> fails;
    SmTessellationReport rep;
    wl::check(SmApiTessellate(b, pb, p, TRUE, &fails, &rep), "SmApiTessellate");
    struct Own { SmPolyBrep* p; ~Own() { delete p; } } own{pb};

    SmTArray<SmFace*> faces;
    b->GetFaces(faces);
    const auto by = polyfaces_by_face(pb, faces);
    MeshOut o;
    o.V.reserve(by.size() * 64);
    for (unsigned fi = 0; fi < faces.GetSize(); ++fi) {
        const uint32_t start = o.T.size();
        append_polyfaces(o, by[fi], faces[fi]);
        o.G.push_back(start);
        o.G.push_back(o.T.size() - start);
    }
    return mesh_js(o, fails, rep.outputManifold);
    WL_END("mesh")
}

// ---- drawing: faces sampled on a grid
/**
 * How finely to sample a face on a grid: steps across its box in u and v that keep the facets within `tol` of the
 * surface and turning by at most `ang` (radians) from one to the next. Measured along five lines each way from the
 * curvature across each line: the steps an adaptive split would take (the need integrated along the line), doubled
 * where the need is uneven but never past what its worst spot needs — a sharp turn inside one span (a thread's
 * crest) is rounded a little rather than sampled finely all round. At least one step per knot span.
 */
struct GridPlan { double nu = 0, nv = 0; };
GridPlan plan_grid(const SmSurface* s, const SmExtent2d& box, ULONG su, ULONG sv, double tol, double ang) {
    GridPlan g;
    const SmPoint2d lo = box.GetMin();
    const double w[2] = {box.GetMax().x - lo.x, box.GetMax().y - lo.y};
    const ULONG spans[2] = {su, sv};
    for (int dir = 0; dir < 2; ++dir) {
        const ULONG m = std::min<ULONG>(4000, std::max<ULONG>(32, spans[dir] * 4));
        double most = 0;
        for (int l = 0; l < 5; ++l) {
            const double across = (l + 0.5) / 5;
            double sum = 0, peak = 0;
            for (ULONG k = 0; k < m; ++k) {
                const double t = (k + 0.5) / m;
                const SmPoint2d uv = dir == 0 ? SmPoint2d(lo.x + w[0] * t, lo.y + w[1] * across) : SmPoint2d(lo.x + w[0] * across, lo.y + w[1] * t);
                SmPoint3d p;
                SmVector3d du, dv, duv, duu, dvv;
                if (s->Evaluate2ndDerivatives(uv, TRUE, TRUE, p, du, dv, duv, duu, dvv) != SM_SUCCESS) continue;
                const double lu = len(du), lv = len(dv);
                // derivatives along the line, per unit of t (the fraction of the box)
                const SmVector3d a = dir == 0 ? du : dv, c = dir == 0 ? duu : dvv;
                const double L = w[dir], sp = (dir == 0 ? lu : lv) * L;
                if (!(sp > 1e-12)) continue;
                const SmVector3d d1(a.x * L, a.y * L, a.z * L), d2(c.x * L * L, c.y * L * L, c.z * L * L);
                const double along = dot(d2, d1) / (sp * sp);
                const double kp = len(SmVector3d(d2.x - d1.x * along, d2.y - d1.y * along, d2.z - d1.z * along));
                // a step h bows kp·h²/8 off the surface and turns by kp·h/sp
                const double need = std::max(std::sqrt(kp / (8 * tol)), kp / sp / ang);
                sum += need / m;
                peak = std::max(peak, need);
            }
            most = std::max(most, std::min(peak, 2 * sum));
        }
        (dir == 0 ? g.nu : g.nv) = std::max({most, double(spans[dir]), 4.0});
    }
    return g;
}
/** distinct knot spans of a B-spline surface across [a, b] of one parameter */
ULONG knot_spans(const SmBSplineSurface* s, SmSurfParamType d, double a, double b) {
    SmTArray<double> k;
    SmTArray<ULONG> m;
    const SmExtent1d iv(a, b);
    if (s->GetKnots(d, k, &m, &iv) != SM_SUCCESS || k.GetSize() < 2) return 1;
    return k.GetSize() - 1;
}
/**
 * A face whose one loop runs along the sides of its parameter box to within `tol` (in space), though its trims
 * aren't exactly lines of constant u or v — a thread cut to a nut's height, whose trims came from intersecting the
 * flank with the nut's top and bottom (the planes the thread's ends lie in). Each edge is sampled, each sample's
 * place on the surface found, moved onto the nearest side of the box, and must stay within `tol`; `box` is set to
 * the face's parameter box.
 */
bool sides_of_box(const SmFace* f, SmExtent2d& box, double tol) {
    SmTArray<SmLoop*> loops;
    f->GetLoops(loops);
    if (loops.GetSize() != 1) return false;
    SmTArray<SmEdge*> es;
    loops[0]->GetEdges(es);
    if (es.GetSize() < 2 || es.GetSize() > 4) return false;
    box = f->GetUVDomain();
    const SmSurface* s = f->GetSurface();
    const SmPoint2d lo = box.GetMin(), hi = box.GetMax();
    for (ULONG i = 0; i < es.GetSize(); ++i) {
        const EdgeRun r = edge_run(es[i]);
        int side = -1;  // 0: u at either end (a seam may come back as either), 1: v low, 2: v high
        for (int k = 1; k < 8; ++k) {  // (its ends sit on corners, on two sides at once)
            const SmVector3d p = at(r.c, r.t0 + (r.t1 - r.t0) * k / 8);
            SmPoint3d q;
            SmTArray<SmPoint2d> uvs;
            double d = 0;
            if (SmApiSurfaceClosestPoint(s, p, q, uvs, d) != SM_SUCCESS || !uvs.GetSize() || d > tol) return false;
            const SmPoint2d uv = uvs[0];
            // the nearest side of the box, and the point moved onto it
            const double du = std::min(std::fabs(uv.x - lo.x), std::fabs(uv.x - hi.x)) / (hi.x - lo.x);
            const double dl = std::fabs(uv.y - lo.y) / (hi.y - lo.y), dh = std::fabs(uv.y - hi.y) / (hi.y - lo.y);
            const int near = du <= std::min(dl, dh) ? 0 : dl <= dh ? 1 : 2;
            if (side < 0) side = near;
            else if (near != side) return false;
            const SmPoint2d m = near == 0 ? SmPoint2d(std::fabs(uv.x - lo.x) < std::fabs(uv.x - hi.x) ? lo.x : hi.x, uv.y)
                                          : SmPoint2d(uv.x, near == 1 ? lo.y : hi.y);
            SmVector3d on;
            if (SmApiEvaluateSurfacePoint(s, m, on) != SM_SUCCESS || len(on - p) > tol) return false;
        }
    }
    return true;
}
/**
 * The face as a grid of samples, appended to `o` — if it's worth it: a B-spline face that is all of a rectangle of
 * its surface's parameters (one loop of four trims along constant u or v: a thread's flank, a twisted extrusion's
 * side, a loft) and either big (as many control points as a thread's flank) or with parameter lines meeting at
 * under 15° somewhere (a twist: SMLib's tessellator, which triangulates in the parameter plane, folds a third of a
 * thread's triangles, as both directions run round the axis 2° apart). Points and normals (outward, from the
 * surface's derivatives) at the grid's corners; each cell split along its shorter diagonal. False: not gridded.
 */
/** a face sampled on a grid: how many cells each way, and which way round its normal runs */
struct GridInfo { ULONG nu = 0, nv = 0; double sense = 1, tol = 0; };
bool append_grid(MeshOut& o, const SmFace* f, double tol, double ang, double maxPoints, GridInfo* info = nullptr) {
    const SmBSplineSurface* bs = dynamic_cast<const SmBSplineSurface*>(f->GetSurface());
    if (!bs) return false;
    // worth it? (cheap tests first: most B-spline faces — a fillet, a gear's tooth — mesh fine)
    {
        const SmExtent2d d = f->GetUVDomain();
        const SmPoint2d a = d.GetMin(), z = d.GetMax();
        if (!(z.x > a.x) || !(z.y > a.y)) return false;
        const ULONG poles = (knot_spans(bs, SM_SP_U, a.x, z.x) + bs->GetDegree(SM_SP_U)) * (knot_spans(bs, SM_SP_V, a.y, z.y) + bs->GetDegree(SM_SP_V));
        bool sheared = false;
        for (int i = 0; i < 5 && !sheared && poles < 3000; ++i)
            for (int j = 0; j < 5 && !sheared; ++j) {
                SmPoint3d p;
                SmVector3d du, dv;
                if (bs->Evaluate1stDerivatives(SmPoint2d(a.x + (z.x - a.x) * (i + 0.5) / 5, a.y + (z.y - a.y) * (j + 0.5) / 5), TRUE, TRUE, p, du, dv) != SM_SUCCESS) continue;
                const double lu = len(du), lv = len(dv);
                sheared = lu > 0 && lv > 0 && len(cross(du, dv)) < 0.25 * lu * lv;
            }
        if (poles < 3000 && !sheared) return false;
    }
    SmExtent2d box;
    SmBoolean rect = FALSE, natural = FALSE;
    if (f->FindIfRectangularTrim(box, rect, natural) != SM_SUCCESS) return false;
    if (rect != TRUE && !sides_of_box(f, box, tol)) return false;
    const SmPoint2d lo = box.GetMin(), hi = box.GetMax();
    if (!(hi.x > lo.x) || !(hi.y > lo.y)) return false;
    const ULONG su = knot_spans(bs, SM_SP_U, lo.x, hi.x), sv = knot_spans(bs, SM_SP_V, lo.y, hi.y);
    GridPlan g = plan_grid(bs, box, su, sv, tol, ang);
    // too many points: fewer each way, in proportion
    const double want = (g.nu + 1) * (g.nv + 1);
    if (want > maxPoints) { const double k = std::sqrt(maxPoints / want); g.nu *= k; g.nv *= k; }
    const ULONG nu = std::max<ULONG>(2, ULONG(std::ceil(g.nu))), nv = std::max<ULONG>(2, ULONG(std::ceil(g.nv)));
    const double sense = face_sense(f);
    if (info) { info->nu = nu; info->nv = nv; info->sense = sense; info->tol = tol; }
    const uint32_t base = o.V.size() / 3;
    o.V.reserve(o.V.size() + (nu + 1) * (nv + 1) * 3);
    o.N.reserve(o.N.size() + (nu + 1) * (nv + 1) * 3);
    std::vector<uint8_t> flat;  // samples where the derivatives give no normal (a pole): from the neighbours after
    for (ULONG i = 0; i <= nu; ++i) {
        const double u = i == nu ? hi.x : lo.x + (hi.x - lo.x) * i / nu;
        for (ULONG j = 0; j <= nv; ++j) {
            const double v = j == nv ? hi.y : lo.y + (hi.y - lo.y) * j / nv;
            SmPoint3d p;
            SmVector3d du, dv;
            SmVector3d n(0, 0, 0);
            if (bs->Evaluate1stDerivatives(SmPoint2d(u, v), i < nu, j < nv, p, du, dv) != SM_SUCCESS) return false;
            const SmVector3d c = cross(du, dv);
            const double cl = len(c);
            if (cl > 1e-12 * std::max(1.0, len(du) * len(dv))) n = SmVector3d(c.x * sense / cl, c.y * sense / cl, c.z * sense / cl);
            else { if (flat.empty()) flat.assign((nu + 1) * (nv + 1), 0); flat[i * (nv + 1) + j] = 1; }
            o.V.insert(o.V.end(), {float(p.x), float(p.y), float(p.z)});
            o.N.insert(o.N.end(), {float(n.x), float(n.y), float(n.z)});
        }
    }
    const auto at = [&](ULONG i, ULONG j) { return base + uint32_t(i * (nv + 1) + j); };
    const auto P = [&](uint32_t k) { return SmVector3d(o.V[k * 3], o.V[k * 3 + 1], o.V[k * 3 + 2]); };
    if (!flat.empty()) {
        // a pole: the normal of the cells round it, from their corners
        for (ULONG i = 0; i <= nu; ++i)
            for (ULONG j = 0; j <= nv; ++j) {
                if (!flat[i * (nv + 1) + j]) continue;
                const ULONG i0 = i ? i - 1 : 0, i1 = i < nu ? i + 1 : nu, j0 = j ? j - 1 : 0, j1 = j < nv ? j + 1 : nv;
                const SmVector3d a = P(at(i1, j)) - P(at(i0, j)), b = P(at(i, j1)) - P(at(i, j0));
                SmVector3d n = unit(cross(a, b), SmVector3d(0, 0, 0));
                if (len(n) == 0) {
                    // both ways collapse (the pole itself): the cells beside it
                    const SmVector3d c1 = P(at(i1, j1)) - P(at(i0, j0)), c2 = P(at(i0, j1)) - P(at(i1, j0));
                    n = unit(cross(c1, c2), SmVector3d(0, 0, 0));
                }
                const uint32_t k = at(i, j);
                o.N[k * 3] = float(n.x * sense); o.N[k * 3 + 1] = float(n.y * sense); o.N[k * 3 + 2] = float(n.z * sense);
            }
    }
    // cells wound round the outward normal: (u, v) → (u + 1, v) → (u, v + 1) is anticlockwise about du × dv
    for (ULONG i = 0; i < nu; ++i)
        for (ULONG j = 0; j < nv; ++j) {
            const uint32_t a = at(i, j), b = at(i + 1, j), c = at(i + 1, j + 1), d = at(i, j + 1);
            // WASMLib: of the two ways to split the cell, the one whose triangles both face the way the surface does
            // there (on a thread's sheared cells the shorter diagonal can fold one back); the shorter when both do
            const auto Nv = [&](uint32_t k) { return SmVector3d(o.N[k * 3], o.N[k * 3 + 1], o.N[k * 3 + 2]); };
            const auto agree = [&](uint32_t x, uint32_t y, uint32_t z) {
                const SmVector3d n = cross(P(y) - P(x), P(z) - P(x)) * sense, m = Nv(x) + Nv(y) + Nv(z);
                const double l = len(n) * len(m);
                return l > 0 ? dot(n, m) / l : 1.0;
            };
            const double qac = std::min(agree(a, b, c), agree(a, c, d)), qbd = std::min(agree(a, b, d), agree(b, c, d));
            const bool shortAC = len(P(c) - P(a)) <= len(P(d) - P(b));
            const bool ac = (qac > 0) == (qbd > 0) ? (std::min(qac, qbd) > 0.5 ? shortAC : qac >= qbd) : qac > 0;
            if (sense > 0) {
                if (ac) o.T.insert(o.T.end(), {a, b, c, a, c, d});
                else o.T.insert(o.T.end(), {a, b, d, b, c, d});
            } else {
                if (ac) o.T.insert(o.T.end(), {a, c, b, a, d, c});
                else o.T.insert(o.T.end(), {a, d, b, b, d, c});
            }
        }
    return true;
}
/**
 * WASMLib: a gridded face made to meet its neighbours exactly, for export. `g` holds the face's grid (from append_grid,
 * (nu+1)×(nv+1) points, i along u); `pfs` its triangles from tessellating the whole Brep, whose outline is the run of
 * points the faces beside it share. The grid's inside points are kept (its cells, unfolded), and the band between the
 * grid's outermost inside ring and the face's own outline is filled by walking both round the face in step — each
 * outline point placed by where it lands on the grid's edge — so every outline point is a corner of the result and the
 * file stays watertight. False (the caller keeps SMLib's triangles) when the face has no single outline or a grid too
 * coarse to have an inside.
 */
bool append_grid_sealed(MeshOut& o, const MeshOut& g, const GridInfo& gi, const std::vector<SmPoint3d>& cands) {
    const ULONG nu = gi.nu, nv = gi.nv;
    if (nu < 3 || nv < 3) return false;
    // the grid's edge as a closed run of points, (0,0) → (nu,0) → (nu,nv) → (0,nv), and how far round each is
    const auto G = [&](ULONG i, ULONG j) { const size_t k = (i * (nv + 1) + j) * 3; return SmVector3d(g.V[k], g.V[k + 1], g.V[k + 2]); };
    std::vector<std::pair<ULONG, ULONG>> rim;
    for (ULONG i = 0; i < nu; ++i) rim.push_back({i, 0});
    for (ULONG j = 0; j < nv; ++j) rim.push_back({nu, j});
    for (ULONG i = nu; i > 0; --i) rim.push_back({i, nv});
    for (ULONG j = nv; j > 0; --j) rim.push_back({0, j});
    const size_t R = rim.size();
    std::vector<SmVector3d> rp(R);
    for (size_t k = 0; k < R; ++k) rp[k] = G(rim[k].first, rim[k].second);
    // the four sides as runs of rim segments
    const size_t side[5] = {0, nu, nu + nv, 2 * nu + nv, R};
    // a closed face's seam: two sides on top of each other (a thread's flank, its profile closed)
    const double eps = std::max(2 * gi.tol, 1e-6);
    const bool vSeam = len(G(nu / 2, 0) - G(nu / 2, nv)) < eps && len(G(0, 0) - G(0, nv)) < eps;
    const bool uSeam = len(G(0, nv / 2) - G(nu, nv / 2)) < eps && len(G(0, 0) - G(nu, 0)) < eps;
    // where a point lands on one side: (distance, place round the rim)
    const auto onSide = [&](const SmVector3d& q, int sd) {
        double best = 1e300, at = side[sd];
        bool bestClamped = true;
        for (size_t k = side[sd]; k < side[sd + 1]; ++k) {
            const SmVector3d a = rp[k], b = rp[(k + 1) % R], ab = b - a;
            const double l2 = dot(ab, ab);
            double t0 = l2 > 0 ? dot(q - a, ab) / l2 : 0, t = std::max(0.0, std::min(1.0, t0));
            const SmVector3d d = q - (a + ab * t);
            const double dd = dot(d, d);
            // (a closed side — the end of a thread, round to where it began — meets itself at the corner: a point by
            // the corner goes on the segment it's partway along, not the one whose end merely touches it)
            const bool clamped = t0 <= 0 || t0 >= 1;
            if (dd < best * (1 - 1e-9) - 1e-24 || (dd <= best * (1 + 1e-9) + 1e-24 && bestClamped && !clamped)) { best = dd; at = k + t; bestClamped = clamped; }
        }
        return std::make_pair(std::sqrt(best), at);
    };
    // the outline: the points this face shares with the faces beside it (a point on an edge belongs to both faces), and
    // the points down its seam — each placed on every side it lies on (a seam point on both of its sides, a corner of
    // the face on the two sides it ends) and put in order round the rim. (The tessellation's own triangles can meet
    // T-wise inside a face, so its edges don't say what's outline.)
    struct OP { int c; double s; };  // c: the candidate (−1: a grid point down a seam)
    std::vector<OP> ops;
    {
        // (only points near the face's box are looked at)
        SmVector3d lo(1e300, 1e300, 1e300), hi(-1e300, -1e300, -1e300);
        for (const auto& q : rp) { lo.x = std::min(lo.x, q.x); lo.y = std::min(lo.y, q.y); lo.z = std::min(lo.z, q.z); hi.x = std::max(hi.x, q.x); hi.y = std::max(hi.y, q.y); hi.z = std::max(hi.z, q.z); }
        for (size_t ci = 0; ci < cands.size(); ++ci) {
            const SmVector3d q(cands[ci].x, cands[ci].y, cands[ci].z);
            if (q.x < lo.x - eps || q.y < lo.y - eps || q.z < lo.z - eps || q.x > hi.x + eps || q.y > hi.y + eps || q.z > hi.z + eps) continue;
            // (a point of a face beside it lies on its edge: on more than one side only where sides truly meet, at a
            // corner — a thread's seam starts almost along its end, a hair apart)
            std::vector<double> at;
            double nearest = 1e300, nearAt = 0;
            for (int sd = 0; sd < 4; ++sd) {
                // (a seam's own points come from the grid; a point of a face beside it lands on a seam only at a corner)
                const bool seamSide = (vSeam && (sd == 0 || sd == 2)) || (uSeam && (sd == 1 || sd == 3));
                const auto [d, sAt] = onSide(q, sd);
                if (!seamSide && d < nearest) { nearest = d; nearAt = sAt; }
                if (d < 1e-5 * (1 + len(q))) {
                    double sm = std::fmod(sAt, double(R));
                    bool dup = false;
                    for (double x : at) { double dd = std::abs(x - sm); if (std::min(dd, R - dd) < 1e-7) dup = true; }
                    if (!dup) at.push_back(sm);
                }
            }
            if (nearest > eps) continue;  // not on this face's edge
            if (at.empty()) at.push_back(std::fmod(nearAt, double(R)));
            for (double sm : at) ops.push_back({int(ci), sm});
        }
    }
    // down a seam the outline is the grid's own row (the same points on both of its sides, so they meet exactly, and a
    // band of whole cells along it): a seam meets no other face, only itself
    std::vector<std::pair<double, SmVector3d>> seamPts;
    if (vSeam) for (ULONG i = 1; i < nu; ++i) { const SmVector3d q = G(i, 0); seamPts.push_back({double(i), q}); seamPts.push_back({double(nu + nv + (nu - i)), q}); }
    if (uSeam) for (ULONG j = 1; j < nv; ++j) { const SmVector3d q = G(0, j); seamPts.push_back({double(nu + j), q}); seamPts.push_back({double(2 * nu + nv + (nv - j)), q}); }
    for (const auto& sp : seamPts) ops.push_back({-1, sp.first});
    if (ops.size() < 4) return false;
    std::stable_sort(ops.begin(), ops.end(), [](const OP& x, const OP& y) { return x.s < y.s; });
    // the inside ring, (1,1) → (nu-1,1) → (nu-1,nv-1) → (1,nv-1), each placed at the rim point it faces
    std::vector<std::pair<ULONG, ULONG>> ring;
    for (ULONG i = 1; i < nu - 1; ++i) ring.push_back({i, 1});
    for (ULONG j = 1; j < nv - 1; ++j) ring.push_back({nu - 1, j});
    for (ULONG i = nu - 1; i > 1; --i) ring.push_back({i, nv - 1});
    for (ULONG j = nv - 1; j > 1; --j) ring.push_back({1, j});
    const auto rimIndex = [&](ULONG i, ULONG j) {
        const ULONG ri = i == 1 ? 0 : i == nu - 1 ? nu : i, rj = j == 1 ? 0 : j == nv - 1 ? nv : j;
        for (size_t k = 0; k < R; ++k) if (rim[k].first == ri && rim[k].second == rj) return double(k);
        return 0.0;
    };
    // vertices: the grid's inside points, then the outline's (a seam point once for each side it's on)
    const uint32_t base = o.V.size() / 3;
    const auto gi_ = [&](ULONG i, ULONG j) { return base + uint32_t((i - 1) * (nv - 1) + (j - 1)); };
    for (ULONG i = 1; i < nu; ++i) for (ULONG j = 1; j < nv; ++j) {
        const size_t k = (i * (nv + 1) + j) * 3;
        o.V.insert(o.V.end(), {g.V[k], g.V[k + 1], g.V[k + 2]});
        o.N.insert(o.N.end(), {g.N[k], g.N[k + 1], g.N[k + 2]});
    }
    const uint32_t ob = o.V.size() / 3;
    std::vector<double> os(ops.size());
    for (size_t k = 0; k < ops.size(); ++k) {
        SmPoint3d q;
        if (ops[k].c >= 0) q = cands[ops[k].c];
        else { const double sk = ops[k].s; for (const auto& sp : seamPts) if (sp.first == sk) { q = SmPoint3d(sp.second.x, sp.second.y, sp.second.z); break; } }
        os[k] = ops[k].s;
        // (the normal of the nearest rim point: the outline's own may be the neighbour's)
        const auto& rc = rim[size_t(std::floor(ops[k].s + 0.5)) % R];
        const size_t nk = (rc.first * (nv + 1) + rc.second) * 3;
        o.V.insert(o.V.end(), {float(q.x), float(q.y), float(q.z)});
        o.N.insert(o.N.end(), {g.N[nk], g.N[nk + 1], g.N[nk + 2]});
    }
    const size_t outlineN = ops.size();
    // inside cells, wound as append_grid winds them
    const double sense = gi.sense;
    const auto cell = [&](uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
        if (sense > 0) o.T.insert(o.T.end(), {a, b, c, a, c, d}); else o.T.insert(o.T.end(), {a, c, b, a, d, c});
    };
    for (ULONG i = 1; i + 1 < nu; ++i) for (ULONG j = 1; j + 1 < nv; ++j) cell(gi_(i, j), gi_(i + 1, j), gi_(i + 1, j + 1), gi_(i, j + 1));
    const auto tri = [&](uint32_t a, uint32_t b, uint32_t c) { if (sense > 0) o.T.insert(o.T.end(), {a, b, c}); else o.T.insert(o.T.end(), {a, c, b}); };
    // the band: both runs from the outline point placed lowest, round once, in step (rim order is anticlockwise about
    // du × dv, the same way the cells go)
    const size_t no = outlineN, nr = ring.size();
    std::vector<double> rs(nr);
    for (size_t k = 0; k < nr; ++k) rs[k] = rimIndex(ring[k].first, ring[k].second);
    size_t o0 = std::min_element(os.begin(), os.end()) - os.begin();
    size_t r0 = 0; { double best = 1e300; for (size_t k = 0; k < nr; ++k) { double d = rs[k] - os[o0]; if (d < 0) d += R; if (d < best) { best = d; r0 = k; } } }
    const auto unwrap = [&](double s, double from) { while (s < from - 1e-9) s += R; return s; };
    size_t a = 0, b = 0;
    const double s0 = os[o0];
    while (a < no || b < nr) {
        // (the step that closes each run comes back round to where it began: a whole turn on)
        const double sa = a < no ? (a + 1 == no ? s0 + R : unwrap(os[(o0 + a + 1) % no], s0)) : 1e300;
        const double sb = b < nr ? (b + 1 == nr ? unwrap(rs[r0], s0) + R : unwrap(rs[(r0 + b + 1) % nr], s0)) : 1e300;
        const uint32_t A0 = ob + uint32_t((o0 + a) % no), B0 = gi_(ring[(r0 + b) % nr].first, ring[(r0 + b) % nr].second);
        // (in step round the face, and of two ways on the one with the shorter new side: no long fans across the band)
        bool stepA = a < no && (b >= nr || sa <= sb);
        if (a < no && b < nr) {
            const uint32_t A1 = ob + uint32_t((o0 + a + 1) % no), B1 = gi_(ring[(r0 + b + 1) % nr].first, ring[(r0 + b + 1) % nr].second);
            const auto Pv = [&](uint32_t k) { return SmVector3d(o.V[k * 3], o.V[k * 3 + 1], o.V[k * 3 + 2]); };
            const double dA = len(Pv(A1) - Pv(B0)), dB = len(Pv(B1) - Pv(A0));
            // (but never so far out of step that a run gets ahead by more than a grid cell)
            if (std::abs(sa - sb) < 1.0) stepA = dA <= dB;
        }
        if (stepA) {
            // (a corner point placed on both its sides comes twice in a row: nothing between the two)
            const uint32_t A1 = ob + uint32_t((o0 + a + 1) % no);
            if (o.V[A0 * 3] != o.V[A1 * 3] || o.V[A0 * 3 + 1] != o.V[A1 * 3 + 1] || o.V[A0 * 3 + 2] != o.V[A1 * 3 + 2]) tri(A0, A1, B0);
            ++a;
        }
        else { tri(A0, gi_(ring[(r0 + b + 1) % nr].first, ring[(r0 + b + 1) % nr].second), B0); ++b; }
    }
    return true;
}
/**
 * Triangles for drawing a Brep: as `mesh`, except faces worth sampling on a grid (see append_grid) are — the rest are
 * copied out and tessellated without them, so a thread's flank costs no tessellation at all. `grid`: { tol (largest
 * distance off the surface), angleDeg (largest turn between neighbouring facets), maxPoints (per face), sealed }. Not
 * watertight (the grid's edges don't share the neighbours' points): for the view — unless `sealed`, for export: then
 * the whole Brep is tessellated as `mesh` does and each grid stitched to its face's outline (append_grid_sealed). Same
 * arrays as mesh, plus `gridded` (Uint8Array, 1 for a face sampled on a grid).
 */
val displayMesh(const val& h, const val& params, const val& grid) {
    WL_BEGIN
    SmBrep* b = wl::ptr<SmBrep>(h);
    SmTessellationParams p;
    if (wl::present(params)) wl::from(params, p);
    const double tol = grid["tol"].as<double>(), ang = grid["angleDeg"].as<double>() * M_PI / 180;
    const double maxPoints = wl::present(grid["maxPoints"]) ? grid["maxPoints"].as<double>() : 400000;
    // WASMLib: { sealed: true } — for export: the gridded faces meet their neighbours point for point (see
    // append_grid_sealed), the whole Brep tessellated as `mesh` does and the grid stitched to each face's outline
    const bool sealed = wl::present(grid["sealed"]) && grid["sealed"].as<bool>();
    if (!(tol > 0) || !(ang > 0)) throw std::runtime_error("displayMesh: grid tol and angleDeg must be positive");

    SmTArray<SmFace*> faces;
    b->GetFaces(faces);
    const unsigned nf = faces.GetSize();
    std::vector<MeshOut> grids(nf);
    std::vector<uint8_t> gridded(nf, 0);
    SmTArray<SmFace*> rest;
    std::vector<unsigned> restAt;  // rest[k] is faces[restAt[k]]
    std::vector<GridInfo> infos(nf);
    for (unsigned i = 0; i < nf; ++i) {
        if (append_grid(grids[i], faces[i], tol, ang, maxPoints, &infos[i])) gridded[i] = 1;
        else { grids[i] = MeshOut(); rest.Add(faces[i]); restAt.push_back(i); }
    }
    SmTArray<SmTessellationFailure> fails;
    SmTessellationReport rep;
    std::vector<std::vector<SmPolyFace*>> by;
    SmPolyBrep* pb = nullptr;
    SmBrep* copy = nullptr;
    SmTArray<SmFace*> copies;
    struct Own { SmPolyBrep*& p; SmBrep*& b; ~Own() { delete p; delete b; } } own{pb, copy};
    if (rest.GetSize() == nf) {
        wl::check(SmApiTessellate(b, pb, p, TRUE, &fails, &rep), "SmApiTessellate");
        by = polyfaces_by_face(pb, faces);
    } else if (rest.GetSize()) {
        copy = new (*SmApiGetOrCreateContext()) SmBrep();
        wl::check(b->CopyFaces(rest, copy, &copies, FALSE), "SmBrep::CopyFaces");
        if (copies.GetSize() != rest.GetSize()) throw std::runtime_error("displayMesh: copying the faces lost some");
        wl::check(SmApiTessellate(copy, pb, p, TRUE, &fails, &rep), "SmApiTessellate");
        const auto byCopy = polyfaces_by_face(pb, copies);
        by.assign(nf + 1, {});
        for (unsigned k = 0; k < restAt.size(); ++k) by[restAt[k]] = byCopy[k];
    }
    // sealed: the points of the faces tessellated (the ones beside each gridded face among them), for the grids to meet
    std::vector<SmPoint3d> cands;
    if (sealed && pb) {
        SmTArray<SmPolyFace*> pfs;
        pb->GetPolyFaces(pfs);
        std::unordered_set<SmPolyVertex*> seen;
        SmTArray<SmPolyEdge*> loop;
        for (ULONG i = 0; i < pfs.GetSize(); ++i) {
            SmPolyLoop* pl = pfs[i]->GetOuterPolyLoop(); if (!pl) continue;
            loop.RemoveAll(); pl->GetPolyEdges(loop);
            for (ULONG j = 0; j < loop.GetSize(); ++j) { SmPolyVertex* v = loop[j]->GetStartPolyVertex(); if (seen.insert(v).second) cands.push_back(v->GetPoint()); }
        }
    }
    // (each grid sealed first: if one can't be, the whole Brep is meshed as `mesh` does — a part sealed and a part not
    // wouldn't meet)
    std::vector<MeshOut> sealedOut(sealed ? nf : 0);
    if (sealed) for (unsigned i = 0; i < nf; ++i) if (gridded[i] && !append_grid_sealed(sealedOut[i], grids[i], infos[i], cands)) {
        SmTArray<SmTessellationFailure> f2; SmTessellationReport r2; SmPolyBrep* p2 = nullptr;
        struct Own2 { SmPolyBrep*& p; ~Own2() { delete p; } } own2{p2};
        wl::check(SmApiTessellate(b, p2, p, TRUE, &f2, &r2), "SmApiTessellate");
        const auto all = polyfaces_by_face(p2, faces);
        MeshOut m;
        for (unsigned k = 0; k < nf; ++k) { const uint32_t st = m.T.size(); append_polyfaces(m, all[k], faces[k]); m.G.push_back(st); m.G.push_back(m.T.size() - st); }
        val r = mesh_js(m, f2, r2.outputManifold);
        r.set("gridded", typed(std::vector<uint8_t>(nf, 0)));
        return r;
    }
    MeshOut o;
    for (unsigned i = 0; i < nf; ++i) {
        const uint32_t start = o.T.size();
        if (gridded[i] && sealed) {
            const uint32_t off = o.V.size() / 3;
            o.V.insert(o.V.end(), sealedOut[i].V.begin(), sealedOut[i].V.end());
            o.N.insert(o.N.end(), sealedOut[i].N.begin(), sealedOut[i].N.end());
            for (uint32_t t : sealedOut[i].T) o.T.push_back(t + off);
            grids[i] = MeshOut(); sealedOut[i] = MeshOut();
        } else if (gridded[i]) {
            const uint32_t off = o.V.size() / 3;
            o.V.insert(o.V.end(), grids[i].V.begin(), grids[i].V.end());
            o.N.insert(o.N.end(), grids[i].N.begin(), grids[i].N.end());
            for (uint32_t t : grids[i].T) o.T.push_back(t + off);
            grids[i] = MeshOut();
        } else {
            const auto k = std::find(restAt.begin(), restAt.end(), i) - restAt.begin();
            append_polyfaces(o, by[i], copy ? copies[k] : faces[i]);
        }
        o.G.push_back(start);
        o.G.push_back(o.T.size() - start);
    }
    // (sealed: the faces copied out to be tessellated can put a point a hair off zero where the face beside it has it
    // at zero, −1.8e−15 against 0 — as floats, two points; flushed, one)
    if (sealed) for (float& x : o.V) if (std::abs(x) < 1e-7f) x = 0.0f;
    val r = mesh_js(o, fails, false);
    r.set("gridded", typed(gridded));
    return r;
    WL_END("displayMesh")
}
/**
 * Every edge of a Brep as a polyline, in the Brep's edge order: { lines (Float32Array: segment end pairs, xyz),
 * groups (Uint32Array: start, count per edge, in points) }.
 */
val edgeLines(const val& h, double angleTolDeg) {
    WL_BEGIN
    SmBrep* b = wl::ptr<SmBrep>(h);
    SmTArray<SmPoint3d> pts;
    SmTArray<ULONG> counts;
    SmTArray<SmEdge*> es;
    wl::check(SmApiTessellateBoundaries(b, angleTolDeg, pts, counts, es), "SmApiTessellateBoundaries");
    SmTArray<SmEdge*> order;
    b->GetEdges(order);
    std::unordered_map<const SmEdge*, std::pair<ULONG, ULONG>> runs;  // edge → (first point, count)
    ULONG k = 0;
    for (ULONG i = 0; i < es.GetSize() && i < counts.GetSize(); ++i) { runs[es[i]] = {k, counts[i]}; k += counts[i]; }
    std::vector<float> L;
    std::vector<uint32_t> G;
    L.reserve(pts.GetSize() * 6);
    for (ULONG i = 0; i < order.GetSize(); ++i) {
        const uint32_t start = L.size() / 3;
        auto it = runs.find(order[i]);
        if (it != runs.end()) {
            const ULONG a = it->second.first, n = it->second.second;
            for (ULONG j = 1; j < n; ++j) {
                const SmPoint3d& p = pts[a + j - 1];
                const SmPoint3d& q = pts[a + j];
                L.insert(L.end(), {float(p.x), float(p.y), float(p.z), float(q.x), float(q.y), float(q.z)});
            }
        }
        G.push_back(start);
        G.push_back(L.size() / 3 - start);
    }
    val o = val::object();
    o.set("lines", typed(L));
    o.set("groups", typed(G));
    return o;
    WL_END("edgeLines")
}

// ---- transforms (in place)
/** reflect a Brep in the plane through `origin` square to `normal` */
val mirror(const val& h, const val& normal, const val& origin) {
    WL_BEGIN
    SmObject* o = wl::ptr<SmObject>(h);
    if (!o->IsKindOf(SmBrep_TYPE)) throw std::runtime_error("mirror: only a Brep can be mirrored");
    const SmVector3d n = unit(wl::v3(normal)), x = perp(n), y = cross(n, x);
    wl::check(((SmBrep*)o)->Mirror(SmAxis2Placement(wl::v3(origin), x, y)), "SmBrep::Mirror");
    return val::undefined();
    WL_END("mirror")
}
/** a general 4×4 transform (row-major, 12 numbers: three rows of rotation-and-scale then shift) of a Brep */
val transformMatrix(const val& h, const val& m) {
    WL_BEGIN
    SmObject* o = wl::ptr<SmObject>(h);
    if (!o->IsKindOf(SmBrep_TYPE)) throw std::runtime_error("transformMatrix: only a Brep takes a general matrix");
    double M[4][4] = {{0}};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) M[r][c] = m[r * 4 + c].as<double>();
    M[3][3] = 1;
    wl::check(((SmBrep*)o)->Transform(M), "SmBrep::Transform");
    return val::undefined();
    WL_END("transformMatrix")
}

// ---- files: SMLib's own Brep format through the in-memory file system
val writeBrep(const val& h, bool ascii) {
    WL_BEGIN
    const char* path = "/tmp/wl_out.smb";
    wl::check(SmApiWriteBrepToFile(wl::ptr<SmBrep>(h), path, ascii), "SmApiWriteBrepToFile");
    std::ifstream in(path, std::ios::binary);
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    std::remove(path);
    return typed(bytes);
    WL_END("writeBrep")
}
val readBrep(const val& bytes, bool ascii) {
    WL_BEGIN
    const char* path = "/tmp/wl_in.smb";
    {
        std::vector<uint8_t> v = vecFromJSArray<uint8_t>(bytes);
        std::ofstream out(path, std::ios::binary);
        out.write(reinterpret_cast<const char*>(v.data()), v.size());
    }
    SmBrep* b = nullptr;
    const SmApiStatus s = SmApiReadBrepFromFile(path, ascii, TRUE, b);
    std::remove(path);
    wl::check(s, "SmApiReadBrepFromFile");
    return wl::handle(b);
    WL_END("readBrep")
}

}  // namespace

EMSCRIPTEN_BINDINGS(wasmlib_objects) {
    function("init", &init);
    function("destroy", &destroy);
    function("typeOf", &typeOf);
    function("brepTopology", &brepTopology);
    function("brepFaces", &brepFaces);
    function("brepEdges", &brepEdges);
    function("brepVertices", &brepVertices);
    function("brepShellFaces", &brepShellFaces);
    function("faceEdges", &faceEdges);
    function("edgeFaces", &edgeFaces);
    function("edgeVertices", &edgeVertices);
    function("vertexPoint", &vertexPoint);
    function("edgeInfo", &edgeInfo);
    function("edgeSample", &edgeSample);
    function("edgeCurve", &edgeCurve);
    function("curveInfo", &curveInfo);
    function("curveSample", &curveSample);
    function("copyMapped", &copyMapped);
    function("keepFaces", &keepFaces);
    function("curveCopy", &curveCopy);
    function("combineSheets", &combineSheets);
    function("curveTrim", &curveTrim);
    function("curveMirror", &curveMirror);
    function("withAnalytics", &withAnalytics);
    function("faceInfo", &faceInfo);
    function("faceNormal", &faceNormal);
    function("facePoint", &facePoint);
    function("mesh", &mesh);
    function("displayMesh", &displayMesh);
    function("edgeLines", &edgeLines);
    function("mirror", &mirror);
    function("transformMatrix", &transformMatrix);
    function("writeBrep", &writeBrep);
    function("readBrep", &readBrep);
}
