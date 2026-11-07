// WASMLib's hand-written object layer: what SM_API leaves to the caller's C++ — walking a Brep's topology, reading a
// face's or an edge's geometry, meshes as typed arrays, lifetimes, files — batched so that JavaScript asks once per
// question rather than once per number.
#include "wl.h"
#include "geom.h"

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
#include <SmMessages.h>
#include <SmPlane.h>
#include <SmPoly.h>
#include <SmRegion.h>
#include <SmShell.h>
#include <SmSphere.h>
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
    else wl::check(SmApiFaceInternalPoint(f, q), "SmApiFaceInternalPoint");
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
    std::vector<float> V, N;
    std::vector<uint32_t> T, G;
    V.reserve(pfs.GetSize() * 6);
    N.reserve(pfs.GetSize() * 6);
    T.reserve(pfs.GetSize() * 3);
    G.reserve(nf * 2);
    std::unordered_map<SmPolyVertex*, uint32_t> local;
    SmTArray<SmPolyEdge*> loop;
    for (unsigned fi = 0; fi < nf; ++fi) {
        const uint32_t start = T.size();
        SmFace* orig = faces[fi];
        local.clear();
        for (SmPolyFace* pf : by[fi]) {
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
                        if (fs[k] == static_cast<SmTopology*>(orig)) { nv = ns[k]; found = true; break; }
                    if (!found) for (ULONG k = 0; k < ns.GetSize(); ++k) nv += ns[k];
                    nv = unit(nv, wind);
                    if (dot(nv, wind) < 0) nv = SmVector3d(-nv.x, -nv.y, -nv.z);
                    const uint32_t vi = V.size() / 3;
                    V.insert(V.end(), {float(q.x), float(q.y), float(q.z)});
                    N.insert(N.end(), {float(nv.x), float(nv.y), float(nv.z)});
                    it = local.emplace(pv, vi).first;
                }
                id[j] = it->second;
            }
            for (ULONG j = 1; j + 1 < n; ++j) T.insert(T.end(), {id[0], id[j], id[j + 1]});
        }
        G.push_back(start);
        G.push_back(T.size() - start);
    }
    val o = val::object();
    o.set("vertices", typed(V));
    o.set("normals", typed(N));
    o.set("triangles", typed(T));
    o.set("groups", typed(G));
    o.set("failures", wl::js(fails));
    o.set("outputManifold", static_cast<bool>(rep.outputManifold));
    return o;
    WL_END("mesh")
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
    function("faceInfo", &faceInfo);
    function("faceNormal", &faceNormal);
    function("facePoint", &facePoint);
    function("mesh", &mesh);
    function("edgeLines", &edgeLines);
    function("mirror", &mirror);
    function("transformMatrix", &transformMatrix);
    function("writeBrep", &writeBrep);
    function("readBrep", &readBrep);
}
