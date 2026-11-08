// WASMLib's solid-building additions — what SMLib's primitives don't offer, built from its own surfaces and topology:
//   screwSweep    a twisted extrusion (a thread, a helical gear, a twisted vase): a closed planar profile moved along an
//                 axis while it turns about it, as one B-spline face per profile curve and two flat ends
//   mergeDisjoint several Breps copied into one, side by side, with no intersecting (tools that don't touch, cut or
//                 joined in one boolean instead of one at a time)
//   brepBox       a bounding box that's tight on round faces (their control nets bulge), tightening only those at
//                 the extremes, and from their edges where that's exact
#include "wl.h"
#include "geom.h"

#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>
#include <SmContext.h>
#include <SmCurve.h>
#include <SmExtent2d.h>
#include <SmEdge.h>
#include <SmFace.h>
#include <SmVertex.h>
#include <SmRegion.h>
#include <SmSurfTypes.h>
#include <NL_Geometry.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace {
using namespace wlg;

inline SmVector3d add(const SmVector3d& a, const SmVector3d& b) { return SmVector3d(a.x + b.x, a.y + b.y, a.z + b.z); }
inline SmVector3d sub(const SmVector3d& a, const SmVector3d& b) { return SmVector3d(a.x - b.x, a.y - b.y, a.z - b.z); }
inline SmVector3d scl(const SmVector3d& a, double k) { return SmVector3d(a.x * k, a.y * k, a.z * k); }

/** v turned by `a` radians about the unit axis n (Rodrigues) */
inline SmVector3d turn(const SmVector3d& v, const SmVector3d& n, double a) {
    const double c = std::cos(a), s = std::sin(a);
    return add(add(scl(v, c), scl(cross(n, v), s)), scl(n, dot(n, v) * (1 - c)));
}

/** the cubic B-spline basis functions N_{span-3..span}(t) on knots U (Cox–de Boor) */
void basis3(const std::vector<double>& U, int span, double t, double N[4]) {
    double left[4], right[4];
    N[0] = 1;
    for (int j = 1; j <= 3; ++j) {
        left[j] = t - U[span + 1 - j];
        right[j] = U[span + j] - t;
        double saved = 0;
        for (int r = 0; r < j; ++r) {
            const double tmp = N[r] / (right[r + 1] + left[j - r]);
            N[r] = saved + right[r + 1] * tmp;
            saved = left[j - r] * tmp;
        }
        N[j] = saved;
    }
}

/**
 * A clamped cubic B-spline in t ∈ [0, 1] with knots at t_j = j/M, through D(t_j) for j = 0…M with the end derivatives
 * D'(0), D'(1) matched: M + 3 control points. The matrix is the same for every curve fitted on these knots, so it's
 * factored once (Thomas) and each fit is a forward and back substitution.
 */
struct CubicFit {
    int M;
    std::vector<double> knots;          // M + 7
    std::vector<double> a, b, c;        // equation j = 1…M-1: a P_j + b P_{j+1} + c P_{j+2} = D_j
    std::vector<double> cp, den;        // Thomas: modified upper diagonal, pivots
    explicit CubicFit(int m) : M(m) {
        for (int i = 0; i < 4; ++i) knots.push_back(0);
        for (int j = 1; j < M; ++j) knots.push_back(double(j) / M);
        for (int i = 0; i < 4; ++i) knots.push_back(1);
        a.assign(M + 1, 0); b.assign(M + 1, 0); c.assign(M + 1, 0);
        for (int j = 1; j < M; ++j) {
            double N[4];
            basis3(knots, j + 3, double(j) / M, N);   // non-zero: P_j … P_{j+3}, the last 0 at the knot itself
            a[j] = N[0]; b[j] = N[1]; c[j] = N[2];
        }
        // unknowns P_2 … P_M (M - 1 of them), equation j on row j - 1
        const int n = M - 1;
        cp.assign(std::max(n, 1), 0); den.assign(std::max(n, 1), 0);
        for (int r = 0; r < n; ++r) {
            const int j = r + 1;
            const double lower = r > 0 ? a[j] : 0;
            den[r] = b[j] - (r > 0 ? lower * cp[r - 1] : 0);
            cp[r] = (r < n - 1 ? c[j] : 0) / den[r];
        }
    }
    /** control points (M + 3) for data D[0…M] and end derivatives d0, d1 */
    void fit(const std::vector<SmVector3d>& D, const SmVector3d& d0, const SmVector3d& d1, std::vector<SmVector3d>& P) const {
        P.assign(M + 3, SmVector3d(0, 0, 0));
        const double h = 1.0 / (3.0 * M);       // clamped cubic: C'(0) = 3M (P_1 - P_0)
        P[0] = D[0];
        P[1] = add(D[0], scl(d0, h));
        P[M + 2] = D[M];
        P[M + 1] = sub(D[M], scl(d1, h));
        const int n = M - 1;
        if (n <= 0) return;
        std::vector<SmVector3d> y(n);
        for (int r = 0; r < n; ++r) {
            const int j = r + 1;
            SmVector3d rhs = D[j];
            if (j == 1) rhs = sub(rhs, scl(P[1], a[1]));
            if (j == M - 1) rhs = sub(rhs, scl(P[M + 1], c[M - 1]));
            const SmVector3d prev = r > 0 ? scl(y[r - 1], a[j]) : SmVector3d(0, 0, 0);
            y[r] = scl(sub(rhs, prev), 1.0 / den[r]);
        }
        for (int r = n - 2; r >= 0; --r) y[r] = sub(y[r], scl(y[r + 1], cp[r]));
        for (int r = 0; r < n; ++r) P[r + 2] = y[r];
    }
};

/** a point on an edge, a fraction of the way along it */
SmVector3d edge_at(const SmEdge* e, double f) {
    const SmExtent1d iv = e->GetInterval();
    return at(e->GetCurve(), iv.GetMin() + (iv.GetMax() - iv.GetMin()) * f);
}

/**
 * Glue every pair of free (lamina) edges that lie on top of each other — same ends, same points a quarter, half and
 * three quarters along (either way round) — vertices first, then the edges. True when no free edge is left over.
 */
bool glue_matching(SmBrep* brep, double tol) {
    SmTArray<SmEdge*> all;
    brep->GetEdges(all);
    struct E { SmEdge* e; SmVector3d a, b, q1, q2, q3; bool used; };
    std::vector<E> free;
    for (ULONG i = 0; i < all.GetSize(); ++i) {
        SmEdge* e = all[i];
        if (!e->IsLamina()) continue;
        free.push_back({e, edge_at(e, 0), edge_at(e, 1), edge_at(e, 0.25), edge_at(e, 0.5), edge_at(e, 0.75), false});
    }
    const double tol2 = std::max(tol, 1e-6);
    auto near = [&](const SmVector3d& x, const SmVector3d& y) { return len(sub(x, y)) < tol2; };
    for (size_t i = 0; i < free.size(); ++i) {
        if (free[i].used) continue;
        for (size_t j = i + 1; j < free.size(); ++j) {
            if (free[j].used) continue;
            E &A = free[i], &B = free[j];
            const bool same = near(A.a, B.a) && near(A.b, B.b) && near(A.q1, B.q1) && near(A.q2, B.q2) && near(A.q3, B.q3);
            const bool opp = !same && near(A.a, B.b) && near(A.b, B.a) && near(A.q1, B.q3) && near(A.q2, B.q2) && near(A.q3, B.q1);
            if (!same && !opp) continue;
            SmVertex *as = nullptr, *ae = nullptr, *bs = nullptr, *be = nullptr;
            A.e->GetVertices(as, ae);
            B.e->GetVertices(bs, be);
            if (!as || !ae || !bs || !be) return false;
            SmVertex* b1 = same ? bs : be;     // B's vertex at A's start
            SmVertex* b2 = same ? be : bs;     // and at A's end
            if (b1 != as && brep->GlueVertices(as, b1) != SM_SUCCESS) return false;
            if (b2 != ae && b2 != as && ae != as && brep->GlueVertices(ae, b2) != SM_SUCCESS) return false;
            if (brep->GlueEdgesGeneral(A.e, same ? SM_OT_SAME : SM_OT_OPPOSITE, 0.0, TRUE, B.e) != SM_SUCCESS) return false;
            A.used = B.used = true;
            break;
        }
        if (!free[i].used) return false;
    }
    return true;
}

/** the curve as a B-spline we may read (a copy when it had to be converted; `own` says whether to free it) */
const SmBSplineCurve* as_bspline(const SmCurve* c, SmCurve*& own) {
    own = nullptr;
    if (const auto* b = dynamic_cast<const SmBSplineCurve*>(c)) return b;
    wl::check(c->CopyAnalyticAsNurb(*SmApiGetOrCreateContext(), own), "SmCurve::CopyAnalyticAsNurb");
    const auto* b = dynamic_cast<const SmBSplineCurve*>(own);
    if (!b) throw std::runtime_error("screwSweep: a profile curve has no B-spline form");
    return b;
}

/**
 * A twisted extrusion as a solid. `curves`: the profile's closed loops (outline and any holes), all in a plane square
 * to `axis`, as free curves (left alone). Every point of the profile moves by `move` while turning `angleDeg` about
 * the axis through `origin` (right-handed about `axis`, a unit vector). Each profile curve sweeps one B-spline face:
 * its control points each follow their helix, fitted by a cubic B-spline through points a few degrees apart (with the
 * ends' directions exact) — so a section anywhere along it is the profile itself, turned, to within `tol` — and the
 * ends are the profile and its turned copy. Returns { brep, volume, steps }: the volume exactly (the profile's area ×
 * how far it moved along the axis), and how many fitted steps the turn took.
 */
val screwSweep(const val& curves, const val& originV, const val& axisV, const val& moveV, double angleDeg, double tol) {
    WL_BEGIN
    const SmContext* ctx = SmApiGetOrCreateContext();
    const SmVector3d O = wl::v3(originV), n = unit(wl::v3(axisV)), T = wl::v3(moveV);
    const double theta = angleDeg * M_PI / 180.0, height = dot(T, n);
    const unsigned count = curves["length"].as<unsigned>();
    if (!count) throw std::runtime_error("screwSweep: no profile");
    if (!(std::fabs(height) > 0)) throw std::runtime_error("screwSweep: it doesn't move along the axis");

    struct Prof { const SmBSplineCurve* c; SmCurve* own; std::vector<SmVector3d> P; std::vector<double> w; std::vector<double> U; int p; bool rational; };
    std::vector<Prof> profs;
    auto cleanup = [&]() { for (auto& pr : profs) delete pr.own; };
    double rmax = 0;
    try {
        for (unsigned i = 0; i < count; ++i) {
            Prof pr{};
            pr.c = as_bspline(wl::ptr<SmCurve>(curves[i]), pr.own);
            SmTArray<SmPoint3d> pts;
            SmTArray<double> ws, ks;
            wl::check(pr.c->GetControlPolygon(pts, ws), "SmBSplineCurve::GetControlPolygon");
            wl::check(pr.c->GetKnotsAll(ks), "SmBSplineCurve::GetKnotsAll");
            pr.rational = pr.c->IsRational() && ws.GetSize() == pts.GetSize();
            pr.p = int(pr.c->GetDegree());
            for (ULONG k = 0; k < pts.GetSize(); ++k) {
                pr.P.push_back(SmVector3d(pts[k].x, pts[k].y, pts[k].z));
                pr.w.push_back(pr.rational ? ws[k] : 1.0);
                const SmVector3d r = sub(pr.P.back(), O);
                rmax = std::max(rmax, len(sub(r, scl(n, dot(r, n)))));
            }
            for (ULONG k = 0; k < ks.GetSize(); ++k) pr.U.push_back(ks[k]);
            profs.push_back(std::move(pr));
        }

        // how far apart along the turn the helices are fitted: a cubic through points h radians apart on a circle of
        // radius r strays about 5/384 r h⁴ — within tol at h = (384 tol / 5 r)^¼ (no more than 30°)
        const double h = std::min(M_PI / 6, std::pow(384.0 * std::max(tol, 1e-9) / (5.0 * std::max(rmax, 1e-9)), 0.25));
        const int M = std::max(1, int(std::ceil(std::fabs(theta) / h - 1e-9)));
        const CubicFit fit(M);

        SmBrep* brep = new (*ctx) SmBrep();
        try {
            std::vector<SmSurface*> sides;
            std::vector<SmVector3d> D(M + 1), Q;
            for (auto& pr : profs) {
                const int nu = int(pr.P.size());
                std::vector<std::vector<gw_CPOINT>> net(nu, std::vector<gw_CPOINT>(M + 3));
                for (int i = 0; i < nu; ++i) {
                    const SmVector3d r0 = sub(pr.P[i], O);
                    for (int j = 0; j <= M; ++j) {
                        const double t = double(j) / M;
                        D[j] = add(add(O, turn(r0, n, theta * t)), scl(T, t));
                    }
                    const SmVector3d d0 = add(scl(cross(n, r0), theta), T);
                    const SmVector3d d1 = add(scl(cross(n, turn(r0, n, theta)), theta), T);
                    fit.fit(D, d0, d1, Q);
                    for (int k = 0; k < M + 3; ++k) {
                        gw_CPOINT& cpt = net[i][k];
                        const double w = pr.w[i];
                        if (pr.rational) { cpt.x = Q[k].x * w; cpt.y = Q[k].y * w; cpt.z = Q[k].z * w; cpt.w = w; }
                        else { cpt.x = Q[k].x; cpt.y = Q[k].y; cpt.z = Q[k].z; cpt.w = NL_NOW; }
                    }
                }
                std::vector<gw_CPOINT*> rows(nu);
                for (int i = 0; i < nu; ++i) rows[i] = net[i].data();
                std::vector<double> U = pr.U, V = fit.knots;
                SmBSplineSurface* s = new (*ctx) SmBSplineSurface(ULONG(nu - 1), ULONG(M + 2), short(pr.p), short(3),
                    ULONG(U.size() - 1), ULONG(V.size() - 1), rows.data(), U.data(), V.data());
                sides.push_back(s);
            }
            // the ends: each side surface's first and last rows are the profile and its turned copy, exactly
            SmTArray<SmCurve*> starts, ends;
            for (SmSurface* s : sides) {
                const SmExtent2d dom = s->GetNaturalUVDomain();
                SmBSplineCurve *c0 = nullptr, *c1 = nullptr;
                wl::check(s->CreateIsoParametricCurve(*ctx, SM_SP_V, dom.GetMin().y, tol * 0.1, c0), "SmSurface::CreateIsoParametricCurve");
                starts.Add(c0);
                wl::check(s->CreateIsoParametricCurve(*ctx, SM_SP_V, dom.GetMax().y, tol * 0.1, c1), "SmSurface::CreateIsoParametricCurve");
                ends.Add(c1);
            }
            for (SmSurface* s : sides) {
                SmTArray<SmFace*> fs;
                const SmStatus st = brep->CreateFacesFromSurface(s, s->GetNaturalUVDomain(), SM_CT_G1, fs);
                if (st != SM_SUCCESS) { for (ULONG k = 0; k < starts.GetSize(); ++k) { delete starts[k]; delete ends[k]; } wl::check(st, "SmBrep::CreateFacesFromSurface"); }
            }
            SmTArray<SmFace*> startFaces, endFaces;
            SmStatus st = brep->CreatePlanarFacesWith3DCurves(brep->GetInfiniteRegion(), starts, tol, startFaces);
            if (st != SM_SUCCESS) { for (ULONG k = 0; k < ends.GetSize(); ++k) delete ends[k]; wl::check(st, "SmBrep::CreatePlanarFacesWith3DCurves"); }
            wl::check(brep->CreatePlanarFacesWith3DCurves(brep->GetInfiniteRegion(), ends, tol, endFaces), "SmBrep::CreatePlanarFacesWith3DCurves");
            double area = 0;
            for (ULONG k = 0; k < startFaces.GetSize(); ++k) {
                double a = 0;
                wl::check(SmApiFaceComputeArea(startFaces[k], 1e-8, a), "SmApiFaceComputeArea");
                area += a;
            }
            // the faces were made apart: put them together. Which edges meet is known (the profile curves' shared ends
            // sweep the same helices, and each end face is bounded by copies of its side faces' edges), so they're
            // glued directly — SMLib's general stitch searches and measures every pair (most of the time here, on
            // a long thread). If any edge is left without its partner, the general stitch does it after all.
            const bool glued = glue_matching(brep, tol);
            if (!glued) wl::check(brep->StitchAndOrient(), "SmBrep::StitchAndOrient");
            else {
                SmBoolean maybeOpen = FALSE;
                wl::check(brep->OrientTrimmedSurfaces(TRUE, maybeOpen, FALSE, nullptr), "SmBrep::OrientTrimmedSurfaces");
            }
            cleanup();
            val o = val::object();
            o.set("brep", wl::handle(brep));
            o.set("volume", area * std::fabs(height));
            o.set("steps", M);
            o.set("glued", glued);
            return o;
        } catch (...) { delete brep; throw; }
    } catch (...) { cleanup(); throw; }
    WL_END("screwSweep")
}

/** one new Brep holding copies of all of these, side by side — no intersecting: for Breps that don't touch */
val mergeDisjoint(const val& breps) {
    WL_BEGIN
    SmBrep* dest = new (*SmApiGetOrCreateContext()) SmBrep();
    const unsigned n = breps["length"].as<unsigned>();
    for (unsigned i = 0; i < n; ++i) {
        const SmStatus s = dest->MergeBrep(*wl::ptr<SmBrep>(breps[i]), nullptr, nullptr, FALSE);
        if (s != SM_SUCCESS) { delete dest; wl::check(s, "SmBrep::MergeBrep"); }
    }
    return wl::handle(dest);
    WL_END("mergeDisjoint")
}

/**
 * A Brep's bounding box, tight where it matters and quick where it doesn't. SMLib's quick box goes by control nets,
 * which bulge up to two fifths of a radius outside round faces turned off the axes — a turned cylinder's box comes
 * out too big; its tight box is exact but slow (70 ms on a plate with 48 holes). So: the quick box of every face,
 * then only round faces that reach outside the rest are tightened — a flat face from its outer loop's edges, a
 * cylinder or cone from its edges (a ruled face never reaches past its edges), only those edges that stick out;
 * anything else round (spheres, tori, rational B-splines) from its own tight box. Plain B-spline faces keep their
 * nets' box, which hugs them. [[min], [max]].
 */
val brepBox(const val& h) {
    WL_BEGIN
    const SmBrep* b = wl::ptr<SmBrep>(h);
    SmTArray<SmFace*> fs;
    b->GetFaces(fs);
    SmPoint3d lo, hi;
    if (!fs.GetSize()) {
        wl::check(SmApiBrepBoundingBox(b, FALSE, lo, hi), "SmApiBrepBoundingBox");
    } else {
        enum Kind { NET, FLAT, RULED, ROUND };
        struct F { SmFace* f; SmPoint3d a, z; Kind k; };
        std::vector<F> faces;
        for (ULONG i = 0; i < fs.GetSize(); ++i) {
            const SmSurface* srf = fs[i]->GetSurface();
            const ULONG t = srf ? srf->GetType() : 0;
            Kind k = t == SmPlane_TYPE ? FLAT : (t == SmCylinder_TYPE || t == SmCone_TYPE) ? RULED : ROUND;
            if (k == ROUND) if (const auto* bs = dynamic_cast<const SmBSplineSurface*>(srf)) if (!bs->IsRational()) k = NET;
            F f{fs[i], {}, {}, k};
            wl::check(SmApiFaceBoundingBox(fs[i], FALSE, f.a, f.z), "SmApiFaceBoundingBox");
            faces.push_back(f);
        }
        lo = SmPoint3d(HUGE_VAL, HUGE_VAL, HUGE_VAL);
        hi = SmPoint3d(-HUGE_VAL, -HUGE_VAL, -HUGE_VAL);
        auto grow = [&](const SmPoint3d& a, const SmPoint3d& z) {
            lo = SmPoint3d(std::min(lo.x, a.x), std::min(lo.y, a.y), std::min(lo.z, a.z));
            hi = SmPoint3d(std::max(hi.x, z.x), std::max(hi.y, z.y), std::max(hi.z, z.z));
        };
        for (const F& f : faces) if (f.k == NET) grow(f.a, f.z);
        // the biggest first: they set the extremes, and the small ones inside them need no work
        std::vector<const F*> rest;
        for (const F& f : faces) if (f.k != NET) rest.push_back(&f);
        auto diag = [](const F* f) { return len(SmVector3d(f->z.x - f->a.x, f->z.y - f->a.y, f->z.z - f->a.z)); };
        std::sort(rest.begin(), rest.end(), [&](const F* x, const F* y) { return diag(x) > diag(y); });
        const double eps = 1e-9;
        auto inside = [&](const SmPoint3d& a, const SmPoint3d& z) {
            return a.x >= lo.x - eps && a.y >= lo.y - eps && a.z >= lo.z - eps && z.x <= hi.x + eps && z.y <= hi.y + eps && z.z <= hi.z + eps;
        };
        for (const F* f : rest) {
            if (inside(f->a, f->z)) continue;
            SmTArray<SmEdge*> es;
            if (f->k == FLAT) f->f->GetOuterLoopEdges(es);
            else if (f->k == RULED) f->f->GetEdges(es);
            if (!es.GetSize()) {
                SmPoint3d a, z;
                wl::check(SmApiFaceBoundingBox(f->f, TRUE, a, z), "SmApiFaceBoundingBox");
                grow(a, z);
                continue;
            }
            // (an edge whose quick box is inside already can't widen it: only those sticking out are worked out)
            for (ULONG j = 0; j < es.GetSize(); ++j) {
                SmPoint3d a, z;
                wl::check(SmApiEdgeBoundingBox(es[j], FALSE, a, z), "SmApiEdgeBoundingBox");
                if (inside(a, z)) continue;
                wl::check(SmApiEdgeBoundingBox(es[j], TRUE, a, z), "SmApiEdgeBoundingBox");
                grow(a, z);
            }
        }
    }
    val o = val::array();
    o.set(0, wl::js(SmVector3d(lo.x, lo.y, lo.z)));
    o.set(1, wl::js(SmVector3d(hi.x, hi.y, hi.z)));
    return o;
    WL_END("brepBox")
}

}  // namespace

EMSCRIPTEN_BINDINGS(wasmlib_build) {
    function("brepBox", &brepBox);
    function("screwSweep", &screwSweep);
    function("mergeDisjoint", &mergeDisjoint);
}
