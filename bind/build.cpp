// WASMLib's solid-building additions — what SMLib's primitives don't offer, built from its own surfaces and topology:
//   screwSweep    a twisted extrusion (a thread, a helical gear, a twisted vase): a closed planar profile moved along an
//                 axis while it turns about it, as one B-spline face per profile curve and two flat ends
//   mergeDisjoint several Breps copied into one, side by side, with no intersecting (tools that don't touch, cut or
//                 joined in one boolean instead of one at a time)
//   brepBox       a bounding box that's tight on round faces (their control nets bulge), tightening only those at
//                 the extremes, and from their edges where that's exact
//   brepFromFaces a solid (or shell) from faces given as NURBS surfaces bounded by loops of NURBS curves — a shape
//                 from another kernel brought into SMLib (DadCAD's STEP and mesh imports, read by OpenCascade)
//   sweepPath     a profile swept along a path of several curves joined end to end, with mitred corners where the
//                 path turns, the profile kept where it was drawn (SMLib's curve sweep takes one curve and moves the
//                 profile onto it; its planar-path sweep re-places the profile)
#include "wl.h"
#include "geom.h"

#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>
#include <SmCompositeCurve.h>
#include <SmContext.h>
#include <SmCurve.h>
#include <SmExtent2d.h>
#include <SmEdge.h>
#include <SmFace.h>
#include <SmPrimitiveCreation.h>
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

// ---------------------------------------------------------------- brepFromFaces
/** knots that start and end p + 1 times over (SMLib's NURBS are clamped: an unclamped, periodic net reads past its ends) */
bool clamped(const std::vector<double>& U, int p) {
    const size_t m = U.size() - 1;
    for (int i = 1; i <= p; ++i)
        if (U[i] != U[0] || U[m - i] != U[m]) return false;
    return U[m] > U[0];
}
/** a NURBS curve from { p, U (every knot), P (x y z per control point), W (weights, or null) } */
SmBSplineCurve* nurbs_curve(const val& c) {
    const int p = c["p"].as<int>();
    const std::vector<double> U = vecFromJSArray<double>(c["U"]), P = vecFromJSArray<double>(c["P"]);
    const bool rational = !c["W"].isNull() && !c["W"].isUndefined();
    const std::vector<double> W = rational ? vecFromJSArray<double>(c["W"]) : std::vector<double>();
    const size_t n = P.size() / 3;
    if (p < 1 || n < 2 || U.size() != n + p + 1 || (rational && W.size() != n)) throw std::runtime_error("brepFromFaces: a curve's control points, weights and knots don't match");
    if (!clamped(U, p)) throw std::runtime_error("brepFromFaces: a curve's knots aren't clamped (each end p + 1 times over)");
    std::vector<gw_CPOINT> Pw(n);
    for (size_t i = 0; i < n; ++i) {
        const double w = rational ? W[i] : 1.0;
        Pw[i].x = P[3 * i] * w; Pw[i].y = P[3 * i + 1] * w; Pw[i].z = P[3 * i + 2] * w;
        Pw[i].w = rational ? w : NL_NOW;
    }
    std::vector<double> UU = U;
    return new (*SmApiGetOrCreateContext()) SmBSplineCurve(3, ULONG(n - 1), ULONG(UU.size() - 1), short(p), Pw.data(), UU.data());
}

/** a NURBS surface from { p, q, U, V (every knot), nu, nv, P (x y z, nv per row of u), W (or null) } */
SmBSplineSurface* nurbs_surface(const val& s) {
    const int p = s["p"].as<int>(), q = s["q"].as<int>(), nu = s["nu"].as<int>(), nv = s["nv"].as<int>();
    std::vector<double> U = vecFromJSArray<double>(s["U"]), V = vecFromJSArray<double>(s["V"]);
    const std::vector<double> P = vecFromJSArray<double>(s["P"]);
    const bool rational = !s["W"].isNull() && !s["W"].isUndefined();
    const std::vector<double> W = rational ? vecFromJSArray<double>(s["W"]) : std::vector<double>();
    if (nu < 2 || nv < 2 || P.size() != size_t(3 * nu * nv) || U.size() != size_t(nu + p + 1) || V.size() != size_t(nv + q + 1)
        || (rational && W.size() != size_t(nu * nv)))
        throw std::runtime_error("brepFromFaces: a surface's control points, weights and knots don't match");
    if (p < 1 || q < 1 || !clamped(U, p) || !clamped(V, q)) throw std::runtime_error("brepFromFaces: a surface's knots aren't clamped (each end p + 1 times over)");
    std::vector<std::vector<gw_CPOINT>> net(nu, std::vector<gw_CPOINT>(nv));
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j) {
            const size_t k = size_t(i) * nv + j;
            const double w = rational ? W[k] : 1.0;
            gw_CPOINT& c = net[i][j];
            c.x = P[3 * k] * w; c.y = P[3 * k + 1] * w; c.z = P[3 * k + 2] * w;
            c.w = rational ? w : NL_NOW;
        }
    std::vector<gw_CPOINT*> rows(nu);
    for (int i = 0; i < nu; ++i) rows[i] = net[i].data();
    return new (*SmApiGetOrCreateContext()) SmBSplineSurface(ULONG(nu - 1), ULONG(nv - 1), short(p), short(q),
        ULONG(U.size() - 1), ULONG(V.size() - 1), rows.data(), U.data(), V.data());
}

/**
 * A solid from faces made elsewhere. `faces`: [{ surface, reversed, loops: [{ curves, senses }] }] — each face's
 * surface (NURBS), whether the material is on the side its normal points away from (`reversed`), and its boundary:
 * the outer loop first, running anticlockwise seen from the material's outside, then the holes, each a chain of
 * curves (NURBS, `senses[i]` -1 where curve i runs against the loop). A seam is listed twice, once each way.
 * Each face is made on its own, then they're stitched along the edges they share and turned outward. Returns
 * { brep, solid: closed manifold solid?, faces: how many faces were made, failed: indexes of faces that couldn't be }.
 */
val brepFromFaces(const val& faces, double tol) {
    WL_BEGIN
    const SmContext* ctx = SmApiGetOrCreateContext();
    SmBrep* brep = new (*ctx) SmBrep();
    val failed = val::array();
    int made = 0;
    try {
        const unsigned nf = faces["length"].as<unsigned>();
        for (unsigned fi = 0; fi < nf; ++fi) {
            const val f = faces[fi];
            SmBSplineSurface* srf = nullptr;
            SmTArray<SmCurve*> curves;
            SmTArray<SmOrientType> orients;
            SmTArray<ULONG> loops;
            try {
                srf = nurbs_surface(f["surface"]);
                const val ls = f["loops"];
                const unsigned nl = ls["length"].as<unsigned>();
                for (unsigned li = 0; li < nl; ++li) {
                    const val cs = ls[li]["curves"], ss = ls[li]["senses"];
                    const unsigned nc = cs["length"].as<unsigned>();
                    ULONG count = 0;
                    for (unsigned ci = 0; ci < nc; ++ci) {
                        const val spec = cs[ci];
                        SmBSplineCurve* c = nurbs_curve(spec);
                        if (spec["from"].isUndefined() || spec["from"].isNull()) {
                            curves.Add(c);
                            orients.Add(ss[ci].as<int>() < 0 ? SM_OT_OPPOSITE : SM_OT_SAME);
                            count++;
                            continue;
                        }
                        // a stretch of the curve: from `from` to `to`, along the curve's parameter or against it
                        // (`along`); a closed curve may be crossed at its start, in two pieces
                        curves.Add(c);   // (owned by the list from here, for the clean-up)
                        const SmExtent1d iv = c->GetNaturalInterval();
                        auto param = [&](const val& pt) {
                            SmPoint3d q, near;
                            q = wl::v3(pt);
                            double t = 0, d = 0;
                            wl::check(SmApiCurveClosestPoint(c, q, near, t, d), "SmApiCurveClosestPoint");
                            return t;
                        };
                        const double t0 = param(spec["from"]), t1 = param(spec["to"]);
                        const bool along = spec["along"].as<bool>();
                        const double eps = 1e-9 * std::max(1.0, iv.GetLength());
                        auto piece = [&](SmBSplineCurve* cc, double a, double b, bool fwd) {
                            if (b - a > eps && (a > iv.GetMin() + eps || b < iv.GetMax() - eps)) { SmExtent1d ivt(a, b); wl::check(cc->Trim(ivt), "SmBSplineCurve::Trim"); }
                            orients.Add(fwd ? SM_OT_SAME : SM_OT_OPPOSITE);
                        };
                        curves.RemoveLast();
                        if (std::fabs(t0 - t1) <= eps) {           // the whole (closed) curve
                            curves.Add(c); piece(c, iv.GetMin(), iv.GetMax(), along); count++;
                        } else if (along ? t0 < t1 : t0 > t1) {    // one stretch
                            curves.Add(c); piece(c, std::min(t0, t1), std::max(t0, t1), along); count++;
                        } else {                                   // across the start of a closed curve
                            SmBSplineCurve* c2 = new (*SmApiGetOrCreateContext()) SmBSplineCurve(*c);
                            if (along) {
                                curves.Add(c);  piece(c, t0, iv.GetMax(), true);
                                curves.Add(c2); piece(c2, iv.GetMin(), t1, true);
                            } else {
                                curves.Add(c);  piece(c, iv.GetMin(), t0, false);
                                curves.Add(c2); piece(c2, t1, iv.GetMax(), false);
                            }
                            count += 2;
                        }
                    }
                    loops.Add(count);
                }
            } catch (...) {
                delete srf;
                for (ULONG k = 0; k < curves.GetSize(); ++k) delete curves[k];
                throw;
            }
            SmRegion* newRegion = nullptr;
            SmShell* newShell = nullptr;
            SmFace* newFace = nullptr;
            SmTArray<SmPoint3d> noPoints;
            if (!loops.GetSize()) {   // no boundary: the whole surface (a full sphere or torus)
                const SmStatus st = brep->CreateFaceInRegionFromSurface(nullptr, srf, srf->GetNaturalUVDomain(), newShell, newFace);
                if (st == SM_SUCCESS && newFace) made++;
                else failed.call<void>("push", fi);
                continue;
            }
            // (MakeFaceWithCurves takes the surface and curves over, whether or not it succeeds)
            const SmStatus st = brep->MakeFaceWithCurves(nullptr, loops, &curves, nullptr, orients, noPoints, srf,
                srf->GetNaturalUVDomain(), f["reversed"].as<bool>() ? SM_OT_OPPOSITE : SM_OT_SAME,
                newRegion, newShell, newFace);
            if (st == SM_SUCCESS && newFace) made++;
            else failed.call<void>("push", fi);
        }
        // the faces were made apart: sew them along their shared edges, and turn them all outward
        wl::check(brep->StitchAndOrient(tol > 0 ? tol : -0.1), "SmBrep::StitchAndOrient");
        SmBoolean solid = FALSE;
        SmApiBrepIsManifoldSolid(brep, solid);
        val o = val::object();
        o.set("brep", wl::handle(brep));
        o.set("solid", bool(solid));
        o.set("faces", made);
        o.set("failed", failed);
        return o;
    } catch (...) { delete brep; throw; }
    WL_END("brepFromFaces")
}

}  // namespace

// ---------------------------------------------------------------- sweepPath
/** a copy of a free curve, run the other way when `back` */
SmCurve* copy_curve(const SmCurve* c, bool back) {
    SmCurve* out = nullptr;
    wl::check(c->Copy(*SmApiGetOrCreateContext(), out), "SmCurve::Copy");
    if (back) {
        SmExtent1d now;
        if (out->ReverseParameterization(out->GetNaturalInterval(), now) != SM_SUCCESS) { delete out; throw std::runtime_error("sweepPath: reversing a curve failed"); }
    }
    return out;
}
SmVector3d curve_end(const SmCurve* c, bool end, SmVector3d* tangent = nullptr) {
    const SmExtent1d iv = c->GetNaturalInterval();
    SmVector3d d;
    const SmVector3d p = at(c, end ? iv.GetMax() : iv.GetMin(), &d);
    if (tangent) *tangent = unit(d);
    return p;
}
/** the rigid motion turning by `a` radians about the unit axis n through C */
SmAxis2Placement turn_about(const SmVector3d& C, const SmVector3d& n, double a) {
    SmAxis2Placement m;
    m.SetCanonical(sub(C, turn(C, n, a)), turn(SmVector3d(1, 0, 0), n, a), turn(SmVector3d(0, 1, 0), n, a));
    return m;
}
/** a box filling the slab of space within `size` of the plane through C square to n, on its side `sign` (±1) */
SmBrep* half_space(const SmVector3d& C, const SmVector3d& n, int sign, double size) {
    SmBrep* b = nullptr;
    wl::check(SmApiCreateBox(SmVector3d(-size, -size, sign > 0 ? 0 : -size), 2 * size, 2 * size, size, b), "SmApiCreateBox");
    const SmVector3d x = perp(n), y = cross(n, x);
    SmAxis2Placement m;
    m.SetCanonical(C, x, y);
    wl::check(SmApiTransform(b, m), "SmApiTransform");
    return b;
}

/**
 * A closed profile swept along one curve, kept where it lies. SMLib's curve sweep moves the profile's first point (or
 * its middle) onto the path's start before sweeping, which moves a profile drawn off the path. This is the same sweep
 * without that move: the path's start put at the origin heading along x and the profile moved with it, SMLib's sweep
 * surface (NLib's: a frame that turns with the path as little as it can) for each chain of profile curves, split where
 * the chain has corners, both ends capped flat, stitched, and moved back. `endProf` gets the profile where the sweep
 * ends (free curves, the caller's).
 */
SmBrep* sweep_along(const std::vector<SmCurve*>& prof, const SmBSplineCurve* path, std::vector<SmCurve*>& endProf) {
    const SmContext* ctx = SmApiGetOrCreateContext();
    const double tol = SM_ZONE_TOL_3D;
    const SmVector3d O(0, 0, 0), X(1, 0, 0), Y(0, 1, 0);
    SmVector3d pd[2];
    wl::check(path->Evaluate(path->GetNaturalInterval().GetMin(), 1, TRUE, pd), "SmBSplineCurve::Evaluate");
    SmAxis2Placement toOrigin(O, X, Y);
    toOrigin.Translate(scl(pd[0], -1));
    SmVector3d fx, fy, fz;
    pd[1].MakeUnitOrthoVectors(nullptr, fx, fy, fz);
    const SmAxis2Placement rot(O, fx, fy, O, X, Y);
    SmAxis2Placement rotBack, toOriginBack;
    rot.Invert(rotBack);
    toOrigin.Invert(toOriginBack);

    std::vector<SmCurve*> loose;                 // freed on the way out
    SmTArray<SmBSplineCurve*> nurbs;
    SmTArray<SmCompositeCurve*> comps;
    SmBrep* brep = nullptr;
    struct Free {
        std::vector<SmCurve*>& l; SmTArray<SmBSplineCurve*>& n; SmTArray<SmCompositeCurve*>& c; SmBrep*& b;
        ~Free() { for (auto* x : l) delete x; for (ULONG i = 0; i < n.GetSize(); ++i) delete n[i]; for (ULONG i = 0; i < c.GetSize(); ++i) delete c[i]; delete b; }
    } fr{loose, nurbs, comps, brep};

    SmBSplineCurve* tr = new (*ctx) SmBSplineCurve(*path);
    loose.push_back(tr);
    tr->Transform(toOrigin);
    tr->Transform(rot);
    for (auto* c : prof) {
        SmBSplineCurve* b = nullptr;
        if (const auto* bc = dynamic_cast<const SmBSplineCurve*>(c)) b = new (*ctx) SmBSplineCurve(*bc);
        else {
            SmCurve* k = nullptr;
            wl::check(c->CopyAnalyticAsNurb(*ctx, k), "SmCurve::CopyAnalyticAsNurb");
            b = dynamic_cast<SmBSplineCurve*>(k);
            if (!b) { delete k; throw std::runtime_error("sweepPath: a profile curve has no B-spline form"); }
        }
        b->Transform(toOrigin);
        b->Transform(rot);
        nurbs.Add(b);
    }
    brep = new (*ctx) SmBrep();
    // the profile's curves joined into chains (each chain one sweep surface); the chains own what they took
    wl::check(SmCompositeCurve::BuildCompositesFromCurves(*ctx, nurbs, TRUE, 0.0, brep->GetTolerance(), 0.0, 0.0, 0.0, 0.0, comps), "SmCompositeCurve::BuildCompositesFromCurves");
    for (ULONG i = 0; i < comps.GetSize(); ++i) {
        ULONG dim = 0;
        SmBoolean closedC = FALSE;
        SmTArray<SmCurve*> taken;
        wl::check(comps[i]->GetCanonical(dim, taken, closedC), "SmCompositeCurve::GetCanonical");
        for (ULONG j = 0; j < taken.GetSize(); ++j)
            for (ULONG k = 0; k < nurbs.GetSize(); ++k)
                if (nurbs[k] == taken[j]) nurbs[k] = nullptr;
    }
    SmTArray<SmCurve*> starts, ends;
    for (ULONG i = 0; i < comps.GetSize(); ++i) {
        SmBSplineCurve* chain = nullptr;
        wl::check(comps[i]->MakeCompositeNurb(*ctx, chain), "SmCompositeCurve::MakeCompositeNurb");
        SmBSplineSurface* srf = nullptr;
        const SmStatus st = SmBSplineSurface::CreateSweepSurface(*ctx, chain, tr, nullptr, FALSE, tol, srf);
        delete chain;
        wl::check(st, "SmBSplineSurface::CreateSweepSurface");
        // u runs along the path: the profile where the sweep starts and ends
        const SmExtent2d dom = srf->GetNaturalUVDomain();
        SmBSplineCurve *c0 = nullptr, *c1 = nullptr;
        if (srf->CreateIsoParametricCurve(*ctx, SM_SP_U, dom.GetMin().x, tol / 10.0, c0) != SM_SUCCESS ||
            srf->CreateIsoParametricCurve(*ctx, SM_SP_U, dom.GetMax().x, tol / 10.0, c1) != SM_SUCCESS) {
            delete c0; delete c1; delete srf;
            throw std::runtime_error("sweepPath: the sweep's ends couldn't be found");
        }
        starts.Add(c0); ends.Add(c1);
        loose.push_back(c0); loose.push_back(c1);
        SmTArray<SmFace*> faces;
        wl::check(brep->CreateFacesFromSurface(srf, dom, SM_CT_G1, faces), "SmBrep::CreateFacesFromSurface");
    }
    brep->Transform(rotBack);
    brep->Transform(toOriginBack);
    for (ULONG i = 0; i < starts.GetSize(); ++i) {
        starts[i]->Transform(rotBack); starts[i]->Transform(toOriginBack);
        ends[i]->Transform(rotBack); ends[i]->Transform(toOriginBack);
        SmCurve* k = nullptr;
        wl::check(ends[i]->Copy(*ctx, k), "SmCurve::Copy");
        endProf.push_back(k);
    }
    // the two ends capped; the Brep owns the curves its faces took
    const auto cap = [&](SmTArray<SmCurve*>& cs) {
        SmTArray<SmFace*> faces;
        const SmStatus st = brep->CreatePlanarFacesWith3DCurves(nullptr, cs, tol, faces);
        for (ULONG i = 0; i < cs.GetSize(); ++i)
            if (cs[i]->GetOwner() != nullptr) loose.erase(std::remove(loose.begin(), loose.end(), cs[i]), loose.end());
        wl::check(st, "SmBrep::CreatePlanarFacesWith3DCurves");
    };
    cap(starts);
    cap(ends);
    brep->StitchAndOrient();
    SmBrep* out = brep;
    brep = nullptr;
    return out;
}

/**
 * A closed profile swept along a path of several curves (as OpenCascade's pipe shell with right-corner transitions).
 * `profile`: one closed loop of free curves, sitting at the path's start; `path`: free curves in order, each starting
 * where the last one ends (either way round: they're turned to run on, and the whole path is run from whichever end
 * the profile sits at). Each curve is swept on its own, the profile kept where it lies: along a line it's pushed (a
 * prism), round an arc it's turned about the arc's axis (planes, cylinders and tori, exact); along anything else it's
 * carried by SMLib's sweep surface without being moved onto the path first (sweep_along). The next piece
 * starts from where the last one left the profile — turned about the corner by the turn the path makes there, if it
 * turns by more than `cornerDeg` (a smooth join carries straight on). Such a corner is closed with a mitre: each pipe runs on straight past the corner and is cut where
 * they meet, on the plane halfway between their directions. All the pieces are joined. Inputs are left alone; returns
 * the solid (yours).
 */
val sweepPath(const val& profileV, const val& pathV, double tol, double cornerDeg) {
    WL_BEGIN
    const SmContext* ctx = SmApiGetOrCreateContext();
    std::vector<SmCurve*> owned;
    std::vector<SmBrep*> breps;
    struct Free { std::vector<SmCurve*>& c; std::vector<SmBrep*>& b; ~Free() { for (auto* x : c) delete x; for (auto* x : b) delete x; } } fr{owned, breps};
    const unsigned np = profileV["length"].as<unsigned>(), n = pathV["length"].as<unsigned>();
    if (!np || !n) throw std::runtime_error("sweepPath: no profile or no path");
    std::vector<SmCurve*> prof;
    SmExtent3d box;
    for (unsigned i = 0; i < np; ++i) {
        SmCurve* c = copy_curve(wl::ptr<SmCurve>(profileV[i]), false);
        owned.push_back(c); prof.push_back(c);
        SmExtent3d b;
        c->CalculateBoundingBox(c->GetNaturalInterval(), &b);
        box.AddPoint3d(b.GetMin()); box.AddPoint3d(b.GetMax());
    }
    const SmVector3d mid = box.GetMid();
    const double reach = len(sub(box.GetMax(), box.GetMin()));

    // the path, each curve running on from the last
    std::vector<SmCurve*> path;
    for (unsigned i = 0; i < n; ++i) {
        const SmCurve* c = wl::ptr<SmCurve>(pathV[i]);
        bool back = false;
        if (i == 0 && n > 1) {
            const SmCurve* next = wl::ptr<SmCurve>(pathV[1]);
            const SmVector3d e = curve_end(c, true), s = curve_end(c, false);
            const double de = std::min(len(sub(e, curve_end(next, false))), len(sub(e, curve_end(next, true))));
            const double ds = std::min(len(sub(s, curve_end(next, false))), len(sub(s, curve_end(next, true))));
            back = ds < de;
        } else if (i > 0) {
            const SmVector3d last = curve_end(path.back(), true);
            const double ds = len(sub(curve_end(c, false), last)), de = len(sub(curve_end(c, true), last));
            if (std::min(ds, de) > tol) throw std::runtime_error("sweepPath: the path's curves don't join end to end");
            back = de < ds;
        }
        SmCurve* k = copy_curve(c, back);
        owned.push_back(k); path.push_back(k);
    }
    // run it from the end the profile is at
    if (len(sub(curve_end(path.back(), true), mid)) < len(sub(curve_end(path.front(), false), mid)) - tol) {
        std::reverse(path.begin(), path.end());
        for (auto& k : path) {
            SmExtent1d now;
            if (k->ReverseParameterization(k->GetNaturalInterval(), now) != SM_SUCCESS) throw std::runtime_error("sweepPath: reversing a curve failed");
        }
    }
    const bool closed = n > 1 && len(sub(curve_end(path.back(), true), curve_end(path.front(), false))) <= tol;

    SmBrep* result = nullptr;
    const auto join = [&](SmBrep* b) {
        if (!result) { result = b; return; }
        SmBrep* r = nullptr;
        // the union keeps `result` and deletes b
        auto it = std::find(breps.begin(), breps.end(), b);
        if (it != breps.end()) breps.erase(it);
        const SmStatus st = SmApiBooleanUnion(result, b, r);
        if (st != SM_SUCCESS) { breps.push_back(b); wl::check(st, "SmApiBooleanUnion"); }
    };
    /** pipe `prof` run straight on along d from the corner, cut at the mitre plane (keeping `side` of it) */
    const auto mitre = [&](const std::vector<SmCurve*>& p, const SmVector3d& d, const SmVector3d& C, const SmVector3d& m, int side, double L) {
        SmTArray<SmCurve*> cs;
        for (auto* c : p) cs.Add(c);
        SmBrep* ext = nullptr;
        wl::check(SmApiCreateLinearSweep(cs, d, L, TRUE, ext), "SmApiCreateLinearSweep");
        breps.push_back(ext);
        SmBrep* h = half_space(C, m, side, 4 * (L + reach));
        SmBrep* r = nullptr;
        const SmStatus st = SmApiBooleanIntersection(ext, h, r);
        if (st != SM_SUCCESS) { delete h; wl::check(st, "SmApiBooleanIntersection"); }
        return ext;
    };

    // the corners: corner k is where piece k ends and the next begins
    struct Corner { bool sharp = false; SmVector3d C, t1, t2, bis; double a = 0; };
    std::vector<Corner> corners(n);
    const double cornerCos = std::cos(cornerDeg * M_PI / 180.0);
    for (unsigned k = 0; k < n; ++k) {
        if (k + 1 == n && !closed) break;
        Corner& q = corners[k];
        q.C = curve_end(path[k], true, &q.t1);
        curve_end(path[(k + 1) % n], false, &q.t2);
        const double c12 = std::max(-1.0, std::min(1.0, dot(q.t1, q.t2)));
        if (c12 < -0.999) throw std::runtime_error("sweepPath: the path turns right back on itself");
        // a smooth join (or one within cornerDeg of it): the profile carries straight on
        q.sharp = c12 < cornerCos;
        q.a = std::acos(c12);
        q.bis = unit(add(q.t1, q.t2));
    }
    /** how far a pipe runs on past a corner for its mitre */
    const auto runOn = [&](const Corner& q) { return reach * (std::tan(q.a / 2) + 1) + tol; };
    /** b cut back to `side` of the mitre plane at corner q (b reaching at most `span` from the corner) */
    const auto trim = [&](SmBrep* b, const Corner& q, int side, double span) {
        SmBrep* h = half_space(q.C, q.bis, side, span);
        SmBrep* r = nullptr;
        const SmStatus st = SmApiBooleanIntersection(b, h, r);
        if (st != SM_SUCCESS) { delete h; wl::check(st, "SmApiBooleanIntersection"); }
    };

    std::vector<SmCurve*> cur = prof;   // the profile where this piece starts
    const auto moved = [&](const std::vector<SmCurve*>& p, const SmAxis2Placement& m) {
        std::vector<SmCurve*> out;
        for (auto* c : p) {
            SmCurve* k = copy_curve(c, false);
            owned.push_back(k);
            wl::check(SmApiTransform(k, m), "SmApiTransform");
            out.push_back(k);
        }
        return out;
    };
    for (unsigned i = 0; i < n; ++i) {
        const SmCurve* c = path[i];
        const bool last = i + 1 == n && !closed;
        SmTArray<SmCurve*> cs;
        for (auto* k : cur) cs.Add(k);
        SmVector3d S, tS, E, tE;
        S = curve_end(c, false, &tS);
        E = curve_end(c, true, &tE);
        const double size = std::max(1.0, len(sub(E, S)) + reach);
        SmPoint3d lp;
        SmVector3d lv;
        SmAxis2Placement pl;
        double rad = 0, a0 = 0, a1 = 0;
        SmBrep* piece = nullptr;
        std::vector<SmCurve*> endProf;
        if (c->GetType() == SmLine_TYPE || c->IsLine(12, 1e-9 * size, lp, lv)) {
            // a straight piece: the profile pushed along it, as it lies
            const SmVector3d d = sub(E, S);
            wl::check(SmApiCreateLinearSweep(cs, unit(d), len(d), TRUE, piece), "SmApiCreateLinearSweep");
            breps.push_back(piece);
            // at a corner, cut back to the mitre plane: inside the corner it reaches past it, and the next pipe holds
            // that side (for a profile that's lopsided across the turn, it would poke out of it)
            const Corner* before = i > 0 ? &corners[i - 1] : closed ? &corners[n - 1] : nullptr;
            if (!last && corners[i].sharp) trim(piece, corners[i], -1, 2 * (len(d) + runOn(corners[i]) + reach));
            if (before && before->sharp) trim(piece, *before, +1, 2 * (len(d) + runOn(*before) + reach));
            SmAxis2Placement m;
            m.SetCanonical(d, SmVector3d(1, 0, 0), SmVector3d(0, 1, 0));
            if (!last) endProf = moved(cur, m);
        } else if (c->IsArc(24, 1e-9 * size, pl, rad, a0, a1) && rad > 0 && std::isfinite(rad)) {
            // round an arc: the profile turned about its axis
            const SmVector3d O = pl.GetOrigin();
            SmVector3d z = unit(pl.GetZAxis());
            if (dot(cross(z, sub(S, O)), tS) < 0) z = scl(z, -1);
            const SmVector3d u = unit(sub(S, O)), v = cross(z, u), w = sub(E, O);
            double a = std::atan2(dot(w, v), dot(w, u));
            if (a <= 1e-9) a += 2 * M_PI;
            if (len(sub(E, S)) <= tol) a = 2 * M_PI;   // a whole circle
            wl::check(SmApiCreateRotationalSweep(cs, O, z, a * 180.0 / M_PI, TRUE, piece), "SmApiCreateRotationalSweep");
            breps.push_back(piece);
            if (!last) endProf = moved(cur, turn_about(O, z, a));
        } else {
            // any other curve: SMLib's sweep surface, the profile kept where it lies (sweep_along)
            SmBSplineCurve* seg = dynamic_cast<SmBSplineCurve*>(path[i]);
            if (!seg) {
                SmCurve* nurb = nullptr;
                wl::check(path[i]->CopyAnalyticAsNurb(*ctx, nurb), "SmCurve::CopyAnalyticAsNurb");
                owned.push_back(nurb);
                seg = dynamic_cast<SmBSplineCurve*>(nurb);
                if (!seg) throw std::runtime_error("sweepPath: a path curve has no B-spline form");
            }
            std::vector<SmCurve*> e;
            piece = sweep_along(cur, seg, e);
            breps.push_back(piece);
            owned.insert(owned.end(), e.begin(), e.end());
            if (!last) endProf = e;
        }
        join(piece);
        if (last) break;
        const Corner& q = corners[i];
        std::vector<SmCurve*> nextProf = endProf;
        if (q.sharp) {
            nextProf = moved(endProf, turn_about(q.C, unit(cross(q.t1, q.t2)), q.a));
            // the mitre on the outside of the corner: each pipe run on to the plane halfway between the directions
            join(mitre(endProf, q.t1, q.C, q.bis, -1, runOn(q)));
            join(mitre(nextProf, scl(q.t2, -1), q.C, q.bis, +1, runOn(q)));
        }
        if (i + 1 == n) break;   // closed: the last corner is done
        cur = nextProf;
    }
    // the result is the caller's
    auto it = std::find(breps.begin(), breps.end(), result);
    if (it != breps.end()) breps.erase(it);
    return wl::handle(result);
    WL_END("sweepPath")
}

EMSCRIPTEN_BINDINGS(wasmlib_build) {
    function("brepFromFaces", &brepFromFaces);
    function("sweepPath", &sweepPath);
    function("brepBox", &brepBox);
    function("screwSweep", &screwSweep);
    function("mergeDisjoint", &mergeDisjoint);
}
