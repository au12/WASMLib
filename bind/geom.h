// Geometry helpers shared by the object layer and the STEP writer.
#pragma once
#include "wl.h"

#include <SmCurve.h>
#include <SmEdge.h>
#include <SmFace.h>
#include <SmFaceuse.h>
#include <SmRegion.h>
#include <SmShell.h>
#include <SmSurface.h>
#include <SmVertex.h>

#include <cmath>
#include <utility>

namespace wlg {

inline double dot(const SmVector3d& a, const SmVector3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double len(const SmVector3d& a) { return std::sqrt(dot(a, a)); }
inline SmVector3d unit(const SmVector3d& a, const SmVector3d& fallback = SmVector3d(0, 0, 1)) {
    const double l = len(a);
    return (!std::isfinite(l) || l <= 1e-300) ? fallback : SmVector3d(a.x / l, a.y / l, a.z / l);
}
inline SmVector3d cross(const SmVector3d& a, const SmVector3d& b) {
    return SmVector3d(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
inline SmVector3d perp(const SmVector3d& d) {
    return unit(cross(d, std::fabs(d.z) < 0.9 ? SmVector3d(0, 0, 1) : SmVector3d(1, 0, 0)));
}
// ---------------------------------------------------------------- faces: which way is out
/** +1 when the surface's normal points out of the material, -1 when it points in (SmPyLib's rule: the upward
 *  faceuse — the side the surface normal points to — belongs to the infinite region, or to a void) */
inline double face_sense(const SmFace* f) {
    const SmBrep* b = f->GetBrep();
    const SmFaceuse* up = f->GetUpwardFaceuse();
    if (!b || !up || !up->GetShell()) return 1.0;
    // the surface's normal points outward when it points into the infinite region — or into a void inside the solid
    const SmRegion* r = up->GetShell()->GetRegion();
    return r == b->GetInfiniteRegion() || (r && r->IsVoid() == TRUE) ? 1.0 : -1.0;
}
/** a point on the face: one inside it, or — for a sliver SMLib can't find a point inside (a triangle a tenth of a
 *  millimetre long) — the middle of its corners, which on a flat or nearly flat sliver lies on it */
inline SmVector3d face_point(const SmFace* f) {
    SmVector3d q;
    if (SmApiFaceInternalPoint(f, q) == SM_SUCCESS) return q;
    SmTArray<SmEdge*> es;
    f->GetEdges(es);
    SmVector3d s(0, 0, 0);
    int n = 0;
    for (ULONG i = 0; i < es.GetSize(); ++i) {
        SmVertex *a = nullptr, *b = nullptr;
        es[i]->GetVertices(a, b);
        for (const SmVertex* v : {a, b}) if (v) { const SmVector3d p = v->GetPoint(); s.x += p.x; s.y += p.y; s.z += p.z; n++; }
    }
    if (!n) wl::check(SM_ERR, "SmApiFaceInternalPoint");
    return SmVector3d(s.x / n, s.y / n, s.z / n);
}
/** the outward normal at the face point nearest p */
inline SmVector3d face_normal_near(const SmFace* f, const SmVector3d& p, SmVector3d* where = nullptr) {
    const SmSurface* s = f->GetSurface();
    SmPoint3d q;
    SmTArray<SmPoint2d> uvs;
    double d = 0;
    wl::check(SmApiSurfaceClosestPoint(s, p, q, uvs, d), "SmApiSurfaceClosestPoint");
    SmVector3d n;
    wl::check(SmApiEvaluateSurfaceNormal(s, uvs[0], n), "SmApiEvaluateSurfaceNormal");
    if (where) *where = q;
    n = unit(n);
    const double k = face_sense(f);
    return SmVector3d(n.x * k, n.y * k, n.z * k);
}

// ---------------------------------------------------------------- edges: arc length
/** an edge as a stretch of its curve: parameters run t0 → t1 from the start vertex to the end vertex */
struct EdgeRun {
    const SmCurve* c = nullptr;
    double t0 = 0, t1 = 1;
    SmVector3d a, b;
};
inline SmVector3d at(const SmCurve* c, double t, SmVector3d* d1 = nullptr) {
    SmVector3d pd[2];
    wl::check(c->Evaluate(t, d1 ? 1 : 0, TRUE, pd), "SmCurve::Evaluate");
    if (d1) *d1 = pd[1];
    return pd[0];
}
inline EdgeRun edge_run(const SmEdge* e) {
    EdgeRun r;
    r.c = e->GetCurve();
    const SmExtent1d iv = e->GetInterval();
    SmVertex *vs = nullptr, *ve = nullptr;
    e->GetVertices(vs, ve);
    r.a = vs ? vs->GetPoint() : at(r.c, iv.GetMin());
    r.b = ve ? ve->GetPoint() : at(r.c, iv.GetMax());
    r.t0 = iv.GetMin();
    r.t1 = iv.GetMax();
    // the edge may run against its curve
    const SmVector3d p0 = at(r.c, r.t0);
    SmVector3d da(p0.x - r.a.x, p0.y - r.a.y, p0.z - r.a.z), db(p0.x - r.b.x, p0.y - r.b.y, p0.z - r.b.z);
    if (vs != ve && len(db) < len(da)) std::swap(r.t0, r.t1);
    return r;
}

}  // namespace wlg
