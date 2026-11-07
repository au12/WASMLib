// A STEP writer (ISO 10303-21, AP214 "automotive_design"): SMLib has none of its own — usd-brep exchanges B-reps through
// USD and OpenCascade — so this writes SMLib's Breps out directly. Planes, cylinders, cones, spheres and tori are
// written as such; lines, circles and ellipses too; everything else as (rational) B-splines from SMLib's own control
// nets. A Brep with geometry that has no B-spline form of its own (offset or blend surfaces, curves on surfaces) is
// written from a copy turned to NURBS first.
#include "wl.h"
#include "geom.h"

#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>
#include <SmCircle.h>
#include <SmCone.h>
#include <SmCylinder.h>
#include <SmEllipse.h>
#include <SmLine.h>
#include <SmPlane.h>
#include <SmSphere.h>
#include <SmTorus.h>

#include <cstdio>
#include <ctime>
#include <string>
#include <unordered_map>
#include <vector>

using namespace wlg;

namespace {

/** a STEP real: always with a decimal point */
std::string R(double v) {
    if (!std::isfinite(v)) v = 0;
    if (std::fabs(v) < 1e-300) return "0.";
    char b[40];
    std::snprintf(b, sizeof b, "%.15G", v);
    std::string s(b);
    const auto e = s.find('E');
    if (s.find('.') == std::string::npos) s.insert(e == std::string::npos ? s.size() : e, ".");
    return s;
}
std::string S(const std::string& in) {
    std::string o;
    for (unsigned char c : in) {
        if (c == '\'') o += "''";
        else if (c == '\\') o += "\\\\";
        else if (c >= 32 && c < 127) o += char(c);
        else o += '_';
    }
    return "'" + o + "'";
}
std::string ref(int i) { return "#" + std::to_string(i); }
template <class F> std::string list(size_t n, F f) {
    std::string s = "(";
    for (size_t i = 0; i < n; ++i) { if (i) s += ","; s += f(i); }
    return s + ")";
}

/** surfaces and curves this writer can take as they are */
bool writable(const SmSurface* s) {
    switch (s->GetType()) {
        case SmPlane_TYPE: case SmCylinder_TYPE: case SmCone_TYPE: case SmSphere_TYPE: case SmTorus_TYPE: return true;
        default: return dynamic_cast<const SmBSplineSurface*>(s) != nullptr;
    }
}
bool writable(const SmCurve* c) { return c && dynamic_cast<const SmBSplineCurve*>(c) != nullptr; }

class Writer {
public:
    std::string data;
    int n = 0;
    int ctx = 0, mm = 0;

    int add(const std::string& e) {
        ++n;
        data += "#" + std::to_string(n) + "=" + e + ";\n";
        return n;
    }
    int point(const SmVector3d& p) { return add("CARTESIAN_POINT(''," + list(3, [&](size_t i) { return R(p[i]); }) + ")"); }
    int dir(const SmVector3d& d) { const SmVector3d u = unit(d); return add("DIRECTION(''," + list(3, [&](size_t i) { return R(u[i]); }) + ")"); }
    int axis(const SmVector3d& o, const SmVector3d& z, const SmVector3d& x) {
        const int p = point(o), zz = dir(z), xx = dir(x);
        return add("AXIS2_PLACEMENT_3D(''," + ref(p) + "," + ref(zz) + "," + ref(xx) + ")");
    }
    int axis(const SmAxis2Placement& pl) { return axis(pl.GetOrigin(), pl.GetZAxis(), pl.GetXAxis()); }

    void header() {
        // units and the geometric context
        mm = add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )");
        const int rad = add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )");
        const int sr = add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )");
        const int unc = add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07)," + ref(mm) + ",'distance_accuracy_value','confusion accuracy')");
        ctx = add("( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((" + ref(unc) + ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((" +
                  ref(mm) + "," + ref(rad) + "," + ref(sr) + ")) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )");
    }

    // ---------------------------------------------------------------- geometry
    /** the surface under a face; `same` says whether its natural normal points out of the material */
    int surface(const SmFace* f, bool& same) {
        const SmSurface* s = f->GetSurface();
        SmVector3d q;
        wl::check(SmApiFaceInternalPoint(f, q), "SmApiFaceInternalPoint");
        const SmVector3d out = face_normal_near(f, q);
        SmVector3d nat;  // the written surface's own normal at q
        int id = 0;
        SmAxis2Placement pl;
        double a = 0, b = 0;
        const auto radial = [](const SmVector3d& p, const SmVector3d& o, const SmVector3d& z) {
            SmVector3d d(p.x - o.x, p.y - o.y, p.z - o.z);
            const double k = dot(d, z);
            return unit(SmVector3d(d.x - z.x * k, d.y - z.y * k, d.z - z.z * k));
        };
        switch (s->GetType()) {
            case SmPlane_TYPE:
                wl::check(((const SmPlane*)s)->GetCanonical(pl), "SmPlane::GetCanonical");
                id = add("PLANE(''," + ref(axis(pl)) + ")");
                nat = unit(pl.GetZAxis());
                break;
            case SmCylinder_TYPE: {
                wl::check(((const SmCylinder*)s)->GetCanonical(pl, a), "SmCylinder::GetCanonical");
                id = add("CYLINDRICAL_SURFACE(''," + ref(axis(pl)) + "," + R(a) + ")");
                nat = radial(q, pl.GetOrigin(), unit(pl.GetZAxis()));
                break;
            }
            case SmCone_TYPE: {
                wl::check(((const SmCone*)s)->GetCanonical(pl, a, b), "SmCone::GetCanonical");
                SmVector3d z = unit(pl.GetZAxis()), x = unit(pl.GetXAxis());
                if (std::fabs(b) < 1e-12) {  // a cone of zero angle is a cylinder
                    id = add("CYLINDRICAL_SURFACE(''," + ref(axis(pl.GetOrigin(), z, x)) + "," + R(a) + ")");
                    nat = radial(q, pl.GetOrigin(), z);
                    break;
                }
                // STEP's semi-angle is positive: a cone narrowing along its axis is written with the axis turned round
                if (b < 0) { z = SmVector3d(-z.x, -z.y, -z.z); b = -b; }
                const double t = b * M_PI / 180;
                id = add("CONICAL_SURFACE(''," + ref(axis(pl.GetOrigin(), z, x)) + "," + R(a) + "," + R(t) + ")");
                const SmVector3d r = radial(q, pl.GetOrigin(), z);
                nat = unit(SmVector3d(r.x * std::cos(t) - z.x * std::sin(t), r.y * std::cos(t) - z.y * std::sin(t), r.z * std::cos(t) - z.z * std::sin(t)));
                break;
            }
            case SmSphere_TYPE: {
                wl::check(((const SmSphere*)s)->GetCanonical(pl, a), "SmSphere::GetCanonical");
                id = add("SPHERICAL_SURFACE(''," + ref(axis(pl)) + "," + R(a) + ")");
                const SmVector3d o = pl.GetOrigin();
                nat = unit(SmVector3d(q.x - o.x, q.y - o.y, q.z - o.z));
                break;
            }
            case SmTorus_TYPE: {
                wl::check(((const SmTorus*)s)->GetCanonical(pl, a, b), "SmTorus::GetCanonical");
                id = add("TOROIDAL_SURFACE(''," + ref(axis(pl)) + "," + R(a) + "," + R(b) + ")");
                const SmVector3d o = pl.GetOrigin(), r = radial(q, o, unit(pl.GetZAxis()));
                const SmVector3d c(o.x + r.x * a, o.y + r.y * a, o.z + r.z * a);
                nat = unit(SmVector3d(q.x - c.x, q.y - c.y, q.z - c.z));
                break;
            }
            default: {
                const SmBSplineSurface* bs = dynamic_cast<const SmBSplineSurface*>(s);
                if (!bs) throw std::runtime_error("STEP: a surface with no B-spline form");
                id = spline(bs);
                // its own normal at q: the u and v derivatives' cross product
                SmPoint3d cp;
                SmTArray<SmPoint2d> uvs;
                double d = 0;
                wl::check(SmApiSurfaceClosestPoint(s, q, cp, uvs, d), "SmApiSurfaceClosestPoint");
                SmVector3d p, du, dv;
                wl::check(SmApiEvaluateSurfaceDerivatives(s, uvs[0], p, du, dv), "SmApiEvaluateSurfaceDerivatives");
                nat = unit(cross(du, dv));
            }
        }
        same = dot(nat, out) > 0;
        return id;
    }

    int spline(const SmBSplineSurface* s) {
        ULONG nu = 0, nv = 0;
        SmTArray<SmPoint3d> cps;
        SmTArray<double> w;
        wl::check(s->GetControlPointNet(nu, nv, cps, w), "SmBSplineSurface::GetControlPointNet");
        SmTArray<double> ku, kv;
        SmTArray<ULONG> mu, mv;
        wl::check(s->GetKnots(SM_SP_U, ku, &mu), "SmBSplineSurface::GetKnots");
        wl::check(s->GetKnots(SM_SP_V, kv, &mv), "SmBSplineSurface::GetKnots");
        const ULONG du = s->GetDegree(SM_SP_U), dv = s->GetDegree(SM_SP_V);
        // which way the net is laid out: compare the (umax, vmin) corner with the candidates
        bool uFast = true;
        if (nu > 1 && nv > 1) {
            SmVector3d corner;
            wl::check(SmApiEvaluateSurfacePoint(s, SmPoint2d(ku[ku.GetSize() - 1], kv[0]), corner), "SmApiEvaluateSurfacePoint");
            const SmVector3d a = cps[nu - 1], b = cps[(nu - 1) * nv];
            uFast = len(SmVector3d(a.x - corner.x, a.y - corner.y, a.z - corner.z)) <= len(SmVector3d(b.x - corner.x, b.y - corner.y, b.z - corner.z));
        }
        const auto at = [&](ULONG i, ULONG j) { return uFast ? j * nu + i : i * nv + j; };
        std::vector<int> pts(nu * nv);
        for (ULONG i = 0; i < nu; ++i)
            for (ULONG j = 0; j < nv; ++j) pts[i * nv + j] = point(cps[at(i, j)]);
        const std::string net = list(nu, [&](size_t i) { return list(nv, [&](size_t j) { return ref(pts[i * nv + j]); }); });
        const std::string knots = list(mu.GetSize(), [&](size_t i) { return std::to_string(mu[i]); }) + "," +
                                  list(mv.GetSize(), [&](size_t i) { return std::to_string(mv[i]); }) + "," +
                                  list(ku.GetSize(), [&](size_t i) { return R(ku[i]); }) + "," +
                                  list(kv.GetSize(), [&](size_t i) { return R(kv[i]); });
        if (!w.GetSize())
            return add("B_SPLINE_SURFACE_WITH_KNOTS(''," + std::to_string(du) + "," + std::to_string(dv) + "," + net +
                       ",.UNSPECIFIED.,.F.,.F.,.F.," + knots + ",.UNSPECIFIED.)");
        const std::string ws = list(nu, [&](size_t i) { return list(nv, [&](size_t j) { return R(w[at(i, j)]); }); });
        return add("( BOUNDED_SURFACE() B_SPLINE_SURFACE(" + std::to_string(du) + "," + std::to_string(dv) + "," + net +
                   ",.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS(" + knots +
                   ",.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(" + ws +
                   ") REPRESENTATION_ITEM('') SURFACE() )");
    }

    int spline(const SmBSplineCurve* c) {
        const ULONG n = c->GetNumberControlPoints(), deg = c->GetDegree();
        std::vector<double> cp(4 * n);
        wl::check(c->GetControlPointsExpert(SM_CP_EUCLIDIAN_RATIONAL, 0, n - 1, 4, cp.data()), "SmBSplineCurve::GetControlPointsExpert");
        SmTArray<double> k;
        SmTArray<ULONG> m;
        wl::check(c->GetKnots(k, &m), "SmBSplineCurve::GetKnots");
        std::vector<int> pts(n);
        for (ULONG i = 0; i < n; ++i) pts[i] = point(SmVector3d(cp[4 * i], cp[4 * i + 1], cp[4 * i + 2]));
        const std::string ps = list(n, [&](size_t i) { return ref(pts[i]); });
        const std::string knots = list(m.GetSize(), [&](size_t i) { return std::to_string(m[i]); }) + "," +
                                  list(k.GetSize(), [&](size_t i) { return R(k[i]); });
        if (!c->IsRational())
            return add("B_SPLINE_CURVE_WITH_KNOTS(''," + std::to_string(deg) + "," + ps + ",.UNSPECIFIED.,.F.,.F.," + knots + ",.UNSPECIFIED.)");
        return add("( BOUNDED_CURVE() B_SPLINE_CURVE(" + std::to_string(deg) + "," + ps + ",.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS(" +
                   knots + ",.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(" +
                   list(n, [&](size_t i) { return R(cp[4 * i + 3]); }) + ") REPRESENTATION_ITEM('') )");
    }

    /** an edge's curve; `same`: the curve runs the way the edge does */
    int curve(const SmEdge* e, const EdgeRun& r, bool& same) {
        const SmCurve* c = r.c;
        // the edge's direction, halfway along
        const double tm = 0.5 * (r.t0 + r.t1);
        SmVector3d d1;
        const SmVector3d pm = at(c, tm, &d1);
        const double sgn = r.t1 >= r.t0 ? 1.0 : -1.0;
        const SmVector3d te(d1.x * sgn, d1.y * sgn, d1.z * sgn);
        SmAxis2Placement pl;
        double a = 0, b = 0;
        if (c->GetType() == SmLine_TYPE) {
            SmVector3d d(r.b.x - r.a.x, r.b.y - r.a.y, r.b.z - r.a.z);
            const double L = len(d);
            if (L > 1e-12) {
                same = true;
                const int v = add("VECTOR(''," + ref(dir(d)) + "," + R(L) + ")");
                return add("LINE(''," + ref(point(r.a)) + "," + ref(v) + ")");
            }
        } else if (c->GetType() == SmCircle_TYPE && ((const SmCircle*)c)->GetCanonical(pl, a) == SM_SUCCESS) {
            const SmVector3d o = pl.GetOrigin(), z = unit(pl.GetZAxis());
            same = dot(cross(z, SmVector3d(pm.x - o.x, pm.y - o.y, pm.z - o.z)), te) > 0;
            return add("CIRCLE(''," + ref(axis(pl)) + "," + R(a) + ")");
        } else if (c->GetType() == SmEllipse_TYPE && ((const SmEllipse*)c)->GetCanonical(pl, a, b) == SM_SUCCESS) {
            const SmVector3d o = pl.GetOrigin(), z = unit(pl.GetZAxis());
            same = dot(cross(z, SmVector3d(pm.x - o.x, pm.y - o.y, pm.z - o.z)), te) > 0;
            if (a >= b) return add("ELLIPSE(''," + ref(axis(pl)) + "," + R(a) + "," + R(b) + ")");
            // STEP wants the major radius first: turn the frame a quarter
            return add("ELLIPSE(''," + ref(axis(o, z, pl.GetYAxis())) + "," + R(b) + "," + R(a) + ")");
        }
        const SmBSplineCurve* bs = dynamic_cast<const SmBSplineCurve*>(c);
        if (!bs) throw std::runtime_error("STEP: a curve with no B-spline form");
        same = r.t1 >= r.t0;
        return spline(bs);
    }

    // ---------------------------------------------------------------- topology
    std::unordered_map<const SmVertex*, int> verts;
    std::unordered_map<const SmEdge*, int> edges;

    int vertex(const SmVertex* v, const SmVector3d& p) {
        auto it = verts.find(v);
        if (it != verts.end()) return it->second;
        const int id = add("VERTEX_POINT(''," + ref(point(v ? v->GetPoint() : p)) + ")");
        if (v) verts.emplace(v, id);
        return id;
    }
    /** an edge, written once; its EDGE_CURVE runs from its start vertex to its end vertex */
    int edge(const SmEdge* e, bool& runsWithCurveParam) {
        const EdgeRun r = edge_run(e);
        runsWithCurveParam = r.t1 >= r.t0;
        auto it = edges.find(e);
        if (it != edges.end()) return it->second;
        SmVertex *vs = nullptr, *ve = nullptr;
        e->GetVertices(vs, ve);
        bool same = true;
        const int c = curve(e, r, same);
        const int id = add("EDGE_CURVE(''," + ref(vertex(vs, r.a)) + "," + ref(vertex(ve, r.b)) + "," + ref(c) + "," + (same ? ".T." : ".F.") + ")");
        edges.emplace(e, id);
        return id;
    }
    int face(const SmFace* f) {
        bool same = true;
        const int s = surface(f, same);
        SmTArray<ULONG> counts;
        SmTArray<SmEdge*> es;
        SmTArray<SmOrientType> os;
        wl::check(SmApiFaceGetBoundaryLoops(const_cast<SmFace*>(f), counts, es, os), "SmApiFaceGetBoundaryLoops");
        // SMLib's loops run counter-clockwise round the surface's normal (the upward faceuse): round the face's outward
        // normal when its sense is +1, else the other way
        const bool loopsAgree = face_sense(f) > 0;
        std::vector<int> bounds;
        ULONG k = 0;
        for (ULONG l = 0; l < counts.GetSize(); ++l) {
            std::vector<int> oes;
            for (ULONG i = k; i < k + counts[l]; ++i) {
                bool withParam = true;
                const int ec = edge(es[i], withParam);
                // SM_OT_SAME follows the edge's parameter upwards; the EDGE_CURVE runs start vertex → end vertex
                const bool alongParam = os[i] != SM_OT_OPPOSITE;
                oes.push_back(add("ORIENTED_EDGE('',*,*," + ref(ec) + "," + (alongParam == withParam ? ".T." : ".F.") + ")"));
            }
            k += counts[l];
            const int loop = add("EDGE_LOOP(''," + list(oes.size(), [&](size_t i) { return ref(oes[i]); }) + ")");
            bounds.push_back(add(std::string(l == 0 ? "FACE_OUTER_BOUND" : "FACE_BOUND") + "(''," + ref(loop) + "," + (loopsAgree ? ".T." : ".F.") + ")"));
        }
        return add("ADVANCED_FACE(''," + list(bounds.size(), [&](size_t i) { return ref(bounds[i]); }) + "," + ref(s) + "," + (same ? ".T." : ".F.") + ")");
    }

    /** one Brep: a MANIFOLD_SOLID_BREP per closed shell (an open one: a SHELL_BASED_SURFACE_MODEL) */
    std::vector<int> body(const SmBrep* b, const std::string& name, bool solid) {
        SmTArray<SmFace*> all;
        b->GetFaces(all);
        std::vector<const SmShell*> order;
        std::unordered_map<const SmShell*, std::vector<int>> by;
        for (ULONG i = 0; i < all.GetSize(); ++i) {
            const SmFaceuse* fu = all[i]->GetUpwardFaceuse();
            const SmShell* sh = fu ? fu->GetShell() : nullptr;
            if (!by.count(sh)) order.push_back(sh);
            by[sh].push_back(face(all[i]));
        }
        std::vector<int> out;
        for (const SmShell* sh : order) {
            const auto& fs = by[sh];
            const std::string faces = list(fs.size(), [&](size_t i) { return ref(fs[i]); });
            if (solid) out.push_back(add("MANIFOLD_SOLID_BREP(" + S(name) + "," + ref(add("CLOSED_SHELL(''," + faces + ")")) + ")"));
            else out.push_back(add("SHELL_BASED_SURFACE_MODEL(" + S(name) + ",(" + ref(add("OPEN_SHELL(''," + faces + ")")) + "))"));
        }
        return out;
    }

    int colour(const std::vector<double>& rgb, int item) {
        const int c = add("COLOUR_RGB(''," + R(rgb[0]) + "," + R(rgb[1]) + "," + R(rgb[2]) + ")");
        const int fasc = add("FILL_AREA_STYLE_COLOUR(''," + ref(c) + ")");
        const int fas = add("FILL_AREA_STYLE('',(" + ref(fasc) + "))");
        const int ssfa = add("SURFACE_STYLE_FILL_AREA(" + ref(fas) + ")");
        const int sss = add("SURFACE_SIDE_STYLE('',(" + ref(ssfa) + "))");
        const int ssu = add("SURFACE_STYLE_USAGE(.BOTH.," + ref(sss) + ")");
        const int psa = add("PRESENTATION_STYLE_ASSIGNMENT((" + ref(ssu) + "))");
        return add("STYLED_ITEM('color',(" + ref(psa) + ")," + ref(item) + ")");
    }
};

/** every face's surface and every edge's curve can be written as it is */
bool all_writable(const SmBrep* b) {
    SmTArray<SmFace*> fs;
    SmTArray<SmEdge*> es;
    b->GetFaces(fs);
    b->GetEdges(es);
    for (ULONG i = 0; i < fs.GetSize(); ++i) if (!writable(fs[i]->GetSurface())) return false;
    for (ULONG i = 0; i < es.GetSize(); ++i) if (!writable(es[i]->GetCurve())) return false;
    return true;
}

/**
 * STEP text for these bodies: items = [{ brep, name, color?: [r, g, b] (0…1) }], one product named `product`, each
 * body a named solid (coloured when given).
 */
val stepWrite(const val& items, const std::string& product) {
    WL_BEGIN
    Writer w;
    w.header();
    std::vector<int> reps, styled;
    std::vector<SmBrep*> temps;
    struct Free { std::vector<SmBrep*>& t; ~Free() { for (auto* b : t) delete b; } } fr{temps};
    const unsigned n = items["length"].as<unsigned>();
    for (unsigned i = 0; i < n; ++i) {
        val it = items[i];
        SmBrep* b = wl::ptr<SmBrep>(it["brep"]);
        if (!all_writable(b)) {
            // geometry with no B-spline form of its own: write a NURBS copy
            SmBrep* c = nullptr;
            wl::check(SmApiBrepCopy(b, c), "SmApiBrepCopy");
            temps.push_back(c);
            wl::check(SmApiTurnToNurbs(c), "SmApiTurnToNurbs");
            b = c;
        }
        SmBoolean solid = FALSE;
        SmApiBrepIsManifoldSolid(b, solid);
        const std::string name = wl::present(it["name"]) ? it["name"].as<std::string>() : "body";
        const std::vector<int> bodies = w.body(b, name, solid != FALSE);
        reps.insert(reps.end(), bodies.begin(), bodies.end());
        val col = it["color"];
        if (wl::present(col)) {
            const std::vector<double> rgb = {col[0].as<double>(), col[1].as<double>(), col[2].as<double>()};
            for (int bd : bodies) styled.push_back(w.colour(rgb, bd));
        }
    }
    // the product structure round the shapes
    const int origin = w.axis(SmVector3d(0, 0, 0), SmVector3d(0, 0, 1), SmVector3d(1, 0, 0));
    std::vector<int> repItems = {origin};
    repItems.insert(repItems.end(), reps.begin(), reps.end());
    const int shapeRep = w.add("ADVANCED_BREP_SHAPE_REPRESENTATION(" + S(product) + "," +
                               list(repItems.size(), [&](size_t i) { return ref(repItems[i]); }) + "," + ref(w.ctx) + ")");
    const int appCtx = w.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
    w.add("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000," + ref(appCtx) + ")");
    const int prodCtx = w.add("PRODUCT_CONTEXT(''," + ref(appCtx) + ",'mechanical')");
    const int prod = w.add("PRODUCT(" + S(product) + "," + S(product) + ",'',(" + ref(prodCtx) + "))");
    w.add("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(" + ref(prod) + "))");
    const int form = w.add("PRODUCT_DEFINITION_FORMATION('',''," + ref(prod) + ")");
    const int pdCtx = w.add("PRODUCT_DEFINITION_CONTEXT('part definition'," + ref(appCtx) + ",'design')");
    const int pd = w.add("PRODUCT_DEFINITION('design',''," + ref(form) + "," + ref(pdCtx) + ")");
    const int pds = w.add("PRODUCT_DEFINITION_SHAPE('',''," + ref(pd) + ")");
    w.add("SHAPE_DEFINITION_REPRESENTATION(" + ref(pds) + "," + ref(shapeRep) + ")");
    if (!styled.empty())
        w.add("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION(''," + list(styled.size(), [&](size_t i) { return ref(styled[i]); }) + "," + ref(w.ctx) + ")");

    char stamp[32];
    const std::time_t now = std::time(nullptr);
    std::strftime(stamp, sizeof stamp, "%Y-%m-%dT%H:%M:%S", std::gmtime(&now));
    std::string out = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('WASMLib STEP export'),'2;1');\n";
    out += "FILE_NAME(" + S(product + ".step") + ",'" + stamp + "',(''),(''),'WASMLib (SMLib)','WASMLib','');\n";
    out += "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n";
    out += w.data;
    out += "ENDSEC;\nEND-ISO-10303-21;\n";
    return val(out);
    WL_END("stepWrite")
}

}  // namespace

EMSCRIPTEN_BINDINGS(wasmlib_step) {
    function("stepWrite", &stepWrite);
}
