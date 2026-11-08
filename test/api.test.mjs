// WASMLib's own checks of its JavaScript API: run with `node test/api.test.mjs` after building.
import createWASMLib from "../dist/wasmlib.mjs";

const W = await createWASMLib();
W.init();
let pass = 0, fail = 0;
const check = (name, ok, detail = "") => { if (ok) pass++; else { fail++; console.log(`FAIL ${name} ${detail}`); } };
const near = (a, b, tol = 1e-6) => Math.abs(a - b) <= tol * Math.max(1, Math.abs(b));
const vol = (b) => W.SmApiBrepComputeVolume(b, 1e-8);

// primitives and queries
const box = W.SmApiCreateBox([0, 0, 0], 10, 20, 30);
check("box volume", near(vol(box), 6000));
const t = W.brepTopology(box);
check("box topology", t.faces.length === 6 && t.edges.length === 12 && t.vertices.length === 8);
check("box is a manifold solid", !!W.SmApiBrepIsManifoldSolid(box) && W.SmApiBrepMaterialCensus(box).rlSolidCount === 1);
const bb = W.SmApiBrepBoundingBox(box, 1);
check("tight bounds", near(bb.rMax[0], 10) && near(bb.rMax[1], 20) && near(bb.rMax[2], 30));
for (const f of t.faces) {
  const { normal, point } = W.faceNormal(f, null);
  // outward: the point plus a little along the normal is outside the box
  const q = point.map((x, i) => x + normal[i] * 1e-3);
  const inside = q[0] > 0 && q[0] < 10 && q[1] > 0 && q[1] < 20 && q[2] > 0 && q[2] < 30;
  check("box face normals point out", !inside, JSON.stringify({ normal, point }));
}

// a boolean, with SM_API's ownership: the result is the first operand; the second is gone
const cyl = W.SmApiCreateCylinder([5, 10, -1], 3, 32);
const a = W.SmApiBrepCopy(box), b = W.SmApiBrepCopy(cyl);
const cut = W.SmApiBooleanDifference(a, b);
check("difference keeps the first operand", cut === a);
check("difference volume", near(vol(cut), 6000 - Math.PI * 9 * 30, 1e-6));
const hole = W.brepFaces(cut).map((f) => W.faceInfo(f)).find((i) => i.kind === "cylinder");
check("the hole is a cylinder face, normal into the hole", hole && hole.sense === -1 && near(hole.radius, 3));

// fillets on a copy
const { brep: fb, map } = W.copyMapped(box, W.brepEdges(box).slice(0, 4));
W.SmApiFilletEdges(fb, map, 2, 1, 1, 1);
check("fillet four edges", W.brepFaces(fb).length === 10 && vol(fb) < 6000);

// meshes: closed, outward
const m = W.mesh(cut, { chordHeightTol: 0.01, curveAngleTolDeg: 10, surfaceAngleTolDeg: 10 });
let v6 = 0;
const V = m.vertices, T = m.triangles;
for (let i = 0; i < T.length; i += 3) {
  const p = 3 * T[i], q = 3 * T[i + 1], r = 3 * T[i + 2];
  v6 += V[p] * (V[q + 1] * V[r + 2] - V[q + 2] * V[r + 1]) - V[p + 1] * (V[q] * V[r + 2] - V[q + 2] * V[r]) + V[p + 2] * (V[q] * V[r + 1] - V[q + 1] * V[r]);
}
check("mesh volume is close and positive (wound outward)", near(v6 / 6, vol(cut), 5e-3), String(v6 / 6));
check("mesh groups: one per face", m.groups.length / 2 === W.brepFaces(cut).length && m.failures.length === 0);
const el = W.edgeLines(cut, 10);
check("edge lines: one group per edge", el.groups.length / 2 === W.brepEdges(cut).length);

// curves
const c = W.SmApiCreateCircle_Circle([0, 0, 0], 2);
const ci = W.curveInfo(c);
check("circle info", ci.kind === "circle" && near(ci.radius, 2) && near(ci.length, 4 * Math.PI, 1e-4));
const s = W.curveSample(c, [0.25], false);
check("circle sampled by length", near(Math.hypot(s[0], s[1]), 2) && near(Math.atan2(s[1], s[0]), Math.PI / 2, 1e-3));

// building: a screw sweep — a disc 2 off the axis, turned twice round it while rising 5: a closed solid whose volume
// is the disc's area × 5, given back exactly, and the integrated volume and the mesh agree with it
const meshVol = (b) => {
  const mm = W.mesh(b, { chordHeightTol: 0.002, curveAngleTolDeg: 5, surfaceAngleTolDeg: 5 });
  let v = 0;
  const X = mm.vertices, I = mm.triangles;
  for (let i = 0; i < I.length; i += 3) {
    const p = 3 * I[i], q = 3 * I[i + 1], r = 3 * I[i + 2];
    v += X[p] * (X[q + 1] * X[r + 2] - X[q + 2] * X[r + 1]) - X[p + 1] * (X[q] * X[r + 2] - X[q + 2] * X[r]) + X[p + 2] * (X[q] * X[r + 1] - X[q + 1] * X[r]);
  }
  return v / 6;
};
const disc = W.SmApiCreateCircle_Circle([2, 0, 0], 1);
const sw = W.screwSweep([disc], [0, 0, 0], [0, 0, 1], [0, 0, 5], 720, 1e-4);
check("screw sweep: a closed solid", !!W.SmApiBrepIsManifoldSolid(sw.brep) && W.SmApiBrepMaterialCensus(sw.brep).rlSolidCount === 1 && sw.glued);
check("screw sweep: its exact volume", near(sw.volume, Math.PI * 5, 1e-9), String(sw.volume));
check("screw sweep: integrated volume agrees", near(W.SmApiBrepComputeVolume(sw.brep, 1e-4), Math.PI * 5, 1e-4));
check("screw sweep: mesh volume agrees", near(meshVol(sw.brep), Math.PI * 5, 5e-3), String(meshVol(sw.brep)));
const swBox = W.brepBox(sw.brep);
check("screw sweep: it stays within the turning disc", near(swBox[1][0], 3, 1e-3) && near(swBox[0][0], -3, 1e-3) && near(swBox[0][2], 0) && near(swBox[1][2], 5), JSON.stringify(swBox));
W.destroy(disc);

// building: Breps side by side, and a tight box round a turned cylinder (its control net bulges up to two fifths of r)
const moved = W.SmApiBrepCopy(box);
W.SmApiTranslate(moved, [20, 0, 0]);
const twins = W.mergeDisjoint([box, moved]);
check("merge disjoint: two solids, both volumes", W.SmApiBrepMaterialCensus(twins).rlSolidCount === 2 && near(vol(twins), 12000));
const tilted = W.SmApiCreateCylinder([0, 0, 0], 2, 10);
W.SmApiRotate(tilted, [0, 0, 0], [1, 0, 0], 90);
const tb = W.brepBox(tilted);
check("brepBox: tight on a turned cylinder", near(tb[0][0], -2, 1e-6) && near(tb[1][0], 2, 1e-6) && near(tb[0][2], -2, 1e-6) && near(tb[1][2], 2, 1e-6), JSON.stringify(tb));
for (const p of [sw.brep, moved, twins, tilted]) W.destroy(p);

// files
const back = W.readBrep(W.writeBrep(cut, false), false);
check(".smb round trip", near(vol(back), vol(cut)));
const step = W.stepWrite([{ brep: cut, name: "plate", color: [1, 0.5, 0] }], "plate");
check("STEP text", step.startsWith("ISO-10303-21;") && step.includes("MANIFOLD_SOLID_BREP('plate'") && step.includes("CYLINDRICAL_SURFACE") && step.includes("COLOUR_RGB"));
// a hole's wall is in the same solid as the rest (its upward faceuse is on the material side: grouping by it split them)
check("STEP: a plate with a hole is one solid", (step.match(/MANIFOLD_SOLID_BREP/g) ?? []).length === 1, step.match(/MANIFOLD_SOLID_BREP[^;]*/g)?.join(" | "));
const hollow = W.SmApiBooleanDifference(W.SmApiCreateBox([0, 0, 0], 20, 20, 20), W.SmApiCreateBox([5, 5, 5], 10, 10, 10));
const hstep = W.stepWrite([{ brep: hollow, name: "hollow" }], "hollow");
check("STEP: a hollow is a solid with a void", /BREP_WITH_VOIDS\('hollow',#\d+,\(#\d+\)\)/.test(hstep) && /ORIENTED_CLOSED_SHELL\('',\*,#\d+,\.F\.\)/.test(hstep) && !hstep.includes("MANIFOLD_SOLID_BREP"));

// a solid from faces: a 10 mm cube, each face a flat NURBS patch with four lines round it
const line = (a, b) => ({ p: 1, U: [0, 0, 1, 1], P: [...a, ...b], W: null });
const at = (o, du, dv, i, j) => o.map((x, k) => 10 * (x + i * du[k] + j * dv[k]));
const quad = (o, du, dv) => {
  const c = [at(o, du, dv, 0, 0), at(o, du, dv, 1, 0), at(o, du, dv, 1, 1), at(o, du, dv, 0, 1)];
  return { surface: { p: 1, q: 1, U: [0, 0, 1, 1], V: [0, 0, 1, 1], nu: 2, nv: 2, P: [...c[0], ...c[3], ...c[1], ...c[2]], W: null }, reversed: false,
    loops: [{ curves: [0, 1, 2, 3].map((k) => line(c[k], c[(k + 1) % 4])), senses: [1, 1, 1, 1] }] };
};
const cubeFaces = [quad([0, 0, 0], [0, 1, 0], [1, 0, 0]), quad([0, 0, 1], [1, 0, 0], [0, 1, 0]), quad([0, 0, 0], [1, 0, 0], [0, 0, 1]),
  quad([0, 1, 0], [0, 0, 1], [1, 0, 0]), quad([0, 0, 0], [0, 0, 1], [0, 1, 0]), quad([1, 0, 0], [0, 1, 0], [0, 0, 1])];
const fromFaces = W.brepFromFaces(cubeFaces, 1e-6);
check("brepFromFaces: a closed solid", fromFaces.solid && fromFaces.faces === 6 && fromFaces.failed.length === 0 && !!W.SmApiBrepIsManifoldSolid(fromFaces.brep));
check("brepFromFaces: its volume", near(vol(fromFaces.brep), 1000, 1e-6), String(vol(fromFaces.brep)));
check("brepFromFaces: faces stitched (12 edges)", W.brepTopology(fromFaces.brep).edges.length === 12);
try { W.brepFromFaces([{ ...cubeFaces[0], surface: { ...cubeFaces[0].surface, U: [-1, 0, 1, 2] } }], 1e-6); check("brepFromFaces: unclamped knots refused", false); }
catch (e) { check("brepFromFaces: unclamped knots refused, saying so", /aren't clamped/.test(e.message), e.message); }
W.destroy(hollow); W.destroy(fromFaces.brep);

// a path sweep: a Ø4 circle along two lines meeting square (mitred), and a line, a round bend and a line
const ring = W.SmApiCreateCircle_Circle([0, 0, 0], 2);
const seg = (a, b) => W.SmApiCreateLineSegment(a, b);
const ell = [seg([0, 0, 0], [0, 0, 20]), seg([0, 0, 20], [20, 0, 20])];
const swept = W.sweepPath([ring], ell, 1e-5, 0.5);
check("sweepPath: a solid round a mitred corner", !!W.SmApiBrepIsManifoldSolid(swept) && W.SmApiBrepMaterialCensus(swept).rlSolidCount === 1);
check("sweepPath: its volume (area × path)", near(vol(swept), 4 * Math.PI * 40, 1e-6), String(vol(swept)));
check("sweepPath: the profile stays where it was drawn", near(W.SmApiBrepBoundingBox(swept, 1).rMin[0], -2, 1e-6) && near(W.SmApiBrepBoundingBox(swept, 1).rMax[2], 22, 1e-6));
const back2 = [seg([20, 0, 20], [0, 0, 20]), seg([0, 0, 20], [0, 0, 0])];   // given the other way round
const swept2 = W.sweepPath([ring], back2, 1e-5, 0.5);
check("sweepPath: a path given the other way round runs from the profile", near(vol(swept2), 4 * Math.PI * 40, 1e-6), String(vol(swept2)));
for (const x of [ring, ...ell, ...back2, swept, swept2]) W.destroy(x);

// errors name the call and the status
try { W.SmApiCreateBox([0, 0, 0], -1, 1, 1); check("a bad call throws", false); }
catch (e) { check("a bad call throws with the status", /SmApiCreateBox failed: SM_ERR_INVALID_INPUT/.test(e.message), e.message); }

for (const p of [box, cyl, cut, fb, back, c]) W.destroy(p);
console.log(`${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
