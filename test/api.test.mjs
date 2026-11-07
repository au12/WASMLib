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

// files
const back = W.readBrep(W.writeBrep(cut, false), false);
check(".smb round trip", near(vol(back), vol(cut)));
const step = W.stepWrite([{ brep: cut, name: "plate", color: [1, 0.5, 0] }], "plate");
check("STEP text", step.startsWith("ISO-10303-21;") && step.includes("MANIFOLD_SOLID_BREP('plate'") && step.includes("CYLINDRICAL_SURFACE") && step.includes("COLOUR_RGB"));

// errors name the call and the status
try { W.SmApiCreateBox([0, 0, 0], -1, 1, 1); check("a bad call throws", false); }
catch (e) { check("a bad call throws with the status", /SmApiCreateBox failed: SM_ERR_INVALID_INPUT/.test(e.message), e.message); }

for (const p of [box, cyl, cut, fb, back, c]) W.destroy(p);
console.log(`${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
