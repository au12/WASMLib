import createWASMLib from "../dist/wasmlib.mjs";
const t0 = performance.now();
const W = await createWASMLib();
W.init();
console.log("load", (performance.now() - t0).toFixed(0), "ms");
const box = W.SmApiCreateBox([0, 0, 0], 10, 10, 10);
console.log("box", box, W.typeOf(box));
const topo = W.brepTopology(box);
console.log("faces", topo.faces.length, "edges", topo.edges.length, "verts", topo.vertices.length, "shells", topo.shells, "regions", topo.regions);
console.log("volume", W.SmApiBrepComputeVolume(box, 1e-6));
console.log("face0", W.faceInfo(topo.faces[0]), W.faceNormal(topo.faces[0], null));
console.log("edge0", W.edgeInfo(topo.edges[0]), Array.from(W.edgeSample(topo.edges[0], [0, 0.5, 1], true)));
const sph = W.SmApiCreateSphere([10, 10, 10], 4);
const a = W.SmApiBrepCopy(box), b = W.SmApiBrepCopy(sph);
const t1 = performance.now();
const diff = W.SmApiBooleanDifference(a, b);
console.log("diff", diff === a, (performance.now() - t1).toFixed(1), "ms; volume", W.SmApiBrepComputeVolume(diff, 1e-6), "expect", 1000 - (4 / 3 * Math.PI * 64) / 8);
const m = W.mesh(diff, { chordHeightTol: 0.01, curveAngleTolDeg: 15, surfaceAngleTolDeg: 15 });
let vol = 0; const V = m.vertices, T = m.triangles;
for (let i = 0; i < T.length; i += 3) { const a = 3 * T[i], b = 3 * T[i + 1], c = 3 * T[i + 2];
  vol += (V[a] * (V[b + 1] * V[c + 2] - V[b + 2] * V[c + 1]) - V[a + 1] * (V[b] * V[c + 2] - V[b + 2] * V[c]) + V[a + 2] * (V[b] * V[c + 1] - V[b + 1] * V[c])) / 6; }
console.log("mesh", V.length / 3, "verts", T.length / 3, "tris", "groups", m.groups.length / 2, "signed vol", vol.toFixed(2), "failures", m.failures.length);
// normals agree with winding?
let agree = 0, n = 0; const N = m.normals;
for (let i = 0; i < T.length; i += 3) { const a = 3 * T[i], b = 3 * T[i + 1], c = 3 * T[i + 2];
  const ux = V[b] - V[a], uy = V[b + 1] - V[a + 1], uz = V[b + 2] - V[a + 2], vx = V[c] - V[a], vy = V[c + 1] - V[a + 1], vz = V[c + 2] - V[a + 2];
  const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx; n++; if (nx * N[a] + ny * N[a + 1] + nz * N[a + 2] > 0) agree++; }
console.log("normals agree with winding", agree, "/", n);
const el = W.edgeLines(diff, 10); console.log("edge lines", el.lines.length / 6, "segments", el.groups.length / 2, "edges");
const bytes = W.writeBrep(diff, false); console.log("smb bytes", bytes.length);
const back = W.readBrep(bytes, false); console.log("read back volume", W.SmApiBrepComputeVolume(back, 1e-6));
try { W.SmApiCreateBox([0, 0, 0], -1, 10, 10); } catch (e) { console.log("error ->", e.message); }
for (const f of W.brepFaces(diff)) { const i = W.faceInfo(f); if (i.kind !== "plane") console.log("curved face", i.kind, i.radius, i.sense, W.faceNormal(f, [10, 10, 6]).normal); }
W.destroy(diff); W.destroy(back); W.destroy(box); W.destroy(sph);
