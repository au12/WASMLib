// WASMLib: NVIDIA's SMLib B-rep kernel as a WebAssembly module.
//
//   import createWASMLib from "wasmlib";
//   const W = await createWASMLib();
//   W.init();
//   const box = W.SmApiCreateBox([0, 0, 0], 10, 10, 10);
//
// Two layers: every SM_API function one to one (SmApi, generated from upstream's headers), and the object layer
// below for what SM_API leaves to C++ callers (walking topology, reading geometry, meshes as typed arrays, files).
import type { SmApi } from "./smapi.gen";
import type { Ptr, TessellationParams, TessellationFailure, V3 } from "./types";

export * from "./types";
export type { SmApi } from "./smapi.gen";

export interface EdgeInfo {
  kind: "line" | "circle" | "ellipse" | "parabola" | "hyperbola" | "bspline" | "offset" | "other" | "none";
  start: V3; end: V3;
  /** the edge's stretch of its curve, start to end (t1 < t0 when it runs against the curve) */
  t0: number; t1: number;
  /** starts and ends on the same vertex */
  closed: boolean;
  /** circles and ellipses */
  center?: V3; axis?: V3; xAxis?: V3; radius?: number; radius2?: number;
}
export interface FaceInfo {
  kind: "plane" | "cylinder" | "cone" | "sphere" | "torus" | "bspline" | "revolution" | "extrusion" | "offset" | "blend" | "other";
  /** +1: the surface's normal points out of the material; -1: into it */
  sense: 1 | -1;
  /** the face's parameter box [uMin, uMax, vMin, vMax] */
  uv: [number, number, number, number];
  /** analytic surfaces: their frame (axis = the surface's own normal / axis) and sizes */
  origin?: V3; axis?: V3; xAxis?: V3; radius?: number; minorRadius?: number; semiAngleDeg?: number;
}
export interface MeshArrays {
  vertices: Float32Array; normals: Float32Array; triangles: Uint32Array;
  /** start, count per face in the Brep's face order (in triangle-index units) */
  groups: Uint32Array;
  failures: TessellationFailure[];
  outputManifold: boolean;
}

export interface Objects {
  /** make the kernel's context and catch its error messages (call once, first) */
  init(): void;
  /** free an object you own (a Brep, a PolyBrep, a free curve or surface) */
  destroy(p: Ptr<string>): void;
  typeOf(p: Ptr<string>): { id: number; cls: string } | null;

  brepTopology(b: Ptr<"SmBrep">): { faces: Ptr<"SmFace">[]; edges: Ptr<"SmEdge">[]; vertices: Ptr<"SmVertex">[]; shells: number; regions: number };
  brepFaces(b: Ptr<"SmBrep">): Ptr<"SmFace">[];
  brepEdges(b: Ptr<"SmBrep">): Ptr<"SmEdge">[];
  brepVertices(b: Ptr<"SmBrep">): Ptr<"SmVertex">[];
  /** faces grouped by the shell they belong to */
  brepShellFaces(b: Ptr<"SmBrep">): Ptr<"SmFace">[][];
  faceEdges(f: Ptr<"SmFace">, outerOnly: boolean): Ptr<"SmEdge">[];
  edgeFaces(e: Ptr<"SmEdge">): Ptr<"SmFace">[];
  edgeVertices(e: Ptr<"SmEdge">): [Ptr<"SmVertex"> | null, Ptr<"SmVertex"> | null];
  vertexPoint(v: Ptr<"SmVertex">): V3;

  edgeInfo(e: Ptr<"SmEdge">): EdgeInfo;
  /** points (then unit tangents, if asked) at fractions of the edge's length: xyz (txyz) per fraction */
  edgeSample(e: Ptr<"SmEdge">, fractions: number[], tangents: boolean): Float64Array;
  /** a free copy of the edge's curve, trimmed to it and running start to end (yours to destroy) */
  edgeCurve(e: Ptr<"SmEdge">): Ptr<"SmCurve">;
  /** a free curve: kind, ends, length, and its circle or ellipse when it is one */
  curveInfo(c: Ptr<"SmCurve">): EdgeInfo & { length: number };
  /** points (then unit tangents) at fractions of a free curve's length */
  curveSample(c: Ptr<"SmCurve">, fractions: number[], tangents: boolean): Float64Array;
  /** a free curve's mirror image (a new B-spline curve, yours) in the plane through `origin` square to `normal` */
  curveMirror(c: Ptr<"SmCurve">, normal: V3, origin: V3): Ptr<"SmCurve">;
  /** trim a free curve, in place, to [t0, t1] of its parameter */
  curveTrim(c: Ptr<"SmCurve">, t0: number, t1: number): void;
  /** a copy of a free curve (yours to destroy) */
  curveCopy(c: Ptr<"SmCurve">): Ptr<"SmCurve">;
  faceInfo(f: Ptr<"SmFace">): FaceInfo;
  /** the outward normal at the face point nearest `p` (null: a point inside the face) */
  faceNormal(f: Ptr<"SmFace">, p: V3 | null): { normal: V3; point: V3 };
  /** a point on the face's surface at (u, v) as fractions of its parameter box */
  facePoint(f: Ptr<"SmFace">, u: number, v: number): V3;

  /** a copy of a Brep, and where these of its faces / edges / vertices went in the copy */
  copyMapped(b: Ptr<"SmBrep">, topology: Ptr<string>[]): { brep: Ptr<"SmBrep">; map: Ptr<string>[] };
  /** a copy of a Brep with its surfaces recognized as planes, cylinders, cones, spheres or tori where they are */
  withAnalytics(b: Ptr<"SmBrep">): Ptr<"SmBrep">;
  /** a new Brep with copies of just these faces of a Brep: connected where they share edges (`merge`), or each alone */
  keepFaces(b: Ptr<"SmBrep">, faces: Ptr<"SmFace">[], merge: boolean): Ptr<"SmBrep">;
  /** one Brep with copies of every face of these Breps, side by side (no intersecting or stitching: sew it after) */
  combineSheets(breps: Ptr<"SmBrep">[]): Ptr<"SmBrep">;
  /** one new Brep holding copies of all of these, side by side with no intersecting — for Breps that don't touch
   *  (several tools then cut or joined in one boolean) */
  mergeDisjoint(breps: Ptr<"SmBrep">[]): Ptr<"SmBrep">;
  /**
   * A twisted extrusion as a solid: the closed profile `curves` (loops in a plane square to `axis`, left alone) moved
   * by `move` while turning `angleDeg` about the axis through `origin` (right-handed). One B-spline face per profile
   * curve, fitted to the true screw surface within `tol` (its control points' helices, cubic, ends exact), and the
   * profile and its turned copy as the flat ends. `volume`: exact (the profile's area × the distance along the axis);
   * `steps`: how many fitted steps the turn took.
   */
  screwSweep(curves: Ptr<"SmCurve">[], origin: V3, axis: V3, move: V3, angleDeg: number, tol: number): { brep: Ptr<"SmBrep">; volume: number; steps: number; glued: boolean };
  /**
   * A solid (or shell) from faces made elsewhere — a shape from another kernel brought into SMLib. Each face: its
   * NURBS surface (`P` x y z, `nv` per row of u; `U`, `V` every knot; `W` weights or null), `reversed` when the
   * material is on the side the surface normal points away from, and its loops (outer first: anticlockwise seen from
   * outside; then holes), each a chain of NURBS curves with `senses[i]` -1 where curve i runs against the loop. A
   * curve may be given whole and cut to the stretch `from` → `to`, which runs `along` its parameter or against it
   * (crossing a closed curve's start if need be). No loops: the whole surface. The faces are made one by one, then
   * stitched and turned outward. `solid`: a closed manifold solid came out; `failed`: faces that couldn't be made.
   */
  /**
   * A closed profile (one loop of free curves, at the path's start) swept along a path of free curves joined end to end
   * (in order, either way round), each curve swept on its own and the corners the path turns at by more than
   * `cornerDeg` closed with a mitre. Inputs left alone; the solid is yours.
   */
  sweepPath(profile: Ptr<"SmCurve">[], path: Ptr<"SmCurve">[], tol: number, cornerDeg: number): Ptr<"SmBrep">;
  brepFromFaces(faces: { surface: { p: number; q: number; U: number[]; V: number[]; nu: number; nv: number; P: number[]; W: number[] | null }; reversed: boolean; loops: { curves: { p: number; U: number[]; P: number[]; W: number[] | null; from?: V3; to?: V3; along?: boolean }[]; senses: number[] }[] }[], tol: number, opts: { areas?: number[] } | null): { brep: Ptr<"SmBrep">; solid: boolean; faces: number; failed: number[]; areas?: number[] };
  /** a Brep's bounding box, quick and close: the control nets' box, with the round faces that reach outside the rest
   *  (whose nets bulge — a turned cylinder's by up to two fifths of its radius) tightened, from their edges where that's exact */
  brepBox(b: Ptr<"SmBrep">): [V3, V3];

  /** triangles of a whole Brep, grouped by face, normals outward */
  mesh(b: Ptr<"SmBrep">, params: TessellationParams | null): MeshArrays;
  /**
   * Triangles for drawing a Brep: as `mesh`, except big B-spline faces that are all of a rectangle of their parameters
   * (a thread's flank, a twisted extrusion's side, a loft) are sampled on a grid, with normals from the surface — SMLib's
   * tessellator folds a third of a thread's triangles — and the other faces are tessellated without them. `grid`: the
   * grid's largest distance off the surface (`tol`), largest turn between neighbouring facets (`angleDeg`), and most
   * points per face (`maxPoints`, default 400 000). Not watertight: for the view, not for export. `gridded`: 1 for each
   * face sampled on a grid.
   */
  displayMesh(b: Ptr<"SmBrep">, params: TessellationParams | null, grid: { tol: number; angleDeg: number; maxPoints?: number }): MeshArrays & { gridded: Uint8Array };
  /** every edge as a polyline (segment end pairs), grouped by edge in the Brep's edge order */
  edgeLines(b: Ptr<"SmBrep">, angleTolDeg: number): { lines: Float32Array; groups: Uint32Array };

  /** reflect a Brep, in place, in the plane through `origin` square to `normal` */
  mirror(b: Ptr<"SmBrep">, normal: V3, origin: V3): void;
  /** any affine map of a Brep, in place: 12 numbers, three rows of [m00 m01 m02 tx] */
  transformMatrix(b: Ptr<"SmBrep">, rows: number[]): void;

  /**
   * STEP (AP214) text for these bodies — WASMLib's own writer (SMLib has none): one product, each body a named solid,
   * coloured when given (`color`: r, g, b from 0 to 1). Analytic surfaces and curves are written as such, the rest as
   * B-splines.
   */
  stepWrite(items: { brep: Ptr<"SmBrep">; name?: string; color?: [number, number, number] | null }[], product: string): string;

  /** SMLib's own Brep file format (binary unless `ascii`) */
  writeBrep(b: Ptr<"SmBrep">, ascii: boolean): Uint8Array;
  readBrep(bytes: Uint8Array, ascii: boolean): Ptr<"SmBrep">;
}

export type WASMLib = SmApi & Objects & {
  /** Emscripten's in-memory file system (for SM_API's file functions) */
  FS: any;
};

export interface ModuleOptions {
  locateFile?(path: string, prefix: string): string;
  wasmBinary?: ArrayBuffer | Uint8Array;
  print?(s: string): void;
  printErr?(s: string): void;
}

export default function createWASMLib(options?: ModuleOptions): Promise<WASMLib>;
