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
  /** a copy of a free curve (yours to destroy) */
  curveCopy(c: Ptr<"SmCurve">): Ptr<"SmCurve">;
  faceInfo(f: Ptr<"SmFace">): FaceInfo;
  /** the outward normal at the face point nearest `p` (null: a point inside the face) */
  faceNormal(f: Ptr<"SmFace">, p: V3 | null): { normal: V3; point: V3 };
  /** a point on the face's surface at (u, v) as fractions of its parameter box */
  facePoint(f: Ptr<"SmFace">, u: number, v: number): V3;

  /** a copy of a Brep, and where these of its faces / edges / vertices went in the copy */
  copyMapped(b: Ptr<"SmBrep">, topology: Ptr<string>[]): { brep: Ptr<"SmBrep">; map: Ptr<string>[] };
  /** a new Brep with copies of just these faces of a Brep: connected where they share edges (`merge`), or each alone */
  keepFaces(b: Ptr<"SmBrep">, faces: Ptr<"SmFace">[], merge: boolean): Ptr<"SmBrep">;
  /** one Brep with copies of every face of these Breps, side by side (no intersecting or stitching: sew it after) */
  combineSheets(breps: Ptr<"SmBrep">[]): Ptr<"SmBrep">;

  /** triangles of a whole Brep, grouped by face, normals outward */
  mesh(b: Ptr<"SmBrep">, params: TessellationParams | null): MeshArrays;
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
