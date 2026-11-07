// The value types WASMLib's functions take and give back.

export type V2 = [number, number];
export type V3 = [number, number, number];

/**
 * A kernel object: its address in the WebAssembly heap, as a number. The tag says what it points at; it exists only
 * for the type checker. Who frees an object follows SM_API's contract for the call that made it (see each
 * function's notes); free one with `destroy`.
 */
// eslint-disable-next-line @typescript-eslint/no-unused-vars
export type Ptr<T extends string = string> = number;

/** tessellation tolerances (SmTessellationParams); anything left out keeps SMLib's default */
export interface TessellationParams {
  /** largest distance from a facet to the surface (0: no limit) */
  chordHeightTol?: number;
  /** largest turn, in degrees, between neighbouring segments along an edge */
  curveAngleTolDeg?: number;
  /** largest turn, in degrees, between neighbouring facets */
  surfaceAngleTolDeg?: number;
  /** longest facet edge (0: no limit) */
  maxEdgeLength?: number;
  /** most elongated facet allowed (0: no limit) */
  maxAspectRatio?: number;
}

/** sweep options (SmSweepOptions); all off by default */
export interface SweepOptions {
  capEnds?: boolean;
  translational?: boolean;
  moveProfileCenterToPath?: boolean;
  orientProfilePerpendicularToPath?: boolean;
  moveToProfile?: boolean;
}

/** a rotation then a move (SmAxis2Placement): the local frame's origin and its x and y axes, in world coordinates */
export interface Placement { origin: V3; xAxis: V3; yAxis: V3 }

export interface TessellationFailure { faceIndex: number; stage: number; status: number; statusName: string }
export interface TessellationReport {
  failures: TessellationFailure[];
  laminaEdges: number;
  spineEdges: number;
  inputManifold: boolean;
  outputManifold: boolean;
  hasMesh: boolean;
}
