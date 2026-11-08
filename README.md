# WASMLib

**NVIDIA's SMLib B-rep solid-modelling kernel, compiled to WebAssembly from source.**

[SMLib](https://github.com/NVIDIA-Omniverse/usd-brep) is the NURBS / analytic boundary-representation kernel NVIDIA
open-sourced (Apache-2.0) in usd-brep: booleans, fillets and chamfers, shelling and offsets, sweeps, skins,
intersections, healing, tessellation. WASMLib carries its `SMLib` and `SM_API` source (from usd-brep 1.0.1, in
[`smlib/`](smlib/README.md)), builds it for the browser, workers and Node, and adds what a JavaScript caller needs:

- **A faster SMLib.** WASMLib's changes to SMLib are commits in this repo, each listed with its measurement in
  [`smlib/README.md`](smlib/README.md): surface derivatives summed the textbook way (and, for degrees 1–3, in code
  written for the degree), booleans that skip face pairs, coplanar edges, edges and vertices that can't meet, deleting
  faces and closing regions without walking the whole shell each time, loop lists gathered in one pass, volumes that
  integrate the volume rather than every mass property, and point tests on flat faces that skip loops the point is
  nowhere near. The same answers (volumes within the accuracy asked for); in DadCAD a knurled knob builds 3.9× faster
  than on the unchanged SMLib, engraved text 2.2–2.4×, a plate of 48 holes 1.7×, a filleted box 1.4×, a shell 1.3×.

- **All of SM_API, one to one** — 176 functions, generated from upstream's headers (`tools/gen_smapi.py`), with
  TypeScript declarations carrying upstream's parameter notes.
- **An object layer** for what SM_API leaves to C++ callers, batched so JavaScript asks once per question: walking a
  Brep's topology; the analytic surface or curve under a face or edge (B-splines that are really lines or circular
  arcs say so); outward normals; meshes as typed arrays grouped by face; edge polylines; mirror and general affine
  transforms; SMLib's own `.smb` files.
- **Solids SMLib has no call for** (`bind/build.cpp`): a **screw sweep** — a closed profile moved along an axis while
  it turns about it (threads, helical gears, twisted vases) — as one B-spline face per profile curve, fitted to the
  true screw surface within a tolerance you give, its faces glued along the edges known to meet (SMLib's general stitch
  searches every pair: most of the time on a long thread) and its volume returned exactly; **`mergeDisjoint`**, Breps
  copied side by side into one with no intersecting (SM_API's list union drops pieces of disjoint ones); and
  **`brepBox`**, a bounding box as quick as SMLib's own but tight on turned round faces, whose control nets bulge up
  to two fifths of a radius outside them (SMLib's tight box is ~800× slower on a plate with 48 holes).
- **Sweeps along a path of several curves** (`sweepPath`) — SMLib's curve sweep takes one curve, and moves the
  profile's first point (or its middle) onto it; its planar-path sweep re-places the profile by its own rule.
  `sweepPath` keeps the profile where it was drawn: along a line it's pushed (a prism), round an arc turned about the
  arc's axis (planes, cylinders and tori), along anything else carried by SMLib's own sweep surface without being moved
  first. Smooth joins carry on; corners are mitred, as OpenCascade's "right corner" pipe — each pipe run on to the
  plane halfway between the two directions. Volumes and positions match OpenCascade's on straight, bent, cornered,
  closed and free-form paths, with the profile on the path or off it.
- **Shapes from elsewhere** (`brepFromFaces`) — faces given as NURBS surfaces with loops of NURBS curves, made one
  by one and stitched into a solid: how DadCAD brings STEP files (read in TypeScript) and OpenCascade's shapes into
  SMLib. Knots must be clamped; a solid with hollows is built whole and the hollows taken away.
- **A STEP (AP214) writer** — SMLib has none. Planes, cylinders, cones, spheres, tori, lines, circles and ellipses
  are written as such, everything else as (rational) B-splines from SMLib's own control nets, with names and
  colours; a solid with hollows as one solid with voids. Checked by reading the files back into OpenCascade (volumes
  agree, shapes valid).

| | |
|---|---|
| WebAssembly | 7.1 MB (2.4 MB gzip, 2.0 MB brotli) |
| Start-up | ~40 ms to instantiate in Node 22 |
| Upstream's SM_API test suite | **127 / 127 pass** with WASMLib's changes, as without — in WebAssembly (57 s; 78 s without them) and natively (53 s, g++ -O2; 73 s without) |
| Build | Emscripten 7.0: `-O3 -flto -msimd128 -fwasm-exceptions`, 16 MB stack, growable heap to 4 GB |
| Licence | Apache-2.0 (WASMLib and SMLib alike) |

## Use

```js
import createWASMLib from "wasmlib";

const W = await createWASMLib();
W.init();                                               // the kernel's context, and its error messages caught

const plate = W.SmApiCreateBox([0, 0, 0], 60, 40, 6);
const hole = W.SmApiCreateCylinder([30, 20, -1], 4, 8);
const part = W.SmApiBooleanDifference(plate, hole);     // SM_API's contract: the result is `plate`, `hole` is gone

const { brep, map } = W.copyMapped(part, W.brepEdges(part).slice(0, 4));
W.SmApiFilletEdges(brep, map, 2, 1, 1, 1);              // fillets work in place: on a copy here

console.log(W.SmApiBrepComputeVolume(brep, 1e-6));
const m = W.mesh(brep, { chordHeightTol: 0.01, surfaceAngleTolDeg: 10 });   // Float32Array / Uint32Array, by face
const step = W.stepWrite([{ brep, name: "plate", color: [0.9, 0.5, 0.1] }], "plate");

W.destroy(part); W.destroy(brep);
```

- **Handles** are numbers (the object's address). Who owns what follows SM_API exactly — each function's notes in
  `js/smapi.gen.d.ts` say what it consumes, changes or hands over. Faces, edges and vertices belong to their Brep.
  Free what you own with `destroy`.
- **Errors** throw a JavaScript `Error` naming the call, the status and the kernel's own messages:
  `SmApiCreateBox failed: SM_ERR_INVALID_INPUT`.
- Points and vectors are `[x, y, z]`; the object layer's functions are listed in `js/wasmlib.d.ts`.

## Build

```bash
git clone https://github.com/au12/WASMLib && cd WASMLib
source /path/to/emsdk/emsdk_env.sh       # Emscripten 7.0
tools/build.sh                           # → dist/wasmlib.mjs, dist/wasmlib.wasm
tools/build.sh test                      # also upstream's SM_API test suite, run in Node
tools/build.sh native                    # the same suite built natively (the reference)
npm test                                 # WASMLib's own checks of its JavaScript API
```

`dist/` is committed, so `npm install github:au12/WASMLib` works without a toolchain.

## How it compares

WASMLib was built to compare SMLib with OpenCascade inside [DadCAD](https://github.com/au12/DadCAD), whose modelling
engine runs on a kernel interface with interchangeable backends. The full head-to-head — the same parts built by the
same engine on OpenCascade 7 (opencascade.js), OpenCascade 8 (occt-wasm), WASMLib and smlibwasm — is in DadCAD's
[`docs/kernel.md`](https://github.com/au12/DadCAD/blob/main/docs/kernel.md), with what the screw sweep and the
changes to SMLib made. In short (November 2026, Node 22, median of 3):

| | OpenCascade 7 (opencascade.js) | WASMLib | smlibwasm |
|---|---:|---:|---:|
| DadCAD's engine tests (220) | 220 | 220 | 220 |
| WebAssembly, brotli | 5.7 MB | **2.0 MB** | 2.6 MB |
| Load | 360 ms | **123 ms** | 216 ms |
| Fillet a box's 12 edges, then its volume | 38 ms | **33 ms** | 111 ms |
| “DadCAD 2026” engraved into a plate | 699 ms | **622 ms** | 2 039 ms |
| 48 holes cut one at a time | 991 ms | **383 ms** | 726 ms |
| M8 bolt, real thread: build + STL mesh | 7.5 s | **1.4 s** | 6.5 s |
| M8 nut, tapped hole: build | 2.0 s | **0.17 s** | 9.8 s |
| Helical gear | 0.94 s | **0.73 s** | 2.8 s |
| Knurled knob | 2.0 s | **1.2 s** | 8.1 s |
| Shell a filleted box | **58 ms** | 183 ms | 576 ms |
| STEP export of a filleted box | 97 ms | **61 ms** | 141 ms |

Every part builds valid with the same volume on each, and WASMLib beats smlibwasm on all 12. Against OpenCascade it
is now ahead on knurls, text, gears, the tapped nut, booleans one at a time and meshing, and level on fillets; still
slower on shells (3.2×), one boolean with many tools (2.7×) and a bolt's thread.

## What SMLib doesn't do

No STEP or IGES import (DadCAD reads STEP itself and builds the faces with `brepFromFaces`), no conversion of a mesh
into a Brep, no draft of existing faces, no two-distance chamfer, no imprinting of curves onto a face (only projecting
through a whole body), no twisted extrusion (WASMLib's screw sweep is that), no sweep along several curves
(`sweepPath` is that). DadCAD's backend builds the draft, the chamfer and the imprint from what SMLib does have; see
its `src/kernel/geom/wasmlib`.

## Licence

WASMLib is Apache-2.0 (`LICENSE`). SMLib and SM_API (`smlib/`) are © NVIDIA Corporation & Affiliates, Apache-2.0
(`LICENSE.usd-brep`, also `smlib/LICENSE`), with WASMLib's changes marked in each file it changed; see `NOTICE`. None
of usd-brep's third-party components (OpenUSD, oneTBB, …) are compiled in.
