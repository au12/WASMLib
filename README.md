# WASMLib

**NVIDIA's SMLib B-rep solid-modelling kernel, compiled to WebAssembly from source.**

[SMLib](https://github.com/NVIDIA-Omniverse/usd-brep) is the NURBS / analytic boundary-representation kernel NVIDIA
open-sourced (Apache-2.0) in usd-brep: booleans, fillets and chamfers, shelling and offsets, sweeps, skins,
intersections, healing, tessellation. WASMLib builds its `SMLib` and `SM_API` sources — unmodified, from the
`upstream` submodule (usd-brep 1.0.1) — for the browser, workers and Node, and adds what a JavaScript caller needs:

- **All of SM_API, one to one** — 176 functions, generated from upstream's headers (`tools/gen_smapi.py`), with
  TypeScript declarations carrying upstream's parameter notes.
- **An object layer** for what SM_API leaves to C++ callers, batched so JavaScript asks once per question: walking a
  Brep's topology; the analytic surface or curve under a face or edge (B-splines that are really lines or circular
  arcs say so); outward normals; meshes as typed arrays grouped by face; edge polylines; mirror and general affine
  transforms; SMLib's own `.smb` files.
- **A STEP (AP214) writer** — SMLib has none. Planes, cylinders, cones, spheres, tori, lines, circles and ellipses
  are written as such, everything else as (rational) B-splines from SMLib's own control nets, with names and
  colours. Checked by reading the files back into OpenCascade (volumes agree, shapes valid).

| | |
|---|---|
| WebAssembly | 6.9 MB (2.4 MB gzip, 2.0 MB brotli) |
| Start-up | ~40 ms to instantiate in Node 22 |
| Upstream's SM_API test suite | **127 / 127 pass** in WebAssembly (78 s) — and natively (73 s, g++ -O2): within ~7% of native |
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
git clone --recursive https://github.com/au12/WASMLib && cd WASMLib
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
[`docs/kernel.md`](https://github.com/au12/DadCAD/blob/kernel-interface/docs/kernel.md).

## What SMLib doesn't do

No STEP or IGES import, no conversion of a mesh into a Brep, no draft of existing faces, no two-distance chamfer, no
imprinting of curves onto a face (only projecting through a whole body). DadCAD's backend builds the last three from
what SMLib does have; see its `src/kernel/geom/wasmlib`.

## Licence

WASMLib is Apache-2.0 (`LICENSE`). SMLib and SM_API are © NVIDIA Corporation & Affiliates, Apache-2.0
(`LICENSE.usd-brep`); see `NOTICE`. None of usd-brep's third-party components (OpenUSD, oneTBB, …) are compiled in.
