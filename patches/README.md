# Patches to SMLib

WASMLib builds SMLib from the `upstream` submodule ([usd-brep](https://github.com/NVIDIA-Omniverse/usd-brep) 1.0.1)
untouched, then applies the patches here at configure time: CMake copies each file a patch touches into the build tree,
applies the patches in order, and compiles those copies in place of upstream's (see `CMakeLists.txt`). Delete this
folder and you build plain upstream SMLib.

Every patch is for speed only. Each one is written so the answers can't change — a test skipped only where its answer
is known in advance — except 0004, whose volumes move within the accuracy asked for (see below). Upstream's own SM_API
test suite passes all 135 tests with them, as without, and 26% quicker (73 s → 53 s natively).

The cases they were measured on are real calls from [DadCAD](https://github.com/au12/DadCAD)'s benchmark parts,
replayed natively (`g++ -O2`, minimum of several runs) and profiled with callgrind.

| Patch | What | Measured |
|---|---|---|
| `0001-surface-derivatives-homogeneous-sum` | `N_SrfDerivs` works out a NURBS surface's derivatives by summing the homogeneous control net once and applying the quotient rule to the point (The NURBS Book, A4.4) instead of taking the rational derivatives of every one of the (p+1)(q+1) basis functions and then summing those. The same values, a fraction of the work. It's under every curved-surface evaluation with derivatives: cylinders and cones go through their NURBS form here too. | 4 200 → 1 600 instructions a call; a fillet 27 → 21 ms, a volume 16 → 11 ms, a shell 159 → 131 ms |
| `0002-boolean-skip-face-pairs-apart` | A boolean intersected every face of one body with every face of the other — surface/surface intersection and curve classification for each pair. Now each face's cheap bounding box is worked out once, and pairs whose boxes are apart by more than the tolerances are skipped: they can't meet. | joining a knurl's 3 400-facet ring to its knob: 435 456 face intersections → 3 062; one letter cut from a plate 97 → 61 ms |
| `0003-coplanar-faces-skip-edges-apart` | When two faces lie in one plane, every edge of both was classified against both faces. Edges whose boxes are apart from the other face's box can't bound where the two overlap, so they're left out. A 126-sided knob end lying in the plane of 126 small facets had each of its edges classified 126 times. | that join: 72 904 curve classifications → 10 760, 3.6 s → 1.2 s natively |
| `0004-volume-integrates-volume-only` | `SmApiBrepComputeVolume` integrated every mass property at once — area, volume, first and second moments, products of inertia: 26 numbers at every step — to return one. It now asks each face for area and volume only (the per-face routine has always taken a mask; its step control is area-based either way). | a filleted box's volume 10.7 → 6.4 ms. Volumes move in the 6th–7th significant figure, well inside the accuracy asked for: a filleted box (exactly 47 109.079 mm³) 47 109.159 → 47 109.132; a Ø30 × 15 cylinder (exactly 10 602.875) 10 602.879 → 10 603.020 |
| `0005-flat-face-loops-skip-points-apart` | Classifying a point on a face marched round every loop of the face. On a flat face, a point outside a loop's UV box (by more than the tolerance) is outside that loop, so the march is skipped. A plate with many letters or holes cut into it otherwise marched round all of them for every point. | cutting the 5th, 10th and 11th letters: 54 → 32, 79 → 39, 201 → 72 ms |

Tried and dropped (not here): a shorter connectivity search before `DeleteFace` / `DeleteEdge`'s full shell walk —
on a knurl the two ends of each deleted edge only connect the long way round, so the search seldom proved anything and
cost more; and building a polyhedron by gluing faces pair by pair — region tracking makes every glue walk the shell,
3× slower than SMLib's stitch.

To change a patch: edit the file in `upstream/`, `git -C upstream diff <file> > patches/NNNN-name.patch`, then
`git -C upstream checkout <file>` so the submodule stays clean; the next configure applies it.
