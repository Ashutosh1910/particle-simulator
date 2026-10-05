# Particle Simulator

A 2D particle and rigid-body simulator built with [raylib](https://www.raylib.com/).
It started as a ball simulator with a uniform-grid broad phase. It now supports
six interchangeable broad phases, multithreading and an optional GPU path, and
four physics models: rigid bodies, an SPH fluid, Lennard-Jones molecules and
N-body gravity.

## Building

You need a C++17 compiler and raylib 5.x. On macOS: `brew install raylib`.

```sh
make            # builds ./psim (raylib found with pkg-config)
make run        # build and start
make test       # headless unit tests (no raylib needed)
make bench      # headless broad-phase benchmark
```

If raylib lives somewhere custom, set `RAYLIB_PREFIX`:
`make RAYLIB_PREFIX=/path/to/raylib`.

### GPU compute (optional, Linux/Windows)

The GPU path uses OpenGL 4.3 compute shaders. macOS supports OpenGL only up to
4.1, so there the option is shown greyed out with that reason. On Linux and
Windows:

```sh
make raylib-gl43                                        # builds raylib 5.5 with OpenGL 4.3
make GPU=1 RAYLIB_PREFIX=third_party/raylib-gl43
./psim --gpu-selftest                                   # compares GPU and CPU results
```

## Using it

The panel on the right always shows which **state** the simulator is in:

| State | Time | What the mouse does |
|-------|------|---------------------|
| **RUNNING** | advances | Grab / Attract / Repel act on the simulation |
| **PAUSED** | frozen | Step (N) advances one frame; Grab moves objects |
| **EDITING** | frozen | edit tools add, draw and erase objects |

Picking an edit tool (Ball, Rope, Blob, Box, Polygon, Wall, Erase) enters
EDITING automatically, and Space starts the simulation again. A tool that the
current scene's physics model can't use is greyed out. Hovering it shows the
reason in the hint bar at the bottom.

| Key | Action | Key | Action |
|-----|--------|-----|--------|
| Space | run / pause | E | edit mode on / off |
| N, → | step one frame (paused) | R | reset the scene |
| 1–0 | tools in toolbar order | G | broad-phase overlay |
| C | colour mode | Tab | next panel tab |
| H, F1 | help | Esc | close / cancel / leave edit |
| Q | quit | | |

### Scenes

| Scene | Model | Shows |
|-------|-------|-------|
| Ball pit | Rigid | gravity, restitution, friction, grabbing |
| Elastic gas (energy check) | Rigid | the original simulator; total energy stays flat |
| Mixed sizes | Rigid | where a uniform grid struggles and trees shine |
| Broad-phase stress test | Rigid | up to 150k particles: broad phases, threads, GPU |
| Ropes & cloth | Rigid | distance constraints, pinning, tearing |
| Soft bodies | Rigid | area-preserving rings, static ramps |
| Boxes & ramps | Rigid | convex polygons (SAT), stacking, friction |
| SPH dam break | SPH | fluid with floating boxes, density colouring |
| Lennard-Jones gas / liquid | LJ | condensation, Maxwell-Boltzmann speed histogram |
| Lennard-Jones crystal | LJ | crystallisation under gravity |
| Galaxy (Barnes-Hut) | N-body | rotating disc, Barnes-Hut vs direct summation |
| Galaxy collision | N-body | two discs and their tidal tails |

Command-line options: `./psim --help`.

## How it works

```
src/core        Vec2/AABB math, struct-of-arrays particle storage, thread pool
src/broadphase  Broadphase interface + 6 implementations
src/physics     World: models, integrators, contacts, constraints, rigid bodies
src/app         raylib/raygui front end, scenes, GPU path, video recording
tests/          headless unit tests      bench/  broad-phase benchmark
tools/          scripted demo recording (Xvfb + xdotool)
```

### Broad phase

Every broad phase implements the same interface. `build()` takes the particle
boxes, `findPairs()` returns every pair whose boxes overlap, and `queryAABB()`
answers region queries, which the polygon bodies use to find nearby particles.

| Broad phase | Idea | Good at |
|-------------|------|---------|
| Uniform grid | counting-sort into cells ≥ largest box; each cell vs itself + 4 forward neighbours | uniform sizes (fastest here) |
| Spatial hash | unbounded cells hashed into ~2n buckets | no world bounds, sparse worlds |
| Quadtree | recursive in-place partition, ≤ 8 per leaf | clustered particles |
| Sweep and prune | sort by left edge, sweep along x | mixed sizes, no tuning |
| BVH | top-down median split, exact node bounds | very mixed sizes (fewest tests) |
| Brute force | all n(n−1)/2 pairs | reference / baseline |

The tests check that all six return exactly the same pairs as brute force, for
uniform, mixed-size, clustered and degenerate inputs, and for any thread count.
The **Compare all broad phases** button in the Performance tab runs the same
check on the live particles and shows tests, pairs and time for each method.

Sample from `make bench` on a 4-core container (ms per build + pair search):

| 20,000 particles | uniform | mixed sizes | clustered |
|------------------|--------:|------------:|----------:|
| Brute force      | 729     | 750         | 285       |
| Uniform grid     | 1.4     | 9.8         | 4.0       |
| Spatial hash     | 4.9     | 19.4        | 9.4       |
| Quadtree         | 9.6     | 22.3        | 15.5      |
| Sweep and prune  | 9.6     | 11.2        | 14.2      |
| BVH              | 11.7    | 13.7        | 17.1      |

### Multithreading

A small thread pool splits loops into contiguous chunks. Pair search runs per
particle (or per grid row) in parallel. Contact solving is Gauss-Seidel, so it
is sequential by nature. To parallelise it anyway, the world is cut into vertical
stripes at least one interaction range wide. Each pair belongs to the stripe of
its left-most particle, so even stripes can't share particles and run in
parallel, followed by the odd stripes. The stripe layout doesn't depend on the
thread count, so results are **bit-identical for 1 or N threads** (tested).

### Integrators and time stepping

The app advances the simulation in fixed steps (`dt` = 1/60 s, split into
substeps) using an accumulator. If a frame takes too long, the leftover time is
dropped rather than spiralling, and the hint bar says so.

* **Symplectic Euler**: `v += a dt; x += v dt`. Contacts use positional
  correction plus impulses with restitution and Coulomb friction. With
  restitution 1, kinetic energy is conserved exactly.
* **Verlet**:
  * Rigid model: position-based Verlet (PBD). Contacts and constraints project
    positions and velocity is derived from the motion. A velocity pass restores
    restitution and friction. Very stable for piles and cloth.
  * Force models (Lennard-Jones, N-body): velocity Verlet. It is second order:
    in the tests its energy error is 0.13% against 2.2% for Euler.

### Physics models

* **Rigid**: spheres of any size (mass ∝ r²) with gravity, drag, mouse
  attract/repel, restitution and friction, plus:
  * distance constraints for ropes and cloth, with optional tearing
  * soft bodies: rings of particles that keep their area
  * convex polygons: SAT with reference-face clipping and a sequential-impulse
    solver with rotation and friction
  * static obstacles and user-drawn walls
* **SPH**: double-density relaxation (Clavet et al. 2005), solved Jacobi-style
  so it runs in parallel. Floating boxes interact with the fluid.
* **Lennard-Jones**: shifted-force LJ potential with a 2.5σ cutoff, an optional
  Berendsen thermostat, and a live speed histogram against the 2D
  Maxwell-Boltzmann distribution.
* **N-body**: softened gravity, either direct O(n²) or a Barnes-Hut quadtree
  with an adjustable opening angle. With θ = 0 it matches direct summation
  exactly (tested).

### GPU path

Runs plain balls in the Rigid model on the GPU with five compute passes per
substep:

1. clear the cell counts
2. integrate and count particles per cell
3. prefix sum over the cells
4. scatter particles into their cells
5. Jacobi collision, averaged over each particle's contacts

Positions are read back every frame so the CPU world, the editing tools and the
statistics stay in sync.

## Recording the demo

`tools/record_demo.py` starts the app on a virtual X display, drives it with
real mouse and keyboard input through xdotool, and records a fixed 60 fps
timeline with ffmpeg:

```sh
python3 tools/record_demo.py --psim ./psim --out demo.mp4
```

## Licences

raygui (zlib) is vendored in `third_party/raygui.h`. The embedded DejaVu Sans
font atlases are covered by `third_party/DEJAVU-LICENSE.txt`.
