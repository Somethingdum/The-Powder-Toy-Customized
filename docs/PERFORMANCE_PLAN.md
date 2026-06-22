# Performance & Resolution Plan

Target hardware: hybrid 24-core CPU (**8 Performance cores + 16 Efficiency
cores**), 32 GB RAM. Goal: large, fine-grained, fast, smooth simulation that
uses the cores where it is safe to do so.

> **Hybrid-core note.** The parallel physics work is fork-join with a barrier
> every tick, so it finishes only as fast as the *slowest* worker. Mixing slow
> E-cores with fast P-cores creates stragglers that stall every tick. Default
> worker count therefore targets the **8 P-cores**, not all 24. See Phase 2.

> **Save compatibility is intentionally abandoned.** Changing `XRES`/`YRES`
> and `CELL` makes saves incompatible with stock TPT and the online save
> browser. That is an accepted cost of this fork.

---

## Locked decisions

| Setting        | Value                          | Source of truth            |
|----------------|--------------------------------|----------------------------|
| Sim resolution | **2560 × 1600 (1600p)**, fixed | `src/SimulationConfig.h`   |
| `CELL`         | **2** (finer air/pressure grid)| `src/SimulationConfig.h`   |
| Size mechanism | Compile-time constant, **no in-game slider** | — |
| Sim tick rate  | **60/s** (normal motion speed) | `src/FpsLimit.h`           |
| Render rate    | **240/s** via interpolation    | `src/FpsLimit.h` + Renderer|
| Build          | `-O3 -march=native` + LTO      | meson                      |

### Derived constants (for `SimulationConfig.h`)
```cpp
constexpr int CELL = 2;                      // was 4
constexpr Vec2<int> CELLS = Vec2(1280, 800); // was Vec2(153, 96)
// => RES = 2560 x 1600
// => XCELLS=1280, YCELLS=800, NCELL=1,024,000
// => NPART = 4,096,000  (was 235,008; ~17x)
```

### Memory budget at 1600p (per Simulation instance)
- `parts` (NPART x ~64 B) .................. ~262 MB
- `pmap` + `photons` + `pmap_count` (4 B ea) ~49 MB
- `gol` (NPART x 5 x 4 B) .................. ~82 MB
- air grids @ CELL=2 (NCELL x ~10 x 4 B) ... ~41 MB
- **~0.45 GB per sim; ~3 live copies (main, render thread, save renderer) => ~1.4 GB.** Fine on 32 GB.

### Caveats to confirm during implementation
1. **Window vs panel fit.** `WINDOW = RES + (BARSIZE=17, MENUSIZE=40)` =>
   **2577 × 1640**, which overflows a 2560×1600 panel by 17 px wide / 40 px
   tall (UI chrome). Options: run borderless/fullscreen and accept overlap,
   or shrink sim area. Default: keep sim area at 2560×1600, use fullscreen.
2. **Heap allocation confirmed.** `Simulation` is created via
   `Simulation::Factory()` behind `std::unique_ptr` (`GameModel.h:71`), so the
   large arrays do not hit the stack. No change needed, but keep it that way.
3. **Cost scales with *active* particles, not NPART.** An empty 1600p sim is
   cheap; only filled regions cost time. So FPS depends on how full the screen
   is, not just resolution.

---

## Phase 0 — Build tuning (effort: tiny, risk: none)

Tune the binary for this exact CPU.

- meson: `-Dbuildtype=release -Db_lto=true` and add `-march=native -O3` to
  C/C++ args (via `meson setup` `-Dcpp_args=-march=native` or a
  machine/native file).
- `-march=native` makes the binary **non-portable** to other CPUs — acceptable
  for a personal build.
- Verify the build still passes its existing checks after enabling LTO (LTO can
  surface ODR / inlining issues).

**Deliverable:** documented build command (or a meson option) producing an
LTO + native-tuned release binary.

---

## Phase 1 — Resolution to 1600p @ CELL=2 (effort: small, risk: save-compat only)

Edit `src/SimulationConfig.h`:
- `CELL = 2`
- `CELLS = Vec2(1280, 800)`

Then build and fix any fallout:
- Anything that assumed `CELL == 4` arithmetic (search for literal `4` near
  `CELL`, `ISTP`, `CFDS` — these are already derived from `CELL`, good).
- Confirm no fixed-size stack buffers sized by `NPART`/`NCELL` anywhere
  (they're members of heap objects — verify).
- Sanity-run: place particles, confirm air/pressure/heat behave and the window
  opens at the new size.

**Deliverable:** a 2560×1600, CELL=2 sim that launches and simulates.

---

## Phase 2 — Multi-core grid solvers (effort: medium, risk: low / slight dynamics change)

The wins that actually use your 24 cores **safely**:

### 2a. Air pressure/velocity (`src/simulation/Air.cpp::update_air`) — DONE
- On inspection the dominant passes turned out **better than feared** and
  needed **no Jacobi conversion** (results stay bit-identical to serial):
  - *pressure pass*: reads `vx/vy` (not written here), writes only `pv[y][x]`.
  - *velocity pass*: reads `pv` (not written here), writes only `vx/vy[y][x]`.
  - *advection pass* (the expensive 3x3 kernel + ray-march): already writes to
    separate `ovx/ovy/opv` buffers then `memcpy`s back — already double-buffered.
- Each pass's outer `y` loop is split into contiguous row-blocks via
  `RowWorkerPool` (`src/common/RowWorkerPool.h`), with the implicit barrier
  between passes preserved (`ForRows` blocks until done). Because no per-cell
  arithmetic changed and each cell reads only pass-invariant inputs, output is
  **bit-identical** to the serial loop regardless of thread count.
- At CELL=2 there are ~1.0M cells, so this is exactly where the extra cost
  lives — biggest payoff here.

### 2b. Ambient heat (`update_airh`) — left SERIAL (intentionally)
- Unlike `update_air`, the heat pass writes `vx[y][x]`/`vy[y][x]` **in place**
  while also reading `vx/vy` neighbours (convection), making it genuinely
  Gauss-Seidel. Parallelizing it would change results. It is also the optional
  ambient-heat path, not the always-on cost, so it is deliberately untouched
  for now. Revisit with a double-buffer conversion if it becomes a bottleneck.

### 2b. Gravity
- Already FFT-threaded (`src/simulation/gravity/Fft.cpp`). Verify the FFT thread
  count scales up; bump it toward core count if it is hardcoded low.

### Infrastructure (hybrid-core aware)
- Add a small reusable thread pool (or use OpenMP `parallel for`) rather than
  spawning threads each tick.
- **Default worker count = 8 (the P-cores), NOT 24.** A per-tick barrier means
  the slowest worker gates the whole pass; E-cores would be stragglers and a
  9th+ thread on an E-core can make a tick *slower* than 8 P-core threads.
- Where the OS allows, pin workers to P-cores (Linux: `sched_setaffinity`;
  Windows: `SetThreadSelectedCpuSetMasks` / `SetThreadAffinityMask`). P/E
  detection is fiddly cross-platform, so ship a configurable worker count with
  default 8 and let the affinity pinning be best-effort.
- Give E-cores a role instead of wasting them: run *non-barrier* background work
  there (e.g. the existing separate render-thread sim copy, save
  rendering/thumbnails, autosave) so the 16 E-cores stay useful without dragging
  the physics barrier.

**Deliverable:** air/heat/gravity solvers running across N cores; measurable
FPS gain on a pressure/heat-heavy scene.

---

## Phase 3 — Render interpolation for true 240 FPS at normal speed (effort: medium, risk: low)

Problem restated: sim@60 + render@240 with no interpolation just redraws
**duplicate frames** — no benefit. We want normal motion speed *and* smooth
240 FPS.

Approach (renderer-only; physics untouched):
1. Keep two snapshots of particle positions: `prev` (last tick) and `cur`
   (this tick). `x, y, vx, vy` are already floats.
2. The main loop (`src/PowderToySDL.cpp`) **already** runs `SimTick` and `Draw`
   on **separate schedules** (`tickSchedule` / `drawSchedule`) — the infra
   exists. Set sim cap 60, draw cap 240.
3. On each draw, compute `alpha = time_since_last_tick / tick_interval`
   (clamped 0..1) and render each particle at
   `lerp(prev.pos, cur.pos, alpha)` in `src/graphics/Renderer.cpp`.
4. Handle edge cases: newly created/destroyed particles (no `prev`), and
   teleport-like jumps (skip interpolation past a distance threshold) to avoid
   smearing.
5. TPT already has a `rendererThreadSim` copy — wire the interpolation against
   the render-thread sim so it doesn't fight the physics thread.

Defaults in `src/FpsLimit.h`:
```cpp
constexpr auto DefaultFpsLimit  = FpsLimitExplicit{ 60.f };  // sim speed (normal)
constexpr auto DefaultDrawLimit = DrawLimitExplicit{ 240 };  // smooth visuals
```

**Deliverable:** genuinely smooth 240 FPS with sand/liquids moving at normal
real-time speed.

---

## Phase 4 — Parallel particle movement (effort: large, risk: HIGH — deferred)

Not in current scope. Recorded for later.

- `SimulationImpl::UpdateParticles` walks particles in index order; each reads
  **and writes** neighbours' `pmap` and triggers reactions. Parallelizing
  causes write races on `pmap` and order-dependent reaction outcomes →
  results become non-deterministic (saves stop reproducing).
- Viable approach if pursued: spatial tiling (e.g. checkerboard of tiles
  processed in two+ phases) with halo regions and a conflict-resolution rule
  for particles crossing tile boundaries. This **changes physics behavior** and
  is a multi-week effort. Treat as research, not a config change.
- Note: the `UpdateParticles(start, end)` signature is for the debug
  single-step tool, **not** existing threading scaffolding.

---

## Suggested order of execution

1. **Phase 0** (build) and **Phase 1** (1600p/CELL=2) together — get the big
   sim running and tuned first; measure baseline FPS.
2. **Phase 3** (interpolation) — immediate visible smoothness win, low risk.
3. **Phase 2** (parallel grid solvers) — recover the FPS that 1600p/CELL=2
   costs by spreading air/heat/gravity across cores.
4. **Phase 4** — only if single-thread particle update is still the bottleneck
   and the determinism tradeoff is acceptable.

Each phase is an independent commit on `claude/cool-clarke-lgak94`, with a
quick before/after FPS note in the commit message.
