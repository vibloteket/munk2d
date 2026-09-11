# Munk2D optimization log

Short record of measured optimization experiments. Percentages are lower-is-faster.
For Munk2D 2.x, candidates are accepted only after tests, exact MunkBench validation,
and alternating baseline/candidate measurements. Major-version experiments may use
the versioned `munkbench-v4` behavior envelope when trajectory changes are intentional,
but must remain deterministic and pass its stability and scenario invariants. Check
this file before repeating an experiment. Reducing solver iterations, collision
accuracy, convergence, or numeric precision is not treated as an optimization; such
quality/performance tradeoffs require explicit approval before implementation.

## Accepted

| Area | Change | Result | PR |
|---|---|---|---|
| BBTree | Larger fat bounds for isolated fast movers | SlowExplosion about -14%; neutral elsewhere | [#7](https://github.com/vibloteket/munk2d/pull/7) |
| BBTree | Reuse BB already fetched by `LeafUpdate` | SlowExplosion about -3% standalone | [#8](https://github.com/vibloteket/munk2d/pull/8) |
| BBTree | Cache bounds and simplify insertion-cost arithmetic | Diagonal -10%, SlowExplosion -9%, Multifixture -6% incremental | [#9](https://github.com/vibloteket/munk2d/pull/9), [#10](https://github.com/vibloteket/munk2d/pull/10) |
| Callbacks | Skip default no-op begin/pre/post callbacks | Contact workloads about -0.5% to -1.8% | [#11](https://github.com/vibloteket/munk2d/pull/11) |
| Narrow phase | Direct built-in collision dispatch | Multifixture -4.7%; collision workloads about -0.7% to -2% | [#12](https://github.com/vibloteket/munk2d/pull/12) |
| Solver | Dedicated scalar frictionless path | Zero-friction contact workloads about -2% to -10%; friction 0.7 control neutral | [#13](https://github.com/vibloteket/munk2d/pull/13) |
| GJK | Start polygon support scan at vertex 1 | Polygon-heavy workloads about -0.3% to -0.9% | [#16](https://github.com/vibloteket/munk2d/pull/16) |
| GJK/EPA | Dispatch support points by shape pair | Polygon/contact workloads about -1% to -2% | [#17](https://github.com/vibloteket/munk2d/pull/17) |
| GJK/EPA | Pair-specific cached support lookup | Relevant GJK workloads about -0.4% to -1.0% | [#20](https://github.com/vibloteket/munk2d/pull/20) |
| BBTree (next major) | Dense leaf array for sequential iteration while retaining hash lookup | AddPair -2.7%, SlowExplosion -4.6%, Multifixture -1.7%; neutral to about -0.6% elsewhere | [#32](https://github.com/vibloteket/munk2d/pull/32) |
| Hash set (next major) | Intrusive active-bin list for filtering without scanning empty buckets | AddPair/MostlyStatic/SleepWake -4% to -5%; N2 -5%; neutral to about -1.5% elsewhere | — |

## Rejected: BBTree

| Experiment | Outcome / reason |
|---|---|
| Global fat-AABB coefficient sweep | Large sparse-world wins, but AddPair/Diagonal/Multifixture regressed up to 10–70%; trajectories changed. |
| Velocity-only adaptive fat AABB | SlowExplosion -26%, but dense scenes regressed badly; trajectories changed. |
| Initial traversal stack size 16/32/64/128 | Mostly within 1%; current 64 retained. |
| Contiguous leaf list (strict 2.x validation) | AddPair/SlowExplosion improved 2–3%, but changed deterministic iteration order and validation trajectories. Revisited for the next major release below. |
| Direct leaf-array pair marking | Removed internal-node traversal, but changed contact order too much: MildN2 energy/contact counts exceeded the v4 behavior envelope. |
| Reuse one BBTree query stack per marking pass | Byte-exact and improved Diagonal/SlowExplosion, but repeated broad runs regressed MixedStaticDynamic/FrictionalPyramid about 0.3–0.4%. |
| Cache master-tree stamp in marking context | Byte-exact and SlowExplosion -1.2%, but AddPair +1.5%, N2 +0.5%, and MostlyStatic +1.2%. |
| Cache area in every node | Diagonal/SlowExplosion improved 1–2%, MixedStaticDynamic regressed and nodes grew. |
| Manual `MarkLeafQuery` intersection | Neutral; Diagonal regressed about 1%. |
| Stop ancestor updates when bounds already contain leaf | Small/noisy; no general win. |

## Rejected: solver and data layout

| Experiment | Outcome / reason |
|---|---|
| Direct scalar vector expansion | Changed floating-point evaluation order and trajectories. |
| `restrict` body/arbiter pointers | Mixed; FallingSquares regressed about 0.7%. |
| Cache arbiter array/count outside loops | Near neutral; no primary-workload win. |
| Force contact-loop unrolling | Small mixed changes; FallingSquares did not improve. |
| Explicit one-/two-contact solver paths | One-contact cases improved, FallingSquares regressed 0.3–0.6%. |
| Cache tangent / hoist `cpvperp(n)` | Compiler already did the useful work; neutral. |
| Local body velocity/bias copies with one write-back | Realistic friction/callback/sleep scenes regressed 2–4%. |
| Combined normal/tangent effective-mass calculation | Changed floating-point order; exact validation failed. |
| Inline two contacts in `cpArbiter` | Arbiter grew from 184 to about 376 bytes; contact workloads regressed 1–2%. |
| Reorder `cpBody` hot fields | No general win; BigMobile/SlowExplosion regressed about 1%. |
| Reorder `cpArbiter` hot fields | Neutral/mixed; callback workloads regressed. |
| Reorder `cpContact` hot fields | Neutral; no robust improvement. |
| Move contact hashes out of `cpContact` | Contact shrank 96 to 88 bytes but arbiter grew; callbacks regressed about 1%. |
| Direct/default body integrator calls | SleepWake improved, several contact workloads regressed about 0.5%. |
| Cache damping and collision-bias powers | Mixed and required extra `cpSpace` state. |
| Unit-damping `pow` fast path | Too small/mixed. |
| Skip default `separate` callbacks | About 1% at one SleepWake size; gain vanished when scaled. |
| Explicit impulse helper expansion | Helped callback/sleep cases, neutral or negative elsewhere. |
| Frictionless solver omitting `surface_velocity` | Improved zero-friction cases about 0.3–1.1% and was neutral in SurfaceVelocity, but a sloped control changed floating-point results. |
| Early-stop generic hash filtering after visiting all entries | SleepWake -1.7% once, but scaling was neutral/slower; rejected. |
| Dense cached-arbiter list for sequential filtering | SleepWake/AddPair improved 3–4%, but Multifixture regressed 1.1% and SlowExplosion about 1%; duplicate array/hash maintenance was workload-dependent. |
| O(1) bucket unlink during active-bin filtering | Added a bucket backpointer, but larger bins caused broad 0.1–0.7% regressions; removal chains were already short. |
| Directly unroll max-two contact persistence matching | Neutral. |
| Separate static-body solver selected per arbiter | Callback/SleepWake improved about 7.5%, but dynamic-only contact workloads regressed 1–2%. |
| Cache body solver state locally per arbiter | Realistic friction/callback/sleep scenarios regressed 2–4%. |
| Frictionless solver ignoring `surface_velocity` | Benchmarks improved up to 1.1%, but a sloped non-zero-surface-velocity control changed floating-point results. |
| Frictionless static-body specialized loops | Static-contact cases improved 3–8%, but dynamic-only cases paid dispatch/code-size overhead; no general win. |
| Explicit cached body type field | SleepWake/AddPair/N2 improved 1–2%, FrictionalPyramid/FallingSquares regressed about 0.6%. |
| Inline internal body-type derivation | SleepWake/MostlyStatic improved about 1%, Diagonal and other cases regressed or were noisy. |
| Split bias and velocity solver passes | Frictional workloads regressed 2–5%. |
| Zero-friction/zero-elasticity pre-step loop | Several zero-material cases improved 0.5–1.3%, FrictionalPyramid regressed about 0.25%. |
| Exact tangent-mass dot-product identity | Several cases improved below 1%, FrictionalPyramid remained about 0.2% slower after 50 rounds. |
| Filter cached-impulse calls in caller | Neutral. |
| Directly unroll two-contact persistence matching | Neutral. |
| Hoist all-frictionless dispatch out of solver iterations | SleepWake/CollisionCallbacks improved 0.8–1.4%, but FallingSquares/FrictionalPyramid/SurfaceVelocity regressed 0.4–0.7%. |
| Hoist empty constraint-list path out of solver iterations | SleepWake improved about 1.4%, but FrictionalPyramid/SurfaceVelocity regressed about 0.5–0.8%. |
| Prefetch next arbiter/contact/body solver data | Broad prefetch improved FallingSquares about 1.1% but regressed friction cases about 0.3%; contact-only prefetch retained a roughly 0.3% SurfaceVelocity regression; frictionless-only prefetch regressed SurfaceVelocity about 0.6%. |

## Rejected: collision, callbacks, and shape updates

| Experiment | Outcome / reason |
|---|---|
| Direct circle shape update globally | Large isolated gains, but polygon/joint workloads regressed. |
| Direct circle update only in all-circle/sparse worlds | SlowExplosion improved about 11%, but several unrelated workloads regressed slightly; bookkeeping too broad. |
| Empty collision-handler hash fast path | Some contact cases improved 0.5–1.3%, Diagonal/Multifixture mixed or worse. |
| Per-arbiter handler cache with generation | AddPair improved about 1.3%, otherwise neutral/noisy; added state. |
| Hash comparison before equality callback | Neutral overall. |
| Initialize polygon AABB from first vertex | Multifixture about -1.4%, otherwise neutral/noisy. |
| Four-vertex polygon cache-data loop | Multifixture -3.4%, but MostlyStatic regressed 0.6–0.7%. |
| Zero-elasticity bounce fast path | Neutral overall. |

## Rejected: GJK/EPA

| Experiment | Outcome / reason |
|---|---|
| Three-point EPA first-iteration specialization | Exact, but neutral/mixed. |
| Four-vertex polygon support specialization | Box scenes improved 1–3%, FallingSquares/Tumbler regressed 3–5% from code-layout effects. |
| Replace polygon edge modulo with branches | Some realistic scenes improved, FallingSquares/Tumbler regressed 0.5–0.7% on x86. |
| Pointer iteration in polygon support scan | Neutral. |
| Scalar rewrite of `ClosestDist` | Neutral/mixed. |
| Two-vertices-per-iteration polygon support scan | Callback/sleep improved, but FallingSquares +1.1% and Tumbler +2.0%. |
| Three-point EPA loop removal preserving comparison order | Exact, but no measurable gain after compilation. |
| Cache surviving EPA edge distances | Byte-exact, but shallow EPA made reuse lookup costlier; FallingSquares/AddPair regressed about 0.3–0.4%. |
| Iterative EPA with fixed/shallow hull buffers | Byte-exact and callbacks improved about 0.5%, but Multifixture regressed 1.2% and MostlyStatic 0.4%. |
| Two-at-a-time polygon support scan | Callback/sleep improved, FallingSquares/Tumbler regressed 1–2%. |
| Pair-kind switch replacing support function pointer | Broad regressions around 0.2–2%; indirect call was better predicted. |
| Iterative GJK loop | Byte-exact; friction/surface cases improved 0.3–0.6%, but AddPair regressed 0.5% and Multifixture 1.3%. |
| Explicit cached collision-ID unpack | Byte-exact and callbacks improved slightly, but Multifixture regressed about 0.8%; compiler already optimized shifts. |
| Direct scalar bounce projection | Neutral. |
| Zero-surface-velocity fast path in arbiter update | Neutral/mixed; branch cost matched saved projection work. |
| Pass contact clipping structs by const pointer | Byte-exact, but neutral/mixed; explicit edge temporaries did not reduce generated-code cost. |
| Reuse edge radius offsets in contact clipping | Byte-exact, but neutral/mixed; Multifixture regressed about 0.4%. |
| Reuse edge deltas in contact clipping | Tumbler improved 2.9%, but Multifixture/MostlyStatic regressed about 0.8–1.1%; rejected. |
| Constraint-list null guard in broad-phase rejection | Neutral. |

## Benchmark and CI changes

- [#14](https://github.com/vibloteket/munk2d/pull/14): email only changed baseline/current gallery PNGs.
- [#15](https://github.com/vibloteket/munk2d/pull/15): add FrictionalPyramid, CollisionCallbacks, and SleepWake; protocol `munkbench-v2` has 16 scenarios.
- SurfaceVelocity adds a permanent non-zero surface-velocity counterexample; protocol `munkbench-v3` has 17 scenarios.
- SurfaceVelocity adds a conveyor-style non-zero surface-velocity control; protocol `munkbench-v3` has 17 scenarios.
- A permanent CI change disabling third-party APT repositories was rejected; transient mirror failures should be rerun.

## Current profile notes

- `cpArbiterApplyImpulse` remains the largest contact-heavy hotspot (roughly 36–59% self time).
- GJK/EPA calls are usually shallow; support-point work matters more than recursion depth. FrictionalPyramid EPA exits at iteration 1 about 99.5% of the time; FallingSquares exits at iteration 1–2 about 97% of the time; CollisionCallbacks commonly reaches iteration 2–3.
- Realistic contact workloads show low L1D miss rates (roughly 0.4–1.1%); simple struct reordering has not helped.
- Focused `cpArbiter` hot-field packing kept the structure at 184 bytes but was neutral/mixed. Focused `cpContact` packing was also neutral.
