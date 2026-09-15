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
| BBTree (next major) | Dense leaf array for sequential iteration while retaining hash lookup | AddPair -2.7%, SlowExplosion -4.6%, Multifixture -1.7%; neutral to about -0.6% elsewhere. Deterministic contact-order changes can shift sleep timing; ConstraintMix has one additional sleeping Pivot component at step 600, with all constraints still owned. | [#32](https://github.com/vibloteket/munk2d/pull/32) |
| Hash set (next major) | Intrusive active-bin list for filtering without scanning empty buckets | AddPair/MostlyStatic/SleepWake -4% to -5%; N2 -5%; neutral to about -1.5% elsewhere | [#33](https://github.com/vibloteket/munk2d/pull/33) |
| Solver | Skip zero-effect impulse writes to a static/kinematic collision body | Static-contact workloads -3% to -10%; dynamic-only controls neutral | [#40](https://github.com/vibloteket/munk2d/pull/40) |
| Pivot Joint | Reuse local anchor offsets when applying the iteration impulse | PivotConstraints about -0.8%; ConstraintMix and integration controls neutral | [#44](https://github.com/vibloteket/munk2d/pull/44) |
| Pivot/Groove Joint | Skip vector-length clamp when `maxForce` is infinite | Pivot -1.9% to -2.1%, Groove -5.0% to -6.8%; mixed/integration controls neutral | — |

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
| Cache `maxForce*dt` in six constraint structs | Byte-exact; small isolated wins were inconsistent at reference size, ConstraintMix neutral/slower, and larger structs regressed BigMobile about 1%. |
| Represent empty spring warm-start callbacks as `NULL` | Byte-exact and ConstraintMix slightly faster, but a generic null guard regressed Ratchet/Rotary Limit 3–5%. |
| Skip zero-effect Gear Joint writes with inner branches | Byte-exact, but isolated Gear and ConstraintMix were neutral/slower. |
| Reuse Damped Spring, Slide, Pin, and Groove local solver values | Byte-exact, but compilers already retained/equivalently generated most values; no robust mixed/reference win. |
| Filter inactive Ratchet/Rotary Limit constraints into a solver list | Isolated limit workloads improved 21–22%, but list/filter/code-layout overhead regressed unrelated smoke cases. |
| Switch inactive Ratchet/Rotary Limit constraints to no-op classes | Isolated limit workloads improved 6–8%, but ConstraintMix regressed about 0.2%. |
| Direct scalar vector expansion | Changed floating-point evaluation order and trajectories. |
| `restrict` body/arbiter pointers | Mixed; FallingSquares regressed about 0.7%. |
| Cache arbiter array/count outside loops | Near neutral; no primary-workload win. |
| Dense per-step arbiter solver headers | Contact-heavy zero-friction cases improved 0.7–1.0%, but header build/dispatch regressed N2/MostlyStatic/SurfaceVelocity 0.7–1.2%. |
| Partition active arbiters by friction before solving | AddPair improved 2%, but partition build/two loops regressed MostlyStatic 1.2% and MixedStaticDynamic/Multifixture 0.5–0.6%. |
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
| Reuse cached polygon normals for unchanged rigid rotation | Byte-exact and Multifixture -1.8–2.3%, but CollisionCallbacks/SurfaceVelocity regressed 0.35–0.39%. |
| Split polygon vertex/bounds and normal transform loops | Byte-exact but neutral/mixed; N2/MostlyStatic regressed about 0.2–0.4%. |
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
| Reuse final GJK support indexes for clipping edge | Full/segment-only variants improved contact workloads about 1%, but Multifixture regressed 0.6–0.9% in focused runs. |
| Hoist duplicate contact-distance guard to clipping callers | Byte-exact and small contact wins, but Multifixture regressed 1.1%. |
| Direct scalar bounce projection | Neutral. |
| Zero-surface-velocity fast path in arbiter update | Neutral/mixed; branch cost matched saved projection work. |
| Pass contact clipping structs by const pointer | Byte-exact, but neutral/mixed; explicit edge temporaries did not reduce generated-code cost. |
| Reuse edge radius offsets in contact clipping | Byte-exact, but neutral/mixed; Multifixture regressed about 0.4%. |
| Reuse edge deltas in contact clipping | Tumbler improved 2.9%, but Multifixture/MostlyStatic regressed about 0.8–1.1%; rejected. |
| Constraint-list null guard in broad-phase rejection | Neutral. |

## Constraint collision-filter measurements (2026-09-14, initial version)

- Baseline `3ec55fb`, independent of PRs #49/#50. A blocking constraint must occur in both bodies' lists. Probe at most four nodes from each side; if the other list ends, reject nothing. Otherwise finish the original A-side scan from its current position. The probe bound never truncates the full correctness decision, and the extra B-side work is bounded at four nodes. No persistent counters, allocation or layout changes.
- Compared an empty-list guard and bounded probing in a new public-API full-step scenario: a dynamic circle against a ring of 128 static circles, with unrelated pin joints attached to the dynamic circle and/or static wall body. Default solver settings and real contacts remain enabled. Stationary and controlled oscillating configurations exercise cached and reinserted pairs. Setup/cleanup are untimed; callback counts and body-state hashes match.
- Local GCC 15.2 / i5-12400T / Release-LTO / CPU 2. Initial seven-pair scenario sweep (64/256/1,024 joints plus zero/balanced controls), followed by nine-pair final-candidate and boundary checks with A/A controls. Final candidate reduced full-step time about 53% for 1 dynamic / 1,024 static joints and about 46% for the reversed counts in the moving scene. Degrees 0 and 4 on the short side retain the gain; degree 5 correctly falls back to the long scan. These are workload-specific step-time gains, not a general 2x engine-speed claim.
- A seemingly simpler counterpart-only predicate in the long tail regressed balanced high-degree cases 7–8%. Restoring the original predicate structure in the long tail eliminated that regression in the measured build; final balanced 1,024/1,024 controls were approximately 3–4% faster. Keep this failed variant documented rather than assuming fewer source-level comparisons is faster.
- Six alternating smoke/reference pairs for the guard/final candidate plus A/A (936 benchmark pairs): ordinary scenarios mostly near neutral. Final negative controls include DampedRotarySpringConstraints +0.88% smoke/+1.31% reference, Pivot +0.67% smoke and SlowExplosion +0.55% reference. ConstraintMix -0.37%/-0.90%. Small regressions are explicitly weighed against the asymmetric-scene gains.
- All 26 smoke/reference summaries are byte-exact with repeats. Tests cover both pair orientations, degrees 0/1/3/4/5/7/63, indirect and multiple direct blockers, deep blockers, same-body rejection, removal/reinsertion, setters and sleeping bodies. Release and strict ASan/LSan/UBSan tests, full smoke and targeted full-step scenarios pass. Earlier null-list experiments did not have this asymmetric-degree workload coverage.

## Short-A-first refinement (2026-09-14)

- Follow-up measurements found a small-list cost in the initial interleaved probe: roughly 1–2% in several balanced 1/1–3/3 cases. The revised implementation first completes up to four A-side nodes, returning immediately if A ends. Only long A lists trigger the bounded B-side probe, then the unchanged remaining A-side scan. Empty B still short-circuits before scanning A.
- Compared master, initial PR version and refinement together: nine alternating pairs plus A/A, stationary/moving scenes, both orientations of 1–3/0 and balanced 1/1–3/3, with both default `collideBodies=true` and false on unrelated joints. Revised balanced small-case medians were approximately -1.54% to +0.47% relative to master; the previous consistent 1–2% penalty largely disappeared. These small differences are not universal guarantees or proof of a general speedup.
- Seven-pair large/asymmetric and 4/5-node boundary follow-up retained the gains: with false on unrelated joints, stationary 1/1,024 was about -52.75%, moving 1,024/1 about -46.47%. Default-true controls also retained gains. Balanced large-list controls did not regress. All callback counts and state hashes match.
- Repeated full smoke/reference comparison (six pairs plus A/A, 624 benchmark pairs) remains mostly near neutral, with negative controls such as SlowExplosion +0.57% reference and RotaryLimit +0.81% smoke. All 26 summaries match baseline byte-for-byte with repeats. Release and strict ASan/LSan/UBSan tests/full smoke/targeted boundary scenarios pass.

## Correctness fixes discovered during optimization work

- Callback-triggered wake of a sleeping contact component could restore arbiters before contact-graph rebuilding while retaining their old links, causing assertions or corrupt graph links. `cpSpaceStep` and `cpHastySpaceStep` now unthread only arbiters appended during the collision-phase unlock before rebuilding them. Other wake/unlock phases retain their graph. Regression tests cover dynamic/dynamic and ground contacts, nested queries, pre/post-solve callbacks, and repeated sleep/wake, including single-thread HastySpace. Debug/Release and strict ASan/LSan/UBSan tests pass; unaffected targeted traces remain byte-exact. No benchmarks were run while the user's performance hold is active.

## Benchmark and CI changes

- [#14](https://github.com/vibloteket/munk2d/pull/14): email only changed baseline/current gallery PNGs.
- [#15](https://github.com/vibloteket/munk2d/pull/15): add FrictionalPyramid, CollisionCallbacks, and SleepWake; protocol `munkbench-v2` has 16 scenarios.
- SurfaceVelocity adds a permanent non-zero surface-velocity counterexample; protocol `munkbench-v3` has 17 scenarios.
- SurfaceVelocity adds a conveyor-style non-zero surface-velocity control; protocol `munkbench-v3` has 17 scenarios.
- Focused coverage for Slide, Pivot, Groove, Gear, Ratchet, Rotary Limit, Damped Spring, and Damped Rotary Spring plus integrated `ConstraintMix` brings `munkbench-v5` to 26 scenarios. BigMobile and Tumbler also cover Pin Joint and Simple Motor.
- Benchmark cleanup now wakes sleeping components before collecting constraints, fixing a baseline `ConstraintMix` smoke leak of 2,656 bytes in 11 constraints. A CTest regression covers active/sleeping constraints with and without shapes and actual mix teardown at steps 0/181/600. Empty contact summaries also avoid `qsort(NULL, 0, ...)`. All 26 smoke/reference summaries remain byte-exact; strict ASan/LSan/UBSan passes. This is a benchmark correctness fix, not an engine optimization.
- A permanent CI change disabling third-party APT repositories was rejected; transient mirror failures should be rerun.

## Sleep/wake bookkeeping measurements (2026-09-13)

- Baseline `c1fd277`, including the callback/contact-wake correctness fix in PR #47. Tested wake-queue scan removal, redundant explicit-sleep deletion removal, and their combination independently. The wake queue retains a Debug uniqueness assertion; no body/constraint layout changes or physics shortcuts are involved.
- Local i5-12400T, GCC 15.2, Release/LTO, CPU 2 affinity. Nine alternating lifecycle pairs plus A/A controls, four sizes (100/1,000/5,000/10,000), identical body counts and iteration checksums. `tools/sleep-wake-lifecycle.c` exercises the public API, timing only sleep or wake; setup, validation and cleanup are excluded. Bodies have no shapes/constraints; one static query shape triggers locked wake. These are isolated lifecycle measurements, not complete simulation speedups.
- At 10,000 bodies, the combined candidate changed queued wake from 12.320 ms to 0.113 ms (about 109x) and explicit sleep from 18.382 ms to 6.234 ms (about 2.95x). Queue-only retains the queued-wake gain; sleep-only retains the sleep gain.
- Tradeoff: the unlocked-wake control at 10,000 bodies changed from 0.0672 ms to 0.0739 ms (about +10%, roughly +7 microseconds). This must not be hidden by quoting only the favorable operations.
- Eight alternating smoke/reference MunkBench pairs plus A/A controls: ordinary contact/integration cases are mostly within roughly half a percent. The combined reference run had isolated Gear/Ratchet/Rotary Limit regressions of +1.32/+2.04/+1.71%; a separate 12-pair focused follow-up measured +0.28/+0.57/+0.02%, with substantial A/A variation. Mixed ConstraintMix was near neutral. Results are not uniformly positive and should be judged as a bounded tradeoff against the demonstrated lifecycle scaling wins, not as a universal frame-time gain.
- All three candidates passed CTest and byte-exact summaries for all 26 smoke/reference scenarios at 0,1,10,50,180,181,final, with deterministic repeats. PR #47's standard/HastySpace callback graph tests are retained. No previously rejected floating-point or quality tradeoff is included.

## Constraint-removal measurements (2026-09-13)

- Integration refresh after merging master `92876e9` (PRs #49/#51/#52): the conflict was documentation-only and all measurement sections were retained. Combined Release and strict ASan/LSan/UBSan tests pass 8/8; all 26 smoke/reference summaries are byte-identical to current master with repeats. The timing figures below remain measurements against the original baseline, not new combined-performance measurements.

- Baseline `3ec55fb`; independent of BBTree PR #49, which was not yet merged at measurement time. Tested indexed active-constraint removal, iterative per-body unlink, and their combination separately. Array removal preserves swap-with-last order; unlink preserves adjacency order and removes recursion. The index fits default 64-bit/double padding: cpConstraint stays 104 bytes, cpPinJoint 216, preSolve offset 80. Other scalar/boolean configurations use the old array path; single-precision layout remains 88/144/64 respectively.
- Same local GCC 15.2 / i5-12400T / Release-LTO / CPU 2 setup. Seven alternating lifecycle pairs plus A/A controls at 128/1,024/4,096 constraints, two topologies (parallel constraints on one body, or one constraint per body around a static hub), four phases. No shapes or physics steps; setup/validation/freeing excluded. The optional `tools/constraint-lifecycle.c` reproduces these workloads.
- At 4,096 hub constraints, combined oldest-first removal changed 36.779 -> 10.873 ms (~3.4x), newest-first 2.277 -> 0.053 ms (~40x), and sleep 2.165 -> 1.078 ms (~2x). Wake regressed 0.0274 -> 0.0379 ms (~38%, about 11 microseconds). On the more artificial parallel topology, sleep changed 2.065 -> 0.0117 ms (~177x) but wake 0.0065 -> 0.0096 ms (~48%, about 3 microseconds). Do not present these as whole-frame or solver speedups.
- Iterative unlink alone gives roughly 3x oldest-first removal but not the array-removal/sleep gains. Indexed removal has a real wake-side bookkeeping cost. The combined proposal deliberately trades that cost for much larger sleep/removal savings.
- Six alternating smoke/reference MunkBench pairs for iterative/combined plus A/A controls (936 unique pairs): combined changes mostly within a percent, with small negative controls including Slide +0.67% smoke/+0.63% reference and Diagonal +0.53% reference. ConstraintMix was -0.19%/-0.11%. No uniform speedup claim.
- All four builds pass 6/6 CTest; all three candidates give byte-exact summaries for all 26 smoke/reference scenarios with repeats. Combined strict ASan/LSan/UBSan, full smoke and all lifecycle modes pass. Added array/adjacency order, membership, removal/reinsertion and sleep/wake tests; single-precision/custom-bool fallback tests and C++ compilation pass.

## BBTree leaf-index measurements (2026-09-13)

- Baseline `3ec55fb`, after PR #48. Cache each leaf's dense-array index and use the same swap-with-last order instead of scanning for the leaf at removal. The fast path is restricted to default timestamp types with 64-bit pointers/32-bit unsigned int, where the index fits existing padding. Default Node remains 64 bytes; other configurations retain the original layout and linear path. Custom 64-bit timestamps were tested through that fallback (Node stays 64 bytes, rather than growing to 72).
- Same local GCC 15.2 / i5-12400T / Release-LTO / CPU 2 setup. Seven alternating lifecycle pairs plus A/A controls at 1,000/5,000/10,000/20,000 bodies, using the extended `tools/sleep-wake-lifecycle.c` with shapes. At 10,000, shape removal changed 6.304 -> 0.341 ms (~18.5x) and explicit sleep with shape migration 18.813 -> 12.429 ms (~34% lower). At 20,000, removal changed 24.710 -> 0.516 ms (~47.8x), sleep 71.079 -> 46.277 ms (~35% lower). These measure removal/sleep phases, not full frames or complete object destruction.
- Wake with shapes was approximately neutral/slightly slower (~0.2–1.1% in the larger tested cases). The fixture's wake order already favors the original leaf search; do not claim that every removal-containing operation becomes faster. Shape-less queued-wake control is also near neutral at 10,000.
- Six alternating smoke/reference MunkBench pairs plus A/A controls: reference contact/integration changes were within roughly 0.5%. Initial smoke Multifixture +1.18% reduced to +0.14% in a separate 12-pair focused follow-up (reference +0.03%). MostlyStaticSingleBody retained a small negative tradeoff: about +0.52% smoke / +0.38% reference in follow-up. No broad universal speedup is claimed. The old audit's large unrelated constraint regressions did not recur on this newer baseline.
- All 26 smoke/reference summaries are byte-identical, including repeated runs. Regression tests check first/middle/last/final removal, exact iteration order, membership, recycling, reindex/optimize and 512 transfers between paired static/dynamic trees. Release and strict ASan/LSan/UBSan tests pass, as do custom-timestamp fallback tests. The portability-guard change produced a byte-identical default Release executable to the measured candidate.

## Polygon point-query measurements (2026-09-14)

- Baseline `5392b7a`, including merged PRs #49/#51. `cpPolyShapePointQuery` tracks the smallest squared edge distance and only evaluates sqrt when that minimum improves. It still compares rounded distances strictly before changing the selected point/normal: different squared values can have identical rounded roots. No persistent state, layout change, approximation or reordered edge traversal.
- A direct secondary-tree BB shrink was rejected before implementation: query point (-0.5,0), a dynamic circle reporting distance0.25, and a static circle centered at2^53 with radius2^53 make the original nearest query choose the large circle at rounded distance0. A BB narrowed to0.25 excludes that original winner. This is why the query-bound shortcut cannot simply be treated as byte-exact.
- Seven alternating query pairs plus A/A controls, 3/4/8/16/64 vertices, mixed/interior/exterior points and a full static-space nearest query over64 polygons. Local GCC15.2/i5-12400T/Release-LTO/CPU2. Direct mixed queries were about13% faster at8 vertices,20% at16,8% at64;4 vertices were near neutral. Full-space nearest queries improved about3% at4 vertices and7% at16/64. Counts, result hashes and distance sums match. These are query timings, not frame-time gains.
- A separate <=4-vertex bypass was tested but not retained: it reduced the useful16-vertex gain and was unnecessary after repeated measurements. First short pilot timings were not used for acceptance; A/A variation was substantial in some tiny-query cases.
- Six alternating smoke/reference MunkBench pairs plus A/A: mostly near neutral. Initial reference SlowExplosion+1.25% and DampedRotarySpring+1.21% did not persist in a12-pair focused follow-up (+0.06% and-0.63%); ConstraintMix was+0.06% in follow-up. Small/code-layout-sensitive differences are not universal gains or costs.
- All26 smoke/reference summaries are byte-exact with repeats. Regression oracle mirrors the old implementation and checks every result field across sizes, rotations, bevels, vertices/edges, tiny distances and non-finite classifications. Explicit rounded-root tie: first edge squared distance1+2^-52 and next edge1 both round to root1; keep the first feature. Release and strict ASan/LSan/UBSan7/7, full smoke, targeted single-precision test and C++ compilation pass.

## Matching-last array deletion (2026-09-14)

- Baseline `7811bdb`, after PR #50. `cpArrayDeleteObj` checks for a matching last entry before the linear search. This preserves exact array contents even with duplicates: the original algorithm would replace the first match with an equal last value, then clear the last slot. No layout/state/allocation changes and no different removal order as observable through the array.
- Exhaustive reference comparison for all length0–5 arrays over two pointers/NULL and present/absent targets (1,456 cases), plus10,000 mixed operations, covers duplicates, empty arrays, cleared slots and growth. Release/strict ASan/LSan/UBSan9/9, full smoke and all lifecycle modes pass; all26 smoke/reference summaries remain byte-exact with repeats.
- Local GCC15.2/i5-12400T/Release-LTO/CPU2; nine alternating lifecycle pairs plus A/A, sizes1,000/5,000/10,000. Public body APIs, no shapes/constraints, setup/cleanup excluded. At10,000: forward removal6.190->3.135ms, reverse removal12.226->0.0917ms, grouped sleep6.235->3.166ms, reverse component wake12.225->0.0823ms. These are ordered lifecycle-operation gains, not universal frame-time improvements.
- Controls at10,000: always-first removal0.0979->0.0978ms, random removal6.262->6.282ms (paired median near zero), automatic-sleep step0.3159->0.3151ms. Smaller first/auto-sleep controls showed small costs up to roughly0.8%; no claim that every order improves.
- Six alternating smoke/reference MunkBench pairs plus A/A: contact/integration scenes mostly within a few tenths of a percent. Initial isolated DampedRotarySpring smoke+1.63% and Ratchet reference+1.25% were not stable in a12-pair follow-up; follow-up DampedRotary reference+1.00% coincided with A/A+1.40%. ConstraintMix remained near neutral. Keep such tiny/layout-sensitive changes separate from the demonstrated elimination of long array scans.

## Positive-radius segment queries: correctness fix (2026-09-15)

- Separate from the segment-query performance prototype and nearest-point PR #54. Reproduced on `280c011` and independently confirmed by the user in Pymunk: circle at (0,0), radius1; sweep (-3,1.5) to (3,1.5), query radius0.75. Direct shape query hits, but both space queries can miss after adding an unrelated circle at (100,100). Spatial hash also misses. Cause: centreline-only candidate selection.
- BBTree now expands queried node bounds by the positive query radius, preserving its near-first traversal and first-hit clipping. The zero-radius entry passes zero without expanding bounds. No public API, spatial-index vtable, or persistent-layout changes.
- Other indexes use the swept AABB and the original precise shape query. Spatial hash falls back to enumerating objects when cell coordinates are unsafe or the query would visit more cells than indexed objects; this also avoids pathological large empty hash rectangles. Non-finite bounds use object enumeration.
- Positive-radius all-hit queries retain endpoint hits by traversing with an infinite cutoff; first-hit queries retain their existing strict alpha<1 rule and static-before-dynamic priority. Callback order remains unspecified. Shape intersection math, filters, sensors and lock/post-step behavior are unchanged.
- Rejected an AABB-only implementation for BBTree: in a pilot, long mixed positive-radius first-hit queries took about21.5 microseconds versus0.67 microseconds with expanded-node traversal. Both produced the same result hash in that fixture. Comparisons against the broken old positive-radius path are not valid speedup claims because its results differ.
- Regression tests fail before the fix and pass after it. Tests compare all returned records and minimum hit fraction against direct shape queries across BBTree/hash, static/dynamic, circles/segments/rounded boxes/16-gons, reversed/diagonal/zero-length sweeps, tangencies, endpoints, starts inside, filters/sensors, ties, duplicate suppression, post-step delivery, very large and infinite radii.
- Release and strict ASan/UBSan:10/10 CTests; float radius regression and C++11 regression pass. All26 smoke/reference summaries exact twice each. Sixty zero-radius ordered candidate traces and returned-result hashes match baseline.
- Zero-radius controls (seven AB/BA+A/A pairs, GCC15.2 Release/LTO, CPU2, mixed1024): most measured deltas -0.25..-5.52%; very cheap outside-scene misses +0.78% first /+3.30% all (sub-nanosecond costs). No separate performance optimization or whole-frame gain is claimed.

## Current profile notes

- `cpArbiterApplyImpulse` remains the largest contact-heavy hotspot (roughly 36–59% self time).
- GJK/EPA calls are usually shallow; support-point work matters more than recursion depth. FrictionalPyramid EPA exits at iteration 1 about 99.5% of the time; FallingSquares exits at iteration 1–2 about 97% of the time; CollisionCallbacks commonly reaches iteration 2–3.
- Realistic contact workloads show low L1D miss rates (roughly 0.4–1.1%); simple struct reordering has not helped.
- Focused `cpArbiter` hot-field packing kept the structure at 184 bytes but was neutral/mixed. Focused `cpContact` packing was also neutral.
