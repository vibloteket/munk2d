# MunkBench

MunkBench is the standalone Munk2D benchmark suite. It exercises a range of
simulation setups so Munk2D performance and stability changes can be measured
without demo, rendering, or Python dependencies.

The benchmark scenarios are based on the suite from the
[`box2d-optimized`](https://github.com/mtsamis/box2d-optimized) fork of Box2D,
which proposed multiple Box2D engine improvements. The scenes are useful for
tracking relative changes within Munk2D, but raw timings should not be compared
directly with Box2D results unless the resulting simulation state is also
reviewed for equivalence.

Detailed descriptions of each benchmark, their design intent, and the original
performance findings are documented in
[`BENCHMARKS-REFERENCE.md`](BENCHMARKS-REFERENCE.md), adapted from the
Box2D-optimized dissertation.

Build:

```sh
cmake -B build -S . -DBUILD_DEMOS=OFF -DBUILD_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build --target munkbench
```

MunkBench should use the project's normal optimized build without experimental
floating-point flags. Compiler flags such as `-ffast-math` can be evaluated
separately once the benchmark baseline is stable.

Run the representative reference size for every benchmark:

```sh
./build/benchmarks/munkbench --profile reference
```

`reference` is the default profile. Use `smoke` for fast CI and setup-overhead
checks, or `extended` for the original full-size workloads:

```sh
./build/benchmarks/munkbench --profile smoke
./build/benchmarks/munkbench --profile extended
```

Optimization comparisons should measure both `smoke` and `reference`. Structural
or algorithmic changes should additionally run relevant `extended` sizes or a
size sweep.

Run selected benchmarks:

```sh
./build/benchmarks/munkbench -b N2 SlowExplosion -s 100
```

Run a size sweep using each benchmark's configured range:

```sh
./build/benchmarks/munkbench -b N2 -s -1
```

For repeatable raw samples, add untimed warm-ups and use each benchmark's
calibrated batch count. Batch counts are calibrated separately for the `smoke` and `reference` profiles:

```sh
./build/benchmarks/munkbench --profile reference --warmup 2 --samples 10 --batch auto
```

An explicit batch count is useful for focused experiments:

```sh
./build/benchmarks/munkbench -b N2 --warmup 1 --samples 5 --batch 50
```

Each batched run creates, simulates, and destroys a fresh independent world. The
reported times are totals for the whole batch; divide by `batch` when a per-run
value is needed. Scenario size and physics step count are unchanged. `auto` is
calibrated for named profiles; use an explicit batch with `--size`.

Output is CSV, with one row per raw sample:

```csv
version,benchmark,size,sample,batch,init_time,run_time
```

## Isolated shape/sleep/wake lifecycle measurements

`tools/sleep-wake-lifecycle.c` is a separate POSIX microbenchmark, not a new
scenario in the versioned MunkBench protocol. It exercises the public API with
constraint-less bodies, optionally with one circle shape per body. Sleep/wake
modes use one sleeping group; a static query shape provides the callback for
locked wake. Use the same compiler, harness and build flags for both variants:

```sh
cc -O3 -DNDEBUG -Iinclude benchmarks/tools/sleep-wake-lifecycle.c \
  build/src/libchipmunk.a -lm -o build/sleep-wake-lifecycle
./build/sleep-wake-lifecycle queued 10000 3
./build/sleep-wake-lifecycle sleep 10000 3
./build/sleep-wake-lifecycle unlocked 10000 3
./build/sleep-wake-lifecycle remove 10000 3
./build/sleep-wake-lifecycle sleep 10000 3 shapes
./build/sleep-wake-lifecycle queued 10000 3 shapes
```

Arguments are mode, body count, independent repetitions and optional `shapes`.
`remove` implies shapes and measures only `cpSpaceRemoveShape` for each body;
shape freeing and body removal happen afterward, outside the timer. Other modes
remain shape-less unless `shapes` is supplied. Shapes occupy a non-overlapping
grid; no physics steps or contact solving run in this harness.

Each output row is `mode,bodies,repetitions,total_seconds,order_checksum`.
Record the `shapes` option separately with the run configuration. Divide time by
repetitions for per-operation time. `queued` includes the query, callback, queue
insertion and activation at unlock; `unlocked` is the control without the queue;
`sleep` measures the explicit sleep loop. With shapes, sleep/wake includes spatial-
index migration. Initialization, correctness checks and cleanup are excluded.
Body counts/wake state, shape membership/counts and iteration checksums must match
across variants. These timings do not predict total frame time, complete teardown
or contact-heavy wake costs. Always measure normal MunkBench controls as well,
and alternate baseline/candidate execution order.

## Isolated constraint lifecycle measurements

`tools/constraint-lifecycle.c` measures constraint bookkeeping separately from
physics stepping. `hub` gives each constraint its own dynamic body connected to
one shared static anchor; `parallel` puts all constraints between the same static
anchor and one dynamic body. Both alternate A/B orientation. No shapes or physics
steps are involved. Compile the same harness against each variant's library:

```sh
cc -O3 -DNDEBUG -Iinclude benchmarks/tools/constraint-lifecycle.c \
  build/src/libchipmunk.a -lm -o build/constraint-lifecycle
./build/constraint-lifecycle oldest hub 4096 3
./build/constraint-lifecycle newest hub 4096 3
./build/constraint-lifecycle sleep parallel 4096 3
./build/constraint-lifecycle wake parallel 4096 3
```

Arguments are mode, topology, constraint count and independent repetitions.
`oldest`/`newest` remove constraints in creation/reverse-creation order; `sleep`
measures explicit body sleep and `wake` measures subsequent activation. Setup,
validation and cleanup are outside the timer. Output is
`mode,topology,constraints,repetitions,total_seconds,order_checksum`.
Divide by repetitions for per-operation time. Compare checksums and record both
favorable operations and wake/normal-simulation controls; these are not solver
or whole-frame speedups. This optional POSIX harness does not change MunkBench's
versioned scenario protocol.

## Validation summaries

MunkBench can emit checkpoint summaries for stability/correctness validation:

```sh
./build/benchmarks/munkbench --summary-json -b N2 -s 25 --checkpoints 0,1,10,100
```

The summary run stops at the last requested checkpoint, keeping smoke tests fast.
Use `final` as a checkpoint to run through the benchmark's configured final step.
The JSON reports both configured `steps` and actual `simulated_steps`.

The JSON includes body, shape, active-constraint, contact-pair and contact-point
counts, dynamic body bounds, shape bounds, aggregate position and velocity sums,
kinetic energy, max velocities, sleeping body counts, and an `invalid_values`
count for NaN/Inf detection. `constraints` counts only constraints in the active
solver list; constraints belonging to sleeping components remain owned by the
space but are temporarily absent from that list. Scenario metrics such as
`ConstraintMix.constraint_count` report the total constraints created and owned.

### Behavior-envelope comparison

Major-version optimization work may intentionally change floating-point evaluation
or deterministic processing order. In those cases, compare summaries with the
`munkbench-v4` behavior envelope instead of requiring byte-identical trajectories:

```sh
bun benchmarks/tools/compare-behavior.ts \
  --baseline baseline-summary.json \
  --candidate candidate-summary.json
```

The envelope in `behavior-envelope-v4.json` keeps topology and finite-value checks
strict, bounds aggregate motion/contact/energy changes, and enforces scenario-specific
callback, friction, and sleep/wake behavior. It does not make every divergent
trajectory valid: candidates outside any limit fail with the metric, values, and
allowed difference in the JSON report. Repeated runs of one build should still be
byte-identical unless an explicitly non-deterministic execution mode is introduced.

Passing this envelope demonstrates bounded compatibility, not equal solver precision.
Changes that intentionally reduce iteration count, collision accuracy, convergence,
or numerical precision are quality/performance tradeoffs rather than ordinary
optimizations and require an explicit design decision before implementation. The v4
envelope is intended primarily for mathematically equivalent implementation changes
whose floating-point ordering or deterministic processing order differs.


## Constraint coverage

MunkBench includes focused, collision-free scenarios for each built-in constraint
type not isolated by the original suite: slide, pivot, groove, gear, ratchet,
rotary limit, damped spring, and damped rotary spring. `BigMobile` covers pin
joints and `Tumbler` covers a simple motor plus a pin joint.

Each focused scenario reports final and peak constraint error plus peak impulse.
This verifies that the solver is active and provides type-specific correctness
signals for future optimization work. The eight scenarios use all three size
profiles and plus an integrated `ConstraintMix` workload bring `munkbench-v5` to 26 scenarios. `ConstraintMix` interleaves all ten types with dynamic, static, and kinematic bodies, collision shapes, frictional contacts, and sleeping/wake behavior.

## SVG snapshots

A selected benchmark can be rendered to SVG for visual comparison:

```sh
./build/benchmarks/munkbench -b N2 -s 25 --svg n2-step-100.svg --step 100
```

Use `--svg -` to write the SVG to stdout.

## Recording historical results

The recording tool builds a clean revision, runs tests and behavior validation,
collects raw timing samples, and appends one immutable JSON file to the orphan
`benchmark-data` branch:

```sh
bun benchmarks/tools/record-results.ts \
  --revision master \
  --environment reference-server-v1
```

Defaults are two warm-ups, ten samples, the `reference` profile with calibrated
automatic batches, protocol `munkbench-v5`, and remote branch
`origin/benchmark-data`. Version 2 adds
non-zero-friction, collision-callback, and sleep/wake coverage. The working tree must
be clean. Use `--dry-run --output run.json` to validate and inspect a record
without committing or pushing it.

Recorded paths are organized by environment, protocol, year, timestamp, and
commit. Changing the machine, compiler baseline, benchmark workload, or timing
semantics should start a new environment or protocol series instead of silently
continuing an incompatible graph. The current versioned format is documented by
[`results-schema-v5.json`](results-schema-v5.json). Earlier schemas remain in the
repository for historical 13- and 17-scenario records.

## Constraint collision-filter scenario

`tools/constraint-filter-scenario.c` is an optional POSIX scenario for asymmetric
constraint degrees. A large dynamic circle contacts a ring of static circles.
Pin joints connect the dynamic circle and/or the shared static wall body to
separate shape-less bodies; none connects the two colliding bodies directly.
All these joints have `collideBodies=false`, so they must not suppress the ring
contacts. Default force limits, solver iterations and real contact solving remain
in use. The scenario uses zero gravity and starts the joints at their rest lengths.

```sh
cc -O3 -DNDEBUG -Iinclude benchmarks/tools/constraint-filter-scenario.c \
  build/src/libchipmunk.a -lm -o build/constraint-filter-scenario
./build/constraint-filter-scenario stationary 1 1024 128 100
./build/constraint-filter-scenario moving 1024 1 128 100
./build/constraint-filter-scenario stationary 1024 1024 128 100
```

Arguments are mode, joint count on the dynamic circle, joint count on the static
wall body, wall-circle count (a positive multiple of four), and timed steps.
`stationary` exercises cached broad-phase pairs. `moving` uses position callbacks
to move the dynamic circle and its connected bodies together between x offsets
+6 and -6, exercising leaf reinsertion; it is controlled motion, not a prediction
of an unconstrained physical trajectory. Pair argument ordering may differ
between those broad-phase paths.

Four untimed steps initialize the scene. Only subsequent `cpSpaceStep` calls are
timed, excluding construction and cleanup. Output is
`mode,dynamic_joints,static_joints,walls,steps,total_seconds,pre_solve_calls,state_hash`.
Compare callback counts and finite body-state hashes as well as timings. Include
zero-degree, balanced-degree and probe-boundary controls; this scenario does not
replace the normal MunkBench suite or change its versioned protocol.

## Polygon point-query measurements

`tools/poly-point-query.c` is an optional POSIX query benchmark. `shape`, `inside`
and `outside` query one beveled regular polygon with mixed, interior or exterior
points. `space` calls `cpSpacePointQueryNearest` over 64 static polygons with an
unbounded initial distance. All modes use the same deterministic set of 1,024
points for validation/warm-up and repeat it during timing.

```sh
cc -O3 -DNDEBUG -Iinclude benchmarks/tools/poly-point-query.c \
  build/src/libchipmunk.a -lm -o build/poly-point-query
./build/poly-point-query shape 4 1000000
./build/poly-point-query inside 16 1000000
./build/poly-point-query space 16 20000
```

Arguments are mode, vertex count (3–64), and timed query count. Output is
`mode,vertices,queries,total_seconds,result_hash,distance_sum`. The untimed hash
covers shape identity, distance, point and gradient. The timed loop also sums
distances to consume the results. Check both hash and sum across builds.
Construction, warm-up/validation and cleanup are excluded. No physics stepping
runs here; query gains do not imply equivalent full-simulation gains.
