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

Run the default size for every benchmark:

```sh
./build/benchmarks/munkbench
```

Run selected benchmarks:

```sh
./build/benchmarks/munkbench -b N2 SlowExplosion -s 100
```

Run a size sweep using each benchmark's configured range:

```sh
./build/benchmarks/munkbench -b N2 -s -1
```

For repeatable raw samples, add untimed warm-ups and use each benchmark's
calibrated batch count. Current batch counts target roughly 100 ms per sample at
the configured default size on the calibration host:

```sh
./build/benchmarks/munkbench --warmup 2 --samples 10 --batch auto
```

An explicit batch count is useful for focused experiments:

```sh
./build/benchmarks/munkbench -b N2 --warmup 1 --samples 5 --batch 50
```

Each batched run creates, simulates, and destroys a fresh independent world. The
reported times are totals for the whole batch; divide by `batch` when a per-run
value is needed. Scenario size and physics step count are unchanged. `auto` is
calibrated for default sizes; use an explicit batch when running another size.

Output is CSV, with one row per raw sample:

```csv
version,benchmark,size,sample,batch,init_time,run_time
```

## Validation summaries

MunkBench can emit checkpoint summaries for stability/correctness validation:

```sh
./build/benchmarks/munkbench --summary-json -b N2 -s 25 --checkpoints 0,1,10,100
```

The summary run stops at the last requested checkpoint, keeping smoke tests fast.
Use `final` as a checkpoint to run through the benchmark's configured final step.
The JSON reports both configured `steps` and actual `simulated_steps`.

The JSON includes body, shape, constraint, contact-pair and contact-point counts,
dynamic body bounds, shape bounds, aggregate position and velocity sums, kinetic
energy, max velocities, sleeping body counts, and an `invalid_values` count for
NaN/Inf detection.

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

Defaults are two warm-ups, ten samples, calibrated automatic batches, protocol
`munkbench-v3`, and remote branch `origin/benchmark-data`. Version 2 adds
non-zero-friction, collision-callback, and sleep/wake coverage. The working tree must
be clean. Use `--dry-run --output run.json` to validate and inspect a record
without committing or pushing it.

Recorded paths are organized by environment, protocol, year, timestamp, and
commit. Changing the machine, compiler baseline, benchmark workload, or timing
semantics should start a new environment or protocol series instead of silently
continuing an incompatible graph. The current versioned format is documented by
[`results-schema-v3.json`](results-schema-v3.json). The v1 and v2 schemas remain in the
repository for historical 13-scenario records.
