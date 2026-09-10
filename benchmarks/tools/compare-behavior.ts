#!/usr/bin/env bun
// SCRIPT_JDOC: Compare two MunkBench summary JSON files using the versioned behavior envelope.

import { resolve } from "node:path";

type Json = Record<string, any>;

type Failure = {
  benchmark: string;
  step?: number;
  metric: string;
  baseline?: unknown;
  candidate?: unknown;
  allowed?: string;
};

const args = process.argv.slice(2);
function option(name: string): string | undefined {
  const i = args.indexOf(name);
  if (i < 0) return undefined;
  if (i + 1 >= args.length) throw new Error(`Missing value for ${name}`);
  return args[i + 1];
}
function usage(): never {
  console.error("Usage: bun benchmarks/tools/compare-behavior.ts --baseline FILE --candidate FILE [--envelope FILE]");
  process.exit(2);
}
if (args.includes("--help") || args.includes("-h")) usage();
const baselinePath = option("--baseline");
const candidatePath = option("--candidate");
const envelopePath = option("--envelope") ?? "benchmarks/behavior-envelope-v4.json";
if (!baselinePath || !candidatePath) usage();

async function json(path: string): Promise<Json> {
  return JSON.parse(await Bun.file(resolve(path)).text());
}

const [baseline, candidate, envelope] = await Promise.all([
  json(baselinePath), json(candidatePath), json(envelopePath),
]);
const failures: Failure[] = [];
const warnings: Failure[] = [];
const byName = (document: Json) => new Map(document.benchmarks.map((b: Json) => [b.benchmark, b]));
const baselineByName = byName(baseline);
const candidateByName = byName(candidate);
const limits = envelope.limits;

function fail(benchmark: string, metric: string, data: Partial<Failure> = {}) {
  failures.push({ benchmark, metric, ...data });
}
function finite(value: unknown): value is number {
  return typeof value === "number" && Number.isFinite(value);
}
function tolerance(reference: number, relative: number, absolute: number): number {
  return Math.max(Math.abs(reference) * relative, absolute);
}
function compareScalar(name: string, step: number, metric: string, a: number, b: number, relative: number, absolute: number) {
  const allowed = tolerance(a, relative, absolute);
  if (!finite(b) || Math.abs(b - a) > allowed) {
    fail(name, metric, { step, baseline: a, candidate: b, allowed: `absolute difference <= ${allowed}` });
  }
}
function checkpointAt(benchmark: Json, step: number): Json | undefined {
  return benchmark.checkpoints.find((c: Json) => c.step === step);
}

for (const [name, base] of baselineByName) {
  const cand = candidateByName.get(name);
  if (!cand) {
    fail(name, "benchmark", { baseline: "present", candidate: "missing" });
    continue;
  }
  if (cand.size !== base.size) fail(name, "size", { baseline: base.size, candidate: cand.size, allowed: "exact" });
  if (cand.simulated_steps !== base.simulated_steps) fail(name, "simulated_steps", { baseline: base.simulated_steps, candidate: cand.simulated_steps, allowed: "exact" });

  for (const baseCheckpoint of base.checkpoints) {
    const step = baseCheckpoint.step;
    const current = checkpointAt(cand, step);
    if (!current) {
      fail(name, "checkpoint", { step, baseline: "present", candidate: "missing" });
      continue;
    }
    if (current.invalid_values !== 0) fail(name, "invalid_values", { step, baseline: 0, candidate: current.invalid_values, allowed: "exactly zero" });
    for (const metric of envelope.exact_integer_metrics) {
      if (current[metric] !== baseCheckpoint[metric]) fail(name, metric, { step, baseline: baseCheckpoint[metric], candidate: current[metric], allowed: "exact" });
    }
    for (const metric of envelope.finite_metrics) {
      if (!finite(current[metric])) fail(name, metric, { step, baseline: baseCheckpoint[metric], candidate: current[metric], allowed: "finite" });
    }
    if (current.max_linear_velocity > limits.max_linear_velocity) fail(name, "max_linear_velocity", { step, candidate: current.max_linear_velocity, allowed: `<= ${limits.max_linear_velocity}` });
    if (current.max_angular_velocity > limits.max_angular_velocity) fail(name, "max_angular_velocity", { step, candidate: current.max_angular_velocity, allowed: `<= ${limits.max_angular_velocity}` });

    const bodyCount = Math.max(1, baseCheckpoint.dynamic_bodies);
    for (const metric of ["sum_position", "sum_velocity"]) {
      for (let axis = 0; axis < 2; axis++) compareScalar(name, step, `${metric}[${axis}]`, baseCheckpoint[metric][axis], current[metric][axis], limits.aggregate_relative, limits.aggregate_absolute_per_body * bodyCount);
    }
    for (const metric of ["body_bounds", "shape_bounds"]) {
      if (baseCheckpoint[metric] === null || current[metric] === null) {
        if (baseCheckpoint[metric] !== current[metric]) fail(name, metric, { step, baseline: baseCheckpoint[metric], candidate: current[metric], allowed: "both null or both arrays" });
      } else {
        for (let i = 0; i < 4; i++) compareScalar(name, step, `${metric}[${i}]`, baseCheckpoint[metric][i], current[metric][i], limits.bounds_relative, limits.bounds_absolute);
      }
    }
    for (const metric of ["contact_pairs", "contact_points"]) compareScalar(name, step, metric, baseCheckpoint[metric], current[metric], limits.contact_relative, limits.contact_absolute);
    compareScalar(name, step, "total_kinetic_energy", baseCheckpoint.total_kinetic_energy, current.total_kinetic_energy, limits.energy_ratio - 1, limits.energy_absolute);
    compareScalar(name, step, "max_linear_velocity", baseCheckpoint.max_linear_velocity, current.max_linear_velocity, limits.max_velocity_ratio - 1, limits.max_velocity_absolute);
    compareScalar(name, step, "max_angular_velocity", baseCheckpoint.max_angular_velocity, current.max_angular_velocity, limits.max_velocity_ratio - 1, limits.max_velocity_absolute);
  }
}
for (const name of candidateByName.keys()) if (!baselineByName.has(name)) warnings.push({ benchmark: name, metric: "benchmark", candidate: "new", allowed: "not compared" });

const friction = candidateByName.get("FrictionalPyramid");
const frictionRule = envelope.scenario_invariants.FrictionalPyramid;
if (!friction || friction.checkpoints.at(-1)?.contact_pairs < frictionRule.final_contact_pairs_min) fail("FrictionalPyramid", "final_contact_pairs", { candidate: friction?.checkpoints.at(-1)?.contact_pairs, allowed: `>= ${frictionRule.final_contact_pairs_min}` });
const callback = candidateByName.get("CollisionCallbacks")?.benchmark_metrics;
const callbackRule = envelope.scenario_invariants.CollisionCallbacks;
for (const metric of ["begin_count", "pre_solve_count", "post_solve_count", "wildcard_count"]) if (!callback || callback[metric] < callbackRule[`${metric}_min`]) fail("CollisionCallbacks", metric, { candidate: callback?.[metric], allowed: `>= ${callbackRule[`${metric}_min`]}` });
if (callbackRule.pre_solve_greater_than_post_solve && (!callback || callback.pre_solve_count <= callback.post_solve_count)) fail("CollisionCallbacks", "pre_solve_count", { candidate: callback, allowed: "pre_solve_count > post_solve_count" });
const sleep = candidateByName.get("SleepWake");
const sleepRule = envelope.scenario_invariants.SleepWake;
const sleeping = (step: number) => checkpointAt(sleep, step)?.sleeping_bodies;
if (!sleep || sleeping(sleepRule.sleeping_bodies_min_at_step.step) < sleepRule.sleeping_bodies_min_at_step.value) fail("SleepWake", "sleeping_bodies", { step: sleepRule.sleeping_bodies_min_at_step.step, candidate: sleeping(sleepRule.sleeping_bodies_min_at_step.step), allowed: `>= ${sleepRule.sleeping_bodies_min_at_step.value}` });
if (!sleep || sleeping(sleepRule.sleeping_bodies_decrease.after) >= sleeping(sleepRule.sleeping_bodies_decrease.before)) fail("SleepWake", "sleeping_bodies_decrease", { baseline: sleeping(sleepRule.sleeping_bodies_decrease.before), candidate: sleeping(sleepRule.sleeping_bodies_decrease.after), allowed: "after < before" });

const report = { protocol: envelope.protocol, passed: failures.length === 0, compared_benchmarks: baselineByName.size, failures, warnings };
console.log(JSON.stringify(report, null, 2));
if (failures.length) process.exit(1);
