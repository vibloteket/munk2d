#!/usr/bin/env bun
// SCRIPT_JDOC: Record one versioned MunkBench run as append-only JSON for the benchmark-data branch.

import { mkdir, rm } from "node:fs/promises";
import { basename, resolve } from "node:path";

const SCHEMA_VERSION = 3;
const args = process.argv.slice(2);

function option(name: string, fallback?: string): string | undefined {
  const i = args.indexOf(name);
  if (i < 0) return fallback;
  if (i + 1 >= args.length) throw new Error(`Missing value for ${name}`);
  return args[i + 1];
}

function has(name: string): boolean {
  return args.includes(name);
}

function usage(): never {
  console.error(
    `Usage: bun benchmarks/tools/record-results.ts [options]\n\nOptions:\n  --revision REF       Revision to record (default: HEAD)\n  --environment ID     Stable environment series ID (default: hostname)\n  --protocol ID        Benchmark protocol ID (default: munkbench-v3)\n  --samples N          Raw samples per benchmark (default: 10)\n  --warmup N           Untimed warmups (default: 2)\n  --data-branch NAME   Results branch (default: benchmark-data)\n  --remote NAME        Git remote (default: origin)\n  --output PATH        Also copy generated run JSON to PATH\n  --dry-run            Generate and validate without committing/pushing\n`,
  );
  process.exit(2);
}

if (has("--help") || has("-h")) usage();

const revision = option("--revision", "HEAD")!;
const environmentArg = option("--environment");
const protocol = option("--protocol", "munkbench-v3")!;
const samples = Number(option("--samples", "10"));
const warmup = Number(option("--warmup", "2"));
const dataBranch = option("--data-branch", "benchmark-data")!;
const remote = option("--remote", "origin")!;
const output = option("--output");
const dryRun = has("--dry-run");
if (
  !Number.isInteger(samples) ||
  samples < 1 ||
  !Number.isInteger(warmup) ||
  warmup < 0
)
  usage();

async function run(
  cmd: string[],
  cwd = process.cwd(),
  env?: Record<string, string>,
): Promise<string> {
  const proc = Bun.spawn(cmd, {
    cwd,
    env: { ...process.env, ...env },
    stdout: "pipe",
    stderr: "inherit",
  });
  const stdout = await new Response(proc.stdout).text();
  const code = await proc.exited;
  if (code !== 0) throw new Error(`${cmd.join(" ")} exited with ${code}`);
  return stdout.trim();
}

async function readText(path: string): Promise<string | null> {
  const file = Bun.file(path);
  return (await file.exists()) ? await file.text() : null;
}

function slug(value: string): string {
  const result = value
    .trim()
    .toLowerCase()
    .replace(/[^a-z0-9._-]+/g, "-")
    .replace(/^-+|-+$/g, "");
  if (!result) throw new Error(`Invalid empty identifier from: ${value}`);
  return result;
}

function parseCsv(text: string): Record<string, string>[] {
  const lines = text.trim().split(/\r?\n/);
  const headers = lines.shift()!.split(",");
  return lines
    .filter(Boolean)
    .map((line) =>
      Object.fromEntries(
        line.split(",").map((value, i) => [headers[i], value]),
      ),
    );
}

function quantile(values: number[], p: number): number {
  const sorted = [...values].sort((a, b) => a - b);
  const position = (sorted.length - 1) * p;
  const low = Math.floor(position),
    high = Math.ceil(position);
  return sorted[low] + (sorted[high] - sorted[low]) * (position - low);
}

const repo = resolve(process.cwd());
const statusLines = (await run(["git", "status", "--porcelain"], repo))
  .split(/\r?\n/)
  .filter(Boolean);
const allowedDuringDryRun = [
  ".github/workflows/build.yml",
  "benchmarks/BENCHMARKS-REFERENCE.md",
  "benchmarks/README.md",
  "benchmarks/munkbench.c",
  "benchmarks/results-schema-v3.json",
  "benchmarks/tools",
];
if (
  statusLines.some(
    (line) =>
      !dryRun ||
      !allowedDuringDryRun.some((path) =>
        line.slice(2).trimStart().startsWith(path),
      ),
  )
)
  throw new Error("Working tree must be clean before recording");
const commit = await run(["git", "rev-parse", `${revision}^{commit}`], repo);
const sourceDate = await run(
  ["git", "show", "-s", "--format=%cI", commit],
  repo,
);
const tags = (await run(["git", "tag", "--points-at", commit], repo))
  .split(/\r?\n/)
  .filter(Boolean);
const timestamp = new Date().toISOString();
const hostname = (await run(["hostname"])).trim();
const environmentId = slug(environmentArg ?? hostname);
const cpuModel =
  (await readText("/proc/cpuinfo"))
    ?.match(/^model name\s*:\s*(.+)$/m)?.[1]
    ?.trim() ?? "unknown";
const cpuQuota = (await readText("/sys/fs/cgroup/cpu.max"))?.trim() ?? null;
const compiler = (await run(["cc", "--version"])).split(/\r?\n/)[0];
const cmake = (await run(["cmake", "--version"])).split(/\r?\n/)[0];
const loadAverage =
  (await readText("/proc/loadavg"))
    ?.trim()
    .split(/\s+/)
    .slice(0, 3)
    .map(Number) ?? [];

const tempRoot = resolve(repo, ".benchmark-recording");
const sourceDir = resolve(tempRoot, "source");
const buildDir = resolve(tempRoot, "build");
const dataDir = resolve(tempRoot, "data");
await rm(tempRoot, { recursive: true, force: true });
await mkdir(tempRoot, { recursive: true });

try {
  await run(["git", "worktree", "add", "--detach", sourceDir, commit], repo);
  await run(
    [
      "cmake",
      "-S",
      sourceDir,
      "-B",
      buildDir,
      "-DCMAKE_BUILD_TYPE=Release",
      "-DBUILD_DEMOS=OFF",
      "-DBUILD_TESTING=ON",
      "-DBUILD_BENCHMARKS=ON",
      "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
    ],
    repo,
  );
  await run(["cmake", "--build", buildDir, "--parallel", "2"], repo);
  await run(["ctest", "--test-dir", buildDir, "--output-on-failure"], repo);
  const compileCommands = await Bun.file(
    resolve(buildDir, "compile_commands.json"),
  ).text();
  if (compileCommands.includes("-ffast-math"))
    throw new Error("Refusing to record a build containing -ffast-math");

  const executable = resolve(
    buildDir,
    "benchmarks",
    process.platform === "win32" ? "munkbench.exe" : "munkbench",
  );
  const validationText = await run(
    [executable, "--summary-json", "--checkpoints", "0,1,10,50,180,181"],
    sourceDir,
  );
  const validation = JSON.parse(validationText);
  const validationByName = new Map(
    validation.benchmarks.map((benchmark: any) => [
      benchmark.benchmark,
      benchmark,
    ]),
  );
  const callbackMetrics = (validationByName.get("CollisionCallbacks") as any)
    ?.benchmark_metrics;
  const sleepCheckpoints =
    (validationByName.get("SleepWake") as any)?.checkpoints ?? [];
  const sleepingAt = (step: number) =>
    sleepCheckpoints.find((checkpoint: any) => checkpoint.step === step)
      ?.sleeping_bodies;
  const validationOk =
    validation.benchmarks.length === 17 &&
    validation.benchmarks.every((b: any) =>
      b.checkpoints.every((c: any) => c.invalid_values === 0),
    ) &&
    callbackMetrics?.begin_count > 0 &&
    callbackMetrics?.pre_solve_count > callbackMetrics?.post_solve_count &&
    callbackMetrics?.wildcard_count > 0 &&
    sleepingAt(50) > 0 &&
    sleepingAt(181) < sleepingAt(180);
  if (!validationOk) throw new Error("Behavior validation failed");

  const csv = await run(
    [
      "taskset",
      "-c",
      "0",
      executable,
      "--warmup",
      String(warmup),
      "--samples",
      String(samples),
      "--batch",
      "auto",
    ],
    sourceDir,
  );
  const rows = parseCsv(csv);
  const names = [...new Set(rows.map((row) => row.benchmark))];
  if (names.length !== 17)
    throw new Error(`Expected 17 benchmarks, got ${names.length}`);
  const benchmarks = names.map((name) => {
    const selected = rows.filter((row) => row.benchmark === name);
    if (selected.length !== samples)
      throw new Error(
        `${name}: expected ${samples} samples, got ${selected.length}`,
      );
    const batch = Number(selected[0].batch);
    const runTotals = selected.map((row) => Number(row.run_time));
    const initTotals = selected.map((row) => Number(row.init_time));
    return {
      name,
      size: Number(selected[0].size),
      batch,
      samples: selected.map((row) => ({
        sample: Number(row.sample),
        init_time_s: Number(row.init_time),
        run_time_s: Number(row.run_time),
      })),
      summary: {
        median_run_time_s: quantile(runTotals, 0.5) / batch,
        p05_run_time_s: quantile(runTotals, 0.05) / batch,
        p95_run_time_s: quantile(runTotals, 0.95) / batch,
        median_init_time_s: quantile(initTotals, 0.5) / batch,
      },
    };
  });

  const runId = `${timestamp.replace(/[:.]/g, "-")}_${commit.slice(0, 12)}`;
  const record = {
    schema_version: SCHEMA_VERSION,
    run_id: runId,
    timestamp,
    revision: { commit, source_date: sourceDate, tags },
    harness: { protocol, commit },
    environment: {
      id: environmentId,
      hostname,
      cpu_model: cpuModel,
      cpu_quota: cpuQuota,
      compiler,
      cmake,
      platform: `${process.platform}-${process.arch}`,
      load_average_start: loadAverage,
    },
    configuration: {
      build_type: "Release",
      extra_c_flags: [],
      warmup,
      samples,
      batch: "auto",
      cpu_affinity: "0",
    },
    validation: { passed: true, checkpoints: "0,1,10,50,180,181" },
    benchmarks,
  };

  const year = timestamp.slice(0, 4);
  const relativeRunPath = `runs/${environmentId}/${slug(protocol)}/${year}/${runId}.json`;
  const json = JSON.stringify(record, null, 2) + "\n";
  if (output) await Bun.write(resolve(repo, output), json);
  if (dryRun) {
    console.log(
      JSON.stringify(
        { dry_run: true, run: relativeRunPath, benchmarks: benchmarks.length },
        null,
        2,
      ),
    );
  } else {
    const remoteBranchExists =
      Bun.spawnSync(
        ["git", "ls-remote", "--exit-code", "--heads", remote, dataBranch],
        { cwd: repo },
      ).exitCode === 0;
    if (remoteBranchExists) {
      await run(
        [
          "git",
          "worktree",
          "add",
          "--detach",
          dataDir,
          `${remote}/${dataBranch}`,
        ],
        repo,
      );
    } else {
      await run(["git", "worktree", "add", "--detach", dataDir, commit], repo);
      await run(["git", "checkout", "--orphan", dataBranch], dataDir);
      await run(["git", "rm", "-rf", "."], dataDir);
      await Bun.write(
        resolve(dataDir, "README.md"),
        "# MunkBench data\n\nAppend-only raw benchmark results for vibloteket/munk2d.\n",
      );
    }
    const destination = resolve(dataDir, relativeRunPath);
    if (await Bun.file(destination).exists())
      throw new Error(`Run already exists: ${relativeRunPath}`);
    await mkdir(resolve(destination, ".."), { recursive: true });
    await Bun.write(destination, json);
    await Bun.write(resolve(dataDir, "schema-version"), `${SCHEMA_VERSION}\n`);
    await run(["git", "add", "README.md", "schema-version", "runs"], dataDir);
    await run(
      [
        "git",
        "commit",
        "-m",
        `benchmark: record ${commit.slice(0, 12)} on ${environmentId}`,
      ],
      dataDir,
    );
    await run(["git", "push", remote, `HEAD:${dataBranch}`], dataDir);
    console.log(
      JSON.stringify(
        { recorded: true, branch: dataBranch, run: relativeRunPath },
        null,
        2,
      ),
    );
  }
} finally {
  for (const dir of [dataDir, sourceDir]) {
    if (
      Bun.spawnSync(["git", "worktree", "list", "--porcelain"], { cwd: repo })
        .stdout.toString()
        .includes(`worktree ${dir}`)
    ) {
      Bun.spawnSync(["git", "worktree", "remove", "--force", dir], {
        cwd: repo,
        stdout: "inherit",
        stderr: "inherit",
      });
    }
  }
  await rm(tempRoot, { recursive: true, force: true });
}
