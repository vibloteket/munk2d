#define _POSIX_C_SOURCE 200809L
#include "chipmunk/chipmunk.h"
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifdef COUNT_QUERIES
#include "chipmunk/chipmunk_private.h"
static cpShapeClass counted[3];
static const cpShapeClass *original[3];
static uint64_t pointCalls, outsideStarts, segmentCalls, segmentHits, discardedHits;
static uint64_t traceHash = 1469598103934665603ULL;
static int firstMode;
static cpFloat currentRadius, bestAlpha;
static void countPoint(const cpShape *shape, cpVect p, cpPointQueryInfo *info) {
  pointCalls++;
  traceHash = (traceHash ^ (uint64_t)(uintptr_t)cpShapeGetUserData(shape)) * 1099511628211ULL;
  original[shape->klass->type]->pointQuery(shape, p, info);
  if (info->distance <= currentRadius) {
    if (firstMode)
      bestAlpha = 0;
  } else
    outsideStarts++;
}
static void countSegment(const cpShape *shape, cpVect a, cpVect b, cpFloat radius,
                         cpSegmentQueryInfo *info) {
  segmentCalls++;
  original[shape->klass->type]->segmentQuery(shape, a, b, radius, info);
  if (info->shape) {
    segmentHits++;
    if (firstMode) {
      if (info->alpha < bestAlpha)
        bestAlpha = info->alpha;
      else
        discardedHits++;
    }
  }
}
#endif
static void check(int ok) {
  if (!ok) {
    fputs("Segment benchmark validation failed\n", stderr);
    exit(1);
  }
}
static double now(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec + t.tv_nsec * 1e-9;
}
static uint32_t rnd(uint32_t *s) {
  *s ^= *s << 13;
  *s ^= *s >> 17;
  *s ^= *s << 5;
  return *s;
}
typedef struct Sink {
  uint64_t count, hash;
  double sum;
  int hashing;
} Sink;
static void collect(cpShape *shape, cpVect p, cpVect normal, cpFloat alpha, void *data) {
  Sink *s = (Sink *)data;
  s->count++;
  s->sum += alpha;
  if (s->hashing) {
    double values[] = {alpha, p.x, p.y, normal.x, normal.y};
    for (int i = 0; i < 5; i++) {
      uint64_t bits;
      check(isfinite(values[i]));
      memcpy(&bits, &values[i], sizeof(bits));
      s->hash = (s->hash ^ bits) * 1099511628211ULL;
    }
    s->hash = (s->hash ^ (uint64_t)(uintptr_t)cpShapeGetUserData(shape)) * 1099511628211ULL;
  }
}
static void query(cpSpace *space, cpVect a, cpVect b, cpFloat radius, int first, Sink *sink) {
  if (!first)
    cpSpaceSegmentQuery(space, a, b, radius, CP_SHAPE_FILTER_ALL, collect, sink);
  else {
    cpSegmentQueryInfo info;
    if (cpSpaceSegmentQueryFirst(space, a, b, radius, CP_SHAPE_FILTER_ALL, &info))
      collect((cpShape *)info.shape, info.point, info.normal, info.alpha, sink);
  }
}
int main(int argc, char **argv) {
  check(argc == 7);
  const char *api = argv[1], *kind = argv[2], *pattern = argv[4];
  int n = atoi(argv[3]), queries = atoi(argv[6]);
  cpFloat radius = strtod(argv[5], NULL);
  int first = !strcmp(api, "first");
  check(first || !strcmp(api, "all"));
  check(n > 0 && queries > 0);
  int type = !strcmp(kind, "circle")    ? 0
             : !strcmp(kind, "segment") ? 1
             : !strcmp(kind, "box")     ? 2
             : !strcmp(kind, "poly16")  ? 3
             : !strcmp(kind, "mixed")   ? 4
                                        : -1;
  check(type >= 0);
  cpSpace *space = cpSpaceNew();
  cpBody *dynamic = cpSpaceAddBody(space, cpBodyNew(1, 1));
  cpShape **shapes = (cpShape **)calloc(n, sizeof(*shapes));
  check(shapes != NULL);
  int columns = (int)ceil(sqrt((double)n)), rows = (n + columns - 1) / columns;
  for (int i = 0; i < n; i++) {
    cpVect center = cpv(4 * (i % columns), 4 * (i / columns));
    cpBody *body = i % 2 ? dynamic : cpSpaceGetStaticBody(space);
    int t = type == 4 ? i % 4 : type;
    if (t == 0)
      shapes[i] = cpCircleShapeNew(body, 1, center);
    else if (t == 1)
      shapes[i] =
          cpSegmentShapeNew(body, cpvadd(center, cpv(-1, -1)), cpvadd(center, cpv(1, 1)), 0.125);
    else if (t == 2)
      shapes[i] = cpBoxShapeNew2(
          body, cpBBNew(center.x - 1, center.y - 1, center.x + 1, center.y + 1), 0.125);
    else {
      cpVect vertices[16];
      for (int j = 0; j < 16; j++)
        vertices[j] = cpvadd(center, cpvforangle(6.2831853071795864769 * j / 16 + 0.173));
      shapes[i] = cpPolyShapeNewRaw(body, 16, vertices, 0.125);
    }
    cpSpaceAddShape(space, shapes[i]);
    cpShapeSetUserData(shapes[i], (cpDataPointer)(uintptr_t)(i + 1));
    cpShapeSetSensor(shapes[i], i % 5 == 0);
  }
#ifdef MOVED_SNAPSHOT
  cpBodySetPosition(dynamic, cpv(1.125, -0.625));
  cpBodySetVelocity(dynamic, cpv(8, -3));
  cpSpaceReindexShapesForBody(space, dynamic);
  cpBodySetPosition(dynamic, cpv(-0.875, 1.375));
  cpBodySetVelocity(dynamic, cpv(-4, 9));
  cpSpaceReindexShapesForBody(space, dynamic);
#endif
  cpVect starts[256], ends[256];
  uint32_t state = 123456789;
  for (int i = 0; i < 256; i++) {
    double u = rnd(&state) / 4294967296.0, v = rnd(&state) / 4294967296.0;
    cpFloat extent = 4 * cpfmax(columns, rows);
    if (!strcmp(pattern, "short")) {
      starts[i] = cpv((4 * columns - 4) * u, (4 * rows - 4) * v);
      ends[i] =
          cpvadd(starts[i], cpvmult(cpvforangle(6.2831853071795864769 * (i + 0.371) / 256), 8));
    } else if (!strcmp(pattern, "long")) {
      starts[i] = cpv(-4, (4 * rows - 4) * u);
      ends[i] = cpv(4 * columns, (4 * rows - 4) * v);
    } else if (!strcmp(pattern, "horizontal")) {
      starts[i] = cpv(-4, (4 * rows - 4) * u);
      ends[i] = cpv(4 * columns, starts[i].y);
    } else if (!strcmp(pattern, "vertical")) {
      starts[i] = cpv((4 * columns - 4) * u, -4);
      ends[i] = cpv(starts[i].x, 4 * rows);
    } else if (!strcmp(pattern, "fan")) {
      starts[i] = cpv(4 * (columns / 2) + 2.13, 4 * (rows / 2) + 2.21);
      ends[i] = cpvadd(starts[i],
                       cpvmult(cpvforangle(6.2831853071795864769 * (i + 0.371) / 256), extent));
    } else if (!strcmp(pattern, "miss")) {
      starts[i] = cpv(-8, -8 - u);
      ends[i] = cpv(4 * columns, -8 - v);
    } else {
      check(0);
    }
  }
  Sink warm = {0, 1469598103934665603ULL, 0, 1};
  for (int i = 0; i < 256; i++)
    query(space, starts[i], ends[i], radius, first, &warm);
  Sink timed = {0, 0, 0, 0};
  double start = now();
  for (int i = 0; i < queries; i++)
    query(space, starts[i % 256], ends[i % 256], radius, first, &timed);
  double elapsed = now() - start;
  check(isfinite(timed.sum));
  printf("%s,%s,%d,%s,%.17g,%d,%.9f,%" PRIu64 ",%" PRIu64 ",%.17g\n", api, kind, n, pattern,
         (double)radius, queries, elapsed, warm.hash, timed.count, timed.sum);
#ifdef COUNT_QUERIES
  firstMode = first;
  currentRadius = radius;
  for (int i = 0; i < n; i++) {
    int t = shapes[i]->klass->type;
    original[t] = shapes[i]->klass;
  }
  for (int t = 0; t < 3; t++)
    if (original[t]) {
      counted[t] = *original[t];
      counted[t].pointQuery = countPoint;
      counted[t].segmentQuery = countSegment;
    }
  for (int i = 0; i < n; i++)
    shapes[i]->klass = &counted[shapes[i]->klass->type];
  Sink measured = {0, 1469598103934665603ULL, 0, 1};
  for (int i = 0; i < 256; i++) {
    bestAlpha = 1;
    traceHash = (traceHash ^ (uint64_t)i) * 1099511628211ULL;
    query(space, starts[i], ends[i], radius, first, &measured);
  }
  check(measured.hash == warm.hash && measured.count == warm.count && measured.sum == warm.sum);
  printf("COUNTERS,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu32
         ",%" PRIu32 "\n",
         pointCalls, outsideStarts, segmentCalls, segmentHits, discardedHits, measured.count,
         (uint32_t)(traceHash >> 32), (uint32_t)traceHash);
  for (int i = 0; i < n; i++)
    shapes[i]->klass = original[shapes[i]->klass->type];
#endif
  for (int i = 0; i < n; i++) {
    cpSpaceRemoveShape(space, shapes[i]);
    cpShapeFree(shapes[i]);
  }
  cpSpaceRemoveBody(space, dynamic);
  cpBodyFree(dynamic);
  cpSpaceFree(space);
  free(shapes);
  return 0;
}
