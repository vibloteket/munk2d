#define _POSIX_C_SOURCE 200809L
#include "chipmunk/chipmunk.h"
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
static void check(int ok) {
  if (!ok) {
    fputs("Point-query benchmark validation failed\n", stderr);
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
static void collect(cpShape *shape, cpVect p, cpFloat d, cpVect g, void *data) {
  Sink *s = (Sink *)data;
  s->count++;
  s->sum += d;
  if (s->hashing) {
    double values[] = {d, p.x, p.y, g.x, g.y};
    for (int i = 0; i < 5; i++) {
      uint64_t bits;
      check(isfinite(values[i]));
      memcpy(&bits, &values[i], sizeof(bits));
      s->hash = (s->hash ^ bits) * 1099511628211ULL;
    }
    s->hash = (s->hash ^ (uint64_t)(uintptr_t)cpShapeGetUserData(shape)) * 1099511628211ULL;
  }
}
static void query(cpSpace *space, cpShape **shapes, int n, cpVect p, int mode, cpFloat limit,
                  int index, Sink *sink) {
  if (mode == 0)
    cpSpacePointQuery(space, p, limit, CP_SHAPE_FILTER_ALL, collect, sink);
  else {
    cpPointQueryInfo info;
    if (mode == 1)
      cpSpacePointQueryNearest(space, p, limit, CP_SHAPE_FILTER_ALL, &info);
    else
      cpShapePointQuery(shapes[index % n], p, &info);
    if (info.shape)
      collect((cpShape *)info.shape, info.point, info.distance, info.gradient, sink);
  }
}
int main(int argc, char **argv) {
  check(argc == 6);
  const char *api = argv[1], *kind = argv[2];
  int n = atoi(argv[3]), queries = atoi(argv[5]);
  cpFloat limit = (cpFloat)strtod(argv[4], NULL);
  int mode = !strcmp(api, "all") ? 0 : !strcmp(api, "nearest") ? 1 : !strcmp(api, "shape") ? 2 : -1;
  check(mode >= 0 && n > 0 && queries > 0);
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
  cpVect points[256];
  uint32_t state = 123456789;
  for (int i = 0; i < 256; i++)
    points[i] = cpv((4 * columns + 4) * (rnd(&state) / 4294967296.0) - 4,
                    (4 * rows + 4) * (rnd(&state) / 4294967296.0) - 4);
  Sink warm = {0, 1469598103934665603ULL, 0, 1};
  for (int i = 0; i < 256; i++)
    query(space, shapes, n, points[i], mode, limit, i, &warm);
  Sink timed = {0, 0, 0, 0};
  double start = now();
  for (int i = 0; i < queries; i++)
    query(space, shapes, n, points[i % 256], mode, limit, i, &timed);
  double elapsed = now() - start;
  check(isfinite(timed.sum));
  printf("%s,%s,%d,%.17g,%d,%.9f,%" PRIu64 ",%" PRIu64 ",%.17g\n", api, kind, n, (double)limit,
         queries, elapsed, warm.hash, timed.count, timed.sum);
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
