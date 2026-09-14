#include "chipmunk/chipmunk_private.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define MAX_HITS 32
#define check(ok)                                                                                  \
  do {                                                                                             \
    if (!(ok)) {                                                                                   \
      fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #ok);                                     \
      exit(1);                                                                                     \
    }                                                                                              \
  } while (0)
static void scalar(cpFloat a, cpFloat b) {
  if (isnan(a) && isnan(b))
    return;
  check(memcmp(&a, &b, sizeof(a)) == 0);
}
static void same(cpPointQueryInfo a, cpPointQueryInfo b) {
  check(a.shape == b.shape);
  scalar(a.distance, b.distance);
  scalar(a.point.x, b.point.x);
  scalar(a.point.y, b.point.y);
  scalar(a.gradient.x, b.gradient.x);
  scalar(a.gradient.y, b.gradient.y);
}
static cpPointQueryInfo full_query(cpShape *shape, cpVect point) {
  cpPointQueryInfo info;
  if (shape->klass->type == CP_CIRCLE_SHAPE) {
    cpCircleShape *circle = (cpCircleShape *)shape;
    cpVect delta = cpvsub(point, circle->tc);
    cpFloat d = cpvlength(delta), r = circle->r;
    info.shape = shape;
    cpFloat scale = d > 0 ? r / d : r;
    info.point = cpvadd(circle->tc, cpvmult(delta, scale));
    info.distance = d - r;
    info.gradient = d > MAGIC_EPSILON ? cpvmult(delta, 1.0f / d) : cpv(0, 1);
  } else if (shape->klass->type == CP_SEGMENT_SHAPE) {
    cpSegmentShape *seg = (cpSegmentShape *)shape;
    cpVect closest = cpClosetPointOnSegment(point, seg->ta, seg->tb);
    cpVect delta = cpvsub(point, closest);
    cpFloat d = cpvlength(delta), r = seg->r;
    cpVect g = cpvmult(delta, 1.0f / d);
    info.shape = shape;
    info.point = d ? cpvadd(closest, cpvmult(g, r)) : closest;
    info.distance = d - r;
    info.gradient = d > MAGIC_EPSILON ? g : seg->n;
  } else
    cpShapePointQuery(shape, point, &info);
  return info;
}
typedef struct Hits {
  cpPointQueryInfo info[MAX_HITS];
  int count;
} Hits;
typedef struct Context {
  cpVect point;
  cpFloat limit;
  cpShapeFilter filter;
  Hits *hits;
  cpPointQueryInfo best;
} Context;
static void collect(cpShape *shape, cpVect point, cpFloat distance, cpVect gradient, void *data) {
  Hits *hits = (Hits *)data;
  check(hits->count < MAX_HITS);
  cpPointQueryInfo info = {shape, point, distance, gradient};
  hits->info[hits->count++] = info;
}
static cpCollisionID reference_hit(void *object, void *candidate, cpCollisionID id, void *data) {
  Context *ctx = (Context *)object;
  cpShape *shape = (cpShape *)candidate;
  (void)data;
  if (!cpShapeFilterReject(shape->filter, ctx->filter)) {
    cpPointQueryInfo info = full_query(shape, ctx->point);
    if (info.shape && info.distance < ctx->limit && ctx->hits)
      collect(shape, info.point, info.distance, info.gradient, ctx->hits);
    if (info.distance < ctx->best.distance)
      ctx->best = info;
  }
  return id;
}
static void compare_queries(cpSpace *space, cpVect p, cpFloat limit, cpShapeFilter filter) {
  Hits expected = {{{0}}, 0}, actual = {{{0}}, 0};
  cpPointQueryInfo initial = {NULL, cpvzero, limit, cpvzero};
  Context ctx = {p, limit, filter, &expected, initial};
  cpBB bb = cpBBNewForCircle(p, cpfmax(limit, 0));
  /* This reference collector does not mutate the space or queue callbacks. */
  cpSpatialIndexQuery(space->dynamicShapes, &ctx, bb, reference_hit, NULL);
  cpSpatialIndexQuery(space->staticShapes, &ctx, bb, reference_hit, NULL);
  cpSpacePointQuery(space, p, limit, filter, collect, &actual);
  check(actual.count == expected.count);
  for (int i = 0; i < actual.count; i++)
    same(actual.info[i], expected.info[i]);
  cpPointQueryInfo nearest;
  cpShape *shape = cpSpacePointQueryNearest(space, p, limit, filter, &nearest);
  check(shape == ctx.best.shape);
  same(nearest, ctx.best);
  check(cpSpacePointQueryNearest(space, p, limit, filter, NULL) == ctx.best.shape);
}
int main(void) {
  cpSpace *space = cpSpaceNew();
  cpBody *body = cpSpaceAddBody(space, cpBodyNew(1, 1));
  cpSpaceSetSleepTimeThreshold(space, 100);
  cpShape *shapes[MAX_HITS];
  int count = 0;
  const cpFloat radii[] = {0, 1e-8, 0.5, 2};
  for (int i = 0; i < 8; i++) {
    cpBody *owner = i % 2 ? body : cpSpaceGetStaticBody(space);
    shapes[count] = cpSpaceAddShape(
        space, cpCircleShapeNew(owner, radii[i % 4], cpv((i % 4) - 1.5, (i / 4) - 0.5)));
    cpShapeSetSensor(shapes[count], i % 3 == 0);
    cpShapeSetFilter(shapes[count],
                     cpShapeFilterNew(i % 3 == 0 ? 1 : 0, i % 2 ? 1 : 2, CP_ALL_CATEGORIES));
    count++;
  }
  shapes[count++] = cpSpaceAddShape(space, cpSegmentShapeNew(body, cpv(-2, -2), cpv(2, -2), 0.1));
  shapes[count++] = cpSpaceAddShape(
      space, cpSegmentShapeNew(cpSpaceGetStaticBody(space), cpv(-2, 1), cpv(2, 2), 0));
  shapes[count++] = cpSpaceAddShape(
      space, cpBoxShapeNew2(cpSpaceGetStaticBody(space), cpBBNew(-1, -1, 1, 1), 0.1));
  const cpFloat limits[] = {-3, -1, -0.0, 0.000001, 0.5, 1, 3, INFINITY, -INFINITY, NAN};
  const cpShapeFilter filters[] = {
      CP_SHAPE_FILTER_ALL, {1, CP_ALL_CATEGORIES, CP_ALL_CATEGORIES}, {0, 1, 1}};
  for (int phase = 0; phase < 2; phase++) {
    if (phase)
      cpBodySleep(body);
    for (int x = -3; x <= 3; x++)
      for (int y = -3; y <= 3; y++)
        for (int l = 0; l < 10; l++)
          for (int f = 0; f < 3; f++)
            compare_queries(space, cpv(0.5 * x, 0.5 * y), limits[l], filters[f]);
    cpBodyActivate(body);
  }
  for (int i = 0; i < count; i++) {
    cpVect points[] = {cpvzero, cpBBCenter(cpShapeGetBB(shapes[i])), cpv(1e-8, 0), cpv(INFINITY, 0),
                       cpv(NAN, 0)};
    for (int p = 0; p < 5; p++) {
      cpPointQueryInfo expected = full_query(shapes[i], points[p]), actual;
      cpShapePointQuery(shapes[i], points[p], &actual);
      same(actual, expected);
    }
  }
  /* Exact cutoffs and adjacent representable distances, including private-class fallbacks. */
  for (int i = 0; i < count; i++) {
    cpVect p = cpv(0.37, -0.11);
    cpFloat d = full_query(shapes[i], p).distance;
    cpFloat lower = CP_USE_DOUBLES ? nextafter(d, -INFINITY) : nextafterf(d, -INFINITY);
    cpFloat upper = CP_USE_DOUBLES ? nextafter(d, INFINITY) : nextafterf(d, INFINITY);
    compare_queries(space, p, d, CP_SHAPE_FILTER_ALL);
    compare_queries(space, p, lower, CP_SHAPE_FILTER_ALL);
    compare_queries(space, p, upper, CP_SHAPE_FILTER_ALL);
    const cpShapeClass *original = shapes[i]->klass;
    cpShapeClass clone = *original;
    shapes[i]->klass = &clone;
    compare_queries(space, p, INFINITY, CP_SHAPE_FILTER_ALL);
    compare_queries(space, p, d, CP_SHAPE_FILTER_ALL);
    clone.type = (cpShapeType)99;
    compare_queries(space, p, INFINITY, CP_SHAPE_FILTER_ALL);
    shapes[i]->klass = original;
  }
  /* Preserve large-coordinate rounded distances; no new BB-based rejection. */
  cpFloat huge = CP_USE_DOUBLES ? ldexp(1.0, 53) : ldexp(1.0, 24);
  shapes[count++] =
      cpSpaceAddShape(space, cpCircleShapeNew(cpSpaceGetStaticBody(space), huge, cpv(huge, 0)));
  compare_queries(space, cpv(-0.5, 0), 1, CP_SHAPE_FILTER_ALL);
  compare_queries(space, cpv(-0.5, 0), 0.25, CP_SHAPE_FILTER_ALL);
  for (int i = 0; i < count; i++) {
    cpSpaceRemoveShape(space, shapes[i]);
    cpShapeFree(shapes[i]);
  }
  cpSpaceRemoveBody(space, body);
  cpBodyFree(body);
  cpSpaceFree(space);
  space = cpSpaceNew();
  body = cpSpaceAddBody(space, cpBodyNew(1, 1));
  cpBodySetPosition(body, cpv(-0.125, 0));
  cpShape *small = cpSpaceAddShape(space, cpCircleShapeNew(body, 0.125, cpvzero));
  cpShape *large =
      cpSpaceAddShape(space, cpCircleShapeNew(cpSpaceGetStaticBody(space), huge, cpv(huge, 0)));
  compare_queries(space, cpv(-0.5, 0), 1, CP_SHAPE_FILTER_ALL);
  cpPointQueryInfo nearest;
  check(cpSpacePointQueryNearest(space, cpv(-0.5, 0), 1, CP_SHAPE_FILTER_ALL, &nearest) == large);
  scalar(nearest.distance, 0);
  /* Equal distances across indexes retain the first (dynamic) shape, even a sensor. */
  cpShape *dynamicTie = cpSpaceAddShape(space, cpCircleShapeNew(body, 1, cpv(0.125, 0)));
  cpShape *staticTie =
      cpSpaceAddShape(space, cpCircleShapeNew(cpSpaceGetStaticBody(space), 1, cpvzero));
  cpShapeSetSensor(dynamicTie, cpTrue);
  compare_queries(space, cpv(0.25, 0), 1, CP_SHAPE_FILTER_ALL);
  check(cpSpacePointQueryNearest(space, cpv(0.25, 0), 1, CP_SHAPE_FILTER_ALL, &nearest) ==
        dynamicTie);
  cpSpaceRemoveShape(space, dynamicTie);
  cpShapeFree(dynamicTie);
  cpSpaceRemoveShape(space, staticTie);
  cpShapeFree(staticTie);
  cpSpaceRemoveShape(space, small);
  cpShapeFree(small);
  cpSpaceRemoveShape(space, large);
  cpShapeFree(large);
  cpSpaceRemoveBody(space, body);
  cpBodyFree(body);
  cpSpaceFree(space);
  puts("Point-query limits preserve full results, filtering, sensors, ties and sleeping shapes.");
  return 0;
}
