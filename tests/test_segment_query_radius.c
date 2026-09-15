#include "chipmunk/chipmunk.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x)                                                                                   \
  do {                                                                                             \
    if (!(x)) {                                                                                    \
      fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x);                                      \
      exit(1);                                                                                     \
    }                                                                                              \
  } while (0)
#define MAX_SHAPES 16
static void scalar(cpFloat a, cpFloat b) {
  CHECK((isnan(a) && isnan(b)) || memcmp(&a, &b, sizeof(a)) == 0);
}
static void same(cpSegmentQueryInfo a, cpSegmentQueryInfo b) {
  CHECK(a.shape == b.shape);
  scalar(a.alpha, b.alpha);
  scalar(a.point.x, b.point.x);
  scalar(a.point.y, b.point.y);
  scalar(a.normal.x, b.normal.x);
  scalar(a.normal.y, b.normal.y);
}
typedef struct Results {
  cpSpace *space;
  cpSegmentQueryInfo hits[MAX_SHAPES];
  int count, postSteps;
} Results;
static void after_query(cpSpace *space, void *key, void *data) {
  Results *r = (Results *)data;
  CHECK(key == data);
  CHECK(!cpSpaceIsLocked(space));
  r->postSteps++;
}
static void collect(cpShape *shape, cpVect point, cpVect normal, cpFloat alpha, void *data) {
  Results *r = (Results *)data;
  CHECK(cpSpaceIsLocked(r->space));
  CHECK(r->count < MAX_SHAPES);
  for (int i = 0; i < r->count; i++)
    CHECK(r->hits[i].shape != shape);
  cpSegmentQueryInfo info = {shape, point, normal, alpha};
  r->hits[r->count++] = info;
  if (r->count == 1)
    CHECK(cpSpaceAddPostStepCallback(r->space, after_query, r, r));
}
static int rejected(cpShapeFilter a, cpShapeFilter b) {
  return (a.group != 0 && a.group == b.group) || !(a.categories & b.mask) ||
         !(b.categories & a.mask);
}
static void compare(cpSpace *space, cpShape **shapes, int count, cpVect a, cpVect b, cpFloat radius,
                    cpShapeFilter filter) {
  cpSegmentQueryInfo expected[MAX_SHAPES];
  int n = 0;
  cpFloat best = 1;
  for (int i = 0; i < count; i++)
    if (!rejected(cpShapeGetFilter(shapes[i]), filter)) {
      cpSegmentQueryInfo info;
      if (cpShapeSegmentQuery(shapes[i], a, b, radius, &info)) {
        expected[n++] = info;
        if (info.alpha < best)
          best = info.alpha;
      }
    }
  Results r = {space, {{0}}, 0, 0};
  cpSpaceSegmentQuery(space, a, b, radius, filter, collect, &r);
  CHECK(r.count == n);
  CHECK(r.postSteps == (n > 0));
  for (int i = 0; i < n; i++) {
    int found = 0;
    for (int j = 0; j < r.count; j++)
      if (expected[i].shape == r.hits[j].shape) {
        same(expected[i], r.hits[j]);
        found++;
      }
    CHECK(found == 1);
  }
  cpSegmentQueryInfo first;
  cpShape *shape = cpSpaceSegmentQueryFirst(space, a, b, radius, filter, &first);
  if (best < 1) {
    CHECK(shape != NULL);
    scalar(first.alpha, best);
    int found = 0;
    for (int i = 0; i < n; i++)
      if (expected[i].shape == shape) {
        same(first, expected[i]);
        found++;
      }
    CHECK(found == 1);
  } else {
    CHECK(shape == NULL);
    CHECK(first.shape == NULL);
    scalar(first.alpha, 1);
    scalar(first.point.x, b.x);
    scalar(first.point.y, b.y);
  }
  CHECK(cpSpaceSegmentQueryFirst(space, a, b, radius, filter, NULL) == shape);
}
static void reproduction(int hash, int dynamic) {
  cpSpace *space = cpSpaceNew();
  if (hash)
    cpSpaceUseSpatialHash(space, 0.25, 10007);
  cpBody *body = dynamic ? cpSpaceAddBody(space, cpBodyNew(1, 1)) : cpSpaceGetStaticBody(space);
  cpShape *shapes[2];
  shapes[0] = cpSpaceAddShape(space, cpCircleShapeNew(body, 1, cpvzero));
  cpShapeSetSensor(shapes[0], cpTrue);
  cpVect a = cpv(-3, 1.5), b = cpv(3, 1.5);
  compare(space, shapes, 1, a, b, 0.75, CP_SHAPE_FILTER_ALL);
  shapes[1] = cpSpaceAddShape(space, cpCircleShapeNew(body, 1, cpv(100, 100)));
  compare(space, shapes, 2, a, b, 0.75, CP_SHAPE_FILTER_ALL);
  compare(space, shapes, 2, b, a, 0.75, CP_SHAPE_FILTER_ALL);
  compare(space, shapes, 2, cpv(-3, 1.75), cpv(3, 1.75), 0.75, CP_SHAPE_FILTER_ALL);
  compare(space, shapes, 2, cpv(1.5, -3), cpv(1.5, 3), 0.75, CP_SHAPE_FILTER_ALL);
  compare(space, shapes, 2, cpv(1.5, 0), cpv(1.5, 0), 0.75, CP_SHAPE_FILTER_ALL);
  /* All-hit queries include an endpoint hit; first retains its strict alpha<1 rule. */
  compare(space, shapes, 2, cpv(-3, 0), cpv(-1.5, 0), 0.5, CP_SHAPE_FILTER_ALL);
  compare(space, shapes, 2, a, b, 1e20, CP_SHAPE_FILTER_ALL);
  compare(space, shapes, 2, a, b, INFINITY, CP_SHAPE_FILTER_ALL);
  for (int i = 0; i < 2; i++) {
    cpSpaceRemoveShape(space, shapes[i]);
    cpShapeFree(shapes[i]);
  }
  if (dynamic) {
    cpSpaceRemoveBody(space, body);
    cpBodyFree(body);
  }
  cpSpaceFree(space);
}
static void matrix(int hash) {
  cpSpace *space = cpSpaceNew();
  if (hash)
    cpSpaceUseSpatialHash(space, 1, 1009);
  cpBody *body = cpSpaceAddBody(space, cpBodyNew(1, 1));
  cpShape *shapes[MAX_SHAPES];
  int count = 0;
  for (int i = 0; i < 3; i++) {
    cpBody *owner = i % 2 ? body : cpSpaceGetStaticBody(space);
    cpVect c = cpv(i * 3 - 3, 0.37);
    shapes[count++] = cpSpaceAddShape(space, cpCircleShapeNew(owner, 0.7, c));
    shapes[count++] = cpSpaceAddShape(
        space, cpSegmentShapeNew(owner, cpvadd(c, cpv(-0.8, 2)), cpvadd(c, cpv(0.8, 2.6)), 0.1));
    shapes[count++] = cpSpaceAddShape(
        space, cpBoxShapeNew2(owner, cpBBNew(c.x - 0.6, -2.6, c.x + 0.6, -1.4), 0.1));
  }
  cpVect vertices[16];
  for (int i = 0; i < 16; i++)
    vertices[i] = cpvadd(cpv(0, -3.7), cpvmult(cpvforangle(6.283185307179586 * i / 16), 0.9));
  shapes[count++] = cpSpaceAddShape(space, cpPolyShapeNewRaw(body, 16, vertices, 0.15));
  for (int i = 0; i < count; i++) {
    cpShapeSetSensor(shapes[i], i % 3 == 0);
    cpShapeSetFilter(shapes[i],
                     cpShapeFilterNew(i % 4 == 0 ? 3 : 0, i % 2 ? 1 : 2, CP_ALL_CATEGORIES));
  }
  /* Equal first-hit fractions across indexes keep the static-index result. */
  shapes[count++] = cpSpaceAddShape(space, cpCircleShapeNew(body, 0.7, cpv(-3, 0.37)));
  cpSegmentQueryInfo tie;
  CHECK(cpSpaceSegmentQueryFirst(space, cpv(-6, 0.37), cpv(6, 0.37), 0.75, CP_SHAPE_FILTER_ALL,
                                 &tie) == shapes[0]);
  cpShapeFilter filters[] = {
      CP_SHAPE_FILTER_ALL, {3, CP_ALL_CATEGORIES, CP_ALL_CATEGORIES}, {0, 1, 1}, {0, 0, 0}};
  cpFloat radii[] = {0.05, 0.75, 2};
  for (int j = 0; j < 41; j++)
    for (int k = 0; k < 3; k++)
      for (int f = 0; f < 4; f++) {
        cpFloat y = (j - 20) * 0.19;
        cpVect a = cpv(-6, y), b = cpv(6, y + 0.31);
        compare(space, shapes, count, a, b, radii[k], filters[f]);
        compare(space, shapes, count, b, a, radii[k], filters[f]);
      }
  compare(space, shapes, count, cpv(-6, -5), cpv(6, 5), 0.75, CP_SHAPE_FILTER_ALL);
  compare(space, shapes, count, cpv(-3, 0.37), cpv(6, 4), 0.75, CP_SHAPE_FILTER_ALL);
  compare(space, shapes, count, cpv(-3, 0.37), cpv(-3, 0.37), 0.75, CP_SHAPE_FILTER_ALL);
  /* A small hash-cell query exercises the BB fallback, not the all-object fallback. */
  compare(space, shapes, count, cpv(-3, 0.37), cpv(-3, 0.37), 0.05, CP_SHAPE_FILTER_ALL);
  /* Thin-ray control, deliberately away from exact endpoint/tangent boundaries. */
  compare(space, shapes, count, cpv(-6, 0.51), cpv(6, 0.83), 0, CP_SHAPE_FILTER_ALL);
  for (int i = 0; i < count; i++) {
    cpSpaceRemoveShape(space, shapes[i]);
    cpShapeFree(shapes[i]);
  }
  cpSpaceRemoveBody(space, body);
  cpBodyFree(body);
  cpSpaceFree(space);
}
int main(void) {
  for (int h = 0; h < 2; h++) {
    for (int d = 0; d < 2; d++)
      reproduction(h, d);
    matrix(h);
  }
  puts("Positive-radius segment queries match direct shape tests for both indexes.");
  return 0;
}
