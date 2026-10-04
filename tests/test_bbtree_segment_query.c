#include "chipmunk/chipmunk.h"
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Pre-optimization query used as an exact arithmetic oracle. */
static cpFloat reference_query(cpBB bb, cpVect a, cpVect b) {
  cpVect delta = cpvsub(b, a);
  cpFloat tmin = -INFINITY, tmax = INFINITY;

  if (delta.x == 0.0f) {
    if (a.x < bb.l || bb.r < a.x)
      return INFINITY;
  } else {
    cpFloat t1 = (bb.l - a.x) / delta.x;
    cpFloat t2 = (bb.r - a.x) / delta.x;
    tmin = cpfmax(tmin, cpfmin(t1, t2));
    tmax = cpfmin(tmax, cpfmax(t1, t2));
  }

  if (delta.y == 0.0f) {
    if (a.y < bb.b || bb.t < a.y)
      return INFINITY;
  } else {
    cpFloat t1 = (bb.b - a.y) / delta.y;
    cpFloat t2 = (bb.t - a.y) / delta.y;
    tmin = cpfmax(tmin, cpfmin(t1, t2));
    tmax = cpfmin(tmax, cpfmax(t1, t2));
  }

  if (tmin <= tmax && 0.0f <= tmax && tmin <= 1.0f) {
    return cpfmax(tmin, 0.0f);
  } else {
    return INFINITY;
  }
}

static uint64_t state = 0x932573fad326a519ULL;
static uint64_t next(void) {
  state ^= state << 13;
  state ^= state >> 7;
  state ^= state << 17;
  return state;
}
static cpFloat value(int mode) {
  if (mode == 0)
    return (cpFloat)((int)(next() % 32001) - 16000) / 16;
  if (mode == 1) {
    cpFloat f;
    uint64_t b = next();
    memcpy(&f, &b, sizeof(f));
    return f;
  }
  cpFloat special[] = {0.0,
                       -0.0,
                       1,
                       -1,
                       0.25,
                       -0.25,
                       INFINITY,
                       -INFINITY,
                       NAN,
                       CP_USE_DOUBLES ? DBL_MIN : FLT_MIN,
                       CP_USE_DOUBLES ? DBL_MAX : FLT_MAX,
                       1e-20,
                       1e20};
  return special[next() % (sizeof(special) / sizeof(special[0]))];
}
int main(void) {
  uint64_t checked = 0, accepted = 0;
  for (int i = 0; i < 1000000; i++) {
    int mode = i % 3;
    cpFloat l = value(mode), bottom = value(mode), r = value(mode), top = value(mode);
    cpFloat ax = value(mode), ay = value(mode), bx = value(mode), by = value(mode);
    cpBB bb = cpBBNew(l, bottom, r, top);
    cpVect a = cpv(ax, ay), b = cpv(bx, by);
    if (i % 7 == 0)
      b.x = a.x;
    if (i % 11 == 0)
      b.y = a.y;
    cpFloat expected = reference_query(bb, a, b);
    cpFloat below =
        CP_USE_DOUBLES ? nextafter(expected, -INFINITY) : nextafterf(expected, -INFINITY);
    cpFloat above = CP_USE_DOUBLES ? nextafter(expected, INFINITY) : nextafterf(expected, INFINITY);
    cpFloat limits[] = {-INFINITY, -0.5,     -0.0, 0,        0.25,  0.5,
                        1,         INFINITY, NAN,  expected, below, above};
    for (int j = 0; j < (int)(sizeof(limits) / sizeof(limits[0])); j++) {
      cpFloat actual = cpBBSegmentQuery(bb, a, b);
      int want = expected < limits[j], got = actual < limits[j];
      if (want != got || memcmp(&expected, &actual, sizeof(actual))) {
        fprintf(stderr,
                "Mismatch case=%d limit=%.17g expected=%.17g actual=%.17g "
                "bb={%.17g,%.17g,%.17g,%.17g} a={%.17g,%.17g} b={%.17g,%.17g}\n",
                i, (double)limits[j], (double)expected, (double)actual, (double)bb.l, (double)bb.b,
                (double)bb.r, (double)bb.t, (double)a.x, (double)a.y, (double)b.x, (double)b.y);
        return 1;
      }
      checked++;
      accepted += want;
    }
  }
  printf("checked=%llu accepted=%llu; all fractions and cutoff decisions preserved\n",
         (unsigned long long)checked, (unsigned long long)accepted);
  return 0;
}
