#define _POSIX_C_SOURCE 200809L
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "chipmunk/chipmunk.h"

static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec + t.tv_nsec*1e-9; }
static void check(int ok) { if(!ok){ fputs("Lifecycle validation failed\n",stderr); exit(1); } }
static void wake(cpShape *shape, void *data) { (void)shape; cpBodyActivate((cpBody *)data); }
static void count_body(cpBody *body, void *data) { (void)body; (*(int *)data)++; }
static void hash_body(cpBody *body, void *data) { uint64_t *hash=(uint64_t *)data; *hash=(*hash ^ (uint64_t)(uintptr_t)cpBodyGetUserData(body))*1099511628211ULL; }
static void count_shape(cpShape *shape, void *data) { (void)shape; (*(int *)data)++; }
static void hash_shape(cpShape *shape, void *data) { hash_body(cpShapeGetBody(shape),data); }

int main(int argc,char **argv) {
  check(argc==4||(argc==5&&strcmp(argv[4],"shapes")==0));
  const char *mode=argv[1];
  int n=atoi(argv[2]), repeats=atoi(argv[3]);
  int sleeping=strcmp(mode,"sleep")==0, queued=strcmp(mode,"queued")==0;
  int removing=strcmp(mode,"remove")==0, with_shapes=argc==5||removing;
  check((sleeping||queued||removing||strcmp(mode,"unlocked")==0)&&n>0&&repeats>0);
  double elapsed=0;
  uint64_t hash=1469598103934665603ULL;
  for(int repeat=0;repeat<repeats;repeat++) {
    cpSpace *space=cpSpaceNew();
    cpSpaceSetSleepTimeThreshold(space,100);
    cpShape *trigger=cpSpaceAddShape(space,cpCircleShapeNew(cpSpaceGetStaticBody(space),1,cpvzero));
    cpBody **bodies=(cpBody **)calloc(n,sizeof(*bodies));
    cpShape **shapes=with_shapes?(cpShape **)calloc(n,sizeof(*shapes)):NULL;
    check(bodies!=NULL&&(!with_shapes||shapes!=NULL));
    for(int i=0;i<n;i++) {
      bodies[i]=cpSpaceAddBody(space,cpBodyNew(1,1));
      cpBodySetUserData(bodies[i],(cpDataPointer)(uintptr_t)(i+1));
      if(with_shapes) {
        cpBodySetPosition(bodies[i],cpv(3*(i%128),3*(i/128)));
        shapes[i]=cpSpaceAddShape(space,cpCircleShapeNew(bodies[i],0.25,cpvzero));
      }
    }
    double start=0;
    if(removing) {
      start=now();
      for(int i=0;i<n;i++) cpSpaceRemoveShape(space,shapes[i]);
      elapsed+=now()-start;
    } else {
      if(sleeping) start=now();
      for(int i=0;i<n;i++) cpBodySleepWithGroup(bodies[i],i==0?NULL:bodies[0]);
      if(sleeping) elapsed+=now()-start;
      for(int i=0;i<n;i++) check(cpBodyIsSleeping(bodies[i]));
      if(!sleeping) {
        start=now();
        if(queued) cpSpaceBBQuery(space,cpBBNew(-2,-2,2,2),CP_SHAPE_FILTER_ALL,wake,bodies[0]);
        else cpBodyActivate(bodies[0]);
        elapsed+=now()-start;
      } else cpBodyActivate(bodies[0]);
    }
    // All verification and cleanup is deliberately outside the timed phase.
    check(!cpSpaceIsLocked(space));
    int count=0; cpSpaceEachBody(space,count_body,&count); check(count==n);
    cpSpaceEachBody(space,hash_body,&hash);
    if(with_shapes) {
      int shape_count=0; cpSpaceEachShape(space,count_shape,&shape_count);
      check(shape_count==(removing?1:n+1));
      cpSpaceEachShape(space,hash_shape,&hash);
      for(int i=0;i<n;i++) check(cpSpaceContainsShape(space,shapes[i])==!removing);
    }
    for(int i=0;i<n;i++) check(!cpBodyIsSleeping(bodies[i]));
    for(int i=0;i<n;i++) {
      if(with_shapes) { if(!removing) cpSpaceRemoveShape(space,shapes[i]); cpShapeFree(shapes[i]); }
      cpSpaceRemoveBody(space,bodies[i]); cpBodyFree(bodies[i]);
    }
    cpSpaceRemoveShape(space,trigger); cpShapeFree(trigger);
    cpSpaceFree(space); free(shapes); free(bodies);
  }
  printf("%s,%d,%d,%.9f,%" PRIu64 "\n",mode,n,repeats,elapsed,hash);
}
