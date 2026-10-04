#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "chipmunk/chipmunk.h"
static void check(int ok){if(!ok){fputs("Constraint lifecycle validation failed\n",stderr);exit(1);}}
static double now(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
typedef struct Summary{int count;uint64_t hash;}Summary;
static void collect(cpConstraint *c,void *data){Summary *s=(Summary *)data;s->count++;s->hash=(s->hash^(uint64_t)(uintptr_t)cpConstraintGetUserData(c))*1099511628211ULL;}
static void collect_body(cpBody *body,cpConstraint *c,void *data){(void)body;collect(c,data);}
int main(int argc,char **argv){
  check(argc==5);const char *mode=argv[1],*topology=argv[2];int n=atoi(argv[3]),repeats=atoi(argv[4]);
  int removing=!strcmp(mode,"oldest")||!strcmp(mode,"newest"),sleeping=!strcmp(mode,"sleep"),hub=!strcmp(topology,"hub");
  check((removing||sleeping||!strcmp(mode,"wake"))&&(hub||!strcmp(topology,"parallel"))&&n>0&&repeats>0);
  double elapsed=0;uint64_t hash=1469598103934665603ULL;
  for(int repeat=0;repeat<repeats;repeat++){
    cpSpace *space=cpSpaceNew();cpSpaceSetSleepTimeThreshold(space,100);
    int bodyCount=hub?n:1;cpBody **bodies=(cpBody **)calloc(bodyCount,sizeof(*bodies));cpConstraint **constraints=(cpConstraint **)calloc(n,sizeof(*constraints));check(bodies&&constraints);
    for(int i=0;i<bodyCount;i++){bodies[i]=cpSpaceAddBody(space,cpBodyNew(1,1));cpBodySetPosition(bodies[i],cpv(i+1,0));}
    cpBody *anchor=cpSpaceGetStaticBody(space);
    for(int i=0;i<n;i++){cpBody *b=bodies[hub?i:0];constraints[i]=cpSpaceAddConstraint(space,cpPinJointNew(i%2?b:anchor,i%2?anchor:b,cpvzero,cpvzero));cpConstraintSetUserData(constraints[i],(cpDataPointer)(uintptr_t)(i+1));}
    double start=0;
    if(removing){start=now();for(int i=0;i<n;i++)cpSpaceRemoveConstraint(space,constraints[!strcmp(mode,"newest")?n-1-i:i]);elapsed+=now()-start;}
    else{
      if(sleeping)start=now();
      for(int i=0;i<bodyCount;i++)cpBodySleepWithGroup(bodies[i],i?bodies[0]:NULL);
      if(sleeping)elapsed+=now()-start;
      Summary asleep={0,0};cpSpaceEachConstraint(space,collect,&asleep);check(asleep.count==0);
      if(!sleeping)start=now();cpBodyActivate(bodies[0]);if(!sleeping)elapsed+=now()-start;
    }
    Summary active={0,hash};cpSpaceEachConstraint(space,collect,&active);check(active.count==(removing?0:n));hash=active.hash;
    Summary links={0,0};cpBodyEachConstraint(anchor,collect_body,&links);check(links.count==(removing?0:n));
    for(int i=0;i<n;i++){check(cpSpaceContainsConstraint(space,constraints[i])==!removing);if(!removing)cpSpaceRemoveConstraint(space,constraints[i]);cpConstraintFree(constraints[i]);}
    for(int i=0;i<bodyCount;i++){cpSpaceRemoveBody(space,bodies[i]);cpBodyFree(bodies[i]);}
    cpSpaceFree(space);free(bodies);free(constraints);
  }
  printf("%s,%s,%d,%d,%.9f,%" PRIu64 "\n",mode,topology,n,repeats,elapsed,hash);
}
