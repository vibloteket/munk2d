#define _POSIX_C_SOURCE 200809L
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "chipmunk/chipmunk.h"
static void check(int ok){if(!ok){fputs("Body lifecycle validation failed\n",stderr);exit(1);}}
static double now(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
static unsigned int rnd(unsigned int *s){*s^=*s<<13;*s^=*s>>17;*s^=*s<<5;return *s;}
typedef struct Summary{int count;uint64_t hash;}Summary;
static void collect(cpBody *body,void *data){Summary *s=(Summary *)data;s->count++;s->hash=(s->hash^(uint64_t)(uintptr_t)cpBodyGetUserData(body))*1099511628211ULL;}
int main(int argc,char **argv){
  check(argc==4);const char *mode=argv[1];int n=atoi(argv[2]),repeats=atoi(argv[3]);
  int reverse=!strcmp(mode,"reverse"),random=!strcmp(mode,"random"),first=!strcmp(mode,"first"),forward=!strcmp(mode,"forward");
  int removing=reverse||random||first||forward,wake=!strcmp(mode,"wake-reverse"),automatic=!strcmp(mode,"auto-sleep"),sleep=!strcmp(mode,"sleep-group");
  check(n>0&&repeats>0&&(removing||wake||automatic||sleep));
  double elapsed=0;uint64_t hash=1469598103934665603ULL;
  for(int repeat=0;repeat<repeats;repeat++){
    cpSpace *space=cpSpaceNew();cpBody **bodies=(cpBody **)calloc(n,sizeof(*bodies));int *order=(int *)calloc(n,sizeof(*order));check(bodies&&order);
    for(int i=0;i<n;i++){bodies[i]=cpSpaceAddBody(space,cpBodyNew(1,1));cpBodySetUserData(bodies[i],(cpDataPointer)(uintptr_t)(i+1));order[i]=i;}
    if(random){unsigned int seed=123456789;for(int i=n-1;i>0;i--){int j=(int)(rnd(&seed)%(unsigned int)(i+1)),t=order[i];order[i]=order[j];order[j]=t;}}
    if(automatic){cpSpaceStep(space,1.0/60.0);cpSpaceSetSleepTimeThreshold(space,0.001);}
    else cpSpaceSetSleepTimeThreshold(space,100);
    if(wake)for(int i=0;i<n;i++)cpBodySleep(bodies[i]);
    double start=now();
    if(removing)for(int i=0;i<n;i++){int k=reverse?n-1-i:first?(i==0?0:n-i):order[i];cpSpaceRemoveBody(space,bodies[k]);}
    else if(wake)for(int i=n-1;i>=0;i--)cpBodyActivate(bodies[i]);
    else if(automatic)cpSpaceStep(space,1.0/60.0);
    else for(int i=0;i<n;i++)cpBodySleepWithGroup(bodies[i],i==0?NULL:bodies[0]);
    elapsed+=now()-start;
    Summary summary={0,hash};cpSpaceEachBody(space,collect,&summary);check(summary.count==(removing?0:n));hash=summary.hash;
    for(int i=0;i<n;i++){
      check(cpSpaceContainsBody(space,bodies[i])==!removing);
      if(!removing)check(cpBodyIsSleeping(bodies[i])==(automatic||sleep));
    }
    if(!removing)for(int i=0;i<n;i++)cpBodyActivate(bodies[i]);
    for(int i=0;i<n;i++){if(!removing)cpSpaceRemoveBody(space,bodies[i]);cpBodyFree(bodies[i]);}
    cpSpaceFree(space);free(bodies);free(order);
  }
  printf("%s,%d,%d,%.9f,%" PRIu64 "\n",mode,n,repeats,elapsed,hash);return 0;
}
