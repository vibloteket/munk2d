#define _POSIX_C_SOURCE 200809L
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "chipmunk/chipmunk.h"
static void check(int ok){if(!ok){fputs("Filter scenario validation failed\n",stderr);exit(1);}}
static double now(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
typedef struct Motion{cpVect rest;int sign;}Motion;
static void move_component(cpBody *body,cpFloat dt){
  Motion *motion=(Motion *)cpBodyGetUserData(body);
  cpBodyUpdatePosition(body,dt); /* Retain normal bias reset and angular integration. */
  motion->sign=-motion->sign;
  cpBodySetPosition(body,cpvadd(motion->rest,cpv(6*motion->sign,0)));
}
static void pre_solve(cpArbiter *arb,cpSpace *space,void *data){(void)arb;(void)space;(*(uint64_t *)data)++;}
static void hash_body(cpBody *body,void *data){
  uint64_t *hash=(uint64_t *)data;cpVect p=cpBodyGetPosition(body),v=cpBodyGetVelocity(body);
  double values[]={p.x,p.y,v.x,v.y,cpBodyGetAngle(body),cpBodyGetAngularVelocity(body)};
  for(int i=0;i<6;i++){uint64_t bits;check(isfinite(values[i]));memcpy(&bits,&values[i],sizeof(bits));*hash=(*hash^bits)*1099511628211ULL;}
}
int main(int argc,char **argv){
  check(argc==6);const char *mode=argv[1];int moving=!strcmp(mode,"moving");
  int degreeA=atoi(argv[2]),degreeB=atoi(argv[3]),contacts=atoi(argv[4]),steps=atoi(argv[5]);
  check((moving||!strcmp(mode,"stationary"))&&degreeA>=0&&degreeB>=0&&contacts>0&&contacts%4==0&&steps>0);
  cpSpace *space=cpSpaceNew();cpBody *anchor=cpSpaceGetStaticBody(space);
  int count=1+degreeA+degreeB;
  cpBody **bodies=(cpBody **)calloc(count,sizeof(*bodies));Motion *motion=(Motion *)calloc(count,sizeof(*motion));
  cpConstraint **joints=(cpConstraint **)calloc(degreeA+degreeB+1,sizeof(*joints));cpShape **walls=(cpShape **)calloc(contacts,sizeof(*walls));check(bodies&&motion&&joints&&walls);
  for(int i=0;i<count;i++){
    bodies[i]=cpSpaceAddBody(space,cpBodyNew(1,1));
    motion[i].rest=i==0?cpvzero:cpv(100+2*i,50);motion[i].sign=-1;
    cpBodySetPosition(bodies[i],motion[i].rest);
    if(moving&&i<=degreeA){cpBodySetUserData(bodies[i],&motion[i]);cpBodySetPositionUpdateFunc(bodies[i],move_component);}
  }
  for(int i=0;i<degreeA+degreeB;i++){
    cpBody *a=i<degreeA?bodies[0]:anchor,*b=bodies[i+1];
    joints[i]=cpSpaceAddConstraint(space,cpPinJointNew(i%2?a:b,i%2?b:a,cpvzero,cpvzero));
    cpConstraintSetCollideBodies(joints[i],cpFalse);
  }
  cpShape *hub=cpSpaceAddShape(space,cpCircleShapeNew(bodies[0],20,cpvzero));
  for(int i=0;i<contacts;i++)walls[i]=cpSpaceAddShape(space,cpCircleShapeNew(anchor,1,cpvmult(cpvforangle(6.2831853071795864769*i/contacts),20.99)));
  uint64_t calls=0;cpCollisionHandler *handler=cpSpaceAddGlobalCollisionHandler(space);handler->preSolveFunc=pre_solve;handler->userData=&calls;
  /* Default force limits, precision, iterations, and real contact solving. */
  for(int i=0;i<4;i++)cpSpaceStep(space,1.0/60.0);
  calls=0;double start=now();for(int i=0;i<steps;i++)cpSpaceStep(space,1.0/60.0);double elapsed=now()-start;
  check(calls>0&&calls<=(uint64_t)contacts*steps);
  uint64_t hash=1469598103934665603ULL;cpSpaceEachBody(space,hash_body,&hash);
  printf("%s,%d,%d,%d,%d,%.9f,%" PRIu64 ",%" PRIu64 "\n",mode,degreeA,degreeB,contacts,steps,elapsed,calls,hash);
  for(int i=0;i<degreeA+degreeB;i++){cpSpaceRemoveConstraint(space,joints[i]);cpConstraintFree(joints[i]);}
  cpSpaceRemoveShape(space,hub);cpShapeFree(hub);
  for(int i=0;i<contacts;i++){cpSpaceRemoveShape(space,walls[i]);cpShapeFree(walls[i]);}
  for(int i=0;i<count;i++){cpSpaceRemoveBody(space,bodies[i]);cpBodyFree(bodies[i]);}
  cpSpaceFree(space);free(bodies);free(motion);free(joints);free(walls);return 0;
}
