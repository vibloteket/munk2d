#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/cpContactSolver.h"
#include "chipmunk/cpHastySpace.h"
#define CHECK(x) do { if(!(x)){fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x);exit(1);} } while(0)

typedef struct Fixture {cpSpace *space;cpBody *body[256];cpShape *shape[257];int count;cpConstraint *joint;cpBool hasty;} Fixture;
static void addBody(Fixture *f, int index, cpBool stack)
{
 cpBody *b=cpSpaceAddBody(f->space,cpBodyNew(1,cpMomentForBox(1,1,1)));
 cpBodySetPosition(b,stack?cpv((index%2)*1.1,0.4+(index/2)*1.01):cpv(-200+2*index,0.4));
 cpShape *shape=cpSpaceAddShape(f->space,cpBoxShapeNew(b,1,1,0));
 cpShapeSetFriction(shape,0.7);f->body[index]=b;f->shape[index]=shape;f->count++;
}
static void makeWithSpace(Fixture *f,int count,cpBool stack,cpSpace *space)
{
 memset(f,0,sizeof(*f));f->space=space;cpSpaceSetGravity(f->space,cpv(0,-10));cpSpaceSetCollisionSlop(f->space,0.005);
 f->shape[256]=cpSpaceAddShape(f->space,cpSegmentShapeNew(cpSpaceGetStaticBody(f->space),cpv(-1000,0),cpv(1000,0),0));
 cpShapeSetFriction(f->shape[256],1);
 for(int i=0;i<count;i++)addBody(f,i,stack);
}
static void make(Fixture *f,int count,cpBool stack){makeWithSpace(f,count,stack,cpSpaceNew());}
static void removeLast(Fixture *f)
{
 int i=--f->count;cpSpaceRemoveShape(f->space,f->shape[i]);cpShapeFree(f->shape[i]);cpSpaceRemoveBody(f->space,f->body[i]);cpBodyFree(f->body[i]);
}
static void cleanup(Fixture *f)
{
 if(f->joint){cpSpaceRemoveConstraint(f->space,f->joint);cpConstraintFree(f->joint);}
 while(f->count)removeLast(f);
 cpSpaceRemoveShape(f->space,f->shape[256]);cpShapeFree(f->shape[256]);if(f->hasty)cpHastySpaceFree(f->space);else cpSpaceFree(f->space);
}
static void compare(Fixture *a,Fixture *b)
{
 CHECK(a->count==b->count);
 for(int i=0;i<a->count;i++){
  cpBody *x=a->body[i],*y=b->body[i];
  CHECK(isfinite(x->p.x)&&isfinite(x->p.y)&&isfinite(x->v.x)&&isfinite(x->v.y)&&isfinite(x->w));
  CHECK(memcmp(&x->p,&y->p,sizeof(cpVect))==0);CHECK(memcmp(&x->v,&y->v,sizeof(cpVect))==0);
  CHECK(memcmp(&x->v_bias,&y->v_bias,sizeof(cpVect))==0);CHECK(memcmp(&x->a,&y->a,sizeof(cpFloat))==0);
  CHECK(memcmp(&x->w,&y->w,sizeof(cpFloat))==0);CHECK(memcmp(&x->w_bias,&y->w_bias,sizeof(cpFloat))==0);
 }
 CHECK(a->space->arbiters->num==b->space->arbiters->num);
 for(int i=0;i<a->space->arbiters->num;i++){
  cpArbiter *x=(cpArbiter *)a->space->arbiters->arr[i],*y=(cpArbiter *)b->space->arbiters->arr[i];CHECK(x->count==y->count);
  for(int j=0;j<x->count;j++){CHECK(x->contacts[j].jnAcc==y->contacts[j].jnAcc);CHECK(x->contacts[j].jtAcc==y->contacts[j].jtAcc);CHECK(x->contacts[j].jBias==y->contacts[j].jBias);}
 }
}
static void features(void)
{
 unsigned int required=(1u<<26)|(1u<<27)|(1u<<28);
 CHECK(cpContactSolverCheckFeatures(7,required,1u<<5,6));
 for(int bit=26;bit<=28;bit++)CHECK(!cpContactSolverCheckFeatures(7,required&~(1u<<bit),1u<<5,6));
 CHECK(!cpContactSolverCheckFeatures(6,required,1u<<5,6));CHECK(!cpContactSolverCheckFeatures(7,required,0,6));
 CHECK(!cpContactSolverCheckFeatures(7,required,1u<<5,0));CHECK(!cpContactSolverCheckFeatures(7,required,1u<<5,2));CHECK(!cpContactSolverCheckFeatures(7,required,1u<<5,4));
}
static int callbackCount,postStepCount;
static void switchAfterStep(cpSpace *s,void *key,void *data){(void)key;(void)data;CHECK(!cpSpaceIsLocked(s));CHECK(cpSpaceSetContactSolver(s,CP_CONTACT_SOLVER_ORIGINAL));postStepCount++;}
static void postSolve(cpArbiter *arb,cpSpace *s,void *data)
{
 (void)arb;(void)data;CHECK(cpSpaceIsLocked(s));CHECK(!cpSpaceSetContactSolver(s,CP_CONTACT_SOLVER_ORIGINAL));CHECK(cpSpaceGetContactSolver(s)==CP_CONTACT_SOLVER_AVX2);callbackCount++;
 cpSpaceAddPostStepCallback(s,switchAfterStep,&callbackCount,NULL);
}
int main(void)
{
 features();CHECK(cpContactSolverIsAvailable(CP_CONTACT_SOLVER_ORIGINAL));CHECK(!cpContactSolverIsAvailable((cpContactSolverType)99));
 Fixture a,b;make(&a,8,cpFalse);CHECK(cpSpaceGetContactSolver(a.space)==CP_CONTACT_SOLVER_ORIGINAL);CHECK(!cpSpaceSetContactSolver(a.space,(cpContactSolverType)99));
 cpSpaceLock(a.space);CHECK(!cpSpaceSetContactSolver(a.space,CP_CONTACT_SOLVER_ORIGINAL));CHECK(!cpSpaceSetContactSolver(a.space,CP_CONTACT_SOLVER_AVX2));cpSpaceUnlock(a.space,cpTrue);
 if(!cpContactSolverIsAvailable(CP_CONTACT_SOLVER_AVX2)){
  CHECK(!cpSpaceSetContactSolver(a.space,CP_CONTACT_SOLVER_AVX2));CHECK(cpSpaceGetContactSolver(a.space)==CP_CONTACT_SOLVER_ORIGINAL);cpSpaceStep(a.space,1.0/60);cleanup(&a);
  puts("PASS: CPU/OS feature matrix, unavailable build/CPU and original-default API.");return 0;
 }
 CHECK(cpSpaceSetContactSolver(a.space,CP_CONTACT_SOLVER_AVX2));cpContactSolverContext *ctx=cpContactSolverGet(a.space);CHECK(ctx&&!ctx->graphMemory&&!ctx->solverMemory);
 make(&b,8,cpFalse);
 for(int i=0;i<8;i++){cpSpaceStep(a.space,1.0/60);cpSpaceStep(b.space,1.0/60);compare(&a,&b);}CHECK(ctx->usedLastStep);
 void *graph=ctx->graphMemory,*solver=ctx->solverMemory;int capacity=ctx->graphCapacity;
 cpSpaceStep(a.space,1.0/60);CHECK(ctx->graphMemory==graph&&ctx->solverMemory==solver);
 for(int i=8;i<160;i++)addBody(&a,i,cpFalse);cpSpaceStep(a.space,1.0/60);CHECK(ctx->graphCapacity>capacity);CHECK(ctx->solverBodyCapacity>=160);
 while(a.count>8)removeLast(&a);cpSpaceStep(a.space,1.0/60);CHECK(ctx->usedLastStep);CHECK(ctx->bodyStride<=9);CHECK(ctx->bodyCount==0&&ctx->packetCount==0);
 /* Free shapes/bodies while context still exists: destroy must not dereference them. */
 cleanup(&a);cleanup(&b);

 make(&a,16,cpTrue);make(&b,16,cpTrue);CHECK(cpSpaceSetContactSolver(a.space,CP_CONTACT_SOLVER_AVX2));CHECK(cpSpaceSetContactSolver(b.space,CP_CONTACT_SOLVER_AVX2));cpContactSolverGet(b.space)->kernel=cpContactSolverKernelScalar;
 for(int step=0;step<600;step++){int iterations=step%3==0?1:step%3==1?10:20;cpSpaceSetIterations(a.space,iterations);cpSpaceSetIterations(b.space,iterations);cpFloat dt=step%2?1.0/60:1.0/120;cpSpaceStep(a.space,dt);cpSpaceStep(b.space,dt);compare(&a,&b);}cleanup(&a);cleanup(&b);

 make(&a,8,cpFalse);make(&b,8,cpFalse);a.joint=cpSpaceAddConstraint(a.space,cpPinJointNew(a.body[0],a.body[1],cpvzero,cpvzero));b.joint=cpSpaceAddConstraint(b.space,cpPinJointNew(b.body[0],b.body[1],cpvzero,cpvzero));CHECK(cpSpaceSetContactSolver(a.space,CP_CONTACT_SOLVER_AVX2));
 for(int i=0;i<30;i++){cpSpaceStep(a.space,1.0/60);cpSpaceStep(b.space,1.0/60);compare(&a,&b);CHECK(!cpContactSolverGet(a.space)->usedLastStep);}
 CHECK(!cpContactSolverGet(a.space)->solverMemory);CHECK(cpSpaceSetContactSolver(a.space,CP_CONTACT_SOLVER_ORIGINAL));CHECK(cpContactSolverGet(a.space)==NULL);cleanup(&a);cleanup(&b);

 /*13 contacts on one writable body cannot fit in12 colors; no data is committed. */
 cpSpace *space=cpSpaceNew();CHECK(cpSpaceSetContactSolver(space,CP_CONTACT_SOLVER_AVX2));cpBody *body=cpBodyNew(1,1);cpArbiter arbs[13];struct cpContact contacts[13];memset(arbs,0,sizeof(arbs));memset(contacts,0,sizeof(contacts));
 for(int i=0;i<13;i++){arbs[i].body_a=body;arbs[i].body_b=cpSpaceGetStaticBody(space);arbs[i].count=1;arbs[i].contacts=&contacts[i];cpArrayPush(space->arbiters,&arbs[i]);}
 ctx=cpContactSolverGet(space);CHECK(!cpContactSolverStep(space,ctx));CHECK(!ctx->solverMemory);CHECK(ctx->bodyCount==0&&ctx->packetCount==0);CHECK(body->v.x==0&&body->v.y==0);space->arbiters->num=0;cpBodyFree(body);cpSpaceFree(space);

 make(&a,8,cpFalse);CHECK(cpSpaceSetContactSolver(a.space,CP_CONTACT_SOLVER_AVX2));cpSpaceAddGlobalCollisionHandler(a.space)->postSolveFunc=postSolve;cpSpaceStep(a.space,1.0/60);CHECK(callbackCount>0&&postStepCount==1);CHECK(cpSpaceGetContactSolver(a.space)==CP_CONTACT_SOLVER_ORIGINAL);cleanup(&a);
 space=cpSpaceNew();CHECK(cpSpaceSetContactSolver(space,CP_CONTACT_SOLVER_AVX2));cpSpaceDestroy(space);memset(space,0,sizeof(*space));cpSpaceInit(space);CHECK(cpSpaceGetContactSolver(space)==CP_CONTACT_SOLVER_ORIGINAL);cpSpaceFree(space);
 make(&a,8,cpFalse);cpSpaceSetSleepTimeThreshold(a.space,0.5);cpBodySleep(a.body[0]);CHECK(cpBodyIsSleeping(a.body[0]));CHECK(cpSpaceSetContactSolver(a.space,CP_CONTACT_SOLVER_AVX2));CHECK(cpBodyIsSleeping(a.body[0]));CHECK(cpSpaceSetContactSolver(a.space,CP_CONTACT_SOLVER_ORIGINAL));CHECK(cpBodyIsSleeping(a.body[0]));cleanup(&a);
 makeWithSpace(&a,8,cpFalse,cpHastySpaceNew());makeWithSpace(&b,8,cpFalse,cpHastySpaceNew());a.hasty=b.hasty=cpTrue;cpHastySpaceSetThreads(a.space,1);cpHastySpaceSetThreads(b.space,1);CHECK(cpSpaceSetContactSolver(a.space,CP_CONTACT_SOLVER_AVX2));
 for(int i=0;i<12;i++){cpHastySpaceStep(a.space,1.0/60);cpHastySpaceStep(b.space,1.0/60);compare(&a,&b);}CHECK(!cpContactSolverGet(a.space)->graphMemory);cleanup(&a);cleanup(&b);
 puts("PASS: CPU/OS gating, API, original default, arena reuse/grow/shrink, scalar equivalence, varying dt/iterations, fallback, callback locks, sleep, HastySpace independence and lifecycle.");return 0;
}
