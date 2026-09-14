#include <stdio.h>
#include <stdlib.h>
#include "chipmunk/chipmunk_private.h"

static void check(int ok){if(!ok){fputs("Constraint collision-filter regression\n",stderr);exit(1);}}
static void pre_solve(cpArbiter *arb,cpSpace *space,void *data){(void)arb;(void)space;(*(int *)data)++;}
static cpConstraint *joint(cpSpace *space,cpBody *a,cpBody *b){
  cpConstraint *c=cpSpaceAddConstraint(space,cpPinJointNew(a,b,cpvzero,cpvzero));
  cpConstraintSetCollideBodies(c,cpFalse);
  return c;
}
static void probe(cpSpace *space,cpShape *a,cpShape *b,int *calls,int allowed){
  int before=*calls;
  cpSpaceLock(space);
  cpSpaceCollideShapes(a,b,0,space);
  cpSpaceUnlock(space,cpTrue);
  check(*calls-before==allowed);
  before=*calls;
  cpSpaceLock(space);
  cpSpaceCollideShapes(b,a,0,space);
  cpSpaceUnlock(space,cpTrue);
  check(*calls-before==allowed);
}
static void run_case(int countA,int countB,int staticA){
  cpSpace *space=cpSpaceNew();
  cpSpaceSetSleepTimeThreshold(space,100);
  cpBody *a=staticA?cpSpaceGetStaticBody(space):cpSpaceAddBody(space,cpBodyNew(1,1));
  cpBody *b=cpSpaceAddBody(space,cpBodyNew(1,1));
  cpBody *c=cpSpaceAddBody(space,cpBodyNew(1,1));
  cpBody *d=cpSpaceAddBody(space,cpBodyNew(1,1));
  cpBodySetPosition(b,cpv(1,0));cpBodySetPosition(c,cpv(100,0));cpBodySetPosition(d,cpv(120,0));
  cpShape *sa=cpSpaceAddShape(space,cpCircleShapeNew(a,1,cpvzero));
  cpShape *sb=cpSpaceAddShape(space,cpCircleShapeNew(b,1,cpvzero));
  cpShapeSetSensor(sa,cpTrue); /* Check filtering callbacks without solving contacts. */
  int calls=0;
  cpCollisionHandler *handler=cpSpaceAddGlobalCollisionHandler(space);
  handler->preSolveFunc=pre_solve;handler->userData=&calls;
  cpSpacePushFreshContactBuffer(space);
  probe(space,sa,sb,&calls,1);

  cpConstraint *fillers[128];int count=0;
  for(int i=0;i<countA;i++)fillers[count++]=joint(space,i%2?a:c,i%2?c:a);
  for(int i=0;i<countB;i++)fillers[count++]=joint(space,i%2?d:b,i%2?b:d);
  /* Blocking A-C / B-D joints must not block A-B. */
  probe(space,sa,sb,&calls,1);
  cpConstraint *first=joint(space,a,b);
  probe(space,sa,sb,&calls,0);
  cpConstraintSetCollideBodies(first,cpTrue);
  probe(space,sa,sb,&calls,1);
  cpConstraint *second=joint(space,b,a);
  cpConstraintSetCollideBodies(second,cpTrue);
  cpConstraintSetCollideBodies(first,cpFalse);
  /* Move filler joints ahead of both shared joints, testing deep blockers. */
  for(int i=0;i<count;i++){cpSpaceRemoveConstraint(space,fillers[i]);cpSpaceAddConstraint(space,fillers[i]);}
  probe(space,sa,sb,&calls,0);
  cpConstraintSetCollideBodies(first,cpTrue);
  cpConstraintSetCollideBodies(second,cpFalse);
  probe(space,sa,sb,&calls,0);
  cpSpaceRemoveConstraint(space,second);cpConstraintFree(second);
  probe(space,sa,sb,&calls,1);
  cpConstraintSetCollideBodies(first,cpFalse);
  cpBodySleep(b);
  check(cpBodyIsSleeping(b));
  /* Sleeping constraints remain linked to bodies even when not active. */
  probe(space,sa,sb,&calls,0);
  cpConstraintSetCollideBodies(first,cpTrue);
  check(!cpBodyIsSleeping(b));
  probe(space,sa,sb,&calls,1);
  cpSpaceRemoveConstraint(space,first);cpConstraintFree(first);
  probe(space,sa,sb,&calls,1);
  for(int i=0;i<count;i++){cpSpaceRemoveConstraint(space,fillers[i]);cpConstraintFree(fillers[i]);}
  cpSpaceRemoveShape(space,sa);cpShapeFree(sa);cpSpaceRemoveShape(space,sb);cpShapeFree(sb);
  cpSpaceRemoveBody(space,b);cpBodyFree(b);cpSpaceRemoveBody(space,c);cpBodyFree(c);cpSpaceRemoveBody(space,d);cpBodyFree(d);
  if(!staticA){cpSpaceRemoveBody(space,a);cpBodyFree(a);}
  cpSpaceFree(space);
}
int main(void){
  const int sizes[]={0,1,7,63};
  for(int a=0;a<4;a++)for(int b=0;b<4;b++)for(int isStatic=0;isStatic<2;isStatic++)run_case(sizes[a],sizes[b],isStatic);
  puts("Constraint filtering: both orientations, asymmetric degrees, multiple blockers and sleep/wake passed.");
  return 0;
}
