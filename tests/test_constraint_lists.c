#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "chipmunk/chipmunk_private.h"

#define N 64

typedef struct List { cpConstraint *items[N]; int count, seen; } List;
static void check(int ok) { if(!ok){ fputs("Constraint-list regression\n",stderr); exit(1); } }
static void visit(cpConstraint *constraint,void *data) {
  List *list=(List *)data;
  check(list->seen<list->count && list->items[list->seen]==constraint);
  list->seen++;
}
static void visit_body(cpBody *body,cpConstraint *constraint,void *data) { (void)body; visit(constraint,data); }
static void verify(cpSpace *space,cpBody *body,List *active,List *linked) {
  active->seen=0; cpSpaceEachConstraint(space,visit,active); check(active->seen==active->count);
  linked->seen=0; cpBodyEachConstraint(body,visit_body,linked); check(linked->seen==linked->count);
  linked->seen=0; cpBodyEachConstraint(cpSpaceGetStaticBody(space),visit_body,linked); check(linked->seen==linked->count);
#ifdef CP_CONSTRAINT_ARRAY_INDEX
  for(int i=0;i<active->count;i++) check(active->items[i]->activeIndex==i);
#endif
}
static void add(cpSpace *space,cpConstraint *c,List *active,List *linked) {
  cpSpaceAddConstraint(space,c);
  active->items[active->count++]=c;
  memmove(linked->items+1,linked->items,linked->count*sizeof(cpConstraint *));
  linked->items[0]=c; linked->count++;
}
static void remove_at(cpSpace *space,int slot,List *active,List *linked) {
  cpConstraint *c=active->items[slot];
  cpSpaceRemoveConstraint(space,c);
  active->items[slot]=active->items[--active->count];
  int at=0; while(at<linked->count && linked->items[at]!=c) at++;
  check(at<linked->count);
  linked->count--;
  memmove(linked->items+at,linked->items+at+1,(linked->count-at)*sizeof(cpConstraint *));
  check(!cpSpaceContainsConstraint(space,c));
#ifdef CP_CONSTRAINT_ARRAY_INDEX
  check(c->activeIndex==-1);
#endif
}
static void sleep_wake(cpSpace *space,cpBody *body,List *active,List *linked) {
  cpBodySleep(body);
  active->count=0;
  verify(space,body,active,linked);
#ifdef CP_CONSTRAINT_ARRAY_INDEX
  for(int i=0;i<linked->count;i++) check(linked->items[i]->activeIndex==-1);
#endif
  cpBodyActivate(body);
  memcpy(active->items,linked->items,linked->count*sizeof(cpConstraint *));
  active->count=linked->count;
  verify(space,body,active,linked);
}
int main(void) {
  cpSpace *space=cpSpaceNew();
  cpSpaceSetSleepTimeThreshold(space,100);
  cpBody *body=cpSpaceAddBody(space,cpBodyNew(1,1));
  cpBodySetPosition(body,cpv(2,0));
  cpBody *anchor=cpSpaceGetStaticBody(space);
  cpConstraint *all[N];
  List active={{0},0,0},linked={{0},0,0};
  for(int i=0;i<N;i++) {
    all[i]=cpPinJointNew(i%2?body:anchor,i%2?anchor:body,cpvzero,cpvzero);
    add(space,all[i],&active,&linked);
  }
  verify(space,body,&active,&linked);
  for(int i=0;i<N;i++) {
    remove_at(space,i%3==0?0:i%3==1?active.count/2:active.count-1,&active,&linked);
    verify(space,body,&active,&linked);
  }
  for(int i=0;i<N;i++) add(space,all[(17*i)%N],&active,&linked);
  sleep_wake(space,body,&active,&linked);
  unsigned int random=0x12345678u;
  for(int step=0;step<256;step++) {
    random^=random<<13; random^=random>>17; random^=random<<5;
    cpConstraint *c=all[random%N];
    if(cpSpaceContainsConstraint(space,c)) {
      int slot=0; while(slot<active.count && active.items[slot]!=c) slot++;
      check(slot<active.count); remove_at(space,slot,&active,&linked);
    } else add(space,c,&active,&linked);
    verify(space,body,&active,&linked);
    if(step%19==0) sleep_wake(space,body,&active,&linked);
  }
  while(active.count) remove_at(space,active.count-1,&active,&linked);
  verify(space,body,&active,&linked);
  for(int i=0;i<N;i++) cpConstraintFree(all[i]);
  cpSpaceRemoveBody(space,body); cpBodyFree(body); cpSpaceFree(space);
  puts("Constraint array and body-list ordering/membership tests passed.");
  return 0;
}
