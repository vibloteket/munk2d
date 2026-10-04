#include <stdio.h>
#include <stdlib.h>
#include "chipmunk/chipmunk_private.h"

static void check(int ok){if(!ok){fputs("Array deletion regression\n",stderr);exit(1);}}
static void reference_delete(cpArray *array,void *object){
  for(int i=0;i<array->num;i++)if(array->arr[i]==object){
    array->num--;array->arr[i]=array->arr[array->num];array->arr[array->num]=NULL;return;
  }
}
static void compare(cpArray *a,cpArray *b){
  check(a->num==b->num&&a->max==b->max);
  for(int i=0;i<a->num;i++)check(a->arr[i]==b->arr[i]);
}
int main(void){
  int keys[4]={0,1,2,3};void *values[]={&keys[0],&keys[1],NULL,&keys[2]};
  /* Exhaustive duplicate/NULL/absent/first/last cases through length five. */
  int states=1;
  for(int length=0;length<=5;length++){
    if(length)states*=3;
    for(int state=0;state<states;state++)for(int target=0;target<4;target++){
      cpArray *a=cpArrayNew(8),*b=cpArrayNew(8);int code=state;
      for(int i=0;i<length;i++){void *p=values[code%3];code/=3;cpArrayPush(a,p);cpArrayPush(b,p);}
      cpArrayDeleteObj(a,values[target]);reference_delete(b,values[target]);compare(a,b);
      if(a->num<length)check(a->arr[a->num]==NULL&&b->arr[b->num]==NULL);
      cpArrayFree(a);cpArrayFree(b);
    }
  }
  cpArray *a=cpArrayNew(0),*b=cpArrayNew(0);unsigned int random=123456789;
  for(int step=0;step<10000;step++){
    random^=random<<13;random^=random>>17;random^=random<<5;
    if(a->num<128&&(random&3u)==0){void *p=values[(random>>2)%4];cpArrayPush(a,p);cpArrayPush(b,p);}
    else{int before=a->num;void *p=a->num&&(random&1u)?a->arr[a->num-1]:values[(random>>2)%4];cpArrayDeleteObj(a,p);reference_delete(b,p);if(a->num<before)check(a->arr[a->num]==NULL&&b->arr[b->num]==NULL);}
    compare(a,b);
  }
  cpArrayFree(a);cpArrayFree(b);
  puts("Array deletion matches original order/counts with duplicate and NULL entries.");
  return 0;
}
