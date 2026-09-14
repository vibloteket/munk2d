#define _POSIX_C_SOURCE 200809L
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "chipmunk/chipmunk.h"
static void check(int ok){if(!ok){fputs("Point-query benchmark validation failed\n",stderr);exit(1);}}
static double now(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
static uint32_t random32(uint32_t *s){*s^=*s<<13;*s^=*s>>17;*s^=*s<<5;return *s;}
static void hash_info(cpPointQueryInfo *info,uint64_t *hash){
  double values[]={info->distance,info->point.x,info->point.y,info->gradient.x,info->gradient.y};
  for(int i=0;i<5;i++){uint64_t bits;check(isfinite(values[i]));memcpy(&bits,&values[i],sizeof(bits));*hash=(*hash^bits)*1099511628211ULL;}
  *hash=(*hash^(uint64_t)(uintptr_t)cpShapeGetUserData(info->shape))*1099511628211ULL;
}
int main(int argc,char **argv){
  check(argc==4);const char *mode=argv[1];int vertices=atoi(argv[2]),queries=atoi(argv[3]);
  int many=!strcmp(mode,"space"),inside=!strcmp(mode,"inside"),outside=!strcmp(mode,"outside");
  check((many||inside||outside||!strcmp(mode,"shape"))&&vertices>=3&&vertices<=64&&queries>0);
  cpSpace *space=cpSpaceNew();cpShape *shapes[64];int count=many?64:1;
  for(int i=0;i<count;i++){
    cpVect verts[64],center=cpv(8*(i%8),8*(i/8));
    for(int j=0;j<vertices;j++)verts[j]=cpvadd(center,cpvmult(cpvforangle(6.2831853071795864769*j/vertices+0.173),3));
    shapes[i]=cpSpaceAddShape(space,cpPolyShapeNewRaw(cpSpaceGetStaticBody(space),vertices,verts,0.125));
    cpShapeSetUserData(shapes[i],(cpDataPointer)(uintptr_t)(i+1));
  }
  cpVect points[1024];uint32_t random=123456789;
  for(int i=0;i<1024;i++){
    double x=random32(&random)/4294967296.0,y=random32(&random)/4294967296.0;
    if(many)points[i]=cpv(64*x-4,64*y-4);
    else if(inside)points[i]=cpv(x-0.5,y-0.5);
    else if(outside)points[i]=cpvmult(cpvforangle(6.2831853071795864769*x),6+6*y);
    else points[i]=cpv(18*x-9,18*y-9);
  }
  uint64_t hash=1469598103934665603ULL;
  for(int i=0;i<1024;i++){
    cpPointQueryInfo info;
    if(many)cpSpacePointQueryNearest(space,points[i],INFINITY,CP_SHAPE_FILTER_ALL,&info);
    else cpShapePointQuery(shapes[0],points[i],&info);
    check(info.shape!=NULL);hash_info(&info,&hash);
  }
  cpFloat sum=0;double start=now();
  for(int i=0;i<queries;i++){
    cpPointQueryInfo info;
    if(many)cpSpacePointQueryNearest(space,points[i%1024],INFINITY,CP_SHAPE_FILTER_ALL,&info);
    else cpShapePointQuery(shapes[0],points[i%1024],&info);
    sum+=info.distance;
  }
  double elapsed=now()-start;check(isfinite(sum));
  printf("%s,%d,%d,%.9f,%" PRIu64 ",%.17g\n",mode,vertices,queries,elapsed,hash,(double)sum);
  for(int i=0;i<count;i++){cpSpaceRemoveShape(space,shapes[i]);cpShapeFree(shapes[i]);}cpSpaceFree(space);return 0;
}
