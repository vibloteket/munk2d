#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "chipmunk/chipmunk_private.h"

static void check(int ok){if(!ok){fputs("Polygon point-query regression\n",stderr);exit(1);}}
/* The pre-optimization implementation is the compatibility oracle. */
static cpPointQueryInfo reference_query(cpPolyShape *poly,cpVect p){
  int count=poly->count;struct cpSplittingPlane *planes=poly->planes;cpFloat r=poly->r;
  cpVect v0=planes[count-1].v0;cpFloat minDist=INFINITY;
  cpVect closestPoint=cpvzero,closestNormal=cpvzero;cpBool outside=cpFalse;
  for(int i=0;i<count;i++){
    cpVect v1=planes[i].v0;
    outside=outside||(cpvdot(planes[i].n,cpvsub(p,v1))>0.0f);
    cpVect closest=cpClosetPointOnSegment(p,v0,v1);
    cpFloat dist=cpvdist(p,closest);
    if(dist<minDist){minDist=dist;closestPoint=closest;closestNormal=planes[i].n;}
    v0=v1;
  }
  cpFloat dist=outside?minDist:-minDist;
  cpVect g=cpvmult(cpvsub(p,closestPoint),1.0f/dist);
  cpPointQueryInfo info;
  info.shape=(cpShape *)poly;info.point=cpvadd(closestPoint,cpvmult(g,r));info.distance=dist-r;
  info.gradient=minDist>MAGIC_EPSILON?g:closestNormal;
  return info;
}
static void equal_scalar(cpFloat a,cpFloat b){
  if(isnan(a)&&isnan(b))return;
  check(memcmp(&a,&b,sizeof(cpFloat))==0);
}
static void compare(cpShape *shape,cpVect p){
  cpPointQueryInfo expected=reference_query((cpPolyShape *)shape,p),actual;
  cpFloat distance=cpShapePointQuery(shape,p,&actual);
  check(actual.shape==expected.shape);equal_scalar(distance,expected.distance);
  equal_scalar(actual.distance,expected.distance);
  equal_scalar(actual.point.x,expected.point.x);equal_scalar(actual.point.y,expected.point.y);
  equal_scalar(actual.gradient.x,expected.gradient.x);equal_scalar(actual.gradient.y,expected.gradient.y);
  equal_scalar(cpShapePointQuery(shape,p,NULL),expected.distance);
}
static void rounded_tie(void){
  cpBody *body=cpBodyNew(1,1);
  cpFloat epsilon=cpfsqrt(CP_USE_DOUBLES?DBL_EPSILON:FLT_EPSILON);
  cpVect verts[]={cpv(1,epsilon),cpv(1,-1),cpv(2,-1),cpv(2,epsilon)};
  cpShape *shape=cpPolyShapeNewRaw(body,4,verts,0);cpShapeCacheBB(shape);
  cpPointQueryInfo info;cpShapePointQuery(shape,cpvzero,&info);
  /* First edge: squared distance 1+epsilon^2; next edge: 1. Both
   * square roots round to 1. Preserve the FIRST feature, not min squared. */
  equal_scalar(info.distance,1);equal_scalar(info.point.y,epsilon);
  equal_scalar(info.gradient.y,-epsilon);compare(shape,cpvzero);
  cpShapeFree(shape);cpBodyFree(body);
}
int main(void){
  rounded_tie();
  const int counts[]={3,4,5,8,16,32};
  const cpFloat scales[]={0.000001,1,1000000};
  for(int s=0;s<3;s++)for(int n=0;n<6;n++)for(int bevel=0;bevel<2;bevel++)for(int rotated=0;rotated<2;rotated++){
    cpFloat scale=scales[s];int count=counts[n];cpVect vertices[32];
    cpBody *body=cpBodyNew(1,1);cpBodySetAngle(body,rotated?0.37:0);
    cpBodySetPosition(body,cpv(1.25*scale,-2.5*scale));
    for(int i=0;i<count;i++)vertices[i]=cpvmult(cpvforangle(6.2831853071795864769*i/count),3*scale);
    cpShape *shape=cpPolyShapeNewRaw(body,count,vertices,bevel?0.25*scale:0);cpShapeCacheBB(shape);
    for(int x=-8;x<=8;x++)for(int y=-8;y<=8;y++)compare(shape,cpvadd(cpBodyGetPosition(body),cpv(0.7*x*scale,0.7*y*scale)));
    cpPolyShape *poly=(cpPolyShape *)shape;
    for(int i=0;i<count;i++){
      cpVect v=poly->planes[i].v0;
      compare(shape,v);
      compare(shape,cpv(nextafter(v.x,INFINITY),v.y));
      compare(shape,cpv(nextafter(v.x,-INFINITY),v.y));
      compare(shape,cpvlerp(v,poly->planes[(i+1)%count].v0,0.5));
    }
    cpShapeFree(shape);cpBodyFree(body);
  }
  cpBody *body=cpBodyNew(1,1);cpVect box[]={cpv(0,0),cpv(1,0),cpv(1,1),cpv(0,1)};
  cpShape *shape=cpPolyShapeNewRaw(body,4,box,0);cpShapeCacheBB(shape);
  compare(shape,cpv(-1e-160,-1e-160));compare(shape,cpv(-1e-200,-1e-200));
  compare(shape,cpv(INFINITY,0));compare(shape,cpv(NAN,0));
  cpShapeFree(shape);cpBodyFree(body);
  puts("Polygon point queries preserve distances, features, gradients and rounded ties.");
  return 0;
}
