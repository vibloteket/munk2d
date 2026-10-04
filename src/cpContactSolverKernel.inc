/* Instantiated for vector and scalar arithmetic; keep operation order identical. */
typedef struct {V x,y,w,bx,by,bw;} Velocity;
static inline Velocity getVelocity(cpContactSolverContext *p,const int *ids){
 int s=p->bodyStride;double *v=p->velocity;Velocity b={gather(v,ids),gather(v+s,ids),gather(v+2*s,ids),gather(v+3*s,ids),gather(v+4*s,ids),gather(v+5*s,ids)};return b;
}
static inline void scatter(double *p,const int *ids,int lanes,V value){
#ifdef PACKET_SCALAR
 (void)lanes;p[ids[0]]=value;
#else
 double x[CP_CONTACT_SOLVER_LANES];store(x,value);for(int l=0;l<lanes;l++)p[ids[l]]=x[l];
#endif
}

static inline void putVelocity(cpContactSolverContext *p,const int *ids,int lanes,Velocity b){
 int s=p->bodyStride;double *v=p->velocity;scatter(v,ids,lanes,b.x);scatter(v+s,ids,lanes,b.y);scatter(v+2*s,ids,lanes,b.w);scatter(v+3*s,ids,lanes,b.bx);scatter(v+4*s,ids,lanes,b.by);scatter(v+5*s,ids,lanes,b.bw);
}
static inline V dot(V ax,V ay,V bx,V by){return add(mul(ax,bx),mul(ay,by));}
static inline void impulse(Velocity *v,V m,V i,V rx,V ry,V jx,V jy,V jbx,V jby){
 v->bx=add(v->bx,mul(jbx,m));v->by=add(v->by,mul(jby,m));v->bw=add(v->bw,mul(i,sub(mul(rx,jby),mul(ry,jbx))));
 v->x=add(v->x,mul(jx,m));v->y=add(v->y,mul(jy,m));v->w=add(v->w,mul(i,sub(mul(rx,jy),mul(ry,jx))));
}
static void solvePacket(cpContactSolverContext *p,cpContactSolverPacket *q,int lane){
 Velocity a=getVelocity(p,q->a+lane),b=getVelocity(p,q->b+lane);
 V nx=load(q->nx),ny=load(q->ny),sx=load(q->sx),sy=load(q->sy),friction=load(q->u);
 for(int k=0;k<q->count;k++){
  cpContactSolverContact *c=&q->con[k];V r1x=load(c->r1x),r1y=load(c->r1y),r2x=load(c->r2x),r2y=load(c->r2y);
  V vb1x=add(a.bx,mul(neg(r1y),a.bw)),vb1y=add(a.by,mul(r1x,a.bw));
  V vb2x=add(b.bx,mul(neg(r2y),b.bw)),vb2y=add(b.by,mul(r2x,b.bw));
  V vrx=add(sub(add(b.x,mul(neg(r2y),b.w)),add(a.x,mul(neg(r1y),a.w))),sx);
  V vry=add(sub(add(b.y,mul(r2x,b.w)),add(a.y,mul(r1x,a.w))),sy);
  V vbn=dot(sub(vb2x,vb1x),sub(vb2y,vb1y),nx,ny),vrn=dot(vrx,vry,nx,ny);
  V nMass=load(c->nMass),oldBias=load(c->jBias),oldN=load(c->jnAcc);
  V jBias=vmax(add(oldBias,mul(sub(load(c->bias),vbn),nMass)),zero());
  V jn=vmax(add(oldN,mul(neg(add(load(c->bounce),vrn)),nMass)),zero());
  store(c->jBias,jBias);store(c->jnAcc,jn);
  V db=sub(jBias,oldBias),dn=sub(jn,oldN),jbx=mul(nx,db),jby=mul(ny,db),jx,jy;
  if(q->friction){
   V vrt=dot(vrx,vry,neg(ny),nx),limit=mul(friction,jn),oldT=load(c->jtAcc);
   V jt=vmin(vmax(add(oldT,mul(neg(vrt),load(c->tMass))),neg(limit)),limit);store(c->jtAcc,jt);
   V dt=sub(jt,oldT);jx=sub(mul(nx,dn),mul(ny,dt));jy=add(mul(nx,dt),mul(ny,dn));
  }else{jx=mul(nx,dn);jy=mul(ny,dn);}
  if(q->mode!=1)impulse(&a,load(q->am),load(q->ai),r1x,r1y,neg(jx),neg(jy),neg(jbx),neg(jby));
  if(q->mode!=2)impulse(&b,load(q->bm),load(q->bi),r2x,r2y,jx,jy,jbx,jby);
 }
 if(q->mode!=1)putVelocity(p,q->a+lane,q->lanes,a);if(q->mode!=2)putVelocity(p,q->b+lane,q->lanes,b);
}
