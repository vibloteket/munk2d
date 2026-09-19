/* This translation unit alone is built for AVX2, and is excluded from LTO.
 * Entry is reachable only after the generic CPU/OS capability check. */
#include "cpContactSolver.h"
#if CP_AVX2_CONTACT_SOLVER && CP_USE_DOUBLES
#include <immintrin.h>

typedef __m256d V;
static inline V load(const double *p){ return _mm256_loadu_pd(p); }
static inline void store(double *p, V v){ _mm256_storeu_pd(p, v); }
static inline V add(V a, V b){ return _mm256_add_pd(a, b); }
static inline V sub(V a, V b){ return _mm256_sub_pd(a, b); }
static inline V mul(V a, V b){ return _mm256_mul_pd(a, b); }
static inline V neg(V a){ return _mm256_xor_pd(a, _mm256_set1_pd(-0.0)); }
static inline V vmax(V a, V b){ return _mm256_max_pd(a, b); }
static inline V vmin(V a, V b){ return _mm256_min_pd(a, b); }
static inline V zero(void){ return _mm256_setzero_pd(); }
static inline V gather(const double *p, const int *ids){
 /* Four ordinary loads avoid the expensive hardware gather on older AVX2
  * CPUs, especially with Gather Data Sampling microcode mitigations. Keep
  * the lane order and exact double values; no solver arithmetic changes. */
 return _mm256_setr_pd(p[ids[0]], p[ids[1]], p[ids[2]], p[ids[3]]);
}
#include "cpContactSolverKernel.inc"
#include "cpContactSolverScalar.inc"

void
cpContactSolverKernelAvx2(cpContactSolverContext *s, int iterations)
{
 for(int iteration = 0; iteration < iterations; ++iteration){
  for(int i = 0; i < s->packetCount; ++i){
   cpContactSolverPacket *p = &s->packets[i];
   if(p->lanes == 1) solveScalarPacket(s, p, 0);
   else solvePacket(s, p, 0);
  }
 }
}
#endif
